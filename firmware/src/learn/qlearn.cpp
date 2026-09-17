#include "learn/qlearn.h"

#include <Arduino.h>
#include <string.h>

#include "config.h"
#include "hal/hal.h"

namespace learn {
namespace {

constexpr uint32_t kMagic = 0x54425131;  // "TBQ1"
constexpr uint8_t kVersion = 1;

// Q-values are stored as int16 hundredths in RAM as well as on flash: one
// representation means a save is a plain copy, and +/-327 is far wider than
// any value this reward scale can reach.
constexpr float kScale = 100.0f;
constexpr int16_t kQMax = INT16_MAX;

struct Checkpoint {
  uint32_t magic;
  uint8_t version;
  uint8_t states;
  uint8_t actions;
  uint8_t reserved;
  float epsilon;
  uint32_t steps;
  int16_t q[kStateCount * kActionCount];
};

// Shared between the control loop (update) and loop() (save). On the ESP32
// those are different cores, so every read of more than one field goes
// through a critical section.
Checkpoint g_table{};
bool g_loaded = false;
volatile bool g_savePending = false;
uint32_t g_lastSaveMs = 0;
uint32_t g_longestSaveMs = 0;
uint32_t g_saves = 0;
uint16_t g_lastSaveBytes = 0;

// Checkpoints are flushed one changed byte per loop() pass, so the control loop
// keeps ticking through a save instead of freezing for seconds. g_flash mirrors
// what storage holds, byte for byte, so unchanged bytes cost nothing.
//
// A power cut mid-flush leaves a mix of old and new Q-values. That is tolerable
// - they are estimates being revised anyway - and begin() still rejects a
// header or epsilon that came out corrupt.
Checkpoint g_flash{};
Checkpoint g_flushing{};
bool g_flushActive = false;
size_t g_flushCursor = 0;
uint32_t g_flushStartedMs = 0;
uint16_t g_flushBytes = 0;

int16_t& cell(uint8_t state, uint8_t action) {
  return g_table.q[state * kActionCount + action];
}

void blank() {
  g_table = Checkpoint{};
  g_table.magic = kMagic;
  g_table.version = kVersion;
  g_table.states = kStateCount;
  g_table.actions = kActionCount;
  g_table.epsilon = cfg::kEpsilonStart;
}

float maxQ(uint8_t state) {
  int16_t best = cell(state, 0);
  for (uint8_t a = 1; a < kActionCount; ++a) {
    if (cell(state, a) > best) best = cell(state, a);
  }
  return static_cast<float>(best) / kScale;
}

uint8_t band(uint16_t mm) {
  if (mm < cfg::kLearnNearMm) return 0;
  if (mm < cfg::kLearnFarMm) return 1;
  return 2;
}

}  // namespace

void begin() {
  g_loaded = false;
  Checkpoint stored{};
  // A table saved under a different layout is not a partial match, it is
  // noise; start over rather than learn on top of misfiled values.
  const bool read = hal::persistLoad(hal::Slot::kQTable, &stored, sizeof(stored));
  // Mirror storage as it is, valid or not, so the first flush only rewrites
  // bytes that actually differ.
  if (read) g_flash = stored;
  if (read &&
      stored.magic == kMagic && stored.version == kVersion &&
      stored.states == kStateCount && stored.actions == kActionCount &&
      stored.epsilon >= 0.0f && stored.epsilon <= 1.0f) {
    g_table = stored;
    g_loaded = true;
  } else {
    blank();
  }
  randomSeed(micros());
}

uint8_t encodeState(uint16_t frontMm, uint16_t leftMm, uint16_t rightMm,
                    Action previous) {
  // Side: which wall, if any, is close. Left wins a tie, matching roaming.
  uint8_t side = 0;
  if (leftMm < cfg::kSideNearMm) {
    side = 1;
  } else if (rightMm < cfg::kSideNearMm) {
    side = 2;
  }
  return static_cast<uint8_t>((band(frontMm) * 3 + side) * kActionCount + previous);
}

Action choose(uint8_t state) {
  hal::CriticalSection lock;
  if (random(10000) < static_cast<long>(g_table.epsilon * 10000.0f)) {
    return static_cast<Action>(random(kActionCount));
  }
  int16_t best = cell(state, 0);
  for (uint8_t a = 1; a < kActionCount; ++a) {
    if (cell(state, a) > best) best = cell(state, a);
  }
  uint8_t ties[kActionCount];
  uint8_t tieCount = 0;
  for (uint8_t a = 0; a < kActionCount; ++a) {
    if (cell(state, a) == best) ties[tieCount++] = a;
  }
  return static_cast<Action>(ties[random(tieCount)]);
}

void update(uint8_t state, Action action, float reward, uint8_t nextState) {
  hal::CriticalSection lock;
  const float current = static_cast<float>(cell(state, action)) / kScale;
  const float target = reward + cfg::kLearnGamma * maxQ(nextState);
  float next = (current + cfg::kLearnAlpha * (target - current)) * kScale;
  if (next > kQMax) next = kQMax;
  if (next < -kQMax) next = -kQMax;
  cell(state, action) = static_cast<int16_t>(lroundf(next));

  ++g_table.steps;
  const float decayed = g_table.epsilon * cfg::kEpsilonDecay;
  g_table.epsilon = decayed < cfg::kEpsilonMin ? cfg::kEpsilonMin : decayed;
}

void reset() {
  hal::CriticalSection lock;
  blank();
}

void requestSave() { g_savePending = true; }
bool savePending() { return g_savePending || g_flushActive; }

namespace {
void finishFlush() {
  g_lastSaveMs = millis() - g_flushStartedMs;
  if (g_lastSaveMs > g_longestSaveMs) g_longestSaveMs = g_lastSaveMs;
  g_lastSaveBytes = g_flushBytes;
  ++g_saves;
  g_flushActive = false;
}
}  // namespace

void service() {
  if (!g_flushActive) {
    if (!g_savePending) return;
    // Flush a snapshot, not the live table: learning keeps changing it, and a
    // target that moves under the cursor might never finish.
    {
      hal::CriticalSection lock;
      g_flushing = g_table;
    }
    g_savePending = false;
    g_flushActive = true;
    g_flushCursor = 0;
    g_flushBytes = 0;
    g_flushStartedMs = millis();

    if (!hal::persistByteAddressable()) {
      hal::persistSave(hal::Slot::kQTable, &g_flushing, sizeof(g_flushing));
      g_flash = g_flushing;
      g_flushBytes = sizeof(g_flushing);
      finishFlush();
      return;
    }
  }

  // At most one changed byte per call.
  const uint8_t* want = reinterpret_cast<const uint8_t*>(&g_flushing);
  uint8_t* have = reinterpret_cast<uint8_t*>(&g_flash);
  while (g_flushCursor < sizeof(Checkpoint) && want[g_flushCursor] == have[g_flushCursor]) {
    ++g_flushCursor;
  }
  if (g_flushCursor < sizeof(Checkpoint)) {
    hal::persistWriteByte(hal::Slot::kQTable, g_flushCursor, want[g_flushCursor]);
    have[g_flushCursor] = want[g_flushCursor];
    ++g_flushCursor;
    ++g_flushBytes;
    return;
  }
  finishFlush();
}

float q(uint8_t state, uint8_t action) {
  hal::CriticalSection lock;
  return static_cast<float>(cell(state, action)) / kScale;
}

float epsilon() {
  hal::CriticalSection lock;
  return g_table.epsilon;
}

uint32_t steps() {
  hal::CriticalSection lock;
  return g_table.steps;
}

bool loadedFromCheckpoint() { return g_loaded; }
uint32_t lastSaveMs() { return g_lastSaveMs; }
uint32_t longestSaveMs() { return g_longestSaveMs; }
uint16_t lastSaveBytes() { return g_lastSaveBytes; }
uint32_t saves() { return g_saves; }

const char* actionName(uint8_t action) {
  switch (action) {
    case kForward: return "fwd";
    case kTurnLeft: return "left";
    case kTurnRight: return "right";
    case kBack: return "back";
    default: return "?";
  }
}

}  // namespace learn
