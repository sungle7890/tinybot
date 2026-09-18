// Host-side tests for the Q-learner: `pio test -e native`.
//
// Everything the learner touches outside itself - time, randomness, storage -
// is faked here, so each test states exactly what the robot would have seen.

#include <unity.h>

#include <cstring>
#include <deque>

#include "config.h"
#include "hal/hal.h"
#include "learn/history.h"
#include "learn/qlearn.h"

// --- Fakes ------------------------------------------------------------------

namespace {

// Slot-addressed so the Q-table and the session log cannot be tested on top
// of each other - the bug this fake is most likely to have to catch.
uint8_t g_storage[2][1024];
uint8_t* slot(hal::Slot s) { return g_storage[s == hal::Slot::kQTable ? 0 : 1]; }
bool g_byteAddressable = true;
int g_byteWrites = 0;
int g_blobWrites = 0;
std::deque<long> g_randoms;  // scripted results for random(); empty -> 0
uint32_t g_nowMs = 0;

void eraseStorage() { std::memset(g_storage, 0xFF, sizeof(g_storage)); }
constexpr size_t kSlotBytes = sizeof(g_storage[0]);

// Runs service() until the checkpoint is fully written. Returns the number of
// calls, and fails if any single call wrote more than one byte.
int flush() {
  int calls = 0;
  while (learn::savePending()) {
    const int before = g_byteWrites;
    learn::service();
    ++calls;
    if (g_byteAddressable) {
      TEST_ASSERT_LESS_OR_EQUAL_MESSAGE(1, g_byteWrites - before,
                                        "one service() call wrote several bytes");
    }
    TEST_ASSERT_LESS_THAN_MESSAGE(2000, calls, "flush never finished");
  }
  return calls;
}

constexpr float kHundredth = 0.0051f;  // Q-values are stored in hundredths

}  // namespace

uint32_t millis() { return g_nowMs; }
uint32_t micros() { return g_nowMs * 1000; }
void randomSeed(unsigned long) {}
long random(long howBig) {
  if (g_randoms.empty()) return 0;
  const long v = g_randoms.front();
  g_randoms.pop_front();
  return v % howBig;
}
long random(long howSmall, long howBig) { return howSmall + random(howBig - howSmall); }

namespace hal {
CriticalSection::CriticalSection() {}
CriticalSection::~CriticalSection() {}

bool persistLoad(Slot s, void* data, size_t len) {
  if (s == Slot::kCalibration || len > kSlotBytes) return false;
  std::memcpy(data, slot(s), len);
  return true;
}
bool persistSave(Slot s, const void* data, size_t len) {
  if (s == Slot::kCalibration || len > kSlotBytes) return false;
  std::memcpy(slot(s), data, len);
  ++g_blobWrites;
  return true;
}
bool persistByteAddressable() { return g_byteAddressable; }
bool persistWriteByte(Slot s, size_t offset, uint8_t value) {
  if (s == Slot::kCalibration || offset >= kSlotBytes) return false;
  slot(s)[offset] = value;
  ++g_byteWrites;
  return true;
}
}  // namespace hal

void setUp() {
  eraseStorage();
  g_byteAddressable = true;
  g_byteWrites = 0;
  g_blobWrites = 0;
  g_randoms.clear();
  g_nowMs = 0;
  learn::begin();
  history::begin();
}

void tearDown() {}

int flushHistory() {
  int calls = 0;
  while (history::savePending()) {
    history::service();
    if (++calls > 4000) TEST_FAIL_MESSAGE("history flush never finished");
  }
  return calls;
}

history::Run madeRun(uint8_t mode, float metres) {
  history::Run r{};
  r.mode = mode;
  r.seconds = 60;
  r.forwardMeters = metres;
  r.contacts = 1;
  r.steps = 300;
  r.meanReward = 0.4f;
  r.stopReason = history::kStopCommand;
  return r;
}

// --- Boot -------------------------------------------------------------------

void test_blank_storage_boots_a_blank_table() {
  TEST_ASSERT_FALSE(learn::loadedFromCheckpoint());
  TEST_ASSERT_EQUAL_UINT32(0, learn::steps());
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, cfg::kEpsilonStart, learn::epsilon());
  for (uint8_t s = 0; s < learn::kStateCount; ++s) {
    for (uint8_t a = 0; a < learn::kActionCount; ++a) {
      TEST_ASSERT_EQUAL_FLOAT(0.0f, learn::q(s, a));
    }
  }
}

// --- State encoding ---------------------------------------------------------

void test_front_bands_split_at_the_configured_thresholds() {
  const uint16_t open = 1000;  // sides far away
  const uint8_t near = learn::encodeState(cfg::kLearnNearMm - 1, open, open, learn::kForward);
  const uint8_t mid = learn::encodeState(cfg::kLearnNearMm, open, open, learn::kForward);
  const uint8_t midTop = learn::encodeState(cfg::kLearnFarMm - 1, open, open, learn::kForward);
  const uint8_t far = learn::encodeState(cfg::kLearnFarMm, open, open, learn::kForward);
  TEST_ASSERT_EQUAL_UINT8(0, near);
  TEST_ASSERT_EQUAL_UINT8(12, mid);
  TEST_ASSERT_EQUAL_UINT8(mid, midTop);
  TEST_ASSERT_EQUAL_UINT8(24, far);
}

void test_side_band_prefers_the_left_wall_like_roaming_does() {
  const uint16_t near = cfg::kSideNearMm - 1;
  const uint16_t open = 1000;
  TEST_ASSERT_EQUAL_UINT8(4, learn::encodeState(0, near, open, learn::kForward));
  TEST_ASSERT_EQUAL_UINT8(8, learn::encodeState(0, open, near, learn::kForward));
  TEST_ASSERT_EQUAL_UINT8(4, learn::encodeState(0, near, near, learn::kForward));
}

void test_every_combination_maps_inside_the_table() {
  const uint16_t ranges[] = {0, cfg::kLearnNearMm, cfg::kLearnFarMm, 8190};
  const uint16_t sides[] = {0, 8190};
  bool seen[learn::kStateCount] = {};
  for (uint16_t f : ranges) {
    for (uint16_t l : sides) {
      for (uint16_t r : sides) {
        for (uint8_t p = 0; p < learn::kActionCount; ++p) {
          const uint8_t s = learn::encodeState(f, l, r, static_cast<learn::Action>(p));
          TEST_ASSERT_LESS_THAN_UINT8(learn::kStateCount, s);
          seen[s] = true;
        }
      }
    }
  }
  // 3 front bands x 3 side bands x 4 previous actions, each reachable.
  for (bool hit : seen) TEST_ASSERT_TRUE(hit);
}

// --- Choosing ---------------------------------------------------------------

void test_greedy_choice_takes_the_best_action() {
  learn::update(5, learn::kTurnRight, 3.0f, 0);
  g_randoms = {9999};  // above epsilon: exploit
  TEST_ASSERT_EQUAL_UINT8(learn::kTurnRight, learn::choose(5));
}

void test_ties_break_at_random_not_always_forward() {
  // A blank table ties every action; picking the first would mean a new robot
  // only ever drives forward and never learns what the others are worth.
  g_randoms = {9999, 3};
  TEST_ASSERT_EQUAL_UINT8(learn::kBack, learn::choose(0));
  g_randoms = {9999, 1};
  TEST_ASSERT_EQUAL_UINT8(learn::kTurnLeft, learn::choose(0));
}

void test_exploration_ignores_the_table() {
  learn::update(5, learn::kTurnRight, 3.0f, 0);
  g_randoms = {0, 0};  // below epsilon: explore, then pick action 0
  TEST_ASSERT_EQUAL_UINT8(learn::kForward, learn::choose(5));
}

void test_best_never_explores() {
  // A policy run is judged against the rules; random steps would handicap it.
  learn::update(5, learn::kTurnRight, 3.0f, 0);
  g_randoms = {0, 0, 0, 0};  // would force exploration in choose()
  TEST_ASSERT_EQUAL_UINT8(learn::kTurnRight, learn::best(5));
  TEST_ASSERT_EQUAL_UINT32(1, learn::steps());  // and it learns nothing
}

// --- Updating ---------------------------------------------------------------

void test_update_follows_the_q_learning_rule() {
  // Q = 0 + alpha * (1 + gamma * 0 - 0)
  learn::update(0, learn::kForward, 1.0f, 1);
  TEST_ASSERT_FLOAT_WITHIN(kHundredth, cfg::kLearnAlpha * 1.0f, learn::q(0, 0));

  // Bootstraps from the best action of the next state.
  const float before = learn::q(1, learn::kBack);
  learn::update(1, learn::kBack, 0.0f, 0);
  const float expected = before + cfg::kLearnAlpha *
                                      (0.0f + cfg::kLearnGamma * learn::q(0, 0) - before);
  TEST_ASSERT_FLOAT_WITHIN(kHundredth, expected, learn::q(1, learn::kBack));
}

void test_updates_smaller_than_a_hundredth_are_lost() {
  // Documents a real limit of the int16-hundredths storage: an update that
  // moves a value by less than 0.005 rounds away. With rewards around +/-1
  // that only blurs values near convergence, but it means a reward scale much
  // smaller than today's would silently stop learning.
  learn::update(0, learn::kForward, 0.02f, 0);  // alpha * 0.02 = 0.004
  TEST_ASSERT_EQUAL_FLOAT(0.0f, learn::q(0, learn::kForward));
}

void test_values_clamp_instead_of_wrapping() {
  for (int i = 0; i < 200; ++i) learn::update(0, learn::kForward, 1000.0f, 0);
  TEST_ASSERT_TRUE(learn::q(0, learn::kForward) > 300.0f);
  for (int i = 0; i < 400; ++i) learn::update(0, learn::kForward, -1000.0f, 0);
  TEST_ASSERT_TRUE(learn::q(0, learn::kForward) < -300.0f);
}

void test_epsilon_decays_per_step_down_to_the_floor() {
  learn::update(0, learn::kForward, 0.0f, 0);
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, cfg::kEpsilonStart * cfg::kEpsilonDecay, learn::epsilon());
  TEST_ASSERT_EQUAL_UINT32(1, learn::steps());
  for (int i = 0; i < 20000; ++i) learn::update(0, learn::kForward, 0.0f, 0);
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, cfg::kEpsilonMin, learn::epsilon());
}

void test_reset_forgets_values_and_exploration() {
  for (int i = 0; i < 500; ++i) learn::update(3, learn::kTurnLeft, 1.0f, 3);
  learn::reset();
  TEST_ASSERT_EQUAL_FLOAT(0.0f, learn::q(3, learn::kTurnLeft));
  TEST_ASSERT_EQUAL_UINT32(0, learn::steps());
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, cfg::kEpsilonStart, learn::epsilon());
}

// --- Checkpoints ------------------------------------------------------------

void test_checkpoint_survives_a_reboot() {
  learn::update(7, learn::kTurnLeft, 2.0f, 0);
  learn::update(30, learn::kBack, -5.0f, 7);
  const float left = learn::q(7, learn::kTurnLeft);
  const float back = learn::q(30, learn::kBack);
  const float eps = learn::epsilon();

  learn::requestSave();
  flush();

  learn::reset();   // RAM gone...
  learn::begin();   // ...and back from storage
  TEST_ASSERT_TRUE(learn::loadedFromCheckpoint());
  TEST_ASSERT_EQUAL_FLOAT(left, learn::q(7, learn::kTurnLeft));
  TEST_ASSERT_EQUAL_FLOAT(back, learn::q(30, learn::kBack));
  TEST_ASSERT_EQUAL_FLOAT(eps, learn::epsilon());
  TEST_ASSERT_EQUAL_UINT32(2, learn::steps());
}

void test_a_later_save_writes_only_the_bytes_that_changed() {
  learn::update(7, learn::kTurnLeft, 2.0f, 0);
  learn::requestSave();
  flush();
  g_byteWrites = 0;

  // One more step: one cell (2 bytes), epsilon (<= 4) and steps (<= 4).
  learn::update(7, learn::kTurnLeft, 2.0f, 0);
  learn::requestSave();
  flush();
  TEST_ASSERT_LESS_OR_EQUAL(10, g_byteWrites);
  TEST_ASSERT_EQUAL_UINT16(g_byteWrites, learn::lastSaveBytes());
}

void test_saving_again_with_no_changes_writes_nothing() {
  learn::update(7, learn::kTurnLeft, 2.0f, 0);
  learn::requestSave();
  flush();
  g_byteWrites = 0;

  learn::requestSave();
  flush();
  TEST_ASSERT_EQUAL_INT(0, g_byteWrites);
}

void test_a_change_made_mid_flush_is_not_lost() {
  // The flush reads the live table as it goes, so a value changed after the
  // cursor passed it is simply written by the next save - never dropped.
  learn::update(1, learn::kForward, 1.0f, 0);
  learn::requestSave();
  learn::service();  // flush begins
  learn::update(2, learn::kForward, 1.0f, 0);
  flush();

  learn::requestSave();
  flush();
  learn::begin();
  TEST_ASSERT_TRUE(learn::q(1, learn::kForward) > 0.0f);
  TEST_ASSERT_TRUE(learn::q(2, learn::kForward) > 0.0f);
}

void test_a_save_requested_mid_flush_runs_after_it() {
  learn::update(1, learn::kForward, 1.0f, 0);
  learn::requestSave();
  learn::service();
  learn::update(2, learn::kForward, 1.0f, 0);
  learn::requestSave();  // arrives while the first flush is still going
  flush();
  learn::begin();
  TEST_ASSERT_TRUE(learn::q(2, learn::kForward) > 0.0f);
}

void test_whole_blob_storage_saves_in_one_call() {
  g_byteAddressable = false;  // the ESP32's NVS
  learn::update(1, learn::kForward, 1.0f, 0);
  learn::requestSave();
  TEST_ASSERT_EQUAL_INT(1, flush());
  TEST_ASSERT_EQUAL_INT(1, g_blobWrites);
}

void test_corrupt_epsilon_is_rejected_at_boot() {
  learn::update(1, learn::kForward, 1.0f, 0);
  learn::requestSave();
  flush();

  const float bogus = 2.0f;
  std::memcpy(slot(hal::Slot::kQTable) + 8, &bogus, sizeof(bogus));  // magic(4) + 4 header
  learn::begin();
  TEST_ASSERT_FALSE(learn::loadedFromCheckpoint());
  TEST_ASSERT_EQUAL_FLOAT(0.0f, learn::q(1, learn::kForward));
}

void test_a_failed_load_does_not_claim_the_previous_one() {
  learn::update(1, learn::kForward, 1.0f, 0);
  learn::requestSave();
  flush();
  learn::begin();
  TEST_ASSERT_TRUE(learn::loadedFromCheckpoint());

  eraseStorage();
  learn::begin();
  TEST_ASSERT_FALSE(learn::loadedFromCheckpoint());
}

// --- Session history --------------------------------------------------------

void test_history_starts_empty_and_ids_start_at_one() {
  TEST_ASSERT_EQUAL_UINT8(0, history::count());
  history::record(madeRun(4, 1.5f));
  TEST_ASSERT_EQUAL_UINT8(1, history::count());
  TEST_ASSERT_EQUAL_UINT32(1, history::at(0).id);
  TEST_ASSERT_EQUAL_FLOAT(1.5f, history::at(0).forwardMeters);
}

void test_history_keeps_the_newest_runs_in_order() {
  for (int i = 0; i < history::kMax + 3; ++i) {
    history::record(madeRun(i % 2 ? 5 : 4, static_cast<float>(i)));
  }
  TEST_ASSERT_EQUAL_UINT8(history::kMax, history::count());
  // The three oldest fell off; what is left is still oldest-first.
  TEST_ASSERT_EQUAL_FLOAT(3.0f, history::at(0).forwardMeters);
  TEST_ASSERT_EQUAL_FLOAT(static_cast<float>(history::kMax + 2),
                          history::at(history::kMax - 1).forwardMeters);
  for (uint8_t i = 1; i < history::count(); ++i) {
    TEST_ASSERT_EQUAL_UINT32(history::at(i - 1).id + 1, history::at(i).id);
  }
}

void test_history_survives_a_reboot() {
  history::record(madeRun(5, 14.11f));
  history::record(madeRun(4, 3.2f));
  flushHistory();

  history::begin();
  TEST_ASSERT_EQUAL_UINT8(2, history::count());
  TEST_ASSERT_EQUAL_FLOAT(14.11f, history::at(0).forwardMeters);
  TEST_ASSERT_EQUAL_UINT8(4, history::at(1).mode);
  TEST_ASSERT_EQUAL_UINT32(3, history::nextId());
}

void test_clearing_history_keeps_ids_moving_forward() {
  history::record(madeRun(5, 1.0f));
  history::record(madeRun(5, 2.0f));
  history::clear();
  flushHistory();
  TEST_ASSERT_EQUAL_UINT8(0, history::count());
  history::record(madeRun(5, 3.0f));
  TEST_ASSERT_EQUAL_UINT32(3, history::at(0).id);  // not back to 1
}

void test_history_and_q_table_do_not_share_storage() {
  learn::update(4, learn::kTurnLeft, 2.0f, 0);
  learn::requestSave();
  flush();
  history::record(madeRun(5, 9.0f));
  flushHistory();

  learn::begin();
  history::begin();
  TEST_ASSERT_TRUE(learn::loadedFromCheckpoint());
  TEST_ASSERT_TRUE(learn::q(4, learn::kTurnLeft) > 0.0f);
  TEST_ASSERT_EQUAL_UINT8(1, history::count());
  TEST_ASSERT_EQUAL_FLOAT(9.0f, history::at(0).forwardMeters);
}

int main() {
  UNITY_BEGIN();
  RUN_TEST(test_blank_storage_boots_a_blank_table);
  RUN_TEST(test_front_bands_split_at_the_configured_thresholds);
  RUN_TEST(test_side_band_prefers_the_left_wall_like_roaming_does);
  RUN_TEST(test_every_combination_maps_inside_the_table);
  RUN_TEST(test_greedy_choice_takes_the_best_action);
  RUN_TEST(test_ties_break_at_random_not_always_forward);
  RUN_TEST(test_exploration_ignores_the_table);
  RUN_TEST(test_best_never_explores);
  RUN_TEST(test_update_follows_the_q_learning_rule);
  RUN_TEST(test_updates_smaller_than_a_hundredth_are_lost);
  RUN_TEST(test_values_clamp_instead_of_wrapping);
  RUN_TEST(test_epsilon_decays_per_step_down_to_the_floor);
  RUN_TEST(test_reset_forgets_values_and_exploration);
  RUN_TEST(test_checkpoint_survives_a_reboot);
  RUN_TEST(test_a_later_save_writes_only_the_bytes_that_changed);
  RUN_TEST(test_saving_again_with_no_changes_writes_nothing);
  RUN_TEST(test_a_change_made_mid_flush_is_not_lost);
  RUN_TEST(test_a_save_requested_mid_flush_runs_after_it);
  RUN_TEST(test_whole_blob_storage_saves_in_one_call);
  RUN_TEST(test_corrupt_epsilon_is_rejected_at_boot);
  RUN_TEST(test_a_failed_load_does_not_claim_the_previous_one);
  RUN_TEST(test_history_starts_empty_and_ids_start_at_one);
  RUN_TEST(test_history_keeps_the_newest_runs_in_order);
  RUN_TEST(test_history_survives_a_reboot);
  RUN_TEST(test_clearing_history_keeps_ids_moving_forward);
  RUN_TEST(test_history_and_q_table_do_not_share_storage);
  return UNITY_END();
}
