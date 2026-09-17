#include "learn/history.h"

#include <Arduino.h>
#include <string.h>

#include "hal/hal.h"
#include "store/trickle.h"

namespace history {
namespace {

constexpr uint32_t kMagic = 0x54425231;  // "TBR1"
constexpr uint8_t kVersion = 1;

struct Book {
  uint32_t magic;
  uint8_t version;
  uint8_t count;
  uint16_t reserved;
  uint32_t nextId;
  Run runs[kMax];
};

Book g_book{};
Book g_mirror{};
store::Trickle g_store(hal::Slot::kRuns, &g_book, &g_mirror, sizeof(Book));

void blank() {
  memset(&g_book, 0, sizeof(g_book));
  g_book.magic = kMagic;
  g_book.version = kVersion;
  g_book.nextId = 1;
}

}  // namespace

void begin() {
  Book stored{};
  if (hal::persistLoad(hal::Slot::kRuns, &stored, sizeof(stored)) &&
      stored.magic == kMagic && stored.version == kVersion &&
      stored.count <= kMax && stored.nextId > 0) {
    g_book = stored;
  } else {
    blank();
  }
  g_store.seedMirror();
}

void record(const Run& run) {
  Run copy = run;
  copy.id = g_book.nextId++;
  if (g_book.count < kMax) {
    g_book.runs[g_book.count++] = copy;
  } else {
    // Drop the oldest. A memmove of ten records is cheap, and keeping them in
    // order means the save only rewrites the bytes that actually moved.
    memmove(&g_book.runs[0], &g_book.runs[1], sizeof(Run) * (kMax - 1));
    g_book.runs[kMax - 1] = copy;
  }
  g_store.requestSave();
}

uint8_t count() { return g_book.count; }
const Run& at(uint8_t index) { return g_book.runs[index]; }
uint32_t nextId() { return g_book.nextId; }

void clear() {
  const uint32_t keepId = g_book.nextId;
  blank();
  g_book.nextId = keepId;  // ids never repeat, even after a clear
  g_store.requestSave();
}

void service() { g_store.service(); }
bool savePending() { return g_store.pending(); }

const char* stopReasonName(uint8_t reason) {
  switch (reason) {
    case kStopCommand: return "stopped";
    case kStopStuck: return "stuck";
    case kStopTimeCap: return "time cap";
    case kStopSafety: return "safety";
    default: return "?";
  }
}

}  // namespace history
