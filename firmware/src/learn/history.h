#pragma once
#include <stdint.h>

// A log of finished driving sessions, kept on the robot.
//
// Rule-based roaming and learning have to be compared on the same numbers, and
// those numbers are easy to lose: the radio drops out, the robot drives out of
// range, someone pulls the battery. So every session that ends records what it
// did, in EEPROM, and `h` prints them back.
//
// Nothing here judges a run. It stores what happened.
namespace history {

constexpr uint8_t kMax = 10;  // oldest is dropped

struct Run {
  uint32_t id;           // 1, 2, 3 ... across the robot's whole life
  uint8_t mode;          // control::Mode, so auto and learn can be told apart
  uint8_t contacts;
  uint8_t escapes;
  uint8_t stopReason;    // StopReason below
  uint16_t seconds;
  uint16_t steps;        // learning decisions; 0 for rule-based roaming
  float forwardMeters;
  float meanReward;
  float epsilonEnd;
};

enum StopReason : uint8_t {
  kStopUnknown = 0,
  kStopCommand = 1,   // `s`, or any mode change from the console
  kStopStuck = 2,     // no progress for cfg::kAutoStuckMs
  kStopTimeCap = 3,   // session limit
  kStopSafety = 4,    // a latched fault
};

void begin();

// Appends a run and asks for a save. Called when a session ends.
void record(const Run& run);

uint8_t count();                 // how many are stored, newest last
const Run& at(uint8_t index);    // 0 = oldest kept
uint32_t nextId();

void clear();

// Same trickled save as the Q-table: called every loop() pass.
void service();
bool savePending();

const char* stopReasonName(uint8_t reason);

}  // namespace history
