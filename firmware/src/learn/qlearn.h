#pragma once
#include <stdint.h>

// Tabular Q-learning, small enough to live in 300 bytes of EEPROM.
//
// This module is pure bookkeeping: it never touches a motor or a sensor. The
// control loop turns ranges into a state, asks for an action, drives it for
// one step, measures what happened and hands back a reward. Keeping the
// hardware out makes the learner the one thing here that could be unit-tested
// on the host.
namespace learn {

enum Action : uint8_t { kForward = 0, kTurnLeft = 1, kTurnRight = 2, kBack = 3 };
constexpr uint8_t kActionCount = 4;
constexpr uint8_t kStateCount = 36;  // see cfg::kLearnNearMm

// Loads the checkpoint if one is stored, otherwise starts from a blank table.
void begin();

uint8_t encodeState(uint16_t frontMm, uint16_t leftMm, uint16_t rightMm,
                    Action previous);

// Epsilon-greedy. Ties break at random, or a blank table always picks forward.
Action choose(uint8_t state);

// One Q-learning update, then epsilon decays.
void update(uint8_t state, Action action, float reward, uint8_t nextState);

// Forget everything learned, including the decayed epsilon.
void reset();

// Saving is slow on the R4 and must not run inside a control tick, so the
// loop only asks for it; service(), called every loop() pass, trickles it out.
void requestSave();
bool savePending();
void service();

float q(uint8_t state, uint8_t action);
float epsilon();
uint32_t steps();
bool loadedFromCheckpoint();
// A checkpoint trickles out a byte per loop() pass while the robot keeps
// driving, so these are how long a save took to complete, not a freeze.
uint32_t lastSaveMs();
uint32_t longestSaveMs();
uint16_t lastSaveBytes();  // bytes that actually changed in the last save
uint32_t saves();

const char* actionName(uint8_t action);

}  // namespace learn
