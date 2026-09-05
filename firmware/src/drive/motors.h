#pragma once
#include <stdint.h>

// TB6612FNG driver. Duty is signed, -1000..+1000; positive is forward.
namespace motors {

void begin();

// Applies the deadband from cfg::kDutyDeadband: any non-zero request smaller
// than the deadband is raised to it, so a "move slowly" command actually moves.
void setLeft(int16_t duty);
void setRight(int16_t duty);
void set(int16_t leftDuty, int16_t rightDuty);

void coast();   // both inputs low - the robot rolls
void brake();   // both inputs high - short brake
void enable(bool on);   // STBY pin

int16_t leftDuty();
int16_t rightDuty();

}  // namespace motors
