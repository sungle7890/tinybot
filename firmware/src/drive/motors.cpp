#include "drive/motors.h"

#include <Arduino.h>

#include "config.h"
#include "hal/hal.h"
#include "pins.h"

namespace motors {
namespace {

int16_t g_leftDuty = 0;
int16_t g_rightDuty = 0;

int16_t applyDeadband(int16_t duty) {
  if (duty == 0) return 0;
  const int16_t magnitude = duty > 0 ? duty : static_cast<int16_t>(-duty);
  const int16_t corrected =
      magnitude < cfg::kDutyDeadband ? cfg::kDutyDeadband : magnitude;
  return duty > 0 ? corrected : static_cast<int16_t>(-corrected);
}

int16_t clampDuty(int16_t duty) {
  if (duty > cfg::kDutyMax) return cfg::kDutyMax;
  if (duty < -cfg::kDutyMax) return -cfg::kDutyMax;
  return duty;
}

void driveSide(int16_t duty, uint8_t in1, uint8_t in2, uint8_t pwmPin) {
  const bool forward = duty >= 0;
  const uint16_t magnitude = forward ? duty : -duty;

  digitalWrite(in1, forward ? HIGH : LOW);
  digitalWrite(in2, forward ? LOW : HIGH);
  hal::pwmWrite(pwmPin, magnitude, cfg::kDutyMax);
}

}  // namespace

void begin() {
  const uint8_t outputs[] = {pins::kMotorStandby, pins::kLeftIn1, pins::kLeftIn2,
                             pins::kRightIn1, pins::kRightIn2};
  for (uint8_t pin : outputs) {
    pinMode(pin, OUTPUT);
    digitalWrite(pin, LOW);
  }

  hal::pwmBegin(pins::kLeftPwm);
  hal::pwmBegin(pins::kRightPwm);

  coast();
  enable(true);
}

void setLeft(int16_t duty) {
  g_leftDuty = clampDuty(applyDeadband(duty));
  driveSide(g_leftDuty, pins::kLeftIn1, pins::kLeftIn2, pins::kLeftPwm);
}

void setRight(int16_t duty) {
  g_rightDuty = clampDuty(applyDeadband(duty));
  driveSide(g_rightDuty, pins::kRightIn1, pins::kRightIn2, pins::kRightPwm);
}

void set(int16_t leftDuty, int16_t rightDuty) {
  setLeft(leftDuty);
  setRight(rightDuty);
}

void coast() {
  g_leftDuty = 0;
  g_rightDuty = 0;
  digitalWrite(pins::kLeftIn1, LOW);
  digitalWrite(pins::kLeftIn2, LOW);
  digitalWrite(pins::kRightIn1, LOW);
  digitalWrite(pins::kRightIn2, LOW);
  hal::pwmWrite(pins::kLeftPwm, 0, cfg::kDutyMax);
  hal::pwmWrite(pins::kRightPwm, 0, cfg::kDutyMax);
}

void brake() {
  g_leftDuty = 0;
  g_rightDuty = 0;
  digitalWrite(pins::kLeftIn1, HIGH);
  digitalWrite(pins::kLeftIn2, HIGH);
  digitalWrite(pins::kRightIn1, HIGH);
  digitalWrite(pins::kRightIn2, HIGH);
  hal::pwmWrite(pins::kLeftPwm, cfg::kDutyMax, cfg::kDutyMax);
  hal::pwmWrite(pins::kRightPwm, cfg::kDutyMax, cfg::kDutyMax);
}

void enable(bool on) { digitalWrite(pins::kMotorStandby, on ? HIGH : LOW); }

int16_t leftDuty() { return g_leftDuty; }
int16_t rightDuty() { return g_rightDuty; }

}  // namespace motors
