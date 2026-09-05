#include "drive/motors.h"

#include <Arduino.h>

#include "config.h"
#include "pins.h"

namespace motors {
namespace {

constexpr uint8_t kLeftChannel = 0;
constexpr uint8_t kRightChannel = 1;
constexpr int16_t kPwmMax = (1 << cfg::kPwmResBits) - 1;

int16_t g_leftDuty = 0;
int16_t g_rightDuty = 0;

void attachPwm(uint8_t pin, uint8_t channel) {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  (void)channel;
  ledcAttach(pin, cfg::kPwmFreqHz, cfg::kPwmResBits);
#else
  ledcSetup(channel, cfg::kPwmFreqHz, cfg::kPwmResBits);
  ledcAttachPin(pin, channel);
#endif
}

void writePwm(uint8_t pin, uint8_t channel, uint32_t value) {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  (void)channel;
  ledcWrite(pin, value);
#else
  (void)pin;
  ledcWrite(channel, value);
#endif
}

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

void driveSide(int16_t duty, uint8_t in1, uint8_t in2, uint8_t pwmPin,
               uint8_t channel) {
  const bool forward = duty >= 0;
  const int32_t magnitude = forward ? duty : -duty;

  digitalWrite(in1, forward ? HIGH : LOW);
  digitalWrite(in2, forward ? LOW : HIGH);
  writePwm(pwmPin, channel,
           static_cast<uint32_t>(magnitude * kPwmMax / cfg::kDutyMax));
}

}  // namespace

void begin() {
  const uint8_t outputs[] = {pins::kMotorStandby, pins::kLeftIn1, pins::kLeftIn2,
                             pins::kRightIn1, pins::kRightIn2};
  for (uint8_t pin : outputs) {
    pinMode(pin, OUTPUT);
    digitalWrite(pin, LOW);
  }

  attachPwm(pins::kLeftPwm, kLeftChannel);
  attachPwm(pins::kRightPwm, kRightChannel);

  coast();
  enable(true);
}

void setLeft(int16_t duty) {
  g_leftDuty = clampDuty(applyDeadband(duty));
  driveSide(g_leftDuty, pins::kLeftIn1, pins::kLeftIn2, pins::kLeftPwm,
            kLeftChannel);
}

void setRight(int16_t duty) {
  g_rightDuty = clampDuty(applyDeadband(duty));
  driveSide(g_rightDuty, pins::kRightIn1, pins::kRightIn2, pins::kRightPwm,
            kRightChannel);
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
  writePwm(pins::kLeftPwm, kLeftChannel, 0);
  writePwm(pins::kRightPwm, kRightChannel, 0);
}

void brake() {
  g_leftDuty = 0;
  g_rightDuty = 0;
  digitalWrite(pins::kLeftIn1, HIGH);
  digitalWrite(pins::kLeftIn2, HIGH);
  digitalWrite(pins::kRightIn1, HIGH);
  digitalWrite(pins::kRightIn2, HIGH);
  writePwm(pins::kLeftPwm, kLeftChannel, kPwmMax);
  writePwm(pins::kRightPwm, kRightChannel, kPwmMax);
}

void enable(bool on) { digitalWrite(pins::kMotorStandby, on ? HIGH : LOW); }

int16_t leftDuty() { return g_leftDuty; }
int16_t rightDuty() { return g_rightDuty; }

}  // namespace motors
