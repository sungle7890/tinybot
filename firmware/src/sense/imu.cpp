#include "sense/imu.h"

#include <Arduino.h>
#include <math.h>

#include "config.h"
#include "hal/hal.h"

namespace imu {
namespace {

constexpr uint8_t kAddress = 0x68;

constexpr uint8_t kRegSmplrtDiv = 0x19;
constexpr uint8_t kRegConfig = 0x1A;
constexpr uint8_t kRegGyroConfig = 0x1B;
constexpr uint8_t kRegAccelConfig = 0x1C;
constexpr uint8_t kRegAccelXoutH = 0x3B;
constexpr uint8_t kRegPwrMgmt1 = 0x6B;
constexpr uint8_t kRegWhoAmI = 0x75;

// +/-4 g and +/-500 dps full scale.
constexpr float kAccelLsbPerG = 8192.0f;
constexpr float kGyroLsbPerDps = 65.5f;

// Baseline follows slow orientation change but not an impact. At 50 Hz this is
// roughly a 0.5 s time constant.
constexpr float kBaselineAlpha = 0.04f;
constexpr float kImpactThresholdG = 0.6f;

bool g_present = false;
uint8_t g_whoAmI = 0;

float g_ax = 0.0f, g_ay = 0.0f, g_az = 0.0f;
float g_yawRate = 0.0f;
float g_bias = 0.0f;
bool g_calibrated = false;
uint16_t g_calibrateLeft = 0;
float g_calibrateSum = 0.0f;
float g_calibrateMax = 0.0f;
float g_headingDeg = 0.0f;
uint32_t g_lastUpdateMs = 0;
float g_baseX = 0.0f, g_baseY = 0.0f, g_baseZ = 0.0f;
bool g_baselineSeeded = false;
float g_shock = 0.0f;

bool writeReg(uint8_t reg, uint8_t value) {
  hal::i2c().beginTransmission(kAddress);
  hal::i2c().write(reg);
  hal::i2c().write(value);
  return hal::i2c().endTransmission() == 0;
}

bool readRegs(uint8_t reg, uint8_t* out, uint8_t len) {
  hal::i2c().beginTransmission(kAddress);
  hal::i2c().write(reg);
  if (hal::i2c().endTransmission(false) != 0) return false;
  if (hal::i2c().requestFrom(kAddress, len) != len) return false;
  for (uint8_t i = 0; i < len; ++i) out[i] = hal::i2c().read();
  return true;
}

int16_t be16(const uint8_t* p) {
  return static_cast<int16_t>((static_cast<uint16_t>(p[0]) << 8) | p[1]);
}

}  // namespace

bool begin() {
  uint8_t who = 0;
  if (!readRegs(kRegWhoAmI, &who, 1)) return false;
  g_whoAmI = who;

  if (!writeReg(kRegPwrMgmt1, 0x80)) return false;  // device reset
  delay(100);
  if (!writeReg(kRegPwrMgmt1, 0x01)) return false;  // wake, PLL on gyro X
  delay(10);

  writeReg(kRegConfig, 0x03);       // DLPF 44 Hz - well below the 50 Hz loop
  writeReg(kRegSmplrtDiv, 0x04);    // 1 kHz / 5 = 200 Hz sample rate
  writeReg(kRegGyroConfig, 0x08);   // +/-500 dps
  writeReg(kRegAccelConfig, 0x08);  // +/-4 g

  g_present = true;
  g_baselineSeeded = false;
  return true;
}

bool present() { return g_present; }
uint8_t whoAmI() { return g_whoAmI; }

void update() {
  if (!g_present) return;

  uint8_t buf[14];
  if (!readRegs(kRegAccelXoutH, buf, sizeof(buf))) return;

  g_ax = be16(&buf[0]) / kAccelLsbPerG;
  g_ay = be16(&buf[2]) / kAccelLsbPerG;
  g_az = be16(&buf[4]) / kAccelLsbPerG;
  // buf[6..7] is temperature; buf[8..13] is gyro X/Y/Z.
  const float rawYaw = be16(&buf[12]) / kGyroLsbPerDps;

  if (g_calibrateLeft > 0) {
    g_calibrateSum += rawYaw;
    const float swing = fabsf(rawYaw - (g_calibrateSum / (cfg::kGyroBiasSamples - g_calibrateLeft + 1)));
    if (swing > g_calibrateMax) g_calibrateMax = swing;
    if (--g_calibrateLeft == 0) {
      // Moving during the measurement would bake that motion into the bias.
      if (g_calibrateMax < cfg::kGyroStillDps) {
        g_bias = g_calibrateSum / cfg::kGyroBiasSamples;
        g_calibrated = true;
      }
    }
  }

  g_yawRate = rawYaw - g_bias;

  // Integrate on measured time, not on the nominal tick: a stretched tick
  // would otherwise under-count the turn it contains.
  const uint32_t now = millis();
  if (g_lastUpdateMs != 0 && g_calibrateLeft == 0) {
    const float dt = (now - g_lastUpdateMs) / 1000.0f;
    if (dt > 0.0f && dt < 0.5f) g_headingDeg += g_yawRate * dt;
  }
  g_lastUpdateMs = now;

  if (!g_baselineSeeded) {
    g_baseX = g_ax;
    g_baseY = g_ay;
    g_baseZ = g_az;
    g_baselineSeeded = true;
  } else {
    g_baseX += kBaselineAlpha * (g_ax - g_baseX);
    g_baseY += kBaselineAlpha * (g_ay - g_baseY);
    g_baseZ += kBaselineAlpha * (g_az - g_baseZ);
  }

  const float dx = g_ax - g_baseX;
  const float dy = g_ay - g_baseY;
  const float dz = g_az - g_baseZ;
  g_shock = sqrtf(dx * dx + dy * dy + dz * dz);
}

float accelX() { return g_ax; }
float accelY() { return g_ay; }
float accelZ() { return g_az; }
float yawRateDps() { return g_yawRate; }
float headingDeg() { return g_headingDeg; }
void zeroHeading() { g_headingDeg = 0.0f; }

void calibrate() {
  g_calibrateLeft = cfg::kGyroBiasSamples;
  g_calibrateSum = 0.0f;
  g_calibrateMax = 0.0f;
  g_calibrated = false;
}

bool calibrating() { return g_calibrateLeft > 0; }
bool calibrated() { return g_calibrated; }
float biasDps() { return g_bias; }
float shock() { return g_shock; }
bool impact() { return g_present && g_shock > kImpactThresholdG; }

}  // namespace imu
