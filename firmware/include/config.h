#pragma once
#include <stdint.h>

namespace cfg {

// --- Control loop -----------------------------------------------------------
// 50 Hz matches the reference design in docs/01-microduck-analysis.md.
constexpr uint32_t kLoopHz       = 50;
constexpr uint32_t kLoopPeriodMs = 1000 / kLoopHz;  // 20 ms
constexpr int      kControlCore  = 0;  // ESP32 only; the R4 has one core

// Phase 1 exit criterion: jitter within +/-2 ms of the period.
constexpr uint32_t kJitterBudgetUs = 2000;

// --- Board options ----------------------------------------------------------
#if defined(TINYBOT_PLATFORM_R4)
// Qwiic is a second I2C bus (Wire1) at 3.3 V. Using it costs no header pins,
// which is what lets the pin map fit. Set false to fall back to the 5 V header
// bus on A4/A5 - and then move kBumperRight off A4 (see pins_r4.h).
constexpr bool kUseQwiic = true;
#endif

// --- Motors -----------------------------------------------------------------
// Duty is a platform-independent fraction; hal::pwmWrite hides the resolution.
constexpr int16_t kDutyMax = 1000;   // public API range is -1000..+1000

// The pack drives the motors directly, so duty sets their average voltage:
// average = supply x duty. The Romi's 120:1 mini plastic gearmotors (Pololu
// 1520) are rated 4.5 V and specified to run comfortably over 3-6 V, while six
// 1.5 V cells give 9 V, so full duty would be half again over the rating -
// hotter motors, faster brush wear, and a stall current that climbs with it.
// kDutyCeiling caps every motor command so the average stays within
// kMotorMaxVolts. Re-measure kSupplyVolts at the driver and it re-derives.
constexpr float kSupplyVolts = 9.0f;
constexpr float kMotorMaxVolts = 6.0f;

constexpr int16_t dutyCeilingFor(float supply, float motorMax) {
  return supply <= motorMax
             ? kDutyMax
             : static_cast<int16_t>(static_cast<float>(kDutyMax) * motorMax / supply);
}
constexpr int16_t kDutyCeiling = dutyCeilingFor(kSupplyVolts, kMotorMaxVolts);

// Below this the motors whine without turning. Measured per chassis; the
// default is a starting guess to be corrected during bring-up.
constexpr int16_t kDutyDeadband = 120;

// --- Encoders ---------------------------------------------------------------
// Romi encoders give 12 counts per motor revolution counting both edges of both
// channels. This firmware decodes 2x (both edges of channel A only), so:
//   6 counts/motor-rev x 120:1 gearbox = 720 per wheel revolution
//   70 mm wheel -> 0.2199 m circumference
//   720 / 0.2199 = 3274 counts/m
// A computed starting point, not a measurement. Run `cal` and overwrite it.
constexpr float kDefaultCountsPerMeter = 3274.0f;

// --- Encoder sanity ---------------------------------------------------------
// The motors top out near 200 rpm at the wheel, which is about 48 counts per
// 20 ms tick, so anything far above that is noise on a floating input rather
// than motion - exactly what a disconnected encoder ground produces.
constexpr int32_t kMaxCountsPerTick = 150;

// Duty applied but nothing turning: either the robot is stalled against
// something or an encoder signal has come loose. Both want the motors off.
constexpr uint16_t kStallTicks = 25;          // 0.5 s at 50 Hz
constexpr int32_t kStallCountsPerTick = 2;

// During a straight drive the wheels track each other closely. This much
// divergence (about 12 cm) means one side is not being measured.
constexpr int32_t kDriveMismatchCounts = 400;

// --- Straight-line drive (calibration aid) ----------------------------------
constexpr int16_t  kDriveDuty      = 400;   // gentle; calibration is not a race
constexpr float    kHeadingKp      = 1.2f;  // duty per count of L/R divergence
constexpr int16_t  kHeadingCorrMax = 250;
constexpr uint32_t kDriveTimeoutMs = 15000;

// --- Sensors ----------------------------------------------------------------
constexpr uint32_t kI2cFreqHz         = 400000;
constexpr uint32_t kTofTimingBudgetUs = 20000;  // one measurement per loop tick
constexpr uint16_t kTofOutOfRangeMm   = 8190;   // VL53L0X saturation value
// A reading older than this is stale; the control loop must treat it as unknown
// rather than as "nothing in front of me".
constexpr uint32_t kSensorStaleMs = 200;

// --- Serial -----------------------------------------------------------------
constexpr unsigned long kSerialBaud = 115200;

// --- Telemetry --------------------------------------------------------------
constexpr size_t kTelemetryQueueLen = 32;
constexpr size_t kTelemetryLineMax  = 128;

}  // namespace cfg
