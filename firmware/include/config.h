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

// --- Telemetry --------------------------------------------------------------
constexpr size_t kTelemetryQueueLen = 32;
constexpr size_t kTelemetryLineMax  = 128;

}  // namespace cfg
