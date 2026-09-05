#pragma once
#include <stdint.h>

namespace cfg {

// --- Control loop -----------------------------------------------------------
// 50 Hz matches the reference design in docs/01-microduck-analysis.md.
// The loop runs pinned to one core so comms cannot jitter it.
constexpr uint32_t kLoopHz       = 50;
constexpr uint32_t kLoopPeriodMs = 1000 / kLoopHz;  // 20 ms
constexpr int      kControlCore  = 0;               // Arduino loop() owns core 1

// Phase 1 exit criterion: jitter within +/-2 ms of the period.
constexpr uint32_t kJitterBudgetUs = 2000;

// --- Motors -----------------------------------------------------------------
constexpr uint32_t kPwmFreqHz  = 20000;  // above audible
constexpr uint8_t  kPwmResBits = 10;     // 0..1023
constexpr int16_t  kDutyMax    = 1000;   // public API range is -1000..+1000

// Below this the motors whine without turning. Measured per chassis; the
// default is a starting guess to be corrected during bring-up.
constexpr int16_t kDutyDeadband = 120;

// --- Encoders ---------------------------------------------------------------
// PROVISIONAL. The real value comes from the `cal` console command and is
// persisted in NVS. Do not trust this number for anything.
constexpr float kDefaultCountsPerMeter = 3000.0f;

// --- Straight-line drive (calibration aid) ----------------------------------
constexpr int16_t kDriveDuty      = 400;   // gentle; calibration is not a race
constexpr float   kHeadingKp      = 1.2f;  // duty per count of L/R divergence
constexpr int16_t kHeadingCorrMax = 250;
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

}  // namespace cfg
