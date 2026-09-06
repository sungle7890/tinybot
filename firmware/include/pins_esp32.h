#pragma once
#include <stdint.h>

// ============================================================================
//  PIN MAP - PROVISIONAL
// ----------------------------------------------------------------------------
//  Chosen for an ESP32-S3-DevKitC-1. Verify against the actual board before
//  wiring anything: pin availability differs between S3 modules.
//
//  Deliberately avoided:
//    GPIO 0, 3, 45, 46   strapping pins
//    GPIO 19, 20         native USB D-/D+
//    GPIO 26..32         SPI flash
//    GPIO 33..37         octal PSRAM on N8R8/N16R8 modules
//    GPIO 43, 44         UART0 (the USB serial console)
//
//  Keep this in sync with hardware/wiring.md.
// ============================================================================

namespace pins {

// --- TB6612FNG motor driver -------------------------------------------------
constexpr uint8_t kMotorStandby = 17;

constexpr uint8_t kLeftPwm  = 4;   // PWMA
constexpr uint8_t kLeftIn1  = 5;   // AIN1
constexpr uint8_t kLeftIn2  = 6;   // AIN2

constexpr uint8_t kRightPwm = 7;   // PWMB
constexpr uint8_t kRightIn1 = 15;  // BIN1
constexpr uint8_t kRightIn2 = 16;  // BIN2

// --- Quadrature encoders ----------------------------------------------------
// Channel A is the interrupt pin; B is sampled inside the ISR for direction.
constexpr uint8_t kLeftEncA  = 8;
constexpr uint8_t kLeftEncB  = 9;
constexpr uint8_t kRightEncA = 10;
constexpr uint8_t kRightEncB = 11;

// --- I2C (VL53L0X x3, MPU6050) ----------------------------------------------
constexpr uint8_t kI2cSda = 13;
constexpr uint8_t kI2cScl = 14;

// --- VL53L0X XSHUT ----------------------------------------------------------
// All three sensors boot at 0x29. They are brought up one at a time and
// readdressed; see src/sense/tof.cpp.
constexpr uint8_t kTofXshutFront = 1;
constexpr uint8_t kTofXshutLeft  = 2;
constexpr uint8_t kTofXshutRight = 42;

// --- Safety inputs ----------------------------------------------------------
// Normally-open to GND, INPUT_PULLUP. These are hard interrupts, never
// policy inputs (docs/02-architecture.md design rule 3).
constexpr uint8_t kBumperLeft  = 41;
constexpr uint8_t kBumperRight = 40;

}  // namespace pins
