#pragma once
#include <stdint.h>

// ============================================================================
//  PIN MAP - Arduino UNO R4 WiFi  (PROVISIONAL, verify against the board)
// ----------------------------------------------------------------------------
//  Two constraints drove this layout:
//
//  1. The R4 WiFi officially supports external interrupts on D2 and D3 ONLY.
//     Those two go to the encoder A channels, which are the only signals fast
//     enough to need them. The bumpers are polled inside the control step
//     instead - a chassis pressed against a wall holds the switch closed for
//     far longer than one 20 ms tick, so nothing is missed in practice.
//
//  2. I2C runs on the Qwiic connector, which is a SECOND bus (Wire1) and is
//     3.3 V only. That costs nothing in header pins and frees A4/A5, which is
//     what makes this map fit at all. Set cfg::kUseQwiic false to move to the
//     5 V header bus on A4/A5, and then move the right bumper to D13.
//
//  Reserved: D0/D1 are UART0 (the USB console). The 12x8 LED matrix is
//  charlieplexed on dedicated MCU pins and costs no header GPIO.
// ============================================================================

namespace pins {

// --- TB6612FNG motor driver -------------------------------------------------
constexpr uint8_t kMotorStandby = 11;

constexpr uint8_t kLeftPwm  = 5;   // PWMA - PWM capable
constexpr uint8_t kLeftIn1  = 7;   // AIN1
constexpr uint8_t kLeftIn2  = 8;   // AIN2

constexpr uint8_t kRightPwm = 6;   // PWMB - PWM capable
constexpr uint8_t kRightIn1 = 9;   // BIN1
constexpr uint8_t kRightIn2 = 10;  // BIN2

// --- Quadrature encoders ----------------------------------------------------
// A channels MUST be D2/D3: the only interrupt-capable pins on this board.
constexpr uint8_t kLeftEncA  = 2;
constexpr uint8_t kLeftEncB  = 4;
constexpr uint8_t kRightEncA = 3;
constexpr uint8_t kRightEncB = 12;

// --- I2C --------------------------------------------------------------------
// Unused when cfg::kUseQwiic is true; Wire1 owns the Qwiic connector and needs
// no header pins. Listed for the fallback wiring.
constexpr uint8_t kI2cSda = A4;
constexpr uint8_t kI2cScl = A5;

// --- VL53L0X XSHUT ----------------------------------------------------------
// Qwiic carries only SDA/SCL/3V3/GND, so XSHUT still needs one wire each.
constexpr uint8_t kTofXshutFront = A0;
constexpr uint8_t kTofXshutLeft  = A1;
constexpr uint8_t kTofXshutRight = A2;

// --- Safety inputs ----------------------------------------------------------
// Polled in the control step, not interrupt-driven. See the note above.
constexpr uint8_t kBumperLeft  = A3;
constexpr uint8_t kBumperRight = A4;   // free because I2C is on Qwiic

// Spare: A5, D13 (D13 drives the onboard LED - poor choice for an input).

}  // namespace pins
