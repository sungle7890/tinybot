#pragma once
#include <Arduino.h>
#include <Wire.h>
#include <stdint.h>

// Everything platform-specific lives behind this interface. The two
// implementations are hal_esp32.cpp and hal_r4.cpp; platformio.ini excludes
// the one that does not apply.
//
// The boards differ most in how the control loop is driven:
//   ESP32-S3  a FreeRTOS task pinned to its own core, independent of loop()
//   UNO R4    cooperative - a micros() deadline checked from loop()
// Both are exercised through controlLoopBegin() + controlLoopService(), and
// the jitter statistics in control_loop.cpp measure whichever one is running.
// Do not assume the cooperative one is good enough; check the `j` command.
namespace hal {

// --- Interrupt-safe sections ------------------------------------------------
// RAII. Keep the scope tiny: on the R4 this disables interrupts outright.
class CriticalSection {
 public:
  CriticalSection();
  ~CriticalSection();
  CriticalSection(const CriticalSection&) = delete;
  CriticalSection& operator=(const CriticalSection&) = delete;
};

// Same, for use inside an ISR.
class CriticalSectionIsr {
 public:
  CriticalSectionIsr();
  ~CriticalSectionIsr();
  CriticalSectionIsr(const CriticalSectionIsr&) = delete;
  CriticalSectionIsr& operator=(const CriticalSectionIsr&) = delete;
};

// --- PWM --------------------------------------------------------------------
// Resolution is hidden: callers pass a fraction as duty/dutyMax.
void pwmBegin(uint8_t pin);
void pwmWrite(uint8_t pin, uint16_t duty, uint16_t dutyMax);

// --- GPIO -------------------------------------------------------------------
// Safe to call from an ISR on both platforms.
int fastRead(uint8_t pin);

// --- Persistence ------------------------------------------------------------
// One opaque blob. NVS on the ESP32, emulated EEPROM on the R4.
bool persistLoad(void* data, size_t len);
bool persistSave(const void* data, size_t len);

// --- Control loop -----------------------------------------------------------
void controlLoopBegin(void (*step)(), uint32_t hz);
// Call from loop(). A no-op where the loop runs as its own task.
void controlLoopService();

// --- Misc -------------------------------------------------------------------
uint32_t freeBytes();
const char* boardName();

// I2C bus the sensors live on. Already begun by i2cBegin().
TwoWire& i2c();
void i2cBegin(uint32_t frequencyHz);

}  // namespace hal

// Attribute for ISR functions (ESP32 wants them resident in IRAM).
#if defined(TINYBOT_PLATFORM_ESP32)
#define HAL_ISR IRAM_ATTR
#else
#define HAL_ISR
#endif
