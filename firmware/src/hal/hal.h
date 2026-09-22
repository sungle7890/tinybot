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
// Opaque blobs, one per slot so a checkpoint cannot overwrite the calibration.
// NVS keys on the ESP32, fixed offsets into emulated EEPROM on the R4.
enum class Slot : uint8_t { kCalibration, kQTable, kRuns };
bool persistLoad(Slot slot, void* data, size_t len);
bool persistSave(Slot slot, const void* data, size_t len);
// The R4's emulated EEPROM costs ~44 ms per changed byte (measured), blocking
// the one thread the control loop also runs on, so a large write has to be
// spread out a byte at a time. NVS on the ESP32 has no byte addressing - and
// runs on the other core - so it keeps writing whole blobs.
bool persistByteAddressable();
bool persistWriteByte(Slot slot, size_t offset, uint8_t value);

// --- Control loop -----------------------------------------------------------
void controlLoopBegin(void (*step)(), uint32_t hz);
// Call from loop(). A no-op where the loop runs as its own task.
void controlLoopService();

// Microseconds left before the next tick is due. Work that could overrun the
// budget checks this first. Effectively unlimited where the loop is a task.
uint32_t microsUntilNextTick();

// --- Serial -----------------------------------------------------------------
// True when writing `len` bytes to Serial now cannot delay the next control
// tick. The boards need different answers: the ESP32's Serial buffers and
// reports free space, while the Renesas UART blocks in write() until every
// byte is on the wire and does not implement availableForWrite() at all.
bool serialWriteFitsBeforeNextTick(size_t len);

// --- Misc -------------------------------------------------------------------
// Why the board last started. Read once, at the top of setup(), before
// anything can clear it: a run that ends with the robot simply gone - no
// session in the log, the tick counter back at zero - looks the same whether
// the battery sagged under the motors or the firmware faulted, and the chip
// knows which it was.
void captureResetCause();
// Restart the board. Used to check that the flags above are read correctly,
// and to recover a robot that is idle but wedged, without the power switch.
void reboot();
const char* resetCause();
uint32_t resetBits();

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
