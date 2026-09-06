#if defined(TINYBOT_PLATFORM_R4)

#include "hal/hal.h"

#include <EEPROM.h>

#include "config.h"

// newlib's heap break. Declared here rather than inside namespace hal, or the
// linker looks for hal::sbrk.
extern "C" char* sbrk(int increment);

namespace hal {
namespace {

constexpr int kEepromBase = 0;

void (*g_step)() = nullptr;
uint32_t g_periodUs = 0;
uint32_t g_nextDueUs = 0;
bool g_running = false;

uint8_t g_nestingDepth = 0;

}  // namespace

// --- Interrupt-safe sections ------------------------------------------------
// The RA4M1 has no spinlock primitive and only one core, so this is a plain
// interrupt disable. Nesting is counted so an inner scope cannot re-enable
// interrupts an outer scope still needs off.
CriticalSection::CriticalSection() {
  noInterrupts();
  ++g_nestingDepth;
}

CriticalSection::~CriticalSection() {
  if (--g_nestingDepth == 0) interrupts();
}

// Already inside an ISR: interrupts are off, so this is a no-op.
CriticalSectionIsr::CriticalSectionIsr() {}
CriticalSectionIsr::~CriticalSectionIsr() {}

// --- PWM --------------------------------------------------------------------
// The R4 core exposes no frequency control for analogWrite, so the carrier
// stays at the core default - low enough to be audible as motor whine. It
// drives the TB6612 correctly; if the noise becomes a problem the fix is an
// FspTimer-driven PWM output, local to this file.
void pwmBegin(uint8_t pin) {
  pinMode(pin, OUTPUT);
  analogWrite(pin, 0);
}

void pwmWrite(uint8_t pin, uint16_t duty, uint16_t dutyMax) {
  const uint32_t scaled =
      dutyMax == 0 ? 0 : (static_cast<uint32_t>(duty) * 255u) / dutyMax;
  analogWrite(pin, static_cast<int>(scaled));
}

// --- GPIO -------------------------------------------------------------------
int fastRead(uint8_t pin) { return digitalRead(pin); }

// --- Persistence ------------------------------------------------------------
bool persistLoad(void* data, size_t len) {
  if (kEepromBase + static_cast<int>(len) > EEPROM.length()) return false;
  uint8_t* out = static_cast<uint8_t*>(data);
  for (size_t i = 0; i < len; ++i) out[i] = EEPROM.read(kEepromBase + i);
  return true;
}

bool persistSave(const void* data, size_t len) {
  if (kEepromBase + static_cast<int>(len) > EEPROM.length()) return false;
  const uint8_t* in = static_cast<const uint8_t*>(data);
  // EEPROM.update only writes changed bytes, which matters: the RA4M1 data
  // flash has a finite erase count and the Q-table will checkpoint often.
  for (size_t i = 0; i < len; ++i) EEPROM.update(kEepromBase + i, in[i]);
  return true;
}

// --- Control loop -----------------------------------------------------------
// Cooperative. Nothing in loop() may block, or the tick slips - which is why
// telemetry printing checks Serial.availableForWrite() before writing.
void controlLoopBegin(void (*step)(), uint32_t hz) {
  g_step = step;
  g_periodUs = 1000000ul / hz;
  g_nextDueUs = micros() + g_periodUs;
  g_running = true;
}

void controlLoopService() {
  if (!g_running || g_step == nullptr) return;

  // Signed comparison so the unsigned wrap at ~71 minutes is handled.
  if (static_cast<int32_t>(micros() - g_nextDueUs) < 0) return;

  g_nextDueUs += g_periodUs;
  g_step();

  // If a tick was missed badly, resynchronise rather than trying to catch up
  // by running the step repeatedly - a burst of back-to-back steps would be
  // worse for the robot than one skipped period. The skip shows up in the
  // jitter statistics either way.
  if (static_cast<int32_t>(micros() - g_nextDueUs) > 0) {
    g_nextDueUs = micros() + g_periodUs;
  }
}

// --- Misc -------------------------------------------------------------------
uint32_t freeBytes() {
  // The RA4M1 core exposes no heap-walk API. The gap between the top of the
  // heap and the current stack pointer is the number that actually matters:
  // it is how much room is left before the two collide.
  char stackTop;
  return static_cast<uint32_t>(&stackTop - sbrk(0));
}

const char* boardName() { return "UNO R4 WiFi (RA4M1)"; }

TwoWire& i2c() { return cfg::kUseQwiic ? Wire1 : Wire; }

void i2cBegin(uint32_t frequencyHz) {
  i2c().begin();
  i2c().setClock(frequencyHz);
}

}  // namespace hal

#endif  // TINYBOT_PLATFORM_R4
