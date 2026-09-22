#if defined(TINYBOT_PLATFORM_R4)

#include "hal/hal.h"

#include <EEPROM.h>
#include <string.h>

#include "config.h"

// newlib's heap break. Declared here rather than inside namespace hal, or the
// linker looks for hal::sbrk.
extern "C" char* sbrk(int increment);

namespace hal {
namespace {

// The calibration blob has lived at offset 0 since Phase 1; moving it would
// orphan the value already stored on the robot. The Q-table goes after it,
// with room for the calibration to grow.
int slotBase(Slot slot) {
  switch (slot) {
    case Slot::kCalibration: return 0;
    case Slot::kQTable: return 64;
    default: return 512;  // kRuns, clear of the Q-table's 304 bytes
  }
}

// Leave this much of the period untouched after a write, so a line that
// finishes late still does not push the tick past its jitter budget.
constexpr uint32_t kSerialMarginUs = 2500;

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

// --- Reset cause ------------------------------------------------------------
namespace {
char g_resetCause[64] = "not read";
uint32_t g_resetBits = 0;

void append(const char* what) {
  const size_t used = strlen(g_resetCause);
  if (used) strncat(g_resetCause, ", ", sizeof(g_resetCause) - used - 1);
  strncat(g_resetCause, what, sizeof(g_resetCause) - strlen(g_resetCause) - 1);
}
}  // namespace

void captureResetCause() {
  const uint8_t r0 = R_SYSTEM->RSTSR0;
  const uint16_t r1 = R_SYSTEM->RSTSR1;
  g_resetBits = (static_cast<uint32_t>(r1) << 8) | r0;

  g_resetCause[0] = '\0';
  if (r0 & R_SYSTEM_RSTSR0_PORF_Msk) append("power-on");
  // The voltage monitors fire when the supply dips below their threshold -
  // what a motor stall does to a tired battery pack.
  if (r0 & R_SYSTEM_RSTSR0_LVD0RF_Msk) append("brown-out (LVD0)");
  if (r0 & R_SYSTEM_RSTSR0_LVD1RF_Msk) append("brown-out (LVD1)");
  if (r0 & R_SYSTEM_RSTSR0_LVD2RF_Msk) append("brown-out (LVD2)");
  if (r1 & R_SYSTEM_RSTSR1_IWDTRF_Msk) append("watchdog");
  if (r1 & R_SYSTEM_RSTSR1_WDTRF_Msk) append("watchdog");
  if (r1 & R_SYSTEM_RSTSR1_SWRF_Msk) append("software");
  if (r1 & R_SYSTEM_RSTSR1_RPERF_Msk) append("RAM parity");
  if (r1 & R_SYSTEM_RSTSR1_REERF_Msk) append("RAM ECC");
  if (r1 & (R_SYSTEM_RSTSR1_BUSSRF_Msk | R_SYSTEM_RSTSR1_BUSMRF_Msk)) append("bus error");
  if (r1 & R_SYSTEM_RSTSR1_SPERF_Msk) append("stack pointer error");
  if (g_resetCause[0] == '\0') {
    // The reset pin and the debug probe leave no flag, and neither does a
    // cause the core already cleared before this ran.
    strcpy(g_resetCause, "pin or unflagged");
  }

  // Clearing is what makes the next boot's answer meaningful. These registers
  // sit behind the protect register, so it has to be unlocked first.
  R_SYSTEM->PRCR = 0xA50B;
  R_SYSTEM->RSTSR0 = 0;
  R_SYSTEM->RSTSR1 = 0;
  R_SYSTEM->PRCR = 0xA500;
}

const char* resetCause() { return g_resetCause; }
uint32_t resetBits() { return g_resetBits; }

// --- Persistence ------------------------------------------------------------
bool persistLoad(Slot slot, void* data, size_t len) {
  const int base = slotBase(slot);
  if (base + static_cast<int>(len) > EEPROM.length()) return false;
  uint8_t* out = static_cast<uint8_t*>(data);
  for (size_t i = 0; i < len; ++i) out[i] = EEPROM.read(base + i);
  return true;
}

bool persistSave(Slot slot, const void* data, size_t len) {
  const int base = slotBase(slot);
  if (base + static_cast<int>(len) > EEPROM.length()) return false;
  const uint8_t* in = static_cast<const uint8_t*>(data);
  // EEPROM.update only writes changed bytes, which matters: the RA4M1 data
  // flash has a finite erase count and the Q-table will checkpoint often.
  for (size_t i = 0; i < len; ++i) EEPROM.update(base + i, in[i]);
  return true;
}

bool persistByteAddressable() { return true; }

bool persistWriteByte(Slot slot, size_t offset, uint8_t value) {
  const int at = slotBase(slot) + static_cast<int>(offset);
  if (at >= EEPROM.length()) return false;
  EEPROM.update(at, value);
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

// --- Serial -----------------------------------------------------------------
// UART::write() busy-waits until the last byte is sent, so a write costs its
// full transmission time: 10 bits per byte (start, 8 data, stop). Allow it only
// when that time, plus a margin, fits before the next tick is due.
bool serialWriteFitsBeforeNextTick(size_t len) {
  if (!g_running) return true;
  const uint32_t writeUs =
      static_cast<uint32_t>(len) * 10ul * 1000000ul / cfg::kSerialBaud;
  const int32_t remainingUs = static_cast<int32_t>(g_nextDueUs - micros());
  return remainingUs > static_cast<int32_t>(writeUs + kSerialMarginUs);
}

uint32_t microsUntilNextTick() {
  if (!g_running) return UINT32_MAX;
  const int32_t remaining = static_cast<int32_t>(g_nextDueUs - micros());
  return remaining > 0 ? static_cast<uint32_t>(remaining) : 0;
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
