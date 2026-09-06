#if defined(TINYBOT_PLATFORM_ESP32)

#include "hal/hal.h"

#include <Preferences.h>
#include <driver/gpio.h>

#include "config.h"
#include "pins.h"

namespace hal {
namespace {

constexpr char kNvsNamespace[] = "tinybot";
constexpr char kNvsKeyBlob[] = "state";

constexpr uint32_t kPwmFreqHz = 20000;  // above audible
constexpr uint8_t kPwmResBits = 10;
constexpr uint16_t kPwmMax = (1 << kPwmResBits) - 1;

portMUX_TYPE g_mux = portMUX_INITIALIZER_UNLOCKED;

void (*g_step)() = nullptr;
uint32_t g_periodMs = 0;

#if ESP_ARDUINO_VERSION_MAJOR < 3
uint8_t g_nextChannel = 0;
uint8_t g_channelOf[SOC_GPIO_PIN_COUNT] = {0};
#endif

void controlTask(void*) {
  TickType_t lastWake = xTaskGetTickCount();
  const TickType_t period = pdMS_TO_TICKS(g_periodMs);
  for (;;) {
    vTaskDelayUntil(&lastWake, period);
    if (g_step != nullptr) g_step();
  }
}

}  // namespace

CriticalSection::CriticalSection() { portENTER_CRITICAL(&g_mux); }
CriticalSection::~CriticalSection() { portEXIT_CRITICAL(&g_mux); }

CriticalSectionIsr::CriticalSectionIsr() { portENTER_CRITICAL_ISR(&g_mux); }
CriticalSectionIsr::~CriticalSectionIsr() { portEXIT_CRITICAL_ISR(&g_mux); }

void pwmBegin(uint8_t pin) {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcAttach(pin, kPwmFreqHz, kPwmResBits);
#else
  const uint8_t channel = g_nextChannel++;
  g_channelOf[pin] = channel;
  ledcSetup(channel, kPwmFreqHz, kPwmResBits);
  ledcAttachPin(pin, channel);
#endif
}

void pwmWrite(uint8_t pin, uint16_t duty, uint16_t dutyMax) {
  const uint32_t scaled =
      dutyMax == 0 ? 0 : (static_cast<uint32_t>(duty) * kPwmMax) / dutyMax;
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcWrite(pin, scaled);
#else
  ledcWrite(g_channelOf[pin], scaled);
#endif
}

// gpio_get_level() is IRAM-resident; digitalRead() is not guaranteed to be.
int fastRead(uint8_t pin) {
  return gpio_get_level(static_cast<gpio_num_t>(pin));
}

bool persistLoad(void* data, size_t len) {
  Preferences prefs;
  if (!prefs.begin(kNvsNamespace, /*readOnly=*/true)) return false;
  const size_t read = prefs.getBytes(kNvsKeyBlob, data, len);
  prefs.end();
  return read == len;
}

bool persistSave(const void* data, size_t len) {
  Preferences prefs;
  if (!prefs.begin(kNvsNamespace, /*readOnly=*/false)) return false;
  const size_t written = prefs.putBytes(kNvsKeyBlob, data, len);
  prefs.end();
  return written == len;
}

void controlLoopBegin(void (*step)(), uint32_t hz) {
  g_step = step;
  g_periodMs = 1000ul / hz;
  xTaskCreatePinnedToCore(controlTask, "control", 4096, nullptr,
                          configMAX_PRIORITIES - 2, nullptr, cfg::kControlCore);
}

// The task runs on its own core; loop() has nothing to do for it.
void controlLoopService() {}

uint32_t freeBytes() { return ESP.getFreeHeap(); }

const char* boardName() { return "ESP32-S3"; }

TwoWire& i2c() { return Wire; }

void i2cBegin(uint32_t frequencyHz) {
  Wire.begin(pins::kI2cSda, pins::kI2cScl, frequencyHz);
}

}  // namespace hal

#endif  // TINYBOT_PLATFORM_ESP32
