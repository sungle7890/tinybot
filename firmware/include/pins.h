#pragma once

// Pin maps are per board. Keep each in sync with hardware/wiring.md.
#if defined(TINYBOT_PLATFORM_R4)
#include "pins_r4.h"
#elif defined(TINYBOT_PLATFORM_ESP32)
#include "pins_esp32.h"
#else
#error "Define TINYBOT_PLATFORM_R4 or TINYBOT_PLATFORM_ESP32 (see platformio.ini)"
#endif
