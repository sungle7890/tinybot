#pragma once

// Serial.printf() is an ESP32 extension; the Renesas UART class has no such
// method. This is the portable stand-in used everywhere in the firmware.
namespace fmt {

// Formats into a stack buffer and writes it to Serial. Output longer than the
// buffer is truncated rather than split, which is fine for console text.
void printf(const char* format, ...) __attribute__((format(printf, 1, 2)));

}  // namespace fmt
