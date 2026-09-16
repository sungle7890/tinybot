#pragma once

#include <Arduino.h>

// Serial.printf() is an ESP32 extension; the Renesas UART class has no such
// method. This is the portable stand-in used everywhere in the firmware.
//
// Output goes to Serial by default. setSink() redirects it, which is how a
// console command run over the network sends its reply back to the client
// instead of the USB cable.
namespace fmt {

void printf(const char* format, ...) __attribute__((format(printf, 1, 2)));
void println(const char* text);

void setSink(Print* sink);   // nullptr restores Serial
Print& sink();

}  // namespace fmt
