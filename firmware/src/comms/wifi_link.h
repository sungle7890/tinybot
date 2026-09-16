#pragma once
#include <Arduino.h>
#include <stdint.h>

// Wi-Fi link for the UNO R4 WiFi: the same console commands and the same
// telemetry, over HTTP, so the robot can drive untethered.
//
// The control loop on this board is driven cooperatively from loop(), so every
// network operation here is gated on having enough time before the next tick.
// Nothing in this file runs while a tick is due.
namespace wifi_link {

bool begin();          // false when the board has no Wi-Fi (ESP32 build)
void service();        // call from loop()

bool connected();
const char* ipAddress();
bool isAccessPoint();  // true when the robot made its own network

// True while a client is pulling telemetry over the network.
bool streamingTelemetry();

// Connection counters, for working out where a request got stuck.
void report(Print& out);

}  // namespace wifi_link
