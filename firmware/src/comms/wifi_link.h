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

bool connected();  // joined and still joined, or running our own network
const char* ipAddress();

// The link was up and has dropped. The control loop ends any driving session
// on this: without the link there is no way left to send `s`. A rejoin is
// attempted every 15 s, but only while the robot is idle, because it blocks.
bool linkLost();
uint32_t drops();
uint32_t rejoins();

// Firmware update over the air. `url` must be plain http:// and point at an
// .ota image (host/ota/make_ota.py). Runs once the robot is idle and the reply
// has gone out; success ends in a reset into the new firmware.
bool requestOta(const char* url);
const char* otaStatus();
int otaCode();
bool isAccessPoint();  // true when the robot made its own network

// True while a client is pulling telemetry over the network.
bool streamingTelemetry();

// Connection counters, for working out where a request got stuck.
void report(Print& out);

}  // namespace wifi_link
