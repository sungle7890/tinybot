// Copy this file to include/secrets.h and fill in your own network.
// include/secrets.h is git-ignored, so the password never leaves this machine.
//
//   cp include/secrets.example.h include/secrets.h
//
// Leave TINYBOT_WIFI_SSID empty to skip joining a network: the robot then
// creates its own Wi-Fi called "tinybot" (password below) that you connect to.
#pragma once

#define TINYBOT_WIFI_SSID ""
#define TINYBOT_WIFI_PASS ""

// Used only for the robot's own network, when the two above are empty.
#define TINYBOT_AP_SSID "tinybot"
#define TINYBOT_AP_PASS "tinybot1234"
