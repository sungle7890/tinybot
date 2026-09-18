// Copy this file to include/secrets.h and fill in your own network.
// include/secrets.h is git-ignored, so the password never leaves this machine.
//
//   cp include/secrets.example.h include/secrets.h
//
// Leave TINYBOT_WIFI_SSID empty to skip joining a network. The robot then opens
// its own Wi-Fi called "tinybot" - but only if TINYBOT_AP_PASS is set below.
#pragma once

#define TINYBOT_WIFI_SSID ""
#define TINYBOT_WIFI_PASS ""

// The robot's own network, used when the two above are empty or joining fails.
// Choose your own password, 8 characters or more. Leave it empty and the robot
// opens no network of its own (USB still works). There is deliberately no
// default: the robot's command API has no login, so anyone who joins this
// network can drive the robot.
#define TINYBOT_AP_SSID "tinybot"
#define TINYBOT_AP_PASS ""
