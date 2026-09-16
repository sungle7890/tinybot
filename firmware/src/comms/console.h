#pragma once
#include <Arduino.h>

// Line-based serial console for Phase 1 bring-up. Runs on the Arduino loop()
// core, never in the control task.
namespace console {

void begin();
void poll();          // read and dispatch one line if available
void printBanner();
void printStatus();

// Run one command line, writing its reply to `out` (used by the network link).
void execute(const char* line, Print& out);

}  // namespace console
