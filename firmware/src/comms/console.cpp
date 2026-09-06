#include "comms/console.h"

#include <Arduino.h>
#include <stdlib.h>
#include <string.h>

#include "comms/fmt.h"
#include "comms/telemetry.h"
#include "config.h"
#include "control/control_loop.h"
#include "hal/hal.h"
#include "drive/encoders.h"
#include "drive/motors.h"
#include "safety/safety.h"
#include "sense/imu.h"
#include "sense/tof.h"

namespace console {
namespace {

constexpr size_t kLineMax = 64;
char g_line[kLineMax];
size_t g_len = 0;

// Splits "cmd arg1 arg2" in place. Returns the argument count.
int tokenize(char* line, char* argv[], int maxArgs) {
  int argc = 0;
  char* cursor = line;
  while (argc < maxArgs) {
    while (*cursor == ' ' || *cursor == '\t') ++cursor;
    if (*cursor == '\0') break;
    argv[argc++] = cursor;
    while (*cursor != '\0' && *cursor != ' ' && *cursor != '\t') ++cursor;
    if (*cursor == '\0') break;
    *cursor++ = '\0';
  }
  return argc;
}

void printHelp() {
  Serial.println(F(
      "commands:\n"
      "  ?              this help\n"
      "  st             status\n"
      "  m <l> <r>      manual duty, -1000..1000 each\n"
      "  f <duty>       both wheels at <duty>\n"
      "  s              stop (coast)\n"
      "  b              brake\n"
      "  e              encoder counts and metres\n"
      "  z              zero the encoders\n"
      "  t              ToF ranges\n"
      "  i              IMU\n"
      "  j              loop jitter stats\n"
      "  jz             reset jitter stats\n"
      "  d <mm>         drive straight <mm>, closed loop on the encoders\n"
      "  cal <mm>       after `d`, report the measured distance to recalibrate\n"
      "  cpm <value>    set counts-per-metre directly\n"
      "  v              toggle the telemetry stream\n"
      "  c              clear a latched safety stop"));
}

void cmdStatus() {
  const control::LoopStats s = control::stats();
  fmt::printf("mode          : %s\n",
                control::modeName(control::mode()));
  fmt::printf("safety        : %s (reason 0x%02X)\n",
                safety::tripped() ? "TRIPPED" : "ok", safety::reason());
  fmt::printf("duty          : L=%d R=%d\n", motors::leftDuty(),
                motors::rightDuty());
  fmt::printf("encoders      : L=%ld R=%ld  (%.1f counts/m)\n",
                static_cast<long>(encoders::leftCount()),
                static_cast<long>(encoders::rightCount()),
                encoders::countsPerMeter());
  fmt::printf("tof present   : front=%d left=%d right=%d\n",
                tof::present(tof::kFront), tof::present(tof::kLeft),
                tof::present(tof::kRight));
  fmt::printf("imu           : %s (WHO_AM_I 0x%02X)\n",
                imu::present() ? "ok" : "MISSING", imu::whoAmI());
  fmt::printf("loop          : %lu ticks, %lu overruns, telemetry dropped %lu\n",
                static_cast<unsigned long>(s.ticks),
                static_cast<unsigned long>(s.overruns),
                static_cast<unsigned long>(telemetry::dropped()));
  fmt::printf("free memory   : %lu bytes\n",
                static_cast<unsigned long>(hal::freeBytes()));
}

void cmdJitter() {
  const control::LoopStats s = control::stats();
  if (s.ticks == 0) {
    Serial.println(F("no ticks recorded yet"));
    return;
  }
  fmt::printf("period target : %lu us\n",
                static_cast<unsigned long>(cfg::kLoopPeriodMs * 1000));
  fmt::printf("ticks         : %lu\n", static_cast<unsigned long>(s.ticks));
  fmt::printf("min / max     : %lu / %lu us\n",
                static_cast<unsigned long>(s.minUs),
                static_cast<unsigned long>(s.maxUs));
  fmt::printf("mean +/- sd   : %.1f +/- %.1f us\n", s.meanUs, s.stdevUs);
  fmt::printf("overruns      : %lu (outside +/-%lu us)\n",
                static_cast<unsigned long>(s.overruns),
                static_cast<unsigned long>(cfg::kJitterBudgetUs));
  fmt::printf("PHASE 1 GATE  : %s\n",
                s.overruns == 0 ? "PASS" : "FAIL - see docs/04-roadmap.md");
}

void cmdTof() {
  for (uint8_t i = 0; i < tof::kCount; ++i) {
    const tof::Index idx = static_cast<tof::Index>(i);
    if (!tof::present(idx)) {
      fmt::printf("%-6s: absent\n", tof::name(idx));
      continue;
    }
    fmt::printf("%-6s: %u mm  (age %lu ms%s)\n", tof::name(idx),
                  tof::rangeMm(idx),
                  static_cast<unsigned long>(tof::ageMs(idx)),
                  tof::fresh(idx) ? "" : ", STALE");
  }
}

void cmdImu() {
  if (!imu::present()) {
    Serial.println(F("imu absent"));
    return;
  }
  fmt::printf("accel : %+.3f %+.3f %+.3f g\n", imu::accelX(), imu::accelY(),
                imu::accelZ());
  fmt::printf("yaw   : %+.1f dps\n", imu::yawRateDps());
  fmt::printf("shock : %.3f g%s\n", imu::shock(),
                imu::impact() ? "  <-- IMPACT" : "");
}

void cmdDrive(int argc, char* argv[]) {
  if (argc < 2) {
    Serial.println(F("usage: d <mm>"));
    return;
  }
  const float metres = strtof(argv[1], nullptr) / 1000.0f;
  if (!control::requestDriveDistance(metres)) {
    Serial.println(F("refused: distance too small, or safety is latched"));
    return;
  }
  fmt::printf("driving %.3f m ...\n", metres);
}

void cmdCalibrate(int argc, char* argv[]) {
  if (argc < 2) {
    Serial.println(F("usage: cal <measured_mm>   (run `d 1000` first)"));
    return;
  }
  const float commanded = control::driveCommandedMeters();
  const float measured = strtof(argv[1], nullptr) / 1000.0f;
  if (!(commanded > 0.0f)) {
    Serial.println(F("no drive on record - run `d 1000` first"));
    return;
  }
  const float before = encoders::countsPerMeter();
  if (!encoders::calibrateFrom(commanded, measured)) {
    Serial.println(F("calibration refused: implausible numbers"));
    return;
  }
  const float error = 100.0f * (measured - commanded) / commanded;
  fmt::printf("commanded %.3f m, measured %.3f m  (error %+.1f %%)\n",
                commanded, measured, error);
  fmt::printf("counts/m %.1f -> %.1f, saved to NVS\n", before,
                encoders::countsPerMeter());
}

void dispatch(char* line) {
  char* argv[4];
  const int argc = tokenize(line, argv, 4);
  if (argc == 0) return;

  const char* cmd = argv[0];

  if (!strcmp(cmd, "?") || !strcmp(cmd, "help")) {
    printHelp();
  } else if (!strcmp(cmd, "st")) {
    cmdStatus();
  } else if (!strcmp(cmd, "m")) {
    if (argc < 3) {
      Serial.println(F("usage: m <left> <right>"));
      return;
    }
    control::requestManual(static_cast<int16_t>(strtol(argv[1], nullptr, 10)),
                           static_cast<int16_t>(strtol(argv[2], nullptr, 10)));
    Serial.println(F("ok"));
  } else if (!strcmp(cmd, "f")) {
    if (argc < 2) {
      Serial.println(F("usage: f <duty>"));
      return;
    }
    const int16_t duty = static_cast<int16_t>(strtol(argv[1], nullptr, 10));
    control::requestManual(duty, duty);
    Serial.println(F("ok"));
  } else if (!strcmp(cmd, "s")) {
    control::requestIdle();
    Serial.println(F("ok"));
  } else if (!strcmp(cmd, "b")) {
    control::requestManual(0, 0);
    motors::brake();
    Serial.println(F("ok"));
  } else if (!strcmp(cmd, "e")) {
    fmt::printf("L=%ld (%.4f m)  R=%ld (%.4f m)  mean %.4f m\n",
                  static_cast<long>(encoders::leftCount()), encoders::leftMeters(),
                  static_cast<long>(encoders::rightCount()), encoders::rightMeters(),
                  encoders::meters());
  } else if (!strcmp(cmd, "z")) {
    encoders::reset();
    Serial.println(F("ok"));
  } else if (!strcmp(cmd, "t")) {
    cmdTof();
  } else if (!strcmp(cmd, "i")) {
    cmdImu();
  } else if (!strcmp(cmd, "j")) {
    cmdJitter();
  } else if (!strcmp(cmd, "jz")) {
    control::resetStats();
    Serial.println(F("ok"));
  } else if (!strcmp(cmd, "d")) {
    cmdDrive(argc, argv);
  } else if (!strcmp(cmd, "cal")) {
    cmdCalibrate(argc, argv);
  } else if (!strcmp(cmd, "cpm")) {
    if (argc < 2) {
      Serial.println(F("usage: cpm <counts_per_metre>"));
      return;
    }
    Serial.println(encoders::setCountsPerMeter(strtof(argv[1], nullptr))
                       ? F("ok")
                       : F("refused: implausible value"));
  } else if (!strcmp(cmd, "v")) {
    telemetry::setEnabled(!telemetry::enabled());
    if (!telemetry::enabled()) Serial.println(F("telemetry off"));
  } else if (!strcmp(cmd, "c")) {
    safety::clear();
    control::requestIdle();
    Serial.println(F("safety cleared"));
  } else {
    fmt::printf("unknown command '%s' - try ?\n", cmd);
  }
}

}  // namespace

void begin() { g_len = 0; }

void printBanner() {
  Serial.println();
  Serial.println(F("tinybot firmware - Phase 1 (hardware bring-up)"));
  fmt::printf("board %s, build %s %s\n", hal::boardName(), __DATE__, __TIME__);
  Serial.println(F("type ? for commands"));
}

void printStatus() { cmdStatus(); }

void poll() {
  while (Serial.available() > 0) {
    const char ch = static_cast<char>(Serial.read());
    if (ch == '\r') continue;
    if (ch == '\n') {
      g_line[g_len] = '\0';
      if (g_len > 0) dispatch(g_line);
      g_len = 0;
      continue;
    }
    if (g_len + 1 < kLineMax) g_line[g_len++] = ch;
  }
}

}  // namespace console
