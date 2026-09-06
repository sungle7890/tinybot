#include "comms/fmt.h"

#include <Arduino.h>
#include <stdarg.h>
#include <stdio.h>

namespace fmt {
namespace {
constexpr size_t kBufferSize = 192;
}

void printf(const char* format, ...) {
  char buffer[kBufferSize];
  va_list args;
  va_start(args, format);
  const int len = vsnprintf(buffer, sizeof(buffer), format, args);
  va_end(args);
  if (len <= 0) return;
  Serial.write(buffer);
}

}  // namespace fmt
