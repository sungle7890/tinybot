#include "comms/fmt.h"

#include <stdarg.h>
#include <stdio.h>

namespace fmt {
namespace {
constexpr size_t kBufferSize = 192;
Print* g_sink = nullptr;
}  // namespace

void setSink(Print* sink) { g_sink = sink; }
Print& sink() { return g_sink ? *g_sink : Serial; }

void printf(const char* format, ...) {
  char buffer[kBufferSize];
  va_list args;
  va_start(args, format);
  const int len = vsnprintf(buffer, sizeof(buffer), format, args);
  va_end(args);
  if (len <= 0) return;
  sink().write(buffer);
}

void println(const char* text) {
  sink().write(text);
  sink().write("\n");
}

}  // namespace fmt
