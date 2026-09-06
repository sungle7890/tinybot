#pragma once
#include <stddef.h>

#include "hal/hal.h"

// Single-producer / single-consumer ring buffer, replacing the FreeRTOS queue
// so the same telemetry path works on both boards.
//
// push() never blocks and never waits: a full buffer drops the sample and says
// so. That is the intended behaviour - losing telemetry is acceptable, delaying
// a control tick is not (docs/02-architecture.md design rule 1).
//
// Namespaced because the Arduino Renesas core puts its own arduino::RingBuffer
// in global scope.
namespace hal {

template <typename T, size_t N>
class RingBuffer {
 public:
  bool push(const T& item) {
    hal::CriticalSection lock;
    const size_t next = advance(head_);
    if (next == tail_) return false;  // full
    buffer_[head_] = item;
    head_ = next;
    return true;
  }

  bool pop(T& out) {
    hal::CriticalSection lock;
    if (head_ == tail_) return false;  // empty
    out = buffer_[tail_];
    tail_ = advance(tail_);
    return true;
  }

  bool empty() const { return head_ == tail_; }

 private:
  static size_t advance(size_t index) { return (index + 1) % N; }

  T buffer_[N];
  volatile size_t head_ = 0;
  volatile size_t tail_ = 0;
};

}  // namespace hal
