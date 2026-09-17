#include "store/trickle.h"

#include <Arduino.h>
#include <string.h>

namespace store {

Trickle::Trickle(hal::Slot slot, const void* live, void* mirror, size_t len)
    : slot_(slot),
      live_(static_cast<const uint8_t*>(live)),
      mirror_(static_cast<uint8_t*>(mirror)),
      len_(len) {}

void Trickle::seedMirror() {
  // Read storage as it is, valid or not: the point is only to know which bytes
  // already match, so the first save does not rewrite the whole block.
  if (!hal::persistLoad(slot_, mirror_, len_)) memset(mirror_, 0, len_);
}

void Trickle::requestSave() { requested_ = true; }

bool Trickle::pending() const { return requested_ || flushing_; }

void Trickle::finish() {
  lastMs_ = millis() - startedMs_;
  if (lastMs_ > longestMs_) longestMs_ = lastMs_;
  lastBytes_ = bytes_;
  ++saves_;
  flushing_ = false;
}

void Trickle::service() {
  if (!flushing_) {
    if (!requested_) return;
    requested_ = false;
    flushing_ = true;
    cursor_ = 0;
    bytes_ = 0;
    startedMs_ = millis();

    if (!hal::persistByteAddressable()) {
      hal::persistSave(slot_, live_, len_);
      memcpy(mirror_, live_, len_);
      bytes_ = static_cast<uint16_t>(len_);
      finish();
      return;
    }
  }

  while (cursor_ < len_ && live_[cursor_] == mirror_[cursor_]) ++cursor_;
  if (cursor_ < len_) {
    hal::persistWriteByte(slot_, cursor_, live_[cursor_]);
    mirror_[cursor_] = live_[cursor_];
    ++cursor_;
    ++bytes_;
    return;
  }
  finish();
}

}  // namespace store
