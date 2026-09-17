#pragma once
#include <stddef.h>
#include <stdint.h>

#include "hal/hal.h"

// Persisting a block without stalling the control loop.
//
// The R4's emulated EEPROM costs about 44 ms per changed byte (measured; see
// hardware/bringup-log.md), and it blocks the one thread the control loop runs
// on. Writing a 300-byte block in one go froze the robot for 2.8 s. So a save
// is trickled: one changed byte per service() call, against a mirror of what
// storage already holds, skipping bytes that did not change. Storage that has
// no byte addressing (NVS on the ESP32, which writes from another core anyway)
// still writes the whole block in one call.
//
// Callers own their data; this only moves bytes.
namespace store {

class Trickle {
 public:
  // `live` is the block to save; `mirror` is scratch of the same size that
  // tracks what storage holds. Both outlive the Trickle.
  Trickle(hal::Slot slot, const void* live, void* mirror, size_t len);

  // Fills the mirror from storage. Call once, after loading, so the first save
  // only writes what differs from what is already there.
  void seedMirror();

  void requestSave();
  bool pending() const;

  // Call from loop(). Writes at most one byte per call.
  void service();

  uint32_t saves() const { return saves_; }
  uint32_t lastMs() const { return lastMs_; }
  uint32_t longestMs() const { return longestMs_; }
  uint16_t lastBytes() const { return lastBytes_; }

 private:
  void finish();

  hal::Slot slot_;
  const uint8_t* live_;
  uint8_t* mirror_;
  size_t len_;

  // No snapshot: the flush reads the live block as it goes, so a save can mix
  // bytes from slightly different moments. For Q-values - estimates under
  // revision - that is harmless, and whatever the cursor passed before it
  // changed is simply written by the next save, because the mirror still
  // holds the old byte.
  volatile bool requested_ = false;
  bool flushing_ = false;
  size_t cursor_ = 0;
  uint32_t startedMs_ = 0;
  uint16_t bytes_ = 0;

  uint32_t saves_ = 0;
  uint32_t lastMs_ = 0;
  uint32_t longestMs_ = 0;
  uint16_t lastBytes_ = 0;
};

}  // namespace store
