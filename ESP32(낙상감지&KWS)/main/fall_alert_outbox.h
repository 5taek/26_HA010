#ifndef FALL_ALERT_OUTBOX_H_
#define FALL_ALERT_OUTBOX_H_

#include <cstdint>
#include <cstring>

struct FallAlertRecord {
  uint32_t version;
  uint64_t sequence;
  uint64_t device_time_ms;
  char event_id[33];
  char state[20];
  char reason[64];
  float score;
};

// Persistent storage is supplied by the caller. Never mutate RAM until the
// durable write succeeds; a failed acknowledgement must remain retryable.
class FallAlertOutbox {
 public:
  static constexpr int kCapacity = 32;
  FallAlertRecord records[kCapacity] = {};
  bool occupied[kCapacity] = {};

  int SelectNext() const {
    int selected = -1;
    for (int i = 0; i < kCapacity; ++i) {
      if (!occupied[i]) continue;
      const bool danger = IsDanger(records[i]);
      if (selected < 0 ||
          (danger && !IsDanger(records[selected])) ||
          (danger == IsDanger(records[selected]) &&
           records[i].sequence < records[selected].sequence)) selected = i;
    }
    return selected;
  }

  int FreeSlot(const FallAlertRecord& record) const {
    (void)record;
    int slot = -1;
    for (int i = 0; i < kCapacity; ++i) {
      if (!occupied[i] && slot < 0) slot = i;
    }
    if (slot >= 0) return slot;
    // Always retain the newest arrival by replacing the oldest sequence,
    // regardless of state. Transmission priority remains danger-first.
    for (int i = 0; i < kCapacity; ++i) {
      if (slot < 0 || records[i].sequence < records[slot].sequence) slot = i;
    }
    return slot;
  }

  template <typename Persist>
  bool Add(const FallAlertRecord& record, Persist persist) {
    const int slot = FreeSlot(record);
    if (slot < 0 || !persist(slot, record)) return false;
    records[slot] = record;
    occupied[slot] = true;
    return true;
  }

  template <typename Erase>
  bool Acknowledge(const char* event_id, Erase erase) {
    for (int i = 0; i < kCapacity; ++i) {
      if (occupied[i] && std::strcmp(records[i].event_id, event_id) == 0) {
        if (!erase(i)) return false;
        occupied[i] = false;
        return true;
      }
    }
    return false;
  }

  static bool IsDanger(const FallAlertRecord& record) {
    return std::strcmp(record.state, "danger_fall") == 0;
  }
};
#endif
