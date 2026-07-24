#pragma once

#include <array>
#include <atomic>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <mutex>
#include <optional>
#include <utility>

namespace odrive_wheels_driver {
namespace detail {

// One overwriteable pending value per slot. A slow consumer never creates a
// backlog: producers replace stale pending data and only contend on this short
// critical section, never on the work performed by the consumer.
template<typename EventT, std::size_t SlotCount>
class LatestEventMailbox {
  static_assert(SlotCount > 0U, "LatestEventMailbox needs at least one slot");

public:
  bool store(std::size_t slot_index, EventT event) {
    if (slot_index >= SlotCount) {
      return false;
    }
    {
      std::lock_guard<std::mutex> lock(mutex_);
      auto& slot = slots_[slot_index];
      slot.event = std::move(event);
      slot.sequence = next_sequence_++;
    }
    condition_.notify_one();
    return true;
  }

  bool wait_take(EventT& event, const std::atomic<bool>& stop_requested) {
    std::unique_lock<std::mutex> lock(mutex_);
    condition_.wait(lock, [this, &stop_requested]() {
      return stop_requested.load() || has_pending_unlocked();
    });
    if (stop_requested.load()) {
      return false;
    }

    std::size_t selected = 0U;
    bool selected_valid = false;
    for (std::size_t index = 0U; index < SlotCount; ++index) {
      if (!slots_[index].event.has_value()) {
        continue;
      }
      if (!selected_valid ||
          slots_[index].sequence < slots_[selected].sequence) {
        selected = index;
        selected_valid = true;
      }
    }

    event = std::move(*slots_[selected].event);
    slots_[selected].event.reset();
    return true;
  }

  bool empty() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return !has_pending_unlocked();
  }

  void clear() {
    std::lock_guard<std::mutex> lock(mutex_);
    for (auto& slot : slots_) {
      slot.event.reset();
      slot.sequence = 0U;
    }
    next_sequence_ = 0U;
  }

  void notify_all() {
    condition_.notify_all();
  }

private:
  struct Slot {
    std::optional<EventT> event;
    std::uint64_t sequence = 0U;
  };

  bool has_pending_unlocked() const {
    for (const auto& slot : slots_) {
      if (slot.event.has_value()) {
        return true;
      }
    }
    return false;
  }

  mutable std::mutex mutex_;
  std::condition_variable condition_;
  std::array<Slot, SlotCount> slots_{};
  std::uint64_t next_sequence_ = 0U;
};

}  // namespace detail
}  // namespace odrive_wheels_driver
