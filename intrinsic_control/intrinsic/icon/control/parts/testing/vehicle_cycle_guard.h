// Copyright 2026 Intrinsic Innovation LLC
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     https://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

// Test-only guard for one vehicle ICON cycle.
//
// Allocation checks call the existing malloc counter
// (MallocCounterInit / MallocCounterCount) when the target is built with
// INTRINSIC_MALLOC_TEST. Blocking checks use the mutex, condition, and queue
// doubles in this header. Those doubles record a wait on this guard. They do
// not sleep and they are not linked into production targets. Deadline checks
// subtract ICON Clock::Now() samples, so a ManualClock drives the budget.

#ifndef INTRINSIC_ICON_CONTROL_PARTS_TESTING_VEHICLE_CYCLE_GUARD_H_
#define INTRINSIC_ICON_CONTROL_PARTS_TESTING_VEHICLE_CYCLE_GUARD_H_

#include <stddef.h>
#include <stdint.h>

#include <array>
#include <utility>
#include <vector>

#include "absl/status/status.h"
#include "absl/strings/string_view.h"
#include "intrinsic/icon/testing/malloc_test.h"
#include "intrinsic/icon/utils/clock.h"
#include "intrinsic/icon/utils/realtime_status.h"

namespace intrinsic::icon {

class VehicleCycleBoundedQueue {
 public:
  static constexpr int kCapacity = 8;

  int size() const { return size_; }
  int capacity() const { return kCapacity; }
  bool empty() const { return size_ == 0; }
  bool full() const { return size_ == kCapacity; }

  // Fails when the queue is already at kCapacity. Does not allocate and does
  // not record a blocking wait.
  bool TryPush(int value);

  bool TryPop(int* value);

  // Pushes until the queue is full. Returns the number of items inserted.
  int Fill();

  // Pops up to `limit` items. Returns how many were removed.
  int Drain(int limit);

 private:
  std::array<int, kCapacity> data_{};
  int size_ = 0;
};

// Minimum configured cycle load moves no queued item. Maximum load moves one
// full bounded queue. Both stay inside VehicleCycleBoundedQueue::kCapacity.
inline constexpr int kVehicleCycleMinLoad = 0;
inline constexpr int kVehicleCycleMaxLoad = VehicleCycleBoundedQueue::kCapacity;

struct VehicleCycleGuardConfig {
  size_t max_allocations = 0;
  int max_blocking_waits = 0;
  // ManualClock advance allowed inside one Measure. Equal to the budget
  // passes. One nanosecond past it fails.
  Duration max_cycle = Milliseconds(1);
  int min_cycle_load = kVehicleCycleMinLoad;
  int max_cycle_load = kVehicleCycleMaxLoad;
};

// Reports the first budget breach. The status message always contains
// `cycle=<index> path=<label> budget=<value> observed=<value> kind=<kind>`.
class VehicleCycleGuard {
 public:
  explicit VehicleCycleGuard(VehicleCycleGuardConfig config = {});

  const VehicleCycleGuardConfig& config() const { return config_; }

  // True only in the malloc-counting test variant. The non-counting variant
  // compiles the same source with the existing malloc macros as no-ops.
  static constexpr bool AllocationChecksEnabled() {
#if INTRINSIC_MALLOC_TEST
    return true;
#else
    return false;
#endif
  }

  // Called by the test doubles below. Must run on the thread inside Measure.
  static void RecordActiveBlockingWait();

  // Runs `cycle` once under the allocation, blocking, and ManualClock budgets.
  // `cycle_index` and `call_path` are copied into any failure message.
  // `call_path` must outlive this call.
  template <typename CycleFn>
  absl::Status Measure(int cycle_index, absl::string_view call_path,
                       CycleFn&& cycle);

  // `prepare` runs outside the budgets (streaming input setup is non-realtime).
  // `cycle` runs inside Measure. Stops on the first failure.
  template <typename PrepareFn, typename CycleFn>
  absl::Status RunRepeated(int cycle_count, absl::string_view call_path,
                           PrepareFn&& prepare, CycleFn&& cycle);

 private:
  class ScopedActivation {
   public:
    explicit ScopedActivation(VehicleCycleGuard* guard) : previous_(Current()) {
      Current() = guard;
    }
    ~ScopedActivation() { Current() = previous_; }

    ScopedActivation(const ScopedActivation&) = delete;
    ScopedActivation& operator=(const ScopedActivation&) = delete;

   private:
    VehicleCycleGuard* previous_;
  };

  static VehicleCycleGuard*& Current();

  absl::Status FinishMeasure(int cycle_index, absl::string_view call_path,
                             RealtimeStatus cycle_status, Time start,
                             size_t allocations, bool check_allocations) const;

  absl::Status PrepareFailure(int cycle_index, absl::string_view call_path,
                              const absl::Status& status) const;

  absl::Status Violation(absl::string_view kind, int cycle_index,
                         absl::string_view call_path, int64_t budget,
                         int64_t observed) const;

  VehicleCycleGuardConfig config_;
  int blocking_waits_ = 0;
};

// Records one blocking wait. Does not take a pthread mutex and does not sleep.
class VehicleCycleMutex {
 public:
  void Wait() { VehicleCycleGuard::RecordActiveBlockingWait(); }
};

// Records one blocking wait. Does not wait on a pthread condition and does
// not sleep.
class VehicleCycleConditionVariable {
 public:
  void Wait() { VehicleCycleGuard::RecordActiveBlockingWait(); }
};

// Push grows heap storage. A capacity increase records one blocking wait so
// the guard reports it even when malloc counting is compiled out. The malloc
// counter still sees the same growth when INTRINSIC_MALLOC_TEST is set.
class VehicleCycleUnboundedQueue {
 public:
  int size() const { return static_cast<int>(storage_.size()); }

  void Push(int value);

 private:
  std::vector<int> storage_;
};

template <typename CycleFn>
absl::Status VehicleCycleGuard::Measure(int cycle_index,
                                        absl::string_view call_path,
                                        CycleFn&& cycle) {
  blocking_waits_ = 0;
  const Time start = Clock::Now();
  size_t allocations = 0;
  bool check_allocations = false;
  RealtimeStatus cycle_status;
  {
    ScopedActivation activation(this);
#if INTRINSIC_MALLOC_TEST
    MallocCounterInit();
    check_allocations = true;
#endif
    cycle_status = std::forward<CycleFn>(cycle)();
#if INTRINSIC_MALLOC_TEST
    allocations = MallocCounterCount();
#endif
  }
  return FinishMeasure(cycle_index, call_path, cycle_status, start, allocations,
                       check_allocations);
}

template <typename PrepareFn, typename CycleFn>
absl::Status VehicleCycleGuard::RunRepeated(int cycle_count,
                                            absl::string_view call_path,
                                            PrepareFn&& prepare,
                                            CycleFn&& cycle) {
  if (cycle_count <= 0) {
    return Violation("cycle_count", /*cycle_index=*/0, call_path,
                     /*budget=*/1, /*observed=*/cycle_count);
  }
  for (int cycle_index = 0; cycle_index < cycle_count; ++cycle_index) {
    const absl::Status prepare_status = prepare(cycle_index);
    if (!prepare_status.ok()) {
      return PrepareFailure(cycle_index, call_path, prepare_status);
    }
    const absl::Status status =
        Measure(cycle_index, call_path, [&] { return cycle(cycle_index); });
    if (!status.ok()) {
      return status;
    }
  }
  return absl::OkStatus();
}

}  // namespace intrinsic::icon

#endif  // INTRINSIC_ICON_CONTROL_PARTS_TESTING_VEHICLE_CYCLE_GUARD_H_
