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

#include "intrinsic/icon/control/parts/testing/vehicle_cycle_guard.h"

#include <stdint.h>

#include "absl/log/check.h"
#include "absl/status/status.h"
#include "absl/strings/str_cat.h"
#include "absl/strings/string_view.h"
#include "intrinsic/icon/utils/clock.h"
#include "intrinsic/icon/utils/realtime_status.h"

namespace intrinsic::icon {

bool VehicleCycleBoundedQueue::TryPush(int value) {
  if (size_ >= kCapacity) {
    return false;
  }
  data_[size_] = value;
  ++size_;
  return true;
}

bool VehicleCycleBoundedQueue::TryPop(int* value) {
  if (size_ == 0 || value == nullptr) {
    return false;
  }
  *value = data_[0];
  for (int i = 1; i < size_; ++i) {
    data_[i - 1] = data_[i];
  }
  --size_;
  return true;
}

int VehicleCycleBoundedQueue::Fill() {
  int added = 0;
  while (TryPush(added)) {
    ++added;
  }
  return added;
}

int VehicleCycleBoundedQueue::Drain(int limit) {
  int drained = 0;
  int value = 0;
  while (drained < limit && TryPop(&value)) {
    ++drained;
  }
  return drained;
}

void VehicleCycleUnboundedQueue::Push(int value) {
  const size_t capacity_before = storage_.capacity();
  storage_.push_back(value);
  if (storage_.capacity() > capacity_before) {
    VehicleCycleGuard::RecordActiveBlockingWait();
  }
}

VehicleCycleGuard::VehicleCycleGuard(VehicleCycleGuardConfig config)
    : config_(config) {}

VehicleCycleGuard*& VehicleCycleGuard::Current() {
  thread_local VehicleCycleGuard* current = nullptr;
  return current;
}

void VehicleCycleGuard::RecordActiveBlockingWait() {
  VehicleCycleGuard* guard = Current();
  CHECK(guard != nullptr) << "blocking wait outside a guarded vehicle cycle";
  ++guard->blocking_waits_;
}

absl::Status VehicleCycleGuard::Violation(absl::string_view kind,
                                          int cycle_index,
                                          absl::string_view call_path,
                                          int64_t budget,
                                          int64_t observed) const {
  return absl::FailedPreconditionError(
      absl::StrCat("cycle=", cycle_index, " path=", call_path,
                   " budget=", budget, " observed=", observed, " kind=", kind));
}

absl::Status VehicleCycleGuard::PrepareFailure(
    int cycle_index, absl::string_view call_path,
    const absl::Status& status) const {
  return absl::FailedPreconditionError(absl::StrCat(
      "cycle=", cycle_index, " path=", call_path,
      " budget=0 observed=0 kind=prepare status=", status.message()));
}

absl::Status VehicleCycleGuard::FinishMeasure(int cycle_index,
                                              absl::string_view call_path,
                                              RealtimeStatus cycle_status,
                                              Time start, size_t allocations,
                                              bool check_allocations) const {
  if (blocking_waits_ > config_.max_blocking_waits) {
    return Violation("blocking", cycle_index, call_path,
                     config_.max_blocking_waits, blocking_waits_);
  }
  if (check_allocations && allocations > config_.max_allocations) {
    return Violation("allocation", cycle_index, call_path,
                     static_cast<int64_t>(config_.max_allocations),
                     static_cast<int64_t>(allocations));
  }
  const Duration elapsed = Clock::Now() - start;
  if (elapsed > config_.max_cycle) {
    return Violation("deadline", cycle_index, call_path,
                     ToInt64Nanoseconds(config_.max_cycle),
                     ToInt64Nanoseconds(elapsed));
  }
  if (!cycle_status.ok()) {
    return absl::FailedPreconditionError(
        absl::StrCat("cycle=", cycle_index, " path=", call_path,
                     " budget=0 observed=0 kind=cycle_status status=",
                     cycle_status.message()));
  }
  return absl::OkStatus();
}

}  // namespace intrinsic::icon
