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

// Reference-to-body-wrench step (PDR §4.1, §5, §11, §15).
//
// Evaluate accepts a MotionReferenceRt, a vehicle-state snapshot, and an
// update period. It returns a BodyWrenchRt or a typed status. This is not
// a control law and not an allocator. A failure writes a finite-zero
// wrench. That wrench is a neutral command only when the status is kOk.
// Any other status means the same zero bits are not a command. ICON
// remains the only writer of actuator commands. This package does not
// link ICON, does not call ApplyCommand, and does not run on the ICON
// cycle.
//
// BodyWrenchRt matches dynamics::BodyWrenchRt: frame, force in newtons,
// and torque in newton-meters, at the body origin, REP-103 axes. This
// package does not depend on dynamics or allocation, so the type lives
// here. FrameId is guidance::FrameId. Its enumerators match
// dynamics::FrameId.
//
// The control boundary reads snapshot id from reference.snapshot_id and
// state.snapshot_id. A mismatch is kStale. The update period is the
// Duration argument. reference.update_period is the period the reference
// was built with and must be finite and non-negative. MotionReferenceRt,
// VehicleStateRt, and Duration are the guidance types.
//
// Soft-real-time scheduling is 10 to 50 Hz (PDR §11). The interface does
// not reject another non-negative period. Zero is an instantaneous
// sample. A negative or non-finite period is invalid. Expiry of a
// DesiredMotion horizon is the guidance step's check. This step fails
// closed when the reference is empty, invalid, non-finite, or stale.
//
// Ownership. The caller owns the reference, the state, and the result.
// Evaluate does not retain its arguments.
//
// Thread safety. Evaluate is const. Implementations with no
// unsynchronized mutable members may be called concurrently on one
// instance. ValidateControlInputs reads only its arguments. This package
// takes no locks. Status text is a static string view.
// ValidateControlInputs and ZeroWrenchController do not allocate.

#ifndef INTRINSIC_VEHICLE_CONTROL_REFERENCE_CONTROL_H_
#define INTRINSIC_VEHICLE_CONTROL_REFERENCE_CONTROL_H_

#include <array>
#include <string_view>

#include "intrinsic/vehicle/guidance/guidance_step.h"

namespace intrinsic::vehicle::control {

using Duration = guidance::Duration;
using FrameId = guidance::FrameId;
using MotionReferenceRt = guidance::MotionReferenceRt;
using VehicleStateRt = guidance::VehicleStateRt;

// Generalized force at the body origin. Force is newtons. Torque is
// newton-meters. Field order matches dynamics::BodyWrenchRt. frame is
// kBody on the zero value this step returns. The status, not a flag on
// this struct, says whether that zero is the neutral wrench.
struct BodyWrenchRt {
  FrameId frame = FrameId::kBody;
  std::array<double, 3> force_n = {};
  std::array<double, 3> torque_n_m = {};
};

// Append codes. kOk is the only success code. Names match the guidance
// status family.
enum class ControlStatusCode {
  kOk = 0,
  kInvalidArgument = 1,
  kStale = 2,
  kInvalid = 3,
  kMissingObjective = 4,
};

struct ControlStatus {
  ControlStatusCode code = ControlStatusCode::kOk;
  std::string_view message;

  [[nodiscard]] bool ok() const { return code == ControlStatusCode::kOk; }

  [[nodiscard]] static ControlStatus Ok() { return {}; }

  [[nodiscard]] static ControlStatus InvalidArgument(std::string_view message) {
    return {ControlStatusCode::kInvalidArgument, message};
  }

  [[nodiscard]] static ControlStatus Stale(std::string_view message) {
    return {ControlStatusCode::kStale, message};
  }

  [[nodiscard]] static ControlStatus Invalid(std::string_view message) {
    return {ControlStatusCode::kInvalid, message};
  }

  [[nodiscard]] static ControlStatus MissingObjective(
      std::string_view message) {
    return {ControlStatusCode::kMissingObjective, message};
  }
};

// Fixed-size status return. value() on failure is the T stored by Failure,
// which is zero-initialized when the caller omits it.
template <typename T>
class StatusOr {
 public:
  [[nodiscard]] static StatusOr Ok(T value) {
    StatusOr out;
    out.ok_ = true;
    out.status_ = ControlStatus::Ok();
    out.value_ = value;
    return out;
  }

  [[nodiscard]] static StatusOr Failure(ControlStatus status, T value = T{}) {
    StatusOr out;
    out.ok_ = false;
    out.status_ = status;
    out.value_ = value;
    return out;
  }

  [[nodiscard]] bool ok() const { return ok_; }
  [[nodiscard]] const ControlStatus& status() const { return status_; }
  [[nodiscard]] const T& value() const { return value_; }

 private:
  StatusOr() = default;
  bool ok_ = false;
  ControlStatus status_;
  T value_{};
};

// Shared input contract. Returns the first defect and does not allocate.
//
// Order: update period; state pose frame, finiteness, and unit
// quaternion; reference update period; present pose; present twist;
// missing pose and twist; snapshot identity.
//
// A reference field that is not present is not read. Both a pose and a
// twist may be present. Neither present is kMissingObjective, including
// when the stored arrays hold a finite pose. That stored pose is not a
// reference. A bad frame and a non-unit quaternion are kInvalid.
// Non-finite values and a negative update period are kInvalidArgument.
// A zero update period is valid. snapshot_id values that differ are
// kStale. This function does not apply a gain and does not map the
// reference onto a wrench.
[[nodiscard]] ControlStatus ValidateControlInputs(
    const MotionReferenceRt& reference, const VehicleStateRt& state,
    Duration update_period);

// Narrow reference-control boundary. Implementations must reject the
// defects ValidateControlInputs names, must return a finite wrench, and
// must keep Evaluate const. An implementation with no mutable members
// may be called concurrently on one instance. An implementation that
// owns integrator state stores it in a mutable member and is not safe
// for concurrent Evaluate calls on that instance.
// On failure the wrench is finite zeros and is not a command.
class ReferenceController {
 public:
  virtual ~ReferenceController() = default;

  ReferenceController(const ReferenceController&) = delete;
  ReferenceController& operator=(const ReferenceController&) = delete;
  ReferenceController(ReferenceController&&) = delete;
  ReferenceController& operator=(ReferenceController&&) = delete;

  [[nodiscard]] virtual StatusOr<BodyWrenchRt> Evaluate(
      const MotionReferenceRt& reference, const VehicleStateRt& state,
      Duration update_period) const = 0;

 protected:
  ReferenceController() = default;
};

}  // namespace intrinsic::vehicle::control

#endif  // INTRINSIC_VEHICLE_CONTROL_REFERENCE_CONTROL_H_
