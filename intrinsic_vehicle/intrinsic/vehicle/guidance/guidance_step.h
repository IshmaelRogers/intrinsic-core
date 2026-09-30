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

// DesiredMotion-to-reference step (PDR §4.1, §5, §11, §15).
//
// Evaluate accepts a fixed-size DesiredMotionRt, an optional vehicle-state
// snapshot, and an update period. It returns a MotionReferenceRt or a
// typed status. This is not a guidance law. A failure writes an empty
// reference: has_pose and has_twist are false, and every numeric field is
// a finite zero. An empty reference is not a pose and not a command.
//
// DesiredMotionRt mirrors the approved DesiredMotion fields. It does not
// parse protobuf. snapshot_id is the monotonic stamped identity. Callers
// copy StampedHeader.sequence or the world snapshot id into that field.
// The objective is one of pose, body twist, or an opaque trajectory id.
// The id is a fixed buffer of at most 64 bytes. It is not trajectory
// samples. horizon_s and confidence are optional. A supplied confidence
// is dimensionless and must lie in [0, 1].
//
// VehicleStateRt matches dynamics::VehicleStateRt for pose frame,
// position, orientation, and body twist, then appends snapshot_id.
// This package does not depend on dynamics. FrameId enumerators match
// dynamics::FrameId and are not renumbered. Body axes are REP-103.
// Twist order is surge, sway, heave, roll, pitch, yaw. Orientation is a
// Hamilton quaternion stored x, y, z, w. This package does not convert
// ENU and NED.
//
// MotionReferenceRt is the reference a later controller consumes. On
// success it echoes snapshot_id from the intent and the update period
// that produced it. EchoGuidance copies a pose or twist objective and
// does not invent the other one.
//
// Soft-real-time scheduling is 10 to 50 Hz (PDR §11). The interface does
// not reject another non-negative period. Zero is an instantaneous
// sample, the same rule as dynamics. A negative or non-finite period is
// invalid. A supplied horizon shorter than the update period is expired.
// When a state snapshot is supplied, a different snapshot_id is stale.
// Both fail closed. ICON remains the only actuator writer. This package
// does not link ICON and does not run on the ICON cycle.
//
// Ownership. The caller owns the intent, the state, and the result.
// Evaluate does not retain its arguments. A null state pointer means no
// vehicle-state snapshot was supplied. TrajectoryId::view() is valid
// only while that TrajectoryId is alive.
//
// Thread safety. Evaluate is const. Implementations with no
// unsynchronized mutable members may be called concurrently on one
// instance. ValidateGuidanceInputs reads only its arguments. This
// package takes no locks. Status text is a static string view.
// ValidateGuidanceInputs and the fakes do not allocate heap memory.

#ifndef INTRINSIC_VEHICLE_GUIDANCE_GUIDANCE_STEP_H_
#define INTRINSIC_VEHICLE_GUIDANCE_GUIDANCE_STEP_H_

#include <array>
#include <cstddef>
#include <cstdint>
#include <string_view>

namespace intrinsic::vehicle::guidance {

inline constexpr int kSpatialDof = 6;

// Body-twist order. Matches dynamics.
inline constexpr int kSurge = 0;
inline constexpr int kSway = 1;
inline constexpr int kHeave = 2;
inline constexpr int kRoll = 3;
inline constexpr int kPitch = 4;
inline constexpr int kYaw = 5;

inline constexpr int kX = 0;
inline constexpr int kY = 1;
inline constexpr int kZ = 2;

// Hamilton quaternion stored x, y, z, w.
inline constexpr int kQuatX = 0;
inline constexpr int kQuatY = 1;
inline constexpr int kQuatZ = 2;
inline constexpr int kQuatW = 3;

// Absolute tolerance for |norm(quaternion) - 1|. A difference equal to
// this passes. Helpers do not renormalize. Matches dynamics.
inline constexpr double kUnitQuaternionTolerance = 1e-9;

// Opaque trajectory id capacity. Not a sample count.
inline constexpr std::size_t kTrajectoryIdCapacity = 64;

// Soft-real-time band from PDR §11. Evaluate does not enforce it.
inline constexpr double kSoftRealtimeMinHz = 10;
inline constexpr double kSoftRealtimeMaxHz = 50;

// Explicit frame ids. Values match dynamics::FrameId. Append only.
enum class FrameId {
  kUnspecified = 0,
  kWorldEnu = 1,
  kWorldNed = 2,
  kBody = 3,
};

// SI seconds. Zero is an instantaneous sample. Negative is invalid.
struct Duration {
  double seconds = 0;
};

// Fixed-size state snapshot the step reads. The fields above snapshot_id
// match dynamics::VehicleStateRt. snapshot_id is the monotonic snapshot
// this sample belongs to. Dynamics does not carry that id.
struct VehicleStateRt {
  FrameId pose_frame = FrameId::kUnspecified;
  std::array<double, 3> position_m = {};
  std::array<double, 4> orientation_xyzw = {0, 0, 0, 1};
  std::array<double, kSpatialDof> body_twist = {};
  std::uint64_t snapshot_id = 0;
};

// One objective. kUnset means the oneof was not supplied. That is not a
// pose. Values match the host ObjectiveKind tags.
enum class ObjectiveKind {
  kUnset = 0,
  kPose = 1,
  kTwist = 2,
  kTrajectory = 3,
};

// Fixed buffer. length is the byte count. Bytes past length are not part
// of the id. view() points at this object and is not retained by Evaluate.
struct TrajectoryId {
  std::array<char, kTrajectoryIdCapacity> bytes = {};
  std::uint8_t length = 0;

  [[nodiscard]] std::string_view view() const {
    const std::size_t count =
        length <= kTrajectoryIdCapacity ? length : kTrajectoryIdCapacity;
    return std::string_view(bytes.data(), count);
  }
};

// Copies id into destination and zeros the unused tail. An id longer
// than kTrajectoryIdCapacity leaves destination unchanged and returns
// false. This function does not allocate. An empty id is stored.
[[nodiscard]] inline bool AssignTrajectoryId(TrajectoryId& destination,
                                             std::string_view id) {
  if (id.size() > kTrajectoryIdCapacity) {
    return false;
  }
  destination.bytes.fill('\0');
  for (std::size_t i = 0; i < id.size(); ++i) {
    destination.bytes[i] = id[i];
  }
  destination.length = static_cast<std::uint8_t>(id.size());
  return true;
}

// Fixed-size intent. Pose fields are read only for kPose. Twist fields
// are read only for kTwist. trajectory_id is read only for kTrajectory.
// has_horizon and has_confidence are false when that value was not
// supplied. A supplied zero is present and is not absence.
struct DesiredMotionRt {
  std::uint64_t snapshot_id = 0;
  ObjectiveKind objective = ObjectiveKind::kUnset;
  FrameId pose_frame = FrameId::kUnspecified;
  std::array<double, 3> position_m = {};
  std::array<double, 4> orientation_xyzw = {0, 0, 0, 1};
  FrameId twist_frame = FrameId::kUnspecified;
  std::array<double, kSpatialDof> body_twist = {};
  TrajectoryId trajectory_id;
  bool has_horizon = false;
  double horizon_s = 0;
  bool has_confidence = false;
  double confidence = 0;
};

// Reference the controller consumes. has_pose and has_twist select which
// stored numbers are a reference. Both false is an empty reference: the
// arrays are finite zeros and are not a default pose. update_period is
// the step that produced this reference. On failure it is zero, including
// when the supplied period was rejected.
struct MotionReferenceRt {
  std::uint64_t snapshot_id = 0;
  Duration update_period;
  bool has_pose = false;
  FrameId pose_frame = FrameId::kUnspecified;
  std::array<double, 3> position_m = {};
  std::array<double, 4> orientation_xyzw = {};
  bool has_twist = false;
  FrameId twist_frame = FrameId::kUnspecified;
  std::array<double, kSpatialDof> body_twist = {};
};

// Append codes. kOk is the only success code.
enum class GuidanceStatusCode {
  kOk = 0,
  kInvalidArgument = 1,
  kStale = 2,
  kInvalid = 3,
  kMissingObjective = 4,
};

struct GuidanceStatus {
  GuidanceStatusCode code = GuidanceStatusCode::kOk;
  std::string_view message;

  [[nodiscard]] bool ok() const { return code == GuidanceStatusCode::kOk; }

  [[nodiscard]] static GuidanceStatus Ok() { return {}; }

  [[nodiscard]] static GuidanceStatus InvalidArgument(
      std::string_view message) {
    return {GuidanceStatusCode::kInvalidArgument, message};
  }

  [[nodiscard]] static GuidanceStatus Stale(std::string_view message) {
    return {GuidanceStatusCode::kStale, message};
  }

  [[nodiscard]] static GuidanceStatus Invalid(std::string_view message) {
    return {GuidanceStatusCode::kInvalid, message};
  }

  [[nodiscard]] static GuidanceStatus MissingObjective(
      std::string_view message) {
    return {GuidanceStatusCode::kMissingObjective, message};
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
    out.status_ = GuidanceStatus::Ok();
    out.value_ = value;
    return out;
  }

  [[nodiscard]] static StatusOr Failure(GuidanceStatus status, T value = T{}) {
    StatusOr out;
    out.ok_ = false;
    out.status_ = status;
    out.value_ = value;
    return out;
  }

  [[nodiscard]] bool ok() const { return ok_; }
  [[nodiscard]] const GuidanceStatus& status() const { return status_; }
  [[nodiscard]] const T& value() const { return value_; }

 private:
  StatusOr() = default;
  bool ok_ = false;
  GuidanceStatus status_;
  T value_{};
};

// Shared input contract. Returns the first defect and does not allocate.
//
// Order: update period; state, when non-null (pose frame, finiteness,
// unit quaternion); supplied confidence; supplied horizon; objective;
// snapshot identity, when state is non-null; horizon expiry.
//
// A null state skips the state and snapshot checks. Fields of an
// objective that was not selected are not read. An unset horizon or
// confidence is not a defect. A zero update period is valid. A negative
// update period, horizon, or a non-finite value is kInvalidArgument.
// Confidence outside [0, 1] is kInvalidArgument. A bad frame, a non-unit
// quaternion, and an empty trajectory id are kInvalid. An unset objective
// is kMissingObjective. A snapshot_id that differs from the state, or a
// supplied horizon shorter than the update period, is kStale. A horizon
// equal to the update period still covers that step. A zero horizon with
// a zero update period is an instantaneous sample and is not expired.
// A structurally valid trajectory id returns kOk here. Sampling it is
// not this function.
[[nodiscard]] GuidanceStatus ValidateGuidanceInputs(
    const DesiredMotionRt& intent, const VehicleStateRt* state,
    Duration update_period);

// Narrow guidance boundary. Implementations must reject the defects
// ValidateGuidanceInputs names, must return a finite reference, and must
// keep Evaluate const and free of unsynchronized mutable state. On
// failure the reference is empty. This class does not write actuator
// commands.
class GuidanceStep {
 public:
  virtual ~GuidanceStep() = default;

  GuidanceStep(const GuidanceStep&) = delete;
  GuidanceStep& operator=(const GuidanceStep&) = delete;
  GuidanceStep(GuidanceStep&&) = delete;
  GuidanceStep& operator=(GuidanceStep&&) = delete;

  // state may be null. Evaluate does not retain state or intent.
  [[nodiscard]] virtual StatusOr<MotionReferenceRt> Evaluate(
      const DesiredMotionRt& intent, const VehicleStateRt* state,
      Duration update_period) const = 0;

 protected:
  GuidanceStep() = default;
};

}  // namespace intrinsic::vehicle::guidance

#endif  // INTRINSIC_VEHICLE_GUIDANCE_GUIDANCE_STEP_H_
