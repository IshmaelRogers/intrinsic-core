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

// Polymorphic free-body dynamics interface (PDR §6).
//
// Evaluate accepts a fixed-size state, body wrench, environment, and time
// step. It returns the state derivative and diagnostics, or a status. The
// derivative is an instantaneous rate. Evaluate does not integrate it.
//
// Callers opt in by constructing a concrete VehicleDynamics. This package
// installs no process-wide model. With no instance, vehicle dynamics stay
// absent and manipulator control is unchanged.
//
// Allocation. Evaluate does not map a wrench to actuator commands, does not
// read actuator health, and does not call the allocation package. Control
// allocation remains a later real-time interface. Returned values are
// fixed-size. ValidateEvaluationInputs does not allocate heap memory.
// Status text points at static string literals. Evaluate does not retain
// its arguments after it returns.
//
// Thread safety. Evaluate is const. Concurrent Evaluate calls on one
// instance are safe when the implementation has no unsynchronized mutable
// members. ValidateEvaluationInputs reads only its arguments. This package
// takes no locks and is not used from the ICON cycle.
//
// This header includes only the C++ standard library. Body axes are
// REP-103: x forward, y left, z up. Twist and body-acceleration order is
// surge, sway, heave, roll, pitch, yaw, the same order as the parameters
// package. Orientation is a Hamilton quaternion stored x, y, z, w. Frames
// are explicit. This package does not convert ENU and NED.

#ifndef INTRINSIC_VEHICLE_DYNAMICS_VEHICLE_DYNAMICS_H_
#define INTRINSIC_VEHICLE_DYNAMICS_VEHICLE_DYNAMICS_H_

#include <array>
#include <string_view>

namespace intrinsic::vehicle::dynamics {

inline constexpr int kSpatialDof = 6;

// Body-twist and body-acceleration order.
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
// this passes. Helpers do not renormalize.
inline constexpr double kUnitQuaternionTolerance = 1e-9;

// Explicit frame ids. Append values. Do not renumber.
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

// Fixed-size evaluation state. pose_frame is the frame of position_m and
// orientation_xyzw. body_twist is the velocity of the body origin in the
// body frame: linear meters/second, then angular radians/second.
struct VehicleStateRt {
  FrameId pose_frame = FrameId::kUnspecified;
  std::array<double, 3> position_m = {};
  std::array<double, 4> orientation_xyzw = {0, 0, 0, 1};
  std::array<double, kSpatialDof> body_twist = {};
};

// Generalized force at the body origin. Force is newtons. Torque is
// newton-meters. frame must be kBody.
struct BodyWrenchRt {
  FrameId frame = FrameId::kBody;
  std::array<double, 3> force_n = {};
  std::array<double, 3> torque_n_m = {};
};

// Evaluation-time environment snapshot. This is not the configuration
// schema in the parameters package. gravity_m_s2 is a magnitude. Current
// is meters/second in current_frame. This package does not convert it.
struct EnvironmentRt {
  double gravity_m_s2 = 0;
  double fluid_density_kg_m3 = 0;
  std::array<double, 3> current_velocity_m_s = {};
  FrameId current_frame = FrameId::kUnspecified;
};

// Instantaneous state derivative. position_dot_m_s is the rate of
// position_m in pose_frame. orientation_dot_xyzw is the rate of the stored
// quaternion. body_acceleration is body-frame linear meters/second^2, then
// angular radians/second^2, in surge..yaw order.
struct StateDerivative {
  std::array<double, 3> position_dot_m_s = {};
  std::array<double, 4> orientation_dot_xyzw = {};
  std::array<double, kSpatialDof> body_acceleration = {};
};

// Diagnostics for one evaluation. Append fields. model_id points at a
// static string with program lifetime.
struct DynamicsDiagnostics {
  std::string_view model_id;
  double dt_s = 0;
  // Wrench added by the model, excluding the input wrench.
  std::array<double, 3> model_force_n = {};
  std::array<double, 3> model_torque_n_m = {};
  // True when the implementation consumed the input wrench as a force.
  bool input_wrench_used = false;
  // True when the implementation solved actuator commands.
  bool allocation_invoked = false;
  // Marine-force terms. Append-only. Callers that do not compose them,
  // including ZeroForceDynamics, leave them zero.
  //
  // relative_twist is ν_r. The next four arrays are body-frame
  // generalized forces in surge, sway, heave, roll, pitch, yaw order.
  // Their sum is hydrodynamic_wrench:
  //
  //   rigid_body_coriolis_wrench = -C_RB(ν) ν
  //   added_mass_coriolis_wrench = -C_A(ν_r) ν_r
  //   damping_wrench = τ_damp(ν_r) from the damping helper
  //   restoring_wrench = τ_g from the restoring helper
  //
  // The damping and restoring helpers already return the force on the
  // body (τ_damp = -(D_L + D_Q) ν_r, and τ_g is weight plus buoyancy).
  // Substituting those helper signs gives
  // -C_RB(ν) ν - C_A(ν_r) ν_r - D(ν_r) ν_r - g(η).
  // hydrodynamic_wrench is copied into model_force_n and model_torque_n_m.
  // total_wrench is the input wrench plus hydrodynamic_wrench. It is the
  // derivative input passed through from BodyWrenchRt. This result has
  // no mass-matrix field, so composition does not form M or ν̇.
  std::array<double, kSpatialDof> relative_twist = {};
  std::array<double, kSpatialDof> rigid_body_coriolis_wrench = {};
  std::array<double, kSpatialDof> added_mass_coriolis_wrench = {};
  std::array<double, kSpatialDof> damping_wrench = {};
  std::array<double, kSpatialDof> restoring_wrench = {};
  std::array<double, kSpatialDof> hydrodynamic_wrench = {};
  std::array<double, kSpatialDof> total_wrench = {};
};

struct DynamicsResult {
  StateDerivative derivative;
  DynamicsDiagnostics diagnostics;
};

// Append codes. kOk is the only success code.
enum class DynamicsErrorCode {
  kOk = 0,
  kInvalidArgument = 1,
};

struct DynamicsStatus {
  DynamicsErrorCode code = DynamicsErrorCode::kOk;
  std::string_view message;

  [[nodiscard]] bool ok() const { return code == DynamicsErrorCode::kOk; }

  [[nodiscard]] static DynamicsStatus Ok() { return {}; }

  [[nodiscard]] static DynamicsStatus InvalidArgument(
      std::string_view message) {
    return {DynamicsErrorCode::kInvalidArgument, message};
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
    out.status_ = DynamicsStatus::Ok();
    out.value_ = value;
    return out;
  }

  [[nodiscard]] static StatusOr Failure(DynamicsStatus status, T value = T{}) {
    StatusOr out;
    out.ok_ = false;
    out.status_ = status;
    out.value_ = value;
    return out;
  }

  [[nodiscard]] bool ok() const { return ok_; }
  [[nodiscard]] const DynamicsStatus& status() const { return status_; }
  [[nodiscard]] const T& value() const { return value_; }

 private:
  StatusOr() = default;
  bool ok_ = false;
  DynamicsStatus status_;
  T value_{};
};

// Shared input contract. Returns the first defect and does not allocate.
// Order: pose frame, state finiteness, unit quaternion, wrench frame,
// wrench finiteness, environment finiteness, gravity sign, density sign,
// current finiteness, current frame, time step.
//
// Pose frame must be world_enu or world_ned. Wrench frame must be body.
// Current frame must be world_enu, world_ned, or body. A zero time step,
// zero gravity, and zero density are valid. Negative time, gravity, and
// density are invalid. Non-finite values are invalid.
[[nodiscard]] DynamicsStatus ValidateEvaluationInputs(
    const VehicleStateRt& state, const BodyWrenchRt& wrench,
    const EnvironmentRt& environment, Duration dt);

// Narrow dynamics boundary. Implementations must reject the defects
// ValidateEvaluationInputs names, must return a finite derivative, and
// must keep Evaluate const and free of unsynchronized mutable state.
class VehicleDynamics {
 public:
  virtual ~VehicleDynamics() = default;

  VehicleDynamics(const VehicleDynamics&) = delete;
  VehicleDynamics& operator=(const VehicleDynamics&) = delete;
  VehicleDynamics(VehicleDynamics&&) = delete;
  VehicleDynamics& operator=(VehicleDynamics&&) = delete;

  [[nodiscard]] virtual StatusOr<DynamicsResult> Evaluate(
      const VehicleStateRt& state, const BodyWrenchRt& wrench,
      const EnvironmentRt& environment, Duration dt) const = 0;

 protected:
  VehicleDynamics() = default;
};

}  // namespace intrinsic::vehicle::dynamics

#endif  // INTRINSIC_VEHICLE_DYNAMICS_VEHICLE_DYNAMICS_H_
