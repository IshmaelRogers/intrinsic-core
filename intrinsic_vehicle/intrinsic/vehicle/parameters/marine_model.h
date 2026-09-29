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

#ifndef INTRINSIC_VEHICLE_PARAMETERS_MARINE_MODEL_H_
#define INTRINSIC_VEHICLE_PARAMETERS_MARINE_MODEL_H_

#include <array>
#include <string>
#include <string_view>
#include <vector>

namespace intrinsic::vehicle::parameters {

// SI marine-model parameters for one free-body vehicle. Body axes are
// REP-103: x forward, y left, z up. This header does not evaluate
// dynamics, convert frames, or allocate thruster commands.
//
// Callers opt in by constructing a MarineModel and calling
// ValidateMarineModel. No model is installed when that call is absent.
// ValidateMarineModel reads only its argument, is safe to call from
// multiple threads on distinct inputs, and allocates only the returned
// errors. Do not call it from the ICON cycle.

inline constexpr int kInertiaDof = 3;
inline constexpr int kSpatialDof = 6;

// Body-twist order: linear x, y, z, then angular x, y, z.
inline constexpr int kSurge = 0;
inline constexpr int kSway = 1;
inline constexpr int kHeave = 2;
inline constexpr int kRoll = 3;
inline constexpr int kPitch = 4;
inline constexpr int kYaw = 5;

// Absolute tolerance for |a_ij - a_ji|. A difference equal to this passes.
inline constexpr double kMatrixSymmetryTolerance = 1e-9;
// Absolute tolerance for |norm(direction) - 1|.
inline constexpr double kUnitVectorTolerance = 1e-9;

// Explicit frame ids. They match the embodiment and vehicle contracts.
// This package does not convert among them.
inline constexpr std::string_view kWorldEnuFrameId = "world_enu";
inline constexpr std::string_view kWorldNedFrameId = "world_ned";
inline constexpr std::string_view kBodyFrameId = "body";

struct Vec3 {
  double x = 0;
  double y = 0;
  double z = 0;
};

// mass_kg: kilograms, must be > 0.
// inertia_about_center_of_gravity_kg_m2: row-major 3x3, kg·m², about the
// center of gravity in body axes. Symmetric and positive definite.
// Parallel-axis transport to the body origin is a later dynamics term.
struct MassInertia {
  double mass_kg = 0;
  std::array<double, kInertiaDof * kInertiaDof>
      inertia_about_center_of_gravity_kg_m2 = {};
};

// Meters from the body origin, expressed in the body frame.
struct Centers {
  Vec3 center_of_gravity_m;
  Vec3 center_of_buoyancy_m;
};

// displaced_volume_m3: cubic meters, must be > 0. Buoyancy force is
// density * gravity * volume and is not stored here.
struct Buoyancy {
  double displaced_volume_m3 = 0;
};

// Row-major 6x6 added-mass matrix in body-twist order.
// Diagonal units are kg for surge/sway/heave and kg·m² for roll/pitch/yaw.
// Off-diagonal units are the product of the two generalized-mass units.
// Symmetric and positive definite.
struct AddedMass {
  std::array<double, kSpatialDof * kSpatialDof> coefficients = {};
};

// linear_coefficients: row-major 6x6, symmetric and positive definite.
// Diagonal units are N/(m/s) for surge/sway/heave and N·m/(rad/s) for
// roll/pitch/yaw. Off-diagonal units are the corresponding products.
// quadratic_coefficients: six diagonal coefficients, each >= 0.
// Units are N/(m/s)² and N·m/(rad/s)². Zero is an allowed coefficient.
struct Damping {
  std::array<double, kSpatialDof * kSpatialDof> linear_coefficients = {};
  std::array<double, kSpatialDof> quadratic_coefficients = {};
};

// gravity_m_s2: positive magnitude. Direction is world down and is not
// stored. fluid_density_kg_m3: must be > 0.
// current_velocity_m_s: meters/second in current_frame_id.
// current_frame_id: world_enu, world_ned, or body. Empty and any other id
// are errors. The vector is not converted.
struct Environment {
  double gravity_m_s2 = 0;
  double fluid_density_kg_m3 = 0;
  Vec3 current_velocity_m_s;
  std::string current_frame_id;
};

// Configuration-time health default for one fixed thruster. A later
// allocator may read this value. This package does not command actuators,
// scale an effectiveness matrix, or discover faults.
enum class ThrusterHealthState {
  // Full configured bounds and efficiency. health_derate is 1.
  kNominal = 0,
  // Later consumers command neutral thrust. health_derate is 0.
  kDisabled = 1,
  // Later consumers scale by health_derate in (0, 1).
  kDerated = 2,
  // Thrust stuck at zero. health_derate is 0.
  kStuckOff = 3,
  // Single-actuator failure. health_derate is 0.
  kFailed = 4,
};

// Pose, axis, force bounds, slew, efficiency, and health default for one
// fixed thruster. position_m is meters from the body origin.
// direction_body is a unit axis. Both are expressed in frame_id, which
// must be the body frame (REP-103: x forward, y left, z up). Thrust bounds
// are newtons and are non-negative magnitudes; at least one bound must be
// > 0. Slew limits are newtons per second and must be > 0. efficiency is
// dimensionless in (0, 1]. health_derate is dimensionless and must match
// health. Allocation is not this struct.
struct ThrusterGeometry {
  std::string name;
  std::string frame_id;
  Vec3 position_m;
  Vec3 direction_body;
  double max_forward_thrust_n = 0;
  double max_reverse_thrust_n = 0;
  double max_forward_slew_n_per_s = 0;
  double max_reverse_slew_n_per_s = 0;
  double efficiency = 0;
  ThrusterHealthState health = ThrusterHealthState::kNominal;
  double health_derate = 1;
};

struct MarineModel {
  std::string model_id;
  MassInertia mass_inertia;
  Centers centers;
  Buoyancy buoyancy;
  AddedMass added_mass;
  Damping damping;
  Environment environment;
  std::vector<ThrusterGeometry> thrusters;
};

enum class ModelErrorCode {
  // A required string is empty.
  kEmpty = 0,
  // A numeric value is NaN or infinite.
  kNonFinite = 1,
  // A finite value is not strictly positive.
  kNotPositive = 2,
  // A finite value is negative. Zero remains allowed for that field.
  kNegative = 3,
  // A matrix entry differs from its transpose beyond tolerance.
  kAsymmetric = 4,
  // A symmetric matrix failed the Cholesky positive-definite check.
  kNotPositiveDefinite = 5,
  // Frame id is not allowed for that field.
  kFrame = 6,
  // A finite non-zero direction is not a unit vector.
  kNotUnit = 7,
  // Thruster name repeats an earlier thruster.
  kDuplicate = 8,
  // A finite direction has norm at or below the unit tolerance.
  kZeroAxis = 9,
  // A finite value is outside the allowed interval.
  kOutOfRange = 10,
};

struct FieldError {
  // Dotted path. Matrix entries use [row,col]. Thrusters use thrusters[i].
  // Both thrust bounds zero uses thrusters[i].thrust_bounds.
  // A zero axis uses thrusters[i].direction_body.
  std::string field;
  ModelErrorCode code = ModelErrorCode::kEmpty;
  std::string message;
};

struct ValidationResult {
  std::vector<FieldError> errors;

  [[nodiscard]] bool ok() const { return errors.empty(); }
};

// Collects every defect in schema order. Does not stop at the first error.
// Positive definiteness is checked only after every entry is finite and
// the matrix is symmetric. The definiteness test is unpivoted Cholesky
// with no pivot slack: a non-positive pivot fails.
[[nodiscard]] ValidationResult ValidateMarineModel(const MarineModel& model);

}  // namespace intrinsic::vehicle::parameters

#endif  // INTRINSIC_VEHICLE_PARAMETERS_MARINE_MODEL_H_
