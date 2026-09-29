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

#include "intrinsic/vehicle/dynamics/gravity_buoyancy_restoring_wrench.h"

#include <array>
#include <cmath>
#include <cstring>
#include <limits>
#include <numbers>
#include <string_view>

#include "gtest/gtest.h"
#include "intrinsic/vehicle/dynamics/linear_quadratic_damping_wrench.h"
#include "intrinsic/vehicle/dynamics/zero_force_dynamics.h"
#include "intrinsic/vehicle/parameters/marine_model.h"
#include "intrinsic/vehicle/parameters/six_thruster_uuv_example.h"

namespace {

using intrinsic::vehicle::dynamics::BodyWrenchRt;
using intrinsic::vehicle::dynamics::ComputeGravityBuoyancyRestoringWrench;
using intrinsic::vehicle::dynamics::ComputeLinearQuadraticDampingWrench;
using intrinsic::vehicle::dynamics::DampingWrench;
using intrinsic::vehicle::dynamics::Duration;
using intrinsic::vehicle::dynamics::DynamicsErrorCode;
using intrinsic::vehicle::dynamics::EnvironmentRt;
using intrinsic::vehicle::dynamics::FrameId;
using intrinsic::vehicle::dynamics::kSpatialDof;
using intrinsic::vehicle::dynamics::kUnitQuaternionTolerance;
using intrinsic::vehicle::dynamics::RestoringWrench;
using intrinsic::vehicle::dynamics::StatusOr;
using intrinsic::vehicle::dynamics::VehicleStateRt;
using intrinsic::vehicle::dynamics::ZeroForceDynamics;
using intrinsic::vehicle::parameters::Buoyancy;
using intrinsic::vehicle::parameters::Centers;
using intrinsic::vehicle::parameters::Damping;
using intrinsic::vehicle::parameters::Environment;
using intrinsic::vehicle::parameters::MakeSixThrusterUuvExample;
using intrinsic::vehicle::parameters::MarineModel;
using intrinsic::vehicle::parameters::MassInertia;
using intrinsic::vehicle::parameters::ValidateMarineModel;
using intrinsic::vehicle::parameters::Vec3;

// Hand-calculated wrench fixtures use this absolute tolerance. Identity
// poses below use dyadic inputs, so those products are exact in IEEE
// binary64. Rotations that include sqrt(2)/2 are compared at the same
// tolerance. 1e-12 matches the damping and Coriolis fixtures.
constexpr double kAbsTolerance = 1e-12;

constexpr std::string_view kMassNonFiniteMessage =
    "mass_inertia.mass_kg must be finite";
constexpr std::string_view kMassNotPositiveMessage =
    "mass_inertia.mass_kg must be greater than zero";
constexpr std::string_view kCenterOfGravityNonFiniteMessage =
    "centers.center_of_gravity_m must be finite";
constexpr std::string_view kCenterOfBuoyancyNonFiniteMessage =
    "centers.center_of_buoyancy_m must be finite";
constexpr std::string_view kVolumeNonFiniteMessage =
    "buoyancy.displaced_volume_m3 must be finite";
constexpr std::string_view kVolumeNotPositiveMessage =
    "buoyancy.displaced_volume_m3 must be greater than zero";
constexpr std::string_view kGravityNonFiniteMessage =
    "environment.gravity_m_s2 must be finite";
constexpr std::string_view kGravityNotPositiveMessage =
    "environment.gravity_m_s2 must be greater than zero";
constexpr std::string_view kDensityNonFiniteMessage =
    "environment.fluid_density_kg_m3 must be finite";
constexpr std::string_view kDensityNotPositiveMessage =
    "environment.fluid_density_kg_m3 must be greater than zero";
constexpr std::string_view kPoseFrameMessage =
    "pose_frame must be world_enu or world_ned";
constexpr std::string_view kOrientationNonFiniteMessage =
    "orientation_xyzw must be finite";
constexpr std::string_view kOrientationUnitMessage =
    "orientation_xyzw must have unit norm";
constexpr std::string_view kRestoringWrenchNonFiniteMessage =
    "restoring wrench is not finite";

using Wrench = std::array<double, kSpatialDof>;
using Quaternion = std::array<double, 4>;

constexpr Quaternion kIdentity = {0.0, 0.0, 0.0, 1.0};
// 180 deg about body x. Body +z points navigation -z. Exact.
constexpr Quaternion kRoll180 = {1.0, 0.0, 0.0, 0.0};

// m = 16, g = 4, ρ = 2, V = 8. W = B = 64. All values are powers of two.
constexpr double kMassKg = 16.0;
constexpr double kGravity = 4.0;
constexpr double kDensity = 2.0;
constexpr double kVolume = 8.0;
constexpr double kWeight = 64.0;

// Horizontal centers. Identity ENU, neutral:
//   f_W^b = [0, 0, -64], f_B^b = [0, 0, 64]
//   r_g × f_W = [0.5, 0, 0] × [0, 0, -64] = [0, 32, 0]
//   r_b × f_B = [0, 0.25, 0] × [0, 0, 64] = [16, 0, 0]
//   τ_g = [0, 0, 0, 16, 32, 0]
constexpr Vec3 kOffsetGravity = {0.5, 0.0, 0.0};
constexpr Vec3 kOffsetBuoyancy = {0.0, 0.25, 0.0};
constexpr Wrench kOffsetMoment = {0.0, 0.0, 0.0, 16.0, 32.0, 0.0};

// Same centers, 90 deg roll, neutral. Body +y points ENU +z.
//   f_W^b = [0, -64, 0], f_B^b = [0, 64, 0]
//   r_g × f_W = [0.5, 0, 0] × [0, -64, 0] = [0, 0, -32]
//   r_b × f_B = 0
//   τ_g = [0, 0, 0, 0, 0, -32]
constexpr Wrench kRolledOffsetMoment = {0.0, 0.0, 0.0, 0.0, 0.0, -32.0};

// CB a quarter meter above the origin along body +z, CG at the origin.
// 90 deg roll: f_B^b = [0, 64, 0], r_b × f_B = [0, 0, 0.25] × [0, 64, 0]
//   = [-16, 0, 0]. The roll moment opposes the heel.
constexpr Vec3 kBuoyancyAbove = {0.0, 0.0, 0.25};
constexpr Wrench kRestoringRoll = {0.0, 0.0, 0.0, -16.0, 0.0, 0.0};

struct RestoringInputs {
  MassInertia mass_inertia;
  Buoyancy buoyancy;
  Centers centers;
  Environment environment;
};

RestoringInputs NeutralInputs() {
  RestoringInputs inputs;
  inputs.mass_inertia.mass_kg = kMassKg;
  inputs.buoyancy.displaced_volume_m3 = kVolume;
  inputs.environment.gravity_m_s2 = kGravity;
  inputs.environment.fluid_density_kg_m3 = kDensity;
  inputs.environment.current_frame_id = "body";
  return inputs;
}

Quaternion Roll90() {
  const double half_sqrt2 = std::numbers::sqrt2 / 2.0;
  return {half_sqrt2, 0.0, 0.0, half_sqrt2};
}

Quaternion NedFromEnuWorld() {
  const double half_sqrt2 = std::numbers::sqrt2 / 2.0;
  return {half_sqrt2, half_sqrt2, 0.0, 0.0};
}

// q_ned_from_body = q_ned_from_enu ⊗ q_enu_from_body. Same Hamilton product
// as the embodiment frame policy.
Quaternion HamiltonProduct(const Quaternion& lhs, const Quaternion& rhs) {
  return {
      (lhs[3] * rhs[0]) + (lhs[0] * rhs[3]) + (lhs[1] * rhs[2]) -
          (lhs[2] * rhs[1]),
      (lhs[3] * rhs[1]) - (lhs[0] * rhs[2]) + (lhs[1] * rhs[3]) +
          (lhs[2] * rhs[0]),
      (lhs[3] * rhs[2]) + (lhs[0] * rhs[1]) - (lhs[1] * rhs[0]) +
          (lhs[2] * rhs[3]),
      (lhs[3] * rhs[3]) - (lhs[0] * rhs[0]) - (lhs[1] * rhs[1]) -
          (lhs[2] * rhs[2]),
  };
}

Quaternion Negate(const Quaternion& quaternion) {
  return {-quaternion[0], -quaternion[1], -quaternion[2], -quaternion[3]};
}

StatusOr<RestoringWrench> Compute(const RestoringInputs& inputs,
                                  FrameId pose_frame,
                                  const Quaternion& orientation) {
  return ComputeGravityBuoyancyRestoringWrench(
      inputs.mass_inertia, inputs.buoyancy, inputs.centers, inputs.environment,
      pose_frame, orientation);
}

void ExpectBitIdentical(const Wrench& actual, const Wrench& expected) {
  EXPECT_EQ(0, std::memcmp(actual.data(), expected.data(),
                           actual.size() * sizeof(double)));
}

void ExpectNearWrench(const Wrench& actual, const Wrench& expected) {
  for (int i = 0; i < kSpatialDof; ++i) {
    EXPECT_NEAR(actual[i], expected[i], kAbsTolerance) << "component " << i;
  }
}

void ExpectPositiveZero(const Wrench& wrench) {
  const double zero = 0.0;
  for (int i = 0; i < kSpatialDof; ++i) {
    EXPECT_TRUE(std::isfinite(wrench[i])) << i;
    EXPECT_EQ(0, std::memcmp(&wrench[i], &zero, sizeof(double))) << i;
  }
}

void ExpectInvalid(const char* name, std::string_view message,
                   const RestoringInputs& inputs, FrameId pose_frame,
                   const Quaternion& orientation) {
  const StatusOr<RestoringWrench> result =
      Compute(inputs, pose_frame, orientation);
  ASSERT_FALSE(result.ok()) << name;
  EXPECT_EQ(result.status().code, DynamicsErrorCode::kInvalidArgument) << name;
  EXPECT_EQ(result.status().message, message) << name;
  ExpectPositiveZero(result.value().components);
}

TEST(GravityBuoyancyRestoring, NeutralBuoyancyAlignedPoseIsZeroWrench) {
  RestoringInputs coincident = NeutralInputs();
  coincident.centers.center_of_gravity_m = {0.5, -0.25, 0.125};
  coincident.centers.center_of_buoyancy_m = {0.5, -0.25, 0.125};
  const StatusOr<RestoringWrench> enu =
      Compute(coincident, FrameId::kWorldEnu, kIdentity);
  ASSERT_TRUE(enu.ok());
  EXPECT_EQ(enu.status().message, std::string_view());
  ExpectPositiveZero(enu.value().components);
  EXPECT_DOUBLE_EQ(kMassKg * kGravity, kDensity * kGravity * kVolume);

  // Centers share the vertical. The moment arm is parallel to weight.
  RestoringInputs vertical = NeutralInputs();
  vertical.centers.center_of_gravity_m = {0.5, -0.25, -0.5};
  vertical.centers.center_of_buoyancy_m = {0.5, -0.25, 0.25};
  const StatusOr<RestoringWrench> aligned =
      Compute(vertical, FrameId::kWorldEnu, kIdentity);
  ASSERT_TRUE(aligned.ok());
  ExpectPositiveZero(aligned.value().components);

  // The same level vehicle in NED. Body +z still points up.
  const Quaternion ned_level = NedFromEnuWorld();
  const StatusOr<RestoringWrench> ned =
      Compute(vertical, FrameId::kWorldNed, ned_level);
  ASSERT_TRUE(ned.ok());
  ExpectNearWrench(ned.value().components, Wrench{});
  ExpectPositiveZero(enu.value().components);

  // q and -q are the same rotation.
  const StatusOr<RestoringWrench> flipped =
      Compute(coincident, FrameId::kWorldEnu, Negate(kIdentity));
  ASSERT_TRUE(flipped.ok());
  ExpectPositiveZero(flipped.value().components);
}

TEST(GravityBuoyancyRestoring, OffsetCentersMatchRestoringMoment) {
  RestoringInputs inputs = NeutralInputs();
  inputs.centers.center_of_gravity_m = kOffsetGravity;
  inputs.centers.center_of_buoyancy_m = kOffsetBuoyancy;
  const StatusOr<RestoringWrench> result =
      Compute(inputs, FrameId::kWorldEnu, kIdentity);
  ASSERT_TRUE(result.ok());
  ExpectNearWrench(result.value().components, kOffsetMoment);
  EXPECT_NEAR(result.value().components[3], 16.0, kAbsTolerance);
  EXPECT_NEAR(result.value().components[4], 32.0, kAbsTolerance);
  EXPECT_EQ(result.value().components[0], 0.0);
  EXPECT_EQ(result.value().components[1], 0.0);
  EXPECT_EQ(result.value().components[2], 0.0);
  EXPECT_EQ(result.value().components[5], 0.0);

  // Heel 90 deg. CB above CG produces a negative roll moment.
  RestoringInputs metacentric = NeutralInputs();
  metacentric.centers.center_of_buoyancy_m = kBuoyancyAbove;
  const StatusOr<RestoringWrench> roll =
      Compute(metacentric, FrameId::kWorldEnu, Roll90());
  ASSERT_TRUE(roll.ok());
  ExpectNearWrench(roll.value().components, kRestoringRoll);
  EXPECT_LT(roll.value().components[3], 0.0);
  EXPECT_NEAR(roll.value().components[3], -0.25 * kWeight, kAbsTolerance);
}

TEST(GravityBuoyancyRestoring, EnuAndNedExpressTheSameBodyWrench) {
  RestoringInputs inputs = NeutralInputs();
  inputs.centers.center_of_gravity_m = kOffsetGravity;
  inputs.centers.center_of_buoyancy_m = kOffsetBuoyancy;

  const Quaternion enu_level = kIdentity;
  const Quaternion ned_level = HamiltonProduct(NedFromEnuWorld(), enu_level);
  const StatusOr<RestoringWrench> enu =
      Compute(inputs, FrameId::kWorldEnu, enu_level);
  const StatusOr<RestoringWrench> ned =
      Compute(inputs, FrameId::kWorldNed, ned_level);
  ASSERT_TRUE(enu.ok());
  ASSERT_TRUE(ned.ok());
  ExpectNearWrench(enu.value().components, kOffsetMoment);
  ExpectNearWrench(ned.value().components, enu.value().components);
  ExpectNearWrench(ned.value().components, kOffsetMoment);

  // Identity in both frames is not the same physical pose. NED identity
  // points body +z down, so the moment arm flips.
  const StatusOr<RestoringWrench> ned_identity =
      Compute(inputs, FrameId::kWorldNed, kIdentity);
  ASSERT_TRUE(ned_identity.ok());
  ExpectNearWrench(ned_identity.value().components,
                   Wrench{0.0, 0.0, 0.0, -16.0, -32.0, 0.0});

  const Quaternion enu_roll = Roll90();
  const Quaternion ned_roll = HamiltonProduct(NedFromEnuWorld(), enu_roll);
  const StatusOr<RestoringWrench> rolled_enu =
      Compute(inputs, FrameId::kWorldEnu, enu_roll);
  const StatusOr<RestoringWrench> rolled_ned =
      Compute(inputs, FrameId::kWorldNed, ned_roll);
  ASSERT_TRUE(rolled_enu.ok());
  ASSERT_TRUE(rolled_ned.ok());
  ExpectNearWrench(rolled_enu.value().components, kRolledOffsetMoment);
  ExpectNearWrench(rolled_ned.value().components,
                   rolled_enu.value().components);

  // Excess buoyancy is world-up. Level ENU and the matching NED pose both
  // report positive body heave. Inverted, that same world-up force is
  // negative body heave.
  RestoringInputs light = NeutralInputs();
  light.mass_inertia.mass_kg = 8.0;
  const double net_heave = (kDensity * kGravity * kVolume) - (8.0 * kGravity);
  EXPECT_DOUBLE_EQ(net_heave, 32.0);
  const StatusOr<RestoringWrench> light_enu =
      Compute(light, FrameId::kWorldEnu, kIdentity);
  const StatusOr<RestoringWrench> light_ned =
      Compute(light, FrameId::kWorldNed, ned_level);
  ASSERT_TRUE(light_enu.ok());
  ASSERT_TRUE(light_ned.ok());
  constexpr Wrench kLightLevel = {0.0, 0.0, 32.0, 0.0, 0.0, 0.0};
  ExpectNearWrench(light_enu.value().components, kLightLevel);
  ExpectNearWrench(light_ned.value().components, light_enu.value().components);

  const StatusOr<RestoringWrench> inverted =
      Compute(light, FrameId::kWorldEnu, kRoll180);
  ASSERT_TRUE(inverted.ok());
  ExpectNearWrench(inverted.value().components,
                   Wrench{0.0, 0.0, -32.0, 0.0, 0.0, 0.0});
}

TEST(GravityBuoyancyRestoring, IgnoresDragCurrentInertiaAndActuators) {
  MarineModel model = MakeSixThrusterUuvExample();
  ASSERT_TRUE(ValidateMarineModel(model).ok());
  ASSERT_FALSE(model.thrusters.empty());
  ASSERT_GT(model.damping.linear_coefficients[0], 0.0);

  const StatusOr<RestoringWrench> baseline =
      ComputeGravityBuoyancyRestoringWrench(model.mass_inertia, model.buoyancy,
                                            model.centers, model.environment,
                                            FrameId::kWorldEnu, kIdentity);
  ASSERT_TRUE(baseline.ok());
  for (double component : baseline.value().components) {
    EXPECT_TRUE(std::isfinite(component));
  }

  // Non-finite unused fields stay unread. Drag, current, inertia, and
  // thruster commands are not part of τ_g.
  const double nan = std::numeric_limits<double>::quiet_NaN();
  model.mass_inertia.inertia_about_center_of_gravity_kg_m2.fill(nan);
  model.damping.linear_coefficients.fill(nan);
  model.damping.quadratic_coefficients.fill(1.0e6);
  model.added_mass.coefficients.fill(nan);
  model.environment.current_velocity_m_s = {nan, 1.0e6, -1.0e6};
  model.environment.current_frame_id = "not_a_frame";
  model.thrusters.clear();
  const StatusOr<RestoringWrench> contaminated =
      ComputeGravityBuoyancyRestoringWrench(model.mass_inertia, model.buoyancy,
                                            model.centers, model.environment,
                                            FrameId::kWorldEnu, kIdentity);
  ASSERT_TRUE(contaminated.ok());
  ExpectBitIdentical(contaminated.value().components,
                     baseline.value().components);

  Damping damping;
  for (int i = 0; i < kSpatialDof; ++i) {
    damping.linear_coefficients[i * kSpatialDof + i] = 4.0;
  }
  const std::array<double, kSpatialDof> twist = {1.0, -0.5, 0.25,
                                                 0.0, 0.0,  0.0};
  const StatusOr<DampingWrench> drag =
      ComputeLinearQuadraticDampingWrench(damping, twist);
  ASSERT_TRUE(drag.ok());
  EXPECT_LT(drag.value().components[0], 0.0);
  Wrench summed = baseline.value().components;
  for (int i = 0; i < kSpatialDof; ++i) {
    summed[i] += drag.value().components[i];
  }
  EXPECT_NE(0, std::memcmp(summed.data(), baseline.value().components.data(),
                           summed.size() * sizeof(double)));

  // A large actuator wrench and a nonzero twist do not enter Evaluate.
  VehicleStateRt state;
  state.pose_frame = FrameId::kWorldEnu;
  state.orientation_xyzw = kIdentity;
  state.body_twist = twist;
  BodyWrenchRt actuator;
  actuator.force_n = {40.0, -5.0, 12.0};
  actuator.torque_n_m = {1.0, -2.0, 3.0};
  EnvironmentRt environment;
  environment.gravity_m_s2 = kGravity;
  environment.fluid_density_kg_m3 = kDensity;
  environment.current_frame = FrameId::kWorldEnu;
  environment.current_velocity_m_s = {3.0, -1.0, 0.5};
  const ZeroForceDynamics dynamics;
  const StatusOr<intrinsic::vehicle::dynamics::DynamicsResult> evaluated =
      dynamics.Evaluate(state, actuator, environment, Duration{0.25});
  ASSERT_TRUE(evaluated.ok());
  for (double acceleration : evaluated.value().derivative.body_acceleration) {
    EXPECT_EQ(acceleration, 0.0);
  }
  EXPECT_EQ(evaluated.value().diagnostics.model_force_n,
            (std::array<double, 3>{0.0, 0.0, 0.0}));
  EXPECT_EQ(evaluated.value().diagnostics.model_torque_n_m,
            (std::array<double, 3>{0.0, 0.0, 0.0}));
  EXPECT_FALSE(evaluated.value().diagnostics.input_wrench_used);
  EXPECT_FALSE(evaluated.value().diagnostics.allocation_invoked);
}

TEST(GravityBuoyancyRestoring, InvalidInputsReturnStatusAndZeroWrench) {
  const RestoringInputs valid = NeutralInputs();
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const double inf = std::numeric_limits<double>::infinity();

  RestoringInputs nan_mass = valid;
  nan_mass.mass_inertia.mass_kg = nan;
  ExpectInvalid("nan mass", kMassNonFiniteMessage, nan_mass, FrameId::kWorldEnu,
                kIdentity);
  RestoringInputs inf_mass = valid;
  inf_mass.mass_inertia.mass_kg = inf;
  ExpectInvalid("infinite mass", kMassNonFiniteMessage, inf_mass,
                FrameId::kWorldEnu, kIdentity);
  RestoringInputs zero_mass = valid;
  zero_mass.mass_inertia.mass_kg = 0.0;
  ExpectInvalid("zero mass", kMassNotPositiveMessage, zero_mass,
                FrameId::kWorldEnu, kIdentity);
  RestoringInputs negative_mass = valid;
  negative_mass.mass_inertia.mass_kg = -4.0;
  ExpectInvalid("negative mass", kMassNotPositiveMessage, negative_mass,
                FrameId::kWorldEnu, kIdentity);

  RestoringInputs nan_gravity_center = valid;
  nan_gravity_center.centers.center_of_gravity_m = {nan, 0.0, 0.0};
  ExpectInvalid("nan center of gravity", kCenterOfGravityNonFiniteMessage,
                nan_gravity_center, FrameId::kWorldEnu, kIdentity);
  RestoringInputs inf_gravity_center = valid;
  inf_gravity_center.centers.center_of_gravity_m = {0.0, -inf, 0.0};
  ExpectInvalid("infinite center of gravity", kCenterOfGravityNonFiniteMessage,
                inf_gravity_center, FrameId::kWorldEnu, kIdentity);
  RestoringInputs nan_buoyancy_center = valid;
  nan_buoyancy_center.centers.center_of_buoyancy_m = {0.0, 0.0, nan};
  ExpectInvalid("nan center of buoyancy", kCenterOfBuoyancyNonFiniteMessage,
                nan_buoyancy_center, FrameId::kWorldEnu, kIdentity);

  RestoringInputs nan_volume = valid;
  nan_volume.buoyancy.displaced_volume_m3 = nan;
  ExpectInvalid("nan volume", kVolumeNonFiniteMessage, nan_volume,
                FrameId::kWorldEnu, kIdentity);
  RestoringInputs zero_volume = valid;
  zero_volume.buoyancy.displaced_volume_m3 = 0.0;
  ExpectInvalid("zero volume", kVolumeNotPositiveMessage, zero_volume,
                FrameId::kWorldEnu, kIdentity);
  RestoringInputs negative_volume = valid;
  negative_volume.buoyancy.displaced_volume_m3 = -1.0;
  ExpectInvalid("negative volume", kVolumeNotPositiveMessage, negative_volume,
                FrameId::kWorldEnu, kIdentity);

  RestoringInputs nan_gravity = valid;
  nan_gravity.environment.gravity_m_s2 = nan;
  ExpectInvalid("nan gravity", kGravityNonFiniteMessage, nan_gravity,
                FrameId::kWorldEnu, kIdentity);
  RestoringInputs zero_gravity = valid;
  zero_gravity.environment.gravity_m_s2 = 0.0;
  ExpectInvalid("zero gravity", kGravityNotPositiveMessage, zero_gravity,
                FrameId::kWorldEnu, kIdentity);
  RestoringInputs negative_gravity = valid;
  negative_gravity.environment.gravity_m_s2 = -9.8;
  ExpectInvalid("negative gravity", kGravityNotPositiveMessage,
                negative_gravity, FrameId::kWorldEnu, kIdentity);

  RestoringInputs nan_density = valid;
  nan_density.environment.fluid_density_kg_m3 = inf;
  ExpectInvalid("infinite density", kDensityNonFiniteMessage, nan_density,
                FrameId::kWorldEnu, kIdentity);
  RestoringInputs zero_density = valid;
  zero_density.environment.fluid_density_kg_m3 = 0.0;
  ExpectInvalid("zero density", kDensityNotPositiveMessage, zero_density,
                FrameId::kWorldEnu, kIdentity);
  RestoringInputs negative_density = valid;
  negative_density.environment.fluid_density_kg_m3 = -1.0;
  ExpectInvalid("negative density", kDensityNotPositiveMessage,
                negative_density, FrameId::kWorldEnu, kIdentity);

  ExpectInvalid("body pose frame", kPoseFrameMessage, valid, FrameId::kBody,
                kIdentity);
  ExpectInvalid("unspecified pose frame", kPoseFrameMessage, valid,
                FrameId::kUnspecified, kIdentity);

  ExpectInvalid("nan quaternion", kOrientationNonFiniteMessage, valid,
                FrameId::kWorldEnu, {nan, 0.0, 0.0, 1.0});
  ExpectInvalid("infinite quaternion", kOrientationNonFiniteMessage, valid,
                FrameId::kWorldNed, {0.0, inf, 0.0, 0.0});
  ExpectInvalid("zero quaternion", kOrientationUnitMessage, valid,
                FrameId::kWorldEnu, {0.0, 0.0, 0.0, 0.0});
  ExpectInvalid("stretched quaternion", kOrientationUnitMessage, valid,
                FrameId::kWorldEnu, {0.0, 0.0, 0.0, 1.0 + 1e-6});

  // A difference of 1e-12 on w is inside the 1e-9 unit tolerance.
  const StatusOr<RestoringWrench> near_unit =
      Compute(valid, FrameId::kWorldEnu, {0.0, 0.0, 0.0, 1.0 + 1e-12});
  ASSERT_TRUE(near_unit.ok());
  ExpectNearWrench(near_unit.value().components, Wrench{});
  EXPECT_GT(kUnitQuaternionTolerance, 1e-12);

  // Category order: mass, centers, volume, gravity, density, pose frame,
  // orientation, then the wrench.
  RestoringInputs mass_and_pose = valid;
  mass_and_pose.mass_inertia.mass_kg = nan;
  mass_and_pose.centers.center_of_gravity_m.z = inf;
  mass_and_pose.buoyancy.displaced_volume_m3 = -1.0;
  mass_and_pose.environment.gravity_m_s2 = -1.0;
  ExpectInvalid("mass before centers and pose", kMassNonFiniteMessage,
                mass_and_pose, FrameId::kBody, {nan, 0.0, 0.0, 0.0});

  RestoringInputs sign_before_center = valid;
  sign_before_center.mass_inertia.mass_kg = -1.0;
  sign_before_center.centers.center_of_buoyancy_m.x = nan;
  ExpectInvalid("mass sign before center", kMassNotPositiveMessage,
                sign_before_center, FrameId::kWorldEnu, kIdentity);

  RestoringInputs center_order = valid;
  center_order.centers.center_of_gravity_m.y = nan;
  center_order.centers.center_of_buoyancy_m.x = inf;
  center_order.buoyancy.displaced_volume_m3 = nan;
  ExpectInvalid("gravity center before buoyancy center",
                kCenterOfGravityNonFiniteMessage, center_order,
                FrameId::kWorldEnu, kIdentity);

  RestoringInputs buoyancy_center_before_volume = valid;
  buoyancy_center_before_volume.centers.center_of_buoyancy_m.z = inf;
  buoyancy_center_before_volume.buoyancy.displaced_volume_m3 = 0.0;
  ExpectInvalid("buoyancy center before volume",
                kCenterOfBuoyancyNonFiniteMessage,
                buoyancy_center_before_volume, FrameId::kWorldEnu, kIdentity);

  RestoringInputs volume_before_sign = valid;
  volume_before_sign.buoyancy.displaced_volume_m3 = nan;
  volume_before_sign.environment.gravity_m_s2 = 0.0;
  ExpectInvalid("volume non-finite before gravity", kVolumeNonFiniteMessage,
                volume_before_sign, FrameId::kWorldEnu, kIdentity);

  RestoringInputs volume_sign = valid;
  volume_sign.buoyancy.displaced_volume_m3 = -0.5;
  volume_sign.environment.fluid_density_kg_m3 = nan;
  ExpectInvalid("volume sign before density", kVolumeNotPositiveMessage,
                volume_sign, FrameId::kUnspecified, {0.0, 0.0, 0.0, 2.0});

  RestoringInputs gravity_before_density = valid;
  gravity_before_density.environment.gravity_m_s2 = nan;
  gravity_before_density.environment.fluid_density_kg_m3 = -2.0;
  ExpectInvalid("gravity before density", kGravityNonFiniteMessage,
                gravity_before_density, FrameId::kWorldEnu, kIdentity);

  RestoringInputs gravity_sign = valid;
  gravity_sign.environment.gravity_m_s2 = -0.1;
  gravity_sign.environment.fluid_density_kg_m3 = nan;
  ExpectInvalid("gravity sign before density", kGravityNotPositiveMessage,
                gravity_sign, FrameId::kWorldNed, kIdentity);

  RestoringInputs density_before_frame = valid;
  density_before_frame.environment.fluid_density_kg_m3 = 0.0;
  ExpectInvalid("density before pose frame", kDensityNotPositiveMessage,
                density_before_frame, FrameId::kBody, {nan, 0.0, 0.0, 1.0});

  ExpectInvalid("pose frame before orientation", kPoseFrameMessage, valid,
                FrameId::kBody, {nan, 0.0, 0.0, 4.0});
  ExpectInvalid("orientation finite before unit", kOrientationNonFiniteMessage,
                valid, FrameId::kWorldEnu, {nan, 0.0, 0.0, 4.0});

  RestoringInputs huge = valid;
  huge.mass_inertia.mass_kg = 1.0e300;
  huge.environment.gravity_m_s2 = 1.0e200;
  ExpectInvalid("weight overflow", kRestoringWrenchNonFiniteMessage, huge,
                FrameId::kWorldEnu, kIdentity);

  RestoringInputs moment_overflow = valid;
  moment_overflow.centers.center_of_gravity_m = {1.0e308, 0.0, 0.0};
  ExpectInvalid("moment overflow", kRestoringWrenchNonFiniteMessage,
                moment_overflow, FrameId::kWorldEnu, kIdentity);
}

TEST(GravityBuoyancyRestoring, RepeatedEvaluationIsBitIdentical) {
  RestoringInputs inputs = NeutralInputs();
  inputs.centers.center_of_gravity_m = kOffsetGravity;
  inputs.centers.center_of_buoyancy_m = kOffsetBuoyancy;
  const Quaternion ned_roll = HamiltonProduct(NedFromEnuWorld(), Roll90());
  const StatusOr<RestoringWrench> first =
      Compute(inputs, FrameId::kWorldNed, ned_roll);
  const StatusOr<RestoringWrench> second =
      Compute(inputs, FrameId::kWorldNed, ned_roll);
  ASSERT_TRUE(first.ok());
  ASSERT_TRUE(second.ok());
  EXPECT_EQ(first.status().code, second.status().code);
  EXPECT_EQ(first.status().message, second.status().message);
  ExpectBitIdentical(first.value().components, second.value().components);
  ExpectNearWrench(first.value().components, kRolledOffsetMoment);

  RestoringInputs bad = inputs;
  bad.environment.fluid_density_kg_m3 = std::numeric_limits<double>::infinity();
  const StatusOr<RestoringWrench> bad_first =
      Compute(bad, FrameId::kWorldEnu, kIdentity);
  const StatusOr<RestoringWrench> bad_second =
      Compute(bad, FrameId::kWorldEnu, kIdentity);
  ASSERT_FALSE(bad_first.ok());
  ASSERT_FALSE(bad_second.ok());
  EXPECT_EQ(bad_first.status().code, bad_second.status().code);
  EXPECT_EQ(bad_first.status().message, bad_second.status().message);
  EXPECT_EQ(bad_first.status().message, kDensityNonFiniteMessage);
  EXPECT_EQ(bad_first.status().code, DynamicsErrorCode::kInvalidArgument);
  ExpectBitIdentical(bad_first.value().components,
                     bad_second.value().components);
  ExpectPositiveZero(bad_first.value().components);
}

TEST(GravityBuoyancyRestoring, EvaluateDoesNotApplyRestoringWrench) {
  RestoringInputs inputs = NeutralInputs();
  inputs.mass_inertia.mass_kg = 8.0;
  inputs.centers.center_of_gravity_m = kOffsetGravity;
  const StatusOr<RestoringWrench> restoring =
      Compute(inputs, FrameId::kWorldEnu, kIdentity);
  ASSERT_TRUE(restoring.ok());
  EXPECT_GT(std::abs(restoring.value().components[2]) +
                std::abs(restoring.value().components[4]),
            1.0);

  VehicleStateRt state;
  state.pose_frame = FrameId::kWorldEnu;
  state.orientation_xyzw = kIdentity;
  state.body_twist = {0.2, -0.1, 0.0, 0.0, 0.3, -0.4};
  EnvironmentRt environment;
  environment.gravity_m_s2 = kGravity;
  environment.fluid_density_kg_m3 = kDensity;
  environment.current_frame = FrameId::kWorldNed;
  const ZeroForceDynamics model;
  const StatusOr<intrinsic::vehicle::dynamics::DynamicsResult> evaluated =
      model.Evaluate(state, BodyWrenchRt{}, environment, Duration{0.25});
  ASSERT_TRUE(evaluated.ok());
  for (double acceleration : evaluated.value().derivative.body_acceleration) {
    EXPECT_EQ(acceleration, 0.0);
  }
  EXPECT_EQ(evaluated.value().diagnostics.model_force_n,
            (std::array<double, 3>{0.0, 0.0, 0.0}));
  EXPECT_EQ(evaluated.value().diagnostics.model_torque_n_m,
            (std::array<double, 3>{0.0, 0.0, 0.0}));
  EXPECT_FALSE(evaluated.value().diagnostics.input_wrench_used);
  EXPECT_FALSE(evaluated.value().diagnostics.allocation_invoked);
  EXPECT_NE(evaluated.value().derivative.position_dot_m_s,
            (std::array<double, 3>{0.0, 0.0, 0.0}));
}

}  // namespace
