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

#include "intrinsic/vehicle/parameters/six_thruster_uuv_example.h"

#include <array>
#include <string>

namespace intrinsic::vehicle::parameters {
namespace {

constexpr double kMassKg = 32.0;
constexpr double kGravityMPerS2 = 9.80665;
constexpr double kSeawaterDensityKgPerM3 = 1025.0;
// 0.5% above the neutrally buoyant volume mass/density in this fluid.
constexpr double kDisplacedVolumeM3 =
    (kMassKg * 1.005) / kSeawaterDensityKgPerM3;

std::array<double, kSpatialDof * kSpatialDof> DiagonalSpatial(
    double surge, double sway, double heave, double roll, double pitch,
    double yaw) {
  std::array<double, kSpatialDof * kSpatialDof> matrix = {};
  const double diagonal[] = {surge, sway, heave, roll, pitch, yaw};
  for (int i = 0; i < kSpatialDof; ++i) {
    matrix[i * kSpatialDof + i] = diagonal[i];
  }
  return matrix;
}

// Example slew is the matching thrust bound divided by 0.2 s, written as
// a decimal so the stored value is exact. Not a measurement.
ThrusterGeometry Thruster(const char* name, Vec3 position_m, Vec3 direction,
                          double forward_n, double reverse_n,
                          double forward_slew_n_per_s,
                          double reverse_slew_n_per_s) {
  ThrusterGeometry thruster;
  thruster.name = name;
  thruster.frame_id = std::string(kBodyFrameId);
  thruster.position_m = position_m;
  thruster.direction_body = direction;
  thruster.max_forward_thrust_n = forward_n;
  thruster.max_reverse_thrust_n = reverse_n;
  thruster.max_forward_slew_n_per_s = forward_slew_n_per_s;
  thruster.max_reverse_slew_n_per_s = reverse_slew_n_per_s;
  thruster.efficiency = 1.0;
  thruster.health = ThrusterHealthState::kNominal;
  thruster.health_derate = 1.0;
  return thruster;
}

}  // namespace

MarineModel MakeSixThrusterUuvExample() {
  MarineModel model;
  model.model_id = std::string(kSixThrusterUuvModelId);
  model.mass_inertia.mass_kg = kMassKg;
  // Small products of inertia stay symmetric and positive definite.
  model.mass_inertia.inertia_about_center_of_gravity_kg_m2 = {
      0.50, 0.02, 0.00, 0.02, 2.40, 0.01, 0.00, 0.01, 2.70,
  };
  // Body origin is the geometric center. +z is up, so the center of
  // buoyancy sits above the center of gravity.
  model.centers.center_of_gravity_m = {0.02, 0.00, -0.01};
  model.centers.center_of_buoyancy_m = {0.02, 0.00, 0.03};
  model.buoyancy.displaced_volume_m3 = kDisplacedVolumeM3;

  model.added_mass.coefficients =
      DiagonalSpatial(16.0, 40.0, 48.0, 0.25, 2.0, 2.2);

  model.damping.linear_coefficients =
      DiagonalSpatial(12.0, 30.0, 35.0, 0.4, 1.5, 1.8);
  model.damping.linear_coefficients[kSurge * kSpatialDof + kSway] = 0.5;
  model.damping.linear_coefficients[kSway * kSpatialDof + kSurge] = 0.5;
  // Roll quadratic damping is zero and still valid.
  model.damping.quadratic_coefficients = {20.0, 80.0, 90.0, 0.0, 4.0, 4.5};

  model.environment.gravity_m_s2 = kGravityMPerS2;
  model.environment.fluid_density_kg_m3 = kSeawaterDensityKgPerM3;
  model.environment.current_velocity_m_s = {0.0, 0.0, 0.0};
  model.environment.current_frame_id = std::string(kWorldEnuFrameId);

  model.thrusters = {
      Thruster("surge_port", {0.00, 0.18, 0.00}, {1, 0, 0}, 50, 35, 250, 175),
      Thruster("surge_starboard", {0.00, -0.18, 0.10}, {1, 0, 0}, 50, 35, 250,
               175),
      Thruster("sway_fore", {0.40, 0.00, 0.00}, {0, 1, 0}, 30, 30, 150, 150),
      Thruster("sway_aft", {-0.40, 0.00, -0.12}, {0, 1, 0}, 30, 30, 150, 150),
      Thruster("heave_fore", {0.28, 0.16, 0.00}, {0, 0, 1}, 40, 25, 200, 125),
      Thruster("heave_aft", {-0.28, 0.00, 0.00}, {0, 0, 1}, 40, 25, 200, 125),
  };
  return model;
}

}  // namespace intrinsic::vehicle::parameters
