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
#ifndef INTRINSIC_SIMULATION_RANDOMIZATION_MARINE_PARAMETER_SAMPLER_H_
#define INTRINSIC_SIMULATION_RANDOMIZATION_MARINE_PARAMETER_SAMPLER_H_

#include <array>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "intrinsic/embodiment/frame_policy.h"
#include "intrinsic/hardware/marine/fake_imu.h"
#include "intrinsic/vehicle/parameters/marine_model.h"
#include "intrinsic/world/current_field_component/current_field_component_policy.h"

namespace intrinsic::simulation {

// Seeded uniform sampling of existing marine simulation parameters. Plain
// C++ values only: no protos, no simulator plugin, no new dynamics. Nothing
// here runs unless a caller constructs a config and calls
// SampleMarineParameters. Not for the ICON cycle.
//
// Randomized fields are limited to:
//   * MarineModel mass, displaced volume (buoyancy), the six diagonal linear
//     damping coefficients, and the six quadratic damping coefficients;
//   * ThrusterGeometry.efficiency;
//   * the current velocity (MarineModel.environment.current_velocity_m_s and
//     the world CurrentFieldView velocity), linear x, y, z;
//   * the six FakeImuConfig bias fields.
// A field with no bounds keeps its nominal value and is not sampled. A
// default-constructed config therefore returns the nominal parameters.
//
// Draws use the SplitMix64 / 53-bit-fraction mix of ImuSignedUnitNoise, not
// absl::BitGen and not std:: distributions, so a seed gives the same values
// on every build. Draw k of a seed is
// ImuSignedUnitNoise(seed + k * 0x9E3779B97F4A7C15), the standard SplitMix64
// sequence. Every field owns a fixed draw index (see kMarineSlot*), whether
// or not it has bounds, so enabling a bound on one field never changes the
// value realized for another.

// Closed interval [lo, hi] for one uniform draw. lo == hi realizes exactly
// that value.
struct UniformBounds {
  double lo = 0;
  double hi = 0;
};

inline constexpr int kMarineLinearDampingDiagonals = 6;
inline constexpr int kMarineQuadraticDampingCoefficients = 6;
inline constexpr int kMarineCurrentAxes = 3;
inline constexpr int kMarineImuBiasFields = 6;

struct MarineRandomizationConfig {
  std::optional<UniformBounds> mass_kg;
  std::optional<UniformBounds> displaced_volume_m3;
  // Diagonal entries [i][i] of Damping.linear_coefficients in body-twist
  // order: surge, sway, heave, roll, pitch, yaw. Off-diagonal entries stay
  // nominal so the matrix remains symmetric.
  std::array<std::optional<UniformBounds>, kMarineLinearDampingDiagonals>
      linear_damping_diagonal;
  std::array<std::optional<UniformBounds>, kMarineQuadraticDampingCoefficients>
      quadratic_damping;
  // One bound applied to every thruster. Each thruster draws independently.
  std::optional<UniformBounds> thruster_efficiency;
  // Linear current velocity x, y, z in the nominal current frame.
  std::array<std::optional<UniformBounds>, kMarineCurrentAxes>
      current_velocity_m_s;
  // FakeImuConfig bias order: angular_velocity x, y, z (rad/s), then
  // linear_acceleration x, y, z (m/s^2).
  std::array<std::optional<UniformBounds>, kMarineImuBiasFields> imu_bias;
};

// Nominal values for every field. Fields without bounds are copied from here.
struct MarineNominalParameters {
  vehicle::parameters::MarineModel model;
  hardware::marine::FakeImuConfig imu;
};

// Every sampled or nominal value, in the order of the metadata dump.
struct RealizedMarineParameters {
  uint64_t seed = 0;
  double mass_kg = 0;
  double displaced_volume_m3 = 0;
  std::array<double, kMarineLinearDampingDiagonals> linear_damping_diagonal =
      {};
  std::array<double, kMarineQuadraticDampingCoefficients> quadratic_damping =
      {};
  std::array<double, kMarineCurrentAxes> current_velocity_m_s = {};
  std::array<double, kMarineImuBiasFields> imu_bias = {};
  // One entry per nominal thruster, in model order.
  std::vector<double> thruster_efficiency;
};

// Fixed draw indices. Thruster i uses kMarineSlotThrusterEfficiency + i, so
// the thruster count never shifts any other field.
inline constexpr uint64_t kMarineSlotMass = 0;
inline constexpr uint64_t kMarineSlotDisplacedVolume = 1;
inline constexpr uint64_t kMarineSlotLinearDamping = 2;
inline constexpr uint64_t kMarineSlotQuadraticDamping = 8;
inline constexpr uint64_t kMarineSlotCurrentVelocity = 14;
inline constexpr uint64_t kMarineSlotImuBias = 17;
inline constexpr uint64_t kMarineSlotThrusterEfficiency = 23;

// Returns InvalidArgument, never clamps, when:
//   * a bound is non-finite, lo > hi, or hi - lo overflows;
//   * the nominal model fails ValidateMarineModel;
//   * the model with every bounded field at its lo corner, or at its hi
//     corner, fails ValidateMarineModel (mass <= 0, efficiency outside
//     (0, 1], a non-positive-definite damping matrix, and so on).
// The linear damping matrix is positive definite for every realized value
// because raising a diagonal entry of a positive-definite matrix keeps it
// positive definite, so the lo corner is the worst case. The message names
// the field and the corner.
[[nodiscard]] absl::Status ValidateMarineRandomization(
    const MarineRandomizationConfig& config,
    const MarineNominalParameters& nominal);

// Validates, then realizes one value per field. Same config, nominal, and
// seed give byte-identical FormatMarineMetadata output. Every realized value
// lies in its bounds.
[[nodiscard]] absl::StatusOr<RealizedMarineParameters> SampleMarineParameters(
    const MarineRandomizationConfig& config,
    const MarineNominalParameters& nominal, uint64_t seed);

// Canonical episode-metadata text: a version line, then `path=value` lines
// in fixed order, each ending in '\n'. Floating-point values print with
// "%.17g" so every double round-trips. Paths follow ValidateMarineModel
// field names. No timestamps or host data.
[[nodiscard]] std::string FormatMarineMetadata(
    const RealizedMarineParameters& realized);

// Writes realized values into the matching fields. Other fields are
// untouched. ApplyToMarineModel requires realized.thruster_efficiency to
// have one entry per thruster and returns InvalidArgument otherwise.
[[nodiscard]] absl::Status ApplyToMarineModel(
    const RealizedMarineParameters& realized,
    vehicle::parameters::MarineModel* model);
void ApplyToFakeImuConfig(const RealizedMarineParameters& realized,
                          hardware::marine::FakeImuConfig* config);
// Sets velocity_m_s and velocity_present. The frame id and validity are
// untouched.
void ApplyToCurrentField(const RealizedMarineParameters& realized,
                         world::CurrentFieldView* current_field);

}  // namespace intrinsic::simulation

#endif  // INTRINSIC_SIMULATION_RANDOMIZATION_MARINE_PARAMETER_SAMPLER_H_
