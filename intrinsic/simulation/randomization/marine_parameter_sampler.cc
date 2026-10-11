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
#include "intrinsic/simulation/randomization/marine_parameter_sampler.h"

#include <array>
#include <charconv>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <utility>

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/str_cat.h"
#include "intrinsic/hardware/marine/fake_imu.h"
#include "intrinsic/vehicle/parameters/marine_model.h"
#include "intrinsic/world/current_field_component/current_field_component_policy.h"

namespace intrinsic::simulation {
namespace {

using hardware::marine::FakeImuConfig;
using hardware::marine::ImuSignedUnitNoise;
using vehicle::parameters::kSpatialDof;
using vehicle::parameters::MarineModel;

constexpr uint64_t kSplitMix64Gamma = 0x9E3779B97F4A7C15ULL;

constexpr char kMetadataVersionLine[] = "marine_randomization_metadata_v1\n";

// Uniform in [0, 1) with 2^-53 granularity. ImuSignedUnitNoise returns
// k * 2^-52 - 1 for a 53-bit integer k, so this is exact in binary64.
double UnitFraction(uint64_t seed, uint64_t slot) {
  return (ImuSignedUnitNoise(seed + slot * kSplitMix64Gamma) + 1.0) * 0.5;
}

double DrawUniform(const UniformBounds& bounds, uint64_t seed, uint64_t slot) {
  if (bounds.lo == bounds.hi) {
    return bounds.lo;
  }
  // One correctly rounded fused multiply-add. A plain a * b + c is contracted
  // into an FMA on some targets and not on others, which would change the
  // last bit of the dump between builds.
  const double value =
      std::fma(bounds.hi - bounds.lo, UnitFraction(seed, slot), bounds.lo);
  // fl(hi - lo) can round up, so the sum may land one ulp past hi. This
  // keeps the already-validated interval closed. It never repairs a bound.
  if (value > bounds.hi) {
    return bounds.hi;
  }
  if (value < bounds.lo) {
    return bounds.lo;
  }
  return value;
}

struct LoCorner {
  double operator()(const UniformBounds& bounds, uint64_t) const {
    return bounds.lo;
  }
};

struct HiCorner {
  double operator()(const UniformBounds& bounds, uint64_t) const {
    return bounds.hi;
  }
};

struct SeededDraw {
  uint64_t seed;
  double operator()(const UniformBounds& bounds, uint64_t slot) const {
    return DrawUniform(bounds, seed, slot);
  }
};

// Realizes every field. `pick` maps bounds and a draw slot to a value.
// Unbounded fields take their nominal value.
template <typename Pick>
RealizedMarineParameters Realize(const MarineRandomizationConfig& config,
                                 const MarineNominalParameters& nominal,
                                 uint64_t seed, const Pick& pick) {
  const MarineModel& model = nominal.model;
  const FakeImuConfig& imu = nominal.imu;
  RealizedMarineParameters out;
  out.seed = seed;
  out.mass_kg = config.mass_kg ? pick(*config.mass_kg, kMarineSlotMass)
                               : model.mass_inertia.mass_kg;
  out.displaced_volume_m3 =
      config.displaced_volume_m3
          ? pick(*config.displaced_volume_m3, kMarineSlotDisplacedVolume)
          : model.buoyancy.displaced_volume_m3;
  for (int i = 0; i < kMarineLinearDampingDiagonals; ++i) {
    out.linear_damping_diagonal[i] =
        config.linear_damping_diagonal[i]
            ? pick(*config.linear_damping_diagonal[i],
                   kMarineSlotLinearDamping + i)
            : model.damping.linear_coefficients[i * kSpatialDof + i];
  }
  for (int i = 0; i < kMarineQuadraticDampingCoefficients; ++i) {
    out.quadratic_damping[i] = config.quadratic_damping[i]
                                   ? pick(*config.quadratic_damping[i],
                                          kMarineSlotQuadraticDamping + i)
                                   : model.damping.quadratic_coefficients[i];
  }
  const double nominal_current[kMarineCurrentAxes] = {
      model.environment.current_velocity_m_s.x,
      model.environment.current_velocity_m_s.y,
      model.environment.current_velocity_m_s.z};
  for (int i = 0; i < kMarineCurrentAxes; ++i) {
    out.current_velocity_m_s[i] = config.current_velocity_m_s[i]
                                      ? pick(*config.current_velocity_m_s[i],
                                             kMarineSlotCurrentVelocity + i)
                                      : nominal_current[i];
  }
  const double nominal_bias[kMarineImuBiasFields] = {
      imu.angular_velocity_bias_x_rad_s,   imu.angular_velocity_bias_y_rad_s,
      imu.angular_velocity_bias_z_rad_s,   imu.linear_acceleration_bias_x_m_s2,
      imu.linear_acceleration_bias_y_m_s2, imu.linear_acceleration_bias_z_m_s2};
  for (int i = 0; i < kMarineImuBiasFields; ++i) {
    out.imu_bias[i] = config.imu_bias[i]
                          ? pick(*config.imu_bias[i], kMarineSlotImuBias + i)
                          : nominal_bias[i];
  }
  out.thruster_efficiency.reserve(model.thrusters.size());
  for (size_t i = 0; i < model.thrusters.size(); ++i) {
    out.thruster_efficiency.push_back(
        config.thruster_efficiency
            ? pick(*config.thruster_efficiency, kMarineSlotThrusterBase + i)
            : model.thrusters[i].efficiency);
  }
  return out;
}

absl::Status ValidateBounds(const std::string& field,
                            const std::optional<UniformBounds>& bounds) {
  if (!bounds) {
    return absl::OkStatus();
  }
  if (!std::isfinite(bounds->lo) || !std::isfinite(bounds->hi)) {
    return absl::InvalidArgumentError(
        absl::StrCat(field, ": bounds must be finite"));
  }
  if (bounds->lo > bounds->hi) {
    return absl::InvalidArgumentError(
        absl::StrCat(field, ": lo must be less than or equal to hi"));
  }
  if (!std::isfinite(bounds->hi - bounds->lo)) {
    return absl::InvalidArgumentError(
        absl::StrCat(field, ": hi - lo must be finite"));
  }
  return absl::OkStatus();
}

template <size_t N>
absl::Status ValidateBoundsArray(
    const std::string& field,
    const std::array<std::optional<UniformBounds>, N>& bounds) {
  for (size_t i = 0; i < N; ++i) {
    if (absl::Status status =
            ValidateBounds(absl::StrCat(field, "[", i, "]"), bounds[i]);
        !status.ok()) {
      return status;
    }
  }
  return absl::OkStatus();
}

absl::Status ValidateCorner(const char* corner,
                            const RealizedMarineParameters& realized,
                            const MarineNominalParameters& nominal) {
  MarineModel model = nominal.model;
  if (absl::Status status = ApplyToMarineModel(realized, &model);
      !status.ok()) {
    return status;
  }
  const vehicle::parameters::ValidationResult result =
      vehicle::parameters::ValidateMarineModel(model);
  if (!result.ok()) {
    return absl::InvalidArgumentError(absl::StrCat(
        "bounds produce an invalid model at the ", corner, " corner: ",
        result.errors.front().field, ": ", result.errors.front().message));
  }
  return absl::OkStatus();
}

void AppendValue(std::string* out, const std::string& path, double value) {
  // to_chars ignores the C locale, unlike snprintf.
  char buffer[40];
  const std::to_chars_result result = std::to_chars(
      buffer, buffer + sizeof(buffer), value, std::chars_format::general, 17);
  absl::StrAppend(out, path, "=", std::string_view(buffer, result.ptr - buffer),
                  "\n");
}

}  // namespace

absl::Status ValidateMarineRandomization(
    const MarineRandomizationConfig& config,
    const MarineNominalParameters& nominal) {
  if (absl::Status status = ValidateBounds("mass_kg", config.mass_kg);
      !status.ok()) {
    return status;
  }
  if (absl::Status status =
          ValidateBounds("displaced_volume_m3", config.displaced_volume_m3);
      !status.ok()) {
    return status;
  }
  if (absl::Status status = ValidateBoundsArray("linear_damping_diagonal",
                                                config.linear_damping_diagonal);
      !status.ok()) {
    return status;
  }
  if (absl::Status status =
          ValidateBoundsArray("quadratic_damping", config.quadratic_damping);
      !status.ok()) {
    return status;
  }
  if (absl::Status status =
          ValidateBounds("thruster_efficiency", config.thruster_efficiency);
      !status.ok()) {
    return status;
  }
  if (absl::Status status = ValidateBoundsArray("current_velocity_m_s",
                                                config.current_velocity_m_s);
      !status.ok()) {
    return status;
  }
  if (absl::Status status = ValidateBoundsArray("imu_bias", config.imu_bias);
      !status.ok()) {
    return status;
  }

  const vehicle::parameters::ValidationResult nominal_result =
      vehicle::parameters::ValidateMarineModel(nominal.model);
  if (!nominal_result.ok()) {
    return absl::InvalidArgumentError(absl::StrCat(
        "nominal model is invalid: ", nominal_result.errors.front().field, ": ",
        nominal_result.errors.front().message));
  }

  const double nominal_bias[kMarineImuBiasFields] = {
      nominal.imu.angular_velocity_bias_x_rad_s,
      nominal.imu.angular_velocity_bias_y_rad_s,
      nominal.imu.angular_velocity_bias_z_rad_s,
      nominal.imu.linear_acceleration_bias_x_m_s2,
      nominal.imu.linear_acceleration_bias_y_m_s2,
      nominal.imu.linear_acceleration_bias_z_m_s2};
  for (int i = 0; i < kMarineImuBiasFields; ++i) {
    if (!std::isfinite(nominal_bias[i])) {
      return absl::InvalidArgumentError(
          absl::StrCat("nominal imu bias [", i, "] must be finite"));
    }
  }

  // Two corner checks cover every realized value. ValidateMarineModel has no
  // check that spans fields, so each randomized field is valid on its own
  // interval if it is valid at both ends. The one coupled constraint is the
  // linear damping matrix: raising a diagonal entry of a symmetric
  // positive-definite matrix keeps it positive definite, so the all-lo corner
  // is the worst case.
  if (absl::Status status = ValidateCorner(
          "lo", Realize(config, nominal, 0, LoCorner()), nominal);
      !status.ok()) {
    return status;
  }
  return ValidateCorner("hi", Realize(config, nominal, 0, HiCorner()), nominal);
}

absl::StatusOr<RealizedMarineParameters> SampleMarineParameters(
    const MarineRandomizationConfig& config,
    const MarineNominalParameters& nominal, uint64_t seed) {
  if (absl::Status status = ValidateMarineRandomization(config, nominal);
      !status.ok()) {
    return status;
  }
  return Realize(config, nominal, seed, SeededDraw{seed});
}

std::string FormatMarineMetadata(const RealizedMarineParameters& realized) {
  std::string out = kMetadataVersionLine;
  absl::StrAppend(&out, "seed=", realized.seed, "\n");
  AppendValue(&out, "mass_inertia.mass_kg", realized.mass_kg);
  AppendValue(&out, "buoyancy.displaced_volume_m3",
              realized.displaced_volume_m3);
  for (int i = 0; i < kMarineLinearDampingDiagonals; ++i) {
    AppendValue(&out,
                absl::StrCat("damping.linear_coefficients[", i, ",", i, "]"),
                realized.linear_damping_diagonal[i]);
  }
  for (int i = 0; i < kMarineQuadraticDampingCoefficients; ++i) {
    AppendValue(&out, absl::StrCat("damping.quadratic_coefficients[", i, "]"),
                realized.quadratic_damping[i]);
  }
  constexpr const char* kAxes[kMarineCurrentAxes] = {"x", "y", "z"};
  for (int i = 0; i < kMarineCurrentAxes; ++i) {
    AppendValue(&out,
                absl::StrCat("environment.current_velocity_m_s.", kAxes[i]),
                realized.current_velocity_m_s[i]);
  }
  constexpr const char* kImuBiasNames[kMarineImuBiasFields] = {
      "angular_velocity_bias_x_rad_s",   "angular_velocity_bias_y_rad_s",
      "angular_velocity_bias_z_rad_s",   "linear_acceleration_bias_x_m_s2",
      "linear_acceleration_bias_y_m_s2", "linear_acceleration_bias_z_m_s2"};
  for (int i = 0; i < kMarineImuBiasFields; ++i) {
    AppendValue(&out, absl::StrCat("imu.", kImuBiasNames[i]),
                realized.imu_bias[i]);
  }
  for (size_t i = 0; i < realized.thruster_efficiency.size(); ++i) {
    AppendValue(&out, absl::StrCat("thrusters[", i, "].efficiency"),
                realized.thruster_efficiency[i]);
  }
  return out;
}

absl::Status ApplyToMarineModel(const RealizedMarineParameters& realized,
                                MarineModel* model) {
  if (realized.thruster_efficiency.size() != model->thrusters.size()) {
    return absl::InvalidArgumentError(
        "thruster_efficiency size does not match the model thruster count");
  }
  model->mass_inertia.mass_kg = realized.mass_kg;
  model->buoyancy.displaced_volume_m3 = realized.displaced_volume_m3;
  for (int i = 0; i < kMarineLinearDampingDiagonals; ++i) {
    model->damping.linear_coefficients[i * kSpatialDof + i] =
        realized.linear_damping_diagonal[i];
  }
  model->damping.quadratic_coefficients = realized.quadratic_damping;
  model->environment.current_velocity_m_s.x = realized.current_velocity_m_s[0];
  model->environment.current_velocity_m_s.y = realized.current_velocity_m_s[1];
  model->environment.current_velocity_m_s.z = realized.current_velocity_m_s[2];
  for (size_t i = 0; i < model->thrusters.size(); ++i) {
    model->thrusters[i].efficiency = realized.thruster_efficiency[i];
  }
  return absl::OkStatus();
}

void ApplyToFakeImuConfig(const RealizedMarineParameters& realized,
                          FakeImuConfig* config) {
  config->angular_velocity_bias_x_rad_s = realized.imu_bias[0];
  config->angular_velocity_bias_y_rad_s = realized.imu_bias[1];
  config->angular_velocity_bias_z_rad_s = realized.imu_bias[2];
  config->linear_acceleration_bias_x_m_s2 = realized.imu_bias[3];
  config->linear_acceleration_bias_y_m_s2 = realized.imu_bias[4];
  config->linear_acceleration_bias_z_m_s2 = realized.imu_bias[5];
}

void ApplyToCurrentField(const RealizedMarineParameters& realized,
                         world::CurrentFieldView* current_field) {
  current_field->velocity_present = true;
  current_field->velocity_m_s = embodiment::Vec3{
      realized.current_velocity_m_s[0], realized.current_velocity_m_s[1],
      realized.current_velocity_m_s[2]};
}

}  // namespace intrinsic::simulation
