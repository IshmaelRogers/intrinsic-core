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

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <fstream>
#include <iterator>
#include <limits>
#include <optional>
#include <string>
#include <vector>

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "gmock/gmock.h"
#include "gtest/gtest.h"
#include "intrinsic/embodiment/stamped_header_policy.h"
#include "intrinsic/hardware/marine/fake_imu.h"
#include "intrinsic/hardware/marine/imu.pb.h"
#include "intrinsic/hardware/marine/imu_policy.h"
#include "intrinsic/hardware/marine/measurement_health.pb.h"
#include "intrinsic/vehicle/parameters/marine_model.h"
#include "intrinsic/vehicle/parameters/six_thruster_uuv_example.h"
#include "intrinsic/world/current_field_component/current_field_component_policy.h"

namespace intrinsic::simulation {
namespace {

using hardware::marine::FakeImu;
using hardware::marine::FakeImuConfig;
using hardware::marine::ImuAssessment;
using hardware::marine::ImuMeasurementView;
using hardware::marine::SourceHealthView;
using intrinsic_proto::hardware::marine::ImuMeasurement;
using ::testing::HasSubstr;
using vehicle::parameters::kSpatialDof;
using vehicle::parameters::MakeSixThrusterUuvExample;
using vehicle::parameters::MarineModel;
using vehicle::parameters::ValidateMarineModel;

// Bazel runs tests from the runfiles root of the main repository.
constexpr char kGoldenPath[] =
    "intrinsic/simulation/randomization/testdata/"
    "marine_parameter_sampler_seed_42.golden.txt";

constexpr int kSampleCount = 1000;

MarineNominalParameters Nominal() {
  MarineNominalParameters nominal;
  nominal.model = MakeSixThrusterUuvExample();
  return nominal;
}

// Every randomizable field has bounds. Used by the golden and range tests.
MarineRandomizationConfig FullConfig() {
  MarineRandomizationConfig config;
  config.mass_kg = UniformBounds{10.0, 20.0};
  config.displaced_volume_m3 = UniformBounds{0.01, 0.03};
  const double linear_lo[] = {8.0, 20.0, 25.0, 0.3, 1.0, 1.2};
  for (int i = 0; i < kMarineLinearDampingDiagonals; ++i) {
    config.linear_damping_diagonal[i] =
        UniformBounds{linear_lo[i], linear_lo[i] * 2.0};
  }
  for (int i = 0; i < kMarineQuadraticDampingCoefficients; ++i) {
    config.quadratic_damping[i] = UniformBounds{0.0, 10.0 + i};
  }
  config.thruster_efficiency = UniformBounds{0.75, 1.0};
  config.current_velocity_m_s[0] = UniformBounds{-0.5, 0.5};
  config.current_velocity_m_s[1] = UniformBounds{-0.25, 0.25};
  config.current_velocity_m_s[2] = UniformBounds{-0.1, 0.1};
  for (int i = 0; i < kMarineImuBiasFields; ++i) {
    config.imu_bias[i] = UniformBounds{-0.01 * (i + 1), 0.02 * (i + 1)};
  }
  return config;
}

std::string ReadGolden() {
  std::ifstream file(kGoldenPath, std::ios::binary);
  EXPECT_TRUE(file.good()) << kGoldenPath;
  return std::string(std::istreambuf_iterator<char>(file),
                     std::istreambuf_iterator<char>());
}

RealizedMarineParameters Sample(const MarineRandomizationConfig& config,
                                uint64_t seed) {
  absl::StatusOr<RealizedMarineParameters> realized =
      SampleMarineParameters(config, Nominal(), seed);
  EXPECT_TRUE(realized.ok()) << realized.status();
  return realized.ok() ? *realized : RealizedMarineParameters{};
}

ImuMeasurementView ViewOf(const ImuMeasurement& sample,
                          std::vector<double>* covariance,
                          std::vector<SourceHealthView>* sources) {
  ImuMeasurementView view;
  const auto& health = sample.health();
  view.health.header_present = health.has_header();
  view.health.frame_id = health.header().frame_id();
  view.health.source_time_present = health.header().has_source_time();
  view.health.source_time =
      embodiment::ClockReading{health.header().source_time().seconds(),
                               health.header().source_time().nanos()};
  view.health.receive_time_present = health.header().has_receive_time();
  view.health.receive_time =
      embodiment::ClockReading{health.header().receive_time().seconds(),
                               health.header().receive_time().nanos()};
  view.health.header_validity_present = health.header().has_validity();
  view.health.header_validity_state = health.header().validity().state();
  view.health.state_present = health.has_state();
  view.health.state = health.state();
  view.health.quality_present = health.has_quality();
  view.health.quality = health.quality();
  view.health.covariance_present = health.has_covariance();
  if (view.health.covariance_present) {
    covariance->assign(health.covariance().values().begin(),
                       health.covariance().values().end());
    view.health.covariance = *covariance;
  }
  sources->clear();
  for (const auto& source : health.sources()) {
    sources->push_back(SourceHealthView{
        source.source_id(), source.has_validity(), source.validity().state()});
  }
  view.health.sources = *sources;
  view.angular_velocity_x_present = sample.has_angular_velocity_x_rad_s();
  view.angular_velocity_x_rad_s = sample.angular_velocity_x_rad_s();
  view.angular_velocity_y_present = sample.has_angular_velocity_y_rad_s();
  view.angular_velocity_y_rad_s = sample.angular_velocity_y_rad_s();
  view.angular_velocity_z_present = sample.has_angular_velocity_z_rad_s();
  view.angular_velocity_z_rad_s = sample.angular_velocity_z_rad_s();
  view.linear_acceleration_x_present = sample.has_linear_acceleration_x_m_s2();
  view.linear_acceleration_x_m_s2 = sample.linear_acceleration_x_m_s2();
  view.linear_acceleration_y_present = sample.has_linear_acceleration_y_m_s2();
  view.linear_acceleration_y_m_s2 = sample.linear_acceleration_y_m_s2();
  view.linear_acceleration_z_present = sample.has_linear_acceleration_z_m_s2();
  view.linear_acceleration_z_m_s2 = sample.linear_acceleration_z_m_s2();
  view.orientation_present = sample.has_orientation_xyzw();
  view.orientation_x = sample.orientation_xyzw().x();
  view.orientation_y = sample.orientation_xyzw().y();
  view.orientation_z = sample.orientation_xyzw().z();
  view.orientation_w = sample.orientation_xyzw().w();
  return view;
}

ImuAssessment Assess(const ImuMeasurement& sample) {
  std::vector<double> covariance;
  std::vector<SourceHealthView> sources;
  return hardware::marine::AssessImu(ViewOf(sample, &covariance, &sources));
}

void ExpectInBounds(double value, const std::optional<UniformBounds>& bounds) {
  ASSERT_TRUE(bounds.has_value());
  EXPECT_GE(value, bounds->lo);
  EXPECT_LE(value, bounds->hi);
}

// Acceptance 1.
TEST(MarineParameterSamplerTest, SameSeedAndConfigGiveByteIdenticalMetadata) {
  const MarineRandomizationConfig config = FullConfig();
  const std::string first = FormatMarineMetadata(Sample(config, 42));
  const std::string second = FormatMarineMetadata(Sample(config, 42));
  EXPECT_EQ(first, second);
  EXPECT_EQ(first, ReadGolden());
}

// Acceptance 2.
TEST(MarineParameterSamplerTest, DifferentSeedsDifferAndStayInBounds) {
  const MarineRandomizationConfig config = FullConfig();
  const RealizedMarineParameters baseline = Sample(config, 0);
  int differing_mass = 0;
  int differing_current = 0;
  for (uint64_t seed = 0; seed < kSampleCount; ++seed) {
    const RealizedMarineParameters realized = Sample(config, seed);
    ExpectInBounds(realized.mass_kg, config.mass_kg);
    ExpectInBounds(realized.displaced_volume_m3, config.displaced_volume_m3);
    for (int i = 0; i < kMarineLinearDampingDiagonals; ++i) {
      ExpectInBounds(realized.linear_damping_diagonal[i],
                     config.linear_damping_diagonal[i]);
    }
    for (int i = 0; i < kMarineQuadraticDampingCoefficients; ++i) {
      ExpectInBounds(realized.quadratic_damping[i],
                     config.quadratic_damping[i]);
    }
    for (int i = 0; i < kMarineCurrentAxes; ++i) {
      ExpectInBounds(realized.current_velocity_m_s[i],
                     config.current_velocity_m_s[i]);
    }
    for (int i = 0; i < kMarineImuBiasFields; ++i) {
      ExpectInBounds(realized.imu_bias[i], config.imu_bias[i]);
    }
    ASSERT_EQ(realized.thruster_efficiency.size(), 6u);
    for (double efficiency : realized.thruster_efficiency) {
      ExpectInBounds(efficiency, config.thruster_efficiency);
    }
    if (seed != 0) {
      EXPECT_NE(FormatMarineMetadata(realized), FormatMarineMetadata(baseline));
      differing_mass += realized.mass_kg != baseline.mass_kg;
      differing_current +=
          realized.current_velocity_m_s != baseline.current_velocity_m_s;
    }
  }
  EXPECT_EQ(differing_mass, kSampleCount - 1);
  EXPECT_EQ(differing_current, kSampleCount - 1);
}

TEST(MarineParameterSamplerTest, SamplesSpreadAcrossTheInterval) {
  MarineRandomizationConfig config;
  config.mass_kg = UniformBounds{10.0, 20.0};
  double sum = 0;
  double lowest = 20.0;
  double highest = 10.0;
  for (uint64_t seed = 0; seed < kSampleCount; ++seed) {
    const double mass = Sample(config, seed).mass_kg;
    sum += mass;
    lowest = std::min(lowest, mass);
    highest = std::max(highest, mass);
  }
  EXPECT_NEAR(sum / kSampleCount, 15.0, 0.5);
  EXPECT_LT(lowest, 10.2);
  EXPECT_GT(highest, 19.8);
}

TEST(MarineParameterSamplerTest, EnablingOneBoundDoesNotShiftAnotherField) {
  MarineRandomizationConfig only_mass;
  only_mass.mass_kg = UniformBounds{10.0, 20.0};
  MarineRandomizationConfig mass_and_volume = only_mass;
  mass_and_volume.displaced_volume_m3 = UniformBounds{0.01, 0.03};
  EXPECT_EQ(Sample(only_mass, 7).mass_kg, Sample(mass_and_volume, 7).mass_kg);
}

TEST(MarineParameterSamplerTest, UnboundedFieldsKeepNominalValues) {
  MarineRandomizationConfig config;
  config.mass_kg = UniformBounds{10.0, 20.0};
  const MarineNominalParameters nominal = Nominal();
  const RealizedMarineParameters realized = Sample(config, 9);
  EXPECT_EQ(realized.displaced_volume_m3,
            nominal.model.buoyancy.displaced_volume_m3);
  EXPECT_EQ(realized.quadratic_damping,
            nominal.model.damping.quadratic_coefficients);
  EXPECT_EQ(realized.linear_damping_diagonal[2],
            nominal.model.damping.linear_coefficients[2 * kSpatialDof + 2]);
  EXPECT_EQ(realized.thruster_efficiency[3],
            nominal.model.thrusters[3].efficiency);
}

TEST(MarineParameterSamplerTest, EmptyConfigReturnsNominalValues) {
  MarineNominalParameters nominal = Nominal();
  nominal.imu.angular_velocity_bias_z_rad_s = 0.125;
  nominal.model.environment.current_velocity_m_s = {0.1, -0.2, 0.3};
  absl::StatusOr<RealizedMarineParameters> realized =
      SampleMarineParameters(MarineRandomizationConfig{}, nominal, 123);
  ASSERT_TRUE(realized.ok()) << realized.status();

  MarineModel model = nominal.model;
  model.mass_inertia.mass_kg += 1.0;
  ASSERT_TRUE(ApplyToMarineModel(*realized, &model).ok());
  EXPECT_EQ(model.mass_inertia.mass_kg, nominal.model.mass_inertia.mass_kg);
  EXPECT_EQ(model.damping.linear_coefficients,
            nominal.model.damping.linear_coefficients);
  EXPECT_EQ(model.damping.quadratic_coefficients,
            nominal.model.damping.quadratic_coefficients);
  EXPECT_EQ(model.environment.current_velocity_m_s.y, -0.2);
  FakeImuConfig imu;
  ApplyToFakeImuConfig(*realized, &imu);
  EXPECT_EQ(imu.angular_velocity_bias_z_rad_s, 0.125);
}

// Acceptance 3.
TEST(MarineParameterSamplerTest, DegenerateBoundsRealizeExactValue) {
  MarineRandomizationConfig config;
  config.mass_kg = UniformBounds{12.3456789012345, 12.3456789012345};
  config.thruster_efficiency = UniformBounds{0.8, 0.8};
  config.current_velocity_m_s[1] = UniformBounds{-0.3, -0.3};
  config.imu_bias[4] = UniformBounds{0.007, 0.007};
  for (uint64_t seed : {0ULL, 1ULL, 99ULL, 0xFFFFFFFFFFFFFFFFULL}) {
    const RealizedMarineParameters realized = Sample(config, seed);
    EXPECT_EQ(realized.mass_kg, 12.3456789012345);
    for (double efficiency : realized.thruster_efficiency) {
      EXPECT_EQ(efficiency, 0.8);
    }
    EXPECT_EQ(realized.current_velocity_m_s[1], -0.3);
    EXPECT_EQ(realized.imu_bias[4], 0.007);
  }
}

// Acceptance 4.
TEST(MarineParameterSamplerTest, InvalidBoundsAreRejectedNotClamped) {
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const double inf = std::numeric_limits<double>::infinity();
  const double max = std::numeric_limits<double>::max();
  struct Case {
    const char* name;
    MarineRandomizationConfig config;
    const char* message;
  };
  std::vector<Case> cases;
  const auto add = [&cases](const char* name, auto mutate,
                            const char* message) {
    MarineRandomizationConfig config;
    mutate(config);
    cases.push_back(Case{name, config, message});
  };
  add(
      "lo_greater_than_hi",
      [](auto& c) { c.mass_kg = UniformBounds{20.0, 10.0}; }, "mass_kg");
  add(
      "nan_lo", [&](auto& c) { c.mass_kg = UniformBounds{nan, 10.0}; },
      "finite");
  add(
      "nan_hi", [&](auto& c) { c.imu_bias[2] = UniformBounds{0.0, nan}; },
      "imu_bias[2]");
  add(
      "inf_hi",
      [&](auto& c) { c.current_velocity_m_s[0] = UniformBounds{0.0, inf}; },
      "current_velocity_m_s[0]");
  add(
      "range_overflow",
      [&](auto& c) { c.imu_bias[0] = UniformBounds{-max, max}; }, "finite");
  add(
      "zero_mass_lo", [](auto& c) { c.mass_kg = UniformBounds{0.0, 10.0}; },
      "mass_kg");
  add(
      "negative_mass", [](auto& c) { c.mass_kg = UniformBounds{-2.0, -1.0}; },
      "mass_kg");
  add(
      "zero_volume_lo",
      [](auto& c) { c.displaced_volume_m3 = UniformBounds{0.0, 0.02}; },
      "displaced_volume_m3");
  add(
      "non_positive_linear_damping",
      [](auto& c) { c.linear_damping_diagonal[3] = UniformBounds{-1.0, 2.0}; },
      "linear_coefficients");
  add(
      "negative_quadratic_damping",
      [](auto& c) { c.quadratic_damping[1] = UniformBounds{-0.1, 2.0}; },
      "quadratic_coefficients[1]");
  add(
      "efficiency_above_one",
      [](auto& c) { c.thruster_efficiency = UniformBounds{0.5, 1.1}; },
      "efficiency");
  add(
      "efficiency_zero_lo",
      [](auto& c) { c.thruster_efficiency = UniformBounds{0.0, 0.5}; },
      "efficiency");
  add(
      "damping_not_positive_definite",
      // Nominal surge/sway coupling is 0.5; a 0.1 diagonal at the lo corner
      // makes the leading 2x2 block indefinite.
      [](auto& c) {
        c.linear_damping_diagonal[0] = UniformBounds{0.1, 12.0};
        c.linear_damping_diagonal[1] = UniformBounds{0.1, 30.0};
      },
      "positive definite");
  for (const Case& test_case : cases) {
    SCOPED_TRACE(test_case.name);
    absl::StatusOr<RealizedMarineParameters> realized =
        SampleMarineParameters(test_case.config, Nominal(), 1);
    EXPECT_EQ(realized.status().code(), absl::StatusCode::kInvalidArgument)
        << realized.status();
    EXPECT_THAT(realized.status().message(), HasSubstr(test_case.message));
    EXPECT_EQ(ValidateMarineRandomization(test_case.config, Nominal()).code(),
              absl::StatusCode::kInvalidArgument);
  }
}

TEST(MarineParameterSamplerTest, InvalidNominalModelIsRejected) {
  MarineNominalParameters nominal = Nominal();
  nominal.model.mass_inertia.mass_kg = -1.0;
  absl::StatusOr<RealizedMarineParameters> realized =
      SampleMarineParameters(MarineRandomizationConfig{}, nominal, 1);
  EXPECT_EQ(realized.status().code(), absl::StatusCode::kInvalidArgument);
  EXPECT_THAT(realized.status().message(), HasSubstr("nominal model"));
}

TEST(MarineParameterSamplerTest, NonFiniteNominalImuBiasIsRejected) {
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const double inf = std::numeric_limits<double>::infinity();
  using Field = double FakeImuConfig::*;
  const Field fields[] = {&FakeImuConfig::angular_velocity_bias_x_rad_s,
                          &FakeImuConfig::angular_velocity_bias_y_rad_s,
                          &FakeImuConfig::angular_velocity_bias_z_rad_s,
                          &FakeImuConfig::linear_acceleration_bias_x_m_s2,
                          &FakeImuConfig::linear_acceleration_bias_y_m_s2,
                          &FakeImuConfig::linear_acceleration_bias_z_m_s2};
  for (size_t i = 0; i < std::size(fields); ++i) {
    for (double bad : {nan, inf, -inf}) {
      MarineNominalParameters nominal = Nominal();
      nominal.imu.*fields[i] = bad;
      // Rejected with no bounds, and also when that field is bounded.
      MarineRandomizationConfig bounded;
      bounded.imu_bias[i] = UniformBounds{-1.0, 1.0};
      for (const MarineRandomizationConfig& config :
           {MarineRandomizationConfig{}, bounded}) {
        absl::StatusOr<RealizedMarineParameters> realized =
            SampleMarineParameters(config, nominal, 1);
        EXPECT_EQ(realized.status().code(), absl::StatusCode::kInvalidArgument)
            << "field " << i << " value " << bad;
        EXPECT_THAT(realized.status().message(), HasSubstr("imu bias"));
      }
    }
  }
}

// Acceptance 5.
TEST(MarineParameterSamplerTest, RealizedValuesPassExistingValidators) {
  const MarineRandomizationConfig config = FullConfig();
  const MarineNominalParameters nominal = Nominal();
  for (uint64_t seed = 0; seed < kSampleCount; ++seed) {
    const RealizedMarineParameters realized = Sample(config, seed);
    MarineModel model = nominal.model;
    ASSERT_TRUE(ApplyToMarineModel(realized, &model).ok());
    const auto result = ValidateMarineModel(model);
    ASSERT_TRUE(result.ok())
        << "seed " << seed << ": " << result.errors.front().field << ": "
        << result.errors.front().message;
    EXPECT_EQ(model.mass_inertia.mass_kg, realized.mass_kg);
    EXPECT_EQ(model.damping.linear_coefficients[kSpatialDof + 1],
              realized.linear_damping_diagonal[1]);
    // Off-diagonal coupling is not randomized.
    EXPECT_EQ(model.damping.linear_coefficients[1],
              nominal.model.damping.linear_coefficients[1]);

    FakeImuConfig imu = nominal.imu;
    ApplyToFakeImuConfig(realized, &imu);
    EXPECT_EQ(imu.linear_acceleration_bias_z_m_s2, realized.imu_bias[5]);
    const auto measurement = FakeImu(imu).Measure();
    ASSERT_TRUE(measurement.has_value());
    // A non-zero bias is DEGRADED by design, so `accepted` is false. The
    // validator must still find no structural defect.
    const ImuAssessment assessment = Assess(*measurement);
    EXPECT_EQ(assessment.error, hardware::marine::ImuError::kNone)
        << "seed " << seed;
    EXPECT_EQ(assessment.state,
              hardware::marine::MeasurementStateKind::kDegraded)
        << "seed " << seed;

    world::CurrentFieldView field;
    ApplyToCurrentField(realized, &field);
    EXPECT_TRUE(field.velocity_present);
    EXPECT_EQ(field.velocity_m_s.x, realized.current_velocity_m_s[0]);
    EXPECT_TRUE(embodiment::IsFinite(field.velocity_m_s));
  }
}

TEST(MarineParameterSamplerTest, ApplyRejectsThrusterCountMismatch) {
  RealizedMarineParameters realized = Sample(FullConfig(), 3);
  realized.thruster_efficiency.pop_back();
  MarineModel model = MakeSixThrusterUuvExample();
  EXPECT_EQ(ApplyToMarineModel(realized, &model).code(),
            absl::StatusCode::kInvalidArgument);
}

TEST(MarineParameterSamplerTest, MetadataIsCanonicalText) {
  const std::string metadata = FormatMarineMetadata(Sample(FullConfig(), 42));
  EXPECT_THAT(metadata, ::testing::StartsWith(
                            "marine_randomization_metadata_v1\nseed=42\n"));
  EXPECT_EQ(metadata.back(), '\n');
  EXPECT_THAT(metadata, HasSubstr("\nmass_inertia.mass_kg="));
  EXPECT_THAT(metadata, HasSubstr("\nthrusters[5].efficiency="));
}

}  // namespace
}  // namespace intrinsic::simulation
