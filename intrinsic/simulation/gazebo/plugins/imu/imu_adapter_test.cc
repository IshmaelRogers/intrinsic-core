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

#include "intrinsic/simulation/gazebo/plugins/imu/imu_adapter.h"

#include <array>
#include <bit>
#include <charconv>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <functional>
#include <iterator>
#include <limits>
#include <optional>
#include <string>
#include <string_view>
#include <system_error>
#include <utility>
#include <vector>

#include "gtest/gtest.h"
#include "intrinsic/embodiment/frame_policy.h"
#include "intrinsic/estimation/eskf_propagate.h"
#include "intrinsic/hardware/marine/fake_imu.h"
#include "intrinsic/hardware/marine/imu.pb.h"
#include "intrinsic/hardware/marine/imu_policy.h"
#include "intrinsic/hardware/marine/measurement_health.pb.h"
#include "intrinsic/hardware/marine/measurement_health_policy.h"

namespace intrinsic::simulation {
namespace {

using ::intrinsic::hardware::marine::AssessImu;
using ::intrinsic::hardware::marine::FakeImu;
using ::intrinsic::hardware::marine::FakeImuConfig;
using ::intrinsic::hardware::marine::ImuAssessment;
using ::intrinsic::hardware::marine::ImuError;
using ::intrinsic::hardware::marine::ImuMeasurementView;
using ::intrinsic::hardware::marine::ImuSignedUnitNoise;
using ::intrinsic::hardware::marine::ImuTruth;
using ::intrinsic::hardware::marine::MeasurementStateKind;
using ::intrinsic::hardware::marine::SourceHealthView;
using ::intrinsic_proto::hardware::marine::ImuMeasurement;
using ::intrinsic_proto::hardware::marine::MeasurementHealth;
using Children = std::vector<std::pair<std::string, std::string>>;
using Triple = std::array<double, 3>;
using Sextuple = std::array<double, 6>;
using Quat = std::array<double, 4>;
using StateFn = std::function<ImuSimulatorState(int)>;

constexpr char kGoldenPath[] =
    "intrinsic/simulation/gazebo/plugins/imu/testdata/"
    "imu_adapter_seed_42.golden.txt";
constexpr char kSourcePrefix[] = "intrinsic/simulation/gazebo/plugins/imu/";
constexpr int64_t kStepNanos = 10000000;
constexpr uint64_t kGamma = 0x9E3779B97F4A7C15ULL;
constexpr double kGravity = 9.80665;
constexpr double kSqrtHalf = 0.70710678118654752440;

std::string ReadFile(const std::string& path) {
  std::ifstream file(path, std::ios::binary);
  EXPECT_TRUE(file.is_open()) << path;
  return std::string(std::istreambuf_iterator<char>(file),
                     std::istreambuf_iterator<char>());
}

std::string ToHex(std::string_view bytes) {
  static constexpr char kDigits[] = "0123456789abcdef";
  std::string out(bytes.size() * 2, '\0');
  for (size_t i = 0; i < bytes.size(); ++i) {
    const unsigned char value = static_cast<unsigned char>(bytes[i]);
    out[i * 2] = kDigits[value >> 4];
    out[i * 2 + 1] = kDigits[value & 0x0f];
  }
  return out;
}

std::string IntToString(int64_t value) {
  char buffer[32];
  const std::to_chars_result result =
      std::to_chars(buffer, buffer + sizeof(buffer), value);
  EXPECT_EQ(result.ec, std::errc());
  return std::string(buffer, result.ptr);
}

uint64_t Bits(double value) { return std::bit_cast<uint64_t>(value); }

// ---- SDF construction -----------------------------------------------------

Children DefaultChildren() { return {}; }

Children With(Children children, const std::string& name,
              const std::string& value) {
  for (auto& child : children) {
    if (child.first == name) {
      child.second = value;
      return children;
    }
  }
  children.emplace_back(name, value);
  return children;
}

std::string PluginXml(const Children& children) {
  std::string xml =
      "<plugin filename=\"static://intrinsic::simulation::Imu\" "
      "name=\"intrinsic::simulation::Imu\">";
  for (const auto& [name, value] : children) {
    xml += "<" + name + ">" + value + "</" + name + ">";
  }
  xml += "</plugin>";
  return xml;
}

std::string Window(int start, int end) {
  return "<start_step>" + IntToString(start) + "</start_step><end_step>" +
         IntToString(end) + "</end_step>";
}

// ---- State and result helpers ---------------------------------------------

ImuSimulatorState StateAt(int k, Triple omega_w = {0, 0, 0},
                          Triple accel_w = {0, 0, 0},
                          Quat orientation = {0, 0, 0, 1},
                          int64_t time_offset_steps = 0) {
  ImuSimulatorState state;
  state.link_resolved = true;
  state.kinematics_ready = true;
  state.orientation_xyzw = orientation;
  state.world_angular_velocity_rad_s = omega_w;
  state.world_linear_acceleration_m_s2 = accel_w;
  SplitImuSimTimeNanos(
      (static_cast<int64_t>(k) + time_offset_steps) * kStepNanos,
      &state.time_seconds, &state.time_nanos);
  return state;
}

std::vector<ImuStepResult> RunSteps(const Children& children, int steps,
                                    const StateFn& state_fn) {
  ImuAdapter adapter;
  const ImuSimStatus status =
      adapter.ConfigureFromPluginXml(PluginXml(children));
  EXPECT_TRUE(status.ok()) << status.message;
  std::vector<ImuStepResult> results;
  for (int k = 0; k < steps; ++k) {
    results.push_back(adapter.Step(state_fn(k)));
  }
  return results;
}

std::vector<ImuStepResult> RunSteps(const Children& children, int steps) {
  return RunSteps(children, steps, [](int k) { return StateAt(k); });
}

std::string Bytes(const ImuStepResult& result) {
  EXPECT_TRUE(result.measurement.has_value());
  return result.measurement.has_value()
             ? result.measurement->SerializeAsString()
             : std::string();
}

ImuMeasurementView ViewOf(const ImuMeasurement& sample,
                          std::vector<double>* covariance,
                          std::vector<SourceHealthView>* sources) {
  ImuMeasurementView view;
  const MeasurementHealth& health = sample.health();
  view.health.header_present = health.has_header();
  view.health.frame_id = health.header().frame_id();
  view.health.expected_frame_present = true;
  view.health.expected_frame = "body";
  view.health.source_time_present = health.header().has_source_time();
  if (view.health.source_time_present) {
    view.health.source_time =
        embodiment::ClockReading{health.header().source_time().seconds(),
                                 health.header().source_time().nanos()};
  }
  view.health.receive_time_present = health.header().has_receive_time();
  if (view.health.receive_time_present) {
    view.health.receive_time =
        embodiment::ClockReading{health.header().receive_time().seconds(),
                                 health.header().receive_time().nanos()};
  }
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

// Runs the HAL conformance oracle with expected frame "body".
ImuAssessment Assess(const ImuMeasurement& sample) {
  std::vector<double> covariance;
  std::vector<SourceHealthView> sources;
  return AssessImu(ViewOf(sample, &covariance, &sources));
}

void ExpectOracle(const ImuMeasurement& sample, ImuError error,
                  MeasurementStateKind state, bool accepted) {
  const ImuAssessment assessment = Assess(sample);
  EXPECT_EQ(assessment.error, error);
  EXPECT_EQ(assessment.state, state);
  EXPECT_EQ(assessment.accepted, accepted);
}

void ExpectAccepted(const ImuMeasurement& sample) {
  ExpectOracle(sample, ImuError::kNone, MeasurementStateKind::kValid, true);
  EXPECT_EQ(sample.health().state(), MeasurementHealth::VALID);
}

void ExpectDegradedNotAccepted(const ImuMeasurement& sample) {
  ExpectOracle(sample, ImuError::kNone, MeasurementStateKind::kDegraded, false);
  EXPECT_EQ(sample.health().state(), MeasurementHealth::DEGRADED);
}

Sextuple Values(const ImuMeasurement& sample) {
  return {
      sample.angular_velocity_x_rad_s(),   sample.angular_velocity_y_rad_s(),
      sample.angular_velocity_z_rad_s(),   sample.linear_acceleration_x_m_s2(),
      sample.linear_acceleration_y_m_s2(), sample.linear_acceleration_z_m_s2()};
}

std::array<int64_t, 2> Stamp(
    const intrinsic_proto::embodiment::StampedHeader& header, bool source) {
  const auto& time = source ? header.source_time() : header.receive_time();
  return {time.seconds(), time.nanos()};
}

int64_t TotalNanos(const std::array<int64_t, 2>& stamp) {
  return stamp[0] * 1000000000LL + stamp[1];
}

// The hand-built fake output the adapter must reproduce byte for byte.
ImuMeasurement FakeSample(uint64_t seed, const Sextuple& bias,
                          const Sextuple& truth_values, const Quat& q,
                          int64_t source_nanos, int64_t receive_nanos) {
  FakeImuConfig config;
  config.seed = seed;
  config.angular_velocity_bias_x_rad_s = bias[0];
  config.angular_velocity_bias_y_rad_s = bias[1];
  config.angular_velocity_bias_z_rad_s = bias[2];
  config.linear_acceleration_bias_x_m_s2 = bias[3];
  config.linear_acceleration_bias_y_m_s2 = bias[4];
  config.linear_acceleration_bias_z_m_s2 = bias[5];
  config.delay_seconds = 0;
  config.delay_nanos = 0;
  ImuTruth truth;
  truth.frame_id = "body";
  truth.source_seconds = source_nanos / 1000000000;
  truth.source_nanos = static_cast<int32_t>(source_nanos % 1000000000);
  truth.angular_velocity_x_rad_s = truth_values[0];
  truth.angular_velocity_y_rad_s = truth_values[1];
  truth.angular_velocity_z_rad_s = truth_values[2];
  truth.linear_acceleration_x_m_s2 = truth_values[3];
  truth.linear_acceleration_y_m_s2 = truth_values[4];
  truth.linear_acceleration_z_m_s2 = truth_values[5];
  truth.orientation_present = true;
  truth.orientation_x = q[0];
  truth.orientation_y = q[1];
  truth.orientation_z = q[2];
  truth.orientation_w = q[3];
  truth.quality = 0.75;
  truth.covariance_present = true;
  ImuMeasurement sample = *FakeImu(config).Measure(truth);
  auto* receive =
      sample.mutable_health()->mutable_header()->mutable_receive_time();
  receive->set_seconds(receive_nanos / 1000000000);
  receive->set_nanos(static_cast<int32_t>(receive_nanos % 1000000000));
  return sample;
}

constexpr Sextuple kRestTruth = {0, 0, 0, 0, 0, kGravity};
constexpr Sextuple kNoBias = {0, 0, 0, 0, 0, 0};
constexpr Quat kIdentity = {0, 0, 0, 1};

// ---- Independent noise and drift formula (does not call the adapter) ------

double Unit(uint64_t seed, int k, int i) {
  return ImuSignedUnitNoise(
      seed +
      (8 * static_cast<uint64_t>(k) + static_cast<uint64_t>(i)) * kGamma);
}

// Elapsed simulation time in seconds for step k of a 10 ms run.
double ElapsedSeconds(int64_t steps) {
  return static_cast<double>(steps * kStepNanos) / 1e9;
}

// ---- I1: nominal and fake parity ------------------------------------------

TEST(ImuAdapterTest, NominalAtRestMatchesFakeImuBytes) {
  const std::vector<ImuStepResult> results = RunSteps(DefaultChildren(), 3);
  for (int k = 0; k < 3; ++k) {
    ASSERT_TRUE(results[k].status.ok());
    ASSERT_TRUE(results[k].measurement.has_value());
    const ImuMeasurement& sample = *results[k].measurement;
    EXPECT_EQ(Bytes(results[k]),
              FakeSample(42 + k, kNoBias, kRestTruth, kIdentity, k * kStepNanos,
                         k * kStepNanos)
                  .SerializeAsString())
        << k;
    EXPECT_EQ(Bits(sample.angular_velocity_x_rad_s()), Bits(0.0));
    EXPECT_EQ(Bits(sample.angular_velocity_y_rad_s()), Bits(0.0));
    EXPECT_EQ(Bits(sample.angular_velocity_z_rad_s()), Bits(0.0));
    EXPECT_EQ(Bits(sample.linear_acceleration_x_m_s2()), Bits(0.0));
    EXPECT_EQ(Bits(sample.linear_acceleration_y_m_s2()), Bits(0.0));
    EXPECT_EQ(Bits(sample.linear_acceleration_z_m_s2()), Bits(kGravity));
    EXPECT_EQ(sample.health().header().frame_id(), "body");
    EXPECT_EQ(sample.health().header().sequence(),
              static_cast<uint64_t>(42 + k));
    ASSERT_TRUE(sample.has_orientation_xyzw());
    EXPECT_EQ(Bits(sample.orientation_xyzw().w()), Bits(1.0));
    ExpectAccepted(sample);
  }
}

TEST(ImuAdapterTest, SpecificForceUsesTheApprovedGravityConvention) {
  // The contract constant, and a free-fall sample (a = g) that measures zero.
  EXPECT_EQ(Bits(estimation::kGravityEnu[2]), Bits(-kGravity));
  ImuAdapter adapter;
  ASSERT_TRUE(adapter.ConfigureFromPluginXml(PluginXml({})).ok());
  const ImuStepResult free_fall =
      adapter.Step(StateAt(0, {0, 0, 0}, {0, 0, -kGravity}));
  ASSERT_TRUE(free_fall.measurement.has_value());
  EXPECT_EQ(Bits(free_fall.measurement->linear_acceleration_z_m_s2()),
            Bits(0.0));
  const ImuStepResult accelerating =
      adapter.Step(StateAt(1, {0, 0, 0}, {0, 0, 2.0}));
  EXPECT_EQ(Bits(accelerating.measurement->linear_acceleration_z_m_s2()),
            Bits(2.0 + kGravity));
}

TEST(ImuAdapterTest, YawAndRollRotateAngularVelocityAndSpecificForce) {
  // 90 degrees about +z. Body x points along world +y, body y along -x.
  const Quat yaw = {0, 0, kSqrtHalf, kSqrtHalf};
  // 90 degrees about +x. Body y points along world +z, body z along -y.
  const Quat roll = {kSqrtHalf, 0, 0, kSqrtHalf};
  struct Case {
    Quat q;
    Triple omega_w;
    Triple accel_w;
    Sextuple expected;
  };
  const std::vector<Case> cases = {
      {yaw, {1, 2, 3}, {0, 0, 0}, {2, -1, 3, 0, 0, kGravity}},
      {yaw, {0, 0, 0}, {1, 0, 0}, {0, 0, 0, 0, -1, kGravity}},
      {yaw, {0, 0, 0}, {0, 1, 0}, {0, 0, 0, 1, 0, kGravity}},
      {roll, {1, 2, 3}, {0, 0, 0}, {1, 3, -2, 0, kGravity, 0}},
      {roll, {0, 0, 0}, {0, 0, 1}, {0, 0, 0, 0, kGravity + 1, 0}},
      {roll, {0, 0, 0}, {0, 1, 0}, {0, 0, 0, 0, kGravity, -1}},
  };
  for (const Case& test_case : cases) {
    ImuAdapter adapter;
    ASSERT_TRUE(adapter.ConfigureFromPluginXml(PluginXml({})).ok());
    const ImuStepResult result = adapter.Step(
        StateAt(0, test_case.omega_w, test_case.accel_w, test_case.q));
    ASSERT_TRUE(result.measurement.has_value());
    const Sextuple values = Values(*result.measurement);
    for (int i = 0; i < 6; ++i) {
      EXPECT_NEAR(values[i], test_case.expected[i], 1e-12) << i;
    }
    ExpectAccepted(*result.measurement);
    // Orientation present: the fake writes q, which the adapter never edits.
    EXPECT_EQ(Bits(result.measurement->orientation_xyzw().x()),
              Bits(test_case.q[0]));
    EXPECT_EQ(Bits(result.measurement->orientation_xyzw().z()),
              Bits(test_case.q[2]));
    EXPECT_EQ(Bits(result.measurement->orientation_xyzw().w()),
              Bits(test_case.q[3]));
  }
}

// ---- I2: bias -------------------------------------------------------------

TEST(ImuAdapterTest, BiasIsAddedPerAxisBitExactAndDegrades) {
  const std::vector<std::string> names = {
      "angular_bias_x_rad_s", "angular_bias_y_rad_s", "angular_bias_z_rad_s",
      "linear_bias_x_m_s2",   "linear_bias_y_m_s2",   "linear_bias_z_m_s2"};
  for (int axis = 0; axis < 6; ++axis) {
    for (const double bias_value : {0.2, -0.3}) {
      const std::string text = bias_value > 0 ? "0.2" : "-0.3";
      const std::vector<ImuStepResult> results =
          RunSteps(With(DefaultChildren(), names[axis], text), 4);
      Sextuple bias = kNoBias;
      bias[axis] = bias_value;
      for (int k = 0; k < 4; ++k) {
        ASSERT_TRUE(results[k].measurement.has_value());
        const ImuMeasurement& sample = *results[k].measurement;
        const Sextuple values = Values(sample);
        for (int i = 0; i < 6; ++i) {
          EXPECT_EQ(Bits(values[i]), Bits(kRestTruth[i] + bias[i]))
              << names[axis] << " " << i;
        }
        ExpectDegradedNotAccepted(sample);
        EXPECT_EQ(Bytes(results[k]),
                  FakeSample(42 + k, bias, kRestTruth, kIdentity,
                             k * kStepNanos, k * kStepNanos)
                      .SerializeAsString());
      }
    }
  }
}

// ---- I3: noise ------------------------------------------------------------

TEST(ImuAdapterTest, NoiseMatchesTheContractFormulaBitExact) {
  constexpr double kAngularAmp = 0.01;
  constexpr double kLinearAmp = 0.05;
  const Children children = With(
      With(With(DefaultChildren(), "angular_noise_amplitude_rad_s", "0.01"),
           "linear_noise_amplitude_m_s2", "0.05"),
      "linear_bias_z_m_s2", "0.002");
  const std::vector<ImuStepResult> results = RunSteps(children, 200);
  for (int k = 0; k < 200; ++k) {
    ASSERT_TRUE(results[k].measurement.has_value());
    const Sextuple values = Values(*results[k].measurement);
    Sextuple effective = kNoBias;
    for (int i = 0; i < 6; ++i) {
      const double amp = i < 3 ? kAngularAmp : kLinearAmp;
      const double bias = i == 5 ? 0.002 : 0.0;
      effective[i] = std::fma(amp, Unit(42, k, i), bias);
      EXPECT_EQ(Bits(values[i]), Bits(kRestTruth[i] + effective[i]))
          << k << " " << i;
      EXPECT_LE(std::fabs(effective[i] - bias), amp * (1.0 + 1e-12))
          << k << " " << i;
    }
    ExpectDegradedNotAccepted(*results[k].measurement);
    EXPECT_EQ(Bytes(results[k]),
              FakeSample(42 + k, effective, kRestTruth, kIdentity,
                         k * kStepNanos, k * kStepNanos)
                  .SerializeAsString())
        << k;
  }
}

TEST(ImuAdapterTest, NoiseDrawsDifferPerAxis) {
  const Children children =
      With(With(DefaultChildren(), "angular_noise_amplitude_rad_s", "0.01"),
           "linear_noise_amplitude_m_s2", "0.01");
  const std::vector<ImuStepResult> results = RunSteps(children, 20);
  int distinct_axes = 0;
  for (int k = 0; k < 20; ++k) {
    const Sextuple values = Values(*results[k].measurement);
    if (values[0] != values[1] && values[1] != values[2]) {
      ++distinct_axes;
    }
  }
  EXPECT_GT(distinct_axes, 15);
}

TEST(ImuAdapterTest, NoiseOnlyNeverYieldsAValidSample) {
  const Children children =
      With(With(DefaultChildren(), "angular_noise_amplitude_rad_s", "0.01"),
           "linear_noise_amplitude_m_s2", "0.05");
  const std::vector<ImuStepResult> results = RunSteps(children, 1000);
  for (int k = 0; k < 1000; ++k) {
    ASSERT_TRUE(results[k].measurement.has_value());
    EXPECT_NE(results[k].measurement->health().state(),
              MeasurementHealth::VALID)
        << k;
    ExpectDegradedNotAccepted(*results[k].measurement);
  }
}

TEST(ImuAdapterTest, ZeroEffectiveBiasWithNoiseIsForcedDegraded) {
  // The smallest denormal amplitude underflows fma(amp, u, 0) to exactly 0
  // whenever |u| < 0.5, so the fake alone would return VALID.
  constexpr double kAmp = 5e-324;
  const Children children =
      With(With(DefaultChildren(), "angular_noise_amplitude_rad_s", "5e-324"),
           "linear_noise_amplitude_m_s2", "5e-324");
  const std::vector<ImuStepResult> results = RunSteps(children, 200);
  int all_zero_steps = 0;
  for (int k = 0; k < 200; ++k) {
    ASSERT_TRUE(results[k].measurement.has_value());
    bool all_zero = true;
    for (int i = 0; i < 6; ++i) {
      all_zero = all_zero && std::fma(kAmp, Unit(42, k, i), 0.0) == 0.0;
    }
    if (all_zero) {
      ++all_zero_steps;
      EXPECT_EQ(FakeSample(42 + k, kNoBias, kRestTruth, kIdentity,
                           k * kStepNanos, k * kStepNanos)
                    .health()
                    .state(),
                MeasurementHealth::VALID);
    }
    ExpectDegradedNotAccepted(*results[k].measurement);
  }
  EXPECT_GT(all_zero_steps, 0);
}

// ---- I4: drift ------------------------------------------------------------

TEST(ImuAdapterTest, DriftRampIsBitExactAndResetRestartsIt) {
  const Children children =
      With(With(With(DefaultChildren(), "angular_drift_rad_s2", "1e-3"),
                "linear_drift_m_s3", "2e-3"),
           "angular_bias_x_rad_s", "0.01");
  ImuAdapter adapter;
  ASSERT_TRUE(adapter.ConfigureFromPluginXml(PluginXml(children)).ok());
  constexpr int kSteps = 60;
  std::vector<Sextuple> first;
  for (int k = 0; k < kSteps; ++k) {
    const ImuStepResult result = adapter.Step(StateAt(k));
    ASSERT_TRUE(result.measurement.has_value());
    const Sextuple values = Values(*result.measurement);
    first.push_back(values);
    const double t = ElapsedSeconds(k);
    // omega_x at step k is fma(1e-3, k dt, bias). The truth is exactly 0.
    EXPECT_EQ(Bits(values[0]), Bits(std::fma(1e-3, t, 0.01))) << k;
    EXPECT_EQ(Bits(values[1]), Bits(std::fma(1e-3, t, 0.0))) << k;
    EXPECT_EQ(Bits(values[2]), Bits(std::fma(1e-3, t, 0.0))) << k;
    EXPECT_EQ(Bits(values[3]), Bits(std::fma(2e-3, t, 0.0))) << k;
    EXPECT_EQ(Bits(values[5]), Bits(kGravity + std::fma(2e-3, t, 0.0))) << k;
    // k * 0.01 is the same ramp to within one rounding of the time product.
    EXPECT_NEAR(values[0], std::fma(1e-3, k * 0.01, 0.01), 4e-18) << k;
    // Drift is a bias: always DEGRADED, even at step 0 where it is exactly 0
    // on five axes.
    EXPECT_EQ(result.measurement->health().state(), MeasurementHealth::DEGRADED)
        << k;
    ExpectDegradedNotAccepted(*result.measurement);
  }
  // Reset restarts the ramp even though simulation time keeps counting.
  adapter.Reset();
  for (int k = 0; k < kSteps; ++k) {
    const ImuStepResult result =
        adapter.Step(StateAt(k, {0, 0, 0}, {0, 0, 0}, kIdentity, 1000));
    ASSERT_TRUE(result.measurement.has_value());
    const Sextuple values = Values(*result.measurement);
    EXPECT_EQ(Bits(values[0]), Bits(std::fma(1e-3, ElapsedSeconds(k), 0.01)))
        << k;
    for (int i = 0; i < 6; ++i) {
      EXPECT_EQ(Bits(values[i]), Bits(first[k][i])) << k << " " << i;
    }
  }
}

TEST(ImuAdapterTest, DriftAtStepZeroIsDegradedThroughTheAdapterRule) {
  const Children children =
      With(DefaultChildren(), "angular_drift_rad_s2", "1e-3");
  const std::vector<ImuStepResult> results = RunSteps(children, 3);
  // At step 0 every effective bias is exactly 0, so the fake alone says VALID.
  EXPECT_EQ(
      FakeSample(42, kNoBias, kRestTruth, kIdentity, 0, 0).health().state(),
      MeasurementHealth::VALID);
  for (int k = 0; k < 3; ++k) {
    ASSERT_TRUE(results[k].measurement.has_value());
    ExpectDegradedNotAccepted(*results[k].measurement);
  }
}

TEST(ImuAdapterTest, DriftAndNoiseFoldIntoOneEffectiveBias) {
  const Children children =
      With(With(With(With(DefaultChildren(), "angular_drift_rad_s2", "1e-3"),
                     "angular_noise_amplitude_rad_s", "0.01"),
                "angular_bias_y_rad_s", "0.004"),
           "linear_noise_amplitude_m_s2", "0.02");
  const std::vector<ImuStepResult> results = RunSteps(children, 50);
  for (int k = 0; k < 50; ++k) {
    const Sextuple values = Values(*results[k].measurement);
    const double t = ElapsedSeconds(k);
    for (int i = 0; i < 6; ++i) {
      const double drift_rate = i < 3 ? 1e-3 : 0.0;
      const double amp = i < 3 ? 0.01 : 0.02;
      const double base = i == 1 ? 0.004 : 0.0;
      const double effective =
          std::fma(amp, Unit(42, k, i), std::fma(drift_rate, t, base));
      EXPECT_EQ(Bits(values[i]), Bits(kRestTruth[i] + effective))
          << k << " " << i;
    }
    ExpectDegradedNotAccepted(*results[k].measurement);
  }
}

// ---- I5: latency ----------------------------------------------------------

TEST(ImuAdapterTest, LatencyRampEmitsOriginatedSampleAtLaterReceiveTime) {
  const Children children = With(DefaultChildren(), "latency_steps", "3");
  const std::vector<ImuStepResult> results = RunSteps(children, 12, [](int k) {
    return StateAt(k, {0.01 * k, 0, 0}, {0.1 * k, 0, 0});
  });
  for (int k = 0; k < 12; ++k) {
    ASSERT_TRUE(results[k].status.ok());
    if (k < 3) {
      EXPECT_FALSE(results[k].measurement.has_value()) << k;
      continue;
    }
    ASSERT_TRUE(results[k].measurement.has_value()) << k;
    const ImuMeasurement& sample = *results[k].measurement;
    EXPECT_NEAR(sample.angular_velocity_x_rad_s(), 0.01 * (k - 3), 1e-15) << k;
    EXPECT_NEAR(sample.linear_acceleration_x_m_s2(), 0.1 * (k - 3), 1e-15) << k;
    const auto& header = sample.health().header();
    EXPECT_EQ(TotalNanos(Stamp(header, true)), (k - 3) * kStepNanos);
    EXPECT_EQ(TotalNanos(Stamp(header, false)), k * kStepNanos);
    EXPECT_EQ(header.sequence(), static_cast<uint64_t>(42 + k - 3));
    ExpectAccepted(sample);
  }
}

TEST(ImuAdapterTest, LatencyMatchesFakeWithHandBuiltStamps) {
  const Children children = With(DefaultChildren(), "latency_steps", "2");
  const std::vector<ImuStepResult> results = RunSteps(children, 6);
  for (int k = 2; k < 6; ++k) {
    EXPECT_EQ(Bytes(results[k]),
              FakeSample(42 + (k - 2), kNoBias, kRestTruth, kIdentity,
                         (k - 2) * kStepNanos, k * kStepNanos)
                  .SerializeAsString())
        << k;
  }
}

TEST(ImuAdapterTest, ZeroLatencyReceiveEqualsSource) {
  const std::vector<ImuStepResult> results =
      RunSteps(With(DefaultChildren(), "latency_steps", "0"), 3);
  for (const ImuStepResult& result : results) {
    ASSERT_TRUE(result.measurement.has_value());
    const auto& header = result.measurement->health().header();
    EXPECT_EQ(Stamp(header, true), Stamp(header, false));
  }
}

TEST(ImuAdapterTest, LatencyKeepsReceiveAtOrAfterSourceAcrossSecondBoundary) {
  const Children children = With(DefaultChildren(), "latency_steps", "4");
  ImuAdapter adapter;
  ASSERT_TRUE(adapter.ConfigureFromPluginXml(PluginXml(children)).ok());
  for (int k = 0; k < 10; ++k) {
    ImuSimulatorState state = StateAt(0);
    SplitImuSimTimeNanos(static_cast<int64_t>(k) * 300000000,
                         &state.time_seconds, &state.time_nanos);
    const ImuStepResult result = adapter.Step(state);
    if (!result.measurement.has_value()) {
      continue;
    }
    const auto& header = result.measurement->health().header();
    EXPECT_EQ(
        TotalNanos(Stamp(header, false)) - TotalNanos(Stamp(header, true)),
        4 * 300000000LL);
    ExpectAccepted(*result.measurement);
  }
}

// ---- I6: dropout ----------------------------------------------------------

TEST(ImuAdapterTest, DropoutWithoutLatencyOmitsWindowAndKeepsLaterDraws) {
  const Children noisy =
      With(With(DefaultChildren(), "angular_noise_amplitude_rad_s", "0.01"),
           "linear_noise_amplitude_m_s2", "0.05");
  const Children dropped = With(noisy, "dropout_window", Window(5, 8));
  const std::vector<ImuStepResult> base = RunSteps(noisy, 12);
  const std::vector<ImuStepResult> run = RunSteps(dropped, 12);
  for (int k = 0; k < 12; ++k) {
    ASSERT_TRUE(run[k].status.ok());
    if (k >= 5 && k < 8) {
      EXPECT_FALSE(run[k].measurement.has_value()) << k;
    } else {
      ASSERT_TRUE(run[k].measurement.has_value()) << k;
      EXPECT_EQ(Bytes(run[k]), Bytes(base[k])) << k;
    }
  }
}

TEST(ImuAdapterTest, DropoutWithLatencyShiftsTheGapByTheLatency) {
  const Children noisy = With(
      With(With(DefaultChildren(), "angular_noise_amplitude_rad_s", "0.01"),
           "linear_noise_amplitude_m_s2", "0.05"),
      "latency_steps", "2");
  const Children dropped = With(noisy, "dropout_window", Window(5, 8));
  const std::vector<ImuStepResult> base = RunSteps(noisy, 14);
  const std::vector<ImuStepResult> run = RunSteps(dropped, 14);
  for (int k = 0; k < 14; ++k) {
    const bool absent = k < 2 || (k >= 7 && k < 10);
    ASSERT_EQ(run[k].measurement.has_value(), !absent) << k;
    if (!absent) {
      EXPECT_EQ(Bytes(run[k]), Bytes(base[k])) << k;
    }
  }
}

TEST(ImuAdapterTest, DropoutWindowsAreHalfOpenAndMayBeEmpty) {
  const Children children =
      With(DefaultChildren(), "dropout_window", Window(2, 2));
  for (const ImuStepResult& result : RunSteps(children, 5)) {
    EXPECT_TRUE(result.measurement.has_value());
  }
}

TEST(ImuAdapterTest, MultipleDropoutWindowsAreHonoured) {
  ImuAdapter adapter;
  const std::string xml =
      "<plugin filename=\"static://intrinsic::simulation::Imu\" "
      "name=\"intrinsic::simulation::Imu\">"
      "<dropout_window>" +
      Window(1, 2) + "</dropout_window><dropout_window>" + Window(4, 6) +
      "</dropout_window></plugin>";
  ASSERT_TRUE(adapter.ConfigureFromPluginXml(xml).ok());
  for (int k = 0; k < 8; ++k) {
    const bool absent = k == 1 || k == 4 || k == 5;
    EXPECT_EQ(adapter.Step(StateAt(k)).measurement.has_value(), !absent) << k;
  }
}

TEST(ImuAdapterTest, DriftIsKeyedByElapsedTimeAcrossADropout) {
  const Children children =
      With(With(DefaultChildren(), "angular_drift_rad_s2", "1e-3"),
           "dropout_window", Window(3, 6));
  const std::vector<ImuStepResult> run = RunSteps(children, 10);
  for (int k = 6; k < 10; ++k) {
    ASSERT_TRUE(run[k].measurement.has_value());
    EXPECT_EQ(Bits(run[k].measurement->angular_velocity_x_rad_s()),
              Bits(std::fma(1e-3, ElapsedSeconds(k), 0.0)))
        << k;
  }
}

// ---- I7: seed -------------------------------------------------------------

ImuMeasurement WithoutSequence(ImuMeasurement sample) {
  sample.mutable_health()->mutable_header()->clear_sequence();
  return sample;
}

TEST(ImuAdapterTest, SeedWithoutNoiseOnlyChangesTheSequence) {
  const std::vector<ImuStepResult> a =
      RunSteps(With(DefaultChildren(), "seed", "42"), 10);
  const std::vector<ImuStepResult> b =
      RunSteps(With(DefaultChildren(), "seed", "43"), 10);
  for (int k = 0; k < 10; ++k) {
    ASSERT_TRUE(a[k].measurement.has_value());
    ASSERT_TRUE(b[k].measurement.has_value());
    EXPECT_EQ(a[k].measurement->health().header().sequence(),
              static_cast<uint64_t>(42 + k));
    EXPECT_EQ(b[k].measurement->health().header().sequence(),
              static_cast<uint64_t>(43 + k));
    EXPECT_EQ(WithoutSequence(*a[k].measurement).SerializeAsString(),
              WithoutSequence(*b[k].measurement).SerializeAsString())
        << k;
  }
}

TEST(ImuAdapterTest, SeedWithNoiseChangesTheNoiseAndTheSequence) {
  const Children base =
      With(With(DefaultChildren(), "angular_noise_amplitude_rad_s", "0.01"),
           "linear_noise_amplitude_m_s2", "0.05");
  const std::vector<ImuStepResult> a = RunSteps(With(base, "seed", "42"), 20);
  const std::vector<ImuStepResult> b = RunSteps(With(base, "seed", "43"), 20);
  int differing = 0;
  for (int k = 0; k < 20; ++k) {
    ASSERT_TRUE(a[k].measurement.has_value());
    ASSERT_TRUE(b[k].measurement.has_value());
    if (Values(*a[k].measurement) != Values(*b[k].measurement)) {
      ++differing;
    }
  }
  EXPECT_EQ(differing, 20);
}

TEST(ImuAdapterTest, MaximumSeedWrapsInsteadOfFailing) {
  const std::vector<ImuStepResult> results =
      RunSteps(With(DefaultChildren(), "seed", "18446744073709551615"), 3);
  ASSERT_TRUE(results[2].measurement.has_value());
  EXPECT_EQ(results[0].measurement->health().header().sequence(),
            std::numeric_limits<uint64_t>::max());
  EXPECT_EQ(results[1].measurement->health().header().sequence(), 0u);
  EXPECT_EQ(results[2].measurement->health().header().sequence(), 1u);
}

// ---- I8: config and state robustness --------------------------------------

ImuSimStatus ConfigureWith(const Children& children) {
  ImuAdapter adapter;
  return adapter.ConfigureFromPluginXml(PluginXml(children));
}

TEST(ImuAdapterTest, InvalidConfigIsRejectedAndNeverClamped) {
  std::vector<std::pair<std::string, std::string>> bad = {
      {"angular_noise_amplitude_rad_s", "nan"},
      {"angular_noise_amplitude_rad_s", "inf"},
      {"angular_noise_amplitude_rad_s", "-0.01"},
      {"angular_noise_amplitude_rad_s", "0.05x"},
      {"angular_noise_amplitude_rad_s", ""},
      {"linear_noise_amplitude_m_s2", "nan"},
      {"linear_noise_amplitude_m_s2", "-1"},
      {"linear_noise_amplitude_m_s2", "1,5"},
      {"angular_drift_rad_s2", "nan"},
      {"angular_drift_rad_s2", "inf"},
      {"angular_drift_rad_s2", "1e-3 x"},
      {"linear_drift_m_s3", "-inf"},
      {"linear_drift_m_s3", "1.0x"},
      {"seed", "-1"},
      {"seed", "1.5"},
      {"seed", "abc"},
      {"seed", "18446744073709551616"},
      {"latency_steps", "1001"},
      {"latency_steps", "-1"},
      {"latency_steps", "1.5"},
      {"latency_steps", "2x"},
  };
  for (const char* axis : {"x", "y", "z"}) {
    for (const char* value : {"nan", "inf", "-inf", "1.0x", ""}) {
      bad.emplace_back(std::string("angular_bias_") + axis + "_rad_s", value);
      bad.emplace_back(std::string("linear_bias_") + axis + "_m_s2", value);
    }
  }
  for (const auto& [name, value] : bad) {
    ImuAdapter adapter;
    const ImuSimStatus status = adapter.ConfigureFromPluginXml(
        PluginXml(With(DefaultChildren(), name, value)));
    EXPECT_EQ(status.code, ImuSimErrorCode::kInvalidConfig)
        << name << "=" << value;
    EXPECT_FALSE(adapter.active());
    for (int k = 0; k < 5; ++k) {
      const ImuStepResult result = adapter.Step(StateAt(k));
      EXPECT_EQ(result.status.code, ImuSimErrorCode::kInvalidConfig);
      EXPECT_FALSE(result.measurement.has_value());
    }
    EXPECT_EQ(adapter.StepIndexForTesting(), 0) << name << "=" << value;
  }
}

TEST(ImuAdapterTest, ValidBoundaryConfigIsAccepted) {
  for (const auto& [name, value] :
       std::vector<std::pair<std::string, std::string>>{
           {"latency_steps", "0"},
           {"latency_steps", "1000"},
           {"latency_steps", " 7 "},
           {"angular_noise_amplitude_rad_s", "0"},
           {"linear_noise_amplitude_m_s2", "0"},
           {"angular_drift_rad_s2", "-1e-3"},
           {"linear_drift_m_s3", "0"},
           {"angular_bias_x_rad_s", "-0.5"},
           {"seed", "0"},
           {"made_up", "value"},
           {"link_name", "hull"}}) {
    EXPECT_TRUE(ConfigureWith(With(DefaultChildren(), name, value)).ok())
        << name << "=" << value;
  }
  EXPECT_TRUE(ConfigureWith({}).ok());
}

TEST(ImuAdapterTest, InvalidDropoutWindowsAreRejected) {
  const std::vector<std::string> bad = {
      Window(5, 4),
      Window(-1, 3),
      "<start_step>1</start_step>",
      "<end_step>1</end_step>",
      "<start_step>1.5</start_step><end_step>3</end_step>",
      "<start_step>1</start_step><end_step>3x</end_step>",
      "<start_step>nan</start_step><end_step>3</end_step>",
  };
  for (const std::string& window : bad) {
    ImuAdapter adapter;
    const std::string xml =
        "<plugin filename=\"static://intrinsic::simulation::Imu\" "
        "name=\"intrinsic::simulation::Imu\"><dropout_window>" +
        window + "</dropout_window></plugin>";
    EXPECT_EQ(adapter.ConfigureFromPluginXml(xml).code,
              ImuSimErrorCode::kInvalidConfig)
        << window;
    EXPECT_FALSE(adapter.Step(StateAt(0)).measurement.has_value());
  }
}

TEST(ImuAdapterTest, MoreThanSixtyFourWindowsAreRejected) {
  for (const int count : {64, 65}) {
    std::string xml =
        "<plugin filename=\"static://intrinsic::simulation::Imu\" "
        "name=\"intrinsic::simulation::Imu\">";
    for (int i = 0; i < count; ++i) {
      xml +=
          "<dropout_window>" + Window(2 * i, 2 * i + 1) + "</dropout_window>";
    }
    xml += "</plugin>";
    ImuAdapter adapter;
    const ImuSimStatus status = adapter.ConfigureFromPluginXml(xml);
    if (count == 64) {
      EXPECT_TRUE(status.ok()) << status.message;
    } else {
      EXPECT_EQ(status.code, ImuSimErrorCode::kInvalidConfig);
      EXPECT_FALSE(adapter.Step(StateAt(0)).measurement.has_value());
    }
  }
}

TEST(ImuAdapterTest, UnparseableOrUnconfiguredAdapterFailsClosed) {
  ImuAdapter unconfigured;
  EXPECT_FALSE(unconfigured.active());
  const ImuStepResult result = unconfigured.Step(StateAt(0));
  EXPECT_EQ(result.status.code, ImuSimErrorCode::kInvalidConfig);
  EXPECT_FALSE(result.measurement.has_value());

  ImuAdapter garbage;
  EXPECT_EQ(garbage.ConfigureFromPluginXml("<not-a-plugin/>").code,
            ImuSimErrorCode::kInvalidConfig);
  EXPECT_FALSE(garbage.Step(StateAt(0)).measurement.has_value());
}

TEST(ImuAdapterTest, ReconfigureReplacesAnEarlierBadConfiguration) {
  ImuAdapter adapter;
  ASSERT_FALSE(
      adapter
          .ConfigureFromPluginXml(PluginXml({{"angular_bias_x_rad_s", "nan"}}))
          .ok());
  ASSERT_TRUE(adapter.ConfigureFromPluginXml(PluginXml({})).ok());
  EXPECT_TRUE(adapter.active());
  EXPECT_TRUE(adapter.Step(StateAt(0)).measurement.has_value());
}

std::vector<ImuSimulatorState> BadStates() {
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const double inf = std::numeric_limits<double>::infinity();
  std::vector<ImuSimulatorState> bad;
  ImuSimulatorState state = StateAt(1);
  state.link_resolved = false;
  bad.push_back(state);
  state = StateAt(1);
  state.kinematics_ready = false;
  bad.push_back(state);
  for (const double value : {nan, inf, -inf}) {
    state = StateAt(1);
    state.world_angular_velocity_rad_s[1] = value;
    bad.push_back(state);
    state = StateAt(1);
    state.world_linear_acceleration_m_s2[2] = value;
    bad.push_back(state);
    state = StateAt(1);
    state.orientation_xyzw[1] = value;
    bad.push_back(state);
  }
  state = StateAt(1);
  state.orientation_xyzw = {0, 0, 0, 1.0 + 2e-6};
  bad.push_back(state);
  state = StateAt(1);
  state.orientation_xyzw = {0, 0, 0, 0};
  bad.push_back(state);
  state = StateAt(1);
  state.orientation_xyzw = {
      intrinsic::hardware::marine::kImuCanonicalNonUnitQuaternionX,
      intrinsic::hardware::marine::kImuCanonicalNonUnitQuaternionY,
      intrinsic::hardware::marine::kImuCanonicalNonUnitQuaternionZ,
      intrinsic::hardware::marine::kImuCanonicalNonUnitQuaternionW};
  bad.push_back(state);
  state = StateAt(1);
  state.time_nanos = 1000000000;
  bad.push_back(state);
  state = StateAt(1);
  state.time_nanos = -1;
  bad.push_back(state);
  state = StateAt(1);
  state.time_seconds = -1;
  bad.push_back(state);
  return bad;
}

TEST(ImuAdapterTest, InvalidStateGivesNoSampleAndStillAdvancesTheStep) {
  const std::vector<ImuSimulatorState> bad = BadStates();
  ASSERT_GE(bad.size(), 16u);
  for (size_t index = 0; index < bad.size(); ++index) {
    ImuAdapter adapter;
    ASSERT_TRUE(adapter.ConfigureFromPluginXml(PluginXml({})).ok());
    EXPECT_TRUE(adapter.Step(StateAt(0)).measurement.has_value());
    const ImuStepResult result = adapter.Step(bad[index]);
    EXPECT_EQ(result.status.code, ImuSimErrorCode::kInvalidState) << index;
    EXPECT_FALSE(result.measurement.has_value()) << index;
    EXPECT_EQ(result.step, 1);
    EXPECT_EQ(adapter.StepIndexForTesting(), 2);
    EXPECT_TRUE(adapter.active());
  }
  for (const ImuSimulatorState& bad_state : bad) {
    ImuAdapter adapter;
    ASSERT_TRUE(adapter.ConfigureFromPluginXml(PluginXml({})).ok());
    EXPECT_TRUE(adapter.Step(StateAt(0)).measurement.has_value());
    EXPECT_FALSE(adapter.Step(bad_state).measurement.has_value());
    const ImuStepResult recovered = adapter.Step(StateAt(2));
    EXPECT_TRUE(recovered.status.ok());
    ASSERT_TRUE(recovered.measurement.has_value());
    EXPECT_EQ(recovered.step, 2);
    ExpectAccepted(*recovered.measurement);
  }
}

TEST(ImuAdapterTest, BodyFrameOverflowIsInvalidStateNotAnInfiniteSample) {
  ImuAdapter adapter;
  ASSERT_TRUE(adapter
                  .ConfigureFromPluginXml(
                      PluginXml({{"angular_bias_x_rad_s", "1.7e308"}}))
                  .ok());
  ImuSimulatorState state = StateAt(0);
  EXPECT_TRUE(adapter.Step(state).measurement.has_value());
  state = StateAt(1);
  state.world_angular_velocity_rad_s[0] = 1.7e308;
  const ImuStepResult overflow = adapter.Step(state);
  EXPECT_EQ(overflow.status.code, ImuSimErrorCode::kInvalidState);
  EXPECT_FALSE(overflow.measurement.has_value());
}

TEST(ImuAdapterTest, QuaternionWithinToleranceIsAcceptedAndNeverNormalised) {
  ImuAdapter adapter;
  ASSERT_TRUE(adapter.ConfigureFromPluginXml(PluginXml({})).ok());
  ImuSimulatorState state = StateAt(0);
  state.orientation_xyzw = {0, 0, 0, 1.0 + 5e-7};
  const ImuStepResult result = adapter.Step(state);
  EXPECT_TRUE(result.status.ok());
  ASSERT_TRUE(result.measurement.has_value());
  EXPECT_EQ(Bits(result.measurement->orientation_xyzw().w()), Bits(1.0 + 5e-7));
}

TEST(ImuAdapterTest, SimulationTimeMovingBackwardsIsInvalidState) {
  ImuAdapter adapter;
  ASSERT_TRUE(adapter.ConfigureFromPluginXml(PluginXml({})).ok());
  EXPECT_TRUE(adapter.Step(StateAt(5)).measurement.has_value());
  const ImuStepResult backwards = adapter.Step(StateAt(2));
  EXPECT_EQ(backwards.status.code, ImuSimErrorCode::kInvalidState);
  EXPECT_FALSE(backwards.measurement.has_value());
  EXPECT_TRUE(adapter.Step(StateAt(6)).measurement.has_value());
}

TEST(ImuAdapterTest, NonUnitQuaternionCostsTheDelayedSampleToo) {
  const Children children = With(DefaultChildren(), "latency_steps", "2");
  ImuAdapter adapter;
  ASSERT_TRUE(adapter.ConfigureFromPluginXml(PluginXml(children)).ok());
  std::vector<bool> present;
  for (int k = 0; k < 8; ++k) {
    ImuSimulatorState state = StateAt(k);
    if (k == 3) {
      state.orientation_xyzw = {0, 0, 0, 1.0 + 2e-6};
    }
    present.push_back(adapter.Step(state).measurement.has_value());
  }
  const std::vector<bool> expected = {false, false, true, false,
                                      true,  false, true, true};
  EXPECT_EQ(present, expected);
}

// ---- Determinism, reset ---------------------------------------------------

TEST(ImuAdapterTest, TwoFreshAdaptersAreByteIdenticalOverAThousandSteps) {
  const Children children = With(
      With(
          With(With(DefaultChildren(), "angular_noise_amplitude_rad_s", "0.01"),
               "linear_drift_m_s3", "1e-4"),
          "angular_bias_z_rad_s", "0.01"),
      "latency_steps", "5");
  const StateFn state_fn = [](int k) {
    return StateAt(k, {0.001 * k, 0, 0.1}, {0.01 * k, 0, 0});
  };
  const std::vector<ImuStepResult> first = RunSteps(children, 1000, state_fn);
  const std::vector<ImuStepResult> second = RunSteps(children, 1000, state_fn);
  for (int k = 0; k < 1000; ++k) {
    ASSERT_EQ(first[k].measurement.has_value(),
              second[k].measurement.has_value());
    if (first[k].measurement.has_value()) {
      EXPECT_EQ(Bytes(first[k]), Bytes(second[k])) << k;
    }
  }
}

TEST(ImuAdapterTest, ResetThenRerunReproducesTheSameTrace) {
  const Children children = With(
      With(
          With(With(DefaultChildren(), "angular_noise_amplitude_rad_s", "0.01"),
               "angular_drift_rad_s2", "1e-3"),
          "latency_steps", "3"),
      "dropout_window", Window(4, 6));
  ImuAdapter adapter;
  ASSERT_TRUE(adapter.ConfigureFromPluginXml(PluginXml(children)).ok());
  std::vector<std::optional<std::string>> first;
  for (int k = 0; k < 30; ++k) {
    const ImuStepResult result = adapter.Step(StateAt(k));
    first.push_back(result.measurement.has_value()
                        ? std::optional<std::string>(Bytes(result))
                        : std::nullopt);
  }
  EXPECT_EQ(adapter.StepIndexForTesting(), 30);
  adapter.Reset();
  EXPECT_EQ(adapter.StepIndexForTesting(), 0);
  for (int k = 0; k < 30; ++k) {
    const ImuStepResult result = adapter.Step(StateAt(k));
    EXPECT_EQ(result.step, k);
    if (first[k].has_value()) {
      ASSERT_TRUE(result.measurement.has_value()) << k;
      EXPECT_EQ(Bytes(result), *first[k]) << k;
    } else {
      EXPECT_FALSE(result.measurement.has_value()) << k;
    }
  }
}

TEST(ImuAdapterTest, ResetClearsPendingLatencyQueue) {
  const Children children = With(DefaultChildren(), "latency_steps", "3");
  ImuAdapter adapter;
  ASSERT_TRUE(adapter.ConfigureFromPluginXml(PluginXml(children)).ok());
  for (int k = 0; k < 5; ++k) {
    (void)adapter.Step(StateAt(k));
  }
  adapter.Reset();
  for (int k = 0; k < 3; ++k) {
    EXPECT_FALSE(adapter.Step(StateAt(k)).measurement.has_value()) << k;
  }
  EXPECT_TRUE(adapter.Step(StateAt(3)).measurement.has_value());
}

// ---- I9: golden -----------------------------------------------------------

Children GoldenChildren() {
  Children children = DefaultChildren();
  children = With(children, "angular_bias_x_rad_s", "0.01");
  children = With(children, "linear_bias_z_m_s2", "-0.02");
  children = With(children, "angular_noise_amplitude_rad_s", "0.02");
  children = With(children, "linear_noise_amplitude_m_s2", "0.04");
  children = With(children, "angular_drift_rad_s2", "1e-3");
  children = With(children, "linear_drift_m_s3", "2e-3");
  children = With(children, "latency_steps", "2");
  children = With(children, "dropout_window", Window(5, 7));
  children = With(children, "seed", "42");
  return children;
}

// A rotation of 120 degrees about (1, 1, 1): exactly representable, so the
// trace does not depend on a libm.
ImuSimulatorState GoldenState(int k) {
  return StateAt(k, {0.01 * k, -0.005 * k, 0.1}, {0.02 * k, 0.0, -0.01 * k},
                 {0.5, 0.5, 0.5, 0.5});
}

std::vector<ImuStepResult> GoldenRun() {
  return RunSteps(GoldenChildren(), 20, GoldenState);
}

TEST(ImuAdapterTest, GoldenSeed42Trace) {
  const std::vector<ImuStepResult> results = GoldenRun();
  std::string trace = "imu_sim_trace v1\nseed=42\n";
  for (int k = 0; k < 20; ++k) {
    ASSERT_TRUE(results[k].status.ok());
    trace += "step=" + IntToString(k) + " ";
    trace += results[k].measurement.has_value()
                 ? ToHex(results[k].measurement->SerializeAsString())
                 : std::string("absent");
    trace += "\n";
  }
  const std::string golden = ReadFile(kGoldenPath);
  ASSERT_FALSE(golden.empty());
  EXPECT_EQ(trace, golden);
}

TEST(ImuAdapterTest, GoldenScenarioShapeIsAsContracted) {
  const std::vector<ImuStepResult> results = GoldenRun();
  for (int k = 0; k < 20; ++k) {
    const bool expect_absent = k < 2 || (k >= 7 && k < 9);
    ASSERT_EQ(results[k].measurement.has_value(), !expect_absent) << k;
    if (!expect_absent) {
      ExpectDegradedNotAccepted(*results[k].measurement);
      EXPECT_EQ(results[k].measurement->health().header().sequence(),
                static_cast<uint64_t>(42 + k - 2));
    }
  }
}

// ---- Scope guard ----------------------------------------------------------

TEST(ImuAdapterTest, AdapterHasNoGzIncludeAndPluginIsReadOnly) {
  for (const char* name : {"imu_adapter.h", "imu_adapter.cc"}) {
    const std::string text = ReadFile(std::string(kSourcePrefix) + name);
    ASSERT_FALSE(text.empty()) << name;
    EXPECT_EQ(text.find("#include \"gz/"), std::string::npos) << name;
  }
  for (const char* name :
       {"imu_plugin.h", "imu_plugin.cc", "imu_plugin_register.cc"}) {
    const std::string text = ReadFile(std::string(kSourcePrefix) + name);
    ASSERT_FALSE(text.empty()) << name;
    for (const char* mutator :
         {"AddWorldWrench", "SetGravityEnabled", "SetLinearVelocity",
          "SetAngularVelocity", "SetWorldPoseCmd", "SetComponentData",
          "gz/transport"}) {
      EXPECT_EQ(text.find(mutator), std::string::npos)
          << name << " " << mutator;
    }
  }
}

TEST(ImuAdapterTest, HelperSymbolsDoNotClashWithOtherSensorAdapters) {
  // dvl, depth, and altimeter export SplitSimTimeNanos and the like. The IMU
  // package must only add its own uniquely named helper.
  const std::string text =
      ReadFile(std::string(kSourcePrefix) + "imu_adapter.cc");
  EXPECT_EQ(text.find("SplitSimTimeNanos"), std::string::npos);
  EXPECT_NE(text.find("SplitImuSimTimeNanos"), std::string::npos);
}

}  // namespace
}  // namespace intrinsic::simulation
