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

#include "intrinsic/simulation/gazebo/plugins/surface_fix/surface_fix_adapter.h"

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
#include "intrinsic/hardware/marine/fake_imu.h"
#include "intrinsic/hardware/marine/fake_surface_fix.h"
#include "intrinsic/hardware/marine/measurement_health.pb.h"
#include "intrinsic/hardware/marine/measurement_health_policy.h"
#include "intrinsic/hardware/marine/surface_fix.pb.h"
#include "intrinsic/hardware/marine/surface_fix_policy.h"

namespace intrinsic::simulation {
namespace {

using ::intrinsic::hardware::marine::AssessSurfaceFix;
using ::intrinsic::hardware::marine::FakeSurfaceFix;
using ::intrinsic::hardware::marine::FakeSurfaceFixConfig;
using ::intrinsic::hardware::marine::ImuSignedUnitNoise;
using ::intrinsic::hardware::marine::MeasurementStateKind;
using ::intrinsic::hardware::marine::SourceHealthView;
using ::intrinsic::hardware::marine::SurfaceFixAssessment;
using ::intrinsic::hardware::marine::SurfaceFixError;
using ::intrinsic::hardware::marine::SurfaceFixSourceKind;
using ::intrinsic::hardware::marine::SurfaceFixTruth;
using ::intrinsic::hardware::marine::SurfaceFixView;
using ::intrinsic_proto::hardware::marine::MeasurementHealth;
using FixProto = ::intrinsic_proto::hardware::marine::SurfaceFix;
using Children = std::vector<std::pair<std::string, std::string>>;
using Triple = std::array<double, 3>;
using StateFn = std::function<SurfaceFixSimulatorState(int)>;

constexpr char kGoldenPath[] =
    "intrinsic/simulation/gazebo/plugins/surface_fix/testdata/"
    "surface_fix_adapter_seed_42.golden.txt";
constexpr char kSourcePrefix[] =
    "intrinsic/simulation/gazebo/plugins/surface_fix/";
constexpr int64_t kStepNanos = 10000000;
constexpr uint64_t kGamma = 0x9E3779B97F4A7C15ULL;
constexpr char kExpectedFrame[] = "world_enu";

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

Children DefaultChildren() {
  return {{"surface_z_m", "0"}, {"max_fix_depth_m", "0.5"}};
}

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

Children Without(Children children, const std::string& name) {
  for (auto it = children.begin(); it != children.end(); ++it) {
    if (it->first == name) {
      children.erase(it);
      break;
    }
  }
  return children;
}

std::string PluginXml(const Children& children) {
  std::string xml =
      "<plugin filename=\"static://intrinsic::simulation::SurfaceFix\" "
      "name=\"intrinsic::simulation::SurfaceFix\">";
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

// A link at (x, y, z) in world ENU. The default surface plane is z = 0.
SurfaceFixSimulatorState StateAt(int k, Triple position = {12.5, -3.25, 0.0}) {
  SurfaceFixSimulatorState state;
  state.link_resolved = true;
  state.pose_ready = true;
  state.position_m = position;
  SplitSurfaceFixSimTimeNanos(static_cast<int64_t>(k) * kStepNanos,
                              &state.time_seconds, &state.time_nanos);
  return state;
}

std::vector<SurfaceFixStepResult> RunSteps(const Children& children, int steps,
                                           const StateFn& state_fn) {
  SurfaceFixAdapter adapter;
  const SurfaceFixSimStatus status =
      adapter.ConfigureFromPluginXml(PluginXml(children));
  EXPECT_TRUE(status.ok()) << status.message;
  std::vector<SurfaceFixStepResult> results;
  for (int k = 0; k < steps; ++k) {
    results.push_back(adapter.Step(state_fn(k)));
  }
  return results;
}

std::vector<SurfaceFixStepResult> RunSteps(const Children& children,
                                           int steps) {
  return RunSteps(children, steps, [](int k) { return StateAt(k); });
}

std::string Bytes(const SurfaceFixStepResult& result) {
  EXPECT_TRUE(result.measurement.has_value());
  return result.measurement.has_value()
             ? result.measurement->SerializeAsString()
             : std::string();
}

SurfaceFixView ViewOf(const FixProto& sample, std::vector<double>* covariance,
                      std::vector<SourceHealthView>* sources) {
  SurfaceFixView view;
  const MeasurementHealth& health = sample.health();
  view.health.header_present = health.has_header();
  view.health.frame_id = health.header().frame_id();
  view.health.expected_frame_present = true;
  view.health.expected_frame = kExpectedFrame;
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
  view.position_x_present = sample.has_position_x_m();
  view.position_x_m = sample.position_x_m();
  view.position_y_present = sample.has_position_y_m();
  view.position_y_m = sample.position_y_m();
  view.position_z_present = sample.has_position_z_m();
  view.position_z_m = sample.position_z_m();
  view.source_present = sample.has_source();
  view.source = static_cast<int>(sample.source());
  view.satellite_count_present = sample.has_satellite_count();
  view.satellite_count = sample.satellite_count();
  view.beacon_count_present = sample.has_beacon_count();
  view.beacon_count = sample.beacon_count();
  view.horizontal_accuracy_present = sample.has_horizontal_accuracy_m();
  view.horizontal_accuracy_m = sample.horizontal_accuracy_m();
  view.vertical_accuracy_present = sample.has_vertical_accuracy_m();
  view.vertical_accuracy_m = sample.vertical_accuracy_m();
  view.velocity_x_present = sample.has_velocity_x_m_s();
  view.velocity_x_m_s = sample.velocity_x_m_s();
  view.velocity_y_present = sample.has_velocity_y_m_s();
  view.velocity_y_m_s = sample.velocity_y_m_s();
  view.velocity_z_present = sample.has_velocity_z_m_s();
  view.velocity_z_m_s = sample.velocity_z_m_s();
  return view;
}

// Runs the HAL conformance oracle with expected frame "world_enu".
SurfaceFixAssessment Assess(const FixProto& sample) {
  std::vector<double> covariance;
  std::vector<SourceHealthView> sources;
  return AssessSurfaceFix(ViewOf(sample, &covariance, &sources));
}

void ExpectOracle(const FixProto& sample, SurfaceFixError error,
                  MeasurementStateKind state, bool accepted) {
  const SurfaceFixAssessment assessment = Assess(sample);
  EXPECT_EQ(assessment.error, error);
  EXPECT_EQ(assessment.state, state);
  EXPECT_EQ(assessment.accepted, accepted);
}

void ExpectAccepted(const FixProto& sample) {
  ExpectOracle(sample, SurfaceFixError::kNone, MeasurementStateKind::kValid,
               true);
  EXPECT_EQ(sample.health().state(), MeasurementHealth::VALID);
  EXPECT_EQ(Assess(sample).source, SurfaceFixSourceKind::kGnss);
}

void ExpectDegradedNotAccepted(const FixProto& sample) {
  ExpectOracle(sample, SurfaceFixError::kNone, MeasurementStateKind::kDegraded,
               false);
  EXPECT_EQ(sample.health().state(), MeasurementHealth::DEGRADED);
}

Triple Position(const FixProto& sample) {
  return {sample.position_x_m(), sample.position_y_m(), sample.position_z_m()};
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
FixProto FakeSample(uint64_t seed, const Triple& bias, const Triple& position,
                    int64_t source_nanos, int64_t receive_nanos) {
  FakeSurfaceFixConfig config;
  config.seed = seed;
  config.position_bias_x_m = bias[0];
  config.position_bias_y_m = bias[1];
  config.position_bias_z_m = bias[2];
  config.delay_seconds = 0;
  config.delay_nanos = 0;
  SurfaceFixTruth truth;
  truth.frame_id = kExpectedFrame;
  truth.source_seconds = source_nanos / 1000000000;
  truth.source_nanos = static_cast<int32_t>(source_nanos % 1000000000);
  truth.position_x_m = position[0];
  truth.position_y_m = position[1];
  truth.position_z_m = position[2];
  truth.source_present = true;
  truth.source = 1;
  truth.satellite_count_present = true;
  truth.satellite_count = 12;
  truth.quality = 0.75;
  truth.covariance_present = true;
  FixProto sample = *FakeSurfaceFix(config).Measure(truth);
  auto* receive =
      sample.mutable_health()->mutable_header()->mutable_receive_time();
  receive->set_seconds(receive_nanos / 1000000000);
  receive->set_nanos(static_cast<int32_t>(receive_nanos % 1000000000));
  return sample;
}

constexpr Triple kSurfacedPosition = {12.5, -3.25, 0.0};
constexpr Triple kNoBias = {0, 0, 0};

// ---- Independent noise formula (does not call the adapter) ----------------

double Unit(uint64_t seed, int k, int i) {
  return ImuSignedUnitNoise(
      seed +
      (4 * static_cast<uint64_t>(k) + static_cast<uint64_t>(i)) * kGamma);
}

// ---- S1: nominal and fake parity ------------------------------------------

TEST(SurfaceFixAdapterTest, NominalSurfacedMatchesFakeBytesAndIsAccepted) {
  const std::vector<SurfaceFixStepResult> results =
      RunSteps(DefaultChildren(), 3);
  for (int k = 0; k < 3; ++k) {
    ASSERT_TRUE(results[k].status.ok());
    ASSERT_TRUE(results[k].measurement.has_value()) << k;
    const FixProto& sample = *results[k].measurement;
    EXPECT_EQ(Bytes(results[k]), FakeSample(42 + k, kNoBias, kSurfacedPosition,
                                            k * kStepNanos, k * kStepNanos)
                                     .SerializeAsString())
        << k;
    EXPECT_EQ(
        sample.health().header().frame_id(),
        std::string(embodiment::WorldFrameId(embodiment::WorldFrame::kEnu)));
    EXPECT_EQ(sample.health().header().frame_id(), kExpectedFrame);
    EXPECT_EQ(sample.health().header().sequence(),
              static_cast<uint64_t>(42 + k));
    EXPECT_EQ(sample.source(), FixProto::FIX_SOURCE_GNSS);
    EXPECT_EQ(sample.satellite_count(), 12);
    EXPECT_FALSE(sample.has_beacon_count());
    EXPECT_FALSE(sample.has_horizontal_accuracy_m());
    EXPECT_FALSE(sample.has_vertical_accuracy_m());
    EXPECT_FALSE(sample.has_velocity_x_m_s());
    EXPECT_EQ(Bits(sample.position_x_m()), Bits(12.5));
    EXPECT_EQ(Bits(sample.position_y_m()), Bits(-3.25));
    EXPECT_EQ(Bits(sample.position_z_m()), Bits(0.0));
    ExpectAccepted(sample);
  }
}

TEST(SurfaceFixAdapterTest, PositionIsTheLinkOriginInWorldEnuNotConverted) {
  const std::vector<SurfaceFixStepResult> results =
      RunSteps(DefaultChildren(), 1,
               [](int k) { return StateAt(k, {1234.5, -987.25, -0.25}); });
  ASSERT_TRUE(results[0].measurement.has_value());
  EXPECT_EQ(Position(*results[0].measurement),
            (Triple{1234.5, -987.25, -0.25}));
  ExpectAccepted(*results[0].measurement);
}

TEST(SurfaceFixAdapterTest, SourceTimeIsSimulationTimeAndReceiveEqualsIt) {
  const std::vector<SurfaceFixStepResult> results =
      RunSteps(DefaultChildren(), 150);
  for (int k = 0; k < 150; ++k) {
    ASSERT_TRUE(results[k].measurement.has_value());
    const auto& header = results[k].measurement->health().header();
    EXPECT_EQ(TotalNanos(Stamp(header, true)), k * kStepNanos) << k;
    EXPECT_EQ(Stamp(header, true), Stamp(header, false)) << k;
  }
}

// ---- S2: bias -------------------------------------------------------------

TEST(SurfaceFixAdapterTest, BiasIsAddedPerAxisBitExactAndDegrades) {
  const std::vector<std::string> names = {
      "position_bias_x_m", "position_bias_y_m", "position_bias_z_m"};
  for (int axis = 0; axis < 3; ++axis) {
    for (const double bias_value : {0.2, -0.3}) {
      const std::string text = bias_value > 0 ? "0.2" : "-0.3";
      const std::vector<SurfaceFixStepResult> results =
          RunSteps(With(DefaultChildren(), names[axis], text), 4);
      Triple bias = kNoBias;
      bias[axis] = bias_value;
      for (int k = 0; k < 4; ++k) {
        ASSERT_TRUE(results[k].measurement.has_value());
        const FixProto& sample = *results[k].measurement;
        const Triple position = Position(sample);
        for (int i = 0; i < 3; ++i) {
          EXPECT_EQ(Bits(position[i]), Bits(kSurfacedPosition[i] + bias[i]))
              << names[axis] << " " << i;
        }
        ExpectDegradedNotAccepted(sample);
        EXPECT_EQ(Bytes(results[k]), FakeSample(42 + k, bias, kSurfacedPosition,
                                                k * kStepNanos, k * kStepNanos)
                                         .SerializeAsString());
      }
    }
  }
}

// ---- S3: noise ------------------------------------------------------------

TEST(SurfaceFixAdapterTest, NoiseMatchesTheContractFormulaBitExact) {
  constexpr double kAmp = 0.5;
  const Children children =
      With(With(DefaultChildren(), "noise_amplitude_m", "0.5"),
           "position_bias_z_m", "0.002");
  const std::vector<SurfaceFixStepResult> results = RunSteps(children, 200);
  for (int k = 0; k < 200; ++k) {
    ASSERT_TRUE(results[k].measurement.has_value());
    const Triple position = Position(*results[k].measurement);
    Triple effective = kNoBias;
    for (int i = 0; i < 3; ++i) {
      const double bias = i == 2 ? 0.002 : 0.0;
      effective[i] = std::fma(kAmp, Unit(42, k, i), bias);
      EXPECT_EQ(Bits(position[i]), Bits(kSurfacedPosition[i] + effective[i]))
          << k << " " << i;
      EXPECT_LE(std::fabs(effective[i] - bias), kAmp * (1.0 + 1e-12))
          << k << " " << i;
    }
    ExpectDegradedNotAccepted(*results[k].measurement);
    EXPECT_EQ(Bytes(results[k]),
              FakeSample(42 + k, effective, kSurfacedPosition, k * kStepNanos,
                         k * kStepNanos)
                  .SerializeAsString())
        << k;
  }
}

TEST(SurfaceFixAdapterTest, NoiseOnlyNeverYieldsAValidSample) {
  const std::vector<SurfaceFixStepResult> results =
      RunSteps(With(DefaultChildren(), "noise_amplitude_m", "0.5"), 1000);
  for (int k = 0; k < 1000; ++k) {
    ASSERT_TRUE(results[k].measurement.has_value());
    ExpectDegradedNotAccepted(*results[k].measurement);
  }
}

TEST(SurfaceFixAdapterTest, ZeroEffectiveBiasWithNoiseIsForcedDegraded) {
  // The smallest denormal amplitude underflows fma(amp, u, 0) to exactly 0
  // whenever |u| < 0.5, so the fake alone would return VALID.
  constexpr double kAmp = 5e-324;
  const std::vector<SurfaceFixStepResult> results =
      RunSteps(With(DefaultChildren(), "noise_amplitude_m", "5e-324"), 200);
  int all_zero_steps = 0;
  for (int k = 0; k < 200; ++k) {
    ASSERT_TRUE(results[k].measurement.has_value());
    bool all_zero = true;
    for (int i = 0; i < 3; ++i) {
      all_zero = all_zero && std::fma(kAmp, Unit(42, k, i), 0.0) == 0.0;
    }
    if (all_zero) {
      ++all_zero_steps;
      EXPECT_EQ(FakeSample(42 + k, kNoBias, kSurfacedPosition, k * kStepNanos,
                           k * kStepNanos)
                    .health()
                    .state(),
                MeasurementHealth::VALID);
    }
    ExpectDegradedNotAccepted(*results[k].measurement);
  }
  EXPECT_GT(all_zero_steps, 0);
}

TEST(SurfaceFixAdapterTest, NoiseDrawsDifferPerAxis) {
  const std::vector<SurfaceFixStepResult> results =
      RunSteps(With(DefaultChildren(), "noise_amplitude_m", "0.5"), 20);
  int distinct_axes = 0;
  for (int k = 0; k < 20; ++k) {
    const Triple position = Position(*results[k].measurement);
    if (position[0] != position[1] && position[1] != position[2]) {
      ++distinct_axes;
    }
  }
  EXPECT_GT(distinct_axes, 15);
}

// ---- S4: surface gate -----------------------------------------------------

TEST(SurfaceFixAdapterTest, GateEmitsUpToTheLimitAndNotDeeper) {
  // surface_z_m = 0 and max_fix_depth_m = 0.5. depth = surface_z - z.
  const std::vector<std::pair<double, bool>> cases = {
      {0.0, true},
      {0.25, true},
      {-0.25, true},
      {-0.5, true},
      {-0.4999999999999999, true},
      {-0.5000000000000001, false},
      {-0.5000001, false},
      {-3.0, false}};
  for (const auto& [z, expect_fix] : cases) {
    const std::vector<SurfaceFixStepResult> results =
        RunSteps(DefaultChildren(), 1,
                 [z](int k) { return StateAt(k, {12.5, -3.25, z}); });
    ASSERT_TRUE(results[0].status.ok()) << z;
    EXPECT_EQ(results[0].measurement.has_value(), expect_fix) << z;
  }
}

TEST(SurfaceFixAdapterTest, GateUsesTheConfiguredSurfacePlaneAndLimit) {
  const Children children = With(With(DefaultChildren(), "surface_z_m", "2.0"),
                                 "max_fix_depth_m", "0.1");
  // depth = 2.0 - z.
  const std::vector<std::pair<double, bool>> cases = {
      {2.0, true}, {2.5, true}, {1.95, true}, {1.8999, false}, {0.0, false}};
  for (const auto& [z, expect_fix] : cases) {
    const std::vector<SurfaceFixStepResult> results =
        RunSteps(children, 1, [z](int k) { return StateAt(k, {0, 0, z}); });
    EXPECT_EQ(results[0].measurement.has_value(), expect_fix) << z;
  }
}

TEST(SurfaceFixAdapterTest, ZeroMaxDepthEmitsOnlyAtOrAboveTheSurface) {
  const Children children = With(DefaultChildren(), "max_fix_depth_m", "0");
  const std::vector<double> zs = {1.0, 0.0, -1e-12};
  const std::vector<bool> expected = {true, true, false};
  for (size_t i = 0; i < zs.size(); ++i) {
    const double z = zs[i];
    const std::vector<SurfaceFixStepResult> results =
        RunSteps(children, 1, [z](int k) { return StateAt(k, {0, 0, z}); });
    EXPECT_EQ(results[0].measurement.has_value(), expected[i]) << z;
  }
}

TEST(SurfaceFixAdapterTest, GateHasNoHysteresisAndNeverShiftsNoiseDraws) {
  const Children children = With(DefaultChildren(), "noise_amplitude_m", "0.5");
  // Alternates across the gate every step.
  const StateFn alternating = [](int k) {
    return StateAt(k, {12.5, -3.25, k % 2 == 0 ? 0.0 : -2.0});
  };
  const std::vector<SurfaceFixStepResult> gated =
      RunSteps(children, 40, alternating);
  const std::vector<SurfaceFixStepResult> open = RunSteps(children, 40);
  for (int k = 0; k < 40; ++k) {
    ASSERT_TRUE(gated[k].status.ok());
    EXPECT_EQ(gated[k].measurement.has_value(), k % 2 == 0) << k;
    if (k % 2 == 0) {
      EXPECT_EQ(Bytes(gated[k]), Bytes(open[k])) << k;
    }
  }
}

TEST(SurfaceFixAdapterTest, GateIsNotAnErrorAndLinkStaysActive) {
  SurfaceFixAdapter adapter;
  ASSERT_TRUE(
      adapter.ConfigureFromPluginXml(PluginXml(DefaultChildren())).ok());
  const SurfaceFixStepResult deep = adapter.Step(StateAt(0, {0, 0, -10}));
  EXPECT_TRUE(deep.status.ok());
  EXPECT_FALSE(deep.measurement.has_value());
  EXPECT_TRUE(adapter.active());
  const SurfaceFixStepResult surfaced = adapter.Step(StateAt(1, {0, 0, 0.1}));
  ASSERT_TRUE(surfaced.measurement.has_value());
  EXPECT_EQ(surfaced.step, 1);
  ExpectAccepted(*surfaced.measurement);
}

TEST(SurfaceFixAdapterTest, GateComposesWithLatencyAndDropout) {
  const Children children = With(With(DefaultChildren(), "latency_steps", "2"),
                                 "dropout_window", Window(6, 7));
  // Too deep at steps 3 and 4. Dropout at step 6.
  const StateFn state_fn = [](int k) {
    return StateAt(k, {1.0 * k, 0, (k == 3 || k == 4) ? -5.0 : 0.0});
  };
  const std::vector<SurfaceFixStepResult> results =
      RunSteps(children, 12, state_fn);
  for (int k = 0; k < 12; ++k) {
    const int origin = k - 2;
    const bool expect =
        origin >= 0 && origin != 3 && origin != 4 && origin != 6;
    ASSERT_EQ(results[k].measurement.has_value(), expect) << k;
    if (expect) {
      EXPECT_EQ(Bits(results[k].measurement->position_x_m()),
                Bits(1.0 * origin))
          << k;
      EXPECT_EQ(
          TotalNanos(Stamp(results[k].measurement->health().header(), false)),
          k * kStepNanos);
    }
  }
}

// ---- S5: latency ----------------------------------------------------------

TEST(SurfaceFixAdapterTest, LatencyEmitsOriginatedSampleAtLaterReceiveTime) {
  const Children children = With(DefaultChildren(), "latency_steps", "3");
  const std::vector<SurfaceFixStepResult> results =
      RunSteps(children, 12, [](int k) { return StateAt(k, {0.5 * k, 0, 0}); });
  for (int k = 0; k < 12; ++k) {
    ASSERT_TRUE(results[k].status.ok());
    if (k < 3) {
      EXPECT_FALSE(results[k].measurement.has_value()) << k;
      continue;
    }
    ASSERT_TRUE(results[k].measurement.has_value()) << k;
    const FixProto& sample = *results[k].measurement;
    EXPECT_NEAR(sample.position_x_m(), 0.5 * (k - 3), 1e-15) << k;
    const auto& header = sample.health().header();
    EXPECT_EQ(TotalNanos(Stamp(header, true)), (k - 3) * kStepNanos);
    EXPECT_EQ(TotalNanos(Stamp(header, false)), k * kStepNanos);
    EXPECT_EQ(header.sequence(), static_cast<uint64_t>(42 + k - 3));
    ExpectAccepted(sample);
  }
}

TEST(SurfaceFixAdapterTest, LatencyMatchesFakeWithHandBuiltStamps) {
  const Children children = With(DefaultChildren(), "latency_steps", "2");
  const std::vector<SurfaceFixStepResult> results = RunSteps(children, 6);
  for (int k = 2; k < 6; ++k) {
    EXPECT_EQ(Bytes(results[k]),
              FakeSample(42 + (k - 2), kNoBias, kSurfacedPosition,
                         (k - 2) * kStepNanos, k * kStepNanos)
                  .SerializeAsString())
        << k;
  }
}

TEST(SurfaceFixAdapterTest, LatencyKeepsReceiveAtOrAfterSourceAcrossSecond) {
  const Children children = With(DefaultChildren(), "latency_steps", "4");
  SurfaceFixAdapter adapter;
  ASSERT_TRUE(adapter.ConfigureFromPluginXml(PluginXml(children)).ok());
  for (int k = 0; k < 10; ++k) {
    SurfaceFixSimulatorState state = StateAt(0);
    SplitSurfaceFixSimTimeNanos(static_cast<int64_t>(k) * 300000000,
                                &state.time_seconds, &state.time_nanos);
    const SurfaceFixStepResult result = adapter.Step(state);
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

// ---- S6: dropout ----------------------------------------------------------

TEST(SurfaceFixAdapterTest, DropoutOmitsWindowAndKeepsLaterDraws) {
  const Children noisy = With(DefaultChildren(), "noise_amplitude_m", "0.5");
  const Children dropped = With(noisy, "dropout_window", Window(5, 8));
  const std::vector<SurfaceFixStepResult> base = RunSteps(noisy, 12);
  const std::vector<SurfaceFixStepResult> run = RunSteps(dropped, 12);
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

TEST(SurfaceFixAdapterTest, DropoutWindowIsHalfOpenAndMultipleWindowsCombine) {
  const Children children =
      With(DefaultChildren(), "dropout_window", Window(1, 2));
  std::string xml = PluginXml(children);
  xml.insert(xml.rfind("</plugin>"),
             "<dropout_window>" + Window(4, 6) + "</dropout_window>");
  SurfaceFixAdapter adapter;
  ASSERT_TRUE(adapter.ConfigureFromPluginXml(xml).ok());
  std::vector<bool> present;
  for (int k = 0; k < 8; ++k) {
    present.push_back(adapter.Step(StateAt(k)).measurement.has_value());
  }
  const std::vector<bool> expected = {true,  false, true, true,
                                      false, false, true, true};
  EXPECT_EQ(present, expected);
}

TEST(SurfaceFixAdapterTest, EmptyWindowDropsNothing) {
  const std::vector<SurfaceFixStepResult> results =
      RunSteps(With(DefaultChildren(), "dropout_window", Window(3, 3)), 6);
  for (const SurfaceFixStepResult& result : results) {
    EXPECT_TRUE(result.measurement.has_value());
  }
}

TEST(SurfaceFixAdapterTest, DropoutIsKeyedOnOriginationStepNotEmissionStep) {
  const Children children = With(With(DefaultChildren(), "latency_steps", "3"),
                                 "dropout_window", Window(4, 5));
  const std::vector<SurfaceFixStepResult> results = RunSteps(children, 10);
  for (int k = 0; k < 10; ++k) {
    const int origin = k - 3;
    EXPECT_EQ(results[k].measurement.has_value(), origin >= 0 && origin != 4)
        << k;
  }
}

// ---- S7: seed -------------------------------------------------------------

TEST(SurfaceFixAdapterTest, SeedChangesNoiseAndSequenceDefaultIs42) {
  const Children noisy = With(DefaultChildren(), "noise_amplitude_m", "0.5");
  const std::vector<SurfaceFixStepResult> a = RunSteps(noisy, 10);
  const std::vector<SurfaceFixStepResult> b =
      RunSteps(With(noisy, "seed", "7"), 10);
  const std::vector<SurfaceFixStepResult> same =
      RunSteps(With(noisy, "seed", "42"), 10);
  for (int k = 0; k < 10; ++k) {
    EXPECT_NE(Bytes(a[k]), Bytes(b[k])) << k;
    EXPECT_EQ(Bytes(a[k]), Bytes(same[k])) << k;
    EXPECT_EQ(a[k].measurement->health().header().sequence(),
              static_cast<uint64_t>(42 + k));
    EXPECT_EQ(b[k].measurement->health().header().sequence(),
              static_cast<uint64_t>(7 + k));
    const double expected = std::fma(0.5, Unit(7, k, 0), 0.0);
    EXPECT_EQ(Bits(b[k].measurement->position_x_m()),
              Bits(kSurfacedPosition[0] + expected));
  }
}

TEST(SurfaceFixAdapterTest, MaxSeedWrapsWithoutFailing) {
  const std::vector<SurfaceFixStepResult> results =
      RunSteps(With(DefaultChildren(), "seed", "18446744073709551615"), 3);
  for (const SurfaceFixStepResult& result : results) {
    EXPECT_TRUE(result.measurement.has_value());
  }
}

// ---- S8: configuration and robustness ------------------------------------

TEST(SurfaceFixAdapterTest, SurfaceAndMaxDepthAreRequired) {
  for (const char* name : {"surface_z_m", "max_fix_depth_m"}) {
    SurfaceFixAdapter adapter;
    const SurfaceFixSimStatus status = adapter.ConfigureFromPluginXml(
        PluginXml(Without(DefaultChildren(), name)));
    EXPECT_EQ(status.code, SurfaceFixSimErrorCode::kInvalidConfig) << name;
    EXPECT_FALSE(adapter.active());
    const SurfaceFixStepResult result = adapter.Step(StateAt(0));
    EXPECT_EQ(result.status.code, SurfaceFixSimErrorCode::kInvalidConfig);
    EXPECT_FALSE(result.measurement.has_value());
  }
}

TEST(SurfaceFixAdapterTest, InvalidConfigurationFailsClosed) {
  const std::vector<std::pair<std::string, std::string>> bad = {
      {"surface_z_m", "nan"},
      {"surface_z_m", "inf"},
      {"surface_z_m", ""},
      {"surface_z_m", "abc"},
      {"max_fix_depth_m", "-0.1"},
      {"max_fix_depth_m", "nan"},
      {"max_fix_depth_m", "inf"},
      {"max_fix_depth_m", "1x"},
      {"position_bias_x_m", "nan"},
      {"position_bias_y_m", "inf"},
      {"position_bias_z_m", "-inf"},
      {"position_bias_z_m", "abc"},
      {"noise_amplitude_m", "-1"},
      {"noise_amplitude_m", "nan"},
      {"noise_amplitude_m", "inf"},
      {"latency_steps", "-1"},
      {"latency_steps", "1001"},
      {"latency_steps", "1.5"},
      {"latency_steps", "abc"},
      {"seed", "-1"},
      {"seed", "1.5"},
      {"seed", "18446744073709551616"},
  };
  for (const auto& [name, value] : bad) {
    SurfaceFixAdapter adapter;
    const SurfaceFixSimStatus status = adapter.ConfigureFromPluginXml(
        PluginXml(With(DefaultChildren(), name, value)));
    EXPECT_EQ(status.code, SurfaceFixSimErrorCode::kInvalidConfig)
        << name << "=" << value;
    EXPECT_FALSE(adapter.Step(StateAt(0)).measurement.has_value())
        << name << "=" << value;
  }
}

TEST(SurfaceFixAdapterTest, BoundaryConfigurationIsAccepted) {
  for (const auto& [name, value] :
       std::vector<std::pair<std::string, std::string>>{
           {"latency_steps", "0"},
           {"latency_steps", "1000"},
           {"max_fix_depth_m", "0"},
           {"noise_amplitude_m", "0"},
           {"surface_z_m", "-1e6"},
           {"position_bias_x_m", "-0.0"}}) {
    SurfaceFixAdapter adapter;
    const SurfaceFixSimStatus status = adapter.ConfigureFromPluginXml(
        PluginXml(With(DefaultChildren(), name, value)));
    EXPECT_TRUE(status.ok()) << name << "=" << value << " " << status.message;
  }
}

TEST(SurfaceFixAdapterTest, BadDropoutWindowsAreRejected) {
  for (const std::string& window :
       {Window(5, 4), Window(-1, 3), std::string("<start_step>1</start_step>"),
        std::string("<start_step>x</start_step><end_step>2</end_step>")}) {
    SurfaceFixAdapter adapter;
    const SurfaceFixSimStatus status = adapter.ConfigureFromPluginXml(
        PluginXml(With(DefaultChildren(), "dropout_window", window)));
    EXPECT_EQ(status.code, SurfaceFixSimErrorCode::kInvalidConfig) << window;
    EXPECT_FALSE(adapter.Step(StateAt(0)).measurement.has_value());
  }
}

TEST(SurfaceFixAdapterTest, MoreThanSixtyFourWindowsAreRejected) {
  for (const int count : {64, 65}) {
    std::string xml = PluginXml(DefaultChildren());
    std::string windows;
    for (int i = 0; i < count; ++i) {
      windows +=
          "<dropout_window>" + Window(2 * i, 2 * i + 1) + "</dropout_window>";
    }
    xml.insert(xml.rfind("</plugin>"), windows);
    SurfaceFixAdapter adapter;
    const SurfaceFixSimStatus status = adapter.ConfigureFromPluginXml(xml);
    if (count == 64) {
      EXPECT_TRUE(status.ok()) << status.message;
    } else {
      EXPECT_EQ(status.code, SurfaceFixSimErrorCode::kInvalidConfig);
      EXPECT_FALSE(adapter.Step(StateAt(0)).measurement.has_value());
    }
  }
}

TEST(SurfaceFixAdapterTest, UnparseableOrUnconfiguredAdapterFailsClosed) {
  SurfaceFixAdapter unconfigured;
  EXPECT_FALSE(unconfigured.active());
  const SurfaceFixStepResult result = unconfigured.Step(StateAt(0));
  EXPECT_EQ(result.status.code, SurfaceFixSimErrorCode::kInvalidConfig);
  EXPECT_FALSE(result.measurement.has_value());

  SurfaceFixAdapter garbage;
  EXPECT_EQ(garbage.ConfigureFromPluginXml("<not-a-plugin/>").code,
            SurfaceFixSimErrorCode::kInvalidConfig);
  EXPECT_FALSE(garbage.Step(StateAt(0)).measurement.has_value());
}

TEST(SurfaceFixAdapterTest, ReconfigureReplacesAnEarlierBadConfiguration) {
  SurfaceFixAdapter adapter;
  ASSERT_FALSE(adapter
                   .ConfigureFromPluginXml(PluginXml(
                       With(DefaultChildren(), "position_bias_x_m", "nan")))
                   .ok());
  ASSERT_TRUE(
      adapter.ConfigureFromPluginXml(PluginXml(DefaultChildren())).ok());
  EXPECT_TRUE(adapter.active());
  EXPECT_TRUE(adapter.Step(StateAt(0)).measurement.has_value());
}

TEST(SurfaceFixAdapterTest, LinkNameIsOptionalAndRecorded) {
  SurfaceFixAdapter adapter;
  ASSERT_TRUE(
      adapter.ConfigureFromPluginXml(PluginXml(DefaultChildren())).ok());
  EXPECT_FALSE(adapter.link_name().has_value());
  ASSERT_TRUE(adapter
                  .ConfigureFromPluginXml(PluginXml(
                      With(DefaultChildren(), "link_name", "antenna")))
                  .ok());
  ASSERT_TRUE(adapter.link_name().has_value());
  EXPECT_EQ(*adapter.link_name(), "antenna");
}

std::vector<SurfaceFixSimulatorState> BadStates() {
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const double inf = std::numeric_limits<double>::infinity();
  std::vector<SurfaceFixSimulatorState> bad;
  SurfaceFixSimulatorState state = StateAt(1);
  state.link_resolved = false;
  bad.push_back(state);
  state = StateAt(1);
  state.pose_ready = false;
  bad.push_back(state);
  for (const double value : {nan, inf, -inf}) {
    for (int axis = 0; axis < 3; ++axis) {
      state = StateAt(1);
      state.position_m[axis] = value;
      bad.push_back(state);
    }
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
  state.time_nanos = 1000000000;
  bad.push_back(state);
  state = StateAt(1);
  state.time_nanos = -1;
  bad.push_back(state);
  return bad;
}

TEST(SurfaceFixAdapterTest, InvalidStateGivesNoSampleAndStillAdvancesTheStep) {
  const std::vector<SurfaceFixSimulatorState> bad = BadStates();
  ASSERT_GE(bad.size(), 16u);
  for (size_t index = 0; index < bad.size(); ++index) {
    SurfaceFixAdapter adapter;
    ASSERT_TRUE(
        adapter.ConfigureFromPluginXml(PluginXml(DefaultChildren())).ok());
    EXPECT_TRUE(adapter.Step(StateAt(0)).measurement.has_value());
    const SurfaceFixStepResult result = adapter.Step(bad[index]);
    EXPECT_EQ(result.status.code, SurfaceFixSimErrorCode::kInvalidState)
        << index;
    EXPECT_FALSE(result.measurement.has_value()) << index;
    EXPECT_EQ(result.step, 1);
    EXPECT_EQ(adapter.StepIndexForTesting(), 2);
    EXPECT_TRUE(adapter.active());
    const SurfaceFixStepResult recovered = adapter.Step(StateAt(2));
    EXPECT_TRUE(recovered.status.ok());
    ASSERT_TRUE(recovered.measurement.has_value());
    EXPECT_EQ(recovered.step, 2);
    ExpectAccepted(*recovered.measurement);
  }
}

TEST(SurfaceFixAdapterTest, DerivedPositionOverflowIsInvalidState) {
  SurfaceFixAdapter adapter;
  ASSERT_TRUE(adapter
                  .ConfigureFromPluginXml(PluginXml(
                      With(DefaultChildren(), "position_bias_x_m", "1.7e308")))
                  .ok());
  SurfaceFixSimulatorState state = StateAt(0);
  state.position_m[0] = 1.7e308;
  const SurfaceFixStepResult overflow = adapter.Step(state);
  EXPECT_EQ(overflow.status.code, SurfaceFixSimErrorCode::kInvalidState);
  EXPECT_FALSE(overflow.measurement.has_value());
}

TEST(SurfaceFixAdapterTest, QuaternionWithinToleranceIsAcceptedAsIs) {
  SurfaceFixAdapter adapter;
  ASSERT_TRUE(
      adapter.ConfigureFromPluginXml(PluginXml(DefaultChildren())).ok());
  SurfaceFixSimulatorState state = StateAt(0);
  state.orientation_xyzw = {0, 0, 0, 1.0 + 5e-7};
  const SurfaceFixStepResult result = adapter.Step(state);
  EXPECT_TRUE(result.status.ok());
  EXPECT_TRUE(result.measurement.has_value());
}

TEST(SurfaceFixAdapterTest, InvalidStateCostsTheDelayedSampleToo) {
  const Children children = With(DefaultChildren(), "latency_steps", "2");
  SurfaceFixAdapter adapter;
  ASSERT_TRUE(adapter.ConfigureFromPluginXml(PluginXml(children)).ok());
  std::vector<bool> present;
  for (int k = 0; k < 8; ++k) {
    SurfaceFixSimulatorState state = StateAt(k);
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

TEST(SurfaceFixAdapterTest,
     TwoFreshAdaptersAreByteIdenticalOverAThousandSteps) {
  const Children children =
      With(With(With(DefaultChildren(), "noise_amplitude_m", "0.5"),
                "position_bias_z_m", "0.01"),
           "latency_steps", "5");
  const StateFn state_fn = [](int k) {
    return StateAt(k, {0.001 * k, 0, (k / 50) % 2 == 0 ? 0.0 : -2.0});
  };
  const std::vector<SurfaceFixStepResult> first =
      RunSteps(children, 1000, state_fn);
  const std::vector<SurfaceFixStepResult> second =
      RunSteps(children, 1000, state_fn);
  int emitted = 0;
  for (int k = 0; k < 1000; ++k) {
    ASSERT_EQ(first[k].measurement.has_value(),
              second[k].measurement.has_value());
    if (first[k].measurement.has_value()) {
      ++emitted;
      EXPECT_EQ(Bytes(first[k]), Bytes(second[k])) << k;
    }
  }
  EXPECT_GT(emitted, 0);
  EXPECT_LT(emitted, 1000);
}

TEST(SurfaceFixAdapterTest, ResetThenRerunReproducesTheSameTrace) {
  const Children children =
      With(With(With(DefaultChildren(), "noise_amplitude_m", "0.5"),
                "latency_steps", "3"),
           "dropout_window", Window(4, 6));
  SurfaceFixAdapter adapter;
  ASSERT_TRUE(adapter.ConfigureFromPluginXml(PluginXml(children)).ok());
  std::vector<std::optional<std::string>> first;
  for (int k = 0; k < 30; ++k) {
    const SurfaceFixStepResult result = adapter.Step(StateAt(k));
    first.push_back(result.measurement.has_value()
                        ? std::optional<std::string>(Bytes(result))
                        : std::nullopt);
  }
  EXPECT_EQ(adapter.StepIndexForTesting(), 30);
  adapter.Reset();
  EXPECT_EQ(adapter.StepIndexForTesting(), 0);
  for (int k = 0; k < 30; ++k) {
    const SurfaceFixStepResult result = adapter.Step(StateAt(k));
    EXPECT_EQ(result.step, k);
    if (first[k].has_value()) {
      ASSERT_TRUE(result.measurement.has_value()) << k;
      EXPECT_EQ(Bytes(result), *first[k]) << k;
    } else {
      EXPECT_FALSE(result.measurement.has_value()) << k;
    }
  }
}

TEST(SurfaceFixAdapterTest, ResetClearsPendingLatencyQueue) {
  const Children children = With(DefaultChildren(), "latency_steps", "3");
  SurfaceFixAdapter adapter;
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

// ---- S9: golden -----------------------------------------------------------

Children GoldenChildren() {
  Children children = DefaultChildren();
  children = With(children, "surface_z_m", "0");
  children = With(children, "max_fix_depth_m", "0.5");
  children = With(children, "position_bias_x_m", "0.25");
  children = With(children, "position_bias_z_m", "-0.02");
  children = With(children, "noise_amplitude_m", "0.04");
  children = With(children, "latency_steps", "2");
  children = With(children, "dropout_window", Window(5, 7));
  children = With(children, "seed", "42");
  return children;
}

// The link descends through the gate at step 10 and surfaces again at 15.
// z is exactly representable so the trace does not depend on a libm.
SurfaceFixSimulatorState GoldenState(int k) {
  double z = 0.125;
  if (k >= 10 && k < 15) {
    z = -1.0;
  } else if (k >= 8 && k < 10) {
    z = -0.5;
  }
  return StateAt(k, {0.5 * k, -0.25 * k, z});
}

std::vector<SurfaceFixStepResult> GoldenRun() {
  return RunSteps(GoldenChildren(), 20, GoldenState);
}

TEST(SurfaceFixAdapterTest, GoldenSeed42Trace) {
  const std::vector<SurfaceFixStepResult> results = GoldenRun();
  std::string trace = "surface_fix_sim_trace v1\nseed=42\n";
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

TEST(SurfaceFixAdapterTest, GoldenScenarioShapeIsAsContracted) {
  const std::vector<SurfaceFixStepResult> results = GoldenRun();
  for (int k = 0; k < 20; ++k) {
    // Latency 2: step k emits what originated at k - 2. Origination is absent
    // for dropout [5, 7) and for the gate at origination steps 10..14.
    const int origin = k - 2;
    const bool expect_absent = origin < 0 || (origin >= 5 && origin < 7) ||
                               (origin >= 10 && origin < 15);
    ASSERT_EQ(results[k].measurement.has_value(), !expect_absent) << k;
    if (!expect_absent) {
      ExpectDegradedNotAccepted(*results[k].measurement);
      EXPECT_EQ(results[k].measurement->health().header().sequence(),
                static_cast<uint64_t>(42 + origin));
    }
  }
}

// ---- Scope guard ----------------------------------------------------------

TEST(SurfaceFixAdapterTest, AdapterHasNoGzIncludeAndPluginIsReadOnly) {
  for (const char* name : {"surface_fix_adapter.h", "surface_fix_adapter.cc"}) {
    const std::string text = ReadFile(std::string(kSourcePrefix) + name);
    ASSERT_FALSE(text.empty()) << name;
    EXPECT_EQ(text.find("#include \"gz/"), std::string::npos) << name;
  }
  for (const char* name : {"surface_fix_plugin.h", "surface_fix_plugin.cc",
                           "surface_fix_plugin_register.cc"}) {
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

TEST(SurfaceFixAdapterTest, HelperSymbolsDoNotClashWithOtherSensorAdapters) {
  const std::string text =
      ReadFile(std::string(kSourcePrefix) + "surface_fix_adapter.cc");
  EXPECT_EQ(text.find("SplitSimTimeNanos"), std::string::npos);
  EXPECT_NE(text.find("SplitSurfaceFixSimTimeNanos"), std::string::npos);
}

}  // namespace
}  // namespace intrinsic::simulation
