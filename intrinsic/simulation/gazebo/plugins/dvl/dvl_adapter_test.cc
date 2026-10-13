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

#include "intrinsic/simulation/gazebo/plugins/dvl/dvl_adapter.h"

#include <array>
#include <bit>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <functional>
#include <iterator>
#include <limits>
#include <optional>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include "gtest/gtest.h"
#include "intrinsic/embodiment/frame_policy.h"
#include "intrinsic/hardware/marine/dvl.pb.h"
#include "intrinsic/hardware/marine/dvl_policy.h"
#include "intrinsic/hardware/marine/fake_dvl.h"
#include "intrinsic/hardware/marine/fake_imu.h"
#include "intrinsic/hardware/marine/measurement_health.pb.h"
#include "intrinsic/hardware/marine/measurement_health_policy.h"

namespace intrinsic::simulation {
namespace {

using ::intrinsic::hardware::marine::AssessDvl;
using ::intrinsic::hardware::marine::DvlAssessment;
using ::intrinsic::hardware::marine::DvlError;
using ::intrinsic::hardware::marine::DvlMeasurementView;
using ::intrinsic::hardware::marine::DvlModeKind;
using ::intrinsic::hardware::marine::DvlTruth;
using ::intrinsic::hardware::marine::FakeDvl;
using ::intrinsic::hardware::marine::FakeDvlConfig;
using ::intrinsic::hardware::marine::ImuSignedUnitNoise;
using ::intrinsic::hardware::marine::MeasurementStateKind;
using ::intrinsic::hardware::marine::SourceHealthView;
using ::intrinsic_proto::hardware::marine::DvlMeasurement;
using ::intrinsic_proto::hardware::marine::MeasurementHealth;
using Children = std::vector<std::pair<std::string, std::string>>;
using StateFn = std::function<DvlSimulatorState(int)>;

constexpr char kGoldenPath[] =
    "intrinsic/simulation/gazebo/plugins/dvl/testdata/"
    "dvl_adapter_seed_42.golden.txt";
constexpr char kCurrentGoldenPath[] =
    "intrinsic/scene/sdf/testdata/"
    "current_field_hydrodynamics_constant.golden.sdf";
constexpr int64_t kStepNanos = 10000000;
constexpr uint64_t kGamma = 0x9E3779B97F4A7C15ULL;
constexpr std::array<double, 3> kForwardSway = {0.5, -0.25, 0.0};

std::string ReadFile(const char* path) {
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

uint64_t Bits(double value) { return std::bit_cast<uint64_t>(value); }

// ---- SDF construction -----------------------------------------------------

// Contract default world: flat seabed at z = -50, 0.5 <= a <= 30.
Children DefaultChildren() {
  return {{"seabed_z_m", "-50"},
          {"min_altitude_m", "0.5"},
          {"max_altitude_m", "30"}};
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
  Children kept;
  for (auto& child : children) {
    if (child.first != name) {
      kept.push_back(std::move(child));
    }
  }
  return kept;
}

std::string PluginXml(const Children& children) {
  std::string xml =
      "<plugin filename=\"static://intrinsic::simulation::Dvl\" "
      "name=\"intrinsic::simulation::Dvl\">";
  for (const auto& [name, value] : children) {
    xml += "<" + name + ">" + value + "</" + name + ">";
  }
  xml += "</plugin>";
  return xml;
}

std::string Window(int start, int end) {
  return "<start_step>" + std::to_string(start) + "</start_step><end_step>" +
         std::to_string(end) + "</end_step>";
}

std::string CurrentField(std::string_view frame, std::string_view velocity) {
  return "<frame_id>" + std::string(frame) + "</frame_id><velocity_m_s>" +
         std::string(velocity) + "</velocity_m_s>";
}

// The <current_field> block of the checked-in #121 golden SDF.
std::string GoldenCurrentInner() {
  const std::string golden = ReadFile(kCurrentGoldenPath);
  constexpr std::string_view kOpen = "<current_field>";
  constexpr std::string_view kClose = "</current_field>";
  const size_t open = golden.find(kOpen);
  const size_t close = golden.find(kClose);
  EXPECT_NE(open, std::string::npos);
  EXPECT_NE(close, std::string::npos);
  if (open == std::string::npos || close == std::string::npos) {
    return "";
  }
  return golden.substr(open + kOpen.size(), close - open - kOpen.size());
}

// ---- State and result helpers ---------------------------------------------

DvlSimulatorState StateAt(int k, double z = -40.0,
                          std::array<double, 3> world_velocity = kForwardSway,
                          std::array<double, 4> orientation = {0, 0, 0, 1}) {
  DvlSimulatorState state;
  state.link_resolved = true;
  state.kinematics_ready = true;
  state.position_m = {0, 0, z};
  state.orientation_xyzw = orientation;
  state.world_linear_velocity_m_s = world_velocity;
  SplitSimTimeNanos(static_cast<int64_t>(k) * kStepNanos, &state.time_seconds,
                    &state.time_nanos);
  return state;
}

std::vector<DvlStepResult> RunSteps(const Children& children, int steps,
                                    const StateFn& state_fn) {
  DvlAdapter adapter;
  const DvlSimStatus status =
      adapter.ConfigureFromPluginXml(PluginXml(children));
  EXPECT_TRUE(status.ok()) << status.message;
  std::vector<DvlStepResult> results;
  for (int k = 0; k < steps; ++k) {
    results.push_back(adapter.Step(state_fn(k)));
  }
  return results;
}

std::vector<DvlStepResult> RunSteps(const Children& children, int steps) {
  return RunSteps(children, steps, [](int k) { return StateAt(k); });
}

std::string Bytes(const DvlStepResult& result) {
  EXPECT_TRUE(result.measurement.has_value());
  return result.measurement.has_value()
             ? result.measurement->SerializeAsString()
             : std::string();
}

DvlMeasurementView ViewOf(const DvlMeasurement& sample,
                          std::vector<double>* covariance,
                          std::vector<SourceHealthView>* sources) {
  DvlMeasurementView view;
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
  view.mode_present = sample.has_mode();
  view.mode = sample.mode();
  view.velocity_x_present = sample.has_velocity_x_m_s();
  view.velocity_x_m_s = sample.velocity_x_m_s();
  view.velocity_y_present = sample.has_velocity_y_m_s();
  view.velocity_y_m_s = sample.velocity_y_m_s();
  view.velocity_z_present = sample.has_velocity_z_m_s();
  view.velocity_z_m_s = sample.velocity_z_m_s();
  view.bottom_lock_present = sample.has_bottom_lock();
  view.bottom_lock = sample.bottom_lock();
  view.altitude_present = sample.has_altitude_m();
  view.altitude_m = sample.altitude_m();
  return view;
}

// Runs the HAL conformance oracle with expected frame "body".
DvlAssessment Assess(const DvlMeasurement& sample) {
  std::vector<double> covariance;
  std::vector<SourceHealthView> sources;
  return AssessDvl(ViewOf(sample, &covariance, &sources));
}

void ExpectFinite(const DvlMeasurement& sample) {
  EXPECT_TRUE(std::isfinite(sample.velocity_x_m_s()));
  EXPECT_TRUE(std::isfinite(sample.velocity_y_m_s()));
  EXPECT_TRUE(std::isfinite(sample.velocity_z_m_s()));
  EXPECT_TRUE(sample.has_velocity_x_m_s());
  EXPECT_TRUE(sample.has_velocity_y_m_s());
  EXPECT_TRUE(sample.has_velocity_z_m_s());
  if (sample.has_altitude_m()) {
    EXPECT_TRUE(std::isfinite(sample.altitude_m()));
  }
}

void ExpectAccepted(const DvlMeasurement& sample) {
  const DvlAssessment assessment = Assess(sample);
  EXPECT_EQ(assessment.error, DvlError::kNone);
  EXPECT_EQ(assessment.state, MeasurementStateKind::kValid);
  EXPECT_TRUE(assessment.accepted);
}

void ExpectDegradedNotAccepted(const DvlMeasurement& sample) {
  const DvlAssessment assessment = Assess(sample);
  EXPECT_EQ(assessment.error, DvlError::kNone);
  EXPECT_EQ(assessment.state, MeasurementStateKind::kDegraded);
  EXPECT_FALSE(assessment.accepted);
  EXPECT_EQ(sample.health().state(), MeasurementHealth::DEGRADED);
}

void ExpectLockLoss(const DvlMeasurement& sample) {
  EXPECT_EQ(sample.mode(), DvlMeasurement::MODE_BOTTOM_TRACK);
  ASSERT_TRUE(sample.has_bottom_lock());
  EXPECT_FALSE(sample.bottom_lock());
  EXPECT_FALSE(sample.has_altitude_m());
  EXPECT_EQ(sample.health().state(), MeasurementHealth::INVALID);
  ExpectFinite(sample);
  const DvlAssessment assessment = Assess(sample);
  EXPECT_EQ(assessment.error, DvlError::kNone);
  EXPECT_EQ(assessment.state, MeasurementStateKind::kInvalid);
  EXPECT_FALSE(assessment.accepted);
}

std::array<int64_t, 2> Stamp(
    const intrinsic_proto::embodiment::StampedHeader& header, bool source) {
  const auto& time = source ? header.source_time() : header.receive_time();
  return {time.seconds(), time.nanos()};
}

int64_t TotalNanos(const std::array<int64_t, 2>& stamp) {
  return stamp[0] * 1000000000LL + stamp[1];
}

// ---- Independent noise formula (does not call the adapter) ----------------

double Unit(uint64_t seed, int k, int axis) {
  return ImuSignedUnitNoise(
      seed +
      (3 * static_cast<uint64_t>(k) + static_cast<uint64_t>(axis)) * kGamma);
}

// ---- Fixture 1: bottom-lock -----------------------------------------------

TEST(DvlAdapterTest, BottomLockMatchesFakeDvlBytes) {
  const std::vector<DvlStepResult> results = RunSteps(DefaultChildren(), 3);
  for (int k = 0; k < 3; ++k) {
    ASSERT_TRUE(results[k].status.ok());
    ASSERT_TRUE(results[k].measurement.has_value());
    FakeDvlConfig config;
    config.seed = 42 + k;
    config.delay_seconds = 0;
    config.delay_nanos = 0;
    DvlTruth truth;
    truth.frame_id = "body";
    truth.source_seconds = 0;
    truth.source_nanos = static_cast<int32_t>(k * kStepNanos);
    truth.velocity_x_m_s = 0.5;
    truth.velocity_y_m_s = -0.25;
    truth.velocity_z_m_s = 0.0;
    truth.mode = 1;
    truth.quality = 0.75;
    truth.altitude_present = true;
    truth.altitude_m = 10.0;
    truth.covariance_present = true;
    const std::optional<DvlMeasurement> expected =
        FakeDvl(config).Measure(truth);
    ASSERT_TRUE(expected.has_value());
    EXPECT_EQ(Bytes(results[k]), expected->SerializeAsString()) << k;

    const DvlMeasurement& sample = *results[k].measurement;
    EXPECT_EQ(sample.velocity_x_m_s(), 0.5);
    EXPECT_EQ(sample.velocity_y_m_s(), -0.25);
    EXPECT_EQ(sample.velocity_z_m_s(), 0.0);
    EXPECT_EQ(sample.mode(), DvlMeasurement::MODE_BOTTOM_TRACK);
    EXPECT_TRUE(sample.bottom_lock());
    EXPECT_EQ(sample.altitude_m(), 10.0);
    EXPECT_EQ(sample.health().state(), MeasurementHealth::VALID);
    EXPECT_EQ(sample.health().header().frame_id(), "body");
    EXPECT_EQ(sample.health().header().sequence(), 42u + k);
    ExpectFinite(sample);
    ExpectAccepted(sample);
  }
}

TEST(DvlAdapterTest, AltitudeExactlyAtMinAndMaxStillLocks) {
  // a = z - (-50) is exact for these values.
  for (const double z : {-49.5, -20.0}) {
    DvlAdapter adapter;
    ASSERT_TRUE(
        adapter.ConfigureFromPluginXml(PluginXml(DefaultChildren())).ok());
    const DvlStepResult result = adapter.Step(StateAt(0, z));
    ASSERT_TRUE(result.measurement.has_value()) << z;
    EXPECT_TRUE(result.measurement->bottom_lock()) << z;
    EXPECT_EQ(result.measurement->altitude_m(), z + 50.0);
    ExpectAccepted(*result.measurement);
  }
}

// ---- Fixture 2: frame -----------------------------------------------------

TEST(DvlAdapterTest, YawNinetyDegreesRotatesWorldVelocityIntoBody) {
  const double half_sqrt2 = std::sqrt(0.5);
  DvlAdapter adapter;
  ASSERT_TRUE(
      adapter.ConfigureFromPluginXml(PluginXml(DefaultChildren())).ok());
  const DvlStepResult result = adapter.Step(
      StateAt(0, -40.0, {0, 1, 0}, {0, 0, half_sqrt2, half_sqrt2}));
  ASSERT_TRUE(result.measurement.has_value());
  EXPECT_NEAR(result.measurement->velocity_x_m_s(), 1.0, 1e-12);
  EXPECT_NEAR(result.measurement->velocity_y_m_s(), 0.0, 1e-12);
  EXPECT_NEAR(result.measurement->velocity_z_m_s(), 0.0, 1e-12);
  ExpectAccepted(*result.measurement);
}

TEST(DvlAdapterTest, NonAxisRotationMatchesConjugateRotateVector) {
  const double norm = std::sqrt(0.1 * 0.1 + 0.2 * 0.2 + 0.3 * 0.3 + 0.4 * 0.4);
  const std::array<double, 4> q = {0.1 / norm, 0.2 / norm, 0.3 / norm,
                                   0.4 / norm};
  const std::array<double, 3> world = {0.7, -0.3, 0.2};
  DvlAdapter adapter;
  ASSERT_TRUE(
      adapter.ConfigureFromPluginXml(PluginXml(DefaultChildren())).ok());
  const DvlStepResult result = adapter.Step(StateAt(0, -40.0, world, q));
  ASSERT_TRUE(result.measurement.has_value());

  const embodiment::Vec3 direct = embodiment::RotateVector(
      embodiment::Quaternion{-q[0], -q[1], -q[2], q[3]},
      embodiment::Vec3{world[0], world[1], world[2]});
  EXPECT_EQ(result.measurement->velocity_x_m_s(), direct.x);
  EXPECT_EQ(result.measurement->velocity_y_m_s(), direct.y);
  EXPECT_EQ(result.measurement->velocity_z_m_s(), direct.z);

  // Independent check: v_body = R(q)^T v_w with R built from the quaternion.
  const double x = q[0], y = q[1], z = q[2], w = q[3];
  const double r[3][3] = {
      {1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)},
      {2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)},
      {2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)}};
  for (int i = 0; i < 3; ++i) {
    const double body =
        r[0][i] * world[0] + r[1][i] * world[1] + r[2][i] * world[2];
    const double actual = i == 0   ? result.measurement->velocity_x_m_s()
                          : i == 1 ? result.measurement->velocity_y_m_s()
                                   : result.measurement->velocity_z_m_s();
    EXPECT_NEAR(actual, body, 1e-12) << i;
  }
}

// ---- Fixture 3: water track -----------------------------------------------

TEST(DvlAdapterTest, WaterTrackSubtractsGoldenWorldEnuCurrent) {
  const Children children = With(With(DefaultChildren(), "mode", "water_track"),
                                 "current_field", GoldenCurrentInner());
  DvlAdapter adapter;
  ASSERT_TRUE(adapter.ConfigureFromPluginXml(PluginXml(children)).ok());
  const DvlStepResult result = adapter.Step(StateAt(0));
  ASSERT_TRUE(result.status.ok());
  ASSERT_TRUE(result.measurement.has_value());
  const DvlMeasurement& sample = *result.measurement;
  EXPECT_EQ(sample.velocity_x_m_s(), 0.3);
  EXPECT_EQ(sample.velocity_y_m_s(), -0.15);
  EXPECT_EQ(sample.velocity_z_m_s(), 0.0);
  EXPECT_EQ(sample.mode(), DvlMeasurement::MODE_WATER_TRACK);
  EXPECT_FALSE(sample.has_bottom_lock());
  EXPECT_EQ(sample.health().state(), MeasurementHealth::VALID);
  EXPECT_EQ(sample.altitude_m(), 10.0);
  ExpectAccepted(sample);
  EXPECT_EQ(Assess(sample).mode, DvlModeKind::kWaterTrack);
}

TEST(DvlAdapterTest, WaterTrackNedAndBodyCurrents) {
  const Children base = With(DefaultChildren(), "mode", "water_track");
  {
    // NED (0.2, -0.1, 0.05) is ENU (-0.1, 0.2, -0.05).
    const Children children =
        With(base, "current_field", CurrentField("world_ned", "0.2 -0.1 0.05"));
    DvlAdapter adapter;
    ASSERT_TRUE(adapter.ConfigureFromPluginXml(PluginXml(children)).ok());
    const embodiment::Vec3 enu =
        embodiment::WorldVectorNedToEnu(embodiment::Vec3{0.2, -0.1, 0.05});
    const DvlStepResult result = adapter.Step(StateAt(0));
    ASSERT_TRUE(result.measurement.has_value());
    EXPECT_EQ(result.measurement->velocity_x_m_s(), 0.5 - enu.x);
    EXPECT_EQ(result.measurement->velocity_y_m_s(), -0.25 - enu.y);
    EXPECT_EQ(result.measurement->velocity_z_m_s(), 0.0 - enu.z);
    EXPECT_NEAR(result.measurement->velocity_x_m_s(), 0.6, 1e-12);
    EXPECT_NEAR(result.measurement->velocity_y_m_s(), -0.45, 1e-12);
    EXPECT_NEAR(result.measurement->velocity_z_m_s(), 0.05, 1e-12);
    ExpectAccepted(*result.measurement);
  }
  {
    // A body current is subtracted directly, whatever the attitude.
    const double half_sqrt2 = std::sqrt(0.5);
    const Children children =
        With(base, "current_field", CurrentField("body", "0.2 -0.1 0"));
    DvlAdapter adapter;
    ASSERT_TRUE(adapter.ConfigureFromPluginXml(PluginXml(children)).ok());
    const DvlStepResult result = adapter.Step(
        StateAt(0, -40.0, {0, 1, 0}, {0, 0, half_sqrt2, half_sqrt2}));
    ASSERT_TRUE(result.measurement.has_value());
    EXPECT_NEAR(result.measurement->velocity_x_m_s(), 1.0 - 0.2, 1e-12);
    EXPECT_NEAR(result.measurement->velocity_y_m_s(), 0.0 + 0.1, 1e-12);
    EXPECT_NEAR(result.measurement->velocity_z_m_s(), 0.0, 1e-12);
  }
}

TEST(DvlAdapterTest, WaterTrackRotatesEnuCurrentIntoBody) {
  const double half_sqrt2 = std::sqrt(0.5);
  const std::array<double, 4> q = {0, 0, half_sqrt2, half_sqrt2};
  const Children children =
      With(With(DefaultChildren(), "mode", "water_track"), "current_field",
           CurrentField("world_enu", "0.2 -0.1 0"));
  DvlAdapter adapter;
  ASSERT_TRUE(adapter.ConfigureFromPluginXml(PluginXml(children)).ok());
  const DvlStepResult result = adapter.Step(StateAt(0, -40.0, {0, 1, 0}, q));
  ASSERT_TRUE(result.measurement.has_value());
  // Yaw +90 degrees: R^T (0, 1, 0) = (1, 0, 0), R^T (0.2, -0.1, 0) =
  // (-0.1, -0.2, 0).
  EXPECT_NEAR(result.measurement->velocity_x_m_s(), 1.0 + 0.1, 1e-12);
  EXPECT_NEAR(result.measurement->velocity_y_m_s(), 0.0 + 0.2, 1e-12);
}

TEST(DvlAdapterTest, WaterTrackOutOfRangeKeepsTrackAndOmitsAltitude) {
  const Children children = With(With(DefaultChildren(), "mode", "water_track"),
                                 "current_field", GoldenCurrentInner());
  DvlAdapter adapter;
  ASSERT_TRUE(adapter.ConfigureFromPluginXml(PluginXml(children)).ok());
  for (const double z : {0.0, -60.0}) {
    const DvlStepResult result = adapter.Step(StateAt(0, z));
    ASSERT_TRUE(result.measurement.has_value());
    EXPECT_EQ(result.measurement->mode(), DvlMeasurement::MODE_WATER_TRACK);
    EXPECT_FALSE(result.measurement->has_bottom_lock());
    EXPECT_FALSE(result.measurement->has_altitude_m());
    EXPECT_EQ(result.measurement->health().state(), MeasurementHealth::VALID);
    ExpectAccepted(*result.measurement);
  }
}

TEST(DvlAdapterTest, BottomTrackIgnoresPresentCurrentField) {
  const Children with_current =
      With(DefaultChildren(), "current_field", GoldenCurrentInner());
  const std::vector<DvlStepResult> a = RunSteps(with_current, 3);
  const std::vector<DvlStepResult> b = RunSteps(DefaultChildren(), 3);
  for (int k = 0; k < 3; ++k) {
    EXPECT_EQ(Bytes(a[k]), Bytes(b[k]));
  }
}

TEST(DvlAdapterTest, WaterTrackWithoutCurrentFieldIsInvalidConfig) {
  DvlAdapter adapter;
  const DvlSimStatus status = adapter.ConfigureFromPluginXml(
      PluginXml(With(DefaultChildren(), "mode", "water_track")));
  EXPECT_EQ(status.code, DvlSimErrorCode::kInvalidConfig);
  EXPECT_FALSE(adapter.active());
  for (int k = 0; k < 3; ++k) {
    const DvlStepResult result = adapter.Step(StateAt(k));
    EXPECT_EQ(result.status.code, DvlSimErrorCode::kInvalidConfig);
    EXPECT_FALSE(result.measurement.has_value());
  }
}

// ---- Fixture 4: lock-loss -------------------------------------------------

TEST(DvlAdapterTest, LockLossAboveMaxBelowMinAndBelowSeabed) {
  // a = max + 1e-6, a = min - 1e-6, a = -1 (below the seabed).
  const double altitudes[] = {30.0 + 1e-6, 0.5 - 1e-6, -1.0};
  for (const double a : altitudes) {
    DvlAdapter adapter;
    ASSERT_TRUE(
        adapter.ConfigureFromPluginXml(PluginXml(DefaultChildren())).ok());
    const DvlStepResult lost = adapter.Step(StateAt(0, -50.0 + a));
    ASSERT_TRUE(lost.status.ok());
    ASSERT_TRUE(lost.measurement.has_value()) << a;
    ExpectLockLoss(*lost.measurement);
    EXPECT_EQ(lost.measurement->velocity_x_m_s(), 0.5);
    EXPECT_EQ(lost.measurement->velocity_y_m_s(), -0.25);
    EXPECT_EQ(lost.measurement->velocity_z_m_s(), 0.0);

    // No hysteresis: back in range at the next step is VALID again.
    const DvlStepResult back = adapter.Step(StateAt(1, -40.0));
    ASSERT_TRUE(back.measurement.has_value());
    EXPECT_TRUE(back.measurement->bottom_lock());
    EXPECT_EQ(back.measurement->altitude_m(), 10.0);
    ExpectAccepted(*back.measurement);
  }
}

TEST(DvlAdapterTest, LockLossNeverEmitsNaNEvenWithNoiseAndBias) {
  const Children children =
      With(With(DefaultChildren(), "noise_amplitude_m_s", "0.05"), "bias_m_s",
           "0.1 0 -0.05");
  const std::vector<DvlStepResult> results = RunSteps(
      children, 50, [](int k) { return StateAt(k, k % 2 == 0 ? -10 : -40); });
  for (int k = 0; k < 50; ++k) {
    ASSERT_TRUE(results[k].measurement.has_value());
    ExpectFinite(*results[k].measurement);
    if (k % 2 == 0) {
      ExpectLockLoss(*results[k].measurement);
    }
  }
}

// ---- Fixture 5: constant bias ---------------------------------------------

TEST(DvlAdapterTest, ConstantBiasIsAddedBitExactAndDegrades) {
  const Children children = With(DefaultChildren(), "bias_m_s", "0.1 0 -0.05");
  const std::vector<DvlStepResult> results = RunSteps(children, 3);
  for (const DvlStepResult& result : results) {
    ASSERT_TRUE(result.measurement.has_value());
    const DvlMeasurement& sample = *result.measurement;
    EXPECT_EQ(Bits(sample.velocity_x_m_s()), Bits(0.5 + 0.1));
    EXPECT_EQ(Bits(sample.velocity_y_m_s()), Bits(-0.25 + 0.0));
    EXPECT_EQ(Bits(sample.velocity_z_m_s()), Bits(0.0 + -0.05));
    EXPECT_TRUE(sample.bottom_lock());
    ExpectDegradedNotAccepted(sample);
  }
}

TEST(DvlAdapterTest, BiasStillAddsDuringLockLoss) {
  const Children children = With(DefaultChildren(), "bias_m_s", "0.1 0 -0.05");
  DvlAdapter adapter;
  ASSERT_TRUE(adapter.ConfigureFromPluginXml(PluginXml(children)).ok());
  const DvlStepResult result = adapter.Step(StateAt(0, -10.0));
  ASSERT_TRUE(result.measurement.has_value());
  ExpectLockLoss(*result.measurement);
  EXPECT_EQ(Bits(result.measurement->velocity_x_m_s()), Bits(0.5 + 0.1));
  EXPECT_EQ(Bits(result.measurement->velocity_z_m_s()), Bits(0.0 + -0.05));
}

// ---- Fixture 6: latency ---------------------------------------------------

TEST(DvlAdapterTest, LatencyRampEmitsOriginatedSampleAtLaterReceiveTime) {
  const Children children = With(DefaultChildren(), "latency_steps", "3");
  const std::vector<DvlStepResult> results = RunSteps(children, 12, [](int k) {
    return StateAt(k, -40.0, {0.1 * k, 0.0, 0.0});
  });
  for (int k = 0; k < 12; ++k) {
    ASSERT_TRUE(results[k].status.ok());
    if (k < 3) {
      EXPECT_FALSE(results[k].measurement.has_value()) << k;
      continue;
    }
    ASSERT_TRUE(results[k].measurement.has_value()) << k;
    const DvlMeasurement& sample = *results[k].measurement;
    EXPECT_EQ(sample.velocity_x_m_s(), 0.1 * (k - 3)) << k;
    const auto& header = sample.health().header();
    EXPECT_EQ(TotalNanos(Stamp(header, true)), (k - 3) * kStepNanos);
    EXPECT_EQ(TotalNanos(Stamp(header, false)), k * kStepNanos);
    EXPECT_EQ(
        TotalNanos(Stamp(header, false)) - TotalNanos(Stamp(header, true)),
        3 * kStepNanos);
    EXPECT_EQ(header.sequence(), static_cast<uint64_t>(42 + k - 3));
    ExpectAccepted(sample);
  }
}

TEST(DvlAdapterTest, LatencyMatchesFakeDvlWithHandBuiltDelay) {
  const Children children = With(DefaultChildren(), "latency_steps", "2");
  const std::vector<DvlStepResult> results = RunSteps(children, 6);
  for (int k = 2; k < 6; ++k) {
    FakeDvlConfig config;
    config.seed = 42 + (k - 2);
    config.delay_seconds = 0;
    config.delay_nanos = static_cast<int32_t>(2 * kStepNanos);
    DvlTruth truth;
    truth.frame_id = "body";
    truth.source_seconds = 0;
    truth.source_nanos = static_cast<int32_t>((k - 2) * kStepNanos);
    truth.velocity_x_m_s = 0.5;
    truth.velocity_y_m_s = -0.25;
    truth.velocity_z_m_s = 0.0;
    truth.altitude_m = 10.0;
    EXPECT_EQ(Bytes(results[k]),
              FakeDvl(config).Measure(truth)->SerializeAsString())
        << k;
  }
}

TEST(DvlAdapterTest, ZeroLatencyReceiveEqualsSource) {
  const std::vector<DvlStepResult> results = RunSteps(DefaultChildren(), 5);
  for (int k = 0; k < 5; ++k) {
    ASSERT_TRUE(results[k].measurement.has_value());
    const auto& header = results[k].measurement->health().header();
    EXPECT_EQ(Stamp(header, true), Stamp(header, false));
  }
}

TEST(DvlAdapterTest, LatencyAcrossSecondBoundaryKeepsNanosNormalized) {
  const Children children = With(DefaultChildren(), "latency_steps", "2");
  const std::vector<DvlStepResult> results = RunSteps(children, 4, [](int k) {
    DvlSimulatorState state = StateAt(0);
    SplitSimTimeNanos(995000000LL + k * kStepNanos, &state.time_seconds,
                      &state.time_nanos);
    return state;
  });
  ASSERT_TRUE(results[2].measurement.has_value());
  const auto& header = results[2].measurement->health().header();
  EXPECT_EQ(Stamp(header, true), (std::array<int64_t, 2>{0, 995000000}));
  EXPECT_EQ(Stamp(header, false), (std::array<int64_t, 2>{1, 15000000}));
  EXPECT_EQ(Assess(*results[2].measurement).error, DvlError::kNone);
}

TEST(DvlAdapterTest, LatencyBoundsAreEnforced) {
  const std::string values[] = {"1001", "-1", "abc", "1.5", "", "99999999999"};
  for (const std::string& value : values) {
    DvlAdapter adapter;
    const DvlSimStatus status = adapter.ConfigureFromPluginXml(
        PluginXml(With(DefaultChildren(), "latency_steps", value)));
    EXPECT_EQ(status.code, DvlSimErrorCode::kInvalidConfig) << value;
    EXPECT_FALSE(adapter.Step(StateAt(0)).measurement.has_value()) << value;
  }
  for (const char* value : {"0", "1000"}) {
    DvlAdapter adapter;
    EXPECT_TRUE(adapter
                    .ConfigureFromPluginXml(PluginXml(
                        With(DefaultChildren(), "latency_steps", value)))
                    .ok())
        << value;
  }
}

TEST(DvlAdapterTest, MaximumLatencyEmitsAtStepOneThousand) {
  const Children children = With(DefaultChildren(), "latency_steps", "1000");
  const std::vector<DvlStepResult> results = RunSteps(children, 1002);
  for (int k = 0; k < 1000; ++k) {
    ASSERT_FALSE(results[k].measurement.has_value()) << k;
  }
  ASSERT_TRUE(results[1000].measurement.has_value());
  EXPECT_EQ(results[1000].measurement->health().header().sequence(), 42u);
  ASSERT_TRUE(results[1001].measurement.has_value());
  EXPECT_EQ(results[1001].measurement->health().header().sequence(), 43u);
}

// ---- Fixture 7: seeded noise and byte-identical replay --------------------

TEST(DvlAdapterTest, NoiseMatchesFormulaBitExactWithinAmplitude) {
  constexpr double kAmp = 0.05;
  const std::array<double, 3> bias = {0.01, 0.0, -0.02};
  const Children children =
      With(With(DefaultChildren(), "noise_amplitude_m_s", "0.05"), "bias_m_s",
           "0.01 0 -0.02");
  const std::vector<DvlStepResult> results = RunSteps(children, 200);
  for (int k = 0; k < 200; ++k) {
    ASSERT_TRUE(results[k].measurement.has_value());
    const DvlMeasurement& sample = *results[k].measurement;
    const double truth[3] = {0.5, -0.25, 0.0};
    const double actual[3] = {sample.velocity_x_m_s(), sample.velocity_y_m_s(),
                              sample.velocity_z_m_s()};
    for (int i = 0; i < 3; ++i) {
      const double u = Unit(42, k, i);
      EXPECT_EQ(Bits(actual[i]), Bits(truth[i] + std::fma(kAmp, u, bias[i])))
          << k << " " << i;
    }
    ExpectDegradedNotAccepted(sample);
    EXPECT_TRUE(sample.bottom_lock());
  }
}

TEST(DvlAdapterTest, NoiseOnlyDrawIsBoundedByAmplitude) {
  constexpr double kAmp = 0.05;
  const Children children =
      With(DefaultChildren(), "noise_amplitude_m_s", "0.05");
  const std::vector<DvlStepResult> results = RunSteps(children, 300);
  bool any_nonzero = false;
  for (int k = 0; k < 300; ++k) {
    const DvlMeasurement& sample = *results[k].measurement;
    const double noise[3] = {sample.velocity_x_m_s() - 0.5,
                             sample.velocity_y_m_s() + 0.25,
                             sample.velocity_z_m_s() - 0.0};
    for (const double value : noise) {
      EXPECT_LE(std::fabs(value), kAmp + 1e-15);
      any_nonzero = any_nonzero || value != 0.0;
    }
    ExpectDegradedNotAccepted(sample);
  }
  EXPECT_TRUE(any_nonzero);
}

TEST(DvlAdapterTest, NoiseThatRoundsToZeroStillReportsDegraded) {
  // With the smallest subnormal amplitude, amp * u rounds to zero for any
  // |u| < 0.5, so FakeDvl sees a zero bias and returns VALID. The adapter
  // keeps the HAL rule that noise implies DEGRADED.
  const Children children =
      With(DefaultChildren(), "noise_amplitude_m_s", "5e-324");
  const std::vector<DvlStepResult> results = RunSteps(children, 400);
  int zero_bias_steps = 0;
  for (int k = 0; k < 400; ++k) {
    const double biased_x = std::fma(std::numeric_limits<double>::denorm_min(),
                                     Unit(42, k, 0), 0.0);
    const double biased_y = std::fma(std::numeric_limits<double>::denorm_min(),
                                     Unit(42, k, 1), 0.0);
    const double biased_z = std::fma(std::numeric_limits<double>::denorm_min(),
                                     Unit(42, k, 2), 0.0);
    if (biased_x != 0.0 || biased_y != 0.0 || biased_z != 0.0) {
      continue;
    }
    ++zero_bias_steps;
    ASSERT_TRUE(results[k].measurement.has_value());
    EXPECT_EQ(results[k].measurement->velocity_x_m_s(), 0.5);
    EXPECT_EQ(results[k].measurement->velocity_y_m_s(), -0.25);
    ExpectDegradedNotAccepted(*results[k].measurement);
  }
  EXPECT_GT(zero_bias_steps, 0);
}

TEST(DvlAdapterTest, FreshAdaptersReplayByteIdentical) {
  const Children children =
      With(With(DefaultChildren(), "noise_amplitude_m_s", "0.05"), "bias_m_s",
           "0.01 0 -0.02");
  const StateFn state_fn = [](int k) {
    return StateAt(k, -45.0 + 0.01 * k, {0.5 + 0.001 * k, -0.25, 0.0});
  };
  const std::vector<DvlStepResult> first = RunSteps(children, 1000, state_fn);
  const std::vector<DvlStepResult> second = RunSteps(children, 1000, state_fn);
  for (int k = 0; k < 1000; ++k) {
    ASSERT_EQ(first[k].measurement.has_value(),
              second[k].measurement.has_value());
    if (first[k].measurement.has_value()) {
      EXPECT_EQ(Bytes(first[k]), Bytes(second[k])) << k;
    }
  }
}

TEST(DvlAdapterTest, ResetThenRerunReproducesTheSameTrace) {
  const Children children =
      With(With(With(DefaultChildren(), "noise_amplitude_m_s", "0.05"),
                "latency_steps", "3"),
           "dropout_window", Window(4, 6));
  DvlAdapter adapter;
  ASSERT_TRUE(adapter.ConfigureFromPluginXml(PluginXml(children)).ok());
  std::vector<std::optional<std::string>> first;
  for (int k = 0; k < 30; ++k) {
    const DvlStepResult result = adapter.Step(StateAt(k));
    first.push_back(result.measurement.has_value()
                        ? std::optional<std::string>(Bytes(result))
                        : std::nullopt);
  }
  EXPECT_EQ(adapter.StepIndexForTesting(), 30);
  adapter.Reset();
  EXPECT_EQ(adapter.StepIndexForTesting(), 0);
  for (int k = 0; k < 30; ++k) {
    const DvlStepResult result = adapter.Step(StateAt(k));
    EXPECT_EQ(result.step, k);
    if (first[k].has_value()) {
      ASSERT_TRUE(result.measurement.has_value()) << k;
      EXPECT_EQ(Bytes(result), *first[k]) << k;
    } else {
      EXPECT_FALSE(result.measurement.has_value()) << k;
    }
  }
}

TEST(DvlAdapterTest, ResetClearsPendingLatencyQueue) {
  const Children children = With(DefaultChildren(), "latency_steps", "3");
  DvlAdapter adapter;
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

TEST(DvlAdapterTest, GoldenSeed42Trace) {
  Children children = DefaultChildren();
  children = With(children, "max_altitude_m", "25");
  children = With(children, "bias_m_s", "0.01 0 0");
  children = With(children, "noise_amplitude_m_s", "0.02");
  children = With(children, "latency_steps", "2");
  children = With(children, "dropout_window", Window(5, 7));
  children = With(children, "seed", "42");
  const std::vector<DvlStepResult> results =
      RunSteps(children, 20, [](int k) { return StateAt(k, -40.0 + k); });
  std::string trace = "dvl_sim_trace v1\nseed=42\n";
  for (int k = 0; k < 20; ++k) {
    ASSERT_TRUE(results[k].status.ok());
    trace += "step=" + std::to_string(k) + " ";
    trace += results[k].measurement.has_value()
                 ? ToHex(results[k].measurement->SerializeAsString())
                 : std::string("absent");
    trace += "\n";
  }
  const std::string golden = ReadFile(kGoldenPath);
  ASSERT_FALSE(golden.empty());
  EXPECT_EQ(trace, golden);
}

TEST(DvlAdapterTest, GoldenScenarioShapeIsAsContracted) {
  Children children = DefaultChildren();
  children = With(children, "max_altitude_m", "25");
  children = With(children, "bias_m_s", "0.01 0 0");
  children = With(children, "noise_amplitude_m_s", "0.02");
  children = With(children, "latency_steps", "2");
  children = With(children, "dropout_window", Window(5, 7));
  const std::vector<DvlStepResult> results =
      RunSteps(children, 20, [](int k) { return StateAt(k, -40.0 + k); });
  for (int k = 0; k < 20; ++k) {
    const bool expect_absent = k < 2 || (k >= 7 && k < 9);
    ASSERT_EQ(results[k].measurement.has_value(), !expect_absent) << k;
    if (expect_absent) {
      continue;
    }
    const int origin = k - 2;
    if (origin >= 16) {
      ExpectLockLoss(*results[k].measurement);
    } else {
      EXPECT_TRUE(results[k].measurement->bottom_lock()) << k;
      ExpectDegradedNotAccepted(*results[k].measurement);
    }
  }
}

// ---- Fixture 8: dropout ---------------------------------------------------

TEST(DvlAdapterTest, DropoutWithoutLatencyOmitsWindowAndKeepsLaterDraws) {
  const Children noisy = With(DefaultChildren(), "noise_amplitude_m_s", "0.05");
  const Children dropped = With(noisy, "dropout_window", Window(5, 8));
  const std::vector<DvlStepResult> base = RunSteps(noisy, 12);
  const std::vector<DvlStepResult> run = RunSteps(dropped, 12);
  for (int k = 0; k < 12; ++k) {
    ASSERT_TRUE(run[k].status.ok());
    if (k >= 5 && k < 8) {
      EXPECT_FALSE(run[k].measurement.has_value()) << k;
    } else {
      ASSERT_TRUE(run[k].measurement.has_value()) << k;
      EXPECT_EQ(Bytes(run[k]), Bytes(base[k])) << k;
    }
  }
  EXPECT_EQ(run[8].measurement->health().header().sequence(), 42u + 8);
}

TEST(DvlAdapterTest, DropoutWithLatencyShiftsAbsenceByLatency) {
  const Children noisy =
      With(With(DefaultChildren(), "noise_amplitude_m_s", "0.05"),
           "latency_steps", "2");
  const Children dropped = With(noisy, "dropout_window", Window(5, 8));
  const std::vector<DvlStepResult> base = RunSteps(noisy, 14);
  const std::vector<DvlStepResult> run = RunSteps(dropped, 14);
  for (int k = 0; k < 14; ++k) {
    const bool absent = k < 2 || (k >= 7 && k < 10);
    ASSERT_EQ(run[k].measurement.has_value(), !absent) << k;
    if (!absent) {
      EXPECT_EQ(Bytes(run[k]), Bytes(base[k])) << k;
    }
  }
}

TEST(DvlAdapterTest, EmptyAndMultipleDropoutWindows) {
  Children children = DefaultChildren();
  children.emplace_back("dropout_window", Window(3, 3));
  children.emplace_back("dropout_window", Window(2, 3));
  children.emplace_back("dropout_window", Window(6, 7));
  const std::vector<DvlStepResult> run = RunSteps(children, 9);
  for (int k = 0; k < 9; ++k) {
    EXPECT_EQ(run[k].measurement.has_value(), k != 2 && k != 6) << k;
  }
}

TEST(DvlAdapterTest, LockLossDoesNotShiftLaterDraws) {
  const Children noisy = With(DefaultChildren(), "noise_amplitude_m_s", "0.05");
  const std::vector<DvlStepResult> clear = RunSteps(noisy, 8);
  const std::vector<DvlStepResult> lost = RunSteps(
      noisy, 8, [](int k) { return StateAt(k, k == 3 ? -10.0 : -40.0); });
  for (int k = 0; k < 8; ++k) {
    ASSERT_TRUE(lost[k].measurement.has_value());
    if (k == 3) {
      ExpectLockLoss(*lost[k].measurement);
      // Lock-loss carries no noise: truth plus bias only.
      EXPECT_EQ(lost[k].measurement->velocity_x_m_s(), 0.5);
      EXPECT_EQ(lost[k].measurement->velocity_y_m_s(), -0.25);
      continue;
    }
    EXPECT_EQ(Bytes(lost[k]), Bytes(clear[k])) << k;
  }
}

TEST(DvlAdapterTest, InvalidStateDoesNotShiftLaterDraws) {
  const Children noisy = With(DefaultChildren(), "noise_amplitude_m_s", "0.05");
  const std::vector<DvlStepResult> clear = RunSteps(noisy, 8);
  DvlAdapter adapter;
  ASSERT_TRUE(adapter.ConfigureFromPluginXml(PluginXml(noisy)).ok());
  for (int k = 0; k < 8; ++k) {
    DvlSimulatorState state = StateAt(k);
    if (k == 3) {
      state.link_resolved = false;
    }
    const DvlStepResult result = adapter.Step(state);
    EXPECT_EQ(result.step, k);
    if (k == 3) {
      EXPECT_EQ(result.status.code, DvlSimErrorCode::kInvalidState);
      EXPECT_FALSE(result.measurement.has_value());
      continue;
    }
    ASSERT_TRUE(result.measurement.has_value());
    EXPECT_EQ(Bytes(result), Bytes(clear[k])) << k;
  }
}

// ---- Fixture 9: seed independence -----------------------------------------

DvlMeasurement WithoutSequenceOrVelocity(DvlMeasurement sample) {
  sample.mutable_health()->mutable_header()->clear_sequence();
  sample.clear_velocity_x_m_s();
  sample.clear_velocity_y_m_s();
  sample.clear_velocity_z_m_s();
  return sample;
}

TEST(DvlAdapterTest, SeedsDifferOnlyBySequenceWithoutNoise) {
  const std::vector<DvlStepResult> a =
      RunSteps(With(DefaultChildren(), "seed", "42"), 20);
  const std::vector<DvlStepResult> b =
      RunSteps(With(DefaultChildren(), "seed", "43"), 20);
  for (int k = 0; k < 20; ++k) {
    DvlMeasurement x = *a[k].measurement;
    DvlMeasurement y = *b[k].measurement;
    EXPECT_EQ(x.health().header().sequence(), 42u + k);
    EXPECT_EQ(y.health().header().sequence(), 43u + k);
    x.mutable_health()->mutable_header()->clear_sequence();
    y.mutable_health()->mutable_header()->clear_sequence();
    EXPECT_EQ(x.SerializeAsString(), y.SerializeAsString()) << k;
  }
}

TEST(DvlAdapterTest, SeedsDifferOnlyInVelocityAndSequenceWithNoise) {
  const Children noisy = With(DefaultChildren(), "noise_amplitude_m_s", "0.05");
  const std::vector<DvlStepResult> a = RunSteps(With(noisy, "seed", "42"), 20);
  const std::vector<DvlStepResult> b = RunSteps(With(noisy, "seed", "43"), 20);
  bool velocity_differs = false;
  for (int k = 0; k < 20; ++k) {
    EXPECT_EQ(WithoutSequenceOrVelocity(*a[k].measurement).SerializeAsString(),
              WithoutSequenceOrVelocity(*b[k].measurement).SerializeAsString())
        << k;
    velocity_differs = velocity_differs ||
                       a[k].measurement->velocity_x_m_s() !=
                           b[k].measurement->velocity_x_m_s() ||
                       a[k].measurement->velocity_y_m_s() !=
                           b[k].measurement->velocity_y_m_s() ||
                       a[k].measurement->velocity_z_m_s() !=
                           b[k].measurement->velocity_z_m_s();
  }
  EXPECT_TRUE(velocity_differs);
}

TEST(DvlAdapterTest, DefaultSeedIs42) {
  const std::vector<DvlStepResult> implicit =
      RunSteps(With(DefaultChildren(), "noise_amplitude_m_s", "0.05"), 5);
  const std::vector<DvlStepResult> explicit_seed =
      RunSteps(With(With(DefaultChildren(), "noise_amplitude_m_s", "0.05"),
                    "seed", "42"),
               5);
  for (int k = 0; k < 5; ++k) {
    EXPECT_EQ(Bytes(implicit[k]), Bytes(explicit_seed[k]));
  }
}

// ---- Fixture 10: config robustness ----------------------------------------

void ExpectInvalidConfigNoOutput(const Children& children,
                                 const std::string& label) {
  DvlAdapter adapter;
  const DvlSimStatus status =
      adapter.ConfigureFromPluginXml(PluginXml(children));
  EXPECT_EQ(status.code, DvlSimErrorCode::kInvalidConfig) << label;
  EXPECT_FALSE(status.message.empty()) << label;
  EXPECT_FALSE(adapter.active()) << label;
  for (int k = 0; k < 3; ++k) {
    const DvlStepResult result = adapter.Step(StateAt(k));
    EXPECT_EQ(result.status.code, DvlSimErrorCode::kInvalidConfig) << label;
    EXPECT_FALSE(result.measurement.has_value()) << label;
  }
}

TEST(DvlAdapterTest, InvalidConfigurationsFailClosed) {
  const Children base = DefaultChildren();
  const std::vector<std::pair<std::string, Children>> cases = {
      {"missing seabed", Without(base, "seabed_z_m")},
      {"missing max", Without(base, "max_altitude_m")},
      {"nan seabed", With(base, "seabed_z_m", "nan")},
      {"inf seabed", With(base, "seabed_z_m", "inf")},
      {"text seabed", With(base, "seabed_z_m", "deep")},
      {"empty seabed", With(base, "seabed_z_m", "")},
      {"inf max", With(base, "max_altitude_m", "inf")},
      {"zero max", With(base, "max_altitude_m", "0")},
      {"negative max", With(base, "max_altitude_m", "-3")},
      {"nan min", With(base, "min_altitude_m", "nan")},
      {"negative min", With(base, "min_altitude_m", "-0.1")},
      {"min above max", With(base, "min_altitude_m", "31")},
      {"negative amp", With(base, "noise_amplitude_m_s", "-0.01")},
      {"nan amp", With(base, "noise_amplitude_m_s", "nan")},
      {"inf amp", With(base, "noise_amplitude_m_s", "inf")},
      {"bad mode", With(base, "mode", "sideways")},
      {"empty mode", With(base, "mode", "")},
      {"bias two values", With(base, "bias_m_s", "1 2")},
      {"bias four values", With(base, "bias_m_s", "1 2 3 4")},
      {"bias nan", With(base, "bias_m_s", "nan 0 0")},
      {"bias inf", With(base, "bias_m_s", "0 inf 0")},
      {"seed text", With(base, "seed", "abc")},
      {"seed negative", With(base, "seed", "-1")},
      {"window start after end", With(base, "dropout_window", Window(5, 4))},
      {"window negative start", With(base, "dropout_window", Window(-1, 4))},
      {"window missing end",
       With(base, "dropout_window", "<start_step>1</start_step>")},
      {"window missing start",
       With(base, "dropout_window", "<end_step>1</end_step>")},
      {"window not integer",
       With(base, "dropout_window",
            "<start_step>1.5</start_step><end_step>3</end_step>")},
      {"current bad frame",
       With(base, "current_field", CurrentField("map", "0.2 -0.1 0"))},
      {"current missing frame",
       With(base, "current_field", "<velocity_m_s>0.2 -0.1 0</velocity_m_s>")},
      {"current missing velocity",
       With(base, "current_field", "<frame_id>world_enu</frame_id>")},
      {"current nan velocity",
       With(base, "current_field", CurrentField("world_enu", "nan 0 0"))},
      {"current two values",
       With(base, "current_field", CurrentField("world_enu", "0.2 -0.1"))},
      {"water bad current",
       With(With(base, "mode", "water_track"), "current_field",
            CurrentField("map", "0.2 -0.1 0"))},
  };
  for (const auto& [label, children] : cases) {
    ExpectInvalidConfigNoOutput(children, label);
  }
}

TEST(DvlAdapterTest, MoreThanSixtyFourDropoutWindowsIsInvalid) {
  for (const int count : {64, 65, 100}) {
    Children children = DefaultChildren();
    for (int i = 0; i < count; ++i) {
      children.emplace_back("dropout_window", Window(2 * i, 2 * i + 1));
    }
    DvlAdapter adapter;
    const DvlSimStatus status =
        adapter.ConfigureFromPluginXml(PluginXml(children));
    if (count <= kDvlMaxDropoutWindows) {
      EXPECT_TRUE(status.ok()) << count << " " << status.message;
      const std::vector<DvlStepResult> run = RunSteps(children, 130);
      EXPECT_FALSE(run[0].measurement.has_value());
      EXPECT_TRUE(run[1].measurement.has_value());
      EXPECT_FALSE(run[126].measurement.has_value());
    } else {
      ExpectInvalidConfigNoOutput(children, std::to_string(count));
    }
  }
}

TEST(DvlAdapterTest, MissingPluginElementIsInvalidConfig) {
  DvlAdapter adapter;
  const DvlSimStatus status = adapter.ConfigureFromPluginXml("<not_a_plugin/>");
  EXPECT_EQ(status.code, DvlSimErrorCode::kInvalidConfig);
  EXPECT_FALSE(adapter.Step(StateAt(0)).measurement.has_value());
}

TEST(DvlAdapterTest, UnconfiguredAdapterEmitsNothing) {
  DvlAdapter adapter;
  EXPECT_FALSE(adapter.active());
  const DvlStepResult result = adapter.Step(StateAt(0));
  EXPECT_EQ(result.status.code, DvlSimErrorCode::kInvalidConfig);
  EXPECT_FALSE(result.measurement.has_value());
  EXPECT_EQ(adapter.StepIndexForTesting(), 0);
}

TEST(DvlAdapterTest, ReconfigureReplacesPreviousConfiguration) {
  DvlAdapter adapter;
  ASSERT_TRUE(adapter
                  .ConfigureFromPluginXml(
                      PluginXml(With(DefaultChildren(), "latency_steps", "3")))
                  .ok());
  (void)adapter.Step(StateAt(0));
  EXPECT_FALSE(adapter
                   .ConfigureFromPluginXml(
                       PluginXml(Without(DefaultChildren(), "seabed_z_m")))
                   .ok());
  EXPECT_FALSE(adapter.Step(StateAt(0)).measurement.has_value());
  ASSERT_TRUE(
      adapter.ConfigureFromPluginXml(PluginXml(DefaultChildren())).ok());
  EXPECT_EQ(adapter.StepIndexForTesting(), 0);
  EXPECT_TRUE(adapter.Step(StateAt(0)).measurement.has_value());
}

TEST(DvlAdapterTest, LinkNameIsOptional) {
  DvlAdapter adapter;
  ASSERT_TRUE(
      adapter.ConfigureFromPluginXml(PluginXml(DefaultChildren())).ok());
  EXPECT_FALSE(adapter.link_name().has_value());
  ASSERT_TRUE(adapter
                  .ConfigureFromPluginXml(
                      PluginXml(With(DefaultChildren(), "link_name", "hull")))
                  .ok());
  ASSERT_TRUE(adapter.link_name().has_value());
  EXPECT_EQ(*adapter.link_name(), "hull");
}

TEST(DvlAdapterTest, InvalidStateGivesNoSampleAndStepStillAdvances) {
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const double inf = std::numeric_limits<double>::infinity();
  using Mutator = std::function<void(DvlSimulatorState*)>;
  const std::vector<std::pair<std::string, Mutator>> cases = {
      {"unresolved link", [](auto* s) { s->link_resolved = false; }},
      {"kinematics not ready", [](auto* s) { s->kinematics_ready = false; }},
      {"nan position", [&](auto* s) { s->position_m[2] = nan; }},
      {"inf position", [&](auto* s) { s->position_m[0] = inf; }},
      {"nan orientation", [&](auto* s) { s->orientation_xyzw[3] = nan; }},
      {"nan velocity", [&](auto* s) { s->world_linear_velocity_m_s[1] = nan; }},
      {"inf velocity", [&](auto* s) { s->world_linear_velocity_m_s[0] = inf; }},
      {"bad nanos", [](auto* s) { s->time_nanos = 1000000000; }},
      {"negative nanos", [](auto* s) { s->time_nanos = -1; }},
      {"overflowing velocity",
       [](auto* s) {
         s->world_linear_velocity_m_s = {1.7e308, 1.7e308, 1.7e308};
         s->orientation_xyzw = {0.5, 0.5, 0.5, 0.5};
       }},
      {"zero quaternion", [](auto* s) { s->orientation_xyzw = {0, 0, 0, 0}; }},
      {"scaled quaternion (norm 2)",
       [](auto* s) { s->orientation_xyzw = {0, 0, 0, 2}; }},
      {"slightly non-unit quaternion (norm 1.001)",
       [](auto* s) { s->orientation_xyzw = {0, 0, 0, 1.001}; }},
      {"huge quaternion",
       [](auto* s) { s->orientation_xyzw = {1e200, 1e200, 1e200, 1e200}; }},
  };
  for (const auto& [label, mutate] : cases) {
    DvlAdapter adapter;
    ASSERT_TRUE(
        adapter.ConfigureFromPluginXml(PluginXml(DefaultChildren())).ok());
    ASSERT_TRUE(adapter.Step(StateAt(0)).measurement.has_value());
    DvlSimulatorState bad = StateAt(1);
    mutate(&bad);
    const DvlStepResult result = adapter.Step(bad);
    EXPECT_EQ(result.status.code, DvlSimErrorCode::kInvalidState) << label;
    EXPECT_FALSE(result.measurement.has_value()) << label;
    EXPECT_EQ(result.step, 1) << label;
    EXPECT_EQ(adapter.StepIndexForTesting(), 2) << label;
    const DvlStepResult next = adapter.Step(StateAt(2));
    ASSERT_TRUE(next.measurement.has_value()) << label;
    EXPECT_EQ(next.measurement->health().header().sequence(), 44u) << label;
    ExpectFinite(*next.measurement);
  }
}

TEST(DvlAdapterTest, NonUnitQuaternionIsRejectedNotNormalised) {
  DvlAdapter adapter;
  ASSERT_TRUE(
      adapter.ConfigureFromPluginXml(PluginXml(DefaultChildren())).ok());
  const DvlStepResult rejected =
      adapter.Step(StateAt(0, -40.0, {0, 1, 0}, {0, 0, 0, 2}));
  EXPECT_EQ(rejected.status.code, DvlSimErrorCode::kInvalidState);
  EXPECT_FALSE(rejected.measurement.has_value());
  EXPECT_NE(rejected.status.message.find("unit quaternion"), std::string::npos);
  EXPECT_EQ(adapter.StepIndexForTesting(), 1);

  const double half_sqrt2 = std::sqrt(0.5);
  const DvlStepResult within_tolerance = adapter.Step(StateAt(
      1, -40.0, {0, 1, 0}, {0, 0, half_sqrt2 * (1 + 1e-9), half_sqrt2}));
  EXPECT_TRUE(within_tolerance.status.ok());
  ASSERT_TRUE(within_tolerance.measurement.has_value());
}

TEST(DvlAdapterTest, InvalidStateStepSkipsThePendingLatencyItem) {
  const Children children = With(DefaultChildren(), "latency_steps", "2");
  DvlAdapter adapter;
  ASSERT_TRUE(adapter.ConfigureFromPluginXml(PluginXml(children)).ok());
  EXPECT_FALSE(adapter.Step(StateAt(0)).measurement.has_value());
  EXPECT_FALSE(adapter.Step(StateAt(1)).measurement.has_value());
  DvlSimulatorState bad = StateAt(2);
  bad.kinematics_ready = false;
  const DvlStepResult skipped = adapter.Step(bad);
  EXPECT_EQ(skipped.status.code, DvlSimErrorCode::kInvalidState);
  EXPECT_FALSE(skipped.measurement.has_value());
  // The item that originated at step 0 was due at step 2 and is not
  // replayed. Step 3 emits the item that originated at step 1.
  const DvlStepResult next = adapter.Step(StateAt(3));
  ASSERT_TRUE(next.measurement.has_value());
  EXPECT_EQ(next.measurement->health().header().sequence(), 43u);
  const DvlStepResult after = adapter.Step(StateAt(4));
  EXPECT_FALSE(after.measurement.has_value());
  const DvlStepResult last = adapter.Step(StateAt(5));
  ASSERT_TRUE(last.measurement.has_value());
  EXPECT_EQ(last.measurement->health().header().sequence(), 45u);
}

TEST(DvlAdapterTest, SplitSimTimeNanosNormalizesNanos) {
  int64_t seconds = 0;
  int32_t nanos = 0;
  SplitSimTimeNanos(1999999999LL, &seconds, &nanos);
  EXPECT_EQ(seconds, 1);
  EXPECT_EQ(nanos, 999999999);
  SplitSimTimeNanos(0, &seconds, &nanos);
  EXPECT_EQ(seconds, 0);
  EXPECT_EQ(nanos, 0);
  SplitSimTimeNanos(-1, &seconds, &nanos);
  EXPECT_EQ(seconds, -1);
  EXPECT_EQ(nanos, 999999999);
}

// ---- Fixture 12: scope guard ----------------------------------------------

TEST(DvlAdapterTest, PackageSourcesStayInScope) {
  const char* paths[] = {
      "intrinsic/simulation/gazebo/plugins/dvl/dvl_adapter.h",
      "intrinsic/simulation/gazebo/plugins/dvl/dvl_adapter.cc",
      "intrinsic/simulation/gazebo/plugins/dvl/dvl_plugin.h",
      "intrinsic/simulation/gazebo/plugins/dvl/dvl_plugin.cc",
      "intrinsic/simulation/gazebo/plugins/dvl/dvl_plugin_register.cc",
  };
  const char* forbidden[] = {
      "FakeAltimeter",  "FakeImu",          "FakeIns",        "FakePressure",
      "FakeSurfaceFix", "FakeThruster",     "fake_altimeter", "fake_ins",
      "fake_pressure",  "fake_surface_fix", "gz/sensors",     "gz/transport",
      "Advertise",      "Publish(",         "bathymetry",     "mt19937",
      "random_device",  "absl/random",      "lever_arm",      "CameraPlugin",
  };
  for (const char* path : paths) {
    const std::string source = ReadFile(path);
    ASSERT_FALSE(source.empty()) << path;
    for (const char* needle : forbidden) {
      EXPECT_EQ(source.find(needle), std::string::npos)
          << path << " contains " << needle;
    }
  }
  EXPECT_EQ(ReadFile("intrinsic/simulation/gazebo/plugins/dvl/dvl_adapter.h")
                .find("gz/sim/"),
            std::string::npos);
  EXPECT_EQ(ReadFile("intrinsic/simulation/gazebo/plugins/dvl/dvl_adapter.cc")
                .find("gz/sim/"),
            std::string::npos);
  const std::string registration = ReadFile(
      "intrinsic/simulation/gazebo/plugins/dvl/"
      "dvl_plugin_register.cc");
  EXPECT_NE(registration.find("GZ_ADD_STATIC_PLUGIN_ALIAS(Dvl, "
                              "\"intrinsic::simulation::Dvl\")"),
            std::string::npos);
  // FakeDvl is the only writer of a measurement: no other source constructs
  // a DvlMeasurement or sets a velocity.
  const std::string adapter =
      ReadFile("intrinsic/simulation/gazebo/plugins/dvl/dvl_adapter.cc");
  EXPECT_EQ(adapter.find("set_velocity"), std::string::npos);
  EXPECT_EQ(adapter.find("set_bottom_lock"), std::string::npos);
  EXPECT_EQ(adapter.find("set_altitude"), std::string::npos);
  EXPECT_EQ(adapter.find("set_mode"), std::string::npos);
  EXPECT_EQ(adapter.find("add_sources"), std::string::npos);
}

}  // namespace
}  // namespace intrinsic::simulation
