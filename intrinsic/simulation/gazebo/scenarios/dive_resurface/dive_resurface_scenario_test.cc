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

// Fixed-seed dive and resurface scenario (#134, lock id
// uuv-dive-resurface-scenario-v1). A test-local Mover scripts the model pose
// and link velocity from the committed profile. The Depth, SurfaceFix, Dvl and
// Altimeter plugins observe it, the samples feed FakeEstimatorService, and the
// per-step outputs are checked against the C6 table in README.md. There is no
// Physics, no Hydrodynamics and no production envelope bridge.

#include <array>
#include <charconv>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <memory>
#include <optional>
#include <sstream>
#include <string>
#include <string_view>
#include <system_error>
#include <vector>

#include "google/protobuf/descriptor.h"
#include "google/protobuf/timestamp.pb.h"
#include "gtest/gtest.h"
#include "gz/math/Pose3.hh"
#include "gz/math/Vector3.hh"
#include "gz/sim/Entity.hh"
#include "gz/sim/EntityComponentManager.hh"
#include "gz/sim/Server.hh"
#include "gz/sim/ServerConfig.hh"
#include "gz/sim/System.hh"
#include "gz/sim/Util.hh"
#include "gz/sim/components/LinearVelocity.hh"
#include "gz/sim/components/Link.hh"
#include "gz/sim/components/Model.hh"
#include "gz/sim/components/Name.hh"
#include "gz/sim/components/Pose.hh"
#include "intrinsic/embodiment/frame_policy.h"
#include "intrinsic/estimation/estimator.pb.h"
#include "intrinsic/estimation/fake_estimator_service.h"
#include "intrinsic/hardware/marine/altimeter.pb.h"
#include "intrinsic/hardware/marine/altimeter_policy.h"
#include "intrinsic/hardware/marine/dvl.pb.h"
#include "intrinsic/hardware/marine/dvl_policy.h"
#include "intrinsic/hardware/marine/measurement_health.pb.h"
#include "intrinsic/hardware/marine/measurement_health_policy.h"
#include "intrinsic/hardware/marine/pressure_depth.pb.h"
#include "intrinsic/hardware/marine/pressure_depth_policy.h"
#include "intrinsic/hardware/marine/surface_fix.pb.h"
#include "intrinsic/hardware/marine/surface_fix_policy.h"
#include "intrinsic/simulation/gazebo/plugins/altimeter/altimeter_plugin.h"
#include "intrinsic/simulation/gazebo/plugins/depth/depth_plugin.h"
#include "intrinsic/simulation/gazebo/plugins/dvl/dvl_plugin.h"
#include "intrinsic/simulation/gazebo/plugins/surface_fix/surface_fix_plugin.h"
#include "intrinsic/vehicle/proto/vehicle_state.pb.h"
#include "sdf/Element.hh"
#include "sdf/Plugin.hh"
#include "sdf/Root.hh"
#include "sdf/World.hh"

namespace intrinsic::simulation {
namespace {

using ::intrinsic::estimation::FakeEstimatorService;
using ::intrinsic::hardware::marine::AltimeterAssessment;
using ::intrinsic::hardware::marine::AltimeterError;
using ::intrinsic::hardware::marine::AltimeterMeasurementView;
using ::intrinsic::hardware::marine::AssessAltimeter;
using ::intrinsic::hardware::marine::AssessDvl;
using ::intrinsic::hardware::marine::AssessPressureDepth;
using ::intrinsic::hardware::marine::AssessSurfaceFix;
using ::intrinsic::hardware::marine::DvlAssessment;
using ::intrinsic::hardware::marine::DvlError;
using ::intrinsic::hardware::marine::DvlMeasurementView;
using ::intrinsic::hardware::marine::MeasurementHealthView;
using ::intrinsic::hardware::marine::MeasurementStateKind;
using ::intrinsic::hardware::marine::PressureDepthAssessment;
using ::intrinsic::hardware::marine::PressureDepthError;
using ::intrinsic::hardware::marine::PressureDepthMeasurementView;
using ::intrinsic::hardware::marine::SourceHealthView;
using ::intrinsic::hardware::marine::SurfaceFixAssessment;
using ::intrinsic::hardware::marine::SurfaceFixError;
using ::intrinsic::hardware::marine::SurfaceFixView;
using ::intrinsic_proto::estimation::EstimatorConfig;
using ::intrinsic_proto::estimation::EstimatorResult;
using ::intrinsic_proto::estimation::MeasurementEnvelope;
using ::intrinsic_proto::estimation::MeasurementKind;
using ::intrinsic_proto::estimation::REJECT_REASON_NONE;
using ::intrinsic_proto::hardware::marine::AltimeterMeasurement;
using ::intrinsic_proto::hardware::marine::DvlMeasurement;
using ::intrinsic_proto::hardware::marine::MeasurementHealth;
using ::intrinsic_proto::hardware::marine::PressureDepthMeasurement;
using SurfaceFixProto = ::intrinsic_proto::hardware::marine::SurfaceFix;
using ::intrinsic_proto::vehicle::NAVIGATION_MODE_AIDED;
using ::intrinsic_proto::vehicle::NAVIGATION_MODE_DEAD_RECKONING;
using ::intrinsic_proto::vehicle::NAVIGATION_MODE_FAULTED;
using ::intrinsic_proto::vehicle::NAVIGATION_MODE_INITIALIZING;

// ---- Expected outputs (C6). Same values as the README table. ---------------

constexpr char kPackageDir[] =
    "intrinsic/simulation/gazebo/scenarios/dive_resurface/";
constexpr char kWorldFile[] = "dive_resurface.world.sdf";
constexpr char kSensorsFile[] = "dive_resurface_sensors.sdf";
constexpr char kProfileFile[] = "dive_resurface_profile.txt";

constexpr int kSteps = 400;
constexpr int kMaxRunCalls = 420;
constexpr int64_t kStepNs = 125000000;

constexpr int kDropoutStart = 180;
constexpr int kDropoutEnd = 200;

// The surface fix is present for [0, 84) and [315, 400). k = 83 and k = 315
// have d = 0.5 exactly and emit. k = 84 and k = 314 have d = 0.5625 and do not.
constexpr int kFixGateClosesAt = 84;
constexpr int kFixGateReopensAt = 315;

constexpr double kEps = 1e-9;
constexpr double kDepthTolM = 0.01 + kEps;
constexpr double kFixXyTolM = 0.1 + kEps;
constexpr double kDvlTolM_S = 0.005 + kEps;
constexpr double kDvlAltitudeTolM = 0.005 + kEps;
constexpr double kAltimeterTolM = 0.01 + kEps;
constexpr double kHoldDepthM = 5.25;
constexpr double kSurfacedDepthM = 0.25;
constexpr double kSurfacedZM = -0.25;
constexpr double kSeabedZM = -30.0;
constexpr double kDiveVelocityZ_M_S = -0.5;
constexpr double kAscendVelocityZ_M_S = 0.5;

constexpr int kNumSensors = 4;
enum SensorIndex { kDepth = 0, kSurfaceFix = 1, kDvl = 2, kAltimeter = 3 };

bool InDropout(int k) { return k >= kDropoutStart && k < kDropoutEnd; }

bool ExpectFix(int k) { return k < kFixGateClosesAt || k >= kFixGateReopensAt; }

int ExpectedMode(int k) {
  return InDropout(k) ? NAVIGATION_MODE_DEAD_RECKONING : NAVIGATION_MODE_AIDED;
}

// ---- Profile (C4) -----------------------------------------------------------

struct Segment {
  std::string label;
  int64_t start = 0;
  int64_t end = 0;
  double depth_start_m = 0;
  double rate_m_per_step = 0;
};

struct Profile {
  double dt_s = 0;
  double surface_z_m = 0;
  double seabed_z_m = 0;
  int64_t steps = 0;
  std::vector<Segment> segments;
};

std::vector<std::string_view> Tokens(std::string_view line) {
  std::vector<std::string_view> tokens;
  size_t i = 0;
  while (i < line.size()) {
    while (i < line.size() &&
           (line[i] == ' ' || line[i] == '\t' || line[i] == '\r')) {
      ++i;
    }
    const size_t begin = i;
    while (i < line.size() && line[i] != ' ' && line[i] != '\t' &&
           line[i] != '\r') {
      ++i;
    }
    if (i > begin) {
      tokens.push_back(line.substr(begin, i - begin));
    }
  }
  return tokens;
}

bool ParseInt(std::string_view token, int64_t* value) {
  const char* const last = token.data() + token.size();
  const auto result = std::from_chars(token.data(), last, *value);
  return result.ec == std::errc() && result.ptr == last;
}

bool ParseDouble(std::string_view token, double* value) {
  const char* const last = token.data() + token.size();
  const auto result = std::from_chars(token.data(), last, *value);
  return result.ec == std::errc() && result.ptr == last &&
         std::isfinite(*value);
}

// Strict parser. Any malformed line, wrong token count, gap or overlap fails.
// There is no recovery.
std::optional<Profile> ParseProfile(std::string_view text, std::string* error) {
  Profile profile;
  bool have_config = false;
  size_t pos = 0;
  int line_number = 0;
  auto fail = [&](const std::string& message) -> std::optional<Profile> {
    *error = "line " + std::to_string(line_number) + ": " + message;
    return std::nullopt;
  };
  while (pos < text.size()) {
    const size_t newline = text.find('\n', pos);
    const size_t end =
        newline == std::string_view::npos ? text.size() : newline;
    const std::string_view line = text.substr(pos, end - pos);
    pos = newline == std::string_view::npos ? text.size() : newline + 1;
    ++line_number;
    const std::vector<std::string_view> tokens = Tokens(line);
    if (tokens.empty()) {
      return fail("blank line");
    }
    if (tokens[0][0] == '#') {
      continue;
    }
    if (tokens[0] == "CONFIG") {
      if (have_config) {
        return fail("second CONFIG");
      }
      if (tokens.size() != 5) {
        return fail("CONFIG needs 4 values");
      }
      if (!ParseDouble(tokens[1], &profile.dt_s) ||
          !ParseDouble(tokens[2], &profile.surface_z_m) ||
          !ParseDouble(tokens[3], &profile.seabed_z_m) ||
          !ParseInt(tokens[4], &profile.steps)) {
        return fail("CONFIG has a non-numeric token");
      }
      if (profile.dt_s <= 0 || profile.steps <= 0) {
        return fail("CONFIG dt_s and steps must be positive");
      }
      have_config = true;
    } else if (tokens[0] == "SEGMENT") {
      if (tokens.size() != 6) {
        return fail("SEGMENT needs 5 values");
      }
      Segment segment;
      segment.label = std::string(tokens[1]);
      if (!ParseInt(tokens[2], &segment.start) ||
          !ParseInt(tokens[3], &segment.end) ||
          !ParseDouble(tokens[4], &segment.depth_start_m) ||
          !ParseDouble(tokens[5], &segment.rate_m_per_step)) {
        return fail("SEGMENT has a non-numeric token");
      }
      if (segment.end <= segment.start) {
        return fail("SEGMENT end must exceed start");
      }
      const int64_t expected_start =
          profile.segments.empty() ? 0 : profile.segments.back().end;
      if (segment.start > expected_start) {
        return fail("gap before SEGMENT " + segment.label);
      }
      if (segment.start < expected_start) {
        return fail("overlap at SEGMENT " + segment.label);
      }
      profile.segments.push_back(std::move(segment));
    } else {
      return fail("unknown line");
    }
  }
  if (!have_config) {
    *error = "missing CONFIG";
    return std::nullopt;
  }
  if (profile.segments.empty() ||
      profile.segments.back().end != profile.steps) {
    *error = "segments do not cover [0, steps)";
    return std::nullopt;
  }
  return profile;
}

const Segment& SegmentAt(const Profile& profile, int64_t k) {
  for (const Segment& segment : profile.segments) {
    if (k >= segment.start && k < segment.end) {
      return segment;
    }
  }
  ADD_FAILURE() << "step " << k << " outside the profile";
  return profile.segments.back();
}

double DepthAt(const Profile& profile, int64_t k) {
  const Segment& s = SegmentAt(profile, k);
  return s.depth_start_m + s.rate_m_per_step * static_cast<double>(k - s.start);
}

double ZAt(const Profile& profile, int64_t k) {
  return profile.surface_z_m - DepthAt(profile, k);
}

double WorldVelocityZAt(const Profile& profile, int64_t k) {
  return -SegmentAt(profile, k).rate_m_per_step / profile.dt_s;
}

std::string ReadFileOrEmpty(const std::string& path) {
  std::ifstream in(path);
  if (!in) {
    return "";
  }
  std::ostringstream buffer;
  buffer << in.rdbuf();
  return buffer.str();
}

// Bazel runs tests with TEST_SRCDIR set. The main repository is _main.
std::string RunfilesPath(const std::string& name) {
  const char* src = std::getenv("TEST_SRCDIR");
  if (src == nullptr) {
    return "";
  }
  const std::string suffix = std::string(kPackageDir) + name;
  const std::string primary = std::string(src) + "/_main/" + suffix;
  if (std::ifstream(primary)) {
    return primary;
  }
  const char* workspace = std::getenv("TEST_WORKSPACE");
  if (workspace != nullptr) {
    const std::string legacy =
        std::string(src) + "/" + workspace + "/" + suffix;
    if (std::ifstream(legacy)) {
      return legacy;
    }
  }
  return std::string(src) + "/" + suffix;
}

// ---- Mover (D1-A, R1) -------------------------------------------------------

// Sets the model pose and the link world linear velocity from the profile in
// PreUpdate. k comes from simulation time, never from a private counter. The
// PostUpdate half only records, so the world pose is read after the pose was
// written and never from another system.
class Mover final : public gz::sim::System,
                    public gz::sim::ISystemPreUpdate,
                    public gz::sim::ISystemPostUpdate {
 public:
  explicit Mover(const Profile* profile) : profile_(profile) {}

  void PreUpdate(const gz::sim::UpdateInfo& info,
                 gz::sim::EntityComponentManager& ecm) override {
    if (info.paused) {
      return;
    }
    const int64_t ns =
        std::chrono::duration_cast<std::chrono::nanoseconds>(info.simTime)
            .count();
    if (ns % kStepNs != 0) {
      ++inexact_time_count_;
      return;
    }
    const int64_t k = ns / kStepNs - 1;
    last_k_ = k;
    if (k < 0 || k >= profile_->steps) {
      ++out_of_range_count_;
      return;
    }
    const gz::sim::Entity model = ecm.EntityByComponents(
        gz::sim::components::Name("uuv"), gz::sim::components::Model());
    const gz::sim::Entity link = ecm.EntityByComponents(
        gz::sim::components::Name("hull"), gz::sim::components::Link());
    if (model == gz::sim::kNullEntity || link == gz::sim::kNullEntity) {
      return;
    }
    ecm.SetComponentData<gz::sim::components::Pose>(
        model, gz::math::Pose3d(0.0, 0.0, ZAt(*profile_, k), 0.0, 0.0, 0.0));
    ecm.SetComponentData<gz::sim::components::WorldLinearVelocity>(
        link, gz::math::Vector3d(0.0, 0.0, WorldVelocityZAt(*profile_, k)));
  }

  void PostUpdate(const gz::sim::UpdateInfo& info,
                  const gz::sim::EntityComponentManager& ecm) override {
    if (info.paused) {
      return;
    }
    const gz::sim::Entity link = ecm.EntityByComponents(
        gz::sim::components::Name("hull"), gz::sim::components::Link());
    if (link == gz::sim::kNullEntity) {
      return;
    }
    const gz::math::Pose3d pose = gz::sim::worldPose(link, ecm);
    last_pose_ = {pose.Pos().X(), pose.Pos().Y(), pose.Pos().Z(),
                  pose.Rot().X(), pose.Rot().Y(), pose.Rot().Z(),
                  pose.Rot().W()};
    last_sim_time_ns_ =
        std::chrono::duration_cast<std::chrono::nanoseconds>(info.simTime)
            .count();
  }

  // Read only after Server::Run has returned.
  int64_t last_k() const { return last_k_; }
  const std::array<double, 7>& last_pose() const { return last_pose_; }
  int64_t last_sim_time_ns() const { return last_sim_time_ns_; }
  int inexact_time_count() const { return inexact_time_count_; }
  int out_of_range_count() const { return out_of_range_count_; }

 private:
  const Profile* profile_;
  int64_t last_k_ = -1000;
  std::array<double, 7> last_pose_ = {};
  int64_t last_sim_time_ns_ = -1;
  int inexact_time_count_ = 0;
  int out_of_range_count_ = 0;
};

// ---- Oracles ----------------------------------------------------------------

struct HealthStorage {
  std::vector<double> covariance;
  std::vector<SourceHealthView> sources;
};

MeasurementHealthView MakeHealthView(const MeasurementHealth& health,
                                     std::string_view expected_frame,
                                     HealthStorage* storage) {
  MeasurementHealthView view;
  view.header_present = health.has_header();
  view.frame_id = health.header().frame_id();
  view.expected_frame_present = true;
  view.expected_frame = expected_frame;
  view.source_time_present = health.header().has_source_time();
  view.source_time = ::intrinsic::embodiment::ClockReading{
      health.header().source_time().seconds(),
      health.header().source_time().nanos()};
  view.receive_time_present = health.header().has_receive_time();
  view.receive_time = ::intrinsic::embodiment::ClockReading{
      health.header().receive_time().seconds(),
      health.header().receive_time().nanos()};
  view.header_validity_present = health.header().has_validity();
  view.header_validity_state = health.header().validity().state();
  view.state_present = health.has_state();
  view.state = health.state();
  view.quality_present = health.has_quality();
  view.quality = health.quality();
  view.covariance_present = health.has_covariance();
  storage->covariance.assign(health.covariance().values().begin(),
                             health.covariance().values().end());
  view.covariance = storage->covariance;
  for (const auto& source : health.sources()) {
    storage->sources.push_back(SourceHealthView{
        source.source_id(), source.has_validity(), source.validity().state()});
  }
  view.sources = storage->sources;
  return view;
}

PressureDepthAssessment AssessDepth(const PressureDepthMeasurement& sample) {
  HealthStorage storage;
  PressureDepthMeasurementView view;
  view.health = MakeHealthView(sample.health(), "body", &storage);
  view.pressure_present = sample.has_pressure_pa();
  view.pressure_pa = sample.pressure_pa();
  view.depth_present = sample.has_depth_m();
  view.depth_m = sample.depth_m();
  view.provenance_present = sample.has_depth_provenance();
  view.depth_provenance = static_cast<int>(sample.depth_provenance());
  view.density_present = sample.has_fluid_density_kg_m3();
  view.fluid_density_kg_m3 = sample.fluid_density_kg_m3();
  return AssessPressureDepth(view);
}

SurfaceFixAssessment AssessFix(const SurfaceFixProto& sample) {
  HealthStorage storage;
  SurfaceFixView view;
  view.health = MakeHealthView(sample.health(), "world_enu", &storage);
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
  return AssessSurfaceFix(view);
}

DvlAssessment AssessDvlSample(const DvlMeasurement& sample) {
  HealthStorage storage;
  DvlMeasurementView view;
  view.health = MakeHealthView(sample.health(), "body", &storage);
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
  return AssessDvl(view);
}

AltimeterAssessment AssessAltimeterSample(const AltimeterMeasurement& sample) {
  HealthStorage storage;
  AltimeterMeasurementView view;
  view.health = MakeHealthView(sample.health(), "body", &storage);
  view.range_present = sample.has_range_m();
  view.range_m = sample.range_m();
  view.beam_present = sample.has_beam_id();
  view.beam_id = sample.beam_id();
  view.min_present = sample.has_min_range_m();
  view.min_range_m = sample.min_range_m();
  view.max_present = sample.has_max_range_m();
  view.max_range_m = sample.max_range_m();
  view.has_return_present = sample.has_has_return();
  view.has_return = sample.has_return();
  return AssessAltimeter(view);
}

// ---- Estimator glue (D3-A, R2, R3). Test only, not a production bridge. -----

template <typename Sample>
MeasurementEnvelope MakeEnvelope(const Sample& sample,
                                 const std::string& source_id,
                                 MeasurementKind kind) {
  MeasurementEnvelope envelope;
  *envelope.mutable_header() = sample.health().header();
  envelope.set_source_id(source_id);
  envelope.set_kind(kind);
  envelope.set_payload_type(std::string(sample.GetDescriptor()->full_name()));
  envelope.set_payload(sample.SerializeAsString());
  return envelope;
}

google::protobuf::Timestamp PredictTime(int64_t k) {
  const int64_t ns = (k + 1) * kStepNs;
  google::protobuf::Timestamp time;
  time.set_seconds(ns / 1000000000);
  time.set_nanos(static_cast<int32_t>(ns % 1000000000));
  return time;
}

// ---- Scenario run -----------------------------------------------------------

struct StepRecord {
  int64_t mover_k = 0;
  int64_t sim_time_ns = 0;
  std::array<double, 7> pose = {};
  std::array<int64_t, kNumSensors> adapter_step = {};
  std::array<int64_t, kNumSensors> result_step = {};
  std::array<bool, kNumSensors> status_ok = {};
  std::optional<PressureDepthMeasurement> depth;
  std::optional<SurfaceFixProto> fix;
  std::optional<DvlMeasurement> dvl;
  std::optional<AltimeterMeasurement> altimeter;
  std::vector<int> ingest_reasons;
  int predict_reason = -1;
  int mode = -1;

  int PresentCount() const {
    return static_cast<int>(depth.has_value()) +
           static_cast<int>(fix.has_value()) +
           static_cast<int>(dvl.has_value()) +
           static_cast<int>(altimeter.has_value());
  }
};

struct ScenarioRun {
  bool setup_ok = false;
  int initialize_reason = -1;
  int initial_mode = -1;
  int run_calls = 0;
  int inexact_time_count = 0;
  int out_of_range_count = 0;
  std::vector<StepRecord> steps;
};

class DiveResurfaceEnvironment : public testing::Environment {
 public:
  void SetUp() override {
    // Empty server config skips the default Physics, UserCommands, and
    // SceneBroadcaster systems. HOME avoids a write to /.gz.
    setenv("GZ_SIM_SERVER_CONFIG_PATH", "", 1);
    setenv("HOME", "/tmp", 1);
    setenv("GZ_PARTITION", "intrinsic_dive_resurface_test", 1);
  }
};

testing::Environment* const kPartitionEnvironment =
    testing::AddGlobalTestEnvironment(new DiveResurfaceEnvironment);

std::unique_ptr<gz::sim::Server> MakeServer() {
  gz::sim::ServerConfig config;
  config.SetHeadlessRendering(true);
  if (!config.SetSdfFile(RunfilesPath(kWorldFile))) {
    return nullptr;
  }
  auto server = std::make_unique<gz::sim::Server>(config);
  server->SetUpdatePeriod(std::chrono::steady_clock::duration::zero());
  return server;
}

// Adds one sensor plugin to model "uuv" with the element parsed from the
// sensors file, the way AddDepth does in depth_plugin_test.cc.
template <typename Plugin>
std::shared_ptr<Plugin> AddSensor(gz::sim::Server* server,
                                  const sdf::Plugins& plugins, size_t index,
                                  std::string_view expected_name) {
  auto plugin = std::make_shared<Plugin>();
  const std::optional<gz::sim::Entity> model = server->EntityByName("uuv");
  EXPECT_TRUE(model.has_value());
  EXPECT_LT(index, plugins.size());
  if (!model.has_value() || index >= plugins.size()) {
    return plugin;
  }
  EXPECT_EQ(plugins[index].Name(), expected_name);
  std::shared_ptr<const sdf::Element> element = plugins[index].Element();
  EXPECT_TRUE(server
                  ->AddSystem(std::static_pointer_cast<gz::sim::System>(plugin),
                              model, element)
                  .value_or(false))
      << expected_name;
  return plugin;
}

ScenarioRun RunScenario(const Profile& profile) {
  ScenarioRun run;
  std::unique_ptr<gz::sim::Server> server = MakeServer();
  EXPECT_TRUE(server != nullptr);
  if (server == nullptr) {
    return run;
  }
  sdf::Root root;
  const sdf::Errors errors = root.Load(RunfilesPath(kSensorsFile));
  EXPECT_TRUE(errors.empty()) << (errors.empty() ? "" : errors[0].Message());
  if (!errors.empty() || root.WorldCount() != 1) {
    return run;
  }
  const sdf::Plugins& plugins = root.WorldByIndex(0)->Plugins();
  EXPECT_EQ(plugins.size(), 4u);
  if (plugins.size() != 4u) {
    return run;
  }

  // The Mover and all four plugins are added before the first Run.
  auto mover = std::make_shared<Mover>(&profile);
  EXPECT_TRUE(server->AddSystem(mover).value_or(false));
  const auto depth = AddSensor<Depth>(server.get(), plugins, 0,
                                      "intrinsic::simulation::Depth");
  const auto fix = AddSensor<SurfaceFix>(server.get(), plugins, 1,
                                         "intrinsic::simulation::SurfaceFix");
  const auto dvl =
      AddSensor<Dvl>(server.get(), plugins, 2, "intrinsic::simulation::Dvl");
  const auto altimeter = AddSensor<Altimeter>(
      server.get(), plugins, 3, "intrinsic::simulation::Altimeter");

  FakeEstimatorService estimator;
  EstimatorConfig config;
  config.set_seed(42);
  run.initialize_reason = estimator.Initialize(config).reason();
  run.initial_mode = estimator.GetState().state().mode();

  for (int call = 0;
       call < kMaxRunCalls && static_cast<int>(run.steps.size()) < kSteps;
       ++call) {
    const int64_t before = depth->AdapterForTesting().StepIndexForTesting();
    EXPECT_TRUE(server->Run(/*_blocking=*/true, 1, /*_paused=*/false));
    ++run.run_calls;
    if (depth->AdapterForTesting().StepIndexForTesting() != before + 1) {
      continue;
    }

    StepRecord record;
    record.mover_k = mover->last_k();
    record.sim_time_ns = mover->last_sim_time_ns();
    record.pose = mover->last_pose();
    record.adapter_step = {
        depth->AdapterForTesting().StepIndexForTesting(),
        fix->AdapterForTesting().StepIndexForTesting(),
        dvl->AdapterForTesting().StepIndexForTesting(),
        altimeter->AdapterForTesting().StepIndexForTesting()};
    record.result_step = {depth->LastResultForTesting().step,
                          fix->LastResultForTesting().step,
                          dvl->LastResultForTesting().step,
                          altimeter->LastResultForTesting().step};
    record.status_ok = {depth->LastResultForTesting().status.ok(),
                        fix->LastResultForTesting().status.ok(),
                        dvl->LastResultForTesting().status.ok(),
                        altimeter->LastResultForTesting().status.ok()};
    record.depth = depth->LastResultForTesting().measurement;
    record.fix = fix->LastResultForTesting().measurement;
    record.dvl = dvl->LastResultForTesting().measurement;
    record.altimeter = altimeter->LastResultForTesting().measurement;

    // Ingest in the order Depth, SurfaceFix, Dvl, Altimeter, then Predict.
    if (record.depth.has_value()) {
      record.ingest_reasons.push_back(
          estimator
              .Ingest(MakeEnvelope(*record.depth, "depth",
                                   MeasurementKind::MEASUREMENT_KIND_PRESSURE))
              .reason());
    }
    if (record.fix.has_value()) {
      record.ingest_reasons.push_back(
          estimator
              .Ingest(
                  MakeEnvelope(*record.fix, "surface_fix",
                               MeasurementKind::MEASUREMENT_KIND_SURFACE_FIX))
              .reason());
    }
    if (record.dvl.has_value()) {
      record.ingest_reasons.push_back(
          estimator
              .Ingest(MakeEnvelope(*record.dvl, "dvl",
                                   MeasurementKind::MEASUREMENT_KIND_DVL))
              .reason());
    }
    if (record.altimeter.has_value()) {
      record.ingest_reasons.push_back(
          estimator
              .Ingest(MakeEnvelope(*record.altimeter, "altimeter",
                                   MeasurementKind::MEASUREMENT_KIND_ALTIMETER))
              .reason());
    }
    record.predict_reason =
        estimator.Predict(PredictTime(record.mover_k)).reason();
    record.mode = estimator.GetState().state().mode();
    run.steps.push_back(std::move(record));
  }
  run.inexact_time_count = mover->inexact_time_count();
  run.out_of_range_count = mover->out_of_range_count();
  run.setup_ok = true;
  return run;
}

Profile LoadProfileOrFail() {
  std::string error;
  const std::optional<Profile> profile =
      ParseProfile(ReadFileOrEmpty(RunfilesPath(kProfileFile)), &error);
  EXPECT_TRUE(profile.has_value()) << error;
  return profile.value_or(Profile());
}

class DiveResurfaceScenarioTest : public testing::Test {
 protected:
  static void SetUpTestSuite() {
    profile_ = new Profile(LoadProfileOrFail());
    run_ = new ScenarioRun(RunScenario(*profile_));
  }

  static void TearDownTestSuite() {
    delete run_;
    delete profile_;
    run_ = nullptr;
    profile_ = nullptr;
  }

  static const Profile& profile() { return *profile_; }
  static const ScenarioRun& run() { return *run_; }

  // True when T1 holds. Every later check assumes it, so a misalignment is
  // reported once and the profile and windows are never shifted to hide it.
  static bool Aligned() {
    if (!run().setup_ok || run().steps.size() != static_cast<size_t>(kSteps) ||
        run().inexact_time_count != 0 || run().out_of_range_count != 0) {
      return false;
    }
    for (int k = 0; k < kSteps; ++k) {
      const StepRecord& s = run().steps[k];
      if (s.mover_k != k) {
        return false;
      }
      for (int i = 0; i < kNumSensors; ++i) {
        if (s.adapter_step[i] != s.adapter_step[0] || s.result_step[i] != k ||
            !s.status_ok[i]) {
          return false;
        }
      }
    }
    return true;
  }

 private:
  static Profile* profile_;
  static ScenarioRun* run_;
};

Profile* DiveResurfaceScenarioTest::profile_ = nullptr;
ScenarioRun* DiveResurfaceScenarioTest::run_ = nullptr;

int64_t TimeNs(const google::protobuf::Timestamp& time) {
  return time.seconds() * 1000000000LL + time.nanos();
}

std::string BytesOrAbsent(const StepRecord& s, int sensor) {
  switch (sensor) {
    case kDepth:
      return s.depth.has_value() ? s.depth->SerializeAsString() : "absent";
    case kSurfaceFix:
      return s.fix.has_value() ? s.fix->SerializeAsString() : "absent";
    case kDvl:
      return s.dvl.has_value() ? s.dvl->SerializeAsString() : "absent";
    default:
      return s.altimeter.has_value() ? s.altimeter->SerializeAsString()
                                     : "absent";
  }
}

// ---- Profile parser ---------------------------------------------------------

constexpr char kValidProfile[] =
    "# comment\n"
    "CONFIG 0.125 0 -30 400\n"
    "SEGMENT A 0 80 0.25 0\n"
    "SEGMENT B 80 160 0.3125 0.0625\n"
    "SEGMENT C 160 400 5.25 0\n";

TEST(ProfileParserTest, ParsesTheCommittedProfile) {
  std::string error;
  const std::optional<Profile> profile =
      ParseProfile(ReadFileOrEmpty(RunfilesPath(kProfileFile)), &error);
  ASSERT_TRUE(profile.has_value()) << error;
  EXPECT_EQ(profile->dt_s, 0.125);
  EXPECT_EQ(profile->surface_z_m, 0.0);
  EXPECT_EQ(profile->seabed_z_m, kSeabedZM);
  EXPECT_EQ(profile->steps, kSteps);
  ASSERT_EQ(profile->segments.size(), 5u);
  const char* const labels[] = {"SURFACE_HOLD", "DIVE", "HOLD", "ASCEND",
                                "SURFACED"};
  const int64_t starts[] = {0, 80, 160, 240, 320};
  const int64_t ends[] = {80, 160, 240, 320, 400};
  const double depth_starts[] = {0.25, 0.3125, 5.25, 5.1875, 0.25};
  const double rates[] = {0, 0.0625, 0, -0.0625, 0};
  for (int i = 0; i < 5; ++i) {
    EXPECT_EQ(profile->segments[i].label, labels[i]) << i;
    EXPECT_EQ(profile->segments[i].start, starts[i]) << i;
    EXPECT_EQ(profile->segments[i].end, ends[i]) << i;
    EXPECT_EQ(profile->segments[i].depth_start_m, depth_starts[i]) << i;
    EXPECT_EQ(profile->segments[i].rate_m_per_step, rates[i]) << i;
  }
  // The gate boundary values are exact in double.
  EXPECT_EQ(DepthAt(*profile, 83), 0.5);
  EXPECT_EQ(DepthAt(*profile, 84), 0.5625);
  EXPECT_EQ(DepthAt(*profile, 314), 0.5625);
  EXPECT_EQ(DepthAt(*profile, 315), 0.5);
  EXPECT_EQ(DepthAt(*profile, 399), kSurfacedDepthM);
  EXPECT_EQ(WorldVelocityZAt(*profile, 100), kDiveVelocityZ_M_S);
  EXPECT_EQ(WorldVelocityZAt(*profile, 250), kAscendVelocityZ_M_S);
  EXPECT_EQ(WorldVelocityZAt(*profile, 200), 0.0);
}

TEST(ProfileParserTest, AcceptsAWellFormedProfile) {
  std::string error;
  const std::optional<Profile> profile = ParseProfile(kValidProfile, &error);
  ASSERT_TRUE(profile.has_value()) << error;
  EXPECT_EQ(profile->segments.size(), 3u);
}

TEST(ProfileParserTest, RejectsMalformedProfiles) {
  struct Case {
    const char* name;
    std::string text;
  };
  const std::vector<Case> cases = {
      {"missing CONFIG", "SEGMENT A 0 80 0.25 0\nSEGMENT B 80 400 0.3 0\n"},
      {"gap",
       "CONFIG 0.125 0 -30 400\nSEGMENT A 0 80 0.25 0\n"
       "SEGMENT B 90 400 0.3 0\n"},
      {"overlap",
       "CONFIG 0.125 0 -30 400\nSEGMENT A 0 80 0.25 0\n"
       "SEGMENT B 70 400 0.3 0\n"},
      {"wrong token count in SEGMENT",
       "CONFIG 0.125 0 -30 400\nSEGMENT A 0 80 0.25\n"
       "SEGMENT B 80 400 0.3 0\n"},
      {"extra token in SEGMENT",
       "CONFIG 0.125 0 -30 400\nSEGMENT A 0 80 0.25 0 9\n"
       "SEGMENT B 80 400 0.3 0\n"},
      {"wrong token count in CONFIG",
       "CONFIG 0.125 0 -30\nSEGMENT A 0 400 0.25 0\n"},
      {"non-numeric depth",
       "CONFIG 0.125 0 -30 400\nSEGMENT A 0 80 deep 0\n"
       "SEGMENT B 80 400 0.3 0\n"},
      {"non-numeric step",
       "CONFIG 0.125 0 -30 400\nSEGMENT A 0 8x 0.25 0\n"
       "SEGMENT B 80 400 0.3 0\n"},
      {"non-numeric CONFIG",
       "CONFIG 0.125 zero -30 400\nSEGMENT A 0 400 0.25 0\n"},
      {"second CONFIG",
       "CONFIG 0.125 0 -30 400\nCONFIG 0.125 0 -30 400\n"
       "SEGMENT A 0 400 0.25 0\n"},
      {"does not cover steps",
       "CONFIG 0.125 0 -30 400\nSEGMENT A 0 80 0.25 0\n"},
      {"does not start at zero",
       "CONFIG 0.125 0 -30 400\nSEGMENT A 10 400 0.25 0\n"},
      {"unknown line", "CONFIG 0.125 0 -30 400\nOTHER A\n"},
      {"blank line", "CONFIG 0.125 0 -30 400\n\nSEGMENT A 0 400 0.25 0\n"},
  };
  for (const Case& c : cases) {
    std::string error;
    EXPECT_FALSE(ParseProfile(c.text, &error).has_value()) << c.name;
    EXPECT_FALSE(error.empty()) << c.name;
  }
}

// ---- T1-T9 ------------------------------------------------------------------

TEST_F(DiveResurfaceScenarioTest, T1_Alignment) {
  ASSERT_TRUE(run().setup_ok);
  ASSERT_EQ(run().steps.size(), static_cast<size_t>(kSteps))
      << "run_calls=" << run().run_calls;
  EXPECT_LE(run().run_calls, kMaxRunCalls);
  EXPECT_EQ(run().inexact_time_count, 0);
  EXPECT_EQ(run().out_of_range_count, 0);
  for (int k = 0; k < kSteps; ++k) {
    const StepRecord& s = run().steps[k];
    EXPECT_EQ(s.mover_k, k) << k;
    EXPECT_EQ(s.sim_time_ns, (k + 1) * kStepNs) << k;
    for (int i = 0; i < kNumSensors; ++i) {
      EXPECT_EQ(s.adapter_step[i], s.adapter_step[0]) << k << " sensor " << i;
      EXPECT_EQ(s.result_step[i], s.mover_k) << k << " sensor " << i;
      EXPECT_TRUE(s.status_ok[i]) << k << " sensor " << i;
    }
    const int64_t expected_ns = (k + 1) * kStepNs;
    if (s.depth.has_value()) {
      EXPECT_EQ(TimeNs(s.depth->health().header().source_time()), expected_ns)
          << k;
    }
    if (s.fix.has_value()) {
      EXPECT_EQ(TimeNs(s.fix->health().header().source_time()), expected_ns)
          << k;
    }
    if (s.dvl.has_value()) {
      EXPECT_EQ(TimeNs(s.dvl->health().header().source_time()), expected_ns)
          << k;
    }
    if (s.altimeter.has_value()) {
      EXPECT_EQ(TimeNs(s.altimeter->health().header().source_time()),
                expected_ns)
          << k;
    }
    EXPECT_EQ(s.pose[0], 0.0) << k;
    EXPECT_EQ(s.pose[1], 0.0) << k;
    EXPECT_EQ(s.pose[2], ZAt(profile(), k)) << k;
    EXPECT_EQ(s.pose[3], 0.0) << k;
    EXPECT_EQ(s.pose[4], 0.0) << k;
    EXPECT_EQ(s.pose[5], 0.0) << k;
    EXPECT_EQ(s.pose[6], 1.0) << k;
  }
}

TEST_F(DiveResurfaceScenarioTest, T2_Depth) {
  ASSERT_TRUE(Aligned()) << "T1 failed; stop and report, do not shift.";
  for (int k = 0; k < kSteps; ++k) {
    const StepRecord& s = run().steps[k];
    ASSERT_EQ(s.depth.has_value(), !InDropout(k)) << k;
    if (!s.depth.has_value()) {
      continue;
    }
    EXPECT_LE(std::abs(s.depth->depth_m() - DepthAt(profile(), k)), kDepthTolM)
        << k << " depth_m=" << s.depth->depth_m();
    if (k >= 160 && k < 240) {
      EXPECT_LE(std::abs(s.depth->depth_m() - kHoldDepthM), kDepthTolM) << k;
    }
    const PressureDepthAssessment a = AssessDepth(*s.depth);
    EXPECT_EQ(a.error, PressureDepthError::kNone) << k;
    EXPECT_EQ(a.state, MeasurementStateKind::kDegraded) << k;
    EXPECT_FALSE(a.accepted) << k;
    EXPECT_EQ(s.depth->health().header().frame_id(), "body") << k;
  }
}

TEST_F(DiveResurfaceScenarioTest, T3_SurfaceFixAndGateBoundary) {
  ASSERT_TRUE(Aligned()) << "T1 failed; stop and report, do not shift.";
  EXPECT_TRUE(run().steps[83].fix.has_value());
  EXPECT_FALSE(run().steps[84].fix.has_value());
  EXPECT_FALSE(run().steps[314].fix.has_value());
  EXPECT_TRUE(run().steps[315].fix.has_value());
  for (int k = 0; k < kSteps; ++k) {
    const StepRecord& s = run().steps[k];
    ASSERT_EQ(s.fix.has_value(), ExpectFix(k)) << k;
    if (!s.fix.has_value()) {
      continue;
    }
    EXPECT_LE(std::abs(s.fix->position_x_m()), kFixXyTolM) << k;
    EXPECT_LE(std::abs(s.fix->position_y_m()), kFixXyTolM) << k;
    EXPECT_EQ(s.fix->health().header().frame_id(), "world_enu") << k;
    const SurfaceFixAssessment a = AssessFix(*s.fix);
    EXPECT_EQ(a.error, SurfaceFixError::kNone) << k;
    EXPECT_EQ(a.state, MeasurementStateKind::kDegraded) << k;
    EXPECT_FALSE(a.accepted) << k;
  }
}

TEST_F(DiveResurfaceScenarioTest, T4_Dvl) {
  ASSERT_TRUE(Aligned()) << "T1 failed; stop and report, do not shift.";
  for (int k = 0; k < kSteps; ++k) {
    const StepRecord& s = run().steps[k];
    ASSERT_EQ(s.dvl.has_value(), !InDropout(k)) << k;
    if (!s.dvl.has_value()) {
      continue;
    }
    const DvlMeasurement& m = *s.dvl;
    EXPECT_EQ(m.mode(), DvlMeasurement::MODE_BOTTOM_TRACK) << k;
    EXPECT_TRUE(m.bottom_lock()) << k;
    EXPECT_LE(std::abs(m.altitude_m() - (30.0 - DepthAt(profile(), k))),
              kDvlAltitudeTolM)
        << k << " altitude_m=" << m.altitude_m();
    const double expected_vz = WorldVelocityZAt(profile(), k);
    EXPECT_LE(std::abs(m.velocity_z_m_s() - expected_vz), kDvlTolM_S)
        << k << " velocity_z_m_s=" << m.velocity_z_m_s();
    EXPECT_LE(std::abs(m.velocity_x_m_s()), kDvlTolM_S) << k;
    EXPECT_LE(std::abs(m.velocity_y_m_s()), kDvlTolM_S) << k;
    const DvlAssessment a = AssessDvlSample(m);
    EXPECT_EQ(a.error, DvlError::kNone) << k;
    EXPECT_EQ(a.state, MeasurementStateKind::kDegraded) << k;
    EXPECT_FALSE(a.accepted) << k;
  }
}

TEST_F(DiveResurfaceScenarioTest, T5_Altimeter) {
  ASSERT_TRUE(Aligned()) << "T1 failed; stop and report, do not shift.";
  for (int k = 0; k < kSteps; ++k) {
    const StepRecord& s = run().steps[k];
    ASSERT_EQ(s.altimeter.has_value(), !InDropout(k)) << k;
    if (!s.altimeter.has_value()) {
      continue;
    }
    const AltimeterMeasurement& m = *s.altimeter;
    EXPECT_TRUE(m.has_return()) << k;
    EXPECT_LE(std::abs(m.range_m() - (30.0 - DepthAt(profile(), k))),
              kAltimeterTolM)
        << k << " range_m=" << m.range_m();
    const AltimeterAssessment a = AssessAltimeterSample(m);
    EXPECT_EQ(a.error, AltimeterError::kNone) << k;
    EXPECT_EQ(a.state, MeasurementStateKind::kDegraded) << k;
    EXPECT_FALSE(a.accepted) << k;
  }
}

TEST_F(DiveResurfaceScenarioTest, T6_Ingest) {
  ASSERT_TRUE(Aligned()) << "T1 failed; stop and report, do not shift.";
  for (int k = 0; k < kSteps; ++k) {
    const StepRecord& s = run().steps[k];
    EXPECT_EQ(static_cast<int>(s.ingest_reasons.size()), s.PresentCount()) << k;
    for (const int reason : s.ingest_reasons) {
      EXPECT_EQ(reason, REJECT_REASON_NONE) << k;
    }
  }
}

TEST_F(DiveResurfaceScenarioTest, T7_Mode) {
  ASSERT_TRUE(Aligned()) << "T1 failed; stop and report, do not shift.";
  EXPECT_EQ(run().initialize_reason, REJECT_REASON_NONE);
  EXPECT_EQ(run().initial_mode, NAVIGATION_MODE_INITIALIZING);
  for (int k = 0; k < kSteps; ++k) {
    const StepRecord& s = run().steps[k];
    EXPECT_EQ(s.predict_reason, REJECT_REASON_NONE) << k;
    EXPECT_EQ(s.mode, ExpectedMode(k)) << k;
    EXPECT_NE(s.mode, NAVIGATION_MODE_FAULTED) << k;
    if (InDropout(k)) {
      // SurfaceFix is gated here because d = 5.25 > 0.5.
      EXPECT_EQ(s.PresentCount(), 0) << k;
    }
  }
}

TEST_F(DiveResurfaceScenarioTest, T8_FinalSurfacedState) {
  ASSERT_TRUE(Aligned()) << "T1 failed; stop and report, do not shift.";
  const StepRecord& s = run().steps[kSteps - 1];
  EXPECT_EQ(s.pose[2], kSurfacedZM);
  ASSERT_TRUE(s.depth.has_value());
  EXPECT_LE(std::abs(s.depth->depth_m() - kSurfacedDepthM), kDepthTolM);
  ASSERT_TRUE(s.fix.has_value());
  const SurfaceFixAssessment a = AssessFix(*s.fix);
  EXPECT_EQ(a.error, SurfaceFixError::kNone);
  EXPECT_EQ(a.state, MeasurementStateKind::kDegraded);
  EXPECT_EQ(s.mode, NAVIGATION_MODE_AIDED);
  EXPECT_EQ(SegmentAt(profile(), kSteps - 1).label, "SURFACED");
}

TEST_F(DiveResurfaceScenarioTest, T9_RepeatIsBitIdentical) {
  ASSERT_TRUE(Aligned()) << "T1 failed; stop and report, do not shift.";
  const ScenarioRun second = RunScenario(profile());
  ASSERT_TRUE(second.setup_ok);
  ASSERT_EQ(second.steps.size(), run().steps.size());
  EXPECT_EQ(second.initialize_reason, run().initialize_reason);
  EXPECT_EQ(second.initial_mode, run().initial_mode);
  for (size_t k = 0; k < run().steps.size(); ++k) {
    const StepRecord& a = run().steps[k];
    const StepRecord& b = second.steps[k];
    for (int sensor = 0; sensor < kNumSensors; ++sensor) {
      EXPECT_EQ(BytesOrAbsent(a, sensor), BytesOrAbsent(b, sensor))
          << k << " sensor " << sensor;
    }
    EXPECT_EQ(a.ingest_reasons, b.ingest_reasons) << k;
    EXPECT_EQ(a.predict_reason, b.predict_reason) << k;
    EXPECT_EQ(a.mode, b.mode) << k;
    EXPECT_EQ(std::memcmp(a.pose.data(), b.pose.data(), sizeof(a.pose)), 0)
        << k;
  }
}

}  // namespace
}  // namespace intrinsic::simulation
