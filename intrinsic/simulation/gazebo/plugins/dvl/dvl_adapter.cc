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
#include <charconv>
#include <cmath>
#include <cstdint>
#include <optional>
#include <string>
#include <string_view>
#include <system_error>
#include <utility>

#include "absl/strings/str_cat.h"
#include "intrinsic/embodiment/frame_policy.h"
#include "intrinsic/embodiment/proto/stamped_header.pb.h"
#include "intrinsic/hardware/marine/dvl.pb.h"
#include "intrinsic/hardware/marine/fake_dvl.h"
#include "intrinsic/hardware/marine/fake_imu.h"
#include "intrinsic/hardware/marine/measurement_health.pb.h"
#include "intrinsic/vehicle/parameters/marine_model.h"
#include "intrinsic/world/current_field_component/current_field_component_policy.h"
#include "sdf/Element.hh"
#include "sdf/Root.hh"
#include "sdf/World.hh"

namespace intrinsic::simulation {
namespace {

using ::intrinsic::hardware::marine::AddClockDelay;
using ::intrinsic::hardware::marine::DvlTruth;
using ::intrinsic::hardware::marine::FakeDvl;
using ::intrinsic::hardware::marine::FakeDvlConfig;
using ::intrinsic::hardware::marine::ImuSignedUnitNoise;
using ::intrinsic::vehicle::parameters::kBodyFrameId;
using ::intrinsic::vehicle::parameters::kWorldEnuFrameId;
using ::intrinsic::vehicle::parameters::kWorldNedFrameId;
using ::intrinsic_proto::hardware::marine::DvlMeasurement;
using ::intrinsic_proto::hardware::marine::MeasurementHealth;

// SplitMix64 increment. The draw key for noise index `3k + i` is
// `seed + (3k + i) * kSplitMix64Gamma` with uint64 wrap, the same
// convention as the #123 sampler slots.
constexpr uint64_t kSplitMix64Gamma = 0x9E3779B97F4A7C15ULL;
constexpr int kAxes = 3;
constexpr int64_t kNanosPerSecond = 1000000000;
constexpr double kQuality = 0.75;
constexpr double kUnitQuaternionTolerance = 1e-6;

bool IsSpace(char value) {
  return value == ' ' || value == '\t' || value == '\n' || value == '\r';
}

std::string_view Trim(std::string_view text) {
  while (!text.empty() && IsSpace(text.front())) {
    text.remove_prefix(1);
  }
  while (!text.empty() && IsSpace(text.back())) {
    text.remove_suffix(1);
  }
  return text;
}

template <typename T>
bool ParseWhole(std::string_view text, T* out) {
  text = Trim(text);
  if (text.empty()) {
    return false;
  }
  T value{};
  const char* end = text.data() + text.size();
  const std::from_chars_result parsed =
      std::from_chars(text.data(), end, value);
  if (parsed.ec != std::errc() || parsed.ptr != end) {
    return false;
  }
  *out = value;
  return true;
}

bool ParseFinite(std::string_view text, double* out) {
  double value = 0;
  if (!ParseWhole(text, &value) || !std::isfinite(value)) {
    return false;
  }
  *out = value;
  return true;
}

// Copied from hydrodynamics_adapter.cc (private there). Not finite-checked
// here: callers apply the same finite check as for scalars.
bool ParseThree(std::string_view text, double* x, double* y, double* z) {
  double values[3];
  int count = 0;
  const char* ptr = text.data();
  const char* end = ptr + text.size();
  while (ptr < end && count < 3) {
    while (ptr < end && IsSpace(*ptr)) {
      ++ptr;
    }
    if (ptr == end) {
      break;
    }
    double value = 0;
    const std::from_chars_result parsed = std::from_chars(ptr, end, value);
    if (parsed.ec != std::errc() || parsed.ptr == ptr) {
      return false;
    }
    values[count++] = value;
    ptr = parsed.ptr;
  }
  while (ptr < end && IsSpace(*ptr)) {
    ++ptr;
  }
  if (ptr != end || count != 3) {
    return false;
  }
  *x = values[0];
  *y = values[1];
  *z = values[2];
  return true;
}

bool ParseThreeFinite(std::string_view text, std::array<double, 3>* out) {
  double x = 0;
  double y = 0;
  double z = 0;
  if (!ParseThree(text, &x, &y, &z) || !std::isfinite(x) || !std::isfinite(y) ||
      !std::isfinite(z)) {
    return false;
  }
  *out = {x, y, z};
  return true;
}

// Copied from hydrodynamics_adapter.cc (private there). #121 does not write
// validity into SDF. A fresh STATE_VALID view lets AssessCurrentField keep
// ownership of the frame and velocity checks.
world::MarineComponentValidityView AcceptedSdfValidity() {
  world::MarineComponentValidityView validity;
  validity.present = true;
  validity.source_id = "dvl_sdf";
  validity.observation_time_present = true;
  validity.observation_time = world::TimeParts{0, 0};
  validity.validity_horizon_present = true;
  validity.validity_horizon = world::TimeParts{1, 0};
  validity.validity_present = true;
  validity.validity_state = 1;
  return validity;
}

sdf::ElementPtr FindPluginElement(const sdf::Root& root) {
  if (root.WorldCount() == 0 || root.WorldByIndex(0) == nullptr) {
    return nullptr;
  }
  const sdf::World* world = root.WorldByIndex(0);
  const sdf::Plugins& plugins = world->Plugins();
  if (!plugins.empty()) {
    return plugins.front().Element();
  }
  if (world->Element() != nullptr) {
    return world->Element()->FindElement("plugin");
  }
  return nullptr;
}

std::optional<std::string> ChildText(const sdf::Element& parent,
                                     const char* name) {
  const sdf::ElementConstPtr child = parent.FindElement(name);
  if (child == nullptr) {
    return std::nullopt;
  }
  return child->Get<std::string>();
}

bool Finite3(const std::array<double, 3>& value) {
  return std::isfinite(value[0]) && std::isfinite(value[1]) &&
         std::isfinite(value[2]);
}

}  // namespace

void SplitSimTimeNanos(int64_t total_nanos, int64_t* seconds, int32_t* nanos) {
  int64_t whole = total_nanos / kNanosPerSecond;
  int64_t rest = total_nanos % kNanosPerSecond;
  if (rest < 0) {
    rest += kNanosPerSecond;
    whole -= 1;
  }
  *seconds = whole;
  *nanos = static_cast<int32_t>(rest);
}

DvlSimStatus DvlAdapter::Fail(DvlSimErrorCode code, std::string_view message) {
  config_status_.code = code;
  config_status_.message = std::string(message);
  return config_status_;
}

DvlSimStatus DvlAdapter::Configure(const sdf::Element& plugin_element) {
  config_status_ = DvlSimStatus();
  link_name_.reset();
  water_track_ = false;
  seabed_z_m_ = 0;
  min_altitude_m_ = 0;
  max_altitude_m_ = 0;
  current_frame_ = CurrentFrame::kWorldEnu;
  current_m_s_ = {};
  bias_m_s_ = {};
  noise_amplitude_m_s_ = 0;
  latency_steps_ = 0;
  dropout_windows_.clear();
  seed_ = kDvlDefaultSeed;
  Reset();

  if (const auto link = ChildText(plugin_element, "link_name");
      link.has_value() && !link->empty()) {
    link_name_ = *link;
  }

  if (const auto mode = ChildText(plugin_element, "mode"); mode.has_value()) {
    const std::string_view text = Trim(*mode);
    if (text == "water_track") {
      water_track_ = true;
    } else if (text != "bottom_track") {
      return Fail(DvlSimErrorCode::kInvalidConfig,
                  "mode must be bottom_track or water_track");
    }
  }

  const auto seabed = ChildText(plugin_element, "seabed_z_m");
  if (!seabed.has_value() || !ParseFinite(*seabed, &seabed_z_m_)) {
    return Fail(DvlSimErrorCode::kInvalidConfig,
                "seabed_z_m is required and must be finite");
  }

  if (const auto min = ChildText(plugin_element, "min_altitude_m");
      min.has_value() && !ParseFinite(*min, &min_altitude_m_)) {
    return Fail(DvlSimErrorCode::kInvalidConfig,
                "min_altitude_m must be finite");
  }
  const auto max = ChildText(plugin_element, "max_altitude_m");
  if (!max.has_value() || !ParseFinite(*max, &max_altitude_m_) ||
      max_altitude_m_ <= 0) {
    return Fail(DvlSimErrorCode::kInvalidConfig,
                "max_altitude_m is required, finite, and > 0");
  }
  if (min_altitude_m_ < 0 || min_altitude_m_ > max_altitude_m_) {
    return Fail(DvlSimErrorCode::kInvalidConfig,
                "altitude bounds must satisfy 0 <= min_altitude_m <= "
                "max_altitude_m");
  }

  if (const auto bias = ChildText(plugin_element, "bias_m_s");
      bias.has_value() && !ParseThreeFinite(*bias, &bias_m_s_)) {
    return Fail(DvlSimErrorCode::kInvalidConfig,
                "bias_m_s must be three finite numbers");
  }
  if (const auto amp = ChildText(plugin_element, "noise_amplitude_m_s");
      amp.has_value() &&
      (!ParseFinite(*amp, &noise_amplitude_m_s_) || noise_amplitude_m_s_ < 0)) {
    return Fail(DvlSimErrorCode::kInvalidConfig,
                "noise_amplitude_m_s must be finite and >= 0");
  }
  if (const auto latency = ChildText(plugin_element, "latency_steps");
      latency.has_value()) {
    int64_t steps = 0;
    if (!ParseWhole(*latency, &steps) || steps < 0 ||
        steps > kDvlMaxLatencySteps) {
      return Fail(DvlSimErrorCode::kInvalidConfig,
                  absl::StrCat("latency_steps must be an integer in [0, ",
                               kDvlMaxLatencySteps, "]"));
    }
    latency_steps_ = static_cast<int>(steps);
  }
  if (const auto seed = ChildText(plugin_element, "seed"); seed.has_value()) {
    if (!ParseWhole(*seed, &seed_)) {
      return Fail(DvlSimErrorCode::kInvalidConfig,
                  "seed must be an unsigned 64-bit integer");
    }
  }

  for (sdf::ElementConstPtr window =
           plugin_element.FindElement("dropout_window");
       window != nullptr; window = window->GetNextElement("dropout_window")) {
    if (static_cast<int>(dropout_windows_.size()) >= kDvlMaxDropoutWindows) {
      return Fail(DvlSimErrorCode::kInvalidConfig,
                  absl::StrCat("at most ", kDvlMaxDropoutWindows,
                               " dropout_window elements are allowed"));
    }
    const auto start = ChildText(*window, "start_step");
    const auto end = ChildText(*window, "end_step");
    DropoutWindow parsed;
    if (!start.has_value() || !end.has_value() ||
        !ParseWhole(*start, &parsed.start_step) ||
        !ParseWhole(*end, &parsed.end_step) || parsed.start_step < 0 ||
        parsed.start_step > parsed.end_step) {
      return Fail(DvlSimErrorCode::kInvalidConfig,
                  "dropout_window needs integers 0 <= start_step <= "
                  "end_step");
    }
    dropout_windows_.push_back(parsed);
  }

  const sdf::ElementConstPtr current =
      plugin_element.FindElement("current_field");
  if (current == nullptr) {
    if (water_track_) {
      return Fail(DvlSimErrorCode::kInvalidConfig,
                  "water_track requires a current_field element");
    }
    return config_status_;
  }

  world::CurrentFieldView view;
  view.present = true;
  view.validity = AcceptedSdfValidity();
  std::string frame_storage;
  if (const auto frame = ChildText(*current, "frame_id"); frame.has_value()) {
    frame_storage = *frame;
  }
  view.frame_id = frame_storage;
  double vx = 0;
  double vy = 0;
  double vz = 0;
  const auto velocity = ChildText(*current, "velocity_m_s");
  if (!velocity.has_value() || !ParseThree(*velocity, &vx, &vy, &vz)) {
    view.velocity_present = false;
  } else {
    view.velocity_present = true;
    view.velocity_m_s = embodiment::Vec3{vx, vy, vz};
  }
  const world::CurrentFieldAssessment assessment =
      world::AssessCurrentField(view, world::TimeParts{0, 0});
  if (assessment.error != world::CurrentFieldError::kNone ||
      !assessment.accepted) {
    return Fail(DvlSimErrorCode::kInvalidConfig,
                "current_field was rejected by AssessCurrentField");
  }
  current_m_s_ = {view.velocity_m_s.x, view.velocity_m_s.y,
                  view.velocity_m_s.z};
  if (view.frame_id == kWorldNedFrameId) {
    current_frame_ = CurrentFrame::kWorldNed;
  } else if (view.frame_id == kBodyFrameId) {
    current_frame_ = CurrentFrame::kBody;
  } else if (view.frame_id == kWorldEnuFrameId) {
    current_frame_ = CurrentFrame::kWorldEnu;
  }
  return config_status_;
}

DvlSimStatus DvlAdapter::ConfigureFromPluginXml(std::string_view plugin_xml) {
  const std::string wrapped =
      absl::StrCat("<sdf version=\"1.8\"><world name=\"dvl_parse\">",
                   plugin_xml, "</world></sdf>");
  sdf::Root root;
  // As in the hydrodynamics adapter: a plugin without filename can make the
  // loader record an error while the element tree is still usable.
  root.LoadSdfString(wrapped);
  const sdf::ElementPtr plugin = FindPluginElement(root);
  if (plugin == nullptr) {
    Reset();
    link_name_.reset();
    return Fail(DvlSimErrorCode::kInvalidConfig,
                "dvl plugin sdf could not be parsed");
  }
  return Configure(*plugin);
}

bool DvlAdapter::InDropout(int64_t step) const {
  for (const DropoutWindow& window : dropout_windows_) {
    if (step >= window.start_step && step < window.end_step) {
      return true;
    }
  }
  return false;
}

void DvlAdapter::Reset() {
  step_ = 0;
  queue_.clear();
}

DvlStepResult DvlAdapter::Step(const DvlSimulatorState& state) {
  DvlStepResult result;
  if (!config_status_.ok()) {
    result.status = config_status_;
    return result;
  }
  const int64_t k = step_++;
  result.step = k;

  // 1. Truth. An invalid state enqueues nothing, and nothing is emitted.
  std::optional<DvlMeasurement> produced;
  const auto close_step = [&]() -> std::optional<DvlMeasurement> {
    queue_.push_back(std::move(produced));
    produced.reset();
    if (static_cast<int>(queue_.size()) <= latency_steps_) {
      return std::nullopt;
    }
    std::optional<DvlMeasurement> emitted = std::move(queue_.front());
    queue_.pop_front();
    return emitted;
  };
  const auto invalid_state = [&](std::string_view message) {
    close_step();
    result.status.code = DvlSimErrorCode::kInvalidState;
    result.status.message = std::string(message);
    return result;
  };
  if (!state.link_resolved) {
    return invalid_state("target link is unresolved");
  }
  if (!state.kinematics_ready || !Finite3(state.position_m) ||
      !Finite3(state.world_linear_velocity_m_s) ||
      !std::isfinite(state.orientation_xyzw[0]) ||
      !std::isfinite(state.orientation_xyzw[1]) ||
      !std::isfinite(state.orientation_xyzw[2]) ||
      !std::isfinite(state.orientation_xyzw[3])) {
    return invalid_state("link kinematics are unavailable or not finite");
  }
  const double quaternion_norm =
      std::sqrt(state.orientation_xyzw[0] * state.orientation_xyzw[0] +
                state.orientation_xyzw[1] * state.orientation_xyzw[1] +
                state.orientation_xyzw[2] * state.orientation_xyzw[2] +
                state.orientation_xyzw[3] * state.orientation_xyzw[3]);
  if (!(std::fabs(quaternion_norm - 1.0) <= kUnitQuaternionTolerance)) {
    return invalid_state("orientation is not a unit quaternion");
  }
  if (state.time_nanos < 0 || state.time_nanos >= kNanosPerSecond) {
    return invalid_state("simulation time nanos must be in [0, 1e9)");
  }

  const embodiment::Quaternion body_from_world{
      -state.orientation_xyzw[0], -state.orientation_xyzw[1],
      -state.orientation_xyzw[2], state.orientation_xyzw[3]};
  const embodiment::Vec3 world_velocity{state.world_linear_velocity_m_s[0],
                                        state.world_linear_velocity_m_s[1],
                                        state.world_linear_velocity_m_s[2]};
  embodiment::Vec3 velocity =
      embodiment::RotateVector(body_from_world, world_velocity);
  if (water_track_) {
    embodiment::Vec3 current_body{current_m_s_[0], current_m_s_[1],
                                  current_m_s_[2]};
    if (current_frame_ == CurrentFrame::kWorldNed) {
      current_body = embodiment::RotateVector(
          body_from_world, embodiment::WorldVectorNedToEnu(current_body));
    } else if (current_frame_ == CurrentFrame::kWorldEnu) {
      current_body = embodiment::RotateVector(body_from_world, current_body);
    }
    velocity = embodiment::Vec3{velocity.x - current_body.x,
                                velocity.y - current_body.y,
                                velocity.z - current_body.z};
  }
  const double altitude_m = state.position_m[2] - seabed_z_m_;
  if (!std::isfinite(velocity.x) || !std::isfinite(velocity.y) ||
      !std::isfinite(velocity.z) || !std::isfinite(altitude_m)) {
    return invalid_state("derived velocity or altitude is not finite");
  }

  // 2. Lock. Water track never sets lock_loss: the fake would force the
  // mode to bottom track.
  const bool in_range =
      min_altitude_m_ <= altitude_m && altitude_m <= max_altitude_m_;
  const bool lock_loss = !water_track_ && !in_range;

  // 3. Noise. Always drawn by index so that dropout, lock-loss, and latency
  // never shift a later step's draws.
  const std::array<double, kAxes> truth_velocity = {velocity.x, velocity.y,
                                                    velocity.z};
  std::array<double, kAxes> bias = bias_m_s_;
  for (int i = 0; i < kAxes; ++i) {
    const double unit = ImuSignedUnitNoise(
        seed_ + (3 * static_cast<uint64_t>(k) + static_cast<uint64_t>(i)) *
                    kSplitMix64Gamma);
    if (!lock_loss) {
      // fma: one rounding, independent of compiler contraction.
      bias[i] = std::fma(noise_amplitude_m_s_, unit, bias_m_s_[i]);
    }
  }

  // 4. Dropout. FakeDvl is not called and nothing is enqueued for k.
  if (!InDropout(k)) {
    // 5. FakeDvl. The delay is added at emission, when the receive stamp
    // is known.
    FakeDvlConfig config;
    config.seed = seed_ + static_cast<uint64_t>(k);
    config.bias_x_m_s = bias[0];
    config.bias_y_m_s = bias[1];
    config.bias_z_m_s = bias[2];
    config.delay_seconds = 0;
    config.delay_nanos = 0;
    config.dropout = false;
    config.lock_loss = lock_loss;
    DvlTruth truth;
    truth.frame_id = std::string(kBodyFrameId);
    truth.source_seconds = state.time_seconds;
    truth.source_nanos = state.time_nanos;
    truth.velocity_x_m_s = truth_velocity[0];
    truth.velocity_y_m_s = truth_velocity[1];
    truth.velocity_z_m_s = truth_velocity[2];
    truth.mode =
        static_cast<int>(water_track_ ? DvlMeasurement::MODE_WATER_TRACK
                                      : DvlMeasurement::MODE_BOTTOM_TRACK);
    truth.quality = kQuality;
    truth.altitude_present = in_range;
    truth.altitude_m = altitude_m;
    truth.covariance_present = true;
    produced = FakeDvl(config).Measure(truth);
    if (produced.has_value() && noise_amplitude_m_s_ != 0.0 && !lock_loss &&
        produced->health().state() == MeasurementHealth::VALID) {
      produced->mutable_health()->set_state(MeasurementHealth::DEGRADED);
    }
  }

  // 6. Latency.
  std::optional<DvlMeasurement> emitted = close_step();
  if (emitted.has_value()) {
    intrinsic_proto::embodiment::StampedHeader* header =
        emitted->mutable_health()->mutable_header();
    const int64_t source_seconds = header->source_time().seconds();
    const int32_t source_nanos = header->source_time().nanos();
    int64_t receive_seconds = source_seconds;
    int32_t receive_nanos = source_nanos;
    AddClockDelay(&receive_seconds, &receive_nanos,
                  state.time_seconds - source_seconds,
                  state.time_nanos - source_nanos);
    header->mutable_receive_time()->set_seconds(receive_seconds);
    header->mutable_receive_time()->set_nanos(receive_nanos);
  }
  result.measurement = std::move(emitted);
  return result;
}

}  // namespace intrinsic::simulation
