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
#include <charconv>
#include <cmath>
#include <cstdint>
#include <limits>
#include <optional>
#include <string>
#include <string_view>
#include <system_error>
#include <utility>

#include "absl/strings/str_cat.h"
#include "intrinsic/embodiment/frame_policy.h"
#include "intrinsic/embodiment/proto/stamped_header.pb.h"
#include "intrinsic/estimation/eskf_propagate.h"
#include "intrinsic/hardware/marine/fake_imu.h"
#include "intrinsic/hardware/marine/imu.pb.h"
#include "intrinsic/hardware/marine/measurement_health.pb.h"
#include "intrinsic/vehicle/parameters/marine_model.h"
#include "sdf/Element.hh"
#include "sdf/Root.hh"
#include "sdf/World.hh"

namespace intrinsic::simulation {
namespace {

using ::intrinsic::hardware::marine::FakeImu;
using ::intrinsic::hardware::marine::FakeImuConfig;
using ::intrinsic::hardware::marine::ImuSignedUnitNoise;
using ::intrinsic::hardware::marine::ImuTruth;
using ::intrinsic::vehicle::parameters::kBodyFrameId;
using ::intrinsic_proto::hardware::marine::ImuMeasurement;
using ::intrinsic_proto::hardware::marine::MeasurementHealth;

// SplitMix64 increment. The draw key for noise slot `8k + i` is
// `seed + (8k + i) * kImuSplitMix64Gamma` with uint64 wrap, the same
// convention as the DVL, depth, and altimeter adapters.
constexpr uint64_t kImuSplitMix64Gamma = 0x9E3779B97F4A7C15ULL;
constexpr int kImuAxes = 3;
constexpr int64_t kImuNanosPerSecond = 1000000000;
constexpr int64_t kImuMaxTimeSeconds =
    std::numeric_limits<int64_t>::max() / kImuNanosPerSecond - 1;
constexpr double kImuQuality = 0.75;
constexpr double kImuUnitQuaternionTolerance = 1e-6;

bool ImuIsSpace(char value) {
  return value == ' ' || value == '\t' || value == '\n' || value == '\r';
}

std::string_view ImuTrim(std::string_view text) {
  while (!text.empty() && ImuIsSpace(text.front())) {
    text.remove_prefix(1);
  }
  while (!text.empty() && ImuIsSpace(text.back())) {
    text.remove_suffix(1);
  }
  return text;
}

template <typename T>
bool ParseImuWhole(std::string_view text, T* out) {
  text = ImuTrim(text);
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

bool ParseImuFinite(std::string_view text, double* out) {
  double value = 0;
  if (!ParseImuWhole(text, &value) || !std::isfinite(value)) {
    return false;
  }
  *out = value;
  return true;
}

sdf::ElementPtr FindImuPluginElement(const sdf::Root& root) {
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

std::optional<std::string> ImuChildText(const sdf::Element& parent,
                                        const char* name) {
  const sdf::ElementConstPtr child = parent.FindElement(name);
  if (child == nullptr) {
    return std::nullopt;
  }
  return child->Get<std::string>();
}

// Reads an optional finite scalar. Returns false and fills `message` when the
// element is present and invalid.
bool ReadImuScalar(const sdf::Element& parent, const char* name,
                   bool non_negative, double* out, std::string* message) {
  const std::optional<std::string> text = ImuChildText(parent, name);
  if (!text.has_value()) {
    return true;
  }
  double value = 0;
  if (!ParseImuFinite(*text, &value) || (non_negative && value < 0)) {
    *message = absl::StrCat(
        name, non_negative ? " must be finite and >= 0" : " must be finite");
    return false;
  }
  *out = value;
  return true;
}

bool ImuFinite3(const std::array<double, 3>& value) {
  return std::isfinite(value[0]) && std::isfinite(value[1]) &&
         std::isfinite(value[2]);
}

}  // namespace

void SplitImuSimTimeNanos(int64_t total_nanos, int64_t* seconds,
                          int32_t* nanos) {
  int64_t whole = total_nanos / kImuNanosPerSecond;
  int64_t rest = total_nanos % kImuNanosPerSecond;
  if (rest < 0) {
    rest += kImuNanosPerSecond;
    whole -= 1;
  }
  *seconds = whole;
  *nanos = static_cast<int32_t>(rest);
}

ImuSimStatus ImuAdapter::Fail(ImuSimErrorCode code, std::string_view message) {
  config_status_.code = code;
  config_status_.message = std::string(message);
  return config_status_;
}

ImuSimStatus ImuAdapter::Configure(const sdf::Element& plugin_element) {
  config_status_ = ImuSimStatus();
  link_name_.reset();
  angular_bias_rad_s_ = {};
  linear_bias_m_s2_ = {};
  angular_noise_amplitude_rad_s_ = 0;
  linear_noise_amplitude_m_s2_ = 0;
  angular_drift_rad_s2_ = 0;
  linear_drift_m_s3_ = 0;
  latency_steps_ = 0;
  dropout_windows_.clear();
  seed_ = kImuDefaultSeed;
  Reset();

  if (const auto link = ImuChildText(plugin_element, "link_name");
      link.has_value() && !link->empty()) {
    link_name_ = *link;
  }

  static constexpr const char* kAxisNames[kImuAxes] = {"x", "y", "z"};
  std::string message;
  for (int i = 0; i < kImuAxes; ++i) {
    if (!ReadImuScalar(
            plugin_element,
            absl::StrCat("angular_bias_", kAxisNames[i], "_rad_s").c_str(),
            false, &angular_bias_rad_s_[i], &message) ||
        !ReadImuScalar(
            plugin_element,
            absl::StrCat("linear_bias_", kAxisNames[i], "_m_s2").c_str(), false,
            &linear_bias_m_s2_[i], &message)) {
      return Fail(ImuSimErrorCode::kInvalidConfig, message);
    }
  }
  if (!ReadImuScalar(plugin_element, "angular_noise_amplitude_rad_s", true,
                     &angular_noise_amplitude_rad_s_, &message) ||
      !ReadImuScalar(plugin_element, "linear_noise_amplitude_m_s2", true,
                     &linear_noise_amplitude_m_s2_, &message) ||
      !ReadImuScalar(plugin_element, "angular_drift_rad_s2", false,
                     &angular_drift_rad_s2_, &message) ||
      !ReadImuScalar(plugin_element, "linear_drift_m_s3", false,
                     &linear_drift_m_s3_, &message)) {
    return Fail(ImuSimErrorCode::kInvalidConfig, message);
  }

  if (const auto latency = ImuChildText(plugin_element, "latency_steps");
      latency.has_value()) {
    int64_t steps = 0;
    if (!ParseImuWhole(*latency, &steps) || steps < 0 ||
        steps > kImuMaxLatencySteps) {
      return Fail(ImuSimErrorCode::kInvalidConfig,
                  absl::StrCat("latency_steps must be an integer in [0, ",
                               kImuMaxLatencySteps, "]"));
    }
    latency_steps_ = static_cast<int>(steps);
  }
  if (const auto seed = ImuChildText(plugin_element, "seed");
      seed.has_value()) {
    if (!ParseImuWhole(*seed, &seed_)) {
      return Fail(ImuSimErrorCode::kInvalidConfig,
                  "seed must be an unsigned 64-bit integer");
    }
  }

  for (sdf::ElementConstPtr window =
           plugin_element.FindElement("dropout_window");
       window != nullptr; window = window->GetNextElement("dropout_window")) {
    if (static_cast<int>(dropout_windows_.size()) >= kImuMaxDropoutWindows) {
      return Fail(ImuSimErrorCode::kInvalidConfig,
                  absl::StrCat("at most ", kImuMaxDropoutWindows,
                               " dropout_window elements are allowed"));
    }
    const auto start = ImuChildText(*window, "start_step");
    const auto end = ImuChildText(*window, "end_step");
    DropoutWindow parsed;
    if (!start.has_value() || !end.has_value() ||
        !ParseImuWhole(*start, &parsed.start_step) ||
        !ParseImuWhole(*end, &parsed.end_step) || parsed.start_step < 0 ||
        parsed.start_step > parsed.end_step) {
      return Fail(ImuSimErrorCode::kInvalidConfig,
                  "dropout_window needs integers 0 <= start_step <= "
                  "end_step");
    }
    dropout_windows_.push_back(parsed);
  }
  return config_status_;
}

ImuSimStatus ImuAdapter::ConfigureFromPluginXml(std::string_view plugin_xml) {
  const std::string wrapped =
      absl::StrCat("<sdf version=\"1.8\"><world name=\"imu_parse\">",
                   plugin_xml, "</world></sdf>");
  sdf::Root root;
  // A plugin without filename can make the loader record an error while the
  // element tree is still usable.
  root.LoadSdfString(wrapped);
  const sdf::ElementPtr plugin = FindImuPluginElement(root);
  if (plugin == nullptr) {
    Reset();
    link_name_.reset();
    return Fail(ImuSimErrorCode::kInvalidConfig,
                "imu plugin sdf could not be parsed");
  }
  return Configure(*plugin);
}

bool ImuAdapter::InDropout(int64_t step) const {
  for (const DropoutWindow& window : dropout_windows_) {
    if (step >= window.start_step && step < window.end_step) {
      return true;
    }
  }
  return false;
}

void ImuAdapter::Reset() {
  step_ = 0;
  origin_nanos_.reset();
  queue_.clear();
}

ImuStepResult ImuAdapter::Step(const ImuSimulatorState& state) {
  ImuStepResult result;
  if (!config_status_.ok()) {
    result.status = config_status_;
    return result;
  }
  const int64_t k = step_++;
  result.step = k;

  const bool time_in_range =
      state.time_nanos >= 0 && state.time_nanos < kImuNanosPerSecond &&
      state.time_seconds >= 0 && state.time_seconds <= kImuMaxTimeSeconds;
  const int64_t time_nanos_total =
      time_in_range ? state.time_seconds * kImuNanosPerSecond + state.time_nanos
                    : 0;
  if (time_in_range && !origin_nanos_.has_value()) {
    origin_nanos_ = time_nanos_total;
  }

  // 1. Truth. An invalid state enqueues an empty slot, and nothing is
  // emitted.
  std::optional<ImuMeasurement> produced;
  const auto close_step = [&]() -> std::optional<ImuMeasurement> {
    queue_.push_back(std::move(produced));
    produced.reset();
    if (static_cast<int>(queue_.size()) <= latency_steps_) {
      return std::nullopt;
    }
    std::optional<ImuMeasurement> emitted = std::move(queue_.front());
    queue_.pop_front();
    return emitted;
  };
  const auto invalid_state = [&](std::string_view message) {
    close_step();
    result.status.code = ImuSimErrorCode::kInvalidState;
    result.status.message = std::string(message);
    return result;
  };
  if (!state.link_resolved) {
    return invalid_state("target link is unresolved");
  }
  if (!state.kinematics_ready ||
      !ImuFinite3(state.world_angular_velocity_rad_s) ||
      !ImuFinite3(state.world_linear_acceleration_m_s2) ||
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
  if (!(std::fabs(quaternion_norm - 1.0) <= kImuUnitQuaternionTolerance)) {
    return invalid_state("orientation is not a unit quaternion");
  }
  if (!time_in_range) {
    return invalid_state("simulation time is out of range");
  }
  if (time_nanos_total < *origin_nanos_) {
    return invalid_state("simulation time moved backwards");
  }
  const double elapsed_s =
      static_cast<double>(time_nanos_total - *origin_nanos_) / 1e9;

  const embodiment::Quaternion body_from_world{
      -state.orientation_xyzw[0], -state.orientation_xyzw[1],
      -state.orientation_xyzw[2], state.orientation_xyzw[3]};
  const embodiment::Vec3 world_angular{state.world_angular_velocity_rad_s[0],
                                       state.world_angular_velocity_rad_s[1],
                                       state.world_angular_velocity_rad_s[2]};
  const embodiment::Vec3 world_specific_force{
      state.world_linear_acceleration_m_s2[0] - estimation::kGravityEnu[0],
      state.world_linear_acceleration_m_s2[1] - estimation::kGravityEnu[1],
      state.world_linear_acceleration_m_s2[2] - estimation::kGravityEnu[2]};
  const embodiment::Vec3 angular =
      embodiment::RotateVector(body_from_world, world_angular);
  const embodiment::Vec3 force =
      embodiment::RotateVector(body_from_world, world_specific_force);
  const std::array<double, 2 * kImuAxes> truth_values = {
      angular.x, angular.y, angular.z, force.x, force.y, force.z};

  // 3. Noise. Always drawn by index so that dropout and latency never shift
  // a later step's draws. fma gives one rounding, independent of compiler
  // contraction. Drift is folded into the bias first.
  std::array<double, 2 * kImuAxes> bias = {};
  for (int i = 0; i < 2 * kImuAxes; ++i) {
    const bool angular_axis = i < kImuAxes;
    const double base =
        angular_axis ? angular_bias_rad_s_[i] : linear_bias_m_s2_[i - kImuAxes];
    const double drift_rate =
        angular_axis ? angular_drift_rad_s2_ : linear_drift_m_s3_;
    const double amplitude = angular_axis ? angular_noise_amplitude_rad_s_
                                          : linear_noise_amplitude_m_s2_;
    const double unit = ImuSignedUnitNoise(
        seed_ + (kImuNoiseSlotsPerStep * static_cast<uint64_t>(k) +
                 static_cast<uint64_t>(i)) *
                    kImuSplitMix64Gamma);
    bias[i] = std::fma(amplitude, unit, std::fma(drift_rate, elapsed_s, base));
  }
  bool derived_finite = true;
  for (int i = 0; i < 2 * kImuAxes; ++i) {
    derived_finite = derived_finite && std::isfinite(truth_values[i]) &&
                     std::isfinite(bias[i]) &&
                     std::isfinite(truth_values[i] + bias[i]);
  }
  if (!derived_finite) {
    return invalid_state("derived body-frame value is not finite");
  }

  // 4. Dropout. FakeImu is not called and an empty slot is enqueued for k.
  if (!InDropout(k)) {
    // 5. FakeImu. The receive stamp is set at emission, when it is known.
    FakeImuConfig config;
    config.seed = seed_ + static_cast<uint64_t>(k);
    config.angular_velocity_bias_x_rad_s = bias[0];
    config.angular_velocity_bias_y_rad_s = bias[1];
    config.angular_velocity_bias_z_rad_s = bias[2];
    config.linear_acceleration_bias_x_m_s2 = bias[3];
    config.linear_acceleration_bias_y_m_s2 = bias[4];
    config.linear_acceleration_bias_z_m_s2 = bias[5];
    config.angular_drift_rad_s_per_seed = 0;
    config.linear_drift_m_s2_per_seed = 0;
    config.angular_noise_amplitude_rad_s = 0;
    config.linear_noise_amplitude_m_s2 = 0;
    config.delay_seconds = 0;
    config.delay_nanos = 0;
    config.dropout = false;
    config.invalid_orientation = false;
    ImuTruth truth;
    truth.frame_id = std::string(kBodyFrameId);
    truth.source_seconds = state.time_seconds;
    truth.source_nanos = state.time_nanos;
    truth.angular_velocity_x_rad_s = truth_values[0];
    truth.angular_velocity_y_rad_s = truth_values[1];
    truth.angular_velocity_z_rad_s = truth_values[2];
    truth.linear_acceleration_x_m_s2 = truth_values[3];
    truth.linear_acceleration_y_m_s2 = truth_values[4];
    truth.linear_acceleration_z_m_s2 = truth_values[5];
    truth.orientation_present = true;
    truth.orientation_x = state.orientation_xyzw[0];
    truth.orientation_y = state.orientation_xyzw[1];
    truth.orientation_z = state.orientation_xyzw[2];
    truth.orientation_w = state.orientation_xyzw[3];
    truth.quality = kImuQuality;
    truth.covariance_present = true;
    produced = FakeImu(config).Measure(truth);
    const bool stochastic = angular_noise_amplitude_rad_s_ != 0.0 ||
                            linear_noise_amplitude_m_s2_ != 0.0 ||
                            angular_drift_rad_s2_ != 0.0 ||
                            linear_drift_m_s3_ != 0.0;
    if (produced.has_value() && stochastic &&
        produced->health().state() == MeasurementHealth::VALID) {
      produced->mutable_health()->set_state(MeasurementHealth::DEGRADED);
    }
  }

  // 6. Latency.
  std::optional<ImuMeasurement> emitted = close_step();
  if (emitted.has_value()) {
    intrinsic_proto::embodiment::StampedHeader* header =
        emitted->mutable_health()->mutable_header();
    header->mutable_receive_time()->set_seconds(state.time_seconds);
    header->mutable_receive_time()->set_nanos(state.time_nanos);
  }
  result.measurement = std::move(emitted);
  return result;
}

}  // namespace intrinsic::simulation
