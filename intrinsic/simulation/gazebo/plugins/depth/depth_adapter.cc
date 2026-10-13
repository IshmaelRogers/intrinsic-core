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

#include "intrinsic/simulation/gazebo/plugins/depth/depth_adapter.h"

#include <charconv>
#include <cmath>
#include <cstdint>
#include <optional>
#include <string>
#include <string_view>
#include <system_error>
#include <utility>

#include "absl/strings/str_cat.h"
#include "intrinsic/embodiment/proto/stamped_header.pb.h"
#include "intrinsic/hardware/marine/fake_imu.h"
#include "intrinsic/hardware/marine/fake_pressure_depth.h"
#include "intrinsic/hardware/marine/measurement_health.pb.h"
#include "intrinsic/hardware/marine/pressure_depth.pb.h"
#include "intrinsic/vehicle/parameters/marine_model.h"
#include "sdf/Element.hh"
#include "sdf/Root.hh"
#include "sdf/World.hh"

namespace intrinsic::simulation {
namespace {

using ::intrinsic::hardware::marine::FakePressureDepth;
using ::intrinsic::hardware::marine::FakePressureDepthConfig;
using ::intrinsic::hardware::marine::ImuSignedUnitNoise;
using ::intrinsic::hardware::marine::PressureDepthTruth;
using ::intrinsic::vehicle::parameters::kBodyFrameId;
using ::intrinsic_proto::hardware::marine::MeasurementHealth;
using ::intrinsic_proto::hardware::marine::PressureDepthMeasurement;

// SplitMix64 increment. The draw key for noise index `2k + i` is
// `seed + (2k + i) * kSplitMix64Gamma` with uint64 wrap, the same convention
// as the #123 sampler slots and the DVL adapter.
constexpr uint64_t kSplitMix64Gamma = 0x9E3779B97F4A7C15ULL;
constexpr int64_t kNanosPerSecond = 1000000000;
constexpr double kQuality = 0.75;
constexpr double kUnitQuaternionTolerance = 1e-6;
constexpr int kDirectProvenance =
    static_cast<int>(PressureDepthMeasurement::DEPTH_PROVENANCE_DIRECT);

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

}  // namespace

void SplitDepthSimTimeNanos(int64_t total_nanos, int64_t* seconds,
                            int32_t* nanos) {
  int64_t whole = total_nanos / kNanosPerSecond;
  int64_t rest = total_nanos % kNanosPerSecond;
  if (rest < 0) {
    rest += kNanosPerSecond;
    whole -= 1;
  }
  *seconds = whole;
  *nanos = static_cast<int32_t>(rest);
}

DepthSimStatus DepthAdapter::Fail(DepthSimErrorCode code,
                                  std::string_view message) {
  config_status_.code = code;
  config_status_.message = std::string(message);
  return config_status_;
}

DepthSimStatus DepthAdapter::Configure(const sdf::Element& plugin_element) {
  config_status_ = DepthSimStatus();
  link_name_.reset();
  surface_z_m_ = 0;
  max_depth_m_.reset();
  bias_m_ = 0;
  noise_amplitude_m_ = 0;
  latency_steps_ = 0;
  dropout_windows_.clear();
  seed_ = kDepthDefaultSeed;
  Reset();

  if (const auto link = ChildText(plugin_element, "link_name");
      link.has_value() && !link->empty()) {
    link_name_ = *link;
  }

  if (const auto surface = ChildText(plugin_element, "surface_z_m");
      surface.has_value() && !ParseFinite(*surface, &surface_z_m_)) {
    return Fail(DepthSimErrorCode::kInvalidConfig,
                "surface_z_m must be finite");
  }
  if (const auto max = ChildText(plugin_element, "max_depth_m");
      max.has_value()) {
    double value = 0;
    if (!ParseFinite(*max, &value) || value <= 0) {
      return Fail(DepthSimErrorCode::kInvalidConfig,
                  "max_depth_m must be finite and > 0");
    }
    max_depth_m_ = value;
  }
  if (const auto bias = ChildText(plugin_element, "bias_m");
      bias.has_value() && !ParseFinite(*bias, &bias_m_)) {
    return Fail(DepthSimErrorCode::kInvalidConfig, "bias_m must be finite");
  }
  if (const auto amp = ChildText(plugin_element, "noise_amplitude_m");
      amp.has_value() &&
      (!ParseFinite(*amp, &noise_amplitude_m_) || noise_amplitude_m_ < 0)) {
    return Fail(DepthSimErrorCode::kInvalidConfig,
                "noise_amplitude_m must be finite and >= 0");
  }
  if (const auto latency = ChildText(plugin_element, "latency_steps");
      latency.has_value()) {
    int64_t steps = 0;
    if (!ParseWhole(*latency, &steps) || steps < 0 ||
        steps > kDepthMaxLatencySteps) {
      return Fail(DepthSimErrorCode::kInvalidConfig,
                  absl::StrCat("latency_steps must be an integer in [0, ",
                               kDepthMaxLatencySteps, "]"));
    }
    latency_steps_ = static_cast<int>(steps);
  }
  if (const auto seed = ChildText(plugin_element, "seed"); seed.has_value()) {
    if (!ParseWhole(*seed, &seed_)) {
      return Fail(DepthSimErrorCode::kInvalidConfig,
                  "seed must be an unsigned 64-bit integer");
    }
  }

  for (sdf::ElementConstPtr window =
           plugin_element.FindElement("dropout_window");
       window != nullptr; window = window->GetNextElement("dropout_window")) {
    if (static_cast<int>(dropout_windows_.size()) >= kDepthMaxDropoutWindows) {
      return Fail(DepthSimErrorCode::kInvalidConfig,
                  absl::StrCat("at most ", kDepthMaxDropoutWindows,
                               " dropout_window elements are allowed"));
    }
    const auto start = ChildText(*window, "start_step");
    const auto end = ChildText(*window, "end_step");
    DropoutWindow parsed;
    if (!start.has_value() || !end.has_value() ||
        !ParseWhole(*start, &parsed.start_step) ||
        !ParseWhole(*end, &parsed.end_step) || parsed.start_step < 0 ||
        parsed.start_step > parsed.end_step) {
      return Fail(DepthSimErrorCode::kInvalidConfig,
                  "dropout_window needs integers 0 <= start_step <= "
                  "end_step");
    }
    dropout_windows_.push_back(parsed);
  }
  return config_status_;
}

DepthSimStatus DepthAdapter::ConfigureFromPluginXml(
    std::string_view plugin_xml) {
  const std::string wrapped =
      absl::StrCat("<sdf version=\"1.8\"><world name=\"depth_parse\">",
                   plugin_xml, "</world></sdf>");
  sdf::Root root;
  // A plugin without filename can make the loader record an error while the
  // element tree is still usable.
  root.LoadSdfString(wrapped);
  const sdf::ElementPtr plugin = FindPluginElement(root);
  if (plugin == nullptr) {
    Reset();
    link_name_.reset();
    return Fail(DepthSimErrorCode::kInvalidConfig,
                "depth plugin sdf could not be parsed");
  }
  return Configure(*plugin);
}

bool DepthAdapter::InDropout(int64_t step) const {
  for (const DropoutWindow& window : dropout_windows_) {
    if (step >= window.start_step && step < window.end_step) {
      return true;
    }
  }
  return false;
}

void DepthAdapter::Reset() {
  step_ = 0;
  queue_.clear();
}

DepthStepResult DepthAdapter::Step(const DepthSimulatorState& state) {
  DepthStepResult result;
  if (!config_status_.ok()) {
    result.status = config_status_;
    return result;
  }
  const int64_t k = step_++;
  result.step = k;

  std::optional<PressureDepthMeasurement> produced;
  const auto close_step = [&]() -> std::optional<PressureDepthMeasurement> {
    queue_.push_back(std::move(produced));
    produced.reset();
    if (static_cast<int>(queue_.size()) <= latency_steps_) {
      return std::nullopt;
    }
    std::optional<PressureDepthMeasurement> emitted = std::move(queue_.front());
    queue_.pop_front();
    return emitted;
  };
  const auto invalid_state = [&](std::string_view message) {
    close_step();
    result.status.code = DepthSimErrorCode::kInvalidState;
    result.status.message = std::string(message);
    return result;
  };

  // 1. Truth. An invalid state enqueues an empty slot, and nothing is
  // emitted.
  if (!state.link_resolved) {
    return invalid_state("target link is unresolved");
  }
  if (!state.pose_ready || !std::isfinite(state.position_m[0]) ||
      !std::isfinite(state.position_m[1]) ||
      !std::isfinite(state.position_m[2]) ||
      !std::isfinite(state.orientation_xyzw[0]) ||
      !std::isfinite(state.orientation_xyzw[1]) ||
      !std::isfinite(state.orientation_xyzw[2]) ||
      !std::isfinite(state.orientation_xyzw[3])) {
    return invalid_state("link pose is unavailable or not finite");
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
  const double depth_m = surface_z_m_ - state.position_m[2];
  if (!std::isfinite(depth_m)) {
    return invalid_state("derived depth is not finite");
  }

  // 2. Fault class.
  const bool out_of_range =
      depth_m < 0.0 || (max_depth_m_.has_value() && depth_m > *max_depth_m_);

  // 3. Noise. Stateless draw keyed by (2k + 0). Slot (2k + 1) is reserved for
  // a future pressure channel and is never drawn or reused.
  const double unit = ImuSignedUnitNoise(
      seed_ + (2 * static_cast<uint64_t>(k)) * kSplitMix64Gamma);
  // fma: one rounding, independent of compiler contraction.
  const double effective_bias = std::fma(noise_amplitude_m_, unit, bias_m_);
  if (!out_of_range && !std::isfinite(depth_m + effective_bias)) {
    return invalid_state("derived depth with bias is not finite");
  }

  // 4. Dropout. The fake is not called and an empty slot is enqueued.
  if (!InDropout(k)) {
    // 5. Produce. The delay is applied at emission, when the receive stamp
    // is known.
    FakePressureDepthConfig config;
    config.seed = seed_ + static_cast<uint64_t>(k);
    config.pressure_bias_pa = 0;
    config.depth_bias_m = effective_bias;
    config.delay_seconds = 0;
    config.delay_nanos = 0;
    config.dropout = false;
    config.out_of_range = out_of_range;
    PressureDepthTruth truth;
    truth.frame_id = std::string(kBodyFrameId);
    truth.source_seconds = state.time_seconds;
    truth.source_nanos = state.time_nanos;
    truth.pressure_present = false;
    truth.depth_present = true;
    truth.depth_m = depth_m;
    truth.provenance_present = true;
    truth.depth_provenance = kDirectProvenance;
    truth.density_present = false;
    truth.quality = kQuality;
    truth.covariance_present = true;
    produced = FakePressureDepth(config).Measure(truth);
    if (produced.has_value() && noise_amplitude_m_ != 0.0 && !out_of_range &&
        produced->health().state() == MeasurementHealth::VALID) {
      produced->mutable_health()->set_state(MeasurementHealth::DEGRADED);
    }
  }

  // 6. Latency.
  std::optional<PressureDepthMeasurement> emitted = close_step();
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
