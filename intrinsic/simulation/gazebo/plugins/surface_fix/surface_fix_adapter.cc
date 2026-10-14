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
#include "intrinsic/hardware/marine/fake_imu.h"
#include "intrinsic/hardware/marine/fake_surface_fix.h"
#include "intrinsic/hardware/marine/measurement_health.pb.h"
#include "intrinsic/hardware/marine/surface_fix.pb.h"
#include "sdf/Element.hh"
#include "sdf/Root.hh"
#include "sdf/World.hh"

namespace intrinsic::simulation {
namespace {

using ::intrinsic::hardware::marine::FakeSurfaceFix;
using ::intrinsic::hardware::marine::FakeSurfaceFixConfig;
using ::intrinsic::hardware::marine::ImuSignedUnitNoise;
using ::intrinsic::hardware::marine::SurfaceFixTruth;
using ::intrinsic_proto::hardware::marine::MeasurementHealth;
using ::intrinsic_proto::hardware::marine::SurfaceFix;

// SplitMix64 increment. The draw key for noise slot `4k + i` is
// `seed + (4k + i) * kSurfaceFixSplitMix64Gamma` with uint64 wrap, the same
// convention as the DVL, depth, and altimeter adapters.
constexpr uint64_t kSurfaceFixSplitMix64Gamma = 0x9E3779B97F4A7C15ULL;
constexpr int kSurfaceFixAxes = 3;
constexpr int64_t kSurfaceFixNanosPerSecond = 1000000000;
constexpr double kSurfaceFixQuality = 0.75;
constexpr double kSurfaceFixUnitQuaternionTolerance = 1e-6;
constexpr int kSurfaceFixGnssSource = 1;
constexpr int32_t kSurfaceFixSatelliteCount = 12;

bool SurfaceFixIsSpace(char value) {
  return value == ' ' || value == '\t' || value == '\n' || value == '\r';
}

std::string_view SurfaceFixTrim(std::string_view text) {
  while (!text.empty() && SurfaceFixIsSpace(text.front())) {
    text.remove_prefix(1);
  }
  while (!text.empty() && SurfaceFixIsSpace(text.back())) {
    text.remove_suffix(1);
  }
  return text;
}

template <typename T>
bool ParseSurfaceFixWhole(std::string_view text, T* out) {
  text = SurfaceFixTrim(text);
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

bool ParseSurfaceFixFinite(std::string_view text, double* out) {
  double value = 0;
  if (!ParseSurfaceFixWhole(text, &value) || !std::isfinite(value)) {
    return false;
  }
  *out = value;
  return true;
}

sdf::ElementPtr FindSurfaceFixPluginElement(const sdf::Root& root) {
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

std::optional<std::string> SurfaceFixChildText(const sdf::Element& parent,
                                               const char* name) {
  const sdf::ElementConstPtr child = parent.FindElement(name);
  if (child == nullptr) {
    return std::nullopt;
  }
  return child->Get<std::string>();
}

// Reads an optional finite scalar. Returns false and fills `message` when the
// element is present and invalid.
bool ReadSurfaceFixScalar(const sdf::Element& parent, const char* name,
                          bool non_negative, double* out,
                          std::string* message) {
  const std::optional<std::string> text = SurfaceFixChildText(parent, name);
  if (!text.has_value()) {
    return true;
  }
  double value = 0;
  if (!ParseSurfaceFixFinite(*text, &value) || (non_negative && value < 0)) {
    *message = absl::StrCat(
        name, non_negative ? " must be finite and >= 0" : " must be finite");
    return false;
  }
  *out = value;
  return true;
}

}  // namespace

void SplitSurfaceFixSimTimeNanos(int64_t total_nanos, int64_t* seconds,
                                 int32_t* nanos) {
  int64_t whole = total_nanos / kSurfaceFixNanosPerSecond;
  int64_t rest = total_nanos % kSurfaceFixNanosPerSecond;
  if (rest < 0) {
    rest += kSurfaceFixNanosPerSecond;
    whole -= 1;
  }
  *seconds = whole;
  *nanos = static_cast<int32_t>(rest);
}

SurfaceFixSimStatus SurfaceFixAdapter::Fail(SurfaceFixSimErrorCode code,
                                            std::string_view message) {
  config_status_.code = code;
  config_status_.message = std::string(message);
  return config_status_;
}

SurfaceFixSimStatus SurfaceFixAdapter::Configure(
    const sdf::Element& plugin_element) {
  config_status_ = SurfaceFixSimStatus();
  link_name_.reset();
  surface_z_m_ = 0;
  max_fix_depth_m_ = 0;
  position_bias_m_ = {};
  noise_amplitude_m_ = 0;
  latency_steps_ = 0;
  dropout_windows_.clear();
  seed_ = kSurfaceFixDefaultSeed;
  Reset();

  if (const auto link = SurfaceFixChildText(plugin_element, "link_name");
      link.has_value() && !link->empty()) {
    link_name_ = *link;
  }

  const auto surface = SurfaceFixChildText(plugin_element, "surface_z_m");
  if (!surface.has_value() || !ParseSurfaceFixFinite(*surface, &surface_z_m_)) {
    return Fail(SurfaceFixSimErrorCode::kInvalidConfig,
                "surface_z_m is required and must be finite");
  }
  const auto max_depth = SurfaceFixChildText(plugin_element, "max_fix_depth_m");
  if (!max_depth.has_value() ||
      !ParseSurfaceFixFinite(*max_depth, &max_fix_depth_m_) ||
      max_fix_depth_m_ < 0) {
    return Fail(SurfaceFixSimErrorCode::kInvalidConfig,
                "max_fix_depth_m is required, finite, and >= 0");
  }

  static constexpr const char* kAxisNames[kSurfaceFixAxes] = {"x", "y", "z"};
  std::string message;
  for (int i = 0; i < kSurfaceFixAxes; ++i) {
    if (!ReadSurfaceFixScalar(
            plugin_element,
            absl::StrCat("position_bias_", kAxisNames[i], "_m").c_str(), false,
            &position_bias_m_[i], &message)) {
      return Fail(SurfaceFixSimErrorCode::kInvalidConfig, message);
    }
  }
  if (!ReadSurfaceFixScalar(plugin_element, "noise_amplitude_m", true,
                            &noise_amplitude_m_, &message)) {
    return Fail(SurfaceFixSimErrorCode::kInvalidConfig, message);
  }

  if (const auto latency = SurfaceFixChildText(plugin_element, "latency_steps");
      latency.has_value()) {
    int64_t steps = 0;
    if (!ParseSurfaceFixWhole(*latency, &steps) || steps < 0 ||
        steps > kSurfaceFixMaxLatencySteps) {
      return Fail(SurfaceFixSimErrorCode::kInvalidConfig,
                  absl::StrCat("latency_steps must be an integer in [0, ",
                               kSurfaceFixMaxLatencySteps, "]"));
    }
    latency_steps_ = static_cast<int>(steps);
  }
  if (const auto seed = SurfaceFixChildText(plugin_element, "seed");
      seed.has_value()) {
    if (!ParseSurfaceFixWhole(*seed, &seed_)) {
      return Fail(SurfaceFixSimErrorCode::kInvalidConfig,
                  "seed must be an unsigned 64-bit integer");
    }
  }

  for (sdf::ElementConstPtr window =
           plugin_element.FindElement("dropout_window");
       window != nullptr; window = window->GetNextElement("dropout_window")) {
    if (static_cast<int>(dropout_windows_.size()) >=
        kSurfaceFixMaxDropoutWindows) {
      return Fail(SurfaceFixSimErrorCode::kInvalidConfig,
                  absl::StrCat("at most ", kSurfaceFixMaxDropoutWindows,
                               " dropout_window elements are allowed"));
    }
    const auto start = SurfaceFixChildText(*window, "start_step");
    const auto end = SurfaceFixChildText(*window, "end_step");
    DropoutWindow parsed;
    if (!start.has_value() || !end.has_value() ||
        !ParseSurfaceFixWhole(*start, &parsed.start_step) ||
        !ParseSurfaceFixWhole(*end, &parsed.end_step) ||
        parsed.start_step < 0 || parsed.start_step > parsed.end_step) {
      return Fail(SurfaceFixSimErrorCode::kInvalidConfig,
                  "dropout_window needs integers 0 <= start_step <= "
                  "end_step");
    }
    dropout_windows_.push_back(parsed);
  }
  return config_status_;
}

SurfaceFixSimStatus SurfaceFixAdapter::ConfigureFromPluginXml(
    std::string_view plugin_xml) {
  const std::string wrapped =
      absl::StrCat("<sdf version=\"1.8\"><world name=\"surface_fix_parse\">",
                   plugin_xml, "</world></sdf>");
  sdf::Root root;
  // A plugin without filename can make the loader record an error while the
  // element tree is still usable.
  root.LoadSdfString(wrapped);
  const sdf::ElementPtr plugin = FindSurfaceFixPluginElement(root);
  if (plugin == nullptr) {
    Reset();
    link_name_.reset();
    return Fail(SurfaceFixSimErrorCode::kInvalidConfig,
                "surface fix plugin sdf could not be parsed");
  }
  return Configure(*plugin);
}

bool SurfaceFixAdapter::InDropout(int64_t step) const {
  for (const DropoutWindow& window : dropout_windows_) {
    if (step >= window.start_step && step < window.end_step) {
      return true;
    }
  }
  return false;
}

void SurfaceFixAdapter::Reset() {
  step_ = 0;
  queue_.clear();
}

SurfaceFixStepResult SurfaceFixAdapter::Step(
    const SurfaceFixSimulatorState& state) {
  SurfaceFixStepResult result;
  if (!config_status_.ok()) {
    result.status = config_status_;
    return result;
  }
  const int64_t k = step_++;
  result.step = k;

  // 1. Truth. An invalid state enqueues an empty slot, and nothing is
  // emitted.
  std::optional<SurfaceFix> produced;
  const auto close_step = [&]() -> std::optional<SurfaceFix> {
    queue_.push_back(std::move(produced));
    produced.reset();
    if (static_cast<int>(queue_.size()) <= latency_steps_) {
      return std::nullopt;
    }
    std::optional<SurfaceFix> emitted = std::move(queue_.front());
    queue_.pop_front();
    return emitted;
  };
  const auto invalid_state = [&](std::string_view message) {
    close_step();
    result.status.code = SurfaceFixSimErrorCode::kInvalidState;
    result.status.message = std::string(message);
    return result;
  };
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
  if (!(std::fabs(quaternion_norm - 1.0) <=
        kSurfaceFixUnitQuaternionTolerance)) {
    return invalid_state("orientation is not a unit quaternion");
  }
  if (state.time_nanos < 0 || state.time_nanos >= kSurfaceFixNanosPerSecond) {
    return invalid_state("simulation time nanos must be in [0, 1e9)");
  }
  const double depth_m = surface_z_m_ - state.position_m[2];
  if (!std::isfinite(depth_m)) {
    return invalid_state("derived depth is not finite");
  }

  // 3. Noise. Always drawn by index so that the gate, dropout, and latency
  // never shift a later step's draws. fma gives one rounding, independent of
  // compiler contraction.
  std::array<double, kSurfaceFixAxes> bias = {};
  for (int i = 0; i < kSurfaceFixAxes; ++i) {
    const double unit = ImuSignedUnitNoise(
        seed_ + (kSurfaceFixNoiseSlotsPerStep * static_cast<uint64_t>(k) +
                 static_cast<uint64_t>(i)) *
                    kSurfaceFixSplitMix64Gamma);
    bias[i] = std::fma(noise_amplitude_m_, unit, position_bias_m_[i]);
  }
  for (int i = 0; i < kSurfaceFixAxes; ++i) {
    if (!std::isfinite(state.position_m[i] + bias[i])) {
      return invalid_state("derived position is not finite");
    }
  }

  // 2. Gate. Too deep for a fix: no sample is originated. This is the same
  // empty slot as a dropout, and it is not an error.
  const bool too_deep = depth_m > max_fix_depth_m_;

  // 4. Dropout. The fake is not called and an empty slot is enqueued for k.
  if (!too_deep && !InDropout(k)) {
    // 5. FakeSurfaceFix. The receive stamp is set at emission, when it is
    // known.
    FakeSurfaceFixConfig config;
    config.seed = seed_ + static_cast<uint64_t>(k);
    config.position_bias_x_m = bias[0];
    config.position_bias_y_m = bias[1];
    config.position_bias_z_m = bias[2];
    config.delay_seconds = 0;
    config.delay_nanos = 0;
    config.dropout = false;
    config.invalid_fix = false;
    SurfaceFixTruth truth;
    truth.frame_id =
        std::string(embodiment::WorldFrameId(embodiment::WorldFrame::kEnu));
    truth.source_seconds = state.time_seconds;
    truth.source_nanos = state.time_nanos;
    truth.position_x_m = state.position_m[0];
    truth.position_y_m = state.position_m[1];
    truth.position_z_m = state.position_m[2];
    truth.source_present = true;
    truth.source = kSurfaceFixGnssSource;
    truth.satellite_count_present = true;
    truth.satellite_count = kSurfaceFixSatelliteCount;
    truth.beacon_count_present = false;
    truth.horizontal_accuracy_present = false;
    truth.vertical_accuracy_present = false;
    truth.velocity_present = false;
    truth.quality = kSurfaceFixQuality;
    truth.covariance_present = true;
    produced = FakeSurfaceFix(config).Measure(truth);
    if (produced.has_value() && noise_amplitude_m_ != 0.0 &&
        produced->health().state() == MeasurementHealth::VALID) {
      produced->mutable_health()->set_state(MeasurementHealth::DEGRADED);
    }
  }

  // 6. Latency.
  std::optional<SurfaceFix> emitted = close_step();
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
