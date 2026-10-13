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

#include "intrinsic/simulation/gazebo/plugins/altimeter/altimeter_adapter.h"

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
#include "intrinsic/hardware/marine/altimeter.pb.h"
#include "intrinsic/hardware/marine/fake_altimeter.h"
#include "intrinsic/vehicle/parameters/marine_model.h"
#include "sdf/Element.hh"
#include "sdf/Root.hh"
#include "sdf/World.hh"

namespace intrinsic::simulation {
namespace {

using ::intrinsic::hardware::marine::AltimeterTruth;
using ::intrinsic::hardware::marine::FakeAltimeter;
using ::intrinsic::hardware::marine::FakeAltimeterConfig;
using ::intrinsic::hardware::marine::kCanonicalMaxRangeM;
using ::intrinsic::hardware::marine::kCanonicalMinRangeM;
using ::intrinsic::vehicle::parameters::kBodyFrameId;
using ::intrinsic_proto::hardware::marine::AltimeterMeasurement;

constexpr int64_t kNanosPerSecond = 1000000000;
constexpr double kQuality = 0.75;
constexpr double kUnitQuaternionTolerance = 1e-6;
constexpr char kBeamId[] = "down";

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

void SplitAltimeterSimTimeNanos(int64_t total_nanos, int64_t* seconds,
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

AltimeterSimStatus AltimeterAdapter::Fail(AltimeterSimErrorCode code,
                                          std::string_view message) {
  config_status_.code = code;
  config_status_.message = std::string(message);
  return config_status_;
}

AltimeterSimStatus AltimeterAdapter::Configure(
    const sdf::Element& plugin_element) {
  config_status_ = AltimeterSimStatus();
  link_name_.reset();
  seabed_z_m_ = 0;
  noise_amplitude_m_ = 0;
  latency_steps_ = 0;
  dropout_windows_.clear();
  seed_ = kAltimeterDefaultSeed;
  Reset();

  if (const auto link = ChildText(plugin_element, "link_name");
      link.has_value() && !link->empty()) {
    link_name_ = *link;
  }

  const auto seabed = ChildText(plugin_element, "seabed_z_m");
  if (!seabed.has_value() || !ParseFinite(*seabed, &seabed_z_m_)) {
    return Fail(AltimeterSimErrorCode::kInvalidConfig,
                "seabed_z_m is required and must be finite");
  }
  if (const auto amp = ChildText(plugin_element, "noise_amplitude_m");
      amp.has_value() &&
      (!ParseFinite(*amp, &noise_amplitude_m_) || noise_amplitude_m_ < 0)) {
    return Fail(AltimeterSimErrorCode::kInvalidConfig,
                "noise_amplitude_m must be finite and >= 0");
  }
  if (const auto latency = ChildText(plugin_element, "latency_steps");
      latency.has_value()) {
    int64_t steps = 0;
    if (!ParseWhole(*latency, &steps) || steps < 0 ||
        steps > kAltimeterMaxLatencySteps) {
      return Fail(AltimeterSimErrorCode::kInvalidConfig,
                  absl::StrCat("latency_steps must be an integer in [0, ",
                               kAltimeterMaxLatencySteps, "]"));
    }
    latency_steps_ = static_cast<int>(steps);
  }
  if (const auto seed = ChildText(plugin_element, "seed"); seed.has_value()) {
    if (!ParseWhole(*seed, &seed_)) {
      return Fail(AltimeterSimErrorCode::kInvalidConfig,
                  "seed must be an unsigned 64-bit integer");
    }
  }

  for (sdf::ElementConstPtr window =
           plugin_element.FindElement("dropout_window");
       window != nullptr; window = window->GetNextElement("dropout_window")) {
    if (static_cast<int>(dropout_windows_.size()) >=
        kAltimeterMaxDropoutWindows) {
      return Fail(AltimeterSimErrorCode::kInvalidConfig,
                  absl::StrCat("at most ", kAltimeterMaxDropoutWindows,
                               " dropout_window elements are allowed"));
    }
    const auto start = ChildText(*window, "start_step");
    const auto end = ChildText(*window, "end_step");
    DropoutWindow parsed;
    if (!start.has_value() || !end.has_value() ||
        !ParseWhole(*start, &parsed.start_step) ||
        !ParseWhole(*end, &parsed.end_step) || parsed.start_step < 0 ||
        parsed.start_step > parsed.end_step) {
      return Fail(AltimeterSimErrorCode::kInvalidConfig,
                  "dropout_window needs integers 0 <= start_step <= "
                  "end_step");
    }
    dropout_windows_.push_back(parsed);
  }
  return config_status_;
}

AltimeterSimStatus AltimeterAdapter::ConfigureFromPluginXml(
    std::string_view plugin_xml) {
  const std::string wrapped =
      absl::StrCat("<sdf version=\"1.8\"><world name=\"altimeter_parse\">",
                   plugin_xml, "</world></sdf>");
  sdf::Root root;
  // A plugin without filename can make the loader record an error while the
  // element tree is still usable.
  root.LoadSdfString(wrapped);
  const sdf::ElementPtr plugin = FindPluginElement(root);
  if (plugin == nullptr) {
    Reset();
    link_name_.reset();
    return Fail(AltimeterSimErrorCode::kInvalidConfig,
                "altimeter plugin sdf could not be parsed");
  }
  return Configure(*plugin);
}

bool AltimeterAdapter::InDropout(int64_t step) const {
  for (const DropoutWindow& window : dropout_windows_) {
    if (step >= window.start_step && step < window.end_step) {
      return true;
    }
  }
  return false;
}

void AltimeterAdapter::Reset() {
  step_ = 0;
  queue_.clear();
}

AltimeterStepResult AltimeterAdapter::Step(
    const AltimeterSimulatorState& state) {
  AltimeterStepResult result;
  if (!config_status_.ok()) {
    result.status = config_status_;
    return result;
  }
  const int64_t k = step_++;
  result.step = k;

  std::optional<AltimeterMeasurement> produced;
  const auto close_step = [&]() -> std::optional<AltimeterMeasurement> {
    queue_.push_back(std::move(produced));
    produced.reset();
    if (static_cast<int>(queue_.size()) <= latency_steps_) {
      return std::nullopt;
    }
    std::optional<AltimeterMeasurement> emitted = std::move(queue_.front());
    queue_.pop_front();
    return emitted;
  };
  const auto invalid_state = [&](std::string_view message) {
    close_step();
    result.status.code = AltimeterSimErrorCode::kInvalidState;
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
  const double altitude_m = state.position_m[2] - seabed_z_m_;
  if (!std::isfinite(altitude_m) ||
      !std::isfinite(altitude_m + noise_amplitude_m_)) {
    return invalid_state("derived altitude is not finite");
  }

  // 2. Fault class. The blanking zone gives no return, so a no-return never
  // overlaps out-of-range.
  const bool no_return = altitude_m < kCanonicalMinRangeM;
  const bool out_of_range = altitude_m > kCanonicalMaxRangeM;

  // 3. Noise. Drawn inside the fake from seed + k, which is stateless and
  // not applied on either INVALID fault.

  // 4. Dropout. The fake is not called and an empty slot is enqueued.
  if (!InDropout(k)) {
    // 5. Produce. The delay is applied at emission, when the receive stamp
    // is known.
    FakeAltimeterConfig config;
    config.seed = seed_ + static_cast<uint64_t>(k);
    config.noise_amplitude_m = noise_amplitude_m_;
    config.delay_seconds = 0;
    config.delay_nanos = 0;
    config.dropout = false;
    config.no_return = no_return;
    config.out_of_range = out_of_range;
    AltimeterTruth truth;
    truth.frame_id = std::string(kBodyFrameId);
    truth.source_seconds = state.time_seconds;
    truth.source_nanos = state.time_nanos;
    truth.range_present = true;
    truth.range_m = altitude_m;
    truth.beam_present = true;
    truth.beam_id = kBeamId;
    truth.min_present = true;
    truth.min_range_m = kCanonicalMinRangeM;
    truth.max_present = true;
    truth.max_range_m = kCanonicalMaxRangeM;
    truth.has_return_present = true;
    truth.has_return = true;
    truth.quality = kQuality;
    truth.covariance_present = true;
    produced = FakeAltimeter(config).Measure(truth);
  }

  // 6. Latency.
  std::optional<AltimeterMeasurement> emitted = close_step();
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
