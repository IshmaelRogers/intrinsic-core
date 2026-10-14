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

#ifndef INTRINSIC_SIMULATION_GAZEBO_PLUGINS_ALTIMETER_ALTIMETER_ADAPTER_H_
#define INTRINSIC_SIMULATION_GAZEBO_PLUGINS_ALTIMETER_ALTIMETER_ADAPTER_H_

#include <array>
#include <cstdint>
#include <deque>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

#include "intrinsic/hardware/marine/altimeter.pb.h"
#include "sdf/Element.hh"

// Simulator-free adapter from one sample of simulator pose to one optional
// #78 AltimeterMeasurement. The Gazebo system plugin is a separate wrapper.
// This file does not include gz-sim.
//
// Sim and test only. Every measurement is written by
// FakeAltimeter::Measure. The adapter chooses its inputs (truth, fault
// flags, noise amplitude, seed), it does not build the proto. The adapter
// makes exactly one edit to a produced message: the receive stamp set at
// emission. The fake already sets DEGRADED whenever the amplitude is not
// zero.
//
// The range bounds are fixed at the fake's canonical 0.5 m and 100 m, so the
// fake's canonical out-of-range output is self-consistent.
//
// SDF shape (children of the <plugin> element):
//
//   <link_name>            optional. Default is the canonical link.
//   <seabed_z_m>           required. Finite. Flat seabed plane at world-ENU z.
//   <noise_amplitude_m>    optional. Finite and >= 0. Default 0.
//   <latency_steps>        optional integer in [0, 1000]. Default 0.
//   <dropout_window>       0 to 64 elements. Each holds <start_step> and
//                          <end_step>, half-open [start, end) over the
//                          origination step, 0 <= start <= end.
//   <seed>                 optional uint64. Default 42.
//
// Invalid config fails closed: a typed status and no output on any step.

namespace intrinsic::simulation {

inline constexpr int kAltimeterMaxLatencySteps = 1000;
inline constexpr int kAltimeterMaxDropoutWindows = 64;
inline constexpr uint64_t kAltimeterDefaultSeed = 42;

enum class AltimeterSimErrorCode {
  kOk = 0,
  kInvalidConfig = 1,
  kInvalidState = 2,
};

struct AltimeterSimStatus {
  AltimeterSimErrorCode code = AltimeterSimErrorCode::kOk;
  std::string message;

  [[nodiscard]] bool ok() const { return code == AltimeterSimErrorCode::kOk; }
};

// One sample of simulator state, already copied out of the simulator.
// Position and orientation are the world-ENU pose of the link origin.
// Orientation is a Hamilton quaternion, world-ENU-from-body, stored x, y, z,
// w. It is validated but not used by the altitude truth.
struct AltimeterSimulatorState {
  bool link_resolved = false;
  bool pose_ready = false;
  std::array<double, 3> position_m = {};
  std::array<double, 4> orientation_xyzw = {0, 0, 0, 1};
  // Simulation time of this step. Nanos must be in [0, 1e9).
  int64_t time_seconds = 0;
  int32_t time_nanos = 0;
};

struct AltimeterStepResult {
  AltimeterSimStatus status;
  // Origination step index of this call, counted from Configure or Reset.
  int64_t step = 0;
  // Present only when a sample is emitted at this step.
  std::optional<intrinsic_proto::hardware::marine::AltimeterMeasurement>
      measurement;
};

// Splits non-negative nanoseconds since the simulation epoch into whole
// seconds and nanos in [0, 1e9). Named for the altimeter because the DVL
// package defines a function of the same shape in this namespace.
void SplitAltimeterSimTimeNanos(int64_t total_nanos, int64_t* seconds,
                                int32_t* nanos);

class AltimeterAdapter {
 public:
  // Not configured: every Step returns kInvalidConfig and no output.
  AltimeterAdapter() {
    config_status_.code = AltimeterSimErrorCode::kInvalidConfig;
    config_status_.message = "altimeter adapter is not configured";
  }

  AltimeterAdapter(const AltimeterAdapter&) = delete;
  AltimeterAdapter& operator=(const AltimeterAdapter&) = delete;

  // `plugin_element` is the <plugin> element. Clears the step counter and
  // the latency queue.
  AltimeterSimStatus Configure(const sdf::Element& plugin_element);

  // Wraps `plugin_xml` in a minimal SDF world and configures from the first
  // plugin element. `plugin_xml` includes the <plugin> tag.
  AltimeterSimStatus ConfigureFromPluginXml(std::string_view plugin_xml);

  // One simulator step. Pipeline order is part of the contract:
  //   1. truth        a = z_link - seabed_z_m, vertical. kInvalidState (an
  //                   empty slot is enqueued) on an unresolved link, a
  //                   missing or non-finite pose, an orientation that is not
  //                   a unit quaternion (norm differs from 1 by more than
  //                   1e-6, never normalised), or bad time nanos. The step
  //                   index still advances and nothing is emitted.
  //   2. fault        no_return = a < 0.5 (blanking zone, including at or
  //                   below the seabed). out_of_range = a > 100. The two are
  //                   mutually exclusive.
  //   3. noise        drawn inside the fake as amp * AltimeterSignedUnitNoise
  //                   (seed + k). The adapter adds no second draw.
  //   4. dropout      the sample for step k is absent.
  //   5. produce      FakeAltimeter::Measure with seed + k, no delay.
  //   6. latency      FIFO of depth latency_steps. Step k emits the item
  //                   that originated at k - latency_steps, if any.
  // The only edit to a produced message is the receive stamp: the fake is
  // called with delay 0 and at emission the receive time is set to this
  // step's time, so receive - source = latency_steps * dt.
  [[nodiscard]] AltimeterStepResult Step(const AltimeterSimulatorState& state);

  // Back to step 0 with an empty latency queue. The configuration stays.
  void Reset();

  [[nodiscard]] bool active() const { return config_status_.ok(); }
  [[nodiscard]] const std::optional<std::string>& link_name() const {
    return link_name_;
  }
  // Test-only. Production code uses active().
  [[nodiscard]] const AltimeterSimStatus& ConfigStatusForTesting() const {
    return config_status_;
  }
  // Test-only. Number of Step calls since Configure or Reset.
  [[nodiscard]] int64_t StepIndexForTesting() const { return step_; }

 private:
  struct DropoutWindow {
    int64_t start_step = 0;
    int64_t end_step = 0;
  };

  [[nodiscard]] bool InDropout(int64_t step) const;
  AltimeterSimStatus Fail(AltimeterSimErrorCode code, std::string_view message);

  AltimeterSimStatus config_status_;
  std::optional<std::string> link_name_;
  double seabed_z_m_ = 0;
  double noise_amplitude_m_ = 0;
  int latency_steps_ = 0;
  std::vector<DropoutWindow> dropout_windows_;
  uint64_t seed_ = kAltimeterDefaultSeed;

  int64_t step_ = 0;
  // One slot per step since the oldest pending origination. A slot is empty
  // when that step produced no sample.
  std::deque<
      std::optional<intrinsic_proto::hardware::marine::AltimeterMeasurement>>
      queue_;
};

}  // namespace intrinsic::simulation

#endif  // INTRINSIC_SIMULATION_GAZEBO_PLUGINS_ALTIMETER_ALTIMETER_ADAPTER_H_
