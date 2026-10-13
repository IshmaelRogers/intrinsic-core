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

#ifndef INTRINSIC_SIMULATION_GAZEBO_PLUGINS_DEPTH_DEPTH_ADAPTER_H_
#define INTRINSIC_SIMULATION_GAZEBO_PLUGINS_DEPTH_DEPTH_ADAPTER_H_

#include <array>
#include <cstdint>
#include <deque>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

#include "intrinsic/hardware/marine/pressure_depth.pb.h"
#include "sdf/Element.hh"

// Simulator-free adapter from one sample of simulator pose to one optional
// #77 PressureDepthMeasurement. The Gazebo system plugin is a separate
// wrapper. This file does not include gz-sim.
//
// Sim and test only. Every measurement is written by
// FakePressureDepth::Measure. The adapter chooses its inputs (truth, bias,
// out-of-range, seed), it does not build the proto. The adapter makes exactly
// two edits to a produced message, both documented on Step: the receive stamp
// set at emission, and the noise-only DEGRADED rule.
//
// Depth only. No pressure is synthesized: hydrostatic pressure is a new
// equation with no approved source, so `pressure_pa` and the fluid density
// stay absent and the depth provenance is DIRECT.
//
// SDF shape (children of the <plugin> element):
//
//   <link_name>            optional. Default is the canonical link.
//   <surface_z_m>          optional. Finite. Default 0. Flat free-surface
//                          plane at world-ENU z.
//   <max_depth_m>          optional. Finite and > 0. Unset: no rating limit.
//   <bias_m>               optional. Finite. Default 0.
//   <noise_amplitude_m>    optional. Finite and >= 0. Default 0.
//   <latency_steps>        optional integer in [0, 1000]. Default 0.
//   <dropout_window>       0 to 64 elements. Each holds <start_step> and
//                          <end_step>, half-open [start, end) over the
//                          origination step, 0 <= start <= end.
//   <seed>                 optional uint64. Default 42.
//
// Invalid config fails closed: a typed status and no output on any step.

namespace intrinsic::simulation {

inline constexpr int kDepthMaxLatencySteps = 1000;
inline constexpr int kDepthMaxDropoutWindows = 64;
inline constexpr uint64_t kDepthDefaultSeed = 42;

enum class DepthSimErrorCode {
  kOk = 0,
  kInvalidConfig = 1,
  kInvalidState = 2,
};

struct DepthSimStatus {
  DepthSimErrorCode code = DepthSimErrorCode::kOk;
  std::string message;

  [[nodiscard]] bool ok() const { return code == DepthSimErrorCode::kOk; }
};

// One sample of simulator state, already copied out of the simulator.
// Position and orientation are the world-ENU pose of the link origin.
// Orientation is a Hamilton quaternion, world-ENU-from-body, stored x, y, z,
// w. It is validated but not used by the depth truth.
struct DepthSimulatorState {
  bool link_resolved = false;
  bool pose_ready = false;
  std::array<double, 3> position_m = {};
  std::array<double, 4> orientation_xyzw = {0, 0, 0, 1};
  // Simulation time of this step. Nanos must be in [0, 1e9).
  int64_t time_seconds = 0;
  int32_t time_nanos = 0;
};

struct DepthStepResult {
  DepthSimStatus status;
  // Origination step index of this call, counted from Configure or Reset.
  int64_t step = 0;
  // Present only when a sample is emitted at this step.
  std::optional<intrinsic_proto::hardware::marine::PressureDepthMeasurement>
      measurement;
};

// Splits non-negative nanoseconds since the simulation epoch into whole
// seconds and nanos in [0, 1e9). Named for depth because the DVL package
// defines a function of the same shape in this namespace.
void SplitDepthSimTimeNanos(int64_t total_nanos, int64_t* seconds,
                            int32_t* nanos);

class DepthAdapter {
 public:
  // Not configured: every Step returns kInvalidConfig and no output.
  DepthAdapter() {
    config_status_.code = DepthSimErrorCode::kInvalidConfig;
    config_status_.message = "depth adapter is not configured";
  }

  DepthAdapter(const DepthAdapter&) = delete;
  DepthAdapter& operator=(const DepthAdapter&) = delete;

  // `plugin_element` is the <plugin> element. Clears the step counter and
  // the latency queue.
  DepthSimStatus Configure(const sdf::Element& plugin_element);

  // Wraps `plugin_xml` in a minimal SDF world and configures from the first
  // plugin element. `plugin_xml` includes the <plugin> tag.
  DepthSimStatus ConfigureFromPluginXml(std::string_view plugin_xml);

  // One simulator step. Pipeline order is part of the contract:
  //   1. truth        d = surface_z_m - z_link, positive deeper. kInvalidState
  //                   (an empty slot is enqueued) on an unresolved link, a
  //                   missing or non-finite pose, an orientation that is not
  //                   a unit quaternion (norm differs from 1 by more than
  //                   1e-6, never normalised), or bad time nanos. The step
  //                   index still advances and nothing is emitted.
  //   2. fault        out_of_range = d < 0 || (max_depth_m set && d >
  //                   max_depth_m).
  //   3. noise        u_{k,0} = ImuSignedUnitNoise(seed + 2k * gamma), the
  //                   effective bias is fma(amp, u, bias_m). Slot 1 of each
  //                   step is reserved for a future pressure channel and is
  //                   never drawn.
  //   4. dropout      the sample for step k is absent.
  //   5. produce      FakePressureDepth::Measure with seed + k, no delay.
  //   6. latency      FIFO of depth latency_steps. Step k emits the item
  //                   that originated at k - latency_steps, if any.
  // The adapter edits a produced message in two places only:
  //   - Receive stamp. The fake is called with delay 0. At emission the
  //     receive time is set to this step's time, so receive - source =
  //     latency_steps * dt.
  //   - DEGRADED. A non-zero noise amplitude without out-of-range never
  //     leaves a VALID state: a sample the fake returned as VALID (only
  //     possible when the effective bias is exactly 0) is set to DEGRADED.
  [[nodiscard]] DepthStepResult Step(const DepthSimulatorState& state);

  // Back to step 0 with an empty latency queue. The configuration stays.
  void Reset();

  [[nodiscard]] bool active() const { return config_status_.ok(); }
  [[nodiscard]] const std::optional<std::string>& link_name() const {
    return link_name_;
  }
  // Test-only. Production code uses active().
  [[nodiscard]] const DepthSimStatus& ConfigStatusForTesting() const {
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
  DepthSimStatus Fail(DepthSimErrorCode code, std::string_view message);

  DepthSimStatus config_status_;
  std::optional<std::string> link_name_;
  double surface_z_m_ = 0;
  std::optional<double> max_depth_m_;
  double bias_m_ = 0;
  double noise_amplitude_m_ = 0;
  int latency_steps_ = 0;
  std::vector<DropoutWindow> dropout_windows_;
  uint64_t seed_ = kDepthDefaultSeed;

  int64_t step_ = 0;
  // One slot per step since the oldest pending origination. A slot is empty
  // when that step produced no sample.
  std::deque<std::optional<
      intrinsic_proto::hardware::marine::PressureDepthMeasurement>>
      queue_;
};

}  // namespace intrinsic::simulation

#endif  // INTRINSIC_SIMULATION_GAZEBO_PLUGINS_DEPTH_DEPTH_ADAPTER_H_
