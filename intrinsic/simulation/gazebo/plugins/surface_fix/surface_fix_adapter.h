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

#ifndef INTRINSIC_SIMULATION_GAZEBO_PLUGINS_SURFACE_FIX_SURFACE_FIX_ADAPTER_H_
#define INTRINSIC_SIMULATION_GAZEBO_PLUGINS_SURFACE_FIX_SURFACE_FIX_ADAPTER_H_

#include <array>
#include <cstdint>
#include <deque>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

#include "intrinsic/hardware/marine/surface_fix.pb.h"
#include "sdf/Element.hh"

// Simulator-free adapter from one sample of simulator pose to one optional
// #80 SurfaceFix. The Gazebo system plugin is a separate wrapper. This file
// does not include gz-sim.
//
// Sim and test only. Every measurement is written by FakeSurfaceFix::Measure.
// The adapter chooses its inputs (truth, bias, seed, delay), it does not build
// the proto. The adapter makes exactly two edits to a produced message, both
// documented on Step: the receive stamp set at emission, and the noise-only
// DEGRADED rule.
//
// SDF shape (children of the <plugin> element):
//
//   <link_name>            optional. Default is the canonical link.
//   <surface_z_m>          required. Finite. Flat surface plane at world-ENU z.
//   <max_fix_depth_m>      required. Finite and >= 0. Deepest link origin
//                          (below the surface plane) that still gets a fix.
//   <position_bias_{x,y,z}_m> optional. Finite. Default 0.
//   <noise_amplitude_m>    optional. Finite and >= 0. Default 0.
//   <latency_steps>        optional integer in [0, 1000]. Default 0.
//   <dropout_window>       0 to 64 elements. Each holds <start_step> and
//                          <end_step>, half-open [start, end) over the
//                          origination step, 0 <= start <= end.
//   <seed>                 optional uint64. Default 42.
//
// Invalid config fails closed: a typed status and no output on any step.

namespace intrinsic::simulation {

inline constexpr int kSurfaceFixMaxLatencySteps = 1000;
inline constexpr int kSurfaceFixMaxDropoutWindows = 64;
inline constexpr uint64_t kSurfaceFixDefaultSeed = 42;
// Noise slots per step. Slots 0-2 are position x, y, z. Slot 3 is reserved
// and never renumbered.
inline constexpr uint64_t kSurfaceFixNoiseSlotsPerStep = 4;

enum class SurfaceFixSimErrorCode {
  kOk = 0,
  kInvalidConfig = 1,
  kInvalidState = 2,
};

struct SurfaceFixSimStatus {
  SurfaceFixSimErrorCode code = SurfaceFixSimErrorCode::kOk;
  std::string message;

  [[nodiscard]] bool ok() const { return code == SurfaceFixSimErrorCode::kOk; }
};

// One sample of simulator state, already copied out of the simulator.
// Position and orientation are the world-ENU pose of the link origin.
// Orientation is a Hamilton quaternion, world-ENU-from-body, stored x, y, z,
// w.
struct SurfaceFixSimulatorState {
  bool link_resolved = false;
  bool pose_ready = false;
  std::array<double, 3> position_m = {};
  std::array<double, 4> orientation_xyzw = {0, 0, 0, 1};
  // Simulation time of this step. Nanos must be in [0, 1e9).
  int64_t time_seconds = 0;
  int32_t time_nanos = 0;
};

struct SurfaceFixStepResult {
  SurfaceFixSimStatus status;
  // Origination step index of this call, counted from Configure or Reset.
  int64_t step = 0;
  // Present only when a sample is emitted at this step.
  std::optional<intrinsic_proto::hardware::marine::SurfaceFix> measurement;
};

// Splits non-negative nanoseconds since the simulation epoch into whole
// seconds and nanos in [0, 1e9).
void SplitSurfaceFixSimTimeNanos(int64_t total_nanos, int64_t* seconds,
                                 int32_t* nanos);

class SurfaceFixAdapter {
 public:
  // Not configured: every Step returns kInvalidConfig and no output.
  SurfaceFixAdapter() {
    config_status_.code = SurfaceFixSimErrorCode::kInvalidConfig;
    config_status_.message = "surface fix adapter is not configured";
  }

  SurfaceFixAdapter(const SurfaceFixAdapter&) = delete;
  SurfaceFixAdapter& operator=(const SurfaceFixAdapter&) = delete;

  // `plugin_element` is the <plugin> element. Clears the step counter and
  // the latency queue.
  SurfaceFixSimStatus Configure(const sdf::Element& plugin_element);

  // Wraps `plugin_xml` in a minimal SDF world and configures from the first
  // plugin element. `plugin_xml` includes the <plugin> tag.
  SurfaceFixSimStatus ConfigureFromPluginXml(std::string_view plugin_xml);

  // One simulator step. Pipeline order is part of the contract:
  //   1. truth    kInvalidState (empty slot) on an unresolved link, a missing
  //               or non-finite pose, an orientation that is not a unit
  //               quaternion (norm differs from 1 by more than 1e-6, never
  //               normalised), or a time field out of range. The step index
  //               still advances and nothing is emitted.
  //   2. noise    three draws keyed by (4k + i), consumed every step. The
  //               effective per-axis bias is fma(amp, u, bias).
  //   3. gate     d = surface_z_m - z (positive is deeper). When
  //               d > max_fix_depth_m no sample is originated: an empty slot,
  //               like a dropout, and not an error. d == max_fix_depth_m
  //               emits. There is no hysteresis.
  //   4. dropout  the sample for step k is absent.
  //   5. fake     FakeSurfaceFix::Measure with seed + k and the effective
  //               bias, delay 0, dropout false.
  //   6. latency  FIFO of depth latency_steps. Step k emits the item that
  //               originated at k - latency_steps, if any.
  // The position is the link origin in world ENU, frame id
  // WorldFrameId(kEnu), source GNSS, 12 satellites, no velocity, no accuracy.
  // There is no geodetic conversion.
  // The adapter edits a produced message in two places only:
  //   - Receive stamp. The fake is called with delay 0. At emission the
  //     adapter sets the receive time to this step's time.
  //   - DEGRADED. A non-zero noise amplitude never leaves a VALID state: a
  //     sample the fake returned as VALID (only possible when every
  //     effective bias is exactly 0) is set to DEGRADED.
  // A bad-state step enqueues an empty slot, emits nothing, and drops the
  // delayed sample that was due at that step.
  [[nodiscard]] SurfaceFixStepResult Step(
      const SurfaceFixSimulatorState& state);

  // Back to step 0 with an empty latency queue. The configuration stays.
  void Reset();

  [[nodiscard]] bool active() const { return config_status_.ok(); }
  [[nodiscard]] const std::optional<std::string>& link_name() const {
    return link_name_;
  }
  // Test-only. Production code uses active().
  [[nodiscard]] const SurfaceFixSimStatus& ConfigStatusForTesting() const {
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
  SurfaceFixSimStatus Fail(SurfaceFixSimErrorCode code,
                           std::string_view message);

  SurfaceFixSimStatus config_status_;
  std::optional<std::string> link_name_;
  double surface_z_m_ = 0;
  double max_fix_depth_m_ = 0;
  std::array<double, 3> position_bias_m_ = {};
  double noise_amplitude_m_ = 0;
  int latency_steps_ = 0;
  std::vector<DropoutWindow> dropout_windows_;
  uint64_t seed_ = kSurfaceFixDefaultSeed;

  int64_t step_ = 0;
  // One slot per step since the oldest pending origination. A slot is empty
  // when that step produced no sample.
  std::deque<std::optional<intrinsic_proto::hardware::marine::SurfaceFix>>
      queue_;
};

}  // namespace intrinsic::simulation

#endif  // INTRINSIC_SIMULATION_GAZEBO_PLUGINS_SURFACE_FIX_SURFACE_FIX_ADAPTER_H_
