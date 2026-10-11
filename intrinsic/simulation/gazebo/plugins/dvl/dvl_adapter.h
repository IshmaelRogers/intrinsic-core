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

#ifndef INTRINSIC_SIMULATION_GAZEBO_PLUGINS_DVL_DVL_ADAPTER_H_
#define INTRINSIC_SIMULATION_GAZEBO_PLUGINS_DVL_DVL_ADAPTER_H_

#include <array>
#include <cstdint>
#include <deque>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

#include "intrinsic/hardware/marine/dvl.pb.h"
#include "intrinsic/hardware/marine/fake_dvl.h"
#include "sdf/Element.hh"

// Simulator-free adapter from one sample of simulator kinematics to one
// optional #76 DvlMeasurement. The Gazebo system plugin is a separate
// wrapper. This file does not include gz-sim.
//
// Sim and test only. Every measurement is written by FakeDvl::Measure. The
// adapter chooses its inputs (truth, bias, lock-loss, seed, delay), it does
// not build the proto. The adapter makes exactly two edits to a produced
// message, both documented on Step: the receive stamp set at emission (latency
// is applied with AddClockDelay), and the noise-only DEGRADED rule.
//
// SDF shape (children of the <plugin> element):
//
//   <link_name>            optional. Default is the canonical link.
//   <mode>                 optional. bottom_track (default) or water_track.
//   <seabed_z_m>           required. Finite. Flat seabed plane at world-ENU z.
//   <min_altitude_m>       optional. Finite. Default 0. 0 <= min <= max.
//   <max_altitude_m>       required. Finite and > 0.
//   <current_field>        #121 shape. Required for water_track. Validated
//                          whenever present. Used only by water track.
//   <bias_m_s>             optional "x y z". Finite. Default "0 0 0".
//   <noise_amplitude_m_s>  optional. Finite and >= 0. Default 0.
//   <latency_steps>        optional integer in [0, 1000]. Default 0.
//   <dropout_window>       0 to 64 elements. Each holds <start_step> and
//                          <end_step>, half-open [start, end) over the
//                          origination step, 0 <= start <= end.
//   <seed>                 optional uint64. Default 42.
//
// Invalid config fails closed: a typed status and no output on any step.

namespace intrinsic::simulation {

inline constexpr int kDvlMaxLatencySteps = 1000;
inline constexpr int kDvlMaxDropoutWindows = 64;
inline constexpr uint64_t kDvlDefaultSeed = 42;

enum class DvlSimErrorCode {
  kOk = 0,
  kInvalidConfig = 1,
  kInvalidState = 2,
};

struct DvlSimStatus {
  DvlSimErrorCode code = DvlSimErrorCode::kOk;
  std::string message;

  [[nodiscard]] bool ok() const { return code == DvlSimErrorCode::kOk; }
};

// One sample of simulator state, already copied out of the simulator.
// Position and orientation are the world-ENU pose of the link origin.
// Orientation is a Hamilton quaternion, world-ENU-from-body, stored x, y, z,
// w. The linear velocity is of the link origin, expressed in world ENU.
struct DvlSimulatorState {
  bool link_resolved = false;
  bool kinematics_ready = false;
  std::array<double, 3> position_m = {};
  std::array<double, 4> orientation_xyzw = {0, 0, 0, 1};
  std::array<double, 3> world_linear_velocity_m_s = {};
  // Simulation time of this step. Nanos must be in [0, 1e9).
  int64_t time_seconds = 0;
  int32_t time_nanos = 0;
};

struct DvlStepResult {
  DvlSimStatus status;
  // Origination step index of this call, counted from Configure or Reset.
  int64_t step = 0;
  // Present only when a sample is emitted at this step.
  std::optional<intrinsic_proto::hardware::marine::DvlMeasurement> measurement;
};

// Splits non-negative nanoseconds since the simulation epoch into whole
// seconds and nanos in [0, 1e9).
void SplitSimTimeNanos(int64_t total_nanos, int64_t* seconds, int32_t* nanos);

class DvlAdapter {
 public:
  // Not configured: every Step returns kInvalidConfig and no output.
  DvlAdapter() {
    config_status_.code = DvlSimErrorCode::kInvalidConfig;
    config_status_.message = "dvl adapter is not configured";
  }

  DvlAdapter(const DvlAdapter&) = delete;
  DvlAdapter& operator=(const DvlAdapter&) = delete;

  // `plugin_element` is the <plugin> element. Clears the step counter and
  // the latency queue.
  DvlSimStatus Configure(const sdf::Element& plugin_element);

  // Wraps `plugin_xml` in a minimal SDF world and configures from the first
  // plugin element. `plugin_xml` includes the <plugin> tag.
  DvlSimStatus ConfigureFromPluginXml(std::string_view plugin_xml);

  // One simulator step. Pipeline order is part of the contract:
  //   1. truth        kInvalidState (nothing enqueued) on an unresolved
  //                   link, missing or non-finite kinematics, or an
  //                   orientation that is not a unit quaternion (norm
  //                   differs from 1 by more than 1e-6, never normalised).
  //                   The step index still advances and nothing is emitted.
  //   2. lock         lock_loss = bottom_track && altitude out of range.
  //   3. noise        three draws keyed by (3k + i), consumed every step.
  //   4. dropout      the sample for step k is absent.
  //   5. FakeDvl      FakeDvl::Measure with seed + k and the effective bias.
  //   6. latency      FIFO of depth latency_steps. Step k emits the item
  //                   that originated at k - latency_steps, if any.
  // The adapter edits a produced message in two places only:
  //   - Receive stamp. FakeDvl is called with delay 0. At emission the
  //     adapter sets the receive time to this step's time with
  //     AddClockDelay, so receive - source = latency_steps * dt.
  //   - DEGRADED. A non-zero noise amplitude without lock-loss never leaves
  //     a VALID state: a sample the fake returned as VALID is set to
  //     DEGRADED.
  // A bad-state step (stage 1) enqueues an empty slot, emits nothing, and
  // drops the delayed sample that was due at that step. One bad step at k
  // therefore means no sample at step k and none at k + latency_steps.
  [[nodiscard]] DvlStepResult Step(const DvlSimulatorState& state);

  // Back to step 0 with an empty latency queue. The configuration stays.
  void Reset();

  [[nodiscard]] bool active() const { return config_status_.ok(); }
  [[nodiscard]] const std::optional<std::string>& link_name() const {
    return link_name_;
  }
  // Test-only. Production code uses active().
  [[nodiscard]] const DvlSimStatus& ConfigStatusForTesting() const {
    return config_status_;
  }
  // Test-only. Number of Step calls since Configure or Reset.
  [[nodiscard]] int64_t StepIndexForTesting() const { return step_; }

 private:
  enum class CurrentFrame { kWorldEnu, kWorldNed, kBody };

  struct DropoutWindow {
    int64_t start_step = 0;
    int64_t end_step = 0;
  };

  [[nodiscard]] bool InDropout(int64_t step) const;
  DvlSimStatus Fail(DvlSimErrorCode code, std::string_view message);

  DvlSimStatus config_status_;
  std::optional<std::string> link_name_;
  bool water_track_ = false;
  double seabed_z_m_ = 0;
  double min_altitude_m_ = 0;
  double max_altitude_m_ = 0;
  CurrentFrame current_frame_ = CurrentFrame::kWorldEnu;
  std::array<double, 3> current_m_s_ = {};
  std::array<double, 3> bias_m_s_ = {};
  double noise_amplitude_m_s_ = 0;
  int latency_steps_ = 0;
  std::vector<DropoutWindow> dropout_windows_;
  uint64_t seed_ = kDvlDefaultSeed;

  int64_t step_ = 0;
  // One slot per step since the oldest pending origination. A slot is empty
  // when that step produced no sample.
  std::deque<std::optional<intrinsic_proto::hardware::marine::DvlMeasurement>>
      queue_;
};

}  // namespace intrinsic::simulation

#endif  // INTRINSIC_SIMULATION_GAZEBO_PLUGINS_DVL_DVL_ADAPTER_H_
