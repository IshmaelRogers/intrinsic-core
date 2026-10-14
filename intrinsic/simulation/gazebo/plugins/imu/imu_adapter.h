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

#ifndef INTRINSIC_SIMULATION_GAZEBO_PLUGINS_IMU_IMU_ADAPTER_H_
#define INTRINSIC_SIMULATION_GAZEBO_PLUGINS_IMU_IMU_ADAPTER_H_

#include <array>
#include <cstdint>
#include <deque>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

#include "intrinsic/hardware/marine/imu.pb.h"
#include "sdf/Element.hh"

// Simulator-free adapter from one sample of simulator kinematics to one
// optional #79 ImuMeasurement. The Gazebo system plugin is a separate
// wrapper. This file does not include gz-sim.
//
// Sim and test only. Every measurement is written by FakeImu::Measure. The
// adapter chooses its inputs (truth, bias, seed, delay), it does not build
// the proto. The adapter makes exactly two edits to a produced message, both
// documented on Step: the receive stamp set at emission, and the
// noise-or-drift DEGRADED rule.
//
// SDF shape (children of the <plugin> element):
//
//   <link_name>                    optional. Default is the canonical link.
//   <angular_bias_{x,y,z}_rad_s>   optional. Finite. Default 0.
//   <linear_bias_{x,y,z}_m_s2>     optional. Finite. Default 0.
//   <angular_noise_amplitude_rad_s> optional. Finite and >= 0. Default 0.
//   <linear_noise_amplitude_m_s2>  optional. Finite and >= 0. Default 0.
//   <angular_drift_rad_s2>         optional. Finite. Default 0.
//   <linear_drift_m_s3>            optional. Finite. Default 0.
//   <latency_steps>                optional integer in [0, 1000]. Default 0.
//   <dropout_window>               0 to 64 elements. Each holds <start_step>
//                                  and <end_step>, half-open [start, end)
//                                  over the origination step,
//                                  0 <= start <= end.
//   <seed>                         optional uint64. Default 42.
//
// Invalid config fails closed: a typed status and no output on any step.

namespace intrinsic::simulation {

inline constexpr int kImuMaxLatencySteps = 1000;
inline constexpr int kImuMaxDropoutWindows = 64;
inline constexpr uint64_t kImuDefaultSeed = 42;
// Noise slots per step. Slots 0-2 are angular velocity x, y, z. Slots 3-5 are
// specific force x, y, z. Slots 6-7 are reserved and never renumbered.
inline constexpr uint64_t kImuNoiseSlotsPerStep = 8;

enum class ImuSimErrorCode {
  kOk = 0,
  kInvalidConfig = 1,
  kInvalidState = 2,
};

struct ImuSimStatus {
  ImuSimErrorCode code = ImuSimErrorCode::kOk;
  std::string message;

  [[nodiscard]] bool ok() const { return code == ImuSimErrorCode::kOk; }
};

// One sample of simulator state, already copied out of the simulator. The
// orientation is a Hamilton quaternion, world-ENU-from-body, stored x, y, z,
// w. The angular velocity and the linear acceleration are of the link origin,
// expressed in world ENU. The acceleration is the simulator's coordinate
// acceleration, so it does not include gravity.
struct ImuSimulatorState {
  bool link_resolved = false;
  bool kinematics_ready = false;
  std::array<double, 4> orientation_xyzw = {0, 0, 0, 1};
  std::array<double, 3> world_angular_velocity_rad_s = {};
  std::array<double, 3> world_linear_acceleration_m_s2 = {};
  // Simulation time of this step. Nanos must be in [0, 1e9).
  int64_t time_seconds = 0;
  int32_t time_nanos = 0;
};

struct ImuStepResult {
  ImuSimStatus status;
  // Origination step index of this call, counted from Configure or Reset.
  int64_t step = 0;
  // Present only when a sample is emitted at this step.
  std::optional<intrinsic_proto::hardware::marine::ImuMeasurement> measurement;
};

// Splits non-negative nanoseconds since the simulation epoch into whole
// seconds and nanos in [0, 1e9).
void SplitImuSimTimeNanos(int64_t total_nanos, int64_t* seconds,
                          int32_t* nanos);

class ImuAdapter {
 public:
  // Not configured: every Step returns kInvalidConfig and no output.
  ImuAdapter() {
    config_status_.code = ImuSimErrorCode::kInvalidConfig;
    config_status_.message = "imu adapter is not configured";
  }

  ImuAdapter(const ImuAdapter&) = delete;
  ImuAdapter& operator=(const ImuAdapter&) = delete;

  // `plugin_element` is the <plugin> element. Clears the step counter and
  // the latency queue.
  ImuSimStatus Configure(const sdf::Element& plugin_element);

  // Wraps `plugin_xml` in a minimal SDF world and configures from the first
  // plugin element. `plugin_xml` includes the <plugin> tag.
  ImuSimStatus ConfigureFromPluginXml(std::string_view plugin_xml);

  // One simulator step. Pipeline order is part of the contract:
  //   1. truth    kInvalidState (empty slot) on an unresolved link, missing
  //               or non-finite kinematics, an orientation that is not a unit
  //               quaternion (norm differs from 1 by more than 1e-6, never
  //               normalised), time that is out of range or runs backwards,
  //               or a derived body-frame value that is not finite. The step
  //               index still advances and nothing is emitted.
  //   2. fault    none for the IMU.
  //   3. noise    six draws keyed by (8k + i), consumed every step. The
  //               effective per-axis bias is
  //               fma(amp, u, fma(drift_rate, elapsed_s, bias)).
  //   4. dropout  the sample for step k is absent.
  //   5. FakeImu  FakeImu::Measure with seed + k, the effective bias, zero
  //               drift and noise, delay 0, dropout false.
  //   6. latency  FIFO of depth latency_steps. Step k emits the item that
  //               originated at k - latency_steps, if any.
  // The body frame is "body" (FLU). With R the rotation of q:
  //   omega_b = R^T omega_w
  //   f_b     = R^T (a_w - kGravityEnu)    (rest: f_b = (0, 0, +9.80665))
  // The adapter edits a produced message in two places only:
  //   - Receive stamp. FakeImu is called with delay 0. At emission the
  //     adapter sets the receive time to this step's time.
  //   - DEGRADED. When any noise amplitude or drift rate is non-zero, a
  //     sample the fake returned as VALID (only possible when every
  //     effective bias is exactly 0) is set to DEGRADED.
  // A bad-state step enqueues an empty slot, emits nothing, and drops the
  // delayed sample that was due at that step.
  [[nodiscard]] ImuStepResult Step(const ImuSimulatorState& state);

  // Back to step 0 with an empty latency queue and a new drift origin. The
  // configuration stays.
  void Reset();

  [[nodiscard]] bool active() const { return config_status_.ok(); }
  [[nodiscard]] const std::optional<std::string>& link_name() const {
    return link_name_;
  }
  // Test-only. Production code uses active().
  [[nodiscard]] const ImuSimStatus& ConfigStatusForTesting() const {
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
  ImuSimStatus Fail(ImuSimErrorCode code, std::string_view message);

  ImuSimStatus config_status_;
  std::optional<std::string> link_name_;
  std::array<double, 3> angular_bias_rad_s_ = {};
  std::array<double, 3> linear_bias_m_s2_ = {};
  double angular_noise_amplitude_rad_s_ = 0;
  double linear_noise_amplitude_m_s2_ = 0;
  double angular_drift_rad_s2_ = 0;
  double linear_drift_m_s3_ = 0;
  int latency_steps_ = 0;
  std::vector<DropoutWindow> dropout_windows_;
  uint64_t seed_ = kImuDefaultSeed;

  int64_t step_ = 0;
  // Simulation time of the first step with a valid time field since Configure
  // or Reset. Drift is measured from here.
  std::optional<int64_t> origin_nanos_;
  // One slot per step since the oldest pending origination. A slot is empty
  // when that step produced no sample.
  std::deque<std::optional<intrinsic_proto::hardware::marine::ImuMeasurement>>
      queue_;
};

}  // namespace intrinsic::simulation

#endif  // INTRINSIC_SIMULATION_GAZEBO_PLUGINS_IMU_IMU_ADAPTER_H_
