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

#ifndef INTRINSIC_HARDWARE_MARINE_FAKE_THRUSTER_ARRAY_H_
#define INTRINSIC_HARDWARE_MARINE_FAKE_THRUSTER_ARRAY_H_

#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include "intrinsic/hardware/marine/thruster_array.pb.h"

namespace intrinsic::hardware::marine {

// Deterministic thruster-array fake. The same config and command sequence
// yield the same bytes. `seed` is the feedback sequence origin. It is not
// a random source. This fake does not call ICON, allocate thrust, integrate
// dynamics, or talk to hardware.
//
// Slot bounds default to the six-thruster example magnitudes
// (surge ±, sway, heave). Those numbers are copied here. This target does
// not link the example library and does not pull Gazebo.
//
// Each Apply() records the command, then:
// 1. Dropout returns no feedback. The command stays in the delay line.
//    Dropout is an absent sample, not an engaged fault.
// 2. The applied command is the one from `lag_steps` earlier. A missing
//    past sample is neutral (0 N, enabled). That hold is not a repair of
//    the new command.
// 3. Per slot, the stronger fault wins: stuck-off, failed, disabled
//    (configured or enable false), configured derated, efficiency in
//    (0, 1), then nominal. Neutral faults force applied thrust to 0.
// 4. Otherwise optional slew limits the step, saturation clamps to
//    [min_thrust_n, max_thrust_n], and efficiency scales the clamped
//    thrust. An exact bound is not saturated. Saturation is judged before
//    the efficiency scale.
// 5. The feedback header copies the current command header and sets
//    sequence to seed + step. Thrust values come from the delayed command.
//    Stamps are not rewritten to hide lag.
//
// A watchdog or independent hardware timer is out of scope. This fake only
// neutralizes on disable and on configured stuck-off / failed / disabled.

struct FakeThrusterSlotConfig {
  std::string name;
  // Newtons. Reverse is negative or zero. Forward is positive or zero.
  double min_thrust_n = 0;
  double max_thrust_n = 0;
  double max_forward_slew_n_per_s = 0;
  double max_reverse_slew_n_per_s = 0;
  // Dimensionless (0, 1]. The fake does not repair an out-of-range value.
  double efficiency = 1;
  // When set to DISABLED, DERATED, STUCK_OFF, or FAILED, this fault is
  // applied on every step. NOMINAL and an unset fault leave health to
  // enable and efficiency.
  bool fault_health_present = false;
  int fault_health = 0;
  // Reported when fault_health is DERATED. Not a second scale.
  double fault_derate = 1;
};

// Bounds copied from six_thruster_uuv_example. Not a measurement.
std::vector<FakeThrusterSlotConfig> SixThrusterFixtureSlots();

struct FakeThrusterArrayConfig {
  uint64_t seed = 42;
  // Pure delay in Apply() calls. 0 applies the current command. Negative
  // is treated as 0.
  int lag_steps = 0;
  bool slew = false;
  double dt_s = 0.2;
  bool dropout = false;
  std::vector<FakeThrusterSlotConfig> slots = SixThrusterFixtureSlots();
};

class FakeThrusterArray {
public:
  explicit FakeThrusterArray(FakeThrusterArrayConfig config = {})
      : config_(std::move(config)) {}

  const FakeThrusterArrayConfig &config() const { return config_; }

  // Absent when dropout is set. Otherwise one feedback message.
  std::optional<intrinsic_proto::hardware::marine::ThrusterArrayFeedback>
  Apply(const intrinsic_proto::hardware::marine::ThrusterArrayCommand &command);

private:
  struct RecordedSlot {
    double thrust_n = 0;
    bool enabled = true;
    bool name_present = false;
    std::string name;
  };

  FakeThrusterArrayConfig config_;
  std::vector<std::vector<RecordedSlot>> history_;
  std::vector<double> slew_thrust_n_;
  uint64_t step_ = 0;
};

} // namespace intrinsic::hardware::marine

#endif // INTRINSIC_HARDWARE_MARINE_FAKE_THRUSTER_ARRAY_H_
