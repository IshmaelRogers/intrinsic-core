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

#ifndef INTRINSIC_SIMULATION_GAZEBO_PLUGINS_HYDRODYNAMICS_HYDRODYNAMICS_ADAPTER_H_
#define INTRINSIC_SIMULATION_GAZEBO_PLUGINS_HYDRODYNAMICS_HYDRODYNAMICS_ADAPTER_H_

#include <array>
#include <optional>
#include <string>
#include <string_view>

#include "intrinsic/vehicle/dynamics/marine_force_dynamics.h"
#include "intrinsic/world/current_field_component/current_field_component_policy.h"
#include "sdf/Element.hh"

// Simulator-free adapter from a #121 hydrodynamics plugin element to one
// intrinsic_vehicle dynamics evaluation. The Gazebo system plugin is a
// separate wrapper. This class does not include gz-sim.

namespace intrinsic::simulation {

// Append codes. kOk applies a wrench. kAbsent is the opt-in no-op.
enum class HydrodynamicsErrorCode {
  kOk = 0,
  kAbsent = 1,
  kInvalidConfig = 2,
  kInvalidState = 3,
  kDynamics = 4,
};

struct HydrodynamicsStatus {
  HydrodynamicsErrorCode code = HydrodynamicsErrorCode::kOk;
  std::string message;
  // Set when code is kInvalidConfig from AssessCurrentField. Otherwise kNone.
  world::CurrentFieldError field_error = world::CurrentFieldError::kNone;

  [[nodiscard]] bool ok() const {
    return code == HydrodynamicsErrorCode::kOk ||
           code == HydrodynamicsErrorCode::kAbsent;
  }
};

// One sample of simulator state, already copied out of the simulator.
// Position and orientation are the world-ENU pose of the body origin.
// Velocities are of the body origin, expressed in that same world frame.
// The adapter rotates them into the body frame with the embodiment helper.
struct HydrodynamicsSimulatorState {
  bool link_resolved = false;
  bool kinematics_ready = false;
  std::array<double, 3> position_m = {};
  // Hamilton quaternion, world-from-body, stored x, y, z, w.
  std::array<double, 4> orientation_xyzw = {0, 0, 0, 1};
  std::array<double, 3> world_linear_velocity_m_s = {};
  std::array<double, 3> world_angular_velocity_rad_s = {};
  // Simulation step in seconds. Zero and negative are rejected.
  double dt_s = 0;
};

struct HydrodynamicsStepResult {
  HydrodynamicsStatus status;
  // True only after a successful MarineForceDynamics::Evaluate.
  bool dynamics_evaluated = false;
  // Body-frame hydrodynamic wrench. Zero unless dynamics_evaluated.
  std::array<double, 3> body_force_n = {};
  std::array<double, 3> body_torque_n_m = {};
  // Same wrench expressed in world ENU. Zero unless dynamics_evaluated.
  std::array<double, 3> world_force_n = {};
  std::array<double, 3> world_torque_n_m = {};
  // Diagnostics from the evaluation. Zero-initialized when dynamics did
  // not return a finite success. On success this is the dynamics value.
  ::intrinsic::vehicle::dynamics::DynamicsDiagnostics diagnostics;
  // Inputs passed to Evaluate. Meaningful when dynamics_evaluated, and
  // also when a dynamics call was attempted and rejected.
  bool dynamics_called = false;
  ::intrinsic::vehicle::dynamics::VehicleStateRt dynamics_state;
  ::intrinsic::vehicle::dynamics::BodyWrenchRt dynamics_input_wrench;
  ::intrinsic::vehicle::dynamics::EnvironmentRt dynamics_environment;
  ::intrinsic::vehicle::dynamics::Duration dynamics_dt;
};

[[nodiscard]] inline bool AppliesWrench(const HydrodynamicsStepResult& result) {
  return result.dynamics_evaluated &&
         result.status.code == HydrodynamicsErrorCode::kOk;
}

// Parses the #121 <current_field> element and evaluates
// MarineForceDynamics. Vehicle coefficients, gravity, and density come
// from the stored marine model. The default model is
// MakeSixThrusterUuvExample. The commanded input wrench is zero.
//
// Fail closed: any invalid config, unresolved link, non-positive time
// step, or dynamics error yields a typed status and a zero wrench.
class HydrodynamicsAdapter {
 public:
  HydrodynamicsAdapter();
  explicit HydrodynamicsAdapter(
      ::intrinsic::vehicle::parameters::MarineModel model);

  HydrodynamicsAdapter(const HydrodynamicsAdapter&) = delete;
  HydrodynamicsAdapter& operator=(const HydrodynamicsAdapter&) = delete;

  // `plugin_element` is the <plugin> element. No <current_field> child
  // selects the absent (opt-in) state. A present child is checked with
  // AssessCurrentField.
  HydrodynamicsStatus Configure(const sdf::Element& plugin_element);

  // Wraps `plugin_xml` in a minimal SDF world and configures from the
  // first plugin element. `plugin_xml` is the #121 fragment, including
  // the <plugin> tag.
  HydrodynamicsStatus ConfigureFromPluginXml(std::string_view plugin_xml);

  [[nodiscard]] HydrodynamicsStepResult Step(
      const HydrodynamicsSimulatorState& state);

  // Restores the result recorded at the end of the last Configure.
  void Reset();

  [[nodiscard]] bool active() const { return active_; }
  [[nodiscard]] const std::optional<std::string>& link_name() const {
    return link_name_;
  }
  [[nodiscard]] const HydrodynamicsStatus& config_status() const {
    return config_status_;
  }
  [[nodiscard]] const HydrodynamicsStepResult& last_result() const {
    return last_result_;
  }

 private:
  HydrodynamicsStepResult ZeroResult(HydrodynamicsStatus status) const;
  void StoreConfiguredResult();

  HydrodynamicsStatus config_status_;
  bool active_ = false;
  std::optional<std::string> link_name_;
  double gravity_m_s2_ = 0;
  double fluid_density_kg_m3_ = 0;
  ::intrinsic::vehicle::dynamics::FrameId current_frame_ =
      ::intrinsic::vehicle::dynamics::FrameId::kUnspecified;
  std::array<double, 3> current_velocity_m_s_ = {};
  ::intrinsic::vehicle::dynamics::MarineForceDynamics dynamics_;
  HydrodynamicsStepResult initial_result_;
  HydrodynamicsStepResult last_result_;
};

}  // namespace intrinsic::simulation

#endif  // INTRINSIC_SIMULATION_GAZEBO_PLUGINS_HYDRODYNAMICS_HYDRODYNAMICS_ADAPTER_H_
