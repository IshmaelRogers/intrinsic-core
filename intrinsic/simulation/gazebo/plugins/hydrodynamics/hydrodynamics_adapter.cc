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

#include "intrinsic/simulation/gazebo/plugins/hydrodynamics/hydrodynamics_adapter.h"

#include <algorithm>
#include <charconv>
#include <cmath>
#include <string>
#include <utility>

#include "absl/strings/str_cat.h"
#include "intrinsic/embodiment/frame_policy.h"
#include "intrinsic/vehicle/parameters/six_thruster_uuv_example.h"
#include "sdf/Element.hh"
#include "sdf/Root.hh"
#include "sdf/World.hh"

namespace intrinsic::simulation {
namespace {

using ::intrinsic::vehicle::dynamics::BodyWrenchRt;
using ::intrinsic::vehicle::dynamics::Duration;
using ::intrinsic::vehicle::dynamics::DynamicsDiagnostics;
using ::intrinsic::vehicle::dynamics::EnvironmentRt;
using ::intrinsic::vehicle::dynamics::FrameId;
using ::intrinsic::vehicle::dynamics::VehicleStateRt;
using ::intrinsic::vehicle::parameters::kBodyFrameId;
using ::intrinsic::vehicle::parameters::kWorldEnuFrameId;
using ::intrinsic::vehicle::parameters::kWorldNedFrameId;

constexpr std::string_view kSdfParseMessage =
    "hydrodynamics plugin sdf could not be parsed";
constexpr std::string_view kAbsentMessage = "current_field is absent";
constexpr std::string_view kConfiguredMessage = "configured";
constexpr std::string_view kFrameMessage =
    "current_field.frame_id is not world_enu, world_ned, or body";
constexpr std::string_view kVelocityMessage =
    "current_field.velocity_m_s is missing or not finite";
constexpr std::string_view kValidityMessage =
    "current_field.validity was rejected";
constexpr std::string_view kLinkMessage = "target link is unresolved";
constexpr std::string_view kKinematicsMessage =
    "link kinematics are unavailable";
constexpr std::string_view kTimestepMessage =
    "timestep must be finite and strictly positive";
constexpr std::string_view kNonFiniteOutputMessage =
    "dynamics output is not finite";

bool IsSpace(char value) {
  return value == ' ' || value == '\t' || value == '\n' || value == '\r';
}

bool Finite(double value) { return std::isfinite(value); }

bool Finite3(const std::array<double, 3>& value) {
  return Finite(value[0]) && Finite(value[1]) && Finite(value[2]);
}

bool Finite4(const std::array<double, 4>& value) {
  return Finite(value[0]) && Finite(value[1]) && Finite(value[2]) &&
         Finite(value[3]);
}

bool Finite6(const std::array<double, 6>& value) {
  return std::all_of(value.begin(), value.end(), Finite);
}

bool FiniteDiagnostics(const DynamicsDiagnostics& diagnostics) {
  return Finite(diagnostics.dt_s) && Finite3(diagnostics.model_force_n) &&
         Finite3(diagnostics.model_torque_n_m) &&
         Finite6(diagnostics.relative_twist) &&
         Finite6(diagnostics.rigid_body_coriolis_wrench) &&
         Finite6(diagnostics.added_mass_coriolis_wrench) &&
         Finite6(diagnostics.damping_wrench) &&
         Finite6(diagnostics.restoring_wrench) &&
         Finite6(diagnostics.hydrodynamic_wrench) &&
         Finite6(diagnostics.total_wrench);
}

// #121 does not write validity into SDF. A fresh STATE_VALID view lets
// AssessCurrentField keep ownership of the frame and velocity checks.
world::MarineComponentValidityView AcceptedSdfValidity() {
  world::MarineComponentValidityView validity;
  validity.present = true;
  validity.source_id = "hydrodynamics_sdf";
  validity.observation_time_present = true;
  validity.observation_time = world::TimeParts{0, 0};
  validity.validity_horizon_present = true;
  validity.validity_horizon = world::TimeParts{1, 0};
  validity.validity_present = true;
  validity.validity_state = 1;
  return validity;
}

bool ParseThree(std::string_view text, double* x, double* y, double* z) {
  double values[3];
  int count = 0;
  const char* ptr = text.data();
  const char* end = ptr + text.size();
  while (ptr < end && count < 3) {
    while (ptr < end && IsSpace(*ptr)) {
      ++ptr;
    }
    if (ptr == end) {
      break;
    }
    double value = 0;
    const std::from_chars_result parsed = std::from_chars(ptr, end, value);
    if (parsed.ec != std::errc() || parsed.ptr == ptr) {
      return false;
    }
    values[count++] = value;
    ptr = parsed.ptr;
  }
  while (ptr < end && IsSpace(*ptr)) {
    ++ptr;
  }
  if (ptr != end || count != 3) {
    return false;
  }
  *x = values[0];
  *y = values[1];
  *z = values[2];
  return true;
}

// World-from-body quaternion. The conjugate maps a world vector into
// the body frame. embodiment::RotateVector is the existing helper.
std::array<double, 3> WorldVectorToBody(
    const std::array<double, 4>& orientation_xyzw,
    const std::array<double, 3>& world) {
  const embodiment::Quaternion rotation{
      orientation_xyzw[0], orientation_xyzw[1], orientation_xyzw[2],
      orientation_xyzw[3]};
  const embodiment::Quaternion conjugate{-rotation.x, -rotation.y, -rotation.z,
                                         rotation.w};
  const embodiment::Vec3 body = embodiment::RotateVector(
      conjugate, embodiment::Vec3{world[0], world[1], world[2]});
  return {body.x, body.y, body.z};
}

std::array<double, 3> BodyVectorToWorld(
    const std::array<double, 4>& orientation_xyzw,
    const std::array<double, 3>& body) {
  const embodiment::Quaternion rotation{
      orientation_xyzw[0], orientation_xyzw[1], orientation_xyzw[2],
      orientation_xyzw[3]};
  const embodiment::Vec3 world = embodiment::RotateVector(
      rotation, embodiment::Vec3{body[0], body[1], body[2]});
  return {world.x, world.y, world.z};
}

HydrodynamicsStatus ConfigStatus(HydrodynamicsErrorCode code,
                                 std::string_view message,
                                 world::CurrentFieldError field_error) {
  HydrodynamicsStatus status;
  status.code = code;
  status.message = std::string(message);
  status.field_error = field_error;
  return status;
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

}  // namespace

HydrodynamicsAdapter::HydrodynamicsAdapter()
    : HydrodynamicsAdapter(
          ::intrinsic::vehicle::parameters::MakeSixThrusterUuvExample()) {}

HydrodynamicsAdapter::HydrodynamicsAdapter(
    ::intrinsic::vehicle::parameters::MarineModel model)
    : gravity_m_s2_(model.environment.gravity_m_s2),
      fluid_density_kg_m3_(model.environment.fluid_density_kg_m3),
      dynamics_(std::move(model)) {}

HydrodynamicsStepResult HydrodynamicsAdapter::ZeroResult(
    HydrodynamicsStatus status) const {
  HydrodynamicsStepResult result;
  result.status = std::move(status);
  return result;
}

void HydrodynamicsAdapter::StoreConfiguredResult() {
  if (!config_status_.ok()) {
    initial_result_ = ZeroResult(config_status_);
  } else if (!active_) {
    initial_result_ =
        ZeroResult(ConfigStatus(HydrodynamicsErrorCode::kAbsent, kAbsentMessage,
                                world::CurrentFieldError::kNone));
  } else {
    initial_result_ =
        ZeroResult(ConfigStatus(HydrodynamicsErrorCode::kOk, kConfiguredMessage,
                                world::CurrentFieldError::kNone));
  }
  last_result_ = initial_result_;
}

HydrodynamicsStatus HydrodynamicsAdapter::Configure(
    const sdf::Element& plugin_element) {
  active_ = false;
  link_name_.reset();
  current_frame_ = FrameId::kUnspecified;
  current_velocity_m_s_ = {};

  const sdf::ElementConstPtr link = plugin_element.FindElement("link_name");
  if (link != nullptr) {
    const std::string name = link->Get<std::string>();
    if (!name.empty()) {
      link_name_ = name;
    }
  }

  const sdf::ElementConstPtr current =
      plugin_element.FindElement("current_field");
  if (current == nullptr) {
    config_status_ = ConfigStatus(HydrodynamicsErrorCode::kOk, kAbsentMessage,
                                  world::CurrentFieldError::kNone);
    StoreConfiguredResult();
    return config_status_;
  }

  world::CurrentFieldView view;
  view.present = true;
  view.validity = AcceptedSdfValidity();
  std::string frame_storage;
  const sdf::ElementConstPtr frame = current->FindElement("frame_id");
  if (frame != nullptr) {
    frame_storage = frame->Get<std::string>();
  }
  view.frame_id = frame_storage;

  const sdf::ElementConstPtr velocity = current->FindElement("velocity_m_s");
  double vx = 0;
  double vy = 0;
  double vz = 0;
  if (velocity == nullptr ||
      !ParseThree(velocity->Get<std::string>(), &vx, &vy, &vz)) {
    view.velocity_present = false;
  } else {
    view.velocity_present = true;
    view.velocity_m_s = embodiment::Vec3{vx, vy, vz};
  }

  const world::CurrentFieldAssessment assessment =
      world::AssessCurrentField(view, world::TimeParts{0, 0});
  if (assessment.error != world::CurrentFieldError::kNone ||
      !assessment.accepted) {
    std::string_view message = kValidityMessage;
    if (assessment.error == world::CurrentFieldError::kFrameId) {
      message = kFrameMessage;
    } else if (assessment.error == world::CurrentFieldError::kVelocity) {
      message = kVelocityMessage;
    }
    config_status_ = ConfigStatus(HydrodynamicsErrorCode::kInvalidConfig,
                                  message, assessment.error);
    StoreConfiguredResult();
    return config_status_;
  }

  // Gazebo poses are world ENU. A world_ned current is expressed in that
  // ENU basis with the embodiment helper before Evaluate. Dynamics does
  // not swap the axes itself. A body current is passed through.
  if (view.frame_id == kWorldNedFrameId) {
    const embodiment::Vec3 enu =
        embodiment::WorldVectorNedToEnu(view.velocity_m_s);
    current_velocity_m_s_ = {enu.x, enu.y, enu.z};
    current_frame_ = FrameId::kWorldEnu;
  } else if (view.frame_id == kBodyFrameId) {
    current_velocity_m_s_ = {view.velocity_m_s.x, view.velocity_m_s.y,
                             view.velocity_m_s.z};
    current_frame_ = FrameId::kBody;
  } else {
    current_velocity_m_s_ = {view.velocity_m_s.x, view.velocity_m_s.y,
                             view.velocity_m_s.z};
    current_frame_ = FrameId::kWorldEnu;
  }
  active_ = true;
  config_status_ = ConfigStatus(HydrodynamicsErrorCode::kOk, kConfiguredMessage,
                                world::CurrentFieldError::kNone);
  StoreConfiguredResult();
  return config_status_;
}

HydrodynamicsStatus HydrodynamicsAdapter::ConfigureFromPluginXml(
    std::string_view plugin_xml) {
  const std::string wrapped =
      absl::StrCat("<sdf version=\"1.8\"><world name=\"hydrodynamics_parse\">",
                   plugin_xml, "</world></sdf>");
  sdf::Root root;
  // A plugin without filename, or a non-finite velocity token, can make
  // the loader record an error while still leaving the element tree.
  // AssessCurrentField owns frame and velocity defects, so a present
  // plugin element is configured even when the loader reported errors.
  root.LoadSdfString(wrapped);
  const sdf::ElementPtr plugin = FindPluginElement(root);
  if (plugin == nullptr) {
    active_ = false;
    link_name_.reset();
    config_status_ =
        ConfigStatus(HydrodynamicsErrorCode::kInvalidConfig, kSdfParseMessage,
                     world::CurrentFieldError::kNone);
    StoreConfiguredResult();
    return config_status_;
  }
  return Configure(*plugin);
}

HydrodynamicsStepResult HydrodynamicsAdapter::Step(
    const HydrodynamicsSimulatorState& state) {
  if (!config_status_.ok()) {
    last_result_ = ZeroResult(config_status_);
    return last_result_;
  }
  if (!active_) {
    last_result_ =
        ZeroResult(ConfigStatus(HydrodynamicsErrorCode::kAbsent, kAbsentMessage,
                                world::CurrentFieldError::kNone));
    return last_result_;
  }
  if (!state.link_resolved) {
    last_result_ =
        ZeroResult(ConfigStatus(HydrodynamicsErrorCode::kInvalidState,
                                kLinkMessage, world::CurrentFieldError::kNone));
    return last_result_;
  }
  if (!state.kinematics_ready) {
    last_result_ = ZeroResult(
        ConfigStatus(HydrodynamicsErrorCode::kInvalidState, kKinematicsMessage,
                     world::CurrentFieldError::kNone));
    return last_result_;
  }
  if (!Finite3(state.position_m) || !Finite4(state.orientation_xyzw) ||
      !Finite3(state.world_linear_velocity_m_s) ||
      !Finite3(state.world_angular_velocity_rad_s)) {
    last_result_ = ZeroResult(
        ConfigStatus(HydrodynamicsErrorCode::kInvalidState, kKinematicsMessage,
                     world::CurrentFieldError::kNone));
    return last_result_;
  }
  if (!Finite(state.dt_s) || state.dt_s <= 0) {
    last_result_ = ZeroResult(
        ConfigStatus(HydrodynamicsErrorCode::kInvalidState, kTimestepMessage,
                     world::CurrentFieldError::kNone));
    return last_result_;
  }

  VehicleStateRt vehicle_state;
  vehicle_state.pose_frame = FrameId::kWorldEnu;
  vehicle_state.position_m = state.position_m;
  vehicle_state.orientation_xyzw = state.orientation_xyzw;
  const std::array<double, 3> linear = WorldVectorToBody(
      state.orientation_xyzw, state.world_linear_velocity_m_s);
  const std::array<double, 3> angular = WorldVectorToBody(
      state.orientation_xyzw, state.world_angular_velocity_rad_s);
  vehicle_state.body_twist = {linear[0],  linear[1],  linear[2],
                              angular[0], angular[1], angular[2]};

  BodyWrenchRt input;
  input.frame = FrameId::kBody;

  EnvironmentRt environment;
  environment.gravity_m_s2 = gravity_m_s2_;
  environment.fluid_density_kg_m3 = fluid_density_kg_m3_;
  environment.current_velocity_m_s = current_velocity_m_s_;
  environment.current_frame = current_frame_;
  const Duration dt{state.dt_s};

  const auto evaluated =
      dynamics_.Evaluate(vehicle_state, input, environment, dt);

  HydrodynamicsStepResult result;
  result.dynamics_called = true;
  result.dynamics_state = vehicle_state;
  result.dynamics_input_wrench = input;
  result.dynamics_environment = environment;
  result.dynamics_dt = dt;
  if (!evaluated.ok() || !FiniteDiagnostics(evaluated.value().diagnostics)) {
    result.status = ConfigStatus(
        HydrodynamicsErrorCode::kDynamics,
        evaluated.ok() ? kNonFiniteOutputMessage : evaluated.status().message,
        world::CurrentFieldError::kNone);
    if (!evaluated.ok() && FiniteDiagnostics(evaluated.value().diagnostics)) {
      result.diagnostics = evaluated.value().diagnostics;
    }
    last_result_ = std::move(result);
    return last_result_;
  }

  const DynamicsDiagnostics& diagnostics = evaluated.value().diagnostics;
  result.status = ConfigStatus(HydrodynamicsErrorCode::kOk, kConfiguredMessage,
                               world::CurrentFieldError::kNone);
  result.dynamics_evaluated = true;
  result.diagnostics = diagnostics;
  result.body_force_n = diagnostics.model_force_n;
  result.body_torque_n_m = diagnostics.model_torque_n_m;
  result.world_force_n =
      BodyVectorToWorld(state.orientation_xyzw, result.body_force_n);
  result.world_torque_n_m =
      BodyVectorToWorld(state.orientation_xyzw, result.body_torque_n_m);
  if (!Finite3(result.world_force_n) || !Finite3(result.world_torque_n_m)) {
    result = ZeroResult(ConfigStatus(HydrodynamicsErrorCode::kDynamics,
                                     kNonFiniteOutputMessage,
                                     world::CurrentFieldError::kNone));
    result.dynamics_called = true;
    result.dynamics_state = vehicle_state;
    result.dynamics_input_wrench = input;
    result.dynamics_environment = environment;
    result.dynamics_dt = dt;
  }
  last_result_ = std::move(result);
  return last_result_;
}

void HydrodynamicsAdapter::Reset() { last_result_ = initial_result_; }

}  // namespace intrinsic::simulation
