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

#include "intrinsic/simulation/gazebo/plugins/hydrodynamics/hydrodynamics_plugin.h"

#include <array>
#include <chrono>
#include <string>

#include "absl/log/log.h"
#include "gz/math/Pose3.hh"
#include "gz/math/Vector3.hh"
#include "gz/sim/Link.hh"
#include "gz/sim/Util.hh"
#include "intrinsic/simulation/gazebo/plugins/priority_constants.h"

namespace intrinsic::simulation {
namespace {

double StepSeconds(const gz::sim::UpdateInfo& info) {
  return std::chrono::duration<double>(info.dt).count();
}

gz::math::Vector3d ToVector(const std::array<double, 3>& value) {
  return gz::math::Vector3d(value[0], value[1], value[2]);
}

void FillKinematics(gz::sim::Entity link_entity,
                    const gz::sim::EntityComponentManager& ecm,
                    HydrodynamicsSimulatorState* state) {
  const gz::math::Pose3d pose = gz::sim::worldPose(link_entity, ecm);
  state->position_m = {pose.Pos().X(), pose.Pos().Y(), pose.Pos().Z()};
  state->orientation_xyzw = {pose.Rot().X(), pose.Rot().Y(), pose.Rot().Z(),
                             pose.Rot().W()};
  const gz::sim::Link link(link_entity);
  const std::optional<gz::math::Vector3d> linear =
      link.WorldLinearVelocity(ecm);
  const std::optional<gz::math::Vector3d> angular =
      link.WorldAngularVelocity(ecm);
  if (!linear.has_value() || !angular.has_value()) {
    return;
  }
  state->kinematics_ready = true;
  state->world_linear_velocity_m_s = {linear->X(), linear->Y(), linear->Z()};
  state->world_angular_velocity_rad_s = {angular->X(), angular->Y(),
                                         angular->Z()};
}

}  // namespace

void Hydrodynamics::Configure(const gz::sim::Entity& entity,
                              const std::shared_ptr<const sdf::Element>& sdf,
                              gz::sim::EntityComponentManager& ecm,
                              gz::sim::EventManager& /*event_mgr*/) {
  link_entity_ = gz::sim::kNullEntity;
  gravity_disabled_ = false;
  last_reported_.reset();
  last_reported_message_.clear();
  model_ = gz::sim::Model(entity);

  if (sdf == nullptr) {
    LOG(ERROR) << "Hydrodynamics: missing plugin element";
    return;
  }
  const HydrodynamicsStatus config = adapter_.Configure(*sdf);
  if (!config.ok()) {
    LOG(ERROR) << "Hydrodynamics: " << config.message;
  }
  if (!adapter_.active()) {
    return;
  }
  if (!model_.Valid(ecm)) {
    LOG(ERROR) << "Hydrodynamics: plugin must be attached to a model";
    return;
  }
  if (adapter_.link_name().has_value()) {
    link_entity_ = model_.LinkByName(ecm, *adapter_.link_name());
    if (link_entity_ == gz::sim::kNullEntity) {
      LOG(ERROR) << "Hydrodynamics: unknown link \"" << *adapter_.link_name()
                 << "\"";
    }
  } else {
    link_entity_ = model_.CanonicalLink(ecm);
    if (link_entity_ == gz::sim::kNullEntity) {
      LOG(ERROR) << "Hydrodynamics: model has no canonical link";
    }
  }
  if (link_entity_ != gz::sim::kNullEntity) {
    gz::sim::Link(link_entity_).EnableVelocityChecks(ecm);
  }
}

gz::sim::System::PriorityType Hydrodynamics::ConfigurePriority() {
  // Before physics, matching the existing pre-physics command systems.
  return plugins::kHardwareModuleLauncherPriority;
}

void Hydrodynamics::Report(const HydrodynamicsStepResult& result) {
  if (result.status.code == HydrodynamicsErrorCode::kOk ||
      result.status.code == HydrodynamicsErrorCode::kAbsent) {
    last_reported_.reset();
    last_reported_message_.clear();
    return;
  }
  if (last_reported_ == result.status.code &&
      last_reported_message_ == result.status.message) {
    return;
  }
  LOG(ERROR) << "Hydrodynamics: " << result.status.message;
  last_reported_ = result.status.code;
  last_reported_message_ = result.status.message;
}

void Hydrodynamics::PreUpdate(const gz::sim::UpdateInfo& info,
                              gz::sim::EntityComponentManager& ecm) {
  if (info.paused || !adapter_.active()) {
    return;
  }
  HydrodynamicsSimulatorState state;
  state.dt_s = StepSeconds(info);
  state.link_resolved = link_entity_ != gz::sim::kNullEntity;
  if (state.link_resolved) {
    FillKinematics(link_entity_, ecm, &state);
  }
  const HydrodynamicsStepResult result = adapter_.Step(state);
  Report(result);
  if (link_entity_ == gz::sim::kNullEntity) {
    return;
  }
  gz::sim::Link link(link_entity_);
  if (AppliesWrench(result)) {
    link.SetGravityEnabled(ecm, false);
    gravity_disabled_ = true;
    link.AddWorldWrench(ecm, ToVector(result.world_force_n),
                        ToVector(result.world_torque_n_m));
    return;
  }
  if (gravity_disabled_) {
    link.SetGravityEnabled(ecm, true);
    gravity_disabled_ = false;
  }
}

void Hydrodynamics::Reset(const gz::sim::UpdateInfo& /*info*/,
                          gz::sim::EntityComponentManager& ecm) {
  if (gravity_disabled_ && link_entity_ != gz::sim::kNullEntity) {
    gz::sim::Link(link_entity_).SetGravityEnabled(ecm, true);
  }
  gravity_disabled_ = false;
  last_reported_.reset();
  last_reported_message_.clear();
  adapter_.Reset();
  if (link_entity_ != gz::sim::kNullEntity) {
    gz::sim::Link(link_entity_).EnableVelocityChecks(ecm);
  }
}

}  // namespace intrinsic::simulation
