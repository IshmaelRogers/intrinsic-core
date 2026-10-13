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

#include "intrinsic/simulation/gazebo/plugins/dvl/dvl_plugin.h"

#include <chrono>
#include <cstdint>
#include <optional>
#include <string>

#include "absl/log/log.h"
#include "gz/math/Pose3.hh"
#include "gz/math/Vector3.hh"
#include "gz/sim/Link.hh"
#include "gz/sim/Util.hh"

namespace intrinsic::simulation {
namespace {

void FillKinematics(gz::sim::Entity link_entity,
                    const gz::sim::EntityComponentManager& ecm,
                    DvlSimulatorState* state) {
  const gz::math::Pose3d pose = gz::sim::worldPose(link_entity, ecm);
  state->position_m = {pose.Pos().X(), pose.Pos().Y(), pose.Pos().Z()};
  state->orientation_xyzw = {pose.Rot().X(), pose.Rot().Y(), pose.Rot().Z(),
                             pose.Rot().W()};
  const gz::sim::Link link(link_entity);
  const std::optional<gz::math::Vector3d> linear =
      link.WorldLinearVelocity(ecm);
  if (!linear.has_value()) {
    return;
  }
  state->kinematics_ready = true;
  state->world_linear_velocity_m_s = {linear->X(), linear->Y(), linear->Z()};
}

}  // namespace

void Dvl::Configure(const gz::sim::Entity& entity,
                    const std::shared_ptr<const sdf::Element>& sdf,
                    gz::sim::EntityComponentManager& ecm,
                    gz::sim::EventManager& /*event_mgr*/) {
  link_entity_ = gz::sim::kNullEntity;
  last_result_ = DvlStepResult();
  last_reported_.reset();
  last_reported_message_.clear();
  model_ = gz::sim::Model(entity);

  if (sdf == nullptr) {
    LOG(ERROR) << "Dvl: missing plugin element";
    return;
  }
  const DvlSimStatus config = adapter_.Configure(*sdf);
  if (!config.ok()) {
    LOG(ERROR) << "Dvl: " << config.message;
    return;
  }
  if (!model_.Valid(ecm)) {
    LOG(ERROR) << "Dvl: plugin must be attached to a model";
    return;
  }
  if (adapter_.link_name().has_value()) {
    link_entity_ = model_.LinkByName(ecm, *adapter_.link_name());
    if (link_entity_ == gz::sim::kNullEntity) {
      LOG(ERROR) << "Dvl: unknown link \"" << *adapter_.link_name() << "\"";
    }
  } else {
    link_entity_ = model_.CanonicalLink(ecm);
    if (link_entity_ == gz::sim::kNullEntity) {
      LOG(ERROR) << "Dvl: model has no canonical link";
    }
  }
  if (link_entity_ != gz::sim::kNullEntity) {
    gz::sim::Link(link_entity_).EnableVelocityChecks(ecm);
  }
}

void Dvl::Report(const DvlSimStatus& status) {
  if (status.ok()) {
    last_reported_.reset();
    last_reported_message_.clear();
    return;
  }
  if (last_reported_ == status.code &&
      last_reported_message_ == status.message) {
    return;
  }
  LOG(ERROR) << "Dvl: " << status.message;
  last_reported_ = status.code;
  last_reported_message_ = status.message;
}

void Dvl::PostUpdate(const gz::sim::UpdateInfo& info,
                     const gz::sim::EntityComponentManager& ecm) {
  if (info.paused || !adapter_.active()) {
    return;
  }
  DvlSimulatorState state;
  SplitSimTimeNanos(
      std::chrono::duration_cast<std::chrono::nanoseconds>(info.simTime)
          .count(),
      &state.time_seconds, &state.time_nanos);
  state.link_resolved = link_entity_ != gz::sim::kNullEntity;
  if (state.link_resolved) {
    FillKinematics(link_entity_, ecm, &state);
  }
  last_result_ = adapter_.Step(state);
  Report(last_result_.status);
}

void Dvl::Reset(const gz::sim::UpdateInfo& /*info*/,
                gz::sim::EntityComponentManager& ecm) {
  last_reported_.reset();
  last_reported_message_.clear();
  adapter_.Reset();
  last_result_ = DvlStepResult();
  if (link_entity_ != gz::sim::kNullEntity) {
    gz::sim::Link(link_entity_).EnableVelocityChecks(ecm);
  }
}

}  // namespace intrinsic::simulation
