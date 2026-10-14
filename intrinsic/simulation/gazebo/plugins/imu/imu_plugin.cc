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

#include "intrinsic/simulation/gazebo/plugins/imu/imu_plugin.h"

#include <chrono>
#include <cstdint>
#include <memory>
#include <optional>

#include "absl/log/log.h"
#include "gz/math/Pose3.hh"
#include "gz/math/Vector3.hh"
#include "gz/sim/Entity.hh"
#include "gz/sim/EntityComponentManager.hh"
#include "gz/sim/EventManager.hh"
#include "gz/sim/Link.hh"
#include "gz/sim/Model.hh"
#include "gz/sim/Util.hh"
#include "sdf/Element.hh"

namespace intrinsic::simulation {
namespace {

void EnableImuChecks(gz::sim::Entity link_entity,
                     gz::sim::EntityComponentManager& ecm) {
  const gz::sim::Link link(link_entity);
  link.EnableVelocityChecks(ecm);
  link.EnableAccelerationChecks(ecm);
}

void FillKinematics(gz::sim::Entity link_entity,
                    const gz::sim::EntityComponentManager& ecm,
                    ImuSimulatorState* state) {
  const gz::math::Pose3d pose = gz::sim::worldPose(link_entity, ecm);
  state->orientation_xyzw = {pose.Rot().X(), pose.Rot().Y(), pose.Rot().Z(),
                             pose.Rot().W()};
  const gz::sim::Link link(link_entity);
  const std::optional<gz::math::Vector3d> angular =
      link.WorldAngularVelocity(ecm);
  const std::optional<gz::math::Vector3d> acceleration =
      link.WorldLinearAcceleration(ecm);
  if (!angular.has_value() || !acceleration.has_value()) {
    return;
  }
  state->kinematics_ready = true;
  state->world_angular_velocity_rad_s = {angular->X(), angular->Y(),
                                         angular->Z()};
  state->world_linear_acceleration_m_s2 = {acceleration->X(), acceleration->Y(),
                                           acceleration->Z()};
}

}  // namespace

void Imu::Configure(const gz::sim::Entity& entity,
                    const std::shared_ptr<const sdf::Element>& sdf,
                    gz::sim::EntityComponentManager& ecm,
                    gz::sim::EventManager& /*event_mgr*/) {
  link_entity_ = gz::sim::kNullEntity;
  last_result_ = ImuStepResult();
  last_reported_.reset();
  last_reported_message_.clear();
  model_ = gz::sim::Model(entity);

  if (sdf == nullptr) {
    LOG(ERROR) << "Imu: missing plugin element";
    return;
  }
  const ImuSimStatus config = adapter_.Configure(*sdf);
  if (!config.ok()) {
    LOG(ERROR) << "Imu: " << config.message;
    return;
  }
  if (!model_.Valid(ecm)) {
    LOG(ERROR) << "Imu: plugin must be attached to a model";
    return;
  }
  if (adapter_.link_name().has_value()) {
    link_entity_ = model_.LinkByName(ecm, *adapter_.link_name());
    if (link_entity_ == gz::sim::kNullEntity) {
      LOG(ERROR) << "Imu: unknown link \"" << *adapter_.link_name() << "\"";
    }
  } else {
    link_entity_ = model_.CanonicalLink(ecm);
    if (link_entity_ == gz::sim::kNullEntity) {
      LOG(ERROR) << "Imu: model has no canonical link";
    }
  }
  if (link_entity_ != gz::sim::kNullEntity) {
    EnableImuChecks(link_entity_, ecm);
  }
}

void Imu::Report(const ImuSimStatus& status) {
  if (status.ok()) {
    last_reported_.reset();
    last_reported_message_.clear();
    return;
  }
  if (last_reported_ == status.code &&
      last_reported_message_ == status.message) {
    return;
  }
  LOG(ERROR) << "Imu: " << status.message;
  last_reported_ = status.code;
  last_reported_message_ = status.message;
}

void Imu::PostUpdate(const gz::sim::UpdateInfo& info,
                     const gz::sim::EntityComponentManager& ecm) {
  if (info.paused || !adapter_.active()) {
    return;
  }
  ImuSimulatorState state;
  SplitImuSimTimeNanos(
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

void Imu::Reset(const gz::sim::UpdateInfo& /*info*/,
                gz::sim::EntityComponentManager& ecm) {
  last_reported_.reset();
  last_reported_message_.clear();
  adapter_.Reset();
  last_result_ = ImuStepResult();
  if (link_entity_ != gz::sim::kNullEntity) {
    EnableImuChecks(link_entity_, ecm);
  }
}

}  // namespace intrinsic::simulation
