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

#include <algorithm>
#include <array>
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

}  // namespace

void Imu::FillKinematics(const gz::sim::UpdateInfo& info,
                         const gz::sim::EntityComponentManager& ecm,
                         ImuSimulatorState* state) {
  const gz::sim::Entity link_entity = link_entity_;
  const gz::math::Pose3d pose = gz::sim::worldPose(link_entity, ecm);
  state->orientation_xyzw = {pose.Rot().X(), pose.Rot().Y(), pose.Rot().Z(),
                             pose.Rot().W()};
  const gz::sim::Link link(link_entity);
  const std::optional<gz::math::Vector3d> angular =
      link.WorldAngularVelocity(ecm);
  const std::optional<gz::math::Vector3d> velocity =
      link.WorldLinearVelocity(ecm);
  if (!angular.has_value() || !velocity.has_value()) {
    return;
  }

  // Engines do not all fill the link world linear acceleration (for example
  // bullet-featherstone leaves it 0), so it is differenced from the world
  // linear velocity. bullet-featherstone only republishes a link when its
  // world pose changes by more than 1e-6, so a bit-identical pose means the
  // link was not republished on this step (gated). The compare is exact on
  // every component. gz::math::Pose3d::operator== has a tolerance, and the
  // ECM change flags depend on system ordering, so neither is used.
  const std::array<double, 7> pose_values = {
      pose.Pos().X(), pose.Pos().Y(), pose.Pos().Z(), pose.Rot().X(),
      pose.Rot().Y(), pose.Rot().Z(), pose.Rot().W()};
  const bool fresh = !previous_pose_.has_value() ||
                     !std::equal(pose_values.begin(), pose_values.end(),
                                 previous_pose_->begin());
  previous_pose_ = pose_values;

  std::array<double, 3> acceleration = {0.0, 0.0, 0.0};
  if (fresh) {
    const std::optional<gz::math::Vector3d> fresh_velocity = fresh_velocity_;
    const std::chrono::steady_clock::duration fresh_time = fresh_time_;
    fresh_velocity_ = velocity;
    fresh_time_ = info.simTime;
    if (!fresh_velocity.has_value()) {
      // Warm-up: no previous fresh velocity, so no sample.
      freshness_.fresh = true;
      freshness_.steps_since_fresh = 0;
      return;
    }
    const double dt_s =
        std::chrono::duration<double>(info.simTime - fresh_time).count();
    if (!(dt_s > 0.0)) {
      return;
    }
    const gz::math::Vector3d difference = (*velocity - *fresh_velocity) / dt_s;
    acceleration = {difference.X(), difference.Y(), difference.Z()};
    freshness_.fresh = true;
    freshness_.steps_since_fresh = 0;
  } else {
    freshness_.fresh = false;
    ++freshness_.steps_since_fresh;
    ++freshness_.gated_steps;
  }
  state->kinematics_ready = true;
  state->world_angular_velocity_rad_s = {angular->X(), angular->Y(),
                                         angular->Z()};
  state->world_linear_acceleration_m_s2 = acceleration;
}

void Imu::ClearKinematicsHistory() {
  previous_pose_.reset();
  fresh_velocity_.reset();
  fresh_time_ = std::chrono::steady_clock::duration::zero();
  freshness_ = ImuKinematicsFreshness();
}

void Imu::Configure(const gz::sim::Entity& entity,
                    const std::shared_ptr<const sdf::Element>& sdf,
                    gz::sim::EntityComponentManager& ecm,
                    gz::sim::EventManager& /*event_mgr*/) {
  link_entity_ = gz::sim::kNullEntity;
  ClearKinematicsHistory();
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
    FillKinematics(info, ecm, &state);
  }
  last_result_ = adapter_.Step(state);
  Report(last_result_.status);
}

void Imu::Reset(const gz::sim::UpdateInfo& /*info*/,
                gz::sim::EntityComponentManager& ecm) {
  last_reported_.reset();
  last_reported_message_.clear();
  adapter_.Reset();
  ClearKinematicsHistory();
  last_result_ = ImuStepResult();
  if (link_entity_ != gz::sim::kNullEntity) {
    EnableImuChecks(link_entity_, ecm);
  }
}

}  // namespace intrinsic::simulation
