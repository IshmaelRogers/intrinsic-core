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

#include "intrinsic/simulation/gazebo/plugins/depth/depth_plugin.h"

#include <chrono>
#include <cstdint>
#include <memory>
#include <optional>

#include "absl/log/log.h"
#include "gz/math/Pose3.hh"
#include "gz/sim/Entity.hh"
#include "gz/sim/EntityComponentManager.hh"
#include "gz/sim/EventManager.hh"
#include "gz/sim/Model.hh"
#include "gz/sim/Util.hh"
#include "sdf/Element.hh"

namespace intrinsic::simulation {
namespace {

void FillPose(gz::sim::Entity link_entity,
              const gz::sim::EntityComponentManager& ecm,
              DepthSimulatorState* state) {
  const gz::math::Pose3d pose = gz::sim::worldPose(link_entity, ecm);
  state->position_m = {pose.Pos().X(), pose.Pos().Y(), pose.Pos().Z()};
  state->orientation_xyzw = {pose.Rot().X(), pose.Rot().Y(), pose.Rot().Z(),
                             pose.Rot().W()};
  state->pose_ready = true;
}

}  // namespace

void Depth::Configure(const gz::sim::Entity& entity,
                      const std::shared_ptr<const sdf::Element>& sdf,
                      gz::sim::EntityComponentManager& ecm,
                      gz::sim::EventManager& /*event_mgr*/) {
  link_entity_ = gz::sim::kNullEntity;
  last_result_ = DepthStepResult();
  last_reported_.reset();
  last_reported_message_.clear();
  model_ = gz::sim::Model(entity);

  if (sdf == nullptr) {
    LOG(ERROR) << "Depth: missing plugin element";
    return;
  }
  const DepthSimStatus config = adapter_.Configure(*sdf);
  if (!config.ok()) {
    LOG(ERROR) << "Depth: " << config.message;
    return;
  }
  if (!model_.Valid(ecm)) {
    LOG(ERROR) << "Depth: plugin must be attached to a model";
    return;
  }
  if (adapter_.link_name().has_value()) {
    link_entity_ = model_.LinkByName(ecm, *adapter_.link_name());
    if (link_entity_ == gz::sim::kNullEntity) {
      LOG(ERROR) << "Depth: unknown link \"" << *adapter_.link_name() << "\"";
    }
  } else {
    link_entity_ = model_.CanonicalLink(ecm);
    if (link_entity_ == gz::sim::kNullEntity) {
      LOG(ERROR) << "Depth: model has no canonical link";
    }
  }
}

void Depth::Report(const DepthSimStatus& status) {
  if (status.ok()) {
    last_reported_.reset();
    last_reported_message_.clear();
    return;
  }
  if (last_reported_ == status.code &&
      last_reported_message_ == status.message) {
    return;
  }
  LOG(ERROR) << "Depth: " << status.message;
  last_reported_ = status.code;
  last_reported_message_ = status.message;
}

void Depth::PostUpdate(const gz::sim::UpdateInfo& info,
                       const gz::sim::EntityComponentManager& ecm) {
  if (info.paused || !adapter_.active()) {
    return;
  }
  DepthSimulatorState state;
  SplitDepthSimTimeNanos(
      std::chrono::duration_cast<std::chrono::nanoseconds>(info.simTime)
          .count(),
      &state.time_seconds, &state.time_nanos);
  state.link_resolved = link_entity_ != gz::sim::kNullEntity;
  if (state.link_resolved) {
    FillPose(link_entity_, ecm, &state);
  }
  last_result_ = adapter_.Step(state);
  Report(last_result_.status);
}

void Depth::Reset(const gz::sim::UpdateInfo& /*info*/,
                  gz::sim::EntityComponentManager& /*ecm*/) {
  last_reported_.reset();
  last_reported_message_.clear();
  adapter_.Reset();
  last_result_ = DepthStepResult();
}

}  // namespace intrinsic::simulation
