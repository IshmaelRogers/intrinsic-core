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

#include "intrinsic/simulation/gazebo/scenarios/uuv_reference/trajectory_playback.h"

#include <algorithm>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>
#include <utility>
#include <vector>

#include "gz/math/Pose3.hh"
#include "gz/math/Quaternion.hh"
#include "gz/math/Vector3.hh"
#include "gz/sim/Entity.hh"
#include "gz/sim/EntityComponentManager.hh"
#include "gz/sim/System.hh"
#include "gz/sim/components/LinearVelocity.hh"
#include "gz/sim/components/Link.hh"
#include "gz/sim/components/Model.hh"
#include "gz/sim/components/Name.hh"
#include "gz/sim/components/Pose.hh"
#include "intrinsic/embodiment/frame_policy.h"
#include "intrinsic/motion_planning/vehicle/vehicle_planner_registry.h"
#include "intrinsic/motion_planning/vehicle/vehicle_state_space.h"
#include "intrinsic/vehicle/trajectory_contract_policy.h"

namespace intrinsic::simulation::uuv_reference {
namespace {

using ::intrinsic::motion_planning::vehicle::AsVehicleTrajectoryView;
using ::intrinsic::motion_planning::vehicle::Interpolate;
using ::intrinsic::motion_planning::vehicle::InterpolateResult;
using ::intrinsic::motion_planning::vehicle::StateSpaceError;
using ::intrinsic::motion_planning::vehicle::VehiclePlanningState;
using ::intrinsic::motion_planning::vehicle::VehiclePlanResult;
using ::intrinsic::motion_planning::vehicle::VehiclePlanStatus;
using ::intrinsic::vehicle::AssessVehicleTrajectory;
using ::intrinsic::vehicle::TrajectorySampleView;

constexpr int64_t kNanosPerSecond = 1000000000;

int64_t SampleNs(const TrajectorySampleView& sample) {
  return sample.seconds * kNanosPerSecond + sample.nanos;
}

VehiclePlanningState SampleState(const TrajectorySampleView& sample) {
  VehiclePlanningState state;
  state.position = sample.position;
  state.orientation = sample.orientation;
  state.twist = sample.twist;
  return state;
}

}  // namespace

PlaybackInstallError TrajectoryPlayback::Validate(VehiclePlanResult plan,
                                                  Trajectory* trajectory) {
  if (plan.status != VehiclePlanStatus::kOk) {
    return PlaybackInstallError::kPlanNotOk;
  }
  if (plan.samples.empty()) {
    return PlaybackInstallError::kEmptyTrajectory;
  }
  if (!AssessVehicleTrajectory(AsVehicleTrajectoryView(plan)).accepted) {
    return PlaybackInstallError::kTrajectoryRejected;
  }
  if (plan.frame_id != embodiment::kWorldEnuFrameId) {
    return PlaybackInstallError::kUnsupportedFrame;
  }
  for (const TrajectorySampleView& sample : plan.samples) {
    if (!sample.time_present || !sample.twist_present) {
      return PlaybackInstallError::kMissingTimeOrTwist;
    }
  }
  if (SampleNs(plan.samples.front()) != 0) {
    return PlaybackInstallError::kNonZeroStartTime;
  }
  trajectory->sample_ns.clear();
  trajectory->states.clear();
  trajectory->sample_ns.reserve(plan.samples.size());
  trajectory->states.reserve(plan.samples.size());
  for (const TrajectorySampleView& sample : plan.samples) {
    trajectory->sample_ns.push_back(SampleNs(sample));
    trajectory->states.push_back(SampleState(sample));
  }
  trajectory->plan = std::move(plan);
  return PlaybackInstallError::kOk;
}

std::shared_ptr<TrajectoryPlayback> TrajectoryPlayback::Create(
    TrajectoryPlaybackOptions options, VehiclePlanResult initial_plan,
    PlaybackInstallError* error) {
  if (options.dt_ns <= 0) {
    *error = PlaybackInstallError::kPlanNotOk;
    return nullptr;
  }
  std::shared_ptr<TrajectoryPlayback> playback(
      new TrajectoryPlayback(std::move(options)));
  *error = Validate(std::move(initial_plan), &playback->current_);
  if (*error != PlaybackInstallError::kOk) {
    return nullptr;
  }
  return playback;
}

PlaybackInstallError TrajectoryPlayback::InstallTrajectory(
    VehiclePlanResult plan, int64_t install_step_k) {
  PendingInstall install;
  install.install_step_k = install_step_k;
  const PlaybackInstallError error =
      Validate(std::move(plan), &install.trajectory);
  if (error != PlaybackInstallError::kOk) {
    return error;
  }
  if (install_step_k < last_applied_k_) {
    return PlaybackInstallError::kStaleInstallStep;
  }
  pending_ = std::move(install);
  return PlaybackInstallError::kOk;
}

std::optional<PlaybackStep> TrajectoryPlayback::last_step() const {
  if (steps_.empty()) {
    return std::nullopt;
  }
  return steps_.back();
}

void TrajectoryPlayback::PreUpdate(const gz::sim::UpdateInfo& info,
                                   gz::sim::EntityComponentManager& ecm) {
  if (info.paused) {
    return;
  }
  const int64_t ns =
      std::chrono::duration_cast<std::chrono::nanoseconds>(info.simTime)
          .count();
  if (ns % options_.dt_ns != 0) {
    ++skipped_steps_;
    return;
  }
  const int64_t k = ns / options_.dt_ns - 1;
  if (k < 0 || k <= last_applied_k_) {
    ++skipped_steps_;
    return;
  }

  if (pending_.has_value() && pending_->install_step_k < k) {
    current_ = std::move(pending_->trajectory);
    t_start_ns_ = (pending_->install_step_k + 1) * options_.dt_ns;
    ++serial_;
    pending_.reset();
  }

  const int64_t t_ns = (k + 1) * options_.dt_ns - t_start_ns_;
  const std::vector<int64_t>& times = current_.sample_ns;
  const std::vector<VehiclePlanningState>& states = current_.states;
  VehiclePlanningState state;
  if (t_ns >= times.back()) {
    state = states.back();
  } else {
    // times.front() == 0 < t_ns < times.back(), so i + 1 is a valid index.
    const size_t i = static_cast<size_t>(
        std::upper_bound(times.begin(), times.end(), t_ns) - times.begin() - 1);
    if (t_ns == times[i]) {
      state = states[i];
    } else {
      const double u = static_cast<double>(t_ns - times[i]) /
                       static_cast<double>(times[i + 1] - times[i]);
      const InterpolateResult result = Interpolate(states[i], states[i + 1], u);
      if (result.error != StateSpaceError::kOk) {
        ++interpolation_errors_;
        return;
      }
      state = result.state;
    }
  }

  const gz::sim::Entity model =
      ecm.EntityByComponents(gz::sim::components::Name(options_.model_name),
                             gz::sim::components::Model());
  const gz::sim::Entity link =
      ecm.EntityByComponents(gz::sim::components::Name(options_.link_name),
                             gz::sim::components::Link());
  if (model == gz::sim::kNullEntity || link == gz::sim::kNullEntity) {
    ++missing_entity_steps_;
    return;
  }

  const embodiment::Quaternion& q = state.orientation;
  const gz::math::Pose3d pose(
      gz::math::Vector3d(state.position.x, state.position.y, state.position.z),
      gz::math::Quaterniond(q.w, q.x, q.y, q.z));
  const embodiment::Vec3 velocity = embodiment::RotateVector(
      q, embodiment::Vec3{state.twist.linear_x, state.twist.linear_y,
                          state.twist.linear_z});
  ecm.SetComponentData<gz::sim::components::Pose>(model, pose);
  ecm.SetComponentData<gz::sim::components::WorldLinearVelocity>(
      link, gz::math::Vector3d(velocity.x, velocity.y, velocity.z));

  steps_.push_back(PlaybackStep{k, ns, state, serial_});
  last_applied_k_ = k;
}

}  // namespace intrinsic::simulation::uuv_reference
