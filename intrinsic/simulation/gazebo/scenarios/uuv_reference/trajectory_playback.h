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

#ifndef INTRINSIC_SIMULATION_GAZEBO_SCENARIOS_UUV_REFERENCE_TRAJECTORY_PLAYBACK_H_
#define INTRINSIC_SIMULATION_GAZEBO_SCENARIOS_UUV_REFERENCE_TRAJECTORY_PLAYBACK_H_

#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "gz/sim/EntityComponentManager.hh"
#include "gz/sim/System.hh"
#include "intrinsic/motion_planning/vehicle/vehicle_planner_registry.h"
#include "intrinsic/motion_planning/vehicle/vehicle_state_space.h"

namespace intrinsic::simulation::uuv_reference {

// Policy: intrinsic/simulation/gazebo/scenarios/uuv_reference/README.md
// (lock l43-3-trajectory-playback-v1, issue #263).
//
// Sim and test only. A kinematic trajectory-playback mover: in every exact
// step of the playback period it writes the model Pose and the link
// WorldLinearVelocity from a VehiclePlanResult. There is no Physics system, no
// controller, no clock read and no thread. Simulation time comes only from
// UpdateInfo.simTime, in integer nanoseconds.

enum class PlaybackInstallError {
  kOk = 0,
  kPlanNotOk,
  kEmptyTrajectory,
  kTrajectoryRejected,
  kUnsupportedFrame,
  kMissingTimeOrTwist,
  kNonZeroStartTime,
  kStaleInstallStep,
};

struct TrajectoryPlaybackOptions {
  int64_t dt_ns = 125000000;  // Must be > 0.
  std::string model_name = "uuv";
  std::string link_name = "hull";
};

struct PlaybackStep {
  int64_t k = 0;
  int64_t sim_time_ns = 0;
  motion_planning::vehicle::VehiclePlanningState state;
  uint64_t trajectory_serial = 0;  // 1 = initial, +1 per accepted install.
};

// Step rule. k = simTime_ns / dt_ns - 1, and only when simTime_ns is an exact
// multiple of dt_ns. Step k plays trajectory time t = (k + 1) * dt_ns -
// t_start_ns, where t_start_ns = (install_step_k + 1) * dt_ns. The initial
// trajectory counts as installed at k = -1, so the first played time is dt,
// never t = 0.
class TrajectoryPlayback final : public gz::sim::System,
                                 public gz::sim::ISystemPreUpdate {
 public:
  // Returns nullptr and sets *error when the options or the initial plan are
  // invalid. A non-positive dt_ns reports kPlanNotOk because the error enum
  // has no options value. `error` must be non-null.
  static std::shared_ptr<TrajectoryPlayback> Create(
      TrajectoryPlaybackOptions options,
      motion_planning::vehicle::VehiclePlanResult initial_plan,
      PlaybackInstallError* error);

  // Call only between Server::Run calls. Takes effect at step
  // install_step_k + 1. On any error the current and pending trajectories are
  // kept. A later install replaces a pending one that was not applied yet.
  PlaybackInstallError InstallTrajectory(
      motion_planning::vehicle::VehiclePlanResult plan, int64_t install_step_k);

  void PreUpdate(const gz::sim::UpdateInfo& info,
                 gz::sim::EntityComponentManager& ecm) override;

  // Read only after Server::Run has returned.
  const std::vector<PlaybackStep>& steps() const { return steps_; }
  std::optional<PlaybackStep> last_step() const;
  // Steps that were not applied: non-exact time, k < 0, or a repeated k.
  int64_t skipped_steps() const { return skipped_steps_; }
  int64_t interpolation_errors() const { return interpolation_errors_; }
  int64_t missing_entity_steps() const { return missing_entity_steps_; }

 private:
  // A validated plan with its sample times in integer nanoseconds.
  struct Trajectory {
    motion_planning::vehicle::VehiclePlanResult plan;
    std::vector<int64_t> sample_ns;
    std::vector<motion_planning::vehicle::VehiclePlanningState> states;
  };

  struct PendingInstall {
    Trajectory trajectory;
    int64_t install_step_k = 0;
  };

  explicit TrajectoryPlayback(TrajectoryPlaybackOptions options)
      : options_(std::move(options)) {}

  // Validates `plan` per the install rules (everything except staleness) and
  // fills *trajectory on kOk.
  static PlaybackInstallError Validate(
      motion_planning::vehicle::VehiclePlanResult plan, Trajectory* trajectory);

  TrajectoryPlaybackOptions options_;
  Trajectory current_;
  uint64_t serial_ = 1;
  int64_t t_start_ns_ = 0;
  std::optional<PendingInstall> pending_;
  int64_t last_applied_k_ = -1;
  std::vector<PlaybackStep> steps_;
  int64_t skipped_steps_ = 0;
  int64_t interpolation_errors_ = 0;
  int64_t missing_entity_steps_ = 0;
};

}  // namespace intrinsic::simulation::uuv_reference

#endif  // INTRINSIC_SIMULATION_GAZEBO_SCENARIOS_UUV_REFERENCE_TRAJECTORY_PLAYBACK_H_
