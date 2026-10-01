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

#ifndef INTRINSIC_MOTION_PLANNING_VEHICLE_VEHICLE_KINODYNAMIC_BASELINE_H_
#define INTRINSIC_MOTION_PLANNING_VEHICLE_VEHICLE_KINODYNAMIC_BASELINE_H_

#include <memory>
#include <string>
#include <string_view>

#include "intrinsic/motion_planning/vehicle/vehicle_motion_primitives.h"
#include "intrinsic/motion_planning/vehicle/vehicle_planner_deadline.h"
#include "intrinsic/motion_planning/vehicle/vehicle_planner_registry.h"
#include "intrinsic/motion_planning/vehicle/vehicle_primitive_propagation.h"
#include "intrinsic/motion_planning/vehicle/vehicle_trajectory_validity.h"
#include "intrinsic/vehicle/dynamics/vehicle_dynamics.h"

namespace intrinsic::motion_planning::vehicle {

// Policy: intrinsic_motion_planning/intrinsic/motion_planning/vehicle/
// README.md.
//
// Deterministic discrete kinodynamic lattice Dijkstra (uniform-cost search)
// over GenerateUuvMotionPrimitives. Edges are PropagateUuvMotionPrimitive
// samples accepted by CheckTrajectoryValidity. There is no heuristic, no
// sampling, no smoothing, and no new dynamics model. Dynamics are injected.
//
// String views on KinodynamicBaselineConfig (pose_frame, and the fence
// frame_id / region_id) are borrowed from the caller and must outlive Plan.

// Id stamped when a success request leaves trajectory_id empty.
inline constexpr std::string_view kKinodynamicBaselineTrajectoryId =
    "kinodynamic-baseline";

struct KinodynamicBaselineConfig {
  // Primitive generator input. Must yield kOk with at least one primitive.
  // An empty or rejected set is kInvalidRequest before search.
  VehicleMotionPrimitiveConfig primitives{};

  // Passed unchanged to PropagateUuvMotionPrimitive.
  PropagationConfig propagation{};

  // Validity binding. Clearance samples are not caller-supplied per edge:
  // the planner builds one ClearanceSample per propagated sample from the
  // template below. There is no World occupancy read.
  std::string snapshot_id;
  ::intrinsic::world::WorldSnapshotView descriptor{};
  bool assess_skew = false;
  ::intrinsic::world::ComponentTimings timings{};
  ::intrinsic::world::TimeParts query_time{};
  ::intrinsic::world::WorldSnapshotSkewPolicy skew_policy{};
  VehicleStateBounds bounds{};
  ::intrinsic::safety::AabbGeofence fence{};
  // Empty means fence.frame_id.
  std::string_view pose_frame;
  double min_clearance_m = ::intrinsic::safety::kDefaultMinClearanceM;

  // Applied to every sample of every edge. Must be finite and >= 0.
  double clearance_template_m = 1.0;
  ::intrinsic::safety::ClearanceSource clearance_source =
      ::intrinsic::safety::ClearanceSource::kObstacle;

  // Goal test: Distance(node.state, goal_state) <= goal_tolerance.
  // Must be finite and >= 0.
  double goal_tolerance = 1e-6;

  // Closed-set quantization for position only (meters). Orientation and
  // twist are compared exactly. Must be finite and > 0.
  double position_bin_m = 0.25;

  // Counts validity-accepted child pushes. Must be >= 1. The goal is
  // recognized when that node is dequeued, so a push that fills the budget
  // is not itself dequeued.
  int max_expansions = 256;

  // Not owned. Null means no mid-run poll. Pre-call cancel and deadline are
  // still handled by RunWithDeadline.
  const VehiclePlanRunOptions* run_options = nullptr;
};

// Pure search. `dynamics` is injected (tests use ZeroForceDynamics).
// Does not register itself.
VehiclePlanResult SearchKinodynamicBaseline(
    const KinodynamicBaselineConfig& config, const VehiclePlanRequest& request,
    const ::intrinsic::vehicle::dynamics::VehicleDynamics& dynamics);

class KinodynamicBaselinePlanner : public VehiclePlanner {
 public:
  // Does not take ownership of dynamics. dynamics must outlive Plan calls.
  // A null dynamics pointer yields kInvalidRequest from Plan.
  KinodynamicBaselinePlanner(
      KinodynamicBaselineConfig config,
      const ::intrinsic::vehicle::dynamics::VehicleDynamics* dynamics);

  std::string_view id() const override;
  VehiclePlanResult Plan(const VehiclePlanRequest& request) const override;

 private:
  KinodynamicBaselineConfig config_;
  const ::intrinsic::vehicle::dynamics::VehicleDynamics* dynamics_;
};

std::shared_ptr<KinodynamicBaselinePlanner> MakeKinodynamicBaselinePlanner(
    KinodynamicBaselineConfig config,
    const ::intrinsic::vehicle::dynamics::VehicleDynamics* dynamics);

}  // namespace intrinsic::motion_planning::vehicle

#endif  // INTRINSIC_MOTION_PLANNING_VEHICLE_VEHICLE_KINODYNAMIC_BASELINE_H_
