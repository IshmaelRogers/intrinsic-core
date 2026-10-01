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

#ifndef INTRINSIC_MOTION_PLANNING_VEHICLE_VEHICLE_TRAJECTORY_VALIDITY_H_
#define INTRINSIC_MOTION_PLANNING_VEHICLE_VEHICLE_TRAJECTORY_VALIDITY_H_

#include <string_view>
#include <vector>

#include "intrinsic/motion_planning/vehicle/vehicle_primitive_propagation.h"
#include "intrinsic/motion_planning/vehicle/vehicle_state_space.h"
#include "intrinsic/safety/clearance_rule.h"
#include "intrinsic/safety/geofence_rule.h"
#include "intrinsic/world/marine_component_validity/marine_component_validity_policy.h"
#include "intrinsic/world/world_snapshot/world_snapshot_policy.h"
#include "intrinsic/world/world_snapshot/world_snapshot_skew_policy.h"

namespace intrinsic::motion_planning::vehicle {

// Policy: intrinsic_motion_planning/intrinsic/motion_planning/vehicle/
// README.md.
//
// Snapshot-backed validity check of one propagated trajectory. Each sample is
// checked against approved bounds, a hard AABB geofence, and an injected
// minimum clearance, all bound to one immutable World snapshot id. Clearance
// is never read from World: the caller supplies one ClearanceSample per
// trajectory sample. There is no search, sampling, trajectory rewriting, World
// mutation, or planner registration.

enum class TrajectoryValidityError {
  kOk = 0,
  // Empty samples, empty or mismatched snapshot id, structural snapshot
  // defect, or clearance sample count different from the sample count.
  kBadRequest = 1,
  // Skew withhold, or a clearance sample with snapshot_usable == false.
  kStaleSnapshot = 2,
  // Geofence frame mismatch (CRITICAL from EvaluateGeofenceAabbRule).
  kFrameError = 3,
  // Validate(state, bounds) != kOk.
  kBounds = 4,
  // EvaluateGeofenceAabbRule found the pose outside the AABB.
  kGeofence = 5,
  // EvaluateClearanceRule violated (below minimum or unknown map).
  kClearance = 6,
};

// Locked failed_rule tokens.
inline constexpr std::string_view kValiditySnapshotRule = "snapshot";
inline constexpr std::string_view kValidityBoundsRule = "bounds";

struct TrajectoryValidityResult {
  TrajectoryValidityError error = TrajectoryValidityError::kBadRequest;
  // Index into `samples` of the first failing sample. -1 when the defect is
  // pre-sample (bad request, or a stale descriptor withhold before samples
  // are walked) and when error == kOk.
  int first_invalid_sample = -1;
  // Rule or check id that failed: "snapshot", "bounds", "geofence.aabb", or
  // "clearance.min". Empty when kOk and for the empty-samples and
  // length-mismatch bad requests. Points at static storage.
  std::string_view failed_rule;
};

struct TrajectoryValidityRequest {
  // Required. Non-empty and byte-wise equal to descriptor.snapshot_id. Prefer
  // the 64-character lowercase hex digest of the WorldSnapshotDescriptor.
  std::string_view snapshot_id;

  // Present descriptor bound to this check. Must have present == true and
  // pass AssessWorldSnapshot.
  ::intrinsic::world::WorldSnapshotView descriptor;

  // When true, AssessWorldSnapshotSkew(descriptor, timings, query_time,
  // skew_policy) runs before any sample. A withhold is kStaleSnapshot with
  // failed_rule "snapshot" and first_invalid_sample -1.
  bool assess_skew = false;
  ::intrinsic::world::ComponentTimings timings;
  ::intrinsic::world::TimeParts query_time{};
  ::intrinsic::world::WorldSnapshotSkewPolicy skew_policy{};

  // Propagated trajectory (from PropagateUuvMotionPrimitive or fixtures).
  // Must be non-empty.
  std::vector<PropagationSample> samples;

  // Approved planning bounds, checked with Validate.
  VehicleStateBounds bounds{};

  // Hard AABB geofence. Poses are built in `pose_frame`. The caller keeps one
  // world frame; there is no ENU to NED conversion.
  ::intrinsic::safety::AabbGeofence fence{};

  // Frame id attached to every sample position before the geofence check.
  // Empty (the default) means fence.frame_id. A different value is the test
  // seam that exercises the geofence frame mismatch path.
  std::string_view pose_frame;

  // One injected ClearanceSample per trajectory sample, same length.
  std::vector<::intrinsic::safety::ClearanceSample> clearance_samples;
  double min_clearance_m = ::intrinsic::safety::kDefaultMinClearanceM;
};

// Pure adapter. No search. No World mutation. No planner registration.
//
// Check order, first defect wins. Pre-sample (first_invalid_sample == -1):
//  1. samples empty: kBadRequest, empty failed_rule.
//  2. clearance_samples.size() != samples.size(): kBadRequest, empty
//     failed_rule.
//  3. snapshot_id empty, descriptor not present, or snapshot_id !=
//     descriptor.snapshot_id: kBadRequest, "snapshot".
//  4. AssessWorldSnapshot(descriptor) not accepted: kBadRequest, "snapshot".
//  5. assess_skew and the assessment withholds: kStaleSnapshot, "snapshot".
// Then per sample i in order, stopping at the first failure:
//  6. Validate(state, bounds) != kOk: kBounds, "bounds".
//  7. EvaluateGeofenceAabbRule on GeofencePose{pose_frame or fence.frame_id,
//     position}: CRITICAL frame mismatch is kFrameError, any other CRITICAL is
//     kBadRequest, and a violated (outside) result is kGeofence, all with
//     "geofence.aabb".
//  8. EvaluateClearanceRule(clearance_samples[i], min_clearance_m): CRITICAL
//     snapshot unusable is kStaleSnapshot, any other violation is
//     kClearance, all with "clearance.min".
//  9. All samples pass: kOk.
TrajectoryValidityResult CheckTrajectoryValidity(
    const TrajectoryValidityRequest& request);

}  // namespace intrinsic::motion_planning::vehicle

#endif  // INTRINSIC_MOTION_PLANNING_VEHICLE_VEHICLE_TRAJECTORY_VALIDITY_H_
