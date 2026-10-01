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

#include "intrinsic/motion_planning/vehicle/vehicle_trajectory_validity.h"

#include <cstddef>
#include <string_view>

#include "intrinsic/safety/clearance_rule.h"
#include "intrinsic/safety/geofence_rule.h"
#include "intrinsic/safety/safety_rule_result.h"
#include "intrinsic/world/world_snapshot/world_snapshot_policy.h"
#include "intrinsic/world/world_snapshot/world_snapshot_skew_policy.h"

namespace intrinsic::motion_planning::vehicle {
namespace {

using ::intrinsic::safety::SafetyRuleResult;

constexpr std::string_view kFrameMismatchText = "frame mismatch";
constexpr std::string_view kSnapshotUnusableText = "snapshot unusable";

TrajectoryValidityResult Fail(TrajectoryValidityError error, int index,
                              std::string_view rule) {
  TrajectoryValidityResult result;
  result.error = error;
  result.first_invalid_sample = index;
  result.failed_rule = rule;
  return result;
}

bool Contains(std::string_view text, std::string_view needle) {
  return text.find(needle) != std::string_view::npos;
}

bool IsCritical(const SafetyRuleResult& rule) {
  return rule.severity == ::intrinsic::safety::kSeverityCritical;
}

// Pre-sample checks 1 through 5. Returns true and fills `failure` on a defect.
bool PreSampleFailure(const TrajectoryValidityRequest& request,
                      TrajectoryValidityResult& failure) {
  if (request.samples.empty()) {
    failure = Fail(TrajectoryValidityError::kBadRequest, -1, {});
    return true;
  }
  if (request.clearance_samples.size() != request.samples.size()) {
    failure = Fail(TrajectoryValidityError::kBadRequest, -1, {});
    return true;
  }
  if (request.snapshot_id.empty() || !request.descriptor.present ||
      request.snapshot_id != request.descriptor.snapshot_id) {
    failure =
        Fail(TrajectoryValidityError::kBadRequest, -1, kValiditySnapshotRule);
    return true;
  }
  if (!::intrinsic::world::AssessWorldSnapshot(request.descriptor).accepted) {
    failure =
        Fail(TrajectoryValidityError::kBadRequest, -1, kValiditySnapshotRule);
    return true;
  }
  if (request.assess_skew) {
    const ::intrinsic::world::WorldSnapshotPolicyAssessment skew =
        ::intrinsic::world::AssessWorldSnapshotSkew(
            request.descriptor, request.timings, request.query_time,
            request.skew_policy);
    if (skew.withhold) {
      failure = Fail(TrajectoryValidityError::kStaleSnapshot, -1,
                     kValiditySnapshotRule);
      return true;
    }
  }
  return false;
}

// Checks 6 through 8 for one sample. Returns true and fills `failure`.
bool SampleFailure(const TrajectoryValidityRequest& request, size_t i,
                   TrajectoryValidityResult& failure) {
  const int index = static_cast<int>(i);
  const PropagationSample& sample = request.samples[i];

  if (Validate(sample.state, request.bounds) != StateSpaceError::kOk) {
    failure =
        Fail(TrajectoryValidityError::kBounds, index, kValidityBoundsRule);
    return true;
  }

  ::intrinsic::safety::GeofencePose pose;
  pose.frame_id =
      request.pose_frame.empty() ? request.fence.frame_id : request.pose_frame;
  pose.x = sample.state.position.x;
  pose.y = sample.state.position.y;
  pose.z = sample.state.position.z;
  const SafetyRuleResult geofence =
      ::intrinsic::safety::EvaluateGeofenceAabbRule(request.fence, pose);
  if (geofence.violated) {
    TrajectoryValidityError error = TrajectoryValidityError::kGeofence;
    if (IsCritical(geofence)) {
      error = Contains(geofence.summary, kFrameMismatchText)
                  ? TrajectoryValidityError::kFrameError
                  : TrajectoryValidityError::kBadRequest;
    }
    failure = Fail(error, index, ::intrinsic::safety::kGeofenceAabbRuleId);
    return true;
  }

  const SafetyRuleResult clearance = ::intrinsic::safety::EvaluateClearanceRule(
      request.clearance_samples[i], request.min_clearance_m);
  if (clearance.violated) {
    const TrajectoryValidityError error =
        IsCritical(clearance) &&
                Contains(clearance.summary, kSnapshotUnusableText)
            ? TrajectoryValidityError::kStaleSnapshot
            : TrajectoryValidityError::kClearance;
    failure = Fail(error, index, ::intrinsic::safety::kClearanceMinRuleId);
    return true;
  }
  return false;
}

}  // namespace

TrajectoryValidityResult CheckTrajectoryValidity(
    const TrajectoryValidityRequest& request) {
  TrajectoryValidityResult failure;
  if (PreSampleFailure(request, failure)) return failure;
  for (size_t i = 0; i < request.samples.size(); ++i) {
    if (SampleFailure(request, i, failure)) return failure;
  }
  TrajectoryValidityResult ok;
  ok.error = TrajectoryValidityError::kOk;
  ok.first_invalid_sample = -1;
  return ok;
}

}  // namespace intrinsic::motion_planning::vehicle
