# Copyright 2026 Intrinsic Innovation LLC
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     https://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Snapshot-backed validity check of one propagated trajectory.

Policy: intrinsic_motion_planning/intrinsic/motion_planning/vehicle/README.md.

Python mirror of vehicle_trajectory_validity.h. Each sample is checked against
approved bounds, a hard AABB geofence, and an injected minimum clearance, all
bound to one immutable World snapshot id. Clearance is never read from World:
the caller supplies one ClearanceSample per trajectory sample. There is no
search, sampling, trajectory rewriting, World mutation, or planner
registration.
"""

from collections.abc import Sequence
import dataclasses
import enum
from typing import Optional

from intrinsic.motion_planning.vehicle import vehicle_primitive_propagation
from intrinsic.motion_planning.vehicle import vehicle_state_space
from intrinsic.safety import clearance_rule
from intrinsic.safety import geofence_rule
from intrinsic.safety import safety_rule_result
from intrinsic.world.world_snapshot import world_snapshot_policy
from intrinsic.world.world_snapshot import world_snapshot_skew_policy

PropagationSample = vehicle_primitive_propagation.PropagationSample
VehicleStateBounds = vehicle_state_space.VehicleStateBounds
WorldSnapshotView = world_snapshot_policy.WorldSnapshotView
ComponentTiming = world_snapshot_skew_policy.ComponentTiming
WorldSnapshotSkewPolicy = world_snapshot_skew_policy.WorldSnapshotSkewPolicy
AabbGeofence = geofence_rule.AabbGeofence
ClearanceSample = clearance_rule.ClearanceSample
ClearanceSource = clearance_rule.ClearanceSource

SNAPSHOT_RULE = "snapshot"
BOUNDS_RULE = "bounds"
GEOFENCE_AABB_RULE = geofence_rule.GEOFENCE_AABB_RULE_ID
CLEARANCE_MIN_RULE = clearance_rule.CLEARANCE_MIN_RULE_ID

_FRAME_MISMATCH_TEXT = "frame mismatch"
_SNAPSHOT_UNUSABLE_TEXT = "snapshot unusable"


class TrajectoryValidityError(enum.Enum):
  OK = 0
  # Empty samples, empty or mismatched snapshot id, structural snapshot
  # defect, or clearance sample count different from the sample count.
  BAD_REQUEST = 1
  # Skew withhold, or a clearance sample with snapshot_usable false.
  STALE_SNAPSHOT = 2
  # Geofence frame mismatch (CRITICAL from evaluate_geofence_aabb_rule).
  FRAME_ERROR = 3
  # validate(state, bounds) is not OK.
  BOUNDS = 4
  # The pose is outside the AABB.
  GEOFENCE = 5
  # evaluate_clearance_rule violated (below minimum or unknown map).
  CLEARANCE = 6


@dataclasses.dataclass(frozen=True)
class TrajectoryValidityResult:
  """Result of one validity check.

  first_invalid_sample indexes `samples`. It is -1 for pre-sample defects and
  for OK. failed_rule is "snapshot", "bounds", "geofence.aabb", or
  "clearance.min", and is empty for OK and for the empty-samples and
  length-mismatch bad requests.
  """

  error: TrajectoryValidityError = TrajectoryValidityError.BAD_REQUEST
  first_invalid_sample: int = -1
  failed_rule: str = ""


@dataclasses.dataclass(frozen=True)
class TrajectoryValidityRequest:
  """One validity check. Field meanings match the C++ request.

  `pose_frame` empty (the default) means `fence.frame_id`. A different value
  is the test seam that exercises the geofence frame mismatch path.
  """

  snapshot_id: str = ""
  descriptor: WorldSnapshotView = WorldSnapshotView()
  assess_skew: bool = False
  timings: Sequence[ComponentTiming] = ()
  query_time: world_snapshot_skew_policy.TimeParts = (0, 0)
  skew_policy: WorldSnapshotSkewPolicy = WorldSnapshotSkewPolicy()
  samples: Sequence[PropagationSample] = ()
  bounds: VehicleStateBounds = VehicleStateBounds()
  fence: AabbGeofence = AabbGeofence("", "", 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
  pose_frame: str = ""
  clearance_samples: Sequence[ClearanceSample] = ()
  min_clearance_m: float = clearance_rule.DEFAULT_MIN_CLEARANCE_M


def _fail(
    error: TrajectoryValidityError, index: int, rule: str
) -> TrajectoryValidityResult:
  return TrajectoryValidityResult(error, index, rule)


def _pre_sample_failure(
    request: TrajectoryValidityRequest,
) -> Optional[TrajectoryValidityResult]:
  if not request.samples:
    return _fail(TrajectoryValidityError.BAD_REQUEST, -1, "")
  if len(request.clearance_samples) != len(request.samples):
    return _fail(TrajectoryValidityError.BAD_REQUEST, -1, "")
  if (
      not request.snapshot_id
      or not request.descriptor.present
      or request.snapshot_id != request.descriptor.snapshot_id
  ):
    return _fail(TrajectoryValidityError.BAD_REQUEST, -1, SNAPSHOT_RULE)
  if not world_snapshot_policy.assess_world_snapshot(
      request.descriptor
  ).accepted:
    return _fail(TrajectoryValidityError.BAD_REQUEST, -1, SNAPSHOT_RULE)
  if request.assess_skew:
    skew = world_snapshot_skew_policy.assess_world_snapshot_skew(
        request.descriptor,
        request.timings,
        request.query_time,
        request.skew_policy,
    )
    if skew.withhold:
      return _fail(TrajectoryValidityError.STALE_SNAPSHOT, -1, SNAPSHOT_RULE)
  return None


def _is_critical(rule: safety_rule_result.SafetyRuleResult) -> bool:
  return rule.severity == safety_rule_result.SEVERITY_CRITICAL


def _sample_failure(
    request: TrajectoryValidityRequest, index: int
) -> Optional[TrajectoryValidityResult]:
  sample = request.samples[index]
  if (
      vehicle_state_space.validate(sample.state, request.bounds)
      != vehicle_state_space.StateSpaceError.OK
  ):
    return _fail(TrajectoryValidityError.BOUNDS, index, BOUNDS_RULE)

  fence = request.fence
  x, y, z = sample.state.position
  pose = geofence_rule.GeofencePose(
      frame_id=request.pose_frame or fence.frame_id, x=x, y=y, z=z
  )
  geofence = geofence_rule.evaluate_geofence_aabb_rule(fence, pose)
  if geofence.violated:
    error = TrajectoryValidityError.GEOFENCE
    if _is_critical(geofence):
      error = (
          TrajectoryValidityError.FRAME_ERROR
          if _FRAME_MISMATCH_TEXT in geofence.summary
          else TrajectoryValidityError.BAD_REQUEST
      )
    return _fail(error, index, GEOFENCE_AABB_RULE)

  clearance = clearance_rule.evaluate_clearance_rule(
      request.clearance_samples[index], request.min_clearance_m
  )
  if clearance.violated:
    error = TrajectoryValidityError.CLEARANCE
    if _is_critical(clearance) and _SNAPSHOT_UNUSABLE_TEXT in clearance.summary:
      error = TrajectoryValidityError.STALE_SNAPSHOT
    return _fail(error, index, CLEARANCE_MIN_RULE)
  return None


def check_trajectory_validity(
    request: TrajectoryValidityRequest,
) -> TrajectoryValidityResult:
  """Checks a propagated trajectory against one immutable snapshot.

  Check order, first defect wins. Pre-sample (first_invalid_sample is -1):
    1. samples empty: BAD_REQUEST, empty failed_rule.
    2. clearance_samples length differs from samples: BAD_REQUEST, empty
       failed_rule.
    3. snapshot_id empty, descriptor not present, or snapshot_id differs from
       descriptor.snapshot_id: BAD_REQUEST, "snapshot".
    4. assess_world_snapshot(descriptor) not accepted: BAD_REQUEST,
       "snapshot".
    5. assess_skew and the assessment withholds: STALE_SNAPSHOT, "snapshot".
  Then per sample in order, stopping at the first failure:
    6. validate(state, bounds) is not OK: BOUNDS, "bounds".
    7. evaluate_geofence_aabb_rule: CRITICAL frame mismatch is FRAME_ERROR,
       any other CRITICAL is BAD_REQUEST, and an outside pose is GEOFENCE,
       all with "geofence.aabb".
    8. evaluate_clearance_rule: CRITICAL snapshot unusable is STALE_SNAPSHOT,
       any other violation is CLEARANCE, all with "clearance.min".
    9. All samples pass: OK.

  Args:
    request: The trajectory, snapshot binding, and injected clearance samples.

  Returns:
    The first defect, or OK.
  """
  failure = _pre_sample_failure(request)
  if failure is not None:
    return failure
  for index in range(len(request.samples)):
    failure = _sample_failure(request, index)
    if failure is not None:
      return failure
  return TrajectoryValidityResult(TrajectoryValidityError.OK, -1, "")
