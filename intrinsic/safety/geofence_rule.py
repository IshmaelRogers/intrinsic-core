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

"""Pure axis-aligned bounding-box (AABB) hard geofence.

This does not read World, query a collision map, plan a trajectory, or project
a pose back into the fence. A pose outside the fence is rejected, never
clamped.
"""

import dataclasses
import math
from typing import Optional

from intrinsic.safety import safety_rule_result

GEOFENCE_AABB_RULE_ID = "geofence.aabb"

_BAD_CONFIG_SUMMARY = "geofence bad config"
_UNUSABLE_SUMMARY = "geofence input is not usable"
_FRAME_MISMATCH_SUMMARY = "geofence frame mismatch"

_RESULT = safety_rule_result


@dataclasses.dataclass(frozen=True)
class AabbGeofence:
  """Closed axis-aligned box in `frame_id`.

  `frame_id` and `region_id` must be non-empty. Every bound must be finite
  with `min_i <= max_i`.
  """

  frame_id: str
  region_id: str
  min_x: float
  min_y: float
  min_z: float
  max_x: float
  max_y: float
  max_z: float


@dataclasses.dataclass(frozen=True)
class GeofencePose:
  """Position in meters in `frame_id`, which must equal the fence frame."""

  frame_id: str
  x: float
  y: float
  z: float


def _critical(summary: str) -> _RESULT.SafetyRuleResult:
  return _RESULT.SafetyRuleResult(
      violated=True,
      rule_id=GEOFENCE_AABB_RULE_ID,
      severity=_RESULT.SEVERITY_CRITICAL,
      summary=summary,
      recommended_kind=_RESULT.DECISION_KIND_REJECT,
  )


def _outside(summary: str) -> _RESULT.SafetyRuleResult:
  return _RESULT.SafetyRuleResult(
      violated=True,
      rule_id=GEOFENCE_AABB_RULE_ID,
      severity=_RESULT.SEVERITY_ERROR,
      summary=summary,
      recommended_kind=_RESULT.DECISION_KIND_REJECT,
  )


def _fence_rejection(
    fence: AabbGeofence,
) -> Optional[_RESULT.SafetyRuleResult]:
  if not fence.frame_id or not fence.region_id:
    return _critical(_BAD_CONFIG_SUMMARY)
  bounds = (
      (fence.min_x, fence.max_x),
      (fence.min_y, fence.max_y),
      (fence.min_z, fence.max_z),
  )
  for low, high in bounds:
    if not math.isfinite(low) or not math.isfinite(high) or low > high:
      return _critical(_UNUSABLE_SUMMARY)
  return None


def _pose_rejection(
    fence: AabbGeofence, pose: GeofencePose
) -> Optional[_RESULT.SafetyRuleResult]:
  if not pose.frame_id:
    return _critical(_BAD_CONFIG_SUMMARY)
  if not all(math.isfinite(v) for v in (pose.x, pose.y, pose.z)):
    return _critical(_UNUSABLE_SUMMARY)
  if pose.frame_id != fence.frame_id:
    return _critical(_FRAME_MISMATCH_SUMMARY)
  return None


def _contains(fence: AabbGeofence, pose: GeofencePose) -> bool:
  return (
      fence.min_x <= pose.x <= fence.max_x
      and fence.min_y <= pose.y <= fence.max_y
      and fence.min_z <= pose.z <= fence.max_z
  )


def evaluate_geofence_aabb_rule(
    fence: AabbGeofence, pose: GeofencePose
) -> _RESULT.SafetyRuleResult:
  """Evaluate `geofence.aabb` for one pose.

  An empty fence frame_id, region_id, or pose frame_id, a non-finite bound or
  coordinate, a bound with `min_i > max_i`, or a pose frame that differs from
  the fence frame is CRITICAL recommending REJECT. A pose on or inside the
  closed box is compliant. A pose outside is ERROR recommending REJECT with a
  summary `geofence outside region=<region_id>`. Nothing is ever projected.
  """
  rejected = _fence_rejection(fence) or _pose_rejection(fence, pose)
  if rejected is not None:
    return rejected
  if _contains(fence, pose):
    return _RESULT.SafetyRuleResult()
  return _outside(f"geofence outside region={fence.region_id}")


def evaluate_geofence_aabb_segment_rule(
    fence: AabbGeofence, start: GeofencePose, end: GeofencePose
) -> _RESULT.SafetyRuleResult:
  """Evaluate `geofence.aabb` for a straight segment from start to end.

  The fence is validated once and both poses are validated as in
  `evaluate_geofence_aabb_rule`. The box is convex, so a segment lies in it
  exactly when both endpoints do. Only the endpoints are checked and no edge
  clipping is done. If either endpoint is outside the result is ERROR
  recommending REJECT with a summary
  `geofence segment outside region=<region_id> end=<start|end|both>`.
  """
  rejected = (
      _fence_rejection(fence)
      or _pose_rejection(fence, start)
      or _pose_rejection(fence, end)
  )
  if rejected is not None:
    return rejected
  start_outside = not _contains(fence, start)
  end_outside = not _contains(fence, end)
  if not start_outside and not end_outside:
    return _RESULT.SafetyRuleResult()
  if start_outside and end_outside:
    which = "both"
  else:
    which = "start" if start_outside else "end"
  return _outside(
      f"geofence segment outside region={fence.region_id} end={which}"
  )
