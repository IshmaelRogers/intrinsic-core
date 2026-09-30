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

"""Skew, age, and required/optional assessment of one latched snapshot.

Policy: intrinsic/world/world_snapshot/README.md.

This is a pure function of the descriptor view, the latched per-kind timings,
the query time, and the policy. It does not read World entities, does not
mutate anything, and does not change the descriptor or its digest. Freshness
reuses the marine component validity boundary: a query at
observation_time + validity_horizon is still fresh.
"""

from collections.abc import Iterable
from dataclasses import dataclass
import enum

from intrinsic.world.marine_component_validity import marine_component_validity_policy as validity_policy
from intrinsic.world.world_snapshot import world_snapshot_policy

_VALIDITY = validity_policy
_SNAPSHOT = world_snapshot_policy

TimeParts = _VALIDITY.TimeParts


class SnapshotPolicyStatus(enum.Enum):
  UNSPECIFIED = 0
  FRESH = 1
  PARTIAL = 2
  STALE = 3
  EXCESSIVE_SKEW = 4
  INCOMPLETE = 5


class SnapshotPolicyError(enum.Enum):
  """Defects that stop the policy from running. None of these is a status."""

  NONE = 0
  DESCRIPTOR_NOT_PRESENT = 1
  DESCRIPTOR = 2
  INVALID_POLICY = 3
  INVALID_TIMINGS = 4
  INVALID_QUERY_TIME = 5


DEFAULT_MAX_SKEW: TimeParts = (0, 200000000)
DEFAULT_MAX_AGE: TimeParts = (2, 0)


@dataclass(frozen=True)
class WorldSnapshotSkewPolicy:
  """Opt-in skew, age, and presence policy.

  max_skew and max_age are non-negative with nanos in [0, 1e9). A value equal
  to the bound is within it. max_age bounds query_time - observation_time for
  every kind in the skew set.
  """

  max_skew: TimeParts = DEFAULT_MAX_SKEW
  max_age: TimeParts = DEFAULT_MAX_AGE
  # Must appear in the descriptor, else INCOMPLETE.
  required_kinds: tuple[str, ...] = ()
  # May be absent from the descriptor, which gives PARTIAL.
  optional_kinds: tuple[str, ...] = ()


@dataclass(frozen=True)
class ComponentTiming:
  """Latched timing of one component kind."""

  component_kind: str = ""
  observation_time: TimeParts = (0, 0)
  validity_horizon: TimeParts = (0, 0)
  # Diagnostics only. May be empty.
  source_id: str = ""


@dataclass(frozen=True)
class WorldSnapshotPolicyAssessment:
  """Result of the policy. SnapshotSkewAssessment is an alias.

  offending_kinds is sorted and unique. INCOMPLETE: missing required kinds.
  STALE: every stale kind. EXCESSIVE_SKEW: earliest and latest kinds. Empty
  for FRESH and PARTIAL. withhold is true whenever the snapshot is not
  accepted: do not publish it to safety or planning consumers.
  """

  status: SnapshotPolicyStatus = SnapshotPolicyStatus.UNSPECIFIED
  error: SnapshotPolicyError = SnapshotPolicyError.NONE
  descriptor_error: _SNAPSHOT.SnapshotError = _SNAPSHOT.SnapshotError.NONE
  accepted: bool = False
  withhold: bool = False
  offending_kinds: tuple[str, ...] = ()
  missing_optional_kinds: tuple[str, ...] = ()
  measured_skew_present: bool = False
  measured_skew: TimeParts = (0, 0)
  earliest_kind: str = ""
  latest_kind: str = ""


SnapshotSkewAssessment = WorldSnapshotPolicyAssessment


def _difference(later: TimeParts, earlier: TimeParts) -> TimeParts:
  seconds = later[0] - earlier[0]
  nanos = later[1] - earlier[1]
  if nanos < 0:
    nanos += 1000000000
    seconds -= 1
  return (seconds, nanos)


def _policy_valid(policy: WorldSnapshotSkewPolicy) -> bool:
  if not _VALIDITY.duration_non_negative(policy.max_skew):
    return False
  if not _VALIDITY.duration_non_negative(policy.max_age):
    return False
  required = set()
  for kind in policy.required_kinds:
    if not kind:
      return False
    required.add(kind)
  for kind in policy.optional_kinds:
    if not kind or kind in required:
      return False
  return True


def _timings_by_kind(
    timings: Iterable[ComponentTiming],
) -> dict[str, ComponentTiming] | None:
  by_kind = {}
  for timing in timings:
    if not timing.component_kind or timing.component_kind in by_kind:
      return None
    by_kind[timing.component_kind] = timing
  return by_kind


def _rejected(
    error: SnapshotPolicyError,
    descriptor_error: _SNAPSHOT.SnapshotError = _SNAPSHOT.SnapshotError.NONE,
) -> WorldSnapshotPolicyAssessment:
  return WorldSnapshotPolicyAssessment(
      error=error, descriptor_error=descriptor_error, withhold=True
  )


def assess_world_snapshot_skew(
    descriptor: _SNAPSHOT.WorldSnapshotView,
    timings: Iterable[ComponentTiming],
    query_time: TimeParts,
    policy: WorldSnapshotSkewPolicy = WorldSnapshotSkewPolicy(),
) -> WorldSnapshotPolicyAssessment:
  """Assesses one latched snapshot.

  Order of decisions, first decisive wins:
    0. A structural defect (absent or malformed descriptor, invalid policy,
       duplicate or empty timing kind, query_time nanos out of range) sets
       `error` and leaves `status` UNSPECIFIED. The policy does not run.
    1. INCOMPLETE: a required kind is absent from the descriptor.
    2. STALE: a kind in the skew set has no timing, or is expired by its
       validity_horizon, or is older than max_age.
    3. EXCESSIVE_SKEW: measured skew is strictly greater than max_skew.
    4. PARTIAL: an optional kind is absent from the descriptor.
    5. FRESH.

  Skew set: kinds present in the descriptor, listed in required_kinds or
  optional_kinds, with a supplied timing whose observation_time is usable.
  Kinds that are neither required nor optional are ignored. Timings for kinds
  absent from the descriptor are ignored.

  Args:
    descriptor: Plain view of a latched descriptor.
    timings: Latched per-kind timings.
    query_time: Assessment clock as (seconds, nanos).
    policy: Skew, age, and presence policy.

  Returns:
    The assessment.
  """
  if not descriptor.present:
    return _rejected(SnapshotPolicyError.DESCRIPTOR_NOT_PRESENT)
  structural = _SNAPSHOT.assess_world_snapshot(descriptor)
  if not structural.accepted:
    return _rejected(SnapshotPolicyError.DESCRIPTOR, structural.error)
  if not _policy_valid(policy):
    return _rejected(SnapshotPolicyError.INVALID_POLICY)
  by_kind = _timings_by_kind(timings)
  if by_kind is None:
    return _rejected(SnapshotPolicyError.INVALID_TIMINGS)
  if not _VALIDITY.timestamp_nanos_in_range(query_time[1]):
    return _rejected(SnapshotPolicyError.INVALID_QUERY_TIME)

  present_kinds = {c.component_kind for c in descriptor.components}
  required = set(policy.required_kinds)
  optional = set(policy.optional_kinds)
  missing_required = tuple(sorted(required - present_kinds))
  missing_optional = tuple(sorted(optional - present_kinds))

  stale = set()
  usable = []
  for kind in sorted((required | optional) & present_kinds):
    timing = by_kind.get(kind)
    if timing is None:
      stale.add(kind)
      continue
    # The age ceiling uses the same boundary as a validity horizon.
    if not all(
        _VALIDITY.assess_freshness(
            True,
            True,
            timing.observation_time,
            True,
            bound,
            query_time,
        )
        is _VALIDITY.ComponentFreshness.FRESH
        for bound in (timing.validity_horizon, policy.max_age)
    ):
      stale.add(kind)
    if _VALIDITY.timestamp_nanos_in_range(timing.observation_time[1]):
      usable.append((timing.observation_time, kind))

  measured_skew_present = False
  measured_skew = (0, 0)
  earliest_kind = ""
  latest_kind = ""
  skew_exceeded = False
  if len(usable) >= 2:
    # Ties go to the smallest kind.
    earliest_time, earliest_kind = min(usable)
    latest_time = max(time for time, _ in usable)
    latest_kind = min(kind for time, kind in usable if time == latest_time)
    measured_skew_present = True
    measured_skew = _difference(latest_time, earliest_time)
    skew_exceeded = measured_skew > policy.max_skew

  offending: tuple[str, ...] = ()
  if missing_required:
    status = SnapshotPolicyStatus.INCOMPLETE
    offending = missing_required
  elif stale:
    status = SnapshotPolicyStatus.STALE
    offending = tuple(sorted(stale))
  elif skew_exceeded:
    status = SnapshotPolicyStatus.EXCESSIVE_SKEW
    offending = tuple(sorted({earliest_kind, latest_kind}))
  elif missing_optional:
    status = SnapshotPolicyStatus.PARTIAL
  else:
    status = SnapshotPolicyStatus.FRESH
  accepted = status in (
      SnapshotPolicyStatus.FRESH,
      SnapshotPolicyStatus.PARTIAL,
  )
  return WorldSnapshotPolicyAssessment(
      status=status,
      accepted=accepted,
      withhold=not accepted,
      offending_kinds=offending,
      missing_optional_kinds=missing_optional,
      measured_skew_present=measured_skew_present,
      measured_skew=measured_skew,
      earliest_kind=earliest_kind,
      latest_kind=latest_kind,
  )
