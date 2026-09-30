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

"""Plain-value checks for MarineComponentValidity.

Policy: intrinsic_apis/intrinsic/world/proto/README.md.

These helpers do not parse protobuf, do not mutate World entities, and do
not allocate a snapshot id. Expired is a host assessment. It is not a
Validity state.
"""

from dataclasses import dataclass
import enum
import math

from intrinsic.embodiment import stamped_header_policy

_INT64_MAX = 9223372036854775807

# seconds, nanos. Nanos are in [0, 1000000000) when the time is usable.
TimeParts = tuple[int, int]


class ComponentValidityError(enum.Enum):
  NONE = 0
  SOURCE_ID = 1
  OBSERVATION_TIME = 2
  HORIZON = 3
  CONFIDENCE = 4


class ComponentFreshness(enum.Enum):
  """Unknown, fresh, or expired.

  Unknown: Validity is unset, or the times cannot form a deadline.
  Expired: query_time is strictly after observation_time + horizon.
  Equality with that deadline is fresh.
  """

  UNKNOWN = 0
  FRESH = 1
  EXPIRED = 2


@dataclass(frozen=True)
class MarineComponentValidityView:
  """present is false for an empty message."""

  present: bool = False
  source_id: str = ""
  observation_time_present: bool = False
  observation_time: TimeParts = (0, 0)
  validity_horizon_present: bool = False
  validity_horizon: TimeParts = (0, 0)
  confidence_present: bool = False
  confidence: float = 0.0
  uncertainty_reference: str = ""
  validity_present: bool = False
  validity_state: int = 0


@dataclass(frozen=True)
class MarineComponentAssessment:
  error: ComponentValidityError
  validity: stamped_header_policy.ValidityKind
  freshness: ComponentFreshness
  accepted: bool


def timestamp_nanos_in_range(nanos: int) -> bool:
  return stamped_header_policy.nanos_in_range(nanos)


def duration_non_negative(duration: TimeParts) -> bool:
  return duration[0] >= 0 and timestamp_nanos_in_range(duration[1])


def confidence_in_range(confidence: float) -> bool:
  return math.isfinite(confidence) and 0.0 <= confidence <= 1.0


def _add_non_negative_duration(
    start: TimeParts, duration: TimeParts
) -> TimeParts | None:
  """Deadline, or None when the inputs cannot form one."""
  if not timestamp_nanos_in_range(start[1]) or not duration_non_negative(
      duration
  ):
    return None
  nanos = start[1] + duration[1]
  carry = 0
  if nanos >= stamped_header_policy.NANOS_PER_SECOND:
    nanos -= stamped_header_policy.NANOS_PER_SECOND
    carry = 1
  if carry == 1 and duration[0] == _INT64_MAX:
    return None
  duration_with_carry = duration[0] + carry
  if start[0] > 0 and duration_with_carry > _INT64_MAX - start[0]:
    return None
  return (start[0] + duration_with_carry, nanos)


def _time_strictly_after(query: TimeParts, deadline: TimeParts) -> bool:
  if query[0] != deadline[0]:
    return query[0] > deadline[0]
  return query[1] > deadline[1]


def assess_freshness(
    validity_present: bool,
    observation_present: bool,
    observation: TimeParts,
    horizon_present: bool,
    horizon: TimeParts,
    query: TimeParts,
) -> ComponentFreshness:
  """Host freshness. Does not write a Validity state."""
  if (
      not validity_present
      or not observation_present
      or not horizon_present
      or not timestamp_nanos_in_range(query[1])
  ):
    return ComponentFreshness.UNKNOWN
  deadline = _add_non_negative_duration(observation, horizon)
  if deadline is None:
    return ComponentFreshness.UNKNOWN
  if _time_strictly_after(query, deadline):
    return ComponentFreshness.EXPIRED
  return ComponentFreshness.FRESH


def _assessment(
    error: ComponentValidityError,
    validity: stamped_header_policy.ValidityKind,
    freshness: ComponentFreshness,
) -> MarineComponentAssessment:
  accepted = (
      error is ComponentValidityError.NONE
      and stamped_header_policy.sample_accepted(validity, True)
      and freshness is ComponentFreshness.FRESH
  )
  return MarineComponentAssessment(error, validity, freshness, accepted)


def assess_marine_component_validity(
    component: MarineComponentValidityView, query: TimeParts
) -> MarineComponentAssessment:
  """First defect wins. Absent optional fields are not defects.

  Check order: source id, observation nanos, horizon, confidence. An empty
  view is not a component. Freshness is reported even when a defect is
  present.
  """
  if not component.present:
    return MarineComponentAssessment(
        ComponentValidityError.NONE,
        stamped_header_policy.ValidityKind.ABSENT,
        ComponentFreshness.UNKNOWN,
        False,
    )
  validity = stamped_header_policy.classify_validity(
      component.validity_present, component.validity_state
  )
  error = ComponentValidityError.NONE
  if component.source_id == "":
    error = ComponentValidityError.SOURCE_ID
  elif component.observation_time_present and not timestamp_nanos_in_range(
      component.observation_time[1]
  ):
    error = ComponentValidityError.OBSERVATION_TIME
  elif component.validity_horizon_present and not duration_non_negative(
      component.validity_horizon
  ):
    error = ComponentValidityError.HORIZON
  elif component.confidence_present and not confidence_in_range(
      component.confidence
  ):
    error = ComponentValidityError.CONFIDENCE
  freshness = assess_freshness(
      component.validity_present,
      component.observation_time_present,
      component.observation_time,
      component.validity_horizon_present,
      component.validity_horizon,
      query,
  )
  return _assessment(error, validity, freshness)
