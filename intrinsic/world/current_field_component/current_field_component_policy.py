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

"""Plain-value checks for CurrentFieldComponent.

Policy: intrinsic/world/current_field_component/README.md.

These helpers do not parse protobuf, do not mutate World entities, do not
convert frames, and do not call vehicle dynamics. Embedded validity is
assessed by assess_marine_component_validity. Angular current is zero by
convention and is not a field.
"""

from dataclasses import dataclass
import enum

from intrinsic.embodiment import frame_policy
from intrinsic.embodiment import stamped_header_policy
from intrinsic.world.marine_component_validity import marine_component_validity_policy

_VALIDITY = marine_component_validity_policy

# Matches vehicle Environment.current_frame_id. This package does not convert
# that id into world_enu or world_ned.
BODY_FRAME_ID = "body"

VelocityMps = tuple[float, float, float]


class CurrentFieldError(enum.Enum):
  NONE = 0
  VALIDITY = 1
  FRAME_ID = 2
  VELOCITY = 3


@dataclass(frozen=True)
class CurrentFieldView:
  """present is false for an empty message."""

  present: bool = False
  validity: _VALIDITY.MarineComponentValidityView = (
      _VALIDITY.MarineComponentValidityView()
  )
  frame_id: str = ""
  velocity_present: bool = False
  velocity_m_s: VelocityMps = (0.0, 0.0, 0.0)


@dataclass(frozen=True)
class CurrentFieldAssessment:
  error: CurrentFieldError
  validity: _VALIDITY.MarineComponentAssessment
  accepted: bool


def allowed_current_frame(frame_id: str) -> bool:
  """True for exact world_enu, world_ned, or body. Does not convert."""
  return frame_id in (
      frame_policy.WORLD_ENU_FRAME_ID,
      frame_policy.WORLD_NED_FRAME_ID,
      BODY_FRAME_ID,
  )


def _empty_validity() -> _VALIDITY.MarineComponentAssessment:
  return _VALIDITY.MarineComponentAssessment(
      _VALIDITY.ComponentValidityError.NONE,
      stamped_header_policy.ValidityKind.ABSENT,
      _VALIDITY.ComponentFreshness.UNKNOWN,
      False,
  )


def assess_current_field(
    component: CurrentFieldView, query: _VALIDITY.TimeParts
) -> CurrentFieldAssessment:
  """First structural defect wins.

  Check order: embedded validity (missing, or a MarineComponentValidity
  structural error), frame id, then missing or non-finite velocity. Unknown
  and expired stay on the nested assessment. An empty view is not a
  component.
  """
  if not component.present:
    return CurrentFieldAssessment(
        CurrentFieldError.NONE, _empty_validity(), False
    )
  validity = _VALIDITY.assess_marine_component_validity(
      component.validity, query
  )
  error = CurrentFieldError.NONE
  if (
      not component.validity.present
      or validity.error is not _VALIDITY.ComponentValidityError.NONE
  ):
    error = CurrentFieldError.VALIDITY
  elif not allowed_current_frame(component.frame_id):
    error = CurrentFieldError.FRAME_ID
  elif not component.velocity_present or not frame_policy.is_finite_vec3(
      component.velocity_m_s
  ):
    error = CurrentFieldError.VELOCITY
  accepted = error is CurrentFieldError.NONE and validity.accepted
  return CurrentFieldAssessment(error, validity, accepted)
