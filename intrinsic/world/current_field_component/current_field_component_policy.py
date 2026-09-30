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

Policy: intrinsic_apis/intrinsic/world/proto/README.md.

These helpers do not parse protobuf and do not convert frames.
validity_meta is assessed by assess_marine_component_validity. This module
does not encode expiry. velocity is linear, SI meters per second, ENU.
"""

from dataclasses import dataclass
from dataclasses import field
import enum
import math

from intrinsic.world.marine_component_validity import marine_component_validity_policy

_MARINE = marine_component_validity_policy


class CurrentFieldError(enum.Enum):
  NONE = 0
  VALIDITY = 1
  FRAME_ID = 2
  REPRESENTATION = 3
  VELOCITY = 4


@dataclass(frozen=True)
class CurrentFieldView:
  """present is false for an empty message."""

  present: bool = False
  validity_meta: _MARINE.MarineComponentValidityView = field(
      default_factory=_MARINE.MarineComponentValidityView
  )
  frame_id: str = ""
  constant_present: bool = False
  velocity_x_m_s: float = 0.0
  velocity_y_m_s: float = 0.0
  velocity_z_m_s: float = 0.0


@dataclass(frozen=True)
class CurrentFieldAssessment:
  error: CurrentFieldError
  validity: _MARINE.MarineComponentAssessment
  accepted: bool


def linear_velocity_finite(x_m_s, y_m_s, z_m_s):
  return math.isfinite(x_m_s) and math.isfinite(y_m_s) and math.isfinite(z_m_s)


def _empty_validity(query):
  return _MARINE.assess_marine_component_validity(
      _MARINE.MarineComponentValidityView(), query
  )


def assess_current_field(component, query):
  """First defect wins. Expiry comes from MarineComponentValidity.

  Check order: validity_meta, frame_id, constant arm, velocity components.
  An empty view is not a component. A missing ConstantCurrent arm is a
  defect. Non-finite velocity is a defect only when that arm is present.
  """
  if not component.present:
    return CurrentFieldAssessment(
        CurrentFieldError.NONE, _empty_validity(query), False
    )
  validity = _MARINE.assess_marine_component_validity(
      component.validity_meta, query
  )
  error = CurrentFieldError.NONE
  if (
      not component.validity_meta.present
      or validity.error is not _MARINE.ComponentValidityError.NONE
  ):
    error = CurrentFieldError.VALIDITY
  elif component.frame_id == "":
    error = CurrentFieldError.FRAME_ID
  elif not component.constant_present:
    error = CurrentFieldError.REPRESENTATION
  elif not linear_velocity_finite(
      component.velocity_x_m_s,
      component.velocity_y_m_s,
      component.velocity_z_m_s,
  ):
    error = CurrentFieldError.VELOCITY
  accepted = error is CurrentFieldError.NONE and validity.accepted
  return CurrentFieldAssessment(error, validity, accepted)
