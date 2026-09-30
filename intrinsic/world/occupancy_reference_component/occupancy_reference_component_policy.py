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

"""Plain-value checks for OccupancyReferenceComponent.

Policy: intrinsic/world/occupancy_reference_component/README.md.

These helpers do not parse protobuf, do not mutate World entities, and do
not convert world_enu and world_ned. Embedded validity is assessed by
assess_marine_component_validity.
"""

from dataclasses import dataclass
import enum

from intrinsic.embodiment import frame_policy
from intrinsic.embodiment import stamped_header_policy
from intrinsic.world.marine_component_validity import marine_component_validity_policy

_VALIDITY = marine_component_validity_policy


class OccupancyReferenceError(enum.Enum):
  NONE = 0
  VALIDITY = 1
  FRAME_ID = 2
  ASSET_REFERENCE = 3


@dataclass(frozen=True)
class OccupancyReferenceView:
  """present is false for an empty message."""

  present: bool = False
  validity: _VALIDITY.MarineComponentValidityView = (
      _VALIDITY.MarineComponentValidityView()
  )
  frame_id: str = ""
  asset_reference: str = ""


@dataclass(frozen=True)
class OccupancyReferenceAssessment:
  error: OccupancyReferenceError
  validity: _VALIDITY.MarineComponentAssessment
  accepted: bool


def allowed_occupancy_frame(frame_id: str) -> bool:
  """True for exact world_enu or world_ned. Does not convert frames."""
  return frame_id in (
      frame_policy.WORLD_ENU_FRAME_ID,
      frame_policy.WORLD_NED_FRAME_ID,
  )


def _empty_validity() -> _VALIDITY.MarineComponentAssessment:
  return _VALIDITY.MarineComponentAssessment(
      _VALIDITY.ComponentValidityError.NONE,
      stamped_header_policy.ValidityKind.ABSENT,
      _VALIDITY.ComponentFreshness.UNKNOWN,
      False,
  )


def assess_occupancy_reference(
    component: OccupancyReferenceView, query: _VALIDITY.TimeParts
) -> OccupancyReferenceAssessment:
  """First structural defect wins.

  Check order: embedded validity (missing, or a MarineComponentValidity
  structural error), frame id, then asset reference. Unknown and expired
  stay on the nested assessment. An empty view is not a component.
  """
  if not component.present:
    return OccupancyReferenceAssessment(
        OccupancyReferenceError.NONE, _empty_validity(), False
    )
  validity = _VALIDITY.assess_marine_component_validity(
      component.validity, query
  )
  error = OccupancyReferenceError.NONE
  if (
      not component.validity.present
      or validity.error is not _VALIDITY.ComponentValidityError.NONE
  ):
    error = OccupancyReferenceError.VALIDITY
  elif not allowed_occupancy_frame(component.frame_id):
    error = OccupancyReferenceError.FRAME_ID
  elif component.asset_reference == "":
    error = OccupancyReferenceError.ASSET_REFERENCE
  accepted = error is OccupancyReferenceError.NONE and validity.accepted
  return OccupancyReferenceAssessment(error, validity, accepted)
