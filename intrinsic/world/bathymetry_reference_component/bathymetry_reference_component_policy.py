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

"""Plain-value checks for BathymetryReferenceComponent.

Policy: intrinsic/world/bathymetry_reference_component/README.md.

These helpers do not parse protobuf, do not mutate World entities, and do
not convert world_enu and world_ned. Embedded validity is assessed by
assess_marine_component_validity.
"""

from dataclasses import dataclass
import enum
import math

from intrinsic.embodiment import frame_policy
from intrinsic.embodiment import stamped_header_policy
from intrinsic.world.marine_component_validity import marine_component_validity_policy

_VALIDITY = marine_component_validity_policy


class BathymetryReferenceError(enum.Enum):
  NONE = 0
  VALIDITY = 1
  FRAME_ID = 2
  ASSET_REFERENCE = 3
  VERTICAL_BIAS = 4


@dataclass(frozen=True)
class BathymetryReferenceView:
  """present is false for an empty message."""

  present: bool = False
  validity: _VALIDITY.MarineComponentValidityView = (
      _VALIDITY.MarineComponentValidityView()
  )
  frame_id: str = ""
  asset_reference: str = ""
  vertical_bias_present: bool = False
  vertical_bias_m: float = 0.0


@dataclass(frozen=True)
class BathymetryReferenceAssessment:
  error: BathymetryReferenceError
  validity: _VALIDITY.MarineComponentAssessment
  accepted: bool


def allowed_bathymetry_frame(frame_id: str) -> bool:
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


def assess_bathymetry_reference(
    component: BathymetryReferenceView, query: _VALIDITY.TimeParts
) -> BathymetryReferenceAssessment:
  """First structural defect wins.

  Check order: embedded validity (missing, or a MarineComponentValidity
  structural error), frame id, asset reference, then a present non-finite
  bias. Unknown and expired stay on the nested assessment. An empty view is
  not a component.
  """
  if not component.present:
    return BathymetryReferenceAssessment(
        BathymetryReferenceError.NONE, _empty_validity(), False
    )
  validity = _VALIDITY.assess_marine_component_validity(
      component.validity, query
  )
  error = BathymetryReferenceError.NONE
  if (
      not component.validity.present
      or validity.error is not _VALIDITY.ComponentValidityError.NONE
  ):
    error = BathymetryReferenceError.VALIDITY
  elif not allowed_bathymetry_frame(component.frame_id):
    error = BathymetryReferenceError.FRAME_ID
  elif component.asset_reference == "":
    error = BathymetryReferenceError.ASSET_REFERENCE
  elif component.vertical_bias_present and not math.isfinite(
      component.vertical_bias_m
  ):
    error = BathymetryReferenceError.VERTICAL_BIAS
  accepted = error is BathymetryReferenceError.NONE and validity.accepted
  return BathymetryReferenceAssessment(error, validity, accepted)
