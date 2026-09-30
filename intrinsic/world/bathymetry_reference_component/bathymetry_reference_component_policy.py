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

Policy: intrinsic_apis/intrinsic/world/proto/README.md.

These helpers do not parse protobuf and do not convert ENU to NED.
validity_meta is assessed by assess_marine_component_validity. This module
does not encode expiry.
"""

from dataclasses import dataclass
from dataclasses import field
import enum
import math

from intrinsic.world.marine_component_validity import marine_component_validity_policy

_MARINE = marine_component_validity_policy


class BathymetryReferenceError(enum.Enum):
  NONE = 0
  VALIDITY = 1
  FRAME_ID = 2
  ASSET_REF = 3
  REFERENCE_Z = 4


@dataclass(frozen=True)
class BathymetryReferenceView:
  """present is false for an empty message."""

  present: bool = False
  validity_meta: _MARINE.MarineComponentValidityView = field(
      default_factory=_MARINE.MarineComponentValidityView
  )
  frame_id: str = ""
  bathymetry_asset_ref: str = ""
  reference_z_present: bool = False
  reference_z_m: float = 0.0


@dataclass(frozen=True)
class BathymetryReferenceAssessment:
  error: BathymetryReferenceError
  validity: _MARINE.MarineComponentAssessment
  accepted: bool


def _empty_validity(query):
  return _MARINE.assess_marine_component_validity(
      _MARINE.MarineComponentValidityView(), query
  )


def assess_bathymetry_reference(component, query):
  """First defect wins. Expiry comes from MarineComponentValidity.

  Check order: validity_meta, frame_id, bathymetry_asset_ref, reference_z_m.
  An empty view is not a component. A missing validity_meta is a defect.
  Non-finite reference_z_m is a defect only when that field is present.
  """
  if not component.present:
    return BathymetryReferenceAssessment(
        BathymetryReferenceError.NONE, _empty_validity(query), False
    )
  validity = _MARINE.assess_marine_component_validity(
      component.validity_meta, query
  )
  error = BathymetryReferenceError.NONE
  if (
      not component.validity_meta.present
      or validity.error is not _MARINE.ComponentValidityError.NONE
  ):
    error = BathymetryReferenceError.VALIDITY
  elif component.frame_id == "":
    error = BathymetryReferenceError.FRAME_ID
  elif component.bathymetry_asset_ref == "":
    error = BathymetryReferenceError.ASSET_REF
  elif component.reference_z_present and not math.isfinite(
      component.reference_z_m
  ):
    error = BathymetryReferenceError.REFERENCE_Z
  accepted = error is BathymetryReferenceError.NONE and validity.accepted
  return BathymetryReferenceAssessment(error, validity, accepted)
