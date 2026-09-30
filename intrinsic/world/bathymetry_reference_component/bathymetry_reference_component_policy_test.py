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

"""Validation tests for referenced bathymetry."""

from dataclasses import replace
import math
import unittest

from intrinsic.embodiment import stamped_header_policy
from intrinsic.world.bathymetry_reference_component import bathymetry_reference_component_policy
from intrinsic.world.marine_component_validity import marine_component_validity_policy

_POLICY = bathymetry_reference_component_policy
_MARINE = marine_component_validity_policy


def _meta(**overrides):
  view = _MARINE.MarineComponentValidityView(
      present=True,
      source_id="fusion_0",
      observation_time_present=True,
      observation_time=(1000, 500000000),
      validity_horizon_present=True,
      validity_horizon=(1, 600000000),
      confidence_present=True,
      confidence=0.75,
      uncertainty_reference="cov-ref-7",
      validity_present=True,
      validity_state=1,
  )
  return replace(view, **overrides) if overrides else view


def _valid(**overrides):
  view = _POLICY.BathymetryReferenceView(
      present=True,
      validity_meta=_meta(),
      frame_id="world_enu",
      bathymetry_asset_ref="cas:bathy-grid-1",
      reference_z_present=True,
      reference_z_m=-12.5,
  )
  return replace(view, **overrides) if overrides else view


def _deadline():
  return (1002, 100000000)


class BathymetryReferencePolicyTest(unittest.TestCase):

  def test_empty_message_is_not_an_error(self):
    assessment = _POLICY.assess_bathymetry_reference(
        _POLICY.BathymetryReferenceView(), _deadline()
    )
    self.assertIs(assessment.error, _POLICY.BathymetryReferenceError.NONE)
    self.assertIs(
        assessment.validity.freshness, _MARINE.ComponentFreshness.UNKNOWN
    )
    self.assertFalse(assessment.accepted)

  def test_missing_validity_meta(self):
    assessment = _POLICY.assess_bathymetry_reference(
        _valid(validity_meta=_MARINE.MarineComponentValidityView()),
        _deadline(),
    )
    self.assertIs(assessment.error, _POLICY.BathymetryReferenceError.VALIDITY)
    self.assertFalse(assessment.accepted)

  def test_validity_defect_is_delegated(self):
    view = _valid(validity_meta=_meta(source_id=""))
    assessment = _POLICY.assess_bathymetry_reference(view, _deadline())
    direct = _MARINE.assess_marine_component_validity(
        view.validity_meta, _deadline()
    )
    self.assertIs(assessment.error, _POLICY.BathymetryReferenceError.VALIDITY)
    self.assertIs(
        assessment.validity.error, _MARINE.ComponentValidityError.SOURCE_ID
    )
    self.assertIs(assessment.validity.freshness, direct.freshness)
    self.assertFalse(assessment.accepted)

  def test_first_defect_wins(self):
    view = _valid(
        validity_meta=_meta(source_id=""),
        frame_id="",
        bathymetry_asset_ref="",
        reference_z_m=math.nan,
    )
    self.assertIs(
        _POLICY.assess_bathymetry_reference(view, _deadline()).error,
        _POLICY.BathymetryReferenceError.VALIDITY,
    )
    view = replace(view, validity_meta=_meta())
    self.assertIs(
        _POLICY.assess_bathymetry_reference(view, _deadline()).error,
        _POLICY.BathymetryReferenceError.FRAME_ID,
    )
    view = replace(view, frame_id="world_enu")
    self.assertIs(
        _POLICY.assess_bathymetry_reference(view, _deadline()).error,
        _POLICY.BathymetryReferenceError.ASSET_REF,
    )
    view = replace(view, bathymetry_asset_ref="cas:bathy-grid-1")
    self.assertIs(
        _POLICY.assess_bathymetry_reference(view, _deadline()).error,
        _POLICY.BathymetryReferenceError.REFERENCE_Z,
    )

  def test_rejects_empty_frame_and_asset(self):
    self.assertIs(
        _POLICY.assess_bathymetry_reference(
            _valid(frame_id=""), _deadline()
        ).error,
        _POLICY.BathymetryReferenceError.FRAME_ID,
    )
    self.assertIs(
        _POLICY.assess_bathymetry_reference(
            _valid(bathymetry_asset_ref=""), _deadline()
        ).error,
        _POLICY.BathymetryReferenceError.ASSET_REF,
    )

  def test_reference_z_finite_only_when_set(self):
    absent = _POLICY.assess_bathymetry_reference(
        _valid(reference_z_present=False, reference_z_m=math.nan), _deadline()
    )
    self.assertIs(absent.error, _POLICY.BathymetryReferenceError.NONE)
    self.assertTrue(absent.accepted)
    for z in (0.0, -0.0, -12.5, 3.0):
      self.assertTrue(
          _POLICY.assess_bathymetry_reference(
              _valid(reference_z_m=z), _deadline()
          ).accepted
      )
    for z in (math.nan, math.inf, -math.inf):
      self.assertIs(
          _POLICY.assess_bathymetry_reference(
              _valid(reference_z_m=z), _deadline()
          ).error,
          _POLICY.BathymetryReferenceError.REFERENCE_Z,
      )

  def test_asset_ref_is_opaque(self):
    self.assertTrue(
        _POLICY.assess_bathymetry_reference(
            _valid(bathymetry_asset_ref="1,nan,inf"), _deadline()
        ).accepted
    )

  def test_non_empty_frame_is_not_rewritten(self):
    view = _valid(frame_id="map_enu")
    self.assertTrue(
        _POLICY.assess_bathymetry_reference(view, _deadline()).accepted
    )
    self.assertEqual(view.frame_id, "map_enu")

  def test_exact_horizon_boundary_is_delegated(self):
    view = _valid()
    deadline = _deadline()
    fresh = _POLICY.assess_bathymetry_reference(view, deadline)
    self.assertIs(fresh.error, _POLICY.BathymetryReferenceError.NONE)
    self.assertIs(fresh.validity.freshness, _MARINE.ComponentFreshness.FRESH)
    self.assertTrue(fresh.accepted)
    after = (deadline[0], deadline[1] + 1)
    expired = _POLICY.assess_bathymetry_reference(view, after)
    direct = _MARINE.assess_marine_component_validity(view.validity_meta, after)
    self.assertIs(expired.error, _POLICY.BathymetryReferenceError.NONE)
    self.assertIs(
        expired.validity.freshness, _MARINE.ComponentFreshness.EXPIRED
    )
    self.assertIs(
        expired.validity.validity, stamped_header_policy.ValidityKind.VALID
    )
    self.assertFalse(expired.accepted)
    self.assertIs(expired.validity.freshness, direct.freshness)

  def test_unknown_is_distinct_from_expired(self):
    after = (1002, 100000001)
    unknown = _POLICY.assess_bathymetry_reference(
        _valid(validity_meta=_meta(validity_present=False)), after
    )
    self.assertIs(
        unknown.validity.freshness, _MARINE.ComponentFreshness.UNKNOWN
    )
    self.assertIs(
        unknown.validity.validity, stamped_header_policy.ValidityKind.ABSENT
    )
    self.assertIsNot(
        unknown.validity.freshness, _MARINE.ComponentFreshness.EXPIRED
    )
    invalid = _POLICY.assess_bathymetry_reference(
        _valid(validity_meta=_meta(validity_state=2)), _deadline()
    )
    self.assertIs(
        invalid.validity.validity, stamped_header_policy.ValidityKind.INVALID
    )
    self.assertIs(invalid.validity.freshness, _MARINE.ComponentFreshness.FRESH)
    self.assertFalse(invalid.accepted)
    unspecified = _POLICY.assess_bathymetry_reference(
        _valid(validity_meta=_meta(validity_state=99)), after
    )
    self.assertIs(
        unspecified.validity.validity,
        stamped_header_policy.ValidityKind.UNSPECIFIED,
    )
    self.assertIs(
        unspecified.validity.freshness, _MARINE.ComponentFreshness.EXPIRED
    )
    self.assertFalse(unspecified.accepted)

  def test_nonsense_horizon_stays_unknown(self):
    assessment = _POLICY.assess_bathymetry_reference(
        _valid(validity_meta=_meta(validity_horizon=(-1, 0))), _deadline()
    )
    self.assertIs(assessment.error, _POLICY.BathymetryReferenceError.VALIDITY)
    self.assertIs(
        assessment.validity.error, _MARINE.ComponentValidityError.HORIZON
    )
    self.assertIs(
        assessment.validity.freshness, _MARINE.ComponentFreshness.UNKNOWN
    )


if __name__ == "__main__":
  unittest.main()
