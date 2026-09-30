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

"""Validation tests for BathymetryReferenceComponent."""

import math
import unittest

from intrinsic.embodiment import frame_policy
from intrinsic.embodiment import stamped_header_policy
from intrinsic.world.bathymetry_reference_component import bathymetry_reference_component_policy
from intrinsic.world.marine_component_validity import marine_component_validity_policy

_POLICY = bathymetry_reference_component_policy
_VALIDITY = marine_component_validity_policy


def _valid_validity():
  return _VALIDITY.MarineComponentValidityView(
      present=True,
      source_id="fusion_0",
      observation_time_present=True,
      observation_time=(1700000000, 250000000),
      validity_horizon_present=True,
      validity_horizon=(10, 500000000),
      confidence_present=True,
      confidence=0.75,
      uncertainty_reference="cov-ref-7",
      validity_present=True,
      validity_state=1,
  )


def _valid_bathymetry():
  return _POLICY.BathymetryReferenceView(
      present=True,
      validity=_valid_validity(),
      frame_id=frame_policy.WORLD_ENU_FRAME_ID,
      asset_reference="bathy-ref-7",
      vertical_bias_present=True,
      vertical_bias_m=1.5,
  )


def _deadline():
  # 1700000000.250s + 10.500s = 1700000010.750s.
  return (1700000010, 750000000)


def _replace(view, **kwargs):
  data = dict(view.__dict__)
  data.update(kwargs)
  return type(view)(**data)


class BathymetryReferencePolicyTest(unittest.TestCase):

  def test_empty_message_is_not_an_error(self):
    assessment = _POLICY.assess_bathymetry_reference(
        _POLICY.BathymetryReferenceView(), _deadline()
    )
    self.assertIs(assessment.error, _POLICY.BathymetryReferenceError.NONE)
    self.assertIs(
        assessment.validity.validity, stamped_header_policy.ValidityKind.ABSENT
    )
    self.assertIs(
        assessment.validity.freshness, _VALIDITY.ComponentFreshness.UNKNOWN
    )
    self.assertFalse(assessment.accepted)

  def test_missing_validity_is_first_defect(self):
    view = _replace(
        _valid_bathymetry(),
        validity=_replace(_valid_validity(), present=False),
        frame_id="robot",
        asset_reference="",
        vertical_bias_m=math.nan,
    )
    assessment = _POLICY.assess_bathymetry_reference(view, _deadline())
    self.assertIs(assessment.error, _POLICY.BathymetryReferenceError.VALIDITY)
    self.assertFalse(assessment.accepted)

  def test_first_defect_wins(self):
    view = _replace(
        _valid_bathymetry(),
        validity=_replace(_valid_validity(), source_id=""),
        frame_id="robot",
        asset_reference="",
        vertical_bias_m=math.nan,
    )
    assessment = _POLICY.assess_bathymetry_reference(view, _deadline())
    self.assertIs(assessment.error, _POLICY.BathymetryReferenceError.VALIDITY)
    self.assertIs(
        assessment.validity.error, _VALIDITY.ComponentValidityError.SOURCE_ID
    )

    view = _replace(view, validity=_valid_validity())
    assessment = _POLICY.assess_bathymetry_reference(view, _deadline())
    self.assertIs(assessment.error, _POLICY.BathymetryReferenceError.FRAME_ID)

    view = _replace(view, frame_id=frame_policy.WORLD_NED_FRAME_ID)
    assessment = _POLICY.assess_bathymetry_reference(view, _deadline())
    self.assertIs(
        assessment.error, _POLICY.BathymetryReferenceError.ASSET_REFERENCE
    )

    view = _replace(view, asset_reference="bathy-ref-7")
    assessment = _POLICY.assess_bathymetry_reference(view, _deadline())
    self.assertIs(
        assessment.error, _POLICY.BathymetryReferenceError.VERTICAL_BIAS
    )

    view = _replace(view, vertical_bias_present=False)
    assessment = _POLICY.assess_bathymetry_reference(view, _deadline())
    self.assertIs(assessment.error, _POLICY.BathymetryReferenceError.NONE)
    self.assertTrue(assessment.accepted)

  def test_frame_allow_list(self):
    view = _valid_bathymetry()
    for frame_id in (
        frame_policy.WORLD_ENU_FRAME_ID,
        frame_policy.WORLD_NED_FRAME_ID,
    ):
      accepted = _POLICY.assess_bathymetry_reference(
          _replace(view, frame_id=frame_id), _deadline()
      )
      self.assertTrue(accepted.accepted, frame_id)
    for frame_id in (
        "",
        "enu",
        "ned",
        "world_ENU",
        "body",
        "robot",
        "world_enu ",
    ):
      assessment = _POLICY.assess_bathymetry_reference(
          _replace(view, frame_id=frame_id), _deadline()
      )
      self.assertIs(
          assessment.error, _POLICY.BathymetryReferenceError.FRAME_ID, frame_id
      )

  def test_asset_reference_is_opaque_and_required(self):
    view = _valid_bathymetry()
    empty = _POLICY.assess_bathymetry_reference(
        _replace(view, asset_reference=""), _deadline()
    )
    self.assertIs(empty.error, _POLICY.BathymetryReferenceError.ASSET_REFERENCE)
    for asset_reference in (" ", "sha256:abc,nan"):
      assessment = _POLICY.assess_bathymetry_reference(
          _replace(view, asset_reference=asset_reference), _deadline()
      )
      self.assertTrue(assessment.accepted, asset_reference)

  def test_vertical_bias_unset_is_not_zero(self):
    view = _valid_bathymetry()
    unset = _POLICY.assess_bathymetry_reference(
        _replace(view, vertical_bias_present=False, vertical_bias_m=math.nan),
        _deadline(),
    )
    self.assertTrue(unset.accepted)
    for bias in (0.0, -0.0, -2.5):
      assessment = _POLICY.assess_bathymetry_reference(
          _replace(view, vertical_bias_present=True, vertical_bias_m=bias),
          _deadline(),
      )
      self.assertTrue(assessment.accepted, bias)
    for bias in (math.nan, math.inf, -math.inf):
      assessment = _POLICY.assess_bathymetry_reference(
          _replace(view, vertical_bias_present=True, vertical_bias_m=bias),
          _deadline(),
      )
      self.assertIs(
          assessment.error, _POLICY.BathymetryReferenceError.VERTICAL_BIAS
      )

  def test_enu_and_ned_store_the_same_bias(self):
    view = _replace(_valid_bathymetry(), vertical_bias_m=1.5)
    enu = _POLICY.assess_bathymetry_reference(
        _replace(view, frame_id=frame_policy.WORLD_ENU_FRAME_ID), _deadline()
    )
    ned_view = _replace(view, frame_id=frame_policy.WORLD_NED_FRAME_ID)
    ned = _POLICY.assess_bathymetry_reference(ned_view, _deadline())
    self.assertTrue(enu.accepted)
    self.assertTrue(ned.accepted)
    self.assertEqual(ned_view.vertical_bias_m, 1.5)
    self.assertEqual(ned_view.frame_id, frame_policy.WORLD_NED_FRAME_ID)

  def test_accepted_example_at_deadline(self):
    deadline = _deadline()
    fresh = _POLICY.assess_bathymetry_reference(_valid_bathymetry(), deadline)
    self.assertIs(fresh.error, _POLICY.BathymetryReferenceError.NONE)
    self.assertIs(fresh.validity.freshness, _VALIDITY.ComponentFreshness.FRESH)
    self.assertIs(
        fresh.validity.validity, stamped_header_policy.ValidityKind.VALID
    )
    self.assertTrue(fresh.accepted)
    expired = _POLICY.assess_bathymetry_reference(
        _valid_bathymetry(), (deadline[0], deadline[1] + 1)
    )
    self.assertIs(expired.error, _POLICY.BathymetryReferenceError.NONE)
    self.assertIs(
        expired.validity.freshness, _VALIDITY.ComponentFreshness.EXPIRED
    )
    self.assertFalse(expired.accepted)

  def test_unknown_and_invalid_stay_nested(self):
    absent = _POLICY.assess_bathymetry_reference(
        _replace(
            _valid_bathymetry(),
            validity=_replace(_valid_validity(), validity_present=False),
        ),
        _deadline(),
    )
    self.assertIs(absent.error, _POLICY.BathymetryReferenceError.NONE)
    self.assertIs(
        absent.validity.validity, stamped_header_policy.ValidityKind.ABSENT
    )
    self.assertIs(
        absent.validity.freshness, _VALIDITY.ComponentFreshness.UNKNOWN
    )
    self.assertFalse(absent.accepted)

    invalid = _POLICY.assess_bathymetry_reference(
        _replace(
            _valid_bathymetry(),
            validity=_replace(_valid_validity(), validity_state=2),
        ),
        _deadline(),
    )
    self.assertIs(
        invalid.validity.validity, stamped_header_policy.ValidityKind.INVALID
    )
    self.assertFalse(invalid.accepted)

    unknown = _POLICY.assess_bathymetry_reference(
        _replace(
            _valid_bathymetry(),
            validity=_replace(_valid_validity(), validity_state=99),
        ),
        _deadline(),
    )
    self.assertIs(
        unknown.validity.validity,
        stamped_header_policy.ValidityKind.UNSPECIFIED,
    )
    self.assertIs(unknown.error, _POLICY.BathymetryReferenceError.NONE)
    self.assertFalse(unknown.accepted)

  def test_expired_frame_defect_stays_a_frame_error(self):
    deadline = _deadline()
    assessment = _POLICY.assess_bathymetry_reference(
        _replace(_valid_bathymetry(), frame_id="body"),
        (deadline[0], deadline[1] + 1),
    )
    self.assertIs(assessment.error, _POLICY.BathymetryReferenceError.FRAME_ID)
    self.assertIs(
        assessment.validity.freshness, _VALIDITY.ComponentFreshness.EXPIRED
    )
    self.assertFalse(assessment.accepted)


if __name__ == "__main__":
  unittest.main()
