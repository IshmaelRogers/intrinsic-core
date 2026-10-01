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

"""Geometry tests for the geofence.aabb rule."""

import itertools
import math
import unittest

from intrinsic.safety import geofence_rule
from intrinsic.safety import safety_decision_policy
from intrinsic.safety import safety_rule_result

_RULE = geofence_rule
_DECISION = safety_decision_policy
_RESULT = safety_rule_result

_EPS = 1e-6


def _fence(**overrides):
  fields = dict(
      frame_id="map",
      region_id="dock_area",
      min_x=-1.0,
      min_y=-2.0,
      min_z=-3.0,
      max_x=1.0,
      max_y=2.0,
      max_z=3.0,
  )
  fields.update(overrides)
  return _RULE.AabbGeofence(**fields)


def _pose(x, y, z, frame_id="map"):
  return _RULE.GeofencePose(frame_id=frame_id, x=x, y=y, z=z)


class GeofenceAabbRuleTest(unittest.TestCase):

  def _expect_compliant(self, result):
    self.assertFalse(result.violated)
    self.assertEqual(result.rule_id, "")
    self.assertEqual(result.severity, 0)
    self.assertEqual(result.summary, "")
    self.assertEqual(result.recommended_kind, 0)
    self.assertFalse(result.has_projected_value)
    self.assertEqual(result.projected_value, 0.0)

  def _expect_reject(self, result, severity):
    self.assertTrue(result.violated)
    self.assertEqual(result.rule_id, "geofence.aabb")
    self.assertEqual(result.rule_id, _RULE.GEOFENCE_AABB_RULE_ID)
    self.assertEqual(result.severity, severity)
    self.assertNotEqual(result.summary, "")
    self.assertEqual(result.recommended_kind, 3)
    self.assertEqual(result.recommended_kind, _RESULT.DECISION_KIND_REJECT)
    self.assertEqual(
        result.recommended_kind, _DECISION.SafetyDecisionKind.REJECT.value
    )
    self.assertFalse(result.has_projected_value)
    self.assertEqual(result.projected_value, 0.0)

  def _expect_outside(self, result):
    self._expect_reject(result, 3)
    self.assertEqual(
        result.severity, _DECISION.SafetyFindingSeverityKind.ERROR.value
    )
    self.assertIn("dock_area", result.summary)

  def _expect_critical(self, result):
    self._expect_reject(result, 4)
    self.assertEqual(
        result.severity, _DECISION.SafetyFindingSeverityKind.CRITICAL.value
    )

  def test_rule_id_is_locked(self):
    self.assertEqual(_RULE.GEOFENCE_AABB_RULE_ID, "geofence.aabb")

  def test_inside_center_is_compliant(self):
    self._expect_compliant(
        _RULE.evaluate_geofence_aabb_rule(_fence(), _pose(0, 0, 0))
    )
    self._expect_compliant(
        _RULE.evaluate_geofence_aabb_rule(_fence(), _pose(0.5, -1.5, 2.5))
    )

  def test_faces_edges_and_corners_are_inclusive(self):
    # Every combination of {min, mid, max} on each axis lies on or in the box.
    for x, y, z in itertools.product(
        (-1.0, 0.0, 1.0), (-2.0, 0.0, 2.0), (-3.0, 0.0, 3.0)
    ):
      self._expect_compliant(
          _RULE.evaluate_geofence_aabb_rule(_fence(), _pose(x, y, z))
      )

  def test_just_outside_each_axis_is_rejected(self):
    for pose in (
        _pose(1.0 + _EPS, 0, 0),
        _pose(-1.0 - _EPS, 0, 0),
        _pose(0, 2.0 + _EPS, 0),
        _pose(0, -2.0 - _EPS, 0),
        _pose(0, 0, 3.0 + _EPS),
        _pose(0, 0, -3.0 - _EPS),
    ):
      self._expect_outside(_RULE.evaluate_geofence_aabb_rule(_fence(), pose))

  def test_just_outside_combinations_are_rejected(self):
    for pose in (
        _pose(1.0 + _EPS, 2.0 + _EPS, 0),
        _pose(1.0 + _EPS, 0, 3.0 + _EPS),
        _pose(0, 2.0 + _EPS, 3.0 + _EPS),
        _pose(1.0 + _EPS, 2.0 + _EPS, 3.0 + _EPS),
        _pose(100.0, -100.0, 100.0),
    ):
      self._expect_outside(_RULE.evaluate_geofence_aabb_rule(_fence(), pose))

  def test_outside_summary_names_region_and_never_projects(self):
    result = _RULE.evaluate_geofence_aabb_rule(_fence(), _pose(5, 0, 0))
    self.assertEqual(result.summary, "geofence outside region=dock_area")
    self.assertNotEqual(result.recommended_kind, _RESULT.DECISION_KIND_PROJECT)
    self.assertFalse(result.has_projected_value)

  def test_frame_mismatch_is_critical_even_when_coordinates_inside(self):
    result = _RULE.evaluate_geofence_aabb_rule(
        _fence(), _pose(0, 0, 0, frame_id="odom")
    )
    self._expect_critical(result)
    self.assertIn("frame mismatch", result.summary)

  def test_frame_match_is_case_sensitive(self):
    for frame_id in ("Map", "map "):
      self._expect_critical(
          _RULE.evaluate_geofence_aabb_rule(
              _fence(), _pose(0, 0, 0, frame_id=frame_id)
          )
      )

  def test_min_greater_than_max_is_critical(self):
    for overrides in ({"min_x": 2.0}, {"min_y": 3.0}, {"max_z": -4.0}):
      self._expect_critical(
          _RULE.evaluate_geofence_aabb_rule(
              _fence(**overrides), _pose(0, 0, 0)
          )
      )

  def test_degenerate_box_is_a_valid_point(self):
    fence = _fence(
        min_x=1.0, min_y=1.0, min_z=1.0, max_x=1.0, max_y=1.0, max_z=1.0
    )
    self._expect_compliant(
        _RULE.evaluate_geofence_aabb_rule(fence, _pose(1, 1, 1))
    )

  def test_empty_ids_are_critical(self):
    self._expect_critical(
        _RULE.evaluate_geofence_aabb_rule(
            _fence(frame_id=""), _pose(0, 0, 0, frame_id="")
        )
    )
    self._expect_critical(
        _RULE.evaluate_geofence_aabb_rule(_fence(frame_id=""), _pose(0, 0, 0))
    )
    self._expect_critical(
        _RULE.evaluate_geofence_aabb_rule(_fence(region_id=""), _pose(0, 0, 0))
    )
    self._expect_critical(
        _RULE.evaluate_geofence_aabb_rule(
            _fence(), _pose(0, 0, 0, frame_id="")
        )
    )

  def test_non_finite_pose_is_critical(self):
    for pose in (
        _pose(math.nan, 0, 0),
        _pose(0, math.nan, 0),
        _pose(0, 0, math.nan),
        _pose(math.inf, 0, 0),
        _pose(0, -math.inf, 0),
    ):
      self._expect_critical(_RULE.evaluate_geofence_aabb_rule(_fence(), pose))

  def test_non_finite_bound_is_critical(self):
    for name in ("min_x", "min_y", "min_z", "max_x", "max_y", "max_z"):
      for bad in (math.nan, math.inf, -math.inf):
        self._expect_critical(
            _RULE.evaluate_geofence_aabb_rule(
                _fence(**{name: bad}), _pose(0, 0, 0)
            )
        )


class GeofenceAabbSegmentRuleTest(unittest.TestCase):

  def _expect_compliant(self, result):
    self.assertFalse(result.violated)
    self.assertEqual(result.rule_id, "")
    self.assertEqual(result.severity, 0)
    self.assertEqual(result.summary, "")
    self.assertEqual(result.recommended_kind, 0)
    self.assertFalse(result.has_projected_value)

  def _expect_reject(self, result, severity):
    self.assertTrue(result.violated)
    self.assertEqual(result.rule_id, "geofence.aabb")
    self.assertEqual(result.severity, severity)
    self.assertEqual(result.recommended_kind, _RESULT.DECISION_KIND_REJECT)
    self.assertFalse(result.has_projected_value)
    self.assertEqual(result.projected_value, 0.0)

  def _expect_outside(self, result):
    self._expect_reject(result, 3)
    self.assertIn("dock_area", result.summary)

  def test_both_inside_is_compliant(self):
    self._expect_compliant(
        _RULE.evaluate_geofence_aabb_segment_rule(
            _fence(), _pose(-1, -2, -3), _pose(1, 2, 3)
        )
    )
    self._expect_compliant(
        _RULE.evaluate_geofence_aabb_segment_rule(
            _fence(), _pose(0, 0, 0), _pose(0, 0, 0)
        )
    )

  def test_start_outside_is_rejected(self):
    result = _RULE.evaluate_geofence_aabb_segment_rule(
        _fence(), _pose(5, 0, 0), _pose(0, 0, 0)
    )
    self._expect_outside(result)
    self.assertEqual(
        result.summary, "geofence segment outside region=dock_area end=start"
    )

  def test_end_outside_is_rejected(self):
    result = _RULE.evaluate_geofence_aabb_segment_rule(
        _fence(), _pose(0, 0, 0), _pose(0, 0, 5)
    )
    self._expect_outside(result)
    self.assertEqual(
        result.summary, "geofence segment outside region=dock_area end=end"
    )

  def test_both_outside_is_rejected(self):
    # Crosses the box, but endpoint-only checking rejects it by design.
    crossing = _RULE.evaluate_geofence_aabb_segment_rule(
        _fence(), _pose(-5, 0, 0), _pose(5, 0, 0)
    )
    self._expect_outside(crossing)
    self.assertEqual(
        crossing.summary, "geofence segment outside region=dock_area end=both"
    )
    self._expect_outside(
        _RULE.evaluate_geofence_aabb_segment_rule(
            _fence(), _pose(5, 5, 5), _pose(6, 6, 6)
        )
    )

  def test_invalid_inputs_are_critical(self):
    for start, end in (
        (_pose(0, 0, 0, frame_id="odom"), _pose(0, 0, 0)),
        (_pose(0, 0, 0), _pose(0, 0, 0, frame_id="odom")),
        (_pose(math.nan, 0, 0), _pose(0, 0, 0)),
        (_pose(0, 0, 0), _pose(0, math.inf, 0)),
        (_pose(0, 0, 0, frame_id=""), _pose(0, 0, 0)),
    ):
      self._expect_reject(
          _RULE.evaluate_geofence_aabb_segment_rule(_fence(), start, end), 4
      )

  def test_invalid_pose_beats_outside_endpoint(self):
    self._expect_reject(
        _RULE.evaluate_geofence_aabb_segment_rule(
            _fence(), _pose(5, 0, 0), _pose(0, 0, 0, frame_id="odom")
        ),
        4,
    )

  def test_bad_fence_is_critical(self):
    for fence in (
        _fence(min_x=2.0),
        _fence(region_id=""),
        _fence(max_y=math.nan),
    ):
      self._expect_reject(
          _RULE.evaluate_geofence_aabb_segment_rule(
              fence, _pose(0, 0, 0), _pose(0, 0, 0)
          ),
          4,
      )


if __name__ == "__main__":
  unittest.main()
