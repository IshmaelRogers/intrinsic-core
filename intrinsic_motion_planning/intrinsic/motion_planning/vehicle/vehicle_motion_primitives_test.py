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

"""Tests for the UUV motion-primitive generator Python mirror."""

import dataclasses
import math
import unittest

from intrinsic.motion_planning.vehicle import vehicle_motion_primitives as prim

_Error = prim.MotionPrimitiveSetError
_NAN = math.nan
_INF = math.inf


def _control(axis, value):
  components = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
  if axis >= 0:
    components[axis] = value
  return tuple(components)


def _expect_primitive(test, primitive, name, axis, value, duration):
  test.assertEqual(primitive.id, name)
  test.assertEqual(primitive.duration_s, duration)
  test.assertEqual(primitive.control, _control(axis, value))


def _surge_config():
  return prim.VehicleMotionPrimitiveSetConfig(
      duration_s=1.0, include_hover=True, surge_present=True, surge_m_s=0.5
  )


class VehicleMotionPrimitivesTest(unittest.TestCase):

  def test_axis_index_is_stable(self):
    self.assertEqual(prim.MotionPrimitiveAxis.SURGE.value, 0)
    self.assertEqual(prim.MotionPrimitiveAxis.SWAY.value, 1)
    self.assertEqual(prim.MotionPrimitiveAxis.HEAVE.value, 2)
    self.assertEqual(prim.MotionPrimitiveAxis.ROLL.value, 3)
    self.assertEqual(prim.MotionPrimitiveAxis.PITCH.value, 4)
    self.assertEqual(prim.MotionPrimitiveAxis.YAW.value, 5)
    self.assertEqual(_Error.OK.value, 0)
    self.assertEqual(_Error.BAD_CONFIG.value, 1)
    self.assertEqual(_Error.BOUNDS_VIOLATION.value, 2)

  def test_surge_only_with_hover(self):
    result = prim.generate_motion_primitives(_surge_config())
    self.assertEqual(result.error, _Error.OK)
    self.assertEqual(len(result.primitives), 3)
    _expect_primitive(self, result.primitives[0], "hover", -1, 0.0, 1.0)
    _expect_primitive(self, result.primitives[1], "surge_pos", 0, 0.5, 1.0)
    _expect_primitive(self, result.primitives[2], "surge_neg", 0, -0.5, 1.0)

  def test_empty_set_when_hover_off_and_no_axes(self):
    config = prim.VehicleMotionPrimitiveSetConfig(include_hover=False)
    result = prim.generate_motion_primitives(config)
    self.assertEqual(result.error, _Error.OK)
    self.assertEqual(result.primitives, ())

  def test_all_six_axes_follow_locked_order(self):
    config = prim.VehicleMotionPrimitiveSetConfig(
        duration_s=0.25,
        include_hover=True,
        surge_present=True,
        surge_m_s=1.0,
        sway_present=True,
        sway_m_s=2.0,
        heave_present=True,
        heave_m_s=3.0,
        roll_present=True,
        roll_rad_s=0.1,
        pitch_present=True,
        pitch_rad_s=0.2,
        yaw_present=True,
        yaw_rad_s=0.3,
    )
    result = prim.generate_motion_primitives(config)
    self.assertEqual(result.error, _Error.OK)
    expected = (
        ("hover", -1, 0.0),
        ("surge_pos", 0, 1.0),
        ("surge_neg", 0, -1.0),
        ("sway_pos", 1, 2.0),
        ("sway_neg", 1, -2.0),
        ("heave_pos", 2, 3.0),
        ("heave_neg", 2, -3.0),
        ("roll_pos", 3, 0.1),
        ("roll_neg", 3, -0.1),
        ("pitch_pos", 4, 0.2),
        ("pitch_neg", 4, -0.2),
        ("yaw_pos", 5, 0.3),
        ("yaw_neg", 5, -0.3),
    )
    self.assertEqual(len(result.primitives), len(expected))
    for primitive, (name, axis, value) in zip(result.primitives, expected):
      _expect_primitive(self, primitive, name, axis, value, 0.25)

  def test_skips_disengaged_axes(self):
    config = prim.VehicleMotionPrimitiveSetConfig(
        duration_s=2.0,
        surge_present=True,
        surge_m_s=0.4,
        yaw_present=True,
        yaw_rad_s=0.05,
        sway_m_s=_NAN,
    )
    result = prim.generate_motion_primitives(config)
    self.assertEqual(result.error, _Error.OK)
    self.assertEqual(len(result.primitives), 5)
    _expect_primitive(self, result.primitives[0], "hover", -1, 0.0, 2.0)
    _expect_primitive(self, result.primitives[1], "surge_pos", 0, 0.4, 2.0)
    _expect_primitive(self, result.primitives[2], "surge_neg", 0, -0.4, 2.0)
    _expect_primitive(self, result.primitives[3], "yaw_pos", 5, 0.05, 2.0)
    _expect_primitive(self, result.primitives[4], "yaw_neg", 5, -0.05, 2.0)

  def test_boundary_level_equals_engaged_max(self):
    config = prim.VehicleMotionPrimitiveSetConfig(
        surge_present=True,
        surge_m_s=0.1,
        yaw_present=True,
        yaw_rad_s=0.3,
        bounds=prim.VehicleMotionPrimitiveBounds(
            max_linear_speed_present=True,
            max_linear_speed_m_s=0.1,
            max_angular_speed_present=True,
            max_angular_speed_rad_s=0.3,
        ),
    )
    result = prim.generate_motion_primitives(config)
    self.assertEqual(result.error, _Error.OK)
    self.assertEqual(len(result.primitives), 5)
    _expect_primitive(self, result.primitives[1], "surge_pos", 0, 0.1, 1.0)
    _expect_primitive(self, result.primitives[4], "yaw_neg", 5, -0.3, 1.0)

  def test_zero_speed_bound_accepts_hover_only(self):
    config = prim.VehicleMotionPrimitiveSetConfig(
        bounds=prim.VehicleMotionPrimitiveBounds(
            max_linear_speed_present=True,
            max_linear_speed_m_s=0.0,
            max_angular_speed_present=True,
            max_angular_speed_rad_s=0.0,
        )
    )
    result = prim.generate_motion_primitives(config)
    self.assertEqual(result.error, _Error.OK)
    self.assertEqual(len(result.primitives), 1)
    _expect_primitive(self, result.primitives[0], "hover", -1, 0.0, 1.0)

  def test_linear_level_above_max_rejects_whole_set(self):
    config = prim.VehicleMotionPrimitiveSetConfig(
        surge_present=True,
        surge_m_s=0.5,
        heave_present=True,
        heave_m_s=2.0,
        bounds=prim.VehicleMotionPrimitiveBounds(
            max_linear_speed_present=True, max_linear_speed_m_s=1.0
        ),
    )
    result = prim.generate_motion_primitives(config)
    self.assertEqual(result.error, _Error.BOUNDS_VIOLATION)
    self.assertEqual(result.primitives, ())

  def test_angular_level_above_max_rejects_whole_set(self):
    config = prim.VehicleMotionPrimitiveSetConfig(
        roll_present=True,
        roll_rad_s=0.2,
        bounds=prim.VehicleMotionPrimitiveBounds(
            max_angular_speed_present=True, max_angular_speed_rad_s=0.1
        ),
    )
    result = prim.generate_motion_primitives(config)
    self.assertEqual(result.error, _Error.BOUNDS_VIOLATION)
    self.assertEqual(result.primitives, ())

  def test_linear_axis_ignores_angular_bound_and_reverse(self):
    config = prim.VehicleMotionPrimitiveSetConfig(
        include_hover=False,
        surge_present=True,
        surge_m_s=4.0,
        bounds=prim.VehicleMotionPrimitiveBounds(
            max_angular_speed_present=True, max_angular_speed_rad_s=0.0
        ),
    )
    result = prim.generate_motion_primitives(config)
    self.assertEqual(result.error, _Error.OK)
    self.assertEqual(len(result.primitives), 2)

    config = prim.VehicleMotionPrimitiveSetConfig(
        include_hover=False,
        yaw_present=True,
        yaw_rad_s=4.0,
        bounds=prim.VehicleMotionPrimitiveBounds(
            max_linear_speed_present=True, max_linear_speed_m_s=0.0
        ),
    )
    result = prim.generate_motion_primitives(config)
    self.assertEqual(result.error, _Error.OK)
    self.assertEqual(len(result.primitives), 2)
    _expect_primitive(self, result.primitives[0], "yaw_pos", 5, 4.0, 1.0)

  def test_bad_duration_is_bad_config(self):
    for duration in (0.0, -1.0, _NAN, _INF, -_INF):
      config = dataclasses.replace(
          _surge_config(),
          duration_s=duration,
          bounds=prim.VehicleMotionPrimitiveBounds(
              max_linear_speed_present=True, max_linear_speed_m_s=0.1
          ),
      )
      result = prim.generate_motion_primitives(config)
      self.assertEqual(result.error, _Error.BAD_CONFIG)
      self.assertEqual(result.primitives, ())

  def test_bad_engaged_bounds_are_bad_config(self):
    for value in (-0.1, _NAN, _INF, -_INF):
      linear = dataclasses.replace(
          _surge_config(),
          bounds=prim.VehicleMotionPrimitiveBounds(
              max_linear_speed_present=True, max_linear_speed_m_s=value
          ),
      )
      result = prim.generate_motion_primitives(linear)
      self.assertEqual(result.error, _Error.BAD_CONFIG)
      self.assertEqual(result.primitives, ())

      angular = dataclasses.replace(
          _surge_config(),
          bounds=prim.VehicleMotionPrimitiveBounds(
              max_angular_speed_present=True, max_angular_speed_rad_s=value
          ),
      )
      result = prim.generate_motion_primitives(angular)
      self.assertEqual(result.error, _Error.BAD_CONFIG)
      self.assertEqual(result.primitives, ())

  def test_disengaged_bounds_ignore_invalid_stored_values(self):
    config = dataclasses.replace(
        _surge_config(),
        bounds=prim.VehicleMotionPrimitiveBounds(
            max_linear_speed_m_s=_NAN, max_angular_speed_rad_s=-5.0
        ),
    )
    result = prim.generate_motion_primitives(config)
    self.assertEqual(result.error, _Error.OK)
    self.assertEqual(len(result.primitives), 3)

  def test_non_positive_present_level_is_bad_config(self):
    for level in (0.0, -0.2, _NAN, _INF, -_INF):
      config = prim.VehicleMotionPrimitiveSetConfig(
          duration_s=1.0, sway_present=True, sway_m_s=level
      )
      result = prim.generate_motion_primitives(config)
      self.assertEqual(result.error, _Error.BAD_CONFIG)
      self.assertEqual(result.primitives, ())

  def test_bad_bounds_win_over_a_violating_level(self):
    config = dataclasses.replace(
        _surge_config(),
        surge_m_s=5.0,
        bounds=prim.VehicleMotionPrimitiveBounds(
            max_linear_speed_present=True, max_linear_speed_m_s=-1.0
        ),
    )
    result = prim.generate_motion_primitives(config)
    self.assertEqual(result.error, _Error.BAD_CONFIG)
    self.assertEqual(result.primitives, ())

  def test_same_config_is_deterministic(self):
    config = prim.VehicleMotionPrimitiveSetConfig(
        duration_s=0.5,
        include_hover=True,
        heave_present=True,
        heave_m_s=1.25,
        pitch_present=True,
        pitch_rad_s=0.125,
        bounds=prim.VehicleMotionPrimitiveBounds(
            max_linear_speed_present=True,
            max_linear_speed_m_s=1.25,
            max_angular_speed_present=True,
            max_angular_speed_rad_s=0.125,
        ),
    )
    first = prim.generate_motion_primitives(config)
    second = prim.generate_motion_primitives(config)
    self.assertEqual(first, second)
    self.assertEqual(first.error, _Error.OK)
    self.assertGreater(len(first.primitives), 0)
