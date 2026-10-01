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

"""Tests for the vehicle UUV motion-primitive generator."""

import math
import unittest

from intrinsic.motion_planning.vehicle import vehicle_motion_primitives as vmp

_Error = vmp.PrimitiveSetError
_Mode = vmp.PrimitiveGenerationMode


def _nominal(**overrides):
  fields = dict(max_force_torque=(1.0, 2.0, 3.0, 0.1, 0.2, 0.3), duration_s=0.5)
  fields.update(overrides)
  return vmp.VehicleMotionPrimitiveConfig(**fields)


class VehicleMotionPrimitivesTest(unittest.TestCase):

  def _expect_failure(self, config, error):
    result = vmp.generate_uuv_motion_primitives(config)
    self.assertEqual(result.error, error)
    self.assertEqual(result.primitives, ())

  def test_nominal_axis_aligned_has_thirteen_ordered(self):
    config = _nominal()
    result = vmp.generate_uuv_motion_primitives(config)
    self.assertEqual(result.error, _Error.OK)
    self.assertEqual(len(result.primitives), 13)
    max_ft = config.max_force_torque
    for i, p in enumerate(result.primitives):
      self.assertEqual(p.id, f"uuv-prim-{i:03d}")
      self.assertEqual(p.duration_s, 0.5)
      for axis in range(6):
        self.assertLessEqual(abs(p.control[axis]), max_ft[axis])
    self.assertEqual(result.primitives[0].id, "uuv-prim-000")
    self.assertEqual(result.primitives[12].id, "uuv-prim-012")
    self.assertEqual(result.primitives[0].control, (0.0,) * 6)
    for axis in range(6):
      positive = [0.0] * 6
      negative = [0.0] * 6
      positive[axis] = max_ft[axis]
      negative[axis] = -max_ft[axis]
      self.assertEqual(result.primitives[1 + 2 * axis].control, tuple(positive))
      self.assertEqual(result.primitives[2 + 2 * axis].control, tuple(negative))

  def test_zero_bound_axis_still_emits_both_slots(self):
    result = vmp.generate_uuv_motion_primitives(
        _nominal(max_force_torque=(1.0, 0.0, 3.0, 0.1, 0.2, 0.3))
    )
    nominal = vmp.generate_uuv_motion_primitives(_nominal())
    self.assertEqual(result.error, _Error.OK)
    self.assertEqual(len(result.primitives), 13)
    self.assertEqual(result.primitives[3].control, (0.0,) * 6)
    self.assertEqual(result.primitives[4].control, (0.0,) * 6)
    self.assertEqual(math.copysign(1.0, result.primitives[4].control[1]), 1.0)
    for i in range(13):
      if i in (3, 4):
        continue
      self.assertEqual(
          result.primitives[i].control, nominal.primitives[i].control
      )

  def test_non_zero_primitives_sit_exactly_on_bound(self):
    config = _nominal()
    result = vmp.generate_uuv_motion_primitives(config)
    for p in result.primitives:
      nonzero = [axis for axis in range(6) if p.control[axis] != 0.0]
      self.assertLessEqual(len(nonzero), 1)
      for axis in nonzero:
        self.assertEqual(abs(p.control[axis]), config.max_force_torque[axis])

  def test_bad_config_is_rejected(self):
    for axis in range(6):
      for bad in (-1.0, -1e-12, math.nan, math.inf, -math.inf):
        max_ft = [1.0, 2.0, 3.0, 0.1, 0.2, 0.3]
        max_ft[axis] = bad
        self._expect_failure(
            _nominal(max_force_torque=tuple(max_ft)), _Error.BAD_CONFIG
        )
    for bad in (0.0, -0.5, math.nan, math.inf, -math.inf):
      self._expect_failure(_nominal(duration_s=bad), _Error.BAD_CONFIG)
    self._expect_failure(vmp.VehicleMotionPrimitiveConfig(), _Error.BAD_CONFIG)

  def test_custom_empty_is_empty(self):
    self._expect_failure(_nominal(mode=_Mode.CUSTOM), _Error.EMPTY)

  def test_custom_bad_config_beats_empty(self):
    self._expect_failure(
        _nominal(mode=_Mode.CUSTOM, duration_s=0.0), _Error.BAD_CONFIG
    )

  def test_custom_out_of_bounds_is_rejected_not_clamped(self):
    ok = (0.5, 0.0, 0.0, 0.0, 0.0, 0.0)
    over = (
        (1.0000001, 0, 0, 0, 0, 0),
        (0, -2.5, 0, 0, 0, 0),
        (0, 0, 3.5, 0, 0, 0),
        (0, 0, 0, 0.11, 0, 0),
        (0, 0, 0, 0, -0.21, 0),
        (0, 0, 0, 0, 0, 0.31),
    )
    for bad in over:
      self._expect_failure(
          _nominal(mode=_Mode.CUSTOM, custom_controls=(ok, bad)),
          _Error.BAD_CONFIG,
      )
    for bad in (math.nan, math.inf, -math.inf):
      self._expect_failure(
          _nominal(mode=_Mode.CUSTOM, custom_controls=((0, 0, bad, 0, 0, 0),)),
          _Error.BAD_CONFIG,
      )

  def test_custom_in_bounds_preserves_order(self):
    controls = (
        (0.5, -1.0, 3.0, 0.0, 0.2, -0.3),
        (0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
        (-1.0, 2.0, -3.0, 0.1, -0.2, 0.3),
    )
    result = vmp.generate_uuv_motion_primitives(
        _nominal(mode=_Mode.CUSTOM, custom_controls=controls)
    )
    self.assertEqual(result.error, _Error.OK)
    self.assertEqual(len(result.primitives), 3)
    for i, p in enumerate(result.primitives):
      self.assertEqual(p.id, f"uuv-prim-{i:03d}")
      self.assertEqual(p.control, controls[i])
      self.assertEqual(p.duration_s, 0.5)

  def test_axis_aligned_ignores_custom_controls(self):
    result = vmp.generate_uuv_motion_primitives(
        _nominal(custom_controls=((100.0, 0, 0, 0, 0, 0),))
    )
    self.assertEqual(result.error, _Error.OK)
    self.assertEqual(len(result.primitives), 13)

  def test_is_deterministic(self):
    config = _nominal()
    self.assertEqual(
        vmp.generate_uuv_motion_primitives(config),
        vmp.generate_uuv_motion_primitives(config),
    )


if __name__ == "__main__":
  unittest.main()
