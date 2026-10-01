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

"""Tests for current-aware UUV primitive propagation."""

import math
import unittest

from intrinsic.motion_planning.vehicle import vehicle_primitive_propagation as prop

_Error = prop.PropagationError
_ZERO_TWIST = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
_IDENTITY = (0.0, 0.0, 0.0, 1.0)


def _origin():
  return prop.VehiclePlanningState()


def _surge(duration_s):
  return prop.VehicleMotionPrimitive(
      id="uuv-prim-000",
      control=(10.0, 0.0, 0.0, 0.0, 0.0, 0.0),
      duration_s=duration_s,
  )


def _config(**overrides):
  fields = dict(
      dt_s=0.5,
      max_steps=4,
      mass_diag=(10.0, 10.0, 10.0, 10.0, 10.0, 10.0),
      gravity_m_s2=0.0,
      fluid_density_kg_m3=0.0,
      current_world_enu_m_s=(0.0, 0.0, 0.0),
  )
  fields.update(overrides)
  return prop.PropagationConfig(**fields)


def _rotate_body_to_pose(quaternion, body):
  x, y, z, w = quaternion
  xx, yy, zz = x * x, y * y, z * z
  xy, xz, yz = x * y, x * z, y * z
  wx, wy, wz = w * x, w * y, w * z
  r00 = 1.0 - 2.0 * (yy + zz)
  r01 = 2.0 * (xy - wz)
  r02 = 2.0 * (xz + wy)
  r10 = 2.0 * (xy + wz)
  r11 = 1.0 - 2.0 * (xx + zz)
  r12 = 2.0 * (yz - wx)
  r20 = 2.0 * (xz - wy)
  r21 = 2.0 * (yz + wx)
  r22 = 1.0 - 2.0 * (xx + yy)
  return (
      r00 * body[0] + r01 * body[1] + r02 * body[2],
      r10 * body[0] + r11 * body[1] + r12 * body[2],
      r20 * body[0] + r21 * body[1] + r22 * body[2],
  )


def _quaternion_derivative(quaternion, twist):
  x, y, z, w = quaternion
  wx, wy, wz = twist[3], twist[4], twist[5]
  # 0.5 * q ⊗ (wx, wy, wz, 0), Hamilton product, stored x, y, z, w.
  return (
      0.5 * (w * wx + y * wz - z * wy),
      0.5 * (w * wy - x * wz + z * wx),
      0.5 * (w * wz + x * wy - y * wx),
      0.5 * (-x * wx - y * wy - z * wz),
  )


class ZeroForceDynamics:
  """Python stand-in for intrinsic::vehicle::dynamics::ZeroForceDynamics.

  Pose rate is the rigid-body kinematic map of body twist. The input wrench,
  gravity, density, and current are checked and then left unused.
  input_wrench_used stays false, so planning uses the primitive wrench.
  """

  def evaluate(self, state, wrench, environment, dt):
    if state.pose_frame is not prop.FrameId.WORLD_ENU:
      return self._invalid("state pose frame must be world_enu or world_ned")
    if wrench.frame is not prop.FrameId.BODY:
      return self._invalid("wrench frame must be body")
    if environment.current_frame is not prop.FrameId.WORLD_ENU:
      return self._invalid(
          "environment current frame must be world_enu, world_ned, or body"
      )
    if not math.isfinite(dt.seconds) or dt.seconds < 0.0:
      return self._invalid(
          "time step must be finite and greater than or equal to zero"
      )
    derivative = prop.StateDerivative(
        position_dot_m_s=_rotate_body_to_pose(
            state.orientation_xyzw, state.body_twist[:3]
        ),
        orientation_dot_xyzw=_quaternion_derivative(
            state.orientation_xyzw, state.body_twist
        ),
    )
    diagnostics = prop.DynamicsDiagnostics(
        model_id="zero_force", dt_s=dt.seconds
    )
    return prop.StatusOr(
        status=prop.DynamicsStatus(),
        value=prop.DynamicsResult(
            derivative=derivative, diagnostics=diagnostics
        ),
    )

  def _invalid(self, message):
    return prop.StatusOr(
        status=prop.DynamicsStatus(
            code=prop.DynamicsError.INVALID_ARGUMENT, message=message
        )
    )


class FailingDynamics:

  def evaluate(self, state, wrench, environment, dt):
    del state, wrench, environment, dt
    return prop.StatusOr(
        status=prop.DynamicsStatus(
            code=prop.DynamicsError.INVALID_ARGUMENT, message="injected failure"
        )
    )


class SecondEvaluateFails:

  def __init__(self):
    self._calls = 0
    self._inner = ZeroForceDynamics()

  def evaluate(self, state, wrench, environment, dt):
    self._calls += 1
    if self._calls >= 2:
      return prop.StatusOr(
          status=prop.DynamicsStatus(
              code=prop.DynamicsError.INVALID_ARGUMENT,
              message="second evaluate",
          )
      )
    return self._inner.evaluate(state, wrench, environment, dt)


class TotalWrenchDynamics:
  """input_wrench_used selects total_wrench. Pose rates stay kinematic."""

  def __init__(self):
    self._inner = ZeroForceDynamics()

  def evaluate(self, state, wrench, environment, dt):
    evaluated = self._inner.evaluate(state, wrench, environment, dt)
    if not evaluated.ok():
      return evaluated
    diagnostics = prop.DynamicsDiagnostics(
        model_id="zero_force",
        dt_s=dt.seconds,
        input_wrench_used=True,
        total_wrench=(20.0, 0.0, 0.0, 0.0, 0.0, 0.0),
    )
    return prop.StatusOr(
        status=prop.DynamicsStatus(),
        value=prop.DynamicsResult(
            derivative=evaluated.value.derivative, diagnostics=diagnostics
        ),
    )


class NonFiniteOrientationRate:

  def __init__(self):
    self._inner = ZeroForceDynamics()

  def evaluate(self, state, wrench, environment, dt):
    evaluated = self._inner.evaluate(state, wrench, environment, dt)
    if not evaluated.ok():
      return evaluated
    derivative = prop.StateDerivative(
        position_dot_m_s=evaluated.value.derivative.position_dot_m_s,
        orientation_dot_xyzw=(1e308, 1e308, 1e308, 1e308),
    )
    return prop.StatusOr(
        status=prop.DynamicsStatus(),
        value=prop.DynamicsResult(
            derivative=derivative, diagnostics=evaluated.value.diagnostics
        ),
    )


class VehiclePrimitivePropagationTest(unittest.TestCase):

  def _expect_failure(self, start, primitive, config, dynamics, error):
    result = prop.propagate_uuv_motion_primitive(
        start, primitive, config, dynamics
    )
    self.assertEqual(result.error, error)
    self.assertEqual(result.samples, ())

  def _expect_other_zero(self, state):
    self.assertEqual(state.position[1], 0.0)
    self.assertEqual(state.position[2], 0.0)
    for actual, expected in zip(state.orientation, _IDENTITY):
      self.assertAlmostEqual(actual, expected, delta=1e-9)
    self.assertEqual(state.twist[1:], _ZERO_TWIST[1:])

  def test_zero_current_nominal(self):
    result = prop.propagate_uuv_motion_primitive(
        _origin(), _surge(1.0), _config(), ZeroForceDynamics()
    )
    self.assertEqual(result.error, _Error.OK)
    self.assertEqual(len(result.samples), 3)
    self.assertEqual(result.samples[0].time_s, 0.0)
    self.assertEqual(result.samples[0].state.position[0], 0.0)
    self.assertEqual(result.samples[0].state.twist[0], 0.0)
    self.assertEqual(result.samples[1].time_s, 0.5)
    self.assertEqual(result.samples[1].state.twist[0], 0.5)
    self.assertEqual(result.samples[1].state.position[0], 0.25)
    self._expect_other_zero(result.samples[1].state)
    self.assertEqual(result.samples[2].time_s, 1.0)
    self.assertEqual(result.samples[2].state.twist[0], 1.0)
    self.assertEqual(result.samples[2].state.position[0], 0.75)
    self._expect_other_zero(result.samples[2].state)

  def test_constant_current_adds_world_drift(self):
    result = prop.propagate_uuv_motion_primitive(
        _origin(),
        _surge(1.0),
        _config(current_world_enu_m_s=(0.2, 0.0, 0.0)),
        ZeroForceDynamics(),
    )
    self.assertEqual(result.error, _Error.OK)
    self.assertEqual(len(result.samples), 3)
    self.assertEqual(result.samples[1].time_s, 0.5)
    self.assertEqual(result.samples[1].state.twist[0], 0.5)
    self.assertEqual(result.samples[1].state.position[0], 0.35)
    self._expect_other_zero(result.samples[1].state)
    self.assertEqual(result.samples[2].time_s, 1.0)
    self.assertEqual(result.samples[2].state.twist[0], 1.0)
    self.assertEqual(result.samples[2].state.position[0], 0.95)
    self._expect_other_zero(result.samples[2].state)

  def test_bad_config_is_rejected(self):
    dynamics = ZeroForceDynamics()
    start = _origin()
    primitive = _surge(1.0)
    for dt_s in (0.0, -0.5, math.nan):
      self._expect_failure(
          start, primitive, _config(dt_s=dt_s), dynamics, _Error.BAD_CONFIG
      )
    for max_steps in (0, -3):
      self._expect_failure(
          start,
          primitive,
          _config(max_steps=max_steps),
          dynamics,
          _Error.BAD_CONFIG,
      )
    for mass_z in (0.0, -1.0, math.nan):
      mass = [10.0] * 6
      mass[2] = mass_z
      self._expect_failure(
          start,
          primitive,
          _config(mass_diag=tuple(mass)),
          dynamics,
          _Error.BAD_CONFIG,
      )
    self._expect_failure(
        start,
        primitive,
        _config(gravity_m_s2=-1.0),
        dynamics,
        _Error.BAD_CONFIG,
    )
    self._expect_failure(
        start,
        primitive,
        _config(gravity_m_s2=math.inf),
        dynamics,
        _Error.BAD_CONFIG,
    )
    self._expect_failure(
        start,
        primitive,
        _config(fluid_density_kg_m3=-1.0),
        dynamics,
        _Error.BAD_CONFIG,
    )
    self._expect_failure(
        start,
        primitive,
        _config(current_world_enu_m_s=(0.0, math.nan, 0.0)),
        dynamics,
        _Error.BAD_CONFIG,
    )

  def test_bad_config_beats_bad_start(self):
    start = prop.VehiclePlanningState(position=(math.nan, 0.0, 0.0))
    self._expect_failure(
        start,
        _surge(1.0),
        _config(dt_s=0.0),
        ZeroForceDynamics(),
        _Error.BAD_CONFIG,
    )

  def test_bad_start_is_rejected(self):
    dynamics = ZeroForceDynamics()
    config = _config()
    primitive = _surge(1.0)
    self._expect_failure(
        prop.VehiclePlanningState(position=(0.0, math.inf, 0.0)),
        primitive,
        config,
        dynamics,
        _Error.BAD_START,
    )
    self._expect_failure(
        prop.VehiclePlanningState(twist=(0.0, 0.0, 0.0, 0.0, 0.0, math.nan)),
        primitive,
        config,
        dynamics,
        _Error.BAD_START,
    )
    self._expect_failure(
        prop.VehiclePlanningState(orientation=(0.0, 0.0, 0.0, 2.0)),
        primitive,
        config,
        dynamics,
        _Error.BAD_START,
    )

  def test_bad_primitive_is_rejected(self):
    dynamics = ZeroForceDynamics()
    config = _config()
    for duration_s in (0.0, -1.0, math.nan):
      self._expect_failure(
          _origin(), _surge(duration_s), config, dynamics, _Error.BAD_PRIMITIVE
      )
    primitive = prop.VehicleMotionPrimitive(
        id="uuv-prim-000",
        control=(10.0, 0.0, 0.0, 0.0, math.inf, 0.0),
        duration_s=1.0,
    )
    self._expect_failure(
        _origin(), primitive, config, dynamics, _Error.BAD_PRIMITIVE
    )

  def test_step_budget_fails_before_integrating(self):
    self._expect_failure(
        _origin(),
        _surge(1.0),
        _config(max_steps=1),
        ZeroForceDynamics(),
        _Error.STEP_BUDGET,
    )
    result = prop.propagate_uuv_motion_primitive(
        _origin(), _surge(1.0), _config(max_steps=2), ZeroForceDynamics()
    )
    self.assertEqual(result.error, _Error.OK)
    self.assertEqual(len(result.samples), 3)

  def test_dynamics_failure_clears_samples(self):
    nominal = (_origin(), _surge(1.0), _config())
    self._expect_failure(*nominal, FailingDynamics(), _Error.DYNAMICS_FAILED)
    self._expect_failure(
        *nominal, SecondEvaluateFails(), _Error.DYNAMICS_FAILED
    )
    self._expect_failure(
        *nominal, NonFiniteOrientationRate(), _Error.DYNAMICS_FAILED
    )

  def test_input_wrench_used_selects_total_wrench(self):
    result = prop.propagate_uuv_motion_primitive(
        _origin(), _surge(0.5), _config(), TotalWrenchDynamics()
    )
    self.assertEqual(result.error, _Error.OK)
    self.assertEqual(len(result.samples), 2)
    # a = 20/10 = 2. Semi-implicit: twist = 1, position = 0.5 * 1.
    self.assertEqual(result.samples[1].time_s, 0.5)
    self.assertEqual(result.samples[1].state.twist[0], 1.0)
    self.assertEqual(result.samples[1].state.position[0], 0.5)

  def test_is_deterministic(self):
    args = (_origin(), _surge(1.0), _config())
    first = prop.propagate_uuv_motion_primitive(*args, ZeroForceDynamics())
    second = prop.propagate_uuv_motion_primitive(*args, ZeroForceDynamics())
    self.assertEqual(first.error, _Error.OK)
    self.assertEqual(second.error, _Error.OK)
    self.assertEqual(len(first.samples), len(second.samples))
    for left, right in zip(first.samples, second.samples):
      self.assertEqual(left.time_s, right.time_s)
      self.assertEqual(left.state, right.state)

  def test_remainder_step_lands_on_duration(self):
    result = prop.propagate_uuv_motion_primitive(
        _origin(), _surge(0.75), _config(), ZeroForceDynamics()
    )
    self.assertEqual(result.error, _Error.OK)
    self.assertEqual(len(result.samples), 3)
    self.assertEqual(result.samples[1].time_s, 0.5)
    self.assertEqual(result.samples[1].state.twist[0], 0.5)
    self.assertEqual(result.samples[1].state.position[0], 0.25)
    self.assertEqual(result.samples[2].time_s, 0.75)
    self.assertEqual(result.samples[2].state.twist[0], 0.75)
    self.assertEqual(result.samples[2].state.position[0], 0.4375)
    self.assertAlmostEqual(result.samples[2].time_s, 0.75, delta=1e-12)


if __name__ == "__main__":
  unittest.main()
