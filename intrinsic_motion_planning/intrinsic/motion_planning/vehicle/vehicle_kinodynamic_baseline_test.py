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

"""Tests for the discrete kinodynamic lattice Dijkstra baseline."""

import dataclasses
import math
import threading
import time
import unittest

from intrinsic.motion_planning.vehicle import fake_vehicle_planner
from intrinsic.motion_planning.vehicle import vehicle_kinodynamic_baseline as kd
from intrinsic.motion_planning.vehicle import vehicle_motion_primitives as prims
from intrinsic.motion_planning.vehicle import (
    vehicle_planner_deadline as deadline,
)
from intrinsic.motion_planning.vehicle import (
    vehicle_planner_registry as registry,
)
from intrinsic.motion_planning.vehicle import (
    vehicle_primitive_propagation as prop,
)
from intrinsic.motion_planning.vehicle import vehicle_state_space
from intrinsic.motion_planning.vehicle import (
    vehicle_trajectory_validity as validity,
)
from intrinsic.vehicle import trajectory_contract_policy
from intrinsic.world.world_snapshot import world_snapshot_policy as snapshot

_Status = registry.VehiclePlanStatus
_FRAME = "world_enu"
_REGION = "ops-box"
_OCCUPANCY = snapshot.OCCUPANCY_REFERENCE_KIND
_COMPONENTS = (snapshot.ComponentRevisionView(_OCCUPANCY, 1),)
_MAX = (20.0, 20.0, 20.0, 20.0, 20.0, 20.0)


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
  return (
      0.5 * (w * wx + y * wz - z * wy),
      0.5 * (w * wy - x * wz + z * wx),
      0.5 * (w * wz + x * wy - y * wx),
      0.5 * (-x * wx - y * wy - z * wz),
  )


class ZeroForceDynamics:
  """Python stand-in for ZeroForceDynamics. input_wrench_used stays false."""

  def __init__(self):
    self.calls = 0

  def evaluate(self, state, wrench, environment, dt):
    self.calls += 1
    if state.pose_frame is not prop.FrameId.WORLD_ENU:
      return self._invalid("state pose frame must be world_enu or world_ned")
    if wrench.frame is not prop.FrameId.BODY:
      return self._invalid("wrench frame must be body")
    if environment.current_frame is not prop.FrameId.WORLD_ENU:
      return self._invalid("environment current frame must be world_enu")
    if not math.isfinite(dt.seconds) or dt.seconds < 0.0:
      return self._invalid("time step must be finite and non-negative")
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


class _CancelDynamics(ZeroForceDynamics):
  """Sets the cancel flag on the first evaluate, then delegates."""

  def __init__(self, cancel):
    super().__init__()
    self._cancel = cancel

  def evaluate(self, state, wrench, environment, dt):
    self.calls += 1
    self._cancel.set()
    # Call the parent counter a second time; keep the physics call direct.
    self.calls -= 1
    return super().evaluate(state, wrench, environment, dt)


def _view():
  return snapshot.WorldSnapshotView(
      present=True,
      snapshot_id=snapshot.compute_snapshot_id(1, _COMPONENTS),
      creation_time_present=True,
      creation_time=(100, 0),
      state_epoch=1,
      components=_COMPONENTS,
  )


def _propagation():
  return prop.PropagationConfig(
      dt_s=0.5,
      max_steps=4,
      mass_diag=(10.0, 10.0, 10.0, 10.0, 10.0, 10.0),
      gravity_m_s2=0.0,
      fluid_density_kg_m3=0.0,
      current_world_enu_m_s=(0.0, 0.0, 0.0),
  )


def _controls(*forces):
  return tuple(
      (force, 0.0, 0.0, 0.0, 0.0, 0.0) for force in forces
  )


def _config(forces=(10.0,), min_x=-2.0, max_x=2.0, min_y=-2.0, max_y=2.0):
  view = _view()
  return kd.KinodynamicBaselineConfig(
      primitives=prims.VehicleMotionPrimitiveConfig(
          mode=prims.PrimitiveGenerationMode.CUSTOM,
          max_force_torque=_MAX,
          duration_s=1.0,
          custom_controls=_controls(*forces),
      ),
      propagation=_propagation(),
      snapshot_id=view.snapshot_id,
      descriptor=view,
      fence=validity.AabbGeofence(
          _FRAME, _REGION, min_x, min_y, -2.0, max_x, max_y, 2.0
      ),
      goal_tolerance=1e-6,
      position_bin_m=0.25,
      max_expansions=64,
      clearance_template_m=1.0,
  )


def _origin():
  return vehicle_state_space.VehiclePlanningState()


def _request(goal, start=None, trajectory_id=""):
  return registry.VehiclePlanRequest(
      start_label="start",
      goal_label="goal",
      trajectory_id=trajectory_id,
      states_present=True,
      start_state=start if start is not None else _origin(),
      goal_state=goal,
  )


def _propagate(start, force, propagation=None):
  primitive = prop.VehicleMotionPrimitive(
      id="uuv-prim-000",
      control=(force, 0.0, 0.0, 0.0, 0.0, 0.0),
      duration_s=1.0,
  )
  return prop.propagate_uuv_motion_primitive(
      start,
      primitive,
      _propagation() if propagation is None else propagation,
      ZeroForceDynamics(),
  )


class KinodynamicBaselineTest(unittest.TestCase):

  def assert_empty(self, result, status):
    self.assertIs(result.status, status)
    self.assertFalse(result.header_present)
    self.assertEqual(result.frame_id, "")
    self.assertEqual(result.trajectory_id, "")
    self.assertEqual(result.samples, ())
    self.assertFalse(result.provenance_present)
    view = registry.as_vehicle_trajectory_view(result)
    self.assertFalse(trajectory_contract_policy.trajectory_engaged(view))
    self.assertFalse(
        trajectory_contract_policy.assess_vehicle_trajectory(view).accepted
    )

  def assert_accepted(self, result):
    view = registry.as_vehicle_trajectory_view(result)
    assessment = trajectory_contract_policy.assess_vehicle_trajectory(view)
    self.assertTrue(assessment.accepted)
    self.assertEqual(result.frame_id, _FRAME)
    self.assertEqual(result.model_id, "uuv_planner")
    self.assertTrue(result.header_present)
    self.assertEqual(result.validity_state, 1)
    previous = None
    for sample in result.samples:
      self.assertTrue(sample.time_present)
      self.assertTrue(all(math.isfinite(v) for v in sample.position))
      self.assertTrue(all(math.isfinite(v) for v in sample.orientation))
      self.assertTrue(all(math.isfinite(v) for v in sample.twist))
      if previous is not None:
        self.assertTrue(
            trajectory_contract_policy.sample_time_before(previous, sample)
        )
      previous = sample

  def test_reachable_matches_propagation(self):
    propagated = _propagate(_origin(), 10.0)
    self.assertEqual(propagated.error, prop.PropagationError.OK)
    goal = propagated.samples[-1].state
    self.assertEqual(goal.position[0], 0.75)
    self.assertEqual(goal.twist[0], 1.0)
    result = kd.search_kinodynamic_baseline(
        _config(), _request(goal, trajectory_id="reach"), ZeroForceDynamics()
    )
    self.assertIs(result.status, _Status.OK)
    self.assertEqual(result.trajectory_id, "reach")
    self.assert_accepted(result)
    self.assertEqual(len(result.samples), len(propagated.samples))
    for sample, expected in zip(result.samples, propagated.samples):
      self.assertEqual(sample.position, tuple(expected.state.position))
      self.assertEqual(sample.orientation, tuple(expected.state.orientation))
      self.assertEqual(sample.twist, tuple(expected.state.twist))
    self.assertEqual(result.samples[0].seconds, 0)
    self.assertEqual(result.samples[0].nanos, 0)
    self.assertEqual(result.samples[1].seconds, 0)
    self.assertEqual(result.samples[1].nanos, 500000000)
    self.assertEqual(result.samples[2].seconds, 1)
    self.assertEqual(result.samples[2].nanos, 0)

  def test_blocked_fence_has_no_solution(self):
    propagated = _propagate(_origin(), 10.0)
    goal = propagated.samples[-1].state
    config = _config(min_x=-0.5, max_x=0.3)
    result = kd.search_kinodynamic_baseline(
        config, _request(goal), ZeroForceDynamics()
    )
    self.assert_empty(result, _Status.NO_SOLUTION)

  def test_equal_cost_tie_break_picks_lower_index(self):
    propagated = _propagate(_origin(), 10.0)
    goal = propagated.samples[-1].state
    dynamics = ZeroForceDynamics()
    first = kd.search_kinodynamic_baseline(
        _config(forces=(10.0, 10.0)), _request(goal), dynamics
    )
    second = kd.search_kinodynamic_baseline(
        _config(forces=(10.0, 10.0)), _request(goal), dynamics
    )
    self.assertIs(first.status, _Status.OK)
    self.assertEqual(first, second)
    self.assertEqual(
        first.samples[-1].position, tuple(goal.position)
    )
    self.assertEqual(first.samples[-1].twist, tuple(goal.twist))

    # Equal duration, end states both inside the goal tolerance. The lower
    # primitive index is the edge that is kept, including after reversal.
    near = _propagate(_origin(), 10.0 + 1e-6)
    self.assertLessEqual(
        vehicle_state_space.distance(near.samples[-1].state, goal), 1e-6
    )
    lower = kd.search_kinodynamic_baseline(
        _config(forces=(10.0, 10.0 + 1e-6)), _request(goal), dynamics
    )
    higher_first = kd.search_kinodynamic_baseline(
        _config(forces=(10.0 + 1e-6, 10.0)), _request(goal), dynamics
    )
    self.assertIs(lower.status, _Status.OK)
    self.assertIs(higher_first.status, _Status.OK)
    self.assertEqual(
        lower.samples[-1].position, tuple(propagated.samples[-1].state.position)
    )
    self.assertEqual(
        higher_first.samples[-1].position,
        tuple(near.samples[-1].state.position),
    )
    self.assertNotEqual(
        lower.samples[-1].position, higher_first.samples[-1].position
    )

  def test_deadline_and_cancel_through_harness(self):
    propagated = _propagate(_origin(), 10.0)
    goal = propagated.samples[-1].state
    request = _request(goal, trajectory_id="run")

    cancel = threading.Event()
    options = deadline.VehiclePlanRunOptions(cancel=cancel)
    dynamics = _CancelDynamics(cancel)
    config = dataclasses.replace(_config(), run_options=options)
    planner = kd.make_kinodynamic_baseline_planner(config, dynamics)
    cancelled = deadline.run_with_deadline(planner, request, options)
    self.assert_empty(cancelled, _Status.CANCELLED)
    self.assertGreaterEqual(dynamics.calls, 1)

    past = deadline.VehiclePlanRunOptions(
        deadline_present=True, deadline=time.monotonic() - 1.0
    )
    counting = ZeroForceDynamics()
    past_config = dataclasses.replace(_config(), run_options=past)
    past_planner = kd.make_kinodynamic_baseline_planner(past_config, counting)
    expired = past_planner.plan(request)
    self.assert_empty(expired, _Status.DEADLINE_EXCEEDED)
    self.assertEqual(counting.calls, 0)

    pre_cancel = threading.Event()
    pre_cancel.set()
    pre_options = deadline.VehiclePlanRunOptions(cancel=pre_cancel)
    pre_dynamics = ZeroForceDynamics()
    pre_config = dataclasses.replace(_config(), run_options=pre_options)
    pre_planner = kd.make_kinodynamic_baseline_planner(
        pre_config, pre_dynamics
    )
    skipped = deadline.run_with_deadline(pre_planner, request, pre_options)
    self.assert_empty(skipped, _Status.CANCELLED)
    self.assertEqual(pre_dynamics.calls, 0)

    pre_deadline = deadline.VehiclePlanRunOptions(
        deadline_present=True, deadline=time.monotonic() - 5.0
    )
    pre_deadline_dynamics = ZeroForceDynamics()
    pre_deadline_planner = kd.make_kinodynamic_baseline_planner(
        dataclasses.replace(_config(), run_options=pre_deadline),
        pre_deadline_dynamics,
    )
    skipped_deadline = deadline.run_with_deadline(
        pre_deadline_planner, request, pre_deadline
    )
    self.assert_empty(skipped_deadline, _Status.DEADLINE_EXCEEDED)
    self.assertEqual(pre_deadline_dynamics.calls, 0)

    forward_options = deadline.VehiclePlanRunOptions(
        deadline_present=True, deadline=time.monotonic() + 3600.0
    )
    blocked = deadline.run_with_deadline(
        kd.make_kinodynamic_baseline_planner(
            dataclasses.replace(
                _config(min_x=-0.5, max_x=0.3), run_options=forward_options
            ),
            ZeroForceDynamics(),
        ),
        request,
        forward_options,
    )
    self.assert_empty(blocked, _Status.NO_SOLUTION)
    reached = deadline.run_with_deadline(
        kd.make_kinodynamic_baseline_planner(
            dataclasses.replace(_config(), run_options=forward_options),
            ZeroForceDynamics(),
        ),
        request,
        forward_options,
    )
    self.assertIs(reached.status, _Status.OK)
    self.assertEqual(reached.trajectory_id, "run")
    self.assert_accepted(reached)

  def test_start_equals_goal_is_one_sample(self):
    origin = _origin()
    result = kd.search_kinodynamic_baseline(
        _config(), _request(origin), ZeroForceDynamics()
    )
    self.assertIs(result.status, _Status.OK)
    self.assertEqual(
        result.trajectory_id, kd.KINODYNAMIC_BASELINE_TRAJECTORY_ID
    )
    self.assertEqual(len(result.samples), 1)
    self.assertEqual(result.samples[0].seconds, 0)
    self.assertEqual(result.samples[0].nanos, 0)
    self.assertEqual(result.samples[0].position, (0.0, 0.0, 0.0))
    self.assertEqual(result.samples[0].twist, (0.0, 0.0, 0.0, 0.0, 0.0, 0.0))
    self.assert_accepted(result)

  def test_empty_primitives_are_invalid(self):
    config = dataclasses.replace(
        _config(),
        primitives=prims.VehicleMotionPrimitiveConfig(
            mode=prims.PrimitiveGenerationMode.CUSTOM,
            max_force_torque=_MAX,
            duration_s=1.0,
            custom_controls=(),
        ),
    )
    result = kd.search_kinodynamic_baseline(
        config, _request(_origin()), ZeroForceDynamics()
    )
    self.assert_empty(result, _Status.INVALID_REQUEST)

  def test_invalid_start_quaternion_is_invalid(self):
    start = vehicle_state_space.VehiclePlanningState(
        orientation=(0.0, 0.0, 0.0, 2.0)
    )
    result = kd.search_kinodynamic_baseline(
        _config(), _request(_origin(), start=start), ZeroForceDynamics()
    )
    self.assert_empty(result, _Status.INVALID_REQUEST)

  def test_missing_states_and_null_dynamics_are_invalid(self):
    goal = _propagate(_origin(), 10.0).samples[-1].state
    missing = registry.VehiclePlanRequest(
        start_label="start", goal_label="goal", states_present=False
    )
    self.assert_empty(
        kd.search_kinodynamic_baseline(
            _config(), missing, ZeroForceDynamics()
        ),
        _Status.INVALID_REQUEST,
    )
    planner = kd.make_kinodynamic_baseline_planner(_config(), None)
    self.assert_empty(planner.plan(_request(goal)), _Status.INVALID_REQUEST)
    # states_present wins over a null dynamics object.
    self.assert_empty(planner.plan(missing), _Status.INVALID_REQUEST)

  def test_stale_snapshot_probe_is_invalid(self):
    config = dataclasses.replace(
        _config(),
        assess_skew=True,
        timings=(
            validity.ComponentTiming(
                component_kind=_OCCUPANCY,
                observation_time=(0, 0),
                validity_horizon=(1, 0),
            ),
        ),
        query_time=(5, 0),
        skew_policy=validity.WorldSnapshotSkewPolicy(
            required_kinds=(_OCCUPANCY,)
        ),
    )
    result = kd.search_kinodynamic_baseline(
        config, _request(_origin()), ZeroForceDynamics()
    )
    self.assert_empty(result, _Status.INVALID_REQUEST)

  def test_bad_snapshot_probe_is_invalid(self):
    config = dataclasses.replace(_config(), snapshot_id="not-the-digest")
    result = kd.search_kinodynamic_baseline(
        config, _request(_origin()), ZeroForceDynamics()
    )
    self.assert_empty(result, _Status.INVALID_REQUEST)

  def test_determinism(self):
    goal = _propagate(_origin(), 10.0).samples[-1].state
    request = _request(goal, trajectory_id="same")
    config = _config()
    first = kd.search_kinodynamic_baseline(
        config, request, ZeroForceDynamics()
    )
    second = kd.search_kinodynamic_baseline(
        config, request, ZeroForceDynamics()
    )
    self.assertEqual(first, second)
    self.assertIs(first.status, _Status.OK)
    self.assertEqual(first.trajectory_id, "same")
    self.assertGreater(len(first.samples), 0)

  def test_registry_register_and_lookup(self):
    planner = kd.make_kinodynamic_baseline_planner(
        _config(), ZeroForceDynamics()
    )
    self.assertEqual(
        planner.id(), registry.VEHICLE_PLANNER_KINODYNAMIC_BASELINE
    )
    book = registry.VehiclePlannerRegistry()
    self.assertIs(book.register(planner), registry.PlannerRegistryError.OK)
    error, found = book.lookup(registry.VEHICLE_PLANNER_KINODYNAMIC_BASELINE)
    self.assertIs(error, registry.PlannerRegistryError.OK)
    self.assertIs(found, planner)
    self.assertEqual(
        found.id(), registry.VEHICLE_PLANNER_KINODYNAMIC_BASELINE
    )

  def test_fake_ignores_planning_states(self):
    request = registry.VehiclePlanRequest(
        start_label="dock",
        goal_label="sea",
        states_present=True,
        start_state=vehicle_state_space.VehiclePlanningState(
            orientation=(0.0, 0.0, 0.0, 2.0)
        ),
    )
    result = fake_vehicle_planner.FakeVehiclePlanner().plan(request)
    self.assertIs(result.status, _Status.OK)
    self.assertEqual(len(result.samples), 2)

  def test_two_edge_path_drops_duplicate_joint(self):
    first = _propagate(_origin(), 10.0)
    mid = first.samples[-1].state
    second = _propagate(mid, 10.0)
    goal = second.samples[-1].state
    result = kd.search_kinodynamic_baseline(
        _config(min_x=-2.0, max_x=5.0),
        _request(goal),
        ZeroForceDynamics(),
    )
    self.assertIs(result.status, _Status.OK)
    self.assert_accepted(result)
    self.assertEqual(len(result.samples), 5)
    expected_states = [sample.state for sample in first.samples] + [
        sample.state for sample in second.samples[1:]
    ]
    for sample, state in zip(result.samples, expected_states):
      self.assertEqual(sample.position, tuple(state.position))
      self.assertEqual(sample.twist, tuple(state.twist))
    self.assertEqual(
        [sample.seconds for sample in result.samples], [0, 0, 1, 1, 2]
    )
    self.assertEqual(
        [sample.nanos for sample in result.samples],
        [0, 500000000, 0, 500000000, 0],
    )

  def test_expansion_budget_stops_before_undequeued_goal(self):
    goal = _propagate(_origin(), 10.0).samples[-1].state
    request = _request(goal)
    exhausted = kd.search_kinodynamic_baseline(
        dataclasses.replace(_config(), max_expansions=1),
        request,
        ZeroForceDynamics(),
    )
    self.assert_empty(exhausted, _Status.NO_SOLUTION)
    reached = kd.search_kinodynamic_baseline(
        dataclasses.replace(_config(), max_expansions=2),
        request,
        ZeroForceDynamics(),
    )
    self.assertIs(reached.status, _Status.OK)


if __name__ == "__main__":
  unittest.main()
