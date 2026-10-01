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

"""Tests for vehicle trajectory reconstruction and the replan fixture."""

import dataclasses
import math
import unittest

from intrinsic.motion_planning.vehicle import vehicle_kinodynamic_baseline as kd
from intrinsic.motion_planning.vehicle import vehicle_motion_primitives as prims
from intrinsic.motion_planning.vehicle import (
    vehicle_planner_registry as registry,
)
from intrinsic.motion_planning.vehicle import (
    vehicle_primitive_propagation as prop,
)
from intrinsic.motion_planning.vehicle import vehicle_state_space
from intrinsic.motion_planning.vehicle import (
    vehicle_trajectory_reconstruction as recon,
)
from intrinsic.motion_planning.vehicle import (
    vehicle_trajectory_validity as validity,
)
from intrinsic.safety import clearance_rule
from intrinsic.vehicle import trajectory_contract_policy
from intrinsic.world.world_snapshot import world_snapshot_policy as snapshot

_Error = recon.TrajectoryReconstructionError
_Status = registry.VehiclePlanStatus
_FRAME = "world_enu"
_REGION = "ops-box"
_OCCUPANCY = snapshot.OCCUPANCY_REFERENCE_KIND
_COMPONENTS = (snapshot.ComponentRevisionView(_OCCUPANCY, 1),)
_MAX = (20.0, 20.0, 20.0, 20.0, 20.0, 20.0)
_NAN = float("nan")


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

  def evaluate(self, state, wrench, environment, dt):
    del wrench, environment
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


def _state(x=0.0, y=0.0, vx=0.0):
  return vehicle_state_space.VehiclePlanningState(
      position=(x, y, 0.0), twist=(vx, 0.0, 0.0, 0.0, 0.0, 0.0)
  )


def _sample(time_s, state):
  return prop.PropagationSample(time_s=time_s, state=state)


def _edge(start, end, times=(0.0, 0.5, 1.0)):
  """Straight edge from `start` to `end` sampled at `times`."""
  samples = []
  for time_s in times:
    u = time_s / times[-1]
    state = (
        start
        if u == 0.0
        else end
        if u == 1.0
        else _state(
            start.position[0] + u * (end.position[0] - start.position[0]),
            start.position[1] + u * (end.position[1] - start.position[1]),
        )
    )
    samples.append(_sample(time_s, state))
  return tuple(samples)


def _node(parent, time_s, state, edge_samples=()):
  return recon.ReconstructionNodeView(
      parent=parent, time_s=time_s, state=state, edge_samples=edge_samples
  )


def _request(start=None, goal=None, trajectory_id="", states_present=True):
  return registry.VehiclePlanRequest(
      start_label="start",
      goal_label="goal",
      trajectory_id=trajectory_id,
      states_present=states_present,
      start_state=start if start is not None else _state(),
      goal_state=goal if goal is not None else _state(2.0),
  )


def _chain():
  """Root (x = 0) to A (x = 1) to B (x = 2), one second per edge."""
  s0, s1, s2 = _state(0.0), _state(1.0), _state(2.0)
  return [
      _node(-1, 0.0, s0),
      _node(0, 1.0, s1, _edge(s0, s1)),
      _node(1, 2.0, s2, _edge(s1, s2)),
  ]


def _times(trajectory):
  return [(s.seconds, s.nanos) for s in trajectory.samples]


def _positions(trajectory):
  return [s.position[0] for s in trajectory.samples]


def _accepted(trajectory):
  view = registry.as_vehicle_trajectory_view(trajectory)
  return trajectory_contract_policy.assess_vehicle_trajectory(view).accepted


def _is_strictly_increasing(trajectory):
  return all(
      trajectory_contract_policy.sample_time_before(a, b)
      for a, b in zip(trajectory.samples, trajectory.samples[1:])
  )


def _snapshot_view(epoch):
  return snapshot.WorldSnapshotView(
      present=True,
      snapshot_id=snapshot.compute_snapshot_id(epoch, _COMPONENTS),
      creation_time_present=True,
      creation_time=(100, 0),
      state_epoch=epoch,
      components=_COMPONENTS,
  )


def _propagation():
  return prop.PropagationConfig(
      dt_s=0.5,
      max_steps=4,
      mass_diag=(10.0, 10.0, 10.0, 10.0, 10.0, 10.0),
  )


def _controls(*forces):
  return tuple((force, 0.0, 0.0, 0.0, 0.0, 0.0) for force in forces)


@dataclasses.dataclass(frozen=True)
class _Binding:
  """The obstacle and clearance snapshot binding one plan runs under."""

  epoch: int = 1
  clearance_m: float = 1.0
  bounds: vehicle_state_space.VehicleStateBounds = (
      vehicle_state_space.VehicleStateBounds()
  )


def _fence(max_x=2.0):
  return validity.AabbGeofence(
      _FRAME, _REGION, -2.0, -2.0, -2.0, max_x, 2.0, 2.0
  )


def _config(forces, binding, max_x=2.0):
  view = _snapshot_view(binding.epoch)
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
      bounds=binding.bounds,
      fence=_fence(max_x),
      goal_tolerance=1e-6,
      position_bin_m=0.25,
      max_expansions=64,
      clearance_template_m=binding.clearance_m,
  )


def _propagate_forces(forces):
  state = _state()
  for force in forces:
    primitive = prop.VehicleMotionPrimitive(
        id="uuv-prim-000",
        control=(force, 0.0, 0.0, 0.0, 0.0, 0.0),
        duration_s=1.0,
    )
    result = prop.propagate_uuv_motion_primitive(
        state, primitive, _propagation(), ZeroForceDynamics()
    )
    assert result.error is prop.PropagationError.OK
    state = result.samples[-1].state
  return state


def _check_under(binding, trajectory):
  """Runs CheckTrajectoryValidity on a plan's samples under `binding`."""
  view = _snapshot_view(binding.epoch)
  samples = tuple(
      prop.PropagationSample(
          time_s=s.seconds + s.nanos * 1e-9,
          state=vehicle_state_space.VehiclePlanningState(
              position=s.position, orientation=s.orientation, twist=s.twist
          ),
      )
      for s in trajectory.samples
  )
  clearance = tuple(
      clearance_rule.ClearanceSample(
          frame_id=_FRAME,
          source=clearance_rule.ClearanceSource.OBSTACLE,
          clearance_m=binding.clearance_m,
          snapshot_usable=True,
      )
      for _ in samples
  )
  return validity.check_trajectory_validity(
      validity.TrajectoryValidityRequest(
          snapshot_id=view.snapshot_id,
          descriptor=view,
          samples=samples,
          bounds=binding.bounds,
          fence=_fence(),
          clearance_samples=clearance,
      )
  ).error


class ReconstructionTest(unittest.TestCase):

  def assert_failed(self, result, error, failed_node=None):
    self.assertIs(result.error, error)
    if failed_node is not None:
      self.assertEqual(result.failed_node, failed_node)
    trajectory = result.trajectory
    self.assertIs(trajectory.status, _Status.INVALID_REQUEST)
    self.assertFalse(trajectory.header_present)
    self.assertEqual(trajectory.samples, ())
    self.assertEqual(trajectory.trajectory_id, "")
    self.assertFalse(_accepted(trajectory))

  def test_chain_starts_at_request_and_is_monotonic(self):
    nodes = _chain()
    result = recon.reconstruct_vehicle_trajectory(nodes, 2, _request())
    self.assertIs(result.error, _Error.OK)
    self.assertEqual(result.failed_node, -1)
    trajectory = result.trajectory
    self.assertIs(trajectory.status, _Status.OK)
    self.assertTrue(_accepted(trajectory))
    self.assertTrue(_is_strictly_increasing(trajectory))
    self.assertEqual(
        _times(trajectory),
        [(0, 0), (0, 500000000), (1, 0), (1, 500000000), (2, 0)],
    )
    self.assertEqual(_positions(trajectory), [0.0, 0.5, 1.0, 1.5, 2.0])
    first = trajectory.samples[0]
    self.assertEqual((first.seconds, first.nanos), (0, 0))
    self.assertEqual(first.position, _request().start_state.position)
    self.assertEqual(first.orientation, _request().start_state.orientation)
    self.assertEqual(first.twist, _request().start_state.twist)
    self.assertEqual(trajectory.frame_id, _FRAME)
    self.assertEqual(trajectory.model_id, "uuv_planner")
    self.assertEqual(trajectory.validity_state, 1)
    self.assertTrue(trajectory.header_present)
    self.assertTrue(trajectory.provenance_present)

  def test_duplicate_joint_sample_is_dropped(self):
    nodes = _chain()
    result = recon.reconstruct_vehicle_trajectory(nodes, 2, _request())
    raw = sum(len(node.edge_samples) for node in nodes)
    self.assertEqual(raw, 6)
    self.assertEqual(len(result.trajectory.samples), 5)

  def test_goal_root_is_one_sample(self):
    nodes = _chain()
    result = recon.reconstruct_vehicle_trajectory(nodes, 0, _request())
    self.assertIs(result.error, _Error.OK)
    self.assertEqual(len(result.trajectory.samples), 1)
    self.assertEqual(_times(result.trajectory), [(0, 0)])
    self.assertTrue(_accepted(result.trajectory))

  def test_partial_chain_stops_at_goal(self):
    result = recon.reconstruct_vehicle_trajectory(_chain(), 1, _request())
    self.assertIs(result.error, _Error.OK)
    self.assertEqual(_positions(result.trajectory), [0.0, 0.5, 1.0])

  def test_follows_parent_links_not_array_order(self):
    s0, s1, s2 = _state(0.0), _state(1.0), _state(2.0)
    nodes = [
        _node(2, 2.0, s2, _edge(s1, s2)),
        _node(-1, 0.0, s0),
        _node(1, 1.0, s1, _edge(s0, s1)),
    ]
    result = recon.reconstruct_vehicle_trajectory(nodes, 0, _request())
    self.assertIs(result.error, _Error.OK)
    self.assertEqual(_positions(result.trajectory), [0.0, 0.5, 1.0, 1.5, 2.0])

  def test_sibling_branch_is_ignored(self):
    nodes = _chain()
    side = _state(0.0, 1.0)
    nodes.append(_node(0, 1.0, side, _edge(nodes[0].state, side)))
    result = recon.reconstruct_vehicle_trajectory(nodes, 2, _request())
    self.assertIs(result.error, _Error.OK)
    self.assertEqual(_positions(result.trajectory), [0.0, 0.5, 1.0, 1.5, 2.0])
    branch = recon.reconstruct_vehicle_trajectory(nodes, 3, _request())
    self.assertIs(branch.error, _Error.OK)
    self.assertEqual(
        [s.position[1] for s in branch.trajectory.samples], [0.0, 0.5, 1.0]
    )

  def test_trajectory_id_precedence(self):
    nodes = _chain()
    unnamed = recon.reconstruct_vehicle_trajectory(nodes, 2, _request())
    self.assertEqual(
        unnamed.trajectory.trajectory_id, recon.RECONSTRUCTED_TRAJECTORY_ID
    )
    options = recon.ReconstructionOptions(default_trajectory_id="from-options")
    defaulted = recon.reconstruct_vehicle_trajectory(
        nodes, 2, _request(), options
    )
    self.assertEqual(defaulted.trajectory.trajectory_id, "from-options")
    named = recon.reconstruct_vehicle_trajectory(
        nodes, 2, _request(trajectory_id="from-request"), options
    )
    self.assertEqual(named.trajectory.trajectory_id, "from-request")

  def test_bad_request(self):
    nodes = _chain()
    reconstruct = recon.reconstruct_vehicle_trajectory
    self.assert_failed(
        reconstruct(nodes, 2, _request(states_present=False)),
        _Error.BAD_REQUEST,
        -1,
    )
    self.assert_failed(reconstruct([], 0, _request()), _Error.BAD_REQUEST, -1)
    self.assert_failed(reconstruct(nodes, 3, _request()), _Error.BAD_REQUEST)
    self.assert_failed(reconstruct(nodes, -1, _request()), _Error.BAD_REQUEST)

  def test_bad_chain(self):
    reconstruct = recon.reconstruct_vehicle_trajectory
    nodes = _chain()
    nodes[1] = dataclasses.replace(nodes[1], parent=7)
    self.assert_failed(reconstruct(nodes, 2, _request()), _Error.BAD_CHAIN, 1)

    nodes = _chain()
    nodes[1] = dataclasses.replace(nodes[1], parent=-2)
    self.assert_failed(reconstruct(nodes, 2, _request()), _Error.BAD_CHAIN, 1)

    nodes = _chain()
    nodes[1] = dataclasses.replace(nodes[1], parent=1)
    self.assert_failed(reconstruct(nodes, 2, _request()), _Error.BAD_CHAIN)

    nodes = _chain()
    nodes[0] = dataclasses.replace(nodes[0], parent=2)
    self.assert_failed(reconstruct(nodes, 2, _request()), _Error.BAD_CHAIN)

  def test_start_mismatch(self):
    reconstruct = recon.reconstruct_vehicle_trajectory
    nodes = _chain()
    self.assert_failed(
        reconstruct(nodes, 2, _request(start=_state(0.5))),
        _Error.START_MISMATCH,
        0,
    )
    shifted = _chain()
    shifted[0] = dataclasses.replace(shifted[0], time_s=0.25)
    self.assert_failed(
        reconstruct(shifted, 2, _request()), _Error.START_MISMATCH, 0
    )
    self.assert_failed(
        reconstruct(nodes, 0, _request(start=_state(vx=1.0))),
        _Error.START_MISMATCH,
        0,
    )

  def test_disjoint_edge(self):
    reconstruct = recon.reconstruct_vehicle_trajectory
    s0, s1 = _state(0.0), _state(1.0)

    empty = _chain()
    empty[1] = dataclasses.replace(empty[1], edge_samples=())
    self.assert_failed(
        reconstruct(empty, 2, _request()), _Error.DISJOINT_EDGE, 1
    )

    late = _chain()
    late[1] = dataclasses.replace(
        late[1], edge_samples=_edge(s0, s1, (0.25, 0.5, 1.0))
    )
    self.assert_failed(
        reconstruct(late, 2, _request()), _Error.DISJOINT_EDGE, 1
    )

    wrong_start = _chain()
    wrong_start[2] = dataclasses.replace(
        wrong_start[2], edge_samples=_edge(_state(1.5), _state(2.0))
    )
    self.assert_failed(
        reconstruct(wrong_start, 2, _request()), _Error.DISJOINT_EDGE, 2
    )

    wrong_end = _chain()
    wrong_end[1] = dataclasses.replace(
        wrong_end[1], edge_samples=_edge(s0, _state(0.75))
    )
    self.assert_failed(
        reconstruct(wrong_end, 2, _request()), _Error.DISJOINT_EDGE, 1
    )

  def test_non_monotonic_time(self):
    reconstruct = recon.reconstruct_vehicle_trajectory
    s0, s1 = _state(0.0), _state(1.0)

    def with_edge(times):
      nodes = _chain()
      nodes[1] = dataclasses.replace(
          nodes[1], edge_samples=_edge(s0, s1, times)
      )
      return nodes

    for times in ((0.0, 0.5, 0.5, 1.0), (0.0, 0.75, 0.5, 1.0)):
      self.assert_failed(
          reconstruct(with_edge(times), 2, _request()),
          _Error.NON_MONOTONIC_TIME,
          1,
      )
    for bad in (-0.5, _NAN, math.inf):
      nodes = _chain()
      nodes[1] = dataclasses.replace(
          nodes[1],
          edge_samples=(
              _sample(0.0, s0),
              _sample(bad, _state(0.5)),
              _sample(1.0, s1),
          ),
      )
      self.assert_failed(
          reconstruct(nodes, 2, _request()), _Error.NON_MONOTONIC_TIME, 1
      )

    backwards = _chain()
    backwards[2] = dataclasses.replace(backwards[2], time_s=0.5)
    self.assert_failed(
        reconstruct(backwards, 2, _request()), _Error.NON_MONOTONIC_TIME, 2
    )

    nan_time = _chain()
    nan_time[1] = dataclasses.replace(nan_time[1], time_s=_NAN)
    self.assert_failed(
        reconstruct(nan_time, 2, _request()), _Error.NON_MONOTONIC_TIME, 1
    )

  def test_sub_nanosecond_step_is_not_silently_dropped(self):
    s0, s1 = _state(0.0), _state(1.0)
    nodes = _chain()
    nodes[1] = dataclasses.replace(
        nodes[1],
        edge_samples=(
            _sample(0.0, s0),
            _sample(1e-12, _state(0.5)),
            _sample(1.0, s1),
        ),
    )
    self.assert_failed(
        recon.reconstruct_vehicle_trajectory(nodes, 2, _request()),
        _Error.NON_MONOTONIC_TIME,
        1,
    )

  def test_contract_rejection(self):
    skewed = vehicle_state_space.VehiclePlanningState(
        orientation=(0.0, 0.0, 0.0, 2.0)
    )
    nodes = [_node(-1, 0.0, skewed)]
    result = recon.reconstruct_vehicle_trajectory(
        nodes, 0, _request(start=skewed)
    )
    self.assert_failed(result, _Error.REJECTED, -1)

  def test_failure_never_leaks_partial_trajectory(self):
    nodes = _chain()
    nodes[2] = dataclasses.replace(nodes[2], time_s=_NAN)
    result = recon.reconstruct_vehicle_trajectory(nodes, 2, _request())
    self.assert_failed(result, _Error.NON_MONOTONIC_TIME, 2)

  def test_deterministic(self):
    nodes = _chain()
    first = recon.reconstruct_vehicle_trajectory(nodes, 2, _request())
    second = recon.reconstruct_vehicle_trajectory(nodes, 2, _request())
    self.assertEqual(first, second)


class SearchUsesReconstructionTest(unittest.TestCase):

  def test_search_matches_manual_chain_reconstruction(self):
    forces = (10.0,)
    binding = _Binding()
    goal = _propagate_forces((10.0, 10.0))
    request = _request(goal=goal, trajectory_id="two-edge")
    searched = kd.search_kinodynamic_baseline(
        _config(forces, binding, max_x=5.0), request, ZeroForceDynamics()
    )
    self.assertIs(searched.status, _Status.OK)

    state = _state()
    nodes = [_node(-1, 0.0, state)]
    for force in (10.0, 10.0):
      primitive = prop.VehicleMotionPrimitive(
          id="uuv-prim-000",
          control=(force, 0.0, 0.0, 0.0, 0.0, 0.0),
          duration_s=1.0,
      )
      edge = prop.propagate_uuv_motion_primitive(
          nodes[-1].state, primitive, _propagation(), ZeroForceDynamics()
      )
      nodes.append(
          _node(
              len(nodes) - 1,
              nodes[-1].time_s + 1.0,
              edge.samples[-1].state,
              edge.samples,
          )
      )
    manual = recon.reconstruct_vehicle_trajectory(nodes, 2, request)
    self.assertIs(manual.error, _Error.OK)
    self.assertEqual(searched, manual.trajectory)

  def test_search_default_id_is_baseline_id(self):
    result = kd.search_kinodynamic_baseline(
        _config((10.0,), _Binding()),
        _request(goal=_state()),
        ZeroForceDynamics(),
    )
    self.assertIs(result.status, _Status.OK)
    self.assertEqual(
        result.trajectory_id, kd.KINODYNAMIC_BASELINE_TRAJECTORY_ID
    )
    self.assertEqual(len(result.samples), 1)


class ReplanFixtureTest(unittest.TestCase):
  """Same start, goal, and dynamics. Only the snapshot binding changes."""

  _FORCES = (10.0, -10.0, 5.0, -5.0)
  _SLOW_ENVELOPE = vehicle_state_space.VehicleStateBounds(
      max_linear_speed_present=True, max_linear_speed_m_s=0.6
  )

  def plan(self, binding):
    goal = _propagate_forces((10.0, -10.0))
    request = _request(goal=goal, trajectory_id="replan")
    return kd.search_kinodynamic_baseline(
        _config(self._FORCES, binding), request, ZeroForceDynamics()
    )

  def test_first_plan_admits_the_fast_route(self):
    binding = _Binding(epoch=1)
    first = self.plan(binding)
    self.assertIs(first.status, _Status.OK)
    self.assertTrue(_accepted(first))
    self.assertTrue(_is_strictly_increasing(first))
    self.assertEqual(first.samples[0].position, (0.0, 0.0, 0.0))
    self.assertEqual(first.samples[-1].position, (1.0, 0.0, 0.0))
    self.assertEqual(len(first.samples), 5)
    self.assertEqual(
        (first.samples[-1].seconds, first.samples[-1].nanos), (2, 0)
    )
    self.assertEqual(max(s.twist[0] for s in first.samples), 1.0)
    self.assertIs(
        _check_under(binding, first), validity.TrajectoryValidityError.OK
    )

  def test_updated_snapshot_selects_a_different_safe_route(self):
    first_binding = _Binding(epoch=1)
    updated = _Binding(epoch=2, bounds=self._SLOW_ENVELOPE)
    first = self.plan(first_binding)
    second = self.plan(updated)
    self.assertIs(first.status, _Status.OK)
    self.assertIs(second.status, _Status.OK)

    self.assertNotEqual(first.samples, second.samples)
    self.assertEqual(len(second.samples), 9)
    self.assertEqual(second.samples[0], first.samples[0])
    self.assertEqual(second.samples[-1].position, first.samples[-1].position)
    self.assertEqual(second.samples[-1].twist, first.samples[-1].twist)
    self.assertEqual(
        (second.samples[-1].seconds, second.samples[-1].nanos), (4, 0)
    )
    self.assertLessEqual(max(s.twist[0] for s in second.samples), 0.5)

    self.assertTrue(_accepted(second))
    self.assertTrue(_is_strictly_increasing(second))
    self.assertIs(
        _check_under(updated, second), validity.TrajectoryValidityError.OK
    )
    self.assertIs(
        _check_under(updated, first), validity.TrajectoryValidityError.BOUNDS
    )

  def test_updated_obstacle_is_typed_no_solution(self):
    first_binding = _Binding(epoch=1)
    blocked = _Binding(epoch=2, clearance_m=0.2)
    first = self.plan(first_binding)
    self.assertIs(first.status, _Status.OK)
    second = self.plan(blocked)
    self.assertIs(second.status, _Status.NO_SOLUTION)
    self.assertFalse(second.header_present)
    self.assertEqual(second.samples, ())
    self.assertFalse(_accepted(second))
    self.assertIs(
        _check_under(blocked, first),
        validity.TrajectoryValidityError.CLEARANCE,
    )

  def test_replan_is_deterministic(self):
    for binding in (
        _Binding(epoch=1),
        _Binding(epoch=2, bounds=self._SLOW_ENVELOPE),
        _Binding(epoch=2, clearance_m=0.2),
    ):
      self.assertEqual(self.plan(binding), self.plan(binding))

  def test_snapshot_id_binds_each_plan(self):
    first = _config(self._FORCES, _Binding(epoch=1))
    updated = _config(self._FORCES, _Binding(epoch=2))
    self.assertNotEqual(first.snapshot_id, updated.snapshot_id)
    stale = dataclasses.replace(updated, snapshot_id=first.snapshot_id)
    goal = _propagate_forces((10.0, -10.0))
    result = kd.search_kinodynamic_baseline(
        stale, _request(goal=goal), ZeroForceDynamics()
    )
    self.assertIs(result.status, _Status.INVALID_REQUEST)


if __name__ == "__main__":
  unittest.main()
