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

"""Deterministic discrete kinodynamic lattice Dijkstra for a UUV.

Policy: intrinsic_motion_planning/intrinsic/motion_planning/vehicle/README.md.

Python mirror of vehicle_kinodynamic_baseline.h. Uniform-cost search over
generate_uuv_motion_primitives. Edges are propagate_uuv_motion_primitive
samples accepted by check_trajectory_validity. There is no heuristic, no
sampling, no smoothing, and no new dynamics model. Dynamics are injected.
"""

from collections.abc import Sequence
import dataclasses
import heapq
import math
import struct
import time

from intrinsic.motion_planning.vehicle import vehicle_motion_primitives
from intrinsic.motion_planning.vehicle import vehicle_planner_deadline
from intrinsic.motion_planning.vehicle import (
    vehicle_planner_registry as registry,
)
from intrinsic.motion_planning.vehicle import vehicle_primitive_propagation
from intrinsic.motion_planning.vehicle import vehicle_state_space
from intrinsic.motion_planning.vehicle import vehicle_trajectory_reconstruction
from intrinsic.motion_planning.vehicle import vehicle_trajectory_validity
from intrinsic.safety import clearance_rule
from intrinsic.world.world_snapshot import world_snapshot_skew_policy

KINODYNAMIC_BASELINE_TRAJECTORY_ID = "kinodynamic-baseline"

VehicleMotionPrimitiveConfig = (
    vehicle_motion_primitives.VehicleMotionPrimitiveConfig
)
PropagationConfig = vehicle_primitive_propagation.PropagationConfig
VehiclePlanRunOptions = vehicle_planner_deadline.VehiclePlanRunOptions


@dataclasses.dataclass(frozen=True)
class KinodynamicBaselineConfig:
  """Lattice search configuration. Field meanings match the C++ config.

  `run_options` is not owned. None means no mid-run poll. Pre-call cancel
  and deadline are still handled by run_with_deadline.
  """

  primitives: VehicleMotionPrimitiveConfig = dataclasses.field(
      default_factory=VehicleMotionPrimitiveConfig
  )
  propagation: PropagationConfig = dataclasses.field(
      default_factory=PropagationConfig
  )
  snapshot_id: str = ""
  descriptor: vehicle_trajectory_validity.WorldSnapshotView = (
      dataclasses.field(
          default_factory=vehicle_trajectory_validity.WorldSnapshotView
      )
  )
  assess_skew: bool = False
  timings: Sequence[vehicle_trajectory_validity.ComponentTiming] = ()
  query_time: world_snapshot_skew_policy.TimeParts = (0, 0)
  skew_policy: vehicle_trajectory_validity.WorldSnapshotSkewPolicy = (
      dataclasses.field(
          default_factory=vehicle_trajectory_validity.WorldSnapshotSkewPolicy
      )
  )
  bounds: vehicle_state_space.VehicleStateBounds = dataclasses.field(
      default_factory=vehicle_state_space.VehicleStateBounds
  )
  fence: vehicle_trajectory_validity.AabbGeofence = (
      vehicle_trajectory_validity.AabbGeofence(
          "", "", 0.0, 0.0, 0.0, 0.0, 0.0, 0.0
      )
  )
  # Empty means fence.frame_id.
  pose_frame: str = ""
  min_clearance_m: float = clearance_rule.DEFAULT_MIN_CLEARANCE_M
  # Applied to every sample of every edge. Must be finite and >= 0.
  clearance_template_m: float = 1.0
  clearance_source: clearance_rule.ClearanceSource = (
      clearance_rule.ClearanceSource.OBSTACLE
  )
  # Distance(node, goal) <= goal_tolerance. Finite and >= 0.
  goal_tolerance: float = 1e-6
  # Position bins in meters. Orientation and twist are exact. Finite and > 0.
  position_bin_m: float = 0.25
  # Counts validity-accepted child pushes. Must be >= 1. The goal is
  # recognized when that node is dequeued.
  max_expansions: int = 256
  run_options: VehiclePlanRunOptions | None = None


@dataclasses.dataclass
class _Node:
  state: vehicle_state_space.VehiclePlanningState
  g_cost: float
  parent: int
  primitive_index: int
  parent_expand_seq: int
  node_seq: int
  edge_samples: tuple


def _failure(status: registry.VehiclePlanStatus) -> registry.VehiclePlanResult:
  return registry.VehiclePlanResult(status=status)


def _finite_non_negative(value: float) -> bool:
  return isinstance(value, (int, float)) and not isinstance(value, bool) and (
      math.isfinite(value) and value >= 0.0
  )


def _reconstructed(reconstructed) -> registry.VehiclePlanResult:
  """A defect is unreachable for a chain this search builds.

  It fails closed as NO_SOLUTION and never returns a trajectory.
  """
  reconstruction = vehicle_trajectory_reconstruction
  if reconstructed.error is not reconstruction.TrajectoryReconstructionError.OK:
    return _failure(registry.VehiclePlanStatus.NO_SOLUTION)
  return reconstructed.trajectory


def _reconstruction_options():
  return vehicle_trajectory_reconstruction.ReconstructionOptions(
      default_trajectory_id=KINODYNAMIC_BASELINE_TRAJECTORY_ID
  )


def _start_goal_success(request) -> registry.VehiclePlanResult:
  root = vehicle_trajectory_reconstruction.ReconstructionNodeView(
      state=request.start_state
  )
  return _reconstructed(
      vehicle_trajectory_reconstruction.reconstruct_vehicle_trajectory(
          (root,), 0, request, _reconstruction_options()
      )
  )


def _success_from_node(nodes: list[_Node], goal_index: int, request):
  views = tuple(
      vehicle_trajectory_reconstruction.ReconstructionNodeView(
          parent=node.parent,
          time_s=node.g_cost,
          state=node.state,
          edge_samples=node.edge_samples,
      )
      for node in nodes
  )
  return _reconstructed(
      vehicle_trajectory_reconstruction.reconstruct_vehicle_trajectory(
          views, goal_index, request, _reconstruction_options()
      )
  )


def _clearance_frame(config: KinodynamicBaselineConfig) -> str:
  if config.pose_frame:
    return config.pose_frame
  return config.fence.frame_id


def _template_clearance(config: KinodynamicBaselineConfig, count: int):
  frame = _clearance_frame(config)
  return tuple(
      clearance_rule.ClearanceSample(
          frame_id=frame,
          source=config.clearance_source,
          clearance_m=config.clearance_template_m,
          snapshot_usable=True,
      )
      for _ in range(count)
  )


def _validity_binding(config: KinodynamicBaselineConfig, samples):
  return vehicle_trajectory_validity.TrajectoryValidityRequest(
      snapshot_id=config.snapshot_id,
      descriptor=config.descriptor,
      assess_skew=config.assess_skew,
      timings=config.timings,
      query_time=config.query_time,
      skew_policy=config.skew_policy,
      samples=samples,
      bounds=config.bounds,
      fence=config.fence,
      pose_frame=config.pose_frame,
      clearance_samples=_template_clearance(config, len(samples)),
      min_clearance_m=config.min_clearance_m,
  )


def _double_bits(value: float) -> bytes:
  return struct.pack("d", float(value))


def _closed_key(state, bin_m: float):
  position = state.position
  orientation = state.orientation
  return (
      math.floor(position[0] / bin_m),
      math.floor(position[1] / bin_m),
      math.floor(position[2] / bin_m),
      _double_bits(orientation[0]),
      _double_bits(orientation[1]),
      _double_bits(orientation[2]),
      _double_bits(orientation[3]),
      *(_double_bits(component) for component in state.twist),
  )


def _poll(options: VehiclePlanRunOptions | None):
  if options is None:
    return None
  if options.cancel is not None and options.cancel.is_set():
    return registry.VehiclePlanStatus.CANCELLED
  if options.deadline_present and time.monotonic() >= options.deadline:
    return registry.VehiclePlanStatus.DEADLINE_EXCEEDED
  return None


def _push(open_heap, node: _Node, index: int) -> None:
  heapq.heappush(
      open_heap,
      (
          node.g_cost,
          node.primitive_index,
          node.parent_expand_seq,
          node.node_seq,
          index,
      ),
  )


def search_kinodynamic_baseline(
    config: KinodynamicBaselineConfig,
    request: registry.VehiclePlanRequest,
    dynamics,
) -> registry.VehiclePlanResult:
  """Uniform-cost lattice search. Does not register a planner."""
  invalid = registry.VehiclePlanStatus.INVALID_REQUEST
  if not request.states_present:
    return _failure(invalid)
  if dynamics is None:
    return _failure(invalid)
  if (
      vehicle_state_space.validate(request.start_state, config.bounds)
      is not vehicle_state_space.StateSpaceError.OK
      or vehicle_state_space.validate(request.goal_state, config.bounds)
      is not vehicle_state_space.StateSpaceError.OK
  ):
    return _failure(invalid)
  if not _finite_non_negative(config.goal_tolerance):
    return _failure(invalid)
  if (
      not isinstance(config.position_bin_m, (int, float))
      or isinstance(config.position_bin_m, bool)
      or not math.isfinite(config.position_bin_m)
      or not config.position_bin_m > 0.0
  ):
    return _failure(invalid)
  if type(config.max_expansions) is not int or config.max_expansions < 1:
    return _failure(invalid)
  if not _finite_non_negative(config.clearance_template_m):
    return _failure(invalid)

  generated = vehicle_motion_primitives.generate_uuv_motion_primitives(
      config.primitives
  )
  if (
      generated.error is not vehicle_motion_primitives.PrimitiveSetError.OK
      or not generated.primitives
  ):
    return _failure(invalid)

  probe_sample = vehicle_primitive_propagation.PropagationSample(
      time_s=0.0, state=request.start_state
  )
  probed = vehicle_trajectory_validity.check_trajectory_validity(
      _validity_binding(config, (probe_sample,))
  )
  bad = vehicle_trajectory_validity.TrajectoryValidityError
  if probed.error is bad.BAD_REQUEST or probed.error is bad.STALE_SNAPSHOT:
    return _failure(invalid)
  if (
      probed.error is bad.OK
      and vehicle_state_space.distance(request.start_state, request.goal_state)
      <= config.goal_tolerance
  ):
    return _start_goal_success(request)

  nodes: list[_Node] = [
      _Node(
          state=request.start_state,
          g_cost=0.0,
          parent=-1,
          primitive_index=-1,
          parent_expand_seq=0,
          node_seq=0,
          edge_samples=(),
      )
  ]
  open_heap: list[tuple] = []
  _push(open_heap, nodes[0], 0)
  next_node_seq = 1
  next_expand_seq = 1
  expansions = 0
  settled: dict[tuple, float] = {}

  while open_heap and expansions < config.max_expansions:
    polled = _poll(config.run_options)
    if polled is not None:
      return _failure(polled)
    _g, _primitive, _expand, _seq, index = heapq.heappop(open_heap)
    key = _closed_key(nodes[index].state, config.position_bin_m)
    if key in settled:
      continue
    settled[key] = nodes[index].g_cost
    # Start==goal with an OK probe already returned. A start node in this
    # loop failed that probe, so it is not a success.
    if (
        nodes[index].parent >= 0
        and vehicle_state_space.distance(nodes[index].state, request.goal_state)
        <= config.goal_tolerance
    ):
      return _success_from_node(nodes, index, request)

    this_expand_seq = next_expand_seq
    next_expand_seq += 1
    parent_g = nodes[index].g_cost
    parent_state = nodes[index].state
    for primitive_index, primitive in enumerate(generated.primitives):
      polled = _poll(config.run_options)
      if polled is not None:
        return _failure(polled)
      propagated = vehicle_primitive_propagation.propagate_uuv_motion_primitive(
          parent_state, primitive, config.propagation, dynamics
      )
      if (
          propagated.error
          is not vehicle_primitive_propagation.PropagationError.OK
          or not propagated.samples
      ):
        continue
      edge = vehicle_trajectory_validity.check_trajectory_validity(
          _validity_binding(config, propagated.samples)
      )
      if edge.error is not bad.OK:
        continue
      child_state = propagated.samples[-1].state
      child_key = _closed_key(child_state, config.position_bin_m)
      if child_key in settled:
        continue
      child = _Node(
          state=child_state,
          g_cost=parent_g + primitive.duration_s,
          parent=index,
          primitive_index=primitive_index,
          parent_expand_seq=this_expand_seq,
          node_seq=next_node_seq,
          edge_samples=tuple(propagated.samples),
      )
      next_node_seq += 1
      nodes.append(child)
      _push(open_heap, child, len(nodes) - 1)
      expansions += 1
  return _failure(registry.VehiclePlanStatus.NO_SOLUTION)


class KinodynamicBaselinePlanner(registry.VehiclePlanner):
  """Planner registered under the kinodynamic baseline id.

  Does not take ownership of `dynamics`. `dynamics` must outlive plan calls.
  A None dynamics object yields INVALID_REQUEST.
  """

  def __init__(self, config: KinodynamicBaselineConfig, dynamics) -> None:
    self._config = config
    self._dynamics = dynamics

  def id(self) -> str:
    return registry.VEHICLE_PLANNER_KINODYNAMIC_BASELINE

  def plan(
      self, request: registry.VehiclePlanRequest
  ) -> registry.VehiclePlanResult:
    return search_kinodynamic_baseline(self._config, request, self._dynamics)


def make_kinodynamic_baseline_planner(
    config: KinodynamicBaselineConfig, dynamics
) -> KinodynamicBaselinePlanner:
  return KinodynamicBaselinePlanner(config, dynamics)
