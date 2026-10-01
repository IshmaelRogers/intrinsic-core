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

"""Reconstruct a vehicle trajectory from a parent-linked lattice chain.

Policy: intrinsic_motion_planning/intrinsic/motion_planning/vehicle/README.md.

Python mirror of vehicle_trajectory_reconstruction.h. This is the
concatenation step of search_kinodynamic_baseline, extracted unchanged. It
does not search, expand, smooth, retime, or read World, and it never edits a
sample: a chain it cannot reconstruct as given is rejected, not repaired.
"""

from collections.abc import Sequence
import dataclasses
import enum
import math

from intrinsic.embodiment import frame_policy
from intrinsic.motion_planning.vehicle import (
    vehicle_planner_registry as registry,
)
from intrinsic.motion_planning.vehicle import vehicle_primitive_propagation
from intrinsic.motion_planning.vehicle import vehicle_state_space
from intrinsic.vehicle import trajectory_contract_policy

# Id stamped when neither the request nor the options name a trajectory.
RECONSTRUCTED_TRAJECTORY_ID = "reconstructed-trajectory"
_MODEL_ID = "uuv_planner"
_NANOS_PER_SECOND = 1_000_000_000


class TrajectoryReconstructionError(enum.Enum):
  """First defect wins, root first and, within one node, in this order."""

  OK = 0
  # states_present is false, the node array is empty, or goal_index is out of
  # range.
  BAD_REQUEST = 1
  # A parent index is out of range, the chain loops, or it does not reach a
  # root (parent == -1).
  BAD_CHAIN = 2
  # The root does not hold the request start state exactly at time 0.
  START_MISMATCH = 3
  # A non-root node has no edge samples, or its edge does not begin at the
  # parent state at relative t = 0, or does not end at the node state.
  DISJOINT_EDGE = 4
  # A time is non-finite or negative, edge times are not strictly increasing,
  # a node time precedes its parent, or the concatenated trajectory is not
  # strictly increasing at nanosecond resolution.
  NON_MONOTONIC_TIME = 5
  # assess_vehicle_trajectory did not accept the reconstructed trajectory.
  REJECTED = 6


@dataclasses.dataclass(frozen=True)
class ReconstructionNodeView:
  """One search node. Mirrors the C++ ReconstructionNodeView."""

  # Index of the parent in the node sequence. -1 marks the root.
  parent: int = -1
  # Absolute time of this node in seconds (the search's cumulative cost). The
  # root must be exactly 0.
  time_s: float = 0.0
  # State at the end of the edge. The root holds the request start state.
  state: vehicle_state_space.VehiclePlanningState = dataclasses.field(
      default_factory=vehicle_state_space.VehiclePlanningState
  )
  # Edge from the parent: PropagationSample times are relative to the parent,
  # the first sample is the parent state at t = 0, and the last sample is
  # `state`. Ignored for the root.
  edge_samples: Sequence[vehicle_primitive_propagation.PropagationSample] = ()


@dataclasses.dataclass(frozen=True)
class ReconstructionOptions:
  # Used when the request leaves trajectory_id empty. Empty means
  # RECONSTRUCTED_TRAJECTORY_ID.
  default_trajectory_id: str = ""


@dataclasses.dataclass(frozen=True)
class TrajectoryReconstructionResult:
  error: TrajectoryReconstructionError = (
      TrajectoryReconstructionError.BAD_REQUEST
  )
  # Index of the node that carries the defect, or -1 when the defect is not
  # tied to one node (and for OK).
  failed_node: int = -1
  # status OK and a full trajectory only when error is OK. Otherwise status
  # is INVALID_REQUEST and no trajectory is carried.
  trajectory: registry.VehiclePlanResult = dataclasses.field(
      default_factory=registry.VehiclePlanResult
  )


def _failure(
    error: TrajectoryReconstructionError, failed_node: int = -1
) -> TrajectoryReconstructionResult:
  return TrajectoryReconstructionResult(error=error, failed_node=failed_node)


def _same_state(a, b) -> bool:
  return (
      tuple(a.position) == tuple(b.position)
      and tuple(a.orientation) == tuple(b.orientation)
      and tuple(a.twist) == tuple(b.twist)
  )


def _finite_non_negative(value: float) -> bool:
  return math.isfinite(value) and value >= 0.0


def _llround(value: float) -> int:
  """Half away from zero, matching C++ llround."""
  if value >= 0.0:
    return int(math.floor(value + 0.5))
  return int(math.ceil(value - 0.5))


def _split_time(time_s: float) -> tuple[int, int]:
  whole = math.floor(time_s)
  nanos = _llround((time_s - whole) * _NANOS_PER_SECOND)
  seconds = int(whole)
  if nanos >= _NANOS_PER_SECOND:
    nanos -= _NANOS_PER_SECOND
    seconds += 1
  if nanos < 0:
    nanos = 0
  return seconds, nanos


def _to_sample(state, time_s: float):
  seconds, nanos = _split_time(time_s)
  return trajectory_contract_policy.TrajectorySampleView(
      time_present=True,
      seconds=seconds,
      nanos=nanos,
      position=tuple(state.position),
      orientation=tuple(state.orientation),
      twist_present=True,
      twist=tuple(state.twist),
      acceleration_present=False,
  )


def _trajectory_id(request, options: ReconstructionOptions) -> str:
  return (
      request.trajectory_id
      or options.default_trajectory_id
      or RECONSTRUCTED_TRAJECTORY_ID
  )


def _walk_to_root(nodes, goal_index: int):
  """Returns (chain root first, failed_node). The chain is empty on error."""
  chain: list[int] = []
  index = goal_index
  while True:
    if len(chain) >= len(nodes):
      return [], index
    chain.append(index)
    parent = nodes[index].parent
    if parent == -1:
      break
    if type(parent) is not int or parent < 0 or parent >= len(nodes):
      return [], index
    index = parent
  chain.reverse()
  return chain, -1


def _check_edge(parent, node) -> TrajectoryReconstructionError:
  edge = node.edge_samples
  if (
      not edge
      or edge[0].time_s != 0.0
      or not _same_state(edge[0].state, parent.state)
      or not _same_state(edge[-1].state, node.state)
  ):
    return TrajectoryReconstructionError.DISJOINT_EDGE
  if not _finite_non_negative(node.time_s) or node.time_s < parent.time_s:
    return TrajectoryReconstructionError.NON_MONOTONIC_TIME
  for i, sample in enumerate(edge):
    if not _finite_non_negative(sample.time_s) or (
        i > 0 and not edge[i - 1].time_s < sample.time_s
    ):
      return TrajectoryReconstructionError.NON_MONOTONIC_TIME
  return TrajectoryReconstructionError.OK


def reconstruct_vehicle_trajectory(
    nodes: Sequence[ReconstructionNodeView],
    goal_index: int,
    request: registry.VehiclePlanRequest,
    options: ReconstructionOptions = ReconstructionOptions(),
) -> TrajectoryReconstructionResult:
  """Walks parent links from goal_index to the root and concatenates edges.

  Each edge's first sample (the parent state at t = 0) repeats the previous
  sample and is dropped. Every other sample is kept at absolute time
  parent.time_s + sample.time_s, split into seconds and nanos. A chain whose
  goal is the root yields one sample at t = 0.

  The result has frame world_enu, model id uuv_planner, validity state 1, and
  a trajectory_id from the request or the options. Its first sample is the
  request start state at t = 0, times are strictly increasing, and
  assess_vehicle_trajectory accepts it.
  """
  error = TrajectoryReconstructionError
  if (
      not request.states_present
      or not nodes
      or type(goal_index) is not int
      or goal_index < 0
      or goal_index >= len(nodes)
  ):
    return _failure(error.BAD_REQUEST)
  chain, failed_node = _walk_to_root(nodes, goal_index)
  if not chain:
    return _failure(error.BAD_CHAIN, failed_node)

  root = nodes[chain[0]]
  if root.time_s != 0.0 or not _same_state(root.state, request.start_state):
    return _failure(error.START_MISMATCH, chain[0])

  samples = [_to_sample(request.start_state, 0.0)]
  for link in range(1, len(chain)):
    parent = nodes[chain[link - 1]]
    node = nodes[chain[link]]
    edge_error = _check_edge(parent, node)
    if edge_error is not error.OK:
      return _failure(edge_error, chain[link])
    # Sample 0 repeats the parent state at the parent time and is dropped.
    for sample in node.edge_samples[1:]:
      view = _to_sample(sample.state, parent.time_s + sample.time_s)
      if not trajectory_contract_policy.sample_time_before(samples[-1], view):
        return _failure(error.NON_MONOTONIC_TIME, chain[link])
      samples.append(view)

  trajectory = registry.VehiclePlanResult(
      status=registry.VehiclePlanStatus.OK,
      header_present=True,
      validity_present=True,
      validity_state=1,
      frame_id=frame_policy.WORLD_ENU_FRAME_ID,
      trajectory_id=_trajectory_id(request, options),
      samples=tuple(samples),
      provenance_present=True,
      model_id=_MODEL_ID,
  )
  assessment = trajectory_contract_policy.assess_vehicle_trajectory(
      registry.as_vehicle_trajectory_view(trajectory)
  )
  if not assessment.accepted:
    return _failure(error.REJECTED)
  return TrajectoryReconstructionResult(
      error=error.OK, failed_node=-1, trajectory=trajectory
  )
