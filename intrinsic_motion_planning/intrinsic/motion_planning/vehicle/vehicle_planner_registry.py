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

"""Exact-match registry of vehicle planner implementations.

Policy: intrinsic_motion_planning/intrinsic/motion_planning/vehicle/README.md.

Python mirror of vehicle_planner_registry.h. This is not the embodiment
category id ai.intrinsic.capability.planning. There is no search, no
default planner, and no gRPC service.
"""

import abc
import dataclasses
import enum

from intrinsic.motion_planning.vehicle import vehicle_state_space
from intrinsic.vehicle import trajectory_contract_policy

# Well-known vehicle planner implementation ids (not the embodiment
# category id).
VEHICLE_PLANNER_FAKE = "ai.intrinsic.vehicle_planner.fake"
# Discrete kinodynamic lattice Dijkstra. Implemented by
# KinodynamicBaselinePlanner.
VEHICLE_PLANNER_KINODYNAMIC_BASELINE = (
    "ai.intrinsic.vehicle_planner.kinodynamic_baseline"
)


class PlannerRegistryError(enum.Enum):
  OK = 0
  EMPTY_ID = 1
  DUPLICATE_ID = 2
  NOT_FOUND = 3
  NULL_PLANNER = 4


class VehiclePlanStatus(enum.Enum):
  OK = 0
  NO_SOLUTION = 1
  INVALID_REQUEST = 2
  CANCELLED = 3
  DEADLINE_EXCEEDED = 4


@dataclasses.dataclass(frozen=True)
class VehiclePlanRequest:
  """Start and goal labels, plus optional planning states.

  The fake requires both labels and ignores the planning-state fields.
  The kinodynamic baseline requires `states_present`.
  """

  start_label: str = ""
  goal_label: str = ""
  # Empty means the planner chooses its own id. The fake uses
  # "fake-trajectory". The kinodynamic baseline uses "kinodynamic-baseline".
  trajectory_id: str = ""
  # Additive planning state. Default false. The fake ignores these fields.
  states_present: bool = False
  start_state: vehicle_state_space.VehiclePlanningState = dataclasses.field(
      default_factory=vehicle_state_space.VehiclePlanningState
  )
  goal_state: vehicle_state_space.VehiclePlanningState = dataclasses.field(
      default_factory=vehicle_state_space.VehiclePlanningState
  )


@dataclasses.dataclass(frozen=True)
class VehiclePlanResult:
  """Owned plan. A trajectory is present only when status is OK."""

  status: VehiclePlanStatus = VehiclePlanStatus.INVALID_REQUEST
  header_present: bool = False
  validity_present: bool = False
  validity_state: int = 0
  frame_id: str = ""
  trajectory_id: str = ""
  samples: tuple[trajectory_contract_policy.TrajectorySampleView, ...] = ()
  provenance_present: bool = False
  model_id: str = ""


def as_vehicle_trajectory_view(
    result: VehiclePlanResult,
) -> trajectory_contract_policy.VehicleTrajectoryView:
  """View of `result`. A non-ok status yields an empty, unengaged view."""
  if result.status is not VehiclePlanStatus.OK:
    return trajectory_contract_policy.VehicleTrajectoryView()
  return trajectory_contract_policy.VehicleTrajectoryView(
      header_present=result.header_present,
      validity_present=result.validity_present,
      validity_state=result.validity_state,
      frame_id=result.frame_id,
      trajectory_id=result.trajectory_id,
      samples=result.samples,
      provenance_present=result.provenance_present,
      model_id=result.model_id,
  )


class VehiclePlanner(abc.ABC):
  """Planner implementation selected by an exact id."""

  @abc.abstractmethod
  def id(self) -> str:
    """Exact planner implementation id."""

  @abc.abstractmethod
  def plan(self, request: VehiclePlanRequest) -> VehiclePlanResult:
    """Returns a plan. Does not search."""


class VehiclePlannerRegistry:
  """Exact map of implementations. Does not construct a planner on lookup."""

  def __init__(self) -> None:
    self._planners: dict[str, VehiclePlanner] = {}

  def register(self, planner: VehiclePlanner | None) -> PlannerRegistryError:
    """First defect wins: null, empty id, duplicate id, then store.

    A duplicate leaves the first registration in place.
    """
    if planner is None:
      return PlannerRegistryError.NULL_PLANNER
    planner_id = planner.id()
    if planner_id == "":
      return PlannerRegistryError.EMPTY_ID
    if planner_id in self._planners:
      return PlannerRegistryError.DUPLICATE_ID
    self._planners[planner_id] = planner
    return PlannerRegistryError.OK

  def lookup(
      self, planner_id: str
  ) -> tuple[PlannerRegistryError, VehiclePlanner | None]:
    """Returns `(error, planner)`. `planner` is set only on OK."""
    if planner_id == "":
      return PlannerRegistryError.EMPTY_ID, None
    planner = self._planners.get(planner_id)
    if planner is None:
      return PlannerRegistryError.NOT_FOUND, None
    return PlannerRegistryError.OK, planner

  def size(self) -> int:
    return len(self._planners)
