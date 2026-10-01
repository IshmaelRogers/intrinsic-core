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

"""Deterministic fake vehicle planner.

Policy: intrinsic_motion_planning/intrinsic/motion_planning/vehicle/README.md.

Python mirror of fake_vehicle_planner.h. No sleep, threads, randomness,
World access, or StateSpace search. A success result is the fixed
two-sample world_enu fixture from the vehicle trajectory examples.
"""

import dataclasses

from intrinsic.embodiment import frame_policy
from intrinsic.motion_planning.vehicle import (
    vehicle_planner_registry as registry,
)
from intrinsic.vehicle import trajectory_contract_policy

# Id stamped when a success request leaves trajectory_id empty.
FAKE_VEHICLE_TRAJECTORY_ID = "fake-trajectory"


@dataclasses.dataclass(frozen=True)
class FakeVehiclePlannerConfig:
  """Status and the equal-label failure switch. Both are deterministic."""

  status: registry.VehiclePlanStatus = registry.VehiclePlanStatus.OK
  # When the labels are equal and this is false, plan still returns OK with
  # the two-sample fixture. When true, that case is NO_SOLUTION.
  fail_when_start_equals_goal: bool = False


def _fixture_sample(
    seconds: int, nanos: int, x: float, angular_z: float
) -> trajectory_contract_policy.TrajectorySampleView:
  """Shape of vehicle_trajectory_two_sample.textproto."""
  return trajectory_contract_policy.TrajectorySampleView(
      time_present=True,
      seconds=seconds,
      nanos=nanos,
      position=(x, 2.0, -3.0),
      orientation=(0.0, 0.0, 0.0, 1.0),
      twist_present=True,
      twist=(0.5, 0.0, 0.0, 0.0, 0.0, angular_z),
  )


def _failure(status: registry.VehiclePlanStatus) -> registry.VehiclePlanResult:
  return registry.VehiclePlanResult(status=status)


def _success(
    request: registry.VehiclePlanRequest,
) -> registry.VehiclePlanResult:
  trajectory_id = request.trajectory_id or FAKE_VEHICLE_TRAJECTORY_ID
  return registry.VehiclePlanResult(
      status=registry.VehiclePlanStatus.OK,
      header_present=True,
      validity_present=True,
      validity_state=1,
      frame_id=frame_policy.WORLD_ENU_FRAME_ID,
      trajectory_id=trajectory_id,
      samples=(
          _fixture_sample(1700000010, 0, 1.0, 0.125),
          _fixture_sample(1700000011, 500000000, 1.5, 0.0),
      ),
      provenance_present=True,
      model_id="uuv_planner",
  )


class FakeVehiclePlanner(registry.VehiclePlanner):
  """Planner registered under `ai.intrinsic.vehicle_planner.fake`."""

  def __init__(self, config: FakeVehiclePlannerConfig | None = None) -> None:
    self._config = FakeVehiclePlannerConfig() if config is None else config

  def id(self) -> str:
    return registry.VEHICLE_PLANNER_FAKE

  def plan(
      self, request: registry.VehiclePlanRequest
  ) -> registry.VehiclePlanResult:
    if request.start_label == "" or request.goal_label == "":
      return _failure(registry.VehiclePlanStatus.INVALID_REQUEST)
    if (
        self._config.fail_when_start_equals_goal
        and request.start_label == request.goal_label
    ):
      return _failure(registry.VehiclePlanStatus.NO_SOLUTION)
    if self._config.status is not registry.VehiclePlanStatus.OK:
      return _failure(self._config.status)
    return _success(request)


def make_fake_vehicle_planner(
    config: FakeVehiclePlannerConfig | None = None,
) -> FakeVehiclePlanner:
  return FakeVehiclePlanner(config)
