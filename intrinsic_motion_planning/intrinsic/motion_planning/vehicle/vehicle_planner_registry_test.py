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

"""Tests for the vehicle planner registry and deterministic fake."""

import unittest

from intrinsic.motion_planning.vehicle import fake_vehicle_planner as fake
from intrinsic.motion_planning.vehicle import (
    vehicle_planner_registry as registry,
)
from intrinsic.vehicle import trajectory_contract_policy


class _NamedPlanner(registry.VehiclePlanner):

  def __init__(self, planner_id, status):
    self._planner_id = planner_id
    self._status = status

  def id(self):
    return self._planner_id

  def plan(self, request):
    del request  # Unused. This double only carries an id and a status.
    return registry.VehiclePlanResult(status=self._status)


def _request(start, goal, trajectory_id=""):
  return registry.VehiclePlanRequest(
      start_label=start, goal_label=goal, trajectory_id=trajectory_id
  )


class PlannerTestCase(unittest.TestCase):

  def assert_fixture(self, result, trajectory_id):
    self.assertIs(result.status, registry.VehiclePlanStatus.OK)
    self.assertTrue(result.header_present)
    self.assertTrue(result.validity_present)
    self.assertEqual(result.validity_state, 1)
    self.assertEqual(result.frame_id, "world_enu")
    self.assertEqual(result.trajectory_id, trajectory_id)
    self.assertEqual(len(result.samples), 2)

    first, second = result.samples
    self.assertTrue(first.time_present)
    self.assertEqual(first.seconds, 1700000010)
    self.assertEqual(first.nanos, 0)
    self.assertEqual(first.position, (1.0, 2.0, -3.0))
    self.assertEqual(first.orientation, (0.0, 0.0, 0.0, 1.0))
    self.assertTrue(first.twist_present)
    self.assertEqual(first.twist, (0.5, 0.0, 0.0, 0.0, 0.0, 0.125))

    self.assertTrue(second.time_present)
    self.assertEqual(second.seconds, 1700000011)
    self.assertEqual(second.nanos, 500000000)
    self.assertEqual(second.position, (1.5, 2.0, -3.0))
    self.assertEqual(second.orientation, (0.0, 0.0, 0.0, 1.0))
    self.assertTrue(second.twist_present)
    self.assertEqual(second.twist, (0.5, 0.0, 0.0, 0.0, 0.0, 0.0))

    self.assertTrue(result.provenance_present)
    self.assertEqual(result.model_id, "uuv_planner")
    view = registry.as_vehicle_trajectory_view(result)
    assessment = trajectory_contract_policy.assess_vehicle_trajectory(view)
    self.assertIs(
        assessment.error,
        trajectory_contract_policy.TrajectoryContractError.NONE,
    )
    self.assertTrue(assessment.accepted)

  def assert_absent(self, result, status):
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


class VehiclePlannerRegistryTest(PlannerTestCase):

  def test_register_lookup_plan_success(self):
    planners = registry.VehiclePlannerRegistry()
    planner = fake.make_fake_vehicle_planner()
    self.assertEqual(planner.id(), registry.VEHICLE_PLANNER_FAKE)
    self.assertIs(planners.register(planner), registry.PlannerRegistryError.OK)
    self.assertEqual(planners.size(), 1)
    error, found = planners.lookup(registry.VEHICLE_PLANNER_FAKE)
    self.assertIs(error, registry.PlannerRegistryError.OK)
    self.assertIs(found, planner)
    self.assert_fixture(
        found.plan(_request("start", "goal")), "fake-trajectory"
    )

  def test_duplicate_id_keeps_first(self):
    planners = registry.VehiclePlannerRegistry()
    first = fake.make_fake_vehicle_planner()
    second = fake.make_fake_vehicle_planner(
        fake.FakeVehiclePlannerConfig(
            status=registry.VehiclePlanStatus.NO_SOLUTION
        )
    )
    self.assertIs(planners.register(first), registry.PlannerRegistryError.OK)
    self.assertIs(
        planners.register(second), registry.PlannerRegistryError.DUPLICATE_ID
    )
    self.assertEqual(planners.size(), 1)
    error, found = planners.lookup(registry.VEHICLE_PLANNER_FAKE)
    self.assertIs(error, registry.PlannerRegistryError.OK)
    self.assertIs(found, first)
    self.assert_fixture(
        found.plan(_request("start", "goal")), "fake-trajectory"
    )

  def test_unknown_id_is_not_found(self):
    planners = registry.VehiclePlannerRegistry()
    self.assertIs(
        planners.register(fake.make_fake_vehicle_planner()),
        registry.PlannerRegistryError.OK,
    )
    error, found = planners.lookup(
        "ai.intrinsic.vehicle_planner.kinodynamic_baseline"
    )
    self.assertIs(error, registry.PlannerRegistryError.NOT_FOUND)
    self.assertIsNone(found)
    error, found = planners.lookup("ai.intrinsic.capability.planning")
    self.assertIs(error, registry.PlannerRegistryError.NOT_FOUND)
    self.assertIsNone(found)
    self.assertEqual(planners.size(), 1)

    empty = registry.VehiclePlannerRegistry()
    error, found = empty.lookup(registry.VEHICLE_PLANNER_FAKE)
    self.assertIs(error, registry.PlannerRegistryError.NOT_FOUND)
    self.assertIsNone(found)
    self.assertEqual(empty.size(), 0)

  def test_empty_id_and_null_planner(self):
    planners = registry.VehiclePlannerRegistry()
    empty = _NamedPlanner("", registry.VehiclePlanStatus.OK)
    self.assertIs(
        planners.register(empty), registry.PlannerRegistryError.EMPTY_ID
    )
    self.assertIs(
        planners.register(None), registry.PlannerRegistryError.NULL_PLANNER
    )
    self.assertEqual(planners.size(), 0)
    error, found = planners.lookup("")
    self.assertIs(error, registry.PlannerRegistryError.EMPTY_ID)
    self.assertIsNone(found)

    self.assertIs(
        planners.register(fake.make_fake_vehicle_planner()),
        registry.PlannerRegistryError.OK,
    )
    self.assertEqual(planners.size(), 1)
    error, found = planners.lookup("")
    self.assertIs(error, registry.PlannerRegistryError.EMPTY_ID)
    self.assertIsNone(found)


class FakeVehiclePlannerTest(PlannerTestCase):

  def test_configured_failure_and_invalid_labels(self):
    blocked = fake.FakeVehiclePlanner(
        fake.FakeVehiclePlannerConfig(
            status=registry.VehiclePlanStatus.NO_SOLUTION
        )
    )
    self.assert_absent(
        blocked.plan(_request("start", "goal")),
        registry.VehiclePlanStatus.NO_SOLUTION,
    )

    configured_invalid = fake.FakeVehiclePlanner(
        fake.FakeVehiclePlannerConfig(
            status=registry.VehiclePlanStatus.INVALID_REQUEST
        )
    )
    self.assert_absent(
        configured_invalid.plan(_request("start", "goal")),
        registry.VehiclePlanStatus.INVALID_REQUEST,
    )

    ok = fake.FakeVehiclePlanner()
    self.assert_absent(
        ok.plan(_request("", "goal")),
        registry.VehiclePlanStatus.INVALID_REQUEST,
    )
    self.assert_absent(
        ok.plan(_request("start", "")),
        registry.VehiclePlanStatus.INVALID_REQUEST,
    )
    self.assert_absent(
        ok.plan(_request("", "")), registry.VehiclePlanStatus.INVALID_REQUEST
    )

    # Empty labels win over the equal-label failure switch.
    strict = fake.FakeVehiclePlanner(
        fake.FakeVehiclePlannerConfig(fail_when_start_equals_goal=True)
    )
    self.assert_absent(
        strict.plan(_request("", "")),
        registry.VehiclePlanStatus.INVALID_REQUEST,
    )

  def test_start_equals_goal(self):
    ok = fake.FakeVehiclePlanner()
    self.assert_fixture(ok.plan(_request("dock", "dock")), "fake-trajectory")

    strict = fake.FakeVehiclePlanner(
        fake.FakeVehiclePlannerConfig(fail_when_start_equals_goal=True)
    )
    self.assert_absent(
        strict.plan(_request("dock", "dock")),
        registry.VehiclePlanStatus.NO_SOLUTION,
    )
    self.assert_fixture(strict.plan(_request("dock", "sea")), "fake-trajectory")

  def test_deterministic_success_fixture(self):
    first = fake.FakeVehiclePlanner()
    second = fake.FakeVehiclePlanner()
    request = _request("start", "goal", "traj_alpha")
    a = first.plan(request)
    b = first.plan(request)
    c = second.plan(request)
    self.assert_fixture(a, "traj_alpha")
    self.assertEqual(a, b)
    self.assertEqual(a, c)
    self.assertEqual(fake.FAKE_VEHICLE_TRAJECTORY_ID, "fake-trajectory")


if __name__ == "__main__":
  unittest.main()
