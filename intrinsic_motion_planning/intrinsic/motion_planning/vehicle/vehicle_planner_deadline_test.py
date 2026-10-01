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

"""Tests for the vehicle planner deadline and cancellation harness."""

import threading
import time
import unittest

from intrinsic.motion_planning.vehicle import fake_vehicle_planner as fake
from intrinsic.motion_planning.vehicle import (
    vehicle_planner_deadline as deadline,
)
from intrinsic.motion_planning.vehicle import (
    vehicle_planner_registry as registry,
)
from intrinsic.vehicle import trajectory_contract_policy

_Status = registry.VehiclePlanStatus


def _request(trajectory_id=""):
  return registry.VehiclePlanRequest(
      start_label="start", goal_label="goal", trajectory_id=trajectory_id
  )


class _CountingPlanner(registry.VehiclePlanner):

  def __init__(self, inner):
    self._inner = inner
    self.calls = 0

  def id(self):
    return "test.counting"

  def plan(self, request):
    self.calls += 1
    return self._inner.plan(request)


class _CooperativePlanner(registry.VehiclePlanner):
  """Test-only planner that polls cancel and deadline between work steps.

  Not registered under any production id. After the last step it returns
  whatever `inner` returns.
  """

  def __init__(
      self,
      inner,
      options,
      steps=3,
      on_step=None,
      now=time.monotonic,
      step_sleep=0.0,
  ):
    self._inner = inner
    self._options = options
    self._steps = steps
    self._on_step = on_step
    self._now = now
    self._step_sleep = step_sleep
    self.plan_calls = 0
    self.steps_started = 0

  def id(self):
    return "test.cooperative"

  def plan(self, request):
    self.plan_calls += 1
    for step in range(self._steps):
      if self._on_step is not None:
        self._on_step(step)
      self.steps_started += 1
      cancel = self._options.cancel
      if cancel is not None and cancel.is_set():
        return registry.VehiclePlanResult(status=_Status.CANCELLED)
      if (
          self._options.deadline_present
          and self._now() >= self._options.deadline
      ):
        return registry.VehiclePlanResult(status=_Status.DEADLINE_EXCEEDED)
      if self._step_sleep > 0:
        time.sleep(self._step_sleep)
    return self._inner.plan(request)


class VehiclePlannerDeadlineTest(unittest.TestCase):

  def assert_no_trajectory(self, result, status):
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

  def test_status_values_are_stable(self):
    self.assertEqual(_Status.OK.value, 0)
    self.assertEqual(_Status.NO_SOLUTION.value, 1)
    self.assertEqual(_Status.INVALID_REQUEST.value, 2)
    self.assertEqual(_Status.CANCELLED.value, 3)
    self.assertEqual(_Status.DEADLINE_EXCEEDED.value, 4)

  def test_already_expired_deadline(self):
    planner = _CountingPlanner(fake.make_fake_vehicle_planner())
    options = deadline.VehiclePlanRunOptions(
        deadline_present=True, deadline=time.monotonic() - 1.0
    )
    self.assert_no_trajectory(
        deadline.run_with_deadline(planner, _request(), options),
        _Status.DEADLINE_EXCEEDED,
    )
    self.assertEqual(planner.calls, 0)

    options = deadline.VehiclePlanRunOptions(
        deadline_present=True, deadline=time.monotonic()
    )
    self.assert_no_trajectory(
        deadline.run_with_deadline(planner, _request(), options),
        _Status.DEADLINE_EXCEEDED,
    )
    self.assertEqual(planner.calls, 0)

  def test_pre_cancelled_skips_planner(self):
    planner = _CountingPlanner(fake.make_fake_vehicle_planner())
    cancel = threading.Event()
    cancel.set()
    options = deadline.VehiclePlanRunOptions(cancel=cancel)
    self.assert_no_trajectory(
        deadline.run_with_deadline(planner, _request(), options),
        _Status.CANCELLED,
    )
    self.assertEqual(planner.calls, 0)

  def test_cancel_wins_over_expired_deadline(self):
    planner = _CountingPlanner(fake.make_fake_vehicle_planner())
    cancel = threading.Event()
    cancel.set()
    options = deadline.VehiclePlanRunOptions(
        deadline_present=True,
        deadline=time.monotonic() - 1.0,
        cancel=cancel,
    )
    self.assert_no_trajectory(
        deadline.run_with_deadline(planner, _request(), options),
        _Status.CANCELLED,
    )
    self.assertEqual(planner.calls, 0)

  def test_mid_run_cancel(self):
    cancel = threading.Event()
    options = deadline.VehiclePlanRunOptions(cancel=cancel)

    def on_step(step):
      if step == 2:
        cancel.set()

    planner = _CooperativePlanner(
        fake.make_fake_vehicle_planner(), options, steps=5, on_step=on_step
    )
    self.assert_no_trajectory(
        deadline.run_with_deadline(planner, _request(), options),
        _Status.CANCELLED,
    )
    self.assertEqual(planner.plan_calls, 1)
    self.assertEqual(planner.steps_started, 3)

  def test_mid_run_timeout(self):
    start = time.monotonic()
    clock = [start]
    options = deadline.VehiclePlanRunOptions(
        deadline_present=True, deadline=start + 10.0
    )

    def on_step(step):
      clock[0] = start + 4.0 * (step + 1)

    planner = _CooperativePlanner(
        fake.make_fake_vehicle_planner(),
        options,
        steps=5,
        on_step=on_step,
        now=lambda: clock[0],
    )
    self.assert_no_trajectory(
        deadline.run_with_deadline(planner, _request(), options),
        _Status.DEADLINE_EXCEEDED,
    )
    self.assertEqual(planner.plan_calls, 1)
    # Steps 0 and 1 end at +4s and +8s; step 2 reaches +12s and times out.
    self.assertEqual(planner.steps_started, 3)

  def test_completion_within_budget(self):
    options = deadline.VehiclePlanRunOptions(
        deadline_present=True,
        deadline=time.monotonic() + 3600.0,
        cancel=threading.Event(),
    )
    cooperative = _CooperativePlanner(
        fake.make_fake_vehicle_planner(), options, steps=4
    )
    for planner in (cooperative, fake.make_fake_vehicle_planner()):
      result = deadline.run_with_deadline(
          planner, _request("traj_deadline"), options
      )
      self.assertIs(result.status, _Status.OK)
      self.assertEqual(result.trajectory_id, "traj_deadline")
      self.assertEqual(result.frame_id, "world_enu")
      self.assertEqual(len(result.samples), 2)
      view = registry.as_vehicle_trajectory_view(result)
      self.assertTrue(
          trajectory_contract_policy.assess_vehicle_trajectory(view).accepted
      )
    self.assertEqual(cooperative.steps_started, 4)

  def test_default_options_have_no_bound(self):
    planner = _CountingPlanner(fake.make_fake_vehicle_planner())
    result = deadline.run_with_deadline(
        planner, _request(), deadline.VehiclePlanRunOptions()
    )
    self.assertIs(result.status, _Status.OK)
    self.assertEqual(planner.calls, 1)

  def test_forwards_no_solution_and_invalid_request(self):
    options = deadline.VehiclePlanRunOptions(
        deadline_present=True,
        deadline=time.monotonic() + 3600.0,
        cancel=threading.Event(),
    )
    config = fake.FakeVehiclePlannerConfig(status=_Status.NO_SOLUTION)
    blocked = _CountingPlanner(fake.make_fake_vehicle_planner(config))
    self.assert_no_trajectory(
        deadline.run_with_deadline(blocked, _request(), options),
        _Status.NO_SOLUTION,
    )
    self.assertEqual(blocked.calls, 1)

    cooperative = _CooperativePlanner(
        fake.make_fake_vehicle_planner(config), options, steps=2
    )
    self.assert_no_trajectory(
        deadline.run_with_deadline(cooperative, _request(), options),
        _Status.NO_SOLUTION,
    )

    self.assert_no_trajectory(
        deadline.run_with_deadline(
            fake.make_fake_vehicle_planner(),
            registry.VehiclePlanRequest(),
            options,
        ),
        _Status.INVALID_REQUEST,
    )

  def test_cancel_from_another_thread(self):
    cancel = threading.Event()
    options = deadline.VehiclePlanRunOptions(cancel=cancel)
    # Far more work than the test waits for. Cancel is what ends the run.
    planner = _CooperativePlanner(
        fake.make_fake_vehicle_planner(),
        options,
        steps=100000,
        step_sleep=0.001,
    )
    results = []
    worker = threading.Thread(
        target=lambda: results.append(
            deadline.run_with_deadline(planner, _request(), options)
        )
    )
    worker.start()
    while planner.steps_started < 2:
      time.sleep(0)
    cancel.set()
    worker.join(timeout=30.0)
    self.assertFalse(worker.is_alive())

    self.assertEqual(len(results), 1)
    self.assert_no_trajectory(results[0], _Status.CANCELLED)
    self.assertLess(planner.steps_started, 100000)

  def test_cancel_and_deadline_are_distinct(self):
    self.assertIsNot(_Status.CANCELLED, _Status.DEADLINE_EXCEEDED)
    self.assertIsNot(_Status.CANCELLED, _Status.NO_SOLUTION)
    self.assertIsNot(_Status.DEADLINE_EXCEEDED, _Status.NO_SOLUTION)


if __name__ == "__main__":
  unittest.main()
