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

"""Deadline and cooperative cancellation around a vehicle planner.

Policy: intrinsic_motion_planning/intrinsic/motion_planning/vehicle/README.md.

Python mirror of vehicle_planner_deadline.h. This is a wrapper only: no
search, no sampling, no threads.
"""

import dataclasses
import threading
import time

from intrinsic.motion_planning.vehicle import (
    vehicle_planner_registry as registry,
)


@dataclasses.dataclass(frozen=True)
class VehiclePlanRunOptions:
  """Deadline and cancel policy for one `run_with_deadline` call."""

  # Absolute `time.monotonic()` deadline in seconds. If it has already passed
  # before plan starts, the result is DEADLINE_EXCEEDED and the inner planner
  # is not called. When `deadline_present` is false there is no time bound
  # (cancel still applies).
  deadline_present: bool = False
  deadline: float = 0.0
  # Cooperative cancel flag. None means the run is not cancelable. A flag
  # that is already set before plan starts yields CANCELLED without calling
  # the inner planner.
  cancel: threading.Event | None = None


def run_with_deadline(
    planner: registry.VehiclePlanner,
    request: registry.VehiclePlanRequest,
    options: VehiclePlanRunOptions,
) -> registry.VehiclePlanResult:
  """Runs `planner.plan(request)` under `options`. First applicable wins.

  1. cancel set before start      -> CANCELLED, inner plan not called.
  2. deadline passed before start -> DEADLINE_EXCEEDED, inner plan not called.
  3. otherwise the inner plan result is returned unchanged, including
     non-ok statuses such as NO_SOLUTION and INVALID_REQUEST.

  A planner that polls the same cancel flag and deadline while it works can
  itself return CANCELLED or DEADLINE_EXCEEDED; those are forwarded like any
  other status. A cancelled or expired result never carries a trajectory.

  The call is synchronous on the calling thread. It does not spin or sleep
  and creates no thread.
  """
  if options.cancel is not None and options.cancel.is_set():
    return registry.VehiclePlanResult(
        status=registry.VehiclePlanStatus.CANCELLED
    )
  if options.deadline_present and time.monotonic() >= options.deadline:
    return registry.VehiclePlanResult(
        status=registry.VehiclePlanStatus.DEADLINE_EXCEEDED
    )
  return planner.plan(request)
