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

"""Plain-value checks for thruster array command and feedback.

Policy: intrinsic_hardware/intrinsic/hardware/marine/README.md.

These helpers do not parse protobuf, do not call ICON, do not allocate
thrust, do not integrate dynamics, and do not rewrite thruster health into
embodiment Validity.

Health integers match ThrusterHealthState: NOMINAL=0, DISABLED=1,
DERATED=2, STUCK_OFF=3, FAILED=4. Unset health is absent. A present number
outside 0..4 stays unrecognized.

`frame_id` must be the body frame. An empty id and any other id are
defects. The message type does not supply a second body frame.
"""

from dataclasses import dataclass
from dataclasses import field
import enum
import math

from intrinsic.embodiment import stamped_header_policy
from intrinsic.vehicle import vehicle_contract_policy


class ThrusterHealthKind(enum.Enum):
  ABSENT = 0
  NOMINAL = 1
  DISABLED = 2
  DERATED = 3
  STUCK_OFF = 4
  FAILED = 5
  UNRECOGNIZED = 6


class ThrusterArrayError(enum.Enum):
  NONE = 0
  MISSING_HEADER = 1
  MISSING_FRAME = 2
  WRONG_FRAME = 3
  TIME_REVERSAL = 4
  EMPTY_ARRAY = 5
  EMPTY_NAME = 6
  MISSING_THRUST = 7
  NON_FINITE = 8
  HEALTH = 9
  HEALTH_DERATE = 10
  EFFICIENCY = 11


@dataclass(frozen=True)
class ThrusterArrayAssessment:
  error: ThrusterArrayError
  header_validity: stamped_header_policy.ValidityKind
  accepted: bool


@dataclass(frozen=True)
class ThrusterHeaderView:
  header_present: bool = False
  frame_id: str = ""
  source_time_present: bool = False
  source_time: stamped_header_policy.ClockReading = (0, 0)
  receive_time_present: bool = False
  receive_time: stamped_header_policy.ClockReading = (0, 0)
  header_validity_present: bool = False
  header_validity_state: int = 0


@dataclass(frozen=True)
class ThrusterCommandElementView:
  name_present: bool = False
  name: str = ""
  thrust_present: bool = False
  thrust_n: float = 0.0
  enable_present: bool = False
  enable: bool = False


@dataclass(frozen=True)
class ThrusterFeedbackElementView:
  name_present: bool = False
  name: str = ""
  commanded_thrust_present: bool = False
  commanded_thrust_n: float = 0.0
  measured_thrust_present: bool = False
  measured_thrust_n: float = 0.0
  saturated_present: bool = False
  saturated: bool = False
  health_present: bool = False
  health: int = 0
  health_derate_present: bool = False
  health_derate: float = 0.0
  efficiency_present: bool = False
  efficiency: float = 0.0


@dataclass(frozen=True)
class ThrusterArrayCommandView:
  header: ThrusterHeaderView = field(default_factory=ThrusterHeaderView)
  thrusters: tuple[ThrusterCommandElementView, ...] = ()


@dataclass(frozen=True)
class ThrusterArrayFeedbackView:
  header: ThrusterHeaderView = field(default_factory=ThrusterHeaderView)
  thrusters: tuple[ThrusterFeedbackElementView, ...] = ()


def classify_thruster_health(
    field_present: bool, health: int
) -> ThrusterHealthKind:
  """field_present is the optional health field, not the enum value."""
  if not field_present:
    return ThrusterHealthKind.ABSENT
  return {
      0: ThrusterHealthKind.NOMINAL,
      1: ThrusterHealthKind.DISABLED,
      2: ThrusterHealthKind.DERATED,
      3: ThrusterHealthKind.STUCK_OFF,
      4: ThrusterHealthKind.FAILED,
  }.get(health, ThrusterHealthKind.UNRECOGNIZED)


def thruster_command_enabled(element: ThrusterCommandElementView) -> bool:
  """Unset enable is enabled. Explicit false is the neutralize path."""
  return (not element.enable_present) or element.enable


def thruster_array_command_engaged(sample: ThrusterArrayCommandView) -> bool:
  return sample.header.header_present or bool(sample.thrusters)


def thruster_array_feedback_engaged(sample: ThrusterArrayFeedbackView) -> bool:
  return sample.header.header_present or bool(sample.thrusters)


def _timestamp_precedes(
    lhs: stamped_header_policy.ClockReading,
    rhs: stamped_header_policy.ClockReading,
) -> bool:
  if lhs[0] != rhs[0]:
    return lhs[0] < rhs[0]
  return lhs[1] < rhs[1]


def thruster_derate_matches_health(
    kind: ThrusterHealthKind, derate: float
) -> bool:
  """NOMINAL → 1, DERATED → (0, 1), neutral faults → 0. Exact compare."""
  if kind is ThrusterHealthKind.NOMINAL:
    return derate == 1.0
  if kind is ThrusterHealthKind.DERATED:
    return derate > 0.0 and derate < 1.0
  if kind in (
      ThrusterHealthKind.DISABLED,
      ThrusterHealthKind.STUCK_OFF,
      ThrusterHealthKind.FAILED,
  ):
    return derate == 0.0
  return False


def thruster_efficiency_in_range(efficiency: float) -> bool:
  """Dimensionless (0, 1]. Caller has already required a finite value."""
  return efficiency > 0.0 and efficiency <= 1.0


def _assess_header(header: ThrusterHeaderView) -> ThrusterArrayError:
  if not header.header_present:
    return ThrusterArrayError.MISSING_HEADER
  if not header.frame_id:
    return ThrusterArrayError.MISSING_FRAME
  if header.frame_id != vehicle_contract_policy.BODY_FRAME_ID:
    return ThrusterArrayError.WRONG_FRAME
  if (
      header.source_time_present
      and header.receive_time_present
      and _timestamp_precedes(header.receive_time, header.source_time)
  ):
    return ThrusterArrayError.TIME_REVERSAL
  return ThrusterArrayError.NONE


def _assessment(
    error: ThrusterArrayError,
    header_validity: stamped_header_policy.ValidityKind,
    engaged: bool,
) -> ThrusterArrayAssessment:
  return ThrusterArrayAssessment(
      error, header_validity, engaged and error is ThrusterArrayError.NONE
  )


def assess_thruster_array_command(
    sample: ThrusterArrayCommandView,
) -> ThrusterArrayAssessment:
  """First defect wins. See the module docstring and README.md."""
  header_validity = stamped_header_policy.classify_validity(
      sample.header.header_validity_present, sample.header.header_validity_state
  )
  if not thruster_array_command_engaged(sample):
    return _assessment(ThrusterArrayError.NONE, header_validity, False)
  error = _assess_header(sample.header)
  if error is ThrusterArrayError.NONE and not sample.thrusters:
    error = ThrusterArrayError.EMPTY_ARRAY
  if error is ThrusterArrayError.NONE:
    for element in sample.thrusters:
      if element.name_present and element.name == "":
        error = ThrusterArrayError.EMPTY_NAME
        break
      if not element.thrust_present:
        error = ThrusterArrayError.MISSING_THRUST
        break
      if not math.isfinite(element.thrust_n):
        error = ThrusterArrayError.NON_FINITE
        break
  return _assessment(error, header_validity, True)


def assess_thruster_array_feedback(
    sample: ThrusterArrayFeedbackView,
) -> ThrusterArrayAssessment:
  """First defect wins. `saturated` is diagnostic and is not a defect."""
  header_validity = stamped_header_policy.classify_validity(
      sample.header.header_validity_present, sample.header.header_validity_state
  )
  if not thruster_array_feedback_engaged(sample):
    return _assessment(ThrusterArrayError.NONE, header_validity, False)
  error = _assess_header(sample.header)
  if error is ThrusterArrayError.NONE and not sample.thrusters:
    error = ThrusterArrayError.EMPTY_ARRAY
  if error is ThrusterArrayError.NONE:
    for element in sample.thrusters:
      if element.name_present and element.name == "":
        error = ThrusterArrayError.EMPTY_NAME
        break
      if element.commanded_thrust_present and not math.isfinite(
          element.commanded_thrust_n
      ):
        error = ThrusterArrayError.NON_FINITE
        break
      if element.measured_thrust_present and not math.isfinite(
          element.measured_thrust_n
      ):
        error = ThrusterArrayError.NON_FINITE
        break
      if element.health_derate_present and not math.isfinite(
          element.health_derate
      ):
        error = ThrusterArrayError.NON_FINITE
        break
      if element.efficiency_present and not math.isfinite(element.efficiency):
        error = ThrusterArrayError.NON_FINITE
        break
      health = classify_thruster_health(element.health_present, element.health)
      if health is ThrusterHealthKind.UNRECOGNIZED:
        error = ThrusterArrayError.HEALTH
        break
      if element.health_present and (
          not element.health_derate_present
          or not thruster_derate_matches_health(health, element.health_derate)
      ):
        error = ThrusterArrayError.HEALTH_DERATE
        break
      if element.efficiency_present and not thruster_efficiency_in_range(
          element.efficiency
      ):
        error = ThrusterArrayError.EFFICIENCY
        break
  return _assessment(error, header_validity, True)
