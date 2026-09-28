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

"""Plain-value checks for vehicle state and command protos.

Policy: intrinsic_apis/intrinsic/vehicle/proto/README.md.

These helpers do not parse protobuf, do not convert ENU and NED, and do
not command actuators.

BodyVector order is linear x, y, z, then angular x, y, z. Units come from
the proto field that supplied the vector. For a wrench, the linear triple
is force in newtons and the angular triple is torque in newton-meters.
"""

from collections.abc import Sequence
from dataclasses import dataclass
import enum
import math

from intrinsic.embodiment import frame_policy
from intrinsic.embodiment import stamped_header_policy

BODY_FRAME_ID = "body"
SPATIAL_DOF = 6
COVARIANCE_VALUES = SPATIAL_DOF * SPATIAL_DOF
COVARIANCE_SYMMETRY_TOLERANCE = 1e-9

# linear x, y, z, angular x, y, z.
BodyVector = tuple[float, float, float, float, float, float]


class CovarianceError(enum.Enum):
  NONE = 0
  ABSENT = 1
  WRONG_LENGTH = 2
  NON_FINITE = 3
  ASYMMETRIC = 4


class NavigationModeKind(enum.Enum):
  UNSPECIFIED = 0
  INITIALIZING = 1
  DEAD_RECKONING = 2
  AIDED = 3
  FAULTED = 4
  UNKNOWN = 5


class ContractError(enum.Enum):
  NONE = 0
  NON_FINITE = 1
  QUATERNION = 2
  MISSING_FRAME = 3
  BODY_FRAME = 4
  POSE_COVARIANCE = 5
  TWIST_COVARIANCE = 6
  CONFIDENCE = 7
  HORIZON = 8
  OBJECTIVE = 9
  TRAJECTORY_ID = 10
  PROVENANCE = 11
  SOURCE_ID = 12


class ObjectiveKind(enum.Enum):
  ABSENT = 0
  POSE = 1
  TWIST = 2
  TRAJECTORY = 3


@dataclass(frozen=True)
class ContractAssessment:
  error: ContractError
  validity: stamped_header_policy.ValidityKind
  accepted: bool


@dataclass(frozen=True)
class SourceView:
  source_id: str = ""
  validity_present: bool = False
  validity_state: int = 0


@dataclass(frozen=True)
class VehicleStateView:
  validity_present: bool = False
  validity_state: int = 0
  frame_id: str = ""
  pose_present: bool = False
  position: frame_policy.Vec3 = (0.0, 0.0, 0.0)
  orientation: frame_policy.Quaternion = (0.0, 0.0, 0.0, 1.0)
  twist_present: bool = False
  twist: BodyVector = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
  acceleration_present: bool = False
  acceleration: BodyVector = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
  pose_covariance_present: bool = False
  pose_covariance: Sequence[float] = ()
  twist_covariance_present: bool = False
  twist_covariance: Sequence[float] = ()
  sources: tuple[SourceView, ...] = ()


@dataclass(frozen=True)
class DesiredMotionView:
  header_present: bool = False
  validity_present: bool = False
  validity_state: int = 0
  frame_id: str = ""
  objective: ObjectiveKind = ObjectiveKind.ABSENT
  position: frame_policy.Vec3 = (0.0, 0.0, 0.0)
  orientation: frame_policy.Quaternion = (0.0, 0.0, 0.0, 1.0)
  twist: BodyVector = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
  trajectory_id: str = ""
  confidence_present: bool = False
  confidence: float = 0.0
  horizon_present: bool = False
  horizon_seconds: int = 0
  horizon_nanos: int = 0
  provenance_present: bool = False
  model_id: str = ""


@dataclass(frozen=True)
class BodyWrenchView:
  engaged: bool = False
  validity_present: bool = False
  validity_state: int = 0
  frame_id: str = ""
  force_torque: BodyVector = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)


def covariance_index(row: int, col: int) -> int:
  """Row-major index. row and col are in [0, 5]."""
  return row * SPATIAL_DOF + col


def _finite_body(vector: BodyVector) -> bool:
  return all(math.isfinite(component) for component in vector)


def assess_covariance(
    present: bool, values: Sequence[float]
) -> CovarianceError:
  """present false is unknown. A present sequence is specified data."""
  if not present:
    return CovarianceError.ABSENT
  if len(values) != COVARIANCE_VALUES:
    return CovarianceError.WRONG_LENGTH
  if not all(math.isfinite(value) for value in values):
    return CovarianceError.NON_FINITE
  for row in range(SPATIAL_DOF):
    for col in range(row + 1, SPATIAL_DOF):
      upper = values[covariance_index(row, col)]
      lower = values[covariance_index(col, row)]
      if abs(upper - lower) > COVARIANCE_SYMMETRY_TOLERANCE:
        return CovarianceError.ASYMMETRIC
  return CovarianceError.NONE


def covariance_is_unknown(error: CovarianceError) -> bool:
  return error is CovarianceError.ABSENT


def is_all_zero_covariance(values: Sequence[float]) -> bool:
  return len(values) == COVARIANCE_VALUES and all(
      value == 0.0 for value in values
  )


def classify_navigation_mode(mode: int) -> NavigationModeKind:
  """Wire values other than 0..4 are UNKNOWN. UNKNOWN is not faulted."""
  known = {
      0: NavigationModeKind.UNSPECIFIED,
      1: NavigationModeKind.INITIALIZING,
      2: NavigationModeKind.DEAD_RECKONING,
      3: NavigationModeKind.AIDED,
      4: NavigationModeKind.FAULTED,
  }
  return known.get(mode, NavigationModeKind.UNKNOWN)


def confidence_in_range(confidence: float) -> bool:
  return math.isfinite(confidence) and 0.0 <= confidence <= 1.0


def duration_non_negative(seconds: int, nanos: int) -> bool:
  return (
      seconds >= 0
      and nanos >= 0
      and nanos < stamped_header_policy.NANOS_PER_SECOND
  )


def _assessment(
    error: ContractError, validity: stamped_header_policy.ValidityKind
) -> ContractAssessment:
  accepted = (
      error is ContractError.NONE
      and stamped_header_policy.sample_accepted(validity, True)
  )
  return ContractAssessment(error, validity, accepted)


def _absent_command() -> ContractAssessment:
  return ContractAssessment(
      ContractError.NONE, stamped_header_policy.ValidityKind.ABSENT, False
  )


def assess_vehicle_state(state: VehicleStateView) -> ContractAssessment:
  """First defect wins. Absent optional fields are not defects."""
  validity = stamped_header_policy.classify_validity(
      state.validity_present, state.validity_state
  )
  error = ContractError.NONE
  if state.pose_present:
    if not frame_policy.is_finite_vec3(state.position):
      error = ContractError.NON_FINITE
    elif not frame_policy.is_finite_quaternion(state.orientation):
      error = ContractError.NON_FINITE
    elif not frame_policy.is_normalized(state.orientation):
      error = ContractError.QUATERNION
    elif state.frame_id == "":
      error = ContractError.MISSING_FRAME
  if error is ContractError.NONE and state.twist_present:
    if not _finite_body(state.twist):
      error = ContractError.NON_FINITE
  if error is ContractError.NONE and state.acceleration_present:
    if not _finite_body(state.acceleration):
      error = ContractError.NON_FINITE
  if error is ContractError.NONE:
    pose = assess_covariance(
        state.pose_covariance_present, state.pose_covariance
    )
    if pose not in (CovarianceError.NONE, CovarianceError.ABSENT):
      error = ContractError.POSE_COVARIANCE
  if error is ContractError.NONE:
    twist = assess_covariance(
        state.twist_covariance_present, state.twist_covariance
    )
    if twist not in (CovarianceError.NONE, CovarianceError.ABSENT):
      error = ContractError.TWIST_COVARIANCE
  if error is ContractError.NONE:
    for source in state.sources:
      if source.source_id == "":
        error = ContractError.SOURCE_ID
        break
  return _assessment(error, validity)


def _motion_engaged(motion: DesiredMotionView) -> bool:
  return (
      motion.header_present
      or motion.objective is not ObjectiveKind.ABSENT
      or motion.confidence_present
      or motion.horizon_present
      or motion.provenance_present
  )


def assess_desired_motion(motion: DesiredMotionView) -> ContractAssessment:
  """Empty view is not an intent. Check order matches the C++ helper."""
  if not _motion_engaged(motion):
    return _absent_command()
  validity = stamped_header_policy.classify_validity(
      motion.validity_present, motion.validity_state
  )
  error = ContractError.NONE
  if motion.objective is ObjectiveKind.ABSENT:
    error = ContractError.OBJECTIVE
  elif motion.objective is ObjectiveKind.POSE:
    if not frame_policy.is_finite_vec3(motion.position):
      error = ContractError.NON_FINITE
    elif not frame_policy.is_finite_quaternion(motion.orientation):
      error = ContractError.NON_FINITE
    elif not frame_policy.is_normalized(motion.orientation):
      error = ContractError.QUATERNION
    elif motion.frame_id == "":
      error = ContractError.MISSING_FRAME
  elif motion.objective is ObjectiveKind.TWIST:
    if not _finite_body(motion.twist):
      error = ContractError.NON_FINITE
    elif motion.frame_id != BODY_FRAME_ID:
      error = ContractError.BODY_FRAME
  elif motion.trajectory_id == "":
    error = ContractError.TRAJECTORY_ID
  if error is ContractError.NONE and motion.confidence_present:
    if not confidence_in_range(motion.confidence):
      error = ContractError.CONFIDENCE
  if error is ContractError.NONE and motion.horizon_present:
    if not duration_non_negative(motion.horizon_seconds, motion.horizon_nanos):
      error = ContractError.HORIZON
  if error is ContractError.NONE and motion.provenance_present:
    if motion.model_id == "":
      error = ContractError.PROVENANCE
  return _assessment(error, validity)


def assess_body_wrench(wrench: BodyWrenchView) -> ContractAssessment:
  """Unengaged is not a command. Engaged frame id must be body."""
  if not wrench.engaged:
    return _absent_command()
  validity = stamped_header_policy.classify_validity(
      wrench.validity_present, wrench.validity_state
  )
  error = ContractError.NONE
  if not _finite_body(wrench.force_torque):
    error = ContractError.NON_FINITE
  elif wrench.frame_id != BODY_FRAME_ID:
    error = ContractError.BODY_FRAME
  return _assessment(error, validity)
