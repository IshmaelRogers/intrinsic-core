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

"""Plain-value checks for VehicleTrajectory.

Policy: intrinsic_apis/intrinsic/vehicle/proto/README.md.

These helpers do not parse protobuf, do not interpolate, do not convert
ENU and NED, and do not call World, ICON, or a HAL.
"""

from collections.abc import Sequence
from dataclasses import dataclass
import enum
import math

from intrinsic.embodiment import frame_policy
from intrinsic.embodiment import stamped_header_policy
from intrinsic.vehicle import vehicle_contract_policy


class TrajectoryContractError(enum.Enum):
  NONE = 0
  TRAJECTORY_ID = 1
  EMPTY_SAMPLES = 2
  NON_MONOTONIC_TIME = 3
  MISSING_FRAME = 4
  NON_FINITE = 5
  QUATERNION = 6
  TOLERANCE = 7
  COST = 8
  RISK = 9
  UNCERTAINTY = 10
  PROVENANCE = 11
  SAMPLE_TIME = 12


@dataclass(frozen=True)
class TrajectoryContractAssessment:
  error: TrajectoryContractError
  validity: stamped_header_policy.ValidityKind
  accepted: bool


@dataclass(frozen=True)
class TrajectorySampleView:
  time_present: bool = False
  seconds: int = 0
  nanos: int = 0
  position: frame_policy.Vec3 = (0.0, 0.0, 0.0)
  orientation: frame_policy.Quaternion = (0.0, 0.0, 0.0, 1.0)
  twist_present: bool = False
  twist: vehicle_contract_policy.BodyVector = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
  acceleration_present: bool = False
  acceleration: vehicle_contract_policy.BodyVector = (
      0.0,
      0.0,
      0.0,
      0.0,
      0.0,
      0.0,
  )


@dataclass(frozen=True)
class VehicleTrajectoryView:
  header_present: bool = False
  validity_present: bool = False
  validity_state: int = 0
  frame_id: str = ""
  trajectory_id: str = ""
  samples: tuple[TrajectorySampleView, ...] = ()
  tolerances_present: bool = False
  position_tolerance_m: float = 0.0
  orientation_tolerance_rad: float = 0.0
  linear_velocity_tolerance_m_s: float = 0.0
  angular_velocity_tolerance_rad_s: float = 0.0
  cost_present: bool = False
  cost: float = 0.0
  risk_present: bool = False
  risk: float = 0.0
  uncertainty_present: bool = False
  uncertainty: Sequence[float] = ()
  provenance_present: bool = False
  model_id: str = ""
  metadata_present: bool = False


def trajectory_engaged(trajectory: VehicleTrajectoryView) -> bool:
  return (
      trajectory.header_present
      or trajectory.trajectory_id != ""
      or len(trajectory.samples) > 0
      or trajectory.tolerances_present
      or trajectory.cost_present
      or trajectory.risk_present
      or trajectory.uncertainty_present
      or trajectory.provenance_present
      or trajectory.metadata_present
  )


def sample_time_ok(sample: TrajectorySampleView) -> bool:
  return (
      sample.time_present
      and sample.nanos >= 0
      and sample.nanos < stamped_header_policy.NANOS_PER_SECOND
  )


def sample_time_before(
    earlier: TrajectorySampleView, later: TrajectorySampleView
) -> bool:
  """Strictly earlier, compared as (seconds, nanos)."""
  if earlier.seconds != later.seconds:
    return earlier.seconds < later.seconds
  return earlier.nanos < later.nanos


def tolerance_ok(value: float) -> bool:
  return math.isfinite(value) and value >= 0.0


def cost_ok(cost: float) -> bool:
  return math.isfinite(cost)


def risk_ok(risk: float) -> bool:
  return math.isfinite(risk) and 0.0 <= risk <= 1.0


def _finite_body(vector: vehicle_contract_policy.BodyVector) -> bool:
  return all(math.isfinite(component) for component in vector)


def _assessment(
    error: TrajectoryContractError, validity: stamped_header_policy.ValidityKind
) -> TrajectoryContractAssessment:
  accepted = (
      error is TrajectoryContractError.NONE
      and stamped_header_policy.sample_accepted(validity, True)
  )
  return TrajectoryContractAssessment(error, validity, accepted)


def _not_engaged() -> TrajectoryContractAssessment:
  return TrajectoryContractAssessment(
      TrajectoryContractError.NONE,
      stamped_header_policy.ValidityKind.ABSENT,
      False,
  )


def assess_vehicle_trajectory(
    trajectory: VehicleTrajectoryView,
) -> TrajectoryContractAssessment:
  """First defect wins. An empty view is not engaged.

  Order: trajectory id, sample count, each sample time in index order,
  strict increase, frame id, then each sample's pose, twist, and
  acceleration, then tolerances, cost, risk, uncertainty, and provenance.
  Metadata is never a defect.
  """
  if not trajectory_engaged(trajectory):
    return _not_engaged()
  validity = stamped_header_policy.classify_validity(
      trajectory.validity_present, trajectory.validity_state
  )
  error = TrajectoryContractError.NONE
  if trajectory.trajectory_id == "":
    error = TrajectoryContractError.TRAJECTORY_ID
  elif not trajectory.samples:
    error = TrajectoryContractError.EMPTY_SAMPLES
  else:
    for sample in trajectory.samples:
      if not sample_time_ok(sample):
        error = TrajectoryContractError.SAMPLE_TIME
        break
    if error is TrajectoryContractError.NONE:
      for index in range(1, len(trajectory.samples)):
        if not sample_time_before(
            trajectory.samples[index - 1], trajectory.samples[index]
        ):
          error = TrajectoryContractError.NON_MONOTONIC_TIME
          break
    if error is TrajectoryContractError.NONE and trajectory.frame_id == "":
      error = TrajectoryContractError.MISSING_FRAME
    if error is TrajectoryContractError.NONE:
      for sample in trajectory.samples:
        if not frame_policy.is_finite_vec3(sample.position):
          error = TrajectoryContractError.NON_FINITE
        elif not frame_policy.is_finite_quaternion(sample.orientation):
          error = TrajectoryContractError.NON_FINITE
        elif not frame_policy.is_normalized(sample.orientation):
          error = TrajectoryContractError.QUATERNION
        elif sample.twist_present and not _finite_body(sample.twist):
          error = TrajectoryContractError.NON_FINITE
        elif sample.acceleration_present and not _finite_body(
            sample.acceleration
        ):
          error = TrajectoryContractError.NON_FINITE
        if error is not TrajectoryContractError.NONE:
          break
  if (
      error is TrajectoryContractError.NONE
      and trajectory.tolerances_present
      and not (
          tolerance_ok(trajectory.position_tolerance_m)
          and tolerance_ok(trajectory.orientation_tolerance_rad)
          and tolerance_ok(trajectory.linear_velocity_tolerance_m_s)
          and tolerance_ok(trajectory.angular_velocity_tolerance_rad_s)
      )
  ):
    error = TrajectoryContractError.TOLERANCE
  if (
      error is TrajectoryContractError.NONE
      and trajectory.cost_present
      and not cost_ok(trajectory.cost)
  ):
    error = TrajectoryContractError.COST
  if (
      error is TrajectoryContractError.NONE
      and trajectory.risk_present
      and not risk_ok(trajectory.risk)
  ):
    error = TrajectoryContractError.RISK
  if error is TrajectoryContractError.NONE and trajectory.uncertainty_present:
    covariance = vehicle_contract_policy.assess_covariance(
        True, trajectory.uncertainty
    )
    if covariance is not vehicle_contract_policy.CovarianceError.NONE:
      error = TrajectoryContractError.UNCERTAINTY
  if (
      error is TrajectoryContractError.NONE
      and trajectory.provenance_present
      and trajectory.model_id == ""
  ):
    error = TrajectoryContractError.PROVENANCE
  return _assessment(error, validity)
