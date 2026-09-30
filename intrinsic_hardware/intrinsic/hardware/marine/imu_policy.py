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

"""Plain-value checks for one raw IMU measurement.

Policy: intrinsic_hardware/intrinsic/hardware/marine/README.md.

These helpers do not parse protobuf, do not convert ENU and NED, do not
renormalize quaternions, do not propagate a filter, and do not command
hardware.

Health checks are measurement_health_policy.assess_measurement_health.
This module does not rewrite health state or embodiment Validity.

An engaged sample requires both angular-velocity and linear-acceleration
triples. Orientation is optional. A present orientation is a Hamilton
quaternion (x, y, z, w). Unit norm is an absolute tolerance of 1e-6 on
abs(norm - 1).
"""

from collections.abc import Sequence
from dataclasses import dataclass
import enum
import math

from intrinsic.hardware.marine import measurement_health_policy

from intrinsic.embodiment import frame_policy
from intrinsic.embodiment import stamped_header_policy
from intrinsic.vehicle import vehicle_contract_policy

# Absolute tolerance for |‖q‖ − 1|. The INS policy uses the same value.
UNIT_QUATERNION_TOLERANCE = 1e-6

# Row-major Matrix6 diagonals. Off-diagonal entries stay exactly 0.
ANGULAR_VELOCITY_X_VARIANCE_SLOT = vehicle_contract_policy.covariance_index(
    0, 0
)
ANGULAR_VELOCITY_Y_VARIANCE_SLOT = vehicle_contract_policy.covariance_index(
    1, 1
)
ANGULAR_VELOCITY_Z_VARIANCE_SLOT = vehicle_contract_policy.covariance_index(
    2, 2
)
LINEAR_ACCELERATION_X_VARIANCE_SLOT = vehicle_contract_policy.covariance_index(
    3, 3
)
LINEAR_ACCELERATION_Y_VARIANCE_SLOT = vehicle_contract_policy.covariance_index(
    4, 4
)
LINEAR_ACCELERATION_Z_VARIANCE_SLOT = vehicle_contract_policy.covariance_index(
    5, 5
)

_IMU_VARIANCE_SLOTS = (
    ANGULAR_VELOCITY_X_VARIANCE_SLOT,
    ANGULAR_VELOCITY_Y_VARIANCE_SLOT,
    ANGULAR_VELOCITY_Z_VARIANCE_SLOT,
    LINEAR_ACCELERATION_X_VARIANCE_SLOT,
    LINEAR_ACCELERATION_Y_VARIANCE_SLOT,
    LINEAR_ACCELERATION_Z_VARIANCE_SLOT,
)


class ImuError(enum.Enum):
  NONE = 0
  MISSING_HEALTH = 1
  MISSING_FRAME = 2
  WRONG_FRAME = 3
  TIME_REVERSAL = 4
  NON_FINITE = 5
  QUALITY = 6
  COVARIANCE = 7
  SOURCE_ID = 8
  COVARIANCE_SLOTS = 9
  ANGULAR_VELOCITY = 10
  LINEAR_ACCELERATION = 11
  ORIENTATION = 12


@dataclass(frozen=True)
class ImuAssessment:
  error: ImuError
  state: measurement_health_policy.MeasurementStateKind
  header_validity: stamped_header_policy.ValidityKind
  accepted: bool


@dataclass(frozen=True)
class ImuMeasurementView:
  health: measurement_health_policy.MeasurementHealthView = (
      measurement_health_policy.MeasurementHealthView()
  )
  angular_velocity_x_present: bool = False
  angular_velocity_x_rad_s: float = 0.0
  angular_velocity_y_present: bool = False
  angular_velocity_y_rad_s: float = 0.0
  angular_velocity_z_present: bool = False
  angular_velocity_z_rad_s: float = 0.0
  linear_acceleration_x_present: bool = False
  linear_acceleration_x_m_s2: float = 0.0
  linear_acceleration_y_present: bool = False
  linear_acceleration_y_m_s2: float = 0.0
  linear_acceleration_z_present: bool = False
  linear_acceleration_z_m_s2: float = 0.0
  orientation_present: bool = False
  orientation_x: float = 0.0
  orientation_y: float = 0.0
  orientation_z: float = 0.0
  orientation_w: float = 0.0


def imu_engaged(sample: ImuMeasurementView) -> bool:
  return (
      measurement_health_policy.measurement_engaged(sample.health)
      or sample.angular_velocity_x_present
      or sample.angular_velocity_y_present
      or sample.angular_velocity_z_present
      or sample.linear_acceleration_x_present
      or sample.linear_acceleration_y_present
      or sample.linear_acceleration_z_present
      or sample.orientation_present
  )


def allowed_imu_covariance_only(values: Sequence[float]) -> bool:
  if len(values) != vehicle_contract_policy.COVARIANCE_VALUES:
    return False
  allowed = set(_IMU_VARIANCE_SLOTS)
  for index, value in enumerate(values):
    if index not in allowed and value != 0.0:
      return False
  return True


def unit_quaternion(x: float, y: float, z: float, w: float) -> bool:
  """Finite Hamilton quaternion with |norm - 1| <= 1e-6. Does not renormalize."""
  quaternion = (x, y, z, w)
  if not frame_policy.is_finite_quaternion(quaternion):
    return False
  return (
      abs(frame_policy.quaternion_norm(quaternion) - 1.0)
      <= UNIT_QUATERNION_TOLERANCE
  )


def _from_measurement_error(error) -> ImuError:
  return {
      measurement_health_policy.MeasurementError.NONE: ImuError.NONE,
      measurement_health_policy.MeasurementError.MISSING_FRAME: (
          ImuError.MISSING_FRAME
      ),
      measurement_health_policy.MeasurementError.WRONG_FRAME: (
          ImuError.WRONG_FRAME
      ),
      measurement_health_policy.MeasurementError.TIME_REVERSAL: (
          ImuError.TIME_REVERSAL
      ),
      measurement_health_policy.MeasurementError.NON_FINITE: ImuError.NON_FINITE,
      measurement_health_policy.MeasurementError.QUALITY: ImuError.QUALITY,
      measurement_health_policy.MeasurementError.COVARIANCE: ImuError.COVARIANCE,
      measurement_health_policy.MeasurementError.SOURCE_ID: ImuError.SOURCE_ID,
  }[error]


def assess_imu(sample: ImuMeasurementView) -> ImuAssessment:
  """First defect wins. Absent orientation is not a defect.

  Check order: missing health, #75 health order, covariance slots,
  angular-velocity presence, angular-velocity finiteness,
  linear-acceleration presence, linear-acceleration finiteness, orientation
  finiteness, orientation unit norm. Health state is not rewritten.
  """
  header_validity = stamped_header_policy.classify_validity(
      sample.health.header_validity_present,
      sample.health.header_validity_state,
  )
  if not imu_engaged(sample):
    return ImuAssessment(
        ImuError.NONE,
        measurement_health_policy.MeasurementStateKind.ABSENT,
        header_validity,
        False,
    )

  state = measurement_health_policy.MeasurementStateKind.ABSENT
  error = ImuError.NONE
  if not measurement_health_policy.measurement_engaged(sample.health):
    error = ImuError.MISSING_HEALTH
  else:
    health = measurement_health_policy.assess_measurement_health(sample.health)
    state = health.state
    error = _from_measurement_error(health.error)
    if (
        error is ImuError.NONE
        and sample.health.covariance_present
        and vehicle_contract_policy.assess_covariance(
            True, sample.health.covariance
        )
        is vehicle_contract_policy.CovarianceError.NONE
        and not allowed_imu_covariance_only(sample.health.covariance)
    ):
      error = ImuError.COVARIANCE_SLOTS
  if error is ImuError.NONE and not (
      sample.angular_velocity_x_present
      and sample.angular_velocity_y_present
      and sample.angular_velocity_z_present
  ):
    error = ImuError.ANGULAR_VELOCITY
  if error is ImuError.NONE and not (
      math.isfinite(sample.angular_velocity_x_rad_s)
      and math.isfinite(sample.angular_velocity_y_rad_s)
      and math.isfinite(sample.angular_velocity_z_rad_s)
  ):
    error = ImuError.NON_FINITE
  if error is ImuError.NONE and not (
      sample.linear_acceleration_x_present
      and sample.linear_acceleration_y_present
      and sample.linear_acceleration_z_present
  ):
    error = ImuError.LINEAR_ACCELERATION
  if error is ImuError.NONE and not (
      math.isfinite(sample.linear_acceleration_x_m_s2)
      and math.isfinite(sample.linear_acceleration_y_m_s2)
      and math.isfinite(sample.linear_acceleration_z_m_s2)
  ):
    error = ImuError.NON_FINITE
  if error is ImuError.NONE and sample.orientation_present:
    if not frame_policy.is_finite_quaternion(
        (
            sample.orientation_x,
            sample.orientation_y,
            sample.orientation_z,
            sample.orientation_w,
        )
    ):
      error = ImuError.NON_FINITE
    elif not unit_quaternion(
        sample.orientation_x,
        sample.orientation_y,
        sample.orientation_z,
        sample.orientation_w,
    ):
      error = ImuError.ORIENTATION
  accepted = (
      error is ImuError.NONE
      and state is measurement_health_policy.MeasurementStateKind.VALID
  )
  return ImuAssessment(error, state, header_validity, accepted)
