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

"""Plain-value checks for one vendor INS solution.

Policy: intrinsic_hardware/intrinsic/hardware/marine/README.md.

These helpers do not parse protobuf, do not convert ENU and NED, do not
renormalize quaternions, do not read IMU samples, do not propagate a
filter, and do not command hardware.

Health checks are measurement_health_policy.assess_measurement_health.
This module does not rewrite health state or embodiment Validity.

`source` is the measurement-local enum (SOURCE_UNSPECIFIED=0,
SOURCE_VENDOR_INS=1, SOURCE_EXTERNAL_NAV=2). Unset source is absent. A
present 0 is SOURCE_UNSPECIFIED. Other numbers stay unrecognized.

Unit norm uses the same absolute tolerance as the IMU policy: 1e-6.
"""

from collections.abc import Sequence
from dataclasses import dataclass
import enum
import math

from intrinsic.hardware.marine import measurement_health_policy

from intrinsic.embodiment import frame_policy
from intrinsic.embodiment import stamped_header_policy
from intrinsic.vehicle import vehicle_contract_policy

# Same absolute tolerance as imu_policy.UNIT_QUATERNION_TOLERANCE.
UNIT_QUATERNION_TOLERANCE = 1e-6

POSITION_X_VARIANCE_SLOT = vehicle_contract_policy.covariance_index(0, 0)
POSITION_Y_VARIANCE_SLOT = vehicle_contract_policy.covariance_index(1, 1)
POSITION_Z_VARIANCE_SLOT = vehicle_contract_policy.covariance_index(2, 2)
ATTITUDE_X_VARIANCE_SLOT = vehicle_contract_policy.covariance_index(3, 3)
ATTITUDE_Y_VARIANCE_SLOT = vehicle_contract_policy.covariance_index(4, 4)
ATTITUDE_Z_VARIANCE_SLOT = vehicle_contract_policy.covariance_index(5, 5)

_INS_VARIANCE_SLOTS = (
    POSITION_X_VARIANCE_SLOT,
    POSITION_Y_VARIANCE_SLOT,
    POSITION_Z_VARIANCE_SLOT,
    ATTITUDE_X_VARIANCE_SLOT,
    ATTITUDE_Y_VARIANCE_SLOT,
    ATTITUDE_Z_VARIANCE_SLOT,
)


class InsSourceKind(enum.Enum):
  ABSENT = 0
  UNSPECIFIED = 1
  VENDOR_INS = 2
  EXTERNAL_NAV = 3
  UNRECOGNIZED = 4


class InsError(enum.Enum):
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
  POSITION = 10
  ORIENTATION = 11
  LINEAR_VELOCITY = 12
  ANGULAR_VELOCITY = 13
  SOURCE = 14


@dataclass(frozen=True)
class InsAssessment:
  error: InsError
  state: measurement_health_policy.MeasurementStateKind
  source: InsSourceKind
  header_validity: stamped_header_policy.ValidityKind
  accepted: bool


@dataclass(frozen=True)
class InsSolutionView:
  health: measurement_health_policy.MeasurementHealthView = (
      measurement_health_policy.MeasurementHealthView()
  )
  position_x_present: bool = False
  position_x_m: float = 0.0
  position_y_present: bool = False
  position_y_m: float = 0.0
  position_z_present: bool = False
  position_z_m: float = 0.0
  orientation_present: bool = False
  orientation_x: float = 0.0
  orientation_y: float = 0.0
  orientation_z: float = 0.0
  orientation_w: float = 0.0
  linear_velocity_x_present: bool = False
  linear_velocity_x_m_s: float = 0.0
  linear_velocity_y_present: bool = False
  linear_velocity_y_m_s: float = 0.0
  linear_velocity_z_present: bool = False
  linear_velocity_z_m_s: float = 0.0
  angular_velocity_x_present: bool = False
  angular_velocity_x_rad_s: float = 0.0
  angular_velocity_y_present: bool = False
  angular_velocity_y_rad_s: float = 0.0
  angular_velocity_z_present: bool = False
  angular_velocity_z_rad_s: float = 0.0
  source_present: bool = False
  source: int = 0


def classify_ins_source(field_present: bool, source: int) -> InsSourceKind:
  """field_present is InsSolution.source presence, not the enum value."""
  if not field_present:
    return InsSourceKind.ABSENT
  return {
      0: InsSourceKind.UNSPECIFIED,
      1: InsSourceKind.VENDOR_INS,
      2: InsSourceKind.EXTERNAL_NAV,
  }.get(source, InsSourceKind.UNRECOGNIZED)


def ins_engaged(sample: InsSolutionView) -> bool:
  return (
      measurement_health_policy.measurement_engaged(sample.health)
      or sample.position_x_present
      or sample.position_y_present
      or sample.position_z_present
      or sample.orientation_present
      or sample.linear_velocity_x_present
      or sample.linear_velocity_y_present
      or sample.linear_velocity_z_present
      or sample.angular_velocity_x_present
      or sample.angular_velocity_y_present
      or sample.angular_velocity_z_present
      or sample.source_present
  )


def allowed_ins_covariance_only(values: Sequence[float]) -> bool:
  if len(values) != vehicle_contract_policy.COVARIANCE_VALUES:
    return False
  allowed = set(_INS_VARIANCE_SLOTS)
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


def _known_source(source: InsSourceKind) -> bool:
  return source in (InsSourceKind.VENDOR_INS, InsSourceKind.EXTERNAL_NAV)


def _from_measurement_error(error) -> InsError:
  return {
      measurement_health_policy.MeasurementError.NONE: InsError.NONE,
      measurement_health_policy.MeasurementError.MISSING_FRAME: (
          InsError.MISSING_FRAME
      ),
      measurement_health_policy.MeasurementError.WRONG_FRAME: (
          InsError.WRONG_FRAME
      ),
      measurement_health_policy.MeasurementError.TIME_REVERSAL: (
          InsError.TIME_REVERSAL
      ),
      measurement_health_policy.MeasurementError.NON_FINITE: InsError.NON_FINITE,
      measurement_health_policy.MeasurementError.QUALITY: InsError.QUALITY,
      measurement_health_policy.MeasurementError.COVARIANCE: InsError.COVARIANCE,
      measurement_health_policy.MeasurementError.SOURCE_ID: InsError.SOURCE_ID,
  }[error]


def _present_count(x: bool, y: bool, z: bool) -> int:
  return int(x) + int(y) + int(z)


def assess_ins(sample: InsSolutionView) -> InsAssessment:
  """First defect wins. Absent twist and absent source are not defects.

  Check order: missing health, #75 health order, covariance slots, position
  presence, position finiteness, orientation presence, orientation
  finiteness, orientation unit norm, linear-velocity partial triples,
  linear-velocity finiteness, angular-velocity partial triples,
  angular-velocity finiteness, source kind. Health state is not rewritten.
  """
  header_validity = stamped_header_policy.classify_validity(
      sample.health.header_validity_present,
      sample.health.header_validity_state,
  )
  source = classify_ins_source(sample.source_present, sample.source)
  if not ins_engaged(sample):
    return InsAssessment(
        InsError.NONE,
        measurement_health_policy.MeasurementStateKind.ABSENT,
        InsSourceKind.ABSENT,
        header_validity,
        False,
    )

  state = measurement_health_policy.MeasurementStateKind.ABSENT
  error = InsError.NONE
  if not measurement_health_policy.measurement_engaged(sample.health):
    error = InsError.MISSING_HEALTH
  else:
    health = measurement_health_policy.assess_measurement_health(sample.health)
    state = health.state
    error = _from_measurement_error(health.error)
    if (
        error is InsError.NONE
        and sample.health.covariance_present
        and vehicle_contract_policy.assess_covariance(
            True, sample.health.covariance
        )
        is vehicle_contract_policy.CovarianceError.NONE
        and not allowed_ins_covariance_only(sample.health.covariance)
    ):
      error = InsError.COVARIANCE_SLOTS
  if error is InsError.NONE and not (
      sample.position_x_present
      and sample.position_y_present
      and sample.position_z_present
  ):
    error = InsError.POSITION
  if error is InsError.NONE and not (
      math.isfinite(sample.position_x_m)
      and math.isfinite(sample.position_y_m)
      and math.isfinite(sample.position_z_m)
  ):
    error = InsError.NON_FINITE
  if error is InsError.NONE and not sample.orientation_present:
    error = InsError.ORIENTATION
  if error is InsError.NONE:
    if not frame_policy.is_finite_quaternion(
        (
            sample.orientation_x,
            sample.orientation_y,
            sample.orientation_z,
            sample.orientation_w,
        )
    ):
      error = InsError.NON_FINITE
    elif not unit_quaternion(
        sample.orientation_x,
        sample.orientation_y,
        sample.orientation_z,
        sample.orientation_w,
    ):
      error = InsError.ORIENTATION
  linear = _present_count(
      sample.linear_velocity_x_present,
      sample.linear_velocity_y_present,
      sample.linear_velocity_z_present,
  )
  if error is InsError.NONE and linear not in (0, 3):
    error = InsError.LINEAR_VELOCITY
  if (
      error is InsError.NONE
      and linear == 3
      and not (
          math.isfinite(sample.linear_velocity_x_m_s)
          and math.isfinite(sample.linear_velocity_y_m_s)
          and math.isfinite(sample.linear_velocity_z_m_s)
      )
  ):
    error = InsError.NON_FINITE
  angular = _present_count(
      sample.angular_velocity_x_present,
      sample.angular_velocity_y_present,
      sample.angular_velocity_z_present,
  )
  if error is InsError.NONE and angular not in (0, 3):
    error = InsError.ANGULAR_VELOCITY
  if (
      error is InsError.NONE
      and angular == 3
      and not (
          math.isfinite(sample.angular_velocity_x_rad_s)
          and math.isfinite(sample.angular_velocity_y_rad_s)
          and math.isfinite(sample.angular_velocity_z_rad_s)
      )
  ):
    error = InsError.NON_FINITE
  if (
      error is InsError.NONE
      and sample.source_present
      and not _known_source(source)
  ):
    error = InsError.SOURCE
  accepted = (
      error is InsError.NONE
      and state is measurement_health_policy.MeasurementStateKind.VALID
  )
  return InsAssessment(error, state, source, header_validity, accepted)
