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

"""Plain-value checks for one DVL measurement.

Policy: intrinsic_hardware/intrinsic/hardware/marine/README.md.

These helpers do not parse protobuf, do not convert ENU and NED, and do
not command hardware.

`mode` is the measurement-local enum (MODE_UNSPECIFIED=0,
MODE_BOTTOM_TRACK=1, MODE_WATER_TRACK=2). Unset mode is absent. A present
0 is MODE_UNSPECIFIED. Other numbers stay unrecognized.

Health checks are measurement_health_policy.assess_measurement_health.
This module does not rewrite health state or embodiment Validity.
"""

from collections.abc import Sequence
from dataclasses import dataclass
from dataclasses import field
import enum
import math

from intrinsic.hardware.marine import measurement_health_policy

from intrinsic.embodiment import stamped_header_policy
from intrinsic.vehicle import vehicle_contract_policy


class DvlModeKind(enum.Enum):
  ABSENT = 0
  UNSPECIFIED = 1
  BOTTOM_TRACK = 2
  WATER_TRACK = 3
  UNRECOGNIZED = 4


class DvlError(enum.Enum):
  NONE = 0
  MISSING_HEALTH = 1
  MISSING_FRAME = 2
  WRONG_FRAME = 3
  TIME_REVERSAL = 4
  NON_FINITE = 5
  QUALITY = 6
  COVARIANCE = 7
  SOURCE_ID = 8
  MODE = 9
  VELOCITY = 10
  LOCK = 11
  ALTITUDE = 12
  ANGULAR_COVARIANCE = 13


# Row-major angular variance slots. Linear x, y, z occupy the leading 3x3.
ANGULAR_VARIANCE_SLOTS = (
    vehicle_contract_policy.covariance_index(3, 3),
    vehicle_contract_policy.covariance_index(4, 4),
    vehicle_contract_policy.covariance_index(5, 5),
)


@dataclass(frozen=True)
class DvlAssessment:
  error: DvlError
  state: measurement_health_policy.MeasurementStateKind
  mode: DvlModeKind
  header_validity: stamped_header_policy.ValidityKind
  accepted: bool


@dataclass(frozen=True)
class DvlMeasurementView:
  health: measurement_health_policy.MeasurementHealthView = field(
      default_factory=measurement_health_policy.MeasurementHealthView
  )
  mode_present: bool = False
  mode: int = 0
  velocity_x_present: bool = False
  velocity_x_m_s: float = 0.0
  velocity_y_present: bool = False
  velocity_y_m_s: float = 0.0
  velocity_z_present: bool = False
  velocity_z_m_s: float = 0.0
  bottom_lock_present: bool = False
  bottom_lock: bool = False
  altitude_present: bool = False
  altitude_m: float = 0.0


def classify_dvl_mode(field_present: bool, mode: int) -> DvlModeKind:
  """field_present is DvlMeasurement.mode presence, not the enum value."""
  if not field_present:
    return DvlModeKind.ABSENT
  return {
      0: DvlModeKind.UNSPECIFIED,
      1: DvlModeKind.BOTTOM_TRACK,
      2: DvlModeKind.WATER_TRACK,
  }.get(mode, DvlModeKind.UNRECOGNIZED)


def dvl_engaged(sample: DvlMeasurementView) -> bool:
  return (
      measurement_health_policy.measurement_engaged(sample.health)
      or sample.mode_present
      or sample.velocity_x_present
      or sample.velocity_y_present
      or sample.velocity_z_present
      or sample.bottom_lock_present
      or sample.altitude_present
  )


def dvl_track_mode(mode: DvlModeKind) -> bool:
  return mode in (DvlModeKind.BOTTOM_TRACK, DvlModeKind.WATER_TRACK)


def angular_variances_are_zero(values: Sequence[float]) -> bool:
  if len(values) <= ANGULAR_VARIANCE_SLOTS[-1]:
    return False
  return all(values[index] == 0.0 for index in ANGULAR_VARIANCE_SLOTS)


def _from_measurement_error(error) -> DvlError:
  return {
      measurement_health_policy.MeasurementError.NONE: DvlError.NONE,
      measurement_health_policy.MeasurementError.MISSING_FRAME: (
          DvlError.MISSING_FRAME
      ),
      measurement_health_policy.MeasurementError.WRONG_FRAME: (
          DvlError.WRONG_FRAME
      ),
      measurement_health_policy.MeasurementError.TIME_REVERSAL: (
          DvlError.TIME_REVERSAL
      ),
      measurement_health_policy.MeasurementError.NON_FINITE: (
          DvlError.NON_FINITE
      ),
      measurement_health_policy.MeasurementError.QUALITY: DvlError.QUALITY,
      measurement_health_policy.MeasurementError.COVARIANCE: (
          DvlError.COVARIANCE
      ),
      measurement_health_policy.MeasurementError.SOURCE_ID: DvlError.SOURCE_ID,
  }[error]


def assess_dvl(sample: DvlMeasurementView) -> DvlAssessment:
  """First defect wins. Absent optional fields are not defects.

  Check order: missing health, #75 health order, angular variance slots,
  mode, velocity presence, velocity finiteness, bottom-lock inconsistency,
  altitude finiteness, altitude sign. Health state is not rewritten.
  """
  header_validity = stamped_header_policy.classify_validity(
      sample.health.header_validity_present,
      sample.health.header_validity_state,
  )
  mode = classify_dvl_mode(sample.mode_present, sample.mode)
  if not dvl_engaged(sample):
    return DvlAssessment(
        DvlError.NONE,
        measurement_health_policy.MeasurementStateKind.ABSENT,
        DvlModeKind.ABSENT,
        header_validity,
        False,
    )

  state = measurement_health_policy.MeasurementStateKind.ABSENT
  error = DvlError.NONE
  if not measurement_health_policy.measurement_engaged(sample.health):
    error = DvlError.MISSING_HEALTH
  else:
    health = measurement_health_policy.assess_measurement_health(sample.health)
    state = health.state
    error = _from_measurement_error(health.error)
    if (
        error is DvlError.NONE
        and sample.health.covariance_present
        and vehicle_contract_policy.assess_covariance(
            True, sample.health.covariance
        )
        is vehicle_contract_policy.CovarianceError.NONE
        and not angular_variances_are_zero(sample.health.covariance)
    ):
      error = DvlError.ANGULAR_COVARIANCE
  if error is DvlError.NONE and not dvl_track_mode(mode):
    error = DvlError.MODE
  if error is DvlError.NONE:
    if not (
        sample.velocity_x_present
        and sample.velocity_y_present
        and sample.velocity_z_present
    ):
      error = DvlError.VELOCITY
    elif not (
        math.isfinite(sample.velocity_x_m_s)
        and math.isfinite(sample.velocity_y_m_s)
        and math.isfinite(sample.velocity_z_m_s)
    ):
      error = DvlError.NON_FINITE
  if (
      error is DvlError.NONE
      and mode is DvlModeKind.BOTTOM_TRACK
      and sample.bottom_lock_present
      and not sample.bottom_lock
      and state is measurement_health_policy.MeasurementStateKind.VALID
  ):
    error = DvlError.LOCK
  if error is DvlError.NONE and sample.altitude_present:
    if not math.isfinite(sample.altitude_m):
      error = DvlError.NON_FINITE
    elif sample.altitude_m < 0.0:
      error = DvlError.ALTITUDE
  accepted = (
      error is DvlError.NONE
      and state is measurement_health_policy.MeasurementStateKind.VALID
      and dvl_track_mode(mode)
  )
  return DvlAssessment(error, state, mode, header_validity, accepted)
