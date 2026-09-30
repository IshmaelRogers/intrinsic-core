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

"""Plain-value checks for one altimeter measurement.

Policy: intrinsic_hardware/intrinsic/hardware/marine/README.md.

These helpers do not parse protobuf, do not convert ENU and NED, do not
fuse bathymetry, and do not command hardware.

Health checks are measurement_health_policy.assess_measurement_health.
This module does not rewrite health state or embodiment Validity.

Explicit no-return is has_return present and false. Unset has_return is
absent and is not no-return. A present range with unset has_return is a
range-only sample.
"""

from collections.abc import Sequence
from dataclasses import dataclass
from dataclasses import field
import enum
import math

from intrinsic.hardware.marine import measurement_health_policy

from intrinsic.embodiment import stamped_header_policy
from intrinsic.vehicle import vehicle_contract_policy


class AltimeterError(enum.Enum):
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
  PAYLOAD = 10
  RANGE = 11
  BOUNDS = 12
  NO_RETURN = 13
  BEAM_ID = 14


# Row-major Matrix6 diagonal. Index 0 is range variance (m^2).
RANGE_VARIANCE_SLOT = vehicle_contract_policy.covariance_index(0, 0)


@dataclass(frozen=True)
class AltimeterAssessment:
  error: AltimeterError
  state: measurement_health_policy.MeasurementStateKind
  header_validity: stamped_header_policy.ValidityKind
  accepted: bool


@dataclass(frozen=True)
class AltimeterMeasurementView:
  health: measurement_health_policy.MeasurementHealthView = field(
      default_factory=measurement_health_policy.MeasurementHealthView
  )
  range_present: bool = False
  range_m: float = 0.0
  beam_present: bool = False
  beam_id: str = ""
  min_present: bool = False
  min_range_m: float = 0.0
  max_present: bool = False
  max_range_m: float = 0.0
  has_return_present: bool = False
  has_return: bool = False


def explicit_no_return(sample: AltimeterMeasurementView) -> bool:
  return sample.has_return_present and not sample.has_return


def altimeter_engaged(sample: AltimeterMeasurementView) -> bool:
  return (
      measurement_health_policy.measurement_engaged(sample.health)
      or sample.range_present
      or sample.beam_present
      or sample.min_present
      or sample.max_present
      or sample.has_return_present
  )


def allowed_range_covariance_only(values: Sequence[float]) -> bool:
  if len(values) != vehicle_contract_policy.COVARIANCE_VALUES:
    return False
  for index, value in enumerate(values):
    if index == RANGE_VARIANCE_SLOT:
      continue
    if value != 0.0:
      return False
  return True


def _from_measurement_error(error) -> AltimeterError:
  return {
      measurement_health_policy.MeasurementError.NONE: AltimeterError.NONE,
      measurement_health_policy.MeasurementError.MISSING_FRAME: (
          AltimeterError.MISSING_FRAME
      ),
      measurement_health_policy.MeasurementError.WRONG_FRAME: (
          AltimeterError.WRONG_FRAME
      ),
      measurement_health_policy.MeasurementError.TIME_REVERSAL: (
          AltimeterError.TIME_REVERSAL
      ),
      measurement_health_policy.MeasurementError.NON_FINITE: (
          AltimeterError.NON_FINITE
      ),
      measurement_health_policy.MeasurementError.QUALITY: AltimeterError.QUALITY,
      measurement_health_policy.MeasurementError.COVARIANCE: (
          AltimeterError.COVARIANCE
      ),
      measurement_health_policy.MeasurementError.SOURCE_ID: (
          AltimeterError.SOURCE_ID
      ),
  }[error]


def assess_altimeter(sample: AltimeterMeasurementView) -> AltimeterAssessment:
  """First defect wins. Absent optional fields are not defects.

  Check order: missing health, #75 health order, covariance slots, payload
  presence, range finiteness, range sign, bound finiteness and sign,
  min <= max, range inside present bounds, no-return consistency, empty
  beam id. Health state is not rewritten.
  """
  header_validity = stamped_header_policy.classify_validity(
      sample.health.header_validity_present,
      sample.health.header_validity_state,
  )
  if not altimeter_engaged(sample):
    return AltimeterAssessment(
        AltimeterError.NONE,
        measurement_health_policy.MeasurementStateKind.ABSENT,
        header_validity,
        False,
    )

  state = measurement_health_policy.MeasurementStateKind.ABSENT
  error = AltimeterError.NONE
  if not measurement_health_policy.measurement_engaged(sample.health):
    error = AltimeterError.MISSING_HEALTH
  else:
    health = measurement_health_policy.assess_measurement_health(sample.health)
    state = health.state
    error = _from_measurement_error(health.error)
    if (
        error is AltimeterError.NONE
        and sample.health.covariance_present
        and vehicle_contract_policy.assess_covariance(
            True, sample.health.covariance
        )
        is vehicle_contract_policy.CovarianceError.NONE
        and not allowed_range_covariance_only(sample.health.covariance)
    ):
      error = AltimeterError.COVARIANCE_SLOTS
  if (
      error is AltimeterError.NONE
      and not sample.range_present
      and not explicit_no_return(sample)
  ):
    error = AltimeterError.PAYLOAD
  if error is AltimeterError.NONE and sample.range_present:
    if not math.isfinite(sample.range_m):
      error = AltimeterError.NON_FINITE
    elif sample.range_m < 0.0:
      error = AltimeterError.RANGE
  if error is AltimeterError.NONE and sample.min_present:
    if not math.isfinite(sample.min_range_m):
      error = AltimeterError.NON_FINITE
    elif sample.min_range_m < 0.0:
      error = AltimeterError.BOUNDS
  if error is AltimeterError.NONE and sample.max_present:
    if not math.isfinite(sample.max_range_m):
      error = AltimeterError.NON_FINITE
    elif sample.max_range_m < 0.0:
      error = AltimeterError.BOUNDS
  if (
      error is AltimeterError.NONE
      and sample.min_present
      and sample.max_present
      and sample.min_range_m > sample.max_range_m
  ):
    error = AltimeterError.BOUNDS
  if (
      error is AltimeterError.NONE
      and sample.range_present
      and sample.min_present
      and sample.range_m < sample.min_range_m
  ):
    error = AltimeterError.BOUNDS
  if (
      error is AltimeterError.NONE
      and sample.range_present
      and sample.max_present
      and sample.range_m > sample.max_range_m
  ):
    error = AltimeterError.BOUNDS
  if (
      error is AltimeterError.NONE
      and explicit_no_return(sample)
      and state is measurement_health_policy.MeasurementStateKind.VALID
  ):
    error = AltimeterError.NO_RETURN
  if (
      error is AltimeterError.NONE
      and explicit_no_return(sample)
      and sample.range_present
  ):
    error = AltimeterError.NO_RETURN
  if (
      error is AltimeterError.NONE
      and sample.beam_present
      and not sample.beam_id
  ):
    error = AltimeterError.BEAM_ID
  accepted = (
      error is AltimeterError.NONE
      and state is measurement_health_policy.MeasurementStateKind.VALID
  )
  return AltimeterAssessment(error, state, header_validity, accepted)
