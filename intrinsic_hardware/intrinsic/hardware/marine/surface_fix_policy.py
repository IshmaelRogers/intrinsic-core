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

"""Plain-value checks for one surface position fix.

Policy: intrinsic_hardware/intrinsic/hardware/marine/README.md.

These helpers do not parse protobuf, do not convert lat/lon, WGS84, ENU, or
NED, do not decide whether the vehicle is submerged, do not read INS
solutions, and do not command hardware.

Health checks are measurement_health_policy.assess_measurement_health.
This module does not rewrite health state or embodiment Validity.

`source` is the measurement-local enum (FIX_SOURCE_UNSPECIFIED=0,
FIX_SOURCE_GNSS=1, FIX_SOURCE_ACOUSTIC=2, FIX_SOURCE_OTHER=3). Unset source
is absent. A present 0 is FIX_SOURCE_UNSPECIFIED. Other numbers stay
unrecognized. An engaged sample needs a known source.
"""

from collections.abc import Sequence
from dataclasses import dataclass
from dataclasses import field
import enum
import math

from intrinsic.hardware.marine import measurement_health_policy

from intrinsic.embodiment import stamped_header_policy
from intrinsic.vehicle import vehicle_contract_policy

POSITION_X_VARIANCE_SLOT = vehicle_contract_policy.covariance_index(0, 0)
POSITION_Y_VARIANCE_SLOT = vehicle_contract_policy.covariance_index(1, 1)
POSITION_Z_VARIANCE_SLOT = vehicle_contract_policy.covariance_index(2, 2)

_SURFACE_FIX_VARIANCE_SLOTS = (
    POSITION_X_VARIANCE_SLOT,
    POSITION_Y_VARIANCE_SLOT,
    POSITION_Z_VARIANCE_SLOT,
)


class SurfaceFixSourceKind(enum.Enum):
  ABSENT = 0
  UNSPECIFIED = 1
  GNSS = 2
  ACOUSTIC = 3
  OTHER = 4
  UNRECOGNIZED = 5


class SurfaceFixError(enum.Enum):
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
  SOURCE = 11
  ACCURACY = 12
  COUNT = 13
  VELOCITY = 14


@dataclass(frozen=True)
class SurfaceFixAssessment:
  error: SurfaceFixError
  state: measurement_health_policy.MeasurementStateKind
  source: SurfaceFixSourceKind
  header_validity: stamped_header_policy.ValidityKind
  accepted: bool


@dataclass(frozen=True)
class SurfaceFixView:
  health: measurement_health_policy.MeasurementHealthView = field(
      default_factory=measurement_health_policy.MeasurementHealthView
  )
  position_x_present: bool = False
  position_x_m: float = 0.0
  position_y_present: bool = False
  position_y_m: float = 0.0
  position_z_present: bool = False
  position_z_m: float = 0.0
  source_present: bool = False
  source: int = 0
  satellite_count_present: bool = False
  satellite_count: int = 0
  beacon_count_present: bool = False
  beacon_count: int = 0
  horizontal_accuracy_present: bool = False
  horizontal_accuracy_m: float = 0.0
  vertical_accuracy_present: bool = False
  vertical_accuracy_m: float = 0.0
  velocity_x_present: bool = False
  velocity_x_m_s: float = 0.0
  velocity_y_present: bool = False
  velocity_y_m_s: float = 0.0
  velocity_z_present: bool = False
  velocity_z_m_s: float = 0.0


def classify_surface_fix_source(
    field_present: bool, source: int
) -> SurfaceFixSourceKind:
  """field_present is SurfaceFix.source presence, not the enum value."""
  if not field_present:
    return SurfaceFixSourceKind.ABSENT
  return {
      0: SurfaceFixSourceKind.UNSPECIFIED,
      1: SurfaceFixSourceKind.GNSS,
      2: SurfaceFixSourceKind.ACOUSTIC,
      3: SurfaceFixSourceKind.OTHER,
  }.get(source, SurfaceFixSourceKind.UNRECOGNIZED)


def surface_fix_engaged(sample: SurfaceFixView) -> bool:
  return (
      measurement_health_policy.measurement_engaged(sample.health)
      or sample.position_x_present
      or sample.position_y_present
      or sample.position_z_present
      or sample.source_present
      or sample.satellite_count_present
      or sample.beacon_count_present
      or sample.horizontal_accuracy_present
      or sample.vertical_accuracy_present
      or sample.velocity_x_present
      or sample.velocity_y_present
      or sample.velocity_z_present
  )


def allowed_surface_fix_covariance_only(values: Sequence[float]) -> bool:
  if len(values) != vehicle_contract_policy.COVARIANCE_VALUES:
    return False
  allowed = set(_SURFACE_FIX_VARIANCE_SLOTS)
  for index, value in enumerate(values):
    if index not in allowed and value != 0.0:
      return False
  return True


def _known_source(source: SurfaceFixSourceKind) -> bool:
  return source in (
      SurfaceFixSourceKind.GNSS,
      SurfaceFixSourceKind.ACOUSTIC,
      SurfaceFixSourceKind.OTHER,
  )


def _from_measurement_error(error) -> SurfaceFixError:
  return {
      measurement_health_policy.MeasurementError.NONE: SurfaceFixError.NONE,
      measurement_health_policy.MeasurementError.MISSING_FRAME: (
          SurfaceFixError.MISSING_FRAME
      ),
      measurement_health_policy.MeasurementError.WRONG_FRAME: (
          SurfaceFixError.WRONG_FRAME
      ),
      measurement_health_policy.MeasurementError.TIME_REVERSAL: (
          SurfaceFixError.TIME_REVERSAL
      ),
      measurement_health_policy.MeasurementError.NON_FINITE: (
          SurfaceFixError.NON_FINITE
      ),
      measurement_health_policy.MeasurementError.QUALITY: (
          SurfaceFixError.QUALITY
      ),
      measurement_health_policy.MeasurementError.COVARIANCE: (
          SurfaceFixError.COVARIANCE
      ),
      measurement_health_policy.MeasurementError.SOURCE_ID: (
          SurfaceFixError.SOURCE_ID
      ),
  }[error]


def _present_count(x: bool, y: bool, z: bool) -> int:
  return int(x) + int(y) + int(z)


def assess_surface_fix(sample: SurfaceFixView) -> SurfaceFixAssessment:
  """First defect wins. Absent optional fields are not defects.

  Check order: missing health, #75 health order, covariance slots, position
  presence, position finiteness, source kind, accuracy finiteness and sign,
  satellite and beacon count sign, velocity partial triple, velocity
  finiteness. Health state is not rewritten, so a producer VALID sample with
  a missing source is rejected and stays VALID.
  """
  header_validity = stamped_header_policy.classify_validity(
      sample.health.header_validity_present,
      sample.health.header_validity_state,
  )
  source = classify_surface_fix_source(sample.source_present, sample.source)
  if not surface_fix_engaged(sample):
    return SurfaceFixAssessment(
        SurfaceFixError.NONE,
        measurement_health_policy.MeasurementStateKind.ABSENT,
        SurfaceFixSourceKind.ABSENT,
        header_validity,
        False,
    )

  state = measurement_health_policy.MeasurementStateKind.ABSENT
  error = SurfaceFixError.NONE
  if not measurement_health_policy.measurement_engaged(sample.health):
    error = SurfaceFixError.MISSING_HEALTH
  else:
    health = measurement_health_policy.assess_measurement_health(sample.health)
    state = health.state
    error = _from_measurement_error(health.error)
    if (
        error is SurfaceFixError.NONE
        and sample.health.covariance_present
        and vehicle_contract_policy.assess_covariance(
            True, sample.health.covariance
        )
        is vehicle_contract_policy.CovarianceError.NONE
        and not allowed_surface_fix_covariance_only(sample.health.covariance)
    ):
      error = SurfaceFixError.COVARIANCE_SLOTS
  if error is SurfaceFixError.NONE and not (
      sample.position_x_present
      and sample.position_y_present
      and sample.position_z_present
  ):
    error = SurfaceFixError.POSITION
  if error is SurfaceFixError.NONE and not (
      math.isfinite(sample.position_x_m)
      and math.isfinite(sample.position_y_m)
      and math.isfinite(sample.position_z_m)
  ):
    error = SurfaceFixError.NON_FINITE
  if error is SurfaceFixError.NONE and not _known_source(source):
    error = SurfaceFixError.SOURCE
  if error is SurfaceFixError.NONE and sample.horizontal_accuracy_present:
    if not math.isfinite(sample.horizontal_accuracy_m):
      error = SurfaceFixError.NON_FINITE
    elif sample.horizontal_accuracy_m < 0.0:
      error = SurfaceFixError.ACCURACY
  if error is SurfaceFixError.NONE and sample.vertical_accuracy_present:
    if not math.isfinite(sample.vertical_accuracy_m):
      error = SurfaceFixError.NON_FINITE
    elif sample.vertical_accuracy_m < 0.0:
      error = SurfaceFixError.ACCURACY
  if (
      error is SurfaceFixError.NONE
      and sample.satellite_count_present
      and sample.satellite_count < 0
  ):
    error = SurfaceFixError.COUNT
  if (
      error is SurfaceFixError.NONE
      and sample.beacon_count_present
      and sample.beacon_count < 0
  ):
    error = SurfaceFixError.COUNT
  velocity = _present_count(
      sample.velocity_x_present,
      sample.velocity_y_present,
      sample.velocity_z_present,
  )
  if error is SurfaceFixError.NONE and velocity not in (0, 3):
    error = SurfaceFixError.VELOCITY
  if (
      error is SurfaceFixError.NONE
      and velocity == 3
      and not (
          math.isfinite(sample.velocity_x_m_s)
          and math.isfinite(sample.velocity_y_m_s)
          and math.isfinite(sample.velocity_z_m_s)
      )
  ):
    error = SurfaceFixError.NON_FINITE
  accepted = (
      error is SurfaceFixError.NONE
      and state is measurement_health_policy.MeasurementStateKind.VALID
  )
  return SurfaceFixAssessment(error, state, source, header_validity, accepted)
