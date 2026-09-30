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

"""Plain-value checks for one pressure and depth measurement.

Policy: intrinsic_hardware/intrinsic/hardware/marine/README.md.

These helpers do not parse protobuf, do not convert ENU and NED, do not
integrate hydrostatic pressure, and do not command hardware.

`depth_provenance` is the measurement-local enum
(DEPTH_PROVENANCE_UNSPECIFIED=0, DEPTH_PROVENANCE_DIRECT=1,
DEPTH_PROVENANCE_FROM_PRESSURE=2). Unset provenance is absent. A present
0 is UNSPECIFIED. Other numbers stay unrecognized.

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


class DepthProvenanceKind(enum.Enum):
  ABSENT = 0
  UNSPECIFIED = 1
  DIRECT = 2
  FROM_PRESSURE = 3
  UNRECOGNIZED = 4


class PressureDepthError(enum.Enum):
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
  DEPTH = 11
  PROVENANCE = 12
  DENSITY = 13


# Row-major Matrix6 diagonals. Index 0 is pressure variance (Pa^2).
# Index 7 is depth variance (m^2).
PRESSURE_VARIANCE_SLOT = vehicle_contract_policy.covariance_index(0, 0)
DEPTH_VARIANCE_SLOT = vehicle_contract_policy.covariance_index(1, 1)


@dataclass(frozen=True)
class PressureDepthAssessment:
  error: PressureDepthError
  state: measurement_health_policy.MeasurementStateKind
  provenance: DepthProvenanceKind
  header_validity: stamped_header_policy.ValidityKind
  accepted: bool


@dataclass(frozen=True)
class PressureDepthMeasurementView:
  health: measurement_health_policy.MeasurementHealthView = field(
      default_factory=measurement_health_policy.MeasurementHealthView
  )
  pressure_present: bool = False
  pressure_pa: float = 0.0
  depth_present: bool = False
  depth_m: float = 0.0
  provenance_present: bool = False
  depth_provenance: int = 0
  density_present: bool = False
  fluid_density_kg_m3: float = 0.0


def classify_depth_provenance(
    field_present: bool, provenance: int
) -> DepthProvenanceKind:
  """field_present is depth_provenance presence, not the enum value."""
  if not field_present:
    return DepthProvenanceKind.ABSENT
  return {
      0: DepthProvenanceKind.UNSPECIFIED,
      1: DepthProvenanceKind.DIRECT,
      2: DepthProvenanceKind.FROM_PRESSURE,
  }.get(provenance, DepthProvenanceKind.UNRECOGNIZED)


def pressure_depth_engaged(sample: PressureDepthMeasurementView) -> bool:
  return (
      measurement_health_policy.measurement_engaged(sample.health)
      or sample.pressure_present
      or sample.depth_present
      or sample.provenance_present
      or sample.density_present
  )


def usable_depth_provenance(provenance: DepthProvenanceKind) -> bool:
  return provenance in (
      DepthProvenanceKind.DIRECT,
      DepthProvenanceKind.FROM_PRESSURE,
  )


def allowed_covariance_slots_only(values: Sequence[float]) -> bool:
  if len(values) != vehicle_contract_policy.COVARIANCE_VALUES:
    return False
  for index, value in enumerate(values):
    if index in (PRESSURE_VARIANCE_SLOT, DEPTH_VARIANCE_SLOT):
      continue
    if value != 0.0:
      return False
  return True


def _from_measurement_error(error) -> PressureDepthError:
  return {
      measurement_health_policy.MeasurementError.NONE: PressureDepthError.NONE,
      measurement_health_policy.MeasurementError.MISSING_FRAME: (
          PressureDepthError.MISSING_FRAME
      ),
      measurement_health_policy.MeasurementError.WRONG_FRAME: (
          PressureDepthError.WRONG_FRAME
      ),
      measurement_health_policy.MeasurementError.TIME_REVERSAL: (
          PressureDepthError.TIME_REVERSAL
      ),
      measurement_health_policy.MeasurementError.NON_FINITE: (
          PressureDepthError.NON_FINITE
      ),
      measurement_health_policy.MeasurementError.QUALITY: (
          PressureDepthError.QUALITY
      ),
      measurement_health_policy.MeasurementError.COVARIANCE: (
          PressureDepthError.COVARIANCE
      ),
      measurement_health_policy.MeasurementError.SOURCE_ID: (
          PressureDepthError.SOURCE_ID
      ),
  }[error]


def assess_pressure_depth(
    sample: PressureDepthMeasurementView,
) -> PressureDepthAssessment:
  """First defect wins. Absent optional fields are not defects.

  Check order: missing health, #75 health order, covariance slots, payload
  presence, pressure finiteness, depth finiteness, depth sign, provenance,
  FROM_PRESSURE consistency, density. Health state is not rewritten.
  Pressure has no absolute floor beyond finiteness.
  """
  header_validity = stamped_header_policy.classify_validity(
      sample.health.header_validity_present,
      sample.health.header_validity_state,
  )
  provenance = classify_depth_provenance(
      sample.provenance_present, sample.depth_provenance
  )
  if not pressure_depth_engaged(sample):
    return PressureDepthAssessment(
        PressureDepthError.NONE,
        measurement_health_policy.MeasurementStateKind.ABSENT,
        DepthProvenanceKind.ABSENT,
        header_validity,
        False,
    )

  state = measurement_health_policy.MeasurementStateKind.ABSENT
  error = PressureDepthError.NONE
  if not measurement_health_policy.measurement_engaged(sample.health):
    error = PressureDepthError.MISSING_HEALTH
  else:
    health = measurement_health_policy.assess_measurement_health(sample.health)
    state = health.state
    error = _from_measurement_error(health.error)
    if (
        error is PressureDepthError.NONE
        and sample.health.covariance_present
        and vehicle_contract_policy.assess_covariance(
            True, sample.health.covariance
        )
        is vehicle_contract_policy.CovarianceError.NONE
        and not allowed_covariance_slots_only(sample.health.covariance)
    ):
      error = PressureDepthError.COVARIANCE_SLOTS
  if (
      error is PressureDepthError.NONE
      and not sample.pressure_present
      and not sample.depth_present
  ):
    error = PressureDepthError.PAYLOAD
  if (
      error is PressureDepthError.NONE
      and sample.pressure_present
      and not math.isfinite(sample.pressure_pa)
  ):
    error = PressureDepthError.NON_FINITE
  if error is PressureDepthError.NONE and sample.depth_present:
    if not math.isfinite(sample.depth_m):
      error = PressureDepthError.NON_FINITE
    elif sample.depth_m < 0.0:
      error = PressureDepthError.DEPTH
  if (
      error is PressureDepthError.NONE
      and sample.depth_present
      and not usable_depth_provenance(provenance)
  ):
    error = PressureDepthError.PROVENANCE
  if (
      error is PressureDepthError.NONE
      and provenance is DepthProvenanceKind.FROM_PRESSURE
      and not sample.depth_present
  ):
    error = PressureDepthError.PROVENANCE
  if (
      error is PressureDepthError.NONE
      and provenance is DepthProvenanceKind.FROM_PRESSURE
      and not sample.pressure_present
  ):
    error = PressureDepthError.PROVENANCE
  if (
      error is PressureDepthError.NONE
      and provenance is DepthProvenanceKind.FROM_PRESSURE
      and (
          not sample.density_present
          or not math.isfinite(sample.fluid_density_kg_m3)
          or sample.fluid_density_kg_m3 <= 0.0
      )
  ):
    error = PressureDepthError.DENSITY
  if error is PressureDepthError.NONE and sample.density_present:
    if not math.isfinite(sample.fluid_density_kg_m3):
      error = PressureDepthError.DENSITY
    elif sample.fluid_density_kg_m3 <= 0.0:
      error = PressureDepthError.DENSITY
  accepted = (
      error is PressureDepthError.NONE
      and state is measurement_health_policy.MeasurementStateKind.VALID
  )
  return PressureDepthAssessment(
      error, state, provenance, header_validity, accepted
  )
