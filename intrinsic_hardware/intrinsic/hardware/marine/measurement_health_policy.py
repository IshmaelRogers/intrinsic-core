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

"""Plain-value checks for the marine measurement health envelope.

Policy: intrinsic_hardware/intrinsic/hardware/marine/README.md.

These helpers do not parse protobuf, do not convert ENU and NED, and do
not command hardware.

`state` is the measurement-local enum (UNKNOWN=0, VALID=1, DEGRADED=2,
INVALID=3). It is not embodiment Validity. Unset state is an absent
judgment. A present 0 is UNKNOWN. Numbers outside 0..3 stay unrecognized.

Covariance shape is vehicle.assess_covariance (#17).
"""

from collections.abc import Sequence
from dataclasses import dataclass
import enum
import math

from intrinsic.embodiment import stamped_header_policy
from intrinsic.vehicle import vehicle_contract_policy


class MeasurementStateKind(enum.Enum):
  ABSENT = 0
  UNKNOWN = 1
  VALID = 2
  DEGRADED = 3
  INVALID = 4
  UNRECOGNIZED = 5


class MeasurementError(enum.Enum):
  NONE = 0
  MISSING_FRAME = 1
  WRONG_FRAME = 2
  TIME_REVERSAL = 3
  NON_FINITE = 4
  QUALITY = 5
  COVARIANCE = 6
  SOURCE_ID = 7


@dataclass(frozen=True)
class MeasurementAssessment:
  error: MeasurementError
  state: MeasurementStateKind
  header_validity: stamped_header_policy.ValidityKind
  accepted: bool


@dataclass(frozen=True)
class SourceHealthView:
  source_id: str = ""
  validity_present: bool = False
  validity_state: int = 0


@dataclass(frozen=True)
class MeasurementHealthView:
  header_present: bool = False
  frame_id: str = ""
  expected_frame_present: bool = False
  expected_frame: str = ""
  source_time_present: bool = False
  source_time: stamped_header_policy.ClockReading = (0, 0)
  receive_time_present: bool = False
  receive_time: stamped_header_policy.ClockReading = (0, 0)
  header_validity_present: bool = False
  header_validity_state: int = 0
  state_present: bool = False
  state: int = 0
  quality_present: bool = False
  quality: float = 0.0
  covariance_present: bool = False
  covariance: Sequence[float] = ()
  sources: tuple[SourceHealthView, ...] = ()


def classify_measurement_state(
    field_present: bool, state: int
) -> MeasurementStateKind:
  """field_present is MeasurementHealth.state presence, not the enum value."""
  if not field_present:
    return MeasurementStateKind.ABSENT
  return {
      0: MeasurementStateKind.UNKNOWN,
      1: MeasurementStateKind.VALID,
      2: MeasurementStateKind.DEGRADED,
      3: MeasurementStateKind.INVALID,
  }.get(state, MeasurementStateKind.UNRECOGNIZED)


def measurement_engaged(sample: MeasurementHealthView) -> bool:
  return (
      sample.header_present
      or sample.state_present
      or sample.quality_present
      or sample.covariance_present
      or bool(sample.sources)
      or sample.source_time_present
      or sample.receive_time_present
  )


def timestamp_precedes(
    lhs: stamped_header_policy.ClockReading,
    rhs: stamped_header_policy.ClockReading,
) -> bool:
  """True when lhs is strictly earlier than rhs in (seconds, nanos) order."""
  if lhs[0] != rhs[0]:
    return lhs[0] < rhs[0]
  return lhs[1] < rhs[1]


def quality_in_range(quality: float) -> bool:
  return math.isfinite(quality) and quality >= 0.0 and quality <= 1.0


def assess_measurement_health(
    sample: MeasurementHealthView,
) -> MeasurementAssessment:
  """First defect wins. Absent optional fields are not defects.

  Check order: frame, caller frame, source-time reversal, quality
  finiteness, quality range, covariance shape, source ids. Header
  Validity does not select `accepted`.
  """
  header_validity = stamped_header_policy.classify_validity(
      sample.header_validity_present, sample.header_validity_state
  )
  if not measurement_engaged(sample):
    return MeasurementAssessment(
        MeasurementError.NONE,
        MeasurementStateKind.ABSENT,
        header_validity,
        False,
    )
  state = classify_measurement_state(sample.state_present, sample.state)
  error = MeasurementError.NONE
  if not sample.frame_id:
    error = MeasurementError.MISSING_FRAME
  elif (
      sample.expected_frame_present and sample.frame_id != sample.expected_frame
  ):
    error = MeasurementError.WRONG_FRAME
  elif (
      sample.source_time_present
      and sample.receive_time_present
      and timestamp_precedes(sample.receive_time, sample.source_time)
  ):
    error = MeasurementError.TIME_REVERSAL
  elif sample.quality_present and not math.isfinite(sample.quality):
    error = MeasurementError.NON_FINITE
  elif sample.quality_present and not quality_in_range(sample.quality):
    error = MeasurementError.QUALITY
  else:
    covariance = vehicle_contract_policy.assess_covariance(
        sample.covariance_present, sample.covariance
    )
    if covariance not in (
        vehicle_contract_policy.CovarianceError.NONE,
        vehicle_contract_policy.CovarianceError.ABSENT,
    ):
      error = MeasurementError.COVARIANCE
  if error is MeasurementError.NONE:
    for source in sample.sources:
      if not source.source_id:
        error = MeasurementError.SOURCE_ID
        break
  accepted = (
      error is MeasurementError.NONE and state is MeasurementStateKind.VALID
  )
  return MeasurementAssessment(error, state, header_validity, accepted)
