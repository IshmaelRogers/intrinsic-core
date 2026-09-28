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

"""Clock, sequence, and validity rules for StampedHeader.

Policy: intrinsic_apis/intrinsic/embodiment/proto/README.md.
"""

import enum

CLOCK_DOMAIN_UTC = "utc"
CLOCK_DOMAIN_MONOTONIC = "monotonic"
NANOS_PER_SECOND = 1000000000

ClockReading = tuple[int, int]


class ValidityKind(enum.Enum):
  ABSENT = 0
  UNSPECIFIED = 1
  VALID = 2
  INVALID = 3


def classify_validity(field_present: bool, state: int) -> ValidityKind:
  """field_present is StampedHeader.validity presence, not the enum value."""
  if not field_present:
    return ValidityKind.ABSENT
  if state == 1:
    return ValidityKind.VALID
  if state == 2:
    return ValidityKind.INVALID
  return ValidityKind.UNSPECIFIED


def sample_accepted(kind: ValidityKind, values_finite: bool) -> bool:
  """A usable sample is explicitly valid and finite."""
  return kind is ValidityKind.VALID and values_finite


def is_utc_clock_domain(clock_domain: str) -> bool:
  return clock_domain == CLOCK_DOMAIN_UTC


def is_monotonic_clock_domain(clock_domain: str) -> bool:
  return clock_domain == CLOCK_DOMAIN_MONOTONIC


def sequence_advances(previous: int, current: int) -> bool:
  return current > previous


def nanos_in_range(nanos: int) -> bool:
  return 0 <= nanos < NANOS_PER_SECOND


def monotonic_age_seconds(
    source: ClockReading | None,
    receive: ClockReading | None,
    clock_domain: str,
) -> float | None:
  """Age in seconds for a monotonic clock only.

  utc and every other domain return None so wall clock cannot feed a
  watchdog through this helper.
  """
  if (
      not is_monotonic_clock_domain(clock_domain)
      or source is None
      or receive is None
      or not nanos_in_range(source[1])
      or not nanos_in_range(receive[1])
  ):
    return None
  if receive[0] < source[0] or (
      receive[0] == source[0] and receive[1] < source[1]
  ):
    return None
  seconds = receive[0] - source[0]
  nanos = receive[1] - source[1]
  if nanos < 0:
    seconds -= 1
    nanos += NANOS_PER_SECOND
  return float(seconds) + (float(nanos) / float(NANOS_PER_SECOND))
