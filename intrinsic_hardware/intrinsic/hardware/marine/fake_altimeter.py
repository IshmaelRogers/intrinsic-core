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

"""Deterministic altimeter producer.

Same config and truth yield the same bytes. `seed` is written to
`health.header.sequence` and mixes the noise draw. There is no bathymetry
fusion and no DVL altitude reuse.

Dropout returns None and is checked before no-return and out-of-range.
No-return sets has_return false, omits range, and sets health state
INVALID. It wins over out-of-range and noise. Out-of-range replaces range,
bounds, and has_return with the canonical fixture and sets INVALID. Noise
is not added on either INVALID fault.

A non-zero noise amplitude otherwise adds
`noise_amplitude_m * altimeter_signed_unit_noise(seed)` to a present range
and sets health state DEGRADED. The numeric result is not repaired when it
is negative or outside the bounds. Delay is added to source time to form
receive time and is not repaired when it reverses the stamps.
"""

from dataclasses import dataclass
from typing import Optional

from intrinsic.hardware.marine import altimeter_pb2
from intrinsic.hardware.marine import measurement_health_pb2

from intrinsic.embodiment import stamped_header_policy
from intrinsic.embodiment.proto import stamped_header_pb2
from intrinsic.vehicle import vehicle_contract_policy

_NANOS_PER_SECOND = 1000000000
_MASK64 = (1 << 64) - 1

# Canonical out-of-range fixture. 101 m is strictly above max 100 m.
CANONICAL_MIN_RANGE_M = 0.5
CANONICAL_MAX_RANGE_M = 100.0
CANONICAL_OUT_OF_RANGE_RANGE_M = 101.0


def altimeter_signed_unit_noise(seed: int) -> float:
  """SplitMix64 of seed, mapped onto [-1, 1).

  The 53-bit fraction is exact in IEEE-754 binary64. The C++ fake uses the
  same mix.
  """
  mixed = (seed + 0x9E3779B97F4A7C15) & _MASK64
  mixed = ((mixed ^ (mixed >> 30)) * 0xBF58476D1CE4E5B9) & _MASK64
  mixed = ((mixed ^ (mixed >> 27)) * 0x94D049BB133111EB) & _MASK64
  mixed = (mixed ^ (mixed >> 31)) & _MASK64
  unit = float(mixed >> 11) * (2.0**-53)
  return unit * 2.0 - 1.0


@dataclass(frozen=True)
class FakeAltimeterConfig:
  seed: int = 42
  noise_amplitude_m: float = 0.0
  delay_seconds: int = 1
  delay_nanos: int = -250000000
  dropout: bool = False
  no_return: bool = False
  out_of_range: bool = False


@dataclass(frozen=True)
class AltimeterTruth:
  frame_id: str = "sensor"
  source_seconds: int = 1700000000
  source_nanos: int = 250000000
  range_present: bool = True
  range_m: float = 10.0
  beam_present: bool = True
  beam_id: str = "down"
  min_present: bool = True
  min_range_m: float = CANONICAL_MIN_RANGE_M
  max_present: bool = True
  max_range_m: float = CANONICAL_MAX_RANGE_M
  has_return_present: bool = True
  has_return: bool = True
  quality: float = 0.75
  covariance_present: bool = True


def _div_toward_zero(value, divisor):
  if value < 0:
    return -((-value) // divisor)
  return value // divisor


def _add_clock_delay(seconds, nanos, delay_seconds, delay_nanos):
  """Return (seconds, nanos) after adding a delay. Nanos land in [0, 1e9)."""
  total_nanos = nanos + delay_nanos
  total_seconds = seconds + delay_seconds
  carry = _div_toward_zero(total_nanos, _NANOS_PER_SECOND)
  total_nanos -= carry * _NANOS_PER_SECOND
  if total_nanos < 0:
    total_nanos += _NANOS_PER_SECOND
    carry -= 1
  return total_seconds + carry, total_nanos


def _noise_engaged(config: FakeAltimeterConfig) -> bool:
  return config.noise_amplitude_m != 0.0


def _fill_range_covariance(matrix) -> None:
  values = [0.0] * vehicle_contract_policy.COVARIANCE_VALUES
  values[vehicle_contract_policy.covariance_index(0, 0)] = 0.25
  matrix.values[:] = values


def _copy_beam_and_bounds(truth: AltimeterTruth, sample) -> None:
  if truth.beam_present:
    sample.beam_id = truth.beam_id
  if truth.min_present:
    sample.min_range_m = truth.min_range_m
  if truth.max_present:
    sample.max_range_m = truth.max_range_m


class FakeAltimeter:
  """Fixed-seed altimeter. Default config and truth are the nominal."""

  def __init__(self, config: Optional[FakeAltimeterConfig] = None):
    self._config = config if config is not None else FakeAltimeterConfig()

  @property
  def config(self) -> FakeAltimeterConfig:
    return self._config

  def measure(self, truth: Optional[AltimeterTruth] = None):
    """Return an AltimeterMeasurement, or None when dropout is set."""
    if self._config.dropout:
      return None
    if truth is None:
      truth = AltimeterTruth()
    sample = altimeter_pb2.AltimeterMeasurement()
    health = sample.health
    header = health.header
    header.sequence = self._config.seed
    header.source_time.seconds = truth.source_seconds
    header.source_time.nanos = truth.source_nanos
    receive_seconds, receive_nanos = _add_clock_delay(
        truth.source_seconds,
        truth.source_nanos,
        self._config.delay_seconds,
        self._config.delay_nanos,
    )
    header.receive_time.seconds = receive_seconds
    header.receive_time.nanos = receive_nanos
    header.source_id = "nav_sensor"
    header.frame_id = truth.frame_id
    header.clock_domain = stamped_header_policy.CLOCK_DOMAIN_MONOTONIC
    header.validity.state = stamped_header_pb2.Validity.STATE_VALID

    state = measurement_health_pb2.MeasurementHealth
    if self._config.no_return or self._config.out_of_range:
      health.state = state.INVALID
    elif _noise_engaged(self._config):
      health.state = state.DEGRADED
    else:
      health.state = state.VALID
    health.quality = truth.quality
    if truth.covariance_present:
      _fill_range_covariance(health.covariance)
    primary = health.sources.add()
    primary.source_id = "primary"
    primary.validity.state = stamped_header_pb2.Validity.STATE_VALID
    health.sources.add().source_id = "aiding"

    if self._config.no_return:
      sample.has_return = False
      _copy_beam_and_bounds(truth, sample)
    elif self._config.out_of_range:
      sample.range_m = CANONICAL_OUT_OF_RANGE_RANGE_M
      sample.min_range_m = CANONICAL_MIN_RANGE_M
      sample.max_range_m = CANONICAL_MAX_RANGE_M
      sample.has_return = True
      if truth.beam_present:
        sample.beam_id = truth.beam_id
    else:
      if truth.range_present:
        range_m = truth.range_m
        if _noise_engaged(self._config):
          range_m += (
              self._config.noise_amplitude_m
              * altimeter_signed_unit_noise(self._config.seed)
          )
        sample.range_m = range_m
      _copy_beam_and_bounds(truth, sample)
      if truth.has_return_present:
        sample.has_return = truth.has_return
    return sample
