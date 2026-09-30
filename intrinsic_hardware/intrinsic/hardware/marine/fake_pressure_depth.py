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

"""Deterministic pressure and depth producer.

Same config and truth yield the same bytes. `seed` is written to
`health.header.sequence`. It does not draw noise and it does not integrate
hydrostatic pressure.

Dropout returns None. Otherwise out-of-range forces depth_m to -1 and
health state INVALID. A non-zero bias without out-of-range sets health
state DEGRADED and adds the bias to each present truth field. Delay is
added to source time to form receive time and is not repaired when it
reverses the stamps.
"""

from dataclasses import dataclass
from typing import Optional

from intrinsic.hardware.marine import measurement_health_pb2
from intrinsic.hardware.marine import pressure_depth_pb2

from intrinsic.embodiment import stamped_header_policy
from intrinsic.embodiment.proto import stamped_header_pb2
from intrinsic.vehicle import vehicle_contract_policy

_NANOS_PER_SECOND = 1000000000

# Canonical out-of-range depth, meters. Negative is above the free surface.
CANONICAL_OUT_OF_RANGE_DEPTH_M = -1.0


@dataclass(frozen=True)
class FakePressureDepthConfig:
  seed: int = 42
  pressure_bias_pa: float = 0.0
  depth_bias_m: float = 0.0
  delay_seconds: int = 1
  delay_nanos: int = -250000000
  dropout: bool = False
  out_of_range: bool = False


@dataclass(frozen=True)
class PressureDepthTruth:
  frame_id: str = "sensor"
  source_seconds: int = 1700000000
  source_nanos: int = 250000000
  pressure_present: bool = True
  pressure_pa: float = 200000.0
  depth_present: bool = True
  depth_m: float = 10.0
  provenance_present: bool = True
  depth_provenance: int = 2
  density_present: bool = True
  fluid_density_kg_m3: float = 1025.0
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


def _bias_engaged(config: FakePressureDepthConfig) -> bool:
  return config.pressure_bias_pa != 0.0 or config.depth_bias_m != 0.0


def _fill_pressure_depth_covariance(matrix) -> None:
  values = [0.0] * vehicle_contract_policy.COVARIANCE_VALUES
  values[vehicle_contract_policy.covariance_index(0, 0)] = 1.0
  values[vehicle_contract_policy.covariance_index(1, 1)] = 0.25
  matrix.values[:] = values


class FakePressureDepth:
  """Fixed-seed pressure/depth. Default config and truth are the nominal."""

  def __init__(self, config: Optional[FakePressureDepthConfig] = None):
    self._config = config if config is not None else FakePressureDepthConfig()

  @property
  def config(self) -> FakePressureDepthConfig:
    return self._config

  def measure(self, truth: Optional[PressureDepthTruth] = None):
    """Return a PressureDepthMeasurement, or None when dropout is set."""
    if self._config.dropout:
      return None
    if truth is None:
      truth = PressureDepthTruth()
    sample = pressure_depth_pb2.PressureDepthMeasurement()
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
    if self._config.out_of_range:
      health.state = state.INVALID
    elif _bias_engaged(self._config):
      health.state = state.DEGRADED
    else:
      health.state = state.VALID
    health.quality = truth.quality
    if truth.covariance_present:
      _fill_pressure_depth_covariance(health.covariance)
    primary = health.sources.add()
    primary.source_id = "primary"
    primary.validity.state = stamped_header_pb2.Validity.STATE_VALID
    health.sources.add().source_id = "aiding"

    if truth.pressure_present:
      sample.pressure_pa = truth.pressure_pa + self._config.pressure_bias_pa
    if self._config.out_of_range:
      sample.depth_m = CANONICAL_OUT_OF_RANGE_DEPTH_M
    elif truth.depth_present:
      sample.depth_m = truth.depth_m + self._config.depth_bias_m
    if truth.provenance_present:
      sample.depth_provenance = truth.depth_provenance
    if truth.density_present:
      sample.fluid_density_kg_m3 = truth.fluid_density_kg_m3
    return sample
