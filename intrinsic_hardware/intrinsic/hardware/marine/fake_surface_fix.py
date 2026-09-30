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

"""Deterministic surface-fix producer.

Same config and truth yield the same bytes. `seed` is written to
`health.header.sequence`. This producer does not read FakeIns, does not
gate on submersion, and does not convert geodetic coordinates.

Dropout returns None and is checked first. An invalid fix sets the source
to FIX_SOURCE_UNSPECIFIED (a set field, not an absent one) and health state
INVALID. It wins over bias: bias is not applied. Delay is still applied.

Otherwise health state is DEGRADED when any position bias component is
non-zero, and VALID when all are zero. Bias is added to the position
components and is not repaired. Delay is added to source time to form
receive time and is not repaired when it reverses the stamps.
"""

from dataclasses import dataclass
from typing import Optional

from intrinsic.hardware.marine import measurement_health_pb2
from intrinsic.hardware.marine import surface_fix_pb2
from intrinsic.hardware.marine import surface_fix_policy

from intrinsic.embodiment import stamped_header_policy
from intrinsic.embodiment.proto import stamped_header_pb2
from intrinsic.vehicle import vehicle_contract_policy

_NANOS_PER_SECOND = 1000000000


@dataclass(frozen=True)
class FakeSurfaceFixConfig:
  seed: int = 42
  position_bias_x_m: float = 0.0
  position_bias_y_m: float = 0.0
  position_bias_z_m: float = 0.0
  delay_seconds: int = 1
  delay_nanos: int = -250000000
  dropout: bool = False
  invalid_fix: bool = False


@dataclass(frozen=True)
class SurfaceFixTruth:
  frame_id: str = "gnss"
  source_seconds: int = 1700000000
  source_nanos: int = 250000000
  position_x_m: float = 12.5
  position_y_m: float = -3.25
  position_z_m: float = 1.0
  # Wire source. 1 is FIX_SOURCE_GNSS. 2 is FIX_SOURCE_ACOUSTIC.
  source_present: bool = True
  source: int = 1
  satellite_count_present: bool = True
  satellite_count: int = 12
  beacon_count_present: bool = False
  beacon_count: int = 0
  horizontal_accuracy_present: bool = False
  horizontal_accuracy_m: float = 0.0
  vertical_accuracy_present: bool = False
  vertical_accuracy_m: float = 0.0
  velocity_present: bool = False
  velocity_x_m_s: float = 0.0
  velocity_y_m_s: float = 0.0
  velocity_z_m_s: float = 0.0
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


def _bias_engaged(config: FakeSurfaceFixConfig) -> bool:
  return (
      config.position_bias_x_m != 0.0
      or config.position_bias_y_m != 0.0
      or config.position_bias_z_m != 0.0
  )


def _fill_covariance(matrix) -> None:
  values = [0.0] * vehicle_contract_policy.COVARIANCE_VALUES
  values[surface_fix_policy.POSITION_X_VARIANCE_SLOT] = 1.0
  values[surface_fix_policy.POSITION_Y_VARIANCE_SLOT] = 4.0
  values[surface_fix_policy.POSITION_Z_VARIANCE_SLOT] = 0.25
  matrix.values[:] = values


class FakeSurfaceFix:
  """Fixed-seed surface fix. Default config and truth are the nominal."""

  def __init__(self, config: Optional[FakeSurfaceFixConfig] = None):
    self._config = config if config is not None else FakeSurfaceFixConfig()

  @property
  def config(self) -> FakeSurfaceFixConfig:
    return self._config

  def measure(self, truth: Optional[SurfaceFixTruth] = None):
    """Return a SurfaceFix, or None when dropout is set."""
    if self._config.dropout:
      return None
    if truth is None:
      truth = SurfaceFixTruth()
    config = self._config
    sample = surface_fix_pb2.SurfaceFix()
    health = sample.health
    header = health.header
    header.sequence = config.seed
    header.source_time.seconds = truth.source_seconds
    header.source_time.nanos = truth.source_nanos
    receive_seconds, receive_nanos = _add_clock_delay(
        truth.source_seconds,
        truth.source_nanos,
        config.delay_seconds,
        config.delay_nanos,
    )
    header.receive_time.seconds = receive_seconds
    header.receive_time.nanos = receive_nanos
    header.source_id = "nav_sensor"
    header.frame_id = truth.frame_id
    header.clock_domain = stamped_header_policy.CLOCK_DOMAIN_MONOTONIC
    header.validity.state = stamped_header_pb2.Validity.STATE_VALID

    state = measurement_health_pb2.MeasurementHealth
    if config.invalid_fix:
      health.state = state.INVALID
    elif _bias_engaged(config):
      health.state = state.DEGRADED
    else:
      health.state = state.VALID
    health.quality = truth.quality
    if truth.covariance_present:
      _fill_covariance(health.covariance)
    primary = health.sources.add()
    primary.source_id = "primary"
    primary.validity.state = stamped_header_pb2.Validity.STATE_VALID
    health.sources.add().source_id = "aiding"

    position_x = truth.position_x_m
    position_y = truth.position_y_m
    position_z = truth.position_z_m
    if not config.invalid_fix:
      position_x += config.position_bias_x_m
      position_y += config.position_bias_y_m
      position_z += config.position_bias_z_m
    sample.position_x_m = position_x
    sample.position_y_m = position_y
    sample.position_z_m = position_z
    if config.invalid_fix:
      sample.source = surface_fix_pb2.SurfaceFix.FIX_SOURCE_UNSPECIFIED
    elif truth.source_present:
      sample.source = truth.source
    if truth.satellite_count_present:
      sample.satellite_count = truth.satellite_count
    if truth.beacon_count_present:
      sample.beacon_count = truth.beacon_count
    if truth.horizontal_accuracy_present:
      sample.horizontal_accuracy_m = truth.horizontal_accuracy_m
    if truth.vertical_accuracy_present:
      sample.vertical_accuracy_m = truth.vertical_accuracy_m
    if truth.velocity_present:
      sample.velocity_x_m_s = truth.velocity_x_m_s
      sample.velocity_y_m_s = truth.velocity_y_m_s
      sample.velocity_z_m_s = truth.velocity_z_m_s
    return sample
