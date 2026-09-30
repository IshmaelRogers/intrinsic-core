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

"""Deterministic DVL producer.

Same config and truth yield the same bytes. `seed` is written to
`health.header.sequence`. It does not draw noise.

Dropout returns None. Otherwise lock-loss forces bottom track, lock false,
and health state INVALID. A non-zero bias without lock-loss sets health
state DEGRADED and adds the bias to the truth velocity. Delay is added to
source time to form receive time and is not repaired when it reverses the
stamps.
"""

from dataclasses import dataclass
from typing import Optional

from intrinsic.hardware.marine import dvl_pb2
from intrinsic.hardware.marine import measurement_health_pb2

from intrinsic.embodiment import stamped_header_policy
from intrinsic.embodiment.proto import stamped_header_pb2
from intrinsic.vehicle import vehicle_contract_policy

_NANOS_PER_SECOND = 1000000000


@dataclass(frozen=True)
class FakeDvlConfig:
  seed: int = 42
  bias_x_m_s: float = 0.0
  bias_y_m_s: float = 0.0
  bias_z_m_s: float = 0.0
  delay_seconds: int = 1
  delay_nanos: int = -250000000
  dropout: bool = False
  lock_loss: bool = False


@dataclass(frozen=True)
class DvlTruth:
  frame_id: str = "sensor"
  source_seconds: int = 1700000000
  source_nanos: int = 250000000
  velocity_x_m_s: float = 0.5
  velocity_y_m_s: float = -0.25
  velocity_z_m_s: float = 0.0
  mode: int = 1
  quality: float = 0.75
  altitude_present: bool = True
  altitude_m: float = 10.0
  covariance_present: bool = True


def _div_toward_zero(value, divisor):
  if value < 0:
    return -((-value) // divisor)
  return value // divisor


def add_clock_delay(seconds, nanos, delay_seconds, delay_nanos):
  """Return (seconds, nanos) after adding a delay. Nanos land in [0, 1e9)."""
  total_nanos = nanos + delay_nanos
  total_seconds = seconds + delay_seconds
  carry = _div_toward_zero(total_nanos, _NANOS_PER_SECOND)
  total_nanos -= carry * _NANOS_PER_SECOND
  if total_nanos < 0:
    total_nanos += _NANOS_PER_SECOND
    carry -= 1
  return total_seconds + carry, total_nanos


def _bias_engaged(config: FakeDvlConfig) -> bool:
  return (
      config.bias_x_m_s != 0.0
      or config.bias_y_m_s != 0.0
      or config.bias_z_m_s != 0.0
  )


def _fill_linear_covariance(matrix) -> None:
  values = [0.0] * vehicle_contract_policy.COVARIANCE_VALUES
  values[vehicle_contract_policy.covariance_index(0, 0)] = 0.25
  values[vehicle_contract_policy.covariance_index(1, 1)] = 0.25
  values[vehicle_contract_policy.covariance_index(2, 2)] = 0.25
  matrix.values[:] = values


class FakeDvl:
  """Fixed-seed DVL. Default config and truth are the nominal golden."""

  def __init__(self, config: Optional[FakeDvlConfig] = None):
    self._config = config if config is not None else FakeDvlConfig()

  @property
  def config(self) -> FakeDvlConfig:
    return self._config

  def measure(self, truth: Optional[DvlTruth] = None):
    """Return a DvlMeasurement, or None when dropout is set."""
    if self._config.dropout:
      return None
    if truth is None:
      truth = DvlTruth()
    sample = dvl_pb2.DvlMeasurement()
    health = sample.health
    header = health.header
    header.sequence = self._config.seed
    header.source_time.seconds = truth.source_seconds
    header.source_time.nanos = truth.source_nanos
    receive_seconds, receive_nanos = add_clock_delay(
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
    if self._config.lock_loss:
      health.state = state.INVALID
    elif _bias_engaged(self._config):
      health.state = state.DEGRADED
    else:
      health.state = state.VALID
    health.quality = truth.quality
    if truth.covariance_present:
      _fill_linear_covariance(health.covariance)
    primary = health.sources.add()
    primary.source_id = "primary"
    primary.validity.state = stamped_header_pb2.Validity.STATE_VALID
    health.sources.add().source_id = "aiding"

    mode = (
        dvl_pb2.DvlMeasurement.MODE_BOTTOM_TRACK
        if self._config.lock_loss
        else truth.mode
    )
    sample.mode = mode
    sample.velocity_x_m_s = truth.velocity_x_m_s + self._config.bias_x_m_s
    sample.velocity_y_m_s = truth.velocity_y_m_s + self._config.bias_y_m_s
    sample.velocity_z_m_s = truth.velocity_z_m_s + self._config.bias_z_m_s
    if self._config.lock_loss:
      sample.bottom_lock = False
    elif mode == dvl_pb2.DvlMeasurement.MODE_BOTTOM_TRACK:
      sample.bottom_lock = True
    if truth.altitude_present:
      sample.altitude_m = truth.altitude_m
    return sample
