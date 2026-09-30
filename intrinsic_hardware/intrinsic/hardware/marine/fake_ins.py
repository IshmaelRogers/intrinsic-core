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

"""Deterministic vendor-INS producer.

Same config and truth yield the same bytes. `seed` is written to
`health.header.sequence` and mixes the noise draw. This producer does not
read FakeImu and does not propagate a filter.

Dropout returns None and is checked first. Invalid orientation replaces
the quaternion with the canonical non-unit fixture and sets health state
INVALID. It wins over bias, drift, and noise: those additions are not
applied. Delay is still applied.

Otherwise health state is DEGRADED when any position bias component, any
applicable rate bias, the position drift amplitude, or any applicable
noise amplitude is non-zero. Position bias, drift, and noise always apply.
Drift adds `amplitude * seed` to each position component. Rate bias and
rate noise apply only when that twist triple is present on the truth.
They do not invent an absent triple, and an inapplicable rate fault does
not set DEGRADED. Delay is not repaired when it reverses the stamps.
"""

from dataclasses import dataclass
from typing import Optional

from intrinsic.hardware.marine import ins_pb2
from intrinsic.hardware.marine import ins_policy
from intrinsic.hardware.marine import measurement_health_pb2

from intrinsic.embodiment import stamped_header_policy
from intrinsic.embodiment.proto import stamped_header_pb2
from intrinsic.vehicle import vehicle_contract_policy

_NANOS_PER_SECOND = 1000000000
_MASK64 = (1 << 64) - 1

# Canonical non-unit quaternion. Norm is 2, so abs(norm - 1) is 1.
CANONICAL_NON_UNIT_QUATERNION = (0.0, 0.0, 0.0, 2.0)


def ins_signed_unit_noise(seed: int) -> float:
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
class FakeInsConfig:
  seed: int = 42
  position_bias_x_m: float = 0.0
  position_bias_y_m: float = 0.0
  position_bias_z_m: float = 0.0
  linear_velocity_bias_x_m_s: float = 0.0
  linear_velocity_bias_y_m_s: float = 0.0
  linear_velocity_bias_z_m_s: float = 0.0
  angular_velocity_bias_x_rad_s: float = 0.0
  angular_velocity_bias_y_rad_s: float = 0.0
  angular_velocity_bias_z_rad_s: float = 0.0
  position_drift_m_per_seed: float = 0.0
  position_noise_amplitude_m: float = 0.0
  linear_velocity_noise_amplitude_m_s: float = 0.0
  angular_velocity_noise_amplitude_rad_s: float = 0.0
  delay_seconds: int = 1
  delay_nanos: int = -250000000
  dropout: bool = False
  invalid_orientation: bool = False


@dataclass(frozen=True)
class InsTruth:
  frame_id: str = "ins"
  source_seconds: int = 1700000000
  source_nanos: int = 250000000
  position_x_m: float = 12.0
  position_y_m: float = -4.0
  position_z_m: float = 0.5
  orientation_x: float = 0.0
  orientation_y: float = 0.0
  orientation_z: float = 0.0
  orientation_w: float = 1.0
  linear_velocity_present: bool = True
  linear_velocity_x_m_s: float = 1.5
  linear_velocity_y_m_s: float = 0.0
  linear_velocity_z_m_s: float = -0.25
  angular_velocity_present: bool = True
  angular_velocity_x_rad_s: float = 0.0
  angular_velocity_y_rad_s: float = 0.125
  angular_velocity_z_rad_s: float = 0.0
  source_present: bool = True
  source: int = 1
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


def _position_bias_engaged(config: FakeInsConfig) -> bool:
  return (
      config.position_bias_x_m != 0.0
      or config.position_bias_y_m != 0.0
      or config.position_bias_z_m != 0.0
  )


def _linear_bias_engaged(config: FakeInsConfig) -> bool:
  return (
      config.linear_velocity_bias_x_m_s != 0.0
      or config.linear_velocity_bias_y_m_s != 0.0
      or config.linear_velocity_bias_z_m_s != 0.0
  )


def _angular_bias_engaged(config: FakeInsConfig) -> bool:
  return (
      config.angular_velocity_bias_x_rad_s != 0.0
      or config.angular_velocity_bias_y_rad_s != 0.0
      or config.angular_velocity_bias_z_rad_s != 0.0
  )


def _degraded(config: FakeInsConfig, truth: InsTruth) -> bool:
  if (
      _position_bias_engaged(config)
      or config.position_drift_m_per_seed != 0.0
      or config.position_noise_amplitude_m != 0.0
  ):
    return True
  if truth.linear_velocity_present and (
      _linear_bias_engaged(config)
      or config.linear_velocity_noise_amplitude_m_s != 0.0
  ):
    return True
  if truth.angular_velocity_present and (
      _angular_bias_engaged(config)
      or config.angular_velocity_noise_amplitude_rad_s != 0.0
  ):
    return True
  return False


def _fill_covariance(matrix) -> None:
  values = [0.0] * vehicle_contract_policy.COVARIANCE_VALUES
  values[ins_policy.POSITION_X_VARIANCE_SLOT] = 1.0
  values[ins_policy.POSITION_Y_VARIANCE_SLOT] = 4.0
  values[ins_policy.POSITION_Z_VARIANCE_SLOT] = 0.25
  values[ins_policy.ATTITUDE_X_VARIANCE_SLOT] = 0.0625
  values[ins_policy.ATTITUDE_Y_VARIANCE_SLOT] = 0.125
  values[ins_policy.ATTITUDE_Z_VARIANCE_SLOT] = 0.5
  matrix.values[:] = values


def _set_orientation(sample, x, y, z, w) -> None:
  sample.orientation_xyzw.x = x
  sample.orientation_xyzw.y = y
  sample.orientation_xyzw.z = z
  sample.orientation_xyzw.w = w


class FakeIns:
  """Fixed-seed INS. Default config and truth are the nominal solution."""

  def __init__(self, config: Optional[FakeInsConfig] = None):
    self._config = config if config is not None else FakeInsConfig()

  @property
  def config(self) -> FakeInsConfig:
    return self._config

  def measure(self, truth: Optional[InsTruth] = None):
    """Return an InsSolution, or None when dropout is set."""
    if self._config.dropout:
      return None
    if truth is None:
      truth = InsTruth()
    sample = ins_pb2.InsSolution()
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
    if self._config.invalid_orientation:
      health.state = state.INVALID
    elif _degraded(self._config, truth):
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
    linear_x = truth.linear_velocity_x_m_s
    linear_y = truth.linear_velocity_y_m_s
    linear_z = truth.linear_velocity_z_m_s
    angular_x = truth.angular_velocity_x_rad_s
    angular_y = truth.angular_velocity_y_rad_s
    angular_z = truth.angular_velocity_z_rad_s
    if not self._config.invalid_orientation:
      seed = float(self._config.seed)
      position_x += self._config.position_bias_x_m
      position_y += self._config.position_bias_y_m
      position_z += self._config.position_bias_z_m
      position_x += self._config.position_drift_m_per_seed * seed
      position_y += self._config.position_drift_m_per_seed * seed
      position_z += self._config.position_drift_m_per_seed * seed
      if self._config.position_noise_amplitude_m != 0.0:
        unit = ins_signed_unit_noise(self._config.seed)
        position_x += self._config.position_noise_amplitude_m * unit
        position_y += self._config.position_noise_amplitude_m * unit
        position_z += self._config.position_noise_amplitude_m * unit
      if truth.linear_velocity_present:
        linear_x += self._config.linear_velocity_bias_x_m_s
        linear_y += self._config.linear_velocity_bias_y_m_s
        linear_z += self._config.linear_velocity_bias_z_m_s
        if self._config.linear_velocity_noise_amplitude_m_s != 0.0:
          unit = ins_signed_unit_noise(self._config.seed + 1)
          linear_x += self._config.linear_velocity_noise_amplitude_m_s * unit
          linear_y += self._config.linear_velocity_noise_amplitude_m_s * unit
          linear_z += self._config.linear_velocity_noise_amplitude_m_s * unit
      if truth.angular_velocity_present:
        angular_x += self._config.angular_velocity_bias_x_rad_s
        angular_y += self._config.angular_velocity_bias_y_rad_s
        angular_z += self._config.angular_velocity_bias_z_rad_s
        if self._config.angular_velocity_noise_amplitude_rad_s != 0.0:
          unit = ins_signed_unit_noise(self._config.seed + 2)
          angular_x += (
              self._config.angular_velocity_noise_amplitude_rad_s * unit
          )
          angular_y += (
              self._config.angular_velocity_noise_amplitude_rad_s * unit
          )
          angular_z += (
              self._config.angular_velocity_noise_amplitude_rad_s * unit
          )
    sample.position_x_m = position_x
    sample.position_y_m = position_y
    sample.position_z_m = position_z
    if self._config.invalid_orientation:
      _set_orientation(sample, *CANONICAL_NON_UNIT_QUATERNION)
    else:
      _set_orientation(
          sample,
          truth.orientation_x,
          truth.orientation_y,
          truth.orientation_z,
          truth.orientation_w,
      )
    if truth.linear_velocity_present:
      sample.linear_velocity_x_m_s = linear_x
      sample.linear_velocity_y_m_s = linear_y
      sample.linear_velocity_z_m_s = linear_z
    if truth.angular_velocity_present:
      sample.angular_velocity_x_rad_s = angular_x
      sample.angular_velocity_y_rad_s = angular_y
      sample.angular_velocity_z_rad_s = angular_z
    if truth.source_present:
      sample.source = truth.source
    return sample
