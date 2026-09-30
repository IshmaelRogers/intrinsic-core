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

"""Deterministic raw-IMU producer.

Same config and truth yield the same bytes. `seed` is written to
`health.header.sequence` and mixes the noise draw. There is no filter, no
magnetometer, and no INS coupling.

Dropout returns None and is checked first. Invalid orientation replaces
the quaternion with the canonical non-unit fixture and sets health state
INVALID. It wins over bias, drift, and noise: those additions are not
applied. Delay is still applied.

Otherwise a non-zero bias component, drift amplitude, or noise amplitude
sets health state DEGRADED. Bias is added per component. Drift adds
`amplitude * seed` to every component of that triple. Noise adds
`amplitude * imu_signed_unit_noise(seed)` to angular velocity and
`amplitude * imu_signed_unit_noise(seed + 1)` to linear acceleration.
Results are not repaired. Delay is added to source time to form receive
time and is not repaired when it reverses the stamps.
"""

from dataclasses import dataclass
from typing import Optional

from intrinsic.hardware.marine import imu_pb2
from intrinsic.hardware.marine import imu_policy
from intrinsic.hardware.marine import measurement_health_pb2

from intrinsic.embodiment import stamped_header_policy
from intrinsic.embodiment.proto import stamped_header_pb2
from intrinsic.vehicle import vehicle_contract_policy

_NANOS_PER_SECOND = 1000000000
_MASK64 = (1 << 64) - 1

# Canonical non-unit quaternion. Norm is 2, so abs(norm - 1) is 1.
CANONICAL_NON_UNIT_QUATERNION = (0.0, 0.0, 0.0, 2.0)


def imu_signed_unit_noise(seed: int) -> float:
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
class FakeImuConfig:
  seed: int = 42
  angular_velocity_bias_x_rad_s: float = 0.0
  angular_velocity_bias_y_rad_s: float = 0.0
  angular_velocity_bias_z_rad_s: float = 0.0
  linear_acceleration_bias_x_m_s2: float = 0.0
  linear_acceleration_bias_y_m_s2: float = 0.0
  linear_acceleration_bias_z_m_s2: float = 0.0
  angular_drift_rad_s_per_seed: float = 0.0
  linear_drift_m_s2_per_seed: float = 0.0
  angular_noise_amplitude_rad_s: float = 0.0
  linear_noise_amplitude_m_s2: float = 0.0
  delay_seconds: int = 1
  delay_nanos: int = -250000000
  dropout: bool = False
  invalid_orientation: bool = False


@dataclass(frozen=True)
class ImuTruth:
  frame_id: str = "imu"
  source_seconds: int = 1700000000
  source_nanos: int = 250000000
  angular_velocity_x_rad_s: float = 0.25
  angular_velocity_y_rad_s: float = -0.5
  angular_velocity_z_rad_s: float = 0.125
  linear_acceleration_x_m_s2: float = 0.0
  linear_acceleration_y_m_s2: float = 0.0
  linear_acceleration_z_m_s2: float = 8.0
  orientation_present: bool = True
  orientation_x: float = 0.0
  orientation_y: float = 0.0
  orientation_z: float = 0.0
  orientation_w: float = 1.0
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


def _bias_engaged(config: FakeImuConfig) -> bool:
  return (
      config.angular_velocity_bias_x_rad_s != 0.0
      or config.angular_velocity_bias_y_rad_s != 0.0
      or config.angular_velocity_bias_z_rad_s != 0.0
      or config.linear_acceleration_bias_x_m_s2 != 0.0
      or config.linear_acceleration_bias_y_m_s2 != 0.0
      or config.linear_acceleration_bias_z_m_s2 != 0.0
  )


def _drift_engaged(config: FakeImuConfig) -> bool:
  return (
      config.angular_drift_rad_s_per_seed != 0.0
      or config.linear_drift_m_s2_per_seed != 0.0
  )


def _noise_engaged(config: FakeImuConfig) -> bool:
  return (
      config.angular_noise_amplitude_rad_s != 0.0
      or config.linear_noise_amplitude_m_s2 != 0.0
  )


def _fill_covariance(matrix) -> None:
  values = [0.0] * vehicle_contract_policy.COVARIANCE_VALUES
  values[imu_policy.ANGULAR_VELOCITY_X_VARIANCE_SLOT] = 0.25
  values[imu_policy.ANGULAR_VELOCITY_Y_VARIANCE_SLOT] = 0.5
  values[imu_policy.ANGULAR_VELOCITY_Z_VARIANCE_SLOT] = 0.125
  values[imu_policy.LINEAR_ACCELERATION_X_VARIANCE_SLOT] = 1.0
  values[imu_policy.LINEAR_ACCELERATION_Y_VARIANCE_SLOT] = 2.0
  values[imu_policy.LINEAR_ACCELERATION_Z_VARIANCE_SLOT] = 4.0
  matrix.values[:] = values


def _set_orientation(sample, x, y, z, w) -> None:
  sample.orientation_xyzw.x = x
  sample.orientation_xyzw.y = y
  sample.orientation_xyzw.z = z
  sample.orientation_xyzw.w = w


class FakeImu:
  """Fixed-seed IMU. Default config and truth are the nominal sample."""

  def __init__(self, config: Optional[FakeImuConfig] = None):
    self._config = config if config is not None else FakeImuConfig()

  @property
  def config(self) -> FakeImuConfig:
    return self._config

  def measure(self, truth: Optional[ImuTruth] = None):
    """Return an ImuMeasurement, or None when dropout is set."""
    if self._config.dropout:
      return None
    if truth is None:
      truth = ImuTruth()
    sample = imu_pb2.ImuMeasurement()
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
    degraded = (
        _bias_engaged(self._config)
        or _drift_engaged(self._config)
        or _noise_engaged(self._config)
    )
    if self._config.invalid_orientation:
      health.state = state.INVALID
    elif degraded:
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

    angular_x = truth.angular_velocity_x_rad_s
    angular_y = truth.angular_velocity_y_rad_s
    angular_z = truth.angular_velocity_z_rad_s
    linear_x = truth.linear_acceleration_x_m_s2
    linear_y = truth.linear_acceleration_y_m_s2
    linear_z = truth.linear_acceleration_z_m_s2
    if not self._config.invalid_orientation:
      seed = float(self._config.seed)
      angular_x += self._config.angular_velocity_bias_x_rad_s
      angular_y += self._config.angular_velocity_bias_y_rad_s
      angular_z += self._config.angular_velocity_bias_z_rad_s
      linear_x += self._config.linear_acceleration_bias_x_m_s2
      linear_y += self._config.linear_acceleration_bias_y_m_s2
      linear_z += self._config.linear_acceleration_bias_z_m_s2
      angular_x += self._config.angular_drift_rad_s_per_seed * seed
      angular_y += self._config.angular_drift_rad_s_per_seed * seed
      angular_z += self._config.angular_drift_rad_s_per_seed * seed
      linear_x += self._config.linear_drift_m_s2_per_seed * seed
      linear_y += self._config.linear_drift_m_s2_per_seed * seed
      linear_z += self._config.linear_drift_m_s2_per_seed * seed
      if self._config.angular_noise_amplitude_rad_s != 0.0:
        unit = imu_signed_unit_noise(self._config.seed)
        angular_x += self._config.angular_noise_amplitude_rad_s * unit
        angular_y += self._config.angular_noise_amplitude_rad_s * unit
        angular_z += self._config.angular_noise_amplitude_rad_s * unit
      if self._config.linear_noise_amplitude_m_s2 != 0.0:
        unit = imu_signed_unit_noise(self._config.seed + 1)
        linear_x += self._config.linear_noise_amplitude_m_s2 * unit
        linear_y += self._config.linear_noise_amplitude_m_s2 * unit
        linear_z += self._config.linear_noise_amplitude_m_s2 * unit
    sample.angular_velocity_x_rad_s = angular_x
    sample.angular_velocity_y_rad_s = angular_y
    sample.angular_velocity_z_rad_s = angular_z
    sample.linear_acceleration_x_m_s2 = linear_x
    sample.linear_acceleration_y_m_s2 = linear_y
    sample.linear_acceleration_z_m_s2 = linear_z
    if self._config.invalid_orientation:
      _set_orientation(sample, *CANONICAL_NON_UNIT_QUATERNION)
    elif truth.orientation_present:
      _set_orientation(
          sample,
          truth.orientation_x,
          truth.orientation_y,
          truth.orientation_z,
          truth.orientation_w,
      )
    return sample
