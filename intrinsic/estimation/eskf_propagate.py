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

"""IMU nominal-state propagation only (#82). See ESKF_PROPAGATE.md.

Mirrors eskf_propagate.h. No covariance, Phi, Q, or measurement update.
"""

import dataclasses
import enum
import math
from typing import Optional, Tuple

from intrinsic.estimation import eskf_state

# World ENU gravity (W0), m/s^2.
GRAVITY_ENU = (0.0, 0.0, -9.80665)
MAX_PROPAGATE_DT_S = 1.0
MIN_QUATERNION_NORM = 1e-12

# Spelled as in the C++ header.
kGravityEnu = GRAVITY_ENU
kMaxPropagateDtS = MAX_PROPAGATE_DT_S
kMinQuaternionNorm = MIN_QUATERNION_NORM

_Vec3 = Tuple[float, float, float]
_Quat = Tuple[float, float, float, float]
_Mat3 = Tuple[float, ...]


@dataclasses.dataclass(frozen=True)
class ImuSample:
  """One IMU sample in the body frame, SI."""

  # Specific force as measured by the accelerometer (includes gravity effect).
  accel_m_s2: _Vec3 = (0.0, 0.0, 0.0)
  gyro_rad_s: _Vec3 = (0.0, 0.0, 0.0)


class PropagateStatus(enum.Enum):
  OK = 0
  INVALID_DT = 1
  INVALID_IMU = 2
  INVALID_QUATERNION = 3
  NON_FINITE_STATE = 4


@dataclasses.dataclass(frozen=True)
class PropagateResult:
  ok: bool
  status: PropagateStatus
  # Set only when ok. A rejected call leaves no partial state.
  nominal: Optional[eskf_state.EskfNominal] = None


def _all_finite(values) -> bool:
  return all(math.isfinite(x) for x in values)


def _norm4(q) -> float:
  return math.sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3])


def _reject(status: PropagateStatus) -> PropagateResult:
  return PropagateResult(ok=False, status=status, nominal=None)


def cross(a: _Vec3, b: _Vec3) -> _Vec3:
  return (
      a[1] * b[2] - a[2] * b[1],
      a[2] * b[0] - a[0] * b[2],
      a[0] * b[1] - a[1] * b[0],
  )


def quaternion_multiply(a: _Quat, b: _Quat) -> _Quat:
  """Hamilton product a (x) b, wxyz order."""
  return (
      a[0] * b[0] - a[1] * b[1] - a[2] * b[2] - a[3] * b[3],
      a[0] * b[1] + a[1] * b[0] + a[2] * b[3] - a[3] * b[2],
      a[0] * b[2] - a[1] * b[3] + a[2] * b[0] + a[3] * b[1],
      a[0] * b[3] + a[1] * b[2] - a[2] * b[1] + a[3] * b[0],
  )


def rotation_body_to_world(q_wxyz: _Quat) -> _Mat3:
  """Body->world rotation of q / ||q||, row-major 3x3."""
  n = _norm4(q_wxyz)
  w = q_wxyz[0] / n
  x = q_wxyz[1] / n
  y = q_wxyz[2] / n
  z = q_wxyz[3] / n
  return (
      1.0 - 2.0 * (y * y + z * z),
      2.0 * (x * y - w * z),
      2.0 * (x * z + w * y),
      2.0 * (x * y + w * z),
      1.0 - 2.0 * (x * x + z * z),
      2.0 * (y * z - w * x),
      2.0 * (x * z - w * y),
      2.0 * (y * z + w * x),
      1.0 - 2.0 * (x * x + y * y),
  )


def _mat_vec(r: _Mat3, v: _Vec3) -> _Vec3:
  return (
      r[0] * v[0] + r[1] * v[1] + r[2] * v[2],
      r[3] * v[0] + r[4] * v[1] + r[5] * v[2],
      r[6] * v[0] + r[7] * v[1] + r[8] * v[2],
  )


def _mat_t_vec(r: _Mat3, v: _Vec3) -> _Vec3:
  return (
      r[0] * v[0] + r[3] * v[1] + r[6] * v[2],
      r[1] * v[0] + r[4] * v[1] + r[7] * v[2],
      r[2] * v[0] + r[5] * v[1] + r[8] * v[2],
  )


def propagate_nominal(
    x: eskf_state.EskfNominal, imu: ImuSample, dt_s: float
) -> PropagateResult:
  """Propagates the nominal state one IMU step. Never raises on bad input."""
  # 1. dt.
  if not math.isfinite(dt_s) or not dt_s > 0.0 or dt_s > MAX_PROPAGATE_DT_S:
    return _reject(PropagateStatus.INVALID_DT)

  # 2. IMU, nominal, and quaternion norm.
  if not (_all_finite(imu.accel_m_s2) and _all_finite(imu.gyro_rad_s)):
    return _reject(PropagateStatus.INVALID_IMU)
  if not _all_finite(x.to_tuple()):
    return _reject(PropagateStatus.NON_FINITE_STATE)
  q = x.q_wxyz
  q_norm = _norm4(q)
  if not math.isfinite(q_norm) or q_norm < MIN_QUATERNION_NORM:
    return _reject(PropagateStatus.INVALID_QUATERNION)

  omega = tuple(imu.gyro_rad_s[i] - x.b_g[i] for i in range(3))
  a = tuple(imu.accel_m_s2[i] - x.b_a[i] for i in range(3))

  # 3. Attitude: first-order Hamilton, then renormalize.
  q_dot = tuple(
      0.5 * c
      for c in quaternion_multiply(q, (0.0, omega[0], omega[1], omega[2]))
  )
  q_tmp = tuple(q[i] + dt_s * q_dot[i] for i in range(4))
  tmp_norm = _norm4(q_tmp)
  if not math.isfinite(tmp_norm) or tmp_norm < MIN_QUATERNION_NORM:
    return _reject(PropagateStatus.INVALID_QUATERNION)
  q_new = tuple(c / tmp_norm for c in q_tmp)

  # 4 and 5. Velocity and position use the pre-update q and v.
  r = rotation_body_to_world(q)
  w_cross_v = cross(omega, x.v_body)
  g_body = _mat_t_vec(r, GRAVITY_ENU)
  p_dot = _mat_vec(r, x.v_body)
  v_new = tuple(
      x.v_body[i] + dt_s * (a[i] - w_cross_v[i] + g_body[i]) for i in range(3)
  )
  p_new = tuple(x.p_enu[i] + dt_s * p_dot[i] for i in range(3))

  # 7. Never return a partial or non-finite state.
  if not _all_finite(p_new + q_new + v_new):
    return _reject(PropagateStatus.NON_FINITE_STATE)

  return PropagateResult(
      ok=True,
      status=PropagateStatus.OK,
      nominal=eskf_state.EskfNominal(
          p_enu=p_new, q_wxyz=q_new, v_body=v_new, b_a=x.b_a, b_g=x.b_g
      ),
  )
