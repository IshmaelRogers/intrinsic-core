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

"""Plain-value vehicle StateSpace: interpolate, distance, and validate.

Policy: intrinsic_motion_planning/intrinsic/motion_planning/vehicle/README.md.

Python mirror of vehicle_state_space.h. There is no search, sampling,
collision checking, or World access here. Inputs are never renormalized or
converted between ENU and NED.

Quaternions are Hamilton (x, y, z, w). BodyVector order is linear x, y, z,
then angular x, y, z (REP-103 body axes).
"""

import dataclasses
import enum
import math
import sys

from intrinsic.embodiment import frame_policy
from intrinsic.vehicle import vehicle_contract_policy

BodyVector = vehicle_contract_policy.BodyVector
Vec3 = frame_policy.Vec3
Quaternion = frame_policy.Quaternion

QUATERNION_NORM_TOLERANCE = 1e-9

# Orientations already unit to within a few ulps are copied verbatim so that
# endpoint interpolation is exact.
_EXACT_UNIT_TOLERANCE = 1e-15
_EPSILON = sys.float_info.epsilon


class StateSpaceError(enum.Enum):
  OK = 0
  MIX_PARAMETER = 1  # u not in [0, 1].
  NON_FINITE = 2
  QUATERNION = 3  # Non-unit input orientation.
  BOUNDS = 4  # Failed validate against engaged limits.
  BAD_BOUNDS = 5  # The bounds configuration itself is invalid.


@dataclasses.dataclass(frozen=True)
class VehiclePlanningState:
  """Planning sample. Not the estimator VehicleState wire message."""

  position: Vec3 = (0.0, 0.0, 0.0)
  orientation: Quaternion = (0.0, 0.0, 0.0, 1.0)
  twist: BodyVector = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)


@dataclasses.dataclass(frozen=True)
class VehicleStateBounds:
  """Absent limits do not constrain that axis."""

  position_limits_present: bool = False
  position_min: Vec3 = (0.0, 0.0, 0.0)  # Inclusive when present.
  position_max: Vec3 = (0.0, 0.0, 0.0)  # Inclusive when present.
  max_linear_speed_present: bool = False
  max_linear_speed_m_s: float = 0.0  # Finite and >= 0 when present.
  max_angular_speed_present: bool = False
  max_angular_speed_rad_s: float = 0.0  # Finite and >= 0 when present.


@dataclasses.dataclass(frozen=True)
class InterpolateResult:
  error: StateSpaceError = StateSpaceError.OK
  # Meaningful only when error is StateSpaceError.OK.
  state: VehiclePlanningState = VehiclePlanningState()


def _is_finite_state(state: VehiclePlanningState) -> bool:
  return (
      frame_policy.is_finite_vec3(state.position)
      and frame_policy.is_finite_quaternion(state.orientation)
      and all(math.isfinite(component) for component in state.twist)
  )


def _is_unit(quaternion: Quaternion) -> bool:
  return frame_policy.is_normalized(quaternion, QUATERNION_NORM_TOLERANCE)


def _unit_copy(quaternion: Quaternion) -> Quaternion:
  norm = frame_policy.quaternion_norm(quaternion)
  if abs(norm - 1.0) <= _EXACT_UNIT_TOLERANCE:
    return tuple(quaternion)
  return tuple(component / norm for component in quaternion)


def _lerp(u: float, a: float, b: float) -> float:
  return (1.0 - u) * a + u * b


def _slerp(u: float, a: Quaternion, b: Quaternion) -> Quaternion:
  """Shortest-path slerp. Mirrors intrinsic::eigenmath::Interpolate."""
  cos_omega = sum(x * y for x, y in zip(a, b))
  abs_cos_omega = abs(cos_omega)
  if abs_cos_omega > 1.0 - _EPSILON:
    p0 = 1.0 - u
    p1 = u
  else:
    omega = math.acos(abs_cos_omega)
    sin_omega = math.sin(omega)
    p0 = math.sin((1.0 - u) * omega) / sin_omega
    p1 = math.sin(u * omega) / sin_omega
  if cos_omega < 0:
    p1 = -p1
  return tuple(p0 * x + p1 * y for x, y in zip(a, b))


def interpolate(
    a: VehiclePlanningState, b: VehiclePlanningState, u: float
) -> InterpolateResult:
  """Interpolates from a (u = 0) to b (u = 1).

  Check order: u, finiteness, unit orientations. The first defect wins.
  Position and twist are linear: (1 - u) * a + u * b. Orientation is the
  shortest-path slerp. The output orientation is unit length and lies in a's
  hemisphere, except that u == 1 returns b exactly. u == 0 returns a exactly.

  Args:
    a: Start state.
    b: End state.
    u: Mix parameter in [0, 1]. No extrapolation.

  Returns:
    The result with an error. The state is meaningful only for OK.
  """
  if not (0.0 <= u <= 1.0):  # Also rejects NaN.
    return InterpolateResult(error=StateSpaceError.MIX_PARAMETER)
  if not (_is_finite_state(a) and _is_finite_state(b)):
    return InterpolateResult(error=StateSpaceError.NON_FINITE)
  if not (_is_unit(a.orientation) and _is_unit(b.orientation)):
    return InterpolateResult(error=StateSpaceError.QUATERNION)

  position = tuple(_lerp(u, x, y) for x, y in zip(a.position, b.position))
  twist = tuple(_lerp(u, x, y) for x, y in zip(a.twist, b.twist))
  if u == 0.0:
    orientation = _unit_copy(a.orientation)
  elif u == 1.0:
    orientation = _unit_copy(b.orientation)
  else:
    orientation = _unit_copy(_slerp(u, a.orientation, b.orientation))

  out = VehiclePlanningState(
      position=position, orientation=orientation, twist=twist
  )
  if not _is_finite_state(out):
    return InterpolateResult(error=StateSpaceError.NON_FINITE)
  return InterpolateResult(error=StateSpaceError.OK, state=out)


def distance(a: VehiclePlanningState, b: VehiclePlanningState) -> float:
  """sqrt(|dp|^2 + theta^2 + |dv_lin|^2 + |dw|^2) with all weights 1.0.

  theta is the geodesic angle in [0, pi] of the shortest rotation from a to b.

  Args:
    a: First state.
    b: Second state.

  Returns:
    A non-negative distance, or math.inf when either state is non-finite or
    has a non-unit orientation. Never raises for such inputs.
  """
  if not (
      _is_finite_state(a)
      and _is_finite_state(b)
      and _is_unit(a.orientation)
      and _is_unit(b.orientation)
  ):
    return math.inf

  dx = b.position[0] - a.position[0]
  dy = b.position[1] - a.position[1]
  dz = b.position[2] - a.position[2]
  position_sq = (dx * dx) + (dy * dy) + (dz * dz)

  # conj(a) * b has scalar part dot(a, b) and vector part
  # a.w * b.v - b.w * a.v - a.v x b.v. Swapping a and b negates the vector
  # part exactly, so theta is exactly symmetric.
  ax, ay, az, aw = a.orientation
  bx, by, bz, bw = b.orientation
  dot = (ax * bx) + (ay * by) + (az * bz) + (aw * bw)
  vx = (aw * bx - bw * ax) - (ay * bz - az * by)
  vy = (aw * by - bw * ay) - (az * bx - ax * bz)
  vz = (aw * bz - bw * az) - (ax * by - ay * bx)
  theta = 2.0 * math.atan2(
      math.sqrt((vx * vx) + (vy * vy) + (vz * vz)), abs(dot)
  )

  deltas = [y - x for x, y in zip(a.twist, b.twist)]
  linear_sq = sum(d * d for d in deltas[:3])
  angular_sq = sum(d * d for d in deltas[3:])

  try:
    result = math.sqrt(position_sq + (theta * theta) + linear_sq + angular_sq)
  except (OverflowError, ValueError):
    return math.inf
  return result if math.isfinite(result) else math.inf


def _speed_bound_valid(present: bool, value: float) -> bool:
  return (not present) or (math.isfinite(value) and value >= 0.0)


def _position_bounds_valid(bounds: VehicleStateBounds) -> bool:
  if not bounds.position_limits_present:
    return True
  lo = bounds.position_min
  hi = bounds.position_max
  if any(math.isnan(c) for c in lo) or any(math.isnan(c) for c in hi):
    return False
  return all(low <= high for low, high in zip(lo, hi))


def _norm3(x: float, y: float, z: float) -> float:
  return math.sqrt((x * x) + (y * y) + (z * z))


def validate(
    state: VehiclePlanningState, bounds: VehicleStateBounds
) -> StateSpaceError:
  """Validates a state against bounds.

  Check order: bounds configuration, finiteness, unit orientation, position
  box, linear speed, angular speed. The first defect wins. Default bounds
  check finiteness and unit orientation only.

  Args:
    state: The state to check.
    bounds: Engaged limits. Absent limits do not constrain.

  Returns:
    OK or the first defect.
  """
  if (
      not _position_bounds_valid(bounds)
      or not _speed_bound_valid(
          bounds.max_linear_speed_present, bounds.max_linear_speed_m_s
      )
      or not _speed_bound_valid(
          bounds.max_angular_speed_present, bounds.max_angular_speed_rad_s
      )
  ):
    return StateSpaceError.BAD_BOUNDS
  if not _is_finite_state(state):
    return StateSpaceError.NON_FINITE
  if not _is_unit(state.orientation):
    return StateSpaceError.QUATERNION
  if bounds.position_limits_present:
    for value, low, high in zip(
        state.position, bounds.position_min, bounds.position_max
    ):
      if not (low <= value <= high):
        return StateSpaceError.BOUNDS
  if bounds.max_linear_speed_present:
    speed = _norm3(*state.twist[:3])
    if not speed <= bounds.max_linear_speed_m_s:
      return StateSpaceError.BOUNDS
  if bounds.max_angular_speed_present:
    rate = _norm3(*state.twist[3:])
    if not rate <= bounds.max_angular_speed_rad_s:
      return StateSpaceError.BOUNDS
  return StateSpaceError.OK
