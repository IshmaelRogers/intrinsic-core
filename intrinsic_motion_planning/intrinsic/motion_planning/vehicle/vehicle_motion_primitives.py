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

"""Config-driven body-frame twist primitives for a UUV.

Policy: intrinsic_motion_planning/intrinsic/motion_planning/vehicle/README.md.

Python mirror of vehicle_motion_primitives.h. This is a pure function of the
config: no search, sampling, collision, World, dynamics integration, or
planner registration.

Control is a body-frame twist, the same 6-layout as BodyVector: linear x, y,
z in m/s, then angular x, y, z in rad/s. It is not a wrench.
"""

import dataclasses
import enum
import math

from intrinsic.vehicle import vehicle_contract_policy

BodyVector = vehicle_contract_policy.BodyVector

_ZERO_CONTROL: BodyVector = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)


class MotionPrimitiveAxis(enum.Enum):
  """Body-frame twist axis. The value is the BodyVector component index."""

  SURGE = 0  # linear x
  SWAY = 1  # linear y
  HEAVE = 2  # linear z
  ROLL = 3  # angular x
  PITCH = 4  # angular y
  YAW = 5  # angular z


class MotionPrimitiveSetError(enum.Enum):
  OK = 0
  BAD_CONFIG = 1  # Duration, bounds, or levels are invalid.
  BOUNDS_VIOLATION = 2  # A would-be primitive fails an engaged bound.


@dataclasses.dataclass(frozen=True)
class VehicleMotionPrimitive:
  """One bounded control held for a positive duration."""

  id: str = ""
  control: BodyVector = _ZERO_CONTROL
  duration_s: float = 0.0  # > 0 and finite when the set is valid.


@dataclasses.dataclass(frozen=True)
class VehicleMotionPrimitiveBounds:
  """Speed limits applied to every emitted control.

  An absent limit does not constrain that group. A present limit must be
  finite and >= 0.
  """

  max_linear_speed_present: bool = False
  max_linear_speed_m_s: float = 0.0  # Applies to ||v_lin||.
  max_angular_speed_present: bool = False
  max_angular_speed_rad_s: float = 0.0  # Applies to ||w||.


@dataclasses.dataclass(frozen=True)
class VehicleMotionPrimitiveSetConfig:
  """Generator input.

  An axis is emitted only when its *_present flag is true and its level is
  finite and > 0. A present flag with any other level is BAD_CONFIG.
  """

  duration_s: float = 1.0  # Shared by every emitted primitive.
  # When true, emit a zero-control "hover" primitive first.
  include_hover: bool = True
  surge_present: bool = False
  surge_m_s: float = 0.0
  sway_present: bool = False
  sway_m_s: float = 0.0
  heave_present: bool = False
  heave_m_s: float = 0.0
  roll_present: bool = False
  roll_rad_s: float = 0.0
  pitch_present: bool = False
  pitch_rad_s: float = 0.0
  yaw_present: bool = False
  yaw_rad_s: float = 0.0
  bounds: VehicleMotionPrimitiveBounds = VehicleMotionPrimitiveBounds()


@dataclasses.dataclass(frozen=True)
class GenerateMotionPrimitivesResult:
  """Ordered primitives, or an empty tuple on every non-ok error."""

  error: MotionPrimitiveSetError = MotionPrimitiveSetError.OK
  # Empty is legal when include_hover is false, no axis is present, and the
  # config is otherwise valid.
  primitives: tuple[VehicleMotionPrimitive, ...] = ()


def _speed_bound_valid(present: bool, value: float) -> bool:
  return (not present) or (math.isfinite(value) and value >= 0.0)


def _level_valid(level: float) -> bool:
  return math.isfinite(level) and level > 0.0


def _control_on_axis(axis: MotionPrimitiveAxis, value: float) -> BodyVector:
  components = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
  components[axis.value] = value
  return (
      components[0],
      components[1],
      components[2],
      components[3],
      components[4],
      components[5],
  )


def _within_bounds(
    control: BodyVector, bounds: VehicleMotionPrimitiveBounds
) -> bool:
  if bounds.max_linear_speed_present:
    speed = math.hypot(control[0], control[1], control[2])
    if not speed <= bounds.max_linear_speed_m_s:
      return False
  if bounds.max_angular_speed_present:
    rate = math.hypot(control[3], control[4], control[5])
    if not rate <= bounds.max_angular_speed_rad_s:
      return False
  return True


def _axes(
    config: VehicleMotionPrimitiveSetConfig,
) -> tuple[tuple[bool, float, MotionPrimitiveAxis, str], ...]:
  return (
      (
          config.surge_present,
          config.surge_m_s,
          MotionPrimitiveAxis.SURGE,
          "surge",
      ),
      (config.sway_present, config.sway_m_s, MotionPrimitiveAxis.SWAY, "sway"),
      (
          config.heave_present,
          config.heave_m_s,
          MotionPrimitiveAxis.HEAVE,
          "heave",
      ),
      (
          config.roll_present,
          config.roll_rad_s,
          MotionPrimitiveAxis.ROLL,
          "roll",
      ),
      (
          config.pitch_present,
          config.pitch_rad_s,
          MotionPrimitiveAxis.PITCH,
          "pitch",
      ),
      (config.yaw_present, config.yaw_rad_s, MotionPrimitiveAxis.YAW, "yaw"),
  )


def generate_motion_primitives(
    config: VehicleMotionPrimitiveSetConfig,
) -> GenerateMotionPrimitivesResult:
  """Builds the ordered primitive set for `config`.

  Check order, first defect wins, is BAD_CONFIG:

  1. duration_s non-finite or <= 0.
  2. A present speed limit is non-finite or < 0. Linear, then angular.
     Disengaged limit fields are ignored.
  3. A *_present axis level is non-finite or <= 0, in axis order surge,
     sway, heave, roll, pitch, yaw.

  A valid config emits, in order: hover (if requested), then for each
  engaged axis the +level primitive ("<axis>_pos") and the -level primitive
  ("<axis>_neg"). Other components stay zero. Every primitive uses
  duration_s.

  Every emitted control, including hover, must satisfy the engaged bounds
  (||v_lin|| <= max linear, ||w|| <= max angular, inclusive). If any fails,
  the result is BOUNDS_VIOLATION and primitives is empty.

  Args:
    config: Duration, hover flag, per-axis levels, and speed bounds.

  Returns:
    The error and the ordered primitives. Primitives is empty unless the
    error is OK. An OK result may still be empty.
  """
  bad = MotionPrimitiveSetError.BAD_CONFIG
  if not (math.isfinite(config.duration_s) and config.duration_s > 0.0):
    return GenerateMotionPrimitivesResult(error=bad)
  bounds = config.bounds
  if not _speed_bound_valid(
      bounds.max_linear_speed_present, bounds.max_linear_speed_m_s
  ) or not _speed_bound_valid(
      bounds.max_angular_speed_present, bounds.max_angular_speed_rad_s
  ):
    return GenerateMotionPrimitivesResult(error=bad)

  axes = _axes(config)
  for present, level, _axis, _token in axes:
    if present and not _level_valid(level):
      return GenerateMotionPrimitivesResult(error=bad)

  primitives: list[VehicleMotionPrimitive] = []
  if config.include_hover:
    primitives.append(
        VehicleMotionPrimitive(
            id="hover", control=_ZERO_CONTROL, duration_s=config.duration_s
        )
    )
  for present, level, axis, token in axes:
    if not present:
      continue
    primitives.append(
        VehicleMotionPrimitive(
            id=f"{token}_pos",
            control=_control_on_axis(axis, level),
            duration_s=config.duration_s,
        )
    )
    primitives.append(
        VehicleMotionPrimitive(
            id=f"{token}_neg",
            control=_control_on_axis(axis, -level),
            duration_s=config.duration_s,
        )
    )

  for primitive in primitives:
    if not _within_bounds(primitive.control, bounds):
      return GenerateMotionPrimitivesResult(
          error=MotionPrimitiveSetError.BOUNDS_VIOLATION
      )
  return GenerateMotionPrimitivesResult(
      error=MotionPrimitiveSetError.OK, primitives=tuple(primitives)
  )
