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

"""Deterministic UUV motion-primitive generator.

Policy: intrinsic_motion_planning/intrinsic/motion_planning/vehicle/README.md.

Python mirror of vehicle_motion_primitives.h. A primitive is a constant
body-frame wrench (force in N, torque in N*m) held for a fixed duration.
This is a pure function: no dynamics integration, no search, no collision
checking, no World access.
"""

from collections.abc import Sequence
import dataclasses
import enum
import math

from intrinsic.vehicle import vehicle_contract_policy

BodyVector = vehicle_contract_policy.BodyVector

_NUM_AXES = 6
_ZERO: BodyVector = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)


class PrimitiveSetError(enum.Enum):
  OK = 0
  BAD_CONFIG = 1
  EMPTY = 2


class PrimitiveGenerationMode(enum.Enum):
  # Zero wrench + +/-max on each of the 6 body axes alone -> 13 primitives.
  AXIS_ALIGNED = 0
  # Use config.custom_controls as the control list (still validated).
  CUSTOM = 1


@dataclasses.dataclass(frozen=True)
class VehicleMotionPrimitiveConfig:
  """Generator configuration. BodyVector order is Fx, Fy, Fz, Tx, Ty, Tz."""

  mode: PrimitiveGenerationMode = PrimitiveGenerationMode.AXIS_ALIGNED
  # Inclusive magnitude limits (force N, torque N*m). Finite and >= 0.
  max_force_torque: BodyVector = _ZERO
  # Shared duration for every primitive. Finite and > 0.
  duration_s: float = 0.0
  # Used only when mode is CUSTOM. Empty yields EMPTY after validation.
  custom_controls: Sequence[BodyVector] = ()


@dataclasses.dataclass(frozen=True)
class VehicleMotionPrimitive:
  # "uuv-prim-" + zero-padded index, in generation order.
  id: str
  control: BodyVector
  duration_s: float


@dataclasses.dataclass(frozen=True)
class PrimitiveSetResult:
  error: PrimitiveSetError = PrimitiveSetError.BAD_CONFIG
  # Empty unless error is OK.
  primitives: tuple[VehicleMotionPrimitive, ...] = ()


def _is_vector(value) -> bool:
  try:
    return len(value) == _NUM_AXES
  except TypeError:
    return False


def _max_is_valid(max_force_torque) -> bool:
  return _is_vector(max_force_torque) and all(
      math.isfinite(m) and m >= 0.0 for m in max_force_torque
  )


def _control_is_within(control, max_force_torque) -> bool:
  if not _is_vector(control):
    return False
  return all(
      math.isfinite(u) and abs(u) <= m
      for u, m in zip(control, max_force_torque)
  )


def generate_uuv_motion_primitives(
    config: VehicleMotionPrimitiveConfig,
) -> PrimitiveSetResult:
  """Generates the primitive set for `config`. First defect wins.

  1. A max_force_torque component is non-finite or negative -> BAD_CONFIG.
  2. duration_s is non-finite or <= 0                       -> BAD_CONFIG.
  3. CUSTOM and a control is non-finite or has |component| > max on any
     axis -> BAD_CONFIG. Illegal controls are rejected, never clamped.
  4. CUSTOM with no controls                                -> EMPTY.

  AXIS_ALIGNED order (13 entries): zero; +/-max Fx; +/-max Fy; +/-max Fz;
  +/-max Tx; +/-max Ty; +/-max Tz. Within each pair the positive control
  comes first. A zero bound still emits both entries of its pair, as
  all-zero controls. CUSTOM preserves custom_controls order. Ids are
  uuv-prim-000... in generation order.
  """
  bad = PrimitiveSetResult(error=PrimitiveSetError.BAD_CONFIG)
  max_ft = config.max_force_torque
  if not _max_is_valid(max_ft):
    return bad
  duration = config.duration_s
  if not math.isfinite(duration) or duration <= 0.0:
    return bad

  controls: list[BodyVector] = []
  if config.mode == PrimitiveGenerationMode.AXIS_ALIGNED:
    controls.append(_ZERO)
    for axis in range(_NUM_AXES):
      positive = [0.0] * _NUM_AXES
      negative = [0.0] * _NUM_AXES
      # A zero bound stays +0.0 on both entries so controls compare bit for
      # bit.
      if max_ft[axis] != 0.0:
        positive[axis] = float(max_ft[axis])
        negative[axis] = -float(max_ft[axis])
      controls.append(tuple(positive))
      controls.append(tuple(negative))
  elif config.mode == PrimitiveGenerationMode.CUSTOM:
    for control in config.custom_controls:
      if not _control_is_within(control, max_ft):
        return bad
      controls.append(tuple(float(u) for u in control))
    if not controls:
      return PrimitiveSetResult(error=PrimitiveSetError.EMPTY)
  else:
    return bad

  primitives = tuple(
      VehicleMotionPrimitive(
          id=f"uuv-prim-{index:03d}", control=control, duration_s=duration
      )
      for index, control in enumerate(controls)
  )
  return PrimitiveSetResult(error=PrimitiveSetError.OK, primitives=primitives)
