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

"""Current-aware propagation of one UUV motion primitive.

Policy: intrinsic_motion_planning/intrinsic/motion_planning/vehicle/README.md.

Python mirror of vehicle_primitive_propagation.h. Planning integrates with
semi-implicit Euler and a diagonal mass surrogate. Dynamics are injected
through evaluate(); this module does not reimplement the marine force model.
There is no search, collision checking, or World access.
"""

from collections.abc import Sequence
import dataclasses
import enum
import math

from intrinsic.embodiment import frame_policy
from intrinsic.motion_planning.vehicle import vehicle_motion_primitives
from intrinsic.motion_planning.vehicle import vehicle_state_space

VehiclePlanningState = vehicle_state_space.VehiclePlanningState
VehicleMotionPrimitive = vehicle_motion_primitives.VehicleMotionPrimitive
QUATERNION_NORM_TOLERANCE = vehicle_state_space.QUATERNION_NORM_TOLERANCE

_SPATIAL_DOF = 6


class PropagationError(enum.Enum):
  OK = 0
  BAD_CONFIG = 1
  BAD_START = 2
  BAD_PRIMITIVE = 3
  DYNAMICS_FAILED = 4
  STEP_BUDGET = 5


class FrameId(enum.Enum):
  """Mirrors intrinsic::vehicle::dynamics::FrameId for evaluate()."""

  UNSPECIFIED = 0
  WORLD_ENU = 1
  WORLD_NED = 2
  BODY = 3


class DynamicsError(enum.Enum):
  OK = 0
  INVALID_ARGUMENT = 1


@dataclasses.dataclass(frozen=True)
class Duration:
  seconds: float = 0.0


@dataclasses.dataclass(frozen=True)
class VehicleStateRt:
  pose_frame: FrameId = FrameId.UNSPECIFIED
  position_m: tuple[float, float, float] = (0.0, 0.0, 0.0)
  orientation_xyzw: tuple[float, float, float, float] = (0.0, 0.0, 0.0, 1.0)
  body_twist: tuple[float, float, float, float, float, float] = (
      0.0,
      0.0,
      0.0,
      0.0,
      0.0,
      0.0,
  )


@dataclasses.dataclass(frozen=True)
class BodyWrenchRt:
  frame: FrameId = FrameId.BODY
  force_n: tuple[float, float, float] = (0.0, 0.0, 0.0)
  torque_n_m: tuple[float, float, float] = (0.0, 0.0, 0.0)


@dataclasses.dataclass(frozen=True)
class EnvironmentRt:
  gravity_m_s2: float = 0.0
  fluid_density_kg_m3: float = 0.0
  current_velocity_m_s: tuple[float, float, float] = (0.0, 0.0, 0.0)
  current_frame: FrameId = FrameId.UNSPECIFIED


@dataclasses.dataclass(frozen=True)
class StateDerivative:
  position_dot_m_s: tuple[float, float, float] = (0.0, 0.0, 0.0)
  orientation_dot_xyzw: tuple[float, float, float, float] = (
      0.0,
      0.0,
      0.0,
      0.0,
  )
  body_acceleration: tuple[float, float, float, float, float, float] = (
      0.0,
      0.0,
      0.0,
      0.0,
      0.0,
      0.0,
  )


@dataclasses.dataclass(frozen=True)
class DynamicsDiagnostics:
  model_id: str = ""
  dt_s: float = 0.0
  input_wrench_used: bool = False
  total_wrench: tuple[float, float, float, float, float, float] = (
      0.0,
      0.0,
      0.0,
      0.0,
      0.0,
      0.0,
  )


@dataclasses.dataclass(frozen=True)
class DynamicsResult:
  derivative: StateDerivative = dataclasses.field(
      default_factory=StateDerivative
  )
  diagnostics: DynamicsDiagnostics = dataclasses.field(
      default_factory=DynamicsDiagnostics
  )


@dataclasses.dataclass(frozen=True)
class DynamicsStatus:
  code: DynamicsError = DynamicsError.OK
  message: str = ""

  def ok(self) -> bool:
    return self.code is DynamicsError.OK


@dataclasses.dataclass(frozen=True)
class StatusOr:
  """evaluate() return. ok() is false when VehicleDynamics would reject."""

  status: DynamicsStatus = dataclasses.field(default_factory=DynamicsStatus)
  value: DynamicsResult | None = None

  def ok(self) -> bool:
    return self.status.ok() and self.value is not None


@dataclasses.dataclass(frozen=True)
class PropagationConfig:
  """Substep, step cap, diagonal mass surrogate, and world-ENU current."""

  dt_s: float = 0.0
  max_steps: int = 0
  mass_diag: Sequence[float] = (1.0, 1.0, 1.0, 1.0, 1.0, 1.0)
  gravity_m_s2: float = 0.0
  fluid_density_kg_m3: float = 0.0
  current_world_enu_m_s: Sequence[float] = (0.0, 0.0, 0.0)


@dataclasses.dataclass(frozen=True)
class PropagationSample:
  time_s: float = 0.0  # monotonic, starts at 0
  state: VehiclePlanningState = dataclasses.field(
      default_factory=VehiclePlanningState
  )


@dataclasses.dataclass(frozen=True)
class PropagationResult:
  error: PropagationError = PropagationError.BAD_CONFIG
  # Includes the start sample at t = 0 when OK. Empty unless OK.
  samples: tuple[PropagationSample, ...] = ()


def _fail(error: PropagationError) -> PropagationResult:
  return PropagationResult(error=error, samples=())


def _number(value) -> float | None:
  if isinstance(value, bool) or not isinstance(value, (int, float)):
    return None
  number = float(value)
  if not math.isfinite(number):
    return None
  return number


def _numbers(values, length: int) -> list[float] | None:
  try:
    if len(values) != length:
      return None
  except TypeError:
    return None
  parsed: list[float] = []
  for value in values:
    number = _number(value)
    if number is None:
      return None
    parsed.append(number)
  return parsed


def _config_ok(config: PropagationConfig) -> bool:
  dt_s = _number(config.dt_s)
  if dt_s is None or dt_s <= 0.0:
    return False
  if type(config.max_steps) is not int or config.max_steps < 1:
    return False
  mass = _numbers(config.mass_diag, _SPATIAL_DOF)
  if mass is None or any(entry <= 0.0 for entry in mass):
    return False
  gravity = _number(config.gravity_m_s2)
  if gravity is None or gravity < 0.0:
    return False
  density = _number(config.fluid_density_kg_m3)
  if density is None or density < 0.0:
    return False
  current = _numbers(config.current_world_enu_m_s, 3)
  return current is not None


def _start_ok(start: VehiclePlanningState) -> bool:
  try:
    finite = (
        frame_policy.is_finite_vec3(start.position)
        and frame_policy.is_finite_quaternion(start.orientation)
        and len(start.twist) == _SPATIAL_DOF
        and all(_number(component) is not None for component in start.twist)
    )
  except TypeError:
    return False
  if not finite:
    return False
  return frame_policy.is_normalized(
      start.orientation, QUATERNION_NORM_TOLERANCE
  )


def _primitive_ok(primitive: VehicleMotionPrimitive) -> bool:
  duration = _number(primitive.duration_s)
  if duration is None or duration <= 0.0:
    return False
  try:
    if len(primitive.control) != _SPATIAL_DOF:
      return False
  except TypeError:
    return False
  return all(_number(component) is not None for component in primitive.control)


def _step_count(duration_s: float, dt_s: float, max_steps: int) -> int | None:
  ratio = duration_s / dt_s
  if not math.isfinite(ratio) or ratio < 0.0:
    return None
  n_full = math.floor(ratio)
  if n_full > 2**31 - 1:
    return None
  n = int(n_full)
  rem = duration_s - float(n) * dt_s
  steps = n
  if rem > 0.0:
    if n == 2**31 - 1:
      return None
    steps = n + 1
  if steps > max_steps:
    return None
  return steps


def _to_state_rt(state: VehiclePlanningState) -> VehicleStateRt:
  return VehicleStateRt(
      pose_frame=FrameId.WORLD_ENU,
      position_m=(
          float(state.position[0]),
          float(state.position[1]),
          float(state.position[2]),
      ),
      orientation_xyzw=(
          float(state.orientation[0]),
          float(state.orientation[1]),
          float(state.orientation[2]),
          float(state.orientation[3]),
      ),
      body_twist=tuple(float(component) for component in state.twist),
  )


def _to_wrench(primitive: VehicleMotionPrimitive) -> BodyWrenchRt:
  control = primitive.control
  return BodyWrenchRt(
      frame=FrameId.BODY,
      force_n=(float(control[0]), float(control[1]), float(control[2])),
      torque_n_m=(float(control[3]), float(control[4]), float(control[5])),
  )


def _to_environment(config: PropagationConfig) -> EnvironmentRt:
  current = config.current_world_enu_m_s
  return EnvironmentRt(
      gravity_m_s2=float(config.gravity_m_s2),
      fluid_density_kg_m3=float(config.fluid_density_kg_m3),
      current_velocity_m_s=(
          float(current[0]),
          float(current[1]),
          float(current[2]),
      ),
      current_frame=FrameId.WORLD_ENU,
  )


def _planning_wrench(evaluated: DynamicsResult, wrench: BodyWrenchRt):
  if evaluated.diagnostics.input_wrench_used:
    wrench_values = evaluated.diagnostics.total_wrench
  else:
    wrench_values = wrench.force_n + wrench.torque_n_m
  parsed = _numbers(wrench_values, _SPATIAL_DOF)
  return parsed


def _renormalize(
    quaternion: list[float],
) -> tuple[float, float, float, float] | None:
  # A non-finite or zero norm cannot produce a finite unit quaternion.
  # Dividing a large finite value by an overflowed norm yields zeros.
  norm_sq = sum(component * component for component in quaternion)
  norm = math.sqrt(norm_sq)
  if not math.isfinite(norm) or not norm > 0.0:
    return None
  updated = tuple(component / norm for component in quaternion)
  if not frame_policy.is_finite_quaternion(updated):
    return None
  return updated


def propagate_uuv_motion_primitive(
    start: VehiclePlanningState,
    primitive: VehicleMotionPrimitive,
    config: PropagationConfig,
    dynamics,
) -> PropagationResult:
  """Integrates one primitive. First defect wins. Samples empty unless OK.

  dynamics.evaluate(state, wrench, environment, dt) mirrors
  VehicleDynamics::Evaluate and returns a StatusOr.
  """
  if not _config_ok(config):
    return _fail(PropagationError.BAD_CONFIG)
  if not _start_ok(start):
    return _fail(PropagationError.BAD_START)
  if not _primitive_ok(primitive):
    return _fail(PropagationError.BAD_PRIMITIVE)
  if (
      _step_count(
          float(primitive.duration_s), float(config.dt_s), config.max_steps
      )
      is None
  ):
    return _fail(PropagationError.STEP_BUDGET)

  wrench = _to_wrench(primitive)
  environment = _to_environment(config)
  mass = [float(entry) for entry in config.mass_diag]
  current = [float(entry) for entry in config.current_world_enu_m_s]
  samples = [PropagationSample(time_s=0.0, state=start)]
  state = start
  elapsed = 0.0
  steps_taken = 0
  while elapsed < float(primitive.duration_s):
    if steps_taken >= config.max_steps:
      return _fail(PropagationError.STEP_BUDGET)
    dt_i = min(float(config.dt_s), float(primitive.duration_s) - elapsed)
    first = dynamics.evaluate(
        _to_state_rt(state), wrench, environment, Duration(dt_i)
    )
    if not first.ok():
      return _fail(PropagationError.DYNAMICS_FAILED)
    planning = _planning_wrench(first.value, wrench)
    if planning is None:
      return _fail(PropagationError.DYNAMICS_FAILED)
    twist = tuple(
        float(state.twist[i]) + dt_i * (planning[i] / mass[i])
        for i in range(_SPATIAL_DOF)
    )
    state_mid = VehiclePlanningState(
        position=state.position, orientation=state.orientation, twist=twist
    )
    second = dynamics.evaluate(
        _to_state_rt(state_mid), wrench, environment, Duration(dt_i)
    )
    if not second.ok():
      return _fail(PropagationError.DYNAMICS_FAILED)
    position_dot = second.value.derivative.position_dot_m_s
    orientation_dot = second.value.derivative.orientation_dot_xyzw
    try:
      if len(position_dot) != 3 or len(orientation_dot) != 4:
        return _fail(PropagationError.DYNAMICS_FAILED)
    except TypeError:
      return _fail(PropagationError.DYNAMICS_FAILED)
    position = tuple(
        float(state.position[i]) + dt_i * (float(position_dot[i]) + current[i])
        for i in range(3)
    )
    orientation = [
        float(state.orientation[i]) + dt_i * float(orientation_dot[i])
        for i in range(4)
    ]
    renormalized = _renormalize(orientation)
    if renormalized is None:
      return _fail(PropagationError.DYNAMICS_FAILED)
    state = VehiclePlanningState(
        position=position, orientation=renormalized, twist=twist
    )
    elapsed += dt_i
    steps_taken += 1
    samples.append(PropagationSample(time_s=elapsed, state=state))
  return PropagationResult(error=PropagationError.OK, samples=tuple(samples))
