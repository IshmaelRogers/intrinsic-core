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

"""Deterministic thruster-array fake.

The same config and command sequence yield the same bytes. `seed` is the
feedback sequence origin. It is not a random source. This fake does not
call ICON, allocate thrust, integrate dynamics, or talk to hardware.

Slot bounds default to the six-thruster example magnitudes. Those numbers
are copied here. This module does not import that example and does not
pull Gazebo.

Each apply() records the command, then:

1. Dropout returns None. The command stays in the delay line. Dropout is
   an absent sample, not an engaged fault.
2. The applied command is the one from `lag_steps` earlier. A missing past
   sample is neutral (0 N, enabled). That hold is not a repair of the new
   command.
3. Per slot, the stronger fault wins: stuck-off, failed, disabled
   (configured or enable false), configured derated, efficiency in (0, 1),
   then nominal. Neutral faults force applied thrust to 0.
4. Otherwise optional slew limits the step, saturation clamps to
   [min_thrust_n, max_thrust_n], and efficiency scales the clamped thrust.
   An exact bound is not saturated. Saturation is judged before the
   efficiency scale.
5. The feedback header copies the current command header and sets sequence
   to seed + step. Thrust values come from the delayed command. Stamps are
   not rewritten to hide lag.

A watchdog or independent hardware timer is out of scope. This fake only
neutralizes on disable and on configured stuck-off / failed / disabled.
"""

from dataclasses import dataclass
from dataclasses import field
import math

from intrinsic.hardware.marine import thruster_array_pb2
from intrinsic.hardware.marine import thruster_array_policy

from intrinsic.embodiment import stamped_header_policy
from intrinsic.embodiment.proto import stamped_header_pb2

_HEALTH_NOMINAL = thruster_array_pb2.THRUSTER_HEALTH_NOMINAL
_HEALTH_DISABLED = thruster_array_pb2.THRUSTER_HEALTH_DISABLED
_HEALTH_DERATED = thruster_array_pb2.THRUSTER_HEALTH_DERATED
_HEALTH_STUCK_OFF = thruster_array_pb2.THRUSTER_HEALTH_STUCK_OFF
_HEALTH_FAILED = thruster_array_pb2.THRUSTER_HEALTH_FAILED

_RANK_NONE = 0
_RANK_DERATED = 1
_RANK_DISABLED = 2
_RANK_FAILED = 3
_RANK_STUCK_OFF = 4


@dataclass
class FakeThrusterSlotConfig:
  name: str
  min_thrust_n: float
  max_thrust_n: float
  max_forward_slew_n_per_s: float
  max_reverse_slew_n_per_s: float
  efficiency: float = 1.0
  fault_health: int | None = None
  fault_derate: float = 1.0


def _slot(name, forward_n, reverse_n, forward_slew, reverse_slew):
  return FakeThrusterSlotConfig(
      name=name,
      min_thrust_n=-reverse_n,
      max_thrust_n=forward_n,
      max_forward_slew_n_per_s=forward_slew,
      max_reverse_slew_n_per_s=reverse_slew,
  )


def six_thruster_fixture_slots():
  """Bounds copied from six_thruster_uuv_example. Not a measurement."""
  return [
      _slot("surge_port", 50, 35, 250, 175),
      _slot("surge_starboard", 50, 35, 250, 175),
      _slot("sway_fore", 30, 30, 150, 150),
      _slot("sway_aft", 30, 30, 150, 150),
      _slot("heave_fore", 40, 25, 200, 125),
      _slot("heave_aft", 40, 25, 200, 125),
  ]


def _fallback_slot():
  return _slot("", 50, 35, 250, 175)


@dataclass
class FakeThrusterArrayConfig:
  seed: int = 42
  lag_steps: int = 0
  slew: bool = False
  dt_s: float = 0.2
  dropout: bool = False
  slots: list = field(default_factory=six_thruster_fixture_slots)


def _clamp(value, min_n, max_n):
  if value > max_n:
    return max_n
  if value < min_n:
    return min_n
  return value


def _slew_toward(previous, target, forward_slew, reverse_slew, dt):
  delta = target - previous
  if delta == 0.0:
    return previous
  rate = forward_slew if delta > 0.0 else reverse_slew
  limit = rate * dt
  if not limit > 0.0:
    return previous
  if abs(delta) <= limit:
    return target
  return previous + math.copysign(limit, delta)


def _rank_fault(slot, enabled):
  if slot.fault_health == _HEALTH_STUCK_OFF:
    return _RANK_STUCK_OFF
  if slot.fault_health == _HEALTH_FAILED:
    return _RANK_FAILED
  if (not enabled) or slot.fault_health == _HEALTH_DISABLED:
    return _RANK_DISABLED
  if slot.fault_health == _HEALTH_DERATED:
    return _RANK_DERATED
  if slot.efficiency > 0.0 and slot.efficiency < 1.0:
    return _RANK_DERATED
  return _RANK_NONE


def _health_wire(rank):
  return {
      _RANK_STUCK_OFF: _HEALTH_STUCK_OFF,
      _RANK_FAILED: _HEALTH_FAILED,
      _RANK_DISABLED: _HEALTH_DISABLED,
      _RANK_DERATED: _HEALTH_DERATED,
      _RANK_NONE: _HEALTH_NOMINAL,
  }[rank]


def _health_derate(rank, slot):
  if rank in (_RANK_STUCK_OFF, _RANK_FAILED, _RANK_DISABLED):
    return 0.0
  if rank == _RANK_DERATED:
    if slot.fault_health == _HEALTH_DERATED:
      return slot.fault_derate
    return slot.efficiency
  return 1.0


def _neutral_rank(rank):
  return rank in (_RANK_STUCK_OFF, _RANK_FAILED, _RANK_DISABLED)


def _fill_default_header(header):
  header.source_id = "thruster_array"
  header.frame_id = "body"
  header.clock_domain = stamped_header_policy.CLOCK_DOMAIN_MONOTONIC
  header.validity.state = stamped_header_pb2.Validity.STATE_VALID
  header.source_time.seconds = 1700000000
  header.source_time.nanos = 250000000
  header.receive_time.seconds = 1700000001


class _RecordedSlot:

  def __init__(self):
    self.thrust_n = 0.0
    self.enabled = True
    self.name_present = False
    self.name = ""


class FakeThrusterArray:
  """Stateful pure-delay thruster array. See the module docstring."""

  def __init__(self, config=None):
    self._config = config if config is not None else FakeThrusterArrayConfig()
    self._history = []
    self._slew = []
    self._step = 0

  @property
  def config(self):
    return self._config

  def apply(self, command):
    """Returns feedback, or None when dropout is set."""
    current = []
    for element in command.thrusters:
      slot = _RecordedSlot()
      slot.thrust_n = element.thrust_n if element.HasField("thrust_n") else 0.0
      slot.enabled = thruster_array_policy.thruster_command_enabled(
          thruster_array_policy.ThrusterCommandElementView(
              enable_present=element.HasField("enable"),
              enable=element.enable,
          )
      )
      slot.name_present = element.HasField("name")
      if slot.name_present:
        slot.name = element.name
      current.append(slot)
    self._history.append(current)
    sequence = self._config.seed + self._step
    self._step += 1
    if self._config.dropout:
      return None

    lag = self._config.lag_steps if self._config.lag_steps > 0 else 0
    delayed_index = len(self._history) - 1 - lag
    if delayed_index >= 0:
      applied = self._history[delayed_index]
    else:
      applied = [_RecordedSlot() for _ in current]

    feedback = thruster_array_pb2.ThrusterArrayFeedback()
    if command.HasField("header"):
      feedback.header.CopyFrom(command.header)
    else:
      _fill_default_header(feedback.header)
    feedback.header.sequence = sequence

    for index, recorded in enumerate(applied):
      if 0 <= index < len(self._config.slots):
        slot = self._config.slots[index]
      else:
        slot = _fallback_slot()
      if len(self._slew) <= index:
        self._slew.extend([0.0] * (index + 1 - len(self._slew)))
      rank = _rank_fault(slot, recorded.enabled)
      applied_thrust = 0.0
      saturated = False
      if _neutral_rank(rank):
        self._slew[index] = 0.0
      else:
        if self._config.slew:
          pre = _slew_toward(
              self._slew[index],
              recorded.thrust_n,
              slot.max_forward_slew_n_per_s,
              slot.max_reverse_slew_n_per_s,
              self._config.dt_s,
          )
        else:
          pre = recorded.thrust_n
        clamped = _clamp(pre, slot.min_thrust_n, slot.max_thrust_n)
        self._slew[index] = clamped
        saturated = clamped != pre
        applied_thrust = clamped * slot.efficiency

      element = feedback.thrusters.add()
      if recorded.name_present:
        element.name = recorded.name
      elif slot.name:
        element.name = slot.name
      element.commanded_thrust_n = recorded.thrust_n
      element.measured_thrust_n = applied_thrust
      element.saturated = saturated
      element.health = _health_wire(rank)
      element.health_derate = _health_derate(rank, slot)
      element.efficiency = slot.efficiency
    return feedback
