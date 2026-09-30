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

"""Pure authority transition table.

Policy: intrinsic_apis/intrinsic/safety/proto/README.md.

These helpers work on plain values. They do not parse protobuf, do not
evaluate any safety rule, and do not call ICON, a HAL, Gazebo, or an actuator
API. They never read or write World.
"""

from dataclasses import dataclass
import enum

from intrinsic.safety import safety_decision_policy


class AuthorityMode(enum.Enum):
  UNSPECIFIED = 0
  SHADOW = 1
  RECOMMEND = 2
  CONSTRAINED = 3
  REVOKED = 4
  EMERGENCY = 5
  # Any other wire number. Never rewritten. Never a transition target.
  UNKNOWN = 6


class AuthorityEvent(enum.Enum):
  UNSPECIFIED = 0
  ARM_RECOMMEND = 1
  ENABLE_CONSTRAINED = 2
  REVOKE = 3
  ENTER_EMERGENCY = 4
  CLEAR_EMERGENCY = 5
  RESET_TO_SHADOW = 6
  FAULT = 7
  # Any other wire number. Never causes a transition.
  UNKNOWN = 8


class AuthorityTransitionError(enum.Enum):
  NONE = 0
  # `current` is UNSPECIFIED or unknown. Checked first.
  INVALID_MODE = 1
  # `event` is UNSPECIFIED or unknown.
  INVALID_EVENT = 2
  # Both are valid but the pair is not in the approved table.
  FORBIDDEN = 3


_MODES = {
    0: AuthorityMode.UNSPECIFIED,
    1: AuthorityMode.SHADOW,
    2: AuthorityMode.RECOMMEND,
    3: AuthorityMode.CONSTRAINED,
    4: AuthorityMode.REVOKED,
    5: AuthorityMode.EMERGENCY,
}

_EVENTS = {
    0: AuthorityEvent.UNSPECIFIED,
    1: AuthorityEvent.ARM_RECOMMEND,
    2: AuthorityEvent.ENABLE_CONSTRAINED,
    3: AuthorityEvent.REVOKE,
    4: AuthorityEvent.ENTER_EMERGENCY,
    5: AuthorityEvent.CLEAR_EMERGENCY,
    6: AuthorityEvent.RESET_TO_SHADOW,
    7: AuthorityEvent.FAULT,
}

_LIVE_MODES = frozenset(
    (
        AuthorityMode.SHADOW,
        AuthorityMode.RECOMMEND,
        AuthorityMode.CONSTRAINED,
        AuthorityMode.REVOKED,
        AuthorityMode.EMERGENCY,
    )
)

# Every (mode, event) pair that is not a key here is forbidden.
_APPROVED_TRANSITIONS = {
    (AuthorityMode.SHADOW, AuthorityEvent.ARM_RECOMMEND): (
        AuthorityMode.RECOMMEND
    ),
    (AuthorityMode.SHADOW, AuthorityEvent.ENTER_EMERGENCY): (
        AuthorityMode.EMERGENCY
    ),
    (AuthorityMode.SHADOW, AuthorityEvent.FAULT): AuthorityMode.REVOKED,
    (AuthorityMode.RECOMMEND, AuthorityEvent.ENABLE_CONSTRAINED): (
        AuthorityMode.CONSTRAINED
    ),
    (AuthorityMode.RECOMMEND, AuthorityEvent.REVOKE): AuthorityMode.REVOKED,
    (AuthorityMode.RECOMMEND, AuthorityEvent.ENTER_EMERGENCY): (
        AuthorityMode.EMERGENCY
    ),
    (AuthorityMode.RECOMMEND, AuthorityEvent.RESET_TO_SHADOW): (
        AuthorityMode.SHADOW
    ),
    (AuthorityMode.RECOMMEND, AuthorityEvent.FAULT): AuthorityMode.REVOKED,
    (AuthorityMode.CONSTRAINED, AuthorityEvent.REVOKE): AuthorityMode.REVOKED,
    (AuthorityMode.CONSTRAINED, AuthorityEvent.ENTER_EMERGENCY): (
        AuthorityMode.EMERGENCY
    ),
    (AuthorityMode.CONSTRAINED, AuthorityEvent.RESET_TO_SHADOW): (
        AuthorityMode.SHADOW
    ),
    (AuthorityMode.CONSTRAINED, AuthorityEvent.FAULT): AuthorityMode.REVOKED,
    (AuthorityMode.REVOKED, AuthorityEvent.ENTER_EMERGENCY): (
        AuthorityMode.EMERGENCY
    ),
    (AuthorityMode.REVOKED, AuthorityEvent.RESET_TO_SHADOW): (
        AuthorityMode.SHADOW
    ),
    # Emergency clears only to REVOKED.
    (AuthorityMode.EMERGENCY, AuthorityEvent.CLEAR_EMERGENCY): (
        AuthorityMode.REVOKED
    ),
}

_KIND = safety_decision_policy.SafetyDecisionKind

# Decision kinds that may be emitted under each live mode.
_ALLOWED_DECISION_KINDS = {
    AuthorityMode.SHADOW: frozenset((_KIND.REJECT, _KIND.ABORT)),
    AuthorityMode.RECOMMEND: frozenset((_KIND.REJECT, _KIND.ABORT)),
    AuthorityMode.CONSTRAINED: frozenset(
        (
            _KIND.ACCEPT,
            _KIND.PROJECT,
            _KIND.REJECT,
            _KIND.ABORT,
            _KIND.SURFACE,
        )
    ),
    AuthorityMode.REVOKED: frozenset(
        (_KIND.REJECT, _KIND.ABORT, _KIND.SURFACE)
    ),
    AuthorityMode.EMERGENCY: frozenset(
        (_KIND.REJECT, _KIND.ABORT, _KIND.SURFACE)
    ),
}


@dataclass(frozen=True)
class AuthorityTransitionResult:
  ok: bool = False
  # The target mode when `ok`. Otherwise `current`, or UNSPECIFIED when
  # `current` was not a live mode.
  next_mode: AuthorityMode = AuthorityMode.UNSPECIFIED
  error: AuthorityTransitionError = AuthorityTransitionError.INVALID_MODE


def classify_authority_mode(mode: int) -> AuthorityMode:
  """Wire values other than 0..5 are UNKNOWN."""
  return _MODES.get(mode, AuthorityMode.UNKNOWN)


def classify_authority_event(event: int) -> AuthorityEvent:
  """Wire values other than 0..7 are UNKNOWN."""
  return _EVENTS.get(event, AuthorityEvent.UNKNOWN)


def is_live_authority_mode(mode: AuthorityMode) -> bool:
  """True for SHADOW, RECOMMEND, CONSTRAINED, REVOKED, and EMERGENCY."""
  return mode in _LIVE_MODES


def is_live_authority_event(event: AuthorityEvent) -> bool:
  """True for the seven named events."""
  return event not in (AuthorityEvent.UNSPECIFIED, AuthorityEvent.UNKNOWN)


def initial_authority_mode() -> AuthorityMode:
  """The mode after a restart or cold start. Always SHADOW."""
  return AuthorityMode.SHADOW


def apply_authority_transition(
    current: AuthorityMode, event: AuthorityEvent
) -> AuthorityTransitionResult:
  """Applies one event to one mode. Fails closed.

  A forbidden, unknown, or unspecified input returns ok == False and never a
  silent no-op success.
  """
  if not isinstance(current, AuthorityMode) or not is_live_authority_mode(
      current
  ):
    return AuthorityTransitionResult(
        ok=False,
        next_mode=AuthorityMode.UNSPECIFIED,
        error=AuthorityTransitionError.INVALID_MODE,
    )
  if not isinstance(event, AuthorityEvent) or not is_live_authority_event(
      event
  ):
    return AuthorityTransitionResult(
        ok=False,
        next_mode=current,
        error=AuthorityTransitionError.INVALID_EVENT,
    )
  target = _APPROVED_TRANSITIONS.get((current, event))
  if target is None:
    return AuthorityTransitionResult(
        ok=False,
        next_mode=current,
        error=AuthorityTransitionError.FORBIDDEN,
    )
  return AuthorityTransitionResult(
      ok=True, next_mode=target, error=AuthorityTransitionError.NONE
  )


def decision_allowed_under_authority(
    mode: AuthorityMode, kind: safety_decision_policy.SafetyDecisionKind
) -> bool:
  """Policy matrix for decision kinds that may be emitted under a mode.

  A pure table: no rule is evaluated. Unspecified and unknown modes or kinds
  return False.

  | Mode        | ACCEPT | PROJECT | REJECT | ABORT | SURFACE |
  | SHADOW      |   no   |   no    |  yes   |  yes  |   no    |
  | RECOMMEND   |   no   |   no    |  yes   |  yes  |   no    |
  | CONSTRAINED |  yes   |  yes    |  yes   |  yes  |  yes    |
  | REVOKED     |   no   |   no    |  yes   |  yes  |  yes    |
  | EMERGENCY   |   no   |   no    |  yes   |  yes  |  yes    |
  """
  return kind in _ALLOWED_DECISION_KINDS.get(mode, frozenset())
