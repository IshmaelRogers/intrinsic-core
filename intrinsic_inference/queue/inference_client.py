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

"""Bounded async request queue with a Submit / Poll / Cancel client facade.

Contract: issue #119, comment 5978277220. This is a host-side queue around a
worker (the stub worker by default, or the replay worker of #120 through the
worker argument). It does not call OIP or Triton, load models, open sockets,
or link into ICON or a HAL. submit, poll, cancel, reap, tick
and shutdown never sleep and never wait on a worker.
"""

from __future__ import annotations

import dataclasses
import enum
import re
import threading
import time
from typing import Callable, Optional

from intrinsic.embodiment import stamped_header_policy
from intrinsic_inference.envelope import inference_envelope_contract_policy as policy
from intrinsic_inference.queue import stub_worker as stub_worker_lib

# Fixed maximum of outstanding requests. Locked by the contract.
MAX_OUTSTANDING = 8

_ID_PREFIX = "inf-"
_ID_PATTERN = re.compile(r"inf-(?:[1-9][0-9]{0,19})")

_Error = policy.InferenceEnvelopeContractError


class InferenceQueueStatus(enum.Enum):
  """Single status taxonomy. Names mirror the C++ kXxx enumerators."""

  # Submit accepted, or Poll of a request that is still in progress.
  OK = 0
  # Submit while MAX_OUTSTANDING requests are outstanding. The new request is
  # rejected. Nothing already queued is dropped.
  QUEUE_FULL = 1
  # Poll, Cancel or Reap of an id that was never accepted or is already
  # reaped or cancelled.
  UNKNOWN_ID = 2
  # Cancel won. Terminal; the slot is released.
  CANCELLED = 3
  # Submit after Shutdown, or Poll, Cancel or Reap of a request that Shutdown
  # discarded.
  SHUTDOWN = 4
  # Cancel of a request that already completed.
  ALREADY_COMPLETE = 5
  # Submit of an envelope whose deadline is before its creation time or
  # before the submit clock. Not enqueued.
  DEADLINE_EXPIRED_AT_SUBMIT = 6
  # Poll or Reap of a completed request.
  COMPLETE = 7
  # Submit of an envelope that assess_inference_envelope does not accept for
  # a reason other than the deadline. Not enqueued.
  INVALID_ENVELOPE = 8
  # Replay worker only (#120). Poll or reap of a request that finished with no
  # result: no fixture for the (model, digest, epoch) match key.
  REPLAY_MISS = 9
  # Replay worker only. A fixture matched but its recorded deadline is strictly
  # before the clock. The recorded result is attached with timing unchanged.
  REPLAY_EXPIRED = 10
  # Replay worker only. A fixture matched the key but is unusable (incomplete
  # recorded result, or duplicate key).
  CORRUPT_FIXTURE = 11


_Status = InferenceQueueStatus


@dataclasses.dataclass(frozen=True)
class SubmitOutcome:
  status: InferenceQueueStatus = _Status.OK
  # Set only when status is OK.
  request_id: str = ""
  # The #118 assessor error behind DEADLINE_EXPIRED_AT_SUBMIT (when the
  # envelope itself is the cause) or INVALID_ENVELOPE.
  envelope_error: _Error = _Error.NONE


@dataclasses.dataclass(frozen=True)
class PollOutcome:
  status: InferenceQueueStatus = _Status.UNKNOWN_ID
  # Set when status is COMPLETE or REPLAY_EXPIRED.
  result: Optional[policy.InferenceResultView] = None


@dataclasses.dataclass(frozen=True)
class CancelOutcome:
  status: InferenceQueueStatus = _Status.UNKNOWN_ID


def _system_utc_now() -> policy.TimeParts:
  nanos_total = time.time_ns()
  return (nanos_total // 1_000_000_000, nanos_total % 1_000_000_000)


def _time_before(a: policy.TimeParts, b: policy.TimeParts) -> bool:
  return a[0] < b[0] or (a[0] == b[0] and a[1] < b[1])


def _format_id(request_number: int) -> str:
  return f"{_ID_PREFIX}{request_number}"


def _parse_id(text: str) -> Optional[int]:
  """Accepts only the canonical form _format_id produces."""
  if not isinstance(text, str) or not _ID_PATTERN.fullmatch(text):
    return None
  value = int(text[len(_ID_PREFIX) :])
  return value if value < 1 << 64 else None


@dataclasses.dataclass
class _Slot:
  number: int
  envelope: policy.InferenceEnvelopeView
  # None while in progress. Any worker status is terminal.
  status: Optional[InferenceQueueStatus] = None
  result: Optional[policy.InferenceResultView] = None


class InferenceClient:
  """Thread safe. One lock guards all state; no call waits on anything."""

  def __init__(
      self,
      clock: Optional[Callable[[], policy.TimeParts]] = None,
      complete_on_submit: bool = False,
      worker=None,
  ):
    """Creates an empty queue.

    Args:
      clock: Submit clock for the deadline check, returning (seconds, nanos).
        None uses the system UTC clock.
      complete_on_submit: When true the stub worker completes each request
        inside submit. When false a request stays in progress until tick.
      worker: Object with complete(envelope, queue_request_id, now) returning
        a stub_worker.WorkerOutcome. None uses StubWorker. now is the queue
        clock at the moment the worker produces the terminal result.
    """
    self._clock = clock or _system_utc_now
    self._complete_on_submit = complete_on_submit
    self._worker = worker or stub_worker_lib.StubWorker()
    self._lock = threading.Lock()
    self._slots: dict[int, _Slot] = {}
    self._discarded: set[int] = set()
    self._next_number = 1
    self._shutdown = False

  def submit(self, request: policy.InferenceEnvelopeView) -> SubmitOutcome:
    """First match: SHUTDOWN, envelope checks, QUEUE_FULL, then OK."""
    with self._lock:
      if self._shutdown:
        return SubmitOutcome(_Status.SHUTDOWN)

      assessment = policy.assess_inference_envelope(request)
      common = request.common
      if (
          assessment.error is _Error.DEADLINE
          and common.deadline_present
          and stamped_header_policy.nanos_in_range(common.deadline[1])
      ):
        # #118 rejects deadline < header.source_time; that is the same
        # deadline-before-creation rule.
        return SubmitOutcome(
            _Status.DEADLINE_EXPIRED_AT_SUBMIT, envelope_error=assessment.error
        )
      if assessment.error is not _Error.NONE or not assessment.accepted:
        return SubmitOutcome(
            _Status.INVALID_ENVELOPE, envelope_error=assessment.error
        )
      if _time_before(common.deadline, self._clock()):
        return SubmitOutcome(_Status.DEADLINE_EXPIRED_AT_SUBMIT)

      if len(self._slots) >= MAX_OUTSTANDING:
        return SubmitOutcome(_Status.QUEUE_FULL)

      number = self._next_number
      self._next_number += 1
      slot = _Slot(number=number, envelope=request)
      self._slots[number] = slot
      if self._complete_on_submit:
        self._complete(slot)
      return SubmitOutcome(_Status.OK, _format_id(number))

  def poll(self, request_id: str) -> PollOutcome:
    """OK while in progress, COMPLETE with the result once complete.

    poll does not release the slot, so repeated poll is idempotent.
    """
    with self._lock:
      slot = self._find(request_id)
      if slot is None:
        return PollOutcome(self._missing_status(request_id))
      if slot.status is not None:
        return PollOutcome(slot.status, slot.result)
      return PollOutcome(_Status.OK)

  def cancel(self, request_id: str) -> CancelOutcome:
    """In progress: CANCELLED and the slot is released.

    Complete: ALREADY_COMPLETE and the slot is kept. Exactly one terminal
    state wins.
    """
    with self._lock:
      slot = self._find(request_id)
      if slot is None:
        return CancelOutcome(self._missing_status(request_id))
      if slot.status is not None:
        return CancelOutcome(_Status.ALREADY_COMPLETE)
      del self._slots[slot.number]
      return CancelOutcome(_Status.CANCELLED)

  def reap(self, request_id: str) -> PollOutcome:
    """Releases a completed request and returns its result with COMPLETE.

    An in-progress request returns OK and is kept. This is the explicit reap
    that frees a slot after completion.
    """
    with self._lock:
      slot = self._find(request_id)
      if slot is None:
        return PollOutcome(self._missing_status(request_id))
      if slot.status is None:
        return PollOutcome(_Status.OK)
      del self._slots[slot.number]
      return PollOutcome(slot.status, slot.result)

  def tick(self, max_completions: int = 1) -> int:
    """Test and fake-async hook.

    Lets the worker complete up to max_completions in-progress requests,
    oldest first. Returns the number completed. Does nothing after shutdown.
    """
    with self._lock:
      if self._shutdown:
        return 0
      pending = sorted(
          number for number, slot in self._slots.items() if slot.status is None
      )
      completed = 0
      for number in pending[: max(max_completions, 0)]:
        self._complete(self._slots[number])
        completed += 1
      return completed

  def shutdown(self) -> None:
    """Idempotent. Rejects new submit. Discards every outstanding request."""
    with self._lock:
      if self._shutdown:
        return
      self._shutdown = True
      self._discarded.update(self._slots)
      self._slots.clear()

  def outstanding(self) -> int:
    with self._lock:
      return len(self._slots)

  def is_shutdown(self) -> bool:
    with self._lock:
      return self._shutdown

  def _complete(self, slot: _Slot) -> None:
    outcome = self._worker.complete(
        slot.envelope, _format_id(slot.number), self._clock()
    )
    slot.status = _Status[outcome.status.name]
    slot.result = outcome.result

  def _find(self, request_id: str) -> Optional[_Slot]:
    number = _parse_id(request_id)
    return None if number is None else self._slots.get(number)

  def _missing_status(self, request_id: str) -> InferenceQueueStatus:
    number = _parse_id(request_id)
    if number is not None and number in self._discarded:
      return _Status.SHUTDOWN
    return _Status.UNKNOWN_ID
