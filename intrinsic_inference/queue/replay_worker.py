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

"""Deterministic replay worker. Contract: issue #120, comment 6009314342.

Returns a recorded InferenceResult for a request instead of running a model.
No OIP or Triton call, no model load, no socket, no GPU, no sleep, and no clock
of its own: expiry uses the queue clock passed to complete. Fixtures are read
once, in the constructor.

Match key, exact equality on all three request envelope fields:
common.provenance_model_id, common.input_digest, common.state_epoch. Nothing
else selects a fixture. A key with no fixture is REPLAY_MISS.

Fixture format and layout: see README.md ("Replay worker").
"""

from __future__ import annotations

import dataclasses
import json
import os
from typing import Any, Optional

from intrinsic_inference.envelope import inference_envelope_contract_policy as policy
from intrinsic_inference.queue import stub_worker

_INT64_MIN = -(1 << 63)
_INT64_MAX = (1 << 63) - 1
_INT32_MIN = -(1 << 31)
_INT32_MAX = (1 << 31) - 1
_UINT64_MAX = (1 << 64) - 1

_Key = tuple[str, str, int]


def _reject_constant(name: str) -> None:
  raise ValueError(f"non-finite JSON constant {name}")


class _BadFixture(Exception):
  """A required fixture field is missing or mistyped."""


def _get_string(obj: dict[str, Any], key: str) -> str:
  value = obj.get(key)
  if not isinstance(value, str):
    raise _BadFixture(key)
  return value


def _get_int(obj: dict[str, Any], key: str, low: int, high: int) -> int:
  value = obj.get(key)
  if isinstance(value, bool) or not isinstance(value, int):
    raise _BadFixture(key)
  if not low <= value <= high:
    raise _BadFixture(key)
  return value


def _get_number(obj: dict[str, Any], key: str) -> float:
  value = obj.get(key)
  if isinstance(value, bool) or not isinstance(value, (int, float)):
    raise _BadFixture(key)
  return float(value)


def _get_object(obj: dict[str, Any], key: str) -> dict[str, Any]:
  value = obj.get(key)
  if not isinstance(value, dict):
    raise _BadFixture(key)
  return value


def _get_time(obj: dict[str, Any], key: str) -> policy.TimeParts:
  value = _get_object(obj, key)
  return (
      _get_int(value, "seconds", _INT64_MIN, _INT64_MAX),
      _get_int(value, "nanos", _INT32_MIN, _INT32_MAX),
  )


def _parse_key(root: dict[str, Any]) -> Optional[_Key]:
  """The match key is the only thing that lets a fixture be addressed."""
  match = root.get("match")
  if not isinstance(match, dict):
    return None
  try:
    return (
        _get_string(match, "provenance_model_id"),
        _get_string(match, "input_digest"),
        _get_int(match, "state_epoch", 0, _UINT64_MAX),
    )
  except _BadFixture:
    return None


def _parse_result(
    root: dict[str, Any], key: _Key
) -> Optional[policy.InferenceResultView]:
  """Builds the recorded result, or None when it is unusable.

  Unusable means a required field is missing or mistyped, or the result would
  not pass assess_inference_result. The match key supplies the result's
  provenance model id, input digest, and state epoch, so the two can never
  disagree.
  """
  model, digest, epoch = key
  try:
    result = _get_object(root, "result")
    oip = _get_object(result, "oip")
    world_snapshot_id = ""
    if "world_snapshot_id" in result:
      world_snapshot_id = _get_string(result, "world_snapshot_id")
    confidence_present = "confidence" in result
    uncertainty_present = "uncertainty" in result
    view = policy.InferenceResultView(
        common=policy.InferenceCommonView(
            header_present=True,
            validity_present=True,
            validity_state=_get_int(result, "validity_state", 0, 3),
            frame_id=_get_string(result, "frame_id"),
            source_time_present=True,
            source_time=_get_time(result, "source_time"),
            state_epoch=epoch,
            world_snapshot_id=world_snapshot_id,
            deadline_present=True,
            deadline=_get_time(result, "deadline"),
            validity_horizon_present=True,
            validity_horizon=_get_time(result, "validity_horizon"),
            confidence_present=confidence_present,
            confidence=(
                _get_number(result, "confidence") if confidence_present else 0.0
            ),
            uncertainty_present=uncertainty_present,
            uncertainty=(
                _get_number(result, "uncertainty")
                if uncertainty_present
                else 0.0
            ),
            input_digest=digest,
            provenance_present=True,
            provenance_model_id=model,
        ),
        oip_result=policy.OipIdentifierView(
            message_set=True,
            model_name=_get_string(oip, "model_name"),
            model_version=(
                _get_string(oip, "model_version")
                if "model_version" in oip
                else ""
            ),
            request_id=_get_string(oip, "request_id"),
        ),
        output_digest=_get_string(result, "output_digest"),
    )
  except _BadFixture:
    return None
  if policy.assess_inference_result(view).error is not (
      policy.InferenceEnvelopeContractError.NONE
  ):
    return None
  return view


def _time_before(a: policy.TimeParts, b: policy.TimeParts) -> bool:
  return a[0] < b[0] or (a[0] == b[0] and a[1] < b[1])


class ReplayWorker:
  """Immutable after construction, so safe to share between threads."""

  def __init__(self, fixture_dir: str):
    """Loads every "*.json" file directly inside fixture_dir, sorted by name.

    A missing or unreadable directory loads nothing and every request is
    REPLAY_MISS.

    Args:
      fixture_dir: Directory holding one JSON file per recorded result.
    """
    # None marks a corrupt row.
    self._rows: dict[_Key, Optional[policy.InferenceResultView]] = {}
    self._skipped_files: list[str] = []
    try:
      names = sorted(
          name
          for name in os.listdir(fixture_dir)
          if name.endswith(".json")
          and os.path.isfile(os.path.join(fixture_dir, name))
      )
    except OSError:
      return
    for name in names:
      try:
        with open(os.path.join(fixture_dir, name), "rb") as f:
          root = json.loads(
              f.read().decode("utf-8"), parse_constant=_reject_constant
          )
      except (OSError, ValueError):
        self._skipped_files.append(name)
        continue
      key = _parse_key(root) if isinstance(root, dict) else None
      if key is None:
        self._skipped_files.append(name)
        continue
      if key in self._rows:
        # Two files claim the same key. Neither can be trusted.
        self._rows[key] = None
        continue
      self._rows[key] = _parse_result(root, key)

  @property
  def fixture_count(self) -> int:
    """Fixture keys held, usable or corrupt."""
    return len(self._rows)

  @property
  def skipped_files(self) -> list[str]:
    """File names that were unparseable or lacked a complete match key.

    Requests for such a key are REPLAY_MISS. In sorted order.
    """
    return list(self._skipped_files)

  def complete(
      self,
      envelope: policy.InferenceEnvelopeView,
      queue_request_id: str,
      now: policy.TimeParts,
  ) -> stub_worker.WorkerOutcome:
    """Hit: COMPLETE, or REPLAY_EXPIRED when the recorded deadline < now.

    A key whose fixture is unusable gives CORRUPT_FIXTURE. Anything else is
    REPLAY_MISS. The recorded result is returned as written; nothing is
    rewritten, including the queue request id.
    """
    del queue_request_id
    common = envelope.common
    key = (common.provenance_model_id, common.input_digest, common.state_epoch)
    if key not in self._rows:
      return stub_worker.WorkerOutcome(stub_worker.WorkerStatus.REPLAY_MISS)
    recorded = self._rows[key]
    if recorded is None:
      return stub_worker.WorkerOutcome(stub_worker.WorkerStatus.CORRUPT_FIXTURE)
    expired = _time_before(recorded.common.deadline, now)
    return stub_worker.WorkerOutcome(
        stub_worker.WorkerStatus.REPLAY_EXPIRED
        if expired
        else stub_worker.WorkerStatus.COMPLETE,
        dataclasses.replace(recorded),
    )
