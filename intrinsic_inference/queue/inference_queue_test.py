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

"""Tests for the bounded inference queue, client facade, and stub worker."""

import dataclasses
import threading
import unittest

from intrinsic_inference.envelope import inference_envelope_contract_policy as policy
from intrinsic_inference.queue import inference_client
from intrinsic_inference.queue import stub_worker

_Status = inference_client.InferenceQueueStatus
_Error = policy.InferenceEnvelopeContractError
_SNAPSHOT = "0123456789abcdef" * 4


def _nominal_envelope(**common_overrides):
  """Created at 1700000000s, deadline 1700000000.5s."""
  common = dict(
      header_present=True,
      validity_present=True,
      validity_state=1,
      frame_id="world_enu",
      source_time_present=True,
      source_time=(1700000000, 0),
      state_epoch=42,
      world_snapshot_id=_SNAPSHOT,
      deadline_present=True,
      deadline=(1700000000, 500000000),
      validity_horizon_present=True,
      validity_horizon=(2, 0),
      confidence_present=True,
      confidence=0.9,
      input_digest="sha256:in",
      provenance_present=True,
      provenance_model_id="pose_estimator",
  )
  common.update(common_overrides)
  return policy.InferenceEnvelopeView(
      common=policy.InferenceCommonView(**common),
      oip_request=policy.OipIdentifierView(
          message_set=True,
          model_name="pose_estimator",
          model_version="3",
          request_id="req-0001",
      ),
  )


def _client(now=(1700000000, 100), **kwargs):
  return inference_client.InferenceClient(clock=lambda: now, **kwargs)


def _must_submit(client):
  outcome = client.submit(_nominal_envelope())
  assert outcome.status is _Status.OK, outcome
  return outcome.request_id


class InferenceQueueTest(unittest.TestCase):

  def test_capacity_is_eight(self):
    self.assertEqual(inference_client.MAX_OUTSTANDING, 8)

  def test_empty_queue_unknown_id(self):
    client = _client()
    self.assertEqual(client.outstanding(), 0)
    self.assertIs(client.poll("inf-1").status, _Status.UNKNOWN_ID)
    self.assertIs(client.cancel("inf-1").status, _Status.UNKNOWN_ID)
    self.assertIs(client.reap("inf-1").status, _Status.UNKNOWN_ID)
    self.assertEqual(client.tick(), 0)
    self.assertEqual(client.outstanding(), 0)

  def test_unknown_id_never_invents_a_row(self):
    client = _client()
    request_id = _must_submit(client)
    for bad in (
        "",
        "inf-",
        "inf-0",
        "inf-01",
        "inf-+1",
        "inf- 1",
        "x-1",
        "inf-1x",
        "inf-99999999999999999999999",
        "INF-1",
        "inf-2",
        "inf-9",
    ):
      with self.subTest(bad=bad):
        self.assertIs(client.poll(bad).status, _Status.UNKNOWN_ID)
        self.assertIs(client.cancel(bad).status, _Status.UNKNOWN_ID)
        self.assertIs(client.reap(bad).status, _Status.UNKNOWN_ID)
    self.assertEqual(client.outstanding(), 1)
    self.assertIs(client.poll(request_id).status, _Status.OK)

  def test_one_item_submit_poll_tick_poll(self):
    client = _client()
    request_id = _must_submit(client)
    self.assertEqual(request_id, "inf-1")
    self.assertEqual(client.outstanding(), 1)

    pending = client.poll(request_id)
    self.assertIs(pending.status, _Status.OK)
    self.assertIsNone(pending.result)

    self.assertEqual(client.tick(), 1)
    done = client.poll(request_id)
    self.assertIs(done.status, _Status.COMPLETE)
    self.assertEqual(done.result.oip_result.request_id, "req-0001")
    self.assertEqual(done.result.oip_result.model_name, "pose_estimator")
    self.assertEqual(done.result.common.frame_id, "world_enu")
    self.assertEqual(done.result.common.input_digest, "sha256:in")
    self.assertEqual(done.result.output_digest, "stub-output:inf-1")
    assessment = policy.assess_inference_result(done.result)
    self.assertIs(assessment.error, _Error.NONE)
    self.assertTrue(assessment.accepted)
    self.assertEqual(client.outstanding(), 1)

  def test_complete_on_submit(self):
    client = _client(complete_on_submit=True)
    request_id = _must_submit(client)
    self.assertIs(client.poll(request_id).status, _Status.COMPLETE)
    self.assertEqual(client.tick(), 0)

  def test_full_queue_rejects_ninth_and_keeps_the_eight(self):
    client = _client()
    ids = [_must_submit(client) for _ in range(8)]
    self.assertEqual(client.outstanding(), 8)

    ninth = client.submit(_nominal_envelope())
    self.assertIs(ninth.status, _Status.QUEUE_FULL)
    self.assertEqual(ninth.request_id, "")
    self.assertEqual(client.outstanding(), 8)

    # Reject new: neither the oldest nor the newest accepted request is gone.
    for request_id in ids:
      self.assertIs(client.poll(request_id).status, _Status.OK, request_id)
    self.assertEqual(client.tick(8), 8)
    for request_id in ids:
      self.assertIs(
          client.poll(request_id).status, _Status.COMPLETE, request_id
      )
    # Completed-but-not-reaped still occupies a slot.
    self.assertIs(client.submit(_nominal_envelope()).status, _Status.QUEUE_FULL)

  def test_cancel_and_reap_free_slots(self):
    client = _client()
    ids = [_must_submit(client) for _ in range(8)]
    self.assertIs(client.cancel(ids[3]).status, _Status.CANCELLED)
    self.assertEqual(client.outstanding(), 7)
    self.assertEqual(_must_submit(client), "inf-9")
    self.assertIs(client.submit(_nominal_envelope()).status, _Status.QUEUE_FULL)

    client.tick()
    self.assertIs(client.poll(ids[0]).status, _Status.COMPLETE)
    reaped = client.reap(ids[0])
    self.assertIs(reaped.status, _Status.COMPLETE)
    self.assertIsNotNone(reaped.result)
    self.assertIs(client.poll(ids[0]).status, _Status.UNKNOWN_ID)
    self.assertIs(client.reap(ids[0]).status, _Status.UNKNOWN_ID)
    self.assertEqual(client.outstanding(), 7)
    self.assertEqual(_must_submit(client), "inf-10")

  def test_reap_of_in_progress_keeps_the_slot(self):
    client = _client()
    request_id = _must_submit(client)
    outcome = client.reap(request_id)
    self.assertIs(outcome.status, _Status.OK)
    self.assertIsNone(outcome.result)
    self.assertEqual(client.outstanding(), 1)
    self.assertIs(client.poll(request_id).status, _Status.OK)

  def test_overload_storm_rejects_new_and_stays_bounded(self):
    client = _client()
    accepted = rejected = 0
    for _ in range(5000):
      outcome = client.submit(_nominal_envelope())
      if outcome.status is _Status.OK:
        accepted += 1
      else:
        self.assertIs(outcome.status, _Status.QUEUE_FULL)
        rejected += 1
      self.assertLessEqual(client.outstanding(), 8)
    self.assertEqual(accepted, 8)
    self.assertEqual(rejected, 5000 - 8)
    # The earliest eight stay; nothing newer displaced them.
    for number in range(1, 9):
      self.assertIs(client.poll(f"inf-{number}").status, _Status.OK)
    self.assertIs(client.poll("inf-9").status, _Status.UNKNOWN_ID)

  def test_concurrent_overload_never_exceeds_capacity(self):
    client = _client()
    threads_count, per_thread = 8, 200
    results = [[] for _ in range(threads_count)]
    max_seen = [0] * threads_count

    def submitter(index):
      for _ in range(per_thread):
        results[index].append(client.submit(_nominal_envelope()))
        max_seen[index] = max(max_seen[index], client.outstanding())

    threads = [
        threading.Thread(target=submitter, args=(i,))
        for i in range(threads_count)
    ]
    for thread in threads:
      thread.start()
    for thread in threads:
      thread.join()

    outcomes = [o for per in results for o in per]
    accepted = [o for o in outcomes if o.status is _Status.OK]
    full = [o for o in outcomes if o.status is _Status.QUEUE_FULL]
    self.assertEqual(len(accepted), 8)
    self.assertEqual(len(full), threads_count * per_thread - 8)
    self.assertEqual(len({o.request_id for o in accepted}), 8)
    self.assertLessEqual(max(max_seen), 8)
    self.assertEqual(client.outstanding(), 8)

  def test_shutdown_rejects_submit_even_with_capacity(self):
    client = _client()
    self.assertFalse(client.is_shutdown())
    client.shutdown()
    self.assertTrue(client.is_shutdown())
    self.assertEqual(client.outstanding(), 0)
    outcome = client.submit(_nominal_envelope())
    self.assertIs(outcome.status, _Status.SHUTDOWN)
    self.assertEqual(outcome.request_id, "")
    self.assertEqual(client.outstanding(), 0)

  def test_shutdown_is_idempotent_and_discards_outstanding(self):
    client = _client()
    ids = [_must_submit(client) for _ in range(8)]
    client.tick(3)  # Three complete, five in progress; all are discarded.
    client.shutdown()
    client.shutdown()
    self.assertEqual(client.outstanding(), 0)
    for request_id in ids:
      self.assertIs(client.poll(request_id).status, _Status.SHUTDOWN)
      self.assertIs(client.cancel(request_id).status, _Status.SHUTDOWN)
      self.assertIs(client.reap(request_id).status, _Status.SHUTDOWN)
    self.assertIs(client.poll("inf-99").status, _Status.UNKNOWN_ID)
    self.assertIs(client.cancel("inf-99").status, _Status.UNKNOWN_ID)
    self.assertEqual(client.tick(8), 0)
    self.assertIs(client.submit(_nominal_envelope()).status, _Status.SHUTDOWN)

  def test_shutdown_does_not_relabel_earlier_cancel_or_reap(self):
    client = _client()
    cancelled = _must_submit(client)
    reaped = _must_submit(client)
    kept = _must_submit(client)
    self.assertIs(client.cancel(cancelled).status, _Status.CANCELLED)
    client.tick(2)
    self.assertIs(client.reap(reaped).status, _Status.COMPLETE)
    client.shutdown()
    self.assertIs(client.poll(cancelled).status, _Status.UNKNOWN_ID)
    self.assertIs(client.poll(reaped).status, _Status.UNKNOWN_ID)
    self.assertIs(client.poll(kept).status, _Status.SHUTDOWN)

  def test_duplicate_poll_is_idempotent(self):
    client = _client()
    request_id = _must_submit(client)
    for _ in range(5):
      self.assertIs(client.poll(request_id).status, _Status.OK)
    client.tick()
    first = client.poll(request_id)
    self.assertIs(first.status, _Status.COMPLETE)
    for _ in range(5):
      again = client.poll(request_id)
      self.assertIs(again.status, _Status.COMPLETE)
      self.assertEqual(again.result, first.result)
    self.assertEqual(client.outstanding(), 1)

  def test_cancel_in_progress(self):
    client = _client()
    request_id = _must_submit(client)
    self.assertIs(client.cancel(request_id).status, _Status.CANCELLED)
    self.assertEqual(client.outstanding(), 0)
    self.assertIs(client.poll(request_id).status, _Status.UNKNOWN_ID)
    self.assertIs(client.cancel(request_id).status, _Status.UNKNOWN_ID)
    self.assertEqual(client.tick(), 0)
    self.assertIs(client.poll(request_id).status, _Status.UNKNOWN_ID)

  def test_cancel_after_complete_is_already_complete(self):
    client = _client()
    request_id = _must_submit(client)
    client.tick()
    self.assertIs(client.cancel(request_id).status, _Status.ALREADY_COMPLETE)
    self.assertIs(client.cancel(request_id).status, _Status.ALREADY_COMPLETE)
    self.assertIs(client.poll(request_id).status, _Status.COMPLETE)
    self.assertEqual(client.outstanding(), 1)

  def test_tick_completes_oldest_first(self):
    client = _client()
    a = _must_submit(client)
    b = _must_submit(client)
    c = _must_submit(client)
    self.assertIs(client.cancel(a).status, _Status.CANCELLED)
    d = _must_submit(client)
    self.assertEqual(client.tick(1), 1)
    self.assertIs(client.poll(b).status, _Status.COMPLETE)
    self.assertIs(client.poll(c).status, _Status.OK)
    self.assertIs(client.poll(d).status, _Status.OK)
    self.assertEqual(client.tick(10), 2)
    self.assertEqual(client.tick(10), 0)

  def test_deadline_before_creation_rejected_at_submit(self):
    client = _client()
    envelope = _nominal_envelope(deadline=(1699999999, 999999999))
    # Same rule as the #118 assessor.
    self.assertIs(
        policy.assess_inference_envelope(envelope).error, _Error.DEADLINE
    )
    outcome = client.submit(envelope)
    self.assertIs(outcome.status, _Status.DEADLINE_EXPIRED_AT_SUBMIT)
    self.assertIs(outcome.envelope_error, _Error.DEADLINE)
    self.assertEqual(outcome.request_id, "")
    self.assertEqual(client.outstanding(), 0)

  def test_deadline_before_submit_clock_rejected_at_submit(self):
    client = _client(now=(1700000000, 500000001))
    outcome = client.submit(_nominal_envelope())
    self.assertIs(outcome.status, _Status.DEADLINE_EXPIRED_AT_SUBMIT)
    self.assertIs(outcome.envelope_error, _Error.NONE)
    self.assertEqual(client.outstanding(), 0)

  def test_deadline_equal_to_submit_clock_is_accepted(self):
    client = _client(now=(1700000000, 500000000))
    self.assertIs(client.submit(_nominal_envelope()).status, _Status.OK)
    equal_creation = _nominal_envelope(deadline=(1700000000, 0))
    at_creation = _client(now=(1700000000, 0))
    self.assertIs(at_creation.submit(equal_creation).status, _Status.OK)

  def test_expired_deadline_is_rejected_even_when_full(self):
    client = _client()
    for _ in range(8):
      _must_submit(client)
    envelope = _nominal_envelope(deadline=(1600000000, 0))
    self.assertIs(
        client.submit(envelope).status, _Status.DEADLINE_EXPIRED_AT_SUBMIT
    )
    self.assertEqual(client.outstanding(), 8)

  def test_shutdown_wins_over_expired_deadline(self):
    client = _client()
    client.shutdown()
    envelope = _nominal_envelope(deadline=(1600000000, 0))
    self.assertIs(client.submit(envelope).status, _Status.SHUTDOWN)

  def test_missing_deadline_is_invalid_not_expired(self):
    client = _client()
    outcome = client.submit(_nominal_envelope(deadline_present=False))
    self.assertIs(outcome.status, _Status.INVALID_ENVELOPE)
    self.assertIs(outcome.envelope_error, _Error.DEADLINE)

  def test_unaccepted_envelopes_are_not_enqueued(self):
    client = _client()
    self.assertIs(
        client.submit(policy.InferenceEnvelopeView()).status,
        _Status.INVALID_ENVELOPE,
    )
    outcome = client.submit(_nominal_envelope(frame_id=""))
    self.assertIs(outcome.status, _Status.INVALID_ENVELOPE)
    self.assertIs(outcome.envelope_error, _Error.MISSING_FRAME)
    self.assertIs(
        client.submit(_nominal_envelope(validity_state=2)).status,
        _Status.INVALID_ENVELOPE,
    )
    self.assertEqual(client.outstanding(), 0)

  def test_cancel_races_tick_exactly_one_terminal_wins(self):
    for _ in range(300):
      client = _client()
      request_id = _must_submit(client)
      box = {}
      canceller = threading.Thread(
          target=lambda: box.update(cancel=client.cancel(request_id))
      )
      worker = threading.Thread(target=lambda: box.update(tick=client.tick()))
      canceller.start()
      worker.start()
      canceller.join()
      worker.join()
      if box["cancel"].status is _Status.CANCELLED:
        self.assertEqual(box["tick"], 0)
        self.assertIs(client.poll(request_id).status, _Status.UNKNOWN_ID)
        self.assertEqual(client.outstanding(), 0)
      else:
        self.assertIs(box["cancel"].status, _Status.ALREADY_COMPLETE)
        self.assertEqual(box["tick"], 1)
        self.assertIs(client.poll(request_id).status, _Status.COMPLETE)
        self.assertEqual(client.outstanding(), 1)

  def test_shutdown_races_submit_poll_cancel(self):
    for _ in range(100):
      client = _client()
      ids = [_must_submit(client) for _ in range(4)]
      go = threading.Event()
      submits, polls, cancels = [], [], []

      def submitter():
        go.wait()
        for _ in range(8):
          submits.append(client.submit(_nominal_envelope()).status)

      def poller():
        go.wait()
        for request_id in ids:
          polls.append(client.poll(request_id).status)

      def canceller():
        go.wait()
        for request_id in ids:
          cancels.append(client.cancel(request_id).status)

      def shutter():
        go.wait()
        client.shutdown()

      threads = [
          threading.Thread(target=fn)
          for fn in (submitter, poller, canceller, shutter)
      ]
      for thread in threads:
        thread.start()
      go.set()
      for thread in threads:
        thread.join()

      self.assertLessEqual(
          set(submits), {_Status.OK, _Status.SHUTDOWN, _Status.QUEUE_FULL}
      )
      self.assertLessEqual(
          set(polls), {_Status.OK, _Status.SHUTDOWN, _Status.UNKNOWN_ID}
      )
      self.assertLessEqual(
          set(cancels),
          {_Status.CANCELLED, _Status.SHUTDOWN, _Status.UNKNOWN_ID},
      )
      self.assertTrue(client.is_shutdown())
      self.assertEqual(client.outstanding(), 0)
      self.assertIs(client.submit(_nominal_envelope()).status, _Status.SHUTDOWN)
      self.assertEqual(client.tick(8), 0)


class StubWorkerTest(unittest.TestCase):

  def test_result_passes_assess_inference_result(self):
    result = stub_worker.StubWorker().run(_nominal_envelope(), "inf-7")
    self.assertEqual(result.output_digest, "stub-output:inf-7")
    self.assertTrue(policy.assess_inference_result(result).accepted)

  def test_result_does_not_alias_the_envelope(self):
    envelope = _nominal_envelope()
    result = stub_worker.StubWorker().run(envelope, "inf-7")
    self.assertEqual(result.common, envelope.common)
    self.assertIsNot(result.common, envelope.common)
    self.assertEqual(
        dataclasses.asdict(result.oip_result)["request_id"], "req-0001"
    )


if __name__ == "__main__":
  unittest.main()
