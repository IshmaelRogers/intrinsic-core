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

"""Tests for the deterministic replay worker and its queue integration."""

import dataclasses
import json
import os
import pathlib
import shutil
import tempfile
import unittest

from intrinsic_inference.envelope import inference_envelope_contract_policy as policy
from intrinsic_inference.queue import inference_client
from intrinsic_inference.queue import replay_worker
from intrinsic_inference.queue import stub_worker

_Status = inference_client.InferenceQueueStatus
_Error = policy.InferenceEnvelopeContractError
_SNAPSHOT = "0123456789abcdef" * 4
_FIXTURE_DIR = pathlib.Path(__file__).resolve().parent / "fixtures"


def _envelope(model="pose_estimator", digest="sha256:in", epoch=42):
  """Created at 1700000000s, deadline 1700000000.5s."""
  return policy.InferenceEnvelopeView(
      common=policy.InferenceCommonView(
          header_present=True,
          validity_present=True,
          validity_state=1,
          frame_id="world_enu",
          source_time_present=True,
          source_time=(1700000000, 0),
          state_epoch=epoch,
          world_snapshot_id=_SNAPSHOT,
          deadline_present=True,
          deadline=(1700000000, 500000000),
          validity_horizon_present=True,
          validity_horizon=(2, 0),
          confidence_present=True,
          confidence=0.9,
          input_digest=digest,
          provenance_present=True,
          provenance_model_id=model,
      ),
      oip_request=policy.OipIdentifierView(
          message_set=True,
          model_name="pose_estimator",
          model_version="3",
          request_id="req-0001",
      ),
  )


def _load():
  return replay_worker.ReplayWorker(str(_FIXTURE_DIR))


def _client(worker=None, now=(1700000000, 100), **kwargs):
  return inference_client.InferenceClient(
      clock=lambda: now, worker=worker or _load(), **kwargs
  )


def _submit_tick_poll(client, envelope):
  submitted = client.submit(envelope)
  assert submitted.status is _Status.OK, submitted
  assert client.tick() == 1
  return client.poll(submitted.request_id)


class _TempFixtureDir:
  """Scratch fixture directory, removed on exit."""

  def __enter__(self):
    self.path = tempfile.mkdtemp(prefix="replay_fixtures_")
    return self

  def __exit__(self, *exc_info):
    shutil.rmtree(self.path, ignore_errors=True)

  def write(self, name, text):
    with open(os.path.join(self.path, name), "w", encoding="utf-8") as f:
      f.write(text)

  def copy(self, name, as_name=None):
    shutil.copyfile(
        _FIXTURE_DIR / name, os.path.join(self.path, as_name or name)
    )


class ReplayWorkerTest(unittest.TestCase):

  # 1. Nominal hit

  def test_fixture_directory_has_at_most_eight_files_and_loads(self):
    files = [p for p in _FIXTURE_DIR.iterdir() if p.is_file()]
    self.assertLessEqual(len(files), 8)
    worker = _load()
    self.assertEqual(worker.fixture_count, 5)
    # Only the truncated file has no readable key.
    self.assertEqual(worker.skipped_files, ["corrupt_unparseable.json"])

  def test_nominal_hit_returns_recorded_result(self):
    client = _client()
    submitted = client.submit(_envelope())
    self.assertIs(submitted.status, _Status.OK)
    pending = client.poll(submitted.request_id)
    self.assertIs(pending.status, _Status.OK)
    self.assertIsNone(pending.result)

    self.assertEqual(client.tick(), 1)
    done = client.poll(submitted.request_id)
    self.assertIs(done.status, _Status.COMPLETE)
    result = done.result
    # Match key and recorded digests / provenance.
    self.assertEqual(result.common.provenance_model_id, "pose_estimator")
    self.assertEqual(result.common.input_digest, "sha256:in")
    self.assertEqual(result.common.state_epoch, 42)
    self.assertEqual(result.output_digest, "sha256:replay-out-nominal")
    # Recorded, not the envelope's, and not rewritten by the queue.
    self.assertEqual(result.oip_result.request_id, "rec-0001")
    self.assertEqual(result.oip_result.model_version, "3")
    self.assertEqual(result.common.frame_id, "world_enu")
    self.assertEqual(result.common.world_snapshot_id, _SNAPSHOT)
    # Recorded timing.
    self.assertEqual(result.common.source_time, (1700000000, 0))
    self.assertEqual(result.common.deadline, (1700000000, 500000000))
    self.assertEqual(result.common.validity_horizon, (2, 0))
    self.assertEqual(result.common.confidence, 0.9)
    self.assertEqual(result.common.uncertainty, 0.05)

    assessment = policy.assess_inference_result(result)
    self.assertIs(assessment.error, _Error.NONE)
    self.assertTrue(assessment.accepted)

  def test_complete_on_submit_hit(self):
    client = _client(complete_on_submit=True)
    submitted = client.submit(_envelope())
    done = client.poll(submitted.request_id)
    self.assertIs(done.status, _Status.COMPLETE)
    self.assertEqual(done.result.output_digest, "sha256:replay-out-nominal")

  def test_each_key_field_selects_its_own_row(self):
    client = _client()
    epoch43 = _submit_tick_poll(client, _envelope(epoch=43))
    self.assertIs(epoch43.status, _Status.COMPLETE)
    self.assertEqual(epoch43.result.output_digest, "sha256:replay-out-epoch-43")
    self.assertEqual(epoch43.result.common.state_epoch, 43)

    grasp = _submit_tick_poll(client, _envelope(model="grasp_planner"))
    self.assertIs(grasp.status, _Status.COMPLETE)
    self.assertEqual(grasp.result.output_digest, "sha256:replay-out-grasp")
    self.assertEqual(grasp.result.common.provenance_model_id, "grasp_planner")

  def test_world_snapshot_and_oip_ids_are_not_part_of_the_key(self):
    request = _envelope()
    request = dataclasses.replace(
        request,
        common=dataclasses.replace(
            request.common,
            world_snapshot_id="fedcba9876543210" * 4,
            confidence=0.1,
        ),
        oip_request=dataclasses.replace(
            request.oip_request,
            model_name="something_else",
            request_id="req-9999",
        ),
    )
    done = _submit_tick_poll(_client(), request)
    self.assertIs(done.status, _Status.COMPLETE)
    self.assertEqual(done.result.output_digest, "sha256:replay-out-nominal")

  # 2. Miss

  def test_unknown_key_is_replay_miss_with_no_result(self):
    client = _client()
    for model, digest, epoch in (
        ("unknown_model", "sha256:in", 42),
        ("pose_estimator", "sha256:unknown", 42),
        ("pose_estimator", "sha256:in", 99),
        # Key whose only fixture file is truncated, so it cannot be keyed.
        ("pose_estimator", "sha256:in-trunc", 42),
    ):
      with self.subTest(model=model, digest=digest, epoch=epoch):
        done = _submit_tick_poll(client, _envelope(model, digest, epoch))
        self.assertIs(done.status, _Status.REPLAY_MISS)
        self.assertIsNone(done.result)

  def test_missing_or_empty_directory_misses_everything(self):
    with _TempFixtureDir() as empty:
      for path in (empty.path, os.path.join(empty.path, "does_not_exist")):
        worker = replay_worker.ReplayWorker(path)
        self.assertEqual(worker.fixture_count, 0)
        done = _submit_tick_poll(_client(worker), _envelope())
        self.assertIs(done.status, _Status.REPLAY_MISS)

  # 3. Determinism

  def test_repeated_hit_is_identical(self):
    client = _client()
    first = _submit_tick_poll(client, _envelope())
    second = _submit_tick_poll(client, _envelope())
    third = _submit_tick_poll(_client(), _envelope())
    for outcome in (first, second, third):
      self.assertIs(outcome.status, _Status.COMPLETE)
    self.assertEqual(first.result, second.result)
    self.assertEqual(first.result, third.result)

  # 4. Late / expiry

  def test_recorded_deadline_before_clock_is_expired_with_timing_kept(self):
    # The request deadline (1700000000.5s) is live at the submit clock; only
    # the recorded deadline (1699999999.9s) is in the past.
    done = _submit_tick_poll(_client(), _envelope(digest="sha256:in-late"))
    self.assertIs(done.status, _Status.REPLAY_EXPIRED)
    result = done.result
    self.assertEqual(result.common.source_time, (1699999999, 0))
    self.assertEqual(result.common.deadline, (1699999999, 900000000))
    self.assertEqual(result.common.validity_horizon, (2, 0))
    self.assertEqual(result.output_digest, "sha256:replay-out-late")
    self.assertEqual(result.oip_result.request_id, "rec-0004")

  def test_result_is_late_when_clock_passes_deadline_before_tick(self):
    now = [(1700000000, 100)]
    client = inference_client.InferenceClient(
        clock=lambda: now[0], worker=_load()
    )
    submitted = client.submit(_envelope())
    self.assertIs(submitted.status, _Status.OK)
    now[0] = (1700000000, 600000000)  # Past the recorded deadline of .5s.
    self.assertEqual(client.tick(), 1)
    done = client.poll(submitted.request_id)
    self.assertIs(done.status, _Status.REPLAY_EXPIRED)
    self.assertEqual(done.result.common.deadline, (1700000000, 500000000))
    self.assertEqual(done.result.output_digest, "sha256:replay-out-nominal")

  def test_deadline_equal_to_clock_is_not_expired(self):
    client = _client(now=(1700000000, 500000000))
    self.assertIs(
        _submit_tick_poll(client, _envelope()).status, _Status.COMPLETE
    )
    late = _client(now=(1700000000, 500000001))
    # The request itself is now expired at submit, before the worker runs.
    self.assertIs(
        late.submit(_envelope()).status, _Status.DEADLINE_EXPIRED_AT_SUBMIT
    )

  def test_expired_result_is_still_queue_terminal(self):
    client = _client()
    submitted = client.submit(_envelope(digest="sha256:in-late"))
    self.assertEqual(client.tick(), 1)
    self.assertIs(
        client.cancel(submitted.request_id).status, _Status.ALREADY_COMPLETE
    )
    reaped = client.reap(submitted.request_id)
    self.assertIs(reaped.status, _Status.REPLAY_EXPIRED)
    self.assertIsNotNone(reaped.result)
    self.assertEqual(client.outstanding(), 0)
    self.assertIs(client.poll(submitted.request_id).status, _Status.UNKNOWN_ID)

  # 5. Corrupt fixture
  #
  # Contract choice, documented in the README: a file with a complete match
  # key and an unusable result is CORRUPT_FIXTURE. A file with no readable key
  # (unparseable, or incomplete key) cannot be addressed and is skipped, so
  # its requests are REPLAY_MISS.

  def test_incomplete_recorded_result_is_corrupt_fixture(self):
    done = _submit_tick_poll(_client(), _envelope(digest="sha256:in-corrupt"))
    self.assertIs(done.status, _Status.CORRUPT_FIXTURE)
    self.assertIsNone(done.result)

  def test_corrupt_fixture_does_not_affect_other_rows(self):
    self.assertIs(
        _submit_tick_poll(_client(), _envelope()).status, _Status.COMPLETE
    )

  def test_result_that_fails_assessment_is_corrupt_fixture(self):
    with _TempFixtureDir() as directory:
      # Deadline before source_time: assess_inference_result reports DEADLINE.
      directory.write(
          "bad_deadline.json",
          json.dumps(
              {
                  "match": {
                      "provenance_model_id": "pose_estimator",
                      "input_digest": "sha256:in",
                      "state_epoch": 42,
                  },
                  "result": {
                      "frame_id": "world_enu",
                      "validity_state": 1,
                      "source_time": {"seconds": 10, "nanos": 0},
                      "deadline": {"seconds": 5, "nanos": 0},
                      "validity_horizon": {"seconds": 2, "nanos": 0},
                      "oip": {"model_name": "m", "request_id": "r"},
                      "output_digest": "sha256:o",
                  },
              }
          ),
      )
      worker = replay_worker.ReplayWorker(directory.path)
      done = _submit_tick_poll(_client(worker), _envelope())
      self.assertIs(done.status, _Status.CORRUPT_FIXTURE)

  def test_duplicate_key_is_corrupt_fixture(self):
    with _TempFixtureDir() as directory:
      directory.copy("hit_nominal.json")
      directory.copy("hit_other_epoch.json")
      # Same key as hit_nominal.json under another name.
      directory.copy("hit_nominal.json", as_name="zz_copy.json")
      worker = replay_worker.ReplayWorker(directory.path)
      self.assertEqual(worker.fixture_count, 2)
      client = _client(worker)
      self.assertIs(
          _submit_tick_poll(client, _envelope()).status,
          _Status.CORRUPT_FIXTURE,
      )
      self.assertIs(
          _submit_tick_poll(client, _envelope(epoch=43)).status,
          _Status.COMPLETE,
      )

  def test_files_without_a_key_are_skipped_and_other_extensions_ignored(self):
    with _TempFixtureDir() as directory:
      directory.write("not_json.txt", "{}")
      directory.write("empty.json", "")
      directory.write("array.json", "[]")
      directory.write("no_key.json", '{"match": {"input_digest": "sha256:in"}}')
      directory.write(
          "bad_epoch.json",
          '{"match": {"provenance_model_id": "m", "input_digest": "d",'
          ' "state_epoch": -1}}',
      )
      directory.write(
          "nan.json",
          '{"match": {"provenance_model_id": "m", "input_digest": "d",'
          ' "state_epoch": 1}, "result": {"confidence": NaN}}',
      )
      worker = replay_worker.ReplayWorker(directory.path)
      self.assertEqual(worker.fixture_count, 0)
      self.assertEqual(
          worker.skipped_files,
          [
              "array.json",
              "bad_epoch.json",
              "empty.json",
              "nan.json",
              "no_key.json",
          ],
      )


class ReplayQueueSmokeTest(unittest.TestCase):
  """The #119 queue rules still hold with a ReplayWorker."""

  def test_capacity_eight_reject_new_and_unknown_id(self):
    client = _client()
    ids = []
    for _ in range(inference_client.MAX_OUTSTANDING):
      outcome = client.submit(_envelope())
      self.assertIs(outcome.status, _Status.OK)
      ids.append(outcome.request_id)
    self.assertEqual(client.outstanding(), inference_client.MAX_OUTSTANDING)
    self.assertIs(client.submit(_envelope()).status, _Status.QUEUE_FULL)
    self.assertEqual(client.outstanding(), inference_client.MAX_OUTSTANDING)

    self.assertIs(client.poll("inf-99").status, _Status.UNKNOWN_ID)
    self.assertIs(client.cancel("inf-99").status, _Status.UNKNOWN_ID)
    self.assertIs(client.reap("inf-99").status, _Status.UNKNOWN_ID)

    # The eight accepted requests are intact and complete from the fixture.
    self.assertEqual(
        client.tick(inference_client.MAX_OUTSTANDING),
        inference_client.MAX_OUTSTANDING,
    )
    for request_id in ids:
      self.assertIs(client.poll(request_id).status, _Status.COMPLETE)
    # A terminal replay status releases its slot on reap like any other.
    self.assertIs(client.reap(ids[0]).status, _Status.COMPLETE)
    miss = client.submit(_envelope(model="nobody", epoch=1))
    self.assertIs(miss.status, _Status.OK)
    self.assertEqual(miss.request_id, "inf-9")
    self.assertEqual(client.tick(), 1)
    self.assertIs(client.poll(miss.request_id).status, _Status.REPLAY_MISS)
    self.assertIs(client.reap(miss.request_id).status, _Status.REPLAY_MISS)
    self.assertIs(client.poll(miss.request_id).status, _Status.UNKNOWN_ID)

  def test_cancel_before_tick_and_shutdown_still_work(self):
    client = _client()
    cancelled = client.submit(_envelope())
    self.assertIs(client.cancel(cancelled.request_id).status, _Status.CANCELLED)
    self.assertEqual(client.tick(), 0)
    pending = client.submit(_envelope())
    client.shutdown()
    self.assertIs(client.poll(pending.request_id).status, _Status.SHUTDOWN)
    self.assertIs(client.submit(_envelope()).status, _Status.SHUTDOWN)

  def test_default_client_still_uses_stub_worker(self):
    client = inference_client.InferenceClient(clock=lambda: (1700000000, 100))
    done = _submit_tick_poll(client, _envelope())
    self.assertIs(done.status, _Status.COMPLETE)
    self.assertEqual(done.result.output_digest, "stub-output:inf-1")

  def test_replay_worker_returns_worker_outcome(self):
    outcome = _load().complete(_envelope(), "inf-1", (1700000000, 0))
    self.assertIs(outcome.status, stub_worker.WorkerStatus.COMPLETE)
    self.assertIsNotNone(outcome.result)


if __name__ == "__main__":
  unittest.main()
