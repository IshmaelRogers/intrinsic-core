# Bounded inference request queue

Opt-in bounded async request queue with a `Submit` / `Poll` / `Cancel` /
`Shutdown` client and a stub worker. Contract: issue #119, comment
[`5978277220`](https://github.com/IshmaelRogers/intrinsic-core/issues/119#issuecomment-5978277220).
Parent work package #37. Depends on the #118 envelope host policy in
[`../envelope`](../envelope/README.md).

Not here: any Triton or OIP call, model load, socket, or backend plugin
registry; ICON, HAL, World, or Gazebo; changes to the OIP, Triton, or
envelope protobufs. The queue is not linked into any ICON or HAL target and
is not listed in `.github/baseline/manipulator_targets.tsv`.

## Policy (locked)

| Rule | Value |
| --- | --- |
| Capacity | `kMaxOutstanding = 8` / `MAX_OUTSTANDING = 8` |
| Queue full | Submit rejects the new request with `kQueueFull`. Nothing is dropped or overwritten. |
| Memory | A fixed array of 8 slots. Heap is O(capacity). |
| Blocking | No call sleeps or waits on a worker. One short mutex guards state. |

Outstanding means every request still held: in progress, or completed and not
yet reaped.

## API

- C++, namespace `intrinsic::inference`: `InferenceClient` in
  `inference_client.h`, `StubWorker` in `stub_worker.h`.
- Python: `inference_client.InferenceClient` and `stub_worker.StubWorker`,
  with lower-case method names (`submit`, `poll`, `cancel`, `reap`, `tick`,
  `shutdown`) and upper-case status names (`QUEUE_FULL`).

The request and result types are the #118 plain-value views
(`InferenceEnvelopeView`, `InferenceResultView`). Like the #118 policy, the
queue does not parse protobuf. Submit copies the envelope, so caller strings
need not outlive the call.

| Operation | Result |
| --- | --- |
| `Submit(envelope)` | `SubmitOutcome`: status, `request_id` (`inf-<n>`, never reused), and the #118 assessor error behind a rejection. |
| `Poll(id)` | `kOk` while in progress. `kComplete` plus the result once complete. Never releases the slot. |
| `Cancel(id)` | `kCancelled` for an in-progress request (slot released). `kAlreadyComplete` for a completed one (slot kept). |
| `Reap(id)` | Releases a completed request and returns it with `kComplete`. `kOk` and no change if still in progress. |
| `Tick(n)` | Test and fake-async hook. The stub worker completes up to `n` in-progress requests, oldest first. Returns the count. |
| `Shutdown()` | Idempotent. Rejects new Submit. Discards every outstanding request. |

`Reap` is the one addition to the four locked operations. The contract keeps a
completed request pollable "until reaped" but names no reaper, and without one
eight completed requests would fill the queue for good.

## Status taxonomy

| Status | When |
| --- | --- |
| `kOk` | Submit accepted. Poll of an in-progress request. |
| `kQueueFull` | Submit with 8 outstanding. |
| `kUnknownId` | Poll, Cancel, or Reap of an id never accepted, or already reaped or cancelled. Malformed ids are unknown. |
| `kCancelled` | Cancel won. |
| `kShutdown` | Submit after Shutdown. Poll, Cancel, or Reap of a request that Shutdown discarded. |
| `kAlreadyComplete` | Cancel of a completed request. |
| `kDeadlineExpiredAtSubmit` | Submit with a deadline before `header.source_time` (the #118 `kDeadline` rule) or strictly before the submit clock. Not enqueued. |
| `kComplete` | Poll or Reap of a completed request. |
| `kInvalidEnvelope` | Submit of an envelope that `AssessInferenceEnvelope` does not accept for another reason, or whose header is not `STATE_VALID`. Not enqueued. Added so a bad envelope is never silently queued. |

## Submit order

First match wins:

1. Shutdown: `kShutdown`, even with capacity left.
2. `AssessInferenceEnvelope` (#118). A deadline before creation time gives
   `kDeadlineExpiredAtSubmit`. Any other defect, or a header that is not
   `STATE_VALID`, gives `kInvalidEnvelope`. A missing or malformed deadline is
   invalid, not expired.
3. Deadline strictly before the submit clock: `kDeadlineExpiredAtSubmit`. A
   deadline equal to the clock is accepted.
4. Eight outstanding: `kQueueFull`.
5. Accepted: `kOk` with a new `request_id`.

The submit clock is `InferenceClientOptions.clock` (C++) or the `clock`
argument (Python). The default is the system UTC clock. Tests pass a fixed
clock. Expiry after Submit is not consumed here.

## Races and shutdown

- Cancel against completion: all state changes happen under one lock, so
  exactly one terminal state wins. A Cancel that loses returns
  `kAlreadyComplete`. A completion that loses leaves the request gone.
- After Shutdown, ids it discarded return `kShutdown` from Poll, Cancel, and
  Reap. Ids that were never accepted, or were reaped or cancelled earlier,
  stay `kUnknownId`. At most 8 discarded ids are remembered.
- A cancelled or reaped id is `kUnknownId` afterwards.

## Stub worker

`StubWorker` completes a request on a deterministic path with no I/O, clock, or
sleep. The result echoes the envelope context, copies the OIP request id, and
stamps `output_digest = "stub-output:<request_id>"`, so it passes
`AssessInferenceResult` whenever the envelope was accepted. By default a
request stays in progress until `Tick`. Set `complete_on_submit` to complete
inside Submit instead.

## Replay worker

Contract: issue #120, comment
[`6009314342`](https://github.com/IshmaelRogers/intrinsic-core/issues/120#issuecomment-6009314342).
`ReplayWorker` (`replay_worker.{h,cc,py}`) is a deterministic worker that
returns a recorded result instead of running a model. It does no OIP or Triton
call, model load, socket, or GPU work, and it has no clock of its own. No
episodes, streams, or multi-event fixtures.

### Worker injection

`InferenceClient` takes its worker through `InferenceClientOptions.worker`
(C++, a `std::shared_ptr<const InferenceWorker>`) or the `worker` constructor
argument (Python). Null or omitted means `StubWorker`, so #119 behavior and
tests are unchanged. `InferenceWorker` is the one-method seam
(`Complete(envelope, queue_request_id, now)` / `complete(...)`); `now` is the
queue clock at the moment the worker produces the terminal result. There is no
registry.

```cpp
InferenceClientOptions options;
options.worker = std::make_shared<ReplayWorker>("intrinsic_inference/queue/fixtures");
InferenceClient client(std::move(options));
```

```python
client = inference_client.InferenceClient(
    worker=replay_worker.ReplayWorker("intrinsic_inference/queue/fixtures"))
```

### Match key

A request is a hit when it matches a fixture on all three fields, by exact
equality:

| Field | Source |
| --- | --- |
| model | `common.provenance_model_id` |
| digest | `common.input_digest` |
| epoch | `common.state_epoch` (`uint64`) |

The store is a map keyed by that triple, so a mismatch in any one field is an
absent key and is `kReplayMiss`. There is no partial or fuzzy match.
`world_snapshot_id`, OIP `model_name` / `model_version` / `request_id`,
deadline, horizon, confidence, uncertainty, metadata, and `frame_id` of the
request are not part of the key.

### Fixture layout

Directory `fixtures/`, one JSON file per recorded result, directly in the
directory (no subdirectories), at most 8 files checked in. The worker is built
with an explicit directory and reads every `*.json` file in it once, in sorted
name order. Files with other extensions are ignored. A missing directory loads
nothing, so every request is `kReplayMiss`.

```json
{
  "match": {
    "provenance_model_id": "pose_estimator",
    "input_digest": "sha256:in",
    "state_epoch": 42
  },
  "result": {
    "frame_id": "world_enu",
    "validity_state": 1,
    "source_time": {"seconds": 1700000000, "nanos": 0},
    "deadline": {"seconds": 1700000000, "nanos": 500000000},
    "validity_horizon": {"seconds": 2, "nanos": 0},
    "confidence": 0.9,
    "uncertainty": 0.05,
    "world_snapshot_id": "<64 lowercase hex>",
    "oip": {"model_name": "pose_estimator", "model_version": "3", "request_id": "rec-0001"},
    "output_digest": "sha256:replay-out-nominal"
  }
}
```

Required in `result`: `frame_id`, `validity_state` (the `StampedHeader`
validity enum value; `1` is `STATE_VALID`), `source_time`, `deadline`,
`validity_horizon`, `oip.model_name`, `oip.request_id`, `output_digest`.
Optional: `confidence`, `uncertainty`, `world_snapshot_id`,
`oip.model_version`. The result's provenance model id, input digest, and state
epoch are taken from `match`, so they cannot disagree with the key.

### Replay statuses

| Status | When |
| --- | --- |
| `kComplete` | Hit, and the recorded deadline is not before the queue clock. The recorded result is returned as written. |
| `kReplayMiss` | No fixture for the triple. Terminal, no result. |
| `kReplayExpired` | Hit, and the recorded `deadline` is strictly before the queue clock when the worker completes the request. The recorded result is still attached with `source_time`, `deadline`, `validity_horizon`, and digests unchanged. A deadline equal to the clock is not expired. |
| `kCorruptFixture` | A file has a complete `match` key but its `result` is incomplete, mistyped, fails `AssessInferenceResult`, or a second file claims the same key. Terminal, no result. |

These are terminal like `kComplete`: Poll returns them repeatedly, Cancel gives
`kAlreadyComplete`, and Reap returns them and releases the slot.

Corrupt files, by choice: a file with **no readable key** (not JSON, truncated,
or `match` incomplete) cannot be addressed, so it is skipped and its requests
are `kReplayMiss`. The worker lists such files in `skipped_files()`
(`skipped_files` in Python). Only a file that can be keyed is `kCorruptFixture`.

Recorded results are never rewritten: no clock, digest, or request id is
changed on a hit, and the stub's `stub-output:<id>` digest is not used. Queue
rejects (`kQueueFull`, `kInvalidEnvelope`, `kDeadlineExpiredAtSubmit`,
`kShutdown`) happen before the worker runs, exactly as above.

## Targets

All targets are opt-in.

| Target | Purpose |
| --- | --- |
| `inference_client`, `inference_client_py` | Queue and client facade. |
| `stub_worker`, `stub_worker_py` | Fake worker. |
| `inference_queue_test`, `inference_queue_test_py` | Paired tests: empty, one item, full, overload storm, shutdown, duplicate Poll, unknown id, Cancel, deadline at Submit, and Cancel/Tick and Shutdown races. |
| `replay_worker`, `replay_worker_py` | Replay worker (#120). The C++ library uses `nlohmann_json`. |
| `fixtures` | The checked-in recorded results. |
| `inference_replay_test`, `inference_replay_test_py` | Paired tests: hit, miss, determinism, late/expiry, corrupt fixture, and a queue smoke test with the replay worker. |

```
bazel test //intrinsic_inference/queue/...
```
