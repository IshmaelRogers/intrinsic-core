# Bounded inference request queue

Opt-in bounded async request queue with a `Submit` / `Poll` / `Cancel` /
`Shutdown` client and a stub worker. Contract: issue #119, comment
[`5978277220`](https://github.com/IshmaelRogers/intrinsic-core/issues/119#issuecomment-5978277220).
Parent work package #37. Depends on the #118 envelope host policy in
[`../envelope`](../envelope/README.md).

Not here: any Triton or OIP call, model load, socket, replay store, or backend
plugin API (#120); ICON, HAL, World, or Gazebo; changes to the OIP, Triton, or
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

## Targets

All targets are opt-in.

| Target | Purpose |
| --- | --- |
| `inference_client`, `inference_client_py` | Queue and client facade. |
| `stub_worker`, `stub_worker_py` | Fake worker. |
| `inference_queue_test`, `inference_queue_test_py` | Paired tests: empty, one item, full, overload storm, shutdown, duplicate Poll, unknown id, Cancel, deadline at Submit, and Cancel/Tick and Shutdown races. |

```
bazel test //intrinsic_inference/queue/...
```
