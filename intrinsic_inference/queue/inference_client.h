// Copyright 2026 Intrinsic Innovation LLC
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     https://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef INTRINSIC_INFERENCE_QUEUE_INFERENCE_CLIENT_H_
#define INTRINSIC_INFERENCE_QUEUE_INFERENCE_CLIENT_H_

#include <array>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <string_view>

#include "intrinsic/embodiment/stamped_header_policy.h"
#include "intrinsic_inference/envelope/inference_envelope_contract_policy.h"
#include "intrinsic_inference/queue/stub_worker.h"

namespace intrinsic::inference {

// Bounded async request queue with a Submit / Poll / Cancel client facade.
// Contract: issue #119, comment 5978277220. This is a host-side queue around
// a worker (the stub worker by default, or the replay worker of #120 through
// InferenceClientOptions.worker). It does not call OIP or Triton, load models,
// open sockets, or link into ICON or a HAL. Submit, Poll, Cancel, Reap, Tick
// and Shutdown never sleep and never wait on a worker.

// Fixed maximum of outstanding requests. Locked by the contract.
inline constexpr size_t kMaxOutstanding = 8;

enum class InferenceQueueStatus {
  // Submit accepted, or Poll of a request that is still in progress.
  kOk = 0,
  // Submit while kMaxOutstanding requests are outstanding. The new request is
  // rejected. Nothing already queued is dropped.
  kQueueFull,
  // Poll, Cancel or Reap of an id that was never accepted or is already
  // reaped or cancelled.
  kUnknownId,
  // Cancel won. Terminal; the slot is released.
  kCancelled,
  // Submit after Shutdown, or Poll, Cancel or Reap of a request that
  // Shutdown discarded.
  kShutdown,
  // Cancel of a request that already completed.
  kAlreadyComplete,
  // Submit of an envelope whose deadline is before its creation time or
  // before the submit clock. Not enqueued.
  kDeadlineExpiredAtSubmit,
  // Poll or Reap of a completed request.
  kComplete,
  // Submit of an envelope that AssessInferenceEnvelope does not accept for a
  // reason other than the deadline. Not enqueued.
  kInvalidEnvelope,
  // Replay worker only (#120). Poll or Reap of a request that finished with no
  // result: no fixture for the (model, digest, epoch) match key.
  kReplayMiss,
  // Replay worker only. Poll or Reap of a request whose fixture matched but
  // whose recorded deadline is strictly before the clock. The recorded result
  // shell is attached with its timing unchanged.
  kReplayExpired,
  // Replay worker only. Poll or Reap of a request whose fixture matched the
  // key but is unusable (incomplete recorded result, or duplicate key).
  kCorruptFixture,
};

struct SubmitOutcome {
  InferenceQueueStatus status = InferenceQueueStatus::kOk;
  // Set only when status is kOk.
  std::string request_id;
  // The #118 assessor error behind kDeadlineExpiredAtSubmit (when the
  // envelope itself is the cause) or kInvalidEnvelope.
  InferenceEnvelopeContractError envelope_error =
      InferenceEnvelopeContractError::kNone;
};

struct PollOutcome {
  InferenceQueueStatus status = InferenceQueueStatus::kUnknownId;
  // Set when status is kComplete or kReplayExpired.
  std::optional<InferenceResultShell> result;
};

struct CancelOutcome {
  InferenceQueueStatus status = InferenceQueueStatus::kUnknownId;
};

struct InferenceClientOptions {
  // Submit clock for the deadline check. Empty uses the system UTC clock.
  std::function<embodiment::ClockReading()> clock;
  // When true the stub worker completes each request inside Submit. When
  // false a request stays in progress until Tick.
  bool complete_on_submit = false;
  // Worker that completes requests. Null uses StubWorker. The client keeps a
  // shared reference, so the caller may drop theirs.
  std::shared_ptr<const InferenceWorker> worker;
};

// Thread safe. One mutex guards all state; no call holds it across anything
// but bounded in-memory work.
class InferenceClient {
 public:
  InferenceClient();
  explicit InferenceClient(InferenceClientOptions options);

  InferenceClient(const InferenceClient&) = delete;
  InferenceClient& operator=(const InferenceClient&) = delete;

  // First match: kShutdown, then envelope checks (kDeadlineExpiredAtSubmit,
  // kInvalidEnvelope), then kQueueFull, then kOk with a new request id.
  SubmitOutcome Submit(const InferenceEnvelopeView& request);

  // kOk while in progress. Once the worker has finished: kComplete with the
  // result, or a replay status (kReplayMiss, kReplayExpired, kCorruptFixture).
  // Poll does not release the slot, so repeated Poll is idempotent.
  PollOutcome Poll(std::string_view request_id);

  // In progress: kCancelled and the slot is released. Complete:
  // kAlreadyComplete and the slot is kept. Exactly one terminal state wins.
  CancelOutcome Cancel(std::string_view request_id);

  // Releases a completed request and returns its result with kComplete. An
  // in-progress request returns kOk and is kept. This is the explicit reap
  // that frees a slot after completion.
  PollOutcome Reap(std::string_view request_id);

  // Test and fake-async hook. Lets the worker complete up to
  // max_completions in-progress requests, oldest first. Returns the number
  // completed. Does nothing after Shutdown.
  size_t Tick(size_t max_completions = 1);

  // Idempotent. Rejects new Submit. Discards every outstanding request.
  void Shutdown();

  size_t Outstanding() const;
  bool IsShutdown() const;

 private:
  enum class SlotState { kFree, kInProgress, kComplete };

  struct Slot {
    SlotState state = SlotState::kFree;
    uint64_t id = 0;
    OwnedInferenceEnvelope envelope;
    // Terminal status once state is kComplete. Any worker status counts as
    // complete for Cancel (kAlreadyComplete) and Reap.
    InferenceQueueStatus terminal_status = InferenceQueueStatus::kComplete;
    std::optional<InferenceResultShell> result;
  };

  Slot* FindLocked(std::string_view request_id);
  bool WasDiscardedLocked(std::string_view request_id) const;
  size_t OutstandingLocked() const;
  void CompleteLocked(Slot& slot);

  InferenceClientOptions options_;
  std::shared_ptr<const InferenceWorker> worker_;
  mutable std::mutex mutex_;
  std::array<Slot, kMaxOutstanding> slots_;
  std::array<uint64_t, kMaxOutstanding> discarded_ids_{};
  size_t discarded_count_ = 0;
  uint64_t next_id_ = 1;
  bool shutdown_ = false;
};

}  // namespace intrinsic::inference

#endif  // INTRINSIC_INFERENCE_QUEUE_INFERENCE_CLIENT_H_
