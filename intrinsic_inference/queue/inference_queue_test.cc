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

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <set>
#include <string>
#include <thread>
#include <vector>

#include "gtest/gtest.h"
#include "intrinsic/embodiment/stamped_header_policy.h"
#include "intrinsic_inference/envelope/inference_envelope_contract_policy.h"
#include "intrinsic_inference/queue/inference_client.h"
#include "intrinsic_inference/queue/stub_worker.h"

namespace intrinsic::inference {
namespace {

using embodiment::ClockReading;
using Status = InferenceQueueStatus;

constexpr std::string_view kSnapshot =
    "0123456789abcdef0123456789abcdef0123456789abcdef0123456789abcdef";

// Nominal envelope: created at 1700000000s, deadline 1700000000.5s.
InferenceEnvelopeView NominalEnvelope() {
  InferenceCommonView common;
  common.header_present = true;
  common.validity_present = true;
  common.validity_state = 1;
  common.frame_id = "world_enu";
  common.source_time_present = true;
  common.source_time = {1700000000, 0};
  common.state_epoch = 42;
  common.world_snapshot_id = kSnapshot;
  common.deadline_present = true;
  common.deadline = {1700000000, 500000000};
  common.validity_horizon_present = true;
  common.validity_horizon = {2, 0};
  common.confidence_present = true;
  common.confidence = 0.9;
  common.input_digest = "sha256:in";
  common.provenance_present = true;
  common.provenance_model_id = "pose_estimator";
  OipIdentifierView oip;
  oip.message_set = true;
  oip.model_name = "pose_estimator";
  oip.model_version = "3";
  oip.request_id = "req-0001";
  return InferenceEnvelopeView{common, oip};
}

// Submit clock fixed at the envelope creation time so the deadline is live.
InferenceClientOptions FixedClock(ClockReading now = {1700000000, 100}) {
  InferenceClientOptions options;
  options.clock = [now] { return now; };
  return options;
}

std::string MustSubmit(InferenceClient& client) {
  const SubmitOutcome outcome = client.Submit(NominalEnvelope());
  EXPECT_EQ(outcome.status, Status::kOk);
  return outcome.request_id;
}

TEST(InferenceQueueTest, CapacityIsEight) { EXPECT_EQ(kMaxOutstanding, 8u); }

TEST(InferenceQueueTest, EmptyQueueUnknownId) {
  InferenceClient client(FixedClock());
  EXPECT_EQ(client.Outstanding(), 0u);
  EXPECT_EQ(client.Poll("inf-1").status, Status::kUnknownId);
  EXPECT_EQ(client.Cancel("inf-1").status, Status::kUnknownId);
  EXPECT_EQ(client.Reap("inf-1").status, Status::kUnknownId);
  EXPECT_EQ(client.Tick(), 0u);
  EXPECT_EQ(client.Outstanding(), 0u);
}

TEST(InferenceQueueTest, UnknownIdNeverInventsARow) {
  InferenceClient client(FixedClock());
  const std::string id = MustSubmit(client);
  for (const char* bad :
       {"", "inf-", "inf-0", "inf-01", "inf-+1", "inf- 1", "x-1", "inf-1x",
        "inf-99999999999999999999999", "INF-1", "inf-2", "inf-9"}) {
    EXPECT_EQ(client.Poll(bad).status, Status::kUnknownId) << bad;
    EXPECT_EQ(client.Cancel(bad).status, Status::kUnknownId) << bad;
    EXPECT_EQ(client.Reap(bad).status, Status::kUnknownId) << bad;
  }
  EXPECT_EQ(client.Outstanding(), 1u);
  EXPECT_EQ(client.Poll(id).status, Status::kOk);
}

TEST(InferenceQueueTest, OneItemSubmitPollTickPoll) {
  InferenceClient client(FixedClock());
  const std::string id = MustSubmit(client);
  EXPECT_EQ(id, "inf-1");
  EXPECT_EQ(client.Outstanding(), 1u);

  const PollOutcome pending = client.Poll(id);
  EXPECT_EQ(pending.status, Status::kOk);
  EXPECT_FALSE(pending.result.has_value());

  EXPECT_EQ(client.Tick(), 1u);
  const PollOutcome done = client.Poll(id);
  ASSERT_EQ(done.status, Status::kComplete);
  ASSERT_TRUE(done.result.has_value());
  const InferenceResultView result = done.result->View();
  EXPECT_EQ(result.oip_result.request_id, "req-0001");
  EXPECT_EQ(result.oip_result.model_name, "pose_estimator");
  EXPECT_EQ(result.common.frame_id, "world_enu");
  EXPECT_EQ(result.common.input_digest, "sha256:in");
  EXPECT_EQ(result.output_digest, "stub-output:inf-1");
  const InferenceEnvelopeContractAssessment assessment =
      AssessInferenceResult(result);
  EXPECT_EQ(assessment.error, InferenceEnvelopeContractError::kNone);
  EXPECT_TRUE(assessment.accepted);
  EXPECT_EQ(client.Outstanding(), 1u);
}

TEST(InferenceQueueTest, CompleteOnSubmit) {
  InferenceClientOptions options = FixedClock();
  options.complete_on_submit = true;
  InferenceClient client(options);
  const std::string id = MustSubmit(client);
  EXPECT_EQ(client.Poll(id).status, Status::kComplete);
  EXPECT_EQ(client.Tick(), 0u);
}

TEST(InferenceQueueTest, SubmitCopiesCallerStrings) {
  InferenceClient client(FixedClock());
  std::string id;
  {
    std::string frame = "world_enu";
    std::string digest = "sha256:in";
    std::string model = "pose_estimator";
    std::string request = "req-local";
    InferenceEnvelopeView envelope = NominalEnvelope();
    envelope.common.frame_id = frame;
    envelope.common.input_digest = digest;
    envelope.oip_request.model_name = model;
    envelope.oip_request.request_id = request;
    const SubmitOutcome outcome = client.Submit(envelope);
    ASSERT_EQ(outcome.status, Status::kOk);
    id = outcome.request_id;
    frame.assign("xxxxxxxxx");
    request.assign("yyyyyyyyy");
  }
  client.Tick();
  const PollOutcome done = client.Poll(id);
  ASSERT_EQ(done.status, Status::kComplete);
  EXPECT_EQ(done.result->View().common.frame_id, "world_enu");
  EXPECT_EQ(done.result->View().oip_result.request_id, "req-local");
}

TEST(InferenceQueueTest, FullQueueRejectsNinthAndKeepsTheEight) {
  InferenceClient client(FixedClock());
  std::vector<std::string> ids;
  for (size_t i = 0; i < kMaxOutstanding; ++i) {
    ids.push_back(MustSubmit(client));
  }
  EXPECT_EQ(client.Outstanding(), 8u);

  const SubmitOutcome ninth = client.Submit(NominalEnvelope());
  EXPECT_EQ(ninth.status, Status::kQueueFull);
  EXPECT_TRUE(ninth.request_id.empty());
  EXPECT_EQ(client.Outstanding(), 8u);

  // Reject new: neither the oldest nor the newest accepted request is gone.
  for (const std::string& id : ids) {
    EXPECT_EQ(client.Poll(id).status, Status::kOk) << id;
  }
  EXPECT_EQ(client.Tick(8), 8u);
  for (const std::string& id : ids) {
    EXPECT_EQ(client.Poll(id).status, Status::kComplete) << id;
  }
  // Completed-but-not-reaped still occupies a slot.
  EXPECT_EQ(client.Submit(NominalEnvelope()).status, Status::kQueueFull);
}

TEST(InferenceQueueTest, CancelAndReapFreeSlots) {
  InferenceClient client(FixedClock());
  std::vector<std::string> ids;
  for (size_t i = 0; i < kMaxOutstanding; ++i) {
    ids.push_back(MustSubmit(client));
  }
  EXPECT_EQ(client.Cancel(ids[3]).status, Status::kCancelled);
  EXPECT_EQ(client.Outstanding(), 7u);
  const std::string replacement = MustSubmit(client);
  EXPECT_EQ(replacement, "inf-9");
  EXPECT_EQ(client.Submit(NominalEnvelope()).status, Status::kQueueFull);

  client.Tick();
  EXPECT_EQ(client.Poll(ids[0]).status, Status::kComplete);
  const PollOutcome reaped = client.Reap(ids[0]);
  EXPECT_EQ(reaped.status, Status::kComplete);
  EXPECT_TRUE(reaped.result.has_value());
  EXPECT_EQ(client.Poll(ids[0]).status, Status::kUnknownId);
  EXPECT_EQ(client.Reap(ids[0]).status, Status::kUnknownId);
  EXPECT_EQ(client.Outstanding(), 7u);
  EXPECT_EQ(MustSubmit(client), "inf-10");
}

TEST(InferenceQueueTest, ReapOfInProgressKeepsTheSlot) {
  InferenceClient client(FixedClock());
  const std::string id = MustSubmit(client);
  const PollOutcome outcome = client.Reap(id);
  EXPECT_EQ(outcome.status, Status::kOk);
  EXPECT_FALSE(outcome.result.has_value());
  EXPECT_EQ(client.Outstanding(), 1u);
  EXPECT_EQ(client.Poll(id).status, Status::kOk);
}

TEST(InferenceQueueTest, OverloadStormRejectsNewAndStaysBounded) {
  InferenceClient client(FixedClock());
  size_t accepted = 0;
  size_t rejected = 0;
  for (int i = 0; i < 5000; ++i) {
    const SubmitOutcome outcome = client.Submit(NominalEnvelope());
    if (outcome.status == Status::kOk) {
      ++accepted;
    } else {
      EXPECT_EQ(outcome.status, Status::kQueueFull);
      ++rejected;
    }
    EXPECT_LE(client.Outstanding(), kMaxOutstanding);
  }
  EXPECT_EQ(accepted, kMaxOutstanding);
  EXPECT_EQ(rejected, 5000u - kMaxOutstanding);
  // The earliest eight stay; nothing newer displaced them.
  for (int i = 1; i <= 8; ++i) {
    EXPECT_EQ(client.Poll("inf-" + std::to_string(i)).status, Status::kOk);
  }
  EXPECT_EQ(client.Poll("inf-9").status, Status::kUnknownId);
}

TEST(InferenceQueueTest, ConcurrentOverloadNeverExceedsCapacity) {
  InferenceClient client(FixedClock());
  constexpr int kThreads = 8;
  constexpr int kPerThread = 200;
  std::atomic<int> accepted{0};
  std::atomic<int> full{0};
  std::vector<std::vector<std::string>> ids(kThreads);
  std::vector<std::thread> threads;
  for (int t = 0; t < kThreads; ++t) {
    threads.emplace_back([&, t] {
      for (int i = 0; i < kPerThread; ++i) {
        const SubmitOutcome outcome = client.Submit(NominalEnvelope());
        if (outcome.status == Status::kOk) {
          ++accepted;
          ids[t].push_back(outcome.request_id);
        } else if (outcome.status == Status::kQueueFull) {
          ++full;
        }
        EXPECT_LE(client.Outstanding(), kMaxOutstanding);
      }
    });
  }
  for (std::thread& thread : threads) {
    thread.join();
  }
  EXPECT_EQ(accepted.load(), static_cast<int>(kMaxOutstanding));
  EXPECT_EQ(full.load(),
            kThreads * kPerThread - static_cast<int>(kMaxOutstanding));
  std::set<std::string> unique;
  for (const auto& per_thread : ids) {
    unique.insert(per_thread.begin(), per_thread.end());
  }
  EXPECT_EQ(unique.size(), kMaxOutstanding);
  EXPECT_EQ(client.Outstanding(), kMaxOutstanding);
}

TEST(InferenceQueueTest, ShutdownRejectsSubmitEvenWithCapacity) {
  InferenceClient client(FixedClock());
  EXPECT_FALSE(client.IsShutdown());
  client.Shutdown();
  EXPECT_TRUE(client.IsShutdown());
  EXPECT_EQ(client.Outstanding(), 0u);
  const SubmitOutcome outcome = client.Submit(NominalEnvelope());
  EXPECT_EQ(outcome.status, Status::kShutdown);
  EXPECT_TRUE(outcome.request_id.empty());
  EXPECT_EQ(client.Outstanding(), 0u);
}

TEST(InferenceQueueTest, ShutdownIsIdempotentAndDiscardsOutstanding) {
  InferenceClient client(FixedClock());
  std::vector<std::string> ids;
  for (size_t i = 0; i < kMaxOutstanding; ++i) {
    ids.push_back(MustSubmit(client));
  }
  client.Tick(3);  // Three complete, five in progress; all are discarded.
  client.Shutdown();
  client.Shutdown();
  EXPECT_EQ(client.Outstanding(), 0u);
  for (const std::string& id : ids) {
    EXPECT_EQ(client.Poll(id).status, Status::kShutdown) << id;
    EXPECT_EQ(client.Cancel(id).status, Status::kShutdown) << id;
    EXPECT_EQ(client.Reap(id).status, Status::kShutdown) << id;
  }
  EXPECT_EQ(client.Poll("inf-99").status, Status::kUnknownId);
  EXPECT_EQ(client.Cancel("inf-99").status, Status::kUnknownId);
  EXPECT_EQ(client.Tick(8), 0u);
  EXPECT_EQ(client.Submit(NominalEnvelope()).status, Status::kShutdown);
}

TEST(InferenceQueueTest, ShutdownDoesNotRelabelEarlierCancelOrReap) {
  InferenceClient client(FixedClock());
  const std::string cancelled = MustSubmit(client);
  const std::string reaped = MustSubmit(client);
  const std::string kept = MustSubmit(client);
  EXPECT_EQ(client.Cancel(cancelled).status, Status::kCancelled);
  client.Tick(2);
  EXPECT_EQ(client.Reap(reaped).status, Status::kComplete);
  client.Shutdown();
  EXPECT_EQ(client.Poll(cancelled).status, Status::kUnknownId);
  EXPECT_EQ(client.Poll(reaped).status, Status::kUnknownId);
  EXPECT_EQ(client.Poll(kept).status, Status::kShutdown);
}

TEST(InferenceQueueTest, DuplicatePollIsIdempotent) {
  InferenceClient client(FixedClock());
  const std::string id = MustSubmit(client);
  for (int i = 0; i < 5; ++i) {
    EXPECT_EQ(client.Poll(id).status, Status::kOk);
  }
  client.Tick();
  const PollOutcome first = client.Poll(id);
  ASSERT_EQ(first.status, Status::kComplete);
  for (int i = 0; i < 5; ++i) {
    const PollOutcome again = client.Poll(id);
    ASSERT_EQ(again.status, Status::kComplete);
    EXPECT_EQ(again.result->View().output_digest,
              first.result->View().output_digest);
  }
  EXPECT_EQ(client.Outstanding(), 1u);
}

TEST(InferenceQueueTest, CancelInProgress) {
  InferenceClient client(FixedClock());
  const std::string id = MustSubmit(client);
  EXPECT_EQ(client.Cancel(id).status, Status::kCancelled);
  EXPECT_EQ(client.Outstanding(), 0u);
  EXPECT_EQ(client.Poll(id).status, Status::kUnknownId);
  EXPECT_EQ(client.Cancel(id).status, Status::kUnknownId);
  EXPECT_EQ(client.Tick(), 0u);
  EXPECT_EQ(client.Poll(id).status, Status::kUnknownId);
}

TEST(InferenceQueueTest, CancelAfterCompleteIsAlreadyComplete) {
  InferenceClient client(FixedClock());
  const std::string id = MustSubmit(client);
  client.Tick();
  EXPECT_EQ(client.Cancel(id).status, Status::kAlreadyComplete);
  EXPECT_EQ(client.Cancel(id).status, Status::kAlreadyComplete);
  EXPECT_EQ(client.Poll(id).status, Status::kComplete);
  EXPECT_EQ(client.Outstanding(), 1u);
}

TEST(InferenceQueueTest, TickCompletesOldestFirst) {
  InferenceClient client(FixedClock());
  const std::string a = MustSubmit(client);
  const std::string b = MustSubmit(client);
  const std::string c = MustSubmit(client);
  EXPECT_EQ(client.Cancel(a).status, Status::kCancelled);
  const std::string d = MustSubmit(client);  // Reuses a's slot, newest id.
  EXPECT_EQ(client.Tick(1), 1u);
  EXPECT_EQ(client.Poll(b).status, Status::kComplete);
  EXPECT_EQ(client.Poll(c).status, Status::kOk);
  EXPECT_EQ(client.Poll(d).status, Status::kOk);
  EXPECT_EQ(client.Tick(10), 2u);
  EXPECT_EQ(client.Tick(10), 0u);
}

TEST(InferenceQueueTest, DeadlineBeforeCreationRejectedAtSubmit) {
  InferenceClient client(FixedClock());
  InferenceEnvelopeView envelope = NominalEnvelope();
  envelope.common.deadline = {1699999999, 999999999};
  // Same rule as the #118 assessor.
  ASSERT_EQ(AssessInferenceEnvelope(envelope).error,
            InferenceEnvelopeContractError::kDeadline);
  const SubmitOutcome outcome = client.Submit(envelope);
  EXPECT_EQ(outcome.status, Status::kDeadlineExpiredAtSubmit);
  EXPECT_EQ(outcome.envelope_error, InferenceEnvelopeContractError::kDeadline);
  EXPECT_TRUE(outcome.request_id.empty());
  EXPECT_EQ(client.Outstanding(), 0u);
}

TEST(InferenceQueueTest, DeadlineBeforeSubmitClockRejectedAtSubmit) {
  InferenceClient client(FixedClock({1700000000, 500000001}));
  const SubmitOutcome outcome = client.Submit(NominalEnvelope());
  EXPECT_EQ(outcome.status, Status::kDeadlineExpiredAtSubmit);
  EXPECT_EQ(outcome.envelope_error, InferenceEnvelopeContractError::kNone);
  EXPECT_EQ(client.Outstanding(), 0u);
}

TEST(InferenceQueueTest, DeadlineEqualToSubmitClockIsAccepted) {
  InferenceClient client(FixedClock({1700000000, 500000000}));
  EXPECT_EQ(client.Submit(NominalEnvelope()).status, Status::kOk);
  InferenceEnvelopeView equal_creation = NominalEnvelope();
  equal_creation.common.deadline = equal_creation.common.source_time;
  InferenceClient at_creation(FixedClock({1700000000, 0}));
  EXPECT_EQ(at_creation.Submit(equal_creation).status, Status::kOk);
}

TEST(InferenceQueueTest, ExpiredDeadlineIsRejectedEvenWhenFull) {
  InferenceClient client(FixedClock());
  for (size_t i = 0; i < kMaxOutstanding; ++i) {
    MustSubmit(client);
  }
  InferenceEnvelopeView envelope = NominalEnvelope();
  envelope.common.deadline = {1600000000, 0};
  EXPECT_EQ(client.Submit(envelope).status, Status::kDeadlineExpiredAtSubmit);
  EXPECT_EQ(client.Outstanding(), 8u);
}

TEST(InferenceQueueTest, ShutdownWinsOverExpiredDeadline) {
  InferenceClient client(FixedClock());
  client.Shutdown();
  InferenceEnvelopeView envelope = NominalEnvelope();
  envelope.common.deadline = {1600000000, 0};
  EXPECT_EQ(client.Submit(envelope).status, Status::kShutdown);
}

TEST(InferenceQueueTest, MissingDeadlineIsInvalidNotExpired) {
  InferenceClient client(FixedClock());
  InferenceEnvelopeView envelope = NominalEnvelope();
  envelope.common.deadline_present = false;
  const SubmitOutcome outcome = client.Submit(envelope);
  EXPECT_EQ(outcome.status, Status::kInvalidEnvelope);
  EXPECT_EQ(outcome.envelope_error, InferenceEnvelopeContractError::kDeadline);
}

TEST(InferenceQueueTest, UnacceptedEnvelopesAreNotEnqueued) {
  InferenceClient client(FixedClock());
  EXPECT_EQ(client.Submit(InferenceEnvelopeView{}).status,
            Status::kInvalidEnvelope);

  InferenceEnvelopeView no_frame = NominalEnvelope();
  no_frame.common.frame_id = {};
  const SubmitOutcome outcome = client.Submit(no_frame);
  EXPECT_EQ(outcome.status, Status::kInvalidEnvelope);
  EXPECT_EQ(outcome.envelope_error,
            InferenceEnvelopeContractError::kMissingFrame);

  InferenceEnvelopeView invalid_header = NominalEnvelope();
  invalid_header.common.validity_state = 2;
  EXPECT_EQ(client.Submit(invalid_header).status, Status::kInvalidEnvelope);
  EXPECT_EQ(client.Outstanding(), 0u);
}

TEST(InferenceQueueTest, CancelRacesTickExactlyOneTerminalWins) {
  for (int round = 0; round < 300; ++round) {
    InferenceClient client(FixedClock());
    const std::string id = MustSubmit(client);
    CancelOutcome cancel;
    size_t completed = 0;
    std::thread canceller([&] { cancel = client.Cancel(id); });
    std::thread worker([&] { completed = client.Tick(); });
    canceller.join();
    worker.join();
    if (cancel.status == Status::kCancelled) {
      EXPECT_EQ(completed, 0u);
      EXPECT_EQ(client.Poll(id).status, Status::kUnknownId);
      EXPECT_EQ(client.Outstanding(), 0u);
    } else {
      EXPECT_EQ(cancel.status, Status::kAlreadyComplete);
      EXPECT_EQ(completed, 1u);
      EXPECT_EQ(client.Poll(id).status, Status::kComplete);
      EXPECT_EQ(client.Outstanding(), 1u);
    }
  }
}

TEST(InferenceQueueTest, ShutdownRacesSubmitPollCancel) {
  for (int round = 0; round < 100; ++round) {
    InferenceClient client(FixedClock());
    std::vector<std::string> ids;
    for (int i = 0; i < 4; ++i) {
      ids.push_back(MustSubmit(client));
    }
    std::atomic<bool> go{false};
    std::vector<SubmitOutcome> submits(8);
    std::vector<Status> polls(ids.size());
    std::vector<Status> cancels(ids.size());
    std::thread submitter([&] {
      while (!go) {
      }
      for (SubmitOutcome& outcome : submits) {
        outcome = client.Submit(NominalEnvelope());
      }
    });
    std::thread poller([&] {
      while (!go) {
      }
      for (size_t i = 0; i < ids.size(); ++i) {
        polls[i] = client.Poll(ids[i]).status;
      }
    });
    std::thread canceller([&] {
      while (!go) {
      }
      for (size_t i = 0; i < ids.size(); ++i) {
        cancels[i] = client.Cancel(ids[i]).status;
      }
    });
    std::thread shutter([&] {
      while (!go) {
      }
      client.Shutdown();
    });
    go = true;
    submitter.join();
    poller.join();
    canceller.join();
    shutter.join();

    for (const SubmitOutcome& outcome : submits) {
      EXPECT_TRUE(outcome.status == Status::kOk ||
                  outcome.status == Status::kShutdown ||
                  outcome.status == Status::kQueueFull);
    }
    for (const Status status : polls) {
      EXPECT_TRUE(status == Status::kOk || status == Status::kShutdown ||
                  status == Status::kUnknownId);
    }
    for (const Status status : cancels) {
      EXPECT_TRUE(status == Status::kCancelled || status == Status::kShutdown ||
                  status == Status::kUnknownId);
    }
    EXPECT_TRUE(client.IsShutdown());
    EXPECT_EQ(client.Outstanding(), 0u);
    EXPECT_EQ(client.Submit(NominalEnvelope()).status, Status::kShutdown);
    EXPECT_EQ(client.Tick(8), 0u);
  }
}

TEST(StubWorkerTest, ResultPassesAssessInferenceResult) {
  const OwnedInferenceEnvelope envelope(NominalEnvelope());
  const InferenceResultShell result = StubWorker().Run(envelope, "inf-7");
  const InferenceResultView view = result.View();
  EXPECT_EQ(view.output_digest, "stub-output:inf-7");
  EXPECT_TRUE(AssessInferenceResult(view).accepted);
}

TEST(StubWorkerTest, ShellsAreCopyable) {
  const OwnedInferenceEnvelope envelope(NominalEnvelope());
  InferenceResultShell copy;
  {
    const InferenceResultShell original = StubWorker().Run(envelope, "inf-7");
    copy = original;
  }
  EXPECT_EQ(copy.View().oip_result.request_id, "req-0001");
  EXPECT_TRUE(AssessInferenceResult(copy.View()).accepted);
}

}  // namespace
}  // namespace intrinsic::inference
