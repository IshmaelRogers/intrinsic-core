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

#include <unistd.h>

#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <memory>
#include <string>
#include <string_view>
#include <vector>

#include "gtest/gtest.h"
#include "intrinsic/embodiment/stamped_header_policy.h"
#include "intrinsic_inference/envelope/inference_envelope_contract_policy.h"
#include "intrinsic_inference/queue/inference_client.h"
#include "intrinsic_inference/queue/replay_worker.h"
#include "intrinsic_inference/queue/stub_worker.h"

namespace intrinsic::inference {
namespace {

namespace fs = std::filesystem;
using embodiment::ClockReading;
using Status = InferenceQueueStatus;

constexpr std::string_view kSnapshot =
    "0123456789abcdef0123456789abcdef0123456789abcdef0123456789abcdef";

// Checked-in fixtures: Bazel runfiles when available, else the repo-relative
// path (tests run from the repository root).
std::string FixtureDir() {
  const char* srcdir = std::getenv("TEST_SRCDIR");
  const char* workspace = std::getenv("TEST_WORKSPACE");
  if (srcdir != nullptr && workspace != nullptr) {
    return std::string(srcdir) + "/" + workspace +
           "/intrinsic_inference/queue/fixtures";
  }
  return "intrinsic_inference/queue/fixtures";
}

// Envelope for a (model, digest, epoch) match key. Created at 1700000000s,
// deadline 1700000000.5s. The string views must outlive Submit.
InferenceEnvelopeView Envelope(std::string_view model = "pose_estimator",
                               std::string_view digest = "sha256:in",
                               uint64_t epoch = 42) {
  InferenceCommonView common;
  common.header_present = true;
  common.validity_present = true;
  common.validity_state = 1;
  common.frame_id = "world_enu";
  common.source_time_present = true;
  common.source_time = {1700000000, 0};
  common.state_epoch = epoch;
  common.world_snapshot_id = kSnapshot;
  common.deadline_present = true;
  common.deadline = {1700000000, 500000000};
  common.validity_horizon_present = true;
  common.validity_horizon = {2, 0};
  common.confidence_present = true;
  common.confidence = 0.9;
  common.input_digest = digest;
  common.provenance_present = true;
  common.provenance_model_id = model;
  OipIdentifierView oip;
  oip.message_set = true;
  oip.model_name = "pose_estimator";
  oip.model_version = "3";
  oip.request_id = "req-0001";
  return InferenceEnvelopeView{common, oip};
}

std::shared_ptr<const ReplayWorker> LoadFixtures() {
  return std::make_shared<const ReplayWorker>(FixtureDir());
}

InferenceClientOptions ReplayOptions(
    std::shared_ptr<const InferenceWorker> worker,
    ClockReading now = {1700000000, 100}) {
  InferenceClientOptions options;
  options.clock = [now] { return now; };
  options.worker = std::move(worker);
  return options;
}

// Submit, Tick, Poll one request to its terminal outcome.
PollOutcome SubmitTickPoll(InferenceClient& client,
                           const InferenceEnvelopeView& request) {
  const SubmitOutcome submitted = client.Submit(request);
  EXPECT_EQ(submitted.status, Status::kOk);
  EXPECT_EQ(client.Tick(), 1u);
  return client.Poll(submitted.request_id);
}

// Scratch fixture directory, removed on destruction.
class TempFixtureDir {
 public:
  TempFixtureDir() {
    const char* base = std::getenv("TEST_TMPDIR");
    static int counter = 0;
    path_ =
        fs::path(base != nullptr ? base : fs::temp_directory_path().string()) /
        ("replay_fixtures_" + std::to_string(::getpid()) + "_" +
         std::to_string(counter++));
    fs::create_directories(path_);
  }
  ~TempFixtureDir() {
    std::error_code ec;
    fs::remove_all(path_, ec);
  }
  void Write(const std::string& name, const std::string& text) const {
    std::ofstream(path_ / name, std::ios::binary) << text;
  }
  void Copy(const std::string& name) const {
    fs::copy_file(fs::path(FixtureDir()) / name, path_ / name);
  }
  std::string path() const { return path_.string(); }

 private:
  fs::path path_;
};

// 1. Nominal hit ------------------------------------------------------------

TEST(ReplayWorkerTest, FixtureDirectoryHasAtMostEightFilesAndLoads) {
  size_t files = 0;
  for (const auto& entry : fs::directory_iterator(FixtureDir())) {
    if (entry.is_regular_file()) {
      ++files;
    }
  }
  EXPECT_LE(files, 8u);
  EXPECT_EQ(LoadFixtures()->fixture_count(), 5u);
  // Only the truncated file has no readable key.
  EXPECT_EQ(LoadFixtures()->skipped_files(),
            std::vector<std::string>{"corrupt_unparseable.json"});
}

TEST(ReplayWorkerTest, NominalHitReturnsRecordedResult) {
  InferenceClient client(ReplayOptions(LoadFixtures()));
  const SubmitOutcome submitted = client.Submit(Envelope());
  ASSERT_EQ(submitted.status, Status::kOk);
  const PollOutcome pending = client.Poll(submitted.request_id);
  EXPECT_EQ(pending.status, Status::kOk);
  EXPECT_FALSE(pending.result.has_value());

  ASSERT_EQ(client.Tick(), 1u);
  const PollOutcome done = client.Poll(submitted.request_id);
  ASSERT_EQ(done.status, Status::kComplete);
  ASSERT_TRUE(done.result.has_value());
  const InferenceResultView result = done.result->View();

  // Match key and recorded digests / provenance.
  EXPECT_EQ(result.common.provenance_model_id, "pose_estimator");
  EXPECT_EQ(result.common.input_digest, "sha256:in");
  EXPECT_EQ(result.common.state_epoch, 42u);
  EXPECT_EQ(result.output_digest, "sha256:replay-out-nominal");
  // Recorded, not the envelope's, and not rewritten by the queue.
  EXPECT_EQ(result.oip_result.request_id, "rec-0001");
  EXPECT_EQ(result.oip_result.model_version, "3");
  EXPECT_EQ(result.common.frame_id, "world_enu");
  EXPECT_EQ(result.common.world_snapshot_id, kSnapshot);
  // Recorded timing.
  EXPECT_EQ(result.common.source_time.seconds, 1700000000);
  EXPECT_EQ(result.common.source_time.nanos, 0);
  EXPECT_EQ(result.common.deadline.seconds, 1700000000);
  EXPECT_EQ(result.common.deadline.nanos, 500000000);
  EXPECT_EQ(result.common.validity_horizon.seconds, 2);
  EXPECT_EQ(result.common.validity_horizon.nanos, 0);
  EXPECT_DOUBLE_EQ(result.common.confidence, 0.9);
  EXPECT_DOUBLE_EQ(result.common.uncertainty, 0.05);

  const InferenceEnvelopeContractAssessment assessment =
      AssessInferenceResult(result);
  EXPECT_EQ(assessment.error, InferenceEnvelopeContractError::kNone);
  EXPECT_TRUE(assessment.accepted);
}

TEST(ReplayWorkerTest, CompleteOnSubmitHit) {
  InferenceClientOptions options = ReplayOptions(LoadFixtures());
  options.complete_on_submit = true;
  InferenceClient client(std::move(options));
  const SubmitOutcome submitted = client.Submit(Envelope());
  ASSERT_EQ(submitted.status, Status::kOk);
  const PollOutcome done = client.Poll(submitted.request_id);
  ASSERT_EQ(done.status, Status::kComplete);
  EXPECT_EQ(done.result->View().output_digest, "sha256:replay-out-nominal");
}

TEST(ReplayWorkerTest, EachKeyFieldSelectsItsOwnRow) {
  InferenceClient client(ReplayOptions(LoadFixtures()));
  const PollOutcome epoch43 =
      SubmitTickPoll(client, Envelope("pose_estimator", "sha256:in", 43));
  ASSERT_EQ(epoch43.status, Status::kComplete);
  EXPECT_EQ(epoch43.result->View().output_digest, "sha256:replay-out-epoch-43");
  EXPECT_EQ(epoch43.result->View().common.state_epoch, 43u);

  const PollOutcome grasp =
      SubmitTickPoll(client, Envelope("grasp_planner", "sha256:in", 42));
  ASSERT_EQ(grasp.status, Status::kComplete);
  EXPECT_EQ(grasp.result->View().output_digest, "sha256:replay-out-grasp");
  EXPECT_EQ(grasp.result->View().common.provenance_model_id, "grasp_planner");
}

TEST(ReplayWorkerTest, WorldSnapshotAndOipIdsAreNotPartOfTheKey) {
  InferenceClient client(ReplayOptions(LoadFixtures()));
  InferenceEnvelopeView request = Envelope();
  request.common.world_snapshot_id =
      "fedcba9876543210fedcba9876543210fedcba9876543210fedcba9876543210";
  request.oip_request.model_name = "something_else";
  request.oip_request.request_id = "req-9999";
  request.common.confidence = 0.1;
  const PollOutcome done = SubmitTickPoll(client, request);
  ASSERT_EQ(done.status, Status::kComplete);
  EXPECT_EQ(done.result->View().output_digest, "sha256:replay-out-nominal");
}

// 2. Miss -------------------------------------------------------------------

TEST(ReplayWorkerTest, UnknownKeyIsReplayMissWithNoResult) {
  InferenceClient client(ReplayOptions(LoadFixtures()));
  struct Case {
    std::string_view model;
    std::string_view digest;
    uint64_t epoch;
  };
  for (const Case& c : {
           Case{"unknown_model", "sha256:in", 42},
           Case{"pose_estimator", "sha256:unknown", 42},
           Case{"pose_estimator", "sha256:in", 99},
           // Key whose only fixture file is truncated, so it cannot be keyed.
           Case{"pose_estimator", "sha256:in-trunc", 42},
       }) {
    const PollOutcome done =
        SubmitTickPoll(client, Envelope(c.model, c.digest, c.epoch));
    EXPECT_EQ(done.status, Status::kReplayMiss) << c.model << " " << c.digest;
    EXPECT_FALSE(done.result.has_value());
  }
}

TEST(ReplayWorkerTest, MissingOrEmptyDirectoryMissesEverything) {
  TempFixtureDir empty;
  for (const std::string& dir :
       {empty.path(), empty.path() + "/does_not_exist"}) {
    const auto worker = std::make_shared<const ReplayWorker>(dir);
    EXPECT_EQ(worker->fixture_count(), 0u);
    InferenceClient client(ReplayOptions(worker));
    EXPECT_EQ(SubmitTickPoll(client, Envelope()).status, Status::kReplayMiss);
  }
}

// 3. Determinism --------------------------------------------------------------

TEST(ReplayWorkerTest, RepeatedHitIsIdentical) {
  const auto worker = LoadFixtures();
  InferenceClient client(ReplayOptions(worker));
  const PollOutcome first = SubmitTickPoll(client, Envelope());
  const PollOutcome second = SubmitTickPoll(client, Envelope());
  // A fresh worker over the same directory gives the same shell too.
  InferenceClient other(ReplayOptions(LoadFixtures()));
  const PollOutcome third = SubmitTickPoll(other, Envelope());

  ASSERT_EQ(first.status, Status::kComplete);
  ASSERT_EQ(second.status, Status::kComplete);
  ASSERT_EQ(third.status, Status::kComplete);
  const InferenceResultView a = first.result->View();
  for (const PollOutcome* outcome : {&second, &third}) {
    const InferenceResultView b = outcome->result->View();
    EXPECT_EQ(a.output_digest, b.output_digest);
    EXPECT_EQ(a.common.input_digest, b.common.input_digest);
    EXPECT_EQ(a.common.provenance_model_id, b.common.provenance_model_id);
    EXPECT_EQ(a.common.state_epoch, b.common.state_epoch);
    EXPECT_EQ(a.oip_result.request_id, b.oip_result.request_id);
    EXPECT_EQ(a.common.source_time.seconds, b.common.source_time.seconds);
    EXPECT_EQ(a.common.source_time.nanos, b.common.source_time.nanos);
    EXPECT_EQ(a.common.deadline.seconds, b.common.deadline.seconds);
    EXPECT_EQ(a.common.deadline.nanos, b.common.deadline.nanos);
    EXPECT_EQ(a.common.validity_horizon.seconds,
              b.common.validity_horizon.seconds);
    EXPECT_EQ(a.common.validity_horizon.nanos, b.common.validity_horizon.nanos);
  }
}

// 4. Late / expiry ----------------------------------------------------------

TEST(ReplayWorkerTest, RecordedDeadlineBeforeClockIsExpiredWithTimingKept) {
  // The request deadline (1700000000.5s) is live at the submit clock; only the
  // recorded deadline (1699999999.9s) is in the past.
  InferenceClient client(ReplayOptions(LoadFixtures()));
  const PollOutcome done =
      SubmitTickPoll(client, Envelope("pose_estimator", "sha256:in-late", 42));
  ASSERT_EQ(done.status, Status::kReplayExpired);
  ASSERT_TRUE(done.result.has_value());
  const InferenceResultView result = done.result->View();
  EXPECT_EQ(result.common.source_time.seconds, 1699999999);
  EXPECT_EQ(result.common.source_time.nanos, 0);
  EXPECT_EQ(result.common.deadline.seconds, 1699999999);
  EXPECT_EQ(result.common.deadline.nanos, 900000000);
  EXPECT_EQ(result.common.validity_horizon.seconds, 2);
  EXPECT_EQ(result.common.validity_horizon.nanos, 0);
  EXPECT_EQ(result.output_digest, "sha256:replay-out-late");
  EXPECT_EQ(result.oip_result.request_id, "rec-0004");
}

TEST(ReplayWorkerTest, ResultIsLateWhenClockPassesDeadlineBeforeTick) {
  ClockReading now{1700000000, 100};
  InferenceClientOptions options = ReplayOptions(LoadFixtures());
  options.clock = [&now] { return now; };
  InferenceClient client(std::move(options));
  const SubmitOutcome submitted = client.Submit(Envelope());
  ASSERT_EQ(submitted.status, Status::kOk);

  now = {1700000000, 600000000};  // Past the recorded deadline of .5s.
  ASSERT_EQ(client.Tick(), 1u);
  const PollOutcome done = client.Poll(submitted.request_id);
  ASSERT_EQ(done.status, Status::kReplayExpired);
  const InferenceResultView result = done.result->View();
  EXPECT_EQ(result.common.deadline.seconds, 1700000000);
  EXPECT_EQ(result.common.deadline.nanos, 500000000);
  EXPECT_EQ(result.output_digest, "sha256:replay-out-nominal");
}

TEST(ReplayWorkerTest, DeadlineEqualToClockIsNotExpired) {
  ClockReading now{1700000000, 500000000};
  InferenceClient client(ReplayOptions(LoadFixtures(), now));
  EXPECT_EQ(SubmitTickPoll(client, Envelope()).status, Status::kComplete);

  now = {1700000000, 500000001};
  InferenceClientOptions options = ReplayOptions(LoadFixtures(), now);
  InferenceClient late(std::move(options));
  // The request itself is now expired at Submit, before the worker runs.
  EXPECT_EQ(late.Submit(Envelope()).status, Status::kDeadlineExpiredAtSubmit);
}

TEST(ReplayWorkerTest, ExpiredResultIsStillQueueTerminal) {
  InferenceClient client(ReplayOptions(LoadFixtures()));
  const SubmitOutcome submitted =
      client.Submit(Envelope("pose_estimator", "sha256:in-late", 42));
  ASSERT_EQ(client.Tick(), 1u);
  EXPECT_EQ(client.Cancel(submitted.request_id).status,
            Status::kAlreadyComplete);
  const PollOutcome reaped = client.Reap(submitted.request_id);
  EXPECT_EQ(reaped.status, Status::kReplayExpired);
  EXPECT_TRUE(reaped.result.has_value());
  EXPECT_EQ(client.Outstanding(), 0u);
  EXPECT_EQ(client.Poll(submitted.request_id).status, Status::kUnknownId);
}

// 5. Corrupt fixture --------------------------------------------------------

// Contract choice, documented in the README: a file with a complete match key
// and an unusable result is kCorruptFixture. A file with no readable key
// (unparseable, or incomplete key) cannot be addressed and is skipped, so its
// requests are kReplayMiss.
TEST(ReplayWorkerTest, IncompleteRecordedResultIsCorruptFixture) {
  InferenceClient client(ReplayOptions(LoadFixtures()));
  const PollOutcome done = SubmitTickPoll(
      client, Envelope("pose_estimator", "sha256:in-corrupt", 42));
  EXPECT_EQ(done.status, Status::kCorruptFixture);
  EXPECT_FALSE(done.result.has_value());
}

TEST(ReplayWorkerTest, CorruptFixtureDoesNotAffectOtherRows) {
  InferenceClient client(ReplayOptions(LoadFixtures()));
  EXPECT_EQ(SubmitTickPoll(client, Envelope()).status, Status::kComplete);
}

TEST(ReplayWorkerTest, ResultThatFailsAssessmentIsCorruptFixture) {
  TempFixtureDir dir;
  // Deadline before source_time: AssessInferenceResult reports kDeadline.
  dir.Write("bad_deadline.json", R"({
    "match": {"provenance_model_id": "pose_estimator",
              "input_digest": "sha256:in", "state_epoch": 42},
    "result": {"frame_id": "world_enu", "validity_state": 1,
               "source_time": {"seconds": 10, "nanos": 0},
               "deadline": {"seconds": 5, "nanos": 0},
               "validity_horizon": {"seconds": 2, "nanos": 0},
               "oip": {"model_name": "m", "request_id": "r"},
               "output_digest": "sha256:o"}})");
  InferenceClient client(
      ReplayOptions(std::make_shared<const ReplayWorker>(dir.path())));
  EXPECT_EQ(SubmitTickPoll(client, Envelope()).status, Status::kCorruptFixture);
}

TEST(ReplayWorkerTest, DuplicateKeyIsCorruptFixture) {
  TempFixtureDir dir;
  dir.Copy("hit_nominal.json");
  dir.Copy("hit_other_epoch.json");
  // Same key as hit_nominal.json under another name.
  fs::copy_file(fs::path(FixtureDir()) / "hit_nominal.json",
                fs::path(dir.path()) / "zz_copy.json");
  const auto worker = std::make_shared<const ReplayWorker>(dir.path());
  EXPECT_EQ(worker->fixture_count(), 2u);
  InferenceClient client(ReplayOptions(worker));
  EXPECT_EQ(SubmitTickPoll(client, Envelope()).status, Status::kCorruptFixture);
  EXPECT_EQ(SubmitTickPoll(client, Envelope("pose_estimator", "sha256:in", 43))
                .status,
            Status::kComplete);
}

TEST(ReplayWorkerTest, FilesWithoutAKeyAreSkippedAndOtherExtensionsIgnored) {
  TempFixtureDir dir;
  dir.Write("not_json.txt", "{}");
  dir.Write("empty.json", "");
  dir.Write("array.json", "[]");
  dir.Write("no_key.json", R"({"match": {"input_digest": "sha256:in"}})");
  dir.Write("bad_epoch.json",
            R"({"match": {"provenance_model_id": "m", "input_digest": "d",
                          "state_epoch": -1}})");
  const ReplayWorker worker(dir.path());
  EXPECT_EQ(worker.fixture_count(), 0u);
  EXPECT_EQ(worker.skipped_files(),
            (std::vector<std::string>{"array.json", "bad_epoch.json",
                                      "empty.json", "no_key.json"}));
}

// 6. Queue unchanged with ReplayWorker --------------------------------------

TEST(ReplayQueueSmokeTest, CapacityEightRejectNewAndUnknownId) {
  InferenceClient client(ReplayOptions(LoadFixtures()));
  std::vector<std::string> ids;
  for (size_t i = 0; i < kMaxOutstanding; ++i) {
    const SubmitOutcome outcome = client.Submit(Envelope());
    ASSERT_EQ(outcome.status, Status::kOk);
    ids.push_back(outcome.request_id);
  }
  EXPECT_EQ(client.Outstanding(), kMaxOutstanding);
  EXPECT_EQ(client.Submit(Envelope()).status, Status::kQueueFull);
  EXPECT_EQ(client.Outstanding(), kMaxOutstanding);

  EXPECT_EQ(client.Poll("inf-99").status, Status::kUnknownId);
  EXPECT_EQ(client.Cancel("inf-99").status, Status::kUnknownId);
  EXPECT_EQ(client.Reap("inf-99").status, Status::kUnknownId);

  // The eight accepted requests are intact and complete from the fixture.
  EXPECT_EQ(client.Tick(kMaxOutstanding), kMaxOutstanding);
  for (const std::string& id : ids) {
    EXPECT_EQ(client.Poll(id).status, Status::kComplete);
  }
  // A terminal replay status releases its slot on Reap like any other.
  EXPECT_EQ(client.Reap(ids[0]).status, Status::kComplete);
  EXPECT_EQ(client.Submit(Envelope("nobody", "sha256:in", 1)).status,
            Status::kOk);
  EXPECT_EQ(client.Tick(), 1u);
  const std::string miss_id = "inf-9";
  EXPECT_EQ(client.Poll(miss_id).status, Status::kReplayMiss);
  EXPECT_EQ(client.Reap(miss_id).status, Status::kReplayMiss);
  EXPECT_EQ(client.Poll(miss_id).status, Status::kUnknownId);
}

TEST(ReplayQueueSmokeTest, CancelBeforeTickAndShutdownStillWork) {
  InferenceClient client(ReplayOptions(LoadFixtures()));
  const SubmitOutcome cancelled = client.Submit(Envelope());
  EXPECT_EQ(client.Cancel(cancelled.request_id).status, Status::kCancelled);
  EXPECT_EQ(client.Tick(), 0u);
  const SubmitOutcome pending = client.Submit(Envelope());
  client.Shutdown();
  EXPECT_EQ(client.Poll(pending.request_id).status, Status::kShutdown);
  EXPECT_EQ(client.Submit(Envelope()).status, Status::kShutdown);
}

TEST(ReplayQueueSmokeTest, DefaultClientStillUsesStubWorker) {
  InferenceClientOptions options;
  options.clock = [] { return ClockReading{1700000000, 100}; };
  InferenceClient client(std::move(options));
  const PollOutcome done = SubmitTickPoll(client, Envelope());
  ASSERT_EQ(done.status, Status::kComplete);
  EXPECT_EQ(done.result->View().output_digest, "stub-output:inf-1");
}

TEST(ReplayQueueSmokeTest, ReplayWorkerIsAnInferenceWorker) {
  const std::shared_ptr<const InferenceWorker> worker = LoadFixtures();
  const OwnedInferenceEnvelope owned(Envelope());
  const WorkerOutcome hit = worker->Complete(owned, "inf-1", {1700000000, 0});
  EXPECT_EQ(hit.status, WorkerStatus::kComplete);
  EXPECT_TRUE(hit.result.has_value());
}

}  // namespace
}  // namespace intrinsic::inference
