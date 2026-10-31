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

#include "intrinsic_inference/queue/policy_worker.h"

#include <unistd.h>

#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <map>
#include <memory>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include "intrinsic/embodiment/stamped_header_policy.h"
#include "intrinsic/safety/safety_decision_assessor.h"
#include "intrinsic/vehicle/proto/vehicle_command.pb.h"
#include "intrinsic_inference/envelope/inference_envelope_contract_policy.h"
#include "intrinsic_inference/queue/inference_client.h"
#include "intrinsic_inference/queue/stub_worker.h"
#include "gtest/gtest.h"

namespace intrinsic::inference {
namespace {

namespace fs = std::filesystem;
using embodiment::ClockReading;
using intrinsic_proto::vehicle::DesiredMotion;
using Status = InferenceQueueStatus;

constexpr std::string_view kSnapshot =
    "0123456789abcdef0123456789abcdef0123456789abcdef0123456789abcdef";
constexpr ClockReading kNow = {1700000000, 100};

std::string FixtureDir() {
  const char *srcdir = std::getenv("TEST_SRCDIR");
  const char *workspace = std::getenv("TEST_WORKSPACE");
  if (srcdir != nullptr && workspace != nullptr) {
    return std::string(srcdir) + "/" + workspace +
           "/intrinsic_inference/queue/fixtures/policy";
  }
  return "intrinsic_inference/queue/fixtures/policy";
}

// Envelope for the policy fixtures' match key. Created at 1700000000s,
// deadline 1700000000.5s. The string views must outlive Submit.
InferenceEnvelopeView Envelope(std::string_view digest,
                               std::string_view model = "uuv_shadow_policy",
                               uint64_t epoch = 1) {
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
  oip.model_name = "uuv_shadow_policy";
  oip.model_version = "1";
  oip.request_id = "req-0001";
  return InferenceEnvelopeView{common, oip};
}

DesiredMotion PoseMotion(double x, double z) {
  DesiredMotion motion;
  auto *header = motion.mutable_header();
  header->set_sequence(7);
  header->mutable_source_time()->set_seconds(1700000000);
  header->mutable_receive_time()->set_seconds(1700000000);
  header->set_source_id("uuv_shadow_policy");
  header->set_frame_id("world_enu");
  header->set_clock_domain("utc");
  header->mutable_validity()->set_state(
      intrinsic_proto::embodiment::Validity::STATE_VALID);
  auto *pose = motion.mutable_pose()->mutable_pose_world_from_body();
  pose->mutable_position()->set_x(x);
  pose->mutable_position()->set_z(z);
  pose->mutable_orientation()->set_w(1);
  motion.mutable_provenance()->set_model_id("uuv_shadow_policy");
  return motion;
}

InferenceClientOptions Options(std::shared_ptr<const InferenceWorker> worker,
                               bool complete_on_submit = true) {
  InferenceClientOptions options;
  options.clock = [] { return kNow; };
  options.complete_on_submit = complete_on_submit;
  options.worker = std::move(worker);
  return options;
}

PollOutcome SubmitPoll(InferenceClient &client,
                       const InferenceEnvelopeView &request) {
  const SubmitOutcome submitted = client.Submit(request);
  EXPECT_EQ(submitted.status, Status::kOk);
  return client.Poll(submitted.request_id);
}

std::map<std::string, DesiredMotion, std::less<>> Script() {
  std::map<std::string, DesiredMotion, std::less<>> script;
  script.emplace("sha256:mock-a", PoseMotion(1, -2));
  script.emplace("sha256:mock-b", PoseMotion(3, -4));
  return script;
}

bool IsLowerHex64(const std::string &s) {
  if (s.size() != 64) {
    return false;
  }
  for (const char c : s) {
    if (!((c >= '0' && c <= '9') || (c >= 'a' && c <= 'f'))) {
      return false;
    }
  }
  return true;
}

class TempDir {
public:
  TempDir() {
    const char *base = std::getenv("TEST_TMPDIR");
    static int counter = 0;
    path_ =
        fs::path(base != nullptr ? base : fs::temp_directory_path().string()) /
        ("policy_fixtures_" + std::to_string(::getpid()) + "_" +
         std::to_string(counter++));
    fs::create_directories(path_);
  }
  ~TempDir() {
    std::error_code ec;
    fs::remove_all(path_, ec);
  }
  std::string path() const { return path_.string(); }

private:
  fs::path path_;
};

// W1 store ------------------------------------------------------------------

TEST(DesiredMotionStoreTest, PutFindIdempotentAndStable) {
  DesiredMotionStore store;
  const DesiredMotion motion = PoseMotion(1, -2);
  const std::string digest = store.Put(motion);
  EXPECT_EQ(digest, safety::ComputeOriginalIntentDigest(motion));
  EXPECT_TRUE(IsLowerHex64(digest));
  EXPECT_EQ(store.size(), 1u);

  const DesiredMotion *found = store.Find(digest);
  ASSERT_NE(found, nullptr);
  EXPECT_EQ(found->SerializeAsString(), motion.SerializeAsString());

  EXPECT_EQ(store.Put(motion), digest);
  EXPECT_EQ(store.size(), 1u);
  EXPECT_EQ(store.Find(digest), found);

  EXPECT_EQ(store.Find(std::string(64, '0')), nullptr);
  EXPECT_EQ(store.Find(""), nullptr);

  for (int i = 0; i < 100; ++i) {
    store.Put(PoseMotion(100 + i, -i));
  }
  EXPECT_EQ(store.size(), 101u);
  EXPECT_EQ(store.Find(digest), found);
  EXPECT_EQ(found->SerializeAsString(), motion.SerializeAsString());
}

// W2 mock hit ---------------------------------------------------------------

TEST(MockPolicyWorkerTest, ScriptedHitCompletesWithMotionDigest) {
  auto store = std::make_shared<DesiredMotionStore>();
  InferenceClient client(
      Options(std::make_shared<MockPolicyWorker>(store, Script())));
  EXPECT_EQ(store->size(), 2u);

  const SubmitOutcome submitted = client.Submit(Envelope("sha256:mock-a"));
  ASSERT_EQ(submitted.status, Status::kOk);
  const PollOutcome done = client.Poll(submitted.request_id);
  ASSERT_EQ(done.status, Status::kComplete);
  ASSERT_TRUE(done.result.has_value());
  const InferenceResultView result = done.result->View();

  const DesiredMotion motion = PoseMotion(1, -2);
  EXPECT_EQ(result.output_digest, store->Put(motion));
  EXPECT_EQ(result.output_digest, safety::ComputeOriginalIntentDigest(motion));
  const DesiredMotion *found = store->Find(result.output_digest);
  ASSERT_NE(found, nullptr);
  EXPECT_EQ(found->SerializeAsString(), motion.SerializeAsString());
  EXPECT_EQ(store->size(), 2u);

  const InferenceEnvelopeContractAssessment assessment =
      AssessInferenceResult(result);
  EXPECT_EQ(assessment.error, InferenceEnvelopeContractError::kNone);
  EXPECT_TRUE(assessment.accepted);
  EXPECT_EQ(result.oip_result.request_id, "req-0001");
  EXPECT_EQ(result.common.input_digest, "sha256:mock-a");
}

TEST(MockPolicyWorkerTest, ScriptedHitThroughTick) {
  auto store = std::make_shared<DesiredMotionStore>();
  InferenceClient client(
      Options(std::make_shared<MockPolicyWorker>(store, Script()),
              /*complete_on_submit=*/false));
  const SubmitOutcome submitted = client.Submit(Envelope("sha256:mock-b"));
  ASSERT_EQ(submitted.status, Status::kOk);
  EXPECT_EQ(client.Poll(submitted.request_id).status, Status::kOk);
  ASSERT_EQ(client.Tick(), 1u);
  const PollOutcome done = client.Poll(submitted.request_id);
  ASSERT_EQ(done.status, Status::kComplete);
  EXPECT_EQ(done.result->View().output_digest,
            safety::ComputeOriginalIntentDigest(PoseMotion(3, -4)));
}

// W3 mock miss --------------------------------------------------------------

TEST(MockPolicyWorkerTest, UnscriptedKeyIsReplayMiss) {
  auto store = std::make_shared<DesiredMotionStore>();
  InferenceClient client(
      Options(std::make_shared<MockPolicyWorker>(store, Script())));
  const PollOutcome out = SubmitPoll(client, Envelope("sha256:unscripted"));
  EXPECT_EQ(out.status, Status::kReplayMiss);
  EXPECT_FALSE(out.result.has_value());
}

// W4 tamper -----------------------------------------------------------------

TEST(MockPolicyWorkerTest, TamperedOutputDigestIsNotInStore) {
  auto store = std::make_shared<DesiredMotionStore>();
  InferenceClient client(
      Options(std::make_shared<MockPolicyWorker>(store, Script())));
  const PollOutcome done = SubmitPoll(client, Envelope("sha256:mock-a"));
  ASSERT_EQ(done.status, Status::kComplete);
  InferenceResultView view = done.result->View();
  ASSERT_NE(store->Find(view.output_digest), nullptr);

  std::string tampered(view.output_digest);
  tampered[0] = tampered[0] == '0' ? '1' : '0';
  view.output_digest = tampered;
  const InferenceResultShell copy(view);
  EXPECT_EQ(store->Find(copy.View().output_digest), nullptr);
}

// W5-W8 recorded ------------------------------------------------------------

TEST(RecordedPolicyWorkerTest, ActiveHit) {
  auto store = std::make_shared<DesiredMotionStore>();
  InferenceClient client(
      Options(std::make_shared<RecordedPolicyWorker>(store, FixtureDir())));
  const PollOutcome done = SubmitPoll(client, Envelope("sha256:policy-active"));
  ASSERT_EQ(done.status, Status::kComplete);
  ASSERT_TRUE(done.result.has_value());
  const InferenceResultView result = done.result->View();
  EXPECT_TRUE(AssessInferenceResult(result).accepted);
  EXPECT_EQ(result.oip_result.request_id, "rec-policy-active");

  const DesiredMotion *found = store->Find(result.output_digest);
  ASSERT_NE(found, nullptr);
  EXPECT_EQ(found->pose().pose_world_from_body().position().x(), 4);
  EXPECT_EQ(found->pose().pose_world_from_body().position().z(), -10);
  EXPECT_EQ(found->header().frame_id(), "world_enu");
  EXPECT_EQ(found->provenance().model_id(), "uuv_shadow_policy");
  EXPECT_EQ(safety::ComputeOriginalIntentDigest(*found), result.output_digest);
}

TEST(RecordedPolicyWorkerTest, ActiveHitThroughTick) {
  auto store = std::make_shared<DesiredMotionStore>();
  InferenceClient client(
      Options(std::make_shared<RecordedPolicyWorker>(store, FixtureDir()),
              /*complete_on_submit=*/false));
  const SubmitOutcome submitted =
      client.Submit(Envelope("sha256:policy-active"));
  ASSERT_EQ(submitted.status, Status::kOk);
  ASSERT_EQ(client.Tick(), 1u);
  const PollOutcome done = client.Poll(submitted.request_id);
  ASSERT_EQ(done.status, Status::kComplete);
  EXPECT_NE(store->Find(done.result->View().output_digest), nullptr);
}

TEST(RecordedPolicyWorkerTest, LateIsExpiredWithShellAndMotion) {
  auto store = std::make_shared<DesiredMotionStore>();
  InferenceClient client(
      Options(std::make_shared<RecordedPolicyWorker>(store, FixtureDir())));
  const PollOutcome out = SubmitPoll(client, Envelope("sha256:policy-late"));
  ASSERT_EQ(out.status, Status::kReplayExpired);
  ASSERT_TRUE(out.result.has_value());
  const InferenceResultView result = out.result->View();
  const ClockReading deadline = result.common.deadline;
  EXPECT_TRUE(
      deadline.seconds < kNow.seconds ||
      (deadline.seconds == kNow.seconds && deadline.nanos < kNow.nanos));
  EXPECT_NE(store->Find(result.output_digest), nullptr);
}

TEST(RecordedPolicyWorkerTest, MismatchedDigestIsNotInStore) {
  auto store = std::make_shared<DesiredMotionStore>();
  InferenceClient client(
      Options(std::make_shared<RecordedPolicyWorker>(store, FixtureDir())));
  const PollOutcome out =
      SubmitPoll(client, Envelope("sha256:policy-mismatch"));
  ASSERT_EQ(out.status, Status::kComplete);
  ASSERT_TRUE(out.result.has_value());
  EXPECT_EQ(store->Find(out.result->View().output_digest), nullptr);
}

TEST(RecordedPolicyWorkerTest, FailuresAndLoadReport) {
  auto store = std::make_shared<DesiredMotionStore>();
  auto worker = std::make_shared<RecordedPolicyWorker>(store, FixtureDir());
  EXPECT_EQ(worker->fixture_count(), 4u);
  EXPECT_EQ(worker->skipped_files(),
            std::vector<std::string>{"corrupt_unparseable.json"});
  EXPECT_EQ(worker->loaded_motion_count(), 3u);
  EXPECT_EQ(worker->skipped_motion_files(),
            std::vector<std::string>{"bad.motion.textproto"});
  EXPECT_EQ(store->size(), 3u);

  InferenceClient client(Options(worker));
  const PollOutcome corrupt =
      SubmitPoll(client, Envelope("sha256:policy-corrupt"));
  EXPECT_EQ(corrupt.status, Status::kCorruptFixture);
  EXPECT_FALSE(corrupt.result.has_value());
  const PollOutcome unparseable =
      SubmitPoll(client, Envelope("sha256:policy-unparseable"));
  EXPECT_EQ(unparseable.status, Status::kReplayMiss);
  EXPECT_FALSE(unparseable.result.has_value());
  const PollOutcome unknown = SubmitPoll(client, Envelope("sha256:nothing"));
  EXPECT_EQ(unknown.status, Status::kReplayMiss);
  EXPECT_FALSE(unknown.result.has_value());
}

// W9 empty and missing directories -------------------------------------------

TEST(RecordedPolicyWorkerTest, EmptyAndMissingDirectories) {
  const TempDir empty;
  const std::string missing = empty.path() + "/does_not_exist";
  for (const std::string &dir : {empty.path(), missing}) {
    auto store = std::make_shared<DesiredMotionStore>();
    auto worker = std::make_shared<RecordedPolicyWorker>(store, dir);
    EXPECT_EQ(worker->fixture_count(), 0u);
    EXPECT_EQ(worker->loaded_motion_count(), 0u);
    EXPECT_TRUE(worker->skipped_files().empty());
    EXPECT_TRUE(worker->skipped_motion_files().empty());
    EXPECT_EQ(store->size(), 0u);
    InferenceClient client(Options(worker));
    for (const char *digest : {"sha256:policy-active", "sha256:policy-late"}) {
      const PollOutcome out = SubmitPoll(client, Envelope(digest));
      EXPECT_EQ(out.status, Status::kReplayMiss);
      EXPECT_FALSE(out.result.has_value());
    }
  }
}

// W10 determinism ------------------------------------------------------------

struct Observed {
  Status status;
  std::string output_digest;
  std::string request_id;
  std::string input_digest;
  bool has_motion;
};

std::vector<Observed> RunAll(bool recorded) {
  auto store = std::make_shared<DesiredMotionStore>();
  std::shared_ptr<const InferenceWorker> worker;
  std::vector<std::string> keys;
  if (recorded) {
    worker = std::make_shared<RecordedPolicyWorker>(store, FixtureDir());
    keys = {"sha256:policy-active", "sha256:policy-late",
            "sha256:policy-mismatch", "sha256:policy-corrupt",
            "sha256:unknown"};
  } else {
    worker = std::make_shared<MockPolicyWorker>(store, Script());
    keys = {"sha256:mock-a", "sha256:mock-b", "sha256:unscripted"};
  }
  InferenceClient client(Options(worker));
  std::vector<Observed> observed;
  for (const std::string &key : keys) {
    const PollOutcome out = SubmitPoll(client, Envelope(key));
    Observed row{out.status, "", "", "", false};
    if (out.result.has_value()) {
      const InferenceResultView view = out.result->View();
      row.output_digest = std::string(view.output_digest);
      row.request_id = std::string(view.oip_result.request_id);
      row.input_digest = std::string(view.common.input_digest);
      row.has_motion = store->Find(view.output_digest) != nullptr;
    }
    observed.push_back(std::move(row));
  }
  return observed;
}

TEST(PolicyWorkerDeterminismTest, TwoFreshRunsAreIdentical) {
  for (const bool recorded : {false, true}) {
    const std::vector<Observed> a = RunAll(recorded);
    const std::vector<Observed> b = RunAll(recorded);
    ASSERT_EQ(a.size(), b.size());
    for (size_t i = 0; i < a.size(); ++i) {
      EXPECT_EQ(a[i].status, b[i].status);
      EXPECT_EQ(a[i].output_digest, b[i].output_digest);
      EXPECT_EQ(a[i].request_id, b[i].request_id);
      EXPECT_EQ(a[i].input_digest, b[i].input_digest);
      EXPECT_EQ(a[i].has_motion, b[i].has_motion);
    }
  }
}

} // namespace
} // namespace intrinsic::inference
