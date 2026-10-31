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

#include "intrinsic/simulation/gazebo/scenarios/uuv_reference/shadow_policy_runner.h"

#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <fstream>
#include <map>
#include <memory>
#include <sstream>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include "google/protobuf/text_format.h"
#include "gtest/gtest.h"
#include "intrinsic/embodiment/stamped_header_policy.h"
#include "intrinsic/vehicle/proto/vehicle_command.pb.h"
#include "intrinsic_inference/envelope/inference_envelope_contract_policy.h"
#include "intrinsic_inference/queue/inference_client.h"
#include "intrinsic_inference/queue/policy_worker.h"
#include "intrinsic_inference/queue/stub_worker.h"

namespace intrinsic::simulation::uuv_reference {
namespace {

using embodiment::ClockReading;
using inference::DesiredMotionStore;
using inference::InferenceEnvelopeContractError;
using inference::InferenceQueueStatus;
using inference::InferenceWorker;
using inference::MockPolicyWorker;
using inference::OwnedInferenceEnvelope;
using inference::RecordedPolicyWorker;
using inference::WorkerOutcome;
using intrinsic_proto::vehicle::DesiredMotion;
using Outcome = ShadowPolicyOutcome;
using Status = InferenceQueueStatus;

constexpr int64_t kDt = 125'000'000;
constexpr int64_t kNanosPerSecond = 1'000'000'000;
constexpr std::string_view kSnapshot =
    "0123456789abcdef0123456789abcdef0123456789abcdef0123456789abcdef";
constexpr ClockReading kFixtureOrigin = {1700000000, 0};

std::string RunfilesPath(const std::string& relative) {
  const char* srcdir = std::getenv("TEST_SRCDIR");
  const char* workspace = std::getenv("TEST_WORKSPACE");
  if (srcdir != nullptr && workspace != nullptr) {
    return std::string(srcdir) + "/" + workspace + "/" + relative;
  }
  return relative;
}

std::string FixtureDir() {
  return RunfilesPath("intrinsic_inference/queue/fixtures/policy");
}

std::string ReadFile(const std::string& path) {
  std::ifstream file(path);
  std::stringstream contents;
  contents << file.rdbuf();
  return contents.str();
}

DesiredMotion PoseMotion(double x, double z) {
  DesiredMotion motion;
  auto* header = motion.mutable_header();
  header->set_sequence(7);
  header->mutable_source_time()->set_seconds(1700000000);
  header->mutable_receive_time()->set_seconds(1700000000);
  header->set_source_id("uuv_shadow_policy");
  header->set_frame_id("world_enu");
  header->set_clock_domain("utc");
  header->mutable_validity()->set_state(
      intrinsic_proto::embodiment::Validity::STATE_VALID);
  auto* pose = motion.mutable_pose()->mutable_pose_world_from_body();
  pose->mutable_position()->set_x(x);
  pose->mutable_position()->set_z(z);
  pose->mutable_orientation()->set_w(1);
  motion.mutable_provenance()->set_model_id("uuv_shadow_policy");
  return motion;
}

ShadowEnvelopeInputs Inputs(std::string digest, uint64_t epoch = 1,
                            std::string snapshot = std::string(kSnapshot)) {
  ShadowEnvelopeInputs inputs;
  inputs.input_digest = std::move(digest);
  inputs.state_epoch = epoch;
  inputs.world_snapshot_id = std::move(snapshot);
  return inputs;
}

int64_t StepTime(int64_t k) { return (k + 1) * kDt; }

struct Rig {
  std::shared_ptr<DesiredMotionStore> store =
      std::make_shared<DesiredMotionStore>();
  DesiredMotion motion = PoseMotion(1, -2);
  std::shared_ptr<InferenceWorker> worker;
  std::unique_ptr<ShadowPolicyRunner> runner;
};

ShadowPolicyRunnerOptions BaseOptions(const Rig& rig) {
  ShadowPolicyRunnerOptions options;
  options.worker = rig.worker;
  options.store = rig.store;
  options.dt_ns = kDt;
  return options;
}

// Mock worker scripted with {"sha256:step" -> PoseMotion(1, -2)}.
Rig MockRig(bool complete_on_submit = true, size_t ticks_per_step = 0,
            ClockReading origin = {}) {
  Rig rig;
  rig.worker = std::make_shared<MockPolicyWorker>(
      rig.store, std::map<std::string, DesiredMotion, std::less<>>{
                     {"sha256:step", rig.motion}});
  ShadowPolicyRunnerOptions options = BaseOptions(rig);
  options.complete_on_submit = complete_on_submit;
  options.ticks_per_step = ticks_per_step;
  options.clock_origin = origin;
  rig.runner = std::make_unique<ShadowPolicyRunner>(std::move(options));
  return rig;
}

Rig RecordedRig() {
  Rig rig;
  rig.worker = std::make_shared<RecordedPolicyWorker>(rig.store, FixtureDir());
  ShadowPolicyRunnerOptions options = BaseOptions(rig);
  options.clock_origin = kFixtureOrigin;
  rig.runner = std::make_unique<ShadowPolicyRunner>(std::move(options));
  return rig;
}

// Delegates to a worker and keeps a copy of every envelope it completes.
class RecordingWorker final : public InferenceWorker {
 public:
  explicit RecordingWorker(std::shared_ptr<const InferenceWorker> inner)
      : inner_(std::move(inner)) {}

  WorkerOutcome Complete(const OwnedInferenceEnvelope& envelope,
                         std::string_view queue_request_id,
                         ClockReading now) const override {
    seen_.push_back(envelope);
    return inner_->Complete(envelope, queue_request_id, now);
  }

  const std::vector<OwnedInferenceEnvelope>& seen() const { return seen_; }

 private:
  std::shared_ptr<const InferenceWorker> inner_;
  mutable std::vector<OwnedInferenceEnvelope> seen_;
};

std::string DumpTime(ClockReading time) {
  return std::to_string(time.seconds) + "." + std::to_string(time.nanos);
}

std::string DumpResolution(const ShadowPolicyResolution& r) {
  std::ostringstream out;
  out << "k=" << r.submit_k << " id=" << r.request_id
      << " outcome=" << static_cast<int>(r.outcome)
      << " status=" << static_cast<int>(r.status)
      << " error=" << static_cast<int>(r.envelope_error)
      << " mismatch=" << r.digest_mismatch;
  if (r.result.has_value()) {
    const inference::InferenceResultView v = r.result->View();
    out << " result{" << v.common.frame_id << " " << v.common.validity_state
        << " " << DumpTime(v.common.source_time) << " "
        << DumpTime(v.common.deadline) << " "
        << v.common.validity_horizon.seconds << "."
        << v.common.validity_horizon.nanos << " " << v.common.state_epoch << " "
        << v.common.world_snapshot_id << " " << v.common.input_digest << " "
        << v.common.provenance_model_id << " " << v.common.confidence_present
        << ":" << v.common.confidence << " " << v.common.uncertainty_present
        << ":" << v.common.uncertainty << " " << v.oip_result.model_name << " "
        << v.oip_result.model_version << " " << v.oip_result.request_id << " "
        << v.output_digest << "}";
  } else {
    out << " result{}";
  }
  if (r.motion.has_value()) {
    out << " motion{" << r.motion->ShortDebugString() << "}";
  } else {
    out << " motion{}";
  }
  return out.str();
}

std::string DumpStep(const StepResult& step) {
  std::ostringstream out;
  out << "error=" << step.error << " submitted=" << step.submitted_request_id;
  for (const ShadowPolicyResolution& r : step.resolutions) {
    out << "\n  " << DumpResolution(r);
  }
  return out.str();
}

TEST(ShadowPolicyRunnerTest, S1AbsentWithoutWorker) {
  ShadowPolicyRunnerOptions options;
  options.dt_ns = kDt;
  ShadowPolicyRunner runner(std::move(options));
  EXPECT_FALSE(runner.configured());
  for (int64_t k = 0; k < 10; ++k) {
    const StepResult step = runner.Step(k, StepTime(k), Inputs("sha256:step"));
    EXPECT_FALSE(step.error);
    EXPECT_TRUE(step.submitted_request_id.empty());
    ASSERT_EQ(step.resolutions.size(), 1u);
    EXPECT_EQ(step.resolutions[0].outcome, Outcome::kAbsent);
    EXPECT_EQ(step.resolutions[0].submit_k, k);
    EXPECT_TRUE(step.resolutions[0].request_id.empty());
    EXPECT_FALSE(step.resolutions[0].result.has_value());
    EXPECT_FALSE(step.resolutions[0].motion.has_value());
    EXPECT_EQ(runner.outstanding(), 0u);
  }
}

TEST(ShadowPolicyRunnerTest, S2ActiveMockSynchronous) {
  Rig rig = MockRig();
  EXPECT_TRUE(rig.runner->configured());
  for (int64_t k = 0; k < 3; ++k) {
    const StepResult step =
        rig.runner->Step(k, StepTime(k), Inputs("sha256:step"));
    EXPECT_FALSE(step.error);
    EXPECT_FALSE(step.submitted_request_id.empty());
    ASSERT_EQ(step.resolutions.size(), 1u);
    const ShadowPolicyResolution& r = step.resolutions[0];
    EXPECT_EQ(r.outcome, Outcome::kActive);
    EXPECT_EQ(r.status, Status::kComplete);
    EXPECT_EQ(r.submit_k, k);
    EXPECT_EQ(r.request_id, step.submitted_request_id);
    EXPECT_FALSE(r.digest_mismatch);
    ASSERT_TRUE(r.motion.has_value());
    EXPECT_EQ(r.motion->SerializeAsString(), rig.motion.SerializeAsString());
    ASSERT_TRUE(r.result.has_value());
    EXPECT_EQ(r.result->View().output_digest, rig.store->Put(rig.motion));
    EXPECT_EQ(rig.runner->outstanding(), 0u);
  }
}

TEST(ShadowPolicyRunnerTest, S3ActiveMockTickMode) {
  Rig rig = MockRig(/*complete_on_submit=*/false, /*ticks_per_step=*/1);
  StepResult step = rig.runner->Step(0, StepTime(0), Inputs("sha256:step"));
  EXPECT_FALSE(step.error);
  EXPECT_FALSE(step.submitted_request_id.empty());
  EXPECT_TRUE(step.resolutions.empty());
  EXPECT_EQ(rig.runner->outstanding(), 1u);
  const std::string first_id = step.submitted_request_id;

  step = rig.runner->Step(1, StepTime(1), Inputs("sha256:step"));
  ASSERT_EQ(step.resolutions.size(), 1u);
  const ShadowPolicyResolution& r = step.resolutions[0];
  EXPECT_EQ(r.outcome, Outcome::kActive);
  EXPECT_EQ(r.status, Status::kComplete);
  EXPECT_EQ(r.submit_k, 0);
  EXPECT_EQ(r.request_id, first_id);
  ASSERT_TRUE(r.motion.has_value());
  EXPECT_EQ(r.motion->SerializeAsString(), rig.motion.SerializeAsString());
  EXPECT_FALSE(step.submitted_request_id.empty());
  EXPECT_NE(step.submitted_request_id, first_id);
  EXPECT_EQ(rig.runner->outstanding(), 1u);
}

TEST(ShadowPolicyRunnerTest, S4EnvelopeContent) {
  Rig rig;
  rig.worker = std::make_shared<MockPolicyWorker>(
      rig.store, std::map<std::string, DesiredMotion, std::less<>>{
                     {"sha256:step", rig.motion}});
  auto recorder = std::make_shared<RecordingWorker>(rig.worker);
  rig.worker = recorder;
  ShadowPolicyRunnerOptions options = BaseOptions(rig);
  options.clock_origin = {10, 950'000'000};
  ShadowPolicyRunner runner(std::move(options));

  ASSERT_FALSE(
      runner.Step(0, kDt, Inputs("sha256:step", 7, std::string(kSnapshot)))
          .error);
  ASSERT_FALSE(
      runner.Step(3, 3 * kNanosPerSecond + 1, Inputs("sha256:step")).error);
  ASSERT_EQ(recorder->seen().size(), 2u);

  const inference::InferenceEnvelopeView first = recorder->seen()[0].View();
  EXPECT_TRUE(first.common.header_present);
  EXPECT_TRUE(first.common.validity_present);
  EXPECT_EQ(first.common.validity_state, 1);
  EXPECT_EQ(first.common.frame_id, "world_enu");
  EXPECT_TRUE(first.common.source_time_present);
  EXPECT_EQ(first.common.source_time.seconds, 11);
  EXPECT_EQ(first.common.source_time.nanos, 75'000'000);
  EXPECT_TRUE(first.common.deadline_present);
  EXPECT_EQ(first.common.deadline.seconds, 11);
  EXPECT_EQ(first.common.deadline.nanos, 325'000'000);
  EXPECT_TRUE(first.common.validity_horizon_present);
  EXPECT_EQ(first.common.validity_horizon.seconds, 0);
  EXPECT_EQ(first.common.validity_horizon.nanos, 250'000'000);
  EXPECT_EQ(first.common.state_epoch, 7u);
  EXPECT_EQ(first.common.world_snapshot_id, kSnapshot);
  EXPECT_EQ(first.common.input_digest, "sha256:step");
  EXPECT_TRUE(first.common.provenance_present);
  EXPECT_EQ(first.common.provenance_model_id, "uuv_shadow_policy");
  EXPECT_FALSE(first.common.confidence_present);
  EXPECT_FALSE(first.common.uncertainty_present);
  EXPECT_FALSE(first.common.metadata_present);
  EXPECT_TRUE(first.oip_request.message_set);
  EXPECT_EQ(first.oip_request.model_name, "uuv_shadow_policy");
  EXPECT_EQ(first.oip_request.model_version, "1");
  EXPECT_EQ(first.oip_request.request_id, "shadow-0");
  EXPECT_TRUE(inference::AssessInferenceEnvelope(first).accepted);

  const inference::InferenceEnvelopeView second = recorder->seen()[1].View();
  EXPECT_EQ(second.common.source_time.seconds, 13);
  EXPECT_EQ(second.common.source_time.nanos, 950'000'001);
  EXPECT_EQ(second.common.deadline.seconds, 14);
  EXPECT_EQ(second.common.deadline.nanos, 200'000'001);
  EXPECT_EQ(second.oip_request.request_id, "shadow-3");
  EXPECT_TRUE(inference::AssessInferenceEnvelope(second).accepted);
}

TEST(ShadowPolicyRunnerTest, S4OptionOverrides) {
  Rig rig;
  rig.worker = std::make_shared<MockPolicyWorker>(
      rig.store, std::map<std::string, DesiredMotion, std::less<>>{
                     {"sha256:step", rig.motion}});
  auto recorder = std::make_shared<RecordingWorker>(rig.worker);
  rig.worker = recorder;
  ShadowPolicyRunnerOptions options = BaseOptions(rig);
  options.frame_id = "other_frame";
  options.model_id = "model-x";
  options.model_name = "name-x";
  options.model_version = "9";
  ShadowPolicyRunner runner(std::move(options));
  ASSERT_FALSE(runner.Step(0, kDt, Inputs("sha256:step")).error);
  ASSERT_EQ(recorder->seen().size(), 1u);
  const inference::InferenceEnvelopeView view = recorder->seen()[0].View();
  EXPECT_EQ(view.common.frame_id, "other_frame");
  EXPECT_EQ(view.common.provenance_model_id, "model-x");
  EXPECT_EQ(view.oip_request.model_name, "name-x");
  EXPECT_EQ(view.oip_request.model_version, "9");
}

TEST(ShadowPolicyRunnerTest, S5FailedClasses) {
  {
    Rig rig = MockRig();
    const StepResult step =
        rig.runner->Step(0, StepTime(0), Inputs("sha256:unscripted"));
    ASSERT_EQ(step.resolutions.size(), 1u);
    EXPECT_EQ(step.resolutions[0].outcome, Outcome::kFailed);
    EXPECT_EQ(step.resolutions[0].status, Status::kReplayMiss);
    EXPECT_FALSE(step.resolutions[0].motion.has_value());
  }
  {
    Rig rig = RecordedRig();
    const StepResult step =
        rig.runner->Step(0, StepTime(0), Inputs("sha256:policy-corrupt"));
    ASSERT_EQ(step.resolutions.size(), 1u);
    EXPECT_EQ(step.resolutions[0].outcome, Outcome::kFailed);
    EXPECT_EQ(step.resolutions[0].status, Status::kCorruptFixture);
  }
  {
    Rig rig = RecordedRig();
    const StepResult step =
        rig.runner->Step(0, StepTime(0), Inputs("sha256:policy-unparseable"));
    ASSERT_EQ(step.resolutions.size(), 1u);
    EXPECT_EQ(step.resolutions[0].outcome, Outcome::kFailed);
    EXPECT_EQ(step.resolutions[0].status, Status::kReplayMiss);
  }
  {
    Rig rig = MockRig();
    const StepResult step = rig.runner->Step(0, StepTime(0), Inputs(""));
    EXPECT_FALSE(step.error);
    EXPECT_TRUE(step.submitted_request_id.empty());
    ASSERT_EQ(step.resolutions.size(), 1u);
    EXPECT_EQ(step.resolutions[0].outcome, Outcome::kFailed);
    EXPECT_EQ(step.resolutions[0].status, Status::kInvalidEnvelope);
    EXPECT_EQ(step.resolutions[0].envelope_error,
              InferenceEnvelopeContractError::kDigest);
    EXPECT_TRUE(step.resolutions[0].request_id.empty());
    EXPECT_EQ(step.resolutions[0].submit_k, 0);
    EXPECT_EQ(rig.runner->outstanding(), 0u);
  }
  {
    Rig rig = MockRig();
    const StepResult step =
        rig.runner->Step(0, StepTime(0), Inputs("sha256:step", 1, "XYZ"));
    ASSERT_EQ(step.resolutions.size(), 1u);
    EXPECT_EQ(step.resolutions[0].outcome, Outcome::kFailed);
    EXPECT_EQ(step.resolutions[0].status, Status::kInvalidEnvelope);
    EXPECT_EQ(step.resolutions[0].envelope_error,
              InferenceEnvelopeContractError::kSnapshotId);
    EXPECT_TRUE(step.resolutions[0].request_id.empty());
  }
  {
    Rig rig = RecordedRig();
    const StepResult step =
        rig.runner->Step(0, StepTime(0), Inputs("sha256:policy-mismatch"));
    ASSERT_EQ(step.resolutions.size(), 1u);
    const ShadowPolicyResolution& r = step.resolutions[0];
    EXPECT_EQ(r.outcome, Outcome::kFailed);
    EXPECT_EQ(r.status, Status::kComplete);
    EXPECT_TRUE(r.digest_mismatch);
    EXPECT_FALSE(r.motion.has_value());
    EXPECT_TRUE(r.result.has_value());
  }
}

TEST(ShadowPolicyRunnerTest, S6QueueFullThenCancelsFreeSlots) {
  Rig rig = MockRig(/*complete_on_submit=*/false, /*ticks_per_step=*/0);
  for (int64_t k = 0; k <= 7; ++k) {
    const StepResult step = rig.runner->Step(k, k, Inputs("sha256:step"));
    EXPECT_FALSE(step.error);
    EXPECT_FALSE(step.submitted_request_id.empty());
    EXPECT_TRUE(step.resolutions.empty());
  }
  EXPECT_EQ(rig.runner->outstanding(), 8u);

  StepResult full = rig.runner->Step(8, 8, Inputs("sha256:step"));
  EXPECT_FALSE(full.error);
  EXPECT_TRUE(full.submitted_request_id.empty());
  ASSERT_EQ(full.resolutions.size(), 1u);
  EXPECT_EQ(full.resolutions[0].outcome, Outcome::kFailed);
  EXPECT_EQ(full.resolutions[0].status, Status::kQueueFull);
  EXPECT_EQ(full.resolutions[0].submit_k, 8);
  EXPECT_TRUE(full.resolutions[0].request_id.empty());
  EXPECT_EQ(rig.runner->outstanding(), 8u);

  const StepResult next =
      rig.runner->Step(9, 2 * kDt + 8, Inputs("sha256:step"));
  EXPECT_FALSE(next.error);
  EXPECT_FALSE(next.submitted_request_id.empty());
  ASSERT_EQ(next.resolutions.size(), 8u);
  for (int64_t i = 0; i < 8; ++i) {
    EXPECT_EQ(next.resolutions[i].outcome, Outcome::kLate);
    EXPECT_EQ(next.resolutions[i].status, Status::kCancelled);
    EXPECT_EQ(next.resolutions[i].submit_k, i);
    EXPECT_FALSE(next.resolutions[i].request_id.empty());
    EXPECT_FALSE(next.resolutions[i].result.has_value());
    EXPECT_FALSE(next.resolutions[i].motion.has_value());
  }
  EXPECT_EQ(rig.runner->outstanding(), 1u);
}

TEST(ShadowPolicyRunnerTest, S7LateExpiredAndActiveRecorded) {
  {
    Rig rig = RecordedRig();
    const StepResult step =
        rig.runner->Step(0, StepTime(0), Inputs("sha256:policy-late"));
    ASSERT_EQ(step.resolutions.size(), 1u);
    const ShadowPolicyResolution& r = step.resolutions[0];
    EXPECT_EQ(r.outcome, Outcome::kLate);
    EXPECT_EQ(r.status, Status::kReplayExpired);
    EXPECT_FALSE(r.motion.has_value());
    ASSERT_TRUE(r.result.has_value());
    EXPECT_EQ(r.result->View().common.deadline.seconds, 1699999999);
    EXPECT_EQ(r.result->View().common.deadline.nanos, 900'000'000);
    EXPECT_EQ(rig.runner->outstanding(), 0u);
  }
  {
    Rig rig = RecordedRig();
    const StepResult step =
        rig.runner->Step(0, StepTime(0), Inputs("sha256:policy-active"));
    ASSERT_EQ(step.resolutions.size(), 1u);
    const ShadowPolicyResolution& r = step.resolutions[0];
    EXPECT_EQ(r.outcome, Outcome::kActive);
    EXPECT_EQ(r.status, Status::kComplete);
    EXPECT_FALSE(r.digest_mismatch);
    ASSERT_TRUE(r.motion.has_value());
    DesiredMotion expected;
    ASSERT_TRUE(google::protobuf::TextFormat::ParseFromString(
        ReadFile(FixtureDir() + "/active.motion.textproto"), &expected));
    EXPECT_EQ(r.motion->SerializeAsString(), expected.SerializeAsString());
  }
}

TEST(ShadowPolicyRunnerTest, S8LateWhenNotTerminal) {
  Rig rig = MockRig(/*complete_on_submit=*/false, /*ticks_per_step=*/0);
  StepResult step = rig.runner->Step(0, StepTime(0), Inputs("sha256:step"));
  ASSERT_FALSE(step.submitted_request_id.empty());
  const std::string first_id = step.submitted_request_id;
  EXPECT_EQ(rig.runner->outstanding(), 1u);

  step = rig.runner->Step(1, StepTime(1), Inputs("sha256:step"));
  EXPECT_TRUE(step.resolutions.empty());
  EXPECT_EQ(rig.runner->outstanding(), 2u);

  // t = 375 ms equals the deadline of the first request: not late.
  step = rig.runner->Step(2, StepTime(2), Inputs("sha256:step"));
  EXPECT_TRUE(step.resolutions.empty());
  EXPECT_EQ(rig.runner->outstanding(), 3u);

  // The first request is cancelled and its slot released. The runner owns
  // its client, so the release is observed through outstanding(): the
  // request resolved this step, the second one (deadline 500 ms) did not.
  step = rig.runner->Step(3, StepTime(3), Inputs("sha256:step"));
  ASSERT_EQ(step.resolutions.size(), 1u);
  EXPECT_EQ(step.resolutions[0].outcome, Outcome::kLate);
  EXPECT_EQ(step.resolutions[0].status, Status::kCancelled);
  EXPECT_EQ(step.resolutions[0].submit_k, 0);
  EXPECT_EQ(step.resolutions[0].request_id, first_id);
  EXPECT_EQ(rig.runner->outstanding(), 3u);

  // The second request is next; 500 ms is its deadline, so it is late at 625.
  step = rig.runner->Step(4, StepTime(4), Inputs("sha256:step"));
  ASSERT_EQ(step.resolutions.size(), 1u);
  EXPECT_EQ(step.resolutions[0].submit_k, 1);
  EXPECT_EQ(step.resolutions[0].status, Status::kCancelled);
  EXPECT_NE(step.resolutions[0].request_id, first_id);
}

TEST(ShadowPolicyRunnerTest, S9ClassifierTable) {
  const std::vector<Status> all = {
      Status::kOk,
      Status::kQueueFull,
      Status::kUnknownId,
      Status::kCancelled,
      Status::kShutdown,
      Status::kAlreadyComplete,
      Status::kDeadlineExpiredAtSubmit,
      Status::kComplete,
      Status::kInvalidEnvelope,
      Status::kReplayMiss,
      Status::kReplayExpired,
      Status::kCorruptFixture,
  };
  for (const Status status : all) {
    if (status != Status::kOk) {
      EXPECT_EQ(ClassifySubmitStatus(status), Outcome::kFailed)
          << static_cast<int>(status);
    }
    for (const bool found : {false, true}) {
      Outcome expected = Outcome::kFailed;
      if (status == Status::kComplete && found) {
        expected = Outcome::kActive;
      } else if (status == Status::kReplayExpired ||
                 status == Status::kCancelled) {
        expected = Outcome::kLate;
      }
      EXPECT_EQ(ClassifyTerminal(status, found), expected)
          << static_cast<int>(status) << " found=" << found;
    }
  }
  EXPECT_EQ(ClassifySubmitStatus(Status::kDeadlineExpiredAtSubmit),
            Outcome::kFailed);
}

TEST(ShadowPolicyRunnerTest, S10OptionErrors) {
  const auto expect_error = [](ShadowPolicyRunnerOptions options) {
    ShadowPolicyRunner runner(std::move(options));
    const StepResult step = runner.Step(0, kDt, Inputs("sha256:step"));
    EXPECT_TRUE(step.error);
    EXPECT_TRUE(step.resolutions.empty());
    EXPECT_TRUE(step.submitted_request_id.empty());
    EXPECT_EQ(runner.outstanding(), 0u);
  };
  {
    Rig rig = MockRig();
    ShadowPolicyRunnerOptions options = BaseOptions(rig);
    options.dt_ns = 0;
    expect_error(options);
    options.dt_ns = -1;
    expect_error(options);
  }
  {
    Rig rig = MockRig();
    ShadowPolicyRunnerOptions options = BaseOptions(rig);
    options.store = nullptr;
    expect_error(options);
  }
  {
    Rig rig = MockRig();
    ShadowPolicyRunnerOptions options = BaseOptions(rig);
    options.clock_origin = {-1, 0};
    expect_error(options);
    options.clock_origin = {0, -1};
    expect_error(options);
    options.clock_origin = {0, 1'000'000'000};
    expect_error(options);
  }
  {
    ShadowPolicyRunnerOptions options;
    expect_error(options);
  }
}

TEST(ShadowPolicyRunnerTest, S10StepArgumentErrors) {
  Rig rig = MockRig(/*complete_on_submit=*/false, /*ticks_per_step=*/0);
  const auto expect_rejected = [&](int64_t k, int64_t t, size_t outstanding) {
    const StepResult step = rig.runner->Step(k, t, Inputs("sha256:step"));
    EXPECT_TRUE(step.error) << k << " " << t;
    EXPECT_TRUE(step.resolutions.empty());
    EXPECT_TRUE(step.submitted_request_id.empty());
    EXPECT_EQ(rig.runner->outstanding(), outstanding);
  };

  expect_rejected(0, -1, 0);
  EXPECT_FALSE(rig.runner->Step(1, kDt, Inputs("sha256:step")).error);
  EXPECT_EQ(rig.runner->outstanding(), 1u);

  expect_rejected(1, 2 * kDt, 1);
  expect_rejected(0, 2 * kDt, 1);
  expect_rejected(2, kDt, 1);
  expect_rejected(2, kDt - 1, 1);
  expect_rejected(2, -5, 1);

  const StepResult next = rig.runner->Step(2, 2 * kDt, Inputs("sha256:step"));
  EXPECT_FALSE(next.error);
  EXPECT_FALSE(next.submitted_request_id.empty());
  EXPECT_EQ(rig.runner->outstanding(), 2u);
}

std::vector<std::string> RunMockSync() {
  Rig rig = MockRig();
  std::vector<std::string> dump;
  for (int64_t k = 0; k < 4; ++k) {
    dump.push_back(DumpStep(rig.runner->Step(
        k, StepTime(k), Inputs(k == 2 ? "sha256:x" : "sha256:step"))));
  }
  return dump;
}

std::vector<std::string> RunQueueFull() {
  Rig rig = MockRig(/*complete_on_submit=*/false, /*ticks_per_step=*/0);
  std::vector<std::string> dump;
  for (int64_t k = 0; k <= 8; ++k) {
    dump.push_back(DumpStep(rig.runner->Step(k, k, Inputs("sha256:step"))));
  }
  dump.push_back(
      DumpStep(rig.runner->Step(9, 2 * kDt + 8, Inputs("sha256:step"))));
  return dump;
}

std::vector<std::string> RunRecorded() {
  std::vector<std::string> dump;
  for (const char* digest :
       {"sha256:policy-late", "sha256:policy-active", "sha256:policy-mismatch",
        "sha256:policy-corrupt"}) {
    Rig rig = RecordedRig();
    dump.push_back(DumpStep(rig.runner->Step(0, StepTime(0), Inputs(digest))));
  }
  return dump;
}

std::vector<std::string> RunNotTerminal() {
  Rig rig = MockRig(/*complete_on_submit=*/false, /*ticks_per_step=*/0);
  std::vector<std::string> dump;
  for (int64_t k = 0; k < 6; ++k) {
    dump.push_back(
        DumpStep(rig.runner->Step(k, StepTime(k), Inputs("sha256:step"))));
  }
  return dump;
}

TEST(ShadowPolicyRunnerTest, S11RepeatIsBitIdentical) {
  EXPECT_EQ(RunMockSync(), RunMockSync());
  EXPECT_EQ(RunQueueFull(), RunQueueFull());
  EXPECT_EQ(RunRecorded(), RunRecorded());
  EXPECT_EQ(RunNotTerminal(), RunNotTerminal());
  EXPECT_FALSE(RunQueueFull().empty());
}

TEST(ShadowPolicyRunnerTest, S12SourceScan) {
  const std::string base =
      "intrinsic/simulation/gazebo/scenarios/uuv_reference/";
  for (const char* name :
       {"shadow_policy_runner.h", "shadow_policy_runner.cc"}) {
    const std::string text = ReadFile(RunfilesPath(base + name));
    ASSERT_FALSE(text.empty()) << name;
    for (const char* banned : {"steady_clock", "system_clock", "absl::Now",
                               "sleep", "std::thread", "SystemUtcNow"}) {
      EXPECT_EQ(text.find(banned), std::string::npos) << name << ": " << banned;
    }
  }
}

}  // namespace
}  // namespace intrinsic::simulation::uuv_reference
