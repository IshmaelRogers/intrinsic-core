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

#ifndef INTRINSIC_INFERENCE_QUEUE_POLICY_WORKER_H_
#define INTRINSIC_INFERENCE_QUEUE_POLICY_WORKER_H_

#include <cstddef>
#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <string_view>
#include <vector>

#include "intrinsic/embodiment/stamped_header_policy.h"
#include "intrinsic/vehicle/proto/vehicle_command.pb.h"
#include "intrinsic_inference/queue/replay_worker.h"
#include "intrinsic_inference/queue/stub_worker.h"

namespace intrinsic::inference {

// Policy workers (L44-1). Contract: issue #264, LOCK l44-1-policy-workers-v1.
//
// A policy worker returns an InferenceResult whose output_digest is the
// intent digest of a DesiredMotion. The motion itself travels out of band in a
// DesiredMotionStore keyed by that digest, so neither the queue nor any proto
// changes. Test seams only: no clock, sleep, thread, socket, OIP, or Triton.

// Insert-only side store of DesiredMotion, keyed by
// intrinsic::safety::ComputeOriginalIntentDigest (bare lowercase 64-hex
// SHA-256, no "sha256:" prefix). Thread safe. Entries are never erased, so a
// pointer returned by Find stays valid for the lifetime of the store.
class DesiredMotionStore {
public:
  // Returns ComputeOriginalIntentDigest(motion). Idempotent: a re-Put of an
  // equal motion keeps the first copy.
  std::string Put(const intrinsic_proto::vehicle::DesiredMotion &motion);

  // nullptr when absent.
  const intrinsic_proto::vehicle::DesiredMotion *
  Find(std::string_view output_digest) const;

  size_t size() const;

private:
  mutable std::mutex mutex_;
  std::map<std::string, intrinsic_proto::vehicle::DesiredMotion, std::less<>>
      motions_;
};

// Scripted worker. The script maps an envelope common.input_digest to the
// motion to return. Every scripted motion is Put into the store in the
// constructor; Complete only reads, so it is const, deterministic and does no
// write under the queue lock.
//
// Hit: kComplete with the StubWorker echo of the request, output_digest
// replaced by the motion digest. Miss: kReplayMiss with no result (the enum
// value is reused; there is no mock-specific status).
class MockPolicyWorker final : public InferenceWorker {
public:
  MockPolicyWorker(
      std::shared_ptr<DesiredMotionStore> store,
      std::map<std::string, intrinsic_proto::vehicle::DesiredMotion,
               std::less<>>
          script);

  WorkerOutcome Complete(const OwnedInferenceEnvelope &envelope,
                         std::string_view queue_request_id,
                         embodiment::ClockReading now) const override;

private:
  std::map<std::string, std::string, std::less<>> digest_by_input_digest_;
};

// Recorded worker. Owns a ReplayWorker over fixture_dir and delegates
// Complete to it unchanged. The constructor also loads every
// "*.motion.textproto" directly inside fixture_dir, in sorted filename order,
// into the store. There is no load-time pairing with the JSON fixtures: a
// result whose output_digest has no stored motion is caught at
// DesiredMotionStore::Find. File reads happen only in the constructor.
class RecordedPolicyWorker final : public InferenceWorker {
public:
  RecordedPolicyWorker(std::shared_ptr<DesiredMotionStore> store,
                       const std::string &fixture_dir);

  WorkerOutcome Complete(const OwnedInferenceEnvelope &envelope,
                         std::string_view queue_request_id,
                         embodiment::ClockReading now) const override;

  // ReplayWorker's fixture keys and unkeyable JSON file names.
  size_t fixture_count() const { return replay_.fixture_count(); }
  const std::vector<std::string> &skipped_files() const {
    return replay_.skipped_files();
  }

  // Number of motion textprotos loaded into the store, and the sorted names of
  // the ones that could not be parsed.
  size_t loaded_motion_count() const { return loaded_motion_count_; }
  const std::vector<std::string> &skipped_motion_files() const {
    return skipped_motion_files_;
  }

private:
  ReplayWorker replay_;
  size_t loaded_motion_count_ = 0;
  std::vector<std::string> skipped_motion_files_;
};

} // namespace intrinsic::inference

#endif // INTRINSIC_INFERENCE_QUEUE_POLICY_WORKER_H_
