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

#include "intrinsic_inference/queue/policy_worker.h"

#include <algorithm>
#include <cstddef>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <string_view>
#include <system_error>
#include <utility>
#include <vector>

#include "google/protobuf/text_format.h"
#include "intrinsic/embodiment/stamped_header_policy.h"
#include "intrinsic/safety/safety_decision_assessor.h"
#include "intrinsic/vehicle/proto/vehicle_command.pb.h"
#include "intrinsic_inference/envelope/inference_envelope_contract_policy.h"
#include "intrinsic_inference/queue/replay_worker.h"
#include "intrinsic_inference/queue/stub_worker.h"

namespace intrinsic::inference {
namespace {

namespace fs = std::filesystem;
using intrinsic_proto::vehicle::DesiredMotion;

constexpr std::string_view kMotionSuffix = ".motion.textproto";

bool EndsWith(std::string_view text, std::string_view suffix) {
  return text.size() >= suffix.size() &&
         text.substr(text.size() - suffix.size()) == suffix;
}

} // namespace

std::string DesiredMotionStore::Put(const DesiredMotion &motion) {
  std::string digest = safety::ComputeOriginalIntentDigest(motion);
  std::lock_guard<std::mutex> lock(mutex_);
  motions_.try_emplace(digest, motion);
  return digest;
}

const DesiredMotion *
DesiredMotionStore::Find(std::string_view output_digest) const {
  std::lock_guard<std::mutex> lock(mutex_);
  const auto it = motions_.find(output_digest);
  return it == motions_.end() ? nullptr : &it->second;
}

size_t DesiredMotionStore::size() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return motions_.size();
}

MockPolicyWorker::MockPolicyWorker(
    std::shared_ptr<DesiredMotionStore> store,
    std::map<std::string, DesiredMotion, std::less<>> script) {
  for (const auto &[input_digest, motion] : script) {
    digest_by_input_digest_.emplace(input_digest, store->Put(motion));
  }
}

WorkerOutcome
MockPolicyWorker::Complete(const OwnedInferenceEnvelope &envelope,
                           std::string_view queue_request_id,
                           embodiment::ClockReading /*now*/) const {
  WorkerOutcome outcome;
  const InferenceEnvelopeView request = envelope.View();
  const auto it = digest_by_input_digest_.find(request.common.input_digest);
  if (it == digest_by_input_digest_.end()) {
    outcome.status = WorkerStatus::kReplayMiss;
    return outcome;
  }
  const InferenceResultShell echo =
      StubWorker().Run(envelope, queue_request_id);
  InferenceResultView view = echo.View();
  view.output_digest = it->second;
  outcome.status = WorkerStatus::kComplete;
  outcome.result = InferenceResultShell(view);
  return outcome;
}

RecordedPolicyWorker::RecordedPolicyWorker(
    std::shared_ptr<DesiredMotionStore> store, const std::string &fixture_dir)
    : replay_(fixture_dir) {
  std::error_code ec;
  fs::directory_iterator it(fixture_dir, ec);
  if (ec) {
    return;
  }
  std::vector<fs::path> paths;
  for (const fs::directory_iterator end; it != end; it.increment(ec)) {
    if (ec) {
      break;
    }
    std::error_code file_ec;
    if (it->is_regular_file(file_ec) && !file_ec &&
        EndsWith(it->path().filename().string(), kMotionSuffix)) {
      paths.push_back(it->path());
    }
  }
  std::sort(paths.begin(), paths.end());
  for (const fs::path &path : paths) {
    std::ifstream in(path, std::ios::binary);
    const std::string text((std::istreambuf_iterator<char>(in)),
                           std::istreambuf_iterator<char>());
    DesiredMotion motion;
    if (!in.good() && !in.eof()) {
      skipped_motion_files_.push_back(path.filename().string());
    } else if (!google::protobuf::TextFormat::ParseFromString(text, &motion)) {
      skipped_motion_files_.push_back(path.filename().string());
    } else {
      store->Put(motion);
      ++loaded_motion_count_;
    }
  }
}

WorkerOutcome
RecordedPolicyWorker::Complete(const OwnedInferenceEnvelope &envelope,
                               std::string_view queue_request_id,
                               embodiment::ClockReading now) const {
  return replay_.Complete(envelope, queue_request_id, now);
}

} // namespace intrinsic::inference
