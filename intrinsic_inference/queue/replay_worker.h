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

#ifndef INTRINSIC_INFERENCE_QUEUE_REPLAY_WORKER_H_
#define INTRINSIC_INFERENCE_QUEUE_REPLAY_WORKER_H_

#include <cstddef>
#include <cstdint>
#include <map>
#include <optional>
#include <string>
#include <string_view>
#include <tuple>
#include <vector>

#include "intrinsic/embodiment/stamped_header_policy.h"
#include "intrinsic_inference/queue/stub_worker.h"

namespace intrinsic::inference {

// Deterministic replay worker. Contract: issue #120, comment 6009314342.
//
// Returns a recorded InferenceResult for a request instead of running a model.
// It does no OIP or Triton call, no model load, no socket, no GPU, no sleep,
// and never reads a clock itself: expiry uses the queue clock passed to
// Complete. Fixtures are read once, in the constructor. Complete never touches
// the disk.
//
// Match key, exact equality on all three request envelope fields:
//   common.provenance_model_id, common.input_digest, common.state_epoch.
// Nothing else selects a fixture. A key with no fixture is kReplayMiss.
//
// Fixture format and layout: see README.md ("Replay worker"). One JSON file
// per recorded result, directly in the fixture directory, no subdirectories.
//
// Immutable after construction, so safe to share between threads.
class ReplayWorker : public InferenceWorker {
 public:
  // Loads every "*.json" file directly inside fixture_dir, in sorted filename
  // order. A missing or unreadable directory loads nothing and every request
  // is kReplayMiss.
  explicit ReplayWorker(const std::string& fixture_dir);

  // Fixture keys held, usable or corrupt.
  size_t fixture_count() const { return rows_.size(); }

  // File names (not paths) that could not be keyed because they were
  // unparseable or lacked a complete match key. Requests for such a key are
  // kReplayMiss. In sorted order.
  const std::vector<std::string>& skipped_files() const {
    return skipped_files_;
  }

  // Hit: kComplete with the recorded shell, or kReplayExpired with the same
  // recorded shell when the recorded deadline is strictly before now. A key
  // whose fixture is unusable gives kCorruptFixture. Anything else is
  // kReplayMiss. The recorded shell is returned as written; nothing is
  // rewritten, including the queue request id.
  WorkerOutcome Complete(const OwnedInferenceEnvelope& envelope,
                         std::string_view queue_request_id,
                         embodiment::ClockReading now) const override;

 private:
  using Key = std::tuple<std::string, std::string, uint64_t>;

  struct Row {
    // Empty when the fixture is corrupt.
    std::optional<InferenceResultShell> shell;
    // Recorded deadline, valid only when shell has a value.
    embodiment::ClockReading deadline;
  };

  std::map<Key, Row> rows_;
  std::vector<std::string> skipped_files_;
};

}  // namespace intrinsic::inference

#endif  // INTRINSIC_INFERENCE_QUEUE_REPLAY_WORKER_H_
