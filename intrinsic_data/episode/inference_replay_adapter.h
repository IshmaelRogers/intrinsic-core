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

#ifndef INTRINSIC_DATA_EPISODE_INFERENCE_REPLAY_ADAPTER_H_
#define INTRINSIC_DATA_EPISODE_INFERENCE_REPLAY_ADAPTER_H_

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "intrinsic/embodiment/stamped_header_policy.h"
#include "intrinsic_data/episode/episode_event_policy.h"
#include "intrinsic_data/episode/episode_replay.h"
#include "intrinsic_data/episode/episode_replay_adapter_common.h"

namespace intrinsic::episode {

// Passive inference replay adapter. Contract: lock id
// episode-replay-adapters-planning-safety-inference-v1 (#132), decisions P3-A,
// P4-A and P6-A. Semantics: intrinsic_data/episode/README.md.
//
// Host-only, sim/test only. Feed() takes each EpisodeReplayer event. The
// adapter evaluates the recorded inference_envelope (kind 3) and
// inference_result (kind 4) payloads directly with the #118 assessors and a
// mirror of the #120 key and expiry predicate. It uses no InferenceClient, no
// ReplayWorker and no queue, and it never modifies a result. The expiry clock
// is the inner header.receive_time of the inference record being processed,
// never a wall clock. No thread, no sleep, no clock, no randomness.

// Append-only. Names follow #120.
enum class InferenceOutcome {
  kNone = 0,
  kComplete = 1,
  kReplayExpired = 2,
  // A result with no pending envelope (orphan).
  kReplayMiss = 3,
  // A pending envelope whose result never arrived.
  kMissing = 4,
};

struct InferenceReplayConfig {
  ReplayCompatibilityConfig compatibility;
};

// A pairing outcome (kComplete, kReplayExpired, kReplayMiss) or a missing
// envelope (kMissing).
struct InferenceOutcomeEntry {
  // The result's writer_index, or the envelope's for kMissing.
  uint64_t writer_index = 0;
  InferenceOutcome outcome = InferenceOutcome::kNone;
  // kMissing only: true when swept because its deadline passed, false when
  // still pending at Finish().
  bool expired = false;
  // True when both provenance digests are non-empty and differ.
  bool model_digest_mismatch = false;
  // The paired (or missing) envelope's writer_index. 0 for an orphan.
  uint64_t envelope_writer_index = 0;
  // Trace only, never compared (P5-A).
  std::string world_snapshot_id;
};

struct InferenceReplayStep {
  ReplayAdapterStatus status = ReplayAdapterStatus::kOk;
  // 0 for kFinished.
  uint64_t writer_index = 0;
  // kInferenceEnvelope or kInferenceResult for an owned, parsed, compatible
  // record. kNone otherwise.
  PayloadKind payload_kind = PayloadKind::kNone;
  // inference::InferenceEnvelopeContractError as an int.
  int error = 0;
  bool accepted = false;
  // True only for an accepted result whose inner receive_time is absent or has
  // nanos out of range. Such a result still pairs with the oldest equal-key
  // pending envelope as kComplete, but its deadline is not judged.
  bool receive_time_unusable = false;
  // In order: the kMissing entries swept by this record, then, for an accepted
  // result, its pairing entry.
  std::vector<InferenceOutcomeEntry> outcomes;
};

// One per owned, parsed, compatible record, in feed order.
struct InferenceTraceEntry {
  uint64_t writer_index = 0;
  PayloadKind payload_kind = PayloadKind::kNone;
  int error = 0;
  bool accepted = false;
};

struct InferenceReplayStats {
  uint64_t fed = 0;
  uint64_t envelopes = 0;
  uint64_t envelopes_accepted = 0;
  uint64_t results = 0;
  uint64_t results_accepted = 0;
  // Judged completions only. A pair with an unusable receive_time is counted in
  // unusable_receive_time, not here.
  uint64_t complete = 0;
  uint64_t expired = 0;
  uint64_t orphans = 0;
  // Every accepted result with an unusable inner receive_time, paired or
  // orphaned.
  uint64_t unusable_receive_time = 0;
  uint64_t missing_expired = 0;
  uint64_t missing_unresolved = 0;
  uint64_t model_digest_mismatches = 0;
  uint64_t ignored = 0;
  uint64_t invalid_records = 0;
  uint64_t gaps = 0;
  uint64_t gap_dropped = 0;
  uint64_t after_latch = 0;
};

struct InferenceReplaySummary {
  // kOk or the latched status.
  ReplayAdapterStatus status = ReplayAdapterStatus::kOk;
  // Set by a loss record, an unparsable record, a missing envelope, a model
  // digest mismatch or an accepted result with an unusable inner receive_time.
  bool degraded = false;
  // Lowercase hex SHA-256 over "v1\n" and "<payload_sha256>\n" for each
  // kComplete and kReplayExpired result whose deadline was judged, in feed
  // order.
  std::string inference_chain;
  std::vector<InferenceTraceEntry> trace;
  // Every pairing and missing entry in order of occurrence, including the
  // kMissing entries Finish() reports for envelopes still pending.
  std::vector<InferenceOutcomeEntry> outcomes;
  InferenceReplayStats stats;
};

class InferenceReplayAdapter {
 public:
  // An invalid compatibility config gives nullptr and kInvalidConfig.
  static std::unique_ptr<InferenceReplayAdapter> Create(
      const InferenceReplayConfig& config, ReplayAdapterStatus* status);

  InferenceReplayAdapter(const InferenceReplayAdapter&) = delete;
  InferenceReplayAdapter& operator=(const InferenceReplayAdapter&) = delete;

  InferenceReplayStep Feed(const EpisodeReplayEvent& event);

  // Reports every envelope still pending as kMissing with expired = false.
  // Idempotent: a second call returns the same summary.
  InferenceReplaySummary Finish();

 private:
  struct Pending {
    std::string model_id;
    std::string input_digest;
    uint64_t state_epoch = 0;
    embodiment::ClockReading deadline;
    std::string provenance_digest;
    uint64_t writer_index = 0;
    std::string world_snapshot_id;
  };

  explicit InferenceReplayAdapter(InferenceReplayConfig config);

  // Removes, in pending order, every entry whose deadline is strictly before
  // `now` and which does not match the exclusion key, reporting each as
  // kMissing with expired = true.
  void SweepMissing(embodiment::ClockReading now, const Pending* exclude_key,
                    InferenceReplayStep* step);

  InferenceReplayConfig config_;
  bool latched_ = false;
  ReplayAdapterStatus latched_status_ = ReplayAdapterStatus::kOk;
  bool finished_ = false;
  bool degraded_ = false;
  std::vector<Pending> pending_;
  std::string chain_input_;
  std::vector<InferenceTraceEntry> trace_;
  std::vector<InferenceOutcomeEntry> outcomes_;
  InferenceReplayStats stats_;
  InferenceReplaySummary summary_;
};

}  // namespace intrinsic::episode

#endif  // INTRINSIC_DATA_EPISODE_INFERENCE_REPLAY_ADAPTER_H_
