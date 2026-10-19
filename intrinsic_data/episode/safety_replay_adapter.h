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

#ifndef INTRINSIC_DATA_EPISODE_SAFETY_REPLAY_ADAPTER_H_
#define INTRINSIC_DATA_EPISODE_SAFETY_REPLAY_ADAPTER_H_

#include <cstdint>
#include <memory>
#include <set>
#include <string>
#include <vector>

#include "intrinsic_data/episode/episode_event_policy.h"
#include "intrinsic_data/episode/episode_replay.h"
#include "intrinsic_data/episode/episode_replay_adapter_common.h"

namespace intrinsic::episode {

// Passive safety replay adapter. Contract: lock id
// episode-replay-adapters-planning-safety-inference-v1 (#132), decision P2-A.
// Semantics: intrinsic_data/episode/README.md.
//
// Host-only, sim/test only. Feed() takes each EpisodeReplayer event. The
// adapter verifies the recorded safety_decision (kind 1) and desired_motion
// (kind 10) payloads and links each decision to an earlier replayed motion by
// original_intent_digest. It never calls AggregateSafetyRuleResults, never
// runs a safety supervisor, and never rewrites a record. No thread, no sleep,
// no clock, no randomness.

struct SafetyReplayConfig {
  ReplayCompatibilityConfig compatibility;
};

struct SafetyReplayStep {
  ReplayAdapterStatus status = ReplayAdapterStatus::kOk;
  // 0 for kFinished.
  uint64_t writer_index = 0;
  // kSafetyDecision or kDesiredMotion for an owned, parsed, compatible
  // record. kNone otherwise.
  PayloadKind payload_kind = PayloadKind::kNone;
  // safety::SafetyDecisionError (kSafetyDecision) or vehicle::ContractError
  // (kDesiredMotion), as an int.
  int error = 0;
  bool accepted = false;
  // Lowercase hex SHA-256 of the payload's deterministic serialization. Empty
  // when no owned record was parsed.
  std::string payload_sha256;
  // kSafetyDecision only. safety::SafetyDecisionKind as an int.
  int kind = 0;
  std::string original_intent_digest;
  // True when original_intent_digest is the ComputeOriginalIntentDigest of an
  // earlier replayed motion, accepted or not. Linkage proves a reference
  // only.
  bool intent_linked = false;
  // Trace only, never compared (P5-A).
  std::string snapshot_id;
};

// One per owned, parsed, compatible record, in feed order. A motion entry
// leaves kind, original_intent_digest, intent_linked and snapshot_id at their
// defaults.
struct SafetyTraceEntry {
  uint64_t writer_index = 0;
  PayloadKind payload_kind = PayloadKind::kNone;
  int error = 0;
  bool accepted = false;
  int kind = 0;
  std::string original_intent_digest;
  bool intent_linked = false;
  std::string snapshot_id;
  std::string payload_sha256;
};

struct SafetyReplayStats {
  uint64_t fed = 0;
  uint64_t decisions = 0;
  uint64_t decisions_accepted = 0;
  uint64_t unlinked_decisions = 0;
  uint64_t motions = 0;
  uint64_t motions_accepted = 0;
  uint64_t ignored = 0;
  uint64_t invalid_records = 0;
  uint64_t gaps = 0;
  uint64_t gap_dropped = 0;
  uint64_t after_latch = 0;
};

struct SafetyReplaySummary {
  // kOk or the latched status.
  ReplayAdapterStatus status = ReplayAdapterStatus::kOk;
  // Set by a loss record, an unparsable record, or an unlinked decision.
  bool degraded = false;
  // Lowercase hex SHA-256 over "v1\n" and "<payload_sha256>\n" for each
  // accepted decision, linked or not, in feed order.
  std::string decision_chain;
  std::vector<SafetyTraceEntry> trace;
  SafetyReplayStats stats;
};

class SafetyReplayAdapter {
 public:
  // An invalid compatibility config gives nullptr and kInvalidConfig.
  static std::unique_ptr<SafetyReplayAdapter> Create(
      const SafetyReplayConfig& config, ReplayAdapterStatus* status);

  SafetyReplayAdapter(const SafetyReplayAdapter&) = delete;
  SafetyReplayAdapter& operator=(const SafetyReplayAdapter&) = delete;

  SafetyReplayStep Feed(const EpisodeReplayEvent& event);

  // Idempotent: a second call returns the same summary.
  SafetyReplaySummary Finish();

 private:
  explicit SafetyReplayAdapter(SafetyReplayConfig config);

  SafetyReplayConfig config_;
  bool latched_ = false;
  ReplayAdapterStatus latched_status_ = ReplayAdapterStatus::kOk;
  bool finished_ = false;
  bool degraded_ = false;
  // ComputeOriginalIntentDigest of every replayed motion so far.
  std::set<std::string> seen_intents_;
  std::string decision_chain_input_;
  std::vector<SafetyTraceEntry> trace_;
  SafetyReplayStats stats_;
  SafetyReplaySummary summary_;
};

}  // namespace intrinsic::episode

#endif  // INTRINSIC_DATA_EPISODE_SAFETY_REPLAY_ADAPTER_H_
