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

// RB2, RB4 to RB7 and RB12 to RB14 of lock id
// episode-replay-adapters-planning-safety-inference-v1 (#132), safety side.
// Episodes are built in memory with EpisodeWriter, read with EpisodeReader and
// replayed with EpisodeReplayer in kStep. No fixture file is added. The
// #127/#244 goldens episode_event_v1_{safety_decision,desired_motion} are read
// unmodified. No safety supervisor and no rule aggregation is linked, so none
// can be called.

#include "intrinsic_data/episode/safety_replay_adapter.h"

#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <fstream>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include "absl/status/status.h"
#include "absl/types/span.h"
#include "gtest/gtest.h"
#include "intrinsic/episode/proto/episode_event.pb.h"
#include "intrinsic/safety/safety_decision_assessor.h"
#include "intrinsic/safety/safety_decision_policy.h"
#include "intrinsic/vehicle/vehicle_contract_policy.h"
#include "intrinsic_data/episode/episode_event_policy.h"
#include "intrinsic_data/episode/episode_reader.h"
#include "intrinsic_data/episode/episode_replay.h"
#include "intrinsic_data/episode/episode_replay_adapter_common.h"
#include "intrinsic_data/episode/episode_writer.h"

namespace intrinsic::episode {
namespace {

using intrinsic_proto::episode::EpisodeEvent;

constexpr char kSeed42Digest[] =
    "93150ec8e5fd8f6dcc2d9cae50d84de9b89287b187edfa4a807b0da0e8eb5d6a";
constexpr char kOtherDigest[] =
    "bbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbb";
// SHA-256 of "v1\n".
constexpr char kEmptyChain[] =
    "2d27fbdf4e8ca207afbfa388ca9172fbcc6c70e534af2476b3b704f87debadcf";

class MemorySink : public EpisodeSink {
 public:
  absl::Status Write(absl::Span<const uint8_t> bytes) override {
    output.insert(output.end(), bytes.begin(), bytes.end());
    return absl::OkStatus();
  }
  absl::Status Sync() override { return absl::OkStatus(); }

  std::vector<uint8_t> output;
};

std::string ReadRunfile(const std::string& relative) {
  std::vector<std::string> candidates;
  if (const char* src = std::getenv("TEST_SRCDIR")) {
    if (const char* workspace = std::getenv("TEST_WORKSPACE")) {
      candidates.push_back(std::string(src) + "/" + workspace + "/" + relative);
    }
    candidates.push_back(std::string(src) + "/_main/" + relative);
  }
  for (const std::string& path : candidates) {
    std::ifstream in(path, std::ios::binary);
    if (!in) {
      continue;
    }
    std::ostringstream buffer;
    buffer << in.rdbuf();
    return buffer.str();
  }
  ADD_FAILURE() << "missing runfile " << relative;
  return "";
}

EpisodeEvent ParseGolden(const std::string& name) {
  EpisodeEvent event;
  EXPECT_TRUE(event.ParseFromString(ReadRunfile(
      "intrinsic_data/episode/testdata/episode_event_v1_" + name + ".binpb")))
      << name;
  return event;
}

// Envelope identity copied from the v1 metadata golden. Only the sequence and
// the times (sequence-derived, like the goldens) vary.
EpisodeEvent BaseEvent(uint64_t sequence) {
  EpisodeEvent event;
  auto* header = event.mutable_header();
  header->set_sequence(sequence);
  header->mutable_source_time()->set_seconds(1700000100 + sequence);
  header->mutable_source_time()->set_nanos(250000000);
  header->mutable_receive_time()->set_seconds(1700000100 + sequence);
  header->mutable_receive_time()->set_nanos(251000000);
  header->set_source_id("episode_recorder");
  header->set_frame_id("world_enu");
  header->set_clock_domain("monotonic");
  header->mutable_validity()->set_state(
      intrinsic_proto::embodiment::Validity::STATE_VALID);
  event.set_schema_version(1);
  event.set_config_digest(kSeed42Digest);
  event.mutable_provenance()->set_model_id("marine_parameter_sampler");
  event.mutable_provenance()->set_model_version("v1");
  event.mutable_provenance()->set_digest(std::string("sha256:") +
                                         kSeed42Digest);
  return event;
}

std::string Seed42MarineText() {
  const EpisodeEvent golden = ParseGolden("metadata");
  return golden.episode_metadata().marine_randomization_metadata();
}

EpisodeEvent MetadataEvent(uint64_t sequence, const std::string& text) {
  EpisodeEvent event = BaseEvent(sequence);
  event.mutable_episode_metadata()->set_marine_randomization_metadata(text);
  event.set_config_digest(ComputeConfigDigest(text));
  return event;
}

EpisodeEvent RecordedEvent(uint64_t sequence, const std::string& type,
                           const std::string& bytes) {
  EpisodeEvent event = BaseEvent(sequence);
  event.mutable_recorded_message()->set_message_type(type);
  event.mutable_recorded_message()->set_message_bytes(bytes);
  return event;
}

EpisodeEvent WorldRecordEvent(uint64_t sequence) {
  EpisodeEvent event = BaseEvent(sequence);
  event.mutable_world_snapshot_record();
  return event;
}

EpisodeEvent SafetyEvent(uint64_t sequence) {
  EpisodeEvent event = BaseEvent(sequence);
  event.mutable_safety_decision();
  return event;
}

EpisodeEvent TrajectoryEvent(uint64_t sequence) {
  EpisodeEvent event = BaseEvent(sequence);
  event.mutable_vehicle_trajectory();
  return event;
}

EpisodeEvent InferenceEnvelopeEvent(uint64_t sequence) {
  EpisodeEvent event = BaseEvent(sequence);
  event.mutable_inference_envelope();
  return event;
}

EpisodeEvent InferenceResultEvent(uint64_t sequence) {
  EpisodeEvent event = BaseEvent(sequence);
  event.mutable_inference_result();
  return event;
}

EpisodeEvent DesiredMotionEvent(uint64_t sequence) {
  EpisodeEvent event = BaseEvent(sequence);
  event.mutable_desired_motion();
  return event;
}

std::vector<uint8_t> WriteEpisode(const std::vector<EpisodeEvent>& events) {
  MemorySink sink;
  EpisodeWriterStatus status = EpisodeWriterStatus::kInvalidConfig;
  std::unique_ptr<EpisodeWriter> writer =
      EpisodeWriter::Create(EpisodeWriterConfig{}, &sink, &status);
  EXPECT_EQ(status, EpisodeWriterStatus::kOk);
  if (writer == nullptr) {
    return {};
  }
  for (const EpisodeEvent& event : events) {
    const EpisodeAppendResult result = writer->Append(event);
    EXPECT_EQ(result.status, EpisodeWriterStatus::kOk)
        << "sequence " << event.header().sequence() << " error "
        << static_cast<int>(result.event_error);
  }
  EXPECT_EQ(writer->Close(), EpisodeWriterStatus::kOk);
  return sink.output;
}

// capacity 4 holds 3 events. `first` are queued, `dropped` overflow and
// become one loss record, then `after` is appended behind the loss record.
std::vector<uint8_t> WriteOverflowEpisode(
    const std::vector<EpisodeEvent>& first,
    const std::vector<EpisodeEvent>& dropped, const EpisodeEvent& after) {
  MemorySink sink;
  EpisodeWriterConfig config;
  config.capacity = 4;
  EpisodeWriterStatus status = EpisodeWriterStatus::kInvalidConfig;
  std::unique_ptr<EpisodeWriter> writer =
      EpisodeWriter::Create(config, &sink, &status);
  EXPECT_EQ(status, EpisodeWriterStatus::kOk);
  if (writer == nullptr) {
    return {};
  }
  for (const EpisodeEvent& event : first) {
    EXPECT_EQ(writer->Append(event).status, EpisodeWriterStatus::kOk);
  }
  for (const EpisodeEvent& event : dropped) {
    EXPECT_EQ(writer->Append(event).status, EpisodeWriterStatus::kBufferFull);
  }
  EXPECT_EQ(writer->Flush(), EpisodeWriterStatus::kOk);
  EXPECT_EQ(writer->Append(after).status, EpisodeWriterStatus::kOk);
  EXPECT_EQ(writer->Close(), EpisodeWriterStatus::kOk);
  EXPECT_EQ(writer->Stats().loss_records, 1u);
  return sink.output;
}

// Reads the bytes and replays them in kStep. The bytes must outlive it.
class EpisodeStream {
 public:
  explicit EpisodeStream(const std::vector<uint8_t>& bytes) {
    EpisodeReplayConfig config;
    config.mode = EpisodeReplayMode::kStep;
    EpisodeReplayStatus status = EpisodeReplayStatus::kInvalidConfig;
    replayer_ = EpisodeReplayer::Create(bytes, EpisodeReaderConfig{},
                                        EpisodeReadFilter{}, config, &status);
    EXPECT_EQ(status, EpisodeReplayStatus::kOk);
  }

  bool Next(EpisodeReplayEvent* event) {
    if (replayer_ == nullptr) {
      return false;
    }
    const EpisodeReplayResult result = replayer_->Step();
    if (result.status != EpisodeReplayStatus::kOk) {
      return false;
    }
    *event = result.event;
    return true;
  }

 private:
  std::unique_ptr<EpisodeReplayer> replayer_;
};

// The deps block of the cc_library `target` in intrinsic_data/episode/BUILD.
std::string BuildDepsOf(const std::string& target) {
  const std::string build = ReadRunfile("intrinsic_data/episode/BUILD");
  const std::string anchor = "name = \"" + target + "\",";
  const size_t start = build.find(anchor);
  EXPECT_NE(start, std::string::npos) << target;
  if (start == std::string::npos) {
    return "";
  }
  const size_t deps = build.find("deps = [", start);
  const size_t end = build.find("],", deps);
  EXPECT_NE(deps, std::string::npos);
  EXPECT_NE(end, std::string::npos);
  if (deps == std::string::npos || end == std::string::npos) {
    return "";
  }
  return build.substr(deps, end - deps);
}

void ExpectNoForbiddenDeps(const std::string& deps) {
  for (const char* forbidden : {"queue", "motion_planning", "rule_aggregation",
                                "planner", "worker", "inference_client"}) {
    EXPECT_EQ(deps.find(forbidden), std::string::npos) << forbidden;
  }
}

struct BadEvent {
  std::string name;
  EpisodeEvent event;
  ReplayAdapterStatus expected;
};

// Compatibility failures built on `owned`, an event of a kind the adapter
// owns, and on an ignored kind.
std::vector<BadEvent> BadCompatibilityEvents(const EpisodeEvent& owned) {
  std::vector<BadEvent> bad;
  const std::string other_text = "marine_randomization_metadata_v1\nseed=43\n";

  EpisodeEvent event = owned;
  event.set_schema_version(2);
  bad.push_back(
      {"owned schema 2", event, ReplayAdapterStatus::kIncompatibleSchema});
  event = RecordedEvent(owned.header().sequence(), "x.Y", "");
  event.set_schema_version(2);
  bad.push_back({"ignored kind schema 2", event,
                 ReplayAdapterStatus::kIncompatibleSchema});
  event = owned;
  event.set_schema_version(2);
  event.set_config_digest(kOtherDigest);
  bad.push_back({"schema is checked before the digest", event,
                 ReplayAdapterStatus::kIncompatibleSchema});
  event = owned;
  event.set_config_digest(kOtherDigest);
  bad.push_back(
      {"owned digest", event, ReplayAdapterStatus::kConfigDigestMismatch});
  event = RecordedEvent(owned.header().sequence(), "x.Y", "");
  event.set_config_digest(kOtherDigest);
  bad.push_back({"ignored kind digest", event,
                 ReplayAdapterStatus::kConfigDigestMismatch});
  bad.push_back({"metadata with its own passing digest",
                 MetadataEvent(owned.header().sequence(), other_text),
                 ReplayAdapterStatus::kConfigDigestMismatch});
  return bad;
}

// Payload SHA-256 of the deterministic serialization, computed independently
// (see the PR description) and committed as constants.
constexpr char kShaGoldenDecision[] =
    "679110de1d1f1b72945bd1c3611308d1c29ca19b09cd7326a5fa5a88cc4b4aa1";
constexpr char kShaGoldenMotion[] =
    "4e7f198e7cdf5fdb8b26ca6e6ad4eb5620ce6da734b0bd2dca65965c04edd9c5";
constexpr char kShaLinkedDecision[] =
    "e679383af175cbd1614a5731971387af25f9d0c5c55556edf7d80d6a46c5f17b";
constexpr char kDecisionChainRb2[] =
    "47c71d06edd8dd4441252f9e05064de944ae7aa13022d4b75eb9c231eb1dd35e";
constexpr char kDecisionChainRb7[] =
    "c454566c40e66be49232d5887370ccab13589f31258ca20886918f6165679b7d";
constexpr char kGoldenIntentDigest[] =
    "5c12e76d610e5cf3f5b0d6c402bcfef05db2ec4deb5faacd03cfecdd2f3e34ae";
constexpr char kGoldenSnapshotId[] =
    "6ceb1af0dca7d2b01d4e876fe699fb8c5d5709d5a4938ea3f46e8d355d3b33de";
// ComputeOriginalIntentDigest of the golden desired_motion.
constexpr char kGoldenMotionIntentDigest[] =
    "4e7f198e7cdf5fdb8b26ca6e6ad4eb5620ce6da734b0bd2dca65965c04edd9c5";

SafetyReplayConfig DefaultConfig() {
  SafetyReplayConfig config;
  config.compatibility.expected_config_digest = kSeed42Digest;
  return config;
}

std::unique_ptr<SafetyReplayAdapter> MakeAdapter(
    const SafetyReplayConfig& config = DefaultConfig()) {
  ReplayAdapterStatus status = ReplayAdapterStatus::kInvalidConfig;
  std::unique_ptr<SafetyReplayAdapter> adapter =
      SafetyReplayAdapter::Create(config, &status);
  EXPECT_EQ(status, ReplayAdapterStatus::kOk);
  EXPECT_NE(adapter, nullptr);
  return adapter;
}

std::vector<SafetyReplayStep> FeedAll(SafetyReplayAdapter* adapter,
                                      const std::vector<uint8_t>& bytes) {
  EpisodeStream stream(bytes);
  std::vector<SafetyReplayStep> steps;
  EpisodeReplayEvent event;
  while (stream.Next(&event)) {
    steps.push_back(adapter->Feed(event));
  }
  return steps;
}

// The golden decision (sequence 1) with another original_intent_digest and
// decision_epoch. Valid.
EpisodeEvent DecisionAt(uint64_t sequence, const std::string& digest) {
  EpisodeEvent event = ParseGolden("safety_decision");
  event.mutable_header()->set_sequence(sequence);
  event.mutable_safety_decision()->set_original_intent_digest(digest);
  event.mutable_safety_decision()->set_decision_epoch(sequence);
  return event;
}

// The golden motion (sequence 8) with a longer horizon. Valid.
EpisodeEvent BuiltMotion(uint64_t sequence) {
  EpisodeEvent event = ParseGolden("desired_motion");
  event.mutable_header()->set_sequence(sequence);
  event.mutable_desired_motion()->mutable_horizon()->set_seconds(3);
  return event;
}

// No objective: rejected by the vehicle policy.
EpisodeEvent InvalidMotion(uint64_t sequence) {
  EpisodeEvent event = BuiltMotion(sequence);
  event.mutable_desired_motion()->clear_twist();
  return event;
}

// An ACCEPT decision without a header: rejected by the safety policy.
EpisodeEvent InvalidDecision(uint64_t sequence) {
  EpisodeEvent event = DecisionAt(sequence, kGoldenIntentDigest);
  event.mutable_safety_decision()->clear_header();
  return event;
}

std::string IntentDigestOf(const EpisodeEvent& motion) {
  return safety::ComputeOriginalIntentDigest(motion.desired_motion());
}

std::string Chain(const std::vector<std::string>& shas) {
  std::string input = "v1\n";
  for (const std::string& sha : shas) {
    input += sha + "\n";
  }
  return internal::EpisodeSha256Hex(input);
}

struct Row {
  uint64_t writer_index;
  PayloadKind kind;
  int error;
  bool accepted;
  int decision_kind;
  std::string digest;
  bool linked;
  std::string snapshot_id;
  std::string sha;
};

void ExpectTrace(const std::vector<SafetyTraceEntry>& trace,
                 const std::vector<Row>& expected) {
  ASSERT_EQ(trace.size(), expected.size());
  for (size_t i = 0; i < expected.size(); ++i) {
    EXPECT_EQ(trace[i].writer_index, expected[i].writer_index) << i;
    EXPECT_EQ(trace[i].payload_kind, expected[i].kind) << i;
    EXPECT_EQ(trace[i].error, expected[i].error) << i;
    EXPECT_EQ(trace[i].accepted, expected[i].accepted) << i;
    EXPECT_EQ(trace[i].kind, expected[i].decision_kind) << i;
    EXPECT_EQ(trace[i].original_intent_digest, expected[i].digest) << i;
    EXPECT_EQ(trace[i].intent_linked, expected[i].linked) << i;
    EXPECT_EQ(trace[i].snapshot_id, expected[i].snapshot_id) << i;
    EXPECT_EQ(trace[i].payload_sha256, expected[i].sha) << i;
  }
}

bool StatsEqual(const SafetyReplayStats& a, const SafetyReplayStats& b) {
  return a.fed == b.fed && a.decisions == b.decisions &&
         a.decisions_accepted == b.decisions_accepted &&
         a.unlinked_decisions == b.unlinked_decisions &&
         a.motions == b.motions && a.motions_accepted == b.motions_accepted &&
         a.ignored == b.ignored && a.invalid_records == b.invalid_records &&
         a.gaps == b.gaps && a.gap_dropped == b.gap_dropped &&
         a.after_latch == b.after_latch;
}

void ExpectSummaryEqual(const SafetyReplaySummary& a,
                        const SafetyReplaySummary& b) {
  EXPECT_EQ(a.status, b.status);
  EXPECT_EQ(a.degraded, b.degraded);
  EXPECT_EQ(a.decision_chain, b.decision_chain);
  EXPECT_TRUE(StatsEqual(a.stats, b.stats));
  ExpectTrace(a.trace, [&b] {
    std::vector<Row> rows;
    for (const SafetyTraceEntry& entry : b.trace) {
      rows.push_back({entry.writer_index, entry.payload_kind, entry.error,
                      entry.accepted, entry.kind, entry.original_intent_digest,
                      entry.intent_linked, entry.snapshot_id,
                      entry.payload_sha256});
    }
    return rows;
  }());
}

constexpr int kNoError = 0;
constexpr int kAccept = static_cast<int>(safety::SafetyDecisionKind::kAccept);
const int kObjectiveError =
    static_cast<int>(vehicle::ContractError::kObjective);
const int kMissingHeaderError =
    static_cast<int>(safety::SafetyDecisionError::kMissingHeader);

TEST(SafetyReplayAdapterTest, RB2NominalLinkedAndUnlinked) {
  const EpisodeEvent golden_motion = ParseGolden("desired_motion");
  ASSERT_EQ(IntentDigestOf(golden_motion), kGoldenMotionIntentDigest);
  const std::vector<uint8_t> bytes =
      WriteEpisode({ParseGolden("safety_decision"), golden_motion,
                    DecisionAt(9, kGoldenMotionIntentDigest)});
  std::unique_ptr<SafetyReplayAdapter> adapter = MakeAdapter();
  ASSERT_NE(adapter, nullptr);
  const std::vector<SafetyReplayStep> steps = FeedAll(adapter.get(), bytes);
  ASSERT_EQ(steps.size(), 3u);
  for (const SafetyReplayStep& step : steps) {
    EXPECT_EQ(step.status, ReplayAdapterStatus::kOk);
    EXPECT_TRUE(step.accepted);
    EXPECT_EQ(step.error, kNoError);
  }
  // The golden decision (sequence 1) is recorded before any motion.
  EXPECT_FALSE(steps[0].intent_linked);
  EXPECT_TRUE(steps[2].intent_linked);

  const SafetyReplaySummary summary = adapter->Finish();
  EXPECT_EQ(summary.status, ReplayAdapterStatus::kOk);
  EXPECT_TRUE(summary.degraded);
  ExpectTrace(summary.trace, {{0, PayloadKind::kSafetyDecision, kNoError, true,
                               kAccept, kGoldenIntentDigest, false,
                               kGoldenSnapshotId, kShaGoldenDecision},
                              {1, PayloadKind::kDesiredMotion, kNoError, true,
                               0, "", false, "", kShaGoldenMotion},
                              {2, PayloadKind::kSafetyDecision, kNoError, true,
                               kAccept, kGoldenMotionIntentDigest, true,
                               kGoldenSnapshotId, kShaLinkedDecision}});
  EXPECT_EQ(summary.decision_chain, kDecisionChainRb2);
  EXPECT_EQ(summary.decision_chain,
            Chain({kShaGoldenDecision, kShaLinkedDecision}));
  EXPECT_EQ(summary.stats.fed, 3u);
  EXPECT_EQ(summary.stats.decisions, 2u);
  EXPECT_EQ(summary.stats.decisions_accepted, 2u);
  EXPECT_EQ(summary.stats.unlinked_decisions, 1u);
  EXPECT_EQ(summary.stats.motions, 1u);
  EXPECT_EQ(summary.stats.motions_accepted, 1u);
}

TEST(SafetyReplayAdapterTest, RB4DeterministicAndTheMotionDigestMatches) {
  const std::vector<uint8_t> bytes = WriteEpisode(
      {ParseGolden("safety_decision"), ParseGolden("desired_motion"),
       DecisionAt(9, kGoldenMotionIntentDigest), InvalidMotion(10)});
  std::unique_ptr<SafetyReplayAdapter> first = MakeAdapter();
  std::unique_ptr<SafetyReplayAdapter> second = MakeAdapter();
  ASSERT_NE(first, nullptr);
  ASSERT_NE(second, nullptr);
  const std::vector<SafetyReplayStep> a = FeedAll(first.get(), bytes);
  const std::vector<SafetyReplayStep> b = FeedAll(second.get(), bytes);
  ASSERT_EQ(a.size(), b.size());
  for (size_t i = 0; i < a.size(); ++i) {
    EXPECT_EQ(a[i].status, b[i].status) << i;
    EXPECT_EQ(a[i].writer_index, b[i].writer_index) << i;
    EXPECT_EQ(a[i].payload_kind, b[i].payload_kind) << i;
    EXPECT_EQ(a[i].error, b[i].error) << i;
    EXPECT_EQ(a[i].accepted, b[i].accepted) << i;
    EXPECT_EQ(a[i].payload_sha256, b[i].payload_sha256) << i;
    EXPECT_EQ(a[i].kind, b[i].kind) << i;
    EXPECT_EQ(a[i].original_intent_digest, b[i].original_intent_digest) << i;
    EXPECT_EQ(a[i].intent_linked, b[i].intent_linked) << i;
    EXPECT_EQ(a[i].snapshot_id, b[i].snapshot_id) << i;
  }
  ExpectSummaryEqual(first->Finish(), second->Finish());

  EXPECT_EQ(a[1].payload_sha256, IntentDigestOf(ParseGolden("desired_motion")));
  EXPECT_EQ(a[3].payload_sha256, IntentDigestOf(InvalidMotion(10)));
}

TEST(SafetyReplayAdapterTest, RB5CompatibilityLatches) {
  for (const BadEvent& bad :
       BadCompatibilityEvents(DecisionAt(2, kGoldenIntentDigest))) {
    SCOPED_TRACE(bad.name);
    EpisodeEvent first = ParseGolden("safety_decision");
    const std::vector<uint8_t> bytes =
        WriteEpisode({first, bad.event, BuiltMotion(3)});
    std::unique_ptr<SafetyReplayAdapter> adapter = MakeAdapter();
    ASSERT_NE(adapter, nullptr);
    const std::vector<SafetyReplayStep> steps = FeedAll(adapter.get(), bytes);
    ASSERT_EQ(steps.size(), 3u);
    EXPECT_EQ(steps[0].status, ReplayAdapterStatus::kOk);
    EXPECT_EQ(steps[1].status, bad.expected);
    EXPECT_EQ(steps[1].payload_kind, PayloadKind::kNone);
    EXPECT_TRUE(steps[1].payload_sha256.empty());
    EXPECT_EQ(steps[2].status, ReplayAdapterStatus::kLatched);
    EXPECT_EQ(steps[2].payload_kind, PayloadKind::kNone);
    const SafetyReplaySummary summary = adapter->Finish();
    EXPECT_EQ(summary.status, bad.expected);
    EXPECT_EQ(summary.stats.after_latch, 1u);
    EXPECT_EQ(summary.stats.decisions, 1u);
    EXPECT_EQ(summary.stats.motions, 0u);
    EXPECT_EQ(summary.trace.size(), 1u);
    EXPECT_EQ(summary.decision_chain, Chain({kShaGoldenDecision}));
  }
}

TEST(SafetyReplayAdapterTest, RB6InvalidPayloadsAreRecordedNotChained) {
  const std::vector<uint8_t> bytes =
      WriteEpisode({InvalidDecision(1), InvalidMotion(2), SafetyEvent(3),
                    DesiredMotionEvent(4), DecisionAt(5, kGoldenIntentDigest)});
  std::unique_ptr<SafetyReplayAdapter> adapter = MakeAdapter();
  ASSERT_NE(adapter, nullptr);
  const std::vector<SafetyReplayStep> steps = FeedAll(adapter.get(), bytes);
  ASSERT_EQ(steps.size(), 5u);
  for (const SafetyReplayStep& step : steps) {
    EXPECT_EQ(step.status, ReplayAdapterStatus::kOk);
  }
  EXPECT_FALSE(steps[0].accepted);
  EXPECT_EQ(steps[0].error, kMissingHeaderError);
  EXPECT_FALSE(steps[1].accepted);
  EXPECT_EQ(steps[1].error, kObjectiveError);
  // An empty message is not engaged: not accepted and not an error.
  EXPECT_FALSE(steps[2].accepted);
  EXPECT_EQ(steps[2].error, kNoError);
  EXPECT_FALSE(steps[3].accepted);
  EXPECT_EQ(steps[3].error, kNoError);
  EXPECT_TRUE(steps[4].accepted);

  const SafetyReplaySummary summary = adapter->Finish();
  EXPECT_EQ(summary.status, ReplayAdapterStatus::kOk);
  EXPECT_EQ(summary.trace.size(), 5u);
  EXPECT_EQ(summary.stats.decisions, 3u);
  EXPECT_EQ(summary.stats.decisions_accepted, 1u);
  EXPECT_EQ(summary.stats.motions, 2u);
  EXPECT_EQ(summary.stats.motions_accepted, 0u);
  EXPECT_EQ(summary.decision_chain, Chain({steps[4].payload_sha256}));
}

TEST(SafetyReplayAdapterTest, RB7IntentLinkage) {
  const EpisodeEvent motion = BuiltMotion(2);
  const EpisodeEvent rejected = InvalidMotion(4);
  const std::string motion_digest = IntentDigestOf(motion);
  const std::string rejected_digest = IntentDigestOf(rejected);
  ASSERT_NE(motion_digest, rejected_digest);
  const std::string unseen(64, 'a');
  const std::vector<uint8_t> bytes = WriteEpisode(
      {DecisionAt(1, motion_digest), motion, DecisionAt(3, motion_digest),
       rejected, DecisionAt(5, rejected_digest), DecisionAt(6, unseen)});
  std::unique_ptr<SafetyReplayAdapter> adapter = MakeAdapter();
  ASSERT_NE(adapter, nullptr);
  const std::vector<SafetyReplayStep> steps = FeedAll(adapter.get(), bytes);
  ASSERT_EQ(steps.size(), 6u);
  // The set is checked only against earlier records.
  EXPECT_FALSE(steps[0].intent_linked);
  EXPECT_TRUE(steps[2].intent_linked);
  // Every motion is digested, accepted or not.
  EXPECT_FALSE(steps[3].accepted);
  EXPECT_TRUE(steps[4].intent_linked);
  EXPECT_TRUE(steps[4].accepted);
  EXPECT_FALSE(steps[5].intent_linked);
  for (const SafetyReplayStep& step : steps) {
    EXPECT_EQ(step.status, ReplayAdapterStatus::kOk);
  }

  const SafetyReplaySummary summary = adapter->Finish();
  // An unlinked decision degrades but never latches.
  EXPECT_EQ(summary.status, ReplayAdapterStatus::kOk);
  EXPECT_TRUE(summary.degraded);
  EXPECT_EQ(summary.stats.decisions, 4u);
  EXPECT_EQ(summary.stats.decisions_accepted, 4u);
  EXPECT_EQ(summary.stats.unlinked_decisions, 2u);
  EXPECT_EQ(summary.stats.motions, 2u);
  EXPECT_EQ(summary.stats.motions_accepted, 1u);
  // Linked or not, every accepted decision is chained.
  EXPECT_EQ(summary.decision_chain,
            Chain({steps[0].payload_sha256, steps[2].payload_sha256,
                   steps[4].payload_sha256, steps[5].payload_sha256}));
  EXPECT_EQ(summary.decision_chain, kDecisionChainRb7);

  // With every decision linked the record is not degraded.
  const std::vector<uint8_t> linked_bytes =
      WriteEpisode({motion, DecisionAt(3, motion_digest)});
  std::unique_ptr<SafetyReplayAdapter> linked = MakeAdapter();
  ASSERT_NE(linked, nullptr);
  FeedAll(linked.get(), linked_bytes);
  const SafetyReplaySummary linked_summary = linked->Finish();
  EXPECT_FALSE(linked_summary.degraded);
  EXPECT_EQ(linked_summary.stats.unlinked_decisions, 0u);
}

TEST(SafetyReplayAdapterTest, RB12OtherKindsAreIgnoredAndLossIsAGap) {
  {
    const std::vector<uint8_t> bytes = WriteEpisode({
        TrajectoryEvent(1),
        InferenceEnvelopeEvent(2),
        InferenceResultEvent(3),
        RecordedEvent(4, "intrinsic_proto.estimation.MeasurementEnvelope", ""),
        WorldRecordEvent(5),
        MetadataEvent(6, Seed42MarineText()),
    });
    std::unique_ptr<SafetyReplayAdapter> adapter = MakeAdapter();
    ASSERT_NE(adapter, nullptr);
    const std::vector<SafetyReplayStep> steps = FeedAll(adapter.get(), bytes);
    ASSERT_EQ(steps.size(), 6u);
    for (const SafetyReplayStep& step : steps) {
      EXPECT_EQ(step.status, ReplayAdapterStatus::kIgnoredPayload);
      EXPECT_EQ(step.payload_kind, PayloadKind::kNone);
    }
    const SafetyReplaySummary summary = adapter->Finish();
    EXPECT_EQ(summary.stats.fed, 6u);
    EXPECT_EQ(summary.stats.ignored, 6u);
    EXPECT_TRUE(summary.trace.empty());
    EXPECT_FALSE(summary.degraded);
    EXPECT_EQ(summary.decision_chain, kEmptyChain);
  }
  {
    const std::vector<uint8_t> bytes = WriteOverflowEpisode(
        {MetadataEvent(1, Seed42MarineText()), BuiltMotion(2),
         DecisionAt(3, kGoldenMotionIntentDigest)},
        {DecisionAt(4, kGoldenIntentDigest), BuiltMotion(5)},
        DecisionAt(6, kGoldenMotionIntentDigest));
    std::unique_ptr<SafetyReplayAdapter> adapter = MakeAdapter();
    ASSERT_NE(adapter, nullptr);
    const std::vector<SafetyReplayStep> steps = FeedAll(adapter.get(), bytes);
    ASSERT_EQ(steps.size(), 5u);
    EXPECT_EQ(steps[3].status, ReplayAdapterStatus::kLossRecord);
    // Dropped events consume writer indices 3 and 4, the loss record is 5.
    EXPECT_EQ(steps[3].writer_index, 5u);
    EXPECT_EQ(steps[4].status, ReplayAdapterStatus::kOk);
    EXPECT_EQ(steps[4].writer_index, 6u);
    const SafetyReplaySummary summary = adapter->Finish();
    EXPECT_EQ(summary.status, ReplayAdapterStatus::kOk);
    EXPECT_TRUE(summary.degraded);
    EXPECT_EQ(summary.stats.gaps, 1u);
    EXPECT_EQ(summary.stats.gap_dropped, 2u);
    EXPECT_EQ(summary.stats.decisions, 2u);
    EXPECT_EQ(summary.stats.motions, 1u);
  }
}

TEST(SafetyReplayAdapterTest, RB13Boundaries) {
  {
    const std::vector<uint8_t> bytes = WriteEpisode({});
    std::unique_ptr<SafetyReplayAdapter> adapter = MakeAdapter();
    ASSERT_NE(adapter, nullptr);
    EXPECT_TRUE(FeedAll(adapter.get(), bytes).empty());
    const SafetyReplaySummary summary = adapter->Finish();
    EXPECT_EQ(summary.status, ReplayAdapterStatus::kOk);
    EXPECT_EQ(summary.decision_chain, kEmptyChain);
    EXPECT_EQ(summary.decision_chain,
              internal::EpisodeSha256Hex(std::string("v1\n")));
    EXPECT_TRUE(summary.trace.empty());
    EXPECT_FALSE(summary.degraded);
  }
  {
    const std::vector<uint8_t> bytes =
        WriteEpisode({ParseGolden("safety_decision")});
    std::unique_ptr<SafetyReplayAdapter> adapter = MakeAdapter();
    ASSERT_NE(adapter, nullptr);
    EpisodeStream stream(bytes);
    EpisodeReplayEvent event;
    ASSERT_TRUE(stream.Next(&event));
    const SafetyReplayStep step = adapter->Feed(event);
    EXPECT_EQ(step.status, ReplayAdapterStatus::kOk);
    EXPECT_TRUE(step.accepted);

    const SafetyReplaySummary first = adapter->Finish();
    const SafetyReplayStep after = adapter->Feed(event);
    EXPECT_EQ(after.status, ReplayAdapterStatus::kFinished);
    EXPECT_EQ(after.writer_index, 0u);
    EXPECT_EQ(after.payload_kind, PayloadKind::kNone);
    const SafetyReplaySummary second = adapter->Finish();
    ExpectSummaryEqual(first, second);
    EXPECT_EQ(second.stats.fed, 1u);
    EXPECT_EQ(second.decision_chain, Chain({kShaGoldenDecision}));
  }
}

TEST(SafetyReplayAdapterTest, RB14ConfigAndIsolation) {
  const std::vector<std::string> bad_digests = {
      std::string(63, 'a'),
      std::string(64, 'A'),
      std::string(65, 'a'),
      std::string("g") + std::string(63, 'a'),
  };
  for (const std::string& digest : bad_digests) {
    SafetyReplayConfig config;
    config.compatibility.expected_config_digest = digest;
    ReplayAdapterStatus status = ReplayAdapterStatus::kOk;
    EXPECT_EQ(SafetyReplayAdapter::Create(config, &status), nullptr) << digest;
    EXPECT_EQ(status, ReplayAdapterStatus::kInvalidConfig) << digest;
  }
  ReplayAdapterStatus status = ReplayAdapterStatus::kInvalidConfig;
  EXPECT_NE(SafetyReplayAdapter::Create(SafetyReplayConfig{}, &status),
            nullptr);
  EXPECT_EQ(status, ReplayAdapterStatus::kOk);

  // The call-count proof: the config has one field, the compatibility rules,
  // so there is no supervisor or aggregation hook to inject and none can be
  // called.
  static_assert(sizeof(SafetyReplayConfig) ==
                sizeof(ReplayCompatibilityConfig));
  ExpectNoForbiddenDeps(BuildDepsOf("safety_replay_adapter"));
}

}  // namespace
}  // namespace intrinsic::episode
