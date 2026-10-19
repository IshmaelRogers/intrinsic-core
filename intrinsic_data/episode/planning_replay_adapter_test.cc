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

// RB1, RB4 to RB6 and RB12 to RB14 of lock id
// episode-replay-adapters-planning-safety-inference-v1 (#132), planning side.
// Episodes are built in memory with EpisodeWriter, read with EpisodeReader and
// replayed with EpisodeReplayer in kStep. No fixture file is added. The
// #127/#244 goldens episode_event_v1_{vehicle_trajectory,desired_motion} are
// read unmodified. No planner is linked, so none can be called.

#include "intrinsic_data/episode/planning_replay_adapter.h"

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
#include "intrinsic/vehicle/trajectory_contract_policy.h"
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
constexpr char kShaGoldenTrajectory[] =
    "7341a0e88d7ebb14cf8b1637bd56f72696fabfaec7e6af7b9dc2afab46d45a32";
constexpr char kShaGoldenMotion[] =
    "4e7f198e7cdf5fdb8b26ca6e6ad4eb5620ce6da734b0bd2dca65965c04edd9c5";
constexpr char kShaBuiltTrajectory[] =
    "f562ec16ce00352730e9fac634eae335a53591f07a5b4fb57fe44a606345a38c";
constexpr char kPlanChainRb1[] =
    "a6dffe4a19192a506e555e300145a4ee2b83d809c57329e0fa9f4d7911c9c656";
constexpr char kIntentChainRb1[] =
    "b820b746057bb259164844a575ee699892a9aeac300bb7e2255fe9756bb58d83";

PlanningReplayConfig DefaultConfig() {
  PlanningReplayConfig config;
  config.compatibility.expected_config_digest = kSeed42Digest;
  return config;
}

std::unique_ptr<PlanningReplayAdapter> MakeAdapter(
    const PlanningReplayConfig& config = DefaultConfig()) {
  ReplayAdapterStatus status = ReplayAdapterStatus::kInvalidConfig;
  std::unique_ptr<PlanningReplayAdapter> adapter =
      PlanningReplayAdapter::Create(config, &status);
  EXPECT_EQ(status, ReplayAdapterStatus::kOk);
  EXPECT_NE(adapter, nullptr);
  return adapter;
}

std::vector<PlanningReplayStep> FeedAll(PlanningReplayAdapter* adapter,
                                        const std::vector<uint8_t>& bytes) {
  EpisodeStream stream(bytes);
  std::vector<PlanningReplayStep> steps;
  EpisodeReplayEvent event;
  while (stream.Next(&event)) {
    steps.push_back(adapter->Feed(event));
  }
  return steps;
}

// The golden trajectory (sequence 2) with a new id and a cost. Valid.
EpisodeEvent BuiltTrajectory(uint64_t sequence) {
  EpisodeEvent event = ParseGolden("vehicle_trajectory");
  event.mutable_header()->set_sequence(sequence);
  event.mutable_vehicle_trajectory()->set_trajectory_id("traj_beta");
  event.mutable_vehicle_trajectory()->set_cost(1.5);
  return event;
}

// The golden motion (sequence 8) with a longer horizon. Valid.
EpisodeEvent BuiltMotion(uint64_t sequence) {
  EpisodeEvent event = ParseGolden("desired_motion");
  event.mutable_header()->set_sequence(sequence);
  event.mutable_desired_motion()->mutable_horizon()->set_seconds(3);
  return event;
}

EpisodeEvent InvalidTrajectory(uint64_t sequence) {
  EpisodeEvent event = BuiltTrajectory(sequence);
  event.mutable_vehicle_trajectory()->clear_trajectory_id();
  return event;
}

// A header-only motion has no objective.
EpisodeEvent InvalidMotion(uint64_t sequence) {
  EpisodeEvent event = BuiltMotion(sequence);
  event.mutable_desired_motion()->clear_twist();
  return event;
}

struct Row {
  uint64_t writer_index;
  PayloadKind kind;
  int error;
  bool accepted;
  std::string sha;
};

void ExpectTrace(const std::vector<PlanningTraceEntry>& trace,
                 const std::vector<Row>& expected) {
  ASSERT_EQ(trace.size(), expected.size());
  for (size_t i = 0; i < expected.size(); ++i) {
    EXPECT_EQ(trace[i].writer_index, expected[i].writer_index) << i;
    EXPECT_EQ(trace[i].payload_kind, expected[i].kind) << i;
    EXPECT_EQ(trace[i].error, expected[i].error) << i;
    EXPECT_EQ(trace[i].accepted, expected[i].accepted) << i;
    EXPECT_EQ(trace[i].payload_sha256, expected[i].sha) << i;
  }
}

std::string Chain(const std::vector<std::string>& shas) {
  std::string input = "v1\n";
  for (const std::string& sha : shas) {
    input += sha + "\n";
  }
  return internal::EpisodeSha256Hex(input);
}

bool StatsEqual(const PlanningReplayStats& a, const PlanningReplayStats& b) {
  return a.fed == b.fed && a.trajectories == b.trajectories &&
         a.trajectories_accepted == b.trajectories_accepted &&
         a.motions == b.motions && a.motions_accepted == b.motions_accepted &&
         a.ignored == b.ignored && a.invalid_records == b.invalid_records &&
         a.gaps == b.gaps && a.gap_dropped == b.gap_dropped &&
         a.after_latch == b.after_latch;
}

void ExpectSummaryEqual(const PlanningReplaySummary& a,
                        const PlanningReplaySummary& b) {
  EXPECT_EQ(a.status, b.status);
  EXPECT_EQ(a.degraded, b.degraded);
  EXPECT_EQ(a.plan_chain, b.plan_chain);
  EXPECT_EQ(a.intent_chain, b.intent_chain);
  EXPECT_TRUE(StatsEqual(a.stats, b.stats));
  ASSERT_EQ(a.trace.size(), b.trace.size());
  for (size_t i = 0; i < a.trace.size(); ++i) {
    EXPECT_EQ(a.trace[i].writer_index, b.trace[i].writer_index);
    EXPECT_EQ(a.trace[i].payload_kind, b.trace[i].payload_kind);
    EXPECT_EQ(a.trace[i].error, b.trace[i].error);
    EXPECT_EQ(a.trace[i].accepted, b.trace[i].accepted);
    EXPECT_EQ(a.trace[i].payload_sha256, b.trace[i].payload_sha256);
  }
}

constexpr int kNoError = 0;
const int kTrajectoryIdError =
    static_cast<int>(vehicle::TrajectoryContractError::kTrajectoryId);
const int kObjectiveError =
    static_cast<int>(vehicle::ContractError::kObjective);

TEST(PlanningReplayAdapterTest, RB1Nominal) {
  const std::vector<uint8_t> bytes =
      WriteEpisode({ParseGolden("vehicle_trajectory"),
                    ParseGolden("desired_motion"), BuiltTrajectory(9)});
  std::unique_ptr<PlanningReplayAdapter> adapter = MakeAdapter();
  ASSERT_NE(adapter, nullptr);
  const std::vector<PlanningReplayStep> steps = FeedAll(adapter.get(), bytes);
  ASSERT_EQ(steps.size(), 3u);
  for (const PlanningReplayStep& step : steps) {
    EXPECT_EQ(step.status, ReplayAdapterStatus::kOk);
    EXPECT_TRUE(step.accepted);
    EXPECT_EQ(step.error, kNoError);
  }
  EXPECT_EQ(steps[0].payload_kind, PayloadKind::kVehicleTrajectory);
  EXPECT_EQ(steps[1].payload_kind, PayloadKind::kDesiredMotion);
  EXPECT_EQ(steps[2].payload_kind, PayloadKind::kVehicleTrajectory);
  EXPECT_EQ(steps[0].payload_sha256, kShaGoldenTrajectory);

  const PlanningReplaySummary summary = adapter->Finish();
  EXPECT_EQ(summary.status, ReplayAdapterStatus::kOk);
  EXPECT_FALSE(summary.degraded);
  ExpectTrace(summary.trace, {{0, PayloadKind::kVehicleTrajectory, kNoError,
                               true, kShaGoldenTrajectory},
                              {1, PayloadKind::kDesiredMotion, kNoError, true,
                               kShaGoldenMotion},
                              {2, PayloadKind::kVehicleTrajectory, kNoError,
                               true, kShaBuiltTrajectory}});
  EXPECT_EQ(summary.plan_chain, kPlanChainRb1);
  EXPECT_EQ(summary.intent_chain, kIntentChainRb1);
  EXPECT_EQ(summary.plan_chain,
            Chain({kShaGoldenTrajectory, kShaBuiltTrajectory}));
  EXPECT_EQ(summary.intent_chain, Chain({kShaGoldenMotion}));
  EXPECT_EQ(summary.stats.fed, 3u);
  EXPECT_EQ(summary.stats.trajectories, 2u);
  EXPECT_EQ(summary.stats.trajectories_accepted, 2u);
  EXPECT_EQ(summary.stats.motions, 1u);
  EXPECT_EQ(summary.stats.motions_accepted, 1u);
  EXPECT_EQ(summary.stats.ignored, 0u);
  EXPECT_EQ(summary.stats.invalid_records, 0u);
}

TEST(PlanningReplayAdapterTest, RB4DeterministicAndTheMotionDigestMatches) {
  const std::vector<uint8_t> bytes = WriteEpisode(
      {ParseGolden("vehicle_trajectory"), ParseGolden("desired_motion"),
       BuiltTrajectory(9), InvalidMotion(10)});
  std::unique_ptr<PlanningReplayAdapter> first = MakeAdapter();
  std::unique_ptr<PlanningReplayAdapter> second = MakeAdapter();
  ASSERT_NE(first, nullptr);
  ASSERT_NE(second, nullptr);
  const std::vector<PlanningReplayStep> a = FeedAll(first.get(), bytes);
  const std::vector<PlanningReplayStep> b = FeedAll(second.get(), bytes);
  ASSERT_EQ(a.size(), b.size());
  for (size_t i = 0; i < a.size(); ++i) {
    EXPECT_EQ(a[i].status, b[i].status) << i;
    EXPECT_EQ(a[i].writer_index, b[i].writer_index) << i;
    EXPECT_EQ(a[i].payload_kind, b[i].payload_kind) << i;
    EXPECT_EQ(a[i].error, b[i].error) << i;
    EXPECT_EQ(a[i].accepted, b[i].accepted) << i;
    EXPECT_EQ(a[i].payload_sha256, b[i].payload_sha256) << i;
  }
  ExpectSummaryEqual(first->Finish(), second->Finish());

  const EpisodeEvent motion = ParseGolden("desired_motion");
  EXPECT_EQ(a[1].payload_sha256,
            safety::ComputeOriginalIntentDigest(motion.desired_motion()));
  const EpisodeEvent invalid = InvalidMotion(10);
  EXPECT_EQ(a[3].payload_sha256,
            safety::ComputeOriginalIntentDigest(invalid.desired_motion()));
}

TEST(PlanningReplayAdapterTest, RB5CompatibilityLatches) {
  for (const BadEvent& bad : BadCompatibilityEvents(BuiltTrajectory(2))) {
    SCOPED_TRACE(bad.name);
    EpisodeEvent first = ParseGolden("vehicle_trajectory");
    first.mutable_header()->set_sequence(1);
    const std::vector<uint8_t> bytes =
        WriteEpisode({first, bad.event, BuiltMotion(3)});
    std::unique_ptr<PlanningReplayAdapter> adapter = MakeAdapter();
    ASSERT_NE(adapter, nullptr);
    const std::vector<PlanningReplayStep> steps = FeedAll(adapter.get(), bytes);
    ASSERT_EQ(steps.size(), 3u);
    EXPECT_EQ(steps[0].status, ReplayAdapterStatus::kOk);
    EXPECT_EQ(steps[1].status, bad.expected);
    EXPECT_EQ(steps[1].payload_kind, PayloadKind::kNone);
    EXPECT_TRUE(steps[1].payload_sha256.empty());
    EXPECT_EQ(steps[2].status, ReplayAdapterStatus::kLatched);
    EXPECT_EQ(steps[2].payload_kind, PayloadKind::kNone);
    const PlanningReplaySummary summary = adapter->Finish();
    EXPECT_EQ(summary.status, bad.expected);
    EXPECT_EQ(summary.stats.after_latch, 1u);
    EXPECT_EQ(summary.stats.trajectories, 1u);
    EXPECT_EQ(summary.stats.motions, 0u);
    ExpectTrace(summary.trace, {{0, PayloadKind::kVehicleTrajectory, kNoError,
                                 true, kShaGoldenTrajectory}});
    EXPECT_EQ(summary.plan_chain, Chain({kShaGoldenTrajectory}));
    EXPECT_EQ(summary.intent_chain, kEmptyChain);
  }
}

TEST(PlanningReplayAdapterTest, RB6InvalidPayloadsAreRecordedNotChained) {
  const std::vector<uint8_t> bytes =
      WriteEpisode({InvalidTrajectory(1), InvalidMotion(2), TrajectoryEvent(3),
                    DesiredMotionEvent(4), BuiltTrajectory(5)});
  std::unique_ptr<PlanningReplayAdapter> adapter = MakeAdapter();
  ASSERT_NE(adapter, nullptr);
  const std::vector<PlanningReplayStep> steps = FeedAll(adapter.get(), bytes);
  ASSERT_EQ(steps.size(), 5u);
  for (const PlanningReplayStep& step : steps) {
    EXPECT_EQ(step.status, ReplayAdapterStatus::kOk);
  }
  EXPECT_FALSE(steps[0].accepted);
  EXPECT_EQ(steps[0].error, kTrajectoryIdError);
  EXPECT_FALSE(steps[1].accepted);
  EXPECT_EQ(steps[1].error, kObjectiveError);
  // An empty message is not engaged: not accepted and not an error.
  EXPECT_FALSE(steps[2].accepted);
  EXPECT_EQ(steps[2].error, kNoError);
  EXPECT_FALSE(steps[3].accepted);
  EXPECT_EQ(steps[3].error, kNoError);
  EXPECT_TRUE(steps[4].accepted);

  const PlanningReplaySummary summary = adapter->Finish();
  EXPECT_EQ(summary.status, ReplayAdapterStatus::kOk);
  EXPECT_FALSE(summary.degraded);
  EXPECT_EQ(summary.trace.size(), 5u);
  EXPECT_EQ(summary.stats.trajectories, 3u);
  EXPECT_EQ(summary.stats.trajectories_accepted, 1u);
  EXPECT_EQ(summary.stats.motions, 2u);
  EXPECT_EQ(summary.stats.motions_accepted, 0u);
  EXPECT_EQ(summary.plan_chain, Chain({steps[4].payload_sha256}));
  EXPECT_EQ(summary.intent_chain, kEmptyChain);
}

TEST(PlanningReplayAdapterTest, RB12OtherKindsAreIgnoredAndLossIsAGap) {
  {
    const std::vector<uint8_t> bytes = WriteEpisode({
        SafetyEvent(1),
        InferenceEnvelopeEvent(2),
        InferenceResultEvent(3),
        RecordedEvent(4, "intrinsic_proto.estimation.MeasurementEnvelope", ""),
        WorldRecordEvent(5),
        MetadataEvent(6, Seed42MarineText()),
    });
    std::unique_ptr<PlanningReplayAdapter> adapter = MakeAdapter();
    ASSERT_NE(adapter, nullptr);
    const std::vector<PlanningReplayStep> steps = FeedAll(adapter.get(), bytes);
    ASSERT_EQ(steps.size(), 6u);
    for (const PlanningReplayStep& step : steps) {
      EXPECT_EQ(step.status, ReplayAdapterStatus::kIgnoredPayload);
      EXPECT_EQ(step.payload_kind, PayloadKind::kNone);
    }
    const PlanningReplaySummary summary = adapter->Finish();
    EXPECT_EQ(summary.stats.fed, 6u);
    EXPECT_EQ(summary.stats.ignored, 6u);
    EXPECT_TRUE(summary.trace.empty());
    EXPECT_FALSE(summary.degraded);
    EXPECT_EQ(summary.plan_chain, kEmptyChain);
    EXPECT_EQ(summary.intent_chain, kEmptyChain);
  }
  {
    const std::vector<uint8_t> bytes = WriteOverflowEpisode(
        {MetadataEvent(1, Seed42MarineText()), BuiltTrajectory(2),
         BuiltMotion(3)},
        {BuiltTrajectory(4), BuiltMotion(5)}, BuiltTrajectory(6));
    std::unique_ptr<PlanningReplayAdapter> adapter = MakeAdapter();
    ASSERT_NE(adapter, nullptr);
    const std::vector<PlanningReplayStep> steps = FeedAll(adapter.get(), bytes);
    ASSERT_EQ(steps.size(), 5u);
    EXPECT_EQ(steps[3].status, ReplayAdapterStatus::kLossRecord);
    // Dropped events consume writer indices 3 and 4, the loss record is 5.
    EXPECT_EQ(steps[3].writer_index, 5u);
    EXPECT_EQ(steps[4].status, ReplayAdapterStatus::kOk);
    EXPECT_EQ(steps[4].writer_index, 6u);
    const PlanningReplaySummary summary = adapter->Finish();
    EXPECT_EQ(summary.status, ReplayAdapterStatus::kOk);
    EXPECT_TRUE(summary.degraded);
    EXPECT_EQ(summary.stats.gaps, 1u);
    EXPECT_EQ(summary.stats.gap_dropped, 2u);
    EXPECT_EQ(summary.stats.trajectories, 2u);
    EXPECT_EQ(summary.stats.motions, 1u);
    EXPECT_EQ(summary.trace.size(), 3u);
  }
}

TEST(PlanningReplayAdapterTest, RB13Boundaries) {
  {
    const std::vector<uint8_t> bytes = WriteEpisode({});
    std::unique_ptr<PlanningReplayAdapter> adapter = MakeAdapter();
    ASSERT_NE(adapter, nullptr);
    EXPECT_TRUE(FeedAll(adapter.get(), bytes).empty());
    const PlanningReplaySummary summary = adapter->Finish();
    EXPECT_EQ(summary.status, ReplayAdapterStatus::kOk);
    EXPECT_EQ(summary.plan_chain, kEmptyChain);
    EXPECT_EQ(summary.intent_chain, kEmptyChain);
    EXPECT_EQ(summary.plan_chain,
              internal::EpisodeSha256Hex(std::string("v1\n")));
    EXPECT_TRUE(summary.trace.empty());
    EXPECT_FALSE(summary.degraded);
  }
  {
    const std::vector<uint8_t> bytes = WriteEpisode({BuiltMotion(1)});
    std::unique_ptr<PlanningReplayAdapter> adapter = MakeAdapter();
    ASSERT_NE(adapter, nullptr);
    EpisodeStream stream(bytes);
    EpisodeReplayEvent event;
    ASSERT_TRUE(stream.Next(&event));
    const PlanningReplayStep step = adapter->Feed(event);
    EXPECT_EQ(step.status, ReplayAdapterStatus::kOk);
    EXPECT_TRUE(step.accepted);

    const PlanningReplaySummary first = adapter->Finish();
    const PlanningReplayStep after = adapter->Feed(event);
    EXPECT_EQ(after.status, ReplayAdapterStatus::kFinished);
    EXPECT_EQ(after.writer_index, 0u);
    EXPECT_EQ(after.payload_kind, PayloadKind::kNone);
    const PlanningReplaySummary second = adapter->Finish();
    ExpectSummaryEqual(first, second);
    EXPECT_EQ(second.stats.fed, 1u);
    EXPECT_EQ(second.plan_chain, kEmptyChain);
    EXPECT_EQ(second.intent_chain, Chain({step.payload_sha256}));
  }
}

TEST(PlanningReplayAdapterTest, RB14ConfigAndIsolation) {
  const std::vector<std::string> bad_digests = {
      std::string(63, 'a'),
      std::string(64, 'A'),
      std::string(65, 'a'),
      std::string("g") + std::string(63, 'a'),
  };
  for (const std::string& digest : bad_digests) {
    PlanningReplayConfig config;
    config.compatibility.expected_config_digest = digest;
    ReplayAdapterStatus status = ReplayAdapterStatus::kOk;
    EXPECT_EQ(PlanningReplayAdapter::Create(config, &status), nullptr)
        << digest;
    EXPECT_EQ(status, ReplayAdapterStatus::kInvalidConfig) << digest;
  }
  ReplayAdapterStatus status = ReplayAdapterStatus::kInvalidConfig;
  EXPECT_NE(PlanningReplayAdapter::Create(PlanningReplayConfig{}, &status),
            nullptr);
  EXPECT_EQ(status, ReplayAdapterStatus::kOk);

  // The call-count proof: the config has one field, the compatibility rules,
  // so there is no planner, queue or worker to inject and none can be called.
  static_assert(sizeof(PlanningReplayConfig) ==
                sizeof(ReplayCompatibilityConfig));
  ExpectNoForbiddenDeps(BuildDepsOf("planning_replay_adapter"));
}

}  // namespace
}  // namespace intrinsic::episode
