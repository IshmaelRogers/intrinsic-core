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

// RA2 to RA5 and RA7 to RA10, RA12 of lock id
// episode-replay-adapters-estimator-world-v1 (#131), World side. Episodes are
// built in memory with EpisodeWriter, read with EpisodeReader and replayed
// with EpisodeReplayer in kStep. No fixture file is added. The #244 World
// golden is read only as the snapshot_id negative case (contract F3).

#include "intrinsic_data/episode/world_replay_adapter.h"

#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <fstream>
#include <memory>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include "absl/status/status.h"
#include "absl/types/span.h"
#include "gtest/gtest.h"
#include "intrinsic/episode/proto/episode_event.pb.h"
#include "intrinsic/world/marine_component_validity/marine_component_validity_policy.h"
#include "intrinsic/world/proto/world_snapshot_descriptor.pb.h"
#include "intrinsic/world/world_snapshot/world_snapshot_builder.h"
#include "intrinsic/world/world_snapshot/world_snapshot_policy.h"
#include "intrinsic/world/world_snapshot/world_snapshot_skew_assessor.h"
#include "intrinsic/world/world_snapshot/world_snapshot_skew_policy.h"
#include "intrinsic_data/episode/episode_event_policy.h"
#include "intrinsic_data/episode/episode_reader.h"
#include "intrinsic_data/episode/episode_replay.h"
#include "intrinsic_data/episode/episode_replay_adapter_common.h"
#include "intrinsic_data/episode/episode_writer.h"

namespace intrinsic::episode {
namespace {

using intrinsic_proto::episode::EpisodeEvent;
using world::SnapshotError;
using world::SnapshotPolicyError;
using world::SnapshotPolicyStatus;

constexpr char kSeed42Digest[] =
    "93150ec8e5fd8f6dcc2d9cae50d84de9b89287b187edfa4a807b0da0e8eb5d6a";
constexpr char kOtherDigest[] =
    "bbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbb";
constexpr char kMeasurementType[] =
    "intrinsic_proto.estimation.MeasurementEnvelope";

// Recomputed snapshot_id values (SHA-256 over "v1\nstate_epoch=<n>\n" and the
// "<kind>=<revision>\n" lines, see world_snapshot/README.md), produced by an
// independent script. `kIdA` is the F3 value for state_epoch 7,
// bathymetry_reference=3, current_field=5.
constexpr char kIdA[] =
    "02ed8cd9476beeb4395573244f1744c052d41a16387c5fc1c1375319836a9900";
constexpr char kIdB[] =
    "0118bd208f66f06ebf8c765e6012ecf37a1f0f9ee6b3e7171ff863976213055d";
constexpr char kChainAB[] =
    "b1183df0aebbf1492a39af7fe8e5a355079051aa9aa06559f65373a39efa8347";
// SHA-256 of "v1\n".
constexpr char kEmptyChain[] =
    "2d27fbdf4e8ca207afbfa388ca9172fbcc6c70e534af2476b3b704f87debadcf";
constexpr char kIdIncomplete[] =
    "587e84fa0440a92ea076441701ec2bbdc72424434b824dd33afdacd86df43a3c";
constexpr char kIdStaleHorizon[] =
    "547065a375a7302be2316872d8a7a329838cb89a0090c5bbc990472ab1c44d0c";
constexpr char kIdStaleNoTiming[] =
    "efad4d0fdaeb61ca12484797ed9bd50cdee69b4fd5a9fe6b4ae750d6519125f8";
constexpr char kIdSkew[] =
    "473b755c2b4acd295a63c49727ee31252ccef12c6c922c282a0e672e66638476";
constexpr char kIdDuplicateTimings[] =
    "cbb503c47818190f6076cccf051de48e21df61461a4a99946a48d6d45cdf0483";
constexpr char kIdUnsetHorizon[] =
    "090d379195fbc4e40cc39e8a7b86bf70fc68fbc3cbc45ec08b6bcdf559c66f2f";
constexpr char kChainRa7[] =
    "6f9d302347c719de7c19518f6d27a6512d00c09a04423ec34a89ee873b7ae382";

class MemorySink : public EpisodeSink {
 public:
  absl::Status Write(absl::Span<const uint8_t> bytes) override {
    output.insert(output.end(), bytes.begin(), bytes.end());
    return absl::OkStatus();
  }
  absl::Status Sync() override { return absl::OkStatus(); }

  std::vector<uint8_t> output;
};

std::string ReadGoldenBytes(const std::string& name) {
  const std::string relative =
      "intrinsic_data/episode/testdata/episode_event_v1_" + name + ".binpb";
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
  EXPECT_TRUE(event.ParseFromString(ReadGoldenBytes(name))) << name;
  return event;
}

// Envelope identity copied from the v1 metadata golden. Only the sequence and
// the times (sequence-derived, like the goldens) vary. receive_time is
// 1700000100 + sequence seconds and 251 ms.
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

struct Component {
  std::string kind;
  uint64_t revision;
};

// `horizon_ms` < 0 leaves validity_horizon unset. The observation is
// `ms_before_receive` before the event's receive_time.
struct Timing {
  std::string kind;
  int64_t ms_before_receive;
  int64_t horizon_ms;
};

constexpr int64_t kNanosPerMs = 1000000;
constexpr int64_t kNanosPerSecond = 1000000000;

// A World event whose descriptor snapshot_id recomputes. `envelope_id`
// defaults to the descriptor id.
EpisodeEvent WorldEvent(uint64_t sequence, uint64_t state_epoch,
                        const std::vector<Component>& components,
                        const std::vector<Timing>& timings) {
  EpisodeEvent event = BaseEvent(sequence);
  auto* record = event.mutable_world_snapshot_record();
  auto* snapshot = record->mutable_snapshot();
  std::vector<world::ComponentRevisionView> views;
  for (const Component& component : components) {
    auto* added = snapshot->add_components();
    added->set_component_kind(component.kind);
    added->set_revision(component.revision);
    views.push_back({component.kind, component.revision});
  }
  snapshot->set_state_epoch(state_epoch);
  snapshot->set_snapshot_id(world::ComputeSnapshotId(state_epoch, views));
  snapshot->mutable_creation_time()->set_seconds(1700000100);
  event.set_snapshot_id(snapshot->snapshot_id());
  const int64_t receive_ns =
      (1700000100 + static_cast<int64_t>(sequence)) * kNanosPerSecond +
      251 * kNanosPerMs;
  for (const Timing& timing : timings) {
    auto* added = record->add_timings();
    added->set_component_kind(timing.kind);
    const int64_t observed_ns =
        receive_ns - timing.ms_before_receive * kNanosPerMs;
    added->mutable_observation_time()->set_seconds(observed_ns /
                                                   kNanosPerSecond);
    added->mutable_observation_time()->set_nanos(observed_ns % kNanosPerSecond);
    if (timing.horizon_ms >= 0) {
      added->mutable_validity_horizon()->set_seconds(timing.horizon_ms / 1000);
      added->mutable_validity_horizon()->set_nanos((timing.horizon_ms % 1000) *
                                                   kNanosPerMs);
    }
    added->set_source_id(timing.kind + "_store");
  }
  return event;
}

// Fresh-compatible default timings for bathymetry_reference and current_field.
std::vector<Timing> GoodTimings() {
  return {{"bathymetry_reference", 500, 30000}, {"current_field", 400, 10000}};
}

EpisodeEvent MeasurementLikeEvent(uint64_t sequence) {
  return RecordedEvent(sequence, kMeasurementType, "");
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

WorldReplayConfig DefaultConfig() {
  WorldReplayConfig config;
  config.compatibility.expected_config_digest = kSeed42Digest;
  config.policy.required_kinds = {"bathymetry_reference"};
  config.policy.optional_kinds = {"current_field"};
  return config;
}

std::unique_ptr<WorldReplayAdapter> MakeAdapter(
    const WorldReplayConfig& config = DefaultConfig()) {
  ReplayAdapterStatus status = ReplayAdapterStatus::kInvalidConfig;
  std::unique_ptr<WorldReplayAdapter> adapter =
      WorldReplayAdapter::Create(config, &status);
  EXPECT_EQ(status, ReplayAdapterStatus::kOk);
  EXPECT_NE(adapter, nullptr);
  return adapter;
}

std::vector<WorldReplayStep> FeedAll(WorldReplayAdapter* adapter,
                                     const std::vector<uint8_t>& bytes) {
  EpisodeStream stream(bytes);
  std::vector<WorldReplayStep> steps;
  EpisodeReplayEvent event;
  while (stream.Next(&event)) {
    steps.push_back(adapter->Feed(event));
  }
  return steps;
}

struct Row {
  uint64_t writer_index;
  std::string snapshot_id;
  SnapshotPolicyStatus status;
  bool withhold;
};

void ExpectTrace(const std::vector<WorldTraceEntry>& trace,
                 const std::vector<Row>& expected) {
  ASSERT_EQ(trace.size(), expected.size());
  for (size_t i = 0; i < expected.size(); ++i) {
    EXPECT_EQ(trace[i].writer_index, expected[i].writer_index) << i;
    EXPECT_EQ(trace[i].snapshot_id, expected[i].snapshot_id) << i;
    EXPECT_EQ(trace[i].status, expected[i].status) << i;
    EXPECT_EQ(trace[i].withhold, expected[i].withhold) << i;
  }
}

bool StatsEqual(const WorldReplayStats& a, const WorldReplayStats& b) {
  return a.fed == b.fed && a.assessed == b.assessed &&
         a.accepted == b.accepted && a.withheld == b.withheld &&
         a.ignored == b.ignored && a.invalid_records == b.invalid_records &&
         a.snapshot_id_mismatches == b.snapshot_id_mismatches &&
         a.gaps == b.gaps && a.gap_dropped == b.gap_dropped &&
         a.after_latch == b.after_latch;
}

void ExpectAssessmentEqual(const world::WorldSnapshotPolicyAssessment& a,
                           const world::WorldSnapshotPolicyAssessment& b) {
  EXPECT_EQ(a.status, b.status);
  EXPECT_EQ(a.error, b.error);
  EXPECT_EQ(a.descriptor_error, b.descriptor_error);
  EXPECT_EQ(a.accepted, b.accepted);
  EXPECT_EQ(a.withhold, b.withhold);
  EXPECT_EQ(a.offending_kinds, b.offending_kinds);
  EXPECT_EQ(a.missing_optional_kinds, b.missing_optional_kinds);
  EXPECT_EQ(a.measured_skew_present, b.measured_skew_present);
  EXPECT_EQ(a.measured_skew.seconds, b.measured_skew.seconds);
  EXPECT_EQ(a.measured_skew.nanos, b.measured_skew.nanos);
  EXPECT_EQ(a.earliest_kind, b.earliest_kind);
  EXPECT_EQ(a.latest_kind, b.latest_kind);
}

void ExpectSummaryEqual(const WorldReplaySummary& a,
                        const WorldReplaySummary& b) {
  EXPECT_EQ(a.status, b.status);
  EXPECT_EQ(a.degraded, b.degraded);
  EXPECT_EQ(a.chain_digest, b.chain_digest);
  EXPECT_TRUE(StatsEqual(a.stats, b.stats));
  ASSERT_EQ(a.trace.size(), b.trace.size());
  for (size_t i = 0; i < a.trace.size(); ++i) {
    EXPECT_EQ(a.trace[i].writer_index, b.trace[i].writer_index) << i;
    EXPECT_EQ(a.trace[i].snapshot_id, b.trace[i].snapshot_id) << i;
    EXPECT_EQ(a.trace[i].status, b.trace[i].status) << i;
    EXPECT_EQ(a.trace[i].withhold, b.trace[i].withhold) << i;
  }
}

// The assessment computed straight from the recorded event: the descriptor,
// the timings in record order, and query_time = header.receive_time.
world::WorldSnapshotPolicyAssessment DirectAssessment(
    const EpisodeEvent& event, const world::WorldSnapshotSkewPolicy& policy) {
  const auto& record = event.world_snapshot_record();
  world::ComponentTimings timings;
  for (const auto& timing : record.timings()) {
    world::ComponentTiming converted;
    converted.component_kind = timing.component_kind();
    converted.observation_time = {timing.observation_time().seconds(),
                                  timing.observation_time().nanos()};
    converted.validity_horizon = {timing.validity_horizon().seconds(),
                                  timing.validity_horizon().nanos()};
    converted.source_id = timing.source_id();
    timings.push_back(converted);
  }
  const world::TimeParts query_time = {event.header().receive_time().seconds(),
                                       event.header().receive_time().nanos()};
  return world::AssessWorldSnapshotDescriptorSkew(record.snapshot(), timings,
                                                  query_time, policy);
}

std::vector<EpisodeEvent> NominalEvents() {
  return {
      MetadataEvent(1, Seed42MarineText()),
      WorldEvent(2, 7, {{"bathymetry_reference", 3}, {"current_field", 5}},
                 GoodTimings()),
      MeasurementLikeEvent(3),
      WorldEvent(4, 8, {{"bathymetry_reference", 4}},
                 {{"bathymetry_reference", 300, 30000}}),
  };
}

TEST(WorldReplayAdapterTest, RA2NominalFreshAndPartial) {
  const std::vector<uint8_t> bytes = WriteEpisode(NominalEvents());
  std::unique_ptr<WorldReplayAdapter> adapter = MakeAdapter();
  ASSERT_NE(adapter, nullptr);
  const std::vector<WorldReplayStep> steps = FeedAll(adapter.get(), bytes);
  ASSERT_EQ(steps.size(), 4u);
  EXPECT_EQ(steps[0].status, ReplayAdapterStatus::kIgnoredPayload);
  EXPECT_EQ(steps[1].status, ReplayAdapterStatus::kOk);
  EXPECT_TRUE(steps[1].assessed);
  EXPECT_EQ(steps[1].assessment.status, SnapshotPolicyStatus::kFresh);
  EXPECT_TRUE(steps[1].assessment.accepted);
  EXPECT_EQ(steps[2].status, ReplayAdapterStatus::kIgnoredPayload);
  EXPECT_FALSE(steps[2].assessed);
  EXPECT_EQ(steps[3].status, ReplayAdapterStatus::kOk);
  EXPECT_EQ(steps[3].assessment.status, SnapshotPolicyStatus::kPartial);
  EXPECT_EQ(steps[3].assessment.missing_optional_kinds,
            (std::vector<std::string>{"current_field"}));

  const WorldReplaySummary summary = adapter->Finish();
  EXPECT_EQ(summary.status, ReplayAdapterStatus::kOk);
  EXPECT_FALSE(summary.degraded);
  ExpectTrace(summary.trace,
              {{1, kIdA, SnapshotPolicyStatus::kFresh, false},
               {3, kIdB, SnapshotPolicyStatus::kPartial, false}});
  EXPECT_EQ(summary.chain_digest, kChainAB);
  EXPECT_EQ(summary.stats.fed, 4u);
  EXPECT_EQ(summary.stats.assessed, 2u);
  EXPECT_EQ(summary.stats.accepted, 2u);
  EXPECT_EQ(summary.stats.withheld, 0u);
  EXPECT_EQ(summary.stats.ignored, 2u);
  EXPECT_EQ(summary.stats.invalid_records, 0u);
  EXPECT_EQ(summary.stats.snapshot_id_mismatches, 0u);
}

TEST(WorldReplayAdapterTest, SnapshotIdsRecomputeAndThePinnedDigestHolds) {
  const std::vector<EpisodeEvent> events = NominalEvents();
  EXPECT_EQ(events[1].world_snapshot_record().snapshot().snapshot_id(), kIdA);
  EXPECT_EQ(events[3].world_snapshot_record().snapshot().snapshot_id(), kIdB);
  EXPECT_EQ(ComputeConfigDigest(Seed42MarineText()), kSeed42Digest);
}

TEST(WorldReplayAdapterTest, RA3DeterministicReplay) {
  const std::vector<uint8_t> bytes = WriteEpisode(NominalEvents());
  std::vector<WorldReplayStep> runs[2];
  WorldReplaySummary summaries[2];
  for (int run = 0; run < 2; ++run) {
    std::unique_ptr<WorldReplayAdapter> adapter = MakeAdapter();
    ASSERT_NE(adapter, nullptr);
    runs[run] = FeedAll(adapter.get(), bytes);
    summaries[run] = adapter->Finish();
  }
  ASSERT_EQ(runs[0].size(), runs[1].size());
  for (size_t i = 0; i < runs[0].size(); ++i) {
    EXPECT_EQ(runs[0][i].status, runs[1][i].status) << i;
    EXPECT_EQ(runs[0][i].writer_index, runs[1][i].writer_index) << i;
    EXPECT_EQ(runs[0][i].assessed, runs[1][i].assessed) << i;
    ExpectAssessmentEqual(runs[0][i].assessment, runs[1][i].assessment);
    EXPECT_EQ(runs[0][i].descriptor_error, runs[1][i].descriptor_error) << i;
  }
  ExpectSummaryEqual(summaries[0], summaries[1]);
  for (const WorldReplaySummary& summary : summaries) {
    ExpectTrace(summary.trace,
                {{1, kIdA, SnapshotPolicyStatus::kFresh, false},
                 {3, kIdB, SnapshotPolicyStatus::kPartial, false}});
    EXPECT_EQ(summary.chain_digest, kChainAB);
  }
}

struct BadEvent {
  const char* name;
  EpisodeEvent event;
  ReplayAdapterStatus expected;
};

std::vector<BadEvent> BadCompatibilityEvents() {
  std::vector<BadEvent> bad;
  const std::string other_text = "marine_randomization_metadata_v1\nseed=43\n";
  const auto make_world = [](uint64_t sequence) {
    return WorldEvent(sequence, 7,
                      {{"bathymetry_reference", 3}, {"current_field", 5}},
                      GoodTimings());
  };

  EpisodeEvent event = make_world(2);
  event.set_schema_version(2);
  bad.push_back(
      {"world schema 2", event, ReplayAdapterStatus::kIncompatibleSchema});
  event = SafetyEvent(2);
  event.set_schema_version(2);
  bad.push_back({"ignored kind schema 2", event,
                 ReplayAdapterStatus::kIncompatibleSchema});
  event = MetadataEvent(2, Seed42MarineText());
  event.set_schema_version(2);
  bad.push_back(
      {"metadata schema 2", event, ReplayAdapterStatus::kIncompatibleSchema});
  event = make_world(2);
  event.set_schema_version(2);
  event.set_config_digest(kOtherDigest);
  bad.push_back({"schema is checked before the digest", event,
                 ReplayAdapterStatus::kIncompatibleSchema});

  event = make_world(2);
  event.set_config_digest(kOtherDigest);
  bad.push_back(
      {"world digest", event, ReplayAdapterStatus::kConfigDigestMismatch});
  event = SafetyEvent(2);
  event.set_config_digest(kOtherDigest);
  bad.push_back({"ignored kind digest", event,
                 ReplayAdapterStatus::kConfigDigestMismatch});
  bad.push_back({"metadata with its own passing digest",
                 MetadataEvent(2, other_text),
                 ReplayAdapterStatus::kConfigDigestMismatch});
  return bad;
}

TEST(WorldReplayAdapterTest, RA4CompatibilityLatches) {
  for (const BadEvent& bad : BadCompatibilityEvents()) {
    SCOPED_TRACE(bad.name);
    const std::vector<uint8_t> bytes = WriteEpisode(
        {WorldEvent(1, 7, {{"bathymetry_reference", 3}, {"current_field", 5}},
                    GoodTimings()),
         bad.event,
         WorldEvent(3, 7, {{"bathymetry_reference", 3}, {"current_field", 5}},
                    GoodTimings())});
    std::unique_ptr<WorldReplayAdapter> adapter = MakeAdapter();
    ASSERT_NE(adapter, nullptr);
    const std::vector<WorldReplayStep> steps = FeedAll(adapter.get(), bytes);
    ASSERT_EQ(steps.size(), 3u);
    EXPECT_EQ(steps[0].status, ReplayAdapterStatus::kOk);
    EXPECT_EQ(steps[1].status, bad.expected);
    EXPECT_FALSE(steps[1].assessed);
    EXPECT_EQ(steps[2].status, ReplayAdapterStatus::kLatched);
    EXPECT_FALSE(steps[2].assessed);
    const WorldReplaySummary summary = adapter->Finish();
    EXPECT_EQ(summary.status, bad.expected);
    EXPECT_EQ(summary.stats.after_latch, 1u);
    EXPECT_EQ(summary.stats.assessed, 1u);
    ExpectTrace(summary.trace,
                {{0, kIdA, SnapshotPolicyStatus::kFresh, false}});
  }
}

TEST(WorldReplayAdapterTest, RA4EmptyEventDigestAndEmptyExpectedDigest) {
  EpisodeEvent no_digest = WorldEvent(
      1, 7, {{"bathymetry_reference", 3}, {"current_field", 5}}, GoodTimings());
  no_digest.clear_config_digest();
  EpisodeEvent wrong_digest =
      WorldEvent(2, 8, {{"bathymetry_reference", 4}},
                 {{"bathymetry_reference", 300, 30000}});
  wrong_digest.set_config_digest(kOtherDigest);
  {
    const std::vector<uint8_t> bytes = WriteEpisode({no_digest});
    std::unique_ptr<WorldReplayAdapter> adapter = MakeAdapter();
    ASSERT_NE(adapter, nullptr);
    const std::vector<WorldReplayStep> steps = FeedAll(adapter.get(), bytes);
    ASSERT_EQ(steps.size(), 1u);
    EXPECT_EQ(steps[0].status, ReplayAdapterStatus::kOk);
    const WorldReplaySummary summary = adapter->Finish();
    EXPECT_EQ(summary.status, ReplayAdapterStatus::kOk);
    ExpectTrace(summary.trace,
                {{0, kIdA, SnapshotPolicyStatus::kFresh, false}});
  }
  {
    const std::vector<uint8_t> bytes = WriteEpisode(
        {wrong_digest,
         MetadataEvent(3, "marine_randomization_metadata_v1\nseed=43\n")});
    WorldReplayConfig config = DefaultConfig();
    config.compatibility.expected_config_digest.clear();
    std::unique_ptr<WorldReplayAdapter> adapter = MakeAdapter(config);
    ASSERT_NE(adapter, nullptr);
    const std::vector<WorldReplayStep> steps = FeedAll(adapter.get(), bytes);
    ASSERT_EQ(steps.size(), 2u);
    EXPECT_EQ(steps[0].status, ReplayAdapterStatus::kOk);
    EXPECT_EQ(steps[1].status, ReplayAdapterStatus::kIgnoredPayload);
    const WorldReplaySummary summary = adapter->Finish();
    EXPECT_EQ(summary.status, ReplayAdapterStatus::kOk);
    ExpectTrace(summary.trace,
                {{0, kIdB, SnapshotPolicyStatus::kPartial, false}});
  }
}

TEST(WorldReplayAdapterTest, RA5SnapshotIntegrity) {
  // The #244 golden (F3): its snapshot_id does not recompute.
  const EpisodeEvent golden = ParseGolden("world_snapshot_record");
  ASSERT_EQ(golden.header().sequence(), 7u);

  EpisodeEvent envelope_mismatch = WorldEvent(
      8, 7, {{"bathymetry_reference", 3}, {"current_field", 5}}, GoodTimings());
  envelope_mismatch.set_snapshot_id(kOtherDigest);

  EpisodeEvent duplicate_kind = WorldEvent(
      9, 7, {{"bathymetry_reference", 3}, {"bathymetry_reference", 4}},
      GoodTimings());
  EpisodeEvent no_creation_time =
      WorldEvent(10, 7, {{"bathymetry_reference", 3}}, GoodTimings());
  no_creation_time.mutable_world_snapshot_record()
      ->mutable_snapshot()
      ->clear_creation_time();
  EpisodeEvent empty_kind = WorldEvent(11, 7, {{"", 3}}, GoodTimings());
  EpisodeEvent no_snapshot = BaseEvent(12);
  no_snapshot.mutable_world_snapshot_record();
  EpisodeEvent no_id =
      WorldEvent(13, 7, {{"bathymetry_reference", 3}}, GoodTimings());
  no_id.mutable_world_snapshot_record()
      ->mutable_snapshot()
      ->clear_snapshot_id();
  no_id.clear_snapshot_id();
  EpisodeEvent wrong_epoch =
      WorldEvent(14, 7, {{"bathymetry_reference", 3}, {"current_field", 5}},
                 GoodTimings());
  wrong_epoch.mutable_world_snapshot_record()
      ->mutable_snapshot()
      ->set_state_epoch(9);
  EpisodeEvent empty_envelope_id =
      WorldEvent(15, 7, {{"bathymetry_reference", 3}, {"current_field", 5}},
                 GoodTimings());
  empty_envelope_id.clear_snapshot_id();
  EpisodeEvent good = WorldEvent(16, 8, {{"bathymetry_reference", 4}},
                                 {{"bathymetry_reference", 300, 30000}});

  const std::vector<uint8_t> bytes = WriteEpisode(
      {golden, envelope_mismatch, duplicate_kind, no_creation_time, empty_kind,
       no_snapshot, no_id, wrong_epoch, empty_envelope_id, good});
  std::unique_ptr<WorldReplayAdapter> adapter = MakeAdapter();
  ASSERT_NE(adapter, nullptr);
  const std::vector<WorldReplayStep> steps = FeedAll(adapter.get(), bytes);
  ASSERT_EQ(steps.size(), 10u);

  EXPECT_EQ(steps[0].status, ReplayAdapterStatus::kSnapshotIdMismatch);
  EXPECT_EQ(steps[1].status, ReplayAdapterStatus::kSnapshotIdMismatch);
  EXPECT_EQ(steps[2].status, ReplayAdapterStatus::kInvalidRecord);
  EXPECT_EQ(steps[2].descriptor_error, SnapshotError::kDuplicateComponentKind);
  EXPECT_EQ(steps[3].status, ReplayAdapterStatus::kInvalidRecord);
  EXPECT_EQ(steps[3].descriptor_error, SnapshotError::kCreationTime);
  EXPECT_EQ(steps[4].status, ReplayAdapterStatus::kInvalidRecord);
  EXPECT_EQ(steps[4].descriptor_error, SnapshotError::kEmptyComponentKind);
  EXPECT_EQ(steps[5].status, ReplayAdapterStatus::kInvalidRecord);
  EXPECT_EQ(steps[5].descriptor_error, SnapshotError::kNone);
  EXPECT_EQ(steps[6].status, ReplayAdapterStatus::kInvalidRecord);
  EXPECT_EQ(steps[6].descriptor_error, SnapshotError::kSnapshotId);
  EXPECT_EQ(steps[7].status, ReplayAdapterStatus::kSnapshotIdMismatch);
  // An empty envelope snapshot_id skips only the envelope comparison.
  EXPECT_EQ(steps[8].status, ReplayAdapterStatus::kOk);
  EXPECT_EQ(steps[9].status, ReplayAdapterStatus::kOk);
  for (size_t i = 0; i < 8; ++i) {
    EXPECT_FALSE(steps[i].assessed) << i;
  }

  const WorldReplaySummary summary = adapter->Finish();
  EXPECT_EQ(summary.status, ReplayAdapterStatus::kOk);
  EXPECT_TRUE(summary.degraded);
  EXPECT_EQ(summary.stats.fed, 10u);
  EXPECT_EQ(summary.stats.snapshot_id_mismatches, 3u);
  EXPECT_EQ(summary.stats.invalid_records, 5u);
  EXPECT_EQ(summary.stats.after_latch, 0u);
  EXPECT_EQ(summary.stats.assessed, 2u);
  ExpectTrace(summary.trace,
              {{8, kIdA, SnapshotPolicyStatus::kFresh, false},
               {9, kIdB, SnapshotPolicyStatus::kPartial, false}});
  EXPECT_EQ(summary.chain_digest, kChainAB);
}

TEST(WorldReplayAdapterTest, RA7PolicyStatusesMatchADirectAssessment) {
  const WorldReplayConfig config = DefaultConfig();
  const std::vector<Component> both = {{"bathymetry_reference", 3},
                                       {"current_field", 5}};
  const std::vector<EpisodeEvent> events = {
      // INCOMPLETE: the required kind is absent.
      WorldEvent(1, 1, {{"current_field", 5}}, {{"current_field", 100, 10000}}),
      // STALE: bathymetry_reference expired by its validity horizon.
      WorldEvent(2, 2, both,
                 {{"bathymetry_reference", 1500, 1000},
                  {"current_field", 100, 10000}}),
      // STALE: current_field has no timing.
      WorldEvent(3, 3, both, {{"bathymetry_reference", 100, 30000}}),
      // EXCESSIVE_SKEW: 800 ms between the observations, bound 200 ms.
      WorldEvent(4, 4, both,
                 {{"bathymetry_reference", 900, 30000},
                  {"current_field", 100, 30000}}),
      // Duplicate kind in the timings.
      WorldEvent(5, 5, both,
                 {{"bathymetry_reference", 100, 30000},
                  {"bathymetry_reference", 200, 30000},
                  {"current_field", 100, 30000}}),
      // Unset validity_horizon converts to {0, 0}: expired.
      WorldEvent(
          6, 6, both,
          {{"bathymetry_reference", 100, 30000}, {"current_field", 100, -1}}),
  };
  const SnapshotPolicyStatus expected_status[] = {
      SnapshotPolicyStatus::kIncomplete,  SnapshotPolicyStatus::kStale,
      SnapshotPolicyStatus::kStale,       SnapshotPolicyStatus::kExcessiveSkew,
      SnapshotPolicyStatus::kUnspecified, SnapshotPolicyStatus::kStale};
  const std::string expected_ids[] = {kIdIncomplete,       kIdStaleHorizon,
                                      kIdStaleNoTiming,    kIdSkew,
                                      kIdDuplicateTimings, kIdUnsetHorizon};
  const std::vector<uint8_t> bytes = WriteEpisode(events);
  std::unique_ptr<WorldReplayAdapter> adapter = MakeAdapter(config);
  ASSERT_NE(adapter, nullptr);
  const std::vector<WorldReplayStep> steps = FeedAll(adapter.get(), bytes);
  ASSERT_EQ(steps.size(), events.size());
  for (size_t i = 0; i < events.size(); ++i) {
    SCOPED_TRACE(i);
    EXPECT_EQ(steps[i].status, ReplayAdapterStatus::kOk);
    ASSERT_TRUE(steps[i].assessed);
    ExpectAssessmentEqual(steps[i].assessment,
                          DirectAssessment(events[i], config.policy));
    EXPECT_EQ(steps[i].assessment.status, expected_status[i]);
    EXPECT_TRUE(steps[i].assessment.withhold);
    EXPECT_FALSE(steps[i].assessment.accepted);
    EXPECT_EQ(events[i].world_snapshot_record().snapshot().snapshot_id(),
              expected_ids[i]);
  }
  EXPECT_EQ(steps[0].assessment.offending_kinds,
            (std::vector<std::string>{"bathymetry_reference"}));
  EXPECT_EQ(steps[1].assessment.offending_kinds,
            (std::vector<std::string>{"bathymetry_reference"}));
  EXPECT_EQ(steps[2].assessment.offending_kinds,
            (std::vector<std::string>{"current_field"}));
  EXPECT_EQ(
      steps[3].assessment.offending_kinds,
      (std::vector<std::string>{"bathymetry_reference", "current_field"}));
  EXPECT_EQ(steps[4].assessment.error, SnapshotPolicyError::kInvalidTimings);
  EXPECT_EQ(steps[5].assessment.offending_kinds,
            (std::vector<std::string>{"current_field"}));

  const WorldReplaySummary summary = adapter->Finish();
  EXPECT_EQ(summary.status, ReplayAdapterStatus::kOk);
  EXPECT_FALSE(summary.degraded);
  EXPECT_EQ(summary.stats.assessed, 6u);
  EXPECT_EQ(summary.stats.accepted, 0u);
  EXPECT_EQ(summary.stats.withheld, 6u);
  ExpectTrace(
      summary.trace,
      {{0, kIdIncomplete, SnapshotPolicyStatus::kIncomplete, true},
       {1, kIdStaleHorizon, SnapshotPolicyStatus::kStale, true},
       {2, kIdStaleNoTiming, SnapshotPolicyStatus::kStale, true},
       {3, kIdSkew, SnapshotPolicyStatus::kExcessiveSkew, true},
       {4, kIdDuplicateTimings, SnapshotPolicyStatus::kUnspecified, true},
       {5, kIdUnsetHorizon, SnapshotPolicyStatus::kStale, true}});
  EXPECT_EQ(summary.chain_digest, kChainRa7);
}

TEST(WorldReplayAdapterTest, RA8LossRecordIsAGapAndReplayContinues) {
  const std::vector<Component> both = {{"bathymetry_reference", 3},
                                       {"current_field", 5}};
  const std::vector<uint8_t> bytes =
      WriteOverflowEpisode({MetadataEvent(1, Seed42MarineText()),
                            WorldEvent(2, 7, both, GoodTimings()),
                            WorldEvent(3, 7, both, GoodTimings())},
                           {WorldEvent(4, 7, both, GoodTimings()),
                            WorldEvent(5, 7, both, GoodTimings())},
                           WorldEvent(6, 8, {{"bathymetry_reference", 4}},
                                      {{"bathymetry_reference", 300, 30000}}));
  std::unique_ptr<WorldReplayAdapter> adapter = MakeAdapter();
  ASSERT_NE(adapter, nullptr);
  const std::vector<WorldReplayStep> steps = FeedAll(adapter.get(), bytes);
  ASSERT_EQ(steps.size(), 5u);
  EXPECT_EQ(steps[3].status, ReplayAdapterStatus::kLossRecord);
  // Dropped events consume writer indices 3 and 4, the loss record is 5.
  EXPECT_EQ(steps[3].writer_index, 5u);
  EXPECT_FALSE(steps[3].assessed);
  EXPECT_EQ(steps[4].status, ReplayAdapterStatus::kOk);
  EXPECT_EQ(steps[4].writer_index, 6u);
  const WorldReplaySummary summary = adapter->Finish();
  EXPECT_EQ(summary.status, ReplayAdapterStatus::kOk);
  EXPECT_EQ(summary.stats.gaps, 1u);
  EXPECT_EQ(summary.stats.gap_dropped, 2u);
  const EpisodeReadResult read = ReadEpisode(bytes, EpisodeReaderConfig{});
  ASSERT_EQ(read.losses.size(), 1u);
  EXPECT_EQ(summary.stats.gap_dropped, read.losses[0].dropped_count);
  EXPECT_TRUE(summary.degraded);
  EXPECT_EQ(summary.stats.assessed, 3u);
  ExpectTrace(summary.trace,
              {{1, kIdA, SnapshotPolicyStatus::kFresh, false},
               {2, kIdA, SnapshotPolicyStatus::kFresh, false},
               {6, kIdB, SnapshotPolicyStatus::kPartial, false}});
}

TEST(WorldReplayAdapterTest, RA9OtherKindsAreIgnored) {
  const std::vector<uint8_t> bytes = WriteEpisode({
      SafetyEvent(1),
      TrajectoryEvent(2),
      InferenceEnvelopeEvent(3),
      InferenceResultEvent(4),
      DesiredMotionEvent(5),
      MeasurementLikeEvent(6),
      RecordedEvent(7, "intrinsic_proto.vehicle.BodyTwist", "\x0a\x00"),
      MetadataEvent(8, Seed42MarineText()),
  });
  std::unique_ptr<WorldReplayAdapter> adapter = MakeAdapter();
  ASSERT_NE(adapter, nullptr);
  const std::vector<WorldReplayStep> steps = FeedAll(adapter.get(), bytes);
  ASSERT_EQ(steps.size(), 8u);
  for (const WorldReplayStep& step : steps) {
    EXPECT_EQ(step.status, ReplayAdapterStatus::kIgnoredPayload);
    EXPECT_FALSE(step.assessed);
  }
  const WorldReplaySummary summary = adapter->Finish();
  EXPECT_EQ(summary.status, ReplayAdapterStatus::kOk);
  EXPECT_EQ(summary.stats.fed, 8u);
  EXPECT_EQ(summary.stats.ignored, 8u);
  EXPECT_EQ(summary.stats.assessed, 0u);
  EXPECT_TRUE(summary.trace.empty());
  EXPECT_FALSE(summary.degraded);
  EXPECT_EQ(summary.chain_digest, kEmptyChain);
}

TEST(WorldReplayAdapterTest, RA10EmptyEpisodeAndSingleEvent) {
  {
    const std::vector<uint8_t> bytes = WriteEpisode({});
    EXPECT_EQ(bytes.size(), kEpisodeFileHeaderBytes);
    std::unique_ptr<WorldReplayAdapter> adapter = MakeAdapter();
    ASSERT_NE(adapter, nullptr);
    EXPECT_TRUE(FeedAll(adapter.get(), bytes).empty());
    const WorldReplaySummary summary = adapter->Finish();
    EXPECT_EQ(summary.status, ReplayAdapterStatus::kOk);
    EXPECT_EQ(summary.chain_digest, kEmptyChain);
    EXPECT_EQ(summary.chain_digest,
              internal::EpisodeSha256Hex(std::string("v1\n")));
    EXPECT_TRUE(summary.trace.empty());
    EXPECT_FALSE(summary.degraded);
  }
  {
    const EpisodeEvent only =
        WorldEvent(1, 7, {{"bathymetry_reference", 3}, {"current_field", 5}},
                   GoodTimings());
    const std::vector<uint8_t> bytes = WriteEpisode({only});
    std::unique_ptr<WorldReplayAdapter> adapter = MakeAdapter();
    ASSERT_NE(adapter, nullptr);
    EpisodeStream stream(bytes);
    EpisodeReplayEvent event;
    ASSERT_TRUE(stream.Next(&event));
    const WorldReplayStep step = adapter->Feed(event);
    EXPECT_EQ(step.status, ReplayAdapterStatus::kOk);
    EXPECT_TRUE(step.assessed);
    EXPECT_EQ(step.assessment.status, SnapshotPolicyStatus::kFresh);

    const WorldReplaySummary first = adapter->Finish();
    const WorldReplayStep after = adapter->Feed(event);
    EXPECT_EQ(after.status, ReplayAdapterStatus::kFinished);
    EXPECT_EQ(after.writer_index, 0u);
    EXPECT_FALSE(after.assessed);
    const WorldReplaySummary second = adapter->Finish();
    ExpectSummaryEqual(first, second);
    EXPECT_EQ(second.stats.fed, 1u);
    ExpectTrace(second.trace, {{0, kIdA, SnapshotPolicyStatus::kFresh, false}});
    EXPECT_EQ(second.chain_digest,
              internal::EpisodeSha256Hex(std::string("v1\n") + kIdA + "\n"));
  }
}

TEST(WorldReplayAdapterTest, RA12InvalidConfigIsRejectedNeverClamped) {
  const std::vector<std::string> bad_digests = {
      std::string(63, 'a'),
      std::string(64, 'A'),
      std::string(65, 'a'),
      std::string("g") + std::string(63, 'a'),
  };
  for (const std::string& digest : bad_digests) {
    WorldReplayConfig config = DefaultConfig();
    config.compatibility.expected_config_digest = digest;
    ReplayAdapterStatus status = ReplayAdapterStatus::kOk;
    EXPECT_EQ(WorldReplayAdapter::Create(config, &status), nullptr) << digest;
    EXPECT_EQ(status, ReplayAdapterStatus::kInvalidConfig) << digest;
  }

  std::vector<world::WorldSnapshotSkewPolicy> bad_policies;
  world::WorldSnapshotSkewPolicy policy;
  policy.max_skew = {-1, 0};
  bad_policies.push_back(policy);
  policy = {};
  policy.max_skew = {0, 1000000000};
  bad_policies.push_back(policy);
  policy = {};
  policy.max_age = {0, -1};
  bad_policies.push_back(policy);
  policy = {};
  policy.required_kinds = {""};
  bad_policies.push_back(policy);
  policy = {};
  policy.optional_kinds = {"current_field", ""};
  bad_policies.push_back(policy);
  policy = {};
  policy.required_kinds = {"current_field"};
  policy.optional_kinds = {"current_field"};
  bad_policies.push_back(policy);
  for (const world::WorldSnapshotSkewPolicy& bad : bad_policies) {
    ASSERT_FALSE(world::snapshot_skew_internal::PolicyValid(bad));
    WorldReplayConfig config = DefaultConfig();
    config.policy = bad;
    ReplayAdapterStatus status = ReplayAdapterStatus::kOk;
    EXPECT_EQ(WorldReplayAdapter::Create(config, &status), nullptr);
    EXPECT_EQ(status, ReplayAdapterStatus::kInvalidConfig);
  }

  ReplayAdapterStatus status = ReplayAdapterStatus::kInvalidConfig;
  EXPECT_NE(WorldReplayAdapter::Create(WorldReplayConfig{}, &status), nullptr);
  EXPECT_EQ(status, ReplayAdapterStatus::kOk);
}

}  // namespace
}  // namespace intrinsic::episode
