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

// RA1 to RA6, RA8 to RA13 of lock id episode-replay-adapters-estimator-world-v1
// (#131), estimator side. Episodes are built in memory with EpisodeWriter,
// read with EpisodeReader and replayed with EpisodeReplayer in kStep. No
// fixture file is added.

#include "intrinsic_data/episode/estimator_replay_adapter.h"

#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <fstream>
#include <memory>
#include <optional>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include "absl/status/status.h"
#include "absl/types/span.h"
#include "gtest/gtest.h"
#include "intrinsic/episode/proto/episode_event.pb.h"
#include "intrinsic/estimation/estimator.pb.h"
#include "intrinsic/estimation/estimator_service.h"
#include "intrinsic/estimation/fake_estimator_service.h"
#include "intrinsic_data/episode/episode_event_policy.h"
#include "intrinsic_data/episode/episode_reader.h"
#include "intrinsic_data/episode/episode_replay.h"
#include "intrinsic_data/episode/episode_replay_adapter_common.h"
#include "intrinsic_data/episode/episode_writer.h"

namespace intrinsic::episode {
namespace {

using intrinsic_proto::episode::EpisodeEvent;
using intrinsic_proto::estimation::EstimatorConfig;
using intrinsic_proto::estimation::EstimatorResult;
using intrinsic_proto::estimation::GetStateResponse;
using intrinsic_proto::estimation::MeasurementEnvelope;
using intrinsic_proto::estimation::MeasurementKind;
using intrinsic_proto::estimation::RejectReason;

constexpr char kSeed42Digest[] =
    "93150ec8e5fd8f6dcc2d9cae50d84de9b89287b187edfa4a807b0da0e8eb5d6a";
constexpr char kOtherDigest[] =
    "bbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbb";
constexpr char kMeasurementType[] =
    "intrinsic_proto.estimation.MeasurementEnvelope";

// FNV-1a 64 state digests of FakeEstimatorService, configured with
// EstimatorConfig{clock_domain: "monotonic", seed: 42}. Ingest never changes
// the published pose, mode or epoch, but VehicleState.sources follows the
// per-source health, so the digest moves with it.
//   kInitDigest: after Initialize, no source seen.
//   kDvlHealthy: dvl_0 healthy, the only source.
//   kDvlUnhealthy: dvl_0 unhealthy (3 consecutive rejects), the only source.
//   kDvlImuHealthy: dvl_0 and imu_0 healthy.
constexpr uint64_t kInitDigest = 3988426051458637864ULL;
constexpr uint64_t kDvlHealthy = 15188102644976614422ULL;
constexpr uint64_t kDvlUnhealthy = 13330914255721379339ULL;
constexpr uint64_t kDvlImuHealthy = 10160911158528082483ULL;

// The #244 113-byte MeasurementEnvelope (DVL, BodyTwist), as listed in
// README.md "Replay payload fixtures".
constexpr uint8_t kGoldenMeasurement[113] = {
    0x0a, 0x38, 0x08, 0x01, 0x12, 0x0b, 0x08, 0xe4, 0xe2, 0xcf, 0xaa, 0x06,
    0x10, 0x80, 0xe5, 0x9a, 0x77, 0x1a, 0x0b, 0x08, 0xe4, 0xe2, 0xcf, 0xaa,
    0x06, 0x10, 0xc0, 0xe9, 0xd7, 0x77, 0x22, 0x05, 0x64, 0x76, 0x6c, 0x5f,
    0x30, 0x2a, 0x04, 0x62, 0x6f, 0x64, 0x79, 0x32, 0x09, 0x6d, 0x6f, 0x6e,
    0x6f, 0x74, 0x6f, 0x6e, 0x69, 0x63, 0x3a, 0x02, 0x08, 0x01, 0x12, 0x05,
    0x64, 0x76, 0x6c, 0x5f, 0x30, 0x18, 0x03, 0x22, 0x21, 0x69, 0x6e, 0x74,
    0x72, 0x69, 0x6e, 0x73, 0x69, 0x63, 0x5f, 0x70, 0x72, 0x6f, 0x74, 0x6f,
    0x2e, 0x76, 0x65, 0x68, 0x69, 0x63, 0x6c, 0x65, 0x2e, 0x42, 0x6f, 0x64,
    0x79, 0x54, 0x77, 0x69, 0x73, 0x74, 0x2a, 0x09, 0x09, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0xe0, 0x3f};

class MemorySink : public EpisodeSink {
 public:
  absl::Status Write(absl::Span<const uint8_t> bytes) override {
    output.insert(output.end(), bytes.begin(), bytes.end());
    return absl::OkStatus();
  }
  absl::Status Sync() override { return absl::OkStatus(); }

  std::vector<uint8_t> output;
};

// Wraps a FakeEstimatorService and counts every interface call.
class CountingEstimator : public estimation::EstimatorService {
 public:
  EstimatorResult Initialize(const EstimatorConfig& config) override {
    ++initialize_calls;
    return fake.Initialize(config);
  }
  EstimatorResult Ingest(const MeasurementEnvelope& envelope) override {
    ++ingest_calls;
    ingested.push_back(envelope);
    return fake.Ingest(envelope);
  }
  EstimatorResult Predict(const google::protobuf::Timestamp& to_time) override {
    ++predict_calls;
    return fake.Predict(to_time);
  }
  EstimatorResult Reset(
      const std::optional<intrinsic_proto::vehicle::VehicleState>& prior)
      override {
    ++reset_calls;
    return fake.Reset(prior);
  }
  GetStateResponse GetState() const override { return fake.GetState(); }

  estimation::FakeEstimatorService fake;
  int initialize_calls = 0;
  int ingest_calls = 0;
  int predict_calls = 0;
  int reset_calls = 0;
  std::vector<MeasurementEnvelope> ingested;
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

// The golden envelope with another source and times. source_time is `nanos`
// into `seconds`, receive_time is 1 ms later.
MeasurementEnvelope Measurement(
    const std::string& source_id, int64_t seconds, int32_t nanos,
    MeasurementKind kind = intrinsic_proto::estimation::MEASUREMENT_KIND_DVL) {
  MeasurementEnvelope envelope;
  EXPECT_TRUE(
      envelope.ParseFromArray(kGoldenMeasurement, sizeof(kGoldenMeasurement)));
  envelope.set_source_id(source_id);
  envelope.set_kind(kind);
  envelope.mutable_header()->mutable_source_time()->set_seconds(seconds);
  envelope.mutable_header()->mutable_source_time()->set_nanos(nanos);
  envelope.mutable_header()->mutable_receive_time()->set_seconds(seconds);
  envelope.mutable_header()->mutable_receive_time()->set_nanos(nanos + 1000000);
  return envelope;
}

EpisodeEvent RecordedEvent(uint64_t sequence, const std::string& type,
                           const std::string& bytes) {
  EpisodeEvent event = BaseEvent(sequence);
  event.mutable_recorded_message()->set_message_type(type);
  event.mutable_recorded_message()->set_message_bytes(bytes);
  return event;
}

EpisodeEvent MeasurementEvent(uint64_t sequence,
                              const MeasurementEnvelope& envelope) {
  return RecordedEvent(sequence, kMeasurementType,
                       envelope.SerializeAsString());
}

EpisodeEvent WorldEvent(uint64_t sequence) {
  EpisodeEvent event = BaseEvent(sequence);
  auto* snapshot = event.mutable_world_snapshot_record()->mutable_snapshot();
  snapshot->set_snapshot_id(kOtherDigest);
  snapshot->mutable_creation_time()->set_seconds(1700000100);
  snapshot->set_state_epoch(1);
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

EstimatorReplayConfig DefaultConfig() {
  EstimatorReplayConfig config;
  config.compatibility.expected_config_digest = kSeed42Digest;
  config.estimator_config.set_clock_domain("monotonic");
  config.estimator_config.set_seed(42);
  return config;
}

std::unique_ptr<EstimatorReplayAdapter> MakeAdapter(
    estimation::EstimatorService* service,
    const EstimatorReplayConfig& config = DefaultConfig()) {
  ReplayAdapterStatus status = ReplayAdapterStatus::kInvalidConfig;
  std::unique_ptr<EstimatorReplayAdapter> adapter =
      EstimatorReplayAdapter::Create(service, config, &status);
  EXPECT_EQ(status, ReplayAdapterStatus::kOk);
  EXPECT_NE(adapter, nullptr);
  return adapter;
}

std::vector<EstimatorReplayStep> FeedAll(EstimatorReplayAdapter* adapter,
                                         const std::vector<uint8_t>& bytes) {
  EpisodeStream stream(bytes);
  std::vector<EstimatorReplayStep> steps;
  EpisodeReplayEvent event;
  while (stream.Next(&event)) {
    steps.push_back(adapter->Feed(event));
  }
  return steps;
}

struct Row {
  uint64_t writer_index;
  bool ok;
  int reason;
  uint64_t epoch;
  uint64_t state_digest;
};

void ExpectTrace(const std::vector<EstimatorTraceEntry>& trace,
                 const std::vector<Row>& expected) {
  ASSERT_EQ(trace.size(), expected.size());
  for (size_t i = 0; i < expected.size(); ++i) {
    EXPECT_EQ(trace[i].writer_index, expected[i].writer_index) << i;
    EXPECT_EQ(trace[i].ok, expected[i].ok) << i;
    EXPECT_EQ(trace[i].reason, expected[i].reason) << i;
    EXPECT_EQ(trace[i].epoch, expected[i].epoch) << i;
    EXPECT_EQ(trace[i].state_digest, expected[i].state_digest) << i;
  }
}

constexpr int kNone = intrinsic_proto::estimation::REJECT_REASON_NONE;

// An accepted dvl_0 measurement with dvl_0 the only healthy source.
Row DvlAccepted(uint64_t writer_index) {
  return {writer_index, true, kNone, 1, kDvlHealthy};
}

bool StatsEqual(const EstimatorReplayStats& a, const EstimatorReplayStats& b) {
  return a.fed == b.fed && a.ingested == b.ingested &&
         a.accepted == b.accepted && a.rejected == b.rejected &&
         a.ignored == b.ignored &&
         a.ignored_message_type == b.ignored_message_type &&
         a.invalid_records == b.invalid_records && a.gaps == b.gaps &&
         a.gap_dropped == b.gap_dropped && a.after_latch == b.after_latch;
}

void ExpectSummaryEqual(const EstimatorReplaySummary& a,
                        const EstimatorReplaySummary& b) {
  EXPECT_EQ(a.status, b.status);
  EXPECT_EQ(a.missing_required_sources, b.missing_required_sources);
  EXPECT_EQ(a.degraded, b.degraded);
  EXPECT_EQ(a.final_state_digest, b.final_state_digest);
  EXPECT_TRUE(StatsEqual(a.stats, b.stats));
  ASSERT_EQ(a.trace.size(), b.trace.size());
  for (size_t i = 0; i < a.trace.size(); ++i) {
    EXPECT_EQ(a.trace[i].writer_index, b.trace[i].writer_index) << i;
    EXPECT_EQ(a.trace[i].ok, b.trace[i].ok) << i;
    EXPECT_EQ(a.trace[i].reason, b.trace[i].reason) << i;
    EXPECT_EQ(a.trace[i].epoch, b.trace[i].epoch) << i;
    EXPECT_EQ(a.trace[i].state_digest, b.trace[i].state_digest) << i;
  }
}

// RA1 and RA3 episode: metadata, 3 measurements from 2 sources, a World event.
std::vector<EpisodeEvent> NominalEvents() {
  return {
      MetadataEvent(1, Seed42MarineText()),
      MeasurementEvent(2, Measurement("dvl_0", 1700000100, 0)),
      MeasurementEvent(
          3, Measurement("imu_0", 1700000100, 100000000,
                         intrinsic_proto::estimation::MEASUREMENT_KIND_IMU)),
      MeasurementEvent(4, Measurement("dvl_0", 1700000100, 200000000)),
      WorldEvent(5),
  };
}

TEST(EstimatorReplayAdapterTest, FixtureMatchesTheGoldenAndThePinnedDigest) {
  const EpisodeEvent golden = ParseGolden("recorded_message");
  EXPECT_EQ(golden.recorded_message().message_type(), kMeasurementType);
  EXPECT_EQ(golden.recorded_message().message_bytes(),
            std::string(reinterpret_cast<const char*>(kGoldenMeasurement),
                        sizeof(kGoldenMeasurement)));
  EXPECT_EQ(sizeof(kGoldenMeasurement), 113u);
  EXPECT_EQ(ComputeConfigDigest(Seed42MarineText()), kSeed42Digest);
  const EpisodeEvent metadata = ParseGolden("metadata");
  EXPECT_EQ(metadata.config_digest(), kSeed42Digest);
}

TEST(EstimatorReplayAdapterTest, StatusNumbersArePermanent) {
  EXPECT_EQ(static_cast<int>(ReplayAdapterStatus::kOk), 0);
  EXPECT_EQ(static_cast<int>(ReplayAdapterStatus::kInvalidConfig), 1);
  EXPECT_EQ(static_cast<int>(ReplayAdapterStatus::kIgnoredPayload), 2);
  EXPECT_EQ(static_cast<int>(ReplayAdapterStatus::kIncompatibleSchema), 3);
  EXPECT_EQ(static_cast<int>(ReplayAdapterStatus::kConfigDigestMismatch), 4);
  EXPECT_EQ(static_cast<int>(ReplayAdapterStatus::kSnapshotIdMismatch), 5);
  EXPECT_EQ(static_cast<int>(ReplayAdapterStatus::kInvalidRecord), 6);
  EXPECT_EQ(static_cast<int>(ReplayAdapterStatus::kMissingRequiredStream), 7);
  EXPECT_EQ(static_cast<int>(ReplayAdapterStatus::kLatched), 8);
  EXPECT_EQ(static_cast<int>(ReplayAdapterStatus::kLossRecord), 9);
  EXPECT_EQ(static_cast<int>(ReplayAdapterStatus::kFinished), 10);
  EXPECT_STREQ(ReplayAdapterStatusName(ReplayAdapterStatus::kLatched),
               "latched");
}

TEST(EstimatorReplayAdapterTest, RA1NominalOneInitializeThreeIngests) {
  const std::vector<uint8_t> bytes = WriteEpisode(NominalEvents());
  CountingEstimator service;
  std::unique_ptr<EstimatorReplayAdapter> adapter = MakeAdapter(&service);
  ASSERT_NE(adapter, nullptr);
  EXPECT_EQ(service.initialize_calls, 1);

  const std::vector<EstimatorReplayStep> steps = FeedAll(adapter.get(), bytes);
  ASSERT_EQ(steps.size(), 5u);
  const ReplayAdapterStatus expected_status[] = {
      ReplayAdapterStatus::kIgnoredPayload, ReplayAdapterStatus::kOk,
      ReplayAdapterStatus::kOk, ReplayAdapterStatus::kOk,
      ReplayAdapterStatus::kIgnoredPayload};
  for (size_t i = 0; i < steps.size(); ++i) {
    EXPECT_EQ(steps[i].status, expected_status[i]) << i;
    EXPECT_EQ(steps[i].writer_index, i);
    EXPECT_EQ(steps[i].ingested, steps[i].status == ReplayAdapterStatus::kOk)
        << i;
  }

  EXPECT_EQ(service.initialize_calls, 1);
  EXPECT_EQ(service.ingest_calls, 3);
  EXPECT_EQ(service.predict_calls, 0);
  EXPECT_EQ(service.reset_calls, 0);
  ASSERT_EQ(service.ingested.size(), 3u);
  EXPECT_EQ(service.ingested[0].SerializeAsString(),
            Measurement("dvl_0", 1700000100, 0).SerializeAsString());

  const EstimatorReplaySummary summary = adapter->Finish();
  EXPECT_EQ(summary.status, ReplayAdapterStatus::kOk);
  EXPECT_FALSE(summary.degraded);
  EXPECT_EQ(summary.final_state_digest, kDvlImuHealthy);
  ExpectTrace(summary.trace, {{1, true, kNone, 1, kDvlHealthy},
                              {2, true, kNone, 1, kDvlImuHealthy},
                              {3, true, kNone, 1, kDvlImuHealthy}});
  EXPECT_EQ(summary.stats.fed, 5u);
  EXPECT_EQ(summary.stats.ingested, 3u);
  EXPECT_EQ(summary.stats.accepted, 3u);
  EXPECT_EQ(summary.stats.rejected, 0u);
  EXPECT_EQ(summary.stats.ignored, 2u);
  EXPECT_EQ(summary.stats.ignored_message_type, 0u);
  EXPECT_EQ(service.initialize_calls, 1);
  EXPECT_EQ(service.predict_calls, 0);
  EXPECT_EQ(service.reset_calls, 0);
}

TEST(EstimatorReplayAdapterTest, RA3DeterministicAndDigestMatchesTheFake) {
  const std::vector<uint8_t> bytes = WriteEpisode(NominalEvents());
  std::vector<EstimatorReplayStep> runs[2];
  EstimatorReplaySummary summaries[2];
  for (int run = 0; run < 2; ++run) {
    estimation::FakeEstimatorService fake;
    std::unique_ptr<EstimatorReplayAdapter> adapter = MakeAdapter(&fake);
    ASSERT_NE(adapter, nullptr);
    EpisodeStream stream(bytes);
    EpisodeReplayEvent event;
    while (stream.Next(&event)) {
      const EstimatorReplayStep step = adapter->Feed(event);
      if (step.ingested) {
        EXPECT_EQ(step.state_digest, fake.StateDigest());
      }
      runs[run].push_back(step);
    }
    summaries[run] = adapter->Finish();
    EXPECT_EQ(summaries[run].final_state_digest, fake.StateDigest());
  }
  ASSERT_EQ(runs[0].size(), runs[1].size());
  for (size_t i = 0; i < runs[0].size(); ++i) {
    EXPECT_EQ(runs[0][i].status, runs[1][i].status) << i;
    EXPECT_EQ(runs[0][i].writer_index, runs[1][i].writer_index) << i;
    EXPECT_EQ(runs[0][i].ingested, runs[1][i].ingested) << i;
    EXPECT_EQ(runs[0][i].result.SerializeAsString(),
              runs[1][i].result.SerializeAsString())
        << i;
    EXPECT_EQ(runs[0][i].state_digest, runs[1][i].state_digest) << i;
  }
  ExpectSummaryEqual(summaries[0], summaries[1]);
  for (const EstimatorReplaySummary& summary : summaries) {
    ExpectTrace(summary.trace, {{1, true, kNone, 1, kDvlHealthy},
                                {2, true, kNone, 1, kDvlImuHealthy},
                                {3, true, kNone, 1, kDvlImuHealthy}});
  }
}

TEST(EstimatorReplayAdapterTest,
     RA3DigestFollowsTheServiceThroughTheInterface) {
  const std::vector<uint8_t> bytes = WriteEpisode(NominalEvents());
  estimation::FakeEstimatorService fake;
  std::unique_ptr<EstimatorReplayAdapter> adapter = MakeAdapter(&fake);
  ASSERT_NE(adapter, nullptr);
  EpisodeStream stream(bytes);
  EpisodeReplayEvent event;
  std::vector<uint64_t> digests;
  while (stream.Next(&event)) {
    const EstimatorReplayStep step = adapter->Feed(event);
    if (step.ingested) {
      EXPECT_EQ(step.state_digest, fake.StateDigest());
      digests.push_back(step.state_digest);
      if (digests.size() == 1) {
        ASSERT_TRUE(fake.InjectFault());
      }
    }
  }
  ASSERT_EQ(digests.size(), 3u);
  EXPECT_NE(digests[0], digests[1]);
  EXPECT_EQ(digests[1], digests[2]);
  const EstimatorReplaySummary summary = adapter->Finish();
  EXPECT_EQ(summary.trace[1].reason,
            static_cast<int>(
                intrinsic_proto::estimation::REJECT_REASON_ESTIMATOR_FAULTED));
  EXPECT_EQ(summary.final_state_digest, fake.StateDigest());
}

struct BadEvent {
  const char* name;
  EpisodeEvent event;
  ReplayAdapterStatus expected;
};

std::vector<BadEvent> BadCompatibilityEvents() {
  std::vector<BadEvent> bad;
  const MeasurementEnvelope measurement = Measurement("dvl_0", 1700000101, 0);
  const std::string other_text = "marine_randomization_metadata_v1\nseed=43\n";

  EpisodeEvent event = MeasurementEvent(2, measurement);
  event.set_schema_version(2);
  bad.push_back({"measurement schema 2", event,
                 ReplayAdapterStatus::kIncompatibleSchema});
  event = SafetyEvent(2);
  event.set_schema_version(2);
  bad.push_back({"ignored kind schema 2", event,
                 ReplayAdapterStatus::kIncompatibleSchema});
  event = MetadataEvent(2, Seed42MarineText());
  event.set_schema_version(2);
  bad.push_back(
      {"metadata schema 2", event, ReplayAdapterStatus::kIncompatibleSchema});
  event = MeasurementEvent(2, measurement);
  event.set_schema_version(2);
  event.set_config_digest(kOtherDigest);
  bad.push_back({"schema is checked before the digest", event,
                 ReplayAdapterStatus::kIncompatibleSchema});

  event = MeasurementEvent(2, measurement);
  event.set_config_digest(kOtherDigest);
  bad.push_back({"measurement digest", event,
                 ReplayAdapterStatus::kConfigDigestMismatch});
  event = SafetyEvent(2);
  event.set_config_digest(kOtherDigest);
  bad.push_back({"ignored kind digest", event,
                 ReplayAdapterStatus::kConfigDigestMismatch});
  bad.push_back({"metadata with its own passing digest",
                 MetadataEvent(2, other_text),
                 ReplayAdapterStatus::kConfigDigestMismatch});
  return bad;
}

TEST(EstimatorReplayAdapterTest, RA4CompatibilityLatches) {
  for (const BadEvent& bad : BadCompatibilityEvents()) {
    SCOPED_TRACE(bad.name);
    const std::vector<uint8_t> bytes = WriteEpisode(
        {MeasurementEvent(1, Measurement("dvl_0", 1700000100, 0)), bad.event,
         MeasurementEvent(3, Measurement("dvl_0", 1700000102, 0))});
    estimation::FakeEstimatorService fake;
    std::unique_ptr<EstimatorReplayAdapter> adapter = MakeAdapter(&fake);
    ASSERT_NE(adapter, nullptr);
    const std::vector<EstimatorReplayStep> steps =
        FeedAll(adapter.get(), bytes);
    ASSERT_EQ(steps.size(), 3u);
    EXPECT_EQ(steps[0].status, ReplayAdapterStatus::kOk);
    EXPECT_EQ(steps[1].status, bad.expected);
    EXPECT_FALSE(steps[1].ingested);
    EXPECT_EQ(steps[2].status, ReplayAdapterStatus::kLatched);
    EXPECT_FALSE(steps[2].ingested);
    const EstimatorReplaySummary summary = adapter->Finish();
    EXPECT_EQ(summary.status, bad.expected);
    EXPECT_EQ(summary.stats.after_latch, 1u);
    EXPECT_EQ(summary.stats.ingested, 1u);
    ExpectTrace(summary.trace, {DvlAccepted(0)});
  }
}

TEST(EstimatorReplayAdapterTest, RA4LatchedStatusBeatsMissingRequiredSources) {
  const std::vector<EpisodeEvent> events = {BadCompatibilityEvents()[4].event};
  const std::vector<uint8_t> bytes = WriteEpisode(events);
  estimation::FakeEstimatorService fake;
  EstimatorReplayConfig config = DefaultConfig();
  config.required_sources = {"dvl_0"};
  std::unique_ptr<EstimatorReplayAdapter> adapter = MakeAdapter(&fake, config);
  ASSERT_NE(adapter, nullptr);
  FeedAll(adapter.get(), bytes);
  const EstimatorReplaySummary summary = adapter->Finish();
  EXPECT_EQ(summary.status, ReplayAdapterStatus::kConfigDigestMismatch);
}

TEST(EstimatorReplayAdapterTest, RA4EmptyEventDigestIsAccepted) {
  EpisodeEvent first = MeasurementEvent(1, Measurement("dvl_0", 1700000100, 0));
  first.clear_config_digest();
  EpisodeEvent second =
      MeasurementEvent(2, Measurement("dvl_0", 1700000101, 0));
  second.clear_config_digest();
  const std::vector<uint8_t> bytes = WriteEpisode({first, second});
  estimation::FakeEstimatorService fake;
  std::unique_ptr<EstimatorReplayAdapter> adapter = MakeAdapter(&fake);
  ASSERT_NE(adapter, nullptr);
  const std::vector<EstimatorReplayStep> steps = FeedAll(adapter.get(), bytes);
  ASSERT_EQ(steps.size(), 2u);
  EXPECT_EQ(steps[0].status, ReplayAdapterStatus::kOk);
  EXPECT_EQ(steps[1].status, ReplayAdapterStatus::kOk);
  const EstimatorReplaySummary summary = adapter->Finish();
  EXPECT_EQ(summary.status, ReplayAdapterStatus::kOk);
  ExpectTrace(summary.trace, {DvlAccepted(0), DvlAccepted(1)});
}

TEST(EstimatorReplayAdapterTest, RA4EmptyExpectedDigestDisablesTheCheck) {
  EpisodeEvent first = MeasurementEvent(1, Measurement("dvl_0", 1700000100, 0));
  first.set_config_digest(kOtherDigest);
  const std::vector<uint8_t> bytes = WriteEpisode(
      {first, MetadataEvent(2, "marine_randomization_metadata_v1\nseed=43\n"),
       MeasurementEvent(3, Measurement("dvl_0", 1700000101, 0))});
  estimation::FakeEstimatorService fake;
  EstimatorReplayConfig config = DefaultConfig();
  config.compatibility.expected_config_digest.clear();
  std::unique_ptr<EstimatorReplayAdapter> adapter = MakeAdapter(&fake, config);
  ASSERT_NE(adapter, nullptr);
  const std::vector<EstimatorReplayStep> steps = FeedAll(adapter.get(), bytes);
  ASSERT_EQ(steps.size(), 3u);
  EXPECT_EQ(steps[0].status, ReplayAdapterStatus::kOk);
  EXPECT_EQ(steps[1].status, ReplayAdapterStatus::kIgnoredPayload);
  EXPECT_EQ(steps[2].status, ReplayAdapterStatus::kOk);
  const EstimatorReplaySummary summary = adapter->Finish();
  EXPECT_EQ(summary.status, ReplayAdapterStatus::kOk);
  ExpectTrace(summary.trace, {DvlAccepted(0), DvlAccepted(2)});
}

TEST(EstimatorReplayAdapterTest, RA6RequiredSources) {
  const std::vector<uint8_t> bytes =
      WriteEpisode({MeasurementEvent(1, Measurement("dvl_0", 1700000100, 0))});
  {
    estimation::FakeEstimatorService fake;
    EstimatorReplayConfig config = DefaultConfig();
    config.required_sources = {"zzz", "imu_0", "dvl_0"};
    std::unique_ptr<EstimatorReplayAdapter> adapter =
        MakeAdapter(&fake, config);
    ASSERT_NE(adapter, nullptr);
    FeedAll(adapter.get(), bytes);
    const EstimatorReplaySummary summary = adapter->Finish();
    EXPECT_EQ(summary.status, ReplayAdapterStatus::kMissingRequiredStream);
    EXPECT_EQ(summary.missing_required_sources,
              (std::vector<std::string>{"imu_0", "zzz"}));
    EXPECT_FALSE(summary.degraded);
    ExpectTrace(summary.trace, {DvlAccepted(0)});
  }
  {
    // Absent sources that are not required are fine.
    estimation::FakeEstimatorService fake;
    EstimatorReplayConfig config = DefaultConfig();
    config.required_sources = {"dvl_0"};
    std::unique_ptr<EstimatorReplayAdapter> adapter =
        MakeAdapter(&fake, config);
    ASSERT_NE(adapter, nullptr);
    FeedAll(adapter.get(), bytes);
    const EstimatorReplaySummary summary = adapter->Finish();
    EXPECT_EQ(summary.status, ReplayAdapterStatus::kOk);
    EXPECT_TRUE(summary.missing_required_sources.empty());
    ExpectTrace(summary.trace, {DvlAccepted(0)});
  }
  {
    // A required source that only appears as a reject counts as seen.
    MeasurementEnvelope rejected =
        Measurement("imu_0", 1700000100, 0,
                    intrinsic_proto::estimation::MEASUREMENT_KIND_IMU);
    rejected.mutable_header()->set_clock_domain("utc");
    const std::vector<uint8_t> reject_bytes =
        WriteEpisode({MeasurementEvent(1, Measurement("dvl_0", 1700000100, 0)),
                      MeasurementEvent(2, rejected)});
    estimation::FakeEstimatorService fake;
    EstimatorReplayConfig config = DefaultConfig();
    config.required_sources = {"imu_0", "dvl_0"};
    std::unique_ptr<EstimatorReplayAdapter> adapter =
        MakeAdapter(&fake, config);
    ASSERT_NE(adapter, nullptr);
    FeedAll(adapter.get(), reject_bytes);
    const EstimatorReplaySummary summary = adapter->Finish();
    EXPECT_EQ(summary.status, ReplayAdapterStatus::kOk);
    EXPECT_TRUE(summary.missing_required_sources.empty());
    EXPECT_EQ(summary.stats.rejected, 1u);
    ExpectTrace(
        summary.trace,
        {DvlAccepted(0),
         {1, false,
          static_cast<int>(
              intrinsic_proto::estimation::REJECT_REASON_CLOCK_DOMAIN_MISMATCH),
          1, kDvlHealthy}});
  }
}

TEST(EstimatorReplayAdapterTest, RA8LossRecordIsAGapAndReplayContinues) {
  const std::vector<uint8_t> bytes = WriteOverflowEpisode(
      {MetadataEvent(1, Seed42MarineText()),
       MeasurementEvent(2, Measurement("dvl_0", 1700000100, 0)),
       MeasurementEvent(3, Measurement("dvl_0", 1700000101, 0))},
      {MeasurementEvent(4, Measurement("dvl_0", 1700000102, 0)),
       MeasurementEvent(5, Measurement("dvl_0", 1700000103, 0))},
      MeasurementEvent(6, Measurement("dvl_0", 1700000104, 0)));
  CountingEstimator service;
  std::unique_ptr<EstimatorReplayAdapter> adapter = MakeAdapter(&service);
  ASSERT_NE(adapter, nullptr);
  const std::vector<EstimatorReplayStep> steps = FeedAll(adapter.get(), bytes);
  ASSERT_EQ(steps.size(), 5u);
  EXPECT_EQ(steps[3].status, ReplayAdapterStatus::kLossRecord);
  // Dropped events consume writer indices 3 and 4, the loss record is 5.
  EXPECT_EQ(steps[3].writer_index, 5u);
  EXPECT_FALSE(steps[3].ingested);
  EXPECT_EQ(steps[4].status, ReplayAdapterStatus::kOk);
  EXPECT_EQ(steps[4].writer_index, 6u);
  const EstimatorReplaySummary summary = adapter->Finish();
  EXPECT_EQ(summary.status, ReplayAdapterStatus::kOk);
  EXPECT_EQ(summary.stats.gaps, 1u);
  EXPECT_EQ(summary.stats.gap_dropped, 2u);
  const EpisodeReadResult read = ReadEpisode(bytes, EpisodeReaderConfig{});
  ASSERT_EQ(read.losses.size(), 1u);
  EXPECT_EQ(summary.stats.gap_dropped, read.losses[0].dropped_count);
  EXPECT_TRUE(summary.degraded);
  EXPECT_EQ(summary.stats.ingested, 3u);
  ExpectTrace(summary.trace, {DvlAccepted(1), DvlAccepted(2), DvlAccepted(6)});
  EXPECT_EQ(service.ingest_calls, 3);
  EXPECT_EQ(service.predict_calls, 0);
  EXPECT_EQ(service.reset_calls, 0);
  EXPECT_EQ(service.initialize_calls, 1);
}

TEST(EstimatorReplayAdapterTest, RA9OtherKindsAndOtherMessageTypesAreIgnored) {
  const std::vector<uint8_t> bytes = WriteEpisode({
      SafetyEvent(1),
      TrajectoryEvent(2),
      InferenceEnvelopeEvent(3),
      InferenceResultEvent(4),
      DesiredMotionEvent(5),
      WorldEvent(6),
      RecordedEvent(7, "intrinsic_proto.vehicle.BodyTwist", "\x0a\x00"),
      RecordedEvent(8, "", ""),
  });
  CountingEstimator service;
  std::unique_ptr<EstimatorReplayAdapter> adapter = MakeAdapter(&service);
  ASSERT_NE(adapter, nullptr);
  const std::vector<EstimatorReplayStep> steps = FeedAll(adapter.get(), bytes);
  ASSERT_EQ(steps.size(), 8u);
  for (const EstimatorReplayStep& step : steps) {
    EXPECT_EQ(step.status, ReplayAdapterStatus::kIgnoredPayload);
    EXPECT_FALSE(step.ingested);
  }
  const EstimatorReplaySummary summary = adapter->Finish();
  EXPECT_EQ(summary.status, ReplayAdapterStatus::kOk);
  EXPECT_EQ(summary.stats.fed, 8u);
  EXPECT_EQ(summary.stats.ignored, 6u);
  EXPECT_EQ(summary.stats.ignored_message_type, 2u);
  EXPECT_EQ(summary.stats.ingested, 0u);
  EXPECT_TRUE(summary.trace.empty());
  EXPECT_FALSE(summary.degraded);
  EXPECT_EQ(service.ingest_calls, 0);
}

TEST(EstimatorReplayAdapterTest, RA10EmptyEpisode) {
  const std::vector<uint8_t> bytes = WriteEpisode({});
  EXPECT_EQ(bytes.size(), kEpisodeFileHeaderBytes);
  estimation::FakeEstimatorService fake;
  std::unique_ptr<EstimatorReplayAdapter> adapter = MakeAdapter(&fake);
  ASSERT_NE(adapter, nullptr);
  EXPECT_TRUE(FeedAll(adapter.get(), bytes).empty());
  const EstimatorReplaySummary summary = adapter->Finish();
  EXPECT_EQ(summary.status, ReplayAdapterStatus::kOk);
  EXPECT_EQ(summary.final_state_digest, kInitDigest);
  EXPECT_EQ(summary.final_state_digest, fake.StateDigest());
  EXPECT_TRUE(summary.trace.empty());
  EXPECT_EQ(summary.stats.fed, 0u);

  estimation::FakeEstimatorService required_fake;
  EstimatorReplayConfig config = DefaultConfig();
  config.required_sources = {"dvl_0"};
  std::unique_ptr<EstimatorReplayAdapter> required =
      MakeAdapter(&required_fake, config);
  ASSERT_NE(required, nullptr);
  EXPECT_EQ(required->Finish().status,
            ReplayAdapterStatus::kMissingRequiredStream);
}

TEST(EstimatorReplayAdapterTest, RA10SingleEventFinishAndFeedAfterFinish) {
  const std::vector<uint8_t> bytes =
      WriteEpisode({MeasurementEvent(1, Measurement("dvl_0", 1700000100, 0))});
  estimation::FakeEstimatorService fake;
  std::unique_ptr<EstimatorReplayAdapter> adapter = MakeAdapter(&fake);
  ASSERT_NE(adapter, nullptr);
  EpisodeStream stream(bytes);
  EpisodeReplayEvent event;
  ASSERT_TRUE(stream.Next(&event));
  const EstimatorReplayStep step = adapter->Feed(event);
  EXPECT_EQ(step.status, ReplayAdapterStatus::kOk);
  EXPECT_TRUE(step.ingested);
  EXPECT_TRUE(step.result.ok());
  EXPECT_EQ(step.result.estimator_epoch(), 1u);

  const EstimatorReplaySummary first = adapter->Finish();
  const EstimatorReplayStep after = adapter->Feed(event);
  EXPECT_EQ(after.status, ReplayAdapterStatus::kFinished);
  EXPECT_EQ(after.writer_index, 0u);
  EXPECT_FALSE(after.ingested);
  const EstimatorReplaySummary second = adapter->Finish();
  ExpectSummaryEqual(first, second);
  EXPECT_EQ(second.stats.fed, 1u);
  EXPECT_EQ(second.stats.after_latch, 0u);
  ExpectTrace(second.trace, {DvlAccepted(0)});
}

TEST(EstimatorReplayAdapterTest, RA11EstimatorRejectsAreDataNotErrors) {
  MeasurementEnvelope clock_mismatch = Measurement("dvl_0", 1700000103, 0);
  clock_mismatch.mutable_header()->set_clock_domain("utc");
  const std::vector<uint8_t> bytes = WriteEpisode({
      MeasurementEvent(1, Measurement("dvl_0", 1700000100, 0)),
      MeasurementEvent(2, Measurement("dvl_0", 1700000099, 0)),
      MeasurementEvent(3, Measurement("dvl_0", 1700000100, 0)),
      MeasurementEvent(4, clock_mismatch),
      MeasurementEvent(5, Measurement("dvl_0", 1700000101, 0)),
  });
  CountingEstimator service;
  std::unique_ptr<EstimatorReplayAdapter> adapter = MakeAdapter(&service);
  ASSERT_NE(adapter, nullptr);
  const std::vector<EstimatorReplayStep> steps = FeedAll(adapter.get(), bytes);
  ASSERT_EQ(steps.size(), 5u);
  const RejectReason expected_reason[] = {
      intrinsic_proto::estimation::REJECT_REASON_NONE,
      intrinsic_proto::estimation::REJECT_REASON_OUT_OF_ORDER,
      intrinsic_proto::estimation::REJECT_REASON_DUPLICATE_TIMESTAMP,
      intrinsic_proto::estimation::REJECT_REASON_CLOCK_DOMAIN_MISMATCH,
      intrinsic_proto::estimation::REJECT_REASON_NONE};
  for (size_t i = 0; i < steps.size(); ++i) {
    EXPECT_EQ(steps[i].status, ReplayAdapterStatus::kOk) << i;
    EXPECT_TRUE(steps[i].ingested) << i;
    EXPECT_EQ(steps[i].result.reason(), expected_reason[i]) << i;
    EXPECT_EQ(
        steps[i].result.ok(),
        expected_reason[i] == intrinsic_proto::estimation::REJECT_REASON_NONE)
        << i;
  }
  const EstimatorReplaySummary summary = adapter->Finish();
  EXPECT_EQ(summary.status, ReplayAdapterStatus::kOk);
  EXPECT_FALSE(summary.degraded);
  EXPECT_EQ(summary.stats.ingested, 5u);
  EXPECT_EQ(summary.stats.accepted, 2u);
  EXPECT_EQ(summary.stats.rejected, 3u);
  ExpectTrace(
      summary.trace,
      {{0, true, kNone, 1, kDvlHealthy},
       {1, false,
        static_cast<int>(
            intrinsic_proto::estimation::REJECT_REASON_OUT_OF_ORDER),
        1, kDvlHealthy},
       {2, false,
        static_cast<int>(
            intrinsic_proto::estimation::REJECT_REASON_DUPLICATE_TIMESTAMP),
        1, kDvlHealthy},
       {3, false,
        static_cast<int>(
            intrinsic_proto::estimation::REJECT_REASON_CLOCK_DOMAIN_MISMATCH),
        1, kDvlUnhealthy},
       {4, true, kNone, 1, kDvlHealthy}});
  EXPECT_EQ(service.ingest_calls, 5);
}

TEST(EstimatorReplayAdapterTest, RA12InvalidConfigIsRejectedNeverClamped) {
  const std::vector<std::string> bad_digests = {
      std::string(63, 'a'),
      std::string(64, 'A'),
      std::string(65, 'a'),
      std::string("g") + std::string(63, 'a'),
  };
  for (const std::string& digest : bad_digests) {
    CountingEstimator service;
    EstimatorReplayConfig config = DefaultConfig();
    config.compatibility.expected_config_digest = digest;
    ReplayAdapterStatus status = ReplayAdapterStatus::kOk;
    EXPECT_EQ(EstimatorReplayAdapter::Create(&service, config, &status),
              nullptr)
        << digest;
    EXPECT_EQ(status, ReplayAdapterStatus::kInvalidConfig) << digest;
    EXPECT_EQ(service.initialize_calls, 0) << digest;
  }

  {
    ReplayAdapterStatus status = ReplayAdapterStatus::kOk;
    EXPECT_EQ(EstimatorReplayAdapter::Create(nullptr, DefaultConfig(), &status),
              nullptr);
    EXPECT_EQ(status, ReplayAdapterStatus::kInvalidConfig);
  }

  const std::vector<std::vector<std::string>> bad_sources = {
      {""}, {"dvl_0", ""}, {"dvl_0", "dvl_0"}, {"a", "b", "a"}};
  for (const auto& sources : bad_sources) {
    CountingEstimator service;
    EstimatorReplayConfig config = DefaultConfig();
    config.required_sources = sources;
    ReplayAdapterStatus status = ReplayAdapterStatus::kOk;
    EXPECT_EQ(EstimatorReplayAdapter::Create(&service, config, &status),
              nullptr);
    EXPECT_EQ(status, ReplayAdapterStatus::kInvalidConfig);
    EXPECT_EQ(service.initialize_calls, 0);
  }

  {
    CountingEstimator service;
    EstimatorReplayConfig config = DefaultConfig();
    config.estimator_config.mutable_max_future_skew()->set_seconds(-1);
    ReplayAdapterStatus status = ReplayAdapterStatus::kOk;
    EstimatorResult init_result;
    EXPECT_EQ(
        EstimatorReplayAdapter::Create(&service, config, &status, &init_result),
        nullptr);
    EXPECT_EQ(status, ReplayAdapterStatus::kInvalidConfig);
    EXPECT_EQ(service.initialize_calls, 1);
    EXPECT_FALSE(init_result.ok());
    EXPECT_EQ(init_result.reason(),
              intrinsic_proto::estimation::REJECT_REASON_INVALID_CONFIG);
  }

  {
    CountingEstimator service;
    ReplayAdapterStatus status = ReplayAdapterStatus::kInvalidConfig;
    EstimatorResult init_result;
    std::unique_ptr<EstimatorReplayAdapter> adapter =
        EstimatorReplayAdapter::Create(&service, DefaultConfig(), &status,
                                       &init_result);
    ASSERT_NE(adapter, nullptr);
    EXPECT_EQ(status, ReplayAdapterStatus::kOk);
    EXPECT_TRUE(init_result.ok());
    EXPECT_EQ(init_result.estimator_epoch(), 1u);
    EXPECT_EQ(service.initialize_calls, 1);
  }
}

TEST(EstimatorReplayAdapterTest, RA13UnparsableMeasurementBytes) {
  const std::vector<uint8_t> bytes = WriteEpisode({
      RecordedEvent(1, kMeasurementType, std::string(12, '\xff')),
      MeasurementEvent(2, Measurement("dvl_0", 1700000100, 0)),
  });
  CountingEstimator service;
  std::unique_ptr<EstimatorReplayAdapter> adapter = MakeAdapter(&service);
  ASSERT_NE(adapter, nullptr);
  const std::vector<EstimatorReplayStep> steps = FeedAll(adapter.get(), bytes);
  ASSERT_EQ(steps.size(), 2u);
  EXPECT_EQ(steps[0].status, ReplayAdapterStatus::kInvalidRecord);
  EXPECT_FALSE(steps[0].ingested);
  EXPECT_EQ(steps[1].status, ReplayAdapterStatus::kOk);
  const EstimatorReplaySummary summary = adapter->Finish();
  EXPECT_EQ(summary.status, ReplayAdapterStatus::kOk);
  EXPECT_TRUE(summary.degraded);
  EXPECT_EQ(summary.stats.invalid_records, 1u);
  EXPECT_EQ(summary.stats.ingested, 1u);
  ExpectTrace(summary.trace, {DvlAccepted(1)});
  EXPECT_EQ(service.ingest_calls, 1);
}

}  // namespace
}  // namespace intrinsic::episode
