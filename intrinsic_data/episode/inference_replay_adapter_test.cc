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

// RB3, RB4 to RB6 and RB8 to RB14 of lock id
// episode-replay-adapters-planning-safety-inference-v1 (#132), inference side.
// Episodes are built in memory with EpisodeWriter, read with EpisodeReader and
// replayed with EpisodeReplayer in kStep. No fixture file is added. The
// #127/#244 goldens episode_event_v1_{inference_envelope,inference_result} are
// read unmodified. No InferenceClient, queue or ReplayWorker is linked, so
// none can be called.

#include "intrinsic_data/episode/inference_replay_adapter.h"

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
#include "google/protobuf/io/coded_stream.h"
#include "google/protobuf/io/zero_copy_stream_impl_lite.h"
#include "google/protobuf/timestamp.pb.h"
#include "gtest/gtest.h"
#include "intrinsic/episode/proto/episode_event.pb.h"
#include "intrinsic_data/episode/episode_event_policy.h"
#include "intrinsic_data/episode/episode_reader.h"
#include "intrinsic_data/episode/episode_replay.h"
#include "intrinsic_data/episode/episode_replay_adapter_common.h"
#include "intrinsic_data/episode/episode_writer.h"
#include "intrinsic_inference/envelope/inference_envelope_contract_policy.h"

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

// Payload SHA-256 of the deterministic serialization of the golden
// inference_result and the chain of the golden pair, computed independently
// (see the PR description) and committed as constants.
constexpr char kShaGoldenResult[] =
    "87d11b1c069efa01ea315417ef2984767f7f469df9912d91b178b632ead3fd97";
constexpr char kInferenceChainRb3[] =
    "41084a642f6f5715e47c6e0de080c2e323c61e8c7feed1e4170f959f845f3ee6";
constexpr char kChainRb8Late[] =
    "c2e1e353bc661d57ef8a218b0f0eae26dc3742488abf92026b923b3b23e6636b";
constexpr char kChainRb9[] =
    "7e128cf71be5f0ae147061a924ef53cf9dd80aeb3f4e7f62da2fce3acf131049";
constexpr char kChainRb10[] =
    "ad8c9e280d76121e414bab2987860a88725f92643d01cae0962498ad70bcaf46";
constexpr char kChainRb11[] =
    "0c5a4bb874801f05cb5b552bf1c338aa1e34b12c3810ec7e2f2cefbd1bd33518";
constexpr char kGoldenSnapshot[] =
    "0123456789abcdef0123456789abcdef0123456789abcdef0123456789abcdef";
constexpr char kGoldenModelDigest[] =
    "sha256:efefefefefefefefefefefefefefefefefefefefefefefefefefefefefefefef";

InferenceReplayConfig DefaultConfig() {
  InferenceReplayConfig config;
  config.compatibility.expected_config_digest = kSeed42Digest;
  return config;
}

std::unique_ptr<InferenceReplayAdapter> MakeAdapter(
    const InferenceReplayConfig& config = DefaultConfig()) {
  ReplayAdapterStatus status = ReplayAdapterStatus::kInvalidConfig;
  std::unique_ptr<InferenceReplayAdapter> adapter =
      InferenceReplayAdapter::Create(config, &status);
  EXPECT_EQ(status, ReplayAdapterStatus::kOk);
  EXPECT_NE(adapter, nullptr);
  return adapter;
}

std::vector<InferenceReplayStep> FeedAll(InferenceReplayAdapter* adapter,
                                         const std::vector<uint8_t>& bytes) {
  EpisodeStream stream(bytes);
  std::vector<InferenceReplayStep> steps;
  EpisodeReplayEvent event;
  while (stream.Next(&event)) {
    steps.push_back(adapter->Feed(event));
  }
  return steps;
}

// Inner times are milliseconds after 1700000000 s, the golden's timebase.
void SetMs(google::protobuf::Timestamp* time, int64_t ms) {
  time->set_seconds(1700000000 + ms / 1000);
  time->set_nanos(static_cast<int32_t>((ms % 1000) * 1000000));
}

struct Opt {
  int64_t receive_ms = 1;
  int64_t deadline_ms = 500;
  // Part of the pairing key.
  uint64_t epoch = 42;
  std::string provenance_digest = kGoldenModelDigest;
  std::string output_digest = "sha256:cd";
};

// The golden envelope (sequence 3) with the inner times, key and provenance
// digest of `opt`. Valid.
EpisodeEvent EnvelopeAt(uint64_t sequence, const Opt& opt) {
  EpisodeEvent event = ParseGolden("inference_envelope");
  event.mutable_header()->set_sequence(sequence);
  auto* envelope = event.mutable_inference_envelope();
  SetMs(envelope->mutable_header()->mutable_receive_time(), opt.receive_ms);
  SetMs(envelope->mutable_deadline(), opt.deadline_ms);
  envelope->set_state_epoch(opt.epoch);
  envelope->mutable_provenance()->set_digest(opt.provenance_digest);
  return event;
}

// The golden result (sequence 4), likewise. Valid.
EpisodeEvent ResultAt(uint64_t sequence, const Opt& opt) {
  EpisodeEvent event = ParseGolden("inference_result");
  event.mutable_header()->set_sequence(sequence);
  auto* result = event.mutable_inference_result();
  SetMs(result->mutable_header()->mutable_receive_time(), opt.receive_ms);
  SetMs(result->mutable_deadline(), opt.deadline_ms);
  result->set_state_epoch(opt.epoch);
  result->mutable_provenance()->set_digest(opt.provenance_digest);
  result->set_output_digest(opt.output_digest);
  return event;
}

std::string ResultSha(const EpisodeEvent& event) {
  std::string bytes;
  {
    google::protobuf::io::StringOutputStream stream(&bytes);
    google::protobuf::io::CodedOutputStream coded(&stream);
    coded.SetSerializationDeterministic(true);
    event.inference_result().SerializeToCodedStream(&coded);
  }
  return internal::EpisodeSha256Hex(bytes);
}

std::string Chain(const std::vector<std::string>& shas) {
  std::string input = "v1\n";
  for (const std::string& sha : shas) {
    input += sha + "\n";
  }
  return internal::EpisodeSha256Hex(input);
}

struct Out {
  uint64_t writer_index;
  InferenceOutcome outcome;
  bool expired;
  bool mismatch;
  uint64_t envelope_writer_index;
  std::string world_snapshot_id;
};

void ExpectOutcomes(const std::vector<InferenceOutcomeEntry>& actual,
                    const std::vector<Out>& expected) {
  ASSERT_EQ(actual.size(), expected.size());
  for (size_t i = 0; i < expected.size(); ++i) {
    EXPECT_EQ(actual[i].writer_index, expected[i].writer_index) << i;
    EXPECT_EQ(actual[i].outcome, expected[i].outcome) << i;
    EXPECT_EQ(actual[i].expired, expected[i].expired) << i;
    EXPECT_EQ(actual[i].model_digest_mismatch, expected[i].mismatch) << i;
    EXPECT_EQ(actual[i].envelope_writer_index,
              expected[i].envelope_writer_index)
        << i;
    EXPECT_EQ(actual[i].world_snapshot_id, expected[i].world_snapshot_id) << i;
  }
}

struct Rec {
  uint64_t writer_index;
  PayloadKind kind;
  int error;
  bool accepted;
};

void ExpectTrace(const std::vector<InferenceTraceEntry>& trace,
                 const std::vector<Rec>& expected) {
  ASSERT_EQ(trace.size(), expected.size());
  for (size_t i = 0; i < expected.size(); ++i) {
    EXPECT_EQ(trace[i].writer_index, expected[i].writer_index) << i;
    EXPECT_EQ(trace[i].payload_kind, expected[i].kind) << i;
    EXPECT_EQ(trace[i].error, expected[i].error) << i;
    EXPECT_EQ(trace[i].accepted, expected[i].accepted) << i;
  }
}

bool StatsEqual(const InferenceReplayStats& a, const InferenceReplayStats& b) {
  return a.fed == b.fed && a.envelopes == b.envelopes &&
         a.envelopes_accepted == b.envelopes_accepted &&
         a.results == b.results && a.results_accepted == b.results_accepted &&
         a.complete == b.complete && a.expired == b.expired &&
         a.orphans == b.orphans && a.missing_expired == b.missing_expired &&
         a.missing_unresolved == b.missing_unresolved &&
         a.model_digest_mismatches == b.model_digest_mismatches &&
         a.ignored == b.ignored && a.invalid_records == b.invalid_records &&
         a.gaps == b.gaps && a.gap_dropped == b.gap_dropped &&
         a.after_latch == b.after_latch;
}

std::vector<Out> OutsOf(const std::vector<InferenceOutcomeEntry>& entries) {
  std::vector<Out> outs;
  for (const InferenceOutcomeEntry& entry : entries) {
    outs.push_back({entry.writer_index, entry.outcome, entry.expired,
                    entry.model_digest_mismatch, entry.envelope_writer_index,
                    entry.world_snapshot_id});
  }
  return outs;
}

void ExpectSummaryEqual(const InferenceReplaySummary& a,
                        const InferenceReplaySummary& b) {
  EXPECT_EQ(a.status, b.status);
  EXPECT_EQ(a.degraded, b.degraded);
  EXPECT_EQ(a.inference_chain, b.inference_chain);
  EXPECT_TRUE(StatsEqual(a.stats, b.stats));
  ExpectOutcomes(a.outcomes, OutsOf(b.outcomes));
  ASSERT_EQ(a.trace.size(), b.trace.size());
  for (size_t i = 0; i < a.trace.size(); ++i) {
    EXPECT_EQ(a.trace[i].writer_index, b.trace[i].writer_index);
    EXPECT_EQ(a.trace[i].payload_kind, b.trace[i].payload_kind);
    EXPECT_EQ(a.trace[i].error, b.trace[i].error);
    EXPECT_EQ(a.trace[i].accepted, b.trace[i].accepted);
  }
}

constexpr int kNoError = 0;
const int kMissingFrameError =
    static_cast<int>(inference::InferenceEnvelopeContractError::kMissingFrame);
const int kDigestError =
    static_cast<int>(inference::InferenceEnvelopeContractError::kDigest);
constexpr PayloadKind kEnvelope = PayloadKind::kInferenceEnvelope;
constexpr PayloadKind kResult = PayloadKind::kInferenceResult;

TEST(InferenceReplayAdapterTest, RB3NominalGoldenPairIsComplete) {
  const EpisodeEvent golden_result = ParseGolden("inference_result");
  const std::vector<uint8_t> bytes = WriteEpisode(
      {ParseGolden("inference_envelope"), ParseGolden("inference_result")});
  std::unique_ptr<InferenceReplayAdapter> adapter = MakeAdapter();
  ASSERT_NE(adapter, nullptr);
  const std::vector<InferenceReplayStep> steps = FeedAll(adapter.get(), bytes);
  ASSERT_EQ(steps.size(), 2u);
  for (const InferenceReplayStep& step : steps) {
    EXPECT_EQ(step.status, ReplayAdapterStatus::kOk);
    EXPECT_TRUE(step.accepted);
    EXPECT_EQ(step.error, kNoError);
  }
  EXPECT_TRUE(steps[0].outcomes.empty());
  ExpectOutcomes(steps[1].outcomes, {{1, InferenceOutcome::kComplete, false,
                                      false, 0, kGoldenSnapshot}});

  const InferenceReplaySummary summary = adapter->Finish();
  EXPECT_EQ(summary.status, ReplayAdapterStatus::kOk);
  EXPECT_FALSE(summary.degraded);
  ExpectTrace(summary.trace,
              {{0, kEnvelope, kNoError, true}, {1, kResult, kNoError, true}});
  ExpectOutcomes(summary.outcomes, {{1, InferenceOutcome::kComplete, false,
                                     false, 0, kGoldenSnapshot}});
  EXPECT_EQ(ResultSha(golden_result), kShaGoldenResult);
  EXPECT_EQ(summary.inference_chain, kInferenceChainRb3);
  EXPECT_EQ(summary.inference_chain, Chain({kShaGoldenResult}));
  EXPECT_EQ(summary.stats.fed, 2u);
  EXPECT_EQ(summary.stats.envelopes, 1u);
  EXPECT_EQ(summary.stats.envelopes_accepted, 1u);
  EXPECT_EQ(summary.stats.results, 1u);
  EXPECT_EQ(summary.stats.results_accepted, 1u);
  EXPECT_EQ(summary.stats.complete, 1u);
  EXPECT_EQ(summary.stats.expired, 0u);
  EXPECT_EQ(summary.stats.orphans, 0u);
  EXPECT_EQ(summary.stats.missing_expired, 0u);
  EXPECT_EQ(summary.stats.missing_unresolved, 0u);
}

// The RB9 episode, also used for the determinism and boundary tests.
std::vector<uint8_t> MissingEpisode() {
  Opt a;
  a.receive_ms = 1;
  a.deadline_ms = 500;
  a.epoch = 42;
  Opt b;
  b.receive_ms = 600;
  b.deadline_ms = 5000;
  b.epoch = 43;
  Opt c;
  c.receive_ms = 700;
  c.deadline_ms = 800;
  c.epoch = 44;
  Opt result_b;
  result_b.receive_ms = 900;
  result_b.deadline_ms = 5000;
  result_b.epoch = 43;
  Opt d;
  d.receive_ms = 1000;
  d.deadline_ms = 9000;
  d.epoch = 46;
  return WriteEpisode({EnvelopeAt(1, a), EnvelopeAt(2, b), EnvelopeAt(3, c),
                       ResultAt(4, result_b), EnvelopeAt(5, d)});
}

TEST(InferenceReplayAdapterTest, RB4Deterministic) {
  const std::vector<uint8_t> bytes = MissingEpisode();
  std::unique_ptr<InferenceReplayAdapter> first = MakeAdapter();
  std::unique_ptr<InferenceReplayAdapter> second = MakeAdapter();
  ASSERT_NE(first, nullptr);
  ASSERT_NE(second, nullptr);
  const std::vector<InferenceReplayStep> a = FeedAll(first.get(), bytes);
  const std::vector<InferenceReplayStep> b = FeedAll(second.get(), bytes);
  ASSERT_EQ(a.size(), b.size());
  for (size_t i = 0; i < a.size(); ++i) {
    EXPECT_EQ(a[i].status, b[i].status) << i;
    EXPECT_EQ(a[i].writer_index, b[i].writer_index) << i;
    EXPECT_EQ(a[i].payload_kind, b[i].payload_kind) << i;
    EXPECT_EQ(a[i].error, b[i].error) << i;
    EXPECT_EQ(a[i].accepted, b[i].accepted) << i;
    ExpectOutcomes(a[i].outcomes, OutsOf(b[i].outcomes));
  }
  ExpectSummaryEqual(first->Finish(), second->Finish());
}

TEST(InferenceReplayAdapterTest, RB5CompatibilityLatches) {
  for (const BadEvent& bad : BadCompatibilityEvents(EnvelopeAt(2, Opt{}))) {
    SCOPED_TRACE(bad.name);
    EpisodeEvent first = ParseGolden("inference_envelope");
    first.mutable_header()->set_sequence(1);
    const std::vector<uint8_t> bytes =
        WriteEpisode({first, bad.event, ResultAt(3, Opt{})});
    std::unique_ptr<InferenceReplayAdapter> adapter = MakeAdapter();
    ASSERT_NE(adapter, nullptr);
    const std::vector<InferenceReplayStep> steps =
        FeedAll(adapter.get(), bytes);
    ASSERT_EQ(steps.size(), 3u);
    EXPECT_EQ(steps[0].status, ReplayAdapterStatus::kOk);
    EXPECT_EQ(steps[1].status, bad.expected);
    EXPECT_EQ(steps[1].payload_kind, PayloadKind::kNone);
    EXPECT_EQ(steps[2].status, ReplayAdapterStatus::kLatched);
    EXPECT_EQ(steps[2].payload_kind, PayloadKind::kNone);
    EXPECT_TRUE(steps[2].outcomes.empty());
    const InferenceReplaySummary summary = adapter->Finish();
    EXPECT_EQ(summary.status, bad.expected);
    EXPECT_EQ(summary.stats.after_latch, 1u);
    EXPECT_EQ(summary.stats.envelopes, 1u);
    EXPECT_EQ(summary.stats.results, 0u);
    EXPECT_EQ(summary.inference_chain, kEmptyChain);
    // The pending envelope is reported at Finish, after a latch too.
    ExpectOutcomes(summary.outcomes, {{0, InferenceOutcome::kMissing, false,
                                       false, 0, kGoldenSnapshot}});
  }
}

TEST(InferenceReplayAdapterTest, RB6InvalidPayloadsAreRecordedNotPaired) {
  EpisodeEvent invalid_envelope = EnvelopeAt(1, Opt{});
  invalid_envelope.mutable_inference_envelope()
      ->mutable_header()
      ->clear_frame_id();
  Opt other;
  other.epoch = 43;
  EpisodeEvent invalid_result = ResultAt(4, other);
  invalid_result.mutable_inference_result()->clear_output_digest();

  const std::vector<uint8_t> bytes = WriteEpisode(
      {invalid_envelope, ResultAt(2, Opt{}), EnvelopeAt(3, other),
       invalid_result, InferenceEnvelopeEvent(5), InferenceResultEvent(6)});
  std::unique_ptr<InferenceReplayAdapter> adapter = MakeAdapter();
  ASSERT_NE(adapter, nullptr);
  const std::vector<InferenceReplayStep> steps = FeedAll(adapter.get(), bytes);
  ASSERT_EQ(steps.size(), 6u);
  for (const InferenceReplayStep& step : steps) {
    EXPECT_EQ(step.status, ReplayAdapterStatus::kOk);
  }
  EXPECT_FALSE(steps[0].accepted);
  EXPECT_EQ(steps[0].error, kMissingFrameError);
  // The invalid envelope never entered pending, so this result is an orphan.
  EXPECT_TRUE(steps[1].accepted);
  ExpectOutcomes(steps[1].outcomes, {{1, InferenceOutcome::kReplayMiss, false,
                                      false, 0, kGoldenSnapshot}});
  EXPECT_TRUE(steps[2].accepted);
  // The invalid result does not pair, so envelope 3 stays pending.
  EXPECT_FALSE(steps[3].accepted);
  EXPECT_EQ(steps[3].error, kDigestError);
  EXPECT_TRUE(steps[3].outcomes.empty());
  // An empty message is not engaged: not accepted and not an error.
  EXPECT_FALSE(steps[4].accepted);
  EXPECT_EQ(steps[4].error, kNoError);
  EXPECT_FALSE(steps[5].accepted);
  EXPECT_EQ(steps[5].error, kNoError);

  const InferenceReplaySummary summary = adapter->Finish();
  EXPECT_EQ(summary.status, ReplayAdapterStatus::kOk);
  EXPECT_EQ(summary.inference_chain, kEmptyChain);
  EXPECT_EQ(summary.stats.envelopes, 3u);
  EXPECT_EQ(summary.stats.envelopes_accepted, 1u);
  EXPECT_EQ(summary.stats.results, 3u);
  EXPECT_EQ(summary.stats.results_accepted, 1u);
  EXPECT_EQ(summary.stats.orphans, 1u);
  EXPECT_EQ(summary.stats.missing_unresolved, 1u);
  ExpectOutcomes(
      summary.outcomes,
      {{1, InferenceOutcome::kReplayMiss, false, false, 0, kGoldenSnapshot},
       {2, InferenceOutcome::kMissing, false, false, 2, kGoldenSnapshot}});
}

TEST(InferenceReplayAdapterTest, RB8LateResultIsExpiredAndEqualIsComplete) {
  {
    Opt envelope;
    envelope.receive_ms = 1;
    envelope.deadline_ms = 500;
    Opt result;
    result.receive_ms = 600;
    result.deadline_ms = 500;
    const EpisodeEvent late = ResultAt(2, result);
    const std::vector<uint8_t> bytes =
        WriteEpisode({EnvelopeAt(1, envelope), late});
    std::unique_ptr<InferenceReplayAdapter> adapter = MakeAdapter();
    ASSERT_NE(adapter, nullptr);
    const std::vector<InferenceReplayStep> steps =
        FeedAll(adapter.get(), bytes);
    ASSERT_EQ(steps.size(), 2u);
    // No missing entry: the same-key result pairs and is late.
    ExpectOutcomes(steps[1].outcomes, {{1, InferenceOutcome::kReplayExpired,
                                        false, false, 0, kGoldenSnapshot}});
    const InferenceReplaySummary summary = adapter->Finish();
    EXPECT_FALSE(summary.degraded);
    EXPECT_EQ(summary.stats.expired, 1u);
    EXPECT_EQ(summary.stats.complete, 0u);
    EXPECT_EQ(summary.stats.missing_expired, 0u);
    // The result is chained exactly as recorded.
    EXPECT_EQ(summary.inference_chain, Chain({ResultSha(late)}));
    EXPECT_EQ(summary.inference_chain, kChainRb8Late);
  }
  {
    Opt envelope;
    envelope.receive_ms = 1;
    envelope.deadline_ms = 500;
    Opt result;
    result.receive_ms = 500;
    result.deadline_ms = 500;
    const std::vector<uint8_t> bytes =
        WriteEpisode({EnvelopeAt(1, envelope), ResultAt(2, result)});
    std::unique_ptr<InferenceReplayAdapter> adapter = MakeAdapter();
    ASSERT_NE(adapter, nullptr);
    const std::vector<InferenceReplayStep> steps =
        FeedAll(adapter.get(), bytes);
    ASSERT_EQ(steps.size(), 2u);
    ExpectOutcomes(steps[1].outcomes, {{1, InferenceOutcome::kComplete, false,
                                        false, 0, kGoldenSnapshot}});
    const InferenceReplaySummary summary = adapter->Finish();
    EXPECT_EQ(summary.stats.complete, 1u);
    EXPECT_EQ(summary.stats.expired, 0u);
  }
}

TEST(InferenceReplayAdapterTest, RB9MissingIsSweptBeforeTheRecord) {
  const std::vector<uint8_t> bytes = MissingEpisode();
  std::unique_ptr<InferenceReplayAdapter> adapter = MakeAdapter();
  ASSERT_NE(adapter, nullptr);
  const std::vector<InferenceReplayStep> steps = FeedAll(adapter.get(), bytes);
  ASSERT_EQ(steps.size(), 5u);
  EXPECT_TRUE(steps[0].outcomes.empty());
  // Envelope 1 (deadline 500) is judged against envelope 2's receive_time 600,
  // before envelope 2 is appended.
  ExpectOutcomes(steps[1].outcomes, {{0, InferenceOutcome::kMissing, true,
                                      false, 0, kGoldenSnapshot}});
  EXPECT_TRUE(steps[2].outcomes.empty());
  // The result for epoch 43 sweeps envelope 3 (deadline 800, now 900) and
  // then pairs with envelope 2, which its own key excluded from the sweep.
  ExpectOutcomes(
      steps[3].outcomes,
      {{2, InferenceOutcome::kMissing, true, false, 2, kGoldenSnapshot},
       {3, InferenceOutcome::kComplete, false, false, 1, kGoldenSnapshot}});
  EXPECT_TRUE(steps[4].outcomes.empty());

  const InferenceReplaySummary summary = adapter->Finish();
  EXPECT_EQ(summary.status, ReplayAdapterStatus::kOk);
  EXPECT_TRUE(summary.degraded);
  // Still pending at Finish: unresolved, not expired.
  ExpectOutcomes(
      summary.outcomes,
      {{0, InferenceOutcome::kMissing, true, false, 0, kGoldenSnapshot},
       {2, InferenceOutcome::kMissing, true, false, 2, kGoldenSnapshot},
       {3, InferenceOutcome::kComplete, false, false, 1, kGoldenSnapshot},
       {4, InferenceOutcome::kMissing, false, false, 4, kGoldenSnapshot}});
  ExpectTrace(summary.trace, {{0, kEnvelope, kNoError, true},
                              {1, kEnvelope, kNoError, true},
                              {2, kEnvelope, kNoError, true},
                              {3, kResult, kNoError, true},
                              {4, kEnvelope, kNoError, true}});
  EXPECT_EQ(summary.stats.envelopes, 4u);
  EXPECT_EQ(summary.stats.results, 1u);
  EXPECT_EQ(summary.stats.complete, 1u);
  EXPECT_EQ(summary.stats.missing_expired, 2u);
  EXPECT_EQ(summary.stats.missing_unresolved, 1u);
  EXPECT_EQ(summary.stats.orphans, 0u);
  Opt result_b;
  result_b.receive_ms = 900;
  result_b.deadline_ms = 5000;
  result_b.epoch = 43;
  EXPECT_EQ(summary.inference_chain, Chain({ResultSha(ResultAt(4, result_b))}));
  EXPECT_EQ(summary.inference_chain, kChainRb9);
}

TEST(InferenceReplayAdapterTest, RB10OrphanAndFifoPairing) {
  Opt envelope;
  envelope.receive_ms = 1;
  envelope.deadline_ms = 5000;
  Opt first;
  first.receive_ms = 10;
  first.deadline_ms = 5000;
  first.output_digest = "sha256:01";
  Opt second = first;
  second.output_digest = "sha256:02";
  Opt third = first;
  third.output_digest = "sha256:03";
  const EpisodeEvent r1 = ResultAt(3, first);
  const EpisodeEvent r2 = ResultAt(4, second);
  const std::vector<uint8_t> bytes =
      WriteEpisode({EnvelopeAt(1, envelope), EnvelopeAt(2, envelope), r1, r2,
                    ResultAt(5, third)});
  std::unique_ptr<InferenceReplayAdapter> adapter = MakeAdapter();
  ASSERT_NE(adapter, nullptr);
  const std::vector<InferenceReplayStep> steps = FeedAll(adapter.get(), bytes);
  ASSERT_EQ(steps.size(), 5u);
  // The oldest pending entry with an equal key pairs first.
  ExpectOutcomes(steps[2].outcomes, {{2, InferenceOutcome::kComplete, false,
                                      false, 0, kGoldenSnapshot}});
  ExpectOutcomes(steps[3].outcomes, {{3, InferenceOutcome::kComplete, false,
                                      false, 1, kGoldenSnapshot}});
  ExpectOutcomes(steps[4].outcomes, {{4, InferenceOutcome::kReplayMiss, false,
                                      false, 0, kGoldenSnapshot}});
  const InferenceReplaySummary summary = adapter->Finish();
  EXPECT_FALSE(summary.degraded);
  EXPECT_EQ(summary.stats.complete, 2u);
  EXPECT_EQ(summary.stats.orphans, 1u);
  // An orphan is not chained.
  EXPECT_EQ(summary.inference_chain, Chain({ResultSha(r1), ResultSha(r2)}));
  EXPECT_EQ(summary.inference_chain, kChainRb10);
}

TEST(InferenceReplayAdapterTest, RB11ModelDigestMismatch) {
  {
    Opt envelope;
    Opt result;
    result.provenance_digest = "sha256:" + std::string(64, 'a');
    const std::vector<uint8_t> bytes =
        WriteEpisode({EnvelopeAt(1, envelope), ResultAt(2, result)});
    std::unique_ptr<InferenceReplayAdapter> adapter = MakeAdapter();
    ASSERT_NE(adapter, nullptr);
    const std::vector<InferenceReplayStep> steps =
        FeedAll(adapter.get(), bytes);
    ASSERT_EQ(steps.size(), 2u);
    ExpectOutcomes(steps[1].outcomes, {{1, InferenceOutcome::kComplete, false,
                                        true, 0, kGoldenSnapshot}});
    const InferenceReplaySummary summary = adapter->Finish();
    // Degraded, not latched, and the result is still chained.
    EXPECT_EQ(summary.status, ReplayAdapterStatus::kOk);
    EXPECT_TRUE(summary.degraded);
    EXPECT_EQ(summary.stats.model_digest_mismatches, 1u);
    EXPECT_EQ(summary.stats.complete, 1u);
    EXPECT_EQ(summary.inference_chain, Chain({ResultSha(ResultAt(2, result))}));
    EXPECT_EQ(summary.inference_chain, kChainRb11);
  }
  {
    Opt envelope;
    envelope.provenance_digest = "";
    Opt result;
    result.provenance_digest = "sha256:" + std::string(64, 'a');
    const std::vector<uint8_t> bytes =
        WriteEpisode({EnvelopeAt(1, envelope), ResultAt(2, result)});
    std::unique_ptr<InferenceReplayAdapter> adapter = MakeAdapter();
    ASSERT_NE(adapter, nullptr);
    const std::vector<InferenceReplayStep> steps =
        FeedAll(adapter.get(), bytes);
    ASSERT_EQ(steps.size(), 2u);
    // An empty envelope digest is not flagged.
    ExpectOutcomes(steps[1].outcomes, {{1, InferenceOutcome::kComplete, false,
                                        false, 0, kGoldenSnapshot}});
    const InferenceReplaySummary summary = adapter->Finish();
    EXPECT_FALSE(summary.degraded);
    EXPECT_EQ(summary.stats.model_digest_mismatches, 0u);
  }
}

TEST(InferenceReplayAdapterTest, RB12OtherKindsAreIgnoredAndLossIsAGap) {
  {
    const std::vector<uint8_t> bytes = WriteEpisode({
        SafetyEvent(1),
        TrajectoryEvent(2),
        DesiredMotionEvent(3),
        RecordedEvent(4, "intrinsic_proto.estimation.MeasurementEnvelope", ""),
        WorldRecordEvent(5),
        MetadataEvent(6, Seed42MarineText()),
    });
    std::unique_ptr<InferenceReplayAdapter> adapter = MakeAdapter();
    ASSERT_NE(adapter, nullptr);
    const std::vector<InferenceReplayStep> steps =
        FeedAll(adapter.get(), bytes);
    ASSERT_EQ(steps.size(), 6u);
    for (const InferenceReplayStep& step : steps) {
      EXPECT_EQ(step.status, ReplayAdapterStatus::kIgnoredPayload);
      EXPECT_EQ(step.payload_kind, PayloadKind::kNone);
      EXPECT_TRUE(step.outcomes.empty());
    }
    const InferenceReplaySummary summary = adapter->Finish();
    EXPECT_EQ(summary.stats.fed, 6u);
    EXPECT_EQ(summary.stats.ignored, 6u);
    EXPECT_TRUE(summary.trace.empty());
    EXPECT_TRUE(summary.outcomes.empty());
    EXPECT_FALSE(summary.degraded);
    EXPECT_EQ(summary.inference_chain, kEmptyChain);
  }
  {
    Opt later;
    later.epoch = 44;
    const std::vector<uint8_t> bytes = WriteOverflowEpisode(
        {MetadataEvent(1, Seed42MarineText()), EnvelopeAt(2, Opt{}),
         ResultAt(3, Opt{})},
        {EnvelopeAt(4, Opt{}), ResultAt(5, Opt{})}, EnvelopeAt(6, later));
    std::unique_ptr<InferenceReplayAdapter> adapter = MakeAdapter();
    ASSERT_NE(adapter, nullptr);
    const std::vector<InferenceReplayStep> steps =
        FeedAll(adapter.get(), bytes);
    ASSERT_EQ(steps.size(), 5u);
    EXPECT_EQ(steps[3].status, ReplayAdapterStatus::kLossRecord);
    // Dropped events consume writer indices 3 and 4, the loss record is 5.
    EXPECT_EQ(steps[3].writer_index, 5u);
    EXPECT_EQ(steps[4].status, ReplayAdapterStatus::kOk);
    EXPECT_EQ(steps[4].writer_index, 6u);
    const InferenceReplaySummary summary = adapter->Finish();
    EXPECT_EQ(summary.status, ReplayAdapterStatus::kOk);
    EXPECT_TRUE(summary.degraded);
    EXPECT_EQ(summary.stats.gaps, 1u);
    EXPECT_EQ(summary.stats.gap_dropped, 2u);
    EXPECT_EQ(summary.stats.envelopes, 2u);
    EXPECT_EQ(summary.stats.results, 1u);
    EXPECT_EQ(summary.stats.complete, 1u);
  }
}

TEST(InferenceReplayAdapterTest, RB13Boundaries) {
  {
    const std::vector<uint8_t> bytes = WriteEpisode({});
    std::unique_ptr<InferenceReplayAdapter> adapter = MakeAdapter();
    ASSERT_NE(adapter, nullptr);
    EXPECT_TRUE(FeedAll(adapter.get(), bytes).empty());
    const InferenceReplaySummary summary = adapter->Finish();
    EXPECT_EQ(summary.status, ReplayAdapterStatus::kOk);
    EXPECT_EQ(summary.inference_chain, kEmptyChain);
    EXPECT_EQ(summary.inference_chain,
              internal::EpisodeSha256Hex(std::string("v1\n")));
    EXPECT_TRUE(summary.trace.empty());
    EXPECT_TRUE(summary.outcomes.empty());
    EXPECT_FALSE(summary.degraded);
  }
  {
    const std::vector<uint8_t> bytes =
        WriteEpisode({ParseGolden("inference_envelope")});
    std::unique_ptr<InferenceReplayAdapter> adapter = MakeAdapter();
    ASSERT_NE(adapter, nullptr);
    EpisodeStream stream(bytes);
    EpisodeReplayEvent event;
    ASSERT_TRUE(stream.Next(&event));
    const InferenceReplayStep step = adapter->Feed(event);
    EXPECT_EQ(step.status, ReplayAdapterStatus::kOk);
    EXPECT_TRUE(step.accepted);

    const InferenceReplaySummary first = adapter->Finish();
    const InferenceReplayStep after = adapter->Feed(event);
    EXPECT_EQ(after.status, ReplayAdapterStatus::kFinished);
    EXPECT_EQ(after.writer_index, 0u);
    EXPECT_EQ(after.payload_kind, PayloadKind::kNone);
    const InferenceReplaySummary second = adapter->Finish();
    ExpectSummaryEqual(first, second);
    EXPECT_EQ(second.stats.fed, 1u);
    // Finish is idempotent: the unresolved entry is reported once.
    EXPECT_EQ(second.stats.missing_unresolved, 1u);
    ExpectOutcomes(second.outcomes, {{0, InferenceOutcome::kMissing, false,
                                      false, 0, kGoldenSnapshot}});
    EXPECT_TRUE(second.degraded);
    EXPECT_EQ(second.inference_chain, kEmptyChain);
  }
}

TEST(InferenceReplayAdapterTest, RB14ConfigAndIsolation) {
  const std::vector<std::string> bad_digests = {
      std::string(63, 'a'),
      std::string(64, 'A'),
      std::string(65, 'a'),
      std::string("g") + std::string(63, 'a'),
  };
  for (const std::string& digest : bad_digests) {
    InferenceReplayConfig config;
    config.compatibility.expected_config_digest = digest;
    ReplayAdapterStatus status = ReplayAdapterStatus::kOk;
    EXPECT_EQ(InferenceReplayAdapter::Create(config, &status), nullptr)
        << digest;
    EXPECT_EQ(status, ReplayAdapterStatus::kInvalidConfig) << digest;
  }
  ReplayAdapterStatus status = ReplayAdapterStatus::kInvalidConfig;
  EXPECT_NE(InferenceReplayAdapter::Create(InferenceReplayConfig{}, &status),
            nullptr);
  EXPECT_EQ(status, ReplayAdapterStatus::kOk);

  // The call-count proof: the config has one field, the compatibility rules,
  // so there is no InferenceClient, queue or ReplayWorker to inject and none
  // can be called.
  static_assert(sizeof(InferenceReplayConfig) ==
                sizeof(ReplayCompatibilityConfig));
  ExpectNoForbiddenDeps(BuildDepsOf("inference_replay_adapter"));
}

}  // namespace
}  // namespace intrinsic::episode
