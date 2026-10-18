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

// S5 (#244, lock id episode-replay-payloads-v1): the three replay payload
// goldens go through EpisodeWriter, EpisodeReader and EpisodeReplayer
// unchanged. No writer, reader, or replay logic is edited for them.

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
#include "intrinsic_data/episode/episode_event_policy.h"
#include "intrinsic_data/episode/episode_reader.h"
#include "intrinsic_data/episode/episode_replay.h"
#include "intrinsic_data/episode/episode_writer.h"

namespace intrinsic::episode {
namespace {

using intrinsic_proto::episode::EpisodeEvent;

struct Golden {
  const char* name;
  PayloadKind kind;
};

constexpr Golden kGoldens[] = {
    {"recorded_message", PayloadKind::kRecordedMessage},
    {"world_snapshot_record", PayloadKind::kWorldSnapshotRecord},
    {"desired_motion", PayloadKind::kDesiredMotion},
};

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

// Writes the three goldens in order and returns the episode bytes.
std::vector<uint8_t> WriteEpisode() {
  MemorySink sink;
  EpisodeWriterStatus status = EpisodeWriterStatus::kInvalidConfig;
  std::unique_ptr<EpisodeWriter> writer =
      EpisodeWriter::Create(EpisodeWriterConfig{}, &sink, &status);
  EXPECT_EQ(status, EpisodeWriterStatus::kOk);
  if (writer == nullptr) {
    return {};
  }
  for (const Golden& golden : kGoldens) {
    const EpisodeAppendResult result = writer->Append(ParseGolden(golden.name));
    EXPECT_EQ(result.status, EpisodeWriterStatus::kOk) << golden.name;
    EXPECT_EQ(result.event_error, EpisodeEventError::kNone) << golden.name;
  }
  EXPECT_EQ(writer->Close(), EpisodeWriterStatus::kOk);
  const EpisodeWriterStats stats = writer->Stats();
  EXPECT_EQ(stats.accepted, 3u);
  EXPECT_EQ(stats.written_records, 3u);
  EXPECT_EQ(stats.loss_records, 0u);
  EXPECT_EQ(stats.rejected_reserved, 0u);
  return sink.output;
}

TEST(EpisodeReplayPayloadsTest, S5WriterAcceptsTheThreeGoldens) {
  const std::vector<uint8_t> bytes = WriteEpisode();
  EXPECT_GT(bytes.size(), kEpisodeFileHeaderBytes);
}

TEST(EpisodeReplayPayloadsTest, S5ReaderClassifiesThemAsKnownKinds) {
  const std::vector<uint8_t> bytes = WriteEpisode();
  const EpisodeReadResult result = ReadEpisode(bytes, EpisodeReaderConfig{});
  ASSERT_EQ(result.status, EpisodeReaderStatus::kOk);
  EXPECT_EQ(result.defect, EpisodeReadDefect::kNone);
  ASSERT_EQ(result.records.size(), 3u);
  EXPECT_EQ(result.good_records, 3u);
  EXPECT_EQ(result.stats.unknown_skipped, 0u);
  EXPECT_EQ(result.stats.loss_records, 0u);
  const int kind_numbers[] = {8, 9, 10};
  for (size_t i = 0; i < 3; ++i) {
    const EpisodeRecordEntry& entry = result.records[i];
    EXPECT_EQ(entry.payload_kind, kGoldens[i].kind) << kGoldens[i].name;
    EXPECT_EQ(static_cast<int>(entry.payload_kind), kind_numbers[i]);
    EXPECT_FALSE(entry.skip) << kGoldens[i].name;
    EXPECT_EQ(entry.source_id, "episode_recorder");
    EXPECT_EQ(entry.writer_index, i);
    const std::string golden = ReadGoldenBytes(kGoldens[i].name);
    EXPECT_EQ(std::string(entry.event_bytes.begin(), entry.event_bytes.end()),
              golden)
        << kGoldens[i].name;
  }
}

TEST(EpisodeReplayPayloadsTest, S5KindFiltersSelectEachNewKind) {
  const std::vector<uint8_t> bytes = WriteEpisode();
  for (const Golden& golden : kGoldens) {
    EpisodeReadFilter filter;
    filter.payload_kinds = {golden.kind};
    const EpisodeReadResult result =
        ReadEpisode(bytes, EpisodeReaderConfig{}, filter);
    ASSERT_EQ(result.records.size(), 1u) << golden.name;
    EXPECT_EQ(result.records[0].payload_kind, golden.kind) << golden.name;
  }
}

TEST(EpisodeReplayPayloadsTest, S5ReplayEmitsAllThreeWithoutSkipping) {
  const std::vector<uint8_t> bytes = WriteEpisode();
  EpisodeReplayConfig config;
  config.mode = EpisodeReplayMode::kStep;
  EpisodeReplayStatus status = EpisodeReplayStatus::kInvalidConfig;
  std::unique_ptr<EpisodeReplayer> replayer = EpisodeReplayer::Create(
      bytes, EpisodeReaderConfig{}, EpisodeReadFilter{}, config, &status);
  ASSERT_EQ(status, EpisodeReplayStatus::kOk);
  ASSERT_NE(replayer, nullptr);
  for (size_t i = 0; i < 3; ++i) {
    const EpisodeReplayResult result = replayer->Step();
    ASSERT_EQ(result.status, EpisodeReplayStatus::kOk) << i;
    ASSERT_NE(result.event.record, nullptr) << i;
    EXPECT_EQ(result.event.record->payload_kind, kGoldens[i].kind) << i;
    EXPECT_EQ(result.event.ordinal, i);
    EXPECT_EQ(result.event.writer_index, i);
    EXPECT_EQ(result.event.loss, nullptr) << i;
  }
  EXPECT_EQ(replayer->Step().status, EpisodeReplayStatus::kEndOfStream);
  const EpisodeReplayStats stats = replayer->Stats();
  EXPECT_EQ(stats.total, 3u);
  EXPECT_EQ(stats.emitted, 3u);
  EXPECT_EQ(stats.loss_emitted, 0u);
  EXPECT_EQ(stats.skipped_unknown, 0u);
}

}  // namespace
}  // namespace intrinsic::episode
