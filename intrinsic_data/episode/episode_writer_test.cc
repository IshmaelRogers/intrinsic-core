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

// W1 to W14: the bounded episode writer (#128, lock id
// episode-writer-bounded-v1).

#include "intrinsic_data/episode/episode_writer.h"

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <memory>
#include <new>
#include <sstream>
#include <string>
#include <string_view>
#include <thread>
#include <vector>

#include "absl/crc/crc32c.h"
#include "absl/status/status.h"
#include "absl/types/span.h"
#include "gtest/gtest.h"
#include "intrinsic/episode/proto/episode_event.pb.h"
#include "intrinsic_data/episode/episode_event_policy.h"

// W14: a test-binary-local replacement of the global allocation functions. It
// counts allocations only while armed.
namespace {
std::atomic<bool> g_count_allocations{false};
std::atomic<uint64_t> g_allocations{0};

void CountAllocation() {
  if (g_count_allocations.load(std::memory_order_relaxed)) {
    g_allocations.fetch_add(1, std::memory_order_relaxed);
  }
}

void* AllocateOrThrow(std::size_t size) {
  CountAllocation();
  void* pointer = std::malloc(size == 0 ? 1 : size);
  if (pointer == nullptr) {
    throw std::bad_alloc();
  }
  return pointer;
}

void* AllocateAlignedOrThrow(std::size_t size, std::align_val_t alignment) {
  CountAllocation();
  const std::size_t align = static_cast<std::size_t>(alignment);
  const std::size_t rounded =
      ((size == 0 ? 1 : size) + align - 1) / align * align;
  void* pointer = std::aligned_alloc(align, rounded);
  if (pointer == nullptr) {
    throw std::bad_alloc();
  }
  return pointer;
}
}  // namespace

void* operator new(std::size_t size) { return AllocateOrThrow(size); }
void* operator new[](std::size_t size) { return AllocateOrThrow(size); }
void* operator new(std::size_t size, std::align_val_t alignment) {
  return AllocateAlignedOrThrow(size, alignment);
}
void* operator new[](std::size_t size, std::align_val_t alignment) {
  return AllocateAlignedOrThrow(size, alignment);
}
void operator delete(void* pointer) noexcept { std::free(pointer); }
void operator delete[](void* pointer) noexcept { std::free(pointer); }
void operator delete(void* pointer, std::size_t) noexcept {
  std::free(pointer);
}
void operator delete[](void* pointer, std::size_t) noexcept {
  std::free(pointer);
}
void operator delete(void* pointer, std::align_val_t) noexcept {
  std::free(pointer);
}
void operator delete[](void* pointer, std::align_val_t) noexcept {
  std::free(pointer);
}
void operator delete(void* pointer, std::size_t, std::align_val_t) noexcept {
  std::free(pointer);
}
void operator delete[](void* pointer, std::size_t, std::align_val_t) noexcept {
  std::free(pointer);
}

namespace intrinsic::episode {
namespace {

using intrinsic_proto::episode::EpisodeEvent;
using intrinsic_proto::episode::EpisodeLoss;
using Status = EpisodeWriterStatus;

constexpr const char* kPayloadNames[] = {
    "safety_decision", "vehicle_trajectory", "inference_envelope",
    "inference_result", "metadata"};
constexpr std::string_view kV2Name =
    "episode_event_v2_additive_unknown_payload.binpb";

std::string ReadRunfile(const std::string& relative) {
  const char* src = std::getenv("TEST_SRCDIR");
  const char* workspace = std::getenv("TEST_WORKSPACE");
  std::vector<std::string> candidates;
  if (src != nullptr) {
    const std::string root(src);
    if (workspace != nullptr) {
      candidates.push_back(root + "/" + workspace + "/" + relative);
    }
    candidates.push_back(root + "/_main/" + relative);
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

std::string TestdataBytes(const std::string& name) {
  return ReadRunfile("intrinsic_data/episode/testdata/" + name);
}

std::string GoldenBytes(const std::string& payload) {
  return TestdataBytes("episode_event_v1_" + payload + ".binpb");
}

EpisodeEvent Parse(const std::string& bytes) {
  EpisodeEvent message;
  EXPECT_TRUE(message.ParseFromString(bytes));
  return message;
}

EpisodeEvent Golden(const std::string& payload) {
  return Parse(GoldenBytes(payload));
}

// A valid event with a chosen source and sequence. Padding the provenance
// model_id (unvalidated) controls the serialized size.
EpisodeEvent MakeEvent(const std::string& source, uint64_t sequence,
                       size_t pad = 0) {
  EpisodeEvent event = Golden("safety_decision");
  event.mutable_header()->set_source_id(source);
  event.mutable_header()->set_sequence(sequence);
  if (pad > 0) {
    event.mutable_provenance()->set_model_id(std::string(pad, 'x'));
  }
  return event;
}

std::string ToString(const std::vector<uint8_t>& bytes) {
  return std::string(bytes.begin(), bytes.end());
}

class FakeEpisodeSink : public EpisodeSink {
 public:
  absl::Status Write(absl::Span<const uint8_t> bytes) override {
    const int call = write_calls++;
    if (call == fail_write_call) {
      return absl::UnavailableError("injected write failure");
    }
    writes.emplace_back(bytes.begin(), bytes.end());
    output.insert(output.end(), bytes.begin(), bytes.end());
    return absl::OkStatus();
  }

  absl::Status Sync() override {
    ++sync_calls;
    if (fail_sync) {
      return absl::DataLossError("injected sync failure");
    }
    return absl::OkStatus();
  }

  int calls() const { return write_calls + sync_calls; }

  int fail_write_call = -1;
  bool fail_sync = false;
  int write_calls = 0;
  int sync_calls = 0;
  std::vector<std::vector<uint8_t>> writes;
  std::vector<uint8_t> output;
};

struct DecodedRecord {
  uint32_t length = 0;
  uint32_t crc = 0;
  uint64_t index = 0;
  std::string bytes;
  bool crc_ok = false;
};

struct DecodedFile {
  bool header_ok = false;
  bool well_formed = false;
  std::vector<DecodedRecord> records;
};

uint64_t GetLittleEndian(const std::string& data, size_t offset, int bytes) {
  uint64_t value = 0;
  for (int i = 0; i < bytes; ++i) {
    value |= static_cast<uint64_t>(static_cast<uint8_t>(data[offset + i]))
             << (8 * i);
  }
  return value;
}

DecodedFile Decode(const std::string& data) {
  DecodedFile file;
  const std::string expected_header =
      std::string("IEPISOD1") + '\x01' + '\0' + '\0' + '\0';
  file.header_ok =
      data.size() >= kEpisodeFileHeaderBytes &&
      data.compare(0, kEpisodeFileHeaderBytes, expected_header) == 0;
  size_t offset = kEpisodeFileHeaderBytes;
  while (file.header_ok && offset + kEpisodeRecordHeaderBytes <= data.size()) {
    DecodedRecord record;
    record.length = static_cast<uint32_t>(GetLittleEndian(data, offset, 4));
    record.crc = static_cast<uint32_t>(GetLittleEndian(data, offset + 4, 4));
    record.index = GetLittleEndian(data, offset + 8, 8);
    offset += kEpisodeRecordHeaderBytes;
    if (offset + record.length > data.size()) {
      return file;
    }
    record.bytes = data.substr(offset, record.length);
    record.crc_ok =
        record.crc == static_cast<uint32_t>(absl::ComputeCrc32c(record.bytes));
    offset += record.length;
    file.records.push_back(std::move(record));
  }
  file.well_formed = file.header_ok && offset == data.size();
  return file;
}

std::unique_ptr<EpisodeWriter> MakeWriter(EpisodeSink* sink, size_t capacity,
                                          size_t max_event_bytes) {
  EpisodeWriterConfig config;
  config.capacity = capacity;
  config.max_event_bytes = max_event_bytes;
  Status status = Status::kInvalidConfig;
  auto writer = EpisodeWriter::Create(config, sink, &status);
  EXPECT_EQ(status, Status::kOk);
  EXPECT_NE(writer, nullptr);
  return writer;
}

constexpr size_t kSmallMaxEventBytes = 4096;

void ExpectLoss(const DecodedRecord& record, uint64_t ordinal, uint64_t count,
                uint64_t first, uint64_t last, EpisodeLoss::LossReason reason,
                int64_t seconds, int32_t nanos) {
  const EpisodeEvent loss = Parse(record.bytes);
  EXPECT_EQ(loss.header().sequence(), ordinal);
  EXPECT_EQ(loss.header().source_id(), kEpisodeWriterSourceId);
  EXPECT_EQ(loss.header().frame_id(), kEpisodeWriterFrameId);
  EXPECT_TRUE(loss.header().clock_domain().empty());
  EXPECT_FALSE(loss.header().has_validity());
  for (const auto* time :
       {&loss.header().source_time(), &loss.header().receive_time()}) {
    EXPECT_EQ(time->seconds(), seconds);
    EXPECT_EQ(time->nanos(), nanos);
  }
  EXPECT_TRUE(loss.header().has_source_time());
  EXPECT_TRUE(loss.header().has_receive_time());
  EXPECT_EQ(loss.schema_version(), 1u);
  EXPECT_TRUE(loss.snapshot_id().empty());
  EXPECT_TRUE(loss.config_digest().empty());
  EXPECT_FALSE(loss.has_provenance());
  ASSERT_EQ(loss.payload_case(), EpisodeEvent::kEpisodeLoss);
  EXPECT_EQ(loss.episode_loss().dropped_count(), count);
  EXPECT_EQ(loss.episode_loss().first_dropped_writer_index(), first);
  EXPECT_EQ(loss.episode_loss().last_dropped_writer_index(), last);
  EXPECT_EQ(loss.episode_loss().reason(), reason);
  EXPECT_EQ(loss.episode_loss().dropped_count(), last - first + 1);
  EXPECT_EQ(record.index, last + 1);
}

TEST(EpisodeWriterTest, PolicyKnowsTheLossPayload) {
  EXPECT_EQ(kPayloadCaseEpisodeLoss, 15);
  EXPECT_EQ(static_cast<int>(PayloadKind::kUnknown), 6);
  EXPECT_EQ(static_cast<int>(PayloadKind::kEpisodeLoss), 7);
  EXPECT_EQ(ClassifyPayload(kPayloadCaseEpisodeLoss, false),
            PayloadKind::kEpisodeLoss);
  EXPECT_EQ(static_cast<int>(EpisodeEvent::kEpisodeLoss),
            kPayloadCaseEpisodeLoss);
}

TEST(EpisodeWriterTest, ConstantsAndStatusNumbersArePermanent) {
  EXPECT_EQ(std::string_view(kEpisodeFileMagic, 8), "IEPISOD1");
  EXPECT_EQ(kEpisodeFormatVersion, 1u);
  EXPECT_EQ(kEpisodeFileHeaderBytes, 12u);
  EXPECT_EQ(kEpisodeRecordHeaderBytes, 16u);
  EXPECT_EQ(kDefaultEpisodeWriterCapacity, 1024u);
  EXPECT_EQ(kDefaultEpisodeMaxEventBytes, 65536u);
  EXPECT_EQ(kMinEpisodeWriterCapacity, 2u);
  EXPECT_EQ(kMaxEpisodeWriterCapacity, 65536u);
  EXPECT_EQ(kMinEpisodeMaxEventBytes, 1u);
  EXPECT_EQ(kMaxEpisodeMaxEventBytes, 16u * 1024 * 1024);
  EXPECT_EQ(kMaxEpisodeSources, 64u);
  EXPECT_EQ(kEpisodeWriterSourceId, "episode_writer");
  EXPECT_EQ(kEpisodeWriterFrameId, "episode_writer");
  EXPECT_EQ(static_cast<int>(Status::kOk), 0);
  EXPECT_EQ(static_cast<int>(Status::kInvalidConfig), 1);
  EXPECT_EQ(static_cast<int>(Status::kInvalidEvent), 2);
  EXPECT_EQ(static_cast<int>(Status::kReservedEvent), 3);
  EXPECT_EQ(static_cast<int>(Status::kOutOfOrder), 4);
  EXPECT_EQ(static_cast<int>(Status::kTooManySources), 5);
  EXPECT_EQ(static_cast<int>(Status::kBufferFull), 6);
  EXPECT_EQ(static_cast<int>(Status::kEventTooLarge), 7);
  EXPECT_EQ(static_cast<int>(Status::kWriteFailed), 8);
  EXPECT_EQ(static_cast<int>(Status::kClosed), 9);
  EXPECT_EQ(static_cast<int>(Status::kCancelled), 10);
}

TEST(EpisodeWriterTest, W1NominalGolden) {
  FakeEpisodeSink sink;
  auto writer = MakeWriter(&sink, 16, kSmallMaxEventBytes);
  ASSERT_NE(writer, nullptr);
  for (size_t i = 0; i < 5; ++i) {
    const EpisodeEvent event = Golden(kPayloadNames[i]);
    EXPECT_EQ(event.header().sequence(), i + 1);
    EXPECT_EQ(event.header().source_id(), "episode_recorder");
    const EpisodeAppendResult result = writer->Append(event);
    EXPECT_EQ(result.status, Status::kOk) << kPayloadNames[i];
    EXPECT_EQ(result.writer_index, i);
  }
  EXPECT_EQ(writer->Close(), Status::kOk);
  EXPECT_EQ(sink.sync_calls, 1);

  const std::string output = ToString(sink.output);
  EXPECT_EQ(output, TestdataBytes("episode_writer_v1_nominal.iepisod"));
  const DecodedFile file = Decode(output);
  EXPECT_TRUE(file.well_formed);
  ASSERT_EQ(file.records.size(), 5u);
  for (size_t i = 0; i < 5; ++i) {
    EXPECT_EQ(file.records[i].index, i);
    EXPECT_TRUE(file.records[i].crc_ok) << i;
    EXPECT_EQ(file.records[i].length, file.records[i].bytes.size());
    EXPECT_EQ(file.records[i].bytes, GoldenBytes(kPayloadNames[i])) << i;
  }
  const EpisodeWriterStats stats = writer->Stats();
  EXPECT_EQ(stats.accepted, 5u);
  EXPECT_EQ(stats.written_records, 5u);
  EXPECT_EQ(stats.loss_records, 0u);
  EXPECT_EQ(stats.queued_records, 0u);
  EXPECT_FALSE(stats.pending_loss);
  EXPECT_EQ(stats.next_writer_index, 5u);
  EXPECT_TRUE(writer->FirstSinkError().ok());
}

TEST(EpisodeWriterTest, W1EveryRecordIsOneSinkWrite) {
  FakeEpisodeSink sink;
  auto writer = MakeWriter(&sink, 16, kSmallMaxEventBytes);
  for (size_t i = 0; i < 5; ++i) {
    ASSERT_EQ(writer->Append(Golden(kPayloadNames[i])).status, Status::kOk);
  }
  ASSERT_EQ(writer->Close(), Status::kOk);
  ASSERT_EQ(sink.writes.size(), 6u);
  EXPECT_EQ(sink.writes[0].size(), kEpisodeFileHeaderBytes);
  for (size_t i = 0; i < 5; ++i) {
    EXPECT_EQ(sink.writes[i + 1].size(),
              kEpisodeRecordHeaderBytes + GoldenBytes(kPayloadNames[i]).size());
  }
}

TEST(EpisodeWriterTest, W2EmptyEpisodeIsTheHeaderOnly) {
  FakeEpisodeSink sink;
  auto writer = MakeWriter(&sink, 4, kSmallMaxEventBytes);
  EXPECT_EQ(writer->Close(), Status::kOk);
  const std::vector<uint8_t> expected = {0x49, 0x45, 0x50, 0x49, 0x53, 0x4F,
                                         0x44, 0x31, 0x01, 0x00, 0x00, 0x00};
  EXPECT_EQ(sink.output, expected);
  EXPECT_EQ(sink.write_calls, 1);
  EXPECT_EQ(sink.sync_calls, 1);
}

TEST(EpisodeWriterTest, CreateWritesNothing) {
  FakeEpisodeSink sink;
  auto writer = MakeWriter(&sink, 4, kSmallMaxEventBytes);
  EXPECT_EQ(sink.calls(), 0);
  ASSERT_EQ(writer->Append(MakeEvent("a", 1)).status, Status::kOk);
  EXPECT_EQ(sink.calls(), 0);
}

TEST(EpisodeWriterTest, W3SingleEvent) {
  FakeEpisodeSink sink;
  auto writer = MakeWriter(&sink, 4, kSmallMaxEventBytes);
  const EpisodeEvent event = Golden("safety_decision");
  ASSERT_EQ(writer->Append(event).status, Status::kOk);
  EXPECT_EQ(writer->Close(), Status::kOk);
  const DecodedFile file = Decode(ToString(sink.output));
  EXPECT_TRUE(file.well_formed);
  ASSERT_EQ(file.records.size(), 1u);
  EXPECT_EQ(file.records[0].index, 0u);
  EXPECT_TRUE(file.records[0].crc_ok);
  EXPECT_EQ(file.records[0].bytes, GoldenBytes("safety_decision"));
}

TEST(EpisodeWriterTest, W4EventOfExactlyTheMaximumIsAccepted) {
  const size_t size = MakeEvent("a", 1, 300).ByteSizeLong();
  FakeEpisodeSink sink;
  auto writer = MakeWriter(&sink, 4, size);
  const EpisodeAppendResult result = writer->Append(MakeEvent("a", 1, 300));
  EXPECT_EQ(result.status, Status::kOk);
  EXPECT_EQ(result.writer_index, 0u);
  ASSERT_EQ(writer->Close(), Status::kOk);
  const DecodedFile file = Decode(ToString(sink.output));
  ASSERT_EQ(file.records.size(), 1u);
  EXPECT_EQ(file.records[0].length, size);
}

TEST(EpisodeWriterTest, W4OneByteOverTheMaximumIsDroppedWithALossRecord) {
  const size_t size = MakeEvent("a", 1, 300).ByteSizeLong();
  const EpisodeEvent too_large = MakeEvent("a", 1, 301);
  ASSERT_EQ(too_large.ByteSizeLong(), size + 1);
  FakeEpisodeSink sink;
  auto writer = MakeWriter(&sink, 4, size);
  const EpisodeAppendResult result = writer->Append(too_large);
  EXPECT_EQ(result.status, Status::kEventTooLarge);
  EXPECT_EQ(result.writer_index, 0u);
  EXPECT_EQ(writer->Stats().dropped_too_large, 1u);
  EXPECT_TRUE(writer->Stats().pending_loss);
  ASSERT_EQ(writer->Close(), Status::kOk);
  const DecodedFile file = Decode(ToString(sink.output));
  EXPECT_TRUE(file.well_formed);
  ASSERT_EQ(file.records.size(), 1u);
  ExpectLoss(file.records[0], 1, 1, 0, 0, EpisodeLoss::LOSS_REASON_TOO_LARGE,
             too_large.header().receive_time().seconds(),
             too_large.header().receive_time().nanos());
  EXPECT_EQ(file.records[0].index, 1u);
}

TEST(EpisodeWriterTest, W4CreateRejectsInvalidConfigWithoutClamping) {
  FakeEpisodeSink sink;
  struct Case {
    size_t capacity;
    size_t max_event_bytes;
  };
  const Case cases[] = {
      {0, 1024},
      {1, 1024},
      {65537, 1024},
      {4, 0},
      {4, kMaxEpisodeMaxEventBytes + 1},
  };
  for (const Case& c : cases) {
    EpisodeWriterConfig config;
    config.capacity = c.capacity;
    config.max_event_bytes = c.max_event_bytes;
    Status status = Status::kOk;
    EXPECT_EQ(EpisodeWriter::Create(config, &sink, &status), nullptr)
        << c.capacity << " " << c.max_event_bytes;
    EXPECT_EQ(status, Status::kInvalidConfig);
  }
  Status status = Status::kOk;
  EXPECT_EQ(EpisodeWriter::Create(EpisodeWriterConfig(), nullptr, &status),
            nullptr);
  EXPECT_EQ(status, Status::kInvalidConfig);
  EXPECT_EQ(sink.calls(), 0);
}

TEST(EpisodeWriterTest, W4CreateAcceptsTheBoundsAndTheDefaults) {
  FakeEpisodeSink sink;
  for (const auto& bounds :
       {std::pair<size_t, size_t>{2, 1}, std::pair<size_t, size_t>{65536, 1}}) {
    EpisodeWriterConfig config;
    config.capacity = bounds.first;
    config.max_event_bytes = bounds.second;
    Status status = Status::kInvalidConfig;
    EXPECT_NE(EpisodeWriter::Create(config, &sink, &status), nullptr);
    EXPECT_EQ(status, Status::kOk);
  }
  const EpisodeWriterConfig defaults;
  EXPECT_EQ(defaults.capacity, kDefaultEpisodeWriterCapacity);
  EXPECT_EQ(defaults.max_event_bytes, kDefaultEpisodeMaxEventBytes);
  Status status = Status::kInvalidConfig;
  EXPECT_NE(EpisodeWriter::Create(defaults, &sink, &status), nullptr);
  EXPECT_EQ(status, Status::kOk);
}

TEST(EpisodeWriterTest, W4LossRecordsFitEvenWhenMaxEventBytesIsOne) {
  FakeEpisodeSink sink;
  auto writer = MakeWriter(&sink, 2, 1);
  const EpisodeEvent event = Golden("safety_decision");
  EXPECT_EQ(writer->Append(event).status, Status::kEventTooLarge);
  ASSERT_EQ(writer->Close(), Status::kOk);
  const DecodedFile file = Decode(ToString(sink.output));
  ASSERT_EQ(file.records.size(), 1u);
  EXPECT_TRUE(file.records[0].crc_ok);
}

TEST(EpisodeWriterTest, W5OverflowGolden) {
  FakeEpisodeSink sink;
  auto writer = MakeWriter(&sink, 4, kSmallMaxEventBytes);
  for (size_t i = 0; i < 3; ++i) {
    const EpisodeAppendResult result = writer->Append(Golden(kPayloadNames[i]));
    EXPECT_EQ(result.status, Status::kOk);
    EXPECT_EQ(result.writer_index, i);
  }
  for (size_t i = 3; i < 5; ++i) {
    const EpisodeAppendResult result = writer->Append(Golden(kPayloadNames[i]));
    EXPECT_EQ(result.status, Status::kBufferFull);
    EXPECT_EQ(result.writer_index, i);
  }
  EpisodeWriterStats stats = writer->Stats();
  EXPECT_EQ(stats.accepted, 3u);
  EXPECT_EQ(stats.dropped_overflow, 2u);
  EXPECT_EQ(stats.queued_records, 3u);
  EXPECT_TRUE(stats.pending_loss);
  EXPECT_EQ(stats.next_writer_index, 5u);
  EXPECT_EQ(sink.calls(), 0);

  ASSERT_EQ(writer->Close(), Status::kOk);
  const std::string output = ToString(sink.output);
  EXPECT_EQ(output, TestdataBytes("episode_writer_v1_overflow.iepisod"));
  const DecodedFile file = Decode(output);
  EXPECT_TRUE(file.well_formed);
  ASSERT_EQ(file.records.size(), 4u);
  for (size_t i = 0; i < 3; ++i) {
    EXPECT_EQ(file.records[i].index, i);
    EXPECT_EQ(file.records[i].bytes, GoldenBytes(kPayloadNames[i]));
  }
  const EpisodeEvent last = Golden("metadata");
  ExpectLoss(file.records[3], 1, 2, 3, 4, EpisodeLoss::LOSS_REASON_OVERFLOW,
             last.header().receive_time().seconds(),
             last.header().receive_time().nanos());
  EXPECT_EQ(file.records[3].index, 5u);
  for (const DecodedRecord& record : file.records) {
    EXPECT_TRUE(record.crc_ok);
  }

  stats = writer->Stats();
  EXPECT_EQ(stats.accepted, 3u);
  EXPECT_EQ(stats.loss_records, 1u);
  EXPECT_EQ(stats.written_records, 4u);
  EXPECT_EQ(stats.dropped_overflow, 2u);
  EXPECT_EQ(stats.dropped_too_large, 0u);
  EXPECT_EQ(stats.queued_records, 0u);
  EXPECT_FALSE(stats.pending_loss);
  EXPECT_EQ(stats.next_writer_index, 6u);
}

TEST(EpisodeWriterTest, W5TheLossRecordPassesThePolicy) {
  FakeEpisodeSink sink;
  auto writer = MakeWriter(&sink, 2, kSmallMaxEventBytes);
  ASSERT_EQ(writer->Append(MakeEvent("a", 1)).status, Status::kOk);
  ASSERT_EQ(writer->Append(MakeEvent("a", 2)).status, Status::kBufferFull);
  ASSERT_EQ(writer->Close(), Status::kOk);
  const DecodedFile file = Decode(ToString(sink.output));
  ASSERT_EQ(file.records.size(), 2u);
  const EpisodeEvent loss = Parse(file.records[1].bytes);
  EpisodeEventView view;
  view.header_present = loss.has_header();
  view.source_id = loss.header().source_id();
  view.frame_id = loss.header().frame_id();
  view.source_time_present = loss.header().has_source_time();
  view.source_time = {loss.header().source_time().seconds(),
                      loss.header().source_time().nanos()};
  view.receive_time_present = loss.header().has_receive_time();
  view.receive_time = {loss.header().receive_time().seconds(),
                       loss.header().receive_time().nanos()};
  view.schema_version = loss.schema_version();
  view.payload_case = static_cast<int>(loss.payload_case());
  const EpisodeEventAssessment assessment = AssessEpisodeEvent(view);
  EXPECT_EQ(assessment.error, EpisodeEventError::kNone);
  EXPECT_EQ(assessment.payload_kind, PayloadKind::kEpisodeLoss);
  EXPECT_FALSE(assessment.skip);
}

TEST(EpisodeWriterTest, W5AcceptAfterADropWritesTheLossRightBeforeTheEvent) {
  FakeEpisodeSink sink;
  auto writer = MakeWriter(&sink, 4, kSmallMaxEventBytes);
  for (uint64_t sequence = 1; sequence <= 3; ++sequence) {
    ASSERT_EQ(writer->Append(MakeEvent("a", sequence)).status, Status::kOk);
  }
  const EpisodeEvent dropped = MakeEvent("a", 4);
  EXPECT_EQ(writer->Append(dropped).status, Status::kBufferFull);
  ASSERT_EQ(writer->Flush(), Status::kOk);
  EXPECT_EQ(writer->Stats().queued_records, 0u);
  const EpisodeAppendResult after = writer->Append(MakeEvent("a", 5));
  EXPECT_EQ(after.status, Status::kOk);
  EXPECT_EQ(after.writer_index, 5u);
  ASSERT_EQ(writer->Close(), Status::kOk);

  const DecodedFile file = Decode(ToString(sink.output));
  EXPECT_TRUE(file.well_formed);
  ASSERT_EQ(file.records.size(), 5u);
  const uint64_t expected_indices[] = {0, 1, 2, 4, 5};
  for (size_t i = 0; i < 5; ++i) {
    EXPECT_EQ(file.records[i].index, expected_indices[i]);
  }
  ExpectLoss(file.records[3], 1, 1, 3, 3, EpisodeLoss::LOSS_REASON_OVERFLOW,
             dropped.header().receive_time().seconds(),
             dropped.header().receive_time().nanos());
}

TEST(EpisodeWriterTest, W5AcceptWithAPendingLossMaterializesItFirst) {
  const size_t size = MakeEvent("a", 1, 300).ByteSizeLong();
  FakeEpisodeSink sink;
  auto writer = MakeWriter(&sink, 4, size);
  ASSERT_EQ(writer->Append(MakeEvent("a", 1)).status, Status::kOk);
  const EpisodeEvent too_large = MakeEvent("a", 2, 301);
  const EpisodeAppendResult drop = writer->Append(too_large);
  EXPECT_EQ(drop.status, Status::kEventTooLarge);
  EXPECT_EQ(drop.writer_index, 1u);
  const EpisodeAppendResult accepted = writer->Append(MakeEvent("a", 3));
  EXPECT_EQ(accepted.status, Status::kOk);
  EXPECT_EQ(accepted.writer_index, 3u);
  EXPECT_FALSE(writer->Stats().pending_loss);
  EXPECT_EQ(writer->Stats().queued_records, 3u);
  ASSERT_EQ(writer->Close(), Status::kOk);

  const DecodedFile file = Decode(ToString(sink.output));
  ASSERT_EQ(file.records.size(), 3u);
  EXPECT_EQ(file.records[0].index, 0u);
  EXPECT_EQ(file.records[1].index, 2u);
  EXPECT_EQ(file.records[2].index, 3u);
  ExpectLoss(file.records[1], 1, 1, 1, 1, EpisodeLoss::LOSS_REASON_TOO_LARGE,
             too_large.header().receive_time().seconds(),
             too_large.header().receive_time().nanos());
  EXPECT_EQ(file.records[2].bytes, MakeEvent("a", 3).SerializeAsString());
}

TEST(EpisodeWriterTest, W5ReasonSwitchStartsASecondLossRecord) {
  const size_t size = MakeEvent("a", 1, 300).ByteSizeLong();
  FakeEpisodeSink sink;
  auto writer = MakeWriter(&sink, 3, size);
  ASSERT_EQ(writer->Append(MakeEvent("a", 1)).status, Status::kOk);
  const EpisodeEvent too_large = MakeEvent("a", 2, 301);
  const EpisodeAppendResult first = writer->Append(too_large);
  EXPECT_EQ(first.status, Status::kEventTooLarge);
  EXPECT_EQ(first.writer_index, 1u);

  const EpisodeEvent overflow1 = MakeEvent("a", 3);
  const EpisodeAppendResult second = writer->Append(overflow1);
  EXPECT_EQ(second.status, Status::kBufferFull);
  EXPECT_EQ(second.writer_index, 3u);
  EXPECT_EQ(writer->Stats().queued_records, 2u);
  EXPECT_TRUE(writer->Stats().pending_loss);

  const EpisodeEvent overflow2 = MakeEvent("a", 4);
  const EpisodeAppendResult third = writer->Append(overflow2);
  EXPECT_EQ(third.status, Status::kBufferFull);
  EXPECT_EQ(third.writer_index, 4u);
  ASSERT_EQ(writer->Close(), Status::kOk);

  const DecodedFile file = Decode(ToString(sink.output));
  EXPECT_TRUE(file.well_formed);
  ASSERT_EQ(file.records.size(), 3u);
  EXPECT_EQ(file.records[0].index, 0u);
  ExpectLoss(file.records[1], 1, 1, 1, 1, EpisodeLoss::LOSS_REASON_TOO_LARGE,
             too_large.header().receive_time().seconds(),
             too_large.header().receive_time().nanos());
  ExpectLoss(file.records[2], 2, 2, 3, 4, EpisodeLoss::LOSS_REASON_OVERFLOW,
             overflow2.header().receive_time().seconds(),
             overflow2.header().receive_time().nanos());
  EXPECT_EQ(file.records[1].index, 2u);
  EXPECT_EQ(file.records[2].index, 5u);

  const EpisodeWriterStats stats = writer->Stats();
  EXPECT_EQ(stats.accepted, 1u);
  EXPECT_EQ(stats.loss_records, 2u);
  EXPECT_EQ(stats.dropped_too_large, 1u);
  EXPECT_EQ(stats.dropped_overflow, 2u);
  EXPECT_EQ(stats.written_records, 3u);
  EXPECT_EQ(stats.next_writer_index, 6u);
}

TEST(EpisodeWriterTest, W5SequentialLossesOfBothReasons) {
  const size_t size = MakeEvent("a", 1, 300).ByteSizeLong();
  FakeEpisodeSink sink;
  auto writer = MakeWriter(&sink, 5, size);
  for (uint64_t sequence = 1; sequence <= 4; ++sequence) {
    ASSERT_EQ(writer->Append(MakeEvent("a", sequence)).status, Status::kOk);
  }
  EXPECT_EQ(writer->Append(MakeEvent("a", 5)).status, Status::kBufferFull);
  ASSERT_EQ(writer->Flush(), Status::kOk);
  EXPECT_EQ(writer->Append(MakeEvent("a", 6)).status, Status::kOk);
  const EpisodeEvent too_large = MakeEvent("a", 7, 301);
  EXPECT_EQ(writer->Append(too_large).status, Status::kEventTooLarge);
  ASSERT_EQ(writer->Close(), Status::kOk);

  const DecodedFile file = Decode(ToString(sink.output));
  EXPECT_TRUE(file.well_formed);
  ASSERT_EQ(file.records.size(), 7u);
  const uint64_t expected_indices[] = {0, 1, 2, 3, 5, 6, 8};
  for (size_t i = 0; i < 7; ++i) {
    EXPECT_EQ(file.records[i].index, expected_indices[i]) << i;
  }
  const EpisodeEvent dropped = Golden("safety_decision");
  ExpectLoss(file.records[4], 1, 1, 4, 4, EpisodeLoss::LOSS_REASON_OVERFLOW,
             dropped.header().receive_time().seconds(),
             dropped.header().receive_time().nanos());
  ExpectLoss(file.records[6], 2, 1, 7, 7, EpisodeLoss::LOSS_REASON_TOO_LARGE,
             too_large.header().receive_time().seconds(),
             too_large.header().receive_time().nanos());
}

TEST(EpisodeWriterTest, W5ATooLargeEventOnAFullRingIsAnOverflow) {
  const size_t size = MakeEvent("a", 1, 300).ByteSizeLong();
  FakeEpisodeSink sink;
  auto writer = MakeWriter(&sink, 2, size);
  ASSERT_EQ(writer->Append(MakeEvent("a", 1)).status, Status::kOk);
  EXPECT_EQ(writer->Append(MakeEvent("a", 2, 301)).status, Status::kBufferFull);
  EXPECT_EQ(writer->Stats().dropped_overflow, 1u);
  EXPECT_EQ(writer->Stats().dropped_too_large, 0u);
}

TEST(EpisodeWriterTest, W5NothingQueuedIsTouchedByADrop) {
  FakeEpisodeSink sink;
  auto writer = MakeWriter(&sink, 3, kSmallMaxEventBytes);
  ASSERT_EQ(writer->Append(MakeEvent("a", 1)).status, Status::kOk);
  ASSERT_EQ(writer->Append(MakeEvent("a", 2)).status, Status::kOk);
  for (uint64_t sequence = 3; sequence < 8; ++sequence) {
    EXPECT_EQ(writer->Append(MakeEvent("a", sequence)).status,
              Status::kBufferFull);
  }
  EXPECT_EQ(writer->Stats().queued_records, 2u);
  ASSERT_EQ(writer->Close(), Status::kOk);
  const DecodedFile file = Decode(ToString(sink.output));
  ASSERT_EQ(file.records.size(), 3u);
  EXPECT_EQ(file.records[0].bytes, MakeEvent("a", 1).SerializeAsString());
  EXPECT_EQ(file.records[1].bytes, MakeEvent("a", 2).SerializeAsString());
  ExpectLoss(file.records[2], 1, 5, 2, 6, EpisodeLoss::LOSS_REASON_OVERFLOW,
             Golden("safety_decision").header().receive_time().seconds(),
             Golden("safety_decision").header().receive_time().nanos());
}

TEST(EpisodeWriterTest, W6DuplicateAndLowerSequencesAreOutOfOrder) {
  FakeEpisodeSink sink;
  auto writer = MakeWriter(&sink, 16, kSmallMaxEventBytes);
  EXPECT_EQ(writer->Append(MakeEvent("a", 5)).status, Status::kOk);
  EXPECT_EQ(writer->Append(MakeEvent("a", 5)).status, Status::kOutOfOrder);
  EXPECT_EQ(writer->Append(MakeEvent("a", 4)).status, Status::kOutOfOrder);
  EXPECT_EQ(writer->Append(MakeEvent("a", 0)).status, Status::kOutOfOrder);
  EXPECT_EQ(writer->Append(MakeEvent("a", 6)).status, Status::kOk);
  const EpisodeWriterStats stats = writer->Stats();
  EXPECT_EQ(stats.rejected_out_of_order, 3u);
  EXPECT_EQ(stats.accepted, 2u);
  EXPECT_EQ(stats.next_writer_index, 2u);
}

TEST(EpisodeWriterTest, W6InterleavedSourcesAreIndependent) {
  FakeEpisodeSink sink;
  auto writer = MakeWriter(&sink, 16, kSmallMaxEventBytes);
  EXPECT_EQ(writer->Append(MakeEvent("a", 1)).status, Status::kOk);
  EXPECT_EQ(writer->Append(MakeEvent("b", 1)).status, Status::kOk);
  EXPECT_EQ(writer->Append(MakeEvent("a", 2)).status, Status::kOk);
  EXPECT_EQ(writer->Append(MakeEvent("b", 2)).status, Status::kOk);
  EXPECT_EQ(writer->Append(MakeEvent("a", 2)).status, Status::kOutOfOrder);
  EXPECT_EQ(writer->Append(MakeEvent("b", 1)).status, Status::kOutOfOrder);
}

TEST(EpisodeWriterTest, W6AFirstEventMayHaveSequenceZero) {
  FakeEpisodeSink sink;
  auto writer = MakeWriter(&sink, 16, kSmallMaxEventBytes);
  EXPECT_EQ(writer->Append(MakeEvent("a", 0)).status, Status::kOk);
  EXPECT_EQ(writer->Append(MakeEvent("a", 0)).status, Status::kOutOfOrder);
  EXPECT_EQ(writer->Append(MakeEvent("a", 1)).status, Status::kOk);
}

TEST(EpisodeWriterTest, W6ProducerSequenceIsNeverRewritten) {
  FakeEpisodeSink sink;
  auto writer = MakeWriter(&sink, 16, kSmallMaxEventBytes);
  ASSERT_EQ(writer->Append(MakeEvent("a", 1000)).status, Status::kOk);
  ASSERT_EQ(writer->Close(), Status::kOk);
  const DecodedFile file = Decode(ToString(sink.output));
  ASSERT_EQ(file.records.size(), 1u);
  EXPECT_EQ(Parse(file.records[0].bytes).header().sequence(), 1000u);
}

TEST(EpisodeWriterTest, W6DroppedAndRejectedEventsDoNotAdvanceTheTable) {
  FakeEpisodeSink sink;
  auto writer = MakeWriter(&sink, 3, kSmallMaxEventBytes);
  ASSERT_EQ(writer->Append(MakeEvent("a", 1)).status, Status::kOk);
  ASSERT_EQ(writer->Append(MakeEvent("a", 2)).status, Status::kOk);
  EXPECT_EQ(writer->Append(MakeEvent("a", 50)).status, Status::kBufferFull);
  EXPECT_EQ(writer->Append(MakeEvent("a", 2)).status, Status::kOutOfOrder);
  ASSERT_EQ(writer->Flush(), Status::kOk);
  EXPECT_EQ(writer->Append(MakeEvent("a", 3)).status, Status::kOk);
  EXPECT_EQ(writer->Append(MakeEvent("a", 3)).status, Status::kOutOfOrder);

  EpisodeEvent invalid = MakeEvent("b", 90);
  invalid.clear_payload();
  EXPECT_EQ(writer->Append(invalid).status, Status::kInvalidEvent);
  EXPECT_EQ(writer->Append(MakeEvent("b", 1)).status, Status::kOk);
}

TEST(EpisodeWriterTest, W6ATooLargeEventDoesNotAdvanceTheTable) {
  const size_t size = MakeEvent("a", 1, 300).ByteSizeLong();
  FakeEpisodeSink sink;
  auto writer = MakeWriter(&sink, 8, size);
  ASSERT_EQ(writer->Append(MakeEvent("a", 1)).status, Status::kOk);
  EXPECT_EQ(writer->Append(MakeEvent("a", 90, 301)).status,
            Status::kEventTooLarge);
  EXPECT_EQ(writer->Append(MakeEvent("a", 2)).status, Status::kOk);
  EXPECT_EQ(writer->Append(MakeEvent("new_source", 1, 301)).status,
            Status::kEventTooLarge);
  EXPECT_EQ(writer->Append(MakeEvent("new_source", 1)).status, Status::kOk);
}

TEST(EpisodeWriterTest, W7SixtyFourSourcesAreAccepted) {
  FakeEpisodeSink sink;
  auto writer = MakeWriter(&sink, 128, kSmallMaxEventBytes);
  for (size_t i = 0; i < kMaxEpisodeSources; ++i) {
    EXPECT_EQ(
        writer->Append(MakeEvent("source_" + std::to_string(i), 1)).status,
        Status::kOk)
        << i;
  }
  const EpisodeAppendResult result = writer->Append(MakeEvent("source_64", 1));
  EXPECT_EQ(result.status, Status::kTooManySources);
  EXPECT_EQ(writer->Stats().rejected_too_many_sources, 1u);
  EXPECT_EQ(writer->Append(MakeEvent("source_0", 2)).status, Status::kOk);
  EXPECT_EQ(writer->Append(MakeEvent("source_63", 2)).status, Status::kOk);
  EXPECT_EQ(writer->Append(MakeEvent("source_0", 2)).status,
            Status::kOutOfOrder);
  EXPECT_EQ(writer->Append(MakeEvent("source_64", 1)).status,
            Status::kTooManySources);
  EXPECT_EQ(writer->Stats().next_writer_index, 66u);
}

TEST(EpisodeWriterTest, W8EveryEventErrorIsRejectedAndEchoed) {
  FakeEpisodeSink sink;
  auto writer = MakeWriter(&sink, 16, kSmallMaxEventBytes);
  const std::string upper(64, 'A');
  const struct {
    EpisodeEventError error;
    EpisodeEvent event;
  } cases[] = {
      {EpisodeEventError::kMissingHeader,
       [] {
         EpisodeEvent e = MakeEvent("a", 1);
         e.clear_header();
         return e;
       }()},
      {EpisodeEventError::kSourceId,
       [] {
         EpisodeEvent e = MakeEvent("a", 1);
         e.mutable_header()->clear_source_id();
         return e;
       }()},
      {EpisodeEventError::kMissingFrame,
       [] {
         EpisodeEvent e = MakeEvent("a", 1);
         e.mutable_header()->clear_frame_id();
         return e;
       }()},
      {EpisodeEventError::kSourceTime,
       [] {
         EpisodeEvent e = MakeEvent("a", 1);
         e.mutable_header()->clear_source_time();
         return e;
       }()},
      {EpisodeEventError::kReceiveTime,
       [] {
         EpisodeEvent e = MakeEvent("a", 1);
         e.mutable_header()->mutable_receive_time()->set_nanos(1000000000);
         return e;
       }()},
      {EpisodeEventError::kSchemaVersion,
       [] {
         EpisodeEvent e = MakeEvent("a", 1);
         e.set_schema_version(0);
         return e;
       }()},
      {EpisodeEventError::kSnapshotId,
       [&upper] {
         EpisodeEvent e = MakeEvent("a", 1);
         e.set_snapshot_id(upper);
         return e;
       }()},
      {EpisodeEventError::kConfigDigest,
       [&upper] {
         EpisodeEvent e = MakeEvent("a", 1);
         e.set_config_digest(upper);
         return e;
       }()},
      {EpisodeEventError::kConfigDigestMismatch,
       [] {
         EpisodeEvent e = Golden("metadata");
         e.clear_config_digest();
         return e;
       }()},
      {EpisodeEventError::kMissingPayload,
       [] {
         EpisodeEvent e = MakeEvent("a", 1);
         e.clear_payload();
         return e;
       }()},
  };
  int expected_error = 1;
  for (const auto& c : cases) {
    EXPECT_EQ(static_cast<int>(c.error), expected_error++);
    const EpisodeAppendResult result = writer->Append(c.event);
    EXPECT_EQ(result.status, Status::kInvalidEvent)
        << static_cast<int>(c.error);
    EXPECT_EQ(result.event_error, c.error);
  }
  EXPECT_EQ(expected_error, 11);
  EXPECT_EQ(writer->Stats().rejected_invalid, 10u);
  EXPECT_EQ(writer->Stats().next_writer_index, 0u);
}

TEST(EpisodeWriterTest, W8EmptyMessageIsInvalidWithNoError) {
  FakeEpisodeSink sink;
  auto writer = MakeWriter(&sink, 4, kSmallMaxEventBytes);
  const EpisodeAppendResult result = writer->Append(EpisodeEvent());
  EXPECT_EQ(result.status, Status::kInvalidEvent);
  EXPECT_EQ(result.event_error, EpisodeEventError::kNone);
  EXPECT_EQ(writer->Stats().rejected_invalid, 1u);
}

TEST(EpisodeWriterTest, W8ProducersCannotForgeLossRecords) {
  FakeEpisodeSink sink;
  auto writer = MakeWriter(&sink, 16, kSmallMaxEventBytes);
  EXPECT_EQ(
      writer->Append(MakeEvent(std::string(kEpisodeWriterSourceId), 1)).status,
      Status::kReservedEvent);

  EpisodeEvent forged = MakeEvent("producer", 1);
  forged.mutable_episode_loss()->set_dropped_count(1);
  EXPECT_EQ(forged.payload_case(), EpisodeEvent::kEpisodeLoss);
  const EpisodeAppendResult result = writer->Append(forged);
  EXPECT_EQ(result.status, Status::kReservedEvent);
  EXPECT_EQ(writer->Stats().rejected_reserved, 2u);

  ASSERT_EQ(writer->Close(), Status::kOk);
  EXPECT_EQ(sink.output.size(), kEpisodeFileHeaderBytes);
  EXPECT_EQ(writer->Stats().next_writer_index, 0u);
  EXPECT_EQ(writer->Stats().accepted, 0u);
}

TEST(EpisodeWriterTest, W8InvalidIsCheckedBeforeReserved) {
  FakeEpisodeSink sink;
  auto writer = MakeWriter(&sink, 16, kSmallMaxEventBytes);
  EpisodeEvent event = MakeEvent(std::string(kEpisodeWriterSourceId), 1);
  event.clear_payload();
  EXPECT_EQ(writer->Append(event).status, Status::kInvalidEvent);
}

TEST(EpisodeWriterTest, W8RejectionsWriteNothingAndConsumeNoIndex) {
  FakeEpisodeSink sink;
  auto writer = MakeWriter(&sink, 16, kSmallMaxEventBytes);
  EpisodeEvent invalid = MakeEvent("a", 1);
  invalid.clear_header();
  EXPECT_EQ(writer->Append(invalid).status, Status::kInvalidEvent);
  EXPECT_EQ(
      writer->Append(MakeEvent(std::string(kEpisodeWriterSourceId), 1)).status,
      Status::kReservedEvent);
  const EpisodeAppendResult accepted = writer->Append(MakeEvent("a", 1));
  EXPECT_EQ(accepted.status, Status::kOk);
  EXPECT_EQ(accepted.writer_index, 0u);
  EXPECT_EQ(writer->Append(MakeEvent("a", 1)).status, Status::kOutOfOrder);
  EXPECT_EQ(writer->Stats().queued_records, 1u);
  EXPECT_EQ(writer->Stats().next_writer_index, 1u);
  EXPECT_EQ(sink.calls(), 0);
}

// #240: a payload-less event whose only extra fields are unknown envelope
// fields (U1 tag 6 varint, U2 tag 6 length-delimited, U4 wrong-wire tag 10) is
// not an unknown payload. It is rejected and consumes no writer index.
TEST(EpisodeWriterTest, U1U2U4UnknownEnvelopeFieldsAreMissingPayload) {
  EpisodeEvent base = MakeEvent("a", 1);
  base.clear_payload();
  const std::string prefix = base.SerializeAsString();
  const std::string suffixes[] = {
      std::string("\x30\x01", 2),
      std::string("\x32\x01\x00", 3),
      std::string("\x50\x01", 2),
  };
  FakeEpisodeSink sink;
  auto writer = MakeWriter(&sink, 16, kSmallMaxEventBytes);
  uint64_t rejected = 0;
  for (const std::string& suffix : suffixes) {
    const EpisodeEvent event = Parse(prefix + suffix);
    EXPECT_EQ(event.payload_case(), EpisodeEvent::PAYLOAD_NOT_SET);
    const EpisodeAppendResult result = writer->Append(event);
    EXPECT_EQ(result.status, Status::kInvalidEvent);
    EXPECT_EQ(result.event_error, EpisodeEventError::kMissingPayload);
    EXPECT_EQ(writer->Stats().rejected_invalid, ++rejected);
    EXPECT_EQ(writer->Stats().next_writer_index, 0u);
    EXPECT_EQ(writer->Stats().accepted, 0u);
  }
  EXPECT_EQ(sink.calls(), 0);
  const EpisodeAppendResult accepted = writer->Append(MakeEvent("a", 1));
  EXPECT_EQ(accepted.status, Status::kOk);
  EXPECT_EQ(accepted.writer_index, 0u);
  EXPECT_EQ(writer->Stats().next_writer_index, 1u);
}

TEST(EpisodeWriterTest, W9UnknownPayloadIsAcceptedAndItsBytesAreKept) {
  const std::string golden = TestdataBytes(std::string(kV2Name));
  FakeEpisodeSink sink;
  auto writer = MakeWriter(&sink, 4, kSmallMaxEventBytes);
  const EpisodeAppendResult result = writer->Append(Parse(golden));
  EXPECT_EQ(result.status, Status::kOk);
  EXPECT_EQ(result.writer_index, 0u);
  ASSERT_EQ(writer->Close(), Status::kOk);
  const DecodedFile file = Decode(ToString(sink.output));
  ASSERT_EQ(file.records.size(), 1u);
  EXPECT_TRUE(file.records[0].crc_ok);
  EXPECT_EQ(file.records[0].bytes, golden);
}

TEST(EpisodeWriterTest, W10HeaderWriteFailureLatches) {
  FakeEpisodeSink sink;
  sink.fail_write_call = 0;
  auto writer = MakeWriter(&sink, 8, kSmallMaxEventBytes);
  for (uint64_t sequence = 1; sequence <= 3; ++sequence) {
    ASSERT_EQ(writer->Append(MakeEvent("a", sequence)).status, Status::kOk);
  }
  EXPECT_EQ(writer->Flush(), Status::kWriteFailed);
  EXPECT_EQ(sink.write_calls, 1);
  EXPECT_EQ(writer->Stats().queued_records, 3u);
  EXPECT_EQ(writer->Stats().written_records, 0u);
  EXPECT_EQ(writer->FirstSinkError().code(), absl::StatusCode::kUnavailable);

  EXPECT_EQ(writer->Append(MakeEvent("a", 4)).status, Status::kWriteFailed);
  EXPECT_EQ(writer->Flush(), Status::kWriteFailed);
  EXPECT_EQ(writer->Close(), Status::kWriteFailed);
  EXPECT_EQ(sink.write_calls, 1);
  EXPECT_EQ(sink.sync_calls, 0);
  EXPECT_EQ(writer->Stats().queued_records, 3u);
  EXPECT_EQ(writer->Stats().rejected_after_stop, 1u);
}

TEST(EpisodeWriterTest, W10RecordWriteFailureLatches) {
  FakeEpisodeSink sink;
  sink.fail_write_call = 2;  // The header is call 0, record 0 is call 1.
  auto writer = MakeWriter(&sink, 8, kSmallMaxEventBytes);
  for (uint64_t sequence = 1; sequence <= 4; ++sequence) {
    ASSERT_EQ(writer->Append(MakeEvent("a", sequence)).status, Status::kOk);
  }
  EXPECT_EQ(writer->Flush(), Status::kWriteFailed);
  EXPECT_EQ(sink.write_calls, 3);
  EXPECT_EQ(sink.writes.size(), 2u);
  EXPECT_EQ(writer->Stats().written_records, 1u);
  EXPECT_EQ(writer->Stats().queued_records, 3u);
  EXPECT_EQ(writer->FirstSinkError().code(), absl::StatusCode::kUnavailable);
  EXPECT_EQ(writer->FirstSinkError().message(), "injected write failure");

  EXPECT_EQ(writer->Append(MakeEvent("a", 5)).status, Status::kWriteFailed);
  EXPECT_EQ(writer->Flush(), Status::kWriteFailed);
  EXPECT_EQ(writer->Close(), Status::kWriteFailed);
  EXPECT_EQ(sink.write_calls, 3);
  EXPECT_EQ(sink.sync_calls, 0);
  EXPECT_EQ(writer->Stats().queued_records, 3u);
  EXPECT_EQ(writer->FirstSinkError().message(), "injected write failure");
}

TEST(EpisodeWriterTest, W10CloseReportsAWriteFailureAndEndsClosed) {
  FakeEpisodeSink sink;
  sink.fail_write_call = 1;
  auto writer = MakeWriter(&sink, 8, kSmallMaxEventBytes);
  ASSERT_EQ(writer->Append(MakeEvent("a", 1)).status, Status::kOk);
  EXPECT_EQ(writer->Close(), Status::kWriteFailed);
  EXPECT_EQ(sink.write_calls, 2);
  EXPECT_EQ(sink.sync_calls, 0);
  EXPECT_EQ(writer->Stats().queued_records, 1u);
  EXPECT_EQ(writer->Close(), Status::kWriteFailed);
  EXPECT_EQ(writer->Append(MakeEvent("a", 2)).status, Status::kClosed);
  EXPECT_EQ(sink.calls(), 2);
}

TEST(EpisodeWriterTest, W10SyncFailureLatches) {
  FakeEpisodeSink sink;
  sink.fail_sync = true;
  auto writer = MakeWriter(&sink, 8, kSmallMaxEventBytes);
  ASSERT_EQ(writer->Append(MakeEvent("a", 1)).status, Status::kOk);
  EXPECT_EQ(writer->Close(), Status::kWriteFailed);
  EXPECT_EQ(sink.write_calls, 2);
  EXPECT_EQ(sink.sync_calls, 1);
  EXPECT_EQ(writer->Stats().queued_records, 0u);
  EXPECT_EQ(writer->Stats().written_records, 1u);
  EXPECT_EQ(writer->FirstSinkError().code(), absl::StatusCode::kDataLoss);

  EXPECT_EQ(writer->Close(), Status::kWriteFailed);
  EXPECT_EQ(writer->Append(MakeEvent("a", 2)).status, Status::kClosed);
  EXPECT_EQ(writer->Flush(), Status::kClosed);
  EXPECT_EQ(sink.write_calls, 2);
  EXPECT_EQ(sink.sync_calls, 1);
}

TEST(EpisodeWriterTest, W10TheFirstSinkErrorIsKept) {
  FakeEpisodeSink sink;
  sink.fail_write_call = 1;
  auto writer = MakeWriter(&sink, 8, kSmallMaxEventBytes);
  ASSERT_EQ(writer->Append(MakeEvent("a", 1)).status, Status::kOk);
  ASSERT_EQ(writer->Flush(), Status::kWriteFailed);
  const absl::Status first = writer->FirstSinkError();
  sink.fail_sync = true;
  EXPECT_EQ(writer->Close(), Status::kWriteFailed);
  EXPECT_EQ(writer->FirstSinkError(), first);
  EXPECT_EQ(sink.sync_calls, 0);
}

TEST(EpisodeWriterTest, W11FlushIsIdempotentAndNeverSyncs) {
  FakeEpisodeSink sink;
  auto writer = MakeWriter(&sink, 8, kSmallMaxEventBytes);
  ASSERT_EQ(writer->Append(MakeEvent("a", 1)).status, Status::kOk);
  ASSERT_EQ(writer->Append(MakeEvent("a", 2)).status, Status::kOk);
  EXPECT_EQ(writer->Flush(), Status::kOk);
  EXPECT_EQ(sink.write_calls, 3);
  EXPECT_EQ(sink.sync_calls, 0);
  EXPECT_EQ(writer->Flush(), Status::kOk);
  EXPECT_EQ(writer->Flush(), Status::kOk);
  EXPECT_EQ(sink.write_calls, 3);
  EXPECT_EQ(sink.sync_calls, 0);
  const std::vector<uint8_t> after_first = sink.output;

  ASSERT_EQ(writer->Append(MakeEvent("a", 3)).status, Status::kOk);
  EXPECT_EQ(writer->Flush(), Status::kOk);
  EXPECT_EQ(sink.write_calls, 4);
  EXPECT_EQ(sink.output.size(), after_first.size() + kEpisodeRecordHeaderBytes +
                                    MakeEvent("a", 3).ByteSizeLong());
}

TEST(EpisodeWriterTest, W11TheHeaderIsWrittenOnceByTheFirstFlush) {
  FakeEpisodeSink sink;
  auto writer = MakeWriter(&sink, 8, kSmallMaxEventBytes);
  EXPECT_EQ(writer->Flush(), Status::kOk);
  EXPECT_EQ(sink.output.size(), kEpisodeFileHeaderBytes);
  EXPECT_EQ(writer->Flush(), Status::kOk);
  EXPECT_EQ(writer->Close(), Status::kOk);
  EXPECT_EQ(sink.output.size(), kEpisodeFileHeaderBytes);
  EXPECT_EQ(sink.write_calls, 1);
  EXPECT_EQ(sink.sync_calls, 1);
}

TEST(EpisodeWriterTest, W11CloseTwiceAndUseAfterClose) {
  FakeEpisodeSink sink;
  auto writer = MakeWriter(&sink, 8, kSmallMaxEventBytes);
  ASSERT_EQ(writer->Append(MakeEvent("a", 1)).status, Status::kOk);
  EXPECT_EQ(writer->Close(), Status::kOk);
  const int calls = sink.calls();
  EXPECT_EQ(sink.sync_calls, 1);
  EXPECT_EQ(writer->Close(), Status::kOk);
  EXPECT_EQ(sink.calls(), calls);
  EXPECT_EQ(writer->Append(MakeEvent("a", 2)).status, Status::kClosed);
  EXPECT_EQ(writer->Flush(), Status::kClosed);
  EXPECT_EQ(sink.calls(), calls);
  EXPECT_EQ(writer->Stats().rejected_after_stop, 1u);
}

TEST(EpisodeWriterTest, W12CancelDiscardsEverythingWithoutSinkCalls) {
  FakeEpisodeSink sink;
  auto writer = MakeWriter(&sink, 4, kSmallMaxEventBytes);
  for (uint64_t sequence = 1; sequence <= 5; ++sequence) {
    writer->Append(MakeEvent("a", sequence));
  }
  EpisodeWriterStats stats = writer->Stats();
  ASSERT_EQ(stats.queued_records, 3u);
  ASSERT_TRUE(stats.pending_loss);

  EXPECT_EQ(writer->Cancel(), 3u);
  EXPECT_EQ(sink.calls(), 0);
  stats = writer->Stats();
  EXPECT_EQ(stats.dropped_on_cancel, 3u);
  EXPECT_EQ(stats.queued_records, 0u);
  EXPECT_FALSE(stats.pending_loss);
  EXPECT_EQ(stats.written_records, 0u);

  EXPECT_EQ(writer->Cancel(), 0u);
  EXPECT_EQ(writer->Stats().dropped_on_cancel, 3u);
  EXPECT_EQ(writer->Append(MakeEvent("a", 9)).status, Status::kCancelled);
  EXPECT_EQ(writer->Flush(), Status::kCancelled);
  EXPECT_EQ(writer->Close(), Status::kCancelled);
  EXPECT_EQ(writer->Close(), Status::kCancelled);
  EXPECT_EQ(sink.calls(), 0);
  EXPECT_EQ(writer->Stats().rejected_after_stop, 1u);
}

TEST(EpisodeWriterTest, W12CancelCountsMaterializedLossRecords) {
  const size_t size = MakeEvent("a", 1, 300).ByteSizeLong();
  FakeEpisodeSink sink;
  auto writer = MakeWriter(&sink, 4, size);
  ASSERT_EQ(writer->Append(MakeEvent("a", 1)).status, Status::kOk);
  ASSERT_EQ(writer->Append(MakeEvent("a", 2, 301)).status,
            Status::kEventTooLarge);
  ASSERT_EQ(writer->Append(MakeEvent("a", 3)).status, Status::kOk);
  EXPECT_EQ(writer->Stats().queued_records, 3u);
  EXPECT_EQ(writer->Stats().loss_records, 1u);
  EXPECT_EQ(writer->Cancel(), 3u);
  EXPECT_EQ(writer->Stats().dropped_on_cancel, 3u);
  EXPECT_EQ(sink.calls(), 0);
}

TEST(EpisodeWriterTest, W12CancelAfterAFlushDiscardsOnlyWhatIsQueued) {
  FakeEpisodeSink sink;
  auto writer = MakeWriter(&sink, 8, kSmallMaxEventBytes);
  ASSERT_EQ(writer->Append(MakeEvent("a", 1)).status, Status::kOk);
  ASSERT_EQ(writer->Flush(), Status::kOk);
  ASSERT_EQ(writer->Append(MakeEvent("a", 2)).status, Status::kOk);
  const int calls = sink.calls();
  EXPECT_EQ(writer->Cancel(), 1u);
  EXPECT_EQ(sink.calls(), calls);
  EXPECT_EQ(writer->Stats().written_records, 1u);
}

TEST(EpisodeWriterTest, W12CancelOnAnEmptyWriter) {
  FakeEpisodeSink sink;
  auto writer = MakeWriter(&sink, 8, kSmallMaxEventBytes);
  EXPECT_EQ(writer->Cancel(), 0u);
  EXPECT_EQ(writer->Close(), Status::kCancelled);
  EXPECT_EQ(sink.calls(), 0);
}

TEST(EpisodeWriterTest, ConcurrentAppendsAccountForEveryWriterIndex) {
  constexpr int kProducers = 4;
  constexpr uint64_t kEventsPerProducer = 500;
  const std::string prefix = "producer_";
  FakeEpisodeSink sink;
  auto writer = MakeWriter(&sink, 16, kSmallMaxEventBytes);
  ASSERT_NE(writer, nullptr);
  std::atomic<int> running{kProducers};
  std::vector<std::thread> producers;
  for (int p = 0; p < kProducers; ++p) {
    producers.emplace_back([&writer, &running, &prefix, p] {
      for (uint64_t sequence = 1; sequence <= kEventsPerProducer; ++sequence) {
        writer->Append(MakeEvent(prefix + std::to_string(p), sequence));
      }
      running.fetch_sub(1);
    });
  }
  while (running.load() > 0) {
    EXPECT_EQ(writer->Flush(), Status::kOk);
  }
  for (std::thread& producer : producers) {
    producer.join();
  }
  ASSERT_EQ(writer->Close(), Status::kOk);

  const EpisodeWriterStats stats = writer->Stats();
  EXPECT_EQ(stats.accepted + stats.dropped_overflow + stats.dropped_too_large +
                stats.loss_records,
            stats.next_writer_index);
  EXPECT_EQ(stats.accepted + stats.dropped_overflow,
            kProducers * kEventsPerProducer);

  const DecodedFile file = Decode(ToString(sink.output));
  ASSERT_TRUE(file.well_formed);
  EXPECT_EQ(file.records.size(), stats.written_records);
  std::vector<int> covered(stats.next_writer_index, 0);
  uint64_t previous_index = 0;
  bool first = true;
  std::vector<uint64_t> last_sequence(kProducers, 0);
  for (const DecodedRecord& record : file.records) {
    EXPECT_TRUE(record.crc_ok);
    EXPECT_TRUE(first || record.index > previous_index);
    first = false;
    previous_index = record.index;
    ASSERT_LT(record.index, covered.size());
    ++covered[record.index];
    const EpisodeEvent event = Parse(record.bytes);
    if (event.payload_case() == EpisodeEvent::kEpisodeLoss) {
      const EpisodeLoss& loss = event.episode_loss();
      EXPECT_EQ(loss.dropped_count(), loss.last_dropped_writer_index() -
                                          loss.first_dropped_writer_index() +
                                          1);
      EXPECT_EQ(record.index, loss.last_dropped_writer_index() + 1);
      ASSERT_LT(loss.last_dropped_writer_index(), covered.size());
      for (uint64_t i = loss.first_dropped_writer_index();
           i <= loss.last_dropped_writer_index(); ++i) {
        ++covered[i];
      }
    } else {
      const std::string& source = event.header().source_id();
      const int producer = std::stoi(source.substr(prefix.size()));
      EXPECT_GT(event.header().sequence(), last_sequence[producer]);
      last_sequence[producer] = event.header().sequence();
    }
  }
  for (size_t i = 0; i < covered.size(); ++i) {
    EXPECT_EQ(covered[i], 1) << "writer index " << i;
  }
}

std::string RunScript() {
  FakeEpisodeSink sink;
  auto writer = MakeWriter(&sink, 4, kSmallMaxEventBytes);
  for (size_t i = 0; i < 5; ++i) {
    writer->Append(Golden(kPayloadNames[i]));
  }
  EXPECT_EQ(writer->Close(), Status::kOk);
  return ToString(sink.output);
}

TEST(EpisodeWriterTest, W13OutputIsDeterministic) {
  EXPECT_EQ(RunScript(), RunScript());
}

std::string TestTmpDir() {
  const char* tmp = std::getenv("TEST_TMPDIR");
  return tmp != nullptr ? tmp : "/tmp";
}

TEST(EpisodeWriterTest, W13FileSinkProducesTheSameBytesAsTheFake) {
  const std::string path = TestTmpDir() + "/episode_writer_test_w13.iepisod";
  std::remove(path.c_str());
  auto file_sink = FileEpisodeSink::Open(path);
  ASSERT_TRUE(file_sink.ok()) << file_sink.status();
  {
    auto writer = MakeWriter(file_sink->get(), 4, kSmallMaxEventBytes);
    for (size_t i = 0; i < 5; ++i) {
      writer->Append(Golden(kPayloadNames[i]));
    }
    ASSERT_EQ(writer->Close(), Status::kOk);
  }
  std::ifstream in(path, std::ios::binary);
  std::ostringstream contents;
  contents << in.rdbuf();
  EXPECT_EQ(contents.str(), RunScript());
  EXPECT_EQ(contents.str(),
            TestdataBytes("episode_writer_v1_overflow.iepisod"));
}

TEST(EpisodeWriterTest, W13FileSinkNeverOverwritesAnExistingFile) {
  const std::string path = TestTmpDir() + "/episode_writer_test_exists.iepisod";
  std::remove(path.c_str());
  {
    auto first = FileEpisodeSink::Open(path);
    ASSERT_TRUE(first.ok()) << first.status();
    const uint8_t bytes[] = {1, 2, 3};
    ASSERT_TRUE((*first)->Write(bytes).ok());
    ASSERT_TRUE((*first)->Sync().ok());
  }
  const auto second = FileEpisodeSink::Open(path);
  EXPECT_FALSE(second.ok());
  EXPECT_EQ(second.status().code(), absl::StatusCode::kAlreadyExists);
  std::ifstream in(path, std::ios::binary);
  std::ostringstream contents;
  contents << in.rdbuf();
  EXPECT_EQ(contents.str(), std::string("\x01\x02\x03", 3));
}

TEST(EpisodeWriterTest, W13FileSinkOpenFailsInAMissingDirectory) {
  const auto sink =
      FileEpisodeSink::Open(TestTmpDir() + "/no_such_directory/file.iepisod");
  EXPECT_FALSE(sink.ok());
}

// W14. Each Append runs with the allocation counter armed and must not
// allocate. Everything the call needs is built before arming. Case-14
// EpisodeMetadata events with non-empty text are excluded on purpose: they
// allocate one 64-character hex string inside ComputeConfigDigest (contract
// section 5). That exception is accepted for host-only use.
class AllocationGuard {
 public:
  AllocationGuard() {
    g_allocations.store(0);
    g_count_allocations.store(true);
  }
  ~AllocationGuard() { g_count_allocations.store(false); }
  uint64_t count() const { return g_allocations.load(); }
};

EpisodeAppendResult AppendCounting(EpisodeWriter* writer,
                                   const EpisodeEvent& event,
                                   uint64_t* allocations) {
  AllocationGuard guard;
  const EpisodeAppendResult result = writer->Append(event);
  g_count_allocations.store(false);
  *allocations = guard.count();
  return result;
}

#define EXPECT_APPEND_WITHOUT_ALLOCATION(writer, event, expected_status) \
  do {                                                                   \
    uint64_t allocations = 0;                                            \
    const EpisodeAppendResult result =                                   \
        AppendCounting((writer).get(), (event), &allocations);           \
    EXPECT_EQ(result.status, (expected_status));                         \
    EXPECT_EQ(allocations, 0u) << #event;                                \
  } while (false)

TEST(EpisodeWriterTest, W14AppendDoesNotAllocate) {
  const size_t size = MakeEvent("a", 1, 300).ByteSizeLong();
  FakeEpisodeSink sink;
  auto writer = MakeWriter(&sink, 4, size);
  ASSERT_NE(writer, nullptr);

  const EpisodeEvent first = MakeEvent("a", 1);
  const EpisodeEvent second = MakeEvent("a", 2);
  const EpisodeEvent duplicate = MakeEvent("a", 2);
  const EpisodeEvent other_source = MakeEvent("b", 1);
  const EpisodeEvent too_large = MakeEvent("a", 90, 301);
  const EpisodeEvent overflow = MakeEvent("a", 91);
  const EpisodeEvent overflow_again = MakeEvent("a", 92);
  EpisodeEvent invalid = MakeEvent("a", 3);
  invalid.clear_header();
  const EpisodeEvent empty;
  const EpisodeEvent reserved =
      MakeEvent(std::string(kEpisodeWriterSourceId), 1);
  EpisodeEvent forged = MakeEvent("c", 1);
  forged.mutable_episode_loss()->set_dropped_count(1);
  const EpisodeEvent unknown = Parse(TestdataBytes(std::string(kV2Name)));

  EXPECT_APPEND_WITHOUT_ALLOCATION(writer, first, Status::kOk);
  EXPECT_APPEND_WITHOUT_ALLOCATION(writer, other_source, Status::kOk);
  EXPECT_APPEND_WITHOUT_ALLOCATION(writer, second, Status::kOk);
  EXPECT_APPEND_WITHOUT_ALLOCATION(writer, duplicate, Status::kOutOfOrder);
  EXPECT_APPEND_WITHOUT_ALLOCATION(writer, invalid, Status::kInvalidEvent);
  EXPECT_APPEND_WITHOUT_ALLOCATION(writer, empty, Status::kInvalidEvent);
  EXPECT_APPEND_WITHOUT_ALLOCATION(writer, reserved, Status::kReservedEvent);
  EXPECT_APPEND_WITHOUT_ALLOCATION(writer, forged, Status::kReservedEvent);
  EXPECT_APPEND_WITHOUT_ALLOCATION(writer, overflow, Status::kBufferFull);
  EXPECT_APPEND_WITHOUT_ALLOCATION(writer, overflow_again, Status::kBufferFull);
  ASSERT_EQ(writer->Flush(), Status::kOk);
  EXPECT_APPEND_WITHOUT_ALLOCATION(writer, too_large, Status::kEventTooLarge);
  EXPECT_APPEND_WITHOUT_ALLOCATION(writer, unknown, Status::kOk);
}

TEST(EpisodeWriterTest, W14LossMaterializationAndReasonSwitchDoNotAllocate) {
  const size_t size = MakeEvent("a", 1, 300).ByteSizeLong();
  FakeEpisodeSink sink;
  auto writer = MakeWriter(&sink, 3, size);
  ASSERT_NE(writer, nullptr);
  const EpisodeEvent first = MakeEvent("a", 1);
  const EpisodeEvent too_large = MakeEvent("a", 2, 301);
  const EpisodeEvent overflow = MakeEvent("a", 3);
  EXPECT_APPEND_WITHOUT_ALLOCATION(writer, first, Status::kOk);
  EXPECT_APPEND_WITHOUT_ALLOCATION(writer, too_large, Status::kEventTooLarge);
  // The reason switch materializes the TOO_LARGE loss record.
  EXPECT_APPEND_WITHOUT_ALLOCATION(writer, overflow, Status::kBufferFull);
  EXPECT_EQ(writer->Stats().loss_records, 1u);

  ASSERT_EQ(writer->Flush(), Status::kOk);
  EXPECT_EQ(writer->Stats().loss_records, 2u);

  // Accept with a pending loss materializes it first.
  const EpisodeEvent drop = MakeEvent("a", 5, 301);
  const EpisodeEvent accepted = MakeEvent("a", 6);
  EXPECT_APPEND_WITHOUT_ALLOCATION(writer, drop, Status::kEventTooLarge);
  EXPECT_APPEND_WITHOUT_ALLOCATION(writer, accepted, Status::kOk);
  EXPECT_EQ(writer->Stats().loss_records, 3u);
}

TEST(EpisodeWriterTest, W14NewSourceInsertionAndStopStatesDoNotAllocate) {
  FakeEpisodeSink sink;
  auto writer = MakeWriter(&sink, 128, kSmallMaxEventBytes);
  ASSERT_NE(writer, nullptr);
  std::vector<EpisodeEvent> events;
  for (size_t i = 0; i < kMaxEpisodeSources + 1; ++i) {
    events.push_back(
        MakeEvent("a_long_source_identifier_" + std::to_string(i), 1));
  }
  for (size_t i = 0; i < kMaxEpisodeSources; ++i) {
    EXPECT_APPEND_WITHOUT_ALLOCATION(writer, events[i], Status::kOk);
  }
  EXPECT_APPEND_WITHOUT_ALLOCATION(writer, events[kMaxEpisodeSources],
                                   Status::kTooManySources);

  const EpisodeEvent more = MakeEvent("a_long_source_identifier_0", 2);
  EXPECT_APPEND_WITHOUT_ALLOCATION(writer, more, Status::kOk);
  ASSERT_EQ(writer->Close(), Status::kOk);
  const EpisodeEvent after_close = MakeEvent("a_long_source_identifier_0", 3);
  EXPECT_APPEND_WITHOUT_ALLOCATION(writer, after_close, Status::kClosed);

  FakeEpisodeSink cancelled_sink;
  auto cancelled = MakeWriter(&cancelled_sink, 4, kSmallMaxEventBytes);
  EXPECT_EQ(cancelled->Cancel(), 0u);
  EXPECT_APPEND_WITHOUT_ALLOCATION(cancelled, after_close, Status::kCancelled);

  FakeEpisodeSink failing_sink;
  failing_sink.fail_write_call = 0;
  auto failed = MakeWriter(&failing_sink, 4, kSmallMaxEventBytes);
  ASSERT_EQ(failed->Flush(), Status::kWriteFailed);
  EXPECT_APPEND_WITHOUT_ALLOCATION(failed, after_close, Status::kWriteFailed);
}

TEST(EpisodeWriterTest, W14TheGuardCountsAllocations) {
  uint64_t allocations = 0;
  {
    AllocationGuard guard;
    int* probe = new int(1);
    *static_cast<volatile int*>(probe) = 2;
    delete probe;
    allocations = guard.count();
  }
  EXPECT_GE(allocations, 1u);
}

}  // namespace
}  // namespace intrinsic::episode
