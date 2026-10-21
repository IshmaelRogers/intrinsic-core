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

// RT1 to RT14: the episode reader and corruption detection (#129, lock id
// episode-reader-v1).

#include "intrinsic_data/episode/episode_reader.h"

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <fstream>
#include <limits>
#include <memory>
#include <optional>
#include <ostream>
#include <sstream>
#include <string>
#include <string_view>
#include <vector>

#include "absl/crc/crc32c.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/types/span.h"
#include "gtest/gtest.h"
#include "intrinsic/episode/proto/episode_event.pb.h"
#include "intrinsic_data/episode/episode_event_policy.h"
#include "intrinsic_data/episode/episode_writer.h"

namespace intrinsic::episode {

void PrintTo(EpisodeReadDefect defect, std::ostream* os) {
  *os << "EpisodeReadDefect(" << static_cast<int>(defect) << ")";
}
void PrintTo(EpisodeReaderStatus status, std::ostream* os) {
  *os << "EpisodeReaderStatus(" << static_cast<int>(status) << ")";
}
void PrintTo(PayloadKind kind, std::ostream* os) {
  *os << "PayloadKind(" << static_cast<int>(kind) << ")";
}

namespace {

using intrinsic_proto::episode::EpisodeEvent;
using intrinsic_proto::episode::EpisodeLoss;
using Defect = EpisodeReadDefect;

constexpr uint64_t kMax = std::numeric_limits<uint64_t>::max();
constexpr size_t kFileHeader = 12;
constexpr size_t kRecordHeader = 16;

constexpr const char* kPayloadNames[] = {
    "safety_decision", "vehicle_trajectory", "inference_envelope",
    "inference_result", "metadata"};
constexpr PayloadKind kPayloadKinds[] = {
    PayloadKind::kSafetyDecision, PayloadKind::kVehicleTrajectory,
    PayloadKind::kInferenceEnvelope, PayloadKind::kInferenceResult,
    PayloadKind::kEpisodeMetadata};
constexpr char kV2Name[] = "episode_event_v2_additive_unknown_payload.binpb";

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

std::string Fixture(const std::string& name) {
  return TestdataBytes("episode_reader_v1_" + name + ".iepisod");
}

EpisodeEvent Parse(const std::string& bytes) {
  EpisodeEvent message;
  EXPECT_TRUE(message.ParseFromString(bytes));
  return message;
}

EpisodeEvent Golden(const std::string& payload) {
  return Parse(GoldenBytes(payload));
}

// A valid safety_decision event with a chosen source and sequence. Padding
// the provenance model_id (unvalidated) controls the serialized size.
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

EpisodeEvent MakeTrajectory(const std::string& source, uint64_t sequence) {
  EpisodeEvent event = Golden("vehicle_trajectory");
  event.mutable_header()->set_source_id(source);
  event.mutable_header()->set_sequence(sequence);
  return event;
}

absl::Span<const uint8_t> Span(const std::string& data) {
  return absl::MakeConstSpan(reinterpret_cast<const uint8_t*>(data.data()),
                             data.size());
}

std::string Slice(const std::string& data, size_t length) {
  return data.substr(0, length);
}

void PutLe(std::string& out, uint64_t value, int bytes) {
  for (int i = 0; i < bytes; ++i) {
    out.push_back(static_cast<char>((value >> (8 * i)) & 0xff));
  }
}

uint64_t GetLe(const std::string& data, size_t offset, int bytes) {
  uint64_t value = 0;
  for (int i = 0; i < bytes; ++i) {
    value |= static_cast<uint64_t>(static_cast<uint8_t>(data[offset + i]))
             << (8 * i);
  }
  return value;
}

void SetLe(std::string& data, size_t offset, uint64_t value, int bytes) {
  std::string tmp;
  PutLe(tmp, value, bytes);
  data.replace(offset, bytes, tmp);
}

uint32_t Crc(const std::string& bytes) {
  return static_cast<uint32_t>(absl::ComputeCrc32c(bytes));
}

std::string Header(uint32_t version = 1) {
  std::string header = "IEPISOD1";
  PutLe(header, version, 4);
  return header;
}

std::string Record(uint64_t index, const std::string& event_bytes) {
  std::string record;
  PutLe(record, event_bytes.size(), 4);
  PutLe(record, Crc(event_bytes), 4);
  PutLe(record, index, 8);
  record += event_bytes;
  return record;
}

std::string Record(uint64_t index, const EpisodeEvent& event) {
  return Record(index, event.SerializeAsString());
}

struct RawRecord {
  size_t offset = 0;
  uint32_t length = 0;
  uint64_t index = 0;
  std::string event;
  size_t end() const { return offset + kRecordHeader + length; }
};

// A test-local framing walk over a well-formed file. Independent of the
// reader.
std::vector<RawRecord> Split(const std::string& data) {
  std::vector<RawRecord> out;
  size_t offset = kFileHeader;
  while (offset + kRecordHeader <= data.size()) {
    RawRecord raw;
    raw.offset = offset;
    raw.length = static_cast<uint32_t>(GetLe(data, offset, 4));
    raw.index = GetLe(data, offset + 8, 8);
    raw.event = data.substr(offset + kRecordHeader, raw.length);
    offset = raw.end();
    out.push_back(std::move(raw));
  }
  EXPECT_EQ(offset, data.size());
  return out;
}

EpisodeReadResult Read(const std::string& data,
                       const EpisodeReaderConfig& config = {},
                       const EpisodeReadFilter& filter = {}) {
  return ReadEpisode(Span(data), config, filter);
}

std::vector<uint64_t> Indices(const EpisodeReadResult& result) {
  std::vector<uint64_t> out;
  for (const EpisodeRecordEntry& entry : result.records) {
    out.push_back(entry.writer_index);
  }
  return out;
}

// Checks the status, the defect, and the last-good fields of a result.
void ExpectPrefix(const EpisodeReadResult& result, Defect defect,
                  uint64_t good_records, uint64_t good_end_offset,
                  std::optional<uint64_t> last_index) {
  EXPECT_EQ(result.status, EpisodeReaderStatus::kOk);
  EXPECT_EQ(result.defect, defect);
  EXPECT_EQ(result.good_records, good_records);
  EXPECT_EQ(result.good_end_offset, good_end_offset);
  EXPECT_EQ(result.has_last_good_index, last_index.has_value());
  EXPECT_EQ(result.last_good_writer_index, last_index.value_or(0));
  EXPECT_EQ(result.stats.records, good_records);
  if (defect == Defect::kNone) {
    EXPECT_EQ(result.defect_offset, 0u);
  } else {
    EXPECT_EQ(result.defect_offset, good_end_offset);
  }
  EXPECT_EQ(result.next_expected_writer_index,
            last_index.has_value() ? *last_index + 1 : 0u);
}

void ExpectFixture(const std::string& name, Defect defect,
                   uint64_t good_records, uint64_t good_end_offset,
                   std::optional<uint64_t> last_index) {
  SCOPED_TRACE(name);
  const std::string data = Fixture(name);
  ASSERT_FALSE(data.empty());
  ExpectPrefix(Read(data), defect, good_records, good_end_offset, last_index);
}

struct LossSpec {
  uint64_t index = 5;
  uint64_t sequence = 1;
  uint64_t first = 3;
  uint64_t last = 4;
  uint64_t count = 2;
  int reason = EpisodeLoss::LOSS_REASON_OVERFLOW;
  int64_t source_seconds = 1000;
  int32_t source_nanos = 5;
  int64_t receive_seconds = 1000;
  int32_t receive_nanos = 5;
  std::string source = "episode_writer";
};

EpisodeEvent LossEvent(const LossSpec& spec) {
  EpisodeEvent event;
  auto* header = event.mutable_header();
  header->set_source_id(spec.source);
  header->set_frame_id("episode_writer");
  header->set_sequence(spec.sequence);
  header->mutable_source_time()->set_seconds(spec.source_seconds);
  header->mutable_source_time()->set_nanos(spec.source_nanos);
  header->mutable_receive_time()->set_seconds(spec.receive_seconds);
  header->mutable_receive_time()->set_nanos(spec.receive_nanos);
  event.set_schema_version(1);
  auto* loss = event.mutable_episode_loss();
  loss->set_dropped_count(spec.count);
  loss->set_first_dropped_writer_index(spec.first);
  loss->set_last_dropped_writer_index(spec.last);
  loss->set_reason(static_cast<EpisodeLoss::LossReason>(spec.reason));
  return event;
}

std::string LossRecord(const LossSpec& spec) {
  return Record(spec.index, LossEvent(spec));
}

class NominalFile {
 public:
  NominalFile() : data_(TestdataBytes("episode_writer_v1_nominal.iepisod")) {
    records_ = Split(data_);
  }
  const std::string& data() const { return data_; }
  const std::vector<RawRecord>& records() const { return records_; }
  // The file up to and including `count` records.
  std::string Prefix(size_t count) const {
    return data_.substr(0,
                        count == 0 ? kFileHeader : records_[count - 1].end());
  }

 private:
  std::string data_;
  std::vector<RawRecord> records_;
};

class MemorySink : public EpisodeSink {
 public:
  absl::Status Write(absl::Span<const uint8_t> bytes) override {
    output.append(reinterpret_cast<const char*>(bytes.data()), bytes.size());
    return absl::OkStatus();
  }
  absl::Status Sync() override { return absl::OkStatus(); }
  std::string output;
};

std::unique_ptr<EpisodeWriter> MakeWriter(EpisodeSink* sink, size_t capacity,
                                          size_t max_event_bytes) {
  EpisodeWriterConfig config;
  config.capacity = capacity;
  config.max_event_bytes = max_event_bytes;
  EpisodeWriterStatus status;
  auto writer = EpisodeWriter::Create(config, sink, &status);
  EXPECT_EQ(status, EpisodeWriterStatus::kOk);
  return writer;
}

void ExpectSameResult(const EpisodeReadResult& a, const EpisodeReadResult& b) {
  EXPECT_EQ(a.status, b.status);
  EXPECT_EQ(a.defect, b.defect);
  EXPECT_EQ(a.event_error, b.event_error);
  EXPECT_EQ(a.defect_offset, b.defect_offset);
  EXPECT_EQ(a.good_records, b.good_records);
  EXPECT_EQ(a.good_end_offset, b.good_end_offset);
  EXPECT_EQ(a.has_last_good_index, b.has_last_good_index);
  EXPECT_EQ(a.last_good_writer_index, b.last_good_writer_index);
  EXPECT_EQ(a.next_expected_writer_index, b.next_expected_writer_index);
  ASSERT_EQ(a.records.size(), b.records.size());
  for (size_t i = 0; i < a.records.size(); ++i) {
    const EpisodeRecordEntry& x = a.records[i];
    const EpisodeRecordEntry& y = b.records[i];
    EXPECT_EQ(x.offset, y.offset);
    EXPECT_EQ(x.writer_index, y.writer_index);
    EXPECT_EQ(x.crc32c, y.crc32c);
    EXPECT_EQ(x.event_bytes.data(), y.event_bytes.data());
    EXPECT_EQ(x.event_bytes.size(), y.event_bytes.size());
    EXPECT_EQ(x.payload_kind, y.payload_kind);
    EXPECT_EQ(x.skip, y.skip);
    EXPECT_EQ(x.source_id, y.source_id);
  }
  ASSERT_EQ(a.losses.size(), b.losses.size());
  for (size_t i = 0; i < a.losses.size(); ++i) {
    const EpisodeLossEntry& x = a.losses[i];
    const EpisodeLossEntry& y = b.losses[i];
    EXPECT_EQ(x.writer_index, y.writer_index);
    EXPECT_EQ(x.first_dropped_writer_index, y.first_dropped_writer_index);
    EXPECT_EQ(x.last_dropped_writer_index, y.last_dropped_writer_index);
    EXPECT_EQ(x.dropped_count, y.dropped_count);
    EXPECT_EQ(x.reason, y.reason);
    EXPECT_EQ(x.sequence, y.sequence);
  }
  EXPECT_EQ(a.stats.records, b.stats.records);
  EXPECT_EQ(a.stats.loss_records, b.stats.loss_records);
  EXPECT_EQ(a.stats.unknown_skipped, b.stats.unknown_skipped);
  EXPECT_EQ(a.stats.dropped_overflow, b.stats.dropped_overflow);
  EXPECT_EQ(a.stats.dropped_too_large, b.stats.dropped_too_large);
  EXPECT_EQ(a.stats.dropped_other_reason, b.stats.dropped_other_reason);
  EXPECT_EQ(a.stats.producer_sources, b.stats.producer_sources);
}

std::string EventString(const EpisodeRecordEntry& entry) {
  return std::string(entry.event_bytes.begin(), entry.event_bytes.end());
}

// The writer constants and the reader's own repeated copies must agree.
static_assert(kDefaultEpisodeReaderMaxEventBytes == kMaxEpisodeMaxEventBytes);
static_assert(kFileHeader == kEpisodeFileHeaderBytes);
static_assert(kRecordHeader == kEpisodeRecordHeaderBytes);

// RT1: the nominal writer golden.
TEST(EpisodeReaderTest, RT1NominalGolden) {
  const NominalFile nominal;
  ASSERT_EQ(nominal.data().size(), 4069u);
  const EpisodeReadResult result = Read(nominal.data());
  ExpectPrefix(result, Defect::kNone, 5, 4069, 4);
  EXPECT_EQ(result.next_expected_writer_index, 5u);
  EXPECT_EQ(Indices(result), (std::vector<uint64_t>{0, 1, 2, 3, 4}));
  EXPECT_TRUE(result.losses.empty());
  EXPECT_EQ(result.stats.producer_sources, 1u);
  EXPECT_EQ(result.stats.loss_records, 0u);
  ASSERT_EQ(result.records.size(), 5u);
  for (size_t i = 0; i < 5; ++i) {
    SCOPED_TRACE(kPayloadNames[i]);
    const EpisodeRecordEntry& entry = result.records[i];
    EXPECT_EQ(EventString(entry), GoldenBytes(kPayloadNames[i]));
    EXPECT_EQ(entry.offset, nominal.records()[i].offset);
    EXPECT_EQ(entry.crc32c, Crc(GoldenBytes(kPayloadNames[i])));
    EXPECT_EQ(entry.payload_kind, kPayloadKinds[i]);
    EXPECT_FALSE(entry.skip);
    EXPECT_EQ(entry.source_id, Golden(kPayloadNames[0]).header().source_id());
    // The span points into the caller's buffer.
    EXPECT_EQ(entry.event_bytes.data(),
              reinterpret_cast<const uint8_t*>(nominal.data().data()) +
                  entry.offset + kRecordHeader);
  }
}

// RT2: the overflow writer golden.
TEST(EpisodeReaderTest, RT2OverflowGolden) {
  const std::string data = TestdataBytes("episode_writer_v1_overflow.iepisod");
  ASSERT_EQ(data.size(), 1607u);
  const EpisodeReadResult result = Read(data);
  ExpectPrefix(result, Defect::kNone, 4, 1607, 5);
  EXPECT_EQ(result.next_expected_writer_index, 6u);
  EXPECT_EQ(Indices(result), (std::vector<uint64_t>{0, 1, 2, 5}));
  ASSERT_EQ(result.losses.size(), 1u);
  const EpisodeLossEntry& loss = result.losses[0];
  EXPECT_EQ(loss.writer_index, 5u);
  EXPECT_EQ(loss.first_dropped_writer_index, 3u);
  EXPECT_EQ(loss.last_dropped_writer_index, 4u);
  EXPECT_EQ(loss.dropped_count, 2u);
  EXPECT_EQ(loss.reason, static_cast<int>(EpisodeLoss::LOSS_REASON_OVERFLOW));
  EXPECT_EQ(loss.sequence, 1u);
  EXPECT_EQ(result.stats.records, 4u);
  EXPECT_EQ(result.stats.loss_records, 1u);
  EXPECT_EQ(result.stats.dropped_overflow, 2u);
  EXPECT_EQ(result.stats.dropped_too_large, 0u);
  EXPECT_EQ(result.stats.dropped_other_reason, 0u);
  EXPECT_EQ(result.stats.producer_sources, 1u);
  EXPECT_EQ(result.records.back().payload_kind, PayloadKind::kEpisodeLoss);
  EXPECT_EQ(result.records.back().source_id, "episode_writer");
}

// RT3: empty episode and short headers.
TEST(EpisodeReaderTest, RT3EmptyAndShortHeaders) {
  ExpectFixture("header_only", Defect::kNone, 0, 12, std::nullopt);
  const EpisodeReadResult empty = Read(Fixture("header_only"));
  EXPECT_TRUE(empty.records.empty());
  EXPECT_TRUE(empty.losses.empty());
  EXPECT_EQ(empty.next_expected_writer_index, 0u);

  ExpectPrefix(Read(""), Defect::kTruncatedHeader, 0, 0, std::nullopt);
  ExpectPrefix(Read(Slice(Header(), 11)), Defect::kTruncatedHeader, 0, 0,
               std::nullopt);
  for (size_t length = 0; length < kFileHeader; ++length) {
    ExpectPrefix(Read(Slice(Header(), length)), Defect::kTruncatedHeader, 0, 0,
                 std::nullopt);
  }
}

// RT4: header damage.
TEST(EpisodeReaderTest, RT4HeaderDefects) {
  ExpectFixture("bad_magic", Defect::kBadMagic, 0, 0, std::nullopt);
  ExpectFixture("version_0", Defect::kUnsupportedVersion, 0, 0, std::nullopt);
  ExpectFixture("version_2", Defect::kUnsupportedVersion, 0, 0, std::nullopt);
  for (const char* name : {"bad_magic", "version_0", "version_2"}) {
    const EpisodeReadResult result = Read(Fixture(name));
    EXPECT_TRUE(result.records.empty()) << name;
    EXPECT_TRUE(result.losses.empty()) << name;
  }
  // Every single-byte change of the magic is a bad magic.
  const NominalFile nominal;
  for (size_t i = 0; i < 8; ++i) {
    std::string data = nominal.data();
    data[i] ^= 0x20;
    ExpectPrefix(Read(data), Defect::kBadMagic, 0, 0, std::nullopt);
  }
  std::string version_big = nominal.data();
  SetLe(version_big, 8, 0x01000000, 4);
  ExpectPrefix(Read(version_big), Defect::kUnsupportedVersion, 0, 0,
               std::nullopt);
}

// RT5: the nominal golden cut at every length.
TEST(EpisodeReaderTest, RT5TruncationSweep) {
  const NominalFile nominal;
  const std::string& data = nominal.data();
  for (size_t length = 0; length < data.size(); ++length) {
    SCOPED_TRACE(length);
    const EpisodeReadResult result = Read(Slice(data, length));
    if (length < kFileHeader) {
      ExpectPrefix(result, Defect::kTruncatedHeader, 0, 0, std::nullopt);
      continue;
    }
    uint64_t good = 0;
    size_t good_end = kFileHeader;
    for (const RawRecord& raw : nominal.records()) {
      if (raw.end() <= length) {
        ++good;
        good_end = raw.end();
      }
    }
    const std::optional<uint64_t> last =
        good == 0 ? std::nullopt : std::optional<uint64_t>(good - 1);
    // A cut exactly on a record boundary is a structurally intact prefix.
    const Defect expected =
        good_end == length ? Defect::kNone : Defect::kTruncatedRecord;
    ExpectPrefix(result, expected, good, good_end, last);
    EXPECT_EQ(result.records.size(), good);
  }
}

// RT6: framing and CRC damage.
TEST(EpisodeReaderTest, RT6FramingAndCrcDamage) {
  const NominalFile nominal;
  const RawRecord& r2 = nominal.records()[2];
  const RawRecord& r4 = nominal.records()[4];
  const size_t end2 = nominal.records()[1].end();

  ExpectFixture("crc_flip", Defect::kCrcMismatch, 2, end2, 1);

  std::string event_flip = nominal.data();
  event_flip[r2.offset + kRecordHeader + 3] ^= 0x01;
  ExpectPrefix(Read(event_flip), Defect::kCrcMismatch, 2, end2, 1);

  std::string zero = nominal.data();
  SetLe(zero, r2.offset, 0, 4);
  ExpectPrefix(Read(zero), Defect::kLengthOutOfRange, 2, end2, 1);

  std::string huge = nominal.data();
  SetLe(huge, r2.offset, kDefaultEpisodeReaderMaxEventBytes + 1, 4);
  ExpectPrefix(Read(huge), Defect::kLengthOutOfRange, 2, end2, 1);
  std::string max32 = nominal.data();
  SetLe(max32, r2.offset, 0xffffffffu, 4);
  ExpectPrefix(Read(max32), Defect::kLengthOutOfRange, 2, end2, 1);

  // A limit below the event size rejects the record by length.
  EpisodeReaderConfig tiny;
  tiny.max_event_bytes = 1;
  ASSERT_GT(nominal.records()[0].length, MaxEpisodeLossRecordBytes());
  ExpectPrefix(Read(nominal.data(), tiny), Defect::kLengthOutOfRange, 0, 12,
               std::nullopt);
  // An event exactly at the limit passes, one byte above does not.
  EpisodeReaderConfig exact;
  exact.max_event_bytes = nominal.records()[0].length;
  ExpectPrefix(Read(nominal.Prefix(1), exact), Defect::kNone, 1,
               nominal.records()[0].end(), 0);
  exact.max_event_bytes -= 1;
  ExpectPrefix(Read(nominal.Prefix(1), exact), Defect::kLengthOutOfRange, 0, 12,
               std::nullopt);

  // N + 1 and N - 1 misframe the record, and the CRC no longer matches.
  std::string plus = nominal.data();
  SetLe(plus, r2.offset, r2.length + 1, 4);
  ExpectPrefix(Read(plus), Defect::kCrcMismatch, 2, end2, 1);
  std::string minus = nominal.data();
  SetLe(minus, r2.offset, r2.length - 1, 4);
  ExpectPrefix(Read(minus), Defect::kCrcMismatch, 2, end2, 1);

  // N + 1 on the last record runs past the end of the file.
  std::string plus_last = nominal.data();
  SetLe(plus_last, r4.offset, r4.length + 1, 4);
  ExpectPrefix(Read(plus_last), Defect::kTruncatedRecord, 4, r4.offset, 3);
  // N - 1 on the last record leaves one trailing byte, still a CRC mismatch.
  std::string minus_last = nominal.data();
  SetLe(minus_last, r4.offset, r4.length - 1, 4);
  ExpectPrefix(Read(minus_last), Defect::kCrcMismatch, 4, r4.offset, 3);

  // A file with a trailing partial record header.
  std::string tail = nominal.data() + std::string(7, '\0');
  ExpectPrefix(Read(tail), Defect::kTruncatedRecord, 5, 4069, 4);
}

// RT7: index and loss-record damage.
TEST(EpisodeReaderTest, RT7IndexAndLossDamage) {
  const NominalFile nominal;
  const size_t end2 = nominal.records()[1].end();
  const size_t off2 = nominal.records()[2].offset;

  ExpectFixture("index_gap_uncovered", Defect::kIndexGap, 2, end2, 1);

  std::string duplicate = nominal.data();
  SetLe(duplicate, off2 + 8, 1, 8);
  ExpectPrefix(Read(duplicate), Defect::kIndexNotIncreasing, 2, end2, 1);
  std::string decreasing = nominal.data();
  SetLe(decreasing, off2 + 8, 0, 8);
  ExpectPrefix(Read(decreasing), Defect::kIndexNotIncreasing, 2, end2, 1);
  std::string first_wrong = nominal.data();
  SetLe(first_wrong, nominal.records()[0].offset + 8, 1, 8);
  ExpectPrefix(Read(first_wrong), Defect::kIndexGap, 0, 12, std::nullopt);
  std::string uncovered_big = nominal.data();
  SetLe(uncovered_big, off2 + 8, kMax, 8);
  ExpectPrefix(Read(uncovered_big), Defect::kIndexGap, 2, end2, 1);

  ExpectFixture("bad_loss_count", Defect::kLossRecordInvalid, 3,
                nominal.records()[2].end(), 2);

  // The control case: a valid loss record after three records.
  const std::string prefix = nominal.Prefix(3);
  const size_t prefix_end = prefix.size();
  {
    const EpisodeReadResult result = Read(prefix + LossRecord(LossSpec{}));
    ExpectPrefix(
        result, Defect::kNone, 4,
        prefix_end + kRecordHeader + LossEvent(LossSpec{}).ByteSizeLong(), 5);
    EXPECT_EQ(result.next_expected_writer_index, 6u);
    ASSERT_EQ(result.losses.size(), 1u);
  }

  struct Case {
    const char* name;
    void (*mutate)(LossSpec&);
  };
  const Case cases[] = {
      {"first below expected",
       [](LossSpec& s) {
         s.first = 2;
         s.count = 3;
       }},
      {"first above expected",
       [](LossSpec& s) {
         s.first = 4;
         s.count = 1;
       }},
      {"last below first",
       [](LossSpec& s) {
         s.last = 2;
         s.index = 3;
         s.count = 0;
       }},
      {"last is uint64 max",
       [](LossSpec& s) {
         s.last = kMax;
         s.index = 0;
         s.count = kMax - 2;
       }},
      {"index is last", [](LossSpec& s) { s.index = 4; }},
      {"index above last + 1", [](LossSpec& s) { s.index = 6; }},
      {"count too small", [](LossSpec& s) { s.count = 1; }},
      {"count too large", [](LossSpec& s) { s.count = 3; }},
      {"count zero", [](LossSpec& s) { s.count = 0; }},
      {"reason unspecified",
       [](LossSpec& s) { s.reason = EpisodeLoss::LOSS_REASON_UNSPECIFIED; }},
      {"sequence zero", [](LossSpec& s) { s.sequence = 0; }},
      {"sequence two", [](LossSpec& s) { s.sequence = 2; }},
      {"times differ in seconds", [](LossSpec& s) { s.receive_seconds += 1; }},
      {"times differ in nanos", [](LossSpec& s) { s.receive_nanos += 1; }},
      {"next index would wrap",
       [](LossSpec& s) {
         s.last = kMax - 1;
         s.index = kMax;
         s.count = kMax - 3;
       }},
  };
  for (const Case& c : cases) {
    SCOPED_TRACE(c.name);
    LossSpec spec;
    c.mutate(spec);
    const EpisodeReadResult result = Read(prefix + LossRecord(spec));
    ExpectPrefix(result, Defect::kLossRecordInvalid, 3, prefix_end, 2);
    EXPECT_EQ(result.defect_offset, prefix_end);
  }

  // Reasons above 2 are accepted and counted separately.
  {
    LossSpec spec;
    spec.reason = 3;
    const EpisodeReadResult result = Read(prefix + LossRecord(spec));
    EXPECT_EQ(result.defect, Defect::kNone);
    EXPECT_EQ(result.stats.dropped_other_reason, 2u);
    ASSERT_EQ(result.losses.size(), 1u);
    EXPECT_EQ(result.losses[0].reason, 3);
  }

  // The second loss record needs sequence 2.
  {
    LossSpec second;
    second.index = 7;
    second.first = 6;
    second.last = 6;
    second.count = 1;
    const std::string first_loss = prefix + LossRecord(LossSpec{});
    second.sequence = 1;
    ExpectPrefix(Read(first_loss + LossRecord(second)),
                 Defect::kLossRecordInvalid, 4, first_loss.size(), 5);
    second.sequence = 2;
    const EpisodeReadResult ok = Read(first_loss + LossRecord(second));
    EXPECT_EQ(ok.defect, Defect::kNone);
    EXPECT_EQ(ok.good_records, 5u);
    EXPECT_EQ(ok.next_expected_writer_index, 8u);
  }

  // A loss range must start at the next expected index even after a loss.
  {
    LossSpec second;
    second.index = 8;
    second.first = 7;
    second.last = 7;
    second.count = 1;
    second.sequence = 2;
    const std::string first_loss = prefix + LossRecord(LossSpec{});
    ExpectPrefix(Read(first_loss + LossRecord(second)),
                 Defect::kLossRecordInvalid, 4, first_loss.size(), 5);
  }

  // An ordinary record after a loss that ends one below the largest index
  // would need an index that wraps.
  {
    LossSpec spec;
    spec.last = kMax - 2;
    spec.index = kMax - 1;
    spec.count = kMax - 4;
    const std::string with_loss = prefix + LossRecord(spec);
    const EpisodeReadResult after_loss = Read(with_loss);
    EXPECT_EQ(after_loss.defect, Defect::kNone);
    EXPECT_EQ(after_loss.next_expected_writer_index, kMax);
    const RawRecord& r3 = nominal.records()[3];
    const std::string next = Record(kMax, r3.event);
    ExpectPrefix(Read(with_loss + next), Defect::kIndexNotIncreasing, 4,
                 with_loss.size(), kMax - 1);
  }

  ExpectFixture("index_bit_flip_loss", Defect::kLossRecordInvalid, 3,
                nominal.records()[2].end(), 2);
}

// RT8: producer ordering, source table, reserved events.
TEST(EpisodeReaderTest, RT8OrderingSourcesAndReserved) {
  const NominalFile nominal;
  ExpectFixture("sequence_regression", Defect::kOrderViolation, 2,
                nominal.records()[1].end(), 1);
  ExpectFixture("forged_loss", Defect::kReservedEvent, 3, 1517, 2);

  // An equal sequence and a lower sequence both fail. The first sequence may
  // be 0.
  {
    const std::string ok =
        Header() + Record(0, MakeEvent("a", 0)) + Record(1, MakeEvent("a", 1));
    EXPECT_EQ(Read(ok).defect, Defect::kNone);
    const std::string equal =
        Header() + Record(0, MakeEvent("a", 5)) + Record(1, MakeEvent("a", 5));
    ExpectPrefix(Read(equal), Defect::kOrderViolation, 1,
                 kFileHeader + Record(0, MakeEvent("a", 5)).size(), 0);
    const std::string lower =
        Header() + Record(0, MakeEvent("a", 5)) + Record(1, MakeEvent("a", 4));
    ExpectPrefix(Read(lower), Defect::kOrderViolation, 1,
                 kFileHeader + Record(0, MakeEvent("a", 5)).size(), 0);
    // Sources are independent, and the table survives other sources.
    const std::string interleaved =
        Header() + Record(0, MakeEvent("a", 5)) + Record(1, MakeEvent("b", 1)) +
        Record(2, MakeEvent("a", 6)) + Record(3, MakeEvent("b", 2));
    const EpisodeReadResult result = Read(interleaved);
    EXPECT_EQ(result.defect, Defect::kNone);
    EXPECT_EQ(result.stats.producer_sources, 2u);
    const std::string interleaved_bad =
        interleaved + Record(4, MakeEvent("a", 6));
    EXPECT_EQ(Read(interleaved_bad).defect, Defect::kOrderViolation);
  }

  // 64 sources are accepted, the 65th is not.
  {
    std::string data = Header();
    for (size_t i = 0; i < kMaxEpisodeSources; ++i) {
      data += Record(i, MakeEvent("src_" + std::to_string(i), 1));
    }
    const EpisodeReadResult full = Read(data);
    EXPECT_EQ(full.defect, Defect::kNone);
    EXPECT_EQ(full.stats.producer_sources, kMaxEpisodeSources);
    const size_t full_size = data.size();
    data += Record(kMaxEpisodeSources, MakeEvent("src_overflow", 1));
    ExpectPrefix(Read(data), Defect::kTooManySources, kMaxEpisodeSources,
                 full_size, kMaxEpisodeSources - 1);
    // A known source is still fine at the limit.
    std::string known = std::string(data, 0, full_size) +
                        Record(kMaxEpisodeSources, MakeEvent("src_0", 2));
    EXPECT_EQ(Read(known).defect, Defect::kNone);
  }

  // Reserved events: case 15 from a producer, and the loss source with any
  // other payload, known or unknown.
  const std::string prefix = nominal.Prefix(3);
  {
    LossSpec forged;
    forged.source = "producer";
    ExpectPrefix(Read(prefix + LossRecord(forged)), Defect::kReservedEvent, 3,
                 prefix.size(), 2);
    const EpisodeReadResult result = Read(prefix + LossRecord(forged));
    EXPECT_TRUE(result.losses.empty());
  }
  {
    const std::string known =
        prefix + Record(3, MakeEvent("episode_writer", 1));
    ExpectPrefix(Read(known), Defect::kReservedEvent, 3, prefix.size(), 2);
    EpisodeEvent unknown = Parse(TestdataBytes(kV2Name));
    unknown.mutable_header()->set_source_id("episode_writer");
    const std::string unknown_data = prefix + Record(3, unknown);
    ExpectPrefix(Read(unknown_data), Defect::kReservedEvent, 3, prefix.size(),
                 2);
  }
}

// RT9: unknown payloads are valid and skippable.
TEST(EpisodeReaderTest, RT9UnknownPayload) {
  const std::string v2 = TestdataBytes(kV2Name);
  ExpectFixture("unknown_payload_record", Defect::kNone, 1,
                kFileHeader + kRecordHeader + v2.size(), 0);
  const EpisodeReadResult result = Read(Fixture("unknown_payload_record"));
  ASSERT_EQ(result.records.size(), 1u);
  const EpisodeRecordEntry& entry = result.records[0];
  EXPECT_TRUE(entry.skip);
  EXPECT_EQ(entry.payload_kind, PayloadKind::kUnknown);
  EXPECT_EQ(EventString(entry), v2);
  EXPECT_EQ(entry.writer_index, 0u);
  EXPECT_EQ(entry.crc32c, Crc(v2));
  EXPECT_EQ(result.stats.unknown_skipped, 1u);
  EXPECT_EQ(result.stats.producer_sources, 1u);

  // The unknown record takes part in index and sequence checks.
  const EpisodeEvent parsed = Parse(v2);
  const std::string source = parsed.header().source_id();
  const uint64_t sequence = parsed.header().sequence();
  const std::string after = Fixture("unknown_payload_record") +
                            Record(1, MakeEvent(source, sequence));
  ExpectPrefix(Read(after), Defect::kOrderViolation, 1,
               kFileHeader + kRecordHeader + v2.size(), 0);
  const std::string gap = Fixture("unknown_payload_record") +
                          Record(2, MakeEvent(source, sequence + 1));
  ExpectPrefix(Read(gap), Defect::kIndexGap, 1,
               kFileHeader + kRecordHeader + v2.size(), 0);
  const std::string next = Fixture("unknown_payload_record") +
                           Record(1, MakeEvent(source, sequence + 1));
  EXPECT_EQ(Read(next).defect, Defect::kNone);
}

// #240: a record whose payload-less event carries only an unknown envelope
// field (U1, tag 6 varint) is an invalid event, not an unknown payload.
TEST(EpisodeReaderTest, U1UnknownEnvelopeFieldIsAMissingPayloadRecord) {
  EpisodeEvent base = MakeEvent("a", 1);
  base.clear_payload();
  const std::string u1 = base.SerializeAsString() + std::string("\x30\x01", 2);
  ASSERT_TRUE(Parse(u1).payload_case() == EpisodeEvent::PAYLOAD_NOT_SET);
  const std::string data = Header() + Record(0, u1);
  const EpisodeReadResult result = Read(data);
  EXPECT_EQ(result.status, EpisodeReaderStatus::kOk);
  EXPECT_EQ(result.defect, Defect::kInvalidEvent);
  EXPECT_EQ(result.event_error, EpisodeEventError::kMissingPayload);
  EXPECT_EQ(result.defect_offset, kFileHeader);
  EXPECT_EQ(result.good_records, 0u);
  EXPECT_EQ(result.good_end_offset, kFileHeader);
  EXPECT_TRUE(result.records.empty());
  EXPECT_EQ(result.stats.unknown_skipped, 0u);
}

// RT10: filters.
TEST(EpisodeReaderTest, RT10Filters) {
  // Indices: 0 a/safety, 1 b/trajectory, 2 a/trajectory, 5 loss (3 and 4
  // dropped), 6 b/safety.
  const std::string data =
      Header() + Record(0, MakeEvent("a", 1)) +
      Record(1, MakeTrajectory("b", 1)) + Record(2, MakeTrajectory("a", 2)) +
      LossRecord(LossSpec{}) + Record(6, MakeEvent("b", 2));
  const EpisodeReadResult all = Read(data);
  ExpectPrefix(all, Defect::kNone, 5, data.size(), 6);
  ASSERT_EQ(Indices(all), (std::vector<uint64_t>{0, 1, 2, 5, 6}));

  auto run = [&](const EpisodeReadFilter& filter) {
    const EpisodeReadResult result = Read(data, {}, filter);
    EXPECT_EQ(result.status, EpisodeReaderStatus::kOk);
    EXPECT_EQ(result.defect, all.defect);
    EXPECT_EQ(result.good_records, all.good_records);
    EXPECT_EQ(result.good_end_offset, all.good_end_offset);
    EXPECT_EQ(result.last_good_writer_index, all.last_good_writer_index);
    EXPECT_EQ(result.next_expected_writer_index,
              all.next_expected_writer_index);
    EXPECT_EQ(result.losses.size(), all.losses.size());
    EXPECT_EQ(result.stats.records, all.stats.records);
    EXPECT_EQ(result.stats.loss_records, all.stats.loss_records);
    EXPECT_EQ(result.stats.dropped_overflow, all.stats.dropped_overflow);
    EXPECT_EQ(result.stats.producer_sources, all.stats.producer_sources);
    return Indices(result);
  };
  using Indices_t = std::vector<uint64_t>;

  EpisodeReadFilter by_source;
  by_source.source_ids = {"a"};
  EXPECT_EQ(run(by_source), (Indices_t{0, 2}));
  by_source.source_ids = {"a", "b"};
  EXPECT_EQ(run(by_source), (Indices_t{0, 1, 2, 6}));

  EpisodeReadFilter by_kind;
  by_kind.payload_kinds = {PayloadKind::kVehicleTrajectory};
  EXPECT_EQ(run(by_kind), (Indices_t{1, 2}));
  by_kind.payload_kinds = {PayloadKind::kSafetyDecision,
                           PayloadKind::kEpisodeLoss};
  EXPECT_EQ(run(by_kind), (Indices_t{0, 5, 6}));

  EpisodeReadFilter by_range;
  by_range.has_index_range = true;
  by_range.first_writer_index = 1;
  by_range.last_writer_index = 5;
  EXPECT_EQ(run(by_range), (Indices_t{1, 2, 5}));
  by_range.first_writer_index = 3;
  by_range.last_writer_index = 4;
  EXPECT_EQ(run(by_range), (Indices_t{}));
  by_range.first_writer_index = 2;
  by_range.last_writer_index = 2;
  EXPECT_EQ(run(by_range), (Indices_t{2}));

  EpisodeReadFilter combined;
  combined.source_ids = {"a"};
  combined.payload_kinds = {PayloadKind::kVehicleTrajectory};
  EXPECT_EQ(run(combined), (Indices_t{2}));
  combined.has_index_range = true;
  combined.first_writer_index = 0;
  combined.last_writer_index = 1;
  EXPECT_EQ(run(combined), (Indices_t{}));
  EpisodeReadFilter combined2;
  combined2.source_ids = {"b"};
  combined2.has_index_range = true;
  combined2.first_writer_index = 0;
  combined2.last_writer_index = 1;
  EXPECT_EQ(run(combined2), (Indices_t{1}));

  // Loss records appear in `records` only when they pass the filter, and
  // always in `losses`.
  EpisodeReadFilter loss_source;
  loss_source.source_ids = {"episode_writer"};
  EXPECT_EQ(run(loss_source), (Indices_t{5}));
  EpisodeReadFilter loss_kind;
  loss_kind.payload_kinds = {PayloadKind::kEpisodeLoss};
  EXPECT_EQ(run(loss_kind), (Indices_t{5}));
  EpisodeReadFilter no_loss;
  no_loss.source_ids = {"a", "b"};
  const EpisodeReadResult without_loss = Read(data, {}, no_loss);
  ASSERT_EQ(without_loss.losses.size(), 1u);
  EXPECT_EQ(without_loss.losses[0].writer_index, 5u);

  // Unknown names match nothing and are not an error.
  EpisodeReadFilter unknown_source;
  unknown_source.source_ids = {"nobody"};
  EXPECT_EQ(run(unknown_source), (Indices_t{}));
  EpisodeReadFilter unknown_kind;
  unknown_kind.payload_kinds = {PayloadKind::kUnknown};
  EXPECT_EQ(run(unknown_kind), (Indices_t{}));

  // A filter never hides a defect.
  const NominalFile nominal;
  const std::string damaged = nominal.data().substr(0, 3000);
  EpisodeReadFilter nothing;
  nothing.source_ids = {"nobody"};
  const EpisodeReadResult plain = Read(damaged);
  const EpisodeReadResult filtered = Read(damaged, {}, nothing);
  EXPECT_EQ(plain.defect, Defect::kTruncatedRecord);
  EXPECT_EQ(filtered.defect, Defect::kTruncatedRecord);
  EXPECT_EQ(filtered.good_records, plain.good_records);
  EXPECT_EQ(filtered.good_end_offset, plain.good_end_offset);
  EXPECT_TRUE(filtered.records.empty());
  EXPECT_FALSE(plain.records.empty());

  // The filtered entries keep their file offsets and bytes.
  EpisodeReadFilter one;
  one.has_index_range = true;
  one.first_writer_index = 2;
  one.last_writer_index = 2;
  const EpisodeReadResult single = Read(data, {}, one);
  ASSERT_EQ(single.records.size(), 1u);
  EXPECT_EQ(single.records[0].offset, all.records[2].offset);
  EXPECT_EQ(EventString(single.records[0]), EventString(all.records[2]));

  // An invalid index range is an invalid config and reads nothing.
  EpisodeReadFilter inverted;
  inverted.has_index_range = true;
  inverted.first_writer_index = 3;
  inverted.last_writer_index = 2;
  const EpisodeReadResult bad = Read(data, {}, inverted);
  EXPECT_EQ(bad.status, EpisodeReaderStatus::kInvalidConfig);
  EXPECT_EQ(bad.defect, Defect::kNone);
  EXPECT_EQ(bad.good_records, 0u);
  EXPECT_EQ(bad.good_end_offset, 0u);
  EXPECT_TRUE(bad.records.empty());
  EXPECT_TRUE(bad.losses.empty());
}

// RT11: config bounds and limits.
TEST(EpisodeReaderTest, RT11ConfigBoundsAndLimits) {
  const NominalFile nominal;
  auto expect_invalid = [&](const EpisodeReaderConfig& config) {
    const EpisodeReadResult result = Read(nominal.data(), config);
    EXPECT_EQ(result.status, EpisodeReaderStatus::kInvalidConfig);
    EXPECT_EQ(result.defect, Defect::kNone);
    EXPECT_EQ(result.good_records, 0u);
    EXPECT_EQ(result.good_end_offset, 0u);
    EXPECT_TRUE(result.records.empty());
    EXPECT_TRUE(result.losses.empty());
    EXPECT_EQ(result.stats.records, 0u);
  };
  auto expect_valid = [&](const EpisodeReaderConfig& config) {
    const EpisodeReadResult result = Read(nominal.data(), config);
    EXPECT_EQ(result.status, EpisodeReaderStatus::kOk);
  };

  EpisodeReaderConfig config;
  config.max_event_bytes = 0;
  expect_invalid(config);
  config.max_event_bytes = kMaxEpisodeMaxEventBytes + 1;
  expect_invalid(config);
  config.max_event_bytes = 1;
  expect_valid(config);
  config.max_event_bytes = kMaxEpisodeMaxEventBytes;
  expect_valid(config);

  config = {};
  config.max_records = 0;
  expect_invalid(config);
  config.max_records = kMaxEpisodeReaderMaxRecords + 1;
  expect_invalid(config);
  EXPECT_EQ(kMaxEpisodeReaderMaxRecords, 16'777'216u);
  config.max_records = 1;
  expect_valid(config);
  config.max_records = kMaxEpisodeReaderMaxRecords;
  expect_valid(config);

  config = {};
  config.max_file_bytes = 11;
  expect_invalid(config);
  config.max_file_bytes = 0;
  expect_invalid(config);
  config.max_file_bytes = kMaxEpisodeReaderMaxFileBytes + 1;
  expect_invalid(config);
  EXPECT_EQ(kMaxEpisodeReaderMaxFileBytes, uint64_t{4} << 30);
  config.max_file_bytes = 12;
  EXPECT_EQ(Read(Header(), config).defect, Defect::kNone);
  config.max_file_bytes = kMaxEpisodeReaderMaxFileBytes;
  expect_valid(config);

  // The defaults.
  const EpisodeReaderConfig defaults;
  EXPECT_EQ(defaults.max_event_bytes, 16u * 1024 * 1024);
  EXPECT_EQ(defaults.max_records, 1'000'000u);
  EXPECT_EQ(defaults.max_file_bytes, uint64_t{1} << 30);

  // An invalid config wins over a short buffer and reads nothing.
  config = {};
  config.max_records = 0;
  EXPECT_EQ(Read("", config).status, EpisodeReaderStatus::kInvalidConfig);
  EXPECT_EQ(Read("", config).defect, Defect::kNone);

  // A buffer over max_file_bytes is too large. Equal is fine.
  config = {};
  config.max_file_bytes = nominal.data().size() - 1;
  const EpisodeReadResult too_large = Read(nominal.data(), config);
  EXPECT_EQ(too_large.status, EpisodeReaderStatus::kInputTooLarge);
  EXPECT_EQ(too_large.defect, Defect::kNone);
  EXPECT_EQ(too_large.good_records, 0u);
  EXPECT_TRUE(too_large.records.empty());
  config.max_file_bytes = nominal.data().size();
  ExpectPrefix(Read(nominal.data(), config), Defect::kNone, 5, 4069, 4);

  // max_records.
  config = {};
  config.max_records = 2;
  const EpisodeReadResult capped = Read(nominal.data(), config);
  ExpectPrefix(capped, Defect::kTooManyRecords, 2, nominal.records()[1].end(),
               1);
  EXPECT_EQ(capped.records.size(), 2u);
  config.max_records = 5;
  ExpectPrefix(Read(nominal.data(), config), Defect::kNone, 5, 4069, 4);
  // Loss records count toward the limit.
  const std::string overflow =
      TestdataBytes("episode_writer_v1_overflow.iepisod");
  config.max_records = 3;
  ExpectPrefix(Read(overflow, config), Defect::kTooManyRecords, 3, 1517, 2);
  config.max_records = 4;
  ExpectPrefix(Read(overflow, config), Defect::kNone, 4, 1607, 5);
  // The limit applies only to otherwise valid records.
  config.max_records = 2;
  EXPECT_EQ(Read(nominal.Prefix(2), config).defect, Defect::kNone);
  std::string corrupt_third = nominal.Prefix(3);
  corrupt_third[nominal.records()[2].offset + 4] ^= 1;
  EXPECT_EQ(Read(corrupt_third, config).defect, Defect::kCrcMismatch);

  // The pinned size of the largest writer loss record.
  EXPECT_EQ(MaxEpisodeLossRecordBytes(), 122u);
  LossSpec largest;
  largest.sequence = kMax;
  largest.first = kMax;
  largest.last = kMax;
  largest.count = kMax;
  largest.reason = EpisodeLoss::LOSS_REASON_TOO_LARGE;
  largest.source_seconds = largest.receive_seconds =
      std::numeric_limits<int64_t>::min();
  largest.source_nanos = largest.receive_nanos =
      std::numeric_limits<int32_t>::max();
  EXPECT_EQ(LossEvent(largest).ByteSizeLong(), MaxEpisodeLossRecordBytes());

  // A loss record larger than a tiny max_event_bytes is still read. The
  // writer accepts max_event_bytes = 1 and drops every event as too large.
  MemorySink sink;
  auto writer = MakeWriter(&sink, 4, 1);
  ASSERT_NE(writer, nullptr);
  for (uint64_t i = 0; i < 3; ++i) {
    EXPECT_EQ(writer->Append(MakeEvent("a", i + 1)).status,
              EpisodeWriterStatus::kEventTooLarge);
  }
  ASSERT_EQ(writer->Close(), EpisodeWriterStatus::kOk);
  EpisodeReaderConfig tiny;
  tiny.max_event_bytes = 1;
  const EpisodeReadResult tiny_read = Read(sink.output, tiny);
  EXPECT_EQ(tiny_read.defect, Defect::kNone);
  ASSERT_EQ(tiny_read.losses.size(), 1u);
  EXPECT_EQ(tiny_read.losses[0].first_dropped_writer_index, 0u);
  EXPECT_EQ(tiny_read.losses[0].last_dropped_writer_index, 2u);
  EXPECT_EQ(tiny_read.losses[0].dropped_count, 3u);
  EXPECT_EQ(tiny_read.losses[0].writer_index, 3u);
  EXPECT_EQ(tiny_read.stats.dropped_too_large, 3u);
  EXPECT_GT(Split(sink.output)[0].length, 1u);
}

// RT12: determinism.
TEST(EpisodeReaderTest, RT12Determinism) {
  const NominalFile nominal;
  const std::string overflow =
      TestdataBytes("episode_writer_v1_overflow.iepisod");
  const std::string damaged = Fixture("crc_flip");
  for (const std::string* data : {&nominal.data(), &overflow, &damaged}) {
    ExpectSameResult(Read(*data), Read(*data));
  }
  EpisodeReadFilter filter;
  filter.payload_kinds = {PayloadKind::kSafetyDecision};
  ExpectSameResult(Read(overflow, {}, filter), Read(overflow, {}, filter));
  // A copy of the buffer gives the same entries relative to its own start.
  const std::string copy = nominal.data();
  const EpisodeReadResult a = Read(nominal.data());
  const EpisodeReadResult b = Read(copy);
  ASSERT_EQ(a.records.size(), b.records.size());
  for (size_t i = 0; i < a.records.size(); ++i) {
    EXPECT_EQ(a.records[i].offset, b.records[i].offset);
    EXPECT_EQ(EventString(a.records[i]), EventString(b.records[i]));
  }
}

// RT13: writer round trip, and the file loader.
TEST(EpisodeReaderTest, RT13WriterRoundTrip) {
  // The five goldens through the writer.
  MemorySink nominal_sink;
  {
    auto writer = MakeWriter(&nominal_sink, kDefaultEpisodeWriterCapacity,
                             kDefaultEpisodeMaxEventBytes);
    ASSERT_NE(writer, nullptr);
    uint64_t sequence = 1;
    for (const char* name : kPayloadNames) {
      EpisodeEvent event = Golden(name);
      event.mutable_header()->set_sequence(sequence++);
      EXPECT_EQ(writer->Append(event).status, EpisodeWriterStatus::kOk);
    }
    ASSERT_EQ(writer->Close(), EpisodeWriterStatus::kOk);
  }
  {
    const EpisodeReadResult result = Read(nominal_sink.output);
    ExpectPrefix(result, Defect::kNone, 5, nominal_sink.output.size(), 4);
    EXPECT_EQ(nominal_sink.output,
              TestdataBytes("episode_writer_v1_nominal.iepisod"));
  }

  // A TOO_LARGE drop, an accept after it, then an OVERFLOW drop that switches
  // the reason: two contiguous loss ranges.
  const size_t small = MakeEvent("a", 1).ByteSizeLong();
  const size_t max_event_bytes = small + 100;
  MemorySink sink;
  {
    auto writer = MakeWriter(&sink, 4, max_event_bytes);
    ASSERT_NE(writer, nullptr);
    EXPECT_EQ(writer->Append(MakeEvent("a", 1)).status,
              EpisodeWriterStatus::kOk);  // index 0
    EXPECT_EQ(writer->Append(MakeEvent("a", 2)).status,
              EpisodeWriterStatus::kOk);  // index 1
    EXPECT_EQ(writer->Append(MakeEvent("a", 3, max_event_bytes)).status,
              EpisodeWriterStatus::kEventTooLarge);  // index 2
    EXPECT_EQ(writer->Append(MakeEvent("a", 4)).status,
              EpisodeWriterStatus::kBufferFull);  // index 4, TOO_LARGE at 3
    ASSERT_EQ(writer->Close(), EpisodeWriterStatus::kOk);
  }
  {
    const EpisodeReadResult result = Read(sink.output);
    ExpectPrefix(result, Defect::kNone, 4, sink.output.size(), 5);
    EXPECT_EQ(Indices(result), (std::vector<uint64_t>{0, 1, 3, 5}));
    ASSERT_EQ(result.losses.size(), 2u);
    EXPECT_EQ(result.losses[0].writer_index, 3u);
    EXPECT_EQ(result.losses[0].first_dropped_writer_index, 2u);
    EXPECT_EQ(result.losses[0].last_dropped_writer_index, 2u);
    EXPECT_EQ(result.losses[0].reason,
              static_cast<int>(EpisodeLoss::LOSS_REASON_TOO_LARGE));
    EXPECT_EQ(result.losses[0].sequence, 1u);
    EXPECT_EQ(result.losses[1].writer_index, 5u);
    EXPECT_EQ(result.losses[1].first_dropped_writer_index, 4u);
    EXPECT_EQ(result.losses[1].last_dropped_writer_index, 4u);
    EXPECT_EQ(result.losses[1].reason,
              static_cast<int>(EpisodeLoss::LOSS_REASON_OVERFLOW));
    EXPECT_EQ(result.losses[1].sequence, 2u);
    EXPECT_EQ(result.stats.dropped_too_large, 1u);
    EXPECT_EQ(result.stats.dropped_overflow, 1u);
    EXPECT_EQ(result.stats.loss_records, 2u);
  }

  // An accept after a TOO_LARGE drop writes the loss record first.
  {
    MemorySink after_drop;
    auto writer = MakeWriter(&after_drop, 4, max_event_bytes);
    ASSERT_NE(writer, nullptr);
    EXPECT_EQ(writer->Append(MakeEvent("a", 1)).status,
              EpisodeWriterStatus::kOk);  // index 0
    EXPECT_EQ(writer->Append(MakeEvent("a", 2, max_event_bytes)).status,
              EpisodeWriterStatus::kEventTooLarge);  // index 1
    EXPECT_EQ(writer->Append(MakeEvent("a", 3)).status,
              EpisodeWriterStatus::kOk);  // loss at index 2, event at 3
    ASSERT_EQ(writer->Close(), EpisodeWriterStatus::kOk);
    const EpisodeReadResult result = Read(after_drop.output);
    ExpectPrefix(result, Defect::kNone, 3, after_drop.output.size(), 3);
    EXPECT_EQ(Indices(result), (std::vector<uint64_t>{0, 2, 3}));
    ASSERT_EQ(result.losses.size(), 1u);
    EXPECT_EQ(result.losses[0].first_dropped_writer_index, 1u);
    EXPECT_EQ(result.losses[0].last_dropped_writer_index, 1u);
  }

  // Accept after a drop, with a mix of both reasons and a long run.
  MemorySink mixed;
  {
    auto writer = MakeWriter(&mixed, 8, max_event_bytes);
    ASSERT_NE(writer, nullptr);
    uint64_t sequence = 1;
    for (int round = 0; round < 3; ++round) {
      for (int i = 0; i < 9; ++i) {
        writer->Append(MakeEvent("a", sequence++));
      }
      writer->Append(MakeEvent("a", sequence++, max_event_bytes));
      ASSERT_EQ(writer->Flush(), EpisodeWriterStatus::kOk);
      writer->Append(MakeEvent("b", round + 1));
      writer->Append(MakeEvent("a", sequence++));
    }
    ASSERT_EQ(writer->Close(), EpisodeWriterStatus::kOk);
    const EpisodeWriterStats stats = writer->Stats();
    const EpisodeReadResult result = Read(mixed.output);
    EXPECT_EQ(result.defect, Defect::kNone);
    EXPECT_EQ(result.good_end_offset, mixed.output.size());
    EXPECT_EQ(result.stats.loss_records, stats.loss_records);
    EXPECT_EQ(result.stats.dropped_overflow, stats.dropped_overflow);
    EXPECT_EQ(result.stats.dropped_too_large, stats.dropped_too_large);
    EXPECT_EQ(result.good_records, stats.written_records);
    EXPECT_EQ(result.next_expected_writer_index, stats.next_writer_index);
  }

  // A cancelled writer that never reached the sink leaves a 0-byte file.
  {
    MemorySink empty;
    auto writer = MakeWriter(&empty, 4, 1024);
    ASSERT_NE(writer, nullptr);
    writer->Append(MakeEvent("a", 1));
    writer->Cancel();
    EXPECT_TRUE(empty.output.empty());
    ExpectPrefix(Read(empty.output), Defect::kTruncatedHeader, 0, 0,
                 std::nullopt);
  }
  // A closed empty episode is the header alone.
  {
    MemorySink empty;
    auto writer = MakeWriter(&empty, 4, 1024);
    ASSERT_NE(writer, nullptr);
    ASSERT_EQ(writer->Close(), EpisodeWriterStatus::kOk);
    ExpectPrefix(Read(empty.output), Defect::kNone, 0, 12, std::nullopt);
  }

  // LoadEpisodeFile on a FileEpisodeSink file equals the in-memory bytes.
  const char* tmp = std::getenv("TEST_TMPDIR");
  ASSERT_NE(tmp, nullptr);
  const std::string path = std::string(tmp) + "/rt13_round_trip.iepisod";
  {
    auto file_sink = FileEpisodeSink::Open(path);
    ASSERT_TRUE(file_sink.ok()) << file_sink.status();
    auto writer = MakeWriter(file_sink->get(), 4, max_event_bytes);
    ASSERT_NE(writer, nullptr);
    writer->Append(MakeEvent("a", 1));
    writer->Append(MakeEvent("a", 2));
    writer->Append(MakeEvent("a", 3, max_event_bytes));
    writer->Append(MakeEvent("a", 4));
    writer->Append(MakeEvent("a", 5));
    ASSERT_EQ(writer->Close(), EpisodeWriterStatus::kOk);
  }
  MemorySink twin;
  {
    auto writer = MakeWriter(&twin, 4, max_event_bytes);
    ASSERT_NE(writer, nullptr);
    writer->Append(MakeEvent("a", 1));
    writer->Append(MakeEvent("a", 2));
    writer->Append(MakeEvent("a", 3, max_event_bytes));
    writer->Append(MakeEvent("a", 4));
    writer->Append(MakeEvent("a", 5));
    ASSERT_EQ(writer->Close(), EpisodeWriterStatus::kOk);
  }
  const absl::StatusOr<std::string> loaded =
      LoadEpisodeFile(path, kDefaultEpisodeReaderMaxFileBytes);
  ASSERT_TRUE(loaded.ok()) << loaded.status();
  EXPECT_EQ(*loaded, twin.output);
  EXPECT_EQ(Read(*loaded).defect, Defect::kNone);
  EXPECT_GT(Read(*loaded).stats.loss_records, 0u);

  // The size limit: equal is fine, one below is a failed precondition.
  EXPECT_TRUE(LoadEpisodeFile(path, loaded->size()).ok());
  const absl::StatusOr<std::string> over =
      LoadEpisodeFile(path, loaded->size() - 1);
  ASSERT_FALSE(over.ok());
  EXPECT_EQ(over.status().code(), absl::StatusCode::kFailedPrecondition);

  // A missing path, and an invalid limit.
  const absl::StatusOr<std::string> missing =
      LoadEpisodeFile(std::string(tmp) + "/does_not_exist.iepisod", 1024);
  ASSERT_FALSE(missing.ok());
  EXPECT_EQ(missing.status().code(), absl::StatusCode::kNotFound);
  EXPECT_EQ(LoadEpisodeFile(path, 11).status().code(),
            absl::StatusCode::kInvalidArgument);
  EXPECT_EQ(LoadEpisodeFile(path, 0).status().code(),
            absl::StatusCode::kInvalidArgument);
  EXPECT_EQ(
      LoadEpisodeFile(path, kMaxEpisodeReaderMaxFileBytes + 1).status().code(),
      absl::StatusCode::kInvalidArgument);
  // An invalid limit is reported even for a missing path.
  EXPECT_EQ(LoadEpisodeFile(std::string(tmp) + "/does_not_exist.iepisod", 11)
                .status()
                .code(),
            absl::StatusCode::kInvalidArgument);
  // A directory cannot be read.
  EXPECT_FALSE(LoadEpisodeFile(tmp, 1024).ok());

  // A 0-byte file loads and reads as a truncated header.
  const std::string empty_path = std::string(tmp) + "/rt13_empty.iepisod";
  {
    std::ofstream out(empty_path, std::ios::binary);
  }
  const absl::StatusOr<std::string> empty_loaded =
      LoadEpisodeFile(empty_path, 1024);
  ASSERT_TRUE(empty_loaded.ok()) << empty_loaded.status();
  EXPECT_TRUE(empty_loaded->empty());
  EXPECT_EQ(Read(*empty_loaded).defect, Defect::kTruncatedHeader);

  // The committed goldens load and read.
  const std::string nominal_path = [&] {
    const char* src = std::getenv("TEST_SRCDIR");
    EXPECT_NE(src, nullptr);
    const char* workspace = std::getenv("TEST_WORKSPACE");
    return std::string(src) + "/" +
           (workspace != nullptr ? workspace : "_main") +
           "/intrinsic_data/episode/testdata/episode_writer_v1_nominal.iepisod";
  }();
  const absl::StatusOr<std::string> golden =
      LoadEpisodeFile(nominal_path, kDefaultEpisodeReaderMaxFileBytes);
  ASSERT_TRUE(golden.ok()) << golden.status();
  EXPECT_EQ(*golden, TestdataBytes("episode_writer_v1_nominal.iepisod"));
}

// RT14: a bit flip in writer_index only. The CRC does not cover it.
TEST(EpisodeReaderTest, RT14WriterIndexBitFlips) {
  const NominalFile nominal;
  const size_t end2 = nominal.records()[1].end();
  ExpectFixture("index_bit_flip_gap", Defect::kIndexGap, 2, end2, 1);
  ExpectFixture("index_bit_flip_decrease", Defect::kIndexNotIncreasing, 2, end2,
                1);
  ExpectFixture("index_bit_flip_loss", Defect::kLossRecordInvalid, 3, 1517, 2);

  // Every one of the 64 bits of record 2's index (2): bit 0 gives 3, bit 1
  // gives 0, any higher bit gives a larger index. The CRC stays valid.
  const size_t off2 = nominal.records()[2].offset;
  for (int bit = 0; bit < 64; ++bit) {
    SCOPED_TRACE(bit);
    std::string data = nominal.data();
    SetLe(data, off2 + 8, uint64_t{2} ^ (uint64_t{1} << bit), 8);
    EXPECT_EQ(static_cast<uint32_t>(GetLe(data, off2 + 4, 4)),
              Crc(nominal.records()[2].event));
    const Defect expected =
        bit == 1 ? Defect::kIndexNotIncreasing : Defect::kIndexGap;
    ExpectPrefix(Read(data), expected, 2, end2, 1);
  }

  // Every one of the 64 bits of the loss record's index (5): the record no
  // longer sits at last + 1.
  const std::string overflow =
      TestdataBytes("episode_writer_v1_overflow.iepisod");
  const std::vector<RawRecord> records = Split(overflow);
  ASSERT_EQ(records.size(), 4u);
  for (int bit = 0; bit < 64; ++bit) {
    SCOPED_TRACE(bit);
    std::string data = overflow;
    SetLe(data, records[3].offset + 8, uint64_t{5} ^ (uint64_t{1} << bit), 8);
    ExpectPrefix(Read(data), Defect::kLossRecordInvalid, 3, records[2].end(),
                 2);
  }
}

// The writer's own constants, as the format normative section lists them.
TEST(EpisodeReaderTest, WriterConstantsMatchTheFormat) {
  EXPECT_EQ(std::string(kEpisodeFileMagic, sizeof(kEpisodeFileMagic)),
            "IEPISOD1");
  EXPECT_EQ(kEpisodeFormatVersion, 1u);
  EXPECT_EQ(kMaxEpisodeSources, 64u);
  EXPECT_EQ(kEpisodeWriterSourceId, "episode_writer");
  EXPECT_EQ(kEpisodeWriterFrameId, "episode_writer");
  EXPECT_EQ(Header(), TestdataBytes("episode_writer_v1_nominal.iepisod")
                          .substr(0, kFileHeader));
}

}  // namespace
}  // namespace intrinsic::episode
