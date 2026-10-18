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

// RC1 to RC12: the deterministic episode replay clock (#130, lock id
// episode-replay-clock-v1).

#include "intrinsic_data/episode/episode_replay.h"

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <fstream>
#include <limits>
#include <memory>
#include <new>
#include <ostream>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include "absl/crc/crc32c.h"
#include "absl/types/span.h"
#include "gtest/gtest.h"
#include "intrinsic/embodiment/stamped_header_policy.h"
#include "intrinsic/episode/proto/episode_event.pb.h"
#include "intrinsic_data/episode/episode_event_policy.h"
#include "intrinsic_data/episode/episode_reader.h"

// RC12: a test-binary-local replacement of every global allocation function,
// the throwing and the nothrow forms. It counts allocations only while armed.
namespace {
std::atomic<bool> g_count_allocations{false};
std::atomic<uint64_t> g_allocations{0};

void CountAllocation() {
  if (g_count_allocations.load(std::memory_order_relaxed)) {
    g_allocations.fetch_add(1, std::memory_order_relaxed);
  }
}

void* Allocate(std::size_t size) {
  CountAllocation();
  return std::malloc(size == 0 ? 1 : size);
}

void* AllocateAligned(std::size_t size, std::align_val_t alignment) {
  CountAllocation();
  const std::size_t align = static_cast<std::size_t>(alignment);
  const std::size_t rounded =
      ((size == 0 ? 1 : size) + align - 1) / align * align;
  return std::aligned_alloc(align, rounded);
}

void* AllocateOrThrow(std::size_t size) {
  void* pointer = Allocate(size);
  if (pointer == nullptr) {
    throw std::bad_alloc();
  }
  return pointer;
}

void* AllocateAlignedOrThrow(std::size_t size, std::align_val_t alignment) {
  void* pointer = AllocateAligned(size, alignment);
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
void* operator new(std::size_t size, const std::nothrow_t&) noexcept {
  return Allocate(size);
}
void* operator new[](std::size_t size, const std::nothrow_t&) noexcept {
  return Allocate(size);
}
void* operator new(std::size_t size, std::align_val_t alignment,
                   const std::nothrow_t&) noexcept {
  return AllocateAligned(size, alignment);
}
void* operator new[](std::size_t size, std::align_val_t alignment,
                     const std::nothrow_t&) noexcept {
  return AllocateAligned(size, alignment);
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
void operator delete(void* pointer, const std::nothrow_t&) noexcept {
  std::free(pointer);
}
void operator delete[](void* pointer, const std::nothrow_t&) noexcept {
  std::free(pointer);
}
void operator delete(void* pointer, std::align_val_t,
                     const std::nothrow_t&) noexcept {
  std::free(pointer);
}
void operator delete[](void* pointer, std::align_val_t,
                       const std::nothrow_t&) noexcept {
  std::free(pointer);
}

namespace intrinsic::episode {

void PrintTo(EpisodeReplayStatus status, std::ostream* os) {
  *os << "EpisodeReplayStatus(" << static_cast<int>(status) << ")";
}
void PrintTo(EpisodeReadDefect defect, std::ostream* os) {
  *os << "EpisodeReadDefect(" << static_cast<int>(defect) << ")";
}

namespace {

using embodiment::ClockReading;
using intrinsic_proto::episode::EpisodeEvent;
using intrinsic_proto::episode::EpisodeLoss;
using Status = EpisodeReplayStatus;
using Defect = EpisodeReadDefect;

constexpr size_t kRecordHeader = 16;
constexpr int64_t kSecond = 1'000'000'000;
constexpr int64_t kMs = 1'000'000;
constexpr int64_t kBaseSeconds = 1'700'000'000;
constexpr int64_t kInt64Max = std::numeric_limits<int64_t>::max();

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

std::string Nominal() {
  return TestdataBytes("episode_writer_v1_nominal.iepisod");
}
std::string Overflow() {
  return TestdataBytes("episode_writer_v1_overflow.iepisod");
}
std::string ReaderFixture(const std::string& name) {
  return TestdataBytes("episode_reader_v1_" + name + ".iepisod");
}

EpisodeEvent Parse(const std::string& bytes) {
  EpisodeEvent message;
  EXPECT_TRUE(message.ParseFromString(bytes));
  return message;
}

absl::Span<const uint8_t> Span(const std::string& data) {
  return absl::MakeConstSpan(reinterpret_cast<const uint8_t*>(data.data()),
                             data.size());
}

// Test-local framing with a recomputed CRC32C. The replayer never builds a
// stream.
void PutLe(std::string& out, uint64_t value, int bytes) {
  for (int i = 0; i < bytes; ++i) {
    out.push_back(static_cast<char>((value >> (8 * i)) & 0xff));
  }
}

std::string FileHeader() {
  std::string header = "IEPISOD1";
  PutLe(header, 1, 4);
  return header;
}

std::string Frame(uint64_t index, const EpisodeEvent& event) {
  const std::string bytes = event.SerializeAsString();
  std::string record;
  PutLe(record, bytes.size(), 4);
  PutLe(record, static_cast<uint32_t>(absl::ComputeCrc32c(bytes)), 4);
  PutLe(record, index, 8);
  record += bytes;
  return record;
}

// A valid event. `ns` is the receive time, as nanoseconds after
// kBaseSeconds. The source time is the same.
EpisodeEvent MakeEventAtSeconds(const std::string& source, uint64_t sequence,
                                int64_t seconds, int32_t nanos,
                                const std::string& domain = "monotonic") {
  EpisodeEvent event =
      Parse(TestdataBytes("episode_event_v1_safety_decision.binpb"));
  auto* header = event.mutable_header();
  header->set_source_id(source);
  header->set_sequence(sequence);
  header->mutable_source_time()->set_seconds(seconds);
  header->mutable_source_time()->set_nanos(nanos);
  header->mutable_receive_time()->set_seconds(seconds);
  header->mutable_receive_time()->set_nanos(nanos);
  header->set_clock_domain(domain);
  return event;
}

EpisodeEvent MakeEvent(const std::string& source, uint64_t sequence, int64_t ns,
                       const std::string& domain = "monotonic") {
  return MakeEventAtSeconds(source, sequence, kBaseSeconds + ns / kSecond,
                            static_cast<int32_t>(ns % kSecond), domain);
}

// A writer-shaped loss record: reserved source, empty clock domain, source
// time equal to receive time.
EpisodeEvent MakeLoss(uint64_t ordinal, uint64_t first, uint64_t last,
                      int64_t ns) {
  EpisodeEvent event;
  auto* header = event.mutable_header();
  header->set_source_id("episode_writer");
  header->set_frame_id("episode_writer");
  header->set_sequence(ordinal);
  for (auto* time :
       {header->mutable_source_time(), header->mutable_receive_time()}) {
    time->set_seconds(kBaseSeconds + ns / kSecond);
    time->set_nanos(static_cast<int32_t>(ns % kSecond));
  }
  event.set_schema_version(1);
  auto* loss = event.mutable_episode_loss();
  loss->set_dropped_count(last - first + 1);
  loss->set_first_dropped_writer_index(first);
  loss->set_last_dropped_writer_index(last);
  loss->set_reason(EpisodeLoss::LOSS_REASON_OVERFLOW);
  return event;
}

EpisodeEvent MakeUnknown(const std::string& source, uint64_t sequence,
                         int64_t ns, const std::string& domain) {
  EpisodeEvent event =
      Parse(TestdataBytes("episode_event_v2_additive_unknown_payload.binpb"));
  auto* header = event.mutable_header();
  header->set_source_id(source);
  header->set_sequence(sequence);
  for (auto* time :
       {header->mutable_source_time(), header->mutable_receive_time()}) {
    time->set_seconds(kBaseSeconds + ns / kSecond);
    time->set_nanos(static_cast<int32_t>(ns % kSecond));
  }
  header->set_clock_domain(domain);
  return event;
}

// A file whose records get the writer indices 0, 1, 2, ...
std::string Episode(const std::vector<EpisodeEvent>& events) {
  std::string data = FileHeader();
  for (size_t i = 0; i < events.size(); ++i) {
    data += Frame(i, events[i]);
  }
  return data;
}

// One source, sequences 1, 2, ... and the given receive times.
std::string Timeline(const std::vector<int64_t>& times_ns,
                     const std::string& domain = "monotonic") {
  std::vector<EpisodeEvent> events;
  for (size_t i = 0; i < times_ns.size(); ++i) {
    events.push_back(MakeEvent("a", i + 1, times_ns[i], domain));
  }
  return Episode(events);
}

ClockReading Add(ClockReading time, int64_t ns) {
  __int128 total = static_cast<__int128>(time.seconds) * kSecond + time.nanos;
  total += ns;
  __int128 seconds = total / kSecond;
  __int128 nanos = total % kSecond;
  if (nanos < 0) {
    nanos += kSecond;
    --seconds;
  }
  return {static_cast<int64_t>(seconds), static_cast<int32_t>(nanos)};
}

// ClockReading has no operator==, so tests compare (seconds, nanos) pairs.
std::pair<int64_t, int32_t> P(ClockReading time) {
  return {time.seconds, time.nanos};
}

struct Row {
  uint64_t ordinal = 0;
  uint64_t writer_index = 0;
  int64_t recorded_offset_ns = 0;
  int64_t due_offset_ns = 0;
  bool operator==(const Row&) const = default;
};

std::ostream& operator<<(std::ostream& os, const Row& row) {
  return os << "(" << row.ordinal << ", " << row.writer_index << ", "
            << row.recorded_offset_ns << ", " << row.due_offset_ns << ")";
}

Row RowOf(const EpisodeReplayEvent& event) {
  return {event.ordinal, event.writer_index, event.recorded_offset_ns,
          event.due_offset_ns};
}

// Expected rows from independently written literals.
std::vector<Row> Rows(const std::vector<uint64_t>& indices,
                      const std::vector<int64_t>& recorded,
                      const std::vector<int64_t>& due) {
  std::vector<Row> rows;
  for (size_t i = 0; i < indices.size(); ++i) {
    rows.push_back({i, indices[i], recorded[i], due[i]});
  }
  return rows;
}

EpisodeReplayConfig Step() { return {}; }

EpisodeReplayConfig StepAt(uint32_t num, uint32_t den) {
  EpisodeReplayConfig config;
  config.speed_num = num;
  config.speed_den = den;
  return config;
}

EpisodeReplayConfig Timed(uint32_t num = 1, uint32_t den = 1) {
  EpisodeReplayConfig config;
  config.mode = EpisodeReplayMode::kTimed;
  config.speed_num = num;
  config.speed_den = den;
  return config;
}

EpisodeReplayConfig WithPrefix(EpisodeReplayConfig config) {
  config.allow_defective_prefix = true;
  return config;
}

std::unique_ptr<EpisodeReplayer> Make(const std::string& data,
                                      const EpisodeReplayConfig& config,
                                      const EpisodeReadFilter& filter = {},
                                      const EpisodeReaderConfig& reader = {}) {
  Status status = Status::kInvalidConfig;
  auto replayer =
      EpisodeReplayer::Create(Span(data), reader, filter, config, &status);
  EXPECT_EQ(status, Status::kOk);
  EXPECT_NE(replayer, nullptr);
  return replayer;
}

Status Reject(const std::string& data, const EpisodeReplayConfig& config,
              const EpisodeReadFilter& filter = {},
              const EpisodeReaderConfig& reader = {}) {
  Status status = Status::kOk;
  auto replayer =
      EpisodeReplayer::Create(Span(data), reader, filter, config, &status);
  EXPECT_EQ(replayer, nullptr);
  return status;
}

std::vector<Row> DrainStep(EpisodeReplayer* replayer,
                           EpisodeReplayResult* end = nullptr) {
  std::vector<Row> rows;
  for (int guard = 0; guard < 1000; ++guard) {
    const EpisodeReplayResult result = replayer->Step();
    if (result.status != Status::kOk) {
      if (end != nullptr) {
        *end = result;
      }
      return rows;
    }
    rows.push_back(RowOf(result.event));
  }
  ADD_FAILURE() << "Step never ended";
  return rows;
}

// Starts at `start`, then polls with a fake clock that jumps to each
// next_due. Checks that next_due is exactly start + the event's due offset,
// that one nanosecond earlier is not due, and returns the rows.
std::vector<Row> DrainTimed(EpisodeReplayer* replayer, ClockReading start,
                            EpisodeReplayResult* end = nullptr) {
  EXPECT_EQ(replayer->Start(start), Status::kOk);
  std::vector<Row> rows;
  ClockReading now = start;
  for (int guard = 0; guard < 1000; ++guard) {
    EpisodeReplayResult result = replayer->Poll(now);
    if (result.status == Status::kNotDue) {
      EXPECT_NE(P(result.next_due), P(now));
      EXPECT_EQ(replayer->Poll(Add(result.next_due, -1)).status,
                Status::kNotDue);
      now = result.next_due;
      result = replayer->Poll(now);
    }
    if (result.status != Status::kOk) {
      if (end != nullptr) {
        *end = result;
      }
      return rows;
    }
    EXPECT_EQ(P(now), P(Add(start, result.event.due_offset_ns)));
    rows.push_back(RowOf(result.event));
  }
  ADD_FAILURE() << "Poll never ended";
  return rows;
}

const ClockReading kStart = {1000, 999'999'999};

EpisodeReadFilter KindFilter(std::vector<PayloadKind> kinds) {
  EpisodeReadFilter filter;
  filter.payload_kinds = std::move(kinds);
  return filter;
}

TEST(EpisodeReplayTest, StatusNumbersAndDefaultsArePermanent) {
  EXPECT_EQ(static_cast<int>(Status::kOk), 0);
  EXPECT_EQ(static_cast<int>(Status::kInvalidConfig), 1);
  EXPECT_EQ(static_cast<int>(Status::kDefectiveStream), 2);
  EXPECT_EQ(static_cast<int>(Status::kMixedClockDomain), 3);
  EXPECT_EQ(static_cast<int>(Status::kTimeOutOfRange), 4);
  EXPECT_EQ(static_cast<int>(Status::kWrongMode), 5);
  EXPECT_EQ(static_cast<int>(Status::kNotStarted), 6);
  EXPECT_EQ(static_cast<int>(Status::kNotDue), 7);
  EXPECT_EQ(static_cast<int>(Status::kEndOfStream), 8);
  EXPECT_EQ(static_cast<int>(Status::kEndOfTrustedPrefix), 9);
  EXPECT_EQ(static_cast<int>(Status::kCancelled), 10);
  EXPECT_EQ(static_cast<int>(Status::kInvalidTime), 11);
  EXPECT_EQ(static_cast<int>(EpisodeReplayMode::kStep), 0);
  EXPECT_EQ(static_cast<int>(EpisodeReplayMode::kTimed), 1);

  const EpisodeReplayConfig config;
  EXPECT_EQ(config.mode, EpisodeReplayMode::kStep);
  EXPECT_EQ(config.speed_num, 1u);
  EXPECT_EQ(config.speed_den, 1u);
  EXPECT_FALSE(config.allow_defective_prefix);

  const EpisodeReplayStats stats;
  EXPECT_EQ(stats.total, 0u);
  EXPECT_EQ(stats.emitted, 0u);
  EXPECT_EQ(stats.loss_emitted, 0u);
  EXPECT_EQ(stats.skipped_unknown, 0u);
  EXPECT_EQ(stats.time_regressions, 0u);
  EXPECT_EQ(stats.cancelled_remaining, 0u);
}

// RC1.
TEST(EpisodeReplayTest, RC1StepEmitsTheNominalGoldenOneEventPerCall) {
  const std::string data = Nominal();
  auto replayer = Make(data, Step());
  ASSERT_NE(replayer, nullptr);
  EXPECT_EQ(replayer->Stats().total, 5u);

  for (uint64_t i = 0; i < 5; ++i) {
    const EpisodeReplayResult result = replayer->Step();
    ASSERT_EQ(result.status, Status::kOk) << i;
    EXPECT_EQ(result.event.ordinal, i);
    EXPECT_EQ(result.event.writer_index, i);
    ASSERT_NE(result.event.record, nullptr);
    EXPECT_EQ(result.event.loss, nullptr);
    EXPECT_EQ(result.event.record->writer_index, i);
    EXPECT_EQ(replayer->Stats().emitted, i + 1);
    // The bytes are the file's bytes, not a copy.
    const auto bytes = result.event.record->event_bytes;
    ASSERT_FALSE(bytes.empty());
    EXPECT_GE(bytes.data(), reinterpret_cast<const uint8_t*>(data.data()) +
                                result.event.record->offset + kRecordHeader);
    EXPECT_LE(bytes.data() + bytes.size(),
              reinterpret_cast<const uint8_t*>(data.data()) + data.size());
  }
  for (int i = 0; i < 3; ++i) {
    const EpisodeReplayResult result = replayer->Step();
    EXPECT_EQ(result.status, Status::kEndOfStream);
    EXPECT_EQ(result.reader_defect, Defect::kNone);
  }
  EXPECT_EQ(replayer->Stats().emitted, 5u);
}

TEST(EpisodeReplayTest, RC1StepIgnoresTimeAndReportsTheSameOffsets) {
  const std::string data = Nominal();
  auto replayer = Make(data, Step());
  ASSERT_NE(replayer, nullptr);
  EXPECT_EQ(DrainStep(replayer.get()),
            Rows({0, 1, 2, 3, 4},
                 {0, 1 * kSecond, 2 * kSecond, 3 * kSecond, 4 * kSecond},
                 {0, 1 * kSecond, 2 * kSecond, 3 * kSecond, 4 * kSecond}));

  auto scaled = Make(data, StepAt(10, 1));
  ASSERT_NE(scaled, nullptr);
  EXPECT_EQ(DrainStep(scaled.get()),
            Rows({0, 1, 2, 3, 4},
                 {0, 1 * kSecond, 2 * kSecond, 3 * kSecond, 4 * kSecond},
                 {0, 100 * kMs, 200 * kMs, 300 * kMs, 400 * kMs}));
}

// RC2.
TEST(EpisodeReplayTest, RC2TimedRealTimeIsDueAtTheRecordedOffsets) {
  const std::string data = Nominal();
  auto replayer = Make(data, Timed());
  ASSERT_NE(replayer, nullptr);
  ASSERT_EQ(replayer->Start(kStart), Status::kOk);

  EpisodeReplayResult result = replayer->Poll(kStart);
  ASSERT_EQ(result.status, Status::kOk);
  EXPECT_EQ(RowOf(result.event), (Row{0, 0, 0, 0}));

  for (uint64_t i = 1; i < 5; ++i) {
    const ClockReading due = Add(kStart, static_cast<int64_t>(i) * kSecond);
    // The previous event is already emitted. 1 ns early is not due.
    result = replayer->Poll(Add(due, -1));
    ASSERT_EQ(result.status, Status::kNotDue) << i;
    EXPECT_EQ(P(result.next_due), P(due));
    EXPECT_GE(result.next_due.nanos, 0);
    EXPECT_LT(result.next_due.nanos, embodiment::kNanosPerSecond);
    // Not due also leaves the replayer where it was.
    EXPECT_EQ(replayer->Stats().emitted, i);
    result = replayer->Poll(due);
    ASSERT_EQ(result.status, Status::kOk) << i;
    const int64_t offset = static_cast<int64_t>(i) * kSecond;
    EXPECT_EQ(RowOf(result.event), (Row{i, i, offset, offset}));
  }
  EXPECT_EQ(replayer->Poll(Add(kStart, 100 * kSecond)).status,
            Status::kEndOfStream);
}

TEST(EpisodeReplayTest, RC2PollReturnsAtMostOneEventAndAnEarlierNowIsAllowed) {
  const std::string data = Nominal();
  auto replayer = Make(data, Timed());
  ASSERT_NE(replayer, nullptr);
  ASSERT_EQ(replayer->Start(kStart), Status::kOk);
  const ClockReading late = Add(kStart, 100 * kSecond);
  for (uint64_t i = 0; i < 5; ++i) {
    const EpisodeReplayResult result = replayer->Poll(late);
    ASSERT_EQ(result.status, Status::kOk);
    EXPECT_EQ(result.event.ordinal, i);
  }
  EXPECT_EQ(replayer->Poll(late).status, Status::kEndOfStream);

  auto second = Make(data, Timed());
  ASSERT_NE(second, nullptr);
  ASSERT_EQ(second->Start(kStart), Status::kOk);
  ASSERT_EQ(second->Poll(late).status, Status::kOk);
  // A now before the previous now, even before the start, is not an error.
  const EpisodeReplayResult early = second->Poll(Add(kStart, -50 * kSecond));
  ASSERT_EQ(early.status, Status::kNotDue);
  EXPECT_EQ(P(early.next_due), P(Add(kStart, kSecond)));
  EXPECT_EQ(second->Stats().emitted, 1u);
}

TEST(EpisodeReplayTest, RC2NextDueIsNormalizedAcrossASecondBoundary) {
  const std::string data = Timeline({0, 1, 1'500'000'000});
  auto replayer = Make(data, Timed());
  ASSERT_NE(replayer, nullptr);
  const ClockReading start = {-1, 999'999'999};
  ASSERT_EQ(replayer->Start(start), Status::kOk);
  ASSERT_EQ(replayer->Poll(start).status, Status::kOk);
  EpisodeReplayResult result = replayer->Poll(start);
  ASSERT_EQ(result.status, Status::kNotDue);
  EXPECT_EQ(P(result.next_due), P(ClockReading{0, 0}));
  ASSERT_EQ(replayer->Poll({0, 0}).status, Status::kOk);
  result = replayer->Poll({0, 0});
  ASSERT_EQ(result.status, Status::kNotDue);
  EXPECT_EQ(P(result.next_due), P(ClockReading{1, 499'999'999}));
}

// RC3.
struct SpeedCase {
  uint32_t num;
  uint32_t den;
  std::vector<int64_t> nominal_due;
};

TEST(EpisodeReplayTest, RC3SpeedRatiosKeepTheOrderAndScaleTheOffsets) {
  const std::vector<SpeedCase> cases = {
      {10, 1, {0, 100 * kMs, 200 * kMs, 300 * kMs, 400 * kMs}},
      {1000, 1, {0, 1 * kMs, 2 * kMs, 3 * kMs, 4 * kMs}},
      {1, 2, {0, 2 * kSecond, 4 * kSecond, 6 * kSecond, 8 * kSecond}},
      {1000, 1000, {0, 1 * kSecond, 2 * kSecond, 3 * kSecond, 4 * kSecond}},
  };
  const std::string data = Nominal();
  for (const SpeedCase& c : cases) {
    SCOPED_TRACE(std::to_string(c.num) + "/" + std::to_string(c.den));
    auto replayer = Make(data, Timed(c.num, c.den));
    ASSERT_NE(replayer, nullptr);
    EXPECT_EQ(DrainTimed(replayer.get(), kStart),
              Rows({0, 1, 2, 3, 4},
                   {0, 1 * kSecond, 2 * kSecond, 3 * kSecond, 4 * kSecond},
                   c.nominal_due));
  }
}

TEST(EpisodeReplayTest, RC3ScaledOffsetsTruncateTowardZero) {
  // Recorded offsets in ns.
  const std::string data = Timeline({0, 1, 2, 3, 4, 5, 7, 10});
  auto three = Make(data, StepAt(3, 1));
  ASSERT_NE(three, nullptr);
  const std::vector<Row> rows = DrainStep(three.get());
  const std::vector<int64_t> expected_three = {0, 0, 0, 1, 1, 1, 2, 3};
  ASSERT_EQ(rows.size(), expected_three.size());
  for (size_t i = 0; i < rows.size(); ++i) {
    EXPECT_EQ(rows[i].due_offset_ns, expected_three[i]) << i;
  }
  // 2/3 scales by 3/2.
  const std::string second = Timeline({0, 1, 3, 5});
  auto replayer = Make(second, StepAt(2, 3));
  ASSERT_NE(replayer, nullptr);
  const std::vector<Row> second_rows = DrainStep(replayer.get());
  ASSERT_EQ(second_rows.size(), 4u);
  EXPECT_EQ(second_rows[1].due_offset_ns, 1);
  EXPECT_EQ(second_rows[2].due_offset_ns, 4);
  EXPECT_EQ(second_rows[3].due_offset_ns, 7);
}

TEST(EpisodeReplayTest, RC3TimedAndStepReportTheSameRows) {
  const std::string data = Nominal();
  auto step = Make(data, StepAt(1000, 7));
  auto timed = Make(data, Timed(1000, 7));
  ASSERT_NE(step, nullptr);
  ASSERT_NE(timed, nullptr);
  EXPECT_EQ(DrainStep(step.get()), DrainTimed(timed.get(), kStart));
}

// RC4.
TEST(EpisodeReplayTest, RC4OverflowGoldenEmitsTheLossRecordInFileOrder) {
  const std::string data = Overflow();
  auto replayer = Make(data, Step());
  ASSERT_NE(replayer, nullptr);
  EXPECT_EQ(replayer->Stats().total, 4u);

  std::vector<EpisodeReplayEvent> events;
  for (int i = 0; i < 4; ++i) {
    const EpisodeReplayResult result = replayer->Step();
    ASSERT_EQ(result.status, Status::kOk);
    events.push_back(result.event);
  }
  EXPECT_EQ(replayer->Step().status, Status::kEndOfStream);

  EXPECT_EQ(RowOf(events[0]), (Row{0, 0, 0, 0}));
  EXPECT_EQ(RowOf(events[1]), (Row{1, 1, 1 * kSecond, 1 * kSecond}));
  EXPECT_EQ(RowOf(events[2]), (Row{2, 2, 2 * kSecond, 2 * kSecond}));
  // The loss record takes the last dropped event's time (index 4, +4 s).
  EXPECT_EQ(RowOf(events[3]), (Row{3, 5, 4 * kSecond, 4 * kSecond}));
  for (int i = 0; i < 3; ++i) {
    EXPECT_EQ(events[i].loss, nullptr);
    EXPECT_NE(events[i].record, nullptr);
  }
  ASSERT_NE(events[3].loss, nullptr);
  ASSERT_NE(events[3].record, nullptr);
  EXPECT_EQ(events[3].record->payload_kind, PayloadKind::kEpisodeLoss);
  EXPECT_EQ(events[3].record->writer_index, 5u);
  EXPECT_EQ(events[3].loss->writer_index, 5u);
  EXPECT_EQ(events[3].loss->first_dropped_writer_index, 3u);
  EXPECT_EQ(events[3].loss->last_dropped_writer_index, 4u);
  EXPECT_EQ(events[3].loss->dropped_count, 2u);
  EXPECT_EQ(events[3].loss->reason, EpisodeLoss::LOSS_REASON_OVERFLOW);
  EXPECT_EQ(replayer->Stats().loss_emitted, 1u);
  EXPECT_EQ(replayer->Stats().emitted, 4u);
}

TEST(EpisodeReplayTest, RC4OverflowGoldenInTimedModes) {
  const std::string data = Overflow();
  const std::vector<uint64_t> indices = {0, 1, 2, 5};
  const std::vector<int64_t> recorded = {0, 1 * kSecond, 2 * kSecond,
                                         4 * kSecond};
  struct Case {
    uint32_t num, den;
    std::vector<int64_t> due;
  };
  const std::vector<Case> cases = {
      {1, 1, {0, 1 * kSecond, 2 * kSecond, 4 * kSecond}},
      {10, 1, {0, 100 * kMs, 200 * kMs, 400 * kMs}},
      {1000, 1, {0, 1 * kMs, 2 * kMs, 4 * kMs}},
      {1, 2, {0, 2 * kSecond, 4 * kSecond, 8 * kSecond}},
  };
  for (const Case& c : cases) {
    SCOPED_TRACE(std::to_string(c.num) + "/" + std::to_string(c.den));
    auto replayer = Make(data, Timed(c.num, c.den));
    ASSERT_NE(replayer, nullptr);
    EXPECT_EQ(DrainTimed(replayer.get(), kStart),
              Rows(indices, recorded, c.due));
  }
}

// RC5.
TEST(EpisodeReplayTest, RC5ALossRecordIsEmittedEvenWhenTheFilterExcludesIt) {
  const std::string data = Overflow();
  const EpisodeReadFilter filter =
      KindFilter({PayloadKind::kSafetyDecision, PayloadKind::kVehicleTrajectory,
                  PayloadKind::kInferenceEnvelope});
  auto replayer = Make(data, Step(), filter);
  ASSERT_NE(replayer, nullptr);
  std::vector<EpisodeReplayEvent> events;
  for (int i = 0; i < 4; ++i) {
    const EpisodeReplayResult result = replayer->Step();
    ASSERT_EQ(result.status, Status::kOk);
    events.push_back(result.event);
  }
  EXPECT_EQ(replayer->Step().status, Status::kEndOfStream);
  EXPECT_EQ(RowOf(events[3]), (Row{3, 5, 4 * kSecond, 4 * kSecond}));
  // The filtered read does not contain the loss record, the unfiltered read
  // supplies it.
  EXPECT_EQ(events[3].record, nullptr);
  ASSERT_NE(events[3].loss, nullptr);
  EXPECT_EQ(events[3].loss->first_dropped_writer_index, 3u);
  EXPECT_EQ(events[3].loss->last_dropped_writer_index, 4u);
  EXPECT_EQ(replayer->Stats().total, 4u);
  EXPECT_EQ(replayer->Stats().loss_emitted, 1u);
}

TEST(EpisodeReplayTest, RC5TheFilterOnlyDecidesTheNonLossRecords) {
  const std::string data = Overflow();
  {
    EpisodeReadFilter filter;
    filter.has_index_range = true;
    filter.first_writer_index = 1;
    filter.last_writer_index = 1;
    auto replayer = Make(data, Step(), filter);
    ASSERT_NE(replayer, nullptr);
    // The offsets are from the first emitted event, which is now index 1.
    EXPECT_EQ(DrainStep(replayer.get()),
              Rows({1, 5}, {0, 3 * kSecond}, {0, 3 * kSecond}));
  }
  {
    EpisodeReadFilter filter;
    filter.source_ids = {"nobody"};
    auto replayer = Make(data, Step(), filter);
    ASSERT_NE(replayer, nullptr);
    const EpisodeReplayResult result = replayer->Step();
    ASSERT_EQ(result.status, Status::kOk);
    EXPECT_EQ(RowOf(result.event), (Row{0, 5, 0, 0}));
    EXPECT_NE(result.event.loss, nullptr);
    EXPECT_EQ(replayer->Step().status, Status::kEndOfStream);
  }
  {
    // Only the loss kind: the loss record is in both reads.
    auto replayer = Make(data, Step(), KindFilter({PayloadKind::kEpisodeLoss}));
    ASSERT_NE(replayer, nullptr);
    const EpisodeReplayResult result = replayer->Step();
    ASSERT_EQ(result.status, Status::kOk);
    EXPECT_NE(result.event.record, nullptr);
    EXPECT_NE(result.event.loss, nullptr);
    EXPECT_EQ(replayer->Step().status, Status::kEndOfStream);
  }
}

TEST(EpisodeReplayTest, RC5AnUnknownPayloadRecordIsSkippedAndCounted) {
  const std::string data = Episode({
      MakeEvent("a", 1, 0),
      MakeUnknown("a", 2, 100 * kSecond, "monotonic"),
      MakeEvent("a", 3, 2 * kSecond),
  });
  auto replayer = Make(data, Step());
  ASSERT_NE(replayer, nullptr);
  EXPECT_EQ(replayer->Stats().total, 2u);
  EXPECT_EQ(replayer->Stats().skipped_unknown, 1u);
  // The skipped record takes no timeline slot and no ordinal.
  EXPECT_EQ(DrainStep(replayer.get()),
            Rows({0, 2}, {0, 2 * kSecond}, {0, 2 * kSecond}));
  EXPECT_EQ(replayer->Stats().emitted, 2u);
  EXPECT_EQ(replayer->Stats().skipped_unknown, 1u);

  auto timed = Make(data, Timed());
  ASSERT_NE(timed, nullptr);
  EXPECT_EQ(DrainTimed(timed.get(), kStart),
            Rows({0, 2}, {0, 2 * kSecond}, {0, 2 * kSecond}));
}

TEST(EpisodeReplayTest, RC5ALeadingUnknownPayloadDoesNotMoveTheTimeOrigin) {
  const std::string data = Episode({
      MakeUnknown("a", 1, 50 * kSecond, "monotonic"),
      MakeEvent("a", 2, 60 * kSecond),
      MakeEvent("a", 3, 61 * kSecond),
  });
  auto replayer = Make(data, Step());
  ASSERT_NE(replayer, nullptr);
  EXPECT_EQ(DrainStep(replayer.get()),
            Rows({1, 2}, {0, 1 * kSecond}, {0, 1 * kSecond}));
}

TEST(EpisodeReplayTest, RC5CountsOnlyTheUnknownRecordsThatPassTheFilter) {
  const std::string data = Episode({
      MakeEvent("a", 1, 0),
      MakeUnknown("a", 2, 1 * kSecond, "monotonic"),
  });
  auto replayer =
      Make(data, Step(), KindFilter({PayloadKind::kSafetyDecision}));
  ASSERT_NE(replayer, nullptr);
  EXPECT_EQ(replayer->Stats().skipped_unknown, 0u);
  EXPECT_EQ(replayer->Stats().total, 1u);
}

TEST(EpisodeReplayTest, RC5AnUnknownPayloadFixtureReplaysNothing) {
  const std::string data = ReaderFixture("unknown_payload_record");
  auto replayer = Make(data, Step());
  ASSERT_NE(replayer, nullptr);
  EXPECT_EQ(replayer->Stats().total, 0u);
  EXPECT_EQ(replayer->Stats().skipped_unknown, 1u);
  EXPECT_EQ(replayer->Step().status, Status::kEndOfStream);
}

// RC6.
TEST(EpisodeReplayTest,
     RC6ADecreasingReceiveTimeIsDueImmediatelyAfterItsPredecessor) {
  // Receive times: 0, 1000, 500, 500, 2000 ms.
  const std::string data =
      Timeline({0, 1000 * kMs, 500 * kMs, 500 * kMs, 2000 * kMs});
  auto replayer = Make(data, Step());
  ASSERT_NE(replayer, nullptr);
  // One regression: 500 < 1000. The equal 500 is not one.
  EXPECT_EQ(replayer->Stats().time_regressions, 1u);
  const std::vector<Row> rows = DrainStep(replayer.get());
  EXPECT_EQ(rows, Rows({0, 1, 2, 3, 4},
                       {0, 1000 * kMs, 500 * kMs, 500 * kMs, 2000 * kMs},
                       {0, 1000 * kMs, 1000 * kMs, 1000 * kMs, 2000 * kMs}));
  EXPECT_EQ(replayer->Stats().time_regressions, 1u);

  auto timed = Make(data, Timed(10, 1));
  ASSERT_NE(timed, nullptr);
  EXPECT_EQ(
      DrainTimed(timed.get(), kStart),
      Rows({0, 1, 2, 3, 4}, {0, 1000 * kMs, 500 * kMs, 500 * kMs, 2000 * kMs},
           {0, 100 * kMs, 100 * kMs, 100 * kMs, 200 * kMs}));
}

TEST(EpisodeReplayTest,
     RC6ARegressionBelowTheFirstEventHasANegativeRecordedOffset) {
  const std::string data = Timeline({1000 * kMs, 500 * kMs, 2000 * kMs});
  auto replayer = Make(data, Step());
  ASSERT_NE(replayer, nullptr);
  EXPECT_EQ(replayer->Stats().time_regressions, 1u);
  EXPECT_EQ(DrainStep(replayer.get()),
            Rows({0, 1, 2}, {0, -500 * kMs, 1000 * kMs}, {0, 0, 1000 * kMs}));
}

TEST(EpisodeReplayTest,
     RC6EveryStrictRegressionIsCountedAgainstItsPredecessor) {
  // 0, 3, 1, 2, 1 s: 1 < 3 and 1 < 2 are regressions. 2 > 1 is not.
  const std::string data =
      Timeline({0, 3 * kSecond, 1 * kSecond, 2 * kSecond, 1 * kSecond});
  auto replayer = Make(data, Step());
  ASSERT_NE(replayer, nullptr);
  EXPECT_EQ(replayer->Stats().time_regressions, 2u);
  const std::vector<Row> rows = DrainStep(replayer.get());
  ASSERT_EQ(rows.size(), 5u);
  for (size_t i = 1; i < rows.size(); ++i) {
    EXPECT_GE(rows[i].due_offset_ns, rows[i - 1].due_offset_ns);
  }
  EXPECT_EQ(rows[4].due_offset_ns, 3 * kSecond);
}

TEST(EpisodeReplayTest, RC6RecordedBytesAreNeverModified) {
  const std::vector<EpisodeEvent> events = {MakeEvent("a", 1, 2 * kSecond),
                                            MakeEvent("a", 2, 1 * kSecond)};
  const std::string data = Episode(events);
  const std::string copy = data;
  auto replayer = Make(data, Step());
  ASSERT_NE(replayer, nullptr);
  for (size_t i = 0; i < events.size(); ++i) {
    const EpisodeReplayResult result = replayer->Step();
    ASSERT_EQ(result.status, Status::kOk);
    ASSERT_NE(result.event.record, nullptr);
    const auto bytes = result.event.record->event_bytes;
    EXPECT_EQ(
        std::string(reinterpret_cast<const char*>(bytes.data()), bytes.size()),
        events[i].SerializeAsString());
  }
  EXPECT_EQ(data, copy);
}

// RC7.
TEST(EpisodeReplayTest, RC7AMixedClockDomainIsRejected) {
  const std::string data = Episode(
      {MakeEvent("a", 1, 0, "monotonic"), MakeEvent("b", 1, kSecond, "utc")});
  EXPECT_EQ(Reject(data, Step()), Status::kMixedClockDomain);
  EXPECT_EQ(Reject(data, Timed()), Status::kMixedClockDomain);
  // Compared exactly.
  EXPECT_EQ(Reject(Episode({MakeEvent("a", 1, 0, "utc"),
                            MakeEvent("b", 1, kSecond, "UTC")}),
                   Step()),
            Status::kMixedClockDomain);
}

TEST(EpisodeReplayTest, RC7AnEmptyDomainOnANonLossRecordIsRejected) {
  EXPECT_EQ(Reject(Episode({MakeEvent("a", 1, 0, "")}), Step()),
            Status::kMixedClockDomain);
  EXPECT_EQ(Reject(Episode({MakeEvent("a", 1, 0, "monotonic"),
                            MakeEvent("a", 2, kSecond, "")}),
                   Step()),
            Status::kMixedClockDomain);
}

TEST(EpisodeReplayTest, RC7OneNonEmptyDomainOfAnyValueIsAccepted) {
  for (const char* domain : {"utc", "monotonic", "some_other_clock"}) {
    SCOPED_TRACE(domain);
    const std::string data = Timeline({0, kSecond}, domain);
    auto replayer = Make(data, Step());
    ASSERT_NE(replayer, nullptr);
    EXPECT_EQ(DrainStep(replayer.get()).size(), 2u);
  }
}

TEST(EpisodeReplayTest, RC7ALossRecordWithAnEmptyDomainIsAccepted) {
  // Index 0, a loss of 1 and 2, then index 4.
  const std::string data = Episode({MakeEvent("a", 1, 0)}) +
                           Frame(3, MakeLoss(1, 1, 2, 3 * kSecond)) +
                           Frame(4, MakeEvent("a", 2, 4 * kSecond));
  auto replayer = Make(data, Step());
  ASSERT_NE(replayer, nullptr);
  EXPECT_EQ(DrainStep(replayer.get()),
            Rows({0, 3, 4}, {0, 3 * kSecond, 4 * kSecond},
                 {0, 3 * kSecond, 4 * kSecond}));
  EXPECT_EQ(replayer->Stats().loss_emitted, 1u);

  // A stream of only a loss record has no domain.
  const std::string only_loss = FileHeader() + Frame(2, MakeLoss(1, 0, 1, 0));
  auto loss_only = Make(only_loss, Step());
  ASSERT_NE(loss_only, nullptr);
  const EpisodeReplayResult result = loss_only->Step();
  ASSERT_EQ(result.status, Status::kOk);
  EXPECT_EQ(RowOf(result.event), (Row{0, 2, 0, 0}));
  EXPECT_NE(result.event.loss, nullptr);
  EXPECT_EQ(loss_only->Step().status, Status::kEndOfStream);
}

TEST(EpisodeReplayTest, RC7OnlyEmittedRecordsAreCheckedForTheirDomain) {
  // The unknown payload is not emitted, so its domain does not count.
  const std::string with_unknown = Episode({
      MakeEvent("a", 1, 0, "monotonic"),
      MakeUnknown("a", 2, kSecond, "utc"),
      MakeEvent("a", 3, 2 * kSecond, "monotonic"),
  });
  auto replayer = Make(with_unknown, Step());
  ASSERT_NE(replayer, nullptr);
  EXPECT_EQ(DrainStep(replayer.get()).size(), 2u);

  // A record that the filter excludes does not count either.
  const std::string two_sources = Episode(
      {MakeEvent("a", 1, 0, "monotonic"), MakeEvent("b", 1, kSecond, "utc")});
  EpisodeReadFilter filter;
  filter.source_ids = {"a"};
  auto filtered = Make(two_sources, Step(), filter);
  ASSERT_NE(filtered, nullptr);
  EXPECT_EQ(DrainStep(filtered.get()).size(), 1u);
}

// RC8.
struct DefectCase {
  std::string name;
  std::string data;
  Defect defect;
  std::vector<uint64_t> prefix;
};

std::vector<DefectCase> DefectCases() {
  const std::string nominal = Nominal();
  return {
      {"truncated_last_record",
       nominal.substr(0, nominal.size() - 3),
       Defect::kTruncatedRecord,
       {0, 1, 2, 3}},
      {"index_gap",
       ReaderFixture("index_gap_uncovered"),
       Defect::kIndexGap,
       {0, 1}},
      {"bad_magic", ReaderFixture("bad_magic"), Defect::kBadMagic, {}},
      {"empty_buffer", "", Defect::kTruncatedHeader, {}},
  };
}

TEST(EpisodeReplayTest, RC8ADefectiveStreamIsRejectedByDefault) {
  for (const DefectCase& c : DefectCases()) {
    SCOPED_TRACE(c.name);
    ASSERT_TRUE(c.name == "empty_buffer" || !c.data.empty());
    EXPECT_EQ(Reject(c.data, Step()), Status::kDefectiveStream);
    EXPECT_EQ(Reject(c.data, Timed()), Status::kDefectiveStream);
  }
}

TEST(EpisodeReplayTest, RC8TheOptInReplaysTheTrustedPrefixAndEchoesTheDefect) {
  for (const DefectCase& c : DefectCases()) {
    SCOPED_TRACE(c.name);
    auto replayer = Make(c.data, WithPrefix(Step()));
    ASSERT_NE(replayer, nullptr);
    EXPECT_EQ(replayer->Stats().total, c.prefix.size());
    EpisodeReplayResult end;
    const std::vector<Row> rows = DrainStep(replayer.get(), &end);
    ASSERT_EQ(rows.size(), c.prefix.size());
    for (size_t i = 0; i < rows.size(); ++i) {
      EXPECT_EQ(rows[i].writer_index, c.prefix[i]);
      EXPECT_EQ(rows[i].ordinal, i);
    }
    EXPECT_EQ(end.status, Status::kEndOfTrustedPrefix);
    EXPECT_EQ(end.reader_defect, c.defect);
    // Sticky.
    for (int i = 0; i < 2; ++i) {
      const EpisodeReplayResult again = replayer->Step();
      EXPECT_EQ(again.status, Status::kEndOfTrustedPrefix);
      EXPECT_EQ(again.reader_defect, c.defect);
    }
  }
}

TEST(EpisodeReplayTest, RC8TheOptInInTimedMode) {
  const DefectCase c = DefectCases()[0];
  auto replayer = Make(c.data, WithPrefix(Timed()));
  ASSERT_NE(replayer, nullptr);
  EpisodeReplayResult end;
  const std::vector<Row> rows = DrainTimed(replayer.get(), kStart, &end);
  EXPECT_EQ(rows.size(), 4u);
  EXPECT_EQ(end.status, Status::kEndOfTrustedPrefix);
  EXPECT_EQ(end.reader_defect, Defect::kTruncatedRecord);
  EXPECT_EQ(replayer->Poll(kStart).status, Status::kEndOfTrustedPrefix);
}

TEST(EpisodeReplayTest, RC8ACleanFileEndsWithEndOfStreamEvenWithTheOptIn) {
  const std::string data = Nominal();
  auto replayer = Make(data, WithPrefix(Step()));
  ASSERT_NE(replayer, nullptr);
  EpisodeReplayResult end;
  EXPECT_EQ(DrainStep(replayer.get(), &end).size(), 5u);
  EXPECT_EQ(end.status, Status::kEndOfStream);
  EXPECT_EQ(end.reader_defect, Defect::kNone);
}

TEST(EpisodeReplayTest, RC8TheDefectComesFromTheFilteredRead) {
  const std::string nominal = Nominal();
  const std::string data = nominal.substr(0, nominal.size() - 3);
  EpisodeReadFilter filter;
  filter.source_ids = {"nobody"};
  EXPECT_EQ(Reject(data, Step(), filter), Status::kDefectiveStream);
  auto replayer = Make(data, WithPrefix(Step()), filter);
  ASSERT_NE(replayer, nullptr);
  EpisodeReplayResult end;
  EXPECT_TRUE(DrainStep(replayer.get(), &end).empty());
  EXPECT_EQ(end.status, Status::kEndOfTrustedPrefix);
  EXPECT_EQ(end.reader_defect, Defect::kTruncatedRecord);
}

// RC9.
TEST(EpisodeReplayTest, RC9CancelMidStreamIsIdempotent) {
  const std::string data = Nominal();
  auto replayer = Make(data, Step());
  ASSERT_NE(replayer, nullptr);
  ASSERT_EQ(replayer->Step().status, Status::kOk);
  ASSERT_EQ(replayer->Step().status, Status::kOk);
  EXPECT_EQ(replayer->Cancel(), 3u);
  EXPECT_EQ(replayer->Stats().cancelled_remaining, 3u);
  EXPECT_EQ(replayer->Cancel(), 0u);
  EXPECT_EQ(replayer->Cancel(), 0u);
  EXPECT_EQ(replayer->Stats().cancelled_remaining, 3u);
  EXPECT_EQ(replayer->Step().status, Status::kCancelled);
  EXPECT_EQ(replayer->Step().status, Status::kCancelled);
  EXPECT_EQ(replayer->Poll(kStart).status, Status::kCancelled);
  EXPECT_EQ(replayer->Start(kStart), Status::kCancelled);
  EXPECT_EQ(replayer->Stats().emitted, 2u);
  EXPECT_EQ(replayer->Stats().total, 5u);
}

TEST(EpisodeReplayTest, RC9CancelBeforeTheFirstCallAndAfterTheEnd) {
  const std::string data = Nominal();
  auto fresh = Make(data, Step());
  ASSERT_NE(fresh, nullptr);
  EXPECT_EQ(fresh->Cancel(), 5u);
  EXPECT_EQ(fresh->Step().status, Status::kCancelled);

  auto drained = Make(data, Step());
  ASSERT_NE(drained, nullptr);
  EXPECT_EQ(DrainStep(drained.get()).size(), 5u);
  EXPECT_EQ(drained->Cancel(), 0u);
  EXPECT_EQ(drained->Stats().cancelled_remaining, 0u);
  // Cancelled wins over the sticky end of stream.
  EXPECT_EQ(drained->Step().status, Status::kCancelled);
}

TEST(EpisodeReplayTest, RC9CancelInTimedMode) {
  const std::string data = Nominal();
  auto before_start = Make(data, Timed());
  ASSERT_NE(before_start, nullptr);
  EXPECT_EQ(before_start->Cancel(), 5u);
  EXPECT_EQ(before_start->Start(kStart), Status::kCancelled);
  EXPECT_EQ(before_start->Poll(kStart).status, Status::kCancelled);
  // Cancelled wins over every other check, including an invalid time.
  EXPECT_EQ(before_start->Poll({0, -1}).status, Status::kCancelled);
  EXPECT_EQ(before_start->Start({0, -1}), Status::kCancelled);

  auto running = Make(data, Timed());
  ASSERT_NE(running, nullptr);
  ASSERT_EQ(running->Start(kStart), Status::kOk);
  ASSERT_EQ(running->Poll(kStart).status, Status::kOk);
  EXPECT_EQ(running->Cancel(), 4u);
  EXPECT_EQ(running->Cancel(), 0u);
  EXPECT_EQ(running->Poll(Add(kStart, 100 * kSecond)).status,
            Status::kCancelled);
  EXPECT_EQ(running->Stats().cancelled_remaining, 4u);
  EXPECT_EQ(running->Step().status, Status::kCancelled);
}

TEST(EpisodeReplayTest, RC9WrongModeNotStartedAndInvalidTimeCalls) {
  const std::string data = Nominal();
  auto step = Make(data, Step());
  ASSERT_NE(step, nullptr);
  EXPECT_EQ(step->Poll(kStart).status, Status::kWrongMode);
  EXPECT_EQ(step->Start(kStart), Status::kWrongMode);
  // The wrong-mode calls did not consume anything.
  EXPECT_EQ(step->Stats().emitted, 0u);
  EXPECT_EQ(step->Step().status, Status::kOk);

  auto timed = Make(data, Timed());
  ASSERT_NE(timed, nullptr);
  EXPECT_EQ(timed->Step().status, Status::kWrongMode);
  EXPECT_EQ(timed->Poll(kStart).status, Status::kNotStarted);
  // Not started comes before an invalid time.
  EXPECT_EQ(timed->Poll({0, -1}).status, Status::kNotStarted);

  EXPECT_EQ(timed->Start({0, -1}), Status::kInvalidTime);
  EXPECT_EQ(timed->Start({0, embodiment::kNanosPerSecond}),
            Status::kInvalidTime);
  EXPECT_EQ(timed->Start({0, std::numeric_limits<int32_t>::max()}),
            Status::kInvalidTime);
  EXPECT_EQ(timed->Start({0, std::numeric_limits<int32_t>::min()}),
            Status::kInvalidTime);
  // A failed Start does not start.
  EXPECT_EQ(timed->Poll(kStart).status, Status::kNotStarted);

  EXPECT_EQ(timed->Start(kStart), Status::kOk);
  EXPECT_EQ(timed->Start(kStart), Status::kWrongMode);
  EXPECT_EQ(timed->Start({0, -1}), Status::kWrongMode);
  EXPECT_EQ(timed->Step().status, Status::kWrongMode);
  EXPECT_EQ(timed->Poll({0, -1}).status, Status::kInvalidTime);
  EXPECT_EQ(timed->Poll({0, embodiment::kNanosPerSecond}).status,
            Status::kInvalidTime);
  EXPECT_EQ(timed->Stats().emitted, 0u);
  EXPECT_EQ(timed->Poll(kStart).status, Status::kOk);
}

TEST(EpisodeReplayTest, RC9InvalidTimeComesBeforeTheEndOfStream) {
  const std::string data = ReaderFixture("header_only");
  auto replayer = Make(data, Timed());
  ASSERT_NE(replayer, nullptr);
  ASSERT_EQ(replayer->Start(kStart), Status::kOk);
  EXPECT_EQ(replayer->Poll({0, -1}).status, Status::kInvalidTime);
  EXPECT_EQ(replayer->Poll(kStart).status, Status::kEndOfStream);
}

// RC10.
TEST(EpisodeReplayTest, RC10SpeedOutOfRangeIsRejectedNotClamped) {
  const std::string data = Nominal();
  for (const EpisodeReplayMode mode :
       {EpisodeReplayMode::kTimed, EpisodeReplayMode::kStep}) {
    for (const uint32_t bad : {0u, 1001u, 4294967295u}) {
      SCOPED_TRACE(bad);
      EpisodeReplayConfig config;
      config.mode = mode;
      config.speed_num = bad;
      EXPECT_EQ(Reject(data, config), Status::kInvalidConfig);
      config.speed_num = 1;
      config.speed_den = bad;
      EXPECT_EQ(Reject(data, config), Status::kInvalidConfig);
    }
  }
}

TEST(EpisodeReplayTest, RC10SpeedBoundsAreAccepted) {
  const std::string data = Nominal();
  for (const uint32_t num : {1u, 1000u}) {
    for (const uint32_t den : {1u, 1000u}) {
      SCOPED_TRACE(std::to_string(num) + "/" + std::to_string(den));
      EXPECT_NE(Make(data, Timed(num, den)), nullptr);
    }
  }
}

TEST(EpisodeReplayTest, RC10AnUnknownModeIsRejected) {
  EpisodeReplayConfig config;
  config.mode = static_cast<EpisodeReplayMode>(2);
  EXPECT_EQ(Reject(Nominal(), config), Status::kInvalidConfig);
}

TEST(EpisodeReplayTest, RC10AnInvalidReaderConfigIsRejected) {
  const std::string data = Nominal();
  EpisodeReaderConfig zero_event;
  zero_event.max_event_bytes = 0;
  EXPECT_EQ(Reject(data, Step(), {}, zero_event), Status::kInvalidConfig);
  EpisodeReaderConfig zero_records;
  zero_records.max_records = 0;
  EXPECT_EQ(Reject(data, Step(), {}, zero_records), Status::kInvalidConfig);
  EpisodeReaderConfig small_file;
  small_file.max_file_bytes = 11;
  EXPECT_EQ(Reject(data, Step(), {}, small_file), Status::kInvalidConfig);
  // An invalid filter range is a reader kInvalidConfig.
  EpisodeReadFilter backwards;
  backwards.has_index_range = true;
  backwards.first_writer_index = 3;
  backwards.last_writer_index = 2;
  EXPECT_EQ(Reject(data, Step(), backwards), Status::kInvalidConfig);
}

TEST(EpisodeReplayTest, RC10AReaderStatusThatIsNotOkIsAlwaysRejected) {
  const std::string data = Nominal();
  EpisodeReaderConfig too_small;
  too_small.max_file_bytes = 100;
  EXPECT_EQ(Reject(data, Step(), {}, too_small), Status::kInvalidConfig);
  EXPECT_EQ(Reject(data, WithPrefix(Step()), {}, too_small),
            Status::kInvalidConfig);
  EpisodeReaderConfig zero;
  zero.max_event_bytes = 0;
  EXPECT_EQ(Reject(data, WithPrefix(Timed()), {}, zero),
            Status::kInvalidConfig);
}

TEST(EpisodeReplayTest, RC10AHeaderOnlyStreamEndsOnTheFirstCall) {
  const std::string data = ReaderFixture("header_only");
  auto step = Make(data, Step());
  ASSERT_NE(step, nullptr);
  EXPECT_EQ(step->Stats().total, 0u);
  EXPECT_EQ(step->Step().status, Status::kEndOfStream);
  EXPECT_EQ(step->Step().status, Status::kEndOfStream);

  auto timed = Make(data, Timed());
  ASSERT_NE(timed, nullptr);
  ASSERT_EQ(timed->Start({kInt64Max, 999'999'999}), Status::kOk);
  EXPECT_EQ(timed->Poll(kStart).status, Status::kEndOfStream);
}

TEST(EpisodeReplayTest, RC10EverythingFilteredOutIsAnEmptyStream) {
  EpisodeReadFilter filter;
  filter.source_ids = {"nobody"};
  auto replayer = Make(Nominal(), Step(), filter);
  ASSERT_NE(replayer, nullptr);
  EXPECT_EQ(replayer->Stats().total, 0u);
  EXPECT_EQ(replayer->Step().status, Status::kEndOfStream);
}

TEST(EpisodeReplayTest, RC10ARecordedOffsetOutsideInt64IsRejectedAtCreate) {
  // 1e10 s is 1e19 ns, above INT64_MAX ns.
  EXPECT_EQ(Reject(Episode({MakeEventAtSeconds("a", 1, 0, 0),
                            MakeEventAtSeconds("a", 2, 10'000'000'000, 0)}),
                   Step()),
            Status::kTimeOutOfRange);
  EXPECT_EQ(Reject(Episode({MakeEventAtSeconds("a", 1, 10'000'000'000, 0),
                            MakeEventAtSeconds("a", 2, 0, 0)}),
                   Timed()),
            Status::kTimeOutOfRange);
}

TEST(EpisodeReplayTest, RC10TheInt64NanosecondBoundaryIsExact) {
  // INT64_MAX ns is 9223372036.854775807 s.
  auto at_max =
      Make(Episode({MakeEventAtSeconds("a", 1, 0, 0),
                    MakeEventAtSeconds("a", 2, 9'223'372'036, 854'775'807)}),
           Step());
  ASSERT_NE(at_max, nullptr);
  const std::vector<Row> rows = DrainStep(at_max.get());
  ASSERT_EQ(rows.size(), 2u);
  EXPECT_EQ(rows[1].recorded_offset_ns, kInt64Max);
  EXPECT_EQ(rows[1].due_offset_ns, kInt64Max);

  EXPECT_EQ(
      Reject(Episode({MakeEventAtSeconds("a", 1, 0, 0),
                      MakeEventAtSeconds("a", 2, 9'223'372'036, 854'775'808)}),
             Step()),
      Status::kTimeOutOfRange);

  // INT64_MIN ns is -9223372036.854775808 s.
  auto at_min =
      Make(Episode({MakeEventAtSeconds("a", 1, 9'223'372'036, 854'775'808),
                    MakeEventAtSeconds("a", 2, 0, 0)}),
           Step());
  ASSERT_NE(at_min, nullptr);
  const std::vector<Row> min_rows = DrainStep(at_min.get());
  ASSERT_EQ(min_rows.size(), 2u);
  EXPECT_EQ(min_rows[1].recorded_offset_ns,
            std::numeric_limits<int64_t>::min());
  EXPECT_EQ(min_rows[1].due_offset_ns, 0);
  EXPECT_EQ(
      Reject(Episode({MakeEventAtSeconds("a", 1, 9'223'372'036, 854'775'809),
                      MakeEventAtSeconds("a", 2, 0, 0)}),
             Step()),
      Status::kTimeOutOfRange);
}

TEST(EpisodeReplayTest, RC10ADueOffsetOutsideInt64AfterScalingIsRejected) {
  // 1e16 ns scaled by 1000 is 1e19 ns.
  const std::string data = Timeline({0, 10'000'000'000'000'000});
  EXPECT_NE(Make(data, Timed(1, 1)), nullptr);
  EXPECT_EQ(Reject(data, Timed(1, 1000)), Status::kTimeOutOfRange);
}

TEST(EpisodeReplayTest, RC10AStartThatOverflowsTheClockIsRejected) {
  const std::string data = Timeline({0, 2 * kSecond});
  auto replayer = Make(data, Timed());
  ASSERT_NE(replayer, nullptr);
  EXPECT_EQ(replayer->Start({kInt64Max, 0}), Status::kTimeOutOfRange);
  EXPECT_EQ(replayer->Start({kInt64Max - 1, 999'999'999}),
            Status::kTimeOutOfRange);
  // A rejected Start does not start.
  EXPECT_EQ(replayer->Poll(kStart).status, Status::kNotStarted);
  // The last due offset is 2 s. The largest representable end is
  // {INT64_MAX, 999999999}.
  ASSERT_EQ(replayer->Start({kInt64Max - 2, 999'999'999}), Status::kOk);
  EXPECT_EQ(replayer->Start(kStart), Status::kWrongMode);

  const EpisodeReplayResult first =
      replayer->Poll({kInt64Max - 2, 999'999'999});
  ASSERT_EQ(first.status, Status::kOk);
  const EpisodeReplayResult second =
      replayer->Poll({kInt64Max - 2, 999'999'999});
  ASSERT_EQ(second.status, Status::kNotDue);
  EXPECT_EQ(P(second.next_due), P(ClockReading{kInt64Max, 999'999'999}));

  // A single event never leaves its start.
  auto single = Make(Timeline({0}), Timed());
  ASSERT_NE(single, nullptr);
  EXPECT_EQ(single->Start({kInt64Max, 999'999'999}), Status::kOk);
  // A negative start is fine.
  auto negative = Make(data, Timed());
  ASSERT_NE(negative, nullptr);
  EXPECT_EQ(negative->Start({std::numeric_limits<int64_t>::min(), 0}),
            Status::kOk);
}

// RC11.
TEST(EpisodeReplayTest, RC11TwoReplaysGiveTheSameSequences) {
  const std::string regression =
      Timeline({0, 1000 * kMs, 500 * kMs, 2000 * kMs});
  const std::vector<std::pair<std::string, EpisodeReadFilter>> inputs = {
      {Nominal(), {}},
      {Overflow(), {}},
      {Overflow(), KindFilter({PayloadKind::kSafetyDecision})},
      {regression, {}},
  };
  for (const auto& [data, filter] : inputs) {
    for (const EpisodeReplayConfig& config : {Step(), Timed(), Timed(7, 3)}) {
      auto first = Make(data, config, filter);
      auto second = Make(data, config, filter);
      ASSERT_NE(first, nullptr);
      ASSERT_NE(second, nullptr);
      const bool timed = config.mode == EpisodeReplayMode::kTimed;
      const std::vector<Row> a =
          timed ? DrainTimed(first.get(), kStart) : DrainStep(first.get());
      const std::vector<Row> b =
          timed ? DrainTimed(second.get(), kStart) : DrainStep(second.get());
      EXPECT_FALSE(a.empty());
      EXPECT_EQ(a, b);
      const EpisodeReplayStats x = first->Stats();
      const EpisodeReplayStats y = second->Stats();
      EXPECT_EQ(x.total, y.total);
      EXPECT_EQ(x.emitted, y.emitted);
      EXPECT_EQ(x.loss_emitted, y.loss_emitted);
      EXPECT_EQ(x.skipped_unknown, y.skipped_unknown);
      EXPECT_EQ(x.time_regressions, y.time_regressions);
    }
  }
}

TEST(EpisodeReplayTest, RC11TheReplayDoesNotDependOnTheStartTime) {
  const std::string data = Overflow();
  auto first = Make(data, Timed(3, 2));
  auto second = Make(data, Timed(3, 2));
  ASSERT_NE(first, nullptr);
  ASSERT_NE(second, nullptr);
  EXPECT_EQ(DrainTimed(first.get(), {0, 0}),
            DrainTimed(second.get(), {-5000, 123}));
}

// RC12.
class AllocationGuard {
 public:
  AllocationGuard() {
    g_allocations.store(0);
    g_count_allocations.store(true);
  }
  ~AllocationGuard() { g_count_allocations.store(false); }
  uint64_t count() const { return g_allocations.load(); }
};

// Runs `call` with the counter armed. The result is inspected after the guard
// is disarmed, so a failing assertion cannot count.
template <typename F>
auto Counted(F&& call, uint64_t* allocations) {
  AllocationGuard guard;
  auto result = call();
  g_count_allocations.store(false);
  *allocations = guard.count();
  return result;
}

TEST(EpisodeReplayTest, RC12TheGuardCountsTheThrowingAndNothrowForms) {
  uint64_t allocations = 0;
  Counted(
      [] {
        int* probe = new int(1);
        *static_cast<volatile int*>(probe) = 2;
        delete probe;
        return 0;
      },
      &allocations);
  EXPECT_EQ(allocations, 1u);
  Counted(
      [] {
        int* probe = new (std::nothrow) int(1);
        *static_cast<volatile int*>(probe) = 2;
        delete probe;
        int* array = new (std::nothrow) int[4];
        *static_cast<volatile int*>(array) = 2;
        delete[] array;
        return 0;
      },
      &allocations);
  EXPECT_EQ(allocations, 2u);
  Counted(
      [] {
        int* array = new int[4];
        *static_cast<volatile int*>(array) = 2;
        delete[] array;
        return 0;
      },
      &allocations);
  EXPECT_EQ(allocations, 1u);
}

TEST(EpisodeReplayTest, RC12StepDoesNotAllocate) {
  const std::string data = Overflow();
  auto replayer = Make(data, Step());
  ASSERT_NE(replayer, nullptr);
  EpisodeReplayer* r = replayer.get();
  for (int i = 0; i < 4; ++i) {
    uint64_t allocations = 0;
    const EpisodeReplayResult result =
        Counted([r] { return r->Step(); }, &allocations);
    EXPECT_EQ(result.status, Status::kOk);
    EXPECT_EQ(allocations, 0u) << i;
  }
  for (int i = 0; i < 2; ++i) {
    uint64_t allocations = 0;
    const EpisodeReplayResult result =
        Counted([r] { return r->Step(); }, &allocations);
    EXPECT_EQ(result.status, Status::kEndOfStream);
    EXPECT_EQ(allocations, 0u);
  }
  uint64_t allocations = 0;
  const EpisodeReplayResult wrong =
      Counted([r] { return r->Poll({0, 0}); }, &allocations);
  EXPECT_EQ(wrong.status, Status::kWrongMode);
  EXPECT_EQ(allocations, 0u);
  const EpisodeReplayStats stats =
      Counted([r] { return r->Stats(); }, &allocations);
  EXPECT_EQ(stats.emitted, 4u);
  EXPECT_EQ(allocations, 0u);
}

TEST(EpisodeReplayTest, RC12StartAndPollDoNotAllocate) {
  const std::string data = Overflow();
  auto replayer = Make(data, Timed(1, 2));
  ASSERT_NE(replayer, nullptr);
  EpisodeReplayer* r = replayer.get();
  uint64_t allocations = 0;

  EpisodeReplayResult result =
      Counted([r] { return r->Poll(kStart); }, &allocations);
  EXPECT_EQ(result.status, Status::kNotStarted);
  EXPECT_EQ(allocations, 0u);
  Status status = Counted([r] { return r->Step().status; }, &allocations);
  EXPECT_EQ(status, Status::kWrongMode);
  EXPECT_EQ(allocations, 0u);
  status = Counted([r] { return r->Start({0, -1}); }, &allocations);
  EXPECT_EQ(status, Status::kInvalidTime);
  EXPECT_EQ(allocations, 0u);
  status = Counted([r] { return r->Start({kInt64Max, 0}); }, &allocations);
  EXPECT_EQ(status, Status::kTimeOutOfRange);
  EXPECT_EQ(allocations, 0u);
  status = Counted([r] { return r->Start(kStart); }, &allocations);
  EXPECT_EQ(status, Status::kOk);
  EXPECT_EQ(allocations, 0u);
  status = Counted([r] { return r->Start(kStart); }, &allocations);
  EXPECT_EQ(status, Status::kWrongMode);
  EXPECT_EQ(allocations, 0u);

  result = Counted([r] { return r->Poll({0, -1}); }, &allocations);
  EXPECT_EQ(result.status, Status::kInvalidTime);
  EXPECT_EQ(allocations, 0u);
  result = Counted([r] { return r->Poll(kStart); }, &allocations);
  EXPECT_EQ(result.status, Status::kOk);
  EXPECT_EQ(allocations, 0u);
  result = Counted([r] { return r->Poll(kStart); }, &allocations);
  EXPECT_EQ(result.status, Status::kNotDue);
  EXPECT_EQ(allocations, 0u);
  const ClockReading late = Add(kStart, 100 * kSecond);
  for (int i = 0; i < 3; ++i) {
    result = Counted([r, late] { return r->Poll(late); }, &allocations);
    EXPECT_EQ(result.status, Status::kOk);
    EXPECT_EQ(allocations, 0u);
  }
  result = Counted([r, late] { return r->Poll(late); }, &allocations);
  EXPECT_EQ(result.status, Status::kEndOfStream);
  EXPECT_EQ(allocations, 0u);
}

TEST(EpisodeReplayTest, RC12CancelDoesNotAllocate) {
  const std::string data = Nominal();
  auto replayer = Make(data, Step());
  ASSERT_NE(replayer, nullptr);
  EpisodeReplayer* r = replayer.get();
  ASSERT_EQ(r->Step().status, Status::kOk);
  uint64_t allocations = 0;
  uint64_t remaining = Counted([r] { return r->Cancel(); }, &allocations);
  EXPECT_EQ(remaining, 4u);
  EXPECT_EQ(allocations, 0u);
  remaining = Counted([r] { return r->Cancel(); }, &allocations);
  EXPECT_EQ(remaining, 0u);
  EXPECT_EQ(allocations, 0u);
  const EpisodeReplayResult result =
      Counted([r] { return r->Step(); }, &allocations);
  EXPECT_EQ(result.status, Status::kCancelled);
  EXPECT_EQ(allocations, 0u);

  auto timed = Make(data, Timed());
  ASSERT_NE(timed, nullptr);
  EpisodeReplayer* t = timed.get();
  ASSERT_EQ(t->Start(kStart), Status::kOk);
  const EpisodeReplayResult cancelled_poll = Counted(
      [t] {
        t->Cancel();
        return t->Poll({0, 0});
      },
      &allocations);
  EXPECT_EQ(cancelled_poll.status, Status::kCancelled);
  EXPECT_EQ(allocations, 0u);
}

TEST(EpisodeReplayTest, RC12TheEndOfATrustedPrefixDoesNotAllocate) {
  const std::string nominal = Nominal();
  const std::string data = nominal.substr(0, nominal.size() - 3);
  auto replayer = Make(data, WithPrefix(Step()));
  ASSERT_NE(replayer, nullptr);
  EpisodeReplayer* r = replayer.get();
  for (int i = 0; i < 5; ++i) {
    uint64_t allocations = 0;
    const EpisodeReplayResult result =
        Counted([r] { return r->Step(); }, &allocations);
    EXPECT_EQ(allocations, 0u) << i;
    EXPECT_EQ(result.status, i < 4 ? Status::kOk : Status::kEndOfTrustedPrefix);
  }
}

}  // namespace
}  // namespace intrinsic::episode
