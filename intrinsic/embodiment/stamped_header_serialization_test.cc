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

#include <cstddef>
#include <optional>
#include <string>
#include <string_view>

#include "google/protobuf/unknown_field_set.h"
#include "gtest/gtest.h"
#include "intrinsic/embodiment/frame_policy.h"
#include "intrinsic/embodiment/proto/stamped_header.pb.h"
#include "intrinsic/embodiment/stamped_header_policy.h"

namespace intrinsic::embodiment {
namespace {

using intrinsic_proto::embodiment::StampedHeader;
using intrinsic_proto::embodiment::Validity;

// Canonical serialization of the fixture in FillFixture(). Keep in sync with
// stamped_header_serialization_test.py.
constexpr std::string_view kGoldenHex =
    "082a120b0880e2cfaa061080e59a771a060881e2cfaa062205696d755f302a09776f726c6"
    "45f656e7532096d6f6e6f746f6e69633a020802";

constexpr std::string_view kWithoutValidityHex =
    "082a120b0880e2cfaa061080e59a771a060881e2cfaa062205696d755f302a09776f726c6"
    "45f656e7532096d6f6e6f746f6e6963";

std::string FromHex(std::string_view hex) {
  std::string out;
  out.reserve(hex.size() / 2);
  auto nibble = [](char c) -> int {
    if (c >= '0' && c <= '9') {
      return c - '0';
    }
    return c - 'a' + 10;
  };
  for (size_t i = 0; i < hex.size(); i += 2) {
    out.push_back(
        static_cast<char>((nibble(hex[i]) << 4) | nibble(hex[i + 1])));
  }
  return out;
}

void FillFixture(StampedHeader* header) {
  header->set_sequence(42);
  header->mutable_source_time()->set_seconds(1700000000);
  header->mutable_source_time()->set_nanos(250000000);
  header->mutable_receive_time()->set_seconds(1700000001);
  header->mutable_receive_time()->set_nanos(0);
  header->set_source_id("imu_0");
  header->set_frame_id(std::string(kWorldEnuFrameId));
  header->set_clock_domain(std::string(kClockDomainMonotonic));
  header->mutable_validity()->set_state(Validity::STATE_INVALID);
}

TEST(StampedHeaderSerializationTest, GoldenRoundTrip) {
  StampedHeader header;
  FillFixture(&header);
  const std::string golden = FromHex(kGoldenHex);
  EXPECT_EQ(header.SerializeAsString(), golden);

  StampedHeader parsed;
  ASSERT_TRUE(parsed.ParseFromString(golden));
  EXPECT_EQ(parsed.sequence(), 42u);
  EXPECT_EQ(parsed.source_time().seconds(), 1700000000);
  EXPECT_EQ(parsed.source_time().nanos(), 250000000);
  EXPECT_EQ(parsed.receive_time().seconds(), 1700000001);
  EXPECT_EQ(parsed.receive_time().nanos(), 0);
  EXPECT_EQ(parsed.source_id(), "imu_0");
  EXPECT_EQ(parsed.frame_id(), kWorldEnuFrameId);
  EXPECT_EQ(parsed.clock_domain(), kClockDomainMonotonic);
  EXPECT_TRUE(parsed.has_validity());
  EXPECT_EQ(parsed.validity().state(), Validity::STATE_INVALID);
  EXPECT_EQ(parsed.SerializeAsString(), golden);
}

TEST(StampedHeaderSerializationTest, AbsentValidityIsOmittedAndDistinct) {
  StampedHeader header;
  FillFixture(&header);
  header.clear_validity();
  const std::string without = FromHex(kWithoutValidityHex);
  EXPECT_EQ(header.SerializeAsString(), without);
  EXPECT_TRUE(FromHex(kGoldenHex).starts_with(without));

  StampedHeader parsed;
  ASSERT_TRUE(parsed.ParseFromString(without));
  EXPECT_FALSE(parsed.has_validity());
  EXPECT_EQ(parsed.validity().state(), Validity::STATE_UNSPECIFIED);
  EXPECT_EQ(ClassifyValidity(parsed.has_validity(), parsed.validity().state()),
            ValidityKind::kAbsent);
  EXPECT_EQ(ClassifyValidity(true, static_cast<int>(Validity::STATE_INVALID)),
            ValidityKind::kInvalid);
  EXPECT_FALSE(SampleAccepted(ValidityKind::kAbsent, true));
  EXPECT_FALSE(SampleAccepted(ValidityKind::kInvalid, true));
}

TEST(StampedHeaderSerializationTest, DefaultHeaderIsEmptyAndOptIn) {
  const StampedHeader header;
  EXPECT_EQ(header.SerializeAsString(), "");
  EXPECT_FALSE(header.has_validity());
  EXPECT_FALSE(header.has_source_time());
  EXPECT_FALSE(header.has_receive_time());
  EXPECT_EQ(header.frame_id(), "");
  StampedHeader parsed;
  ASSERT_TRUE(parsed.ParseFromString(""));
  EXPECT_EQ(parsed.SerializeAsString(), "");
}

TEST(StampedHeaderSerializationTest, UnknownFieldIsPreserved) {
  const std::string golden = FromHex(kGoldenHex);
  // Field 100, varint 7. Append-only readers keep this tail.
  const std::string with_unknown = golden + std::string("\xA0\x06\x07", 3);
  StampedHeader parsed;
  ASSERT_TRUE(parsed.ParseFromString(with_unknown));
  EXPECT_EQ(parsed.sequence(), 42u);
  EXPECT_EQ(parsed.frame_id(), "world_enu");
  const google::protobuf::UnknownFieldSet& unknown =
      parsed.GetReflection()->GetUnknownFields(parsed);
  ASSERT_EQ(unknown.field_count(), 1);
  EXPECT_EQ(unknown.field(0).number(), 100);
  EXPECT_EQ(unknown.field(0).varint(), 7u);
  EXPECT_EQ(parsed.SerializeAsString(), with_unknown);
}

TEST(StampedHeaderSerializationTest, MonotonicAgeUsesClockDomain) {
  StampedHeader header;
  FillFixture(&header);
  const std::optional<double> age =
      MonotonicAgeSeconds(ClockReading{header.source_time().seconds(),
                                       header.source_time().nanos()},
                          ClockReading{header.receive_time().seconds(),
                                       header.receive_time().nanos()},
                          header.clock_domain());
  ASSERT_TRUE(age.has_value());
  EXPECT_DOUBLE_EQ(*age, 0.75);

  header.set_clock_domain(std::string(kClockDomainUtc));
  EXPECT_FALSE(MonotonicAgeSeconds(ClockReading{header.source_time().seconds(),
                                                header.source_time().nanos()},
                                   ClockReading{header.receive_time().seconds(),
                                                header.receive_time().nanos()},
                                   header.clock_domain())
                   .has_value());
  EXPECT_TRUE(FrameIdMatches(header.frame_id(), WorldFrame::kEnu));
  EXPECT_FALSE(FrameIdMatches("StampedHeader", WorldFrame::kEnu));
}

}  // namespace
}  // namespace intrinsic::embodiment
