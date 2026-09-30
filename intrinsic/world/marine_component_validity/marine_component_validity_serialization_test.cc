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
#include <string>
#include <string_view>

#include "google/protobuf/unknown_field_set.h"
#include "gtest/gtest.h"
#include "intrinsic/embodiment/proto/stamped_header.pb.h"
#include "intrinsic/embodiment/stamped_header_policy.h"
#include "intrinsic/world/marine_component_validity/marine_component_validity_policy.h"
#include "intrinsic/world/proto/marine_component_validity.pb.h"

namespace intrinsic::world {
namespace {

using intrinsic_proto::embodiment::Validity;
using intrinsic_proto::world::MarineComponentValidity;

// Canonical serialization of FillFixture(). Keep in sync with
// marine_component_validity_serialization_test.py.
constexpr std::string_view kGoldenHex =
    "0a08667573696f6e5f30120b0880e2cfaa061080e59a771a08080a1080cab5ee01"
    "21000000000000e83f2a09636f762d7265662d3732020801";

constexpr std::string_view kWithoutValidityHex =
    "0a08667573696f6e5f30120b0880e2cfaa061080e59a771a08080a1080cab5ee01"
    "21000000000000e83f2a09636f762d7265662d37";

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

void FillFixture(MarineComponentValidity* message) {
  message->set_source_id("fusion_0");
  message->mutable_observation_time()->set_seconds(1700000000);
  message->mutable_observation_time()->set_nanos(250000000);
  message->mutable_validity_horizon()->set_seconds(10);
  message->mutable_validity_horizon()->set_nanos(500000000);
  message->set_confidence(0.75);
  message->set_uncertainty_reference("cov-ref-7");
  message->mutable_validity()->set_state(Validity::STATE_VALID);
}

MarineComponentValidityView ViewOf(const MarineComponentValidity& message,
                                   bool present) {
  MarineComponentValidityView view;
  view.present = present;
  view.source_id = message.source_id();
  view.observation_time_present = message.has_observation_time();
  view.observation_time = TimeParts{message.observation_time().seconds(),
                                    message.observation_time().nanos()};
  view.validity_horizon_present = message.has_validity_horizon();
  view.validity_horizon = TimeParts{message.validity_horizon().seconds(),
                                    message.validity_horizon().nanos()};
  view.confidence_present = message.has_confidence();
  view.confidence = message.confidence();
  view.uncertainty_reference = message.uncertainty_reference();
  view.validity_present = message.has_validity();
  view.validity_state = static_cast<int>(message.validity().state());
  return view;
}

TEST(MarineComponentValiditySerializationTest, GoldenRoundTrip) {
  MarineComponentValidity message;
  FillFixture(&message);
  const std::string golden = FromHex(kGoldenHex);
  EXPECT_EQ(message.SerializeAsString(), golden);

  MarineComponentValidity parsed;
  ASSERT_TRUE(parsed.ParseFromString(golden));
  EXPECT_EQ(parsed.source_id(), "fusion_0");
  EXPECT_EQ(parsed.observation_time().seconds(), 1700000000);
  EXPECT_EQ(parsed.observation_time().nanos(), 250000000);
  EXPECT_EQ(parsed.validity_horizon().seconds(), 10);
  EXPECT_EQ(parsed.validity_horizon().nanos(), 500000000);
  EXPECT_TRUE(parsed.has_confidence());
  EXPECT_DOUBLE_EQ(parsed.confidence(), 0.75);
  EXPECT_EQ(parsed.uncertainty_reference(), "cov-ref-7");
  EXPECT_TRUE(parsed.has_validity());
  EXPECT_EQ(parsed.validity().state(), Validity::STATE_VALID);
  EXPECT_EQ(parsed.SerializeAsString(), golden);
}

TEST(MarineComponentValiditySerializationTest,
     AbsentValidityIsOmittedAndDistinct) {
  MarineComponentValidity message;
  FillFixture(&message);
  message.clear_validity();
  const std::string without = FromHex(kWithoutValidityHex);
  EXPECT_EQ(message.SerializeAsString(), without);
  EXPECT_TRUE(FromHex(kGoldenHex).starts_with(without));

  MarineComponentValidity parsed;
  ASSERT_TRUE(parsed.ParseFromString(without));
  EXPECT_FALSE(parsed.has_validity());
  EXPECT_EQ(parsed.validity().state(), Validity::STATE_UNSPECIFIED);
  const MarineComponentAssessment assessment = AssessMarineComponentValidity(
      ViewOf(parsed, true), TimeParts{1700000010, 750000000});
  EXPECT_EQ(assessment.validity, embodiment::ValidityKind::kAbsent);
  EXPECT_EQ(assessment.freshness, ComponentFreshness::kUnknown);
  EXPECT_EQ(embodiment::ClassifyValidity(
                true, static_cast<int>(Validity::STATE_INVALID)),
            embodiment::ValidityKind::kInvalid);
}

TEST(MarineComponentValiditySerializationTest, DefaultMessageIsEmpty) {
  const MarineComponentValidity message;
  EXPECT_EQ(message.SerializeAsString(), "");
  EXPECT_FALSE(message.has_observation_time());
  EXPECT_FALSE(message.has_validity_horizon());
  EXPECT_FALSE(message.has_confidence());
  EXPECT_FALSE(message.has_validity());
  EXPECT_EQ(message.source_id(), "");
  EXPECT_EQ(message.uncertainty_reference(), "");
  MarineComponentValidity parsed;
  ASSERT_TRUE(parsed.ParseFromString(""));
  EXPECT_EQ(parsed.SerializeAsString(), "");
  const MarineComponentAssessment assessment =
      AssessMarineComponentValidity(ViewOf(parsed, false), TimeParts{0, 0});
  EXPECT_EQ(assessment.error, ComponentValidityError::kNone);
  EXPECT_FALSE(assessment.accepted);
  EXPECT_EQ(AssessMarineComponentValidity(ViewOf(parsed, true), TimeParts{0, 0})
                .error,
            ComponentValidityError::kSourceId);
}

TEST(MarineComponentValiditySerializationTest, UnknownFieldIsPreserved) {
  const std::string golden = FromHex(kGoldenHex);
  const std::string with_unknown = golden + std::string("\xA0\x06\x07", 3);
  MarineComponentValidity parsed;
  ASSERT_TRUE(parsed.ParseFromString(with_unknown));
  EXPECT_EQ(parsed.source_id(), "fusion_0");
  EXPECT_EQ(parsed.uncertainty_reference(), "cov-ref-7");
  const google::protobuf::UnknownFieldSet& unknown =
      parsed.GetReflection()->GetUnknownFields(parsed);
  ASSERT_EQ(unknown.field_count(), 1);
  EXPECT_EQ(unknown.field(0).number(), 100);
  EXPECT_EQ(unknown.field(0).varint(), 7u);
  EXPECT_EQ(parsed.SerializeAsString(), with_unknown);
}

TEST(MarineComponentValiditySerializationTest, UnknownValidityEnumRoundTrips) {
  EXPECT_EQ(Validity::State_MIN, 0);
  EXPECT_EQ(Validity::State_MAX, 2);
  EXPECT_FALSE(Validity::State_IsValid(3));

  const std::string bytes = FromHex("32020863");
  MarineComponentValidity parsed;
  ASSERT_TRUE(parsed.ParseFromString(bytes));
  EXPECT_TRUE(parsed.has_validity());
  EXPECT_EQ(static_cast<int>(parsed.validity().state()), 99);
  EXPECT_FALSE(Validity::State_IsValid(99));
  EXPECT_EQ(parsed.SerializeAsString(), bytes);
  EXPECT_EQ(embodiment::ClassifyValidity(true, 99),
            embodiment::ValidityKind::kUnspecified);

  const std::string with_unknown_field = FromHex("320408011805");
  MarineComponentValidity nested;
  ASSERT_TRUE(nested.ParseFromString(with_unknown_field));
  EXPECT_EQ(nested.validity().state(), Validity::STATE_VALID);
  const google::protobuf::UnknownFieldSet& unknown =
      nested.validity().GetReflection()->GetUnknownFields(nested.validity());
  ASSERT_EQ(unknown.field_count(), 1);
  EXPECT_EQ(unknown.field(0).number(), 3);
  EXPECT_EQ(unknown.field(0).varint(), 5u);
  EXPECT_EQ(nested.SerializeAsString(), with_unknown_field);
}

TEST(MarineComponentValiditySerializationTest,
     ConfidenceZeroIsDistinctFromUnset) {
  MarineComponentValidity unset;
  FillFixture(&unset);
  unset.clear_confidence();
  EXPECT_FALSE(unset.has_confidence());

  MarineComponentValidity zero;
  zero.CopyFrom(unset);
  zero.set_confidence(0.0);
  EXPECT_TRUE(zero.has_confidence());
  EXPECT_DOUBLE_EQ(zero.confidence(), 0.0);
  EXPECT_NE(unset.SerializeAsString(), zero.SerializeAsString());

  MarineComponentValidity parsed;
  ASSERT_TRUE(parsed.ParseFromString(zero.SerializeAsString()));
  EXPECT_TRUE(parsed.has_confidence());
  EXPECT_DOUBLE_EQ(parsed.confidence(), 0.0);
}

TEST(MarineComponentValiditySerializationTest, ParsedDeadlineBoundary) {
  MarineComponentValidity message;
  FillFixture(&message);
  const MarineComponentValidityView view = ViewOf(message, true);
  // 1700000000.250s + 10.500s = 1700000010.750s.
  const TimeParts deadline = TimeParts{1700000010, 750000000};
  const MarineComponentAssessment fresh =
      AssessMarineComponentValidity(view, deadline);
  EXPECT_EQ(fresh.error, ComponentValidityError::kNone);
  EXPECT_EQ(fresh.freshness, ComponentFreshness::kFresh);
  EXPECT_TRUE(fresh.accepted);

  const MarineComponentAssessment expired = AssessMarineComponentValidity(
      view, TimeParts{deadline.seconds, deadline.nanos + 1});
  EXPECT_EQ(expired.freshness, ComponentFreshness::kExpired);
  EXPECT_EQ(expired.validity, embodiment::ValidityKind::kValid);
  EXPECT_FALSE(expired.accepted);
}

}  // namespace
}  // namespace intrinsic::world
