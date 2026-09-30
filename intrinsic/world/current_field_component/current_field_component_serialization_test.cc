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
#include <cstdlib>
#include <fstream>
#include <limits>
#include <sstream>
#include <string>
#include <string_view>
#include <vector>

#include "google/protobuf/text_format.h"
#include "google/protobuf/unknown_field_set.h"
#include "gtest/gtest.h"
#include "intrinsic/embodiment/proto/stamped_header.pb.h"
#include "intrinsic/embodiment/stamped_header_policy.h"
#include "intrinsic/world/current_field_component/current_field_component_policy.h"
#include "intrinsic/world/marine_component_validity/marine_component_validity_policy.h"
#include "intrinsic/world/proto/current_field_component.pb.h"

namespace intrinsic::world {
namespace {

using intrinsic_proto::embodiment::Validity;
using intrinsic_proto::world::CurrentFieldComponent;

// Canonical serialization of testdata/constant_current.textproto.
// Keep in sync with current_field_component_serialization_test.py.
constexpr std::string_view kGoldenHex =
    "0a390a08667573696f6e5f30120b0880e2cfaa061080e59a771a08080a1080cab5ee01"
    "21000000000000e83f2a09636f762d7265662d37320208011209776f726c645f656e75"
    "1a1d0a1b09000000000000f83f11000000000000d0bf19000000000000c03f";

constexpr std::string_view kExampleSuffix =
    "intrinsic/world/current_field_component/testdata/"
    "constant_current.textproto";

// 1700000000.250s + 10.500s.
constexpr TimeParts kDeadline = TimeParts{1700000010, 750000000};

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

std::string LoadText() {
  std::vector<std::string> candidates;
  if (const char *src = std::getenv("TEST_SRCDIR")) {
    candidates.push_back(std::string(src) + "/_main/" +
                         std::string(kExampleSuffix));
    candidates.push_back(std::string(src) + "/" + std::string(kExampleSuffix));
    if (const char *workspace = std::getenv("TEST_WORKSPACE")) {
      candidates.push_back(std::string(src) + "/" + workspace + "/" +
                           std::string(kExampleSuffix));
    }
  }
  candidates.emplace_back(kExampleSuffix);
  for (const std::string &path : candidates) {
    std::ifstream in(path);
    if (!in) {
      continue;
    }
    std::ostringstream buffer;
    buffer << in.rdbuf();
    if (!buffer.str().empty()) {
      return buffer.str();
    }
  }
  return "";
}

CurrentFieldView ViewOf(const CurrentFieldComponent &message, bool present) {
  const auto &meta = message.validity_meta();
  CurrentFieldView view;
  view.present = present;
  view.validity_meta.present = message.has_validity_meta();
  view.validity_meta.source_id = meta.source_id();
  view.validity_meta.observation_time_present = meta.has_observation_time();
  view.validity_meta.observation_time = TimeParts{
      meta.observation_time().seconds(), meta.observation_time().nanos()};
  view.validity_meta.validity_horizon_present = meta.has_validity_horizon();
  view.validity_meta.validity_horizon = TimeParts{
      meta.validity_horizon().seconds(), meta.validity_horizon().nanos()};
  view.validity_meta.confidence_present = meta.has_confidence();
  view.validity_meta.confidence = meta.confidence();
  view.validity_meta.uncertainty_reference = meta.uncertainty_reference();
  view.validity_meta.validity_present = meta.has_validity();
  view.validity_meta.validity_state = static_cast<int>(meta.validity().state());
  view.frame_id = message.frame_id();
  view.constant_present = message.has_constant();
  view.velocity_x_m_s = message.constant().velocity_m_s().x();
  view.velocity_y_m_s = message.constant().velocity_m_s().y();
  view.velocity_z_m_s = message.constant().velocity_m_s().z();
  return view;
}

class CurrentFieldSerializationTest : public ::testing::Test {
 protected:
  void SetUp() override {
    const std::string text = LoadText();
    ASSERT_FALSE(text.empty()) << kExampleSuffix;
    ASSERT_TRUE(google::protobuf::TextFormat::ParseFromString(text, &example_))
        << text;
  }

  CurrentFieldComponent example_;
};

TEST_F(CurrentFieldSerializationTest, TextprotoMatchesGoldenAndValidates) {
  const std::string golden = FromHex(kGoldenHex);
  EXPECT_EQ(example_.SerializeAsString(), golden);
  EXPECT_EQ(example_.frame_id(), "world_enu");
  EXPECT_TRUE(example_.has_constant());
  EXPECT_DOUBLE_EQ(example_.constant().velocity_m_s().x(), 1.5);
  EXPECT_DOUBLE_EQ(example_.constant().velocity_m_s().y(), -0.25);
  EXPECT_DOUBLE_EQ(example_.constant().velocity_m_s().z(), 0.125);
  EXPECT_EQ(example_.validity_meta().validity().state(), Validity::STATE_VALID);

  CurrentFieldComponent parsed;
  ASSERT_TRUE(parsed.ParseFromString(golden));
  EXPECT_EQ(parsed.SerializeAsString(), golden);
  const CurrentFieldAssessment fresh =
      AssessCurrentField(ViewOf(parsed, true), kDeadline);
  EXPECT_EQ(fresh.error, CurrentFieldError::kNone);
  EXPECT_EQ(fresh.validity.freshness, ComponentFreshness::kFresh);
  EXPECT_TRUE(fresh.accepted);

  const TimeParts after = TimeParts{kDeadline.seconds, kDeadline.nanos + 1};
  const CurrentFieldAssessment expired =
      AssessCurrentField(ViewOf(parsed, true), after);
  EXPECT_EQ(expired.error, CurrentFieldError::kNone);
  EXPECT_EQ(expired.validity.freshness, ComponentFreshness::kExpired);
  EXPECT_EQ(expired.validity.validity, embodiment::ValidityKind::kValid);
  EXPECT_FALSE(expired.accepted);
}

TEST_F(CurrentFieldSerializationTest, DefaultMessageIsEmpty) {
  const CurrentFieldComponent message;
  EXPECT_EQ(message.SerializeAsString(), "");
  EXPECT_FALSE(message.has_validity_meta());
  EXPECT_FALSE(message.has_constant());
  EXPECT_EQ(message.frame_id(), "");
  CurrentFieldComponent parsed;
  ASSERT_TRUE(parsed.ParseFromString(""));
  EXPECT_EQ(parsed.SerializeAsString(), "");
  const CurrentFieldAssessment empty =
      AssessCurrentField(ViewOf(parsed, false), kDeadline);
  EXPECT_EQ(empty.error, CurrentFieldError::kNone);
  EXPECT_FALSE(empty.accepted);
}

TEST_F(CurrentFieldSerializationTest, UnknownFieldIsPreserved) {
  const std::string golden = FromHex(kGoldenHex);
  const std::string with_unknown = golden + std::string("\xA0\x06\x07", 3);
  CurrentFieldComponent parsed;
  ASSERT_TRUE(parsed.ParseFromString(with_unknown));
  EXPECT_EQ(parsed.frame_id(), "world_enu");
  EXPECT_TRUE(parsed.has_constant());
  const google::protobuf::UnknownFieldSet &unknown =
      parsed.GetReflection()->GetUnknownFields(parsed);
  ASSERT_EQ(unknown.field_count(), 1);
  EXPECT_EQ(unknown.field(0).number(), 100);
  EXPECT_EQ(unknown.field(0).varint(), 7u);
  EXPECT_EQ(parsed.SerializeAsString(), with_unknown);
}

TEST_F(CurrentFieldSerializationTest, ExplicitZeroCurrentIsPresent) {
  CurrentFieldComponent zeros = example_;
  zeros.clear_constant();
  zeros.mutable_constant();
  EXPECT_TRUE(zeros.has_constant());
  EXPECT_DOUBLE_EQ(zeros.constant().velocity_m_s().x(), 0);
  EXPECT_DOUBLE_EQ(zeros.constant().velocity_m_s().y(), 0);
  EXPECT_DOUBLE_EQ(zeros.constant().velocity_m_s().z(), 0);
  CurrentFieldComponent missing = example_;
  missing.clear_constant();
  EXPECT_NE(zeros.SerializeAsString(), missing.SerializeAsString());
  EXPECT_TRUE(AssessCurrentField(ViewOf(zeros, true), kDeadline).accepted);
  EXPECT_EQ(AssessCurrentField(ViewOf(missing, true), kDeadline).error,
            CurrentFieldError::kRepresentation);
}

TEST_F(CurrentFieldSerializationTest, OmittedAxesAreZero) {
  CurrentFieldComponent message = example_;
  message.clear_constant();
  message.mutable_constant()->mutable_velocity_m_s()->set_x(1.0);
  EXPECT_DOUBLE_EQ(message.constant().velocity_m_s().y(), 0);
  EXPECT_DOUBLE_EQ(message.constant().velocity_m_s().z(), 0);
  EXPECT_TRUE(AssessCurrentField(ViewOf(message, true), kDeadline).accepted);
}

TEST_F(CurrentFieldSerializationTest, RejectsEmptyFrameAndNonFiniteVelocity) {
  CurrentFieldComponent bad = example_;
  bad.set_frame_id("");
  EXPECT_EQ(AssessCurrentField(ViewOf(bad, true), kDeadline).error,
            CurrentFieldError::kFrameId);

  bad = example_;
  bad.mutable_constant()->mutable_velocity_m_s()->set_y(
      std::numeric_limits<double>::quiet_NaN());
  EXPECT_EQ(AssessCurrentField(ViewOf(bad, true), kDeadline).error,
            CurrentFieldError::kVelocity);
}

TEST_F(CurrentFieldSerializationTest, UnknownValidityEnumRoundTrips) {
  CurrentFieldComponent message = example_;
  message.mutable_validity_meta()->mutable_validity()->set_state(
      static_cast<Validity::State>(99));
  const std::string bytes = message.SerializeAsString();
  CurrentFieldComponent parsed;
  ASSERT_TRUE(parsed.ParseFromString(bytes));
  EXPECT_EQ(static_cast<int>(parsed.validity_meta().validity().state()), 99);
  EXPECT_FALSE(Validity::State_IsValid(99));
  EXPECT_EQ(parsed.SerializeAsString(), bytes);
  const CurrentFieldAssessment assessment =
      AssessCurrentField(ViewOf(parsed, true), kDeadline);
  EXPECT_EQ(assessment.validity.validity,
            embodiment::ValidityKind::kUnspecified);
  EXPECT_FALSE(assessment.accepted);
}

}  // namespace
}  // namespace intrinsic::world
