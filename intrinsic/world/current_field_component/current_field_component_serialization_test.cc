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
#include "intrinsic/embodiment/frame_policy.h"
#include "intrinsic/embodiment/proto/stamped_header.pb.h"
#include "intrinsic/embodiment/stamped_header_policy.h"
#include "intrinsic/world/current_field_component/current_field_component_policy.h"
#include "intrinsic/world/proto/current_field_component.pb.h"

namespace intrinsic::world {
namespace {

using intrinsic_proto::embodiment::Validity;
using intrinsic_proto::world::CurrentFieldComponent;

// Canonical serialization of FillFixture(). Keep in sync with
// current_field_component_serialization_test.py.
// velocity_m_s is (0.2, -0.1, 0) m/s. Proto3 omits the zero z component.
constexpr std::string_view kGoldenHex =
    "0a390a08667573696f6e5f30120b0880e2cfaa061080e59a771a08080a1080cab5ee01"
    "21000000000000e83f2a09636f762d7265662d37320208011209776f726c645f656e75"
    "1a12099a9999999999c93f119a9999999999b9bf";

constexpr std::string_view kWithoutVelocityHex =
    "0a390a08667573696f6e5f30120b0880e2cfaa061080e59a771a08080a1080cab5ee01"
    "21000000000000e83f2a09636f762d7265662d37320208011209776f726c645f656e75";

// Test-only plain value. Unknown fields are not copied. Angular current is
// not a member; it is zero by convention.
struct CurrentPlain {
  bool present = false;
  bool validity_message_present = false;
  std::string source_id;
  bool observation_time_present = false;
  TimeParts observation_time;
  bool validity_horizon_present = false;
  TimeParts validity_horizon;
  bool confidence_present = false;
  double confidence = 0;
  std::string uncertainty_reference;
  bool embodiment_validity_present = false;
  int validity_state = 0;
  std::string frame_id;
  bool velocity_present = false;
  embodiment::Vec3 velocity_m_s;
};

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

void FillFixture(CurrentFieldComponent* message) {
  message->mutable_validity()->set_source_id("fusion_0");
  message->mutable_validity()->mutable_observation_time()->set_seconds(
      1700000000);
  message->mutable_validity()->mutable_observation_time()->set_nanos(250000000);
  message->mutable_validity()->mutable_validity_horizon()->set_seconds(10);
  message->mutable_validity()->mutable_validity_horizon()->set_nanos(500000000);
  message->mutable_validity()->set_confidence(0.75);
  message->mutable_validity()->set_uncertainty_reference("cov-ref-7");
  message->mutable_validity()->mutable_validity()->set_state(
      Validity::STATE_VALID);
  message->set_frame_id("world_enu");
  message->mutable_velocity_m_s()->set_x(0.2);
  message->mutable_velocity_m_s()->set_y(-0.1);
  message->mutable_velocity_m_s()->set_z(0.0);
}

CurrentPlain PlainFromProto(const CurrentFieldComponent& message,
                            bool present) {
  CurrentPlain plain;
  plain.present = present;
  plain.validity_message_present = message.has_validity();
  plain.source_id = message.validity().source_id();
  plain.observation_time_present = message.validity().has_observation_time();
  plain.observation_time =
      TimeParts{message.validity().observation_time().seconds(),
                message.validity().observation_time().nanos()};
  plain.validity_horizon_present = message.validity().has_validity_horizon();
  plain.validity_horizon =
      TimeParts{message.validity().validity_horizon().seconds(),
                message.validity().validity_horizon().nanos()};
  plain.confidence_present = message.validity().has_confidence();
  plain.confidence = message.validity().confidence();
  plain.uncertainty_reference = message.validity().uncertainty_reference();
  plain.embodiment_validity_present = message.validity().has_validity();
  plain.validity_state =
      static_cast<int>(message.validity().validity().state());
  plain.frame_id = message.frame_id();
  plain.velocity_present = message.has_velocity_m_s();
  plain.velocity_m_s =
      embodiment::Vec3{message.velocity_m_s().x(), message.velocity_m_s().y(),
                       message.velocity_m_s().z()};
  return plain;
}

void ApplyPlain(const CurrentPlain& plain, CurrentFieldComponent* message) {
  message->Clear();
  if (plain.validity_message_present) {
    auto* validity = message->mutable_validity();
    validity->set_source_id(plain.source_id);
    if (plain.observation_time_present) {
      validity->mutable_observation_time()->set_seconds(
          plain.observation_time.seconds);
      validity->mutable_observation_time()->set_nanos(
          plain.observation_time.nanos);
    }
    if (plain.validity_horizon_present) {
      validity->mutable_validity_horizon()->set_seconds(
          plain.validity_horizon.seconds);
      validity->mutable_validity_horizon()->set_nanos(
          plain.validity_horizon.nanos);
    }
    if (plain.confidence_present) {
      validity->set_confidence(plain.confidence);
    }
    validity->set_uncertainty_reference(plain.uncertainty_reference);
    if (plain.embodiment_validity_present) {
      validity->mutable_validity()->set_state(
          static_cast<Validity::State>(plain.validity_state));
    }
  }
  message->set_frame_id(plain.frame_id);
  if (plain.velocity_present) {
    message->mutable_velocity_m_s()->set_x(plain.velocity_m_s.x);
    message->mutable_velocity_m_s()->set_y(plain.velocity_m_s.y);
    message->mutable_velocity_m_s()->set_z(plain.velocity_m_s.z);
  }
}

// string_view members alias plain. plain must outlive the returned view.
CurrentFieldView ViewFromPlain(const CurrentPlain& plain) {
  CurrentFieldView view;
  view.present = plain.present;
  view.validity.present = plain.validity_message_present;
  view.validity.source_id = plain.source_id;
  view.validity.observation_time_present = plain.observation_time_present;
  view.validity.observation_time = plain.observation_time;
  view.validity.validity_horizon_present = plain.validity_horizon_present;
  view.validity.validity_horizon = plain.validity_horizon;
  view.validity.confidence_present = plain.confidence_present;
  view.validity.confidence = plain.confidence;
  view.validity.uncertainty_reference = plain.uncertainty_reference;
  view.validity.validity_present = plain.embodiment_validity_present;
  view.validity.validity_state = plain.validity_state;
  view.frame_id = plain.frame_id;
  view.velocity_present = plain.velocity_present;
  view.velocity_m_s = plain.velocity_m_s;
  return view;
}

TEST(CurrentFieldSerializationTest, SchemaHasNoAngularField) {
  const auto* descriptor = CurrentFieldComponent::descriptor();
  ASSERT_EQ(descriptor->field_count(), 3);
  EXPECT_EQ(descriptor->field(0)->number(), 1);
  EXPECT_EQ(descriptor->field(0)->name(), "validity");
  EXPECT_EQ(descriptor->field(1)->number(), 2);
  EXPECT_EQ(descriptor->field(1)->name(), "frame_id");
  EXPECT_EQ(descriptor->field(2)->number(), 3);
  EXPECT_EQ(descriptor->field(2)->name(), "velocity_m_s");
  EXPECT_EQ(descriptor->FindFieldByName("angular_velocity_rad_s"), nullptr);
  EXPECT_EQ(descriptor->FindFieldByName("shear"), nullptr);
}

TEST(CurrentFieldSerializationTest, GoldenRoundTrip) {
  CurrentFieldComponent message;
  FillFixture(&message);
  const std::string golden = FromHex(kGoldenHex);
  EXPECT_EQ(message.SerializeAsString(), golden);

  CurrentFieldComponent parsed;
  ASSERT_TRUE(parsed.ParseFromString(golden));
  EXPECT_EQ(parsed.frame_id(), "world_enu");
  EXPECT_TRUE(parsed.has_velocity_m_s());
  EXPECT_DOUBLE_EQ(parsed.velocity_m_s().x(), 0.2);
  EXPECT_DOUBLE_EQ(parsed.velocity_m_s().y(), -0.1);
  EXPECT_DOUBLE_EQ(parsed.velocity_m_s().z(), 0.0);
  EXPECT_EQ(parsed.validity().source_id(), "fusion_0");
  EXPECT_EQ(parsed.validity().validity().state(), Validity::STATE_VALID);
  EXPECT_EQ(parsed.SerializeAsString(), golden);

  const CurrentPlain plain = PlainFromProto(parsed, true);
  CurrentFieldComponent again;
  ApplyPlain(plain, &again);
  EXPECT_EQ(again.SerializeAsString(), golden);
  const CurrentFieldAssessment assessment = AssessCurrentField(
      ViewFromPlain(plain), TimeParts{1700000010, 750000000});
  EXPECT_EQ(assessment.error, CurrentFieldError::kNone);
  EXPECT_TRUE(assessment.accepted);
  EXPECT_DOUBLE_EQ(plain.velocity_m_s.x, 0.2);
  EXPECT_DOUBLE_EQ(plain.velocity_m_s.y, -0.1);
  EXPECT_DOUBLE_EQ(plain.velocity_m_s.z, 0.0);
}

TEST(CurrentFieldSerializationTest, AbsentVelocityIsOmittedAndRejected) {
  CurrentFieldComponent message;
  FillFixture(&message);
  message.clear_velocity_m_s();
  const std::string without = FromHex(kWithoutVelocityHex);
  EXPECT_EQ(message.SerializeAsString(), without);
  EXPECT_TRUE(FromHex(kGoldenHex).starts_with(without));
  EXPECT_FALSE(message.has_velocity_m_s());

  const CurrentPlain plain = PlainFromProto(message, true);
  EXPECT_FALSE(plain.velocity_present);
  EXPECT_EQ(
      AssessCurrentField(ViewFromPlain(plain), TimeParts{1700000010, 750000000})
          .error,
      CurrentFieldError::kVelocity);
}

TEST(CurrentFieldSerializationTest, ExplicitZeroVelocityIsPresent) {
  CurrentFieldComponent message;
  FillFixture(&message);
  message.mutable_velocity_m_s()->set_x(0.0);
  message.mutable_velocity_m_s()->set_y(0.0);
  message.mutable_velocity_m_s()->set_z(0.0);
  EXPECT_TRUE(message.has_velocity_m_s());
  const CurrentPlain plain = PlainFromProto(message, true);
  EXPECT_TRUE(plain.velocity_present);
  EXPECT_DOUBLE_EQ(plain.velocity_m_s.x, 0.0);
  EXPECT_DOUBLE_EQ(plain.velocity_m_s.y, 0.0);
  EXPECT_DOUBLE_EQ(plain.velocity_m_s.z, 0.0);
  EXPECT_TRUE(
      AssessCurrentField(ViewFromPlain(plain), TimeParts{1700000010, 750000000})
          .accepted);
  CurrentFieldComponent again;
  ApplyPlain(plain, &again);
  EXPECT_TRUE(again.has_velocity_m_s());
  EXPECT_EQ(again.SerializeAsString(), message.SerializeAsString());
}

TEST(CurrentFieldSerializationTest, DefaultMessageIsEmpty) {
  const CurrentFieldComponent message;
  EXPECT_EQ(message.SerializeAsString(), "");
  EXPECT_FALSE(message.has_validity());
  EXPECT_FALSE(message.has_velocity_m_s());
  CurrentFieldComponent parsed;
  ASSERT_TRUE(parsed.ParseFromString(""));
  const CurrentPlain plain = PlainFromProto(parsed, false);
  CurrentFieldComponent again;
  ApplyPlain(plain, &again);
  EXPECT_EQ(again.SerializeAsString(), "");
  EXPECT_FALSE(
      AssessCurrentField(ViewFromPlain(plain), TimeParts{0, 0}).accepted);
  CurrentPlain present_plain = plain;
  present_plain.present = true;
  EXPECT_EQ(
      AssessCurrentField(ViewFromPlain(present_plain), TimeParts{0, 0}).error,
      CurrentFieldError::kValidity);
}

TEST(CurrentFieldSerializationTest, UnknownFieldIsPreserved) {
  const std::string golden = FromHex(kGoldenHex);
  const std::string with_unknown = golden + std::string("\xA0\x06\x07", 3);
  CurrentFieldComponent parsed;
  ASSERT_TRUE(parsed.ParseFromString(with_unknown));
  EXPECT_EQ(parsed.frame_id(), "world_enu");
  EXPECT_DOUBLE_EQ(parsed.velocity_m_s().x(), 0.2);
  const google::protobuf::UnknownFieldSet& unknown =
      parsed.GetReflection()->GetUnknownFields(parsed);
  ASSERT_EQ(unknown.field_count(), 1);
  EXPECT_EQ(unknown.field(0).number(), 100);
  EXPECT_EQ(unknown.field(0).varint(), 7u);
  EXPECT_EQ(parsed.SerializeAsString(), with_unknown);
}

TEST(CurrentFieldSerializationTest, UnknownValidityEnumRoundTrips) {
  const std::string bytes = FromHex("0a0432020863");
  CurrentFieldComponent parsed;
  ASSERT_TRUE(parsed.ParseFromString(bytes));
  EXPECT_TRUE(parsed.has_validity());
  EXPECT_EQ(static_cast<int>(parsed.validity().validity().state()), 99);
  EXPECT_FALSE(Validity::State_IsValid(99));
  EXPECT_EQ(parsed.SerializeAsString(), bytes);
  const CurrentPlain plain = PlainFromProto(parsed, true);
  EXPECT_EQ(embodiment::ClassifyValidity(true, plain.validity_state),
            embodiment::ValidityKind::kUnspecified);

  const std::string with_unknown_field = FromHex("0a06320408011805");
  CurrentFieldComponent nested;
  ASSERT_TRUE(nested.ParseFromString(with_unknown_field));
  EXPECT_EQ(nested.validity().validity().state(), Validity::STATE_VALID);
  const google::protobuf::UnknownFieldSet& unknown =
      nested.validity().validity().GetReflection()->GetUnknownFields(
          nested.validity().validity());
  ASSERT_EQ(unknown.field_count(), 1);
  EXPECT_EQ(unknown.field(0).number(), 3);
  EXPECT_EQ(unknown.field(0).varint(), 5u);
  EXPECT_EQ(nested.SerializeAsString(), with_unknown_field);
}

}  // namespace
}  // namespace intrinsic::world
