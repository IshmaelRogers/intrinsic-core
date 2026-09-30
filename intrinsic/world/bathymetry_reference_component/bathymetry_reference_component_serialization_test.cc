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
#include "intrinsic/world/bathymetry_reference_component/bathymetry_reference_component_policy.h"
#include "intrinsic/world/proto/bathymetry_reference_component.pb.h"

namespace intrinsic::world {
namespace {

using intrinsic_proto::embodiment::Validity;
using intrinsic_proto::world::BathymetryReferenceComponent;

// Canonical serialization of FillFixture(). Keep in sync with
// bathymetry_reference_component_serialization_test.py.
constexpr std::string_view kGoldenHex =
    "0a390a08667573696f6e5f30120b0880e2cfaa061080e59a771a08080a1080cab5ee01"
    "21000000000000e83f2a09636f762d7265662d37320208011209776f726c645f656e75"
    "1a0b62617468792d7265662d3721000000000000f83f";

constexpr std::string_view kWithoutBiasHex =
    "0a390a08667573696f6e5f30120b0880e2cfaa061080e59a771a08080a1080cab5ee01"
    "21000000000000e83f2a09636f762d7265662d37320208011209776f726c645f656e75"
    "1a0b62617468792d7265662d37";

// Test-only plain value. Unknown fields are not copied.
struct BathymetryPlain {
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
  std::string asset_reference;
  bool vertical_bias_present = false;
  double vertical_bias_m = 0;
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

void FillFixture(BathymetryReferenceComponent* message) {
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
  message->set_asset_reference("bathy-ref-7");
  message->set_vertical_bias_m(1.5);
}

BathymetryPlain PlainFromProto(const BathymetryReferenceComponent& message,
                               bool present) {
  BathymetryPlain plain;
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
  plain.asset_reference = message.asset_reference();
  plain.vertical_bias_present = message.has_vertical_bias_m();
  plain.vertical_bias_m = message.vertical_bias_m();
  return plain;
}

void ApplyPlain(const BathymetryPlain& plain,
                BathymetryReferenceComponent* message) {
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
  message->set_asset_reference(plain.asset_reference);
  if (plain.vertical_bias_present) {
    message->set_vertical_bias_m(plain.vertical_bias_m);
  }
}

// string_view members alias plain. plain must outlive the returned view.
BathymetryReferenceView ViewFromPlain(const BathymetryPlain& plain) {
  BathymetryReferenceView view;
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
  view.asset_reference = plain.asset_reference;
  view.vertical_bias_present = plain.vertical_bias_present;
  view.vertical_bias_m = plain.vertical_bias_m;
  return view;
}

TEST(BathymetryReferenceSerializationTest, SchemaHasNoInlineGrid) {
  const auto* descriptor = BathymetryReferenceComponent::descriptor();
  ASSERT_EQ(descriptor->field_count(), 4);
  EXPECT_EQ(descriptor->field(0)->number(), 1);
  EXPECT_EQ(descriptor->field(0)->name(), "validity");
  EXPECT_EQ(descriptor->field(1)->number(), 2);
  EXPECT_EQ(descriptor->field(1)->name(), "frame_id");
  EXPECT_EQ(descriptor->field(2)->number(), 3);
  EXPECT_EQ(descriptor->field(2)->name(), "asset_reference");
  EXPECT_EQ(descriptor->field(3)->number(), 4);
  EXPECT_EQ(descriptor->field(3)->name(), "vertical_bias_m");
  EXPECT_EQ(descriptor->FindFieldByName("samples"), nullptr);
  EXPECT_EQ(descriptor->FindFieldByName("heights_m"), nullptr);
}

TEST(BathymetryReferenceSerializationTest, GoldenRoundTrip) {
  BathymetryReferenceComponent message;
  FillFixture(&message);
  const std::string golden = FromHex(kGoldenHex);
  EXPECT_EQ(message.SerializeAsString(), golden);

  BathymetryReferenceComponent parsed;
  ASSERT_TRUE(parsed.ParseFromString(golden));
  EXPECT_EQ(parsed.validity().source_id(), "fusion_0");
  EXPECT_EQ(parsed.validity().observation_time().seconds(), 1700000000);
  EXPECT_EQ(parsed.validity().observation_time().nanos(), 250000000);
  EXPECT_EQ(parsed.validity().validity_horizon().seconds(), 10);
  EXPECT_EQ(parsed.validity().validity_horizon().nanos(), 500000000);
  EXPECT_DOUBLE_EQ(parsed.validity().confidence(), 0.75);
  EXPECT_EQ(parsed.validity().uncertainty_reference(), "cov-ref-7");
  EXPECT_EQ(parsed.validity().validity().state(), Validity::STATE_VALID);
  EXPECT_EQ(parsed.frame_id(), "world_enu");
  EXPECT_EQ(parsed.asset_reference(), "bathy-ref-7");
  EXPECT_TRUE(parsed.has_vertical_bias_m());
  EXPECT_DOUBLE_EQ(parsed.vertical_bias_m(), 1.5);
  EXPECT_EQ(parsed.SerializeAsString(), golden);

  const BathymetryPlain plain = PlainFromProto(parsed, true);
  BathymetryReferenceComponent again;
  ApplyPlain(plain, &again);
  EXPECT_EQ(again.SerializeAsString(), golden);
  const BathymetryReferenceAssessment assessment = AssessBathymetryReference(
      ViewFromPlain(plain), TimeParts{1700000010, 750000000});
  EXPECT_EQ(assessment.error, BathymetryReferenceError::kNone);
  EXPECT_TRUE(assessment.accepted);
}

TEST(BathymetryReferenceSerializationTest, BiasZeroIsDistinctFromUnset) {
  BathymetryReferenceComponent unset;
  FillFixture(&unset);
  unset.clear_vertical_bias_m();
  const std::string without = FromHex(kWithoutBiasHex);
  EXPECT_EQ(unset.SerializeAsString(), without);
  EXPECT_TRUE(FromHex(kGoldenHex).starts_with(without));
  EXPECT_FALSE(unset.has_vertical_bias_m());

  BathymetryReferenceComponent zero;
  zero.CopyFrom(unset);
  zero.set_vertical_bias_m(0.0);
  EXPECT_TRUE(zero.has_vertical_bias_m());
  EXPECT_NE(unset.SerializeAsString(), zero.SerializeAsString());
  const BathymetryPlain plain = PlainFromProto(zero, true);
  EXPECT_TRUE(plain.vertical_bias_present);
  EXPECT_DOUBLE_EQ(plain.vertical_bias_m, 0.0);
  EXPECT_TRUE(AssessBathymetryReference(ViewFromPlain(plain),
                                        TimeParts{1700000010, 750000000})
                  .accepted);
}

TEST(BathymetryReferenceSerializationTest, DefaultMessageIsEmpty) {
  const BathymetryReferenceComponent message;
  EXPECT_EQ(message.SerializeAsString(), "");
  EXPECT_FALSE(message.has_validity());
  EXPECT_FALSE(message.has_vertical_bias_m());
  EXPECT_EQ(message.frame_id(), "");
  EXPECT_EQ(message.asset_reference(), "");
  BathymetryReferenceComponent parsed;
  ASSERT_TRUE(parsed.ParseFromString(""));
  const BathymetryPlain plain = PlainFromProto(parsed, false);
  BathymetryReferenceComponent again;
  ApplyPlain(plain, &again);
  EXPECT_EQ(again.SerializeAsString(), "");
  const BathymetryReferenceAssessment empty =
      AssessBathymetryReference(ViewFromPlain(plain), TimeParts{0, 0});
  EXPECT_EQ(empty.error, BathymetryReferenceError::kNone);
  EXPECT_FALSE(empty.accepted);
  BathymetryPlain present_plain = plain;
  present_plain.present = true;
  EXPECT_EQ(
      AssessBathymetryReference(ViewFromPlain(present_plain), TimeParts{0, 0})
          .error,
      BathymetryReferenceError::kValidity);
}

TEST(BathymetryReferenceSerializationTest, UnknownFieldIsPreserved) {
  const std::string golden = FromHex(kGoldenHex);
  const std::string with_unknown = golden + std::string("\xA0\x06\x07", 3);
  BathymetryReferenceComponent parsed;
  ASSERT_TRUE(parsed.ParseFromString(with_unknown));
  EXPECT_EQ(parsed.asset_reference(), "bathy-ref-7");
  EXPECT_EQ(parsed.frame_id(), "world_enu");
  const google::protobuf::UnknownFieldSet& unknown =
      parsed.GetReflection()->GetUnknownFields(parsed);
  ASSERT_EQ(unknown.field_count(), 1);
  EXPECT_EQ(unknown.field(0).number(), 100);
  EXPECT_EQ(unknown.field(0).varint(), 7u);
  EXPECT_EQ(parsed.SerializeAsString(), with_unknown);
}

TEST(BathymetryReferenceSerializationTest, UnknownValidityEnumRoundTrips) {
  const std::string bytes = FromHex("0a0432020863");
  BathymetryReferenceComponent parsed;
  ASSERT_TRUE(parsed.ParseFromString(bytes));
  EXPECT_TRUE(parsed.has_validity());
  EXPECT_TRUE(parsed.validity().has_validity());
  EXPECT_EQ(static_cast<int>(parsed.validity().validity().state()), 99);
  EXPECT_FALSE(Validity::State_IsValid(99));
  EXPECT_EQ(parsed.SerializeAsString(), bytes);
  const BathymetryPlain plain = PlainFromProto(parsed, true);
  EXPECT_EQ(embodiment::ClassifyValidity(true, plain.validity_state),
            embodiment::ValidityKind::kUnspecified);

  const std::string with_unknown_field = FromHex("0a06320408011805");
  BathymetryReferenceComponent nested;
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
