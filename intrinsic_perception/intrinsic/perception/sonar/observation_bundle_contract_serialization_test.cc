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

#include <cstdint>
#include <cstdlib>
#include <fstream>
#include <sstream>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include "google/protobuf/descriptor.h"
#include "google/protobuf/text_format.h"
#include "gtest/gtest.h"
#include "intrinsic/embodiment/proto/stamped_header.pb.h"
#include "intrinsic/embodiment/stamped_header_policy.h"
#include "intrinsic/perception/proto/sonar/observation_bundle.pb.h"
#include "intrinsic/perception/sonar/observation_bundle_builder.h"
#include "intrinsic/perception/sonar/observation_bundle_contract_policy.h"

namespace intrinsic::perception::sonar {
namespace {

using embodiment::ValidityKind;
using intrinsic_proto::embodiment::Validity;
using intrinsic_proto::perception::sonar::MultimodalObservationBundle;
using intrinsic_proto::perception::sonar::ObservationSlot;
using ProtoModality = intrinsic_proto::perception::sonar::ObservationModality;
using ProtoReason = intrinsic_proto::perception::sonar::AbsentModalityReason;
using Error = ObservationBundleContractError;

// Canonical serialization of examples/observation_bundle_sonar_only.textproto.
// Keep in sync with observation_bundle_contract_serialization_test.py.
constexpr std::string_view kSonarOnlyGoldenHex =
    "0a36080712060880e2cfaa061a0b0880e2cfaa061080c2d72f220862756e646c655f"
    "302a0473796e6332096d6f6e6f746f6e69633a0208011205108084af5f1a660a3808"
    "0b12060880e2cfaa061a0b0880e2cfaa061080dac4092205666c735f302a09666c73"
    "5f736f6e617232096d6f6e6f746f6e69633a020801120e666c732d6672616d652d30"
    "3030311a176170706c69636174696f6e2f782d666c732d6672616d65208001420408"
    "021001420408031002420408041003420408051005a206140a0662756e646c65120a"
    "736f6e61725f6f6e6c79";

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

ObservationSlotView SlotView(const ObservationSlot& slot) {
  ObservationSlotView view;
  view.reference_id = slot.reference_id();
  view.content_type = slot.content_type();
  view.byte_size_present = slot.has_byte_size();
  view.byte_size = slot.byte_size();
  view.frame_id = slot.header().frame_id();
  view.source_time_present = slot.header().has_source_time();
  view.source_time = {slot.header().source_time().seconds(),
                      slot.header().source_time().nanos()};
  view.receive_time_present = slot.header().has_receive_time();
  view.receive_time = {slot.header().receive_time().seconds(),
                       slot.header().receive_time().nanos()};
  view.clock_domain = slot.header().clock_domain();
  return view;
}

ObservationBundleView ViewOf(
    const MultimodalObservationBundle& message,
    const std::vector<AbsentModalityEntryView>& absent) {
  ObservationBundleView view;
  view.header_present = message.has_header();
  view.validity_present = message.header().has_validity();
  view.validity_state = static_cast<int>(message.header().validity().state());
  view.frame_id = message.header().frame_id();
  view.clock_domain = message.header().clock_domain();
  view.source_time_present = message.header().has_source_time();
  view.source_time = {message.header().source_time().seconds(),
                      message.header().source_time().nanos()};
  view.receive_time_present = message.header().has_receive_time();
  view.receive_time = {message.header().receive_time().seconds(),
                       message.header().receive_time().nanos()};
  view.max_skew_present = message.has_max_skew();
  view.max_skew_seconds = message.max_skew().seconds();
  view.max_skew_nanos = message.max_skew().nanos();
  view.fls = SlotView(message.fls());
  view.sss = SlotView(message.sss());
  view.optical = SlotView(message.optical());
  view.point_cloud = SlotView(message.point_cloud());
  const auto& state = message.vehicle_state();
  view.vehicle_state.message_set = message.has_vehicle_state();
  view.vehicle_state.frame_id = state.header().frame_id();
  view.vehicle_state.source_time_present = state.header().has_source_time();
  view.vehicle_state.source_time = {state.header().source_time().seconds(),
                                    state.header().source_time().nanos()};
  view.vehicle_state.receive_time_present = state.header().has_receive_time();
  view.vehicle_state.receive_time = {state.header().receive_time().seconds(),
                                     state.header().receive_time().nanos()};
  view.vehicle_state.clock_domain = state.header().clock_domain();
  view.vehicle_state.state_epoch = state.state_epoch();
  view.vehicle_state.world_snapshot_id = state.world_snapshot_id();
  view.absent = absent;
  view.metadata_present = message.metadata_size() > 0;
  return view;
}

ObservationBundleContractAssessment Assess(
    const MultimodalObservationBundle& message) {
  std::vector<AbsentModalityEntryView> absent;
  absent.reserve(message.absent_size());
  for (const auto& entry : message.absent()) {
    absent.push_back(
        {static_cast<int>(entry.modality()), static_cast<int>(entry.reason())});
  }
  return AssessMultimodalObservationBundle(ViewOf(message, absent));
}

std::string LoadExample(const std::string& name) {
  const char* src = std::getenv("TEST_SRCDIR");
  if (src == nullptr) {
    return "";
  }
  const std::string suffix =
      "intrinsic/perception/proto/sonar/examples/" + name;
  const char* workspace = std::getenv("TEST_WORKSPACE");
  std::vector<std::string> candidates = {
      std::string(src) + "/intrinsic_apis+/" + suffix,
      std::string(src) + "/intrinsic_apis/" + suffix,
  };
  if (workspace != nullptr) {
    candidates.push_back(std::string(src) + "/" + workspace +
                         "/external/intrinsic_apis+/" + suffix);
    candidates.push_back(std::string(src) + "/" + workspace +
                         "/external/intrinsic_apis/" + suffix);
    candidates.push_back(std::string(src) + "/" + workspace + "/" + suffix);
  }
  for (const std::string& path : candidates) {
    std::ifstream in(path);
    if (!in) {
      continue;
    }
    std::ostringstream buffer;
    buffer << in.rdbuf();
    return buffer.str();
  }
  return "";
}

MultimodalObservationBundle ParseExample(const std::string& name) {
  const std::string text = LoadExample(name);
  EXPECT_FALSE(text.empty()) << name;
  MultimodalObservationBundle parsed;
  EXPECT_TRUE(google::protobuf::TextFormat::ParseFromString(text, &parsed))
      << name;
  return parsed;
}

ObservationSlotParts SonarOnlyFls() {
  ObservationSlotParts fls;
  fls.sequence = 11;
  fls.source_seconds = 1700000000;
  fls.set_receive = true;
  fls.receive_seconds = 1700000000;
  fls.receive_nanos = 20000000;
  fls.source_id = "fls_0";
  fls.frame_id = "fls_sonar";
  fls.clock_domain = "monotonic";
  fls.set_validity = true;
  fls.validity_state = Validity::STATE_VALID;
  fls.reference_id = "fls-frame-0001";
  fls.content_type = "application/x-fls-frame";
  fls.set_byte_size = true;
  fls.byte_size = 128;
  return fls;
}

ObservationBundleParts SonarOnlyParts() {
  ObservationBundleParts parts;
  parts.set_header = true;
  parts.header.sequence = 7;
  parts.header.source_seconds = 1700000000;
  parts.header.set_receive = true;
  parts.header.receive_seconds = 1700000000;
  parts.header.receive_nanos = 100000000;
  parts.header.source_id = "bundle_0";
  parts.header.frame_id = "sync";
  parts.header.clock_domain = "monotonic";
  parts.header.set_validity = true;
  parts.header.validity_state = Validity::STATE_VALID;
  parts.set_max_skew = true;
  parts.max_skew_nanos = kFixtureMaxSkewNanos;
  parts.set_fls = true;
  parts.fls = SonarOnlyFls();
  parts.absent = {
      {static_cast<int>(ProtoModality::OBSERVATION_MODALITY_SONAR_SSS),
       static_cast<int>(ProtoReason::ABSENT_MODALITY_REASON_NOT_CONFIGURED)},
      {static_cast<int>(ProtoModality::OBSERVATION_MODALITY_OPTICAL),
       static_cast<int>(ProtoReason::ABSENT_MODALITY_REASON_SENSOR_OFFLINE)},
      {static_cast<int>(ProtoModality::OBSERVATION_MODALITY_POINT_CLOUD),
       static_cast<int>(ProtoReason::ABSENT_MODALITY_REASON_OUT_OF_RANGE)},
      {static_cast<int>(ProtoModality::OBSERVATION_MODALITY_VEHICLE_STATE),
       static_cast<int>(
           ProtoReason::ABSENT_MODALITY_REASON_INTENTIONALLY_OMITTED)},
  };
  parts.metadata = {{"bundle", "sonar_only"}};
  return parts;
}

TEST(ObservationBundleSerializationTest, PackageAndLockedFieldNumbers) {
  const auto* descriptor = MultimodalObservationBundle::descriptor();
  EXPECT_EQ(descriptor->file()->package(), "intrinsic_proto.perception.sonar");
  EXPECT_EQ(descriptor->field_count(), 9);
  EXPECT_EQ(descriptor->FindFieldByName("header")->number(), 1);
  EXPECT_EQ(descriptor->FindFieldByName("max_skew")->number(), 2);
  EXPECT_EQ(descriptor->FindFieldByName("fls")->number(), 3);
  EXPECT_EQ(descriptor->FindFieldByName("sss")->number(), 4);
  EXPECT_EQ(descriptor->FindFieldByName("optical")->number(), 5);
  EXPECT_EQ(descriptor->FindFieldByName("point_cloud")->number(), 6);
  EXPECT_EQ(descriptor->FindFieldByName("vehicle_state")->number(), 7);
  EXPECT_EQ(descriptor->FindFieldByName("absent")->number(), 8);
  EXPECT_EQ(descriptor->FindFieldByName("metadata")->number(), 100);
  EXPECT_EQ(descriptor->FindFieldByName("robot_type"), nullptr);

  const auto* slot = ObservationSlot::descriptor();
  EXPECT_EQ(slot->field_count(), 4);
  EXPECT_EQ(slot->FindFieldByName("header")->number(), 1);
  EXPECT_EQ(slot->FindFieldByName("reference_id")->number(), 2);
  EXPECT_EQ(slot->FindFieldByName("content_type")->number(), 3);
  EXPECT_EQ(slot->FindFieldByName("byte_size")->number(), 4);

  const auto* state =
      intrinsic_proto::perception::sonar::StateReference::descriptor();
  EXPECT_EQ(state->field_count(), 3);
  EXPECT_EQ(state->FindFieldByName("header")->number(), 1);
  EXPECT_EQ(state->FindFieldByName("state_epoch")->number(), 2);
  EXPECT_EQ(state->FindFieldByName("world_snapshot_id")->number(), 3);

  const auto* absent =
      intrinsic_proto::perception::sonar::AbsentModalityEntry::descriptor();
  EXPECT_EQ(absent->field_count(), 2);
  EXPECT_EQ(absent->FindFieldByName("modality")->number(), 1);
  EXPECT_EQ(absent->FindFieldByName("reason")->number(), 2);

  const auto* file = descriptor->file();
  EXPECT_EQ(file->message_type_count(), 4);
  EXPECT_EQ(file->enum_type_count(), 2);
  EXPECT_EQ(file->dependency_count(), 2);
  EXPECT_EQ(file->enum_type(0)->value_count(), 6);
  EXPECT_EQ(file->enum_type(1)->value_count(), 6);
  EXPECT_EQ(ProtoModality::OBSERVATION_MODALITY_SONAR_FLS, 1);
  EXPECT_EQ(ProtoReason::ABSENT_MODALITY_REASON_DROPPED_FOR_SKEW, 4);
}

TEST(ObservationBundleSerializationTest, DoesNotImportFlsOrCameraProtos) {
  const auto* file = MultimodalObservationBundle::descriptor()->file();
  for (int i = 0; i < file->dependency_count(); ++i) {
    const std::string name = file->dependency(i)->name();
    EXPECT_EQ(name.find("fls_frame"), std::string::npos) << name;
    EXPECT_EQ(name.find("sss_frame"), std::string::npos) << name;
    EXPECT_EQ(name.find("/v1/"), std::string::npos) << name;
  }
}

TEST(ObservationBundleSerializationTest, DefaultIsZeroBytesAndNotAccepted) {
  MultimodalObservationBundle empty;
  EXPECT_EQ(empty.SerializeAsString(), "");
  EXPECT_EQ(empty.ByteSizeLong(), 0u);
  EXPECT_FALSE(empty.has_header());
  EXPECT_FALSE(empty.has_max_skew());
  EXPECT_FALSE(empty.has_fls());
  EXPECT_FALSE(empty.has_vehicle_state());
  EXPECT_EQ(empty.absent_size(), 0);
  MultimodalObservationBundle parsed;
  ASSERT_TRUE(parsed.ParseFromString(""));
  EXPECT_EQ(parsed.SerializeAsString(), "");
  const ObservationBundleContractAssessment assessment = Assess(parsed);
  EXPECT_EQ(assessment.error, Error::kNone);
  EXPECT_EQ(assessment.validity, ValidityKind::kAbsent);
  EXPECT_FALSE(assessment.accepted);
}

TEST(ObservationBundleSerializationTest, SonarOnlyGoldenRoundTrip) {
  const MultimodalObservationBundle parsed =
      ParseExample("observation_bundle_sonar_only.textproto");
  const std::string golden = FromHex(kSonarOnlyGoldenHex);
  EXPECT_EQ(parsed.SerializeAsString(), golden);
  MultimodalObservationBundle again;
  ASSERT_TRUE(again.ParseFromString(golden));
  EXPECT_EQ(again.SerializeAsString(), golden);
  EXPECT_EQ(again.header().frame_id(), "sync");
  EXPECT_EQ(again.fls().reference_id(), "fls-frame-0001");
  EXPECT_EQ(again.max_skew().nanos(), kFixtureMaxSkewNanos);
  EXPECT_EQ(again.metadata().at("bundle"), "sonar_only");
  EXPECT_FALSE(again.has_sss());
  EXPECT_FALSE(again.has_optical());
  const ObservationBundleContractAssessment assessment = Assess(again);
  EXPECT_EQ(assessment.error, Error::kNone);
  EXPECT_EQ(assessment.validity, ValidityKind::kValid);
  EXPECT_TRUE(assessment.accepted);
  EXPECT_TRUE(assessment.measured_skew_present);
  EXPECT_EQ(assessment.measured_skew.nanos, 0);
}

TEST(ObservationBundleSerializationTest, BuilderMatchesSonarOnlyFixture) {
  MultimodalObservationBundle built;
  BuildMultimodalObservationBundle(SonarOnlyParts(), &built);
  const MultimodalObservationBundle parsed =
      ParseExample("observation_bundle_sonar_only.textproto");
  EXPECT_EQ(built.SerializeAsString(), parsed.SerializeAsString());
  EXPECT_EQ(built.SerializeAsString(), FromHex(kSonarOnlyGoldenHex));
  EXPECT_TRUE(Assess(built).accepted);
  EXPECT_EQ(built.fls().reference_id(), "fls-frame-0001");
  EXPECT_FALSE(built.has_sss());
  EXPECT_FALSE(built.has_optical());
  EXPECT_FALSE(built.has_point_cloud());
  EXPECT_FALSE(built.has_vehicle_state());
}

TEST(ObservationBundleSerializationTest, BuilderDoesNotRewriteABadSnapshot) {
  ObservationBundleParts parts;
  parts.set_header = true;
  parts.header.frame_id = "sync";
  parts.header.clock_domain = "monotonic";
  parts.header.set_validity = true;
  parts.header.validity_state = Validity::STATE_VALID;
  parts.set_max_skew = true;
  parts.max_skew_nanos = kFixtureMaxSkewNanos;
  parts.vehicle_state.set = true;
  parts.vehicle_state.stamp.frame_id = "body";
  parts.vehicle_state.stamp.clock_domain = "monotonic";
  parts.vehicle_state.stamp.source_seconds = 1700000000;
  parts.vehicle_state.world_snapshot_id = "not-a-hex";
  parts.vehicle_state.state_epoch = 0;
  MultimodalObservationBundle built;
  BuildMultimodalObservationBundle(parts, &built);
  EXPECT_EQ(built.vehicle_state().world_snapshot_id(), "not-a-hex");
  EXPECT_EQ(built.vehicle_state().state_epoch(), 0u);
  EXPECT_EQ(Assess(built).error, Error::kSnapshotId);
}

TEST(ObservationBundleSerializationTest, UnknownFieldIsPreserved) {
  std::string with_unknown = FromHex(kSonarOnlyGoldenHex);
  with_unknown.append("\xA8\x06\x07", 3);
  MultimodalObservationBundle parsed;
  ASSERT_TRUE(parsed.ParseFromString(with_unknown));
  EXPECT_EQ(parsed.header().frame_id(), "sync");
  EXPECT_EQ(parsed.SerializeAsString(), with_unknown);
  EXPECT_TRUE(Assess(parsed).accepted);
}

TEST(ObservationBundleSerializationTest, UnknownNestedFieldIsPreserved) {
  ObservationSlot slot;
  slot.set_reference_id("fls-frame-0001");
  std::string nested = slot.SerializeAsString();
  nested.append("\x78\x01", 2);
  std::string raw;
  raw.push_back('\x1a');
  raw.push_back(static_cast<char>(nested.size()));
  raw += nested;
  MultimodalObservationBundle parsed;
  ASSERT_TRUE(parsed.ParseFromString(raw));
  EXPECT_EQ(parsed.fls().reference_id(), "fls-frame-0001");
  EXPECT_EQ(parsed.SerializeAsString(), raw);
}

TEST(ObservationBundleSerializationTest, FixturesMatchLockedResults) {
  const struct Case {
    const char* name;
    Error error;
    bool accepted;
  } cases[] = {
      {"observation_bundle_sonar_only.textproto", Error::kNone, true},
      {"observation_bundle_camera_only.textproto", Error::kNone, true},
      {"observation_bundle_all_modalities.textproto", Error::kNone, true},
      {"observation_bundle_excessive_skew.textproto", Error::kExcessiveSkew,
       false},
      {"observation_bundle_frame_mismatch.textproto", Error::kFrameMismatch,
       false},
      {"observation_bundle_time_reversal.textproto", Error::kTimeReversal,
       false},
      {"observation_bundle_missing_absent_reason.textproto", Error::kAbsentList,
       false},
      {"observation_bundle_bad_snapshot_id.textproto", Error::kSnapshotId,
       false},
  };
  for (const Case& item : cases) {
    const MultimodalObservationBundle parsed = ParseExample(item.name);
    const ObservationBundleContractAssessment assessment = Assess(parsed);
    EXPECT_EQ(assessment.error, item.error) << item.name;
    EXPECT_EQ(assessment.validity, ValidityKind::kValid) << item.name;
    EXPECT_EQ(assessment.accepted, item.accepted) << item.name;
  }
  const ObservationBundleContractAssessment skew =
      Assess(ParseExample("observation_bundle_excessive_skew.textproto"));
  EXPECT_TRUE(skew.measured_skew_present);
  EXPECT_EQ(skew.measured_skew.seconds, 0);
  EXPECT_EQ(skew.measured_skew.nanos, kFixtureMaxSkewNanos + 1);
  const ObservationBundleContractAssessment all =
      Assess(ParseExample("observation_bundle_all_modalities.textproto"));
  EXPECT_EQ(all.measured_skew.nanos, 180000000);
  EXPECT_TRUE(all.accepted);
}

TEST(ObservationBundleSerializationTest, DeterministicBytes) {
  MultimodalObservationBundle first;
  BuildMultimodalObservationBundle(SonarOnlyParts(), &first);
  MultimodalObservationBundle second;
  BuildMultimodalObservationBundle(SonarOnlyParts(), &second);
  EXPECT_EQ(first.SerializeAsString(), second.SerializeAsString());
  const ObservationBundleContractAssessment a = Assess(first);
  const ObservationBundleContractAssessment b = Assess(second);
  EXPECT_EQ(a.error, b.error);
  EXPECT_EQ(a.accepted, b.accepted);
  EXPECT_EQ(a.measured_skew.nanos, b.measured_skew.nanos);
  (*first.mutable_metadata())["note"] = "";
  (*first.mutable_metadata())["a"] = "1";
  (*second.mutable_metadata())["a"] = "1";
  (*second.mutable_metadata())["note"] = "";
  EXPECT_TRUE(Assess(first).accepted);
  EXPECT_TRUE(Assess(second).accepted);
  EXPECT_EQ(Assess(first).error, Assess(second).error);
}

}  // namespace
}  // namespace intrinsic::perception::sonar
