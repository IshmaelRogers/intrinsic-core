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

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <fstream>
#include <limits>
#include <sstream>
#include <string>
#include <string_view>
#include <vector>

#include "google/protobuf/descriptor.h"
#include "google/protobuf/text_format.h"
#include "gtest/gtest.h"
#include "intrinsic/embodiment/proto/stamped_header.pb.h"
#include "intrinsic/embodiment/stamped_header_policy.h"
#include "intrinsic/perception/proto/sonar/fls_frame.pb.h"
#include "intrinsic/perception/proto/sonar/sss_frame.pb.h"
#include "intrinsic/perception/sonar/sss_frame_contract_policy.h"

namespace intrinsic::perception::sonar {
namespace {

using embodiment::ValidityKind;
using intrinsic_proto::embodiment::Validity;
using intrinsic_proto::perception::sonar::SideScanSonarFrame;
using intrinsic_proto::perception::sonar::SssGeometry;

// Canonical serialization of FillInlineNominal(), which is also
// examples/sss_frame_inline_nominal.textproto. Keep in sync with
// sss_frame_contract_serialization_test.py.
constexpr std::string_view kInlineNominalGoldenHex =
    "0a38081f120b089ee2cfaa061080e59a771a06089fe2cfaa0622057373735f302a09737373"
    "5f"
    "736f6e617232096d6f6e6f746f6e69633a020801122c0a14081011000000000000f03f1900"
    "00"
    "000000c052401214081011000000000000f03f190000000000c052401a320a04626f647912"
    "09"
    "7373735f736f6e61721a1f0a1209000000000000e83f19000000000000e0bf120921000000"
    "00"
    "0000f03f2100000000007097402a84010a40000000000000003e0000803e0000c03e000000"
    "3f"
    "0000203f0000403f0000603f0000803f0000903f0000a03f0000b03f0000c03f0000d03f00"
    "00"
    "e03f0000f03f12400000803e0000c03e0000003f0000203f0000403f0000603f0000803f00"
    "00"
    "903f0000a03f0000b03f0000c03f0000d03f0000e03f0000f03f0000004000000840a20616"
    "0a"
    "0b736f6e61725f6d6f64656c120766697874757265";

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

void FillHeader(intrinsic_proto::embodiment::StampedHeader *header) {
  header->set_sequence(31);
  header->mutable_source_time()->set_seconds(1700000030);
  header->mutable_source_time()->set_nanos(250000000);
  header->mutable_receive_time()->set_seconds(1700000031);
  header->set_source_id("sss_0");
  header->set_frame_id("sss_sonar");
  header->set_clock_domain(std::string(embodiment::kClockDomainMonotonic));
  header->mutable_validity()->set_state(Validity::STATE_VALID);
}

void FillGeometry(SssGeometry *geometry) {
  for (auto *channel :
       {geometry->mutable_port(), geometry->mutable_starboard()}) {
    channel->set_num_samples(16);
    channel->set_min_range_m(1);
    channel->set_max_range_m(75);
  }
}

void FillCalibration(
    intrinsic_proto::perception::sonar::SssCalibration *calibration) {
  calibration->set_parent_frame_id("body");
  calibration->set_sensor_frame_id("sss_sonar");
  auto *pose = calibration->mutable_pose_parent_from_sensor();
  pose->mutable_position()->set_x(0.75);
  pose->mutable_position()->set_z(-0.5);
  pose->mutable_orientation()->set_w(1);
}

void FillInlineNominal(SideScanSonarFrame *frame) {
  FillHeader(frame->mutable_header());
  FillGeometry(frame->mutable_geometry());
  FillCalibration(frame->mutable_calibration());
  frame->set_sound_speed_m_s(1500);
  for (int i = 0; i < 16; ++i) {
    frame->mutable_inline_samples()->add_port_intensity(0.125f *
                                                        static_cast<float>(i));
    frame->mutable_inline_samples()->add_starboard_intensity(
        0.25f + 0.125f * static_cast<float>(i));
  }
  (*frame->mutable_metadata())["sonar_model"] = "fixture";
}

void FillBlobNominal(SideScanSonarFrame *frame) {
  FillHeader(frame->mutable_header());
  FillGeometry(frame->mutable_geometry());
  FillCalibration(frame->mutable_calibration());
  frame->set_sound_speed_m_s(1500);
  auto *blob = frame->mutable_blob_reference();
  blob->set_blob_id("sss-blob-0001");
  blob->set_content_type("application/octet-stream");
  blob->set_byte_size(256);
}

SssChannelGeometryView ChannelOf(
    const intrinsic_proto::perception::sonar::SssChannelGeometry &channel) {
  SssChannelGeometryView view;
  view.num_samples = channel.num_samples();
  view.min_range_m = channel.min_range_m();
  view.max_range_m = channel.max_range_m();
  return view;
}

struct Storage {
  std::vector<float> port;
  std::vector<float> starboard;
};

SssFrameView ViewOf(const SideScanSonarFrame &message, Storage *storage) {
  SssFrameView view;
  view.header_present = message.has_header();
  view.validity_present = message.header().has_validity();
  view.validity_state = static_cast<int>(message.header().validity().state());
  view.frame_id = message.header().frame_id();
  view.geometry.port = ChannelOf(message.geometry().port());
  view.geometry.starboard = ChannelOf(message.geometry().starboard());
  const auto &calibration = message.calibration();
  const auto &pose = calibration.pose_parent_from_sensor();
  view.calibration.present = message.has_calibration();
  view.calibration.parent_frame_id = calibration.parent_frame_id();
  view.calibration.sensor_frame_id = calibration.sensor_frame_id();
  view.calibration.pose_present = calibration.has_pose_parent_from_sensor();
  view.calibration.position = embodiment::Vec3{
      pose.position().x(), pose.position().y(), pose.position().z()};
  view.calibration.orientation =
      embodiment::Quaternion{pose.orientation().x(), pose.orientation().y(),
                             pose.orientation().z(), pose.orientation().w()};
  view.sound_speed_m_s = message.sound_speed_m_s();
  switch (message.payload_case()) {
  case SideScanSonarFrame::kInlineSamples:
    view.payload_mode = SssPayloadMode::kInline;
    break;
  case SideScanSonarFrame::kBlobReference:
    view.payload_mode = SssPayloadMode::kBlob;
    break;
  case SideScanSonarFrame::PAYLOAD_NOT_SET:
    view.payload_mode = SssPayloadMode::kNone;
    break;
  }
  const auto &port = message.inline_samples().port_intensity();
  const auto &starboard = message.inline_samples().starboard_intensity();
  storage->port.assign(port.begin(), port.end());
  storage->starboard.assign(starboard.begin(), starboard.end());
  view.port_intensity = storage->port;
  view.starboard_intensity = storage->starboard;
  const auto &blob = message.blob_reference();
  view.blob_id = blob.blob_id();
  view.blob_byte_size_present = blob.has_byte_size();
  view.blob_byte_size = blob.byte_size();
  view.metadata_present = message.metadata_size() > 0;
  return view;
}

SssFrameContractAssessment Assess(const SideScanSonarFrame &message) {
  Storage storage;
  return AssessSideScanSonarFrame(ViewOf(message, &storage));
}

SideScanSonarFrame RoundTrip(const SideScanSonarFrame &message) {
  SideScanSonarFrame parsed;
  EXPECT_TRUE(parsed.ParseFromString(message.SerializeAsString()));
  return parsed;
}

std::string LoadExample(const std::string &name) {
  const char *src = std::getenv("TEST_SRCDIR");
  if (src == nullptr) {
    return "";
  }
  const std::string suffix =
      "intrinsic/perception/proto/sonar/examples/" + name;
  const char *workspace = std::getenv("TEST_WORKSPACE");
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
  for (const std::string &path : candidates) {
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

SideScanSonarFrame ParseExample(const std::string &name) {
  const std::string text = LoadExample(name);
  EXPECT_FALSE(text.empty()) << name;
  SideScanSonarFrame parsed;
  EXPECT_TRUE(google::protobuf::TextFormat::ParseFromString(text, &parsed))
      << name;
  return parsed;
}

TEST(SssFrameContractSerializationTest, PackageAndLockedFieldNumbers) {
  const auto *descriptor = SideScanSonarFrame::descriptor();
  EXPECT_EQ(descriptor->file()->package(), "intrinsic_proto.perception.sonar");
  EXPECT_EQ(descriptor->field_count(), 7);
  EXPECT_EQ(descriptor->FindFieldByName("header")->number(), 1);
  EXPECT_EQ(descriptor->FindFieldByName("geometry")->number(), 2);
  EXPECT_EQ(descriptor->FindFieldByName("calibration")->number(), 3);
  EXPECT_EQ(descriptor->FindFieldByName("sound_speed_m_s")->number(), 4);
  EXPECT_EQ(descriptor->FindFieldByName("inline_samples")->number(), 5);
  EXPECT_EQ(descriptor->FindFieldByName("blob_reference")->number(), 6);
  EXPECT_EQ(descriptor->FindFieldByName("metadata")->number(), 100);
  ASSERT_EQ(descriptor->oneof_decl_count(), 1);
  EXPECT_EQ(descriptor->oneof_decl(0)->name(), "payload");
  EXPECT_EQ(descriptor->oneof_decl(0)->field_count(), 2);

  const auto *channel =
      intrinsic_proto::perception::sonar::SssChannelGeometry::descriptor();
  EXPECT_EQ(channel->field_count(), 3);
  EXPECT_EQ(channel->FindFieldByName("num_samples")->number(), 1);
  EXPECT_EQ(channel->FindFieldByName("min_range_m")->number(), 2);
  EXPECT_EQ(channel->FindFieldByName("max_range_m")->number(), 3);

  const auto *geometry = SssGeometry::descriptor();
  EXPECT_EQ(geometry->field_count(), 2);
  EXPECT_EQ(geometry->FindFieldByName("port")->number(), 1);
  EXPECT_EQ(geometry->FindFieldByName("starboard")->number(), 2);

  const auto *calibration =
      intrinsic_proto::perception::sonar::SssCalibration::descriptor();
  EXPECT_EQ(calibration->field_count(), 3);
  EXPECT_EQ(calibration->FindFieldByName("parent_frame_id")->number(), 1);
  EXPECT_EQ(calibration->FindFieldByName("sensor_frame_id")->number(), 2);
  EXPECT_EQ(calibration->FindFieldByName("pose_parent_from_sensor")->number(),
            3);

  const auto *inline_samples =
      intrinsic_proto::perception::sonar::SssInlineSamples::descriptor();
  EXPECT_EQ(inline_samples->field_count(), 2);
  EXPECT_EQ(inline_samples->FindFieldByName("port_intensity")->number(), 1);
  EXPECT_EQ(inline_samples->FindFieldByName("starboard_intensity")->number(),
            2);
}

TEST(SssFrameContractSerializationTest, ScopeIsSssOnly) {
  const auto *file = SideScanSonarFrame::descriptor()->file();
  EXPECT_EQ(file->message_type_count(), 5);
  EXPECT_EQ(file->enum_type_count(), 0);
  EXPECT_EQ(SideScanSonarFrame::descriptor()->FindFieldByName("robot_type"),
            nullptr);
}

TEST(SssFrameContractSerializationTest, BlobReferenceIsImportedFromFlsFrame) {
  const auto *field =
      SideScanSonarFrame::descriptor()->FindFieldByName("blob_reference");
  ASSERT_NE(field, nullptr);
  EXPECT_EQ(field->message_type(), intrinsic_proto::perception::sonar::
                                       SonarPayloadBlobReference::descriptor());
  EXPECT_EQ(field->message_type()->file()->name(),
            "intrinsic/perception/proto/sonar/fls_frame.proto");
  EXPECT_EQ(SideScanSonarFrame::descriptor()->file()->FindMessageTypeByName(
                "SonarPayloadBlobReference"),
            nullptr);
}

TEST(SssFrameContractSerializationTest, FlsFrameIsUnchanged) {
  const auto *descriptor = intrinsic_proto::perception::sonar::
      ForwardLookingSonarFrame::descriptor();
  EXPECT_EQ(descriptor->field_count(), 7);
  EXPECT_EQ(descriptor->FindFieldByName("header")->number(), 1);
  EXPECT_EQ(descriptor->FindFieldByName("geometry")->number(), 2);
  EXPECT_EQ(descriptor->FindFieldByName("calibration")->number(), 3);
  EXPECT_EQ(descriptor->FindFieldByName("sound_speed_m_s")->number(), 4);
  EXPECT_EQ(descriptor->FindFieldByName("inline_samples")->number(), 5);
  EXPECT_EQ(descriptor->FindFieldByName("blob_reference")->number(), 6);
  EXPECT_EQ(descriptor->FindFieldByName("metadata")->number(), 100);
}

TEST(SssFrameContractSerializationTest, DefaultIsZeroBytesAndNotAccepted) {
  SideScanSonarFrame empty;
  EXPECT_EQ(empty.SerializeAsString(), "");
  EXPECT_EQ(empty.ByteSizeLong(), 0u);
  EXPECT_FALSE(empty.has_header());
  EXPECT_FALSE(empty.has_geometry());
  EXPECT_FALSE(empty.has_calibration());
  EXPECT_EQ(empty.payload_case(), SideScanSonarFrame::PAYLOAD_NOT_SET);
  EXPECT_EQ(empty.sound_speed_m_s(), 0);
  EXPECT_EQ(empty.metadata_size(), 0);
  SideScanSonarFrame parsed;
  ASSERT_TRUE(parsed.ParseFromString(""));
  EXPECT_EQ(parsed.SerializeAsString(), "");
  const SssFrameContractAssessment assessment = Assess(parsed);
  EXPECT_EQ(assessment.error, SssFrameContractError::kNone);
  EXPECT_EQ(assessment.validity, ValidityKind::kAbsent);
  EXPECT_FALSE(assessment.accepted);
}

TEST(SssFrameContractSerializationTest, GoldenRoundTrip) {
  SideScanSonarFrame frame;
  FillInlineNominal(&frame);
  const std::string golden = FromHex(kInlineNominalGoldenHex);
  EXPECT_EQ(frame.SerializeAsString(), golden);

  SideScanSonarFrame parsed;
  ASSERT_TRUE(parsed.ParseFromString(golden));
  EXPECT_EQ(parsed.SerializeAsString(), golden);
  EXPECT_EQ(parsed.header().frame_id(), "sss_sonar");
  EXPECT_EQ(parsed.geometry().port().num_samples(), 16u);
  EXPECT_EQ(parsed.geometry().starboard().num_samples(), 16u);
  EXPECT_DOUBLE_EQ(parsed.sound_speed_m_s(), 1500);
  EXPECT_EQ(parsed.payload_case(), SideScanSonarFrame::kInlineSamples);
  EXPECT_EQ(parsed.inline_samples().port_intensity_size(), 16);
  EXPECT_EQ(parsed.inline_samples().starboard_intensity_size(), 16);
  EXPECT_FLOAT_EQ(parsed.inline_samples().port_intensity(15), 1.875f);
  EXPECT_FLOAT_EQ(parsed.inline_samples().starboard_intensity(15), 2.125f);
  EXPECT_EQ(parsed.metadata().at("sonar_model"), "fixture");
  const SssFrameContractAssessment assessment = Assess(parsed);
  EXPECT_EQ(assessment.error, SssFrameContractError::kNone);
  EXPECT_EQ(assessment.validity, ValidityKind::kValid);
  EXPECT_TRUE(assessment.accepted);
}

TEST(SssFrameContractSerializationTest, BlobRoundTrip) {
  SideScanSonarFrame frame;
  FillBlobNominal(&frame);
  const SideScanSonarFrame parsed = RoundTrip(frame);
  EXPECT_EQ(parsed.SerializeAsString(), frame.SerializeAsString());
  EXPECT_EQ(parsed.payload_case(), SideScanSonarFrame::kBlobReference);
  EXPECT_EQ(parsed.inline_samples().port_intensity_size(), 0);
  EXPECT_TRUE(Assess(parsed).accepted);
}

TEST(SssFrameContractSerializationTest, UnknownFieldIsPreserved) {
  std::string with_unknown = FromHex(kInlineNominalGoldenHex);
  // Field 101, varint 7. Field 100 is metadata, so this tag is unknown.
  with_unknown.append("\xA8\x06\x07", 3);
  SideScanSonarFrame parsed;
  ASSERT_TRUE(parsed.ParseFromString(with_unknown));
  EXPECT_EQ(parsed.header().frame_id(), "sss_sonar");
  EXPECT_EQ(parsed.SerializeAsString(), with_unknown);
  EXPECT_TRUE(Assess(parsed).accepted);
}

TEST(SssFrameContractSerializationTest,
     UnknownFieldInNestedMessageIsPreserved) {
  SssGeometry geometry;
  FillGeometry(&geometry);
  // Field 15, varint 1 inside SssGeometry.
  std::string nested = geometry.SerializeAsString();
  nested.append("\x78\x01", 2);
  std::string raw;
  raw.push_back('\x12');
  raw.push_back(static_cast<char>(nested.size()));
  raw += nested;
  SideScanSonarFrame parsed;
  ASSERT_TRUE(parsed.ParseFromString(raw));
  EXPECT_EQ(parsed.geometry().port().num_samples(), 16u);
  EXPECT_EQ(parsed.SerializeAsString(), raw);
}

TEST(SssFrameContractSerializationTest, UnknownFieldInChannelIsPreserved) {
  intrinsic_proto::perception::sonar::SssChannelGeometry channel;
  channel.set_num_samples(16);
  channel.set_min_range_m(1);
  channel.set_max_range_m(75);
  std::string nested_channel = channel.SerializeAsString();
  nested_channel.append("\x78\x01", 2);
  std::string port;
  port.push_back('\x0a');
  port.push_back(static_cast<char>(nested_channel.size()));
  port += nested_channel;
  std::string raw;
  raw.push_back('\x12');
  raw.push_back(static_cast<char>(port.size()));
  raw += port;
  SideScanSonarFrame parsed;
  ASSERT_TRUE(parsed.ParseFromString(raw));
  EXPECT_EQ(parsed.geometry().port().num_samples(), 16u);
  EXPECT_EQ(parsed.SerializeAsString(), raw);
}

TEST(SssFrameContractSerializationTest, MetadataShuffleStaysAccepted) {
  SideScanSonarFrame first;
  FillInlineNominal(&first);
  (*first.mutable_metadata())["a"] = "1";
  (*first.mutable_metadata())["note"] = "";
  SideScanSonarFrame second;
  FillInlineNominal(&second);
  (*second.mutable_metadata())["note"] = "";
  (*second.mutable_metadata())["a"] = "1";
  const SssFrameContractAssessment first_assessment = Assess(first);
  const SssFrameContractAssessment second_assessment = Assess(second);
  EXPECT_TRUE(first_assessment.accepted);
  EXPECT_EQ(first_assessment.error, second_assessment.error);
  EXPECT_EQ(first_assessment.validity, second_assessment.validity);
  EXPECT_EQ(first_assessment.accepted, second_assessment.accepted);
  (*first.mutable_metadata())[""] = "nan";
  EXPECT_TRUE(Assess(first).accepted);
}

TEST(SssFrameContractSerializationTest, MetadataAloneIsEngagedAndRejected) {
  SideScanSonarFrame frame;
  (*frame.mutable_metadata())["k"] = "v";
  const SssFrameContractAssessment assessment = Assess(frame);
  EXPECT_EQ(assessment.error, SssFrameContractError::kMissingFrame);
  EXPECT_FALSE(assessment.accepted);
}

TEST(SssFrameContractSerializationTest, PayloadOneofSwitchesModes) {
  SideScanSonarFrame frame;
  FillInlineNominal(&frame);
  frame.mutable_blob_reference()->set_blob_id("sss-blob-0001");
  EXPECT_EQ(frame.payload_case(), SideScanSonarFrame::kBlobReference);
  EXPECT_FALSE(frame.has_inline_samples());
  EXPECT_TRUE(Assess(frame).accepted);
  frame.mutable_inline_samples()->add_port_intensity(0.0f);
  EXPECT_EQ(frame.payload_case(), SideScanSonarFrame::kInlineSamples);
  EXPECT_FALSE(frame.has_blob_reference());
  EXPECT_EQ(Assess(frame).error, SssFrameContractError::kPayloadMismatch);
}

TEST(SssFrameContractSerializationTest, SelectedEmptyArmsSurviveTheWire) {
  SideScanSonarFrame inline_frame;
  FillInlineNominal(&inline_frame);
  inline_frame.mutable_inline_samples()->clear_port_intensity();
  inline_frame.mutable_inline_samples()->clear_starboard_intensity();
  SideScanSonarFrame parsed = RoundTrip(inline_frame);
  EXPECT_EQ(parsed.payload_case(), SideScanSonarFrame::kInlineSamples);
  EXPECT_EQ(Assess(parsed).error, SssFrameContractError::kPayloadMismatch);

  SideScanSonarFrame blob_frame;
  FillBlobNominal(&blob_frame);
  blob_frame.mutable_blob_reference()->Clear();
  parsed = RoundTrip(blob_frame);
  EXPECT_EQ(parsed.payload_case(), SideScanSonarFrame::kBlobReference);
  EXPECT_EQ(Assess(parsed).error, SssFrameContractError::kBlobReference);
}

TEST(SssFrameContractSerializationTest, MissingPayloadAfterTheWire) {
  SideScanSonarFrame frame;
  FillInlineNominal(&frame);
  frame.clear_inline_samples();
  EXPECT_EQ(Assess(RoundTrip(frame)).error,
            SssFrameContractError::kMissingPayload);
}

TEST(SssFrameContractSerializationTest, OneChannelMismatchSurvivesTheWire) {
  SideScanSonarFrame port;
  FillInlineNominal(&port);
  port.mutable_inline_samples()->mutable_port_intensity()->RemoveLast();
  EXPECT_EQ(Assess(RoundTrip(port)).error,
            SssFrameContractError::kPayloadMismatch);
  SideScanSonarFrame starboard;
  FillInlineNominal(&starboard);
  starboard.mutable_inline_samples()
      ->mutable_starboard_intensity()
      ->RemoveLast();
  EXPECT_EQ(Assess(RoundTrip(starboard)).error,
            SssFrameContractError::kPayloadMismatch);
}

TEST(SssFrameContractSerializationTest, BlobByteSizePresence) {
  SideScanSonarFrame frame;
  FillBlobNominal(&frame);
  frame.mutable_blob_reference()->clear_byte_size();
  const SideScanSonarFrame unset = RoundTrip(frame);
  EXPECT_FALSE(unset.blob_reference().has_byte_size());
  EXPECT_TRUE(Assess(unset).accepted);
  frame.mutable_blob_reference()->set_byte_size(0);
  const SideScanSonarFrame zero = RoundTrip(frame);
  EXPECT_TRUE(zero.blob_reference().has_byte_size());
  EXPECT_NE(zero.SerializeAsString(), unset.SerializeAsString());
  EXPECT_EQ(Assess(zero).error, SssFrameContractError::kBlobReference);
  frame.mutable_blob_reference()->set_content_type("");
  frame.mutable_blob_reference()->set_byte_size(1);
  EXPECT_TRUE(Assess(RoundTrip(frame)).accepted);
}

TEST(SssFrameContractSerializationTest, SoundSpeedSurvivesTheWire) {
  for (const double value :
       {0.0, -1.0, std::numeric_limits<double>::quiet_NaN(),
        std::numeric_limits<double>::infinity(),
        -std::numeric_limits<double>::infinity()}) {
    SideScanSonarFrame frame;
    FillBlobNominal(&frame);
    frame.set_sound_speed_m_s(value);
    EXPECT_EQ(Assess(RoundTrip(frame)).error,
              SssFrameContractError::kSoundSpeed)
        << value;
  }
}

TEST(SssFrameContractSerializationTest, NonFiniteSampleSurvivesTheWire) {
  for (const float value : {std::numeric_limits<float>::quiet_NaN(),
                            std::numeric_limits<float>::infinity(),
                            -std::numeric_limits<float>::infinity()}) {
    SideScanSonarFrame port;
    FillInlineNominal(&port);
    port.mutable_inline_samples()->set_port_intensity(7, value);
    const SideScanSonarFrame parsed_port = RoundTrip(port);
    EXPECT_FALSE(std::isfinite(parsed_port.inline_samples().port_intensity(7)));
    EXPECT_EQ(Assess(parsed_port).error, SssFrameContractError::kNonFinite);

    SideScanSonarFrame starboard;
    FillInlineNominal(&starboard);
    starboard.mutable_inline_samples()->set_starboard_intensity(7, value);
    const SideScanSonarFrame parsed_starboard = RoundTrip(starboard);
    EXPECT_FALSE(std::isfinite(
        parsed_starboard.inline_samples().starboard_intensity(7)));
    EXPECT_EQ(Assess(parsed_starboard).error,
              SssFrameContractError::kNonFinite);
  }
}

TEST(SssFrameContractSerializationTest, NonFiniteRangeSurvivesTheWire) {
  for (const double value : {std::numeric_limits<double>::quiet_NaN(),
                             std::numeric_limits<double>::infinity(),
                             -std::numeric_limits<double>::infinity()}) {
    SideScanSonarFrame frame;
    FillBlobNominal(&frame);
    frame.mutable_geometry()->mutable_starboard()->set_max_range_m(value);
    EXPECT_EQ(Assess(RoundTrip(frame)).error, SssFrameContractError::kGeometry);
  }
}

TEST(SssFrameContractSerializationTest, MissingChannelAfterTheWire) {
  SideScanSonarFrame frame;
  FillBlobNominal(&frame);
  frame.mutable_geometry()->clear_starboard();
  const SideScanSonarFrame parsed = RoundTrip(frame);
  EXPECT_FALSE(parsed.geometry().has_starboard());
  EXPECT_EQ(Assess(parsed).error, SssFrameContractError::kGeometry);
}

TEST(SssFrameContractSerializationTest,
     MissingCalibrationPoseIsACalibrationDefect) {
  SideScanSonarFrame frame;
  FillBlobNominal(&frame);
  frame.mutable_calibration()->clear_pose_parent_from_sensor();
  EXPECT_TRUE(frame.has_calibration());
  const SideScanSonarFrame parsed = RoundTrip(frame);
  EXPECT_FALSE(parsed.calibration().has_pose_parent_from_sensor());
  EXPECT_EQ(Assess(parsed).error, SssFrameContractError::kCalibration);
}

TEST(SssFrameContractSerializationTest, CalibrationZeroQuaternionOnTheWire) {
  SideScanSonarFrame frame;
  FillBlobNominal(&frame);
  frame.mutable_calibration()
      ->mutable_pose_parent_from_sensor()
      ->mutable_orientation()
      ->Clear();
  const SideScanSonarFrame parsed = RoundTrip(frame);
  EXPECT_TRUE(parsed.calibration().has_pose_parent_from_sensor());
  EXPECT_EQ(Assess(parsed).error, SssFrameContractError::kQuaternion);
}

TEST(SssFrameContractSerializationTest, ValidityDistinctionsSurviveTheWire) {
  SideScanSonarFrame frame;
  FillBlobNominal(&frame);
  frame.mutable_header()->clear_validity();
  SssFrameContractAssessment assessment = Assess(RoundTrip(frame));
  EXPECT_EQ(assessment.validity, ValidityKind::kAbsent);
  EXPECT_FALSE(assessment.accepted);
  frame.mutable_header()->mutable_validity()->set_state(
      Validity::STATE_INVALID);
  assessment = Assess(RoundTrip(frame));
  EXPECT_EQ(assessment.validity, ValidityKind::kInvalid);
  EXPECT_EQ(assessment.error, SssFrameContractError::kNone);
  EXPECT_FALSE(assessment.accepted);
  frame.mutable_header()->mutable_validity()->set_state(
      Validity::STATE_UNSPECIFIED);
  assessment = Assess(RoundTrip(frame));
  EXPECT_EQ(assessment.validity, ValidityKind::kUnspecified);
  EXPECT_FALSE(assessment.accepted);
}

TEST(SssFrameContractSerializationTest, ValidTextprotoFixtures) {
  const SideScanSonarFrame inline_frame =
      ParseExample("sss_frame_inline_nominal.textproto");
  SideScanSonarFrame expected_inline;
  FillInlineNominal(&expected_inline);
  EXPECT_EQ(inline_frame.SerializeAsString(),
            expected_inline.SerializeAsString());
  EXPECT_EQ(inline_frame.SerializeAsString(), FromHex(kInlineNominalGoldenHex));
  EXPECT_EQ(inline_frame.payload_case(), SideScanSonarFrame::kInlineSamples);
  EXPECT_EQ(inline_frame.inline_samples().port_intensity_size(), 16);
  EXPECT_EQ(inline_frame.inline_samples().starboard_intensity_size(), 16);
  EXPECT_DOUBLE_EQ(inline_frame.sound_speed_m_s(), 1500);
  EXPECT_TRUE(inline_frame.has_calibration());
  EXPECT_EQ(inline_frame.metadata().at("sonar_model"), "fixture");

  const SideScanSonarFrame blob_frame =
      ParseExample("sss_frame_blob_nominal.textproto");
  SideScanSonarFrame expected_blob;
  FillBlobNominal(&expected_blob);
  EXPECT_EQ(blob_frame.SerializeAsString(), expected_blob.SerializeAsString());
  EXPECT_EQ(blob_frame.payload_case(), SideScanSonarFrame::kBlobReference);
  EXPECT_EQ(blob_frame.geometry().SerializeAsString(),
            inline_frame.geometry().SerializeAsString());
  EXPECT_EQ(blob_frame.blob_reference().blob_id(), "sss-blob-0001");
  EXPECT_EQ(blob_frame.inline_samples().port_intensity_size(), 0);

  for (const SideScanSonarFrame *frame : {&inline_frame, &blob_frame}) {
    const SssFrameContractAssessment assessment = Assess(*frame);
    EXPECT_EQ(assessment.error, SssFrameContractError::kNone);
    EXPECT_EQ(assessment.validity, ValidityKind::kValid);
    EXPECT_TRUE(assessment.accepted);
  }
}

TEST(SssFrameContractSerializationTest, InvalidTextprotoFixtures) {
  struct Case {
    const char *name;
    SssFrameContractError error;
  };
  const Case cases[] = {
      {"sss_frame_bad_sound_speed.textproto",
       SssFrameContractError::kSoundSpeed},
      {"sss_frame_payload_mismatch.textproto",
       SssFrameContractError::kPayloadMismatch},
      {"sss_frame_bad_frame.textproto", SssFrameContractError::kMissingFrame},
      {"sss_frame_empty_blob_id.textproto",
       SssFrameContractError::kBlobReference},
  };
  for (const Case &c : cases) {
    SCOPED_TRACE(c.name);
    const SssFrameContractAssessment assessment = Assess(ParseExample(c.name));
    EXPECT_EQ(assessment.error, c.error);
    EXPECT_EQ(assessment.validity, ValidityKind::kValid);
    EXPECT_FALSE(assessment.accepted);
  }
}

TEST(SssFrameContractSerializationTest,
     InvalidFixturesDifferFromNominalByOneDefect) {
  EXPECT_LE(
      ParseExample("sss_frame_bad_sound_speed.textproto").sound_speed_m_s(), 0);
  EXPECT_EQ(ParseExample("sss_frame_bad_frame.textproto").header().frame_id(),
            "");
  const SideScanSonarFrame mismatch =
      ParseExample("sss_frame_payload_mismatch.textproto");
  EXPECT_EQ(
      static_cast<uint32_t>(mismatch.inline_samples().port_intensity_size()),
      mismatch.geometry().port().num_samples());
  EXPECT_NE(static_cast<uint32_t>(
                mismatch.inline_samples().starboard_intensity_size()),
            mismatch.geometry().starboard().num_samples());
  const SideScanSonarFrame empty_blob =
      ParseExample("sss_frame_empty_blob_id.textproto");
  EXPECT_EQ(empty_blob.payload_case(), SideScanSonarFrame::kBlobReference);
  EXPECT_EQ(empty_blob.blob_reference().blob_id(), "");
}

} // namespace
} // namespace intrinsic::perception::sonar
