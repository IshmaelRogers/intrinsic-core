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
#include "intrinsic/perception/sonar/fls_frame_contract_policy.h"

namespace intrinsic::perception::sonar {
namespace {

using embodiment::ValidityKind;
using intrinsic_proto::embodiment::Validity;
using intrinsic_proto::perception::sonar::FlsGeometry;
using intrinsic_proto::perception::sonar::ForwardLookingSonarFrame;

// Canonical serialization of FillInlineNominal(), which is also
// examples/fls_frame_inline_nominal.textproto. Keep in sync with
// fls_frame_contract_serialization_test.py.
constexpr std::string_view kInlineNominalGoldenHex =
    "0a380815120b0894e2cfaa061080e59a771a060895e2cfaa062205666c735f302a09666c73"
    "5f"
    "736f6e617232096d6f6e6f746f6e69633a02080112280804100819000000000000e03f2100"
    "00"
    "00000000444029000000000000e0bf31000000000000e03f1a320a04626f64791209666c73"
    "5f"
    "736f6e61721a1f0a1209000000000000f83f19000000000000d0bf120921000000000000f0"
    "3f"
    "2100000000007097402a83010a8001000000000000003e0000803e0000c03e0000003f0000"
    "20"
    "3f0000403f0000603f0000803f0000903f0000a03f0000b03f0000c03f0000d03f0000e03f"
    "00"
    "00f03f00000040000008400000104000001840000020400000284000003040000038400000"
    "40"
    "4000004840000050400000584000006040000068400000704000007840a206160a0b736f6e"
    "61"
    "725f6d6f64656c120766697874757265";

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

void FillHeader(intrinsic_proto::embodiment::StampedHeader* header) {
  header->set_sequence(21);
  header->mutable_source_time()->set_seconds(1700000020);
  header->mutable_source_time()->set_nanos(250000000);
  header->mutable_receive_time()->set_seconds(1700000021);
  header->set_source_id("fls_0");
  header->set_frame_id("fls_sonar");
  header->set_clock_domain(std::string(embodiment::kClockDomainMonotonic));
  header->mutable_validity()->set_state(Validity::STATE_VALID);
}

void FillGeometry(FlsGeometry* geometry) {
  geometry->set_num_beams(4);
  geometry->set_num_range_bins(8);
  geometry->set_min_range_m(0.5);
  geometry->set_max_range_m(40);
  geometry->set_min_bearing_rad(-0.5);
  geometry->set_max_bearing_rad(0.5);
}

void FillCalibration(
    intrinsic_proto::perception::sonar::FlsCalibration* calibration) {
  calibration->set_parent_frame_id("body");
  calibration->set_sensor_frame_id("fls_sonar");
  auto* pose = calibration->mutable_pose_parent_from_sensor();
  pose->mutable_position()->set_x(1.5);
  pose->mutable_position()->set_z(-0.25);
  pose->mutable_orientation()->set_w(1);
}

void FillInlineNominal(ForwardLookingSonarFrame* frame) {
  FillHeader(frame->mutable_header());
  FillGeometry(frame->mutable_geometry());
  FillCalibration(frame->mutable_calibration());
  frame->set_sound_speed_m_s(1500);
  for (int i = 0; i < 32; ++i) {
    frame->mutable_inline_samples()->add_intensity(0.125f *
                                                   static_cast<float>(i));
  }
  (*frame->mutable_metadata())["sonar_model"] = "fixture";
}

void FillBlobNominal(ForwardLookingSonarFrame* frame) {
  FillHeader(frame->mutable_header());
  FillGeometry(frame->mutable_geometry());
  FillCalibration(frame->mutable_calibration());
  frame->set_sound_speed_m_s(1500);
  auto* blob = frame->mutable_blob_reference();
  blob->set_blob_id("fls-blob-0001");
  blob->set_content_type("application/octet-stream");
  blob->set_byte_size(128);
}

FlsFrameView ViewOf(const ForwardLookingSonarFrame& message,
                    std::vector<float>* storage) {
  FlsFrameView view;
  view.header_present = message.has_header();
  view.validity_present = message.header().has_validity();
  view.validity_state = static_cast<int>(message.header().validity().state());
  view.frame_id = message.header().frame_id();
  const auto& geometry = message.geometry();
  view.geometry.num_beams = geometry.num_beams();
  view.geometry.num_range_bins = geometry.num_range_bins();
  view.geometry.min_range_m = geometry.min_range_m();
  view.geometry.max_range_m = geometry.max_range_m();
  view.geometry.min_bearing_rad = geometry.min_bearing_rad();
  view.geometry.max_bearing_rad = geometry.max_bearing_rad();
  const auto& calibration = message.calibration();
  const auto& pose = calibration.pose_parent_from_sensor();
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
    case ForwardLookingSonarFrame::kInlineSamples:
      view.payload_mode = FlsPayloadMode::kInline;
      break;
    case ForwardLookingSonarFrame::kBlobReference:
      view.payload_mode = FlsPayloadMode::kBlob;
      break;
    case ForwardLookingSonarFrame::PAYLOAD_NOT_SET:
      view.payload_mode = FlsPayloadMode::kNone;
      break;
  }
  const auto& samples = message.inline_samples().intensity();
  storage->assign(samples.begin(), samples.end());
  view.intensity = *storage;
  const auto& blob = message.blob_reference();
  view.blob_id = blob.blob_id();
  view.blob_byte_size_present = blob.has_byte_size();
  view.blob_byte_size = blob.byte_size();
  view.metadata_present = message.metadata_size() > 0;
  return view;
}

FlsFrameContractAssessment Assess(const ForwardLookingSonarFrame& message) {
  std::vector<float> storage;
  return AssessForwardLookingSonarFrame(ViewOf(message, &storage));
}

ForwardLookingSonarFrame RoundTrip(const ForwardLookingSonarFrame& message) {
  ForwardLookingSonarFrame parsed;
  EXPECT_TRUE(parsed.ParseFromString(message.SerializeAsString()));
  return parsed;
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

ForwardLookingSonarFrame ParseExample(const std::string& name) {
  const std::string text = LoadExample(name);
  EXPECT_FALSE(text.empty()) << name;
  ForwardLookingSonarFrame parsed;
  EXPECT_TRUE(google::protobuf::TextFormat::ParseFromString(text, &parsed))
      << name;
  return parsed;
}

TEST(FlsFrameContractSerializationTest, PackageAndLockedFieldNumbers) {
  const auto* descriptor = ForwardLookingSonarFrame::descriptor();
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

  const auto* geometry = FlsGeometry::descriptor();
  EXPECT_EQ(geometry->field_count(), 6);
  EXPECT_EQ(geometry->FindFieldByName("num_beams")->number(), 1);
  EXPECT_EQ(geometry->FindFieldByName("num_range_bins")->number(), 2);
  EXPECT_EQ(geometry->FindFieldByName("min_range_m")->number(), 3);
  EXPECT_EQ(geometry->FindFieldByName("max_range_m")->number(), 4);
  EXPECT_EQ(geometry->FindFieldByName("min_bearing_rad")->number(), 5);
  EXPECT_EQ(geometry->FindFieldByName("max_bearing_rad")->number(), 6);

  const auto* calibration =
      intrinsic_proto::perception::sonar::FlsCalibration::descriptor();
  EXPECT_EQ(calibration->field_count(), 3);
  EXPECT_EQ(calibration->FindFieldByName("parent_frame_id")->number(), 1);
  EXPECT_EQ(calibration->FindFieldByName("sensor_frame_id")->number(), 2);
  EXPECT_EQ(calibration->FindFieldByName("pose_parent_from_sensor")->number(),
            3);

  const auto* inline_samples =
      intrinsic_proto::perception::sonar::FlsInlineSamples::descriptor();
  EXPECT_EQ(inline_samples->field_count(), 1);
  EXPECT_EQ(inline_samples->FindFieldByName("intensity")->number(), 1);

  const auto* blob = intrinsic_proto::perception::sonar::
      SonarPayloadBlobReference::descriptor();
  EXPECT_EQ(blob->field_count(), 3);
  EXPECT_EQ(blob->FindFieldByName("blob_id")->number(), 1);
  EXPECT_EQ(blob->FindFieldByName("content_type")->number(), 2);
  EXPECT_EQ(blob->FindFieldByName("byte_size")->number(), 3);
}

TEST(FlsFrameContractSerializationTest, ScopeIsFlsOnly) {
  const auto* file = ForwardLookingSonarFrame::descriptor()->file();
  EXPECT_EQ(file->message_type_count(), 5);
  EXPECT_EQ(file->enum_type_count(), 0);
  EXPECT_EQ(
      ForwardLookingSonarFrame::descriptor()->FindFieldByName("robot_type"),
      nullptr);
}

TEST(FlsFrameContractSerializationTest, DefaultIsZeroBytesAndNotAccepted) {
  ForwardLookingSonarFrame empty;
  EXPECT_EQ(empty.SerializeAsString(), "");
  EXPECT_EQ(empty.ByteSizeLong(), 0u);
  EXPECT_FALSE(empty.has_header());
  EXPECT_FALSE(empty.has_geometry());
  EXPECT_FALSE(empty.has_calibration());
  EXPECT_EQ(empty.payload_case(), ForwardLookingSonarFrame::PAYLOAD_NOT_SET);
  EXPECT_EQ(empty.sound_speed_m_s(), 0);
  EXPECT_EQ(empty.metadata_size(), 0);
  ForwardLookingSonarFrame parsed;
  ASSERT_TRUE(parsed.ParseFromString(""));
  EXPECT_EQ(parsed.SerializeAsString(), "");
  const FlsFrameContractAssessment assessment = Assess(parsed);
  EXPECT_EQ(assessment.error, FlsFrameContractError::kNone);
  EXPECT_EQ(assessment.validity, ValidityKind::kAbsent);
  EXPECT_FALSE(assessment.accepted);
}

TEST(FlsFrameContractSerializationTest, GoldenRoundTrip) {
  ForwardLookingSonarFrame frame;
  FillInlineNominal(&frame);
  const std::string golden = FromHex(kInlineNominalGoldenHex);
  EXPECT_EQ(frame.SerializeAsString(), golden);

  ForwardLookingSonarFrame parsed;
  ASSERT_TRUE(parsed.ParseFromString(golden));
  EXPECT_EQ(parsed.SerializeAsString(), golden);
  EXPECT_EQ(parsed.header().frame_id(), "fls_sonar");
  EXPECT_EQ(parsed.geometry().num_beams(), 4u);
  EXPECT_EQ(parsed.geometry().num_range_bins(), 8u);
  EXPECT_DOUBLE_EQ(parsed.sound_speed_m_s(), 1500);
  EXPECT_EQ(parsed.payload_case(), ForwardLookingSonarFrame::kInlineSamples);
  EXPECT_EQ(parsed.inline_samples().intensity_size(), 32);
  EXPECT_FLOAT_EQ(parsed.inline_samples().intensity(31), 3.875f);
  EXPECT_EQ(parsed.metadata().at("sonar_model"), "fixture");
  const FlsFrameContractAssessment assessment = Assess(parsed);
  EXPECT_EQ(assessment.error, FlsFrameContractError::kNone);
  EXPECT_EQ(assessment.validity, ValidityKind::kValid);
  EXPECT_TRUE(assessment.accepted);
}

TEST(FlsFrameContractSerializationTest, BlobRoundTrip) {
  ForwardLookingSonarFrame frame;
  FillBlobNominal(&frame);
  const ForwardLookingSonarFrame parsed = RoundTrip(frame);
  EXPECT_EQ(parsed.SerializeAsString(), frame.SerializeAsString());
  EXPECT_EQ(parsed.payload_case(), ForwardLookingSonarFrame::kBlobReference);
  EXPECT_EQ(parsed.inline_samples().intensity_size(), 0);
  EXPECT_TRUE(Assess(parsed).accepted);
}

TEST(FlsFrameContractSerializationTest, UnknownFieldIsPreserved) {
  std::string with_unknown = FromHex(kInlineNominalGoldenHex);
  // Field 101, varint 7. Field 100 is metadata, so this tag is unknown.
  with_unknown.append("\xA8\x06\x07", 3);
  ForwardLookingSonarFrame parsed;
  ASSERT_TRUE(parsed.ParseFromString(with_unknown));
  EXPECT_EQ(parsed.header().frame_id(), "fls_sonar");
  EXPECT_EQ(parsed.SerializeAsString(), with_unknown);
  EXPECT_TRUE(Assess(parsed).accepted);
}

TEST(FlsFrameContractSerializationTest,
     UnknownFieldInNestedMessageIsPreserved) {
  FlsGeometry geometry;
  FillGeometry(&geometry);
  // Field 15, varint 1 inside FlsGeometry.
  std::string nested = geometry.SerializeAsString();
  nested.append("\x78\x01", 2);
  std::string raw;
  raw.push_back('\x12');
  raw.push_back(static_cast<char>(nested.size()));
  raw += nested;
  ForwardLookingSonarFrame parsed;
  ASSERT_TRUE(parsed.ParseFromString(raw));
  EXPECT_EQ(parsed.geometry().num_beams(), 4u);
  EXPECT_EQ(parsed.SerializeAsString(), raw);
}

TEST(FlsFrameContractSerializationTest, MetadataShuffleStaysAccepted) {
  ForwardLookingSonarFrame first;
  FillInlineNominal(&first);
  (*first.mutable_metadata())["a"] = "1";
  (*first.mutable_metadata())["note"] = "";
  ForwardLookingSonarFrame second;
  FillInlineNominal(&second);
  (*second.mutable_metadata())["note"] = "";
  (*second.mutable_metadata())["a"] = "1";
  const FlsFrameContractAssessment first_assessment = Assess(first);
  const FlsFrameContractAssessment second_assessment = Assess(second);
  EXPECT_TRUE(first_assessment.accepted);
  EXPECT_EQ(first_assessment.error, second_assessment.error);
  EXPECT_EQ(first_assessment.validity, second_assessment.validity);
  EXPECT_EQ(first_assessment.accepted, second_assessment.accepted);
  (*first.mutable_metadata())[""] = "nan";
  EXPECT_TRUE(Assess(first).accepted);
}

TEST(FlsFrameContractSerializationTest, MetadataAloneIsEngagedAndRejected) {
  ForwardLookingSonarFrame frame;
  (*frame.mutable_metadata())["k"] = "v";
  const FlsFrameContractAssessment assessment = Assess(frame);
  EXPECT_EQ(assessment.error, FlsFrameContractError::kMissingFrame);
  EXPECT_FALSE(assessment.accepted);
}

TEST(FlsFrameContractSerializationTest, PayloadOneofSwitchesModes) {
  ForwardLookingSonarFrame frame;
  FillInlineNominal(&frame);
  frame.mutable_blob_reference()->set_blob_id("fls-blob-0001");
  EXPECT_EQ(frame.payload_case(), ForwardLookingSonarFrame::kBlobReference);
  EXPECT_FALSE(frame.has_inline_samples());
  EXPECT_TRUE(Assess(frame).accepted);
  frame.mutable_inline_samples()->add_intensity(0.0f);
  EXPECT_EQ(frame.payload_case(), ForwardLookingSonarFrame::kInlineSamples);
  EXPECT_FALSE(frame.has_blob_reference());
  EXPECT_EQ(Assess(frame).error, FlsFrameContractError::kPayloadMismatch);
}

TEST(FlsFrameContractSerializationTest, SelectedEmptyArmsSurviveTheWire) {
  ForwardLookingSonarFrame inline_frame;
  FillInlineNominal(&inline_frame);
  inline_frame.mutable_inline_samples()->clear_intensity();
  ForwardLookingSonarFrame parsed = RoundTrip(inline_frame);
  EXPECT_EQ(parsed.payload_case(), ForwardLookingSonarFrame::kInlineSamples);
  EXPECT_EQ(Assess(parsed).error, FlsFrameContractError::kPayloadMismatch);

  ForwardLookingSonarFrame blob_frame;
  FillBlobNominal(&blob_frame);
  blob_frame.mutable_blob_reference()->Clear();
  parsed = RoundTrip(blob_frame);
  EXPECT_EQ(parsed.payload_case(), ForwardLookingSonarFrame::kBlobReference);
  EXPECT_EQ(Assess(parsed).error, FlsFrameContractError::kBlobReference);
}

TEST(FlsFrameContractSerializationTest, MissingPayloadAfterTheWire) {
  ForwardLookingSonarFrame frame;
  FillInlineNominal(&frame);
  frame.clear_inline_samples();
  EXPECT_EQ(Assess(RoundTrip(frame)).error,
            FlsFrameContractError::kMissingPayload);
}

TEST(FlsFrameContractSerializationTest, BlobByteSizePresence) {
  ForwardLookingSonarFrame frame;
  FillBlobNominal(&frame);
  frame.mutable_blob_reference()->clear_byte_size();
  const ForwardLookingSonarFrame unset = RoundTrip(frame);
  EXPECT_FALSE(unset.blob_reference().has_byte_size());
  EXPECT_TRUE(Assess(unset).accepted);
  frame.mutable_blob_reference()->set_byte_size(0);
  const ForwardLookingSonarFrame zero = RoundTrip(frame);
  EXPECT_TRUE(zero.blob_reference().has_byte_size());
  EXPECT_NE(zero.SerializeAsString(), unset.SerializeAsString());
  EXPECT_EQ(Assess(zero).error, FlsFrameContractError::kBlobReference);
  frame.mutable_blob_reference()->set_content_type("");
  frame.mutable_blob_reference()->set_byte_size(1);
  EXPECT_TRUE(Assess(RoundTrip(frame)).accepted);
}

TEST(FlsFrameContractSerializationTest, SoundSpeedSurvivesTheWire) {
  for (const double value :
       {0.0, -1.0, std::numeric_limits<double>::quiet_NaN(),
        std::numeric_limits<double>::infinity(),
        -std::numeric_limits<double>::infinity()}) {
    ForwardLookingSonarFrame frame;
    FillBlobNominal(&frame);
    frame.set_sound_speed_m_s(value);
    EXPECT_EQ(Assess(RoundTrip(frame)).error,
              FlsFrameContractError::kSoundSpeed)
        << value;
  }
}

TEST(FlsFrameContractSerializationTest, NonFiniteSampleSurvivesTheWire) {
  for (const float value : {std::numeric_limits<float>::quiet_NaN(),
                            std::numeric_limits<float>::infinity(),
                            -std::numeric_limits<float>::infinity()}) {
    ForwardLookingSonarFrame frame;
    FillInlineNominal(&frame);
    frame.mutable_inline_samples()->set_intensity(7, value);
    const ForwardLookingSonarFrame parsed = RoundTrip(frame);
    EXPECT_FALSE(std::isfinite(parsed.inline_samples().intensity(7)));
    EXPECT_EQ(Assess(parsed).error, FlsFrameContractError::kNonFinite);
  }
}

TEST(FlsFrameContractSerializationTest,
     MissingCalibrationPoseIsACalibrationDefect) {
  ForwardLookingSonarFrame frame;
  FillBlobNominal(&frame);
  frame.mutable_calibration()->clear_pose_parent_from_sensor();
  EXPECT_TRUE(frame.has_calibration());
  EXPECT_EQ(Assess(RoundTrip(frame)).error,
            FlsFrameContractError::kCalibration);
}

TEST(FlsFrameContractSerializationTest, CalibrationZeroQuaternionOnTheWire) {
  ForwardLookingSonarFrame frame;
  FillBlobNominal(&frame);
  frame.mutable_calibration()
      ->mutable_pose_parent_from_sensor()
      ->mutable_orientation()
      ->Clear();
  const ForwardLookingSonarFrame parsed = RoundTrip(frame);
  EXPECT_TRUE(parsed.calibration().has_pose_parent_from_sensor());
  EXPECT_EQ(Assess(parsed).error, FlsFrameContractError::kQuaternion);
}

TEST(FlsFrameContractSerializationTest, ValidityDistinctionsSurviveTheWire) {
  ForwardLookingSonarFrame frame;
  FillBlobNominal(&frame);
  frame.mutable_header()->clear_validity();
  FlsFrameContractAssessment assessment = Assess(RoundTrip(frame));
  EXPECT_EQ(assessment.validity, ValidityKind::kAbsent);
  EXPECT_FALSE(assessment.accepted);
  frame.mutable_header()->mutable_validity()->set_state(
      Validity::STATE_INVALID);
  assessment = Assess(RoundTrip(frame));
  EXPECT_EQ(assessment.validity, ValidityKind::kInvalid);
  EXPECT_EQ(assessment.error, FlsFrameContractError::kNone);
  EXPECT_FALSE(assessment.accepted);
  frame.mutable_header()->mutable_validity()->set_state(
      Validity::STATE_UNSPECIFIED);
  assessment = Assess(RoundTrip(frame));
  EXPECT_EQ(assessment.validity, ValidityKind::kUnspecified);
  EXPECT_FALSE(assessment.accepted);
}

TEST(FlsFrameContractSerializationTest, ValidTextprotoFixtures) {
  const ForwardLookingSonarFrame inline_frame =
      ParseExample("fls_frame_inline_nominal.textproto");
  ForwardLookingSonarFrame expected_inline;
  FillInlineNominal(&expected_inline);
  EXPECT_EQ(inline_frame.SerializeAsString(),
            expected_inline.SerializeAsString());
  EXPECT_EQ(inline_frame.SerializeAsString(), FromHex(kInlineNominalGoldenHex));
  EXPECT_EQ(inline_frame.payload_case(),
            ForwardLookingSonarFrame::kInlineSamples);
  EXPECT_EQ(inline_frame.inline_samples().intensity_size(), 32);
  EXPECT_DOUBLE_EQ(inline_frame.sound_speed_m_s(), 1500);
  EXPECT_TRUE(inline_frame.has_calibration());
  EXPECT_EQ(inline_frame.metadata().at("sonar_model"), "fixture");

  const ForwardLookingSonarFrame blob_frame =
      ParseExample("fls_frame_blob_nominal.textproto");
  ForwardLookingSonarFrame expected_blob;
  FillBlobNominal(&expected_blob);
  EXPECT_EQ(blob_frame.SerializeAsString(), expected_blob.SerializeAsString());
  EXPECT_EQ(blob_frame.payload_case(),
            ForwardLookingSonarFrame::kBlobReference);
  EXPECT_EQ(blob_frame.geometry().SerializeAsString(),
            inline_frame.geometry().SerializeAsString());
  EXPECT_EQ(blob_frame.blob_reference().blob_id(), "fls-blob-0001");
  EXPECT_EQ(blob_frame.inline_samples().intensity_size(), 0);

  for (const ForwardLookingSonarFrame* frame : {&inline_frame, &blob_frame}) {
    const FlsFrameContractAssessment assessment = Assess(*frame);
    EXPECT_EQ(assessment.error, FlsFrameContractError::kNone);
    EXPECT_EQ(assessment.validity, ValidityKind::kValid);
    EXPECT_TRUE(assessment.accepted);
  }
}

TEST(FlsFrameContractSerializationTest, InvalidTextprotoFixtures) {
  struct Case {
    const char* name;
    FlsFrameContractError error;
  };
  const Case cases[] = {
      {"fls_frame_bad_sound_speed.textproto",
       FlsFrameContractError::kSoundSpeed},
      {"fls_frame_payload_mismatch.textproto",
       FlsFrameContractError::kPayloadMismatch},
      {"fls_frame_bad_frame.textproto", FlsFrameContractError::kMissingFrame},
      {"fls_frame_empty_blob_id.textproto",
       FlsFrameContractError::kBlobReference},
  };
  for (const Case& c : cases) {
    SCOPED_TRACE(c.name);
    const FlsFrameContractAssessment assessment = Assess(ParseExample(c.name));
    EXPECT_EQ(assessment.error, c.error);
    EXPECT_EQ(assessment.validity, ValidityKind::kValid);
    EXPECT_FALSE(assessment.accepted);
  }
}

TEST(FlsFrameContractSerializationTest,
     InvalidFixturesDifferFromNominalByOneDefect) {
  EXPECT_LE(
      ParseExample("fls_frame_bad_sound_speed.textproto").sound_speed_m_s(), 0);
  EXPECT_EQ(ParseExample("fls_frame_bad_frame.textproto").header().frame_id(),
            "");
  const ForwardLookingSonarFrame mismatch =
      ParseExample("fls_frame_payload_mismatch.textproto");
  EXPECT_NE(static_cast<uint64_t>(mismatch.inline_samples().intensity_size()),
            static_cast<uint64_t>(mismatch.geometry().num_beams()) *
                mismatch.geometry().num_range_bins());
  const ForwardLookingSonarFrame empty_blob =
      ParseExample("fls_frame_empty_blob_id.textproto");
  EXPECT_EQ(empty_blob.payload_case(),
            ForwardLookingSonarFrame::kBlobReference);
  EXPECT_EQ(empty_blob.blob_reference().blob_id(), "");
}

}  // namespace
}  // namespace intrinsic::perception::sonar
