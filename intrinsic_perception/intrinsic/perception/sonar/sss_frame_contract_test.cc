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

#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <vector>

#include "gtest/gtest.h"
#include "intrinsic/embodiment/stamped_header_policy.h"
#include "intrinsic/perception/sonar/sss_frame_contract_policy.h"

namespace intrinsic::perception::sonar {
namespace {

using embodiment::ValidityKind;

constexpr double kNan = std::numeric_limits<double>::quiet_NaN();
constexpr double kInf = std::numeric_limits<double>::infinity();
constexpr float kFloatNan = std::numeric_limits<float>::quiet_NaN();
constexpr float kFloatInf = std::numeric_limits<float>::infinity();

SssChannelGeometryView NominalChannel() {
  SssChannelGeometryView channel;
  channel.num_samples = 16;
  channel.min_range_m = 1.0;
  channel.max_range_m = 75.0;
  return channel;
}

SssGeometryView NominalGeometry() {
  SssGeometryView geometry;
  geometry.port = NominalChannel();
  geometry.starboard = NominalChannel();
  return geometry;
}

SssCalibrationView NominalCalibration() {
  SssCalibrationView calibration;
  calibration.present = true;
  calibration.parent_frame_id = "body";
  calibration.sensor_frame_id = "sss_sonar";
  calibration.pose_present = true;
  calibration.position = embodiment::Vec3{0.75, 0.0, -0.5};
  calibration.orientation = embodiment::Quaternion{0.0, 0.0, 0.0, 1.0};
  return calibration;
}

std::vector<float> Ramp(size_t count, float offset = 0.0f) {
  std::vector<float> samples(count);
  for (size_t i = 0; i < count; ++i) {
    samples[i] = offset + 0.125f * static_cast<float>(i);
  }
  return samples;
}

struct Storage {
  std::vector<float> port = Ramp(16);
  std::vector<float> starboard = Ramp(16, 0.25f);
};

SssFrameView NominalInline(const Storage &storage) {
  SssFrameView frame;
  frame.header_present = true;
  frame.validity_present = true;
  frame.validity_state = 1;
  frame.frame_id = "sss_sonar";
  frame.geometry = NominalGeometry();
  frame.calibration = NominalCalibration();
  frame.sound_speed_m_s = 1500.0;
  frame.payload_mode = SssPayloadMode::kInline;
  frame.port_intensity = storage.port;
  frame.starboard_intensity = storage.starboard;
  frame.metadata_present = true;
  return frame;
}

SssFrameView NominalBlob() {
  SssFrameView frame = NominalInline(Storage{{}, {}});
  frame.payload_mode = SssPayloadMode::kBlob;
  frame.blob_id = "sss-blob-0001";
  frame.blob_byte_size_present = true;
  frame.blob_byte_size = 256;
  return frame;
}

SssFrameContractError ErrorOf(const SssFrameView &frame) {
  return AssessSideScanSonarFrame(frame).error;
}

TEST(SssFrameContractTest, ErrorValuesAreLocked) {
  EXPECT_EQ(static_cast<int>(SssFrameContractError::kNone), 0);
  EXPECT_EQ(static_cast<int>(SssFrameContractError::kMissingFrame), 1);
  EXPECT_EQ(static_cast<int>(SssFrameContractError::kGeometry), 2);
  EXPECT_EQ(static_cast<int>(SssFrameContractError::kSoundSpeed), 3);
  EXPECT_EQ(static_cast<int>(SssFrameContractError::kCalibration), 4);
  EXPECT_EQ(static_cast<int>(SssFrameContractError::kMissingPayload), 5);
  EXPECT_EQ(static_cast<int>(SssFrameContractError::kPayloadMismatch), 6);
  EXPECT_EQ(static_cast<int>(SssFrameContractError::kNonFinite), 7);
  EXPECT_EQ(static_cast<int>(SssFrameContractError::kBlobReference), 8);
  EXPECT_EQ(static_cast<int>(SssFrameContractError::kQuaternion), 9);
}

TEST(SssFrameContractTest, EmptyFrameIsNotEngaged) {
  const SssFrameView empty;
  const SssFrameContractAssessment assessment = AssessSideScanSonarFrame(empty);
  EXPECT_EQ(assessment.error, SssFrameContractError::kNone);
  EXPECT_EQ(assessment.validity, ValidityKind::kAbsent);
  EXPECT_FALSE(assessment.accepted);
  EXPECT_FALSE(SssFrameEngaged(empty));
}

TEST(SssFrameContractTest, EachFieldEngagesTheFrame) {
  std::vector<SssFrameView> engaged(18);
  engaged[0].header_present = true;
  engaged[1].geometry.port.num_samples = 1;
  engaged[2].geometry.port.min_range_m = 1.0;
  engaged[3].geometry.port.max_range_m = 1.0;
  engaged[4].geometry.starboard.num_samples = 1;
  engaged[5].geometry.starboard.min_range_m = 1.0;
  engaged[6].geometry.starboard.max_range_m = 1.0;
  engaged[7].sound_speed_m_s = 1500.0;
  engaged[8].sound_speed_m_s = kNan;
  engaged[9].calibration.present = true;
  engaged[10].payload_mode = SssPayloadMode::kInline;
  engaged[11].payload_mode = SssPayloadMode::kBlob;
  engaged[12].metadata_present = true;
  engaged[13].geometry.port.min_range_m = kNan;
  engaged[14].geometry.starboard.max_range_m = kInf;
  engaged[15].geometry.port.max_range_m = -1.0;
  engaged[16].geometry.starboard.min_range_m = -1.0;
  engaged[17].sound_speed_m_s = -1.0;
  for (const SssFrameView &frame : engaged) {
    EXPECT_TRUE(SssFrameEngaged(frame));
    const SssFrameContractAssessment assessment =
        AssessSideScanSonarFrame(frame);
    EXPECT_NE(assessment.error, SssFrameContractError::kNone);
    EXPECT_FALSE(assessment.accepted);
  }
}

TEST(SssFrameContractTest, NominalInlineAndBlobAreAccepted) {
  const Storage storage;
  for (const SssFrameView &frame : {NominalInline(storage), NominalBlob()}) {
    const SssFrameContractAssessment assessment =
        AssessSideScanSonarFrame(frame);
    EXPECT_EQ(assessment.error, SssFrameContractError::kNone);
    EXPECT_EQ(assessment.validity, ValidityKind::kValid);
    EXPECT_TRUE(assessment.accepted);
  }
}

TEST(SssFrameContractTest, CalibrationAndMetadataAreOptional) {
  const Storage storage;
  SssFrameView bare = NominalInline(storage);
  bare.calibration = SssCalibrationView{};
  bare.metadata_present = false;
  EXPECT_TRUE(AssessSideScanSonarFrame(bare).accepted);
  SssFrameView bare_blob = NominalBlob();
  bare_blob.calibration = SssCalibrationView{};
  bare_blob.metadata_present = false;
  EXPECT_TRUE(AssessSideScanSonarFrame(bare_blob).accepted);
}

TEST(SssFrameContractTest, MetadataNeverChangesTheAssessment) {
  const Storage storage;
  SssFrameView frame = NominalInline(storage);
  const SssFrameContractAssessment with_metadata =
      AssessSideScanSonarFrame(frame);
  frame.metadata_present = false;
  const SssFrameContractAssessment without = AssessSideScanSonarFrame(frame);
  EXPECT_EQ(with_metadata.error, without.error);
  EXPECT_EQ(with_metadata.validity, without.validity);
  EXPECT_EQ(with_metadata.accepted, without.accepted);
  frame.sound_speed_m_s = -1.0;
  EXPECT_EQ(ErrorOf(frame), SssFrameContractError::kSoundSpeed);
  frame.metadata_present = true;
  EXPECT_EQ(ErrorOf(frame), SssFrameContractError::kSoundSpeed);
}

TEST(SssFrameContractTest, ValidityIsClassifiedAndDoesNotRepairDefects) {
  struct Case {
    bool present;
    int state;
    ValidityKind kind;
    bool accepted;
  };
  const Case cases[] = {
      {false, 0, ValidityKind::kAbsent, false},
      {true, 0, ValidityKind::kUnspecified, false},
      {true, 1, ValidityKind::kValid, true},
      {true, 2, ValidityKind::kInvalid, false},
      {true, 99, ValidityKind::kUnspecified, false},
  };
  const Storage storage;
  for (const Case &c : cases) {
    SssFrameView frame = NominalInline(storage);
    frame.validity_present = c.present;
    frame.validity_state = c.state;
    const SssFrameContractAssessment assessment =
        AssessSideScanSonarFrame(frame);
    EXPECT_EQ(assessment.error, SssFrameContractError::kNone);
    EXPECT_EQ(assessment.validity, c.kind);
    EXPECT_EQ(assessment.accepted, c.accepted);
  }
  SssFrameView defective = NominalInline(storage);
  defective.sound_speed_m_s = 0.0;
  const SssFrameContractAssessment assessment =
      AssessSideScanSonarFrame(defective);
  EXPECT_EQ(assessment.validity, ValidityKind::kValid);
  EXPECT_EQ(assessment.error, SssFrameContractError::kSoundSpeed);
  EXPECT_FALSE(assessment.accepted);
}

TEST(SssFrameContractTest, MissingFrame) {
  const Storage storage;
  SssFrameView frame = NominalInline(storage);
  frame.frame_id = "";
  EXPECT_EQ(ErrorOf(frame), SssFrameContractError::kMissingFrame);
  SssFrameView blob = NominalBlob();
  blob.frame_id = "";
  EXPECT_EQ(ErrorOf(blob), SssFrameContractError::kMissingFrame);
  frame.header_present = false;
  EXPECT_EQ(ErrorOf(frame), SssFrameContractError::kMissingFrame);
}

TEST(SssFrameContractTest, BadGeometryOnEitherChannel) {
  using Mutation = void (*)(SssChannelGeometryView *);
  const Mutation mutations[] = {
      [](SssChannelGeometryView *c) { c->num_samples = 0; },
      [](SssChannelGeometryView *c) { c->min_range_m = -0.5; },
      [](SssChannelGeometryView *c) { c->min_range_m = 75.0; },
      [](SssChannelGeometryView *c) { c->min_range_m = 80.0; },
      [](SssChannelGeometryView *c) { c->min_range_m = kNan; },
      [](SssChannelGeometryView *c) { c->min_range_m = kInf; },
      [](SssChannelGeometryView *c) { c->min_range_m = -kInf; },
      [](SssChannelGeometryView *c) { c->max_range_m = kNan; },
      [](SssChannelGeometryView *c) { c->max_range_m = kInf; },
      [](SssChannelGeometryView *c) { c->max_range_m = 1.0; },
      [](SssChannelGeometryView *c) { c->max_range_m = 0.0; },
  };
  const Storage storage;
  for (const Mutation mutate : mutations) {
    SssFrameView port = NominalInline(storage);
    mutate(&port.geometry.port);
    EXPECT_EQ(ErrorOf(port), SssFrameContractError::kGeometry);
    SssFrameView starboard = NominalInline(storage);
    mutate(&starboard.geometry.starboard);
    EXPECT_EQ(ErrorOf(starboard), SssFrameContractError::kGeometry);
  }
}

TEST(SssFrameContractTest, MissingChannelIsAGeometryDefect) {
  const Storage storage;
  SssFrameView no_port = NominalInline(storage);
  no_port.geometry.port = SssChannelGeometryView{};
  EXPECT_EQ(ErrorOf(no_port), SssFrameContractError::kGeometry);
  SssFrameView no_starboard = NominalInline(storage);
  no_starboard.geometry.starboard = SssChannelGeometryView{};
  EXPECT_EQ(ErrorOf(no_starboard), SssFrameContractError::kGeometry);
  SssFrameView none = NominalInline(storage);
  none.geometry = SssGeometryView{};
  EXPECT_EQ(ErrorOf(none), SssFrameContractError::kGeometry);
}

TEST(SssFrameContractTest, GeometryBoundariesAreAccepted) {
  const std::vector<float> one = {0.5f};
  SssFrameView frame = NominalInline(Storage{{}, {}});
  frame.geometry.port = SssChannelGeometryView{1, 0.0, 1e-9};
  frame.geometry.starboard = SssChannelGeometryView{1, 0.0, 1e9};
  frame.port_intensity = one;
  frame.starboard_intensity = one;
  EXPECT_TRUE(AssessSideScanSonarFrame(frame).accepted);
}

TEST(SssFrameContractTest, ChannelsMayDiffer) {
  const std::vector<float> three = {0.0f, 1.0f, 2.0f};
  const std::vector<float> five = {0.0f, 1.0f, 2.0f, 3.0f, 4.0f};
  SssFrameView frame = NominalInline(Storage{{}, {}});
  frame.geometry.port.num_samples = 3;
  frame.geometry.starboard.num_samples = 5;
  frame.port_intensity = three;
  frame.starboard_intensity = five;
  EXPECT_TRUE(AssessSideScanSonarFrame(frame).accepted);
  frame.port_intensity = five;
  frame.starboard_intensity = three;
  EXPECT_EQ(ErrorOf(frame), SssFrameContractError::kPayloadMismatch);
}

TEST(SssFrameContractTest, BadSoundSpeed) {
  const Storage storage;
  for (const double value : {0.0, -1.0, -1500.0, kNan, kInf, -kInf}) {
    SssFrameView inline_frame = NominalInline(storage);
    inline_frame.sound_speed_m_s = value;
    EXPECT_EQ(ErrorOf(inline_frame), SssFrameContractError::kSoundSpeed)
        << value;
    SssFrameView blob = NominalBlob();
    blob.sound_speed_m_s = value;
    EXPECT_EQ(ErrorOf(blob), SssFrameContractError::kSoundSpeed) << value;
  }
  SssFrameView tiny = NominalInline(storage);
  tiny.sound_speed_m_s = 1e-300;
  EXPECT_TRUE(AssessSideScanSonarFrame(tiny).accepted);
}

TEST(SssFrameContractTest, MissingPayload) {
  const Storage storage;
  SssFrameView frame = NominalInline(storage);
  frame.payload_mode = SssPayloadMode::kNone;
  EXPECT_EQ(ErrorOf(frame), SssFrameContractError::kMissingPayload);
}

TEST(SssFrameContractTest, InlineSizeMismatchOnEitherChannel) {
  const std::vector<float> short_channel = Ramp(15);
  const std::vector<float> long_channel = Ramp(17);
  const std::vector<float> none;
  const Storage storage;
  struct Case {
    const std::vector<float> *port;
    const std::vector<float> *starboard;
  };
  const Case cases[] = {
      {&short_channel, &storage.starboard},
      {&long_channel, &storage.starboard},
      {&none, &storage.starboard},
      {&storage.port, &short_channel},
      {&storage.port, &long_channel},
      {&storage.port, &none},
      {&none, &none},
  };
  for (const Case &c : cases) {
    SssFrameView frame = NominalInline(storage);
    frame.port_intensity = *c.port;
    frame.starboard_intensity = *c.starboard;
    EXPECT_EQ(ErrorOf(frame), SssFrameContractError::kPayloadMismatch);
  }
}

TEST(SssFrameContractTest, InlineSizesAreNotPooled) {
  const std::vector<float> fifteen = Ramp(15);
  const std::vector<float> seventeen = Ramp(17);
  const Storage storage;
  SssFrameView frame = NominalInline(storage);
  frame.port_intensity = fifteen;
  frame.starboard_intensity = seventeen;
  EXPECT_EQ(ErrorOf(frame), SssFrameContractError::kPayloadMismatch);
}

TEST(SssFrameContractTest, InlineNonFiniteSamples) {
  for (const float value : {kFloatNan, kFloatInf, -kFloatInf}) {
    for (const size_t index : {size_t{0}, size_t{15}}) {
      Storage bad_port;
      bad_port.port[index] = value;
      EXPECT_EQ(ErrorOf(NominalInline(bad_port)),
                SssFrameContractError::kNonFinite);
      Storage bad_starboard;
      bad_starboard.starboard[index] = value;
      EXPECT_EQ(ErrorOf(NominalInline(bad_starboard)),
                SssFrameContractError::kNonFinite);
    }
  }
}

TEST(SssFrameContractTest, SizeMismatchWinsOverNonFiniteSamples) {
  const std::vector<float> nan_channel(16, kFloatNan);
  const std::vector<float> short_channel(15, 0.0f);
  const Storage storage;
  SssFrameView frame = NominalInline(storage);
  frame.port_intensity = nan_channel;
  frame.starboard_intensity = short_channel;
  EXPECT_EQ(ErrorOf(frame), SssFrameContractError::kPayloadMismatch);
  frame.port_intensity = short_channel;
  frame.starboard_intensity = nan_channel;
  EXPECT_EQ(ErrorOf(frame), SssFrameContractError::kPayloadMismatch);
}

TEST(SssFrameContractTest, ZeroAndNegativeIntensityAreFinite) {
  Storage storage;
  storage.port[0] = 0.0f;
  storage.port[1] = -0.0f;
  for (size_t i = 2; i < storage.port.size(); ++i) {
    storage.port[i] = -3.5f;
  }
  storage.starboard.assign(16, 0.0f);
  EXPECT_TRUE(AssessSideScanSonarFrame(NominalInline(storage)).accepted);
}

TEST(SssFrameContractTest, BlobReference) {
  SssFrameView empty_id = NominalBlob();
  empty_id.blob_id = "";
  EXPECT_EQ(ErrorOf(empty_id), SssFrameContractError::kBlobReference);
  SssFrameView zero_size = NominalBlob();
  zero_size.blob_byte_size = 0;
  EXPECT_EQ(ErrorOf(zero_size), SssFrameContractError::kBlobReference);
  SssFrameView unset_size = NominalBlob();
  unset_size.blob_byte_size_present = false;
  unset_size.blob_byte_size = 0;
  EXPECT_TRUE(AssessSideScanSonarFrame(unset_size).accepted);
  SssFrameView one = NominalBlob();
  one.blob_byte_size = 1;
  EXPECT_TRUE(AssessSideScanSonarFrame(one).accepted);
}

TEST(SssFrameContractTest, BlobModeIgnoresStaleInlineValues) {
  const std::vector<float> stale = {kFloatNan};
  SssFrameView frame = NominalBlob();
  frame.port_intensity = stale;
  EXPECT_TRUE(AssessSideScanSonarFrame(frame).accepted);
}

TEST(SssFrameContractTest, InlineModeIgnoresStaleBlobValues) {
  const Storage storage;
  SssFrameView frame = NominalInline(storage);
  frame.blob_id = "";
  frame.blob_byte_size_present = true;
  frame.blob_byte_size = 0;
  EXPECT_TRUE(AssessSideScanSonarFrame(frame).accepted);
}

TEST(SssFrameContractTest, BadCalibrationFrames) {
  struct Case {
    std::string_view parent;
    std::string_view sensor;
  };
  const Case cases[] = {
      {"", "sss_sonar"},
      {"body", ""},
      {"x", "x"},
      {"sss_sonar", "sss_sonar"},
  };
  const Storage storage;
  for (const Case &c : cases) {
    SssFrameView frame = NominalInline(storage);
    frame.calibration.parent_frame_id = c.parent;
    frame.calibration.sensor_frame_id = c.sensor;
    EXPECT_EQ(ErrorOf(frame), SssFrameContractError::kCalibration);
    SssFrameView blob = NominalBlob();
    blob.calibration.parent_frame_id = c.parent;
    blob.calibration.sensor_frame_id = c.sensor;
    EXPECT_EQ(ErrorOf(blob), SssFrameContractError::kCalibration);
  }
}

TEST(SssFrameContractTest, UnsetCalibrationPoseIsACalibrationDefect) {
  const Storage storage;
  SssFrameView frame = NominalInline(storage);
  frame.calibration.pose_present = false;
  EXPECT_EQ(ErrorOf(frame), SssFrameContractError::kCalibration);
  SssFrameView blob = NominalBlob();
  blob.calibration.pose_present = false;
  EXPECT_EQ(ErrorOf(blob), SssFrameContractError::kCalibration);
}

TEST(SssFrameContractTest, UnsetPoseWinsOverPoseValues) {
  const Storage storage;
  SssFrameView frame = NominalInline(storage);
  frame.calibration.pose_present = false;
  frame.calibration.position = embodiment::Vec3{kNan, 0.0, 0.0};
  frame.calibration.orientation = embodiment::Quaternion{0.0, 0.0, 0.0, 0.0};
  EXPECT_EQ(ErrorOf(frame), SssFrameContractError::kCalibration);
}

TEST(SssFrameContractTest, PresentEmptyCalibrationIsRejected) {
  const Storage storage;
  SssFrameView frame = NominalInline(storage);
  frame.calibration = SssCalibrationView{};
  frame.calibration.present = true;
  EXPECT_EQ(ErrorOf(frame), SssFrameContractError::kCalibration);
}

TEST(SssFrameContractTest, CalibrationNonFinite) {
  const Storage storage;
  const std::array<embodiment::Vec3, 3> positions = {
      embodiment::Vec3{kNan, 0.0, 0.0},
      embodiment::Vec3{0.0, kInf, 0.0},
      embodiment::Vec3{0.0, 0.0, -kInf},
  };
  for (const embodiment::Vec3 &position : positions) {
    SssFrameView frame = NominalInline(storage);
    frame.calibration.position = position;
    EXPECT_EQ(ErrorOf(frame), SssFrameContractError::kNonFinite);
  }
  const std::array<embodiment::Quaternion, 2> orientations = {
      embodiment::Quaternion{kNan, 0.0, 0.0, 1.0},
      embodiment::Quaternion{0.0, 0.0, 0.0, kInf},
  };
  for (const embodiment::Quaternion &orientation : orientations) {
    SssFrameView frame = NominalInline(storage);
    frame.calibration.orientation = orientation;
    EXPECT_EQ(ErrorOf(frame), SssFrameContractError::kNonFinite);
  }
}

TEST(SssFrameContractTest, CalibrationQuaternion) {
  const Storage storage;
  const double half = std::sqrt(0.5);
  const std::array<embodiment::Quaternion, 2> normalized = {
      embodiment::Quaternion{half, 0.0, 0.0, half},
      embodiment::Quaternion{0.0, 0.0, 0.0, -1.0},
  };
  for (const embodiment::Quaternion &orientation : normalized) {
    SssFrameView frame = NominalInline(storage);
    frame.calibration.orientation = orientation;
    EXPECT_EQ(ErrorOf(frame), SssFrameContractError::kNone);
  }
  const std::array<embodiment::Quaternion, 4> rejected = {
      embodiment::Quaternion{0.0, 0.0, 0.0, 0.0},
      embodiment::Quaternion{0.0, 0.0, 0.0, 2.0},
      embodiment::Quaternion{1.0, 1.0, 0.0, 0.0},
      embodiment::Quaternion{0.0, 0.0, 0.0, 0.5},
  };
  for (const embodiment::Quaternion &orientation : rejected) {
    SssFrameView frame = NominalInline(storage);
    frame.calibration.orientation = orientation;
    EXPECT_EQ(ErrorOf(frame), SssFrameContractError::kQuaternion);
  }
}

TEST(SssFrameContractTest, FirstDefectWins) {
  const Storage storage;
  SssFrameView frame = NominalInline(storage);
  frame.frame_id = "";
  frame.geometry = SssGeometryView{};
  frame.sound_speed_m_s = 0.0;
  frame.payload_mode = SssPayloadMode::kNone;
  frame.calibration.parent_frame_id = "";
  EXPECT_EQ(ErrorOf(frame), SssFrameContractError::kMissingFrame);
  frame.frame_id = "sss_sonar";
  EXPECT_EQ(ErrorOf(frame), SssFrameContractError::kGeometry);
  frame.geometry = NominalGeometry();
  EXPECT_EQ(ErrorOf(frame), SssFrameContractError::kSoundSpeed);
  frame.sound_speed_m_s = 1500.0;
  EXPECT_EQ(ErrorOf(frame), SssFrameContractError::kMissingPayload);
  frame.payload_mode = SssPayloadMode::kBlob;
  frame.blob_id = "";
  EXPECT_EQ(ErrorOf(frame), SssFrameContractError::kBlobReference);
  frame.blob_id = "sss-blob-0001";
  EXPECT_EQ(ErrorOf(frame), SssFrameContractError::kCalibration);
  frame.calibration = NominalCalibration();
  EXPECT_EQ(ErrorOf(frame), SssFrameContractError::kNone);
}

TEST(SssFrameContractTest, PayloadDefectWinsOverCalibrationDefect) {
  const Storage storage;
  SssFrameView frame = NominalInline(storage);
  frame.port_intensity = {};
  frame.calibration.pose_present = false;
  EXPECT_EQ(ErrorOf(frame), SssFrameContractError::kPayloadMismatch);
}

TEST(SssFrameContractTest, AssessmentIsDeterministic) {
  const Storage storage;
  const SssFrameView frame = NominalInline(storage);
  const SssFrameContractAssessment first = AssessSideScanSonarFrame(frame);
  const SssFrameContractAssessment second = AssessSideScanSonarFrame(frame);
  EXPECT_EQ(first.error, second.error);
  EXPECT_EQ(first.validity, second.validity);
  EXPECT_EQ(first.accepted, second.accepted);
}

} // namespace
} // namespace intrinsic::perception::sonar
