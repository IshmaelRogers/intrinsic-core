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
#include "intrinsic/perception/sonar/fls_frame_contract_policy.h"

namespace intrinsic::perception::sonar {
namespace {

using embodiment::ValidityKind;

constexpr double kNan = std::numeric_limits<double>::quiet_NaN();
constexpr double kInf = std::numeric_limits<double>::infinity();
constexpr float kFloatNan = std::numeric_limits<float>::quiet_NaN();
constexpr float kFloatInf = std::numeric_limits<float>::infinity();

FlsGeometryView NominalGeometry() {
  FlsGeometryView geometry;
  geometry.num_beams = 4;
  geometry.num_range_bins = 8;
  geometry.min_range_m = 0.5;
  geometry.max_range_m = 40.0;
  geometry.min_bearing_rad = -0.5;
  geometry.max_bearing_rad = 0.5;
  return geometry;
}

FlsCalibrationView NominalCalibration() {
  FlsCalibrationView calibration;
  calibration.present = true;
  calibration.parent_frame_id = "body";
  calibration.sensor_frame_id = "fls_sonar";
  calibration.pose_present = true;
  calibration.position = embodiment::Vec3{1.5, 0.0, -0.25};
  calibration.orientation = embodiment::Quaternion{0.0, 0.0, 0.0, 1.0};
  return calibration;
}

std::vector<float> Ramp(size_t count) {
  std::vector<float> samples(count);
  for (size_t i = 0; i < count; ++i) {
    samples[i] = 0.125f * static_cast<float>(i);
  }
  return samples;
}

FlsFrameView NominalInline(const std::vector<float>& samples) {
  FlsFrameView frame;
  frame.header_present = true;
  frame.validity_present = true;
  frame.validity_state = 1;
  frame.frame_id = "fls_sonar";
  frame.geometry = NominalGeometry();
  frame.calibration = NominalCalibration();
  frame.sound_speed_m_s = 1500.0;
  frame.payload_mode = FlsPayloadMode::kInline;
  frame.intensity = samples;
  frame.metadata_present = true;
  return frame;
}

FlsFrameView NominalBlob() {
  FlsFrameView frame = NominalInline({});
  frame.payload_mode = FlsPayloadMode::kBlob;
  frame.blob_id = "fls-blob-0001";
  frame.blob_byte_size_present = true;
  frame.blob_byte_size = 128;
  return frame;
}

FlsFrameContractError ErrorOf(const FlsFrameView& frame) {
  return AssessForwardLookingSonarFrame(frame).error;
}

TEST(FlsFrameContractTest, ErrorValuesAreLocked) {
  EXPECT_EQ(static_cast<int>(FlsFrameContractError::kNone), 0);
  EXPECT_EQ(static_cast<int>(FlsFrameContractError::kMissingFrame), 1);
  EXPECT_EQ(static_cast<int>(FlsFrameContractError::kGeometry), 2);
  EXPECT_EQ(static_cast<int>(FlsFrameContractError::kSoundSpeed), 3);
  EXPECT_EQ(static_cast<int>(FlsFrameContractError::kCalibration), 4);
  EXPECT_EQ(static_cast<int>(FlsFrameContractError::kMissingPayload), 5);
  EXPECT_EQ(static_cast<int>(FlsFrameContractError::kPayloadMismatch), 6);
  EXPECT_EQ(static_cast<int>(FlsFrameContractError::kNonFinite), 7);
  EXPECT_EQ(static_cast<int>(FlsFrameContractError::kBlobReference), 8);
  EXPECT_EQ(static_cast<int>(FlsFrameContractError::kQuaternion), 9);
}

TEST(FlsFrameContractTest, EmptyFrameIsNotEngaged) {
  const FlsFrameView empty;
  const FlsFrameContractAssessment assessment =
      AssessForwardLookingSonarFrame(empty);
  EXPECT_EQ(assessment.error, FlsFrameContractError::kNone);
  EXPECT_EQ(assessment.validity, ValidityKind::kAbsent);
  EXPECT_FALSE(assessment.accepted);
  EXPECT_FALSE(FlsFrameEngaged(empty));
}

TEST(FlsFrameContractTest, EachFieldEngagesTheFrame) {
  std::vector<FlsFrameView> engaged(13);
  engaged[0].header_present = true;
  engaged[1].geometry.num_beams = 1;
  engaged[2].geometry.num_range_bins = 1;
  engaged[3].geometry.min_range_m = 1.0;
  engaged[4].geometry.max_range_m = 1.0;
  engaged[5].geometry.min_bearing_rad = -1.0;
  engaged[6].geometry.max_bearing_rad = 1.0;
  engaged[7].sound_speed_m_s = 1500.0;
  engaged[8].sound_speed_m_s = kNan;
  engaged[9].calibration.present = true;
  engaged[10].payload_mode = FlsPayloadMode::kInline;
  engaged[11].payload_mode = FlsPayloadMode::kBlob;
  engaged[12].metadata_present = true;
  for (size_t i = 0; i < engaged.size(); ++i) {
    SCOPED_TRACE(i);
    EXPECT_TRUE(FlsFrameEngaged(engaged[i]));
    const FlsFrameContractAssessment assessment =
        AssessForwardLookingSonarFrame(engaged[i]);
    EXPECT_NE(assessment.error, FlsFrameContractError::kNone);
    EXPECT_FALSE(assessment.accepted);
  }
}

TEST(FlsFrameContractTest, NominalInlineAndBlobAreAccepted) {
  const std::vector<float> samples = Ramp(32);
  for (const FlsFrameView& frame : {NominalInline(samples), NominalBlob()}) {
    const FlsFrameContractAssessment assessment =
        AssessForwardLookingSonarFrame(frame);
    EXPECT_EQ(assessment.error, FlsFrameContractError::kNone);
    EXPECT_EQ(assessment.validity, ValidityKind::kValid);
    EXPECT_TRUE(assessment.accepted);
  }
}

TEST(FlsFrameContractTest, CalibrationAndMetadataAreOptional) {
  const std::vector<float> samples = Ramp(32);
  FlsFrameView bare = NominalInline(samples);
  bare.calibration = FlsCalibrationView{};
  bare.metadata_present = false;
  EXPECT_TRUE(AssessForwardLookingSonarFrame(bare).accepted);
  FlsFrameView bare_blob = NominalBlob();
  bare_blob.calibration = FlsCalibrationView{};
  bare_blob.metadata_present = false;
  EXPECT_TRUE(AssessForwardLookingSonarFrame(bare_blob).accepted);
}

TEST(FlsFrameContractTest, MetadataNeverChangesTheAssessment) {
  const std::vector<float> samples = Ramp(32);
  FlsFrameView frame = NominalInline(samples);
  const FlsFrameContractAssessment with_metadata =
      AssessForwardLookingSonarFrame(frame);
  frame.metadata_present = false;
  const FlsFrameContractAssessment without =
      AssessForwardLookingSonarFrame(frame);
  EXPECT_EQ(with_metadata.error, without.error);
  EXPECT_EQ(with_metadata.validity, without.validity);
  EXPECT_EQ(with_metadata.accepted, without.accepted);
  frame.sound_speed_m_s = -1.0;
  EXPECT_EQ(ErrorOf(frame), FlsFrameContractError::kSoundSpeed);
  frame.metadata_present = true;
  EXPECT_EQ(ErrorOf(frame), FlsFrameContractError::kSoundSpeed);
}

TEST(FlsFrameContractTest, ValidityIsClassifiedAndDoesNotRepairDefects) {
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
  const std::vector<float> samples = Ramp(32);
  for (const Case& c : cases) {
    FlsFrameView frame = NominalInline(samples);
    frame.validity_present = c.present;
    frame.validity_state = c.state;
    const FlsFrameContractAssessment assessment =
        AssessForwardLookingSonarFrame(frame);
    EXPECT_EQ(assessment.error, FlsFrameContractError::kNone);
    EXPECT_EQ(assessment.validity, c.kind);
    EXPECT_EQ(assessment.accepted, c.accepted);
  }
  FlsFrameView defective = NominalInline(samples);
  defective.sound_speed_m_s = 0.0;
  const FlsFrameContractAssessment assessment =
      AssessForwardLookingSonarFrame(defective);
  EXPECT_EQ(assessment.validity, ValidityKind::kValid);
  EXPECT_EQ(assessment.error, FlsFrameContractError::kSoundSpeed);
  EXPECT_FALSE(assessment.accepted);
}

TEST(FlsFrameContractTest, MissingFrame) {
  const std::vector<float> samples = Ramp(32);
  FlsFrameView frame = NominalInline(samples);
  frame.frame_id = "";
  EXPECT_EQ(ErrorOf(frame), FlsFrameContractError::kMissingFrame);
  FlsFrameView blob = NominalBlob();
  blob.frame_id = "";
  EXPECT_EQ(ErrorOf(blob), FlsFrameContractError::kMissingFrame);
  frame.header_present = false;
  EXPECT_EQ(ErrorOf(frame), FlsFrameContractError::kMissingFrame);
  for (const char* id : {"world_enu", "world_ned", "anything"}) {
    frame.frame_id = id;
    EXPECT_EQ(ErrorOf(frame), FlsFrameContractError::kNone);
  }
}

TEST(FlsFrameContractTest, BadGeometry) {
  using Mutator = void (*)(FlsGeometryView&);
  const Mutator mutators[] = {
      [](FlsGeometryView& g) { g.num_beams = 0; },
      [](FlsGeometryView& g) { g.num_range_bins = 0; },
      [](FlsGeometryView& g) { g.min_range_m = -0.001; },
      [](FlsGeometryView& g) { g.min_range_m = kNan; },
      [](FlsGeometryView& g) { g.min_range_m = -kInf; },
      [](FlsGeometryView& g) { g.max_range_m = kNan; },
      [](FlsGeometryView& g) { g.max_range_m = kInf; },
      [](FlsGeometryView& g) { g.max_range_m = 0.5; },
      [](FlsGeometryView& g) { g.max_range_m = 0.25; },
      [](FlsGeometryView& g) { g.min_bearing_rad = kNan; },
      [](FlsGeometryView& g) { g.min_bearing_rad = -kInf; },
      [](FlsGeometryView& g) { g.max_bearing_rad = kNan; },
      [](FlsGeometryView& g) { g.max_bearing_rad = kInf; },
      [](FlsGeometryView& g) { g.min_bearing_rad = 0.6; },
  };
  for (size_t i = 0; i < std::size(mutators); ++i) {
    SCOPED_TRACE(i);
    FlsFrameView frame = NominalBlob();
    mutators[i](frame.geometry);
    EXPECT_EQ(ErrorOf(frame), FlsFrameContractError::kGeometry);
  }
}

TEST(FlsFrameContractTest, GeometryBoundariesAreAccepted) {
  using Mutator = void (*)(FlsGeometryView&);
  const Mutator mutators[] = {
      [](FlsGeometryView& g) { g.min_range_m = 0.0; },
      [](FlsGeometryView& g) { g.min_bearing_rad = 0.5; },
      [](FlsGeometryView& g) {
        g.min_bearing_rad = 0.0;
        g.max_bearing_rad = 0.0;
      },
      [](FlsGeometryView& g) { g.max_range_m = 0.5000001; },
  };
  for (size_t i = 0; i < std::size(mutators); ++i) {
    SCOPED_TRACE(i);
    FlsFrameView frame = NominalBlob();
    mutators[i](frame.geometry);
    EXPECT_EQ(ErrorOf(frame), FlsFrameContractError::kNone);
  }
}

TEST(FlsFrameContractTest, BadSoundSpeed) {
  const std::vector<float> samples = Ramp(32);
  for (const double value : {0.0, -0.0, -1500.0, -1e-300, kNan, kInf, -kInf}) {
    SCOPED_TRACE(value);
    FlsFrameView inline_frame = NominalInline(samples);
    inline_frame.sound_speed_m_s = value;
    EXPECT_EQ(ErrorOf(inline_frame), FlsFrameContractError::kSoundSpeed);
    FlsFrameView blob = NominalBlob();
    blob.sound_speed_m_s = value;
    EXPECT_EQ(ErrorOf(blob), FlsFrameContractError::kSoundSpeed);
  }
  for (const double value : {1e-9, 1e9}) {
    FlsFrameView frame = NominalInline(samples);
    frame.sound_speed_m_s = value;
    EXPECT_EQ(ErrorOf(frame), FlsFrameContractError::kNone);
  }
}

TEST(FlsFrameContractTest, MissingPayload) {
  FlsFrameView frame = NominalInline({});
  frame.payload_mode = FlsPayloadMode::kNone;
  EXPECT_EQ(ErrorOf(frame), FlsFrameContractError::kMissingPayload);
}

TEST(FlsFrameContractTest, InlineSizeMismatch) {
  const std::vector<float> all = Ramp(96);
  for (const size_t size : {0, 1, 8, 31, 33, 64}) {
    SCOPED_TRACE(size);
    const std::vector<float> samples(all.begin(), all.begin() + size);
    EXPECT_EQ(ErrorOf(NominalInline(samples)),
              FlsFrameContractError::kPayloadMismatch);
  }
}

TEST(FlsFrameContractTest, InlineSingleSampleGrid) {
  const std::vector<float> one = {0.0f};
  const std::vector<float> two = {0.0f, 0.0f};
  FlsFrameView frame = NominalInline(one);
  frame.geometry.num_beams = 1;
  frame.geometry.num_range_bins = 1;
  EXPECT_EQ(ErrorOf(frame), FlsFrameContractError::kNone);
  frame.intensity = two;
  EXPECT_EQ(ErrorOf(frame), FlsFrameContractError::kPayloadMismatch);
}

TEST(FlsFrameContractTest, InlineLayoutUsesTheProductNotTheSum) {
  const std::vector<float> fifteen = Ramp(15);
  const std::vector<float> eight = Ramp(8);
  FlsFrameView frame = NominalInline(fifteen);
  frame.geometry.num_beams = 3;
  frame.geometry.num_range_bins = 5;
  EXPECT_EQ(ErrorOf(frame), FlsFrameContractError::kNone);
  frame.intensity = eight;
  EXPECT_EQ(ErrorOf(frame), FlsFrameContractError::kPayloadMismatch);
}

TEST(FlsFrameContractTest, InlineHugeDimensionsDoNotOverflow) {
  const std::vector<float> samples = Ramp(32);
  FlsFrameView frame = NominalInline(samples);
  frame.geometry.num_beams = std::numeric_limits<uint32_t>::max();
  frame.geometry.num_range_bins = std::numeric_limits<uint32_t>::max();
  uint64_t count = 0;
  ASSERT_TRUE(ExpectedSampleCount(frame.geometry, &count));
  EXPECT_EQ(count, static_cast<uint64_t>(std::numeric_limits<uint32_t>::max()) *
                       std::numeric_limits<uint32_t>::max());
  EXPECT_EQ(ErrorOf(frame), FlsFrameContractError::kPayloadMismatch);
}

TEST(FlsFrameContractTest, InlineNonFiniteSamples) {
  for (const float bad : {kFloatNan, kFloatInf, -kFloatInf}) {
    for (const size_t index : {0, 17, 31}) {
      SCOPED_TRACE(index);
      std::vector<float> samples(32, 0.5f);
      samples[index] = bad;
      EXPECT_EQ(ErrorOf(NominalInline(samples)),
                FlsFrameContractError::kNonFinite);
    }
  }
}

TEST(FlsFrameContractTest, SizeMismatchWinsOverNonFiniteSamples) {
  const std::vector<float> samples(31, kFloatNan);
  EXPECT_EQ(ErrorOf(NominalInline(samples)),
            FlsFrameContractError::kPayloadMismatch);
}

TEST(FlsFrameContractTest, ZeroAndNegativeIntensityAreFinite) {
  const std::vector<float> zeros(32, 0.0f);
  const std::vector<float> negatives(32, -1.0f);
  EXPECT_EQ(ErrorOf(NominalInline(zeros)), FlsFrameContractError::kNone);
  EXPECT_EQ(ErrorOf(NominalInline(negatives)), FlsFrameContractError::kNone);
}

TEST(FlsFrameContractTest, BlobReference) {
  FlsFrameView frame = NominalBlob();
  frame.blob_id = "";
  EXPECT_EQ(ErrorOf(frame), FlsFrameContractError::kBlobReference);
  frame = NominalBlob();
  frame.blob_byte_size = 0;
  EXPECT_EQ(ErrorOf(frame), FlsFrameContractError::kBlobReference);
  frame.blob_byte_size_present = false;
  EXPECT_EQ(ErrorOf(frame), FlsFrameContractError::kNone);
  frame.blob_byte_size_present = true;
  frame.blob_byte_size = 1;
  EXPECT_EQ(ErrorOf(frame), FlsFrameContractError::kNone);
  frame.blob_byte_size = std::numeric_limits<uint64_t>::max();
  EXPECT_EQ(ErrorOf(frame), FlsFrameContractError::kNone);
}

TEST(FlsFrameContractTest, BlobModeIgnoresStaleInlineValues) {
  const std::vector<float> stale = {kFloatNan};
  FlsFrameView frame = NominalBlob();
  frame.intensity = stale;
  EXPECT_EQ(ErrorOf(frame), FlsFrameContractError::kNone);
}

TEST(FlsFrameContractTest, InlineModeIgnoresStaleBlobValues) {
  const std::vector<float> samples = Ramp(32);
  FlsFrameView frame = NominalInline(samples);
  frame.blob_id = "";
  frame.blob_byte_size_present = true;
  frame.blob_byte_size = 0;
  EXPECT_EQ(ErrorOf(frame), FlsFrameContractError::kNone);
}

TEST(FlsFrameContractTest, BadCalibrationFrames) {
  using Mutator = void (*)(FlsCalibrationView&);
  const Mutator mutators[] = {
      [](FlsCalibrationView& c) { c.parent_frame_id = ""; },
      [](FlsCalibrationView& c) { c.sensor_frame_id = ""; },
      [](FlsCalibrationView& c) {
        c.parent_frame_id = "fls_sonar";
        c.sensor_frame_id = "fls_sonar";
      },
      [](FlsCalibrationView& c) { c.pose_present = false; },
  };
  for (size_t i = 0; i < std::size(mutators); ++i) {
    SCOPED_TRACE(i);
    FlsFrameView frame = NominalBlob();
    mutators[i](frame.calibration);
    EXPECT_EQ(ErrorOf(frame), FlsFrameContractError::kCalibration);
  }
}

TEST(FlsFrameContractTest, PresentEmptyCalibrationIsRejected) {
  FlsFrameView frame = NominalBlob();
  frame.calibration = FlsCalibrationView{};
  frame.calibration.present = true;
  EXPECT_EQ(ErrorOf(frame), FlsFrameContractError::kCalibration);
}

TEST(FlsFrameContractTest, CalibrationNonFinite) {
  using Mutator = void (*)(FlsCalibrationView&);
  const Mutator mutators[] = {
      [](FlsCalibrationView& c) { c.position.x = kNan; },
      [](FlsCalibrationView& c) { c.position.y = kInf; },
      [](FlsCalibrationView& c) { c.position.z = -kInf; },
      [](FlsCalibrationView& c) { c.orientation.x = kNan; },
      [](FlsCalibrationView& c) { c.orientation.w = kInf; },
  };
  for (size_t i = 0; i < std::size(mutators); ++i) {
    SCOPED_TRACE(i);
    FlsFrameView frame = NominalBlob();
    mutators[i](frame.calibration);
    EXPECT_EQ(ErrorOf(frame), FlsFrameContractError::kNonFinite);
  }
}

TEST(FlsFrameContractTest, CalibrationQuaternion) {
  const double half = std::sqrt(2.0) / 2.0;
  const std::array<embodiment::Quaternion, 4> bad = {
      embodiment::Quaternion{0, 0, 0, 0},
      embodiment::Quaternion{0, 0, 0, 2},
      embodiment::Quaternion{1, 1, 1, 1},
      embodiment::Quaternion{0, 0, 0, 1.000001},
  };
  for (const embodiment::Quaternion& orientation : bad) {
    FlsFrameView frame = NominalBlob();
    frame.calibration.orientation = orientation;
    EXPECT_EQ(ErrorOf(frame), FlsFrameContractError::kQuaternion);
  }
  const std::array<embodiment::Quaternion, 3> good = {
      embodiment::Quaternion{0, 0, 0, 1},
      embodiment::Quaternion{0, 0, 0, -1},
      embodiment::Quaternion{half, 0, 0, half},
  };
  for (const embodiment::Quaternion& orientation : good) {
    FlsFrameView frame = NominalBlob();
    frame.calibration.orientation = orientation;
    EXPECT_EQ(ErrorOf(frame), FlsFrameContractError::kNone);
  }
}

TEST(FlsFrameContractTest, FirstDefectWins) {
  const std::vector<float> empty;
  FlsFrameView frame = NominalInline(empty);
  frame.frame_id = "";
  frame.geometry.num_beams = 0;
  frame.sound_speed_m_s = -1.0;
  frame.payload_mode = FlsPayloadMode::kNone;
  frame.calibration.parent_frame_id = "";
  EXPECT_EQ(ErrorOf(frame), FlsFrameContractError::kMissingFrame);
  frame.frame_id = "fls_sonar";
  EXPECT_EQ(ErrorOf(frame), FlsFrameContractError::kGeometry);
  frame.geometry.num_beams = 4;
  EXPECT_EQ(ErrorOf(frame), FlsFrameContractError::kSoundSpeed);
  frame.sound_speed_m_s = 1500.0;
  EXPECT_EQ(ErrorOf(frame), FlsFrameContractError::kMissingPayload);
  frame.payload_mode = FlsPayloadMode::kInline;
  EXPECT_EQ(ErrorOf(frame), FlsFrameContractError::kPayloadMismatch);
  frame.payload_mode = FlsPayloadMode::kBlob;
  EXPECT_EQ(ErrorOf(frame), FlsFrameContractError::kBlobReference);
  frame.blob_id = "fls-blob-0001";
  EXPECT_EQ(ErrorOf(frame), FlsFrameContractError::kCalibration);
}

TEST(FlsFrameContractTest, AssessmentIsDeterministic) {
  const std::vector<float> samples = Ramp(32);
  const FlsFrameContractAssessment first =
      AssessForwardLookingSonarFrame(NominalInline(samples));
  for (int i = 0; i < 3; ++i) {
    const FlsFrameContractAssessment again =
        AssessForwardLookingSonarFrame(NominalInline(samples));
    EXPECT_EQ(again.error, first.error);
    EXPECT_EQ(again.validity, first.validity);
    EXPECT_EQ(again.accepted, first.accepted);
  }
}

}  // namespace
}  // namespace intrinsic::perception::sonar
