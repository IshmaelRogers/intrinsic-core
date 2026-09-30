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

#include "intrinsic/hardware/marine/measurement_health_policy.h"

#include <array>
#include <cmath>
#include <limits>
#include <optional>

#include "gtest/gtest.h"
#include "intrinsic/embodiment/frame_policy.h"
#include "intrinsic/embodiment/stamped_header_policy.h"
#include "intrinsic/vehicle/vehicle_contract_policy.h"

namespace intrinsic::hardware::marine {
namespace {

using embodiment::ValidityKind;

MeasurementHealthView ValidSample() {
  MeasurementHealthView sample;
  sample.header_present = true;
  sample.frame_id = "sensor";
  sample.source_time_present = true;
  sample.source_time = embodiment::ClockReading{1700000000, 250000000};
  sample.receive_time_present = true;
  sample.receive_time = embodiment::ClockReading{1700000001, 0};
  sample.header_validity_present = true;
  sample.header_validity_state = 1;
  sample.state_present = true;
  sample.state = 1;
  sample.quality_present = true;
  sample.quality = 0.75;
  return sample;
}

TEST(MeasurementHealthPolicyTest, EmptySampleIsOptInAndNotAccepted) {
  const MeasurementAssessment assessment =
      AssessMeasurementHealth(MeasurementHealthView{});
  EXPECT_EQ(assessment.error, MeasurementError::kNone);
  EXPECT_EQ(assessment.state, MeasurementStateKind::kAbsent);
  EXPECT_EQ(assessment.header_validity, ValidityKind::kAbsent);
  EXPECT_FALSE(assessment.accepted);
}

TEST(MeasurementHealthPolicyTest, NominalValidSampleIsAccepted) {
  const MeasurementAssessment assessment =
      AssessMeasurementHealth(ValidSample());
  EXPECT_EQ(assessment.error, MeasurementError::kNone);
  EXPECT_EQ(assessment.state, MeasurementStateKind::kValid);
  EXPECT_EQ(assessment.header_validity, ValidityKind::kValid);
  EXPECT_TRUE(assessment.accepted);
}

TEST(MeasurementHealthPolicyTest,
     LocalStatesStayDistinctFromEmbodimentValidity) {
  EXPECT_EQ(ClassifyMeasurementState(false, 0), MeasurementStateKind::kAbsent);
  EXPECT_EQ(ClassifyMeasurementState(true, 0), MeasurementStateKind::kUnknown);
  EXPECT_EQ(ClassifyMeasurementState(true, 1), MeasurementStateKind::kValid);
  EXPECT_EQ(ClassifyMeasurementState(true, 2), MeasurementStateKind::kDegraded);
  EXPECT_EQ(ClassifyMeasurementState(true, 3), MeasurementStateKind::kInvalid);
  EXPECT_EQ(ClassifyMeasurementState(true, 100),
            MeasurementStateKind::kUnrecognized);
  EXPECT_NE(ClassifyMeasurementState(true, 0), MeasurementStateKind::kAbsent);
  EXPECT_NE(ClassifyMeasurementState(true, 100),
            MeasurementStateKind::kInvalid);

  // Embodiment state 3 is not a degraded or invalid value. The measurement
  // enum uses 3 for INVALID. The shared Validity enum is unchanged.
  EXPECT_EQ(embodiment::ClassifyValidity(true, 3), ValidityKind::kUnspecified);
  EXPECT_NE(embodiment::ClassifyValidity(true, 3), ValidityKind::kInvalid);
  EXPECT_EQ(embodiment::ClassifyValidity(false, 2), ValidityKind::kAbsent);
  EXPECT_FALSE(embodiment::SampleAccepted(ValidityKind::kAbsent, true));
}

TEST(MeasurementHealthPolicyTest,
     UnknownDegradedAndInvalidFollowDocumentedStatus) {
  MeasurementHealthView unknown = ValidSample();
  unknown.state = 0;
  const MeasurementAssessment unknown_assessment =
      AssessMeasurementHealth(unknown);
  EXPECT_EQ(unknown_assessment.error, MeasurementError::kNone);
  EXPECT_EQ(unknown_assessment.state, MeasurementStateKind::kUnknown);
  EXPECT_FALSE(unknown_assessment.accepted);

  MeasurementHealthView degraded = ValidSample();
  degraded.state = 2;
  const MeasurementAssessment degraded_assessment =
      AssessMeasurementHealth(degraded);
  EXPECT_EQ(degraded_assessment.error, MeasurementError::kNone);
  EXPECT_EQ(degraded_assessment.state, MeasurementStateKind::kDegraded);
  EXPECT_FALSE(degraded_assessment.accepted);

  MeasurementHealthView dropout = ValidSample();
  dropout.state = 3;
  const MeasurementAssessment dropout_assessment =
      AssessMeasurementHealth(dropout);
  EXPECT_EQ(dropout_assessment.error, MeasurementError::kNone);
  EXPECT_EQ(dropout_assessment.state, MeasurementStateKind::kInvalid);
  EXPECT_FALSE(dropout_assessment.accepted);

  MeasurementHealthView omitted;
  EXPECT_EQ(AssessMeasurementHealth(omitted).state,
            MeasurementStateKind::kAbsent);
  EXPECT_FALSE(AssessMeasurementHealth(omitted).accepted);
}

TEST(MeasurementHealthPolicyTest, AbsentJudgmentIsNotUnknown) {
  MeasurementHealthView sample = ValidSample();
  sample.state_present = false;
  sample.state = 0;
  const MeasurementAssessment assessment = AssessMeasurementHealth(sample);
  EXPECT_EQ(assessment.state, MeasurementStateKind::kAbsent);
  EXPECT_EQ(assessment.error, MeasurementError::kNone);
  EXPECT_FALSE(assessment.accepted);
}

TEST(MeasurementHealthPolicyTest,
     HeaderValidityDoesNotReplaceMeasurementState) {
  MeasurementHealthView sample = ValidSample();
  sample.header_validity_state = 2;
  const MeasurementAssessment invalid_header = AssessMeasurementHealth(sample);
  EXPECT_TRUE(invalid_header.accepted);
  EXPECT_EQ(invalid_header.state, MeasurementStateKind::kValid);
  EXPECT_EQ(invalid_header.header_validity, ValidityKind::kInvalid);

  sample.header_validity_present = false;
  sample.state = 3;
  const MeasurementAssessment invalid_measurement =
      AssessMeasurementHealth(sample);
  EXPECT_FALSE(invalid_measurement.accepted);
  EXPECT_EQ(invalid_measurement.state, MeasurementStateKind::kInvalid);
  EXPECT_EQ(invalid_measurement.header_validity, ValidityKind::kAbsent);
}

TEST(MeasurementHealthPolicyTest, UnrecognizedStateIsKeptAndNotAccepted) {
  MeasurementHealthView sample = ValidSample();
  sample.state = 100;
  const MeasurementAssessment assessment = AssessMeasurementHealth(sample);
  EXPECT_EQ(assessment.error, MeasurementError::kNone);
  EXPECT_EQ(assessment.state, MeasurementStateKind::kUnrecognized);
  EXPECT_FALSE(assessment.accepted);
  EXPECT_EQ(sample.state, 100);
}

TEST(MeasurementHealthPolicyTest, QualityBoundsAreClosedAndFinite) {
  MeasurementHealthView low = ValidSample();
  low.quality = 0.0;
  EXPECT_TRUE(AssessMeasurementHealth(low).accepted);
  MeasurementHealthView high = ValidSample();
  high.quality = 1.0;
  EXPECT_TRUE(AssessMeasurementHealth(high).accepted);

  MeasurementHealthView absent = ValidSample();
  absent.quality_present = false;
  EXPECT_TRUE(AssessMeasurementHealth(absent).accepted);

  MeasurementHealthView below = ValidSample();
  below.quality = std::nextafter(0.0, -1.0);
  EXPECT_EQ(AssessMeasurementHealth(below).error, MeasurementError::kQuality);

  MeasurementHealthView above = ValidSample();
  above.quality = std::nextafter(1.0, 2.0);
  EXPECT_EQ(AssessMeasurementHealth(above).error, MeasurementError::kQuality);

  MeasurementHealthView nan = ValidSample();
  nan.quality = std::numeric_limits<double>::quiet_NaN();
  const MeasurementAssessment nan_assessment = AssessMeasurementHealth(nan);
  EXPECT_EQ(nan_assessment.error, MeasurementError::kNonFinite);
  EXPECT_EQ(nan_assessment.state, MeasurementStateKind::kValid);
  EXPECT_FALSE(nan_assessment.accepted);

  MeasurementHealthView infinite = ValidSample();
  infinite.quality = std::numeric_limits<double>::infinity();
  EXPECT_EQ(AssessMeasurementHealth(infinite).error,
            MeasurementError::kNonFinite);
}

TEST(MeasurementHealthPolicyTest, FrameMustBeExplicitAndCanBeChecked) {
  MeasurementHealthView missing = ValidSample();
  missing.frame_id = "";
  EXPECT_EQ(AssessMeasurementHealth(missing).error,
            MeasurementError::kMissingFrame);

  MeasurementHealthView wrong = ValidSample();
  wrong.expected_frame_present = true;
  wrong.expected_frame = embodiment::kWorldEnuFrameId;
  EXPECT_EQ(AssessMeasurementHealth(wrong).error,
            MeasurementError::kWrongFrame);

  wrong.frame_id = "World_enu";
  EXPECT_EQ(AssessMeasurementHealth(wrong).error,
            MeasurementError::kWrongFrame);

  wrong.frame_id = embodiment::kWorldEnuFrameId;
  EXPECT_TRUE(AssessMeasurementHealth(wrong).accepted);
  EXPECT_TRUE(
      embodiment::FrameIdMatches(wrong.frame_id, embodiment::WorldFrame::kEnu));
  EXPECT_FALSE(embodiment::FrameIdMatches("MeasurementHealth",
                                          embodiment::WorldFrame::kEnu));

  MeasurementHealthView body = ValidSample();
  body.frame_id = "body";
  EXPECT_TRUE(AssessMeasurementHealth(body).accepted);
}

TEST(MeasurementHealthPolicyTest, SourceTimeReversalIsRejectedAndDelayIsNot) {
  MeasurementHealthView equal_time = ValidSample();
  equal_time.receive_time = equal_time.source_time;
  EXPECT_TRUE(AssessMeasurementHealth(equal_time).accepted);

  MeasurementHealthView reversed = ValidSample();
  reversed.receive_time = embodiment::ClockReading{1700000000, 249999999};
  EXPECT_EQ(AssessMeasurementHealth(reversed).error,
            MeasurementError::kTimeReversal);

  MeasurementHealthView delayed = ValidSample();
  delayed.receive_time = embodiment::ClockReading{1700000100, 0};
  EXPECT_TRUE(AssessMeasurementHealth(delayed).accepted);
  const std::optional<double> age =
      embodiment::MonotonicAgeSeconds(delayed.source_time, delayed.receive_time,
                                      embodiment::kClockDomainMonotonic);
  ASSERT_TRUE(age.has_value());
  EXPECT_GT(*age, 0.75);

  MeasurementHealthView one_stamp = ValidSample();
  one_stamp.receive_time_present = false;
  EXPECT_TRUE(AssessMeasurementHealth(one_stamp).accepted);
}

TEST(MeasurementHealthPolicyTest, CovarianceAbsenceIsNotZero) {
  MeasurementHealthView absent = ValidSample();
  EXPECT_FALSE(absent.covariance_present);
  EXPECT_TRUE(AssessMeasurementHealth(absent).accepted);
  EXPECT_TRUE(vehicle::CovarianceIsUnknown(
      vehicle::AssessCovariance(false, absent.covariance)));

  const std::array<double, vehicle::kCovarianceValues> zeros = {};
  MeasurementHealthView present = ValidSample();
  present.covariance_present = true;
  present.covariance = zeros;
  EXPECT_TRUE(AssessMeasurementHealth(present).accepted);
  EXPECT_TRUE(vehicle::IsAllZeroCovariance(zeros));
  EXPECT_FALSE(
      vehicle::CovarianceIsUnknown(vehicle::AssessCovariance(true, zeros)));

  const std::array<double, 35> short_values = {};
  MeasurementHealthView wrong_length = ValidSample();
  wrong_length.covariance_present = true;
  wrong_length.covariance = short_values;
  const MeasurementAssessment short_assessment =
      AssessMeasurementHealth(wrong_length);
  EXPECT_EQ(short_assessment.error, MeasurementError::kCovariance);
  EXPECT_EQ(short_assessment.state, MeasurementStateKind::kValid);
  EXPECT_FALSE(vehicle::CovarianceIsUnknown(
      vehicle::AssessCovariance(true, short_values)));
}

TEST(MeasurementHealthPolicyTest, CovarianceUsesTheVehicleShapeRules) {
  std::array<double, vehicle::kCovarianceValues> values = {};
  values[0] = std::numeric_limits<double>::quiet_NaN();
  MeasurementHealthView sample = ValidSample();
  sample.covariance_present = true;
  sample.covariance = values;
  EXPECT_EQ(AssessMeasurementHealth(sample).error,
            MeasurementError::kCovariance);

  values[0] = 0.25;
  values[1] = 1.0;
  EXPECT_EQ(AssessMeasurementHealth(sample).error,
            MeasurementError::kCovariance);

  values[1] = 1e-12;
  EXPECT_TRUE(AssessMeasurementHealth(sample).accepted);
  values[1] = vehicle::kCovarianceSymmetryTolerance;
  EXPECT_TRUE(AssessMeasurementHealth(sample).accepted);
  values[1] = 1e-6;
  EXPECT_EQ(AssessMeasurementHealth(sample).error,
            MeasurementError::kCovariance);
}

TEST(MeasurementHealthPolicyTest,
     EmptySourceIdIsRejectedAndUnsetValidityIsNot) {
  MeasurementHealthView sample = ValidSample();
  const SourceHealthView sources[] = {
      SourceHealthView{"primary", true, 1},
      SourceHealthView{"", false, 0},
  };
  sample.sources = sources;
  EXPECT_EQ(AssessMeasurementHealth(sample).error, MeasurementError::kSourceId);

  const SourceHealthView aiding[] = {SourceHealthView{"aiding", false, 0}};
  sample.sources = aiding;
  const MeasurementAssessment assessment = AssessMeasurementHealth(sample);
  EXPECT_TRUE(assessment.accepted);
  EXPECT_EQ(embodiment::ClassifyValidity(aiding[0].validity_present,
                                         aiding[0].validity_state),
            ValidityKind::kAbsent);
  EXPECT_NE(embodiment::ClassifyValidity(aiding[0].validity_present,
                                         aiding[0].validity_state),
            ValidityKind::kInvalid);
}

TEST(MeasurementHealthPolicyTest, FirstDefectWins) {
  MeasurementHealthView sample = ValidSample();
  sample.frame_id = "";
  sample.quality = std::numeric_limits<double>::quiet_NaN();
  EXPECT_EQ(AssessMeasurementHealth(sample).error,
            MeasurementError::kMissingFrame);

  sample = ValidSample();
  sample.receive_time = embodiment::ClockReading{1, 0};
  sample.source_time = embodiment::ClockReading{2, 0};
  sample.quality = 2.0;
  EXPECT_EQ(AssessMeasurementHealth(sample).error,
            MeasurementError::kTimeReversal);

  sample = ValidSample();
  sample.quality = 2.0;
  const std::array<double, 1> short_values = {1.0};
  sample.covariance_present = true;
  sample.covariance = short_values;
  EXPECT_EQ(AssessMeasurementHealth(sample).error, MeasurementError::kQuality);

  sample = ValidSample();
  sample.covariance_present = true;
  sample.covariance = short_values;
  const SourceHealthView sources[] = {SourceHealthView{"", false, 0}};
  sample.sources = sources;
  EXPECT_EQ(AssessMeasurementHealth(sample).error,
            MeasurementError::kCovariance);
}

}  // namespace
}  // namespace intrinsic::hardware::marine
