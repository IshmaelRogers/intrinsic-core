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

#include "intrinsic/hardware/marine/dvl_policy.h"

#include <array>
#include <cmath>
#include <limits>

#include "gtest/gtest.h"
#include "intrinsic/embodiment/frame_policy.h"
#include "intrinsic/embodiment/stamped_header_policy.h"
#include "intrinsic/vehicle/vehicle_contract_policy.h"

namespace intrinsic::hardware::marine {
namespace {

using embodiment::ValidityKind;

std::array<double, vehicle::kCovarianceValues> LinearCovariance() {
  std::array<double, vehicle::kCovarianceValues> values = {};
  values[vehicle::CovarianceIndex(0, 0)] = 0.25;
  values[vehicle::CovarianceIndex(1, 1)] = 0.25;
  values[vehicle::CovarianceIndex(2, 2)] = 0.25;
  return values;
}

DvlMeasurementView ValidBottom(
    const std::array<double, vehicle::kCovarianceValues>& covariance) {
  DvlMeasurementView sample;
  sample.health.header_present = true;
  sample.health.frame_id = "sensor";
  sample.health.source_time_present = true;
  sample.health.source_time = embodiment::ClockReading{1700000000, 250000000};
  sample.health.receive_time_present = true;
  sample.health.receive_time = embodiment::ClockReading{1700000001, 0};
  sample.health.header_validity_present = true;
  sample.health.header_validity_state = 1;
  sample.health.state_present = true;
  sample.health.state = 1;
  sample.health.quality_present = true;
  sample.health.quality = 0.75;
  sample.health.covariance_present = true;
  sample.health.covariance = covariance;
  sample.mode_present = true;
  sample.mode = 1;
  sample.velocity_x_present = true;
  sample.velocity_x_m_s = 0.5;
  sample.velocity_y_present = true;
  sample.velocity_y_m_s = -0.25;
  sample.velocity_z_present = true;
  sample.velocity_z_m_s = 0.0;
  sample.bottom_lock_present = true;
  sample.bottom_lock = true;
  sample.altitude_present = true;
  sample.altitude_m = 10.0;
  return sample;
}

TEST(DvlPolicyTest, EmptySampleIsAbsentAndNotAnError) {
  const DvlAssessment assessment = AssessDvl(DvlMeasurementView{});
  EXPECT_EQ(assessment.error, DvlError::kNone);
  EXPECT_EQ(assessment.state, MeasurementStateKind::kAbsent);
  EXPECT_EQ(assessment.mode, DvlModeKind::kAbsent);
  EXPECT_EQ(assessment.header_validity, ValidityKind::kAbsent);
  EXPECT_FALSE(assessment.accepted);
}

TEST(DvlPolicyTest, NominalBottomTrackIsAccepted) {
  const auto covariance = LinearCovariance();
  const DvlAssessment assessment = AssessDvl(ValidBottom(covariance));
  EXPECT_EQ(assessment.error, DvlError::kNone);
  EXPECT_EQ(assessment.state, MeasurementStateKind::kValid);
  EXPECT_EQ(assessment.mode, DvlModeKind::kBottomTrack);
  EXPECT_EQ(assessment.header_validity, ValidityKind::kValid);
  EXPECT_TRUE(assessment.accepted);
}

TEST(DvlPolicyTest, WaterTrackWithAltitudeIsAccepted) {
  const auto covariance = LinearCovariance();
  DvlMeasurementView sample = ValidBottom(covariance);
  sample.mode = 2;
  sample.bottom_lock_present = false;
  sample.bottom_lock = false;
  const DvlAssessment assessment = AssessDvl(sample);
  EXPECT_EQ(assessment.error, DvlError::kNone);
  EXPECT_EQ(assessment.mode, DvlModeKind::kWaterTrack);
  EXPECT_TRUE(assessment.accepted);

  sample.bottom_lock_present = true;
  sample.bottom_lock = false;
  EXPECT_TRUE(AssessDvl(sample).accepted);
  EXPECT_EQ(AssessDvl(sample).error, DvlError::kNone);
}

TEST(DvlPolicyTest, ModeAbsenceIsDistinctFromUnspecified) {
  EXPECT_EQ(ClassifyDvlMode(false, 0), DvlModeKind::kAbsent);
  EXPECT_EQ(ClassifyDvlMode(true, 0), DvlModeKind::kUnspecified);
  EXPECT_EQ(ClassifyDvlMode(true, 1), DvlModeKind::kBottomTrack);
  EXPECT_EQ(ClassifyDvlMode(true, 2), DvlModeKind::kWaterTrack);
  EXPECT_EQ(ClassifyDvlMode(true, 100), DvlModeKind::kUnrecognized);
  EXPECT_NE(ClassifyDvlMode(true, 100), DvlModeKind::kUnspecified);

  const auto covariance = LinearCovariance();
  DvlMeasurementView absent = ValidBottom(covariance);
  absent.mode_present = false;
  absent.mode = 0;
  EXPECT_EQ(AssessDvl(absent).error, DvlError::kMode);
  EXPECT_EQ(AssessDvl(absent).mode, DvlModeKind::kAbsent);

  DvlMeasurementView unspecified = ValidBottom(covariance);
  unspecified.mode = 0;
  EXPECT_EQ(AssessDvl(unspecified).error, DvlError::kMode);
  EXPECT_EQ(AssessDvl(unspecified).mode, DvlModeKind::kUnspecified);

  DvlMeasurementView unknown = ValidBottom(covariance);
  unknown.mode = 100;
  const DvlAssessment assessment = AssessDvl(unknown);
  EXPECT_EQ(assessment.error, DvlError::kMode);
  EXPECT_EQ(assessment.mode, DvlModeKind::kUnrecognized);
  EXPECT_EQ(unknown.mode, 100);
}

TEST(DvlPolicyTest, VelocityMustBePresentAndFinite) {
  const auto covariance = LinearCovariance();
  DvlMeasurementView missing = ValidBottom(covariance);
  missing.velocity_z_present = false;
  missing.velocity_x_m_s = std::numeric_limits<double>::quiet_NaN();
  EXPECT_EQ(AssessDvl(missing).error, DvlError::kVelocity);

  DvlMeasurementView nan = ValidBottom(covariance);
  nan.velocity_y_m_s = std::numeric_limits<double>::quiet_NaN();
  const DvlAssessment nan_assessment = AssessDvl(nan);
  EXPECT_EQ(nan_assessment.error, DvlError::kNonFinite);
  EXPECT_EQ(nan_assessment.state, MeasurementStateKind::kValid);
  EXPECT_EQ(nan.health.state, 1);

  DvlMeasurementView infinite = ValidBottom(covariance);
  infinite.velocity_x_m_s = std::numeric_limits<double>::infinity();
  EXPECT_EQ(AssessDvl(infinite).error, DvlError::kNonFinite);

  DvlMeasurementView zero = ValidBottom(covariance);
  zero.velocity_x_m_s = 0.0;
  zero.velocity_y_m_s = 0.0;
  zero.velocity_z_m_s = 0.0;
  EXPECT_TRUE(AssessDvl(zero).accepted);

  DvlMeasurementView large = ValidBottom(covariance);
  large.velocity_x_m_s = 1.0e6;
  EXPECT_TRUE(AssessDvl(large).accepted);
}

TEST(DvlPolicyTest, QualityBoundsAndLockCombinations) {
  const auto covariance = LinearCovariance();
  DvlMeasurementView low = ValidBottom(covariance);
  low.health.quality = 0.0;
  EXPECT_TRUE(AssessDvl(low).accepted);
  DvlMeasurementView high = ValidBottom(covariance);
  high.health.quality = 1.0;
  EXPECT_TRUE(AssessDvl(high).accepted);

  DvlMeasurementView above = ValidBottom(covariance);
  above.health.quality = std::nextafter(1.0, 2.0);
  EXPECT_EQ(AssessDvl(above).error, DvlError::kQuality);

  DvlMeasurementView lock_loss = ValidBottom(covariance);
  lock_loss.bottom_lock = false;
  lock_loss.health.state = 3;
  const DvlAssessment loss = AssessDvl(lock_loss);
  EXPECT_EQ(loss.error, DvlError::kNone);
  EXPECT_EQ(loss.state, MeasurementStateKind::kInvalid);
  EXPECT_FALSE(loss.accepted);

  DvlMeasurementView inconsistent = ValidBottom(covariance);
  inconsistent.bottom_lock = false;
  EXPECT_EQ(AssessDvl(inconsistent).error, DvlError::kLock);
  EXPECT_EQ(AssessDvl(inconsistent).state, MeasurementStateKind::kValid);

  DvlMeasurementView unlocked_absent = ValidBottom(covariance);
  unlocked_absent.bottom_lock_present = false;
  EXPECT_TRUE(AssessDvl(unlocked_absent).accepted);

  DvlMeasurementView degraded_unlock = ValidBottom(covariance);
  degraded_unlock.bottom_lock = false;
  degraded_unlock.health.state = 2;
  const DvlAssessment degraded = AssessDvl(degraded_unlock);
  EXPECT_EQ(degraded.error, DvlError::kNone);
  EXPECT_EQ(degraded.state, MeasurementStateKind::kDegraded);
  EXPECT_FALSE(degraded.accepted);
}

TEST(DvlPolicyTest, AltitudeIsOptionalNonNegativeOnEitherTrack) {
  const auto covariance = LinearCovariance();
  DvlMeasurementView absent = ValidBottom(covariance);
  absent.altitude_present = false;
  EXPECT_TRUE(AssessDvl(absent).accepted);

  DvlMeasurementView zero = ValidBottom(covariance);
  zero.altitude_m = 0.0;
  EXPECT_TRUE(AssessDvl(zero).accepted);

  DvlMeasurementView negative = ValidBottom(covariance);
  negative.altitude_m = std::nextafter(0.0, -1.0);
  EXPECT_EQ(AssessDvl(negative).error, DvlError::kAltitude);

  DvlMeasurementView nan = ValidBottom(covariance);
  nan.altitude_m = std::numeric_limits<double>::quiet_NaN();
  EXPECT_EQ(AssessDvl(nan).error, DvlError::kNonFinite);
  EXPECT_EQ(nan.health.state, 1);

  DvlMeasurementView water = ValidBottom(covariance);
  water.mode = 2;
  water.bottom_lock_present = false;
  water.altitude_m = 0.0;
  EXPECT_TRUE(AssessDvl(water).accepted);
  water.altitude_m = 4.5;
  EXPECT_TRUE(AssessDvl(water).accepted);
}

TEST(DvlPolicyTest, CovarianceAbsenceZeroAndAngularSlots) {
  std::array<double, vehicle::kCovarianceValues> none = {};
  DvlMeasurementView absent = ValidBottom(none);
  absent.health.covariance_present = false;
  EXPECT_TRUE(AssessDvl(absent).accepted);
  EXPECT_TRUE(vehicle::CovarianceIsUnknown(
      vehicle::AssessCovariance(false, absent.health.covariance)));

  const std::array<double, vehicle::kCovarianceValues> zeros = {};
  DvlMeasurementView zero = ValidBottom(zeros);
  EXPECT_TRUE(AssessDvl(zero).accepted);
  EXPECT_TRUE(vehicle::IsAllZeroCovariance(zeros));
  EXPECT_FALSE(
      vehicle::CovarianceIsUnknown(vehicle::AssessCovariance(true, zeros)));

  std::array<double, 35> short_values = {};
  DvlMeasurementView bad_shape = ValidBottom(none);
  bad_shape.health.covariance = short_values;
  EXPECT_EQ(AssessDvl(bad_shape).error, DvlError::kCovariance);

  auto angular = LinearCovariance();
  angular[kDvlAngularVarianceSlots[0]] = 1e-15;
  DvlMeasurementView angled = ValidBottom(angular);
  EXPECT_EQ(AssessDvl(angled).error, DvlError::kAngularCovariance);

  auto cross = LinearCovariance();
  cross[vehicle::CovarianceIndex(3, 4)] = 0.1;
  cross[vehicle::CovarianceIndex(4, 3)] = 0.1;
  DvlMeasurementView symmetric = ValidBottom(cross);
  EXPECT_TRUE(AssessDvl(symmetric).accepted);
}

TEST(DvlPolicyTest, MetadataDefectsUseHealthOrder) {
  const auto covariance = LinearCovariance();
  DvlMeasurementView missing_frame = ValidBottom(covariance);
  missing_frame.health.frame_id = "";
  EXPECT_EQ(AssessDvl(missing_frame).error, DvlError::kMissingFrame);

  DvlMeasurementView wrong = ValidBottom(covariance);
  wrong.health.expected_frame_present = true;
  wrong.health.expected_frame = embodiment::kWorldEnuFrameId;
  EXPECT_EQ(AssessDvl(wrong).error, DvlError::kWrongFrame);
  wrong.health.frame_id = embodiment::kWorldEnuFrameId;
  EXPECT_TRUE(AssessDvl(wrong).accepted);

  DvlMeasurementView reversed = ValidBottom(covariance);
  reversed.health.receive_time =
      embodiment::ClockReading{1700000000, 249999999};
  EXPECT_EQ(AssessDvl(reversed).error, DvlError::kTimeReversal);

  DvlMeasurementView delayed = ValidBottom(covariance);
  delayed.health.receive_time = embodiment::ClockReading{1700000100, 0};
  EXPECT_TRUE(AssessDvl(delayed).accepted);

  DvlMeasurementView quality_nan = ValidBottom(covariance);
  quality_nan.health.quality = std::numeric_limits<double>::quiet_NaN();
  EXPECT_EQ(AssessDvl(quality_nan).error, DvlError::kNonFinite);
  EXPECT_EQ(AssessDvl(quality_nan).state, MeasurementStateKind::kValid);
}

TEST(DvlPolicyTest, MissingHealthPrecedesPayloadAndValidityStaysPut) {
  DvlMeasurementView payload_only;
  payload_only.velocity_x_present = true;
  payload_only.velocity_x_m_s = 1.0;
  payload_only.mode_present = true;
  payload_only.mode = 1;
  EXPECT_EQ(AssessDvl(payload_only).error, DvlError::kMissingHealth);

  const auto covariance = LinearCovariance();
  DvlMeasurementView sample = ValidBottom(covariance);
  sample.health.header_validity_state = 2;
  const DvlAssessment invalid_header = AssessDvl(sample);
  EXPECT_TRUE(invalid_header.accepted);
  EXPECT_EQ(invalid_header.header_validity, ValidityKind::kInvalid);

  sample.health.state = 3;
  sample.health.header_validity_present = false;
  const DvlAssessment invalid_measurement = AssessDvl(sample);
  EXPECT_FALSE(invalid_measurement.accepted);
  EXPECT_EQ(invalid_measurement.state, MeasurementStateKind::kInvalid);
  EXPECT_EQ(invalid_measurement.header_validity, ValidityKind::kAbsent);

  EXPECT_EQ(embodiment::ClassifyValidity(true, 3), ValidityKind::kUnspecified);
  EXPECT_NE(embodiment::ClassifyValidity(true, 3), ValidityKind::kInvalid);
}

TEST(DvlPolicyTest, FirstDefectWins) {
  const auto covariance = LinearCovariance();
  DvlMeasurementView sample = ValidBottom(covariance);
  sample.health.frame_id = "";
  sample.mode_present = false;
  EXPECT_EQ(AssessDvl(sample).error, DvlError::kMissingFrame);

  sample = ValidBottom(covariance);
  sample.health.quality = 2.0;
  sample.mode = 0;
  EXPECT_EQ(AssessDvl(sample).error, DvlError::kQuality);

  sample = ValidBottom(covariance);
  sample.mode = 100;
  sample.velocity_x_present = false;
  EXPECT_EQ(AssessDvl(sample).error, DvlError::kMode);

  sample = ValidBottom(covariance);
  sample.velocity_z_m_s = std::numeric_limits<double>::quiet_NaN();
  sample.bottom_lock = false;
  EXPECT_EQ(AssessDvl(sample).error, DvlError::kNonFinite);

  sample = ValidBottom(covariance);
  sample.bottom_lock = false;
  sample.altitude_m = -1.0;
  EXPECT_EQ(AssessDvl(sample).error, DvlError::kLock);

  sample = ValidBottom(covariance);
  sample.altitude_m = -1.0;
  EXPECT_EQ(AssessDvl(sample).error, DvlError::kAltitude);

  auto angular = LinearCovariance();
  angular[kDvlAngularVarianceSlots[2]] = 0.0625;
  sample = ValidBottom(angular);
  sample.mode_present = false;
  EXPECT_EQ(AssessDvl(sample).error, DvlError::kAngularCovariance);
}

}  // namespace
}  // namespace intrinsic::hardware::marine
