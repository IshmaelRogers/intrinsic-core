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

#include "intrinsic/hardware/marine/ins_policy.h"

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

static_assert(kInsUnitQuaternionTolerance == 1e-6);

std::array<double, vehicle::kCovarianceValues> InsCovariance() {
  std::array<double, vehicle::kCovarianceValues> values = {};
  values[kPositionXVarianceSlot] = 1.0;
  values[kPositionYVarianceSlot] = 4.0;
  values[kPositionZVarianceSlot] = 0.25;
  values[kAttitudeXVarianceSlot] = 0.0625;
  values[kAttitudeYVarianceSlot] = 0.125;
  values[kAttitudeZVarianceSlot] = 0.5;
  return values;
}

InsSolutionView ValidIns(
    const std::array<double, vehicle::kCovarianceValues>& covariance) {
  InsSolutionView sample;
  sample.health.header_present = true;
  sample.health.frame_id = "ins";
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
  sample.position_x_present = true;
  sample.position_x_m = 12.0;
  sample.position_y_present = true;
  sample.position_y_m = -4.0;
  sample.position_z_present = true;
  sample.position_z_m = 0.5;
  sample.orientation_present = true;
  sample.orientation_w = 1.0;
  sample.linear_velocity_x_present = true;
  sample.linear_velocity_x_m_s = 1.5;
  sample.linear_velocity_y_present = true;
  sample.linear_velocity_y_m_s = 0.0;
  sample.linear_velocity_z_present = true;
  sample.linear_velocity_z_m_s = -0.25;
  sample.angular_velocity_x_present = true;
  sample.angular_velocity_x_rad_s = 0.0;
  sample.angular_velocity_y_present = true;
  sample.angular_velocity_y_rad_s = 0.125;
  sample.angular_velocity_z_present = true;
  sample.angular_velocity_z_rad_s = 0.0;
  sample.source_present = true;
  sample.source = 1;
  return sample;
}

TEST(InsPolicyTest, EmptySampleIsAbsentAndNotAnError) {
  const InsAssessment assessment = AssessIns(InsSolutionView{});
  EXPECT_EQ(assessment.error, InsError::kNone);
  EXPECT_EQ(assessment.state, MeasurementStateKind::kAbsent);
  EXPECT_EQ(assessment.source, InsSourceKind::kAbsent);
  EXPECT_EQ(assessment.header_validity, ValidityKind::kAbsent);
  EXPECT_FALSE(assessment.accepted);
}

TEST(InsPolicyTest, NominalSolutionAndOptionalTwist) {
  const auto covariance = InsCovariance();
  const InsAssessment nominal = AssessIns(ValidIns(covariance));
  EXPECT_EQ(nominal.error, InsError::kNone);
  EXPECT_EQ(nominal.state, MeasurementStateKind::kValid);
  EXPECT_EQ(nominal.source, InsSourceKind::kVendorIns);
  EXPECT_TRUE(nominal.accepted);

  InsSolutionView pose_only = ValidIns(covariance);
  pose_only.linear_velocity_x_present = false;
  pose_only.linear_velocity_y_present = false;
  pose_only.linear_velocity_z_present = false;
  pose_only.angular_velocity_x_present = false;
  pose_only.angular_velocity_y_present = false;
  pose_only.angular_velocity_z_present = false;
  pose_only.source_present = false;
  pose_only.source = 0;
  EXPECT_TRUE(AssessIns(pose_only).accepted);
  EXPECT_EQ(AssessIns(pose_only).source, InsSourceKind::kAbsent);

  InsSolutionView external = ValidIns(covariance);
  external.source = 2;
  const InsAssessment external_assessment = AssessIns(external);
  EXPECT_TRUE(external_assessment.accepted);
  EXPECT_EQ(external_assessment.source, InsSourceKind::kExternalNav);

  InsSolutionView origin = ValidIns(covariance);
  origin.position_x_m = 0.0;
  origin.position_y_m = 0.0;
  origin.position_z_m = 0.0;
  EXPECT_TRUE(AssessIns(origin).accepted);

  InsSolutionView negative = ValidIns(covariance);
  negative.position_z_m = -12.5;
  EXPECT_TRUE(AssessIns(negative).accepted);
}

TEST(InsPolicyTest, PositionAndRequiredOrientation) {
  const auto covariance = InsCovariance();
  InsSolutionView missing_position = ValidIns(covariance);
  missing_position.position_y_present = false;
  missing_position.position_y_m = std::numeric_limits<double>::quiet_NaN();
  EXPECT_EQ(AssessIns(missing_position).error, InsError::kPosition);

  InsSolutionView non_finite = ValidIns(covariance);
  non_finite.position_x_m = std::numeric_limits<double>::infinity();
  EXPECT_EQ(AssessIns(non_finite).error, InsError::kNonFinite);
  EXPECT_EQ(non_finite.health.state, 1);

  InsSolutionView missing_orientation = ValidIns(covariance);
  missing_orientation.orientation_present = false;
  missing_orientation.orientation_w = 1.0;
  EXPECT_EQ(AssessIns(missing_orientation).error, InsError::kOrientation);

  InsSolutionView non_unit = ValidIns(covariance);
  non_unit.orientation_w = 2.0;
  EXPECT_EQ(AssessIns(non_unit).error, InsError::kOrientation);
  EXPECT_DOUBLE_EQ(non_unit.orientation_w, 2.0);

  InsSolutionView edge = ValidIns(covariance);
  edge.orientation_w = 1.0 + kInsUnitQuaternionTolerance;
  EXPECT_TRUE(AssessIns(edge).accepted);

  InsSolutionView outside = ValidIns(covariance);
  outside.orientation_w =
      std::nextafter(1.0 + kInsUnitQuaternionTolerance, 2.0);
  EXPECT_EQ(AssessIns(outside).error, InsError::kOrientation);

  double inside_low = 1.0 - kInsUnitQuaternionTolerance;
  while (!InsUnitQuaternion(0.0, 0.0, 0.0, inside_low)) {
    inside_low = std::nextafter(inside_low, 1.0);
  }
  InsSolutionView low = ValidIns(covariance);
  low.orientation_w = inside_low;
  EXPECT_TRUE(AssessIns(low).accepted);
  InsSolutionView below = ValidIns(covariance);
  below.orientation_w = std::nextafter(inside_low, 0.0);
  EXPECT_EQ(AssessIns(below).error, InsError::kOrientation);

  InsSolutionView nan = ValidIns(covariance);
  nan.orientation_z = std::numeric_limits<double>::quiet_NaN();
  EXPECT_EQ(AssessIns(nan).error, InsError::kNonFinite);
}

TEST(InsPolicyTest, PartialVelocityTriples) {
  const auto covariance = InsCovariance();
  InsSolutionView linear_partial = ValidIns(covariance);
  linear_partial.linear_velocity_z_present = false;
  EXPECT_EQ(AssessIns(linear_partial).error, InsError::kLinearVelocity);

  InsSolutionView angular_partial = ValidIns(covariance);
  angular_partial.angular_velocity_x_present = false;
  angular_partial.angular_velocity_y_present = false;
  EXPECT_EQ(AssessIns(angular_partial).error, InsError::kAngularVelocity);

  InsSolutionView linear_nan = ValidIns(covariance);
  linear_nan.linear_velocity_y_m_s = std::numeric_limits<double>::quiet_NaN();
  EXPECT_EQ(AssessIns(linear_nan).error, InsError::kNonFinite);

  InsSolutionView angular_only = ValidIns(covariance);
  angular_only.linear_velocity_x_present = false;
  angular_only.linear_velocity_y_present = false;
  angular_only.linear_velocity_z_present = false;
  EXPECT_TRUE(AssessIns(angular_only).accepted);
}

TEST(InsPolicyTest, SourceEnumIsKept) {
  const auto covariance = InsCovariance();
  InsSolutionView unspecified = ValidIns(covariance);
  unspecified.source = 0;
  const InsAssessment unspecified_assessment = AssessIns(unspecified);
  EXPECT_EQ(unspecified_assessment.error, InsError::kSource);
  EXPECT_EQ(unspecified_assessment.source, InsSourceKind::kUnspecified);
  EXPECT_EQ(unspecified.source, 0);

  InsSolutionView unknown = ValidIns(covariance);
  unknown.source = 99;
  const InsAssessment unknown_assessment = AssessIns(unknown);
  EXPECT_EQ(unknown_assessment.error, InsError::kSource);
  EXPECT_EQ(unknown_assessment.source, InsSourceKind::kUnrecognized);
  EXPECT_EQ(unknown.source, 99);
  EXPECT_EQ(ClassifyInsSource(true, 99), InsSourceKind::kUnrecognized);
  EXPECT_EQ(ClassifyInsSource(false, 0), InsSourceKind::kAbsent);
}

TEST(InsPolicyTest, CovarianceAbsenceZeroAndSlots) {
  std::array<double, vehicle::kCovarianceValues> none = {};
  InsSolutionView absent = ValidIns(none);
  absent.health.covariance_present = false;
  EXPECT_TRUE(AssessIns(absent).accepted);

  const std::array<double, vehicle::kCovarianceValues> zeros = {};
  EXPECT_TRUE(AssessIns(ValidIns(zeros)).accepted);
  EXPECT_TRUE(vehicle::IsAllZeroCovariance(zeros));

  auto off_slot = InsCovariance();
  off_slot[vehicle::CovarianceIndex(0, 3)] = 0.01;
  off_slot[vehicle::CovarianceIndex(3, 0)] = 0.01;
  EXPECT_EQ(AssessIns(ValidIns(off_slot)).error, InsError::kCovarianceSlots);

  auto velocity_slot = InsCovariance();
  velocity_slot[1] = 1e-12;
  velocity_slot[vehicle::CovarianceIndex(1, 0)] = 1e-12;
  EXPECT_EQ(AssessIns(ValidIns(velocity_slot)).error,
            InsError::kCovarianceSlots);

  std::array<double, 35> short_values = {};
  InsSolutionView bad_shape = ValidIns(none);
  bad_shape.health.covariance = short_values;
  EXPECT_EQ(AssessIns(bad_shape).error, InsError::kCovariance);
}

TEST(InsPolicyTest, MetadataDefectsUseHealthOrder) {
  const auto covariance = InsCovariance();
  InsSolutionView missing_frame = ValidIns(covariance);
  missing_frame.health.frame_id = "";
  EXPECT_EQ(AssessIns(missing_frame).error, InsError::kMissingFrame);

  InsSolutionView wrong = ValidIns(covariance);
  wrong.health.expected_frame_present = true;
  wrong.health.expected_frame = embodiment::kWorldNedFrameId;
  EXPECT_EQ(AssessIns(wrong).error, InsError::kWrongFrame);
  wrong.health.frame_id = embodiment::kWorldNedFrameId;
  EXPECT_TRUE(AssessIns(wrong).accepted);

  InsSolutionView reversed = ValidIns(covariance);
  reversed.health.receive_time = embodiment::ClockReading{1699999999, 0};
  EXPECT_EQ(AssessIns(reversed).error, InsError::kTimeReversal);

  InsSolutionView quality_nan = ValidIns(covariance);
  quality_nan.health.quality = std::numeric_limits<double>::quiet_NaN();
  EXPECT_EQ(AssessIns(quality_nan).error, InsError::kNonFinite);

  SourceHealthView empty_source{"", false, 0};
  InsSolutionView source = ValidIns(covariance);
  source.health.sources = std::span<const SourceHealthView>(&empty_source, 1);
  EXPECT_EQ(AssessIns(source).error, InsError::kSourceId);
}

TEST(InsPolicyTest, QualityEndpointsAndValidityStaysPut) {
  const auto covariance = InsCovariance();
  InsSolutionView low = ValidIns(covariance);
  low.health.quality = 0.0;
  EXPECT_TRUE(AssessIns(low).accepted);
  InsSolutionView high = ValidIns(covariance);
  high.health.quality = 1.0;
  EXPECT_TRUE(AssessIns(high).accepted);

  InsSolutionView sample = ValidIns(covariance);
  sample.health.header_validity_state = 2;
  EXPECT_TRUE(AssessIns(sample).accepted);
  EXPECT_EQ(AssessIns(sample).header_validity, ValidityKind::kInvalid);

  sample.health.state = 3;
  sample.orientation_w = 2.0;
  const InsAssessment marked = AssessIns(sample);
  EXPECT_EQ(marked.error, InsError::kOrientation);
  EXPECT_EQ(marked.state, MeasurementStateKind::kInvalid);
  EXPECT_EQ(sample.health.state, 3);

  InsSolutionView payload_only;
  payload_only.position_x_present = true;
  EXPECT_EQ(AssessIns(payload_only).error, InsError::kMissingHealth);
}

TEST(InsPolicyTest, FirstDefectWins) {
  const auto covariance = InsCovariance();
  InsSolutionView sample = ValidIns(covariance);
  sample.health.frame_id = "";
  sample.position_x_present = false;
  EXPECT_EQ(AssessIns(sample).error, InsError::kMissingFrame);

  auto slotted = InsCovariance();
  slotted[8] = 1e-12;
  slotted[vehicle::CovarianceIndex(2, 1)] = 1e-12;
  sample = ValidIns(slotted);
  sample.orientation_present = false;
  EXPECT_EQ(AssessIns(sample).error, InsError::kCovarianceSlots);

  sample = ValidIns(covariance);
  sample.position_z_present = false;
  sample.orientation_present = false;
  EXPECT_EQ(AssessIns(sample).error, InsError::kPosition);

  sample = ValidIns(covariance);
  sample.position_x_m = std::numeric_limits<double>::quiet_NaN();
  sample.orientation_w = 2.0;
  EXPECT_EQ(AssessIns(sample).error, InsError::kNonFinite);

  sample = ValidIns(covariance);
  sample.orientation_present = false;
  sample.linear_velocity_x_present = false;
  EXPECT_EQ(AssessIns(sample).error, InsError::kOrientation);

  sample = ValidIns(covariance);
  sample.orientation_w = 2.0;
  sample.linear_velocity_y_present = false;
  EXPECT_EQ(AssessIns(sample).error, InsError::kOrientation);

  sample = ValidIns(covariance);
  sample.linear_velocity_z_present = false;
  sample.angular_velocity_x_present = false;
  EXPECT_EQ(AssessIns(sample).error, InsError::kLinearVelocity);

  sample = ValidIns(covariance);
  sample.linear_velocity_x_m_s = std::numeric_limits<double>::infinity();
  sample.angular_velocity_z_present = false;
  EXPECT_EQ(AssessIns(sample).error, InsError::kNonFinite);

  sample = ValidIns(covariance);
  sample.angular_velocity_y_present = false;
  sample.source = 0;
  EXPECT_EQ(AssessIns(sample).error, InsError::kAngularVelocity);

  sample = ValidIns(covariance);
  sample.source = 99;
  EXPECT_EQ(AssessIns(sample).error, InsError::kSource);
  EXPECT_EQ(sample.source, 99);
}

}  // namespace
}  // namespace intrinsic::hardware::marine
