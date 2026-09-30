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

#include "intrinsic/hardware/marine/imu_policy.h"

#include <array>
#include <cmath>
#include <limits>

#include "gtest/gtest.h"
#include "intrinsic/embodiment/frame_policy.h"
#include "intrinsic/embodiment/stamped_header_policy.h"
#include "intrinsic/hardware/marine/ins_policy.h"
#include "intrinsic/vehicle/vehicle_contract_policy.h"

namespace intrinsic::hardware::marine {
namespace {

using embodiment::ValidityKind;

static_assert(kImuUnitQuaternionTolerance == 1e-6);
static_assert(kImuUnitQuaternionTolerance == kInsUnitQuaternionTolerance);

std::array<double, vehicle::kCovarianceValues> ImuCovariance() {
  std::array<double, vehicle::kCovarianceValues> values = {};
  values[kAngularVelocityXVarianceSlot] = 0.25;
  values[kAngularVelocityYVarianceSlot] = 0.5;
  values[kAngularVelocityZVarianceSlot] = 0.125;
  values[kLinearAccelerationXVarianceSlot] = 1.0;
  values[kLinearAccelerationYVarianceSlot] = 2.0;
  values[kLinearAccelerationZVarianceSlot] = 4.0;
  return values;
}

ImuMeasurementView ValidImu(
    const std::array<double, vehicle::kCovarianceValues>& covariance) {
  ImuMeasurementView sample;
  sample.health.header_present = true;
  sample.health.frame_id = "imu";
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
  sample.angular_velocity_x_present = true;
  sample.angular_velocity_x_rad_s = 0.25;
  sample.angular_velocity_y_present = true;
  sample.angular_velocity_y_rad_s = -0.5;
  sample.angular_velocity_z_present = true;
  sample.angular_velocity_z_rad_s = 0.125;
  sample.linear_acceleration_x_present = true;
  sample.linear_acceleration_x_m_s2 = 0.0;
  sample.linear_acceleration_y_present = true;
  sample.linear_acceleration_y_m_s2 = 0.0;
  sample.linear_acceleration_z_present = true;
  sample.linear_acceleration_z_m_s2 = 8.0;
  sample.orientation_present = true;
  sample.orientation_w = 1.0;
  return sample;
}

TEST(ImuPolicyTest, EmptySampleIsAbsentAndNotAnError) {
  const ImuAssessment assessment = AssessImu(ImuMeasurementView{});
  EXPECT_EQ(assessment.error, ImuError::kNone);
  EXPECT_EQ(assessment.state, MeasurementStateKind::kAbsent);
  EXPECT_EQ(assessment.header_validity, ValidityKind::kAbsent);
  EXPECT_FALSE(assessment.accepted);
}

TEST(ImuPolicyTest, NominalWithAndWithoutOrientation) {
  const auto covariance = ImuCovariance();
  const ImuAssessment with_orientation = AssessImu(ValidImu(covariance));
  EXPECT_EQ(with_orientation.error, ImuError::kNone);
  EXPECT_EQ(with_orientation.state, MeasurementStateKind::kValid);
  EXPECT_EQ(with_orientation.header_validity, ValidityKind::kValid);
  EXPECT_TRUE(with_orientation.accepted);

  ImuMeasurementView raw = ValidImu(covariance);
  raw.orientation_present = false;
  raw.orientation_w = 0.0;
  EXPECT_TRUE(AssessImu(raw).accepted);

  ImuMeasurementView zeros = ValidImu(covariance);
  zeros.angular_velocity_x_rad_s = 0.0;
  zeros.angular_velocity_y_rad_s = 0.0;
  zeros.angular_velocity_z_rad_s = 0.0;
  zeros.linear_acceleration_x_m_s2 = 0.0;
  zeros.linear_acceleration_y_m_s2 = 0.0;
  zeros.linear_acceleration_z_m_s2 = 0.0;
  EXPECT_TRUE(AssessImu(zeros).accepted);
}

TEST(ImuPolicyTest, IncompleteTriples) {
  const auto covariance = ImuCovariance();
  ImuMeasurementView missing_rate = ValidImu(covariance);
  missing_rate.angular_velocity_z_present = false;
  missing_rate.angular_velocity_z_rad_s =
      std::numeric_limits<double>::quiet_NaN();
  EXPECT_EQ(AssessImu(missing_rate).error, ImuError::kAngularVelocity);

  ImuMeasurementView missing_accel = ValidImu(covariance);
  missing_accel.linear_acceleration_x_present = false;
  EXPECT_EQ(AssessImu(missing_accel).error, ImuError::kLinearAcceleration);

  ImuMeasurementView rates_only;
  rates_only.angular_velocity_x_present = true;
  rates_only.angular_velocity_y_present = true;
  rates_only.angular_velocity_z_present = true;
  EXPECT_EQ(AssessImu(rates_only).error, ImuError::kMissingHealth);
}

TEST(ImuPolicyTest, NonFinitePayloadDoesNotRewriteState) {
  const auto covariance = ImuCovariance();
  ImuMeasurementView rate = ValidImu(covariance);
  rate.angular_velocity_y_rad_s = std::numeric_limits<double>::infinity();
  const ImuAssessment rate_assessment = AssessImu(rate);
  EXPECT_EQ(rate_assessment.error, ImuError::kNonFinite);
  EXPECT_EQ(rate_assessment.state, MeasurementStateKind::kValid);
  EXPECT_EQ(rate.health.state, 1);

  ImuMeasurementView accel = ValidImu(covariance);
  accel.linear_acceleration_z_m_s2 = std::numeric_limits<double>::quiet_NaN();
  EXPECT_EQ(AssessImu(accel).error, ImuError::kNonFinite);
}

TEST(ImuPolicyTest, QuaternionToleranceDoesNotRenormalize) {
  const auto covariance = ImuCovariance();
  ImuMeasurementView halves = ValidImu(covariance);
  halves.orientation_x = 0.5;
  halves.orientation_y = 0.5;
  halves.orientation_z = 0.5;
  halves.orientation_w = 0.5;
  EXPECT_TRUE(AssessImu(halves).accepted);
  EXPECT_DOUBLE_EQ(halves.orientation_w, 0.5);

  ImuMeasurementView edge = ValidImu(covariance);
  edge.orientation_w = 1.0 + kImuUnitQuaternionTolerance;
  EXPECT_TRUE(AssessImu(edge).accepted);
  EXPECT_DOUBLE_EQ(edge.orientation_w, 1.0 + 1e-6);

  ImuMeasurementView outside = ValidImu(covariance);
  outside.orientation_w =
      std::nextafter(1.0 + kImuUnitQuaternionTolerance, 2.0);
  EXPECT_EQ(AssessImu(outside).error, ImuError::kOrientation);
  EXPECT_EQ(outside.health.state, 1);
  EXPECT_DOUBLE_EQ(outside.orientation_w,
                   std::nextafter(1.0 + kImuUnitQuaternionTolerance, 2.0));

  // 1 - 1e-6 does not survive the square root as a norm exactly 1e-6
  // below 1. Walk up to the first accepted value and reject the neighbor.
  double inside_low = 1.0 - kImuUnitQuaternionTolerance;
  while (!ImuUnitQuaternion(0.0, 0.0, 0.0, inside_low)) {
    inside_low = std::nextafter(inside_low, 1.0);
  }
  ImuMeasurementView low = ValidImu(covariance);
  low.orientation_w = inside_low;
  EXPECT_TRUE(AssessImu(low).accepted);
  EXPECT_DOUBLE_EQ(low.orientation_w, inside_low);

  ImuMeasurementView below = ValidImu(covariance);
  below.orientation_w = std::nextafter(inside_low, 0.0);
  EXPECT_EQ(AssessImu(below).error, ImuError::kOrientation);

  ImuMeasurementView flipped = ValidImu(covariance);
  flipped.orientation_w = -1.0;
  EXPECT_TRUE(AssessImu(flipped).accepted);

  ImuMeasurementView axis = ValidImu(covariance);
  axis.orientation_x = 1.0;
  axis.orientation_w = 0.0;
  EXPECT_TRUE(AssessImu(axis).accepted);

  ImuMeasurementView non_unit = ValidImu(covariance);
  non_unit.orientation_w = 2.0;
  EXPECT_EQ(AssessImu(non_unit).error, ImuError::kOrientation);

  ImuMeasurementView nan = ValidImu(covariance);
  nan.orientation_x = std::numeric_limits<double>::quiet_NaN();
  EXPECT_EQ(AssessImu(nan).error, ImuError::kNonFinite);
}

TEST(ImuPolicyTest, QualityEndpoints) {
  const auto covariance = ImuCovariance();
  ImuMeasurementView low = ValidImu(covariance);
  low.health.quality = 0.0;
  EXPECT_TRUE(AssessImu(low).accepted);
  ImuMeasurementView high = ValidImu(covariance);
  high.health.quality = 1.0;
  EXPECT_TRUE(AssessImu(high).accepted);
  ImuMeasurementView above = ValidImu(covariance);
  above.health.quality = std::nextafter(1.0, 2.0);
  EXPECT_EQ(AssessImu(above).error, ImuError::kQuality);
}

TEST(ImuPolicyTest, CovarianceAbsenceZeroAndSlots) {
  std::array<double, vehicle::kCovarianceValues> none = {};
  ImuMeasurementView absent = ValidImu(none);
  absent.health.covariance_present = false;
  EXPECT_TRUE(AssessImu(absent).accepted);
  EXPECT_TRUE(vehicle::CovarianceIsUnknown(
      vehicle::AssessCovariance(false, absent.health.covariance)));

  const std::array<double, vehicle::kCovarianceValues> zeros = {};
  ImuMeasurementView zero = ValidImu(zeros);
  EXPECT_TRUE(AssessImu(zero).accepted);
  EXPECT_TRUE(vehicle::IsAllZeroCovariance(zeros));
  EXPECT_FALSE(
      vehicle::CovarianceIsUnknown(vehicle::AssessCovariance(true, zeros)));

  auto negative = ImuCovariance();
  negative[kAngularVelocityXVarianceSlot] = -0.25;
  EXPECT_TRUE(AssessImu(ValidImu(negative)).accepted);

  std::array<double, 35> short_values = {};
  ImuMeasurementView bad_shape = ValidImu(none);
  bad_shape.health.covariance = short_values;
  EXPECT_EQ(AssessImu(bad_shape).error, ImuError::kCovariance);

  auto off_slot = ImuCovariance();
  off_slot[1] = 1e-12;
  EXPECT_EQ(AssessImu(ValidImu(off_slot)).error, ImuError::kCovarianceSlots);

  auto cross = ImuCovariance();
  cross[vehicle::CovarianceIndex(0, 1)] = 0.1;
  cross[vehicle::CovarianceIndex(1, 0)] = 0.1;
  EXPECT_EQ(AssessImu(ValidImu(cross)).error, ImuError::kCovarianceSlots);

  auto asymmetric = ImuCovariance();
  asymmetric[vehicle::CovarianceIndex(0, 1)] = 1.0;
  EXPECT_EQ(AssessImu(ValidImu(asymmetric)).error, ImuError::kCovariance);
}

TEST(ImuPolicyTest, MetadataDefectsUseHealthOrder) {
  const auto covariance = ImuCovariance();
  ImuMeasurementView missing_frame = ValidImu(covariance);
  missing_frame.health.frame_id = "";
  EXPECT_EQ(AssessImu(missing_frame).error, ImuError::kMissingFrame);

  ImuMeasurementView wrong = ValidImu(covariance);
  wrong.health.expected_frame_present = true;
  wrong.health.expected_frame = embodiment::kWorldEnuFrameId;
  EXPECT_EQ(AssessImu(wrong).error, ImuError::kWrongFrame);
  wrong.health.frame_id = embodiment::kWorldEnuFrameId;
  EXPECT_TRUE(AssessImu(wrong).accepted);

  ImuMeasurementView reversed = ValidImu(covariance);
  reversed.health.receive_time =
      embodiment::ClockReading{1700000000, 249999999};
  EXPECT_EQ(AssessImu(reversed).error, ImuError::kTimeReversal);

  ImuMeasurementView quality_nan = ValidImu(covariance);
  quality_nan.health.quality = std::numeric_limits<double>::quiet_NaN();
  EXPECT_EQ(AssessImu(quality_nan).error, ImuError::kNonFinite);
  EXPECT_EQ(quality_nan.health.state, 1);

  SourceHealthView empty_source{"", false, 0};
  ImuMeasurementView source = ValidImu(covariance);
  source.health.sources = std::span<const SourceHealthView>(&empty_source, 1);
  EXPECT_EQ(AssessImu(source).error, ImuError::kSourceId);
}

TEST(ImuPolicyTest, MissingHealthPrecedesPayloadAndValidityStaysPut) {
  ImuMeasurementView payload_only;
  payload_only.orientation_present = true;
  payload_only.orientation_w = 1.0;
  EXPECT_EQ(AssessImu(payload_only).error, ImuError::kMissingHealth);

  const auto covariance = ImuCovariance();
  ImuMeasurementView sample = ValidImu(covariance);
  sample.health.header_validity_state = 2;
  const ImuAssessment invalid_header = AssessImu(sample);
  EXPECT_TRUE(invalid_header.accepted);
  EXPECT_EQ(invalid_header.header_validity, ValidityKind::kInvalid);

  sample.health.state = 2;
  sample.health.header_validity_present = false;
  const ImuAssessment degraded = AssessImu(sample);
  EXPECT_EQ(degraded.error, ImuError::kNone);
  EXPECT_EQ(degraded.state, MeasurementStateKind::kDegraded);
  EXPECT_FALSE(degraded.accepted);
  EXPECT_EQ(sample.health.state, 2);

  EXPECT_EQ(embodiment::ClassifyValidity(true, 3), ValidityKind::kUnspecified);
  EXPECT_NE(embodiment::ClassifyValidity(true, 3), ValidityKind::kInvalid);
}

TEST(ImuPolicyTest, FirstDefectWins) {
  const auto covariance = ImuCovariance();
  ImuMeasurementView sample = ValidImu(covariance);
  sample.health.frame_id = "";
  sample.angular_velocity_x_present = false;
  EXPECT_EQ(AssessImu(sample).error, ImuError::kMissingFrame);

  sample = ValidImu(covariance);
  sample.health.quality = 2.0;
  sample.orientation_w = 2.0;
  EXPECT_EQ(AssessImu(sample).error, ImuError::kQuality);

  auto slotted = ImuCovariance();
  slotted[vehicle::CovarianceIndex(0, 1)] = 0.1;
  slotted[vehicle::CovarianceIndex(1, 0)] = 0.1;
  sample = ValidImu(slotted);
  sample.angular_velocity_z_present = false;
  EXPECT_EQ(AssessImu(sample).error, ImuError::kCovarianceSlots);

  sample = ValidImu(covariance);
  sample.angular_velocity_x_present = false;
  sample.linear_acceleration_y_present = false;
  sample.orientation_w = 2.0;
  EXPECT_EQ(AssessImu(sample).error, ImuError::kAngularVelocity);

  sample = ValidImu(covariance);
  sample.angular_velocity_x_rad_s = std::numeric_limits<double>::quiet_NaN();
  sample.linear_acceleration_x_present = false;
  EXPECT_EQ(AssessImu(sample).error, ImuError::kNonFinite);

  sample = ValidImu(covariance);
  sample.linear_acceleration_z_m_s2 = std::numeric_limits<double>::infinity();
  sample.orientation_x = std::numeric_limits<double>::quiet_NaN();
  EXPECT_EQ(AssessImu(sample).error, ImuError::kNonFinite);

  sample = ValidImu(covariance);
  sample.orientation_x = std::numeric_limits<double>::infinity();
  sample.orientation_w = 2.0;
  EXPECT_EQ(AssessImu(sample).error, ImuError::kNonFinite);
}

}  // namespace
}  // namespace intrinsic::hardware::marine
