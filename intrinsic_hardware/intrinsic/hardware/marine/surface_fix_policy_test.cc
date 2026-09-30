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

#include "intrinsic/hardware/marine/surface_fix_policy.h"

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

constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();
constexpr double kInf = std::numeric_limits<double>::infinity();

std::array<double, vehicle::kCovarianceValues> PositionCovariance() {
  std::array<double, vehicle::kCovarianceValues> values = {};
  values[kSurfacePositionXVarianceSlot] = 1.0;
  values[kSurfacePositionYVarianceSlot] = 4.0;
  values[kSurfacePositionZVarianceSlot] = 0.25;
  return values;
}

SurfaceFixView ValidFix(
    const std::array<double, vehicle::kCovarianceValues>& covariance) {
  SurfaceFixView sample;
  sample.health.header_present = true;
  sample.health.frame_id = "gnss";
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
  sample.position_x_m = 12.5;
  sample.position_y_present = true;
  sample.position_y_m = -3.25;
  sample.position_z_present = true;
  sample.position_z_m = 1.0;
  sample.source_present = true;
  sample.source = 1;
  sample.satellite_count_present = true;
  sample.satellite_count = 12;
  return sample;
}

TEST(SurfaceFixPolicyTest, EmptySampleIsAbsentAndNotAnError) {
  const SurfaceFixAssessment assessment = AssessSurfaceFix(SurfaceFixView{});
  EXPECT_EQ(assessment.error, SurfaceFixError::kNone);
  EXPECT_EQ(assessment.state, MeasurementStateKind::kAbsent);
  EXPECT_EQ(assessment.source, SurfaceFixSourceKind::kAbsent);
  EXPECT_EQ(assessment.header_validity, ValidityKind::kAbsent);
  EXPECT_FALSE(assessment.accepted);
}

TEST(SurfaceFixPolicyTest, NominalGnssAcousticAndOtherAreAccepted) {
  const auto covariance = PositionCovariance();
  const SurfaceFixAssessment gnss = AssessSurfaceFix(ValidFix(covariance));
  EXPECT_EQ(gnss.error, SurfaceFixError::kNone);
  EXPECT_EQ(gnss.source, SurfaceFixSourceKind::kGnss);
  EXPECT_TRUE(gnss.accepted);

  SurfaceFixView acoustic = ValidFix(covariance);
  acoustic.source = 2;
  acoustic.satellite_count_present = false;
  acoustic.beacon_count_present = true;
  acoustic.beacon_count = 4;
  const SurfaceFixAssessment acoustic_result = AssessSurfaceFix(acoustic);
  EXPECT_EQ(acoustic_result.source, SurfaceFixSourceKind::kAcoustic);
  EXPECT_TRUE(acoustic_result.accepted);

  SurfaceFixView other = ValidFix(covariance);
  other.source = 3;
  const SurfaceFixAssessment other_result = AssessSurfaceFix(other);
  EXPECT_EQ(other_result.source, SurfaceFixSourceKind::kOther);
  EXPECT_TRUE(other_result.accepted);
}

TEST(SurfaceFixPolicyTest, ZeroAndNegativePositionAreSuppliedValues) {
  const auto covariance = PositionCovariance();
  SurfaceFixView zero = ValidFix(covariance);
  zero.position_x_m = 0.0;
  zero.position_y_m = 0.0;
  zero.position_z_m = 0.0;
  EXPECT_TRUE(AssessSurfaceFix(zero).accepted);
  SurfaceFixView negative = ValidFix(covariance);
  negative.position_z_m = -40.0;
  EXPECT_TRUE(AssessSurfaceFix(negative).accepted);
}

TEST(SurfaceFixPolicyTest, IncompletePositionIsRejected) {
  const auto covariance = PositionCovariance();
  SurfaceFixView x = ValidFix(covariance);
  x.position_x_present = false;
  SurfaceFixView y = ValidFix(covariance);
  y.position_y_present = false;
  SurfaceFixView z = ValidFix(covariance);
  z.position_z_present = false;
  for (const SurfaceFixView& sample : {x, y, z}) {
    const SurfaceFixAssessment assessment = AssessSurfaceFix(sample);
    EXPECT_EQ(assessment.error, SurfaceFixError::kPosition);
    EXPECT_FALSE(assessment.accepted);
  }
  EXPECT_EQ(AssessSurfaceFix(z).state, MeasurementStateKind::kValid);
}

TEST(SurfaceFixPolicyTest, NonFinitePositionIsRejected) {
  const auto covariance = PositionCovariance();
  for (double value : {kNaN, kInf, -kInf}) {
    SurfaceFixView x = ValidFix(covariance);
    x.position_x_m = value;
    SurfaceFixView y = ValidFix(covariance);
    y.position_y_m = value;
    SurfaceFixView z = ValidFix(covariance);
    z.position_z_m = value;
    for (const SurfaceFixView& sample : {x, y, z}) {
      EXPECT_EQ(AssessSurfaceFix(sample).error, SurfaceFixError::kNonFinite);
    }
  }
}

TEST(SurfaceFixPolicyTest, SourceAbsentUnspecifiedAndUnknownAreRejected) {
  const auto covariance = PositionCovariance();
  SurfaceFixView absent = ValidFix(covariance);
  absent.source_present = false;
  absent.source = 0;
  const SurfaceFixAssessment absent_result = AssessSurfaceFix(absent);
  EXPECT_EQ(absent_result.error, SurfaceFixError::kSource);
  EXPECT_EQ(absent_result.source, SurfaceFixSourceKind::kAbsent);
  EXPECT_EQ(absent_result.state, MeasurementStateKind::kValid);
  EXPECT_FALSE(absent_result.accepted);

  SurfaceFixView unspecified = ValidFix(covariance);
  unspecified.source = 0;
  const SurfaceFixAssessment unspecified_result = AssessSurfaceFix(unspecified);
  EXPECT_EQ(unspecified_result.error, SurfaceFixError::kSource);
  EXPECT_EQ(unspecified_result.source, SurfaceFixSourceKind::kUnspecified);

  SurfaceFixView unknown = ValidFix(covariance);
  unknown.source = 99;
  const SurfaceFixAssessment unknown_result = AssessSurfaceFix(unknown);
  EXPECT_EQ(unknown_result.error, SurfaceFixError::kSource);
  EXPECT_EQ(unknown_result.source, SurfaceFixSourceKind::kUnrecognized);
  EXPECT_FALSE(unknown_result.accepted);
}

TEST(SurfaceFixPolicyTest, InvalidFixIsPresentNotAcceptedAndNotRewritten) {
  const auto covariance = PositionCovariance();
  SurfaceFixView invalid = ValidFix(covariance);
  invalid.health.state = 3;
  invalid.source = 0;
  invalid.satellite_count_present = false;
  const SurfaceFixAssessment invalid_result = AssessSurfaceFix(invalid);
  EXPECT_EQ(invalid_result.error, SurfaceFixError::kSource);
  EXPECT_EQ(invalid_result.state, MeasurementStateKind::kInvalid);
  EXPECT_FALSE(invalid_result.accepted);

  SurfaceFixView lock_loss = ValidFix(covariance);
  lock_loss.health.state = 3;
  const SurfaceFixAssessment lock_loss_result = AssessSurfaceFix(lock_loss);
  EXPECT_EQ(lock_loss_result.error, SurfaceFixError::kNone);
  EXPECT_EQ(lock_loss_result.state, MeasurementStateKind::kInvalid);
  EXPECT_FALSE(lock_loss_result.accepted);

  SurfaceFixView degraded = ValidFix(covariance);
  degraded.health.state = 2;
  const SurfaceFixAssessment degraded_result = AssessSurfaceFix(degraded);
  EXPECT_EQ(degraded_result.error, SurfaceFixError::kNone);
  EXPECT_FALSE(degraded_result.accepted);
}

TEST(SurfaceFixPolicyTest, CountsAreOptionalAndZeroIsLegal) {
  const auto covariance = PositionCovariance();
  SurfaceFixView none = ValidFix(covariance);
  none.satellite_count_present = false;
  EXPECT_TRUE(AssessSurfaceFix(none).accepted);
  SurfaceFixView zero = ValidFix(covariance);
  zero.satellite_count = 0;
  zero.beacon_count_present = true;
  zero.beacon_count = 0;
  EXPECT_TRUE(AssessSurfaceFix(zero).accepted);
  SurfaceFixView negative_satellites = ValidFix(covariance);
  negative_satellites.satellite_count = -1;
  EXPECT_EQ(AssessSurfaceFix(negative_satellites).error,
            SurfaceFixError::kCount);
  SurfaceFixView negative_beacons = ValidFix(covariance);
  negative_beacons.beacon_count_present = true;
  negative_beacons.beacon_count = -1;
  EXPECT_EQ(AssessSurfaceFix(negative_beacons).error, SurfaceFixError::kCount);
}

TEST(SurfaceFixPolicyTest, AccuracyIsOptionalFiniteAndNonNegative) {
  const auto covariance = PositionCovariance();
  SurfaceFixView zero = ValidFix(covariance);
  zero.horizontal_accuracy_present = true;
  zero.horizontal_accuracy_m = 0.0;
  zero.vertical_accuracy_present = true;
  zero.vertical_accuracy_m = 0.0;
  EXPECT_TRUE(AssessSurfaceFix(zero).accepted);

  SurfaceFixView negative_horizontal = ValidFix(covariance);
  negative_horizontal.horizontal_accuracy_present = true;
  negative_horizontal.horizontal_accuracy_m = -0.5;
  EXPECT_EQ(AssessSurfaceFix(negative_horizontal).error,
            SurfaceFixError::kAccuracy);
  SurfaceFixView negative_vertical = ValidFix(covariance);
  negative_vertical.vertical_accuracy_present = true;
  negative_vertical.vertical_accuracy_m = -0.5;
  EXPECT_EQ(AssessSurfaceFix(negative_vertical).error,
            SurfaceFixError::kAccuracy);
  for (double value : {kNaN, kInf}) {
    SurfaceFixView horizontal = ValidFix(covariance);
    horizontal.horizontal_accuracy_present = true;
    horizontal.horizontal_accuracy_m = value;
    EXPECT_EQ(AssessSurfaceFix(horizontal).error, SurfaceFixError::kNonFinite);
    SurfaceFixView vertical = ValidFix(covariance);
    vertical.vertical_accuracy_present = true;
    vertical.vertical_accuracy_m = value;
    EXPECT_EQ(AssessSurfaceFix(vertical).error, SurfaceFixError::kNonFinite);
  }
}

TEST(SurfaceFixPolicyTest, VelocityIsAllThreeOrNone) {
  const auto covariance = PositionCovariance();
  SurfaceFixView full = ValidFix(covariance);
  full.velocity_x_present = true;
  full.velocity_y_present = true;
  full.velocity_z_present = true;
  full.velocity_x_m_s = 0.5;
  EXPECT_TRUE(AssessSurfaceFix(full).accepted);

  SurfaceFixView one = ValidFix(covariance);
  one.velocity_y_present = true;
  EXPECT_EQ(AssessSurfaceFix(one).error, SurfaceFixError::kVelocity);
  SurfaceFixView two = ValidFix(covariance);
  two.velocity_x_present = true;
  two.velocity_z_present = true;
  EXPECT_EQ(AssessSurfaceFix(two).error, SurfaceFixError::kVelocity);

  SurfaceFixView non_finite = full;
  non_finite.velocity_y_m_s = kInf;
  EXPECT_EQ(AssessSurfaceFix(non_finite).error, SurfaceFixError::kNonFinite);
}

TEST(SurfaceFixPolicyTest, QualityEndpoints) {
  const auto covariance = PositionCovariance();
  SurfaceFixView low = ValidFix(covariance);
  low.health.quality = 0.0;
  EXPECT_TRUE(AssessSurfaceFix(low).accepted);
  SurfaceFixView high = ValidFix(covariance);
  high.health.quality = 1.0;
  EXPECT_TRUE(AssessSurfaceFix(high).accepted);
  SurfaceFixView above = ValidFix(covariance);
  above.health.quality = std::nextafter(1.0, 2.0);
  EXPECT_EQ(AssessSurfaceFix(above).error, SurfaceFixError::kQuality);
  SurfaceFixView below = ValidFix(covariance);
  below.health.quality = -0.25;
  EXPECT_EQ(AssessSurfaceFix(below).error, SurfaceFixError::kQuality);
}

TEST(SurfaceFixPolicyTest, CovarianceAbsenceZeroAndSlots) {
  const auto covariance = PositionCovariance();
  SurfaceFixView absent = ValidFix(covariance);
  absent.health.covariance_present = false;
  absent.health.covariance = {};
  EXPECT_TRUE(AssessSurfaceFix(absent).accepted);

  const std::array<double, vehicle::kCovarianceValues> zeros = {};
  EXPECT_TRUE(AssessSurfaceFix(ValidFix(zeros)).accepted);

  const std::array<double, 35> short_values = {};
  SurfaceFixView short_sample = ValidFix(covariance);
  short_sample.health.covariance = short_values;
  EXPECT_EQ(AssessSurfaceFix(short_sample).error, SurfaceFixError::kCovariance);

  auto attitude = covariance;
  attitude[vehicle::CovarianceIndex(3, 3)] = 0.5;
  EXPECT_EQ(AssessSurfaceFix(ValidFix(attitude)).error,
            SurfaceFixError::kCovarianceSlots);
  auto cross = covariance;
  cross[vehicle::CovarianceIndex(0, 1)] = 0.0625;
  cross[vehicle::CovarianceIndex(1, 0)] = 0.0625;
  EXPECT_EQ(AssessSurfaceFix(ValidFix(cross)).error,
            SurfaceFixError::kCovarianceSlots);
}

TEST(SurfaceFixPolicyTest, MetadataDefectsUseHealthOrder) {
  const auto covariance = PositionCovariance();
  SurfaceFixView missing_frame = ValidFix(covariance);
  missing_frame.health.frame_id = "";
  EXPECT_EQ(AssessSurfaceFix(missing_frame).error,
            SurfaceFixError::kMissingFrame);

  SurfaceFixView wrong = ValidFix(covariance);
  wrong.health.expected_frame_present = true;
  wrong.health.expected_frame = embodiment::kWorldEnuFrameId;
  EXPECT_EQ(AssessSurfaceFix(wrong).error, SurfaceFixError::kWrongFrame);

  SurfaceFixView reversed = ValidFix(covariance);
  reversed.health.receive_time =
      embodiment::ClockReading{1700000000, 249999999};
  EXPECT_EQ(AssessSurfaceFix(reversed).error, SurfaceFixError::kTimeReversal);

  SurfaceFixView delayed = ValidFix(covariance);
  delayed.health.receive_time = embodiment::ClockReading{1700000100, 0};
  EXPECT_TRUE(AssessSurfaceFix(delayed).accepted);

  const SourceHealthView empty_source{"", false, 0};
  SurfaceFixView source = ValidFix(covariance);
  source.health.sources = std::span<const SourceHealthView>(&empty_source, 1);
  EXPECT_EQ(AssessSurfaceFix(source).error, SurfaceFixError::kSourceId);
}

TEST(SurfaceFixPolicyTest, MissingHealthIsRejected) {
  SurfaceFixView payload_only;
  payload_only.position_x_present = true;
  payload_only.position_x_m = 1.0;
  EXPECT_EQ(AssessSurfaceFix(payload_only).error,
            SurfaceFixError::kMissingHealth);
}

TEST(SurfaceFixPolicyTest, FirstDefectWins) {
  const auto covariance = PositionCovariance();
  SurfaceFixView frame = ValidFix(covariance);
  frame.health.frame_id = "";
  frame.position_x_present = false;
  frame.source = 99;
  EXPECT_EQ(AssessSurfaceFix(frame).error, SurfaceFixError::kMissingFrame);

  SurfaceFixView position = ValidFix(covariance);
  position.position_x_present = false;
  position.source = 99;
  EXPECT_EQ(AssessSurfaceFix(position).error, SurfaceFixError::kPosition);

  SurfaceFixView source = ValidFix(covariance);
  source.source = 99;
  source.satellite_count = -1;
  EXPECT_EQ(AssessSurfaceFix(source).error, SurfaceFixError::kSource);
}

}  // namespace
}  // namespace intrinsic::hardware::marine
