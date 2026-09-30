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

#include "intrinsic/hardware/marine/altimeter_policy.h"

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

std::array<double, vehicle::kCovarianceValues> RangeCovariance() {
  std::array<double, vehicle::kCovarianceValues> values = {};
  values[kRangeVarianceSlot] = 0.25;
  return values;
}

AltimeterMeasurementView ValidRange(
    const std::array<double, vehicle::kCovarianceValues>& covariance) {
  AltimeterMeasurementView sample;
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
  sample.range_present = true;
  sample.range_m = 10.0;
  sample.beam_present = true;
  sample.beam_id = "down";
  sample.min_present = true;
  sample.min_range_m = 0.5;
  sample.max_present = true;
  sample.max_range_m = 100.0;
  sample.has_return_present = true;
  sample.has_return = true;
  return sample;
}

TEST(AltimeterPolicyTest, EmptySampleIsAbsentAndNotAnError) {
  const AltimeterAssessment assessment =
      AssessAltimeter(AltimeterMeasurementView{});
  EXPECT_EQ(assessment.error, AltimeterError::kNone);
  EXPECT_EQ(assessment.state, MeasurementStateKind::kAbsent);
  EXPECT_EQ(assessment.header_validity, ValidityKind::kAbsent);
  EXPECT_FALSE(assessment.accepted);
}

TEST(AltimeterPolicyTest, NominalRangeIsAccepted) {
  const auto covariance = RangeCovariance();
  const AltimeterAssessment assessment =
      AssessAltimeter(ValidRange(covariance));
  EXPECT_EQ(assessment.error, AltimeterError::kNone);
  EXPECT_EQ(assessment.state, MeasurementStateKind::kValid);
  EXPECT_EQ(assessment.header_validity, ValidityKind::kValid);
  EXPECT_TRUE(assessment.accepted);
}

TEST(AltimeterPolicyTest, RangeOnlyAndReturnFlags) {
  const auto covariance = RangeCovariance();
  AltimeterMeasurementView range_only = ValidRange(covariance);
  range_only.has_return_present = false;
  range_only.has_return = false;
  EXPECT_TRUE(AssessAltimeter(range_only).accepted);
  EXPECT_FALSE(ExplicitNoReturn(range_only));

  AltimeterMeasurementView returned = ValidRange(covariance);
  returned.has_return = true;
  EXPECT_TRUE(AssessAltimeter(returned).accepted);

  AltimeterMeasurementView true_without_range = ValidRange(covariance);
  true_without_range.range_present = false;
  true_without_range.range_m = 0.0;
  EXPECT_EQ(AssessAltimeter(true_without_range).error,
            AltimeterError::kPayload);
}

TEST(AltimeterPolicyTest, RangeEndpointsAndSign) {
  const auto covariance = RangeCovariance();
  AltimeterMeasurementView contact = ValidRange(covariance);
  contact.range_m = 0.0;
  contact.min_range_m = 0.0;
  EXPECT_TRUE(AssessAltimeter(contact).accepted);

  AltimeterMeasurementView at_min = ValidRange(covariance);
  at_min.range_m = at_min.min_range_m;
  EXPECT_TRUE(AssessAltimeter(at_min).accepted);

  AltimeterMeasurementView at_max = ValidRange(covariance);
  at_max.range_m = at_max.max_range_m;
  EXPECT_TRUE(AssessAltimeter(at_max).accepted);

  AltimeterMeasurementView equal_bounds = ValidRange(covariance);
  equal_bounds.min_range_m = 10.0;
  equal_bounds.max_range_m = 10.0;
  equal_bounds.range_m = 10.0;
  EXPECT_TRUE(AssessAltimeter(equal_bounds).accepted);

  AltimeterMeasurementView negative = ValidRange(covariance);
  negative.range_m = std::nextafter(0.0, -1.0);
  const AltimeterAssessment negative_assessment = AssessAltimeter(negative);
  EXPECT_EQ(negative_assessment.error, AltimeterError::kRange);
  EXPECT_EQ(negative_assessment.state, MeasurementStateKind::kValid);
  EXPECT_EQ(negative.health.state, 1);

  AltimeterMeasurementView nan = ValidRange(covariance);
  nan.range_m = std::numeric_limits<double>::quiet_NaN();
  EXPECT_EQ(AssessAltimeter(nan).error, AltimeterError::kNonFinite);

  AltimeterMeasurementView infinite = ValidRange(covariance);
  infinite.range_m = std::numeric_limits<double>::infinity();
  EXPECT_EQ(AssessAltimeter(infinite).error, AltimeterError::kNonFinite);
}

TEST(AltimeterPolicyTest, BoundsAndOutOfRange) {
  const auto covariance = RangeCovariance();
  AltimeterMeasurementView below = ValidRange(covariance);
  below.range_m = std::nextafter(0.5, 0.0);
  EXPECT_EQ(AssessAltimeter(below).error, AltimeterError::kBounds);

  AltimeterMeasurementView above = ValidRange(covariance);
  above.range_m = std::nextafter(100.0, 200.0);
  EXPECT_EQ(AssessAltimeter(above).error, AltimeterError::kBounds);
  EXPECT_EQ(above.health.state, 1);

  AltimeterMeasurementView min_only = ValidRange(covariance);
  min_only.max_present = false;
  min_only.range_m = 0.5;
  EXPECT_TRUE(AssessAltimeter(min_only).accepted);
  min_only.range_m = 0.0;
  EXPECT_EQ(AssessAltimeter(min_only).error, AltimeterError::kBounds);

  AltimeterMeasurementView max_only = ValidRange(covariance);
  max_only.min_present = false;
  max_only.range_m = 100.0;
  EXPECT_TRUE(AssessAltimeter(max_only).accepted);
  max_only.range_m = 100.0 + std::numeric_limits<double>::epsilon() * 128.0;
  EXPECT_EQ(AssessAltimeter(max_only).error, AltimeterError::kBounds);

  AltimeterMeasurementView swapped = ValidRange(covariance);
  swapped.min_range_m = 20.0;
  swapped.max_range_m = 10.0;
  swapped.range_m = 15.0;
  EXPECT_EQ(AssessAltimeter(swapped).error, AltimeterError::kBounds);

  AltimeterMeasurementView negative_min = ValidRange(covariance);
  negative_min.min_range_m = -0.1;
  EXPECT_EQ(AssessAltimeter(negative_min).error, AltimeterError::kBounds);

  AltimeterMeasurementView negative_max = ValidRange(covariance);
  negative_max.max_range_m = -1.0;
  EXPECT_EQ(AssessAltimeter(negative_max).error, AltimeterError::kBounds);

  AltimeterMeasurementView nan_min = ValidRange(covariance);
  nan_min.min_range_m = std::numeric_limits<double>::quiet_NaN();
  EXPECT_EQ(AssessAltimeter(nan_min).error, AltimeterError::kNonFinite);

  AltimeterMeasurementView nan_max = ValidRange(covariance);
  nan_max.max_range_m = std::numeric_limits<double>::infinity();
  EXPECT_EQ(AssessAltimeter(nan_max).error, AltimeterError::kNonFinite);
}

TEST(AltimeterPolicyTest, NoReturnConsistency) {
  const auto covariance = RangeCovariance();
  AltimeterMeasurementView no_return = ValidRange(covariance);
  no_return.range_present = false;
  no_return.range_m = 0.0;
  no_return.has_return = false;
  no_return.health.state = 3;
  const AltimeterAssessment consistent = AssessAltimeter(no_return);
  EXPECT_EQ(consistent.error, AltimeterError::kNone);
  EXPECT_EQ(consistent.state, MeasurementStateKind::kInvalid);
  EXPECT_FALSE(consistent.accepted);
  EXPECT_EQ(no_return.health.state, 3);

  AltimeterMeasurementView valid_loss = no_return;
  valid_loss.health.state = 1;
  EXPECT_EQ(AssessAltimeter(valid_loss).error, AltimeterError::kNoReturn);
  EXPECT_EQ(valid_loss.health.state, 1);

  AltimeterMeasurementView degraded = no_return;
  degraded.health.state = 2;
  const AltimeterAssessment degraded_assessment = AssessAltimeter(degraded);
  EXPECT_EQ(degraded_assessment.error, AltimeterError::kNone);
  EXPECT_EQ(degraded_assessment.state, MeasurementStateKind::kDegraded);
  EXPECT_FALSE(degraded_assessment.accepted);

  AltimeterMeasurementView with_range = ValidRange(covariance);
  with_range.has_return = false;
  with_range.health.state = 3;
  EXPECT_EQ(AssessAltimeter(with_range).error, AltimeterError::kNoReturn);
  EXPECT_EQ(with_range.health.state, 3);
}

TEST(AltimeterPolicyTest, BeamIdPresence) {
  const auto covariance = RangeCovariance();
  AltimeterMeasurementView absent = ValidRange(covariance);
  absent.beam_present = false;
  absent.beam_id = "";
  EXPECT_TRUE(AssessAltimeter(absent).accepted);

  AltimeterMeasurementView empty = ValidRange(covariance);
  empty.beam_id = "";
  EXPECT_EQ(AssessAltimeter(empty).error, AltimeterError::kBeamId);
  EXPECT_EQ(empty.health.state, 1);

  AltimeterMeasurementView named = ValidRange(covariance);
  named.beam_id = "beam-2";
  EXPECT_TRUE(AssessAltimeter(named).accepted);
}

TEST(AltimeterPolicyTest, QualityEndpointsAndNeitherPayload) {
  const auto covariance = RangeCovariance();
  AltimeterMeasurementView low = ValidRange(covariance);
  low.health.quality = 0.0;
  EXPECT_TRUE(AssessAltimeter(low).accepted);
  AltimeterMeasurementView high = ValidRange(covariance);
  high.health.quality = 1.0;
  EXPECT_TRUE(AssessAltimeter(high).accepted);
  AltimeterMeasurementView above = ValidRange(covariance);
  above.health.quality = std::nextafter(1.0, 2.0);
  EXPECT_EQ(AssessAltimeter(above).error, AltimeterError::kQuality);

  AltimeterMeasurementView beam_only = ValidRange(covariance);
  beam_only.range_present = false;
  beam_only.has_return_present = false;
  beam_only.range_m = std::numeric_limits<double>::quiet_NaN();
  EXPECT_EQ(AssessAltimeter(beam_only).error, AltimeterError::kPayload);

  AltimeterMeasurementView invalid = ValidRange(covariance);
  invalid.health.state = 3;
  invalid.range_m = -5.0;
  const AltimeterAssessment marked = AssessAltimeter(invalid);
  EXPECT_EQ(marked.error, AltimeterError::kRange);
  EXPECT_EQ(marked.state, MeasurementStateKind::kInvalid);
  EXPECT_FALSE(marked.accepted);
  EXPECT_EQ(invalid.health.state, 3);
}

TEST(AltimeterPolicyTest, CovarianceAbsenceZeroAndSlots) {
  std::array<double, vehicle::kCovarianceValues> none = {};
  AltimeterMeasurementView absent = ValidRange(none);
  absent.health.covariance_present = false;
  EXPECT_TRUE(AssessAltimeter(absent).accepted);
  EXPECT_TRUE(vehicle::CovarianceIsUnknown(
      vehicle::AssessCovariance(false, absent.health.covariance)));

  const std::array<double, vehicle::kCovarianceValues> zeros = {};
  AltimeterMeasurementView zero = ValidRange(zeros);
  EXPECT_TRUE(AssessAltimeter(zero).accepted);
  EXPECT_TRUE(vehicle::IsAllZeroCovariance(zeros));
  EXPECT_FALSE(
      vehicle::CovarianceIsUnknown(vehicle::AssessCovariance(true, zeros)));

  std::array<double, 35> short_values = {};
  AltimeterMeasurementView bad_shape = ValidRange(none);
  bad_shape.health.covariance = short_values;
  EXPECT_EQ(AssessAltimeter(bad_shape).error, AltimeterError::kCovariance);

  auto depth_slot = RangeCovariance();
  depth_slot[vehicle::CovarianceIndex(1, 1)] = 0.25;
  AltimeterMeasurementView slots = ValidRange(depth_slot);
  EXPECT_EQ(AssessAltimeter(slots).error, AltimeterError::kCovarianceSlots);

  auto cross = RangeCovariance();
  cross[vehicle::CovarianceIndex(0, 1)] = 0.1;
  cross[vehicle::CovarianceIndex(1, 0)] = 0.1;
  AltimeterMeasurementView symmetric = ValidRange(cross);
  EXPECT_EQ(AssessAltimeter(symmetric).error, AltimeterError::kCovarianceSlots);

  auto asymmetric = RangeCovariance();
  asymmetric[vehicle::CovarianceIndex(0, 1)] = 1.0;
  AltimeterMeasurementView skewed = ValidRange(asymmetric);
  EXPECT_EQ(AssessAltimeter(skewed).error, AltimeterError::kCovariance);
}

TEST(AltimeterPolicyTest, MetadataDefectsUseHealthOrder) {
  const auto covariance = RangeCovariance();
  AltimeterMeasurementView missing_frame = ValidRange(covariance);
  missing_frame.health.frame_id = "";
  EXPECT_EQ(AssessAltimeter(missing_frame).error,
            AltimeterError::kMissingFrame);

  AltimeterMeasurementView wrong = ValidRange(covariance);
  wrong.health.expected_frame_present = true;
  wrong.health.expected_frame = embodiment::kWorldEnuFrameId;
  EXPECT_EQ(AssessAltimeter(wrong).error, AltimeterError::kWrongFrame);
  wrong.health.frame_id = embodiment::kWorldEnuFrameId;
  EXPECT_TRUE(AssessAltimeter(wrong).accepted);

  AltimeterMeasurementView reversed = ValidRange(covariance);
  reversed.health.receive_time =
      embodiment::ClockReading{1700000000, 249999999};
  EXPECT_EQ(AssessAltimeter(reversed).error, AltimeterError::kTimeReversal);

  AltimeterMeasurementView delayed = ValidRange(covariance);
  delayed.health.receive_time = embodiment::ClockReading{1700000100, 0};
  EXPECT_TRUE(AssessAltimeter(delayed).accepted);

  AltimeterMeasurementView quality_nan = ValidRange(covariance);
  quality_nan.health.quality = std::numeric_limits<double>::quiet_NaN();
  EXPECT_EQ(AssessAltimeter(quality_nan).error, AltimeterError::kNonFinite);
  EXPECT_EQ(AssessAltimeter(quality_nan).state, MeasurementStateKind::kValid);

  SourceHealthView empty_source{"", false, 0};
  AltimeterMeasurementView source = ValidRange(covariance);
  source.health.sources = std::span<const SourceHealthView>(&empty_source, 1);
  EXPECT_EQ(AssessAltimeter(source).error, AltimeterError::kSourceId);
}

TEST(AltimeterPolicyTest, MissingHealthPrecedesPayloadAndValidityStaysPut) {
  AltimeterMeasurementView payload_only;
  payload_only.range_present = true;
  payload_only.range_m = 1.0;
  EXPECT_EQ(AssessAltimeter(payload_only).error,
            AltimeterError::kMissingHealth);

  const auto covariance = RangeCovariance();
  AltimeterMeasurementView sample = ValidRange(covariance);
  sample.health.header_validity_state = 2;
  const AltimeterAssessment invalid_header = AssessAltimeter(sample);
  EXPECT_TRUE(invalid_header.accepted);
  EXPECT_EQ(invalid_header.header_validity, ValidityKind::kInvalid);

  sample.health.state = 2;
  sample.health.header_validity_present = false;
  const AltimeterAssessment degraded = AssessAltimeter(sample);
  EXPECT_EQ(degraded.error, AltimeterError::kNone);
  EXPECT_EQ(degraded.state, MeasurementStateKind::kDegraded);
  EXPECT_FALSE(degraded.accepted);
  EXPECT_EQ(sample.health.state, 2);

  EXPECT_EQ(embodiment::ClassifyValidity(true, 3), ValidityKind::kUnspecified);
  EXPECT_NE(embodiment::ClassifyValidity(true, 3), ValidityKind::kInvalid);
}

TEST(AltimeterPolicyTest, FirstDefectWins) {
  const auto covariance = RangeCovariance();
  AltimeterMeasurementView sample = ValidRange(covariance);
  sample.health.frame_id = "";
  sample.range_m = -1.0;
  EXPECT_EQ(AssessAltimeter(sample).error, AltimeterError::kMissingFrame);

  sample = ValidRange(covariance);
  sample.health.quality = 2.0;
  sample.range_m = -1.0;
  EXPECT_EQ(AssessAltimeter(sample).error, AltimeterError::kQuality);

  auto slotted = RangeCovariance();
  slotted[vehicle::CovarianceIndex(2, 2)] = 1.0;
  sample = ValidRange(slotted);
  sample.range_m = -1.0;
  EXPECT_EQ(AssessAltimeter(sample).error, AltimeterError::kCovarianceSlots);

  sample = ValidRange(covariance);
  sample.range_present = false;
  sample.has_return_present = false;
  sample.range_m = -1.0;
  sample.beam_id = "";
  EXPECT_EQ(AssessAltimeter(sample).error, AltimeterError::kPayload);

  sample = ValidRange(covariance);
  sample.range_m = std::numeric_limits<double>::quiet_NaN();
  sample.min_range_m = -1.0;
  EXPECT_EQ(AssessAltimeter(sample).error, AltimeterError::kNonFinite);

  sample = ValidRange(covariance);
  sample.range_m = -1.0;
  sample.min_range_m = -2.0;
  EXPECT_EQ(AssessAltimeter(sample).error, AltimeterError::kRange);

  sample = ValidRange(covariance);
  sample.min_range_m = -1.0;
  sample.max_range_m = -2.0;
  EXPECT_EQ(AssessAltimeter(sample).error, AltimeterError::kBounds);

  sample = ValidRange(covariance);
  sample.range_m = 200.0;
  sample.has_return = false;
  sample.health.state = 3;
  EXPECT_EQ(AssessAltimeter(sample).error, AltimeterError::kBounds);

  sample = ValidRange(covariance);
  sample.has_return = false;
  sample.health.state = 1;
  sample.beam_id = "";
  EXPECT_EQ(AssessAltimeter(sample).error, AltimeterError::kNoReturn);

  sample = ValidRange(covariance);
  sample.has_return = false;
  sample.health.state = 3;
  sample.beam_id = "";
  EXPECT_EQ(AssessAltimeter(sample).error, AltimeterError::kNoReturn);
}

}  // namespace
}  // namespace intrinsic::hardware::marine
