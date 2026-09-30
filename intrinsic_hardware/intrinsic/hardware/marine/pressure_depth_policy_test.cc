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

#include "intrinsic/hardware/marine/pressure_depth_policy.h"

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

std::array<double, vehicle::kCovarianceValues> PressureDepthCovariance() {
  std::array<double, vehicle::kCovarianceValues> values = {};
  values[kPressureVarianceSlot] = 1.0;
  values[kDepthVarianceSlot] = 0.25;
  return values;
}

PressureDepthMeasurementView ValidFromPressure(
    const std::array<double, vehicle::kCovarianceValues>& covariance) {
  PressureDepthMeasurementView sample;
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
  sample.pressure_present = true;
  sample.pressure_pa = 200000.0;
  sample.depth_present = true;
  sample.depth_m = 10.0;
  sample.provenance_present = true;
  sample.depth_provenance = 2;
  sample.density_present = true;
  sample.fluid_density_kg_m3 = 1025.0;
  return sample;
}

TEST(PressureDepthPolicyTest, EmptySampleIsAbsentAndNotAnError) {
  const PressureDepthAssessment assessment =
      AssessPressureDepth(PressureDepthMeasurementView{});
  EXPECT_EQ(assessment.error, PressureDepthError::kNone);
  EXPECT_EQ(assessment.state, MeasurementStateKind::kAbsent);
  EXPECT_EQ(assessment.provenance, DepthProvenanceKind::kAbsent);
  EXPECT_EQ(assessment.header_validity, ValidityKind::kAbsent);
  EXPECT_FALSE(assessment.accepted);
}

TEST(PressureDepthPolicyTest, NominalFromPressureIsAccepted) {
  const auto covariance = PressureDepthCovariance();
  const PressureDepthAssessment assessment =
      AssessPressureDepth(ValidFromPressure(covariance));
  EXPECT_EQ(assessment.error, PressureDepthError::kNone);
  EXPECT_EQ(assessment.state, MeasurementStateKind::kValid);
  EXPECT_EQ(assessment.provenance, DepthProvenanceKind::kFromPressure);
  EXPECT_EQ(assessment.header_validity, ValidityKind::kValid);
  EXPECT_TRUE(assessment.accepted);
}

TEST(PressureDepthPolicyTest, PressureOnlyAndDirectDepthAreAccepted) {
  const auto covariance = PressureDepthCovariance();
  PressureDepthMeasurementView pressure_only = ValidFromPressure(covariance);
  pressure_only.depth_present = false;
  pressure_only.depth_m = 0.0;
  pressure_only.provenance_present = false;
  pressure_only.depth_provenance = 0;
  pressure_only.density_present = false;
  pressure_only.fluid_density_kg_m3 = 0.0;
  EXPECT_TRUE(AssessPressureDepth(pressure_only).accepted);
  EXPECT_EQ(AssessPressureDepth(pressure_only).provenance,
            DepthProvenanceKind::kAbsent);

  PressureDepthMeasurementView direct = ValidFromPressure(covariance);
  direct.pressure_present = false;
  direct.pressure_pa = 0.0;
  direct.depth_provenance = 1;
  direct.density_present = false;
  const PressureDepthAssessment assessment = AssessPressureDepth(direct);
  EXPECT_TRUE(assessment.accepted);
  EXPECT_EQ(assessment.provenance, DepthProvenanceKind::kDirect);
}

TEST(PressureDepthPolicyTest, ProvenanceAbsenceIsDistinctFromUnspecified) {
  EXPECT_EQ(ClassifyDepthProvenance(false, 0), DepthProvenanceKind::kAbsent);
  EXPECT_EQ(ClassifyDepthProvenance(true, 0),
            DepthProvenanceKind::kUnspecified);
  EXPECT_EQ(ClassifyDepthProvenance(true, 1), DepthProvenanceKind::kDirect);
  EXPECT_EQ(ClassifyDepthProvenance(true, 2),
            DepthProvenanceKind::kFromPressure);
  EXPECT_EQ(ClassifyDepthProvenance(true, 100),
            DepthProvenanceKind::kUnrecognized);
  EXPECT_NE(ClassifyDepthProvenance(true, 100),
            DepthProvenanceKind::kUnspecified);

  const auto covariance = PressureDepthCovariance();
  PressureDepthMeasurementView absent = ValidFromPressure(covariance);
  absent.provenance_present = false;
  absent.depth_provenance = 0;
  EXPECT_EQ(AssessPressureDepth(absent).error, PressureDepthError::kProvenance);
  EXPECT_EQ(AssessPressureDepth(absent).provenance,
            DepthProvenanceKind::kAbsent);

  PressureDepthMeasurementView unspecified = ValidFromPressure(covariance);
  unspecified.depth_provenance = 0;
  EXPECT_EQ(AssessPressureDepth(unspecified).error,
            PressureDepthError::kProvenance);
  EXPECT_EQ(AssessPressureDepth(unspecified).provenance,
            DepthProvenanceKind::kUnspecified);

  PressureDepthMeasurementView unknown = ValidFromPressure(covariance);
  unknown.depth_provenance = 100;
  const PressureDepthAssessment assessment = AssessPressureDepth(unknown);
  EXPECT_EQ(assessment.error, PressureDepthError::kProvenance);
  EXPECT_EQ(assessment.provenance, DepthProvenanceKind::kUnrecognized);
  EXPECT_EQ(unknown.depth_provenance, 100);
}

TEST(PressureDepthPolicyTest, PressureBoundsAreFiniteOnly) {
  const auto covariance = PressureDepthCovariance();
  PressureDepthMeasurementView zero = ValidFromPressure(covariance);
  zero.pressure_pa = 0.0;
  EXPECT_TRUE(AssessPressureDepth(zero).accepted);

  PressureDepthMeasurementView negative = ValidFromPressure(covariance);
  negative.pressure_pa = -101325.0;
  EXPECT_TRUE(AssessPressureDepth(negative).accepted);

  PressureDepthMeasurementView huge = ValidFromPressure(covariance);
  huge.pressure_pa = 1.0e12;
  EXPECT_TRUE(AssessPressureDepth(huge).accepted);

  PressureDepthMeasurementView nan = ValidFromPressure(covariance);
  nan.pressure_pa = std::numeric_limits<double>::quiet_NaN();
  const PressureDepthAssessment nan_assessment = AssessPressureDepth(nan);
  EXPECT_EQ(nan_assessment.error, PressureDepthError::kNonFinite);
  EXPECT_EQ(nan_assessment.state, MeasurementStateKind::kValid);
  EXPECT_EQ(nan.health.state, 1);

  PressureDepthMeasurementView infinite = ValidFromPressure(covariance);
  infinite.pressure_pa = std::numeric_limits<double>::infinity();
  EXPECT_EQ(AssessPressureDepth(infinite).error,
            PressureDepthError::kNonFinite);
}

TEST(PressureDepthPolicyTest, DepthSurfaceZeroAndNegative) {
  const auto covariance = PressureDepthCovariance();
  PressureDepthMeasurementView surface = ValidFromPressure(covariance);
  surface.depth_m = 0.0;
  EXPECT_TRUE(AssessPressureDepth(surface).accepted);

  PressureDepthMeasurementView negative = ValidFromPressure(covariance);
  negative.depth_m = std::nextafter(0.0, -1.0);
  EXPECT_EQ(AssessPressureDepth(negative).error, PressureDepthError::kDepth);
  EXPECT_EQ(negative.health.state, 1);

  PressureDepthMeasurementView nan = ValidFromPressure(covariance);
  nan.depth_m = std::numeric_limits<double>::quiet_NaN();
  EXPECT_EQ(AssessPressureDepth(nan).error, PressureDepthError::kNonFinite);
}

TEST(PressureDepthPolicyTest, DensityAndFromPressureRules) {
  const auto covariance = PressureDepthCovariance();
  PressureDepthMeasurementView tiny = ValidFromPressure(covariance);
  tiny.fluid_density_kg_m3 = std::nextafter(0.0, 1.0);
  EXPECT_TRUE(AssessPressureDepth(tiny).accepted);

  PressureDepthMeasurementView zero = ValidFromPressure(covariance);
  zero.fluid_density_kg_m3 = 0.0;
  EXPECT_EQ(AssessPressureDepth(zero).error, PressureDepthError::kDensity);

  PressureDepthMeasurementView negative = ValidFromPressure(covariance);
  negative.fluid_density_kg_m3 = -1.0;
  EXPECT_EQ(AssessPressureDepth(negative).error, PressureDepthError::kDensity);

  PressureDepthMeasurementView nan = ValidFromPressure(covariance);
  nan.fluid_density_kg_m3 = std::numeric_limits<double>::quiet_NaN();
  EXPECT_EQ(AssessPressureDepth(nan).error, PressureDepthError::kDensity);

  PressureDepthMeasurementView missing_density = ValidFromPressure(covariance);
  missing_density.density_present = false;
  EXPECT_EQ(AssessPressureDepth(missing_density).error,
            PressureDepthError::kDensity);

  PressureDepthMeasurementView missing_pressure = ValidFromPressure(covariance);
  missing_pressure.pressure_present = false;
  EXPECT_EQ(AssessPressureDepth(missing_pressure).error,
            PressureDepthError::kProvenance);

  PressureDepthMeasurementView no_depth = ValidFromPressure(covariance);
  no_depth.depth_present = false;
  EXPECT_EQ(AssessPressureDepth(no_depth).error,
            PressureDepthError::kProvenance);

  PressureDepthMeasurementView direct_density = ValidFromPressure(covariance);
  direct_density.depth_provenance = 1;
  direct_density.density_present = false;
  EXPECT_TRUE(AssessPressureDepth(direct_density).accepted);
  direct_density.density_present = true;
  direct_density.fluid_density_kg_m3 = 0.0;
  EXPECT_EQ(AssessPressureDepth(direct_density).error,
            PressureDepthError::kDensity);

  PressureDepthMeasurementView pressure_unspecified =
      ValidFromPressure(covariance);
  pressure_unspecified.depth_present = false;
  pressure_unspecified.depth_provenance = 0;
  pressure_unspecified.density_present = false;
  EXPECT_TRUE(AssessPressureDepth(pressure_unspecified).accepted);

  PressureDepthMeasurementView pressure_unknown = pressure_unspecified;
  pressure_unknown.depth_provenance = 100;
  EXPECT_TRUE(AssessPressureDepth(pressure_unknown).accepted);
  EXPECT_EQ(AssessPressureDepth(pressure_unknown).provenance,
            DepthProvenanceKind::kUnrecognized);
}

TEST(PressureDepthPolicyTest, QualityEndpointsAndNeitherPayload) {
  const auto covariance = PressureDepthCovariance();
  PressureDepthMeasurementView low = ValidFromPressure(covariance);
  low.health.quality = 0.0;
  EXPECT_TRUE(AssessPressureDepth(low).accepted);
  PressureDepthMeasurementView high = ValidFromPressure(covariance);
  high.health.quality = 1.0;
  EXPECT_TRUE(AssessPressureDepth(high).accepted);
  PressureDepthMeasurementView above = ValidFromPressure(covariance);
  above.health.quality = std::nextafter(1.0, 2.0);
  EXPECT_EQ(AssessPressureDepth(above).error, PressureDepthError::kQuality);

  PressureDepthMeasurementView provenance_only = ValidFromPressure(covariance);
  provenance_only.pressure_present = false;
  provenance_only.depth_present = false;
  provenance_only.pressure_pa = std::numeric_limits<double>::quiet_NaN();
  EXPECT_EQ(AssessPressureDepth(provenance_only).error,
            PressureDepthError::kPayload);

  PressureDepthMeasurementView invalid = ValidFromPressure(covariance);
  invalid.health.state = 3;
  invalid.pressure_pa = -5.0;
  const PressureDepthAssessment marked = AssessPressureDepth(invalid);
  EXPECT_EQ(marked.error, PressureDepthError::kNone);
  EXPECT_EQ(marked.state, MeasurementStateKind::kInvalid);
  EXPECT_FALSE(marked.accepted);
  EXPECT_EQ(invalid.health.state, 3);
}

TEST(PressureDepthPolicyTest, CovarianceAbsenceZeroAndSlots) {
  std::array<double, vehicle::kCovarianceValues> none = {};
  PressureDepthMeasurementView absent = ValidFromPressure(none);
  absent.health.covariance_present = false;
  EXPECT_TRUE(AssessPressureDepth(absent).accepted);
  EXPECT_TRUE(vehicle::CovarianceIsUnknown(
      vehicle::AssessCovariance(false, absent.health.covariance)));

  const std::array<double, vehicle::kCovarianceValues> zeros = {};
  PressureDepthMeasurementView zero = ValidFromPressure(zeros);
  EXPECT_TRUE(AssessPressureDepth(zero).accepted);
  EXPECT_TRUE(vehicle::IsAllZeroCovariance(zeros));
  EXPECT_FALSE(
      vehicle::CovarianceIsUnknown(vehicle::AssessCovariance(true, zeros)));

  std::array<double, 35> short_values = {};
  PressureDepthMeasurementView bad_shape = ValidFromPressure(none);
  bad_shape.health.covariance = short_values;
  EXPECT_EQ(AssessPressureDepth(bad_shape).error,
            PressureDepthError::kCovariance);

  auto off_diagonal = PressureDepthCovariance();
  off_diagonal[vehicle::CovarianceIndex(2, 2)] = 0.25;
  PressureDepthMeasurementView slots = ValidFromPressure(off_diagonal);
  EXPECT_EQ(AssessPressureDepth(slots).error,
            PressureDepthError::kCovarianceSlots);

  auto cross = PressureDepthCovariance();
  cross[vehicle::CovarianceIndex(0, 1)] = 0.1;
  cross[vehicle::CovarianceIndex(1, 0)] = 0.1;
  PressureDepthMeasurementView symmetric = ValidFromPressure(cross);
  EXPECT_EQ(AssessPressureDepth(symmetric).error,
            PressureDepthError::kCovarianceSlots);
}

TEST(PressureDepthPolicyTest, MetadataDefectsUseHealthOrder) {
  const auto covariance = PressureDepthCovariance();
  PressureDepthMeasurementView missing_frame = ValidFromPressure(covariance);
  missing_frame.health.frame_id = "";
  EXPECT_EQ(AssessPressureDepth(missing_frame).error,
            PressureDepthError::kMissingFrame);

  PressureDepthMeasurementView wrong = ValidFromPressure(covariance);
  wrong.health.expected_frame_present = true;
  wrong.health.expected_frame = embodiment::kWorldEnuFrameId;
  EXPECT_EQ(AssessPressureDepth(wrong).error, PressureDepthError::kWrongFrame);
  wrong.health.frame_id = embodiment::kWorldEnuFrameId;
  EXPECT_TRUE(AssessPressureDepth(wrong).accepted);

  PressureDepthMeasurementView reversed = ValidFromPressure(covariance);
  reversed.health.receive_time =
      embodiment::ClockReading{1700000000, 249999999};
  EXPECT_EQ(AssessPressureDepth(reversed).error,
            PressureDepthError::kTimeReversal);

  PressureDepthMeasurementView delayed = ValidFromPressure(covariance);
  delayed.health.receive_time = embodiment::ClockReading{1700000100, 0};
  EXPECT_TRUE(AssessPressureDepth(delayed).accepted);

  PressureDepthMeasurementView quality_nan = ValidFromPressure(covariance);
  quality_nan.health.quality = std::numeric_limits<double>::quiet_NaN();
  EXPECT_EQ(AssessPressureDepth(quality_nan).error,
            PressureDepthError::kNonFinite);
  EXPECT_EQ(AssessPressureDepth(quality_nan).state,
            MeasurementStateKind::kValid);

  SourceHealthView empty_source{"", false, 0};
  PressureDepthMeasurementView source = ValidFromPressure(covariance);
  source.health.sources = std::span<const SourceHealthView>(&empty_source, 1);
  EXPECT_EQ(AssessPressureDepth(source).error, PressureDepthError::kSourceId);
}

TEST(PressureDepthPolicyTest, MissingHealthPrecedesPayloadAndValidityStaysPut) {
  PressureDepthMeasurementView payload_only;
  payload_only.pressure_present = true;
  payload_only.pressure_pa = 1.0;
  EXPECT_EQ(AssessPressureDepth(payload_only).error,
            PressureDepthError::kMissingHealth);

  const auto covariance = PressureDepthCovariance();
  PressureDepthMeasurementView sample = ValidFromPressure(covariance);
  sample.health.header_validity_state = 2;
  const PressureDepthAssessment invalid_header = AssessPressureDepth(sample);
  EXPECT_TRUE(invalid_header.accepted);
  EXPECT_EQ(invalid_header.header_validity, ValidityKind::kInvalid);

  sample.health.state = 2;
  sample.health.header_validity_present = false;
  const PressureDepthAssessment degraded = AssessPressureDepth(sample);
  EXPECT_EQ(degraded.error, PressureDepthError::kNone);
  EXPECT_EQ(degraded.state, MeasurementStateKind::kDegraded);
  EXPECT_FALSE(degraded.accepted);
  EXPECT_EQ(sample.health.state, 2);

  EXPECT_EQ(embodiment::ClassifyValidity(true, 3), ValidityKind::kUnspecified);
  EXPECT_NE(embodiment::ClassifyValidity(true, 3), ValidityKind::kInvalid);
}

TEST(PressureDepthPolicyTest, FirstDefectWins) {
  const auto covariance = PressureDepthCovariance();
  PressureDepthMeasurementView sample = ValidFromPressure(covariance);
  sample.health.frame_id = "";
  sample.depth_m = -1.0;
  EXPECT_EQ(AssessPressureDepth(sample).error,
            PressureDepthError::kMissingFrame);

  sample = ValidFromPressure(covariance);
  sample.health.quality = 2.0;
  sample.depth_provenance = 0;
  EXPECT_EQ(AssessPressureDepth(sample).error, PressureDepthError::kQuality);

  auto slotted = PressureDepthCovariance();
  slotted[kDepthVarianceSlot + 1] = 0.0;
  slotted[vehicle::CovarianceIndex(2, 2)] = 1.0;
  sample = ValidFromPressure(slotted);
  sample.depth_m = -1.0;
  EXPECT_EQ(AssessPressureDepth(sample).error,
            PressureDepthError::kCovarianceSlots);

  sample = ValidFromPressure(covariance);
  sample.pressure_present = false;
  sample.depth_present = false;
  sample.depth_m = -1.0;
  EXPECT_EQ(AssessPressureDepth(sample).error, PressureDepthError::kPayload);

  sample = ValidFromPressure(covariance);
  sample.pressure_pa = std::numeric_limits<double>::quiet_NaN();
  sample.depth_m = -1.0;
  EXPECT_EQ(AssessPressureDepth(sample).error, PressureDepthError::kNonFinite);

  sample = ValidFromPressure(covariance);
  sample.depth_m = -1.0;
  sample.depth_provenance = 0;
  EXPECT_EQ(AssessPressureDepth(sample).error, PressureDepthError::kDepth);

  sample = ValidFromPressure(covariance);
  sample.depth_provenance = 0;
  sample.fluid_density_kg_m3 = 0.0;
  EXPECT_EQ(AssessPressureDepth(sample).error, PressureDepthError::kProvenance);

  sample = ValidFromPressure(covariance);
  sample.pressure_present = false;
  sample.fluid_density_kg_m3 = -1.0;
  EXPECT_EQ(AssessPressureDepth(sample).error, PressureDepthError::kProvenance);

  sample = ValidFromPressure(covariance);
  sample.density_present = false;
  EXPECT_EQ(AssessPressureDepth(sample).error, PressureDepthError::kDensity);
}

}  // namespace
}  // namespace intrinsic::hardware::marine
