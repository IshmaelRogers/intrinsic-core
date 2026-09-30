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

#include "intrinsic/world/bathymetry_reference_component/bathymetry_reference_component_policy.h"

#include <cmath>
#include <limits>

#include "gtest/gtest.h"
#include "intrinsic/world/marine_component_validity/marine_component_validity_policy.h"

namespace intrinsic::world {
namespace {

MarineComponentValidityView ValidMeta() {
  MarineComponentValidityView view;
  view.present = true;
  view.source_id = "fusion_0";
  // 1000.5s + 1.6s = 1002.1s.
  view.observation_time_present = true;
  view.observation_time = TimeParts{1000, 500000000};
  view.validity_horizon_present = true;
  view.validity_horizon = TimeParts{1, 600000000};
  view.confidence_present = true;
  view.confidence = 0.75;
  view.uncertainty_reference = "cov-ref-7";
  view.validity_present = true;
  view.validity_state = 1;
  return view;
}

BathymetryReferenceView ValidView() {
  BathymetryReferenceView view;
  view.present = true;
  view.validity_meta = ValidMeta();
  view.frame_id = "world_enu";
  view.bathymetry_asset_ref = "cas:bathy-grid-1";
  view.reference_z_present = true;
  view.reference_z_m = -12.5;
  return view;
}

TimeParts Deadline() { return TimeParts{1002, 100000000}; }

TEST(BathymetryReferencePolicyTest, EmptyMessageIsNotAnError) {
  const BathymetryReferenceAssessment assessment =
      AssessBathymetryReference(BathymetryReferenceView{}, Deadline());
  EXPECT_EQ(assessment.error, BathymetryReferenceError::kNone);
  EXPECT_EQ(assessment.validity.error, ComponentValidityError::kNone);
  EXPECT_EQ(assessment.validity.freshness, ComponentFreshness::kUnknown);
  EXPECT_FALSE(assessment.accepted);
}

TEST(BathymetryReferencePolicyTest, MissingValidityMeta) {
  BathymetryReferenceView view = ValidView();
  view.validity_meta = MarineComponentValidityView{};
  const BathymetryReferenceAssessment assessment =
      AssessBathymetryReference(view, Deadline());
  EXPECT_EQ(assessment.error, BathymetryReferenceError::kValidity);
  EXPECT_FALSE(assessment.validity.accepted);
  EXPECT_FALSE(assessment.accepted);
}

TEST(BathymetryReferencePolicyTest, ValidityDefectIsDelegated) {
  BathymetryReferenceView view = ValidView();
  view.validity_meta.source_id = "";
  const BathymetryReferenceAssessment assessment =
      AssessBathymetryReference(view, Deadline());
  EXPECT_EQ(assessment.error, BathymetryReferenceError::kValidity);
  EXPECT_EQ(assessment.validity.error, ComponentValidityError::kSourceId);
  EXPECT_EQ(
      assessment.validity.freshness,
      AssessMarineComponentValidity(view.validity_meta, Deadline()).freshness);
  EXPECT_FALSE(assessment.accepted);
}

TEST(BathymetryReferencePolicyTest, FirstDefectWins) {
  BathymetryReferenceView view = ValidView();
  view.validity_meta.source_id = "";
  view.frame_id = "";
  view.bathymetry_asset_ref = "";
  view.reference_z_m = std::numeric_limits<double>::quiet_NaN();
  EXPECT_EQ(AssessBathymetryReference(view, Deadline()).error,
            BathymetryReferenceError::kValidity);

  view.validity_meta.source_id = "fusion_0";
  EXPECT_EQ(AssessBathymetryReference(view, Deadline()).error,
            BathymetryReferenceError::kFrameId);

  view.frame_id = "world_enu";
  EXPECT_EQ(AssessBathymetryReference(view, Deadline()).error,
            BathymetryReferenceError::kAssetRef);

  view.bathymetry_asset_ref = "cas:bathy-grid-1";
  EXPECT_EQ(AssessBathymetryReference(view, Deadline()).error,
            BathymetryReferenceError::kReferenceZ);
}

TEST(BathymetryReferencePolicyTest, RejectsEmptyFrameAndAsset) {
  BathymetryReferenceView view = ValidView();
  view.frame_id = "";
  EXPECT_EQ(AssessBathymetryReference(view, Deadline()).error,
            BathymetryReferenceError::kFrameId);
  view.frame_id = "world_enu";
  view.bathymetry_asset_ref = "";
  EXPECT_EQ(AssessBathymetryReference(view, Deadline()).error,
            BathymetryReferenceError::kAssetRef);
}

TEST(BathymetryReferencePolicyTest, ReferenceZFiniteOnlyWhenSet) {
  BathymetryReferenceView view = ValidView();
  view.reference_z_present = false;
  view.reference_z_m = std::numeric_limits<double>::quiet_NaN();
  EXPECT_EQ(AssessBathymetryReference(view, Deadline()).error,
            BathymetryReferenceError::kNone);
  EXPECT_TRUE(AssessBathymetryReference(view, Deadline()).accepted);

  view.reference_z_present = true;
  for (double z : {0.0, -0.0, -12.5, 3.0}) {
    view.reference_z_m = z;
    EXPECT_TRUE(AssessBathymetryReference(view, Deadline()).accepted) << z;
  }
  for (double z : {std::numeric_limits<double>::quiet_NaN(),
                   std::numeric_limits<double>::infinity(),
                   -std::numeric_limits<double>::infinity()}) {
    view.reference_z_m = z;
    EXPECT_EQ(AssessBathymetryReference(view, Deadline()).error,
              BathymetryReferenceError::kReferenceZ)
        << z;
  }
}

TEST(BathymetryReferencePolicyTest, AssetRefIsOpaque) {
  BathymetryReferenceView view = ValidView();
  view.bathymetry_asset_ref = "1,nan,inf";
  EXPECT_TRUE(AssessBathymetryReference(view, Deadline()).accepted);
}

TEST(BathymetryReferencePolicyTest, NonEmptyFrameIsNotRewritten) {
  BathymetryReferenceView view = ValidView();
  view.frame_id = "map_enu";
  const BathymetryReferenceAssessment assessment =
      AssessBathymetryReference(view, Deadline());
  EXPECT_TRUE(assessment.accepted);
  EXPECT_EQ(view.frame_id, "map_enu");
}

TEST(BathymetryReferencePolicyTest, ExactHorizonBoundaryIsDelegated) {
  const BathymetryReferenceView view = ValidView();
  const TimeParts deadline = Deadline();
  BathymetryReferenceAssessment assessment =
      AssessBathymetryReference(view, deadline);
  EXPECT_EQ(assessment.error, BathymetryReferenceError::kNone);
  EXPECT_EQ(assessment.validity.freshness, ComponentFreshness::kFresh);
  EXPECT_TRUE(assessment.accepted);
  EXPECT_EQ(
      assessment.validity.freshness,
      AssessMarineComponentValidity(view.validity_meta, deadline).freshness);

  const TimeParts after = TimeParts{deadline.seconds, deadline.nanos + 1};
  assessment = AssessBathymetryReference(view, after);
  EXPECT_EQ(assessment.error, BathymetryReferenceError::kNone);
  EXPECT_EQ(assessment.validity.freshness, ComponentFreshness::kExpired);
  EXPECT_EQ(assessment.validity.validity, embodiment::ValidityKind::kValid);
  EXPECT_FALSE(assessment.accepted);
  EXPECT_EQ(assessment.validity.freshness,
            AssessMarineComponentValidity(view.validity_meta, after).freshness);
}

TEST(BathymetryReferencePolicyTest, UnknownIsDistinctFromExpired) {
  BathymetryReferenceView view = ValidView();
  const TimeParts after = TimeParts{1002, 100000001};
  view.validity_meta.validity_present = false;
  BathymetryReferenceAssessment assessment =
      AssessBathymetryReference(view, after);
  EXPECT_EQ(assessment.error, BathymetryReferenceError::kNone);
  EXPECT_EQ(assessment.validity.freshness, ComponentFreshness::kUnknown);
  EXPECT_EQ(assessment.validity.validity, embodiment::ValidityKind::kAbsent);
  EXPECT_NE(assessment.validity.freshness, ComponentFreshness::kExpired);
  EXPECT_FALSE(assessment.accepted);

  view = ValidView();
  view.validity_meta.validity_state = 2;
  assessment = AssessBathymetryReference(view, Deadline());
  EXPECT_EQ(assessment.error, BathymetryReferenceError::kNone);
  EXPECT_EQ(assessment.validity.validity, embodiment::ValidityKind::kInvalid);
  EXPECT_EQ(assessment.validity.freshness, ComponentFreshness::kFresh);
  EXPECT_FALSE(assessment.accepted);

  view.validity_meta.validity_state = 99;
  assessment = AssessBathymetryReference(view, after);
  EXPECT_EQ(assessment.validity.validity,
            embodiment::ValidityKind::kUnspecified);
  EXPECT_EQ(assessment.validity.freshness, ComponentFreshness::kExpired);
  EXPECT_FALSE(assessment.accepted);
}

TEST(BathymetryReferencePolicyTest, NonsenseHorizonStaysUnknown) {
  BathymetryReferenceView view = ValidView();
  view.validity_meta.validity_horizon = TimeParts{-1, 0};
  const BathymetryReferenceAssessment assessment =
      AssessBathymetryReference(view, Deadline());
  EXPECT_EQ(assessment.error, BathymetryReferenceError::kValidity);
  EXPECT_EQ(assessment.validity.error, ComponentValidityError::kHorizon);
  EXPECT_EQ(assessment.validity.freshness, ComponentFreshness::kUnknown);
  EXPECT_FALSE(assessment.accepted);
}

}  // namespace
}  // namespace intrinsic::world
