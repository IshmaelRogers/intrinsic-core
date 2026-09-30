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

#include "intrinsic/world/occupancy_reference_component/occupancy_reference_component_policy.h"

#include <string_view>

#include "gtest/gtest.h"
#include "intrinsic/embodiment/frame_policy.h"
#include "intrinsic/embodiment/stamped_header_policy.h"

namespace intrinsic::world {
namespace {

using embodiment::ValidityKind;

MarineComponentValidityView ValidValidity() {
  MarineComponentValidityView view;
  view.present = true;
  view.source_id = "fusion_0";
  view.observation_time_present = true;
  view.observation_time = TimeParts{1700000000, 250000000};
  view.validity_horizon_present = true;
  view.validity_horizon = TimeParts{10, 500000000};
  view.confidence_present = true;
  view.confidence = 0.75;
  view.uncertainty_reference = "cov-ref-7";
  view.validity_present = true;
  view.validity_state = 1;
  return view;
}

OccupancyReferenceView ValidOccupancy() {
  OccupancyReferenceView view;
  view.present = true;
  view.validity = ValidValidity();
  view.frame_id = embodiment::kWorldEnuFrameId;
  view.asset_reference = "occ-ref-7";
  return view;
}

// 1700000000.250s + 10.500s = 1700000010.750s.
TimeParts Deadline() { return TimeParts{1700000010, 750000000}; }

TEST(OccupancyReferencePolicyTest, EmptyMessageIsNotAnError) {
  const OccupancyReferenceAssessment assessment =
      AssessOccupancyReference(OccupancyReferenceView{}, Deadline());
  EXPECT_EQ(assessment.error, OccupancyReferenceError::kNone);
  EXPECT_EQ(assessment.validity.error, ComponentValidityError::kNone);
  EXPECT_EQ(assessment.validity.validity, ValidityKind::kAbsent);
  EXPECT_EQ(assessment.validity.freshness, ComponentFreshness::kUnknown);
  EXPECT_FALSE(assessment.accepted);
}

TEST(OccupancyReferencePolicyTest, MissingValidityIsFirstDefect) {
  OccupancyReferenceView view = ValidOccupancy();
  view.validity.present = false;
  view.frame_id = "robot";
  view.asset_reference = "";
  const OccupancyReferenceAssessment assessment =
      AssessOccupancyReference(view, Deadline());
  EXPECT_EQ(assessment.error, OccupancyReferenceError::kValidity);
  EXPECT_EQ(assessment.validity.error, ComponentValidityError::kNone);
  EXPECT_FALSE(assessment.accepted);
}

TEST(OccupancyReferencePolicyTest, FirstDefectWins) {
  OccupancyReferenceView view = ValidOccupancy();
  view.validity.source_id = "";
  view.frame_id = "robot";
  view.asset_reference = "";
  OccupancyReferenceAssessment assessment =
      AssessOccupancyReference(view, Deadline());
  EXPECT_EQ(assessment.error, OccupancyReferenceError::kValidity);
  EXPECT_EQ(assessment.validity.error, ComponentValidityError::kSourceId);

  view.validity = ValidValidity();
  assessment = AssessOccupancyReference(view, Deadline());
  EXPECT_EQ(assessment.error, OccupancyReferenceError::kFrameId);

  view.frame_id = embodiment::kWorldNedFrameId;
  assessment = AssessOccupancyReference(view, Deadline());
  EXPECT_EQ(assessment.error, OccupancyReferenceError::kAssetReference);

  view.asset_reference = "occ-ref-7";
  assessment = AssessOccupancyReference(view, Deadline());
  EXPECT_EQ(assessment.error, OccupancyReferenceError::kNone);
  EXPECT_TRUE(assessment.accepted);
}

TEST(OccupancyReferencePolicyTest, FrameAllowList) {
  OccupancyReferenceView view = ValidOccupancy();
  for (const std::string_view frame_id :
       {embodiment::kWorldEnuFrameId, embodiment::kWorldNedFrameId}) {
    view.frame_id = frame_id;
    EXPECT_TRUE(AssessOccupancyReference(view, Deadline()).accepted)
        << frame_id;
  }
  for (const std::string_view frame_id :
       {"", "enu", "ned", "world_ENU", "body", "robot", "world_enu "}) {
    view.frame_id = frame_id;
    EXPECT_EQ(AssessOccupancyReference(view, Deadline()).error,
              OccupancyReferenceError::kFrameId)
        << frame_id;
  }
}

TEST(OccupancyReferencePolicyTest, AssetReferenceIsOpaqueAndRequired) {
  OccupancyReferenceView view = ValidOccupancy();
  view.asset_reference = "";
  EXPECT_EQ(AssessOccupancyReference(view, Deadline()).error,
            OccupancyReferenceError::kAssetReference);

  view.asset_reference = " ";
  EXPECT_TRUE(AssessOccupancyReference(view, Deadline()).accepted);
  view.asset_reference = "sha256:abc,nan";
  EXPECT_TRUE(AssessOccupancyReference(view, Deadline()).accepted);
}

TEST(OccupancyReferencePolicyTest, EnuAndNedAcceptTheSameReference) {
  OccupancyReferenceView view = ValidOccupancy();
  view.frame_id = embodiment::kWorldEnuFrameId;
  EXPECT_TRUE(AssessOccupancyReference(view, Deadline()).accepted);
  view.frame_id = embodiment::kWorldNedFrameId;
  EXPECT_TRUE(AssessOccupancyReference(view, Deadline()).accepted);
  EXPECT_EQ(view.asset_reference, "occ-ref-7");
  EXPECT_EQ(view.frame_id, embodiment::kWorldNedFrameId);
}

TEST(OccupancyReferencePolicyTest, AcceptedExampleAtDeadline) {
  const OccupancyReferenceAssessment fresh =
      AssessOccupancyReference(ValidOccupancy(), Deadline());
  EXPECT_EQ(fresh.error, OccupancyReferenceError::kNone);
  EXPECT_EQ(fresh.validity.freshness, ComponentFreshness::kFresh);
  EXPECT_EQ(fresh.validity.validity, ValidityKind::kValid);
  EXPECT_TRUE(fresh.accepted);

  const OccupancyReferenceAssessment expired = AssessOccupancyReference(
      ValidOccupancy(), TimeParts{Deadline().seconds, Deadline().nanos + 1});
  EXPECT_EQ(expired.error, OccupancyReferenceError::kNone);
  EXPECT_EQ(expired.validity.freshness, ComponentFreshness::kExpired);
  EXPECT_EQ(expired.validity.error, ComponentValidityError::kNone);
  EXPECT_FALSE(expired.accepted);
}

TEST(OccupancyReferencePolicyTest, UnknownAndInvalidStayNested) {
  OccupancyReferenceView view = ValidOccupancy();
  view.validity.validity_present = false;
  OccupancyReferenceAssessment assessment =
      AssessOccupancyReference(view, Deadline());
  EXPECT_EQ(assessment.error, OccupancyReferenceError::kNone);
  EXPECT_EQ(assessment.validity.validity, ValidityKind::kAbsent);
  EXPECT_EQ(assessment.validity.freshness, ComponentFreshness::kUnknown);
  EXPECT_FALSE(assessment.accepted);

  view = ValidOccupancy();
  view.validity.validity_state = 2;
  assessment = AssessOccupancyReference(view, Deadline());
  EXPECT_EQ(assessment.error, OccupancyReferenceError::kNone);
  EXPECT_EQ(assessment.validity.validity, ValidityKind::kInvalid);
  EXPECT_FALSE(assessment.accepted);

  view.validity.validity_state = 99;
  assessment = AssessOccupancyReference(view, Deadline());
  EXPECT_EQ(assessment.validity.validity, ValidityKind::kUnspecified);
  EXPECT_EQ(assessment.error, OccupancyReferenceError::kNone);
  EXPECT_FALSE(assessment.accepted);
}

TEST(OccupancyReferencePolicyTest, ExpiredFrameDefectStaysAFrameError) {
  OccupancyReferenceView view = ValidOccupancy();
  view.frame_id = "body";
  const OccupancyReferenceAssessment assessment = AssessOccupancyReference(
      view, TimeParts{Deadline().seconds, Deadline().nanos + 1});
  EXPECT_EQ(assessment.error, OccupancyReferenceError::kFrameId);
  EXPECT_EQ(assessment.validity.freshness, ComponentFreshness::kExpired);
  EXPECT_FALSE(assessment.accepted);
}

}  // namespace
}  // namespace intrinsic::world
