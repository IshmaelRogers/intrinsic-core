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

#include "intrinsic/world/current_field_component/current_field_component_policy.h"

#include <cmath>
#include <limits>

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

CurrentFieldView ValidCurrent() {
  CurrentFieldView view;
  view.present = true;
  view.validity = ValidValidity();
  view.frame_id = embodiment::kWorldEnuFrameId;
  view.velocity_present = true;
  view.velocity_m_s = embodiment::Vec3{0.2, -0.1, 0.0};
  return view;
}

// 1700000000.250s + 10.500s = 1700000010.750s.
TimeParts Deadline() { return TimeParts{1700000010, 750000000}; }

TEST(CurrentFieldPolicyTest, EmptyMessageIsNotAnError) {
  const CurrentFieldAssessment assessment =
      AssessCurrentField(CurrentFieldView{}, Deadline());
  EXPECT_EQ(assessment.error, CurrentFieldError::kNone);
  EXPECT_EQ(assessment.validity.error, ComponentValidityError::kNone);
  EXPECT_EQ(assessment.validity.validity, ValidityKind::kAbsent);
  EXPECT_EQ(assessment.validity.freshness, ComponentFreshness::kUnknown);
  EXPECT_FALSE(assessment.accepted);
}

TEST(CurrentFieldPolicyTest, MissingValidityIsFirstDefect) {
  CurrentFieldView view = ValidCurrent();
  view.validity.present = false;
  view.frame_id = "robot";
  view.velocity_present = false;
  const CurrentFieldAssessment assessment =
      AssessCurrentField(view, Deadline());
  EXPECT_EQ(assessment.error, CurrentFieldError::kValidity);
  EXPECT_FALSE(assessment.accepted);
}

TEST(CurrentFieldPolicyTest, FirstDefectWins) {
  CurrentFieldView view = ValidCurrent();
  view.validity.source_id = "";
  view.frame_id = "robot";
  view.velocity_present = false;
  CurrentFieldAssessment assessment = AssessCurrentField(view, Deadline());
  EXPECT_EQ(assessment.error, CurrentFieldError::kValidity);
  EXPECT_EQ(assessment.validity.error, ComponentValidityError::kSourceId);

  view.validity = ValidValidity();
  assessment = AssessCurrentField(view, Deadline());
  EXPECT_EQ(assessment.error, CurrentFieldError::kFrameId);

  view.frame_id = kBodyFrameId;
  assessment = AssessCurrentField(view, Deadline());
  EXPECT_EQ(assessment.error, CurrentFieldError::kVelocity);

  view.velocity_present = true;
  view.velocity_m_s = embodiment::Vec3{0.0, 0.0, 0.0};
  assessment = AssessCurrentField(view, Deadline());
  EXPECT_EQ(assessment.error, CurrentFieldError::kNone);
  EXPECT_TRUE(assessment.accepted);
}

TEST(CurrentFieldPolicyTest, FrameAllowList) {
  CurrentFieldView view = ValidCurrent();
  for (const std::string_view frame_id :
       {embodiment::kWorldEnuFrameId, embodiment::kWorldNedFrameId,
        kBodyFrameId}) {
    view.frame_id = frame_id;
    EXPECT_TRUE(AssessCurrentField(view, Deadline()).accepted) << frame_id;
  }
  for (const std::string_view frame_id :
       {"", "enu", "ned", "world_ENU", "BODY", "robot", "body "}) {
    view.frame_id = frame_id;
    EXPECT_EQ(AssessCurrentField(view, Deadline()).error,
              CurrentFieldError::kFrameId)
        << frame_id;
  }
}

TEST(CurrentFieldPolicyTest, VelocityMustBePresentAndFinite) {
  CurrentFieldView view = ValidCurrent();
  view.velocity_present = false;
  view.velocity_m_s =
      embodiment::Vec3{std::numeric_limits<double>::quiet_NaN(),
                       std::numeric_limits<double>::quiet_NaN(),
                       std::numeric_limits<double>::quiet_NaN()};
  EXPECT_EQ(AssessCurrentField(view, Deadline()).error,
            CurrentFieldError::kVelocity);

  view.velocity_present = true;
  view.velocity_m_s = embodiment::Vec3{0.0, 0.0, 0.0};
  EXPECT_TRUE(AssessCurrentField(view, Deadline()).accepted);

  for (embodiment::Vec3 velocity :
       {embodiment::Vec3{std::numeric_limits<double>::quiet_NaN(), 0.0, 0.0},
        embodiment::Vec3{0.0, std::numeric_limits<double>::infinity(), 0.0},
        embodiment::Vec3{0.0, 0.0, -std::numeric_limits<double>::infinity()}}) {
    view.velocity_m_s = velocity;
    EXPECT_EQ(AssessCurrentField(view, Deadline()).error,
              CurrentFieldError::kVelocity);
  }
}

TEST(CurrentFieldPolicyTest, SameNumbersAcceptedInEachFrame) {
  CurrentFieldView view = ValidCurrent();
  view.velocity_m_s = embodiment::Vec3{0.2, -0.1, 0.0};
  for (const std::string_view frame_id :
       {embodiment::kWorldEnuFrameId, embodiment::kWorldNedFrameId,
        kBodyFrameId}) {
    view.frame_id = frame_id;
    const CurrentFieldAssessment assessment =
        AssessCurrentField(view, Deadline());
    EXPECT_TRUE(assessment.accepted) << frame_id;
    EXPECT_DOUBLE_EQ(view.velocity_m_s.x, 0.2);
    EXPECT_DOUBLE_EQ(view.velocity_m_s.y, -0.1);
    EXPECT_DOUBLE_EQ(view.velocity_m_s.z, 0.0);
  }
}

TEST(CurrentFieldPolicyTest, AcceptedExampleAtDeadline) {
  const CurrentFieldAssessment fresh =
      AssessCurrentField(ValidCurrent(), Deadline());
  EXPECT_EQ(fresh.error, CurrentFieldError::kNone);
  EXPECT_EQ(fresh.validity.freshness, ComponentFreshness::kFresh);
  EXPECT_EQ(fresh.validity.validity, ValidityKind::kValid);
  EXPECT_TRUE(fresh.accepted);

  const CurrentFieldAssessment expired = AssessCurrentField(
      ValidCurrent(), TimeParts{Deadline().seconds, Deadline().nanos + 1});
  EXPECT_EQ(expired.error, CurrentFieldError::kNone);
  EXPECT_EQ(expired.validity.freshness, ComponentFreshness::kExpired);
  EXPECT_EQ(expired.validity.error, ComponentValidityError::kNone);
  EXPECT_FALSE(expired.accepted);
}

TEST(CurrentFieldPolicyTest, UnknownAndInvalidStayNested) {
  CurrentFieldView view = ValidCurrent();
  view.validity.validity_present = false;
  CurrentFieldAssessment assessment = AssessCurrentField(view, Deadline());
  EXPECT_EQ(assessment.error, CurrentFieldError::kNone);
  EXPECT_EQ(assessment.validity.validity, ValidityKind::kAbsent);
  EXPECT_EQ(assessment.validity.freshness, ComponentFreshness::kUnknown);
  EXPECT_FALSE(assessment.accepted);

  view = ValidCurrent();
  view.validity.validity_state = 2;
  assessment = AssessCurrentField(view, Deadline());
  EXPECT_EQ(assessment.error, CurrentFieldError::kNone);
  EXPECT_EQ(assessment.validity.validity, ValidityKind::kInvalid);
  EXPECT_FALSE(assessment.accepted);

  view.validity.validity_state = 99;
  assessment = AssessCurrentField(view, Deadline());
  EXPECT_EQ(assessment.validity.validity, ValidityKind::kUnspecified);
  EXPECT_EQ(assessment.error, CurrentFieldError::kNone);
  EXPECT_FALSE(assessment.accepted);
}

}  // namespace
}  // namespace intrinsic::world
