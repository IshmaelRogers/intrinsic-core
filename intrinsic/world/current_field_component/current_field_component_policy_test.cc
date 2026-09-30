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

CurrentFieldView ValidView() {
  CurrentFieldView view;
  view.present = true;
  view.validity_meta = ValidMeta();
  view.frame_id = "world_enu";
  view.constant_present = true;
  view.velocity_x_m_s = 1.5;
  view.velocity_y_m_s = -0.25;
  view.velocity_z_m_s = 0.125;
  return view;
}

TimeParts Deadline() { return TimeParts{1002, 100000000}; }

TEST(CurrentFieldPolicyTest, EmptyMessageIsNotAnError) {
  const CurrentFieldAssessment assessment =
      AssessCurrentField(CurrentFieldView{}, Deadline());
  EXPECT_EQ(assessment.error, CurrentFieldError::kNone);
  EXPECT_EQ(assessment.validity.freshness, ComponentFreshness::kUnknown);
  EXPECT_FALSE(assessment.accepted);
}

TEST(CurrentFieldPolicyTest, MissingValidityMeta) {
  CurrentFieldView view = ValidView();
  view.validity_meta = MarineComponentValidityView{};
  EXPECT_EQ(AssessCurrentField(view, Deadline()).error,
            CurrentFieldError::kValidity);
}

TEST(CurrentFieldPolicyTest, ValidityDefectIsDelegated) {
  CurrentFieldView view = ValidView();
  view.validity_meta.source_id = "";
  const CurrentFieldAssessment assessment =
      AssessCurrentField(view, Deadline());
  EXPECT_EQ(assessment.error, CurrentFieldError::kValidity);
  EXPECT_EQ(assessment.validity.error, ComponentValidityError::kSourceId);
  EXPECT_EQ(
      assessment.validity.freshness,
      AssessMarineComponentValidity(view.validity_meta, Deadline()).freshness);
}

TEST(CurrentFieldPolicyTest, FirstDefectWins) {
  CurrentFieldView view = ValidView();
  view.validity_meta.source_id = "";
  view.frame_id = "";
  view.constant_present = false;
  view.velocity_x_m_s = std::numeric_limits<double>::quiet_NaN();
  EXPECT_EQ(AssessCurrentField(view, Deadline()).error,
            CurrentFieldError::kValidity);

  view.validity_meta.source_id = "fusion_0";
  EXPECT_EQ(AssessCurrentField(view, Deadline()).error,
            CurrentFieldError::kFrameId);

  view.frame_id = "world_enu";
  EXPECT_EQ(AssessCurrentField(view, Deadline()).error,
            CurrentFieldError::kRepresentation);

  view.constant_present = true;
  EXPECT_EQ(AssessCurrentField(view, Deadline()).error,
            CurrentFieldError::kVelocity);
}

TEST(CurrentFieldPolicyTest, RejectsEmptyFrameAndMissingConstant) {
  CurrentFieldView view = ValidView();
  view.frame_id = "";
  EXPECT_EQ(AssessCurrentField(view, Deadline()).error,
            CurrentFieldError::kFrameId);
  view.frame_id = "world_enu";
  view.constant_present = false;
  view.velocity_x_m_s = std::numeric_limits<double>::infinity();
  EXPECT_EQ(AssessCurrentField(view, Deadline()).error,
            CurrentFieldError::kRepresentation);
}

TEST(CurrentFieldPolicyTest, VelocityFiniteWhenConstantPresent) {
  CurrentFieldView view = ValidView();
  view.velocity_x_m_s = 0;
  view.velocity_y_m_s = -0.0;
  view.velocity_z_m_s = 0;
  EXPECT_TRUE(AssessCurrentField(view, Deadline()).accepted);

  view = ValidView();
  view.velocity_x_m_s = std::numeric_limits<double>::quiet_NaN();
  EXPECT_EQ(AssessCurrentField(view, Deadline()).error,
            CurrentFieldError::kVelocity);
  view = ValidView();
  view.velocity_y_m_s = std::numeric_limits<double>::infinity();
  EXPECT_EQ(AssessCurrentField(view, Deadline()).error,
            CurrentFieldError::kVelocity);
  view = ValidView();
  view.velocity_z_m_s = -std::numeric_limits<double>::infinity();
  EXPECT_EQ(AssessCurrentField(view, Deadline()).error,
            CurrentFieldError::kVelocity);
}

TEST(CurrentFieldPolicyTest, NonEmptyFrameIsNotRewritten) {
  CurrentFieldView view = ValidView();
  view.frame_id = "map_enu";
  EXPECT_TRUE(AssessCurrentField(view, Deadline()).accepted);
  EXPECT_EQ(view.frame_id, "map_enu");
}

TEST(CurrentFieldPolicyTest, ExactHorizonBoundaryIsDelegated) {
  const CurrentFieldView view = ValidView();
  const TimeParts deadline = Deadline();
  CurrentFieldAssessment assessment = AssessCurrentField(view, deadline);
  EXPECT_EQ(assessment.error, CurrentFieldError::kNone);
  EXPECT_EQ(assessment.validity.freshness, ComponentFreshness::kFresh);
  EXPECT_TRUE(assessment.accepted);

  const TimeParts after = TimeParts{deadline.seconds, deadline.nanos + 1};
  assessment = AssessCurrentField(view, after);
  EXPECT_EQ(assessment.error, CurrentFieldError::kNone);
  EXPECT_EQ(assessment.validity.freshness, ComponentFreshness::kExpired);
  EXPECT_EQ(assessment.validity.validity, embodiment::ValidityKind::kValid);
  EXPECT_FALSE(assessment.accepted);
  EXPECT_EQ(assessment.validity.freshness,
            AssessMarineComponentValidity(view.validity_meta, after).freshness);
}

TEST(CurrentFieldPolicyTest, UnknownIsDistinctFromExpired) {
  CurrentFieldView view = ValidView();
  const TimeParts after = TimeParts{1002, 100000001};
  view.validity_meta.validity_present = false;
  CurrentFieldAssessment assessment = AssessCurrentField(view, after);
  EXPECT_EQ(assessment.validity.freshness, ComponentFreshness::kUnknown);
  EXPECT_NE(assessment.validity.freshness, ComponentFreshness::kExpired);
  EXPECT_FALSE(assessment.accepted);

  view = ValidView();
  view.validity_meta.validity_state = 2;
  assessment = AssessCurrentField(view, Deadline());
  EXPECT_EQ(assessment.validity.validity, embodiment::ValidityKind::kInvalid);
  EXPECT_EQ(assessment.validity.freshness, ComponentFreshness::kFresh);
  EXPECT_FALSE(assessment.accepted);

  view.validity_meta.validity_state = 99;
  assessment = AssessCurrentField(view, after);
  EXPECT_EQ(assessment.validity.validity,
            embodiment::ValidityKind::kUnspecified);
  EXPECT_EQ(assessment.validity.freshness, ComponentFreshness::kExpired);
  EXPECT_FALSE(assessment.accepted);
}

}  // namespace
}  // namespace intrinsic::world
