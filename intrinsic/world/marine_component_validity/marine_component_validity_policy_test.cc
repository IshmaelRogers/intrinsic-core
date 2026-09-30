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

#include "intrinsic/world/marine_component_validity/marine_component_validity_policy.h"

#include <cmath>
#include <cstdint>
#include <limits>

#include "gtest/gtest.h"
#include "intrinsic/embodiment/stamped_header_policy.h"

namespace intrinsic::world {
namespace {

using embodiment::ValidityKind;

MarineComponentValidityView ValidView() {
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

TimeParts Deadline() { return TimeParts{1002, 100000000}; }

TEST(MarineComponentValidityPolicyTest, EmptyMessageIsNotAnError) {
  const MarineComponentAssessment assessment =
      AssessMarineComponentValidity(MarineComponentValidityView{}, Deadline());
  EXPECT_EQ(assessment.error, ComponentValidityError::kNone);
  EXPECT_EQ(assessment.validity, ValidityKind::kAbsent);
  EXPECT_EQ(assessment.freshness, ComponentFreshness::kUnknown);
  EXPECT_FALSE(assessment.accepted);
}

TEST(MarineComponentValidityPolicyTest, MissingSourceIdOnPresentComponent) {
  MarineComponentValidityView view = ValidView();
  view.source_id = "";
  const MarineComponentAssessment assessment =
      AssessMarineComponentValidity(view, Deadline());
  EXPECT_EQ(assessment.error, ComponentValidityError::kSourceId);
  EXPECT_FALSE(assessment.accepted);
}

TEST(MarineComponentValidityPolicyTest, FirstDefectWins) {
  MarineComponentValidityView view = ValidView();
  view.source_id = "";
  view.observation_time.nanos = -1;
  view.validity_horizon.seconds = -5;
  view.confidence = std::numeric_limits<double>::quiet_NaN();
  EXPECT_EQ(AssessMarineComponentValidity(view, Deadline()).error,
            ComponentValidityError::kSourceId);

  view.source_id = "fusion_0";
  EXPECT_EQ(AssessMarineComponentValidity(view, Deadline()).error,
            ComponentValidityError::kObservationTime);

  view.observation_time.nanos = 500000000;
  EXPECT_EQ(AssessMarineComponentValidity(view, Deadline()).error,
            ComponentValidityError::kHorizon);

  view.validity_horizon.seconds = 1;
  EXPECT_EQ(AssessMarineComponentValidity(view, Deadline()).error,
            ComponentValidityError::kConfidence);
}

TEST(MarineComponentValidityPolicyTest, RejectsNonsenseHorizon) {
  MarineComponentValidityView view = ValidView();
  view.validity_horizon = TimeParts{-1, 0};
  MarineComponentAssessment assessment =
      AssessMarineComponentValidity(view, Deadline());
  EXPECT_EQ(assessment.error, ComponentValidityError::kHorizon);
  EXPECT_EQ(assessment.freshness, ComponentFreshness::kUnknown);

  view.validity_horizon = TimeParts{0, -1};
  EXPECT_EQ(AssessMarineComponentValidity(view, Deadline()).error,
            ComponentValidityError::kHorizon);

  view.validity_horizon = TimeParts{0, embodiment::kNanosPerSecond};
  EXPECT_EQ(AssessMarineComponentValidity(view, Deadline()).error,
            ComponentValidityError::kHorizon);

  view.validity_horizon = TimeParts{0, 0};
  assessment = AssessMarineComponentValidity(view, TimeParts{1000, 500000000});
  EXPECT_EQ(assessment.error, ComponentValidityError::kNone);
  EXPECT_EQ(assessment.freshness, ComponentFreshness::kFresh);
  assessment = AssessMarineComponentValidity(view, TimeParts{1000, 500000001});
  EXPECT_EQ(assessment.freshness, ComponentFreshness::kExpired);
}

TEST(MarineComponentValidityPolicyTest, ConfidenceBounds) {
  MarineComponentValidityView view = ValidView();
  view.confidence_present = false;
  view.confidence = std::numeric_limits<double>::infinity();
  EXPECT_EQ(AssessMarineComponentValidity(view, Deadline()).error,
            ComponentValidityError::kNone);

  view.confidence_present = true;
  for (double confidence : {0.0, -0.0, 1.0, 0.75}) {
    view.confidence = confidence;
    EXPECT_EQ(AssessMarineComponentValidity(view, Deadline()).error,
              ComponentValidityError::kNone)
        << confidence;
  }
  for (double confidence : {-0.1, std::nextafter(1.0, 2.0),
                            std::numeric_limits<double>::quiet_NaN(),
                            std::numeric_limits<double>::infinity()}) {
    view.confidence = confidence;
    EXPECT_EQ(AssessMarineComponentValidity(view, Deadline()).error,
              ComponentValidityError::kConfidence)
        << confidence;
  }
}

TEST(MarineComponentValidityPolicyTest, ExactHorizonBoundary) {
  const MarineComponentValidityView view = ValidView();
  const TimeParts deadline = Deadline();
  MarineComponentAssessment assessment =
      AssessMarineComponentValidity(view, deadline);
  EXPECT_EQ(assessment.freshness, ComponentFreshness::kFresh);
  EXPECT_EQ(assessment.validity, ValidityKind::kValid);
  EXPECT_TRUE(assessment.accepted);

  assessment = AssessMarineComponentValidity(view, TimeParts{1002, 99999999});
  EXPECT_EQ(assessment.freshness, ComponentFreshness::kFresh);
  EXPECT_TRUE(assessment.accepted);

  assessment = AssessMarineComponentValidity(
      view, TimeParts{deadline.seconds, deadline.nanos + 1});
  EXPECT_EQ(assessment.freshness, ComponentFreshness::kExpired);
  EXPECT_EQ(assessment.validity, ValidityKind::kValid);
  EXPECT_FALSE(assessment.accepted);

  assessment = AssessMarineComponentValidity(view, TimeParts{1000, 0});
  EXPECT_EQ(assessment.freshness, ComponentFreshness::kFresh);
}

TEST(MarineComponentValidityPolicyTest, UnknownIsDistinctFromExpired) {
  MarineComponentValidityView view = ValidView();
  const TimeParts after = TimeParts{1002, 100000001};

  view.validity_present = false;
  MarineComponentAssessment assessment =
      AssessMarineComponentValidity(view, after);
  EXPECT_EQ(assessment.freshness, ComponentFreshness::kUnknown);
  EXPECT_EQ(assessment.validity, ValidityKind::kAbsent);
  EXPECT_NE(assessment.freshness, ComponentFreshness::kExpired);

  view = ValidView();
  view.observation_time_present = false;
  assessment = AssessMarineComponentValidity(view, after);
  EXPECT_EQ(assessment.error, ComponentValidityError::kNone);
  EXPECT_EQ(assessment.freshness, ComponentFreshness::kUnknown);
  EXPECT_FALSE(assessment.accepted);

  view = ValidView();
  view.validity_horizon_present = false;
  assessment = AssessMarineComponentValidity(view, after);
  EXPECT_EQ(assessment.freshness, ComponentFreshness::kUnknown);
  EXPECT_FALSE(assessment.accepted);

  view = ValidView();
  view.validity_state = 0;
  assessment = AssessMarineComponentValidity(view, after);
  EXPECT_EQ(assessment.validity, ValidityKind::kUnspecified);
  EXPECT_EQ(assessment.freshness, ComponentFreshness::kExpired);
  EXPECT_FALSE(assessment.accepted);

  view.validity_state = 2;
  assessment = AssessMarineComponentValidity(view, Deadline());
  EXPECT_EQ(assessment.validity, ValidityKind::kInvalid);
  EXPECT_EQ(assessment.freshness, ComponentFreshness::kFresh);
  EXPECT_FALSE(assessment.accepted);

  view.validity_state = 99;
  assessment = AssessMarineComponentValidity(view, after);
  EXPECT_EQ(assessment.validity, ValidityKind::kUnspecified);
  EXPECT_EQ(assessment.freshness, ComponentFreshness::kExpired);
}

TEST(MarineComponentValidityPolicyTest, UncertaintyReferenceIsOpaque) {
  MarineComponentValidityView view = ValidView();
  view.uncertainty_reference = "";
  EXPECT_TRUE(AssessMarineComponentValidity(view, Deadline()).accepted);
  view.uncertainty_reference = "1,nan,inf";
  EXPECT_TRUE(AssessMarineComponentValidity(view, Deadline()).accepted);
}

TEST(MarineComponentValidityPolicyTest, NonsenseQueryIsUnknown) {
  const MarineComponentAssessment assessment =
      AssessMarineComponentValidity(ValidView(), TimeParts{1002, -1});
  EXPECT_EQ(assessment.error, ComponentValidityError::kNone);
  EXPECT_EQ(assessment.freshness, ComponentFreshness::kUnknown);
  EXPECT_FALSE(assessment.accepted);
}

TEST(MarineComponentValidityPolicyTest, DeadlineOverflowIsUnknown) {
  MarineComponentValidityView view = ValidView();
  view.observation_time =
      TimeParts{std::numeric_limits<int64_t>::max(), 999999999};
  view.validity_horizon = TimeParts{0, 1};
  const MarineComponentAssessment assessment =
      AssessMarineComponentValidity(view, TimeParts{0, 0});
  EXPECT_EQ(assessment.error, ComponentValidityError::kNone);
  EXPECT_EQ(assessment.freshness, ComponentFreshness::kUnknown);

  view.observation_time = TimeParts{-1, 800000000};
  view.validity_horizon = TimeParts{0, 300000000};
  EXPECT_EQ(
      AssessMarineComponentValidity(view, TimeParts{0, 100000000}).freshness,
      ComponentFreshness::kFresh);
  EXPECT_EQ(
      AssessMarineComponentValidity(view, TimeParts{0, 100000001}).freshness,
      ComponentFreshness::kExpired);
}

}  // namespace
}  // namespace intrinsic::world
