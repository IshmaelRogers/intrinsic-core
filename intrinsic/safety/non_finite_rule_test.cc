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

#include "intrinsic/safety/non_finite_rule.h"

#include <array>
#include <limits>
#include <span>

#include "gtest/gtest.h"
#include "intrinsic/safety/safety_decision_policy.h"
#include "intrinsic/vehicle/vehicle_contract_policy.h"

namespace intrinsic::safety {
namespace {

using vehicle::BodyVector;
using vehicle::DesiredMotionView;
using vehicle::VehicleStateView;

void ExpectCompliant(const SafetyRuleResult& result) {
  EXPECT_FALSE(result.violated);
  EXPECT_TRUE(result.rule_id.empty());
  EXPECT_EQ(result.severity, 0);
  EXPECT_TRUE(result.summary.empty());
  EXPECT_EQ(result.recommended_kind, 0);
  EXPECT_EQ(result.recommended_kind,
            static_cast<int>(SafetyDecisionKind::kUnspecified));
}

void ExpectNonFinite(const SafetyRuleResult& result) {
  EXPECT_TRUE(result.violated);
  EXPECT_EQ(result.rule_id, "state.non_finite");
  EXPECT_EQ(result.rule_id, kStateNonFiniteRuleId);
  EXPECT_EQ(result.severity, 4);
  EXPECT_EQ(result.severity,
            static_cast<int>(SafetyFindingSeverityKind::kCritical));
  EXPECT_EQ(result.summary, "non-finite state value");
  EXPECT_EQ(result.recommended_kind, 3);
  EXPECT_EQ(result.recommended_kind,
            static_cast<int>(SafetyDecisionKind::kReject));
  EXPECT_NE(result.recommended_kind,
            static_cast<int>(SafetyDecisionKind::kProject));
}

TEST(NonFiniteRuleTest, EmptySpanIsCompliant) {
  ExpectCompliant(EvaluateNonFiniteRule(std::span<const double>{}));
}

TEST(NonFiniteRuleTest, FiniteZerosAreCompliant) {
  const double values[] = {0.0, -0.0, 1.0, -2.5};
  ExpectCompliant(EvaluateNonFiniteRule(values));
}

TEST(NonFiniteRuleTest, NanPositiveInfAndNegativeInfViolate) {
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const double positive_inf = std::numeric_limits<double>::infinity();
  const double negative_inf = -std::numeric_limits<double>::infinity();
  ExpectNonFinite(EvaluateNonFiniteRule(std::array<double, 1>{nan}));
  ExpectNonFinite(EvaluateNonFiniteRule(std::array<double, 1>{positive_inf}));
  ExpectNonFinite(EvaluateNonFiniteRule(std::array<double, 1>{negative_inf}));
  const double mixed[] = {0.0, -0.0, 1.0, nan};
  ExpectNonFinite(EvaluateNonFiniteRule(mixed));
}

TEST(NonFiniteRuleTest, DefaultBodyVectorAndMotionAreCompliant) {
  ExpectCompliant(EvaluateNonFiniteRule(BodyVector{}));
  ExpectCompliant(EvaluateNonFiniteRule(DesiredMotionView{}));
  ExpectCompliant(EvaluateNonFiniteRule(VehicleStateView{}));
}

TEST(NonFiniteRuleTest, BodyVectorNonFiniteLinearComponentViolates) {
  BodyVector nan_linear;
  nan_linear.linear_x = std::numeric_limits<double>::quiet_NaN();
  ExpectNonFinite(EvaluateNonFiniteRule(nan_linear));

  BodyVector negative_inf_linear;
  negative_inf_linear.linear_z = -std::numeric_limits<double>::infinity();
  ExpectNonFinite(EvaluateNonFiniteRule(negative_inf_linear));
}

TEST(NonFiniteRuleTest, DesiredMotionNonFiniteLinearComponentViolates) {
  DesiredMotionView motion;
  motion.twist.linear_y = std::numeric_limits<double>::infinity();
  ExpectNonFinite(EvaluateNonFiniteRule(motion));

  DesiredMotionView nan_position;
  nan_position.position.x = std::numeric_limits<double>::quiet_NaN();
  ExpectNonFinite(EvaluateNonFiniteRule(nan_position));
}

TEST(NonFiniteRuleTest, VehicleStateNonFiniteLinearComponentViolates) {
  VehicleStateView state;
  state.twist.linear_x = std::numeric_limits<double>::quiet_NaN();
  ExpectNonFinite(EvaluateNonFiniteRule(state));

  VehicleStateView acceleration;
  acceleration.acceleration.linear_z = -std::numeric_limits<double>::infinity();
  ExpectNonFinite(EvaluateNonFiniteRule(acceleration));
}

TEST(NonFiniteRuleTest, VehicleStateNonFiniteCovarianceViolates) {
  const double values[] = {std::numeric_limits<double>::quiet_NaN()};
  VehicleStateView state;
  state.pose_covariance_present = true;
  state.pose_covariance = values;
  ExpectNonFinite(EvaluateNonFiniteRule(state));
}

TEST(NonFiniteRuleTest, SignedZeroBodyVectorIsCompliant) {
  BodyVector zeros;
  zeros.linear_x = -0.0;
  zeros.angular_z = 0.0;
  ExpectCompliant(EvaluateNonFiniteRule(zeros));
}

}  // namespace
}  // namespace intrinsic::safety
