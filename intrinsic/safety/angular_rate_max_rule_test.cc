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

#include "intrinsic/safety/angular_rate_max_rule.h"

#include <limits>

#include "gtest/gtest.h"
#include "intrinsic/safety/safety_decision_policy.h"

namespace intrinsic::safety {
namespace {

constexpr double kInf = std::numeric_limits<double>::infinity();
constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();

void ExpectCompliant(const SafetyRuleResult& result) {
  EXPECT_FALSE(result.violated);
  EXPECT_TRUE(result.rule_id.empty());
  EXPECT_EQ(result.severity, 0);
  EXPECT_TRUE(result.summary.empty());
  EXPECT_EQ(result.recommended_kind, 0);
  EXPECT_FALSE(result.has_projected_value);
  EXPECT_EQ(result.projected_value, 0.0);
}

void ExpectClamp(const SafetyRuleResult& result, double limit) {
  EXPECT_TRUE(result.violated);
  EXPECT_EQ(result.rule_id, "rate.angular.max");
  EXPECT_EQ(result.rule_id, kAngularRateMaxRuleId);
  EXPECT_EQ(result.severity, 3);
  EXPECT_EQ(result.severity,
            static_cast<int>(SafetyFindingSeverityKind::kError));
  EXPECT_FALSE(result.summary.empty());
  EXPECT_EQ(result.recommended_kind, 2);
  EXPECT_EQ(result.recommended_kind, kDecisionKindProject);
  EXPECT_EQ(result.recommended_kind,
            static_cast<int>(SafetyDecisionKind::kProject));
  EXPECT_TRUE(result.has_projected_value);
  EXPECT_EQ(result.projected_value, limit);
}

void ExpectRejectCritical(const SafetyRuleResult& result) {
  EXPECT_TRUE(result.violated);
  EXPECT_EQ(result.rule_id, "rate.angular.max");
  EXPECT_EQ(result.severity, 4);
  EXPECT_EQ(result.severity,
            static_cast<int>(SafetyFindingSeverityKind::kCritical));
  EXPECT_FALSE(result.summary.empty());
  EXPECT_EQ(result.recommended_kind, 3);
  EXPECT_EQ(result.recommended_kind,
            static_cast<int>(SafetyDecisionKind::kReject));
  EXPECT_FALSE(result.has_projected_value);
  EXPECT_EQ(result.projected_value, 0.0);
}

TEST(AngularRateMaxRuleTest, DefaultMaxAngularRateIsHalfRadianPerSecond) {
  EXPECT_EQ(kDefaultMaxAngularRate, 0.5);
}

TEST(AngularRateMaxRuleTest, JustInsideIsCompliant) {
  ExpectCompliant(EvaluateAngularRateMaxRule(0.499));
  ExpectCompliant(EvaluateAngularRateMaxRule(0.0));
  ExpectCompliant(EvaluateAngularRateMaxRule(-0.0));
}

TEST(AngularRateMaxRuleTest, EqualToLimitIsCompliant) {
  ExpectCompliant(EvaluateAngularRateMaxRule(0.5));
  ExpectCompliant(EvaluateAngularRateMaxRule(0.5, kDefaultMaxAngularRate));
  ExpectCompliant(EvaluateAngularRateMaxRule(0.0, 0.0));
  ExpectCompliant(EvaluateAngularRateMaxRule(0.25, 0.25));
}

TEST(AngularRateMaxRuleTest, JustOutsideClampsToLimit) {
  ExpectClamp(EvaluateAngularRateMaxRule(0.501), 0.5);
  ExpectClamp(EvaluateAngularRateMaxRule(2.0), 0.5);
  ExpectClamp(EvaluateAngularRateMaxRule(0.31, 0.3), 0.3);
  ExpectClamp(EvaluateAngularRateMaxRule(0.001, 0.0), 0.0);
}

TEST(AngularRateMaxRuleTest, NonFiniteInputRejects) {
  ExpectRejectCritical(EvaluateAngularRateMaxRule(kNaN));
  ExpectRejectCritical(EvaluateAngularRateMaxRule(kInf));
  ExpectRejectCritical(EvaluateAngularRateMaxRule(-kInf));
}

TEST(AngularRateMaxRuleTest, NegativeInputRejects) {
  ExpectRejectCritical(EvaluateAngularRateMaxRule(-0.001));
  ExpectRejectCritical(EvaluateAngularRateMaxRule(-1.0, 10.0));
}

TEST(AngularRateMaxRuleTest, NonFiniteMaxRejects) {
  ExpectRejectCritical(EvaluateAngularRateMaxRule(0.1, kNaN));
  ExpectRejectCritical(EvaluateAngularRateMaxRule(0.1, kInf));
  ExpectRejectCritical(EvaluateAngularRateMaxRule(0.1, -kInf));
}

TEST(AngularRateMaxRuleTest, NegativeMaxRejects) {
  ExpectRejectCritical(EvaluateAngularRateMaxRule(0.1, -0.001));
  ExpectRejectCritical(EvaluateAngularRateMaxRule(5.0, -1.0));
}

TEST(AngularRateMaxRuleTest, NegativeZeroMaxIsUsable) {
  ExpectCompliant(EvaluateAngularRateMaxRule(0.0, -0.0));
}

}  // namespace
}  // namespace intrinsic::safety
