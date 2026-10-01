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

#include "intrinsic/safety/speed_max_rule.h"

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
  EXPECT_EQ(result.rule_id, "speed.linear.max");
  EXPECT_EQ(result.rule_id, kSpeedMaxRuleId);
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
  EXPECT_EQ(result.rule_id, "speed.linear.max");
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

TEST(SpeedMaxRuleTest, DefaultMaxLinearSpeedIsOnePointFiveMetersPerSecond) {
  EXPECT_EQ(kDefaultMaxLinearSpeed, 1.5);
}

TEST(SpeedMaxRuleTest, JustInsideIsCompliant) {
  ExpectCompliant(EvaluateSpeedMaxRule(1.499));
  ExpectCompliant(EvaluateSpeedMaxRule(0.0));
  ExpectCompliant(EvaluateSpeedMaxRule(-0.0));
}

TEST(SpeedMaxRuleTest, EqualToLimitIsCompliant) {
  ExpectCompliant(EvaluateSpeedMaxRule(1.5));
  ExpectCompliant(EvaluateSpeedMaxRule(1.5, kDefaultMaxLinearSpeed));
  ExpectCompliant(EvaluateSpeedMaxRule(0.0, 0.0));
  ExpectCompliant(EvaluateSpeedMaxRule(0.25, 0.25));
}

TEST(SpeedMaxRuleTest, JustOutsideClampsToLimit) {
  ExpectClamp(EvaluateSpeedMaxRule(1.501), 1.5);
  ExpectClamp(EvaluateSpeedMaxRule(3.0), 1.5);
  ExpectClamp(EvaluateSpeedMaxRule(0.51, 0.5), 0.5);
  ExpectClamp(EvaluateSpeedMaxRule(0.001, 0.0), 0.0);
}

TEST(SpeedMaxRuleTest, NonFiniteInputRejects) {
  ExpectRejectCritical(EvaluateSpeedMaxRule(kNaN));
  ExpectRejectCritical(EvaluateSpeedMaxRule(kInf));
  ExpectRejectCritical(EvaluateSpeedMaxRule(-kInf));
}

TEST(SpeedMaxRuleTest, NegativeInputRejects) {
  ExpectRejectCritical(EvaluateSpeedMaxRule(-0.001));
  ExpectRejectCritical(EvaluateSpeedMaxRule(-1.0, 10.0));
}

TEST(SpeedMaxRuleTest, NonFiniteMaxRejects) {
  ExpectRejectCritical(EvaluateSpeedMaxRule(0.1, kNaN));
  ExpectRejectCritical(EvaluateSpeedMaxRule(0.1, kInf));
  ExpectRejectCritical(EvaluateSpeedMaxRule(0.1, -kInf));
}

TEST(SpeedMaxRuleTest, NegativeMaxRejects) {
  ExpectRejectCritical(EvaluateSpeedMaxRule(0.1, -0.001));
  ExpectRejectCritical(EvaluateSpeedMaxRule(5.0, -1.0));
}

TEST(SpeedMaxRuleTest, NegativeZeroMaxIsUsable) {
  ExpectCompliant(EvaluateSpeedMaxRule(0.0, -0.0));
}

}  // namespace
}  // namespace intrinsic::safety
