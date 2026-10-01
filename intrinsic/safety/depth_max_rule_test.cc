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

#include "intrinsic/safety/depth_max_rule.h"

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
  EXPECT_EQ(result.rule_id, "depth.max");
  EXPECT_EQ(result.rule_id, kDepthMaxRuleId);
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
  EXPECT_EQ(result.rule_id, "depth.max");
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

TEST(DepthMaxRuleTest, DefaultMaxDepthIsOneHundredMeters) {
  EXPECT_EQ(kDefaultMaxDepth, 100.0);
}

TEST(DepthMaxRuleTest, JustInsideIsCompliant) {
  ExpectCompliant(EvaluateDepthMaxRule(99.999));
  ExpectCompliant(EvaluateDepthMaxRule(0.0));
  ExpectCompliant(EvaluateDepthMaxRule(-1.0));
}

TEST(DepthMaxRuleTest, EqualToLimitIsCompliant) {
  ExpectCompliant(EvaluateDepthMaxRule(100.0));
  ExpectCompliant(EvaluateDepthMaxRule(100.0, kDefaultMaxDepth));
  ExpectCompliant(EvaluateDepthMaxRule(0.0, 0.0));
  ExpectCompliant(EvaluateDepthMaxRule(25.5, 25.5));
}

TEST(DepthMaxRuleTest, JustOutsideClampsToLimit) {
  ExpectClamp(EvaluateDepthMaxRule(100.001), 100.0);
  ExpectClamp(EvaluateDepthMaxRule(250.0), 100.0);
  ExpectClamp(EvaluateDepthMaxRule(30.1, 30.0), 30.0);
  ExpectClamp(EvaluateDepthMaxRule(0.001, 0.0), 0.0);
}

TEST(DepthMaxRuleTest, NonFiniteDepthRejects) {
  ExpectRejectCritical(EvaluateDepthMaxRule(kNaN));
  ExpectRejectCritical(EvaluateDepthMaxRule(kInf));
  ExpectRejectCritical(EvaluateDepthMaxRule(-kInf));
}

TEST(DepthMaxRuleTest, NonFiniteMaxDepthRejects) {
  ExpectRejectCritical(EvaluateDepthMaxRule(10.0, kNaN));
  ExpectRejectCritical(EvaluateDepthMaxRule(10.0, kInf));
  ExpectRejectCritical(EvaluateDepthMaxRule(10.0, -kInf));
}

TEST(DepthMaxRuleTest, NegativeMaxDepthRejects) {
  ExpectRejectCritical(EvaluateDepthMaxRule(10.0, -0.001));
  ExpectRejectCritical(EvaluateDepthMaxRule(-5.0, -1.0));
}

TEST(DepthMaxRuleTest, NegativeZeroMaxDepthIsUsable) {
  ExpectCompliant(EvaluateDepthMaxRule(0.0, -0.0));
}

}  // namespace
}  // namespace intrinsic::safety
