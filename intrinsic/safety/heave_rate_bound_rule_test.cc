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

#include "intrinsic/safety/heave_rate_bound_rule.h"

#include <limits>
#include <string_view>

#include "gtest/gtest.h"
#include "intrinsic/safety/safety_decision_policy.h"

namespace intrinsic::safety {
namespace {

constexpr std::string_view kDescentId = "rate.descent.max";
constexpr std::string_view kAscentId = "rate.ascent.max";

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

void ExpectClamp(const SafetyRuleResult& result, std::string_view rule_id,
                 double limit) {
  EXPECT_TRUE(result.violated);
  EXPECT_EQ(result.rule_id, rule_id);
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

void ExpectRejectCritical(const SafetyRuleResult& result,
                          std::string_view rule_id) {
  EXPECT_TRUE(result.violated);
  EXPECT_EQ(result.rule_id, rule_id);
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

TEST(HeaveRateBoundRuleTest, RuleIdsAndDefaultsAreLocked) {
  EXPECT_EQ(kDescentRateMaxRuleId, "rate.descent.max");
  EXPECT_EQ(kAscentRateMaxRuleId, "rate.ascent.max");
  EXPECT_EQ(kDefaultMaxDescentRate, 0.5);
  EXPECT_EQ(kDefaultMaxAscentRate, 0.5);
}

TEST(HeaveRateBoundRuleTest, NominalZeroRateIsCompliant) {
  ExpectCompliant(EvaluateHeaveRateBoundRule(0.0));
  ExpectCompliant(EvaluateHeaveRateBoundRule(-0.0));
}

TEST(HeaveRateBoundRuleTest, JustInsideIsCompliant) {
  ExpectCompliant(EvaluateHeaveRateBoundRule(0.499));
  ExpectCompliant(EvaluateHeaveRateBoundRule(-0.499));
}

TEST(HeaveRateBoundRuleTest, EqualToLimitIsCompliant) {
  ExpectCompliant(EvaluateHeaveRateBoundRule(0.5));
  ExpectCompliant(EvaluateHeaveRateBoundRule(-0.5));
  ExpectCompliant(EvaluateHeaveRateBoundRule(0.2, 0.2, 0.7));
  ExpectCompliant(EvaluateHeaveRateBoundRule(-0.7, 0.2, 0.7));
  ExpectCompliant(EvaluateHeaveRateBoundRule(0.0, 0.0, 0.0));
}

TEST(HeaveRateBoundRuleTest, PureDescentOverClampsToDescentLimit) {
  ExpectClamp(EvaluateHeaveRateBoundRule(0.501), kDescentId, 0.5);
  ExpectClamp(EvaluateHeaveRateBoundRule(2.0), kDescentId, 0.5);
  ExpectClamp(EvaluateHeaveRateBoundRule(0.3, 0.2, 0.7), kDescentId, 0.2);
  ExpectClamp(EvaluateHeaveRateBoundRule(0.001, 0.0, 0.7), kDescentId, 0.0);
}

TEST(HeaveRateBoundRuleTest, PureAscentOverClampsToNegativeAscentLimit) {
  ExpectClamp(EvaluateHeaveRateBoundRule(-0.501), kAscentId, -0.5);
  ExpectClamp(EvaluateHeaveRateBoundRule(-2.0), kAscentId, -0.5);
  ExpectClamp(EvaluateHeaveRateBoundRule(-0.8, 0.2, 0.7), kAscentId, -0.7);
  ExpectClamp(EvaluateHeaveRateBoundRule(-0.001, 0.2, 0.0), kAscentId, -0.0);
}

TEST(HeaveRateBoundRuleTest, AsymmetricLimitsAreIndependent) {
  ExpectCompliant(EvaluateHeaveRateBoundRule(0.2, 0.2, 1.0));
  ExpectCompliant(EvaluateHeaveRateBoundRule(-1.0, 0.2, 1.0));
  ExpectClamp(EvaluateHeaveRateBoundRule(0.25, 0.2, 1.0), kDescentId, 0.2);
  ExpectClamp(EvaluateHeaveRateBoundRule(-1.5, 0.2, 1.0), kAscentId, -1.0);
}

TEST(HeaveRateBoundRuleTest, NonFiniteDepthRateRejects) {
  ExpectRejectCritical(EvaluateHeaveRateBoundRule(kNaN), kDescentId);
  ExpectRejectCritical(EvaluateHeaveRateBoundRule(kInf), kDescentId);
  ExpectRejectCritical(EvaluateHeaveRateBoundRule(-kInf), kDescentId);
}

TEST(HeaveRateBoundRuleTest, BadDescentMaxRejectsAsDescent) {
  ExpectRejectCritical(EvaluateHeaveRateBoundRule(0.1, kNaN, 0.5), kDescentId);
  ExpectRejectCritical(EvaluateHeaveRateBoundRule(0.1, kInf, 0.5), kDescentId);
  ExpectRejectCritical(EvaluateHeaveRateBoundRule(0.1, -kInf, 0.5),
                       kDescentId);
  ExpectRejectCritical(EvaluateHeaveRateBoundRule(0.1, -0.001, 0.5),
                       kDescentId);
}

TEST(HeaveRateBoundRuleTest, BadAscentMaxOnlyRejectsAsAscent) {
  ExpectRejectCritical(EvaluateHeaveRateBoundRule(0.1, 0.5, kNaN), kAscentId);
  ExpectRejectCritical(EvaluateHeaveRateBoundRule(0.1, 0.5, kInf), kAscentId);
  ExpectRejectCritical(EvaluateHeaveRateBoundRule(0.1, 0.5, -kInf), kAscentId);
  ExpectRejectCritical(EvaluateHeaveRateBoundRule(0.1, 0.5, -0.001),
                       kAscentId);
}

TEST(HeaveRateBoundRuleTest, BothMaxesBadRejectsAsDescent) {
  ExpectRejectCritical(EvaluateHeaveRateBoundRule(0.1, -1.0, -1.0), kDescentId);
  ExpectRejectCritical(EvaluateHeaveRateBoundRule(0.1, kNaN, kNaN), kDescentId);
}

TEST(HeaveRateBoundRuleTest, BadLimitRejectsEvenWhenRateWouldBeCompliant) {
  ExpectRejectCritical(EvaluateHeaveRateBoundRule(0.0, 0.5, -1.0), kAscentId);
  ExpectRejectCritical(EvaluateHeaveRateBoundRule(0.0, -1.0, 0.5), kDescentId);
}

TEST(HeaveRateBoundRuleTest, NegativeZeroMaxIsUsable) {
  ExpectCompliant(EvaluateHeaveRateBoundRule(0.0, -0.0, -0.0));
}

TEST(HeaveRateBoundRuleTest, SimultaneousViolationYieldsSingleResult) {
  // Only one result is returned per call. With both limits at zero, a
  // positive rate is the descent finding.
  ExpectClamp(EvaluateHeaveRateBoundRule(0.1, 0.0, 0.0), kDescentId, 0.0);
}

}  // namespace
}  // namespace intrinsic::safety
