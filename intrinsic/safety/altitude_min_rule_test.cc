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

#include "intrinsic/safety/altitude_min_rule.h"

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
  EXPECT_EQ(result.rule_id, "altitude.min");
  EXPECT_EQ(result.rule_id, kAltitudeMinRuleId);
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
  EXPECT_EQ(result.rule_id, "altitude.min");
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

TEST(AltitudeMinRuleTest, DefaultMinAltitudeIsTwoMeters) {
  EXPECT_EQ(kDefaultMinAltitude, 2.0);
}

TEST(AltitudeMinRuleTest, JustAboveIsCompliant) {
  ExpectCompliant(EvaluateAltitudeMinRule(true, 2.001));
  ExpectCompliant(EvaluateAltitudeMinRule(true, 50.0));
}

TEST(AltitudeMinRuleTest, EqualToLimitIsCompliant) {
  ExpectCompliant(EvaluateAltitudeMinRule(true, 2.0));
  ExpectCompliant(EvaluateAltitudeMinRule(true, 2.0, kDefaultMinAltitude));
  ExpectCompliant(EvaluateAltitudeMinRule(true, 0.0, 0.0));
  ExpectCompliant(EvaluateAltitudeMinRule(true, 5.5, 5.5));
}

TEST(AltitudeMinRuleTest, JustBelowClampsToLimit) {
  ExpectClamp(EvaluateAltitudeMinRule(true, 1.999), 2.0);
  ExpectClamp(EvaluateAltitudeMinRule(true, 0.0), 2.0);
  ExpectClamp(EvaluateAltitudeMinRule(true, -1.0), 2.0);
  ExpectClamp(EvaluateAltitudeMinRule(true, 4.9, 5.0), 5.0);
}

TEST(AltitudeMinRuleTest, UnknownAltitudeRejectsFailClosed) {
  ExpectRejectCritical(EvaluateAltitudeMinRule(false, 100.0));
  ExpectRejectCritical(EvaluateAltitudeMinRule(false, 0.0));
  ExpectRejectCritical(EvaluateAltitudeMinRule(false, 100.0, 2.0));
  ExpectRejectCritical(EvaluateAltitudeMinRule(false, kNaN, -1.0));
}

TEST(AltitudeMinRuleTest, NonFiniteAltitudeRejects) {
  ExpectRejectCritical(EvaluateAltitudeMinRule(true, kNaN));
  ExpectRejectCritical(EvaluateAltitudeMinRule(true, kInf));
  ExpectRejectCritical(EvaluateAltitudeMinRule(true, -kInf));
}

TEST(AltitudeMinRuleTest, BadMinAltitudeRejects) {
  ExpectRejectCritical(EvaluateAltitudeMinRule(true, 10.0, kNaN));
  ExpectRejectCritical(EvaluateAltitudeMinRule(true, 10.0, kInf));
  ExpectRejectCritical(EvaluateAltitudeMinRule(true, 10.0, -kInf));
  ExpectRejectCritical(EvaluateAltitudeMinRule(true, 10.0, -0.001));
}

}  // namespace
}  // namespace intrinsic::safety
