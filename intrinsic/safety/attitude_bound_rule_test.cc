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

#include "intrinsic/safety/attitude_bound_rule.h"

#include <cmath>
#include <limits>
#include <string_view>

#include "gtest/gtest.h"
#include "intrinsic/safety/safety_decision_policy.h"

namespace intrinsic::safety {
namespace {

constexpr std::string_view kPitchId = kAttitudePitchMaxRuleId;
constexpr std::string_view kRollId = kAttitudeRollMaxRuleId;

constexpr double kInf = std::numeric_limits<double>::infinity();
constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();
constexpr double kPi = 3.14159265358979323846;

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

TEST(AttitudeBoundRuleTest, RuleIdsAndDefaultsAreLocked) {
  EXPECT_EQ(kAttitudePitchMaxRuleId, "attitude.pitch.max");
  EXPECT_EQ(kAttitudeRollMaxRuleId, "attitude.roll.max");
  EXPECT_EQ(kDefaultMaxPitch, kPi / 6.0);
  EXPECT_EQ(kDefaultMaxRoll, kPi / 6.0);
}

TEST(AttitudeBoundRuleTest, WrapToPiMapsIntoHalfOpenInterval) {
  EXPECT_EQ(WrapToPi(0.0), 0.0);
  EXPECT_EQ(WrapToPi(1.0), 1.0);
  EXPECT_EQ(WrapToPi(-1.0), -1.0);
  EXPECT_EQ(WrapToPi(kPi), kPi);
  EXPECT_EQ(WrapToPi(-kPi), kPi);
  EXPECT_NEAR(WrapToPi(kPi + 0.1), -kPi + 0.1, 1e-12);
  EXPECT_NEAR(WrapToPi(-kPi - 0.1), kPi - 0.1, 1e-12);
  EXPECT_NEAR(WrapToPi(2.0 * kPi), 0.0, 1e-12);
  EXPECT_NEAR(WrapToPi(2.0 * kPi + 0.1), 0.1, 1e-12);
  // -3*pi is on the branch cut, so rounding may land on either end.
  EXPECT_NEAR(std::fabs(WrapToPi(-3.0 * kPi)), kPi, 1e-12);
  EXPECT_NEAR(WrapToPi(7.0 * kPi + 0.5), -kPi + 0.5, 1e-9);
}

TEST(AttitudeBoundRuleTest, WrapToPiNonFiniteIsNaN) {
  EXPECT_TRUE(std::isnan(WrapToPi(kNaN)));
  EXPECT_TRUE(std::isnan(WrapToPi(kInf)));
  EXPECT_TRUE(std::isnan(WrapToPi(-kInf)));
}

TEST(AttitudeBoundRuleTest, PitchJustInsideIsCompliant) {
  ExpectCompliant(EvaluateAttitudePitchMaxRule(kDefaultMaxPitch - 1e-9));
  ExpectCompliant(EvaluateAttitudePitchMaxRule(-(kDefaultMaxPitch - 1e-9)));
  ExpectCompliant(EvaluateAttitudePitchMaxRule(0.0));
  ExpectCompliant(EvaluateAttitudePitchMaxRule(-0.0));
}

TEST(AttitudeBoundRuleTest, PitchEqualToLimitIsCompliant) {
  ExpectCompliant(EvaluateAttitudePitchMaxRule(kDefaultMaxPitch));
  ExpectCompliant(EvaluateAttitudePitchMaxRule(-kDefaultMaxPitch));
  ExpectCompliant(EvaluateAttitudePitchMaxRule(0.25, 0.25));
  ExpectCompliant(EvaluateAttitudePitchMaxRule(-0.25, 0.25));
  ExpectCompliant(EvaluateAttitudePitchMaxRule(0.0, 0.0));
}

TEST(AttitudeBoundRuleTest, PitchJustOutsideClampsWithSign) {
  ExpectClamp(EvaluateAttitudePitchMaxRule(kDefaultMaxPitch + 1e-6), kPitchId,
              kDefaultMaxPitch);
  ExpectClamp(EvaluateAttitudePitchMaxRule(-(kDefaultMaxPitch + 1e-6)),
              kPitchId, -kDefaultMaxPitch);
  ExpectClamp(EvaluateAttitudePitchMaxRule(1.0), kPitchId, kDefaultMaxPitch);
  ExpectClamp(EvaluateAttitudePitchMaxRule(-1.0), kPitchId, -kDefaultMaxPitch);
  ExpectClamp(EvaluateAttitudePitchMaxRule(0.3, 0.25), kPitchId, 0.25);
  ExpectClamp(EvaluateAttitudePitchMaxRule(-0.3, 0.25), kPitchId, -0.25);
  ExpectClamp(EvaluateAttitudePitchMaxRule(0.001, 0.0), kPitchId, 0.0);
}

TEST(AttitudeBoundRuleTest, PitchWrapsBeforeCompare) {
  // 2*pi + 0.1 wraps to 0.1, which is inside the default limit.
  ExpectCompliant(EvaluateAttitudePitchMaxRule(2.0 * kPi + 0.1));
  ExpectCompliant(EvaluateAttitudePitchMaxRule(-2.0 * kPi - 0.1));
  // pi + 0.1 wraps to about -(pi - 0.1), which clamps to the negative limit.
  ExpectClamp(EvaluateAttitudePitchMaxRule(kPi + 0.1), kPitchId,
              -kDefaultMaxPitch);
  ExpectClamp(EvaluateAttitudePitchMaxRule(-kPi - 0.1), kPitchId,
              kDefaultMaxPitch);
  // Exactly pi is outside a 30 degree limit and keeps its positive sign.
  ExpectClamp(EvaluateAttitudePitchMaxRule(kPi), kPitchId, kDefaultMaxPitch);
  ExpectClamp(EvaluateAttitudePitchMaxRule(-kPi), kPitchId, kDefaultMaxPitch);
  // A wide limit still accepts a wrapped value that sits on the boundary.
  ExpectCompliant(EvaluateAttitudePitchMaxRule(kPi, kPi));
  ExpectCompliant(EvaluateAttitudePitchMaxRule(-kPi, kPi));
  ExpectCompliant(EvaluateAttitudePitchMaxRule(kPi + 0.1, kPi));
}

TEST(AttitudeBoundRuleTest, PitchNonFiniteInputRejects) {
  ExpectRejectCritical(EvaluateAttitudePitchMaxRule(kNaN), kPitchId);
  ExpectRejectCritical(EvaluateAttitudePitchMaxRule(kInf), kPitchId);
  ExpectRejectCritical(EvaluateAttitudePitchMaxRule(-kInf), kPitchId);
}

TEST(AttitudeBoundRuleTest, PitchNonFiniteMaxRejects) {
  ExpectRejectCritical(EvaluateAttitudePitchMaxRule(0.1, kNaN), kPitchId);
  ExpectRejectCritical(EvaluateAttitudePitchMaxRule(0.1, kInf), kPitchId);
  ExpectRejectCritical(EvaluateAttitudePitchMaxRule(0.1, -kInf), kPitchId);
}

TEST(AttitudeBoundRuleTest, PitchNegativeMaxRejects) {
  ExpectRejectCritical(EvaluateAttitudePitchMaxRule(0.1, -0.001), kPitchId);
  ExpectRejectCritical(EvaluateAttitudePitchMaxRule(0.0, -1.0), kPitchId);
}

TEST(AttitudeBoundRuleTest, PitchNegativeZeroMaxIsUsable) {
  ExpectCompliant(EvaluateAttitudePitchMaxRule(0.0, -0.0));
}

TEST(AttitudeBoundRuleTest, RollJustInsideIsCompliant) {
  ExpectCompliant(EvaluateAttitudeRollMaxRule(kDefaultMaxRoll - 1e-9));
  ExpectCompliant(EvaluateAttitudeRollMaxRule(-(kDefaultMaxRoll - 1e-9)));
  ExpectCompliant(EvaluateAttitudeRollMaxRule(0.0));
  ExpectCompliant(EvaluateAttitudeRollMaxRule(-0.0));
}

TEST(AttitudeBoundRuleTest, RollEqualToLimitIsCompliant) {
  ExpectCompliant(EvaluateAttitudeRollMaxRule(kDefaultMaxRoll));
  ExpectCompliant(EvaluateAttitudeRollMaxRule(-kDefaultMaxRoll));
  ExpectCompliant(EvaluateAttitudeRollMaxRule(0.25, 0.25));
  ExpectCompliant(EvaluateAttitudeRollMaxRule(-0.25, 0.25));
  ExpectCompliant(EvaluateAttitudeRollMaxRule(0.0, 0.0));
}

TEST(AttitudeBoundRuleTest, RollJustOutsideClampsWithSign) {
  ExpectClamp(EvaluateAttitudeRollMaxRule(kDefaultMaxRoll + 1e-6), kRollId,
              kDefaultMaxRoll);
  ExpectClamp(EvaluateAttitudeRollMaxRule(-(kDefaultMaxRoll + 1e-6)), kRollId,
              -kDefaultMaxRoll);
  ExpectClamp(EvaluateAttitudeRollMaxRule(1.0), kRollId, kDefaultMaxRoll);
  ExpectClamp(EvaluateAttitudeRollMaxRule(-1.0), kRollId, -kDefaultMaxRoll);
  ExpectClamp(EvaluateAttitudeRollMaxRule(0.3, 0.25), kRollId, 0.25);
  ExpectClamp(EvaluateAttitudeRollMaxRule(-0.3, 0.25), kRollId, -0.25);
  ExpectClamp(EvaluateAttitudeRollMaxRule(0.001, 0.0), kRollId, 0.0);
}

TEST(AttitudeBoundRuleTest, RollWrapsBeforeCompare) {
  // 2*pi + 0.1 wraps to 0.1, which is inside the default limit.
  ExpectCompliant(EvaluateAttitudeRollMaxRule(2.0 * kPi + 0.1));
  ExpectCompliant(EvaluateAttitudeRollMaxRule(-2.0 * kPi - 0.1));
  // pi + 0.1 wraps to about -(pi - 0.1), which clamps to the negative limit.
  ExpectClamp(EvaluateAttitudeRollMaxRule(kPi + 0.1), kRollId,
              -kDefaultMaxRoll);
  ExpectClamp(EvaluateAttitudeRollMaxRule(-kPi - 0.1), kRollId,
              kDefaultMaxRoll);
  // Exactly pi is outside a 30 degree limit and keeps its positive sign.
  ExpectClamp(EvaluateAttitudeRollMaxRule(kPi), kRollId, kDefaultMaxRoll);
  ExpectClamp(EvaluateAttitudeRollMaxRule(-kPi), kRollId, kDefaultMaxRoll);
  // A wide limit still accepts a wrapped value that sits on the boundary.
  ExpectCompliant(EvaluateAttitudeRollMaxRule(kPi, kPi));
  ExpectCompliant(EvaluateAttitudeRollMaxRule(-kPi, kPi));
  ExpectCompliant(EvaluateAttitudeRollMaxRule(kPi + 0.1, kPi));
}

TEST(AttitudeBoundRuleTest, RollNonFiniteInputRejects) {
  ExpectRejectCritical(EvaluateAttitudeRollMaxRule(kNaN), kRollId);
  ExpectRejectCritical(EvaluateAttitudeRollMaxRule(kInf), kRollId);
  ExpectRejectCritical(EvaluateAttitudeRollMaxRule(-kInf), kRollId);
}

TEST(AttitudeBoundRuleTest, RollNonFiniteMaxRejects) {
  ExpectRejectCritical(EvaluateAttitudeRollMaxRule(0.1, kNaN), kRollId);
  ExpectRejectCritical(EvaluateAttitudeRollMaxRule(0.1, kInf), kRollId);
  ExpectRejectCritical(EvaluateAttitudeRollMaxRule(0.1, -kInf), kRollId);
}

TEST(AttitudeBoundRuleTest, RollNegativeMaxRejects) {
  ExpectRejectCritical(EvaluateAttitudeRollMaxRule(0.1, -0.001), kRollId);
  ExpectRejectCritical(EvaluateAttitudeRollMaxRule(0.0, -1.0), kRollId);
}

TEST(AttitudeBoundRuleTest, RollNegativeZeroMaxIsUsable) {
  ExpectCompliant(EvaluateAttitudeRollMaxRule(0.0, -0.0));
}

TEST(AttitudeBoundRuleTest, PitchAndRollUseTheirOwnRuleIds) {
  ExpectClamp(EvaluateAttitudePitchMaxRule(1.0), "attitude.pitch.max",
              kDefaultMaxPitch);
  ExpectClamp(EvaluateAttitudeRollMaxRule(1.0), "attitude.roll.max",
              kDefaultMaxRoll);
}

}  // namespace
}  // namespace intrinsic::safety
