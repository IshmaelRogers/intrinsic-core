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

#include "intrinsic/safety/clearance_rule.h"

#include <limits>
#include <string>
#include <string_view>

#include "intrinsic/safety/safety_decision_policy.h"
#include "gtest/gtest.h"

namespace intrinsic::safety {
namespace {

constexpr double kInf = std::numeric_limits<double>::infinity();
constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();
constexpr double kEps = 1e-6;

ClearanceSample Sample(ClearanceSource source, double clearance_m) {
  return ClearanceSample{"map", source, clearance_m, true};
}

void ExpectCompliant(const SafetyRuleResult &result) {
  EXPECT_FALSE(result.violated);
  EXPECT_TRUE(result.rule_id.empty());
  EXPECT_EQ(result.severity, 0);
  EXPECT_TRUE(result.summary.empty());
  EXPECT_EQ(result.recommended_kind, 0);
  EXPECT_FALSE(result.has_projected_value);
  EXPECT_EQ(result.projected_value, 0.0);
}

void ExpectCritical(const SafetyRuleResult &result) {
  EXPECT_TRUE(result.violated);
  EXPECT_EQ(result.rule_id, "clearance.min");
  EXPECT_EQ(result.rule_id, kClearanceMinRuleId);
  EXPECT_EQ(result.severity, 4);
  EXPECT_EQ(result.severity,
            static_cast<int>(SafetyFindingSeverityKind::kCritical));
  EXPECT_FALSE(result.summary.empty());
  EXPECT_EQ(result.recommended_kind, 3);
  EXPECT_EQ(result.recommended_kind, kDecisionKindReject);
  EXPECT_EQ(result.recommended_kind,
            static_cast<int>(SafetyDecisionKind::kReject));
  EXPECT_FALSE(result.has_projected_value);
  EXPECT_EQ(result.projected_value, 0.0);
}

void ExpectBelowMin(const SafetyRuleResult &result,
                    std::string_view source_name, double clearance_m) {
  EXPECT_TRUE(result.violated);
  EXPECT_EQ(result.rule_id, "clearance.min");
  EXPECT_EQ(result.severity, 3);
  EXPECT_EQ(result.severity,
            static_cast<int>(SafetyFindingSeverityKind::kError));
  EXPECT_EQ(result.recommended_kind, 3);
  EXPECT_EQ(result.recommended_kind,
            static_cast<int>(SafetyDecisionKind::kReject));
  EXPECT_NE(result.recommended_kind, kDecisionKindProject);
  EXPECT_TRUE(result.has_projected_value);
  EXPECT_NEAR(result.projected_value, clearance_m, 1e-12);
  const std::string expected_source = "source=" + std::string(source_name);
  EXPECT_NE(result.summary.find(expected_source), std::string_view::npos)
      << result.summary;
  EXPECT_NE(result.summary.find("clearance_m="), std::string_view::npos)
      << result.summary;
}

class ClearanceRuleSourceTest
    : public ::testing::TestWithParam<ClearanceSource> {
protected:
  std::string_view Name() const {
    return GetParam() == ClearanceSource::kObstacle ? "obstacle" : "seafloor";
  }
};

TEST(ClearanceRuleTest, RuleIdAndDefaultAreLocked) {
  EXPECT_EQ(kClearanceMinRuleId, "clearance.min");
  EXPECT_EQ(kDefaultMinClearanceM, 0.5);
}

TEST_P(ClearanceRuleSourceTest, JustAboveMinIsCompliant) {
  ExpectCompliant(EvaluateClearanceRule(Sample(GetParam(), 0.5 + kEps)));
}

TEST_P(ClearanceRuleSourceTest, EqualToMinIsCompliant) {
  ExpectCompliant(EvaluateClearanceRule(Sample(GetParam(), 0.5)));
}

TEST_P(ClearanceRuleSourceTest, JustBelowMinIsRejectedWithObservedValue) {
  const double clearance = 0.5 - kEps;
  ExpectBelowMin(EvaluateClearanceRule(Sample(GetParam(), clearance)), Name(),
                 clearance);
}

TEST_P(ClearanceRuleSourceTest, ZeroAndNegativeClearanceAreBelowMin) {
  ExpectBelowMin(EvaluateClearanceRule(Sample(GetParam(), 0.0)), Name(), 0.0);
  ExpectBelowMin(EvaluateClearanceRule(Sample(GetParam(), -1.5)), Name(), -1.5);
}

TEST_P(ClearanceRuleSourceTest, CustomMinClearanceIsHonored) {
  ExpectCompliant(EvaluateClearanceRule(Sample(GetParam(), 2.0), 2.0));
  ExpectBelowMin(EvaluateClearanceRule(Sample(GetParam(), 1.9), 2.0), Name(),
                 1.9);
  ExpectCompliant(EvaluateClearanceRule(Sample(GetParam(), 0.3), 0.25));
}

INSTANTIATE_TEST_SUITE_P(Sources, ClearanceRuleSourceTest,
                         ::testing::Values(ClearanceSource::kObstacle,
                                           ClearanceSource::kSeafloor));

TEST(ClearanceRuleTest, BelowMinSummaryIncludesSourceAndValue) {
  EXPECT_EQ(
      EvaluateClearanceRule(Sample(ClearanceSource::kObstacle, 0.25)).summary,
      "clearance below min source=obstacle clearance_m=0.25");
  EXPECT_EQ(
      EvaluateClearanceRule(Sample(ClearanceSource::kSeafloor, 0.1)).summary,
      "clearance below min source=seafloor clearance_m=0.1");
}

TEST(ClearanceRuleTest, BelowMinSummariesDoNotAlias) {
  const SafetyRuleResult first =
      EvaluateClearanceRule(Sample(ClearanceSource::kObstacle, 0.25));
  const SafetyRuleResult second =
      EvaluateClearanceRule(Sample(ClearanceSource::kSeafloor, 0.1));
  EXPECT_EQ(first.summary,
            "clearance below min source=obstacle clearance_m=0.25");
  EXPECT_EQ(second.summary,
            "clearance below min source=seafloor clearance_m=0.1");
}

TEST(ClearanceRuleTest, UnknownMapIsCriticalEvenWhenClearanceWouldPass) {
  const SafetyRuleResult result =
      EvaluateClearanceRule(Sample(ClearanceSource::kUnknownMap, 100.0));
  ExpectCritical(result);
  EXPECT_NE(result.summary.find("unknown map"), std::string_view::npos);
  ExpectCritical(
      EvaluateClearanceRule(Sample(ClearanceSource::kUnknownMap, 0.0)));
}

TEST(ClearanceRuleTest, OutOfRangeSourceFailsClosed) {
  ExpectCritical(
      EvaluateClearanceRule(Sample(static_cast<ClearanceSource>(99), 100.0)));
}

TEST(ClearanceRuleTest, UnusableSnapshotIsCritical) {
  for (ClearanceSource source :
       {ClearanceSource::kObstacle, ClearanceSource::kSeafloor,
        ClearanceSource::kUnknownMap}) {
    ClearanceSample sample = Sample(source, 100.0);
    sample.snapshot_usable = false;
    const SafetyRuleResult result = EvaluateClearanceRule(sample);
    ExpectCritical(result);
    EXPECT_NE(result.summary.find("snapshot unusable"), std::string_view::npos);
  }
}

TEST(ClearanceRuleTest, EmptyFrameIsCritical) {
  ClearanceSample sample = Sample(ClearanceSource::kObstacle, 100.0);
  sample.frame_id = "";
  ExpectCritical(EvaluateClearanceRule(sample));
}

TEST(ClearanceRuleTest, FrameIdIsAnAuditTagOnly) {
  ClearanceSample sample = Sample(ClearanceSource::kObstacle, 1.0);
  sample.frame_id = "ODOM";
  ExpectCompliant(EvaluateClearanceRule(sample));
}

TEST(ClearanceRuleTest, NonFiniteClearanceIsCritical) {
  for (double bad : {kNaN, kInf, -kInf}) {
    ExpectCritical(
        EvaluateClearanceRule(Sample(ClearanceSource::kObstacle, bad)));
    ExpectCritical(
        EvaluateClearanceRule(Sample(ClearanceSource::kSeafloor, bad)));
  }
}

TEST(ClearanceRuleTest, InvalidMinClearanceIsCritical) {
  for (double bad : {0.0, -0.5, -kEps, kNaN, kInf, -kInf}) {
    ExpectCritical(
        EvaluateClearanceRule(Sample(ClearanceSource::kObstacle, 10.0), bad));
    ExpectCritical(
        EvaluateClearanceRule(Sample(ClearanceSource::kSeafloor, 10.0), bad));
  }
}

} // namespace
} // namespace intrinsic::safety
