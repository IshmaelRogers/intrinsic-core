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

#include "intrinsic/safety/energy_reserve_rule.h"

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

EnergyReserveSample Sample(double available_j, double required_j) {
  return EnergyReserveSample{available_j, required_j, /*energy_known=*/true,
                             /*prediction_horizon_s=*/60.0,
                             /*prediction_usable=*/true};
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
  EXPECT_EQ(result.rule_id, "energy.reserve");
  EXPECT_EQ(result.rule_id, kEnergyReserveRuleId);
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

void ExpectBelowReserve(const SafetyRuleResult &result, double available_j) {
  EXPECT_TRUE(result.violated);
  EXPECT_EQ(result.rule_id, "energy.reserve");
  EXPECT_EQ(result.severity, 3);
  EXPECT_EQ(result.severity,
            static_cast<int>(SafetyFindingSeverityKind::kError));
  EXPECT_EQ(result.recommended_kind, 3);
  EXPECT_EQ(result.recommended_kind,
            static_cast<int>(SafetyDecisionKind::kReject));
  EXPECT_NE(result.recommended_kind, kDecisionKindProject);
  EXPECT_TRUE(result.has_projected_value);
  EXPECT_NEAR(result.projected_value, available_j, 1e-12);
  EXPECT_NE(result.summary.find("available_j="), std::string_view::npos)
      << result.summary;
  EXPECT_NE(result.summary.find("required_j="), std::string_view::npos)
      << result.summary;
}

TEST(EnergyReserveRuleTest, RuleIdIsLocked) {
  EXPECT_EQ(kEnergyReserveRuleId, "energy.reserve");
}

TEST(EnergyReserveRuleTest, AboveRequiredIsCompliant) {
  ExpectCompliant(EvaluateEnergyReserveRule(Sample(1000.0 + kEps, 1000.0)));
  ExpectCompliant(EvaluateEnergyReserveRule(Sample(5000.0, 1000.0)));
}

TEST(EnergyReserveRuleTest, EqualToRequiredIsCompliant) {
  ExpectCompliant(EvaluateEnergyReserveRule(Sample(1000.0, 1000.0)));
  ExpectCompliant(EvaluateEnergyReserveRule(Sample(0.0, 0.0)));
}

TEST(EnergyReserveRuleTest, BelowRequiredIsRejectedWithBothValues) {
  const double available = 1000.0 - kEps;
  const SafetyRuleResult result =
      EvaluateEnergyReserveRule(Sample(available, 1000.0));
  ExpectBelowReserve(result, available);
  EXPECT_NE(result.summary.find("required_j=1000"), std::string_view::npos)
      << result.summary;
}

TEST(EnergyReserveRuleTest, BelowRequiredSummaryIncludesBothValues) {
  EXPECT_EQ(EvaluateEnergyReserveRule(Sample(250.5, 1000.0)).summary,
            "energy below reserve available_j=250.5 required_j=1000");
  EXPECT_EQ(EvaluateEnergyReserveRule(Sample(0.0, 10.0)).summary,
            "energy below reserve available_j=0 required_j=10");
}

TEST(EnergyReserveRuleTest, ZeroAvailableAgainstPositiveRequiredIsBelow) {
  ExpectBelowReserve(EvaluateEnergyReserveRule(Sample(0.0, 10.0)), 0.0);
}

TEST(EnergyReserveRuleTest, BelowReserveSummariesDoNotAlias) {
  const SafetyRuleResult first =
      EvaluateEnergyReserveRule(Sample(250.5, 1000.0));
  const SafetyRuleResult second = EvaluateEnergyReserveRule(Sample(1.0, 2.0));
  EXPECT_EQ(first.summary,
            "energy below reserve available_j=250.5 required_j=1000");
  EXPECT_EQ(second.summary, "energy below reserve available_j=1 required_j=2");
}

TEST(EnergyReserveRuleTest, UnknownEnergyIsCriticalEvenWhenNumbersPass) {
  EnergyReserveSample sample = Sample(5000.0, 1000.0);
  sample.energy_known = false;
  const SafetyRuleResult result = EvaluateEnergyReserveRule(sample);
  ExpectCritical(result);
  EXPECT_NE(result.summary.find("unknown energy"), std::string_view::npos);
  sample = Sample(0.0, 1000.0);
  sample.energy_known = false;
  ExpectCritical(EvaluateEnergyReserveRule(sample));
}

TEST(EnergyReserveRuleTest, UnusablePredictionIsCritical) {
  EnergyReserveSample sample = Sample(5000.0, 1000.0);
  sample.prediction_usable = false;
  const SafetyRuleResult result = EvaluateEnergyReserveRule(sample);
  ExpectCritical(result);
  EXPECT_NE(result.summary.find("prediction unusable"),
            std::string_view::npos);
}

TEST(EnergyReserveRuleTest, NonPositiveOrNonFiniteHorizonIsCritical) {
  for (double bad : {0.0, -1.0, -kEps, kNaN, kInf, -kInf}) {
    EnergyReserveSample sample = Sample(5000.0, 1000.0);
    sample.prediction_horizon_s = bad;
    ExpectCritical(EvaluateEnergyReserveRule(sample));
  }
}

TEST(EnergyReserveRuleTest, NonFiniteAvailableOrRequiredIsCritical) {
  for (double bad : {kNaN, kInf, -kInf}) {
    ExpectCritical(EvaluateEnergyReserveRule(Sample(bad, 1000.0)));
    ExpectCritical(EvaluateEnergyReserveRule(Sample(5000.0, bad)));
  }
}

TEST(EnergyReserveRuleTest, NegativeAvailableOrRequiredIsCritical) {
  ExpectCritical(EvaluateEnergyReserveRule(Sample(-1.0, 1000.0)));
  ExpectCritical(EvaluateEnergyReserveRule(Sample(-1.0, -2.0)));
  ExpectCritical(EvaluateEnergyReserveRule(Sample(5000.0, -1.0)));
}

} // namespace
} // namespace intrinsic::safety
