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

#include "intrinsic/safety/nav_covariance_rule.h"

#include <cmath>
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

void SetDiag(double (&c)[9], double a, double b, double d) {
  for (double &entry : c) {
    entry = 0.0;
  }
  c[0] = a;
  c[4] = b;
  c[8] = d;
}

// Diagonal covariances with the given variances (sigma squared).
NavCovarianceSample Sample(double position_var, double velocity_var) {
  NavCovarianceSample sample;
  SetDiag(sample.position_cov, position_var, 0.01, 0.01);
  SetDiag(sample.velocity_cov, velocity_var, 0.01, 0.01);
  return sample;
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
  EXPECT_EQ(result.rule_id, "nav.covariance");
  EXPECT_EQ(result.rule_id, kNavCovarianceRuleId);
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

void ExpectExceeded(const SafetyRuleResult &result, std::string_view metric,
                    double sigma) {
  EXPECT_TRUE(result.violated);
  EXPECT_EQ(result.rule_id, "nav.covariance");
  EXPECT_EQ(result.severity, 3);
  EXPECT_EQ(result.severity,
            static_cast<int>(SafetyFindingSeverityKind::kError));
  EXPECT_EQ(result.recommended_kind, 3);
  EXPECT_EQ(result.recommended_kind,
            static_cast<int>(SafetyDecisionKind::kReject));
  EXPECT_NE(result.recommended_kind, kDecisionKindProject);
  EXPECT_TRUE(result.has_projected_value);
  EXPECT_NEAR(result.projected_value, sigma, 1e-12);
  EXPECT_NE(result.summary.find("metric=" + std::string(metric)),
            std::string_view::npos)
      << result.summary;
  EXPECT_NE(result.summary.find("sigma="), std::string_view::npos)
      << result.summary;
}

TEST(NavCovarianceRuleTest, RuleIdAndDefaultsAreLocked) {
  EXPECT_EQ(kNavCovarianceRuleId, "nav.covariance");
  EXPECT_EQ(kDefaultMaxPositionSigmaM, 1.0);
  EXPECT_EQ(kDefaultMaxVelocitySigmaMps, 0.5);
}

TEST(NavCovarianceRuleTest, NominalDiagonalIsCompliant) {
  ExpectCompliant(EvaluateNavCovarianceRule(Sample(0.25, 0.04)));
  ExpectCompliant(EvaluateNavCovarianceRule(Sample(0.0, 0.0)));
}

TEST(NavCovarianceRuleTest, PositionJustUnderEqualOver) {
  ExpectCompliant(EvaluateNavCovarianceRule(Sample(1.0 - kEps, 0.04)));
  ExpectCompliant(EvaluateNavCovarianceRule(Sample(1.0, 0.04)));
  const SafetyRuleResult result =
      EvaluateNavCovarianceRule(Sample(1.0 + kEps, 0.04));
  ExpectExceeded(result, "position", std::sqrt(1.0 + kEps));
}

TEST(NavCovarianceRuleTest, VelocityJustUnderEqualOver) {
  ExpectCompliant(EvaluateNavCovarianceRule(Sample(0.25, 0.25 - kEps)));
  ExpectCompliant(EvaluateNavCovarianceRule(Sample(0.25, 0.25)));
  const SafetyRuleResult result =
      EvaluateNavCovarianceRule(Sample(0.25, 0.25 + kEps));
  ExpectExceeded(result, "velocity", std::sqrt(0.25 + kEps));
}

TEST(NavCovarianceRuleTest, SigmaIsSqrtOfMaxDiagonal) {
  NavCovarianceSample sample = Sample(0.25, 0.04);
  sample.position_cov[8] = 4.0;
  const SafetyRuleResult result = EvaluateNavCovarianceRule(sample);
  ExpectExceeded(result, "position", 2.0);
}

TEST(NavCovarianceRuleTest, ExceededSummaryIncludesMetricAndSigma) {
  EXPECT_EQ(EvaluateNavCovarianceRule(Sample(4.0, 0.04)).summary,
            "nav covariance exceeded metric=position sigma=2");
  EXPECT_EQ(EvaluateNavCovarianceRule(Sample(0.25, 1.0)).summary,
            "nav covariance exceeded metric=velocity sigma=1");
}

TEST(NavCovarianceRuleTest, ExceededSummariesDoNotAlias) {
  const SafetyRuleResult first = EvaluateNavCovarianceRule(Sample(4.0, 0.04));
  const SafetyRuleResult second = EvaluateNavCovarianceRule(Sample(0.25, 1.0));
  EXPECT_EQ(first.summary, "nav covariance exceeded metric=position sigma=2");
  EXPECT_EQ(second.summary, "nav covariance exceeded metric=velocity sigma=1");
}

TEST(NavCovarianceRuleTest, BothExceededReportsPositionFirst) {
  const SafetyRuleResult result = EvaluateNavCovarianceRule(Sample(4.0, 1.0));
  ExpectExceeded(result, "position", 2.0);
  EXPECT_EQ(result.summary.find("metric=velocity"), std::string_view::npos);
}

TEST(NavCovarianceRuleTest, CustomThresholdsAreHonored) {
  ExpectCompliant(EvaluateNavCovarianceRule(Sample(4.0, 1.0), 2.0, 1.0));
  ExpectExceeded(EvaluateNavCovarianceRule(Sample(4.0 + kEps, 1.0), 2.0, 1.0),
                 "position", std::sqrt(4.0 + kEps));
  ExpectExceeded(EvaluateNavCovarianceRule(Sample(4.0, 1.0 + kEps), 2.0, 1.0),
                 "velocity", std::sqrt(1.0 + kEps));
}

TEST(NavCovarianceRuleTest, UnknownCovarianceIsCriticalEvenWhenNumbersPass) {
  NavCovarianceSample sample = Sample(0.25, 0.04);
  sample.position_known = false;
  const SafetyRuleResult result = EvaluateNavCovarianceRule(sample);
  ExpectCritical(result);
  EXPECT_NE(result.summary.find("unknown covariance"), std::string_view::npos);
  sample = Sample(0.25, 0.04);
  sample.velocity_known = false;
  ExpectCritical(EvaluateNavCovarianceRule(sample));
  sample.position_known = false;
  ExpectCritical(EvaluateNavCovarianceRule(sample));
}

TEST(NavCovarianceRuleTest, NegativeDiagonalIsCritical) {
  for (int index : {0, 4, 8}) {
    NavCovarianceSample sample = Sample(0.25, 0.04);
    sample.position_cov[index] = -1.0;
    const SafetyRuleResult result = EvaluateNavCovarianceRule(sample);
    ExpectCritical(result);
    EXPECT_NE(result.summary.find("position covariance not usable"),
              std::string_view::npos);
    sample = Sample(0.25, 0.04);
    sample.velocity_cov[index] = -1.0;
    const SafetyRuleResult velocity_result = EvaluateNavCovarianceRule(sample);
    ExpectCritical(velocity_result);
    EXPECT_NE(velocity_result.summary.find("velocity covariance not usable"),
              std::string_view::npos);
  }
}

TEST(NavCovarianceRuleTest, NonPsdWithPositiveDiagonalIsCritical) {
  NavCovarianceSample sample = Sample(1.0, 0.04);
  sample.position_cov[1] = sample.position_cov[3] = 2.0;
  ExpectCritical(EvaluateNavCovarianceRule(sample));
  sample = Sample(0.25, 0.04);
  sample.velocity_cov[0] = sample.velocity_cov[4] = sample.velocity_cov[8] = 1;
  sample.velocity_cov[1] = sample.velocity_cov[3] = 0.9;
  sample.velocity_cov[2] = sample.velocity_cov[6] = 0.9;
  sample.velocity_cov[5] = sample.velocity_cov[7] = -0.9;
  ExpectCritical(EvaluateNavCovarianceRule(sample));
}

TEST(NavCovarianceRuleTest, CorrelatedPsdIsCompliant) {
  NavCovarianceSample sample = Sample(0.25, 0.04);
  sample.position_cov[0] = sample.position_cov[4] = 0.25;
  sample.position_cov[1] = sample.position_cov[3] = 0.2;
  ExpectCompliant(EvaluateNavCovarianceRule(sample));
}

TEST(NavCovarianceRuleTest, AsymmetricMatrixIsCritical) {
  NavCovarianceSample sample = Sample(0.25, 0.04);
  sample.position_cov[1] = 0.01;
  sample.position_cov[3] = 0.0;
  ExpectCritical(EvaluateNavCovarianceRule(sample));
  sample = Sample(0.25, 0.04);
  sample.velocity_cov[5] = 0.0;
  sample.velocity_cov[7] = 1e-3;
  ExpectCritical(EvaluateNavCovarianceRule(sample));
}

TEST(NavCovarianceRuleTest, SmallAsymmetryWithinEpsIsAccepted) {
  NavCovarianceSample sample = Sample(0.25, 0.04);
  sample.position_cov[1] = 5e-10;
  sample.position_cov[3] = 0.0;
  ExpectCompliant(EvaluateNavCovarianceRule(sample));
}

TEST(NavCovarianceRuleTest, NonFiniteEntryIsCritical) {
  for (double bad : {kNaN, kInf, -kInf}) {
    for (int index = 0; index < 9; ++index) {
      NavCovarianceSample sample = Sample(0.25, 0.04);
      sample.position_cov[index] = bad;
      ExpectCritical(EvaluateNavCovarianceRule(sample));
      sample = Sample(0.25, 0.04);
      sample.velocity_cov[index] = bad;
      ExpectCritical(EvaluateNavCovarianceRule(sample));
    }
  }
}

TEST(NavCovarianceRuleTest, BadThresholdsAreCritical) {
  const NavCovarianceSample sample = Sample(0.25, 0.04);
  for (double bad : {0.0, -1.0, kNaN, kInf, -kInf}) {
    const SafetyRuleResult position =
        EvaluateNavCovarianceRule(sample, bad, 0.5);
    ExpectCritical(position);
    EXPECT_NE(position.summary.find("bad config"), std::string_view::npos);
    ExpectCritical(EvaluateNavCovarianceRule(sample, 1.0, bad));
  }
}

TEST(NavCovarianceRuleTest, BadThresholdBeatsUnknown) {
  NavCovarianceSample sample = Sample(0.25, 0.04);
  sample.position_known = false;
  const SafetyRuleResult result = EvaluateNavCovarianceRule(sample, 0.0, 0.5);
  ExpectCritical(result);
  EXPECT_NE(result.summary.find("bad config"), std::string_view::npos);
}

} // namespace
} // namespace intrinsic::safety
