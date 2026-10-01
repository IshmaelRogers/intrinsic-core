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

#include "intrinsic/safety/rule_aggregation.h"

#include <algorithm>
#include <array>
#include <cstddef>
#include <span>
#include <string_view>
#include <vector>

#include "intrinsic/safety/safety_decision_policy.h"
#include "intrinsic/safety/safety_rule_result.h"
#include "gtest/gtest.h"

namespace intrinsic::safety {
namespace {

SafetyRuleResult Compliant() { return SafetyRuleResult(); }

SafetyRuleResult Project(std::string_view rule_id, double value,
                         int severity = kSeverityError) {
  SafetyRuleResult r;
  r.violated = true;
  r.rule_id = rule_id;
  r.severity = severity;
  r.summary = "clamped";
  r.recommended_kind = kDecisionKindProject;
  r.has_projected_value = true;
  r.projected_value = value;
  return r;
}

SafetyRuleResult Reject(std::string_view rule_id,
                        int severity = kSeverityCritical,
                        std::string_view summary = "rejected") {
  SafetyRuleResult r;
  r.violated = true;
  r.rule_id = rule_id;
  r.severity = severity;
  r.summary = summary;
  r.recommended_kind = kDecisionKindReject;
  return r;
}

SafetyRuleResult AuditReject(std::string_view rule_id, double audit_value) {
  SafetyRuleResult r = Reject(rule_id, kSeverityError, "audit");
  r.has_projected_value = true;
  r.projected_value = audit_value;
  return r;
}

AggregatedSafetyResult Aggregate(const std::vector<SafetyRuleResult> &v) {
  return AggregateSafetyRuleResults(std::span<const SafetyRuleResult>(v));
}

void ExpectCompliant(const AggregatedSafetyResult &a, size_t violated_count) {
  EXPECT_FALSE(a.violated);
  EXPECT_TRUE(a.primary_rule_id.empty());
  EXPECT_EQ(a.severity, 0);
  EXPECT_TRUE(a.summary.empty());
  EXPECT_EQ(a.recommended_kind, 0);
  EXPECT_FALSE(a.has_projected_value);
  EXPECT_EQ(a.projected_value, 0.0);
  EXPECT_EQ(a.violated_count, violated_count);
  EXPECT_FALSE(a.projection_conflict);
}

void ExpectSame(const AggregatedSafetyResult &a,
                const AggregatedSafetyResult &b) {
  EXPECT_EQ(a.violated, b.violated);
  EXPECT_EQ(a.primary_rule_id, b.primary_rule_id);
  EXPECT_EQ(a.severity, b.severity);
  EXPECT_EQ(a.summary, b.summary);
  EXPECT_EQ(a.recommended_kind, b.recommended_kind);
  EXPECT_EQ(a.has_projected_value, b.has_projected_value);
  EXPECT_EQ(a.projected_value, b.projected_value);
  EXPECT_EQ(a.violated_count, b.violated_count);
  EXPECT_EQ(a.projection_conflict, b.projection_conflict);
}

// Every permutation of `v` must aggregate to the same result.
void ExpectPermutationInvariant(std::vector<SafetyRuleResult> v) {
  const AggregatedSafetyResult expected = Aggregate(v);
  std::vector<size_t> idx(v.size());
  for (size_t i = 0; i < idx.size(); ++i)
    idx[i] = i;
  do {
    std::vector<SafetyRuleResult> shuffled;
    for (size_t i : idx)
      shuffled.push_back(v[i]);
    ExpectSame(Aggregate(shuffled), expected);
  } while (std::next_permutation(idx.begin(), idx.end()));
}

TEST(RuleAggregationTest, WireNumbersMatchLockedEnums) {
  EXPECT_EQ(kDecisionKindProject,
            static_cast<int>(SafetyDecisionKind::kProject));
  EXPECT_EQ(kDecisionKindReject, static_cast<int>(SafetyDecisionKind::kReject));
  EXPECT_EQ(kSeverityError,
            static_cast<int>(SafetyFindingSeverityKind::kError));
  EXPECT_EQ(kSeverityCritical,
            static_cast<int>(SafetyFindingSeverityKind::kCritical));
}

TEST(RuleAggregationTest, EmptyIsCompliant) {
  ExpectCompliant(Aggregate({}), 0);
  ExpectCompliant(AggregateSafetyRuleResults({}), 0);
}

TEST(RuleAggregationTest, AllCompliantIsCompliant) {
  ExpectCompliant(Aggregate({Compliant(), Compliant(), Compliant()}), 0);
}

TEST(RuleAggregationTest, SingleRejectMirrorsFinding) {
  const AggregatedSafetyResult a =
      Aggregate({Compliant(), Reject("state.age", kSeverityCritical, "stale")});
  EXPECT_TRUE(a.violated);
  EXPECT_EQ(a.primary_rule_id, "state.age");
  EXPECT_EQ(a.severity, kSeverityCritical);
  EXPECT_EQ(a.summary, "stale");
  EXPECT_EQ(a.recommended_kind, kDecisionKindReject);
  EXPECT_FALSE(a.has_projected_value);
  EXPECT_EQ(a.projected_value, 0.0);
  EXPECT_EQ(a.violated_count, 1u);
  EXPECT_FALSE(a.projection_conflict);
}

TEST(RuleAggregationTest, SingleProjectKeepsClampValue) {
  const AggregatedSafetyResult a =
      Aggregate({Compliant(), Project("speed.max", 1.5), Compliant()});
  EXPECT_TRUE(a.violated);
  EXPECT_EQ(a.primary_rule_id, "speed.max");
  EXPECT_EQ(a.severity, kSeverityError);
  EXPECT_EQ(a.summary, "clamped");
  EXPECT_EQ(a.recommended_kind, kDecisionKindProject);
  EXPECT_TRUE(a.has_projected_value);
  EXPECT_EQ(a.projected_value, 1.5);
  EXPECT_EQ(a.violated_count, 1u);
  EXPECT_FALSE(a.projection_conflict);
}

TEST(RuleAggregationTest, RejectBeatsProjectInAnyOrder) {
  const std::vector<SafetyRuleResult> v = {
      Project("speed.max", 1.5),
      Reject("state.age", kSeverityCritical, "stale")};
  const AggregatedSafetyResult a = Aggregate(v);
  EXPECT_EQ(a.recommended_kind, kDecisionKindReject);
  EXPECT_FALSE(a.projection_conflict);
  EXPECT_FALSE(a.has_projected_value);
  EXPECT_EQ(a.primary_rule_id, "state.age");
  EXPECT_EQ(a.severity, kSeverityCritical);
  EXPECT_EQ(a.violated_count, 2u);
  ExpectPermutationInvariant(v);
}

TEST(RuleAggregationTest, RejectBeatsProjectEvenWhenProjectIsPrimary) {
  const std::vector<SafetyRuleResult> v = {
      Project("altitude.min", 3.0, kSeverityCritical),
      Reject("state.age", kSeverityError, "stale")};
  const AggregatedSafetyResult a = Aggregate(v);
  EXPECT_EQ(a.recommended_kind, kDecisionKindReject);
  EXPECT_FALSE(a.projection_conflict);
  EXPECT_EQ(a.primary_rule_id, "altitude.min");
  EXPECT_EQ(a.severity, kSeverityCritical);
  ExpectPermutationInvariant(v);
}

TEST(RuleAggregationTest, TwoProjectsEscalateToConflictReject) {
  const std::vector<SafetyRuleResult> v = {
      Project("speed.max", 1.5), Project("altitude.min", 3.0), Compliant()};
  const AggregatedSafetyResult a = Aggregate(v);
  EXPECT_TRUE(a.violated);
  EXPECT_EQ(a.recommended_kind, kDecisionKindReject);
  EXPECT_TRUE(a.projection_conflict);
  EXPECT_FALSE(a.has_projected_value);
  EXPECT_EQ(a.projected_value, 0.0);
  EXPECT_EQ(a.summary, "conflicting projections");
  EXPECT_EQ(a.primary_rule_id, "altitude.min");
  EXPECT_EQ(a.severity, kSeverityError);
  EXPECT_EQ(a.violated_count, 2u);
  ExpectPermutationInvariant(v);
}

TEST(RuleAggregationTest, EqualProjectedValuesStillConflict) {
  const std::vector<SafetyRuleResult> v = {Project("speed.max", 1.0),
                                           Project("depth.max", 1.0),
                                           Project("heave.rate", 1.0)};
  const AggregatedSafetyResult a = Aggregate(v);
  EXPECT_EQ(a.recommended_kind, kDecisionKindReject);
  EXPECT_TRUE(a.projection_conflict);
  EXPECT_EQ(a.primary_rule_id, "depth.max");
  ExpectPermutationInvariant(v);
}

TEST(RuleAggregationTest, SeverityIsMaxAndPrimaryIsLexFirstAtMax) {
  const std::vector<SafetyRuleResult> v = {
      Reject("nav.covariance", kSeverityError, "e1"),
      Reject("state.age", kSeverityCritical, "c2"),
      Reject("clearance.min", kSeverityError, "e2"),
      Reject("geofence", kSeverityCritical, "c1")};
  const AggregatedSafetyResult a = Aggregate(v);
  EXPECT_EQ(a.severity, kSeverityCritical);
  EXPECT_EQ(a.primary_rule_id, "geofence");
  EXPECT_EQ(a.summary, "c1");
  EXPECT_EQ(a.violated_count, 4u);
  ExpectPermutationInvariant(v);
}

TEST(RuleAggregationTest, TwoErrorsAndOneCritical) {
  const std::vector<SafetyRuleResult> v = {Reject("a.rule", kSeverityError),
                                           Reject("c.rule", kSeverityError),
                                           Reject("z.rule", kSeverityCritical)};
  const AggregatedSafetyResult a = Aggregate(v);
  EXPECT_EQ(a.severity, kSeverityCritical);
  EXPECT_EQ(a.primary_rule_id, "z.rule");
  ExpectPermutationInvariant(v);
}

TEST(RuleAggregationTest, AuditRejectAloneIsNotAConflict) {
  const AggregatedSafetyResult a =
      Aggregate({AuditReject("clearance.min", 0.4)});
  EXPECT_EQ(a.recommended_kind, kDecisionKindReject);
  EXPECT_FALSE(a.projection_conflict);
  EXPECT_EQ(a.primary_rule_id, "clearance.min");
  EXPECT_TRUE(a.has_projected_value);
  EXPECT_EQ(a.projected_value, 0.4);
}

TEST(RuleAggregationTest, TwoAuditRejectsAreNotAConflict) {
  const std::vector<SafetyRuleResult> v = {AuditReject("clearance.min", 0.4),
                                           AuditReject("energy.reserve", 9.0)};
  const AggregatedSafetyResult a = Aggregate(v);
  EXPECT_EQ(a.recommended_kind, kDecisionKindReject);
  EXPECT_FALSE(a.projection_conflict);
  EXPECT_EQ(a.primary_rule_id, "clearance.min");
  EXPECT_EQ(a.projected_value, 0.4);
  ExpectPermutationInvariant(v);
}

TEST(RuleAggregationTest, AuditRejectPlusOneProjectRejectWins) {
  const std::vector<SafetyRuleResult> v = {AuditReject("clearance.min", 0.4),
                                           Project("speed.max", 1.5)};
  const AggregatedSafetyResult a = Aggregate(v);
  EXPECT_EQ(a.recommended_kind, kDecisionKindReject);
  EXPECT_FALSE(a.projection_conflict);
  EXPECT_NE(a.summary, "conflicting projections");
  EXPECT_EQ(a.primary_rule_id, "clearance.min");
  EXPECT_TRUE(a.has_projected_value);
  EXPECT_EQ(a.projected_value, 0.4);
  ExpectPermutationInvariant(v);
}

TEST(RuleAggregationTest, RejectAggregateClearsProjectionWhenPrimaryHasNone) {
  const std::vector<SafetyRuleResult> v = {
      AuditReject("nav.covariance", 0.9),
      Reject("state.age", kSeverityCritical, "stale")};
  const AggregatedSafetyResult a = Aggregate(v);
  EXPECT_EQ(a.primary_rule_id, "state.age");
  EXPECT_FALSE(a.has_projected_value);
  EXPECT_EQ(a.projected_value, 0.0);
  ExpectPermutationInvariant(v);
}

TEST(RuleAggregationTest, UnknownKindFailsClosedToReject) {
  SafetyRuleResult odd = Reject("odd.rule");
  odd.recommended_kind = 7;
  const std::vector<SafetyRuleResult> v = {odd, Project("speed.max", 1.5)};
  const AggregatedSafetyResult a = Aggregate(v);
  EXPECT_EQ(a.recommended_kind, kDecisionKindReject);
  EXPECT_FALSE(a.projection_conflict);
  ExpectPermutationInvariant(v);

  SafetyRuleResult unspecified = Reject("odd.rule");
  unspecified.recommended_kind = kDecisionKindUnspecified;
  EXPECT_EQ(Aggregate({unspecified}).recommended_kind, kDecisionKindReject);
}

TEST(RuleAggregationTest, ViolatedCountIgnoresCompliantInputs) {
  const AggregatedSafetyResult a =
      Aggregate({Compliant(), Reject("a.rule"), Compliant(), Reject("b.rule"),
                 Compliant()});
  EXPECT_EQ(a.violated_count, 2u);
}

TEST(RuleAggregationTest, DoesNotMutateInputs) {
  const std::array<SafetyRuleResult, 2> v = {Project("speed.max", 1.5),
                                             Project("altitude.min", 3.0)};
  const std::array<SafetyRuleResult, 2> copy = v;
  AggregateSafetyRuleResults(std::span<const SafetyRuleResult>(v));
  for (size_t i = 0; i < v.size(); ++i) {
    EXPECT_EQ(v[i].rule_id, copy[i].rule_id);
    EXPECT_EQ(v[i].projected_value, copy[i].projected_value);
    EXPECT_EQ(v[i].recommended_kind, copy[i].recommended_kind);
  }
}

} // namespace
} // namespace intrinsic::safety
