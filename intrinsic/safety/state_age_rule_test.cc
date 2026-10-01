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

#include "intrinsic/safety/state_age_rule.h"

#include <cstdint>
#include <limits>

#include "gtest/gtest.h"
#include "intrinsic/safety/safety_decision_policy.h"

namespace intrinsic::safety {
namespace {

constexpr int64_t kNanosPerSecond = 1000000000;

void ExpectCompliant(const SafetyRuleResult& result) {
  EXPECT_FALSE(result.violated);
  EXPECT_TRUE(result.rule_id.empty());
  EXPECT_EQ(result.severity, 0);
  EXPECT_TRUE(result.summary.empty());
  EXPECT_EQ(result.recommended_kind, 0);
  EXPECT_EQ(result.recommended_kind,
            static_cast<int>(SafetyDecisionKind::kUnspecified));
}

void ExpectAgeViolation(const SafetyRuleResult& result, const char* summary) {
  EXPECT_TRUE(result.violated);
  EXPECT_EQ(result.rule_id, "state.age");
  EXPECT_EQ(result.rule_id, kStateAgeRuleId);
  EXPECT_EQ(result.severity, 3);
  EXPECT_EQ(result.severity,
            static_cast<int>(SafetyFindingSeverityKind::kError));
  EXPECT_EQ(result.summary, summary);
  EXPECT_EQ(result.recommended_kind, 3);
  EXPECT_EQ(result.recommended_kind,
            static_cast<int>(SafetyDecisionKind::kReject));
  EXPECT_NE(result.recommended_kind,
            static_cast<int>(SafetyDecisionKind::kProject));
}

TEST(StateAgeRuleTest, DefaultMaxAgeIsTwoSeconds) {
  EXPECT_EQ(kDefaultStateMaxAge.seconds, 2);
  EXPECT_EQ(kDefaultStateMaxAge.nanos, 0);
}

TEST(StateAgeRuleTest, ExactMaxAgeIsCompliant) {
  const StateTime observation{10, 0};
  const StateTime query{12, 0};
  ExpectCompliant(EvaluateStateAgeRule(query, observation));
  ExpectCompliant(
      EvaluateStateAgeRule(query, observation, kDefaultStateMaxAge));
}

TEST(StateAgeRuleTest, OneNanosecondPastMaxAgeIsViolated) {
  const StateTime observation{10, 0};
  const StateTime query{12, 1};
  ExpectAgeViolation(EvaluateStateAgeRule(query, observation),
                     "state age exceeds max_age");
}

TEST(StateAgeRuleTest, ZeroAgeAndZeroMaxAgeAreCompliant) {
  const StateTime stamp{10, 1};
  ExpectCompliant(EvaluateStateAgeRule(stamp, stamp, StateTime{0, 0}));
}

TEST(StateAgeRuleTest, OneNanosecondPastZeroMaxAgeIsViolated) {
  const StateTime observation{10, 0};
  const StateTime query{10, 1};
  ExpectAgeViolation(EvaluateStateAgeRule(query, observation, StateTime{0, 0}),
                     "state age exceeds max_age");
}

TEST(StateAgeRuleTest, BorrowAcrossSecondMatchesExactNanosecondBound) {
  const StateTime observation{5, kNanosPerSecond - 1};
  const StateTime at_bound{6, 0};
  ExpectCompliant(EvaluateStateAgeRule(at_bound, observation, StateTime{0, 1}));

  const StateTime past_bound{6, 1};
  ExpectAgeViolation(
      EvaluateStateAgeRule(past_bound, observation, StateTime{0, 1}),
      "state age exceeds max_age");
}

TEST(StateAgeRuleTest, NegativeTimestampsUseTheSameBoundary) {
  const StateTime observation{-2, 0};
  ExpectCompliant(
      EvaluateStateAgeRule(StateTime{0, 0}, observation, StateTime{2, 0}));
  ExpectAgeViolation(
      EvaluateStateAgeRule(StateTime{0, 1}, observation, StateTime{2, 0}),
      "state age exceeds max_age");
}

TEST(StateAgeRuleTest, FutureObservationIsViolated) {
  ExpectAgeViolation(
      EvaluateStateAgeRule(StateTime{10, 0}, StateTime{10, 1}, StateTime{2, 0}),
      "state age input is not usable");
  ExpectAgeViolation(EvaluateStateAgeRule(StateTime{10, kNanosPerSecond - 1},
                                          StateTime{11, 0}, StateTime{5, 0}),
                     "state age input is not usable");
}

TEST(StateAgeRuleTest, InvalidStampNanosFailClosed) {
  const StateTime valid{10, 0};
  const StateTime max_age{2, 0};
  ExpectAgeViolation(EvaluateStateAgeRule(StateTime{10, -1}, valid, max_age),
                     "state age input is not usable");
  ExpectAgeViolation(
      EvaluateStateAgeRule(StateTime{10, kNanosPerSecond}, valid, max_age),
      "state age input is not usable");
  ExpectAgeViolation(EvaluateStateAgeRule(valid, StateTime{10, -1}, max_age),
                     "state age input is not usable");
  ExpectAgeViolation(
      EvaluateStateAgeRule(valid, StateTime{10, kNanosPerSecond}, max_age),
      "state age input is not usable");
}

TEST(StateAgeRuleTest, NegativeMaxAgeFailsClosed) {
  const StateTime stamp{10, 0};
  ExpectAgeViolation(EvaluateStateAgeRule(stamp, stamp, StateTime{-1, 0}),
                     "state age input is not usable");
  ExpectAgeViolation(EvaluateStateAgeRule(stamp, stamp, StateTime{0, -1}),
                     "state age input is not usable");
  ExpectAgeViolation(EvaluateStateAgeRule(stamp, stamp, StateTime{-1, -1}),
                     "state age input is not usable");
}

TEST(StateAgeRuleTest, MaxAgeNanosOutOfRangeFailsClosed) {
  const StateTime stamp{10, 0};
  ExpectAgeViolation(
      EvaluateStateAgeRule(stamp, stamp, StateTime{0, kNanosPerSecond}),
      "state age input is not usable");
}

TEST(StateAgeRuleTest, AgeThatDoesNotFitInInt64FailsClosed) {
  const StateTime observation{std::numeric_limits<int64_t>::min(), 0};
  const StateTime query{std::numeric_limits<int64_t>::max(), 0};
  ExpectAgeViolation(EvaluateStateAgeRule(query, observation, StateTime{2, 0}),
                     "state age exceeds max_age");
}

TEST(StateAgeRuleTest, Int64BoundaryEqualityIsCompliant) {
  const StateTime observation{std::numeric_limits<int64_t>::min() + 1, 0};
  const StateTime query{0, 0};
  const StateTime max_age{std::numeric_limits<int64_t>::max(), 0};
  ExpectCompliant(EvaluateStateAgeRule(query, observation, max_age));
  ExpectAgeViolation(
      EvaluateStateAgeRule(StateTime{0, 1}, observation, max_age),
      "state age exceeds max_age");
}

}  // namespace
}  // namespace intrinsic::safety
