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

#include "intrinsic/safety/authority_transition_assessor.h"

#include <array>
#include <cstddef>

#include "gtest/gtest.h"
#include "intrinsic/safety/authority_transition_policy.h"
#include "intrinsic/safety/proto/authority_mode.pb.h"
#include "intrinsic/safety/proto/safety_decision.pb.h"

namespace intrinsic::safety {
namespace {

namespace pb = ::intrinsic_proto::safety;

constexpr std::array<pb::AuthorityMode, 5> kModes = {
    intrinsic_proto::safety::AUTHORITY_MODE_SHADOW,
    intrinsic_proto::safety::AUTHORITY_MODE_RECOMMEND,
    intrinsic_proto::safety::AUTHORITY_MODE_CONSTRAINED,
    intrinsic_proto::safety::AUTHORITY_MODE_REVOKED,
    intrinsic_proto::safety::AUTHORITY_MODE_EMERGENCY};

constexpr std::array<pb::AuthorityEvent, 7> kEvents = {
    intrinsic_proto::safety::AUTHORITY_EVENT_ARM_RECOMMEND,
    intrinsic_proto::safety::AUTHORITY_EVENT_ENABLE_CONSTRAINED,
    intrinsic_proto::safety::AUTHORITY_EVENT_REVOKE,
    intrinsic_proto::safety::AUTHORITY_EVENT_ENTER_EMERGENCY,
    intrinsic_proto::safety::AUTHORITY_EVENT_CLEAR_EMERGENCY,
    intrinsic_proto::safety::AUTHORITY_EVENT_RESET_TO_SHADOW,
    intrinsic_proto::safety::AUTHORITY_EVENT_FAULT};

TEST(AuthorityTransitionAssessorTest, LockedEnumNumbers) {
  EXPECT_EQ(intrinsic_proto::safety::AUTHORITY_MODE_UNSPECIFIED, 0);
  EXPECT_EQ(intrinsic_proto::safety::AUTHORITY_MODE_SHADOW, 1);
  EXPECT_EQ(intrinsic_proto::safety::AUTHORITY_MODE_RECOMMEND, 2);
  EXPECT_EQ(intrinsic_proto::safety::AUTHORITY_MODE_CONSTRAINED, 3);
  EXPECT_EQ(intrinsic_proto::safety::AUTHORITY_MODE_REVOKED, 4);
  EXPECT_EQ(intrinsic_proto::safety::AUTHORITY_MODE_EMERGENCY, 5);
  EXPECT_EQ(intrinsic_proto::safety::AUTHORITY_EVENT_UNSPECIFIED, 0);
  EXPECT_EQ(intrinsic_proto::safety::AUTHORITY_EVENT_ARM_RECOMMEND, 1);
  EXPECT_EQ(intrinsic_proto::safety::AUTHORITY_EVENT_ENABLE_CONSTRAINED, 2);
  EXPECT_EQ(intrinsic_proto::safety::AUTHORITY_EVENT_REVOKE, 3);
  EXPECT_EQ(intrinsic_proto::safety::AUTHORITY_EVENT_ENTER_EMERGENCY, 4);
  EXPECT_EQ(intrinsic_proto::safety::AUTHORITY_EVENT_CLEAR_EMERGENCY, 5);
  EXPECT_EQ(intrinsic_proto::safety::AUTHORITY_EVENT_RESET_TO_SHADOW, 6);
  EXPECT_EQ(intrinsic_proto::safety::AUTHORITY_EVENT_FAULT, 7);
}

TEST(AuthorityTransitionAssessorTest, MatchesPolicyForEveryLiveCell) {
  for (pb::AuthorityMode mode : kModes) {
    for (pb::AuthorityEvent event : kEvents) {
      const AuthorityTransitionResult expected = ApplyAuthorityTransition(
          ClassifyAuthorityMode(static_cast<int>(mode)),
          ClassifyAuthorityEvent(static_cast<int>(event)));
      const AuthorityTransitionProtoResult actual =
          AssessAuthorityTransition(mode, event);
      EXPECT_EQ(actual.ok, expected.ok);
      EXPECT_EQ(actual.error, expected.error);
      EXPECT_EQ(static_cast<int>(actual.next_mode),
                static_cast<int>(expected.next_mode));
    }
  }
}

TEST(AuthorityTransitionAssessorTest, ArmFromShadow) {
  const AuthorityTransitionProtoResult result = AssessAuthorityTransition(
      intrinsic_proto::safety::AUTHORITY_MODE_SHADOW,
      intrinsic_proto::safety::AUTHORITY_EVENT_ARM_RECOMMEND);
  EXPECT_TRUE(result.ok);
  EXPECT_EQ(result.next_mode,
            intrinsic_proto::safety::AUTHORITY_MODE_RECOMMEND);
}

TEST(AuthorityTransitionAssessorTest, UnknownWireNumbersFailClosed) {
  const AuthorityTransitionProtoResult bad_mode = AssessAuthorityTransition(
      static_cast<pb::AuthorityMode>(99),
      intrinsic_proto::safety::AUTHORITY_EVENT_ARM_RECOMMEND);
  EXPECT_FALSE(bad_mode.ok);
  EXPECT_EQ(bad_mode.next_mode,
            intrinsic_proto::safety::AUTHORITY_MODE_UNSPECIFIED);
  EXPECT_EQ(bad_mode.error, AuthorityTransitionError::kInvalidMode);

  const AuthorityTransitionProtoResult bad_event =
      AssessAuthorityTransition(intrinsic_proto::safety::AUTHORITY_MODE_SHADOW,
                                static_cast<pb::AuthorityEvent>(99));
  EXPECT_FALSE(bad_event.ok);
  EXPECT_EQ(bad_event.next_mode,
            intrinsic_proto::safety::AUTHORITY_MODE_SHADOW);
  EXPECT_EQ(bad_event.error, AuthorityTransitionError::kInvalidEvent);

  const AuthorityTransitionProtoResult unspecified = AssessAuthorityTransition(
      intrinsic_proto::safety::AUTHORITY_MODE_UNSPECIFIED,
      intrinsic_proto::safety::AUTHORITY_EVENT_UNSPECIFIED);
  EXPECT_FALSE(unspecified.ok);
}

TEST(AuthorityTransitionAssessorTest, DecisionKindMatrix) {
  const std::array<pb::SafetyDecisionKind, 5> kinds = {
      intrinsic_proto::safety::SAFETY_DECISION_KIND_ACCEPT,
      intrinsic_proto::safety::SAFETY_DECISION_KIND_PROJECT,
      intrinsic_proto::safety::SAFETY_DECISION_KIND_REJECT,
      intrinsic_proto::safety::SAFETY_DECISION_KIND_ABORT,
      intrinsic_proto::safety::SAFETY_DECISION_KIND_SURFACE};
  constexpr bool kAllowed[5][5] = {
      {false, false, true, true, false}, {false, false, true, true, false},
      {true, true, true, true, true},    {false, false, true, true, true},
      {false, false, true, true, true},
  };
  for (size_t m = 0; m < kModes.size(); ++m) {
    for (size_t k = 0; k < kinds.size(); ++k) {
      EXPECT_EQ(AssessDecisionKindUnderAuthority(kModes[m], kinds[k]),
                kAllowed[m][k]);
    }
  }
}

TEST(AuthorityTransitionAssessorTest, DecisionKindUnknownIsFalse) {
  EXPECT_FALSE(AssessDecisionKindUnderAuthority(
      static_cast<pb::AuthorityMode>(99),
      intrinsic_proto::safety::SAFETY_DECISION_KIND_REJECT));
  EXPECT_FALSE(AssessDecisionKindUnderAuthority(
      intrinsic_proto::safety::AUTHORITY_MODE_CONSTRAINED,
      static_cast<pb::SafetyDecisionKind>(99)));
  EXPECT_FALSE(AssessDecisionKindUnderAuthority(
      intrinsic_proto::safety::AUTHORITY_MODE_CONSTRAINED,
      intrinsic_proto::safety::SAFETY_DECISION_KIND_UNSPECIFIED));
  EXPECT_FALSE(AssessDecisionKindUnderAuthority(
      intrinsic_proto::safety::AUTHORITY_MODE_UNSPECIFIED,
      intrinsic_proto::safety::SAFETY_DECISION_KIND_REJECT));
}

TEST(AuthorityTransitionAssessorTest, UnknownWireNumbersSurviveSerialization) {
  intrinsic_proto::safety::SafetyDecision decision;
  decision.set_kind(static_cast<pb::SafetyDecisionKind>(77));
  const pb::AuthorityMode unknown_mode = static_cast<pb::AuthorityMode>(88);
  EXPECT_FALSE(AssessDecisionKindUnderAuthority(unknown_mode, decision.kind()));
  EXPECT_EQ(static_cast<int>(decision.kind()), 77);
  EXPECT_EQ(static_cast<int>(unknown_mode), 88);
}

}  // namespace
}  // namespace intrinsic::safety
