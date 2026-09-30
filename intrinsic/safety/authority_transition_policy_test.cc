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

#include "intrinsic/safety/authority_transition_policy.h"

#include <array>
#include <vector>

#include "gtest/gtest.h"
#include "intrinsic/safety/safety_decision_policy.h"

namespace intrinsic::safety {
namespace {

using M = AuthorityMode;
using E = AuthorityEvent;
using K = SafetyDecisionKind;

constexpr M kUnspec = M::kUnspecified;

constexpr std::array<M, 5> kLiveModes = {
    M::kShadow, M::kRecommend, M::kConstrained, M::kRevoked, M::kEmergency};

constexpr std::array<E, 7> kLiveEvents = {
    E::kArmRecommend,   E::kEnableConstrained, E::kRevoke, E::kEnterEmergency,
    E::kClearEmergency, E::kResetToShadow,     E::kFault};

// Expected target per [from][event], in kLiveModes x kLiveEvents order.
// kUnspec marks a forbidden cell.
//
//                ARM        ENABLE     REVOKE     EMERG      CLEAR      RESET
//                FAULT
constexpr M kExpected[5][7] = {
    /* SHADOW      */ {M::kRecommend, kUnspec, kUnspec, M::kEmergency, kUnspec,
                       kUnspec, M::kRevoked},
    /* RECOMMEND   */
    {kUnspec, M::kConstrained, M::kRevoked, M::kEmergency, kUnspec, M::kShadow,
     M::kRevoked},
    /* CONSTRAINED */
    {kUnspec, kUnspec, M::kRevoked, M::kEmergency, kUnspec, M::kShadow,
     M::kRevoked},
    /* REVOKED     */
    {kUnspec, kUnspec, kUnspec, M::kEmergency, kUnspec, M::kShadow, kUnspec},
    /* EMERGENCY   */
    {kUnspec, kUnspec, kUnspec, kUnspec, M::kRevoked, kUnspec, kUnspec},
};

TEST(AuthorityTransitionPolicyTest, InitialModeIsShadow) {
  EXPECT_EQ(InitialAuthorityMode(), M::kShadow);
  EXPECT_FALSE(
      DecisionAllowedUnderAuthority(InitialAuthorityMode(), K::kAccept));
  EXPECT_FALSE(
      DecisionAllowedUnderAuthority(InitialAuthorityMode(), K::kProject));
}

TEST(AuthorityTransitionPolicyTest, ClassifiesWireNumbers) {
  EXPECT_EQ(ClassifyAuthorityMode(0), M::kUnspecified);
  EXPECT_EQ(ClassifyAuthorityMode(1), M::kShadow);
  EXPECT_EQ(ClassifyAuthorityMode(2), M::kRecommend);
  EXPECT_EQ(ClassifyAuthorityMode(3), M::kConstrained);
  EXPECT_EQ(ClassifyAuthorityMode(4), M::kRevoked);
  EXPECT_EQ(ClassifyAuthorityMode(5), M::kEmergency);
  EXPECT_EQ(ClassifyAuthorityMode(6), M::kUnknown);
  EXPECT_EQ(ClassifyAuthorityMode(-1), M::kUnknown);
  EXPECT_EQ(ClassifyAuthorityMode(1000), M::kUnknown);

  EXPECT_EQ(ClassifyAuthorityEvent(0), E::kUnspecified);
  EXPECT_EQ(ClassifyAuthorityEvent(1), E::kArmRecommend);
  EXPECT_EQ(ClassifyAuthorityEvent(2), E::kEnableConstrained);
  EXPECT_EQ(ClassifyAuthorityEvent(3), E::kRevoke);
  EXPECT_EQ(ClassifyAuthorityEvent(4), E::kEnterEmergency);
  EXPECT_EQ(ClassifyAuthorityEvent(5), E::kClearEmergency);
  EXPECT_EQ(ClassifyAuthorityEvent(6), E::kResetToShadow);
  EXPECT_EQ(ClassifyAuthorityEvent(7), E::kFault);
  EXPECT_EQ(ClassifyAuthorityEvent(8), E::kUnknown);
  EXPECT_EQ(ClassifyAuthorityEvent(-1), E::kUnknown);
  EXPECT_EQ(ClassifyAuthorityEvent(1000), E::kUnknown);
}

TEST(AuthorityTransitionPolicyTest, LiveModeAndEventPredicates) {
  for (M mode : kLiveModes) {
    EXPECT_TRUE(IsLiveAuthorityMode(mode));
  }
  EXPECT_FALSE(IsLiveAuthorityMode(M::kUnspecified));
  EXPECT_FALSE(IsLiveAuthorityMode(M::kUnknown));
  for (E event : kLiveEvents) {
    EXPECT_TRUE(IsLiveAuthorityEvent(event));
  }
  EXPECT_FALSE(IsLiveAuthorityEvent(E::kUnspecified));
  EXPECT_FALSE(IsLiveAuthorityEvent(E::kUnknown));
}

TEST(AuthorityTransitionPolicyTest, ExhaustiveFromByEventMatrix) {
  int allowed = 0;
  int forbidden = 0;
  for (size_t from = 0; from < kLiveModes.size(); ++from) {
    for (size_t event = 0; event < kLiveEvents.size(); ++event) {
      const M current = kLiveModes[from];
      const E ev = kLiveEvents[event];
      const M expected = kExpected[from][event];
      const AuthorityTransitionResult result =
          ApplyAuthorityTransition(current, ev);
      SCOPED_TRACE(::testing::Message() << "from=" << static_cast<int>(current)
                                        << " event=" << static_cast<int>(ev));
      if (expected == kUnspec) {
        ++forbidden;
        EXPECT_FALSE(result.ok);
        EXPECT_EQ(result.next_mode, current);
        EXPECT_EQ(result.error, AuthorityTransitionError::kForbidden);
      } else {
        ++allowed;
        EXPECT_TRUE(result.ok);
        EXPECT_EQ(result.next_mode, expected);
        EXPECT_EQ(result.error, AuthorityTransitionError::kNone);
      }
    }
  }
  EXPECT_EQ(allowed, 15);
  EXPECT_EQ(forbidden, 20);
}

TEST(AuthorityTransitionPolicyTest, ResetFromShadowIsForbiddenNotNoOp) {
  const AuthorityTransitionResult result =
      ApplyAuthorityTransition(M::kShadow, E::kResetToShadow);
  EXPECT_FALSE(result.ok);
  EXPECT_EQ(result.next_mode, M::kShadow);
  EXPECT_EQ(result.error, AuthorityTransitionError::kForbidden);
}

TEST(AuthorityTransitionPolicyTest, EmergencyClearsOnlyToRevoked) {
  for (E event : kLiveEvents) {
    const AuthorityTransitionResult result =
        ApplyAuthorityTransition(M::kEmergency, event);
    if (event == E::kClearEmergency) {
      EXPECT_TRUE(result.ok);
      EXPECT_EQ(result.next_mode, M::kRevoked);
    } else {
      EXPECT_FALSE(result.ok);
      EXPECT_EQ(result.next_mode, M::kEmergency);
    }
  }
}

TEST(AuthorityTransitionPolicyTest, NoEventReturnsToAuthorityFromRevoked) {
  for (E event : kLiveEvents) {
    const AuthorityTransitionResult result =
        ApplyAuthorityTransition(M::kRevoked, event);
    if (result.ok) {
      EXPECT_TRUE(result.next_mode == M::kShadow ||
                  result.next_mode == M::kEmergency);
    }
  }
}

TEST(AuthorityTransitionPolicyTest, InvalidCurrentFailsClosedForEveryEvent) {
  const std::vector<M> invalid_modes = {M::kUnspecified, M::kUnknown};
  const std::vector<E> all_events = {
      E::kUnspecified,   E::kArmRecommend,   E::kEnableConstrained,
      E::kRevoke,        E::kEnterEmergency, E::kClearEmergency,
      E::kResetToShadow, E::kFault,          E::kUnknown};
  for (M mode : invalid_modes) {
    for (E event : all_events) {
      const AuthorityTransitionResult result =
          ApplyAuthorityTransition(mode, event);
      EXPECT_FALSE(result.ok);
      EXPECT_EQ(result.next_mode, M::kUnspecified);
      EXPECT_EQ(result.error, AuthorityTransitionError::kInvalidMode);
    }
  }
}

TEST(AuthorityTransitionPolicyTest, InvalidEventFailsClosedForEveryMode) {
  for (M mode : kLiveModes) {
    for (E event : {E::kUnspecified, E::kUnknown}) {
      const AuthorityTransitionResult result =
          ApplyAuthorityTransition(mode, event);
      EXPECT_FALSE(result.ok);
      EXPECT_EQ(result.next_mode, mode);
      EXPECT_EQ(result.error, AuthorityTransitionError::kInvalidEvent);
    }
  }
}

TEST(AuthorityTransitionPolicyTest, UnknownWireNumbersFailClosed) {
  for (int wire : {6, 7, 99, -1}) {
    const AuthorityTransitionResult bad_mode =
        ApplyAuthorityTransition(ClassifyAuthorityMode(wire), E::kArmRecommend);
    EXPECT_FALSE(bad_mode.ok);
    EXPECT_EQ(bad_mode.next_mode, M::kUnspecified);
  }
  for (int wire : {8, 9, 99, -1}) {
    const AuthorityTransitionResult bad_event =
        ApplyAuthorityTransition(M::kShadow, ClassifyAuthorityEvent(wire));
    EXPECT_FALSE(bad_event.ok);
    EXPECT_EQ(bad_event.next_mode, M::kShadow);
  }
}

TEST(AuthorityTransitionPolicyTest, DefaultResultIsNotOk) {
  const AuthorityTransitionResult result;
  EXPECT_FALSE(result.ok);
  EXPECT_EQ(result.next_mode, M::kUnspecified);
  EXPECT_NE(result.error, AuthorityTransitionError::kNone);
}

TEST(AuthorityTransitionPolicyTest, EmergencyToConstrainedHappyPath) {
  M mode = M::kEmergency;
  const std::array<std::pair<E, M>, 7> steps = {{
      {E::kClearEmergency, M::kRevoked},
      {E::kResetToShadow, M::kShadow},
      {E::kArmRecommend, M::kRecommend},
      {E::kEnableConstrained, M::kConstrained},
      {E::kFault, M::kRevoked},
      {E::kResetToShadow, M::kShadow},
      {E::kEnterEmergency, M::kEmergency},
  }};
  for (const auto& [event, expected] : steps) {
    const AuthorityTransitionResult result =
        ApplyAuthorityTransition(mode, event);
    ASSERT_TRUE(result.ok);
    EXPECT_EQ(result.next_mode, expected);
    mode = result.next_mode;
  }
}

TEST(AuthorityTransitionPolicyTest, EmergencyClearRevokeResetArmEnablePath) {
  M mode = M::kEmergency;
  for (E event : {E::kClearEmergency, E::kResetToShadow, E::kArmRecommend,
                  E::kEnableConstrained}) {
    const AuthorityTransitionResult result =
        ApplyAuthorityTransition(mode, event);
    ASSERT_TRUE(result.ok);
    mode = result.next_mode;
  }
  EXPECT_EQ(mode, M::kConstrained);
}

TEST(AuthorityTransitionPolicyTest, CannotSkipRearmAfterEmergency) {
  EXPECT_FALSE(
      ApplyAuthorityTransition(M::kEmergency, E::kEnableConstrained).ok);
  EXPECT_FALSE(ApplyAuthorityTransition(M::kEmergency, E::kArmRecommend).ok);
  EXPECT_FALSE(ApplyAuthorityTransition(M::kRevoked, E::kArmRecommend).ok);
  EXPECT_FALSE(ApplyAuthorityTransition(M::kRevoked, E::kEnableConstrained).ok);
  EXPECT_FALSE(ApplyAuthorityTransition(M::kShadow, E::kEnableConstrained).ok);
}

// Rows: SHADOW, RECOMMEND, CONSTRAINED, REVOKED, EMERGENCY.
// Columns: ACCEPT, PROJECT, REJECT, ABORT, SURFACE.
constexpr bool kDecisionAllowed[5][5] = {
    /* SHADOW      */ {false, false, true, true, false},
    /* RECOMMEND   */ {false, false, true, true, false},
    /* CONSTRAINED */ {true, true, true, true, true},
    /* REVOKED     */ {false, false, true, true, true},
    /* EMERGENCY   */ {false, false, true, true, true},
};

TEST(AuthorityTransitionPolicyTest, DecisionAllowedMatrix) {
  const std::array<K, 5> kinds = {K::kAccept, K::kProject, K::kReject,
                                  K::kAbort, K::kSurface};
  for (size_t m = 0; m < kLiveModes.size(); ++m) {
    for (size_t k = 0; k < kinds.size(); ++k) {
      SCOPED_TRACE(::testing::Message() << "mode=" << m << " kind=" << k);
      EXPECT_EQ(DecisionAllowedUnderAuthority(kLiveModes[m], kinds[k]),
                kDecisionAllowed[m][k]);
    }
  }
}

TEST(AuthorityTransitionPolicyTest, DecisionAllowedUnknownIsFalse) {
  for (M mode : {M::kUnspecified, M::kUnknown}) {
    for (K kind : {K::kAccept, K::kProject, K::kReject, K::kAbort, K::kSurface,
                   K::kUnspecified, K::kUnknown}) {
      EXPECT_FALSE(DecisionAllowedUnderAuthority(mode, kind));
    }
  }
  for (M mode : kLiveModes) {
    EXPECT_FALSE(DecisionAllowedUnderAuthority(mode, K::kUnspecified));
    EXPECT_FALSE(DecisionAllowedUnderAuthority(mode, K::kUnknown));
  }
}

TEST(AuthorityTransitionPolicyTest, ConstexprEvaluation) {
  static_assert(InitialAuthorityMode() == M::kShadow);
  static_assert(
      ApplyAuthorityTransition(M::kShadow, E::kArmRecommend).next_mode ==
      M::kRecommend);
  static_assert(!ApplyAuthorityTransition(M::kShadow, E::kUnknown).ok);
  static_assert(DecisionAllowedUnderAuthority(M::kConstrained, K::kProject));
  static_assert(!DecisionAllowedUnderAuthority(M::kShadow, K::kAccept));
}

}  // namespace
}  // namespace intrinsic::safety
