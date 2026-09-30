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

#ifndef INTRINSIC_SAFETY_AUTHORITY_TRANSITION_POLICY_H_
#define INTRINSIC_SAFETY_AUTHORITY_TRANSITION_POLICY_H_

#include "intrinsic/safety/safety_decision_policy.h"

namespace intrinsic::safety {

// Policy: intrinsic_apis/intrinsic/safety/proto/README.md.
//
// Pure authority transition table. These functions work on plain values. They
// do not parse protobuf, do not evaluate any safety rule, and do not call
// ICON, a HAL, Gazebo, or an actuator API. They never read or write World.

enum class AuthorityMode {
  kUnspecified = 0,
  kShadow = 1,
  kRecommend = 2,
  kConstrained = 3,
  kRevoked = 4,
  kEmergency = 5,
  // Any other wire number. Never rewritten. Never a transition target.
  kUnknown = 6,
};

// Wire values other than 0..5 are kUnknown.
inline constexpr AuthorityMode ClassifyAuthorityMode(int mode) {
  switch (mode) {
    case 0:
      return AuthorityMode::kUnspecified;
    case 1:
      return AuthorityMode::kShadow;
    case 2:
      return AuthorityMode::kRecommend;
    case 3:
      return AuthorityMode::kConstrained;
    case 4:
      return AuthorityMode::kRevoked;
    case 5:
      return AuthorityMode::kEmergency;
    default:
      return AuthorityMode::kUnknown;
  }
}

// True for SHADOW, RECOMMEND, CONSTRAINED, REVOKED, and EMERGENCY.
inline constexpr bool IsLiveAuthorityMode(AuthorityMode mode) {
  switch (mode) {
    case AuthorityMode::kShadow:
    case AuthorityMode::kRecommend:
    case AuthorityMode::kConstrained:
    case AuthorityMode::kRevoked:
    case AuthorityMode::kEmergency:
      return true;
    case AuthorityMode::kUnspecified:
    case AuthorityMode::kUnknown:
      return false;
  }
  return false;
}

enum class AuthorityEvent {
  kUnspecified = 0,
  kArmRecommend = 1,
  kEnableConstrained = 2,
  kRevoke = 3,
  kEnterEmergency = 4,
  kClearEmergency = 5,
  kResetToShadow = 6,
  kFault = 7,
  // Any other wire number. Never causes a transition.
  kUnknown = 8,
};

// Wire values other than 0..7 are kUnknown.
inline constexpr AuthorityEvent ClassifyAuthorityEvent(int event) {
  switch (event) {
    case 0:
      return AuthorityEvent::kUnspecified;
    case 1:
      return AuthorityEvent::kArmRecommend;
    case 2:
      return AuthorityEvent::kEnableConstrained;
    case 3:
      return AuthorityEvent::kRevoke;
    case 4:
      return AuthorityEvent::kEnterEmergency;
    case 5:
      return AuthorityEvent::kClearEmergency;
    case 6:
      return AuthorityEvent::kResetToShadow;
    case 7:
      return AuthorityEvent::kFault;
    default:
      return AuthorityEvent::kUnknown;
  }
}

// True for the seven named events.
inline constexpr bool IsLiveAuthorityEvent(AuthorityEvent event) {
  return event != AuthorityEvent::kUnspecified &&
         event != AuthorityEvent::kUnknown;
}

enum class AuthorityTransitionError {
  kNone = 0,
  // `current` is UNSPECIFIED or unknown. Checked first.
  kInvalidMode = 1,
  // `event` is UNSPECIFIED or unknown.
  kInvalidEvent = 2,
  // Both are valid but the pair is not in the approved table.
  kForbidden = 3,
};

struct AuthorityTransitionResult {
  bool ok = false;
  // The target mode when `ok`. Otherwise `current`, or kUnspecified when
  // `current` was not a live mode.
  AuthorityMode next_mode = AuthorityMode::kUnspecified;
  AuthorityTransitionError error = AuthorityTransitionError::kInvalidMode;
};

// The mode after a restart or cold start. Always non-authoritative SHADOW.
inline constexpr AuthorityMode InitialAuthorityMode() {
  return AuthorityMode::kShadow;
}

namespace internal {

// Approved target for a live `current` and live `event`, or kUnspecified when
// the pair is forbidden. Every pair not listed here is forbidden.
inline constexpr AuthorityMode ApprovedAuthorityTarget(AuthorityMode current,
                                                       AuthorityEvent event) {
  switch (current) {
    case AuthorityMode::kShadow:
      switch (event) {
        case AuthorityEvent::kArmRecommend:
          return AuthorityMode::kRecommend;
        case AuthorityEvent::kEnterEmergency:
          return AuthorityMode::kEmergency;
        case AuthorityEvent::kFault:
          return AuthorityMode::kRevoked;
        default:
          return AuthorityMode::kUnspecified;
      }
    case AuthorityMode::kRecommend:
      switch (event) {
        case AuthorityEvent::kEnableConstrained:
          return AuthorityMode::kConstrained;
        case AuthorityEvent::kRevoke:
          return AuthorityMode::kRevoked;
        case AuthorityEvent::kEnterEmergency:
          return AuthorityMode::kEmergency;
        case AuthorityEvent::kResetToShadow:
          return AuthorityMode::kShadow;
        case AuthorityEvent::kFault:
          return AuthorityMode::kRevoked;
        default:
          return AuthorityMode::kUnspecified;
      }
    case AuthorityMode::kConstrained:
      switch (event) {
        case AuthorityEvent::kRevoke:
          return AuthorityMode::kRevoked;
        case AuthorityEvent::kEnterEmergency:
          return AuthorityMode::kEmergency;
        case AuthorityEvent::kResetToShadow:
          return AuthorityMode::kShadow;
        case AuthorityEvent::kFault:
          return AuthorityMode::kRevoked;
        default:
          return AuthorityMode::kUnspecified;
      }
    case AuthorityMode::kRevoked:
      switch (event) {
        case AuthorityEvent::kEnterEmergency:
          return AuthorityMode::kEmergency;
        case AuthorityEvent::kResetToShadow:
          return AuthorityMode::kShadow;
        default:
          return AuthorityMode::kUnspecified;
      }
    case AuthorityMode::kEmergency:
      switch (event) {
        // Emergency clears only to REVOKED.
        case AuthorityEvent::kClearEmergency:
          return AuthorityMode::kRevoked;
        default:
          return AuthorityMode::kUnspecified;
      }
    case AuthorityMode::kUnspecified:
    case AuthorityMode::kUnknown:
      return AuthorityMode::kUnspecified;
  }
  return AuthorityMode::kUnspecified;
}

}  // namespace internal

// Applies one event to one mode. Fail closed: a forbidden, unknown, or
// unspecified input returns ok == false and never a silent no-op success.
inline constexpr AuthorityTransitionResult ApplyAuthorityTransition(
    AuthorityMode current, AuthorityEvent event) {
  AuthorityTransitionResult result;
  if (!IsLiveAuthorityMode(current)) {
    result.ok = false;
    result.next_mode = AuthorityMode::kUnspecified;
    result.error = AuthorityTransitionError::kInvalidMode;
    return result;
  }
  result.next_mode = current;
  if (!IsLiveAuthorityEvent(event)) {
    result.ok = false;
    result.error = AuthorityTransitionError::kInvalidEvent;
    return result;
  }
  const AuthorityMode target =
      internal::ApprovedAuthorityTarget(current, event);
  if (target == AuthorityMode::kUnspecified) {
    result.ok = false;
    result.error = AuthorityTransitionError::kForbidden;
    return result;
  }
  result.ok = true;
  result.next_mode = target;
  result.error = AuthorityTransitionError::kNone;
  return result;
}

// Policy matrix for which committed SafetyDecision kinds may be emitted under
// an authority mode. A pure table: no rule is evaluated. Unspecified and
// unknown modes or kinds return false.
//
// | Mode        | ACCEPT | PROJECT | REJECT | ABORT | SURFACE |
// | SHADOW      |   no   |   no    |  yes   |  yes  |   no    |
// | RECOMMEND   |   no   |   no    |  yes   |  yes  |   no    |
// | CONSTRAINED |  yes   |  yes    |  yes   |  yes  |  yes    |
// | REVOKED     |   no   |   no    |  yes   |  yes  |  yes    |
// | EMERGENCY   |   no   |   no    |  yes   |  yes  |  yes    |
inline constexpr bool DecisionAllowedUnderAuthority(AuthorityMode mode,
                                                    SafetyDecisionKind kind) {
  if (!IsLiveAuthorityMode(mode) || !IsKnownCommittedKind(kind)) {
    return false;
  }
  switch (kind) {
    case SafetyDecisionKind::kReject:
    case SafetyDecisionKind::kAbort:
      return true;
    case SafetyDecisionKind::kAccept:
    case SafetyDecisionKind::kProject:
      return mode == AuthorityMode::kConstrained;
    case SafetyDecisionKind::kSurface:
      return mode == AuthorityMode::kConstrained ||
             mode == AuthorityMode::kRevoked ||
             mode == AuthorityMode::kEmergency;
    case SafetyDecisionKind::kUnspecified:
    case SafetyDecisionKind::kUnknown:
      return false;
  }
  return false;
}

}  // namespace intrinsic::safety

#endif  // INTRINSIC_SAFETY_AUTHORITY_TRANSITION_POLICY_H_
