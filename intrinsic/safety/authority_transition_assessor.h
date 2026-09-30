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

#ifndef INTRINSIC_SAFETY_AUTHORITY_TRANSITION_ASSESSOR_H_
#define INTRINSIC_SAFETY_AUTHORITY_TRANSITION_ASSESSOR_H_

#include "intrinsic/safety/authority_transition_policy.h"
#include "intrinsic/safety/proto/authority_mode.pb.h"
#include "intrinsic/safety/proto/safety_decision.pb.h"

namespace intrinsic::safety {

// Policy: intrinsic_apis/intrinsic/safety/proto/README.md.

// Result of a transition on wire enums. Same fields and meaning as
// AuthorityTransitionResult. `next_mode` is AUTHORITY_MODE_UNSPECIFIED when
// `current` was not a live mode.
struct AuthorityTransitionProtoResult {
  bool ok = false;
  intrinsic_proto::safety::AuthorityMode next_mode =
      intrinsic_proto::safety::AUTHORITY_MODE_UNSPECIFIED;
  AuthorityTransitionError error = AuthorityTransitionError::kInvalidMode;
};

// Applies ApplyAuthorityTransition to wire enums. Unknown numbers are passed
// through as raw ints, classified as unknown, and fail closed. They are never
// rewritten.
AuthorityTransitionProtoResult AssessAuthorityTransition(
    intrinsic_proto::safety::AuthorityMode current,
    intrinsic_proto::safety::AuthorityEvent event);

// Applies DecisionAllowedUnderAuthority to wire enums. Unknown numbers return
// false.
bool AssessDecisionKindUnderAuthority(
    intrinsic_proto::safety::AuthorityMode mode,
    intrinsic_proto::safety::SafetyDecisionKind kind);

}  // namespace intrinsic::safety

#endif  // INTRINSIC_SAFETY_AUTHORITY_TRANSITION_ASSESSOR_H_
