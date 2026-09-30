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

#include "intrinsic/safety/authority_transition_policy.h"
#include "intrinsic/safety/proto/authority_mode.pb.h"
#include "intrinsic/safety/proto/safety_decision.pb.h"
#include "intrinsic/safety/safety_decision_policy.h"

namespace intrinsic::safety {

namespace pb = ::intrinsic_proto::safety;

AuthorityTransitionProtoResult AssessAuthorityTransition(
    pb::AuthorityMode current, pb::AuthorityEvent event) {
  const AuthorityTransitionResult result =
      ApplyAuthorityTransition(ClassifyAuthorityMode(static_cast<int>(current)),
                               ClassifyAuthorityEvent(static_cast<int>(event)));
  AuthorityTransitionProtoResult out;
  out.ok = result.ok;
  out.error = result.error;
  // The policy enum is numbered like the wire enum for every live mode, and
  // next_mode is never kUnknown, so this cast never rewrites a wire number.
  out.next_mode = static_cast<pb::AuthorityMode>(result.next_mode);
  return out;
}

bool AssessDecisionKindUnderAuthority(pb::AuthorityMode mode,
                                      pb::SafetyDecisionKind kind) {
  return DecisionAllowedUnderAuthority(
      ClassifyAuthorityMode(static_cast<int>(mode)),
      ClassifySafetyDecisionKind(static_cast<int>(kind)));
}

}  // namespace intrinsic::safety
