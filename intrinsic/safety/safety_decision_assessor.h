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

#ifndef INTRINSIC_SAFETY_SAFETY_DECISION_ASSESSOR_H_
#define INTRINSIC_SAFETY_SAFETY_DECISION_ASSESSOR_H_

#include <string>
#include <vector>

#include "intrinsic/safety/proto/safety_decision.pb.h"
#include "intrinsic/safety/safety_decision_policy.h"
#include "intrinsic/vehicle/proto/vehicle_command.pb.h"
#include "intrinsic/vehicle/vehicle_contract_policy.h"

namespace intrinsic::safety {

// Policy: intrinsic_apis/intrinsic/safety/proto/README.md.

// Maps a DesiredMotion onto the plain view used by AssessDesiredMotion. String
// views point into `motion`, which must outlive the view.
vehicle::DesiredMotionView DesiredMotionViewFromProto(
    const intrinsic_proto::vehicle::DesiredMotion& motion);

// Maps a decision onto the plain view. Unknown enum numbers are copied as
// raw ints. `findings_storage` backs the view's findings span and, like
// `decision`, must outlive the view.
SafetyDecisionView SafetyDecisionViewFromProto(
    const intrinsic_proto::safety::SafetyDecision& decision,
    std::vector<SafetyFindingView>* findings_storage);

// Structural assessment of a decision message. Reads the message once and
// does not modify it. An empty message is not engaged, not an error, and not
// accepted. An unknown kind is never accepted and is not rewritten.
SafetyDecisionAssessment AssessSafetyDecisionProto(
    const intrinsic_proto::safety::SafetyDecision& decision);

// Lowercase hex SHA-256 (64 characters) of the deterministic serialization of
// `motion`. This is the value a filter records in original_intent_digest.
std::string ComputeOriginalIntentDigest(
    const intrinsic_proto::vehicle::DesiredMotion& motion);

}  // namespace intrinsic::safety

#endif  // INTRINSIC_SAFETY_SAFETY_DECISION_ASSESSOR_H_
