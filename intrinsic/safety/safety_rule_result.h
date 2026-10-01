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

#ifndef INTRINSIC_SAFETY_SAFETY_RULE_RESULT_H_
#define INTRINSIC_SAFETY_SAFETY_RULE_RESULT_H_

#include <string_view>

namespace intrinsic::safety {

// Plain-value result of one pure safety rule. This is not a SafetyDecision
// and it does not aggregate findings. `rule_id` and `summary` point at
// static string literals when they are non-empty.

struct SafetyRuleResult {
  // False means the input is compliant and no finding is produced.
  bool violated = false;
  // Empty when !violated. Otherwise the locked rule id.
  std::string_view rule_id;
  // Raw SafetyFindingSeverity wire number. 0 when !violated.
  int severity = 0;
  // Short human text when violated. Empty when !violated.
  std::string_view summary;
  // Raw SafetyDecisionKind wire number. REJECT (3) when violated.
  // UNSPECIFIED (0) when !violated. Never PROJECT, ACCEPT, ABORT, or SURFACE.
  int recommended_kind = 0;
};

// Locked wire numbers from SafetyFindingSeverity and SafetyDecisionKind.
inline constexpr int kSeverityUnspecified = 0;
inline constexpr int kSeverityError = 3;
inline constexpr int kSeverityCritical = 4;
inline constexpr int kDecisionKindUnspecified = 0;
inline constexpr int kDecisionKindReject = 3;

}  // namespace intrinsic::safety

#endif  // INTRINSIC_SAFETY_SAFETY_RULE_RESULT_H_
