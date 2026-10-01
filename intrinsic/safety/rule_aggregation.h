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

#ifndef INTRINSIC_SAFETY_RULE_AGGREGATION_H_
#define INTRINSIC_SAFETY_RULE_AGGREGATION_H_

#include <cstddef>
#include <span>
#include <string_view>

#include "intrinsic/safety/safety_rule_result.h"

namespace intrinsic::safety {

// Pure aggregation of already-evaluated SafetyRuleResults. This is not a
// SafetyDecision, builds no applied intent or digest, and invents no rule ids,
// thresholds, or clamp math.

inline constexpr std::string_view kConflictingProjectionsSummary =
    "conflicting projections";

struct AggregatedSafetyResult {
  // False means the aggregate is compliant (allow / pass-through). The
  // aggregator never sets recommended_kind to ACCEPT.
  bool violated = false;
  // Empty when !violated.
  std::string_view primary_rule_id;
  // Max severity among violated inputs. 0 when !violated.
  int severity = 0;
  // Empty when !violated.
  std::string_view summary;
  // UNSPECIFIED (0) when !violated, else PROJECT (2) or REJECT (3) only.
  int recommended_kind = 0;
  bool has_projected_value = false;
  double projected_value = 0.0;
  // Number of input results with violated == true.
  size_t violated_count = 0;
  // True only when the multi-PROJECT escalation to REJECT was applied.
  bool projection_conflict = false;
};

// Aggregates `results` without mutating them. Compliant inputs are ignored.
// The violated subset is sorted by rule_id ascending (byte order), so input
// order never changes the outcome. Then:
//  - severity is the maximum severity of the violated subset.
//  - The primary is the first finding in sorted order with that severity; its
//    rule_id and summary are used unless a projection conflict overrides the
//    summary.
//  - Any violated finding with recommended_kind REJECT, or with a kind other
//    than PROJECT or REJECT (fail closed), makes the aggregate REJECT. Only the
//    primary's has_projected_value / projected_value are copied (audit only).
//  - Otherwise two or more PROJECT findings make the aggregate REJECT with
//    projection_conflict, no projected value, and summary
//    "conflicting projections".
//  - Otherwise exactly one PROJECT finding makes the aggregate PROJECT with
//    that finding's projected value.
// Audit-only projected values on REJECT findings never count as projections.
//
// `primary_rule_id` and `summary` are views of the inputs' views (or of a
// static literal), so they stay valid as long as the inputs' views do.
AggregatedSafetyResult
AggregateSafetyRuleResults(std::span<const SafetyRuleResult> results);

} // namespace intrinsic::safety

#endif // INTRINSIC_SAFETY_RULE_AGGREGATION_H_
