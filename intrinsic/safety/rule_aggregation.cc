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
#include <compare>
#include <cstddef>
#include <span>
#include <vector>

#include "intrinsic/safety/safety_rule_result.h"

namespace intrinsic::safety {
namespace {

// Total order so that identical rule_ids still sort deterministically.
bool FindingLess(const SafetyRuleResult &a, const SafetyRuleResult &b) {
  if (a.rule_id != b.rule_id)
    return a.rule_id < b.rule_id;
  if (a.severity != b.severity)
    return a.severity < b.severity;
  if (a.recommended_kind != b.recommended_kind) {
    return a.recommended_kind < b.recommended_kind;
  }
  if (a.summary != b.summary)
    return a.summary < b.summary;
  if (a.has_projected_value != b.has_projected_value) {
    return a.has_projected_value < b.has_projected_value;
  }
  return std::strong_order(a.projected_value, b.projected_value) < 0;
}

} // namespace

AggregatedSafetyResult
AggregateSafetyRuleResults(std::span<const SafetyRuleResult> results) {
  std::vector<SafetyRuleResult> violated;
  for (const SafetyRuleResult &result : results) {
    if (result.violated)
      violated.push_back(result);
  }

  AggregatedSafetyResult aggregate;
  aggregate.violated_count = violated.size();
  if (violated.empty())
    return aggregate;

  std::sort(violated.begin(), violated.end(), FindingLess);

  int max_severity = violated.front().severity;
  for (const SafetyRuleResult &finding : violated) {
    max_severity = std::max(max_severity, finding.severity);
  }
  const SafetyRuleResult *primary = nullptr;
  for (const SafetyRuleResult &finding : violated) {
    if (finding.severity == max_severity) {
      primary = &finding;
      break;
    }
  }

  bool any_reject_or_unknown = false;
  size_t project_count = 0;
  const SafetyRuleResult *project_finding = nullptr;
  for (const SafetyRuleResult &finding : violated) {
    if (finding.recommended_kind == kDecisionKindProject) {
      if (project_count == 0)
        project_finding = &finding;
      ++project_count;
    } else {
      any_reject_or_unknown = true;
    }
  }

  aggregate.violated = true;
  aggregate.primary_rule_id = primary->rule_id;
  aggregate.severity = max_severity;
  aggregate.summary = primary->summary;

  if (any_reject_or_unknown) {
    aggregate.recommended_kind = kDecisionKindReject;
    aggregate.has_projected_value = primary->has_projected_value;
    aggregate.projected_value =
        primary->has_projected_value ? primary->projected_value : 0.0;
  } else if (project_count >= 2) {
    aggregate.recommended_kind = kDecisionKindReject;
    aggregate.projection_conflict = true;
    aggregate.summary = kConflictingProjectionsSummary;
  } else {
    aggregate.recommended_kind = kDecisionKindProject;
    aggregate.has_projected_value = project_finding->has_projected_value;
    aggregate.projected_value = project_finding->has_projected_value
                                    ? project_finding->projected_value
                                    : 0.0;
  }
  return aggregate;
}

} // namespace intrinsic::safety
