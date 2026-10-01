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

#include "intrinsic/safety/heave_rate_bound_rule.h"

#include <string_view>

#include "intrinsic/embodiment/frame_policy.h"

namespace intrinsic::safety {
namespace {

constexpr std::string_view kUnusableSummary = "heave rate input is not usable";
constexpr std::string_view kDescentExceededSummary =
    "descent rate exceeds max_descent_rate";
constexpr std::string_view kAscentExceededSummary =
    "ascent rate exceeds max_ascent_rate";

bool LimitUsable(double max_rate) {
  return embodiment::IsFinite(max_rate) && max_rate >= 0.0;
}

}  // namespace

SafetyRuleResult EvaluateHeaveRateBoundRule(double depth_rate,
                                            double max_descent_rate,
                                            double max_ascent_rate) {
  const bool descent_usable = LimitUsable(max_descent_rate);
  const bool ascent_usable = LimitUsable(max_ascent_rate);
  if (!embodiment::IsFinite(depth_rate) || !descent_usable || !ascent_usable) {
    const std::string_view rule_id = (descent_usable && !ascent_usable)
                                         ? kAscentRateMaxRuleId
                                         : kDescentRateMaxRuleId;
    return SafetyRuleResult{true, rule_id, kSeverityCritical, kUnusableSummary,
                            kDecisionKindReject};
  }
  if (depth_rate > max_descent_rate) {
    return SafetyRuleResult{true,
                            kDescentRateMaxRuleId,
                            kSeverityError,
                            kDescentExceededSummary,
                            kDecisionKindProject,
                            /*has_projected_value=*/true,
                            /*projected_value=*/max_descent_rate};
  }
  if (depth_rate < -max_ascent_rate) {
    return SafetyRuleResult{true,
                            kAscentRateMaxRuleId,
                            kSeverityError,
                            kAscentExceededSummary,
                            kDecisionKindProject,
                            /*has_projected_value=*/true,
                            /*projected_value=*/-max_ascent_rate};
  }
  return SafetyRuleResult{};
}

}  // namespace intrinsic::safety
