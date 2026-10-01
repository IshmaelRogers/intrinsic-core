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

#include "intrinsic/safety/angular_rate_max_rule.h"

#include <string_view>

#include "intrinsic/embodiment/frame_policy.h"

namespace intrinsic::safety {
namespace {

constexpr std::string_view kUnusableSummary =
    "angular rate input is not usable";
constexpr std::string_view kExceededSummary =
    "angular rate exceeds max_angular_rate";

}  // namespace

SafetyRuleResult EvaluateAngularRateMaxRule(double angular_rate_mag,
                                            double max_angular_rate) {
  if (!embodiment::IsFinite(angular_rate_mag) ||
      !embodiment::IsFinite(max_angular_rate) || angular_rate_mag < 0.0 ||
      max_angular_rate < 0.0) {
    return SafetyRuleResult{true, kAngularRateMaxRuleId, kSeverityCritical,
                            kUnusableSummary, kDecisionKindReject};
  }
  if (angular_rate_mag <= max_angular_rate) {
    return SafetyRuleResult{};
  }
  return SafetyRuleResult{true,
                          kAngularRateMaxRuleId,
                          kSeverityError,
                          kExceededSummary,
                          kDecisionKindProject,
                          /*has_projected_value=*/true,
                          /*projected_value=*/max_angular_rate};
}

}  // namespace intrinsic::safety
