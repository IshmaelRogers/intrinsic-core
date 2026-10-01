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

#include "intrinsic/safety/altitude_min_rule.h"

#include <string_view>

#include "intrinsic/embodiment/frame_policy.h"

namespace intrinsic::safety {
namespace {

constexpr std::string_view kUnknownSummary = "altitude is unknown";
constexpr std::string_view kUnusableSummary = "altitude input is not usable";
constexpr std::string_view kBelowSummary = "altitude is below min_altitude";

SafetyRuleResult RejectCritical(std::string_view summary) {
  return SafetyRuleResult{true, kAltitudeMinRuleId, kSeverityCritical, summary,
                          kDecisionKindReject};
}

}  // namespace

SafetyRuleResult EvaluateAltitudeMinRule(bool altitude_known, double altitude,
                                         double min_altitude) {
  if (!altitude_known) {
    return RejectCritical(kUnknownSummary);
  }
  if (!embodiment::IsFinite(altitude) || !embodiment::IsFinite(min_altitude) ||
      min_altitude < 0.0) {
    return RejectCritical(kUnusableSummary);
  }
  if (altitude >= min_altitude) {
    return SafetyRuleResult{};
  }
  return SafetyRuleResult{true,
                          kAltitudeMinRuleId,
                          kSeverityError,
                          kBelowSummary,
                          kDecisionKindProject,
                          /*has_projected_value=*/true,
                          /*projected_value=*/min_altitude};
}

}  // namespace intrinsic::safety
