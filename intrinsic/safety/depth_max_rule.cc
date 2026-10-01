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

#include "intrinsic/safety/depth_max_rule.h"

#include <string_view>

#include "intrinsic/embodiment/frame_policy.h"

namespace intrinsic::safety {
namespace {

constexpr std::string_view kUnusableSummary = "depth input is not usable";
constexpr std::string_view kExceededSummary = "depth exceeds max_depth";

}  // namespace

SafetyRuleResult EvaluateDepthMaxRule(double depth, double max_depth) {
  if (!embodiment::IsFinite(depth) || !embodiment::IsFinite(max_depth) ||
      max_depth < 0.0) {
    return SafetyRuleResult{true, kDepthMaxRuleId, kSeverityCritical,
                            kUnusableSummary, kDecisionKindReject};
  }
  if (depth <= max_depth) {
    return SafetyRuleResult{};
  }
  return SafetyRuleResult{true,
                          kDepthMaxRuleId,
                          kSeverityError,
                          kExceededSummary,
                          kDecisionKindProject,
                          /*has_projected_value=*/true,
                          /*projected_value=*/max_depth};
}

}  // namespace intrinsic::safety
