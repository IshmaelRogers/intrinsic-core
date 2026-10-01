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

#ifndef INTRINSIC_SAFETY_NON_FINITE_RULE_H_
#define INTRINSIC_SAFETY_NON_FINITE_RULE_H_

#include <span>
#include <string_view>

#include "intrinsic/safety/safety_rule_result.h"
#include "intrinsic/vehicle/vehicle_contract_policy.h"

namespace intrinsic::safety {

// Pure non-finite gate. Finite checks reuse embodiment::IsFinite. This
// function does not read World, ICON, a HAL, or a vehicle assessor.

inline constexpr std::string_view kStateNonFiniteRuleId = "state.non_finite";

// Empty `values` is compliant. Any NaN or ±Inf is `state.non_finite` at
// CRITICAL, recommending REJECT.
SafetyRuleResult EvaluateNonFiniteRule(std::span<const double> values);

// Flattens linear then angular components into EvaluateNonFiniteRule.
SafetyRuleResult EvaluateNonFiniteRule(vehicle::BodyVector value);

// Flattens position, orientation, twist, and confidence.
SafetyRuleResult EvaluateNonFiniteRule(
    const vehicle::DesiredMotionView& motion);

// Flattens position, orientation, twist, acceleration, and both covariances.
SafetyRuleResult EvaluateNonFiniteRule(const vehicle::VehicleStateView& state);

}  // namespace intrinsic::safety

#endif  // INTRINSIC_SAFETY_NON_FINITE_RULE_H_
