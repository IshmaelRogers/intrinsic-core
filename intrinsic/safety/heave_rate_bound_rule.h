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

#ifndef INTRINSIC_SAFETY_HEAVE_RATE_BOUND_RULE_H_
#define INTRINSIC_SAFETY_HEAVE_RATE_BOUND_RULE_H_

#include <string_view>

#include "intrinsic/safety/safety_rule_result.h"

namespace intrinsic::safety {

// Pure heave (depth-rate) gate. `depth_rate` is meters/second with depth
// positive down, so a positive rate is descent and a negative rate is ascent.
// This function does not read World, ICON, a HAL, or Gazebo and does not build
// a DesiredMotion. It only clamps a scalar.

inline constexpr std::string_view kDescentRateMaxRuleId = "rate.descent.max";
inline constexpr std::string_view kAscentRateMaxRuleId = "rate.ascent.max";

// Fixture defaults in meters/second. Both are magnitudes.
inline constexpr double kDefaultMaxDescentRate = 0.5;
inline constexpr double kDefaultMaxAscentRate = 0.5;

// Returns one result per call: the first applicable case below.
//  1. Non-finite `depth_rate`, `max_descent_rate`, or `max_ascent_rate`, or
//     either max `< 0`: CRITICAL, REJECT, no projection. `rule_id` is
//     `rate.ascent.max` only when the ascent max is the sole unusable
//     input. Otherwise it is `rate.descent.max`.
//  2. `depth_rate > max_descent_rate`: ERROR, PROJECT, `rate.descent.max`,
//     `projected_value == max_descent_rate`.
//  3. `depth_rate < -max_ascent_rate`: ERROR, PROJECT, `rate.ascent.max`,
//     `projected_value == -max_ascent_rate`.
//  4. Otherwise compliant (equality with either limit is compliant).
SafetyRuleResult EvaluateHeaveRateBoundRule(
    double depth_rate, double max_descent_rate = kDefaultMaxDescentRate,
    double max_ascent_rate = kDefaultMaxAscentRate);

}  // namespace intrinsic::safety

#endif  // INTRINSIC_SAFETY_HEAVE_RATE_BOUND_RULE_H_
