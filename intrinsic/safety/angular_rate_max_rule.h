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

#ifndef INTRINSIC_SAFETY_ANGULAR_RATE_MAX_RULE_H_
#define INTRINSIC_SAFETY_ANGULAR_RATE_MAX_RULE_H_

#include <string_view>

#include "intrinsic/safety/safety_rule_result.h"

namespace intrinsic::safety {

// Pure angular-rate-magnitude gate. `angular_rate_mag` is the Euclidean
// magnitude of the body angular velocity, radians/second, >= 0. This function
// does not read World, ICON, a HAL, or Gazebo and does not build a
// DesiredMotion. It only clamps a scalar.

inline constexpr std::string_view kAngularRateMaxRuleId = "rate.angular.max";

// Fixture default in radians/second.
inline constexpr double kDefaultMaxAngularRate = 0.5;

// `rate.angular.max`.
//  - Non-finite `angular_rate_mag` or `max_angular_rate`, `angular_rate_mag <
//  0`, or
//    `max_angular_rate < 0`: CRITICAL, REJECT, no projection.
//  - `angular_rate_mag <= max_angular_rate` (equality is compliant): compliant.
//  - `angular_rate_mag > max_angular_rate`: ERROR, PROJECT,
//    `projected_value == max_angular_rate`.
SafetyRuleResult EvaluateAngularRateMaxRule(
    double angular_rate_mag, double max_angular_rate = kDefaultMaxAngularRate);

}  // namespace intrinsic::safety

#endif  // INTRINSIC_SAFETY_ANGULAR_RATE_MAX_RULE_H_
