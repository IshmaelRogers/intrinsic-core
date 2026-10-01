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

#ifndef INTRINSIC_SAFETY_SPEED_MAX_RULE_H_
#define INTRINSIC_SAFETY_SPEED_MAX_RULE_H_

#include <string_view>

#include "intrinsic/safety/safety_rule_result.h"

namespace intrinsic::safety {

// Pure linear-speed gate. `speed` is the Euclidean magnitude of the body linear
// velocity, meters/second, >= 0. This function does not read World, ICON, a
// HAL, or Gazebo and does not build a DesiredMotion. It only clamps a scalar.

inline constexpr std::string_view kSpeedMaxRuleId = "speed.linear.max";

// Fixture default in meters/second.
inline constexpr double kDefaultMaxLinearSpeed = 1.5;

// `speed.linear.max`.
//  - Non-finite `speed` or `max_linear_speed`, `speed < 0`, or
//    `max_linear_speed < 0`: CRITICAL, REJECT, no projection.
//  - `speed <= max_linear_speed` (equality is compliant): compliant.
//  - `speed > max_linear_speed`: ERROR, PROJECT,
//    `projected_value == max_linear_speed`.
SafetyRuleResult EvaluateSpeedMaxRule(
    double speed, double max_linear_speed = kDefaultMaxLinearSpeed);

}  // namespace intrinsic::safety

#endif  // INTRINSIC_SAFETY_SPEED_MAX_RULE_H_
