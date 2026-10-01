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

#ifndef INTRINSIC_SAFETY_DEPTH_MAX_RULE_H_
#define INTRINSIC_SAFETY_DEPTH_MAX_RULE_H_

#include <string_view>

#include "intrinsic/safety/safety_rule_result.h"

namespace intrinsic::safety {

// Pure maximum-depth gate. Depth is meters positive down from the surface
// (0 at the surface). This function does not read World, ICON, a HAL, or
// Gazebo and does not plan a trajectory. It only clamps a scalar.

inline constexpr std::string_view kDepthMaxRuleId = "depth.max";

// Fixture default in meters.
inline constexpr double kDefaultMaxDepth = 100.0;

// `depth.max`.
//  - Non-finite `depth` or `max_depth`, or `max_depth < 0`: CRITICAL, REJECT,
//    no projection.
//  - `depth <= max_depth` (equality is compliant): compliant.
//  - `depth > max_depth`: ERROR, PROJECT, `projected_value == max_depth`.
SafetyRuleResult EvaluateDepthMaxRule(double depth,
                                      double max_depth = kDefaultMaxDepth);

}  // namespace intrinsic::safety

#endif  // INTRINSIC_SAFETY_DEPTH_MAX_RULE_H_
