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

#ifndef INTRINSIC_SAFETY_ALTITUDE_MIN_RULE_H_
#define INTRINSIC_SAFETY_ALTITUDE_MIN_RULE_H_

#include <string_view>

#include "intrinsic/safety/safety_rule_result.h"

namespace intrinsic::safety {

// Pure minimum seafloor-altitude gate. Altitude is meters positive up from
// the seafloor (clearance to the bottom). This function does not read World,
// ICON, a HAL, or Gazebo and does not check collisions, geofences, or
// horizontal clearance. It only clamps a scalar.

inline constexpr std::string_view kAltitudeMinRuleId = "altitude.min";

// Fixture default in meters.
inline constexpr double kDefaultMinAltitude = 2.0;

// `altitude.min`.
//  - `!altitude_known`: CRITICAL, REJECT, no projection (fails closed).
//  - Non-finite `altitude` or `min_altitude`, or `min_altitude < 0`:
//    CRITICAL, REJECT, no projection.
//  - `altitude >= min_altitude` (equality is compliant): compliant.
//  - `altitude < min_altitude`: ERROR, PROJECT,
//    `projected_value == min_altitude`.
SafetyRuleResult EvaluateAltitudeMinRule(
    bool altitude_known, double altitude,
    double min_altitude = kDefaultMinAltitude);

}  // namespace intrinsic::safety

#endif  // INTRINSIC_SAFETY_ALTITUDE_MIN_RULE_H_
