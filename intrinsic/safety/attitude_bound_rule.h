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

#ifndef INTRINSIC_SAFETY_ATTITUDE_BOUND_RULE_H_
#define INTRINSIC_SAFETY_ATTITUDE_BOUND_RULE_H_

#include <string_view>

#include "intrinsic/safety/safety_rule_result.h"

namespace intrinsic::safety {

// Pure pitch and roll attitude gates. Angles are radians. Each angle is
// wrapped to (-pi, pi] before it is compared with its absolute limit. These
// functions do not read World, ICON, a HAL, or Gazebo and do not build a
// DesiredMotion. They only clamp a scalar.

inline constexpr std::string_view kAttitudePitchMaxRuleId =
    "attitude.pitch.max";
inline constexpr std::string_view kAttitudeRollMaxRuleId = "attitude.roll.max";

// Fixture defaults in radians (30 degrees).
inline constexpr double kDefaultMaxPitch = 3.14159265358979323846 / 6.0;
inline constexpr double kDefaultMaxRoll = 3.14159265358979323846 / 6.0;

// Wraps `angle_rad` to the equivalent angle in (-pi, pi]. An angle already in
// that interval is returned unchanged. Otherwise the result is
// `atan2(sin, cos)`, with -pi mapped to +pi. Non-finite input returns NaN.
double WrapToPi(double angle_rad);

// `attitude.pitch.max`.
//  - Non-finite `pitch_rad` or `max_pitch`, or `max_pitch < 0`: CRITICAL,
//    REJECT, no projection.
//  - `|WrapToPi(pitch_rad)| <= max_pitch` (equality is compliant): compliant.
//  - Otherwise ERROR, PROJECT,
//    `projected_value == copysign(max_pitch, WrapToPi(pitch_rad))`.
SafetyRuleResult EvaluateAttitudePitchMaxRule(
    double pitch_rad, double max_pitch = kDefaultMaxPitch);

// `attitude.roll.max`. Same behavior as pitch with `roll_rad` and `max_roll`.
SafetyRuleResult EvaluateAttitudeRollMaxRule(double roll_rad,
                                             double max_roll = kDefaultMaxRoll);

}  // namespace intrinsic::safety

#endif  // INTRINSIC_SAFETY_ATTITUDE_BOUND_RULE_H_
