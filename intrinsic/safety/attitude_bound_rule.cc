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

#include "intrinsic/safety/attitude_bound_rule.h"

#include <cmath>
#include <string_view>

#include "intrinsic/embodiment/frame_policy.h"

namespace intrinsic::safety {
namespace {

constexpr double kPi = 3.14159265358979323846;

constexpr std::string_view kUnusableSummary = "attitude input is not usable";
constexpr std::string_view kPitchExceededSummary =
    "pitch magnitude exceeds max_pitch";
constexpr std::string_view kRollExceededSummary =
    "roll magnitude exceeds max_roll";

SafetyRuleResult EvaluateAttitudeMaxRule(double angle_rad, double max_angle,
                                         std::string_view rule_id,
                                         std::string_view exceeded_summary) {
  if (!embodiment::IsFinite(angle_rad) || !embodiment::IsFinite(max_angle) ||
      max_angle < 0.0) {
    return SafetyRuleResult{true, rule_id, kSeverityCritical, kUnusableSummary,
                            kDecisionKindReject};
  }
  const double wrapped = WrapToPi(angle_rad);
  if (std::fabs(wrapped) <= max_angle) {
    return SafetyRuleResult{};
  }
  return SafetyRuleResult{
      true,
      rule_id,
      kSeverityError,
      exceeded_summary,
      kDecisionKindProject,
      /*has_projected_value=*/true,
      /*projected_value=*/std::copysign(max_angle, wrapped)};
}

}  // namespace

double WrapToPi(double angle_rad) {
  if (!embodiment::IsFinite(angle_rad)) {
    return std::nan("");
  }
  if (angle_rad > -kPi && angle_rad <= kPi) {
    return angle_rad;
  }
  const double wrapped = std::atan2(std::sin(angle_rad), std::cos(angle_rad));
  return wrapped <= -kPi ? kPi : wrapped;
}

SafetyRuleResult EvaluateAttitudePitchMaxRule(double pitch_rad,
                                              double max_pitch) {
  return EvaluateAttitudeMaxRule(pitch_rad, max_pitch, kAttitudePitchMaxRuleId,
                                 kPitchExceededSummary);
}

SafetyRuleResult EvaluateAttitudeRollMaxRule(double roll_rad, double max_roll) {
  return EvaluateAttitudeMaxRule(roll_rad, max_roll, kAttitudeRollMaxRuleId,
                                 kRollExceededSummary);
}

}  // namespace intrinsic::safety
