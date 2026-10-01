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

#include "intrinsic/safety/non_finite_rule.h"

#include "intrinsic/embodiment/frame_policy.h"

namespace intrinsic::safety {
namespace {

constexpr std::string_view kNonFiniteSummary = "non-finite state value";

SafetyRuleResult NonFiniteViolation() {
  return SafetyRuleResult{true, kStateNonFiniteRuleId, kSeverityCritical,
                          kNonFiniteSummary, kDecisionKindReject};
}

}  // namespace

SafetyRuleResult EvaluateNonFiniteRule(std::span<const double> values) {
  for (const double value : values) {
    if (!embodiment::IsFinite(value)) {
      return NonFiniteViolation();
    }
  }
  return SafetyRuleResult{};
}

SafetyRuleResult EvaluateNonFiniteRule(vehicle::BodyVector value) {
  const double components[] = {
      value.linear_x,  value.linear_y,  value.linear_z,
      value.angular_x, value.angular_y, value.angular_z,
  };
  return EvaluateNonFiniteRule(std::span<const double>(components));
}

SafetyRuleResult EvaluateNonFiniteRule(
    const vehicle::DesiredMotionView& motion) {
  const double components[] = {
      motion.position.x,      motion.position.y,      motion.position.z,
      motion.orientation.x,   motion.orientation.y,   motion.orientation.z,
      motion.orientation.w,   motion.twist.linear_x,  motion.twist.linear_y,
      motion.twist.linear_z,  motion.twist.angular_x, motion.twist.angular_y,
      motion.twist.angular_z, motion.confidence,
  };
  return EvaluateNonFiniteRule(std::span<const double>(components));
}

SafetyRuleResult EvaluateNonFiniteRule(const vehicle::VehicleStateView& state) {
  const double pose[] = {
      state.position.x,    state.position.y,    state.position.z,
      state.orientation.x, state.orientation.y, state.orientation.z,
      state.orientation.w,
  };
  const SafetyRuleResult pose_result =
      EvaluateNonFiniteRule(std::span<const double>(pose));
  if (pose_result.violated) {
    return pose_result;
  }
  const SafetyRuleResult twist = EvaluateNonFiniteRule(state.twist);
  if (twist.violated) {
    return twist;
  }
  const SafetyRuleResult acceleration =
      EvaluateNonFiniteRule(state.acceleration);
  if (acceleration.violated) {
    return acceleration;
  }
  const SafetyRuleResult pose_covariance =
      EvaluateNonFiniteRule(state.pose_covariance);
  if (pose_covariance.violated) {
    return pose_covariance;
  }
  return EvaluateNonFiniteRule(state.twist_covariance);
}

}  // namespace intrinsic::safety
