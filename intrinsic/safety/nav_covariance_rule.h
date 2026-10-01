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

#ifndef INTRINSIC_SAFETY_NAV_COVARIANCE_RULE_H_
#define INTRINSIC_SAFETY_NAV_COVARIANCE_RULE_H_

#include <string_view>

#include "intrinsic/safety/safety_rule_result.h"

namespace intrinsic::safety {

// Pure navigation-covariance rule over plain injected 3x3 covariance matrices.
// This does not read, run, or modify an estimator, ESKF, or filter, and does
// not call a planner. A sample over a sigma limit is REJECTed, never projected.

inline constexpr std::string_view kNavCovarianceRuleId = "nav.covariance";
inline constexpr double kDefaultMaxPositionSigmaM = 1.0;
inline constexpr double kDefaultMaxVelocitySigmaMps = 0.5;

// Row-major 3x3 matrices. Units: position m^2, velocity (m/s)^2.
struct NavCovarianceSample {
  // False means the position covariance is unknown.
  bool position_known = true;
  // False means the velocity covariance is unknown.
  bool velocity_known = true;
  double position_cov[9] = {};
  double velocity_cov[9] = {};
};

// `nav.covariance` for one sample. A matrix is usable when all nine entries
// are finite, `|C[i][j] - C[j][i]| <= 1e-9` for i < j, and it is positive
// semi-definite by its leading principal minors (C00, the leading 2x2
// determinant, and det(C), each `>= -1e-12`). The sigma of a matrix is
// `sqrt(max(C00, C11, C22))`. Checked in this order:
//  - `max_position_sigma_m` or `max_velocity_sigma_mps` non-finite or
//    `<= 0`: CRITICAL, REJECT ("nav covariance bad config").
//  - `!position_known` or `!velocity_known`: CRITICAL, REJECT ("unknown
//    covariance"). Fails closed even when the numbers would pass.
//  - Unusable position matrix: CRITICAL, REJECT ("position covariance not
//    usable").
//  - Unusable velocity matrix: CRITICAL, REJECT ("velocity covariance not
//    usable").
//  - Position sigma `> max_position_sigma_m`: ERROR, REJECT, summary
//    "nav covariance exceeded metric=position sigma=<value>". Takes precedence
//    over velocity when both are exceeded.
//  - Else velocity sigma `> max_velocity_sigma_mps`: ERROR, REJECT, summary
//    "nav covariance exceeded metric=velocity sigma=<value>".
//  - Otherwise (equal to a limit is compliant): compliant.
// On an exceeded result `has_projected_value` is true and `projected_value` is
// the observed sigma. This is for audit only. The decision kind is never
// PROJECT.
//
// The exceeded `summary` embeds the measured value, so it is stored in a
// thread-local ring buffer instead of a string literal. It stays valid until at
// least seven further exceeded results are produced on the same thread. Copy it
// if it must live longer.
SafetyRuleResult EvaluateNavCovarianceRule(
    const NavCovarianceSample &sample,
    double max_position_sigma_m = kDefaultMaxPositionSigmaM,
    double max_velocity_sigma_mps = kDefaultMaxVelocitySigmaMps);

} // namespace intrinsic::safety

#endif // INTRINSIC_SAFETY_NAV_COVARIANCE_RULE_H_
