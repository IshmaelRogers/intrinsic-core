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

#ifndef INTRINSIC_SAFETY_ENERGY_RESERVE_RULE_H_
#define INTRINSIC_SAFETY_ENERGY_RESERVE_RULE_H_

#include <string_view>

#include "intrinsic/safety/safety_rule_result.h"

namespace intrinsic::safety {

// Pure energy-reserve rule over a plain injected sample. This does not predict
// energy, model a battery or state of charge, integrate power over a
// trajectory, call a planner, or rewrite a path. A sample under the required
// energy is REJECTed, never projected.

inline constexpr std::string_view kEnergyReserveRuleId = "energy.reserve";

// Energy is in Joules and the prediction horizon is in seconds. The caller
// folds any configured reserve and fallback energy into `required_j`.
struct EnergyReserveSample {
  // Energy currently available (J).
  double available_j = 0.0;
  // Predicted mission plus fallback requirement (J).
  double required_j = 0.0;
  // False means the available energy is unknown.
  bool energy_known = true;
  // Validity horizon of the prediction (s).
  double prediction_horizon_s = 0.0;
  // False means the prediction is stale, expired, or invalid.
  bool prediction_usable = true;
};

// `energy.reserve` for one sample. Checked in this order:
//  - `!energy_known`: CRITICAL, REJECT ("unknown energy"). Fails closed even
//    when the numbers would pass.
//  - `!prediction_usable`: CRITICAL, REJECT ("prediction unusable").
//  - Non-finite `available_j`, `required_j`, or `prediction_horizon_s`, or
//    `available_j < 0`, `required_j < 0`, or `prediction_horizon_s <= 0`:
//    CRITICAL, REJECT ("energy input is not usable").
//  - `available_j >= required_j` (equal is compliant): compliant.
//  - Otherwise: ERROR, REJECT, summary
//    "energy below reserve available_j=<a> required_j=<r>".
//    `has_projected_value` is true and `projected_value` is `available_j`. This
//    is for audit only. The decision kind is never PROJECT.
//
// The below-reserve `summary` embeds the measured values, so it is stored in a
// thread-local ring buffer instead of a string literal. It stays valid until at
// least seven further below-reserve results are produced on the same thread.
// Copy it if it must live longer.
SafetyRuleResult EvaluateEnergyReserveRule(const EnergyReserveSample &sample);

} // namespace intrinsic::safety

#endif // INTRINSIC_SAFETY_ENERGY_RESERVE_RULE_H_
