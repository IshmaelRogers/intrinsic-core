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

#include "intrinsic/safety/energy_reserve_rule.h"

#include <array>
#include <cmath>
#include <cstddef>
#include <cstdio>
#include <string>
#include <string_view>
#include <utility>

namespace intrinsic::safety {
namespace {

constexpr std::string_view kUnknownEnergySummary = "unknown energy";
constexpr std::string_view kPredictionUnusableSummary = "prediction unusable";
constexpr std::string_view kUnusableSummary = "energy input is not usable";

SafetyRuleResult Critical(std::string_view summary) {
  return SafetyRuleResult{true, kEnergyReserveRuleId, kSeverityCritical,
                          summary, kDecisionKindReject};
}

// Returns a view into thread-local storage that stays valid for the next few
// calls on this thread.
std::string_view StoreSummary(std::string summary) {
  constexpr size_t kSlots = 8;
  thread_local std::array<std::string, kSlots> slots;
  thread_local size_t next = 0;
  std::string &slot = slots[next];
  next = (next + 1) % kSlots;
  slot = std::move(summary);
  return slot;
}

std::string FormatValue(double value) {
  char buffer[32];
  std::snprintf(buffer, sizeof(buffer), "%.9g", value);
  return buffer;
}

} // namespace

SafetyRuleResult EvaluateEnergyReserveRule(const EnergyReserveSample &sample) {
  if (!sample.energy_known) {
    return Critical(kUnknownEnergySummary);
  }
  if (!sample.prediction_usable) {
    return Critical(kPredictionUnusableSummary);
  }
  if (!std::isfinite(sample.available_j) ||
      !std::isfinite(sample.required_j) ||
      !std::isfinite(sample.prediction_horizon_s) || sample.available_j < 0.0 ||
      sample.required_j < 0.0 || sample.prediction_horizon_s <= 0.0) {
    return Critical(kUnusableSummary);
  }
  if (sample.available_j >= sample.required_j) {
    return SafetyRuleResult{};
  }
  return SafetyRuleResult{
      true,
      kEnergyReserveRuleId,
      kSeverityError,
      StoreSummary("energy below reserve available_j=" +
                   FormatValue(sample.available_j) +
                   " required_j=" + FormatValue(sample.required_j)),
      kDecisionKindReject,
      /*has_projected_value=*/true,
      /*projected_value=*/sample.available_j};
}

} // namespace intrinsic::safety
