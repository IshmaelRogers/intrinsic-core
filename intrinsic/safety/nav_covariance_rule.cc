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

#include "intrinsic/safety/nav_covariance_rule.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdio>
#include <string>
#include <string_view>
#include <utility>

namespace intrinsic::safety {
namespace {

constexpr double kSymmetryEps = 1e-9;
constexpr double kMinorFloor = -1e-12;

constexpr std::string_view kBadConfigSummary = "nav covariance bad config";
constexpr std::string_view kUnknownSummary = "unknown covariance";
constexpr std::string_view kPositionUnusableSummary =
    "position covariance not usable";
constexpr std::string_view kVelocityUnusableSummary =
    "velocity covariance not usable";

SafetyRuleResult Critical(std::string_view summary) {
  return SafetyRuleResult{true, kNavCovarianceRuleId, kSeverityCritical,
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

bool IsValidThreshold(double value) {
  return std::isfinite(value) && value > 0;
}

bool IsUsable(const double (&c)[9]) {
  for (double entry : c) {
    if (!std::isfinite(entry)) {
      return false;
    }
  }
  if (std::fabs(c[1] - c[3]) > kSymmetryEps ||
      std::fabs(c[2] - c[6]) > kSymmetryEps ||
      std::fabs(c[5] - c[7]) > kSymmetryEps) {
    return false;
  }
  const double minor1 = c[0];
  const double minor2 = c[0] * c[4] - c[1] * c[3];
  const double minor3 = c[0] * (c[4] * c[8] - c[5] * c[7]) -
                        c[1] * (c[3] * c[8] - c[5] * c[6]) +
                        c[2] * (c[3] * c[7] - c[4] * c[6]);
  return minor1 >= kMinorFloor && minor2 >= kMinorFloor &&
         minor3 >= kMinorFloor;
}

// Entries within the PSD floor may leave a tiny negative maximum diagonal.
double Sigma(const double (&c)[9]) {
  return std::sqrt(std::max({0.0, c[0], c[4], c[8]}));
}

SafetyRuleResult Exceeded(std::string_view metric, double sigma) {
  return SafetyRuleResult{
      true,
      kNavCovarianceRuleId,
      kSeverityError,
      StoreSummary("nav covariance exceeded metric=" + std::string(metric) +
                   " sigma=" + FormatValue(sigma)),
      kDecisionKindReject,
      /*has_projected_value=*/true,
      /*projected_value=*/sigma};
}

} // namespace

SafetyRuleResult EvaluateNavCovarianceRule(const NavCovarianceSample &sample,
                                           double max_position_sigma_m,
                                           double max_velocity_sigma_mps) {
  if (!IsValidThreshold(max_position_sigma_m) ||
      !IsValidThreshold(max_velocity_sigma_mps)) {
    return Critical(kBadConfigSummary);
  }
  if (!sample.position_known || !sample.velocity_known) {
    return Critical(kUnknownSummary);
  }
  if (!IsUsable(sample.position_cov)) {
    return Critical(kPositionUnusableSummary);
  }
  if (!IsUsable(sample.velocity_cov)) {
    return Critical(kVelocityUnusableSummary);
  }
  const double position_sigma = Sigma(sample.position_cov);
  const double velocity_sigma = Sigma(sample.velocity_cov);
  if (position_sigma > max_position_sigma_m) {
    return Exceeded("position", position_sigma);
  }
  if (velocity_sigma > max_velocity_sigma_mps) {
    return Exceeded("velocity", velocity_sigma);
  }
  return SafetyRuleResult{};
}

} // namespace intrinsic::safety
