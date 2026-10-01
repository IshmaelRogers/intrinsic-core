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

#include "intrinsic/safety/clearance_rule.h"

#include <array>
#include <cstddef>
#include <cstdio>
#include <string>
#include <string_view>
#include <utility>

#include "intrinsic/embodiment/frame_policy.h"

namespace intrinsic::safety {
namespace {

constexpr std::string_view kBadConfigSummary = "clearance bad config";
constexpr std::string_view kSnapshotUnusableSummary =
    "clearance snapshot unusable";
constexpr std::string_view kUnknownMapSummary = "clearance unknown map";
constexpr std::string_view kUnusableSummary = "clearance input is not usable";

SafetyRuleResult Critical(std::string_view summary) {
  return SafetyRuleResult{true, kClearanceMinRuleId, kSeverityCritical, summary,
                          kDecisionKindReject};
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

SafetyRuleResult EvaluateClearanceRule(const ClearanceSample &sample,
                                       double min_clearance_m) {
  if (sample.frame_id.empty()) {
    return Critical(kBadConfigSummary);
  }
  if (!sample.snapshot_usable) {
    return Critical(kSnapshotUnusableSummary);
  }
  std::string_view source_name;
  switch (sample.source) {
  case ClearanceSource::kObstacle:
    source_name = "obstacle";
    break;
  case ClearanceSource::kSeafloor:
    source_name = "seafloor";
    break;
  case ClearanceSource::kUnknownMap:
  default:
    return Critical(kUnknownMapSummary);
  }
  if (!embodiment::IsFinite(sample.clearance_m) ||
      !embodiment::IsFinite(min_clearance_m) || min_clearance_m <= 0.0) {
    return Critical(kUnusableSummary);
  }
  if (sample.clearance_m >= min_clearance_m) {
    return SafetyRuleResult{};
  }
  return SafetyRuleResult{
      true,
      kClearanceMinRuleId,
      kSeverityError,
      StoreSummary("clearance below min source=" + std::string(source_name) +
                   " clearance_m=" + FormatValue(sample.clearance_m)),
      kDecisionKindReject,
      /*has_projected_value=*/true,
      /*projected_value=*/sample.clearance_m};
}

} // namespace intrinsic::safety
