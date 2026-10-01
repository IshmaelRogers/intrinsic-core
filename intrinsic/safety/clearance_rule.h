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

#ifndef INTRINSIC_SAFETY_CLEARANCE_RULE_H_
#define INTRINSIC_SAFETY_CLEARANCE_RULE_H_

#include <string_view>

#include "intrinsic/safety/safety_rule_result.h"

namespace intrinsic::safety {

// Pure minimum-clearance rule over a plain injected sample. This does not read
// World, occupancy, or SDF data, call a planner, or rewrite a path. A sample
// under the limit is REJECTed, never clamped or projected.

inline constexpr std::string_view kClearanceMinRuleId = "clearance.min";
inline constexpr double kDefaultMinClearanceM = 0.5;

enum class ClearanceSource {
  kObstacle = 0,   // Static obstacle.
  kSeafloor = 1,   // Terrain or seafloor.
  kUnknownMap = 2, // Map missing or unknown. Always fails closed.
};

// Minimum predicted clearance supplied by the caller. `frame_id` must be
// non-empty and is a case-sensitive audit tag only. The string view is
// borrowed from the caller for the duration of the call only.
struct ClearanceSample {
  std::string_view frame_id;
  ClearanceSource source = ClearanceSource::kObstacle;
  double clearance_m = 0.0;
  // False means the snapshot is stale or invalid.
  bool snapshot_usable = true;
};

// `clearance.min` for one sample. Checked in this order:
//  - Empty `frame_id`: CRITICAL, REJECT ("clearance bad config").
//  - `!snapshot_usable`: CRITICAL, REJECT ("clearance snapshot unusable").
//  - `source == kUnknownMap` (or any value outside the enum): CRITICAL,
//    REJECT ("clearance unknown map"). Never treated as clear.
//  - Non-finite `clearance_m` or `min_clearance_m`, or `min_clearance_m <= 0`:
//    CRITICAL, REJECT ("clearance input is not usable").
//  - `clearance_m >= min_clearance_m` (equal is compliant): compliant.
//  - Otherwise: ERROR, REJECT, summary
//    "clearance below min source=<obstacle|seafloor> clearance_m=<value>".
//    `has_projected_value` is true and `projected_value` is the observed
//    `clearance_m`. This is for audit only. The decision kind is never PROJECT.
//
// The below-min `summary` embeds the measured value, so it is stored in a
// thread-local ring buffer instead of a string literal. It stays valid until at
// least seven further below-min results are produced on the same thread. Copy
// it if it must live longer.
SafetyRuleResult
EvaluateClearanceRule(const ClearanceSample &sample,
                      double min_clearance_m = kDefaultMinClearanceM);

} // namespace intrinsic::safety

#endif // INTRINSIC_SAFETY_CLEARANCE_RULE_H_
