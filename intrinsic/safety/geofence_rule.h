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

#ifndef INTRINSIC_SAFETY_GEOFENCE_RULE_H_
#define INTRINSIC_SAFETY_GEOFENCE_RULE_H_

#include <string_view>

#include "intrinsic/safety/safety_rule_result.h"

namespace intrinsic::safety {

// Pure axis-aligned bounding-box (AABB) hard geofence. This does not read
// World, query a collision map, plan a trajectory, or project a pose back into
// the fence. A pose outside the fence is REJECTed, never clamped.

inline constexpr std::string_view kGeofenceAabbRuleId = "geofence.aabb";

// Closed axis-aligned box in `frame_id`. `frame_id` and `region_id` must be
// non-empty. Every bound must be finite with `min_i <= max_i`. The string
// views are borrowed from the caller for the duration of the call only.
struct AabbGeofence {
  std::string_view frame_id;
  std::string_view region_id;
  double min_x = 0.0;
  double min_y = 0.0;
  double min_z = 0.0;
  double max_x = 0.0;
  double max_y = 0.0;
  double max_z = 0.0;
};

// Position in meters in `frame_id`. `frame_id` must exactly equal the fence
// `frame_id` (case-sensitive).
struct GeofencePose {
  std::string_view frame_id;
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
};

// `geofence.aabb` for one pose.
//  - Empty fence frame_id, region_id, or pose frame_id: CRITICAL, REJECT.
//  - Non-finite bound or coordinate, or any `min_i > max_i`: CRITICAL, REJECT.
//  - `pose.frame_id != fence.frame_id`: CRITICAL, REJECT ("frame mismatch").
//    Never treated as inside.
//  - `min_i <= p_i <= max_i` on every axis (boundary inclusive): compliant.
//  - Otherwise: ERROR, REJECT, summary "geofence outside region=<region_id>".
//    `has_projected_value` is always false.
//
// A violated outside result carries a `summary` that includes the region id.
// Because the region id is caller-owned, that summary is stored in a
// thread-local ring buffer instead of a string literal. It stays valid until
// at least seven further outside results are produced on the same thread.
// Copy it if it must live longer.
SafetyRuleResult EvaluateGeofenceAabbRule(const AabbGeofence& fence,
                                          const GeofencePose& pose);

// `geofence.aabb` for a straight segment from `start` to `end`. The fence is
// validated once and both poses are validated with the rules above (any
// invalid input is CRITICAL, REJECT). The box is convex, so a segment lies in
// it exactly when both endpoints do; only the endpoints are checked and no
// edge clipping is done. If either endpoint is outside the result is ERROR,
// REJECT with a summary "geofence segment outside region=<region_id>
// end=<start|end|both>". Otherwise it is compliant.
SafetyRuleResult EvaluateGeofenceAabbSegmentRule(const AabbGeofence& fence,
                                                 const GeofencePose& start,
                                                 const GeofencePose& end);

}  // namespace intrinsic::safety

#endif  // INTRINSIC_SAFETY_GEOFENCE_RULE_H_
