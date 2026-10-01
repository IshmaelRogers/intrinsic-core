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

#include "intrinsic/safety/geofence_rule.h"

#include <array>
#include <cstddef>
#include <string>
#include <string_view>
#include <utility>

#include "intrinsic/embodiment/frame_policy.h"

namespace intrinsic::safety {
namespace {

constexpr std::string_view kBadConfigSummary = "geofence bad config";
constexpr std::string_view kUnusableSummary = "geofence input is not usable";
constexpr std::string_view kFrameMismatchSummary = "geofence frame mismatch";

SafetyRuleResult Critical(std::string_view summary) {
  return SafetyRuleResult{true, kGeofenceAabbRuleId, kSeverityCritical, summary,
                          kDecisionKindReject};
}

SafetyRuleResult Outside(std::string_view summary) {
  return SafetyRuleResult{true, kGeofenceAabbRuleId, kSeverityError, summary,
                          kDecisionKindReject};
}

// Returns a view into thread-local storage that stays valid for the next few
// calls on this thread.
std::string_view StoreSummary(std::string summary) {
  constexpr size_t kSlots = 8;
  thread_local std::array<std::string, kSlots> slots;
  thread_local size_t next = 0;
  std::string& slot = slots[next];
  next = (next + 1) % kSlots;
  slot = std::move(summary);
  return slot;
}

bool IsFinite(const AabbGeofence& f) {
  return embodiment::IsFinite(f.min_x) && embodiment::IsFinite(f.min_y) &&
         embodiment::IsFinite(f.min_z) && embodiment::IsFinite(f.max_x) &&
         embodiment::IsFinite(f.max_y) && embodiment::IsFinite(f.max_z);
}

bool IsFinite(const GeofencePose& p) {
  return embodiment::IsFinite(p.x) && embodiment::IsFinite(p.y) &&
         embodiment::IsFinite(p.z);
}

// Returns true and fills `result` when the fence itself is unusable.
bool FenceRejected(const AabbGeofence& f, SafetyRuleResult& result) {
  if (f.frame_id.empty() || f.region_id.empty()) {
    result = Critical(kBadConfigSummary);
    return true;
  }
  if (!IsFinite(f) || f.min_x > f.max_x || f.min_y > f.max_y ||
      f.min_z > f.max_z) {
    result = Critical(kUnusableSummary);
    return true;
  }
  return false;
}

// Returns true and fills `result` when the pose is unusable for this fence.
bool PoseRejected(const AabbGeofence& f, const GeofencePose& p,
                  SafetyRuleResult& result) {
  if (p.frame_id.empty()) {
    result = Critical(kBadConfigSummary);
    return true;
  }
  if (!IsFinite(p)) {
    result = Critical(kUnusableSummary);
    return true;
  }
  if (p.frame_id != f.frame_id) {
    result = Critical(kFrameMismatchSummary);
    return true;
  }
  return false;
}

bool Contains(const AabbGeofence& f, const GeofencePose& p) {
  return f.min_x <= p.x && p.x <= f.max_x && f.min_y <= p.y && p.y <= f.max_y &&
         f.min_z <= p.z && p.z <= f.max_z;
}

}  // namespace

SafetyRuleResult EvaluateGeofenceAabbRule(const AabbGeofence& fence,
                                          const GeofencePose& pose) {
  SafetyRuleResult rejected;
  if (FenceRejected(fence, rejected) || PoseRejected(fence, pose, rejected)) {
    return rejected;
  }
  if (Contains(fence, pose)) {
    return SafetyRuleResult{};
  }
  return Outside(StoreSummary("geofence outside region=" +
                              std::string(fence.region_id)));
}

SafetyRuleResult EvaluateGeofenceAabbSegmentRule(const AabbGeofence& fence,
                                                 const GeofencePose& start,
                                                 const GeofencePose& end) {
  SafetyRuleResult rejected;
  if (FenceRejected(fence, rejected) || PoseRejected(fence, start, rejected) ||
      PoseRejected(fence, end, rejected)) {
    return rejected;
  }
  const bool start_outside = !Contains(fence, start);
  const bool end_outside = !Contains(fence, end);
  if (!start_outside && !end_outside) {
    return SafetyRuleResult{};
  }
  const char* which =
      start_outside && end_outside ? "both" : (start_outside ? "start" : "end");
  return Outside(StoreSummary("geofence segment outside region=" +
                              std::string(fence.region_id) +
                              " end=" + which));
}

}  // namespace intrinsic::safety
