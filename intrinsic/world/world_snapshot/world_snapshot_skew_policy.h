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

#ifndef INTRINSIC_WORLD_WORLD_SNAPSHOT_WORLD_SNAPSHOT_SKEW_POLICY_H_
#define INTRINSIC_WORLD_WORLD_SNAPSHOT_WORLD_SNAPSHOT_SKEW_POLICY_H_

#include <algorithm>
#include <cstdint>
#include <limits>
#include <set>
#include <string>
#include <string_view>
#include <vector>

#include "intrinsic/world/marine_component_validity/marine_component_validity_policy.h"
#include "intrinsic/world/world_snapshot/world_snapshot_policy.h"

namespace intrinsic::world {

// Policy: intrinsic/world/world_snapshot/README.md.
//
// Opt-in skew, age, and required/optional assessment of one latched
// WorldSnapshotDescriptor. This is a pure function of the descriptor view,
// the latched per-kind timings, the query time, and the policy. It does not
// read World entities, does not mutate anything, and does not change the
// descriptor or its digest. Freshness reuses the marine component validity
// boundary: a query at observation_time + validity_horizon is still fresh.

enum class SnapshotPolicyStatus {
  kUnspecified = 0,
  kFresh = 1,
  kPartial = 2,
  kStale = 3,
  kExcessiveSkew = 4,
  kIncomplete = 5,
};

// Defects that stop the policy from running. None of these is a status.
// kDescriptor carries the structural #113 error in `descriptor_error`.
enum class SnapshotPolicyError {
  kNone = 0,
  kDescriptorNotPresent = 1,
  kDescriptor = 2,
  kInvalidPolicy = 3,
  kInvalidTimings = 4,
  kInvalidQueryTime = 5,
};

inline constexpr TimeParts kDefaultMaxSkew = {0, 200000000};
inline constexpr TimeParts kDefaultMaxAge = {2, 0};

struct WorldSnapshotSkewPolicy {
  // Non-negative, nanos in [0, 1e9). Skew equal to the bound is within it.
  TimeParts max_skew = kDefaultMaxSkew;
  // Non-negative, nanos in [0, 1e9). Ceiling on query_time - observation_time
  // for every kind in the skew set. An age equal to the bound is within it.
  TimeParts max_age = kDefaultMaxAge;
  // Must appear in the descriptor, else INCOMPLETE.
  std::vector<std::string> required_kinds;
  // May be absent from the descriptor, which gives PARTIAL.
  std::vector<std::string> optional_kinds;
};

// Latched timing of one component kind. Plain values only.
struct ComponentTiming {
  std::string component_kind;
  TimeParts observation_time;
  TimeParts validity_horizon;
  // Diagnostics only. May be empty.
  std::string source_id;
};

using ComponentTimings = std::vector<ComponentTiming>;

struct WorldSnapshotPolicyAssessment {
  SnapshotPolicyStatus status = SnapshotPolicyStatus::kUnspecified;
  // Set when the policy could not run. status is then kUnspecified.
  SnapshotPolicyError error = SnapshotPolicyError::kNone;
  // Structural #113 error when error == kDescriptor.
  SnapshotError descriptor_error = SnapshotError::kNone;
  // True only for kFresh and kPartial.
  bool accepted = false;
  // True whenever the snapshot is not accepted: do not publish it to safety
  // or planning consumers. EXCESSIVE_SKEW always withholds.
  bool withhold = false;
  // Sorted unique kinds behind the decisive failure. INCOMPLETE: missing
  // required kinds. STALE: every stale kind. EXCESSIVE_SKEW: earliest and
  // latest kinds. Empty for kFresh and kPartial.
  std::vector<std::string> offending_kinds;
  // Sorted unique optional kinds absent from the descriptor. Set for kPartial
  // and, when reached, for the other statuses as well.
  std::vector<std::string> missing_optional_kinds;
  // max(observation_time) - min(observation_time) over the skew set. Zero
  // and measured_skew_present == false with fewer than two usable members.
  bool measured_skew_present = false;
  TimeParts measured_skew;
  // Kinds with the minimum and maximum observation_time in the skew set.
  // Ties go to the smallest kind. Empty with fewer than two usable members.
  std::string earliest_kind;
  std::string latest_kind;
};

// Alias used in the contract text.
using SnapshotSkewAssessment = WorldSnapshotPolicyAssessment;

inline const char* SnapshotPolicyStatusName(SnapshotPolicyStatus status) {
  switch (status) {
    case SnapshotPolicyStatus::kUnspecified:
      return "unspecified";
    case SnapshotPolicyStatus::kFresh:
      return "fresh";
    case SnapshotPolicyStatus::kPartial:
      return "partial";
    case SnapshotPolicyStatus::kStale:
      return "stale";
    case SnapshotPolicyStatus::kExcessiveSkew:
      return "excessive_skew";
    case SnapshotPolicyStatus::kIncomplete:
      return "incomplete";
  }
  return "unknown";
}

inline const char* SnapshotPolicyErrorName(SnapshotPolicyError error) {
  switch (error) {
    case SnapshotPolicyError::kNone:
      return "none";
    case SnapshotPolicyError::kDescriptorNotPresent:
      return "descriptor_not_present";
    case SnapshotPolicyError::kDescriptor:
      return "descriptor";
    case SnapshotPolicyError::kInvalidPolicy:
      return "invalid_policy";
    case SnapshotPolicyError::kInvalidTimings:
      return "invalid_timings";
    case SnapshotPolicyError::kInvalidQueryTime:
      return "invalid_query_time";
  }
  return "unknown";
}

namespace snapshot_skew_internal {

inline bool TimeLess(TimeParts a, TimeParts b) {
  return TimeStrictlyAfter(b, a);
}

// later - earlier for later >= earlier, saturating at the int64 range.
inline TimeParts NonNegativeDifference(TimeParts later, TimeParts earlier) {
  int64_t seconds = 0;
  if (__builtin_sub_overflow(later.seconds, earlier.seconds, &seconds)) {
    return {std::numeric_limits<int64_t>::max(), 999999999};
  }
  int32_t nanos = later.nanos - earlier.nanos;
  if (nanos < 0) {
    nanos += embodiment::kNanosPerSecond;
    if (seconds == std::numeric_limits<int64_t>::min()) {
      return {std::numeric_limits<int64_t>::max(), 999999999};
    }
    --seconds;
  }
  return {seconds, nanos};
}

inline std::vector<std::string> SortedUnique(std::set<std::string> kinds) {
  return std::vector<std::string>(kinds.begin(), kinds.end());
}

inline bool PolicyValid(const WorldSnapshotSkewPolicy& policy) {
  if (!DurationNonNegative(policy.max_skew) ||
      !DurationNonNegative(policy.max_age)) {
    return false;
  }
  std::set<std::string_view> required;
  for (const std::string& kind : policy.required_kinds) {
    if (kind.empty()) return false;
    required.insert(kind);
  }
  for (const std::string& kind : policy.optional_kinds) {
    if (kind.empty() || required.count(kind) > 0) return false;
  }
  return true;
}

inline bool TimingsValid(const ComponentTimings& timings) {
  std::set<std::string_view> seen;
  for (const ComponentTiming& timing : timings) {
    if (timing.component_kind.empty() ||
        !seen.insert(timing.component_kind).second) {
      return false;
    }
  }
  return true;
}

inline WorldSnapshotPolicyAssessment Rejected(SnapshotPolicyError error,
                                              SnapshotError descriptor_error) {
  WorldSnapshotPolicyAssessment result;
  result.error = error;
  result.descriptor_error = descriptor_error;
  result.withhold = true;
  return result;
}

}  // namespace snapshot_skew_internal

// Assesses one latched snapshot.
//
// Order of decisions, first decisive wins:
//   0. A structural defect (absent or malformed descriptor, invalid policy,
//      duplicate or empty timing kind, query_time nanos out of range) sets
//      `error` and leaves `status` kUnspecified. The policy does not run.
//   1. INCOMPLETE: a required kind is absent from the descriptor.
//   2. STALE: a kind in the skew set has no timing, or is expired by its
//      validity_horizon, or is older than max_age.
//   3. EXCESSIVE_SKEW: measured skew is strictly greater than max_skew.
//   4. PARTIAL: an optional kind is absent from the descriptor.
//   5. FRESH.
//
// Skew set: kinds present in the descriptor, listed in required_kinds or
// optional_kinds, with a supplied timing whose observation_time is usable.
// Kinds that are neither required nor optional are ignored. Timings for kinds
// absent from the descriptor are ignored.
inline WorldSnapshotPolicyAssessment AssessWorldSnapshotSkew(
    const WorldSnapshotView& descriptor, const ComponentTimings& timings,
    TimeParts query_time, const WorldSnapshotSkewPolicy& policy) {
  using snapshot_skew_internal::Rejected;

  if (!descriptor.present) {
    return Rejected(SnapshotPolicyError::kDescriptorNotPresent,
                    SnapshotError::kNone);
  }
  const SnapshotAssessment structural = AssessWorldSnapshot(descriptor);
  if (!structural.accepted) {
    return Rejected(SnapshotPolicyError::kDescriptor, structural.error);
  }
  if (!snapshot_skew_internal::PolicyValid(policy)) {
    return Rejected(SnapshotPolicyError::kInvalidPolicy, SnapshotError::kNone);
  }
  if (!snapshot_skew_internal::TimingsValid(timings)) {
    return Rejected(SnapshotPolicyError::kInvalidTimings, SnapshotError::kNone);
  }
  if (!TimestampNanosInRange(query_time.nanos)) {
    return Rejected(SnapshotPolicyError::kInvalidQueryTime,
                    SnapshotError::kNone);
  }

  std::set<std::string_view> present_kinds;
  for (const ComponentRevisionView& component : descriptor.components) {
    present_kinds.insert(component.component_kind);
  }
  const std::set<std::string> required(policy.required_kinds.begin(),
                                       policy.required_kinds.end());
  const std::set<std::string> optional(policy.optional_kinds.begin(),
                                       policy.optional_kinds.end());

  std::set<std::string> missing_required;
  for (const std::string& kind : required) {
    if (present_kinds.count(kind) == 0) missing_required.insert(kind);
  }
  std::set<std::string> missing_optional;
  for (const std::string& kind : optional) {
    if (present_kinds.count(kind) == 0) missing_optional.insert(kind);
  }

  WorldSnapshotPolicyAssessment result;
  result.missing_optional_kinds =
      snapshot_skew_internal::SortedUnique(missing_optional);

  std::set<std::string> stale;
  bool have_extremes = false;
  int usable_members = 0;
  TimeParts earliest_time;
  TimeParts latest_time;
  for (const ComponentRevisionView& component : descriptor.components) {
    const std::string& kind = component.component_kind;
    if (required.count(kind) == 0 && optional.count(kind) == 0) continue;
    const ComponentTiming* timing = nullptr;
    for (const ComponentTiming& candidate : timings) {
      if (candidate.component_kind == kind) {
        timing = &candidate;
        break;
      }
    }
    if (timing == nullptr) {
      stale.insert(kind);
      continue;
    }
    const bool observation_usable =
        TimestampNanosInRange(timing->observation_time.nanos);
    // The age ceiling uses the same boundary as a validity horizon.
    for (const TimeParts bound : {timing->validity_horizon, policy.max_age}) {
      if (AssessFreshness(true, true, timing->observation_time, true, bound,
                          query_time) != ComponentFreshness::kFresh) {
        stale.insert(kind);
        break;
      }
    }
    if (!observation_usable) continue;
    ++usable_members;
    if (!have_extremes) {
      earliest_time = latest_time = timing->observation_time;
      result.earliest_kind = result.latest_kind = kind;
      have_extremes = true;
      continue;
    }
    // Ties go to the smallest kind regardless of descriptor order.
    const TimeParts observed = timing->observation_time;
    if (snapshot_skew_internal::TimeLess(observed, earliest_time) ||
        (!snapshot_skew_internal::TimeLess(earliest_time, observed) &&
         kind < result.earliest_kind)) {
      earliest_time = observed;
      result.earliest_kind = kind;
    }
    if (snapshot_skew_internal::TimeLess(latest_time, observed) ||
        (!snapshot_skew_internal::TimeLess(observed, latest_time) &&
         kind < result.latest_kind)) {
      latest_time = observed;
      result.latest_kind = kind;
    }
  }
  bool skew_exceeded = false;
  if (usable_members >= 2) {
    result.measured_skew_present = true;
    result.measured_skew = snapshot_skew_internal::NonNegativeDifference(
        latest_time, earliest_time);
    skew_exceeded = TimeStrictlyAfter(result.measured_skew, policy.max_skew);
  } else {
    result.earliest_kind.clear();
    result.latest_kind.clear();
  }

  if (!missing_required.empty()) {
    result.status = SnapshotPolicyStatus::kIncomplete;
    result.offending_kinds =
        snapshot_skew_internal::SortedUnique(missing_required);
  } else if (!stale.empty()) {
    result.status = SnapshotPolicyStatus::kStale;
    result.offending_kinds = snapshot_skew_internal::SortedUnique(stale);
  } else if (skew_exceeded) {
    result.status = SnapshotPolicyStatus::kExcessiveSkew;
    result.offending_kinds = snapshot_skew_internal::SortedUnique(
        {result.earliest_kind, result.latest_kind});
  } else if (!missing_optional.empty()) {
    result.status = SnapshotPolicyStatus::kPartial;
  } else {
    result.status = SnapshotPolicyStatus::kFresh;
  }
  result.accepted = result.status == SnapshotPolicyStatus::kFresh ||
                    result.status == SnapshotPolicyStatus::kPartial;
  result.withhold = !result.accepted;
  return result;
}

}  // namespace intrinsic::world

#endif  // INTRINSIC_WORLD_WORLD_SNAPSHOT_WORLD_SNAPSHOT_SKEW_POLICY_H_
