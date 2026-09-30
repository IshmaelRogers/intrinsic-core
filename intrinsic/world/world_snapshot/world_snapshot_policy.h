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

#ifndef INTRINSIC_WORLD_WORLD_SNAPSHOT_WORLD_SNAPSHOT_POLICY_H_
#define INTRINSIC_WORLD_WORLD_SNAPSHOT_WORLD_SNAPSHOT_POLICY_H_

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <string>
#include <string_view>
#include <unordered_set>
#include <vector>

#include "intrinsic/embodiment/stamped_header_policy.h"
#include "openssl/sha.h"

namespace intrinsic::world {

// Policy: intrinsic/world/world_snapshot/README.md.
//
// Plain-value checks and the snapshot digest for WorldSnapshotDescriptor.
// These functions do not parse protobuf, do not read World entities, and do
// not apply skew, age, freshness, or partial-status rules.

inline constexpr std::string_view kBathymetryReferenceKind =
    "bathymetry_reference";
inline constexpr std::string_view kCurrentFieldKind = "current_field";
inline constexpr std::string_view kOccupancyReferenceKind =
    "occupancy_reference";
inline constexpr std::string_view kSemanticContactsKind = "semantic_contacts";

inline constexpr std::string_view kSnapshotDigestVersionLine = "v1\n";
inline constexpr size_t kSnapshotIdHexLength = 64;

struct SnapshotTime {
  int64_t seconds = 0;
  int32_t nanos = 0;
};

struct ComponentRevisionView {
  std::string component_kind;
  uint64_t revision = 0;
};

enum class SnapshotError {
  kNone = 0,
  kSnapshotId = 1,
  kCreationTime = 2,
  kEmptyComponentKind = 3,
  kDuplicateComponentKind = 4,
};

struct WorldSnapshotView {
  // False when the caller is not validating a present descriptor. An empty
  // message is not present.
  bool present = false;
  std::string snapshot_id;
  bool creation_time_present = false;
  SnapshotTime creation_time;
  uint64_t state_epoch = 0;
  std::vector<ComponentRevisionView> components;
};

struct SnapshotAssessment {
  SnapshotError error = SnapshotError::kNone;
  // Index into `components` of the first empty or duplicate kind. -1 when
  // the error is not a component error. For a duplicate this is the second
  // occurrence in input order.
  int component_index = -1;
  bool accepted = false;
};

inline bool SnapshotTimeNanosInRange(int32_t nanos) {
  return embodiment::NanosInRange(nanos);
}

// Digest input: "v1\n", "state_epoch=<n>\n", then one "<kind>=<revision>\n"
// line per component sorted by kind (byte-wise, ascending). Decimal unsigned
// integers, no spaces. creation_time is not part of the input. Input order
// does not matter.
inline std::string SnapshotDigestInput(
    uint64_t state_epoch,
    const std::vector<ComponentRevisionView>& components) {
  std::vector<const ComponentRevisionView*> sorted;
  sorted.reserve(components.size());
  for (const ComponentRevisionView& component : components) {
    sorted.push_back(&component);
  }
  std::stable_sort(
      sorted.begin(), sorted.end(),
      [](const ComponentRevisionView* a, const ComponentRevisionView* b) {
        return a->component_kind < b->component_kind;
      });
  std::string input(kSnapshotDigestVersionLine);
  input += "state_epoch=";
  input += std::to_string(state_epoch);
  input += '\n';
  for (const ComponentRevisionView* component : sorted) {
    input += component->component_kind;
    input += '=';
    input += std::to_string(component->revision);
    input += '\n';
  }
  return input;
}

// Lowercase hex SHA-256 of SnapshotDigestInput. 64 characters. Does not
// check for empty or duplicate kinds; use AssessWorldSnapshot for that.
inline std::string ComputeSnapshotId(
    uint64_t state_epoch,
    const std::vector<ComponentRevisionView>& components) {
  const std::string input = SnapshotDigestInput(state_epoch, components);
  uint8_t digest[SHA256_DIGEST_LENGTH];
  SHA256(reinterpret_cast<const uint8_t*>(input.data()), input.size(), digest);
  static constexpr char kHex[] = "0123456789abcdef";
  std::string hex;
  hex.reserve(kSnapshotIdHexLength);
  for (uint8_t byte : digest) {
    hex.push_back(kHex[byte >> 4]);
    hex.push_back(kHex[byte & 0x0f]);
  }
  return hex;
}

// Checks the component list only: the first empty kind or repeated kind.
inline SnapshotAssessment AssessComponentKinds(
    const std::vector<ComponentRevisionView>& components) {
  std::unordered_set<std::string_view> seen;
  for (size_t i = 0; i < components.size(); ++i) {
    const std::string_view kind = components[i].component_kind;
    if (kind.empty()) {
      return {SnapshotError::kEmptyComponentKind, static_cast<int>(i), false};
    }
    if (!seen.insert(kind).second) {
      return {SnapshotError::kDuplicateComponentKind, static_cast<int>(i),
              false};
    }
  }
  return {SnapshotError::kNone, -1, true};
}

// First structural defect wins, in field order: snapshot_id, creation_time,
// then components. An empty message is not a present descriptor. It is not
// an error and it is not accepted. This does not recompute snapshot_id.
inline SnapshotAssessment AssessWorldSnapshot(const WorldSnapshotView& view) {
  if (!view.present) {
    return {SnapshotError::kNone, -1, false};
  }
  if (view.snapshot_id.empty()) {
    return {SnapshotError::kSnapshotId, -1, false};
  }
  if (!view.creation_time_present ||
      !SnapshotTimeNanosInRange(view.creation_time.nanos)) {
    return {SnapshotError::kCreationTime, -1, false};
  }
  return AssessComponentKinds(view.components);
}

inline const char* SnapshotErrorName(SnapshotError error) {
  switch (error) {
    case SnapshotError::kNone:
      return "none";
    case SnapshotError::kSnapshotId:
      return "snapshot_id";
    case SnapshotError::kCreationTime:
      return "creation_time";
    case SnapshotError::kEmptyComponentKind:
      return "empty_component_kind";
    case SnapshotError::kDuplicateComponentKind:
      return "duplicate_component_kind";
  }
  return "unknown";
}

}  // namespace intrinsic::world

#endif  // INTRINSIC_WORLD_WORLD_SNAPSHOT_WORLD_SNAPSHOT_POLICY_H_
