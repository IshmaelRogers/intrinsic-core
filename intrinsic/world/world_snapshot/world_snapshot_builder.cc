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

#include "intrinsic/world/world_snapshot/world_snapshot_builder.h"

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <string>
#include <utility>
#include <vector>

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/str_cat.h"
#include "intrinsic/world/proto/world_snapshot_descriptor.pb.h"
#include "intrinsic/world/world_snapshot/world_snapshot_policy.h"

namespace intrinsic::world {

using intrinsic_proto::world::WorldComponentRevision;
using intrinsic_proto::world::WorldSnapshotDescriptor;

namespace {

absl::Status RejectStatus(const SnapshotAssessment& assessment) {
  std::string message = absl::StrCat("Invalid world snapshot: ",
                                     SnapshotErrorName(assessment.error));
  if (assessment.component_index >= 0) {
    absl::StrAppend(&message, " at component index ",
                    assessment.component_index);
  }
  return absl::InvalidArgumentError(message);
}

}  // namespace

WorldSnapshotView WorldSnapshotViewFromProto(
    const WorldSnapshotDescriptor& descriptor, bool present) {
  WorldSnapshotView view;
  view.present = present;
  view.snapshot_id = descriptor.snapshot_id();
  view.creation_time_present = descriptor.has_creation_time();
  view.creation_time.seconds = descriptor.creation_time().seconds();
  view.creation_time.nanos = descriptor.creation_time().nanos();
  view.state_epoch = descriptor.state_epoch();
  view.components.reserve(descriptor.components_size());
  for (const WorldComponentRevision& component : descriptor.components()) {
    view.components.push_back(
        {component.component_kind(), component.revision()});
  }
  return view;
}

SnapshotAssessment AssessWorldSnapshotDescriptor(
    const WorldSnapshotDescriptor& descriptor) {
  return AssessWorldSnapshot(WorldSnapshotViewFromProto(descriptor, true));
}

WorldSnapshotBuilder& WorldSnapshotBuilder::SetStateEpoch(
    uint64_t state_epoch) {
  state_epoch_reader_ = [state_epoch]() { return state_epoch; };
  return *this;
}

WorldSnapshotBuilder& WorldSnapshotBuilder::SetStateEpochSource(
    ValueReader reader) {
  state_epoch_reader_ = std::move(reader);
  return *this;
}

WorldSnapshotBuilder& WorldSnapshotBuilder::AddComponentRevision(
    std::string component_kind, uint64_t revision) {
  components_.emplace_back(std::move(component_kind),
                           [revision]() { return revision; });
  return *this;
}

WorldSnapshotBuilder& WorldSnapshotBuilder::AddComponentRevisionSource(
    std::string component_kind, ValueReader reader) {
  components_.emplace_back(std::move(component_kind), std::move(reader));
  return *this;
}

WorldSnapshotBuilder& WorldSnapshotBuilder::SetCreationTime(
    SnapshotTime creation_time) {
  clock_ = [creation_time]() { return creation_time; };
  return *this;
}

WorldSnapshotBuilder& WorldSnapshotBuilder::SetClock(Clock clock) {
  clock_ = std::move(clock);
  return *this;
}

absl::StatusOr<WorldSnapshotDescriptor> WorldSnapshotBuilder::Build() const {
  // Reject on configuration defects before any source is read.
  std::vector<ComponentRevisionView> kinds;
  kinds.reserve(components_.size());
  for (const auto& [kind, reader] : components_) {
    if (!reader) {
      return absl::InvalidArgumentError(absl::StrCat(
          "Invalid world snapshot: null source for kind '", kind, "'"));
    }
    kinds.push_back({kind, 0});
  }
  const SnapshotAssessment kind_assessment = AssessComponentKinds(kinds);
  if (!kind_assessment.accepted) {
    return RejectStatus(kind_assessment);
  }
  if (!clock_) {
    return RejectStatus({SnapshotError::kCreationTime, -1, false});
  }
  const SnapshotTime creation_time = clock_();
  if (!SnapshotTimeNanosInRange(creation_time.nanos)) {
    return RejectStatus({SnapshotError::kCreationTime, -1, false});
  }

  std::vector<size_t> order(components_.size());
  for (size_t i = 0; i < order.size(); ++i) {
    order[i] = i;
  }
  std::sort(order.begin(), order.end(), [this](size_t a, size_t b) {
    return components_[a].first < components_[b].first;
  });

  const uint64_t state_epoch = state_epoch_reader_ ? state_epoch_reader_() : 0;
  std::vector<ComponentRevisionView> sorted;
  sorted.reserve(order.size());
  for (size_t index : order) {
    const auto& [kind, reader] = components_[index];
    sorted.push_back({kind, reader()});
  }

  WorldSnapshotDescriptor descriptor;
  descriptor.set_snapshot_id(ComputeSnapshotId(state_epoch, sorted));
  descriptor.mutable_creation_time()->set_seconds(creation_time.seconds);
  descriptor.mutable_creation_time()->set_nanos(creation_time.nanos);
  descriptor.set_state_epoch(state_epoch);
  for (const ComponentRevisionView& component : sorted) {
    WorldComponentRevision* out = descriptor.add_components();
    out->set_component_kind(component.component_kind);
    out->set_revision(component.revision);
  }
  return descriptor;
}

}  // namespace intrinsic::world
