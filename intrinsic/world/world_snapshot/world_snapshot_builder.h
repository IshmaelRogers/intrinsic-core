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

#ifndef INTRINSIC_WORLD_WORLD_SNAPSHOT_WORLD_SNAPSHOT_BUILDER_H_
#define INTRINSIC_WORLD_WORLD_SNAPSHOT_WORLD_SNAPSHOT_BUILDER_H_

#include <cstdint>
#include <functional>
#include <string>
#include <utility>
#include <vector>

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "intrinsic/world/proto/world_snapshot_descriptor.pb.h"
#include "intrinsic/world/world_snapshot/world_snapshot_policy.h"

namespace intrinsic::world {

// Policy: intrinsic/world/world_snapshot/README.md.

// Maps a descriptor onto the plain view used by AssessWorldSnapshot.
WorldSnapshotView WorldSnapshotViewFromProto(
    const intrinsic_proto::world::WorldSnapshotDescriptor& descriptor,
    bool present);

// Structural check of a present descriptor. Does not recompute snapshot_id
// and does not apply skew, age, or partial-status rules.
SnapshotAssessment AssessWorldSnapshotDescriptor(
    const intrinsic_proto::world::WorldSnapshotDescriptor& descriptor);

// Forms one immutable WorldSnapshotDescriptor from configured sources.
//
// Build() reads the state epoch source, each component revision source, and
// the clock at most once, copies the values into an owned descriptor, and
// returns it. The returned descriptor does not alias the sources, so later
// changes to a source do not change it. Build() does not call any validity,
// skew, or age logic and never withholds a well-formed snapshot.
//
// Not thread-safe. Configure, then call Build().
class WorldSnapshotBuilder {
 public:
  using ValueReader = std::function<uint64_t()>;
  using Clock = std::function<SnapshotTime()>;

  // Plain-value or source-backed state epoch. The last call wins. Without
  // either, the epoch is zero.
  WorldSnapshotBuilder& SetStateEpoch(uint64_t state_epoch);
  WorldSnapshotBuilder& SetStateEpochSource(ValueReader reader);

  // Adds one component kind. Kinds are not checked here; Build() rejects an
  // empty or repeated kind before it reads any source. Absence of a kind is
  // expressed by not adding it.
  WorldSnapshotBuilder& AddComponentRevision(std::string component_kind,
                                             uint64_t revision);
  WorldSnapshotBuilder& AddComponentRevisionSource(std::string component_kind,
                                                   ValueReader reader);

  // Creation time is required. Tests pass a fixed time. The last call wins.
  WorldSnapshotBuilder& SetCreationTime(SnapshotTime creation_time);
  WorldSnapshotBuilder& SetClock(Clock clock);

  // Rejects with InvalidArgument for an empty kind, a repeated kind, a null
  // component source, a missing creation time, or creation time nanos outside
  // [0, 1e9).
  absl::StatusOr<intrinsic_proto::world::WorldSnapshotDescriptor> Build() const;

 private:
  ValueReader state_epoch_reader_;
  std::vector<std::pair<std::string, ValueReader>> components_;
  Clock clock_;
};

}  // namespace intrinsic::world

#endif  // INTRINSIC_WORLD_WORLD_SNAPSHOT_WORLD_SNAPSHOT_BUILDER_H_
