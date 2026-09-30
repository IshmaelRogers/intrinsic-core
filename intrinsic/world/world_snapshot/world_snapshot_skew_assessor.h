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

#ifndef INTRINSIC_WORLD_WORLD_SNAPSHOT_WORLD_SNAPSHOT_SKEW_ASSESSOR_H_
#define INTRINSIC_WORLD_WORLD_SNAPSHOT_WORLD_SNAPSHOT_SKEW_ASSESSOR_H_

#include "intrinsic/world/marine_component_validity/marine_component_validity_policy.h"
#include "intrinsic/world/proto/world_snapshot_descriptor.pb.h"
#include "intrinsic/world/world_snapshot/world_snapshot_skew_policy.h"

namespace intrinsic::world {

// Policy: intrinsic/world/world_snapshot/README.md.

// Assesses a built WorldSnapshotDescriptor against latched timings. The
// descriptor is treated as present. It is read once and not modified, and the
// function does not read any World source.
WorldSnapshotPolicyAssessment AssessWorldSnapshotDescriptorSkew(
    const intrinsic_proto::world::WorldSnapshotDescriptor& descriptor,
    const ComponentTimings& timings, TimeParts query_time,
    const WorldSnapshotSkewPolicy& policy);

}  // namespace intrinsic::world

#endif  // INTRINSIC_WORLD_WORLD_SNAPSHOT_WORLD_SNAPSHOT_SKEW_ASSESSOR_H_
