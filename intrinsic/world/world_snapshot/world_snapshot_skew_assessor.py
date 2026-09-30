# Copyright 2026 Intrinsic Innovation LLC
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     https://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Skew policy assessment for a built WorldSnapshotDescriptor.

Policy: intrinsic/world/world_snapshot/README.md.
"""

from collections.abc import Iterable

from intrinsic.world.proto import world_snapshot_descriptor_pb2
from intrinsic.world.world_snapshot import world_snapshot_builder
from intrinsic.world.world_snapshot import world_snapshot_skew_policy

_SKEW = world_snapshot_skew_policy


def assess_world_snapshot_descriptor_skew(
    descriptor: world_snapshot_descriptor_pb2.WorldSnapshotDescriptor,
    timings: Iterable[_SKEW.ComponentTiming],
    query_time: _SKEW.TimeParts,
    policy: _SKEW.WorldSnapshotSkewPolicy = _SKEW.WorldSnapshotSkewPolicy(),
) -> _SKEW.WorldSnapshotPolicyAssessment:
  """Assesses a built descriptor against latched timings.

  The descriptor is treated as present. It is read and not modified, and no
  World source is read.
  """
  return _SKEW.assess_world_snapshot_skew(
      world_snapshot_builder.view_from_proto(descriptor, True),
      timings,
      query_time,
      policy,
  )
