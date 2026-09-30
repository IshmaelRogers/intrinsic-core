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

"""Plain-value checks and the snapshot digest for WorldSnapshotDescriptor.

Policy: intrinsic/world/world_snapshot/README.md.

These helpers do not parse protobuf, do not read World entities, and do not
apply skew, age, freshness, or partial-status rules.
"""

from dataclasses import dataclass
from dataclasses import field
import enum
import hashlib

from intrinsic.embodiment import stamped_header_policy

BATHYMETRY_REFERENCE_KIND = "bathymetry_reference"
CURRENT_FIELD_KIND = "current_field"
OCCUPANCY_REFERENCE_KIND = "occupancy_reference"
SEMANTIC_CONTACTS_KIND = "semantic_contacts"

SNAPSHOT_DIGEST_VERSION_LINE = "v1\n"
SNAPSHOT_ID_HEX_LENGTH = 64

# seconds, nanos. Nanos are in [0, 1000000000) when the time is usable.
SnapshotTime = tuple[int, int]


class SnapshotError(enum.Enum):
  """The first structural defect wins, in field order."""

  NONE = 0
  SNAPSHOT_ID = 1
  CREATION_TIME = 2
  EMPTY_COMPONENT_KIND = 3
  DUPLICATE_COMPONENT_KIND = 4


@dataclass(frozen=True)
class ComponentRevisionView:
  component_kind: str = ""
  revision: int = 0


@dataclass(frozen=True)
class WorldSnapshotView:
  """present is false for an empty message."""

  present: bool = False
  snapshot_id: str = ""
  creation_time_present: bool = False
  creation_time: SnapshotTime = (0, 0)
  state_epoch: int = 0
  components: tuple[ComponentRevisionView, ...] = field(default_factory=tuple)


@dataclass(frozen=True)
class SnapshotAssessment:
  error: SnapshotError
  # Index of the first empty or duplicate kind. -1 for other errors. For a
  # duplicate this is the second occurrence in input order.
  component_index: int
  accepted: bool


def snapshot_time_nanos_in_range(nanos: int) -> bool:
  return stamped_header_policy.nanos_in_range(nanos)


def snapshot_digest_input(
    state_epoch: int, components: tuple[ComponentRevisionView, ...]
) -> bytes:
  """UTF-8 digest input. Sorted by kind, creation_time is not included."""
  lines = [
      SNAPSHOT_DIGEST_VERSION_LINE,
      f"state_epoch={state_epoch}\n",
  ]
  # str ordering is by code point, which matches UTF-8 byte order.
  for component in sorted(components, key=lambda c: c.component_kind):
    lines.append(f"{component.component_kind}={component.revision}\n")
  return "".join(lines).encode("utf-8")


def compute_snapshot_id(
    state_epoch: int, components: tuple[ComponentRevisionView, ...]
) -> str:
  """Lowercase hex SHA-256 of snapshot_digest_input. 64 characters.

  Does not check for empty or duplicate kinds. Use assess_component_kinds.
  """
  return hashlib.sha256(
      snapshot_digest_input(state_epoch, components)
  ).hexdigest()


def assess_component_kinds(
    components: tuple[ComponentRevisionView, ...],
) -> SnapshotAssessment:
  """Checks the first empty kind or repeated kind."""
  seen = set()
  for index, component in enumerate(components):
    kind = component.component_kind
    if not kind:
      return SnapshotAssessment(
          SnapshotError.EMPTY_COMPONENT_KIND, index, False
      )
    if kind in seen:
      return SnapshotAssessment(
          SnapshotError.DUPLICATE_COMPONENT_KIND, index, False
      )
    seen.add(kind)
  return SnapshotAssessment(SnapshotError.NONE, -1, True)


def assess_world_snapshot(view: WorldSnapshotView) -> SnapshotAssessment:
  """First defect wins: snapshot_id, creation_time, then components.

  An empty message is not a present descriptor. It is not an error and it is
  not accepted. This does not recompute snapshot_id.
  """
  if not view.present:
    return SnapshotAssessment(SnapshotError.NONE, -1, False)
  if not view.snapshot_id:
    return SnapshotAssessment(SnapshotError.SNAPSHOT_ID, -1, False)
  if not view.creation_time_present or not snapshot_time_nanos_in_range(
      view.creation_time[1]
  ):
    return SnapshotAssessment(SnapshotError.CREATION_TIME, -1, False)
  return assess_component_kinds(tuple(view.components))
