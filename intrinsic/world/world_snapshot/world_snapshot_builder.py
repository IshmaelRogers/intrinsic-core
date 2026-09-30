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

"""Builds one immutable WorldSnapshotDescriptor from configured sources.

Policy: intrinsic/world/world_snapshot/README.md.

build() reads the state epoch source, each component revision source, and the
clock at most once, copies the values into an owned descriptor, and returns
it. It does not call validity, skew, or age logic and never withholds a
well-formed snapshot.
"""

from collections.abc import Callable

from intrinsic.world.proto import world_snapshot_descriptor_pb2
from intrinsic.world.world_snapshot import world_snapshot_policy

_POLICY = world_snapshot_policy

ValueReader = Callable[[], int]
Clock = Callable[[], _POLICY.SnapshotTime]


class WorldSnapshotBuildError(ValueError):
  """Raised by build() for a rejected configuration."""

  def __init__(
      self,
      error: _POLICY.SnapshotError,
      message: str,
      component_index: int = -1,
  ):
    super().__init__(message)
    self.error = error
    self.component_index = component_index


def view_from_proto(
    descriptor: world_snapshot_descriptor_pb2.WorldSnapshotDescriptor,
    present: bool,
) -> _POLICY.WorldSnapshotView:
  return _POLICY.WorldSnapshotView(
      present=present,
      snapshot_id=descriptor.snapshot_id,
      creation_time_present=descriptor.HasField("creation_time"),
      creation_time=(
          descriptor.creation_time.seconds,
          descriptor.creation_time.nanos,
      ),
      state_epoch=descriptor.state_epoch,
      components=tuple(
          _POLICY.ComponentRevisionView(c.component_kind, c.revision)
          for c in descriptor.components
      ),
  )


def assess_world_snapshot_descriptor(
    descriptor: world_snapshot_descriptor_pb2.WorldSnapshotDescriptor,
) -> _POLICY.SnapshotAssessment:
  """Structural check of a present descriptor. Does not recompute the id."""
  return _POLICY.assess_world_snapshot(view_from_proto(descriptor, True))


def _error_message(assessment: _POLICY.SnapshotAssessment) -> str:
  message = f"Invalid world snapshot: {assessment.error.name.lower()}"
  if assessment.component_index >= 0:
    message += f" at component index {assessment.component_index}"
  return message


class WorldSnapshotBuilder:
  """Configure, then call build(). Not thread-safe."""

  def __init__(self):
    self._state_epoch_reader: ValueReader | None = None
    self._components: list[tuple[str, ValueReader]] = []
    self._clock: Clock | None = None

  def set_state_epoch(self, state_epoch: int) -> "WorldSnapshotBuilder":
    """Plain-value epoch. The last call wins. Without one, the epoch is 0."""
    self._state_epoch_reader = lambda: state_epoch
    return self

  def set_state_epoch_source(
      self, reader: ValueReader
  ) -> "WorldSnapshotBuilder":
    self._state_epoch_reader = reader
    return self

  def add_component_revision(
      self, component_kind: str, revision: int
  ) -> "WorldSnapshotBuilder":
    """Adds one kind. build() rejects an empty or repeated kind."""
    self._components.append((component_kind, lambda: revision))
    return self

  def add_component_revision_source(
      self, component_kind: str, reader: ValueReader
  ) -> "WorldSnapshotBuilder":
    self._components.append((component_kind, reader))
    return self

  def set_creation_time(
      self, creation_time: _POLICY.SnapshotTime
  ) -> "WorldSnapshotBuilder":
    """Required. Tests pass a fixed (seconds, nanos). The last call wins."""
    self._clock = lambda: creation_time
    return self

  def set_clock(self, clock: Clock) -> "WorldSnapshotBuilder":
    self._clock = clock
    return self

  def build(self) -> world_snapshot_descriptor_pb2.WorldSnapshotDescriptor:
    """Returns an owned descriptor.

    Raises:
      WorldSnapshotBuildError: An empty kind, a repeated kind, a null
        component source, a missing creation time, or creation time nanos
        outside [0, 1e9). No source is read for a kind or null-source defect.
    """
    kinds = tuple(
        _POLICY.ComponentRevisionView(kind, 0) for kind, _ in self._components
    )
    kind_assessment = _POLICY.assess_component_kinds(kinds)
    if not kind_assessment.accepted:
      raise WorldSnapshotBuildError(
          kind_assessment.error,
          _error_message(kind_assessment),
          kind_assessment.component_index,
      )
    for kind, reader in self._components:
      if reader is None:
        raise WorldSnapshotBuildError(
            _POLICY.SnapshotError.NONE,
            f"Invalid world snapshot: null source for kind '{kind}'",
        )
    if self._clock is None:
      raise WorldSnapshotBuildError(
          _POLICY.SnapshotError.CREATION_TIME,
          "Invalid world snapshot: creation_time",
      )
    seconds, nanos = self._clock()
    if not _POLICY.snapshot_time_nanos_in_range(nanos):
      raise WorldSnapshotBuildError(
          _POLICY.SnapshotError.CREATION_TIME,
          "Invalid world snapshot: creation_time",
      )

    state_epoch = self._state_epoch_reader() if self._state_epoch_reader else 0
    sorted_components = tuple(
        _POLICY.ComponentRevisionView(kind, reader())
        for kind, reader in sorted(self._components, key=lambda c: c[0])
    )

    descriptor = world_snapshot_descriptor_pb2.WorldSnapshotDescriptor()
    descriptor.snapshot_id = _POLICY.compute_snapshot_id(
        state_epoch, sorted_components
    )
    descriptor.creation_time.seconds = seconds
    descriptor.creation_time.nanos = nanos
    descriptor.state_epoch = state_epoch
    for component in sorted_components:
      entry = descriptor.components.add()
      entry.component_kind = component.component_kind
      entry.revision = component.revision
    return descriptor
