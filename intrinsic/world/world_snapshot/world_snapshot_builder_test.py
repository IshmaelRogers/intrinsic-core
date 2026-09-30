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

"""Tests for WorldSnapshotBuilder."""

import collections
import unittest

from intrinsic.world.proto import world_snapshot_descriptor_pb2
from intrinsic.world.world_snapshot import world_snapshot_builder
from intrinsic.world.world_snapshot import world_snapshot_policy

_BUILDER = world_snapshot_builder
_POLICY = world_snapshot_policy
_PB2 = world_snapshot_descriptor_pb2
_C = _POLICY.ComponentRevisionView

_GOLDEN_SNAPSHOT_ID = (
    "b43197bb590584583288de254822664ae70709d0b9103698d296b2bc7ef30f7e"
)
_FIXED_TIME = (1700000000, 250000000)


class _FakeStore:
  """Fake source store that counts every read."""

  def __init__(self):
    self.epoch = 0
    self.revisions = {}
    self.epoch_reads = 0
    self.reads = collections.Counter()

  def read_epoch(self):
    self.epoch_reads += 1
    return self.epoch

  def reader(self, kind):
    def read():
      self.reads[kind] += 1
      return self.revisions[kind]

    return read

  def total_reads(self):
    return self.epoch_reads + sum(self.reads.values())


def _store_builder(store, kinds):
  builder = _BUILDER.WorldSnapshotBuilder()
  builder.set_state_epoch_source(store.read_epoch)
  for kind in kinds:
    builder.add_component_revision_source(kind, store.reader(kind))
  return builder


class WorldSnapshotBuilderTest(unittest.TestCase):

  def test_golden_descriptor_from_sources(self):
    store = _FakeStore()
    store.epoch = 42
    store.revisions = {"current_field": 7, "bathymetry_reference": 3}
    builder = _store_builder(store, ["current_field", "bathymetry_reference"])
    builder.set_creation_time(_FIXED_TIME)
    descriptor = builder.build()
    self.assertEqual(descriptor.snapshot_id, _GOLDEN_SNAPSHOT_ID)
    self.assertEqual(descriptor.creation_time.seconds, 1700000000)
    self.assertEqual(descriptor.creation_time.nanos, 250000000)
    self.assertEqual(descriptor.state_epoch, 42)
    self.assertEqual(
        [(c.component_kind, c.revision) for c in descriptor.components],
        [("bathymetry_reference", 3), ("current_field", 7)],
    )
    self.assertTrue(
        _BUILDER.assess_world_snapshot_descriptor(descriptor).accepted
    )

  def test_snapshot_id_equals_policy_digest(self):
    builder = _BUILDER.WorldSnapshotBuilder()
    builder.set_state_epoch(9)
    builder.add_component_revision("semantic_contacts", 4)
    builder.add_component_revision("occupancy_reference", 0)
    builder.set_creation_time(_FIXED_TIME)
    descriptor = builder.build()
    self.assertEqual(
        descriptor.snapshot_id,
        _POLICY.compute_snapshot_id(
            9, (_C("occupancy_reference", 0), _C("semantic_contacts", 4))
        ),
    )

  def test_same_epoch_and_revisions_give_same_id_across_times(self):
    def build(time):
      builder = _BUILDER.WorldSnapshotBuilder()
      builder.set_state_epoch(42)
      builder.add_component_revision("bathymetry_reference", 3)
      builder.add_component_revision("current_field", 7)
      builder.set_creation_time(time)
      return builder.build()

    first = build((1700000000, 0))
    second = build((1800000000, 999999999))
    self.assertEqual(first.snapshot_id, second.snapshot_id)
    self.assertEqual(first.snapshot_id, _GOLDEN_SNAPSHOT_ID)
    self.assertNotEqual(
        first.creation_time.seconds, second.creation_time.seconds
    )

  def test_input_order_does_not_change_id_or_component_order(self):
    forward = _BUILDER.WorldSnapshotBuilder()
    forward.set_state_epoch(42)
    forward.add_component_revision("bathymetry_reference", 3)
    forward.add_component_revision("current_field", 7)
    forward.set_creation_time(_FIXED_TIME)
    reverse = _BUILDER.WorldSnapshotBuilder()
    reverse.set_state_epoch(42)
    reverse.add_component_revision("current_field", 7)
    reverse.add_component_revision("bathymetry_reference", 3)
    reverse.set_creation_time(_FIXED_TIME)
    a = forward.build()
    b = reverse.build()
    self.assertEqual(a.snapshot_id, b.snapshot_id)
    self.assertEqual(a.SerializeToString(), b.SerializeToString())
    self.assertEqual(b.components[0].component_kind, "bathymetry_reference")

  def test_components_are_sorted_ascending_including_unknown(self):
    builder = _BUILDER.WorldSnapshotBuilder()
    builder.set_state_epoch(1)
    for kind, revision in (
        ("vendor_x", 1),
        ("semantic_contacts", 2),
        ("current_field", 3),
        ("occupancy_reference", 4),
        ("bathymetry_reference", 5),
    ):
      builder.add_component_revision(kind, revision)
    builder.set_creation_time(_FIXED_TIME)
    descriptor = builder.build()
    self.assertEqual(
        [c.component_kind for c in descriptor.components],
        [
            "bathymetry_reference",
            "current_field",
            "occupancy_reference",
            "semantic_contacts",
            "vendor_x",
        ],
    )

  def test_descriptor_is_immutable_after_source_mutation(self):
    store = _FakeStore()
    store.epoch = 42
    store.revisions = {"current_field": 7, "bathymetry_reference": 3}
    builder = _store_builder(store, ["current_field", "bathymetry_reference"])
    builder.set_creation_time(_FIXED_TIME)
    descriptor = builder.build()
    wire_before = descriptor.SerializeToString()

    store.epoch = 43
    store.revisions["current_field"] = 8
    store.revisions["bathymetry_reference"] = 4

    self.assertEqual(descriptor.SerializeToString(), wire_before)
    self.assertEqual(descriptor.state_epoch, 42)
    self.assertEqual(descriptor.components[1].revision, 7)
    later = builder.build()
    self.assertEqual(later.state_epoch, 43)
    self.assertNotEqual(later.snapshot_id, descriptor.snapshot_id)
    self.assertEqual(descriptor.SerializeToString(), wire_before)

  def test_repeated_reads_of_descriptor_are_stable(self):
    builder = _BUILDER.WorldSnapshotBuilder()
    builder.set_state_epoch(42)
    builder.add_component_revision("current_field", 7)
    builder.add_component_revision("bathymetry_reference", 3)
    builder.set_creation_time(_FIXED_TIME)
    descriptor = builder.build()
    first = descriptor.SerializeToString()
    for _ in range(3):
      self.assertEqual(descriptor.SerializeToString(), first)
      self.assertEqual(descriptor.snapshot_id, _GOLDEN_SNAPSHOT_ID)
      self.assertEqual(len(descriptor.components), 2)
    reparsed = _PB2.WorldSnapshotDescriptor()
    reparsed.ParseFromString(first)
    self.assertEqual(reparsed.SerializeToString(), first)

  def test_each_source_is_read_exactly_once_per_build(self):
    kinds = [
        "semantic_contacts",
        "current_field",
        "occupancy_reference",
        "bathymetry_reference",
    ]
    store = _FakeStore()
    store.epoch = 42
    store.revisions = {
        "current_field": 7,
        "bathymetry_reference": 3,
        "occupancy_reference": 1,
        "semantic_contacts": 2,
    }
    builder = _store_builder(store, kinds)
    clock_reads = []

    def clock():
      clock_reads.append(1)
      return _FIXED_TIME

    builder.set_clock(clock)
    builder.build()
    self.assertEqual(store.epoch_reads, 1)
    for kind in kinds:
      self.assertEqual(store.reads[kind], 1, kind)
    self.assertEqual(store.total_reads(), 5)
    self.assertEqual(len(clock_reads), 1)

    builder.build()
    self.assertEqual(store.epoch_reads, 2)
    self.assertEqual(store.reads["current_field"], 2)

  def test_epoch_only_snapshot_is_allowed(self):
    builder = _BUILDER.WorldSnapshotBuilder()
    builder.set_state_epoch(0)
    builder.set_creation_time(_FIXED_TIME)
    descriptor = builder.build()
    self.assertEqual(descriptor.state_epoch, 0)
    self.assertEqual(len(descriptor.components), 0)
    self.assertEqual(descriptor.snapshot_id, _POLICY.compute_snapshot_id(0, ()))
    self.assertTrue(
        _BUILDER.assess_world_snapshot_descriptor(descriptor).accepted
    )

  def test_unset_epoch_defaults_to_zero(self):
    builder = _BUILDER.WorldSnapshotBuilder()
    builder.add_component_revision("current_field", 1)
    builder.set_creation_time(_FIXED_TIME)
    self.assertEqual(builder.build().state_epoch, 0)

  def test_revision_zero_kind_is_kept(self):
    builder = _BUILDER.WorldSnapshotBuilder()
    builder.set_state_epoch(5)
    builder.add_component_revision("current_field", 0)
    builder.set_creation_time(_FIXED_TIME)
    descriptor = builder.build()
    self.assertEqual(len(descriptor.components), 1)
    self.assertEqual(descriptor.components[0].revision, 0)

  def test_empty_kind_is_rejected_before_any_source_read(self):
    store = _FakeStore()
    store.epoch = 1
    store.revisions = {"": 1, "current_field": 2}
    builder = _store_builder(store, ["current_field", ""])
    builder.set_creation_time(_FIXED_TIME)
    with self.assertRaises(_BUILDER.WorldSnapshotBuildError) as context:
      builder.build()
    self.assertEqual(
        context.exception.error, _POLICY.SnapshotError.EMPTY_COMPONENT_KIND
    )
    self.assertIn("empty_component_kind", str(context.exception))
    self.assertEqual(store.total_reads(), 0)

  def test_duplicate_kind_is_rejected_before_any_source_read(self):
    store = _FakeStore()
    store.epoch = 1
    store.revisions = {"current_field": 2}
    builder = _store_builder(store, ["current_field", "current_field"])
    builder.set_creation_time(_FIXED_TIME)
    with self.assertRaises(_BUILDER.WorldSnapshotBuildError) as context:
      builder.build()
    self.assertEqual(
        context.exception.error, _POLICY.SnapshotError.DUPLICATE_COMPONENT_KIND
    )
    self.assertEqual(context.exception.component_index, 1)
    self.assertIn("duplicate_component_kind", str(context.exception))
    self.assertEqual(store.total_reads(), 0)

  def test_bad_creation_nanos_are_rejected(self):
    for nanos in (-1, 1000000000):
      with self.subTest(nanos=nanos):
        builder = _BUILDER.WorldSnapshotBuilder()
        builder.set_state_epoch(1)
        builder.set_creation_time((1700000000, nanos))
        with self.assertRaises(_BUILDER.WorldSnapshotBuildError) as context:
          builder.build()
        self.assertEqual(
            context.exception.error, _POLICY.SnapshotError.CREATION_TIME
        )

  def test_missing_creation_time_is_rejected(self):
    builder = _BUILDER.WorldSnapshotBuilder()
    builder.set_state_epoch(1)
    with self.assertRaises(_BUILDER.WorldSnapshotBuildError) as context:
      builder.build()
    self.assertEqual(
        context.exception.error, _POLICY.SnapshotError.CREATION_TIME
    )

  def test_null_component_source_is_rejected(self):
    builder = _BUILDER.WorldSnapshotBuilder()
    builder.add_component_revision_source("current_field", None)
    builder.set_creation_time(_FIXED_TIME)
    with self.assertRaises(_BUILDER.WorldSnapshotBuildError):
      builder.build()

  def test_never_withholds_a_well_formed_snapshot(self):
    builder = _BUILDER.WorldSnapshotBuilder()
    builder.set_state_epoch(2**64 - 1)
    builder.add_component_revision("semantic_contacts", 0)
    builder.set_creation_time((0, 0))
    self.assertEqual(builder.build().state_epoch, 2**64 - 1)

  def test_descriptor_assessment_maps_proto_defects(self):
    builder = _BUILDER.WorldSnapshotBuilder()
    builder.set_state_epoch(42)
    builder.add_component_revision("current_field", 7)
    builder.set_creation_time(_FIXED_TIME)
    good = builder.build()

    def assess(descriptor):
      return _BUILDER.assess_world_snapshot_descriptor(descriptor)

    self.assertFalse(assess(_PB2.WorldSnapshotDescriptor()).accepted)

    no_id = _PB2.WorldSnapshotDescriptor()
    no_id.CopyFrom(good)
    no_id.ClearField("snapshot_id")
    self.assertEqual(assess(no_id).error, _POLICY.SnapshotError.SNAPSHOT_ID)

    no_time = _PB2.WorldSnapshotDescriptor()
    no_time.CopyFrom(good)
    no_time.ClearField("creation_time")
    self.assertEqual(assess(no_time).error, _POLICY.SnapshotError.CREATION_TIME)

    bad_nanos = _PB2.WorldSnapshotDescriptor()
    bad_nanos.CopyFrom(good)
    bad_nanos.creation_time.nanos = 1000000000
    self.assertEqual(
        assess(bad_nanos).error, _POLICY.SnapshotError.CREATION_TIME
    )

    empty_kind = _PB2.WorldSnapshotDescriptor()
    empty_kind.CopyFrom(good)
    empty_kind.components.add()
    self.assertEqual(
        assess(empty_kind).error, _POLICY.SnapshotError.EMPTY_COMPONENT_KIND
    )

    duplicate = _PB2.WorldSnapshotDescriptor()
    duplicate.CopyFrom(good)
    duplicate.components.add().component_kind = "current_field"
    assessment = assess(duplicate)
    self.assertEqual(
        assessment.error, _POLICY.SnapshotError.DUPLICATE_COMPONENT_KIND
    )
    self.assertEqual(assessment.component_index, 1)


if __name__ == "__main__":
  unittest.main()
