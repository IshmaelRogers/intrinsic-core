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

"""Tests for the WorldSnapshotDescriptor policy helpers."""

import unittest

from intrinsic.world.world_snapshot import world_snapshot_policy

_POLICY = world_snapshot_policy
_C = _POLICY.ComponentRevisionView

# SHA-256 of "v1\nstate_epoch=42\nbathymetry_reference=3\ncurrent_field=7\n".
# Keep in sync with world_snapshot_policy_test.cc.
_GOLDEN_SNAPSHOT_ID = (
    "b43197bb590584583288de254822664ae70709d0b9103698d296b2bc7ef30f7e"
)
# SHA-256 of "v1\nstate_epoch=0\n".
_EPOCH_ONLY_ZERO_SNAPSHOT_ID = (
    "6c3dd0944e942bfa751163e5b542a230c499cf85ecd0f6ba2df501adfae04b8d"
)


def _valid_view(**overrides):
  fields = dict(
      present=True,
      snapshot_id=_GOLDEN_SNAPSHOT_ID,
      creation_time_present=True,
      creation_time=(1700000000, 250000000),
      state_epoch=42,
      components=(
          _C(_POLICY.BATHYMETRY_REFERENCE_KIND, 3),
          _C(_POLICY.CURRENT_FIELD_KIND, 7),
      ),
  )
  fields.update(overrides)
  return _POLICY.WorldSnapshotView(**fields)


class WorldSnapshotPolicyTest(unittest.TestCase):

  def test_known_kind_strings_are_locked(self):
    self.assertEqual(_POLICY.BATHYMETRY_REFERENCE_KIND, "bathymetry_reference")
    self.assertEqual(_POLICY.CURRENT_FIELD_KIND, "current_field")
    self.assertEqual(_POLICY.OCCUPANCY_REFERENCE_KIND, "occupancy_reference")
    self.assertEqual(_POLICY.SEMANTIC_CONTACTS_KIND, "semantic_contacts")

  def test_digest_input_is_exactly_the_locked_text(self):
    self.assertEqual(
        _POLICY.snapshot_digest_input(
            42, (_C("current_field", 7), _C("bathymetry_reference", 3))
        ),
        b"v1\nstate_epoch=42\nbathymetry_reference=3\ncurrent_field=7\n",
    )

  def test_digest_input_has_no_creation_time_line(self):
    digest_input = _POLICY.snapshot_digest_input(1, (_C("current_field", 2),))
    self.assertNotIn(b"creation_time", digest_input)
    self.assertEqual(digest_input, b"v1\nstate_epoch=1\ncurrent_field=2\n")

  def test_golden_snapshot_id(self):
    snapshot_id = _POLICY.compute_snapshot_id(
        42, (_C("bathymetry_reference", 3), _C("current_field", 7))
    )
    self.assertEqual(snapshot_id, _GOLDEN_SNAPSHOT_ID)
    self.assertEqual(len(snapshot_id), _POLICY.SNAPSHOT_ID_HEX_LENGTH)
    self.assertEqual(snapshot_id, snapshot_id.lower())

  def test_snapshot_id_ignores_input_order(self):
    forward = _POLICY.compute_snapshot_id(
        42, (_C("bathymetry_reference", 3), _C("current_field", 7))
    )
    reversed_order = _POLICY.compute_snapshot_id(
        42, (_C("current_field", 7), _C("bathymetry_reference", 3))
    )
    self.assertEqual(forward, reversed_order)

  def test_sort_is_byte_wise_ascending(self):
    self.assertEqual(
        _POLICY.snapshot_digest_input(0, (_C("a", 1), _C("_b", 2), _C("Z", 3))),
        b"v1\nstate_epoch=0\nZ=3\n_b=2\na=1\n",
    )

  def test_non_ascii_kind_sorts_by_utf8_bytes(self):
    # U+00E9 encodes as 0xC3 0xA9 and sorts after ASCII 'z'.
    self.assertEqual(
        _POLICY.snapshot_digest_input(0, (_C("\u00e9", 1), _C("z", 2))),
        "v1\nstate_epoch=0\nz=2\n\u00e9=1\n".encode("utf-8"),
    )

  def test_epoch_only_snapshot_is_hashable(self):
    self.assertEqual(
        _POLICY.compute_snapshot_id(0, ()), _EPOCH_ONLY_ZERO_SNAPSHOT_ID
    )
    self.assertEqual(
        _POLICY.snapshot_digest_input(0, ()), b"v1\nstate_epoch=0\n"
    )

  def test_epoch_and_revisions_change_the_id(self):
    base = _POLICY.compute_snapshot_id(1, (_C("current_field", 1),))
    self.assertNotEqual(
        base, _POLICY.compute_snapshot_id(2, (_C("current_field", 1),))
    )
    self.assertNotEqual(
        base, _POLICY.compute_snapshot_id(1, (_C("current_field", 2),))
    )
    self.assertNotEqual(
        base, _POLICY.compute_snapshot_id(1, (_C("occupancy_reference", 1),))
    )
    self.assertNotEqual(base, _POLICY.compute_snapshot_id(1, ()))

  def test_revision_zero_is_distinct_from_absent(self):
    self.assertNotEqual(
        _POLICY.compute_snapshot_id(1, (_C("current_field", 0),)),
        _POLICY.compute_snapshot_id(1, ()),
    )
    self.assertEqual(
        _POLICY.snapshot_digest_input(1, (_C("current_field", 0),)),
        b"v1\nstate_epoch=1\ncurrent_field=0\n",
    )

  def test_large_unsigned_values_are_decimal(self):
    big = 2**64 - 1
    self.assertEqual(
        _POLICY.snapshot_digest_input(big, (_C("k", big),)),
        b"v1\nstate_epoch=18446744073709551615\nk=18446744073709551615\n",
    )

  def test_unknown_kind_participates_like_known_kinds(self):
    self.assertEqual(
        _POLICY.snapshot_digest_input(
            1, (_C("vendor_x", 9), _C("current_field", 2))
        ),
        b"v1\nstate_epoch=1\ncurrent_field=2\nvendor_x=9\n",
    )

  def test_empty_view_is_not_present_and_not_accepted(self):
    assessment = _POLICY.assess_world_snapshot(_POLICY.WorldSnapshotView())
    self.assertEqual(assessment.error, _POLICY.SnapshotError.NONE)
    self.assertFalse(assessment.accepted)

  def test_valid_view_is_accepted(self):
    assessment = _POLICY.assess_world_snapshot(_valid_view())
    self.assertEqual(assessment.error, _POLICY.SnapshotError.NONE)
    self.assertEqual(assessment.component_index, -1)
    self.assertTrue(assessment.accepted)

  def test_epoch_only_view_is_accepted(self):
    view = _valid_view(
        components=(),
        state_epoch=0,
        snapshot_id=_EPOCH_ONLY_ZERO_SNAPSHOT_ID,
    )
    self.assertTrue(_POLICY.assess_world_snapshot(view).accepted)

  def test_empty_snapshot_id_is_rejected(self):
    assessment = _POLICY.assess_world_snapshot(_valid_view(snapshot_id=""))
    self.assertEqual(assessment.error, _POLICY.SnapshotError.SNAPSHOT_ID)
    self.assertFalse(assessment.accepted)

  def test_missing_creation_time_is_rejected(self):
    assessment = _POLICY.assess_world_snapshot(
        _valid_view(creation_time_present=False)
    )
    self.assertEqual(assessment.error, _POLICY.SnapshotError.CREATION_TIME)

  def test_bad_creation_nanos_are_rejected(self):
    for nanos in (-1, 1000000000, 2000000000):
      with self.subTest(nanos=nanos):
        assessment = _POLICY.assess_world_snapshot(
            _valid_view(creation_time=(1700000000, nanos))
        )
        self.assertEqual(assessment.error, _POLICY.SnapshotError.CREATION_TIME)
    for nanos in (0, 999999999):
      with self.subTest(nanos=nanos):
        self.assertTrue(
            _POLICY.assess_world_snapshot(
                _valid_view(creation_time=(1700000000, nanos))
            ).accepted
        )

  def test_empty_kind_is_rejected_with_index(self):
    view = _valid_view(components=_valid_view().components + (_C("", 1),))
    assessment = _POLICY.assess_world_snapshot(view)
    self.assertEqual(
        assessment.error, _POLICY.SnapshotError.EMPTY_COMPONENT_KIND
    )
    self.assertEqual(assessment.component_index, 2)

  def test_duplicate_kind_is_rejected_at_second_occurrence(self):
    view = _valid_view(
        components=_valid_view().components + (_C("current_field", 9),)
    )
    assessment = _POLICY.assess_world_snapshot(view)
    self.assertEqual(
        assessment.error, _POLICY.SnapshotError.DUPLICATE_COMPONENT_KIND
    )
    self.assertEqual(assessment.component_index, 2)

  def test_first_defect_in_field_order_wins(self):
    components = _valid_view().components + (_C("", 1),)
    view = _valid_view(
        snapshot_id="", creation_time=(0, -1), components=components
    )
    self.assertEqual(
        _POLICY.assess_world_snapshot(view).error,
        _POLICY.SnapshotError.SNAPSHOT_ID,
    )
    view = _valid_view(creation_time=(0, -1), components=components)
    self.assertEqual(
        _POLICY.assess_world_snapshot(view).error,
        _POLICY.SnapshotError.CREATION_TIME,
    )
    view = _valid_view(components=components)
    self.assertEqual(
        _POLICY.assess_world_snapshot(view).error,
        _POLICY.SnapshotError.EMPTY_COMPONENT_KIND,
    )

  def test_first_component_defect_wins_when_both_kinds_occur(self):
    duplicate_first = _POLICY.assess_component_kinds(
        (_C("a", 1), _C("a", 2), _C("", 3))
    )
    self.assertEqual(
        duplicate_first.error, _POLICY.SnapshotError.DUPLICATE_COMPONENT_KIND
    )
    self.assertEqual(duplicate_first.component_index, 1)
    empty_first = _POLICY.assess_component_kinds(
        (_C("a", 1), _C("", 2), _C("a", 3))
    )
    self.assertEqual(
        empty_first.error, _POLICY.SnapshotError.EMPTY_COMPONENT_KIND
    )
    self.assertEqual(empty_first.component_index, 1)


if __name__ == "__main__":
  unittest.main()
