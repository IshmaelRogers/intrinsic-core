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

"""Tests for the world snapshot skew, age, and required/optional policy."""

import threading
import unittest

from intrinsic.world.proto import world_snapshot_descriptor_pb2
from intrinsic.world.world_snapshot import world_snapshot_builder
from intrinsic.world.world_snapshot import world_snapshot_policy
from intrinsic.world.world_snapshot import world_snapshot_skew_assessor
from intrinsic.world.world_snapshot import world_snapshot_skew_policy

_SKEW = world_snapshot_skew_policy
_SNAPSHOT = world_snapshot_policy
_STATUS = _SKEW.SnapshotPolicyStatus
_ERROR = _SKEW.SnapshotPolicyError

_BATHYMETRY = "bathymetry_reference"
_CURRENT = "current_field"
_OCCUPANCY = "occupancy_reference"
_CONTACTS = "semantic_contacts"

_BASE_SECONDS = 1700000000
_MS = 1000000
_SECOND = 1000000000


def _at(offset_ns):
  seconds, nanos = divmod(offset_ns, _SECOND)
  return (_BASE_SECONDS + seconds, nanos)


def _policy(**overrides):
  values = dict(
      required_kinds=(_BATHYMETRY, _CURRENT),
      optional_kinds=(_OCCUPANCY, _CONTACTS),
  )
  values.update(overrides)
  return _SKEW.WorldSnapshotSkewPolicy(**values)


def _view(kinds):
  return _SNAPSHOT.WorldSnapshotView(
      present=True,
      snapshot_id="id",
      creation_time_present=True,
      creation_time=(_BASE_SECONDS, 0),
      components=tuple(_SNAPSHOT.ComponentRevisionView(k, 1) for k in kinds),
  )


_ALL_KINDS = (_BATHYMETRY, _CURRENT, _OCCUPANCY, _CONTACTS)


def _timing(kind, observation_offset_ns, horizon=(10, 0)):
  return _SKEW.ComponentTiming(
      kind, _at(observation_offset_ns), horizon, "source"
  )


def _all_timings(offset_ns=0):
  return [_timing(kind, offset_ns) for kind in _ALL_KINDS]


def _assess(kinds, timings, query_offset_ns, policy=None):
  return _SKEW.assess_world_snapshot_skew(
      _view(kinds),
      timings,
      _at(query_offset_ns),
      policy if policy is not None else _policy(),
  )


class SkewPolicyTest(unittest.TestCase):

  def test_defaults_are_200ms_skew_and_2s_age(self):
    policy = _SKEW.WorldSnapshotSkewPolicy()
    self.assertEqual(policy.max_skew, (0, 200000000))
    self.assertEqual(policy.max_age, (2, 0))
    self.assertEqual(policy.required_kinds, ())
    self.assertEqual(policy.optional_kinds, ())

  def test_status_numbers_are_locked(self):
    self.assertEqual(
        [s.value for s in _STATUS],
        [0, 1, 2, 3, 4, 5],
    )
    self.assertEqual(_STATUS.FRESH.value, 1)
    self.assertEqual(_STATUS.PARTIAL.value, 2)
    self.assertEqual(_STATUS.STALE.value, 3)
    self.assertEqual(_STATUS.EXCESSIVE_SKEW.value, 4)
    self.assertEqual(_STATUS.INCOMPLETE.value, 5)

  def test_alias_names_the_same_type(self):
    self.assertIs(
        _SKEW.SnapshotSkewAssessment, _SKEW.WorldSnapshotPolicyAssessment
    )

  def test_fresh_when_all_kinds_present_and_within_bounds(self):
    result = _assess(_ALL_KINDS, _all_timings(), 500 * _MS)
    self.assertIs(result.error, _ERROR.NONE)
    self.assertIs(result.status, _STATUS.FRESH)
    self.assertTrue(result.accepted)
    self.assertFalse(result.withhold)
    self.assertEqual(result.offending_kinds, ())
    self.assertEqual(result.missing_optional_kinds, ())
    self.assertTrue(result.measured_skew_present)
    self.assertEqual(result.measured_skew, (0, 0))

  def test_empty_required_and_optional_lists_are_fresh(self):
    result = _assess((), [], 0, _SKEW.WorldSnapshotSkewPolicy())
    self.assertIs(result.status, _STATUS.FRESH)
    self.assertFalse(result.measured_skew_present)

  def test_permitted_partial_when_optional_kind_absent(self):
    result = _assess(
        (_BATHYMETRY, _CURRENT, _OCCUPANCY),
        [
            _timing(_BATHYMETRY, 0),
            _timing(_CURRENT, 50 * _MS),
            _timing(_OCCUPANCY, 100 * _MS),
        ],
        500 * _MS,
    )
    self.assertIs(result.status, _STATUS.PARTIAL)
    self.assertTrue(result.accepted)
    self.assertFalse(result.withhold)
    self.assertEqual(result.offending_kinds, ())
    self.assertEqual(result.missing_optional_kinds, (_CONTACTS,))
    self.assertEqual(result.measured_skew, (0, 100 * _MS))

  def test_all_optional_kinds_absent_is_partial(self):
    result = _assess(
        (_BATHYMETRY, _CURRENT),
        [_timing(_BATHYMETRY, 0), _timing(_CURRENT, 0)],
        0,
    )
    self.assertIs(result.status, _STATUS.PARTIAL)
    self.assertEqual(result.missing_optional_kinds, (_OCCUPANCY, _CONTACTS))

  def test_kinds_outside_required_and_optional_are_ignored(self):
    view = _SNAPSHOT.WorldSnapshotView(
        present=True,
        snapshot_id="id",
        creation_time_present=True,
        creation_time=(_BASE_SECONDS, 0),
        components=tuple(
            _SNAPSHOT.ComponentRevisionView(k, 1)
            for k in (*_ALL_KINDS, "custom_kind")
        ),
    )
    timings = _all_timings() + [_timing("custom_kind", -100 * _SECOND, (0, 1))]
    result = _SKEW.assess_world_snapshot_skew(
        view, timings, _at(500 * _MS), _policy()
    )
    self.assertIs(result.status, _STATUS.FRESH)
    self.assertEqual(result.measured_skew, (0, 0))

  def test_timings_for_kinds_absent_from_descriptor_are_ignored(self):
    timings = [
        _timing(_BATHYMETRY, 0),
        _timing(_CURRENT, 0),
        _timing(_CONTACTS, -50 * _SECOND, (0, 1)),
    ]
    result = _assess((_BATHYMETRY, _CURRENT), timings, 0)
    self.assertIs(result.status, _STATUS.PARTIAL)
    self.assertEqual(result.measured_skew, (0, 0))

  def test_horizon_boundary_is_fresh_and_next_nanosecond_is_stale(self):
    timings = _all_timings()
    timings[1] = _timing(_CURRENT, 0, (0, 500 * _MS))
    at_deadline = _assess(_ALL_KINDS, timings, 500 * _MS)
    self.assertIs(at_deadline.status, _STATUS.FRESH)

    past_deadline = _assess(_ALL_KINDS, timings, 500 * _MS + 1)
    self.assertIs(past_deadline.status, _STATUS.STALE)
    self.assertFalse(past_deadline.accepted)
    self.assertTrue(past_deadline.withhold)
    self.assertEqual(past_deadline.offending_kinds, (_CURRENT,))

  def test_max_age_boundary_is_fresh_and_next_nanosecond_is_stale(self):
    at_limit = _assess(_ALL_KINDS, _all_timings(), 2 * _SECOND)
    self.assertIs(at_limit.status, _STATUS.FRESH)

    past_limit = _assess(_ALL_KINDS, _all_timings(), 2 * _SECOND + 1)
    self.assertIs(past_limit.status, _STATUS.STALE)
    self.assertTrue(past_limit.withhold)
    self.assertEqual(
        past_limit.offending_kinds,
        (_BATHYMETRY, _CURRENT, _OCCUPANCY, _CONTACTS),
    )

  def test_max_age_is_evaluated_per_kind(self):
    timings = _all_timings(1500 * _MS)
    timings[1] = _timing(_CURRENT, 0)
    result = _assess(
        _ALL_KINDS,
        timings,
        2 * _SECOND + 1,
        _policy(max_skew=(5, 0)),
    )
    self.assertIs(result.status, _STATUS.STALE)
    self.assertEqual(result.offending_kinds, (_CURRENT,))

  def test_observation_after_query_time_is_not_stale(self):
    result = _assess(_ALL_KINDS, _all_timings(100 * _MS), 0)
    self.assertIs(result.status, _STATUS.FRESH)

  def test_present_optional_kind_with_expired_horizon_is_stale(self):
    timings = _all_timings()
    timings[2] = _timing(_OCCUPANCY, 0, (0, 1))
    result = _assess(_ALL_KINDS, timings, 2)
    self.assertIs(result.status, _STATUS.STALE)
    self.assertEqual(result.offending_kinds, (_OCCUPANCY,))

  def test_present_kind_without_timing_is_stale(self):
    required_missing = _assess(
        _ALL_KINDS,
        [
            _timing(_BATHYMETRY, 0),
            _timing(_OCCUPANCY, 0),
            _timing(_CONTACTS, 0),
        ],
        0,
    )
    self.assertIs(required_missing.status, _STATUS.STALE)
    self.assertEqual(required_missing.offending_kinds, (_CURRENT,))

    optional_missing = _assess(
        _ALL_KINDS,
        [
            _timing(_BATHYMETRY, 0),
            _timing(_CURRENT, 0),
            _timing(_OCCUPANCY, 0),
        ],
        0,
    )
    self.assertIs(optional_missing.status, _STATUS.STALE)
    self.assertEqual(optional_missing.offending_kinds, (_CONTACTS,))

  def test_unusable_timing_is_stale(self):
    timings = _all_timings()
    timings[0] = _SKEW.ComponentTiming(
        _BATHYMETRY, (_BASE_SECONDS, 1000000000), (10, 0)
    )
    bad_observation = _assess(_ALL_KINDS, timings, 0)
    self.assertIs(bad_observation.status, _STATUS.STALE)
    self.assertEqual(bad_observation.offending_kinds, (_BATHYMETRY,))

    timings[0] = _SKEW.ComponentTiming(_BATHYMETRY, _at(0), (-1, 0))
    bad_horizon = _assess(_ALL_KINDS, timings, 0)
    self.assertIs(bad_horizon.status, _STATUS.STALE)
    self.assertEqual(bad_horizon.offending_kinds, (_BATHYMETRY,))

  def test_skew_boundary_is_within_limit(self):
    timings = _all_timings()
    timings[3] = _timing(_CONTACTS, 200 * _MS)
    result = _assess(_ALL_KINDS, timings, 300 * _MS)
    self.assertIs(result.status, _STATUS.FRESH)
    self.assertTrue(result.accepted)
    self.assertFalse(result.withhold)
    self.assertTrue(result.measured_skew_present)
    self.assertEqual(result.measured_skew, (0, 200 * _MS))

  def test_skew_one_nanosecond_over_withholds_and_names_kinds(self):
    timings = _all_timings(50 * _MS)
    timings[1] = _timing(_CURRENT, 0)
    timings[3] = _timing(_CONTACTS, 200 * _MS + 1)
    result = _assess(_ALL_KINDS, timings, 300 * _MS)
    self.assertIs(result.status, _STATUS.EXCESSIVE_SKEW)
    self.assertFalse(result.accepted)
    self.assertTrue(result.withhold)
    self.assertEqual(result.earliest_kind, _CURRENT)
    self.assertEqual(result.latest_kind, _CONTACTS)
    self.assertEqual(result.offending_kinds, (_CURRENT, _CONTACTS))
    self.assertTrue(result.measured_skew_present)
    self.assertEqual(result.measured_skew, (0, 200 * _MS + 1))

  def test_skew_across_second_boundary_is_measured(self):
    timings = _all_timings(900 * _MS)
    timings[2] = _timing(_OCCUPANCY, 1100 * _MS)
    result = _assess(_ALL_KINDS, timings, 1200 * _MS)
    self.assertIs(result.status, _STATUS.FRESH)
    self.assertEqual(result.measured_skew, (0, 200 * _MS))

  def test_skew_ties_go_to_smallest_kind(self):
    timings = [
        _timing(_BATHYMETRY, 0),
        _timing(_CURRENT, 0),
        _timing(_OCCUPANCY, 300 * _MS),
        _timing(_CONTACTS, 300 * _MS),
    ]
    result = _assess(_ALL_KINDS, timings, 400 * _MS)
    self.assertIs(result.status, _STATUS.EXCESSIVE_SKEW)
    self.assertEqual(result.earliest_kind, _BATHYMETRY)
    self.assertEqual(result.latest_kind, _OCCUPANCY)
    self.assertEqual(result.offending_kinds, (_BATHYMETRY, _OCCUPANCY))

  def test_custom_max_skew_is_honored(self):
    timings = _all_timings()
    timings[0] = _timing(_BATHYMETRY, 1)
    self.assertIs(
        _assess(_ALL_KINDS, timings, 1, _policy(max_skew=(0, 0))).status,
        _STATUS.EXCESSIVE_SKEW,
    )
    self.assertIs(
        _assess(_ALL_KINDS, timings, 1, _policy(max_skew=(0, 1))).status,
        _STATUS.FRESH,
    )

  def test_single_timed_kind_has_no_measured_skew(self):
    result = _assess(
        (_BATHYMETRY,),
        [_timing(_BATHYMETRY, 0)],
        0,
        _SKEW.WorldSnapshotSkewPolicy(required_kinds=(_BATHYMETRY,)),
    )
    self.assertIs(result.status, _STATUS.FRESH)
    self.assertFalse(result.measured_skew_present)
    self.assertEqual(result.measured_skew, (0, 0))
    self.assertEqual(result.earliest_kind, "")
    self.assertEqual(result.latest_kind, "")

  def test_missing_required_kind_is_incomplete_not_partial(self):
    result = _assess((_BATHYMETRY,), [_timing(_BATHYMETRY, 0)], 0)
    self.assertIs(result.status, _STATUS.INCOMPLETE)
    self.assertIsNot(result.status, _STATUS.PARTIAL)
    self.assertFalse(result.accepted)
    self.assertTrue(result.withhold)
    self.assertEqual(result.offending_kinds, (_CURRENT,))
    self.assertEqual(result.missing_optional_kinds, (_OCCUPANCY, _CONTACTS))

  def test_empty_descriptor_with_required_kinds_is_incomplete(self):
    result = _assess((), [], 0)
    self.assertIs(result.status, _STATUS.INCOMPLETE)
    self.assertEqual(result.offending_kinds, (_BATHYMETRY, _CURRENT))

  def test_incomplete_beats_stale_and_skew(self):
    result = _assess(
        (_BATHYMETRY, _OCCUPANCY),
        [_timing(_BATHYMETRY, -100 * _SECOND), _timing(_OCCUPANCY, 0)],
        0,
    )
    self.assertIs(result.status, _STATUS.INCOMPLETE)
    self.assertEqual(result.offending_kinds, (_CURRENT,))

  def test_stale_beats_excessive_skew(self):
    timings = _all_timings()
    timings[0] = _timing(_BATHYMETRY, -1 * _SECOND)
    timings[1] = _timing(_CURRENT, 0, (0, 1))
    result = _assess(_ALL_KINDS, timings, 1 * _SECOND)
    self.assertIs(result.status, _STATUS.STALE)
    self.assertEqual(result.offending_kinds, (_CURRENT,))
    self.assertTrue(result.measured_skew_present)
    self.assertEqual(result.measured_skew, (1, 0))

  def test_excessive_skew_beats_partial(self):
    result = _assess(
        (_BATHYMETRY, _CURRENT),
        [_timing(_BATHYMETRY, 0), _timing(_CURRENT, 201 * _MS)],
        300 * _MS,
    )
    self.assertIs(result.status, _STATUS.EXCESSIVE_SKEW)
    self.assertEqual(result.missing_optional_kinds, (_OCCUPANCY, _CONTACTS))

  def test_structural_descriptor_defect_skips_the_policy(self):
    view = _view(_ALL_KINDS)
    bad_id = _SKEW.assess_world_snapshot_skew(
        _SNAPSHOT.WorldSnapshotView(
            present=True,
            snapshot_id="",
            creation_time_present=True,
            components=view.components,
        ),
        _all_timings(),
        _at(0),
        _policy(),
    )
    self.assertIs(bad_id.error, _ERROR.DESCRIPTOR)
    self.assertIs(bad_id.descriptor_error, _SNAPSHOT.SnapshotError.SNAPSHOT_ID)
    self.assertIs(bad_id.status, _STATUS.UNSPECIFIED)
    self.assertFalse(bad_id.accepted)
    self.assertTrue(bad_id.withhold)

    duplicate = _SKEW.assess_world_snapshot_skew(
        _view((*_ALL_KINDS, _CURRENT)), _all_timings(), _at(0), _policy()
    )
    self.assertIs(duplicate.error, _ERROR.DESCRIPTOR)
    self.assertIs(
        duplicate.descriptor_error,
        _SNAPSHOT.SnapshotError.DUPLICATE_COMPONENT_KIND,
    )

    absent = _SKEW.assess_world_snapshot_skew(
        _SNAPSHOT.WorldSnapshotView(), _all_timings(), _at(0), _policy()
    )
    self.assertIs(absent.error, _ERROR.DESCRIPTOR_NOT_PRESENT)
    self.assertIs(absent.status, _STATUS.UNSPECIFIED)
    self.assertFalse(absent.accepted)

  def test_invalid_policy_timings_and_query_time_are_rejected(self):
    def error_of(policy=None, timings=None, query=None):
      return _SKEW.assess_world_snapshot_skew(
          _view(_ALL_KINDS),
          timings if timings is not None else _all_timings(),
          query if query is not None else _at(0),
          policy if policy is not None else _policy(),
      ).error

    self.assertIs(
        error_of(policy=_policy(max_skew=(-1, 0))), _ERROR.INVALID_POLICY
    )
    self.assertIs(
        error_of(policy=_policy(max_age=(0, 1000000000))),
        _ERROR.INVALID_POLICY,
    )
    self.assertIs(
        error_of(policy=_policy(optional_kinds=(_CURRENT,))),
        _ERROR.INVALID_POLICY,
    )
    self.assertIs(
        error_of(policy=_policy(required_kinds=("",))), _ERROR.INVALID_POLICY
    )
    self.assertIs(
        error_of(timings=_all_timings() + [_timing(_CURRENT, 0)]),
        _ERROR.INVALID_TIMINGS,
    )
    self.assertIs(
        error_of(timings=_all_timings() + [_timing("", 0)]),
        _ERROR.INVALID_TIMINGS,
    )
    self.assertIs(
        error_of(query=(_BASE_SECONDS, -1)), _ERROR.INVALID_QUERY_TIME
    )

  def test_result_is_deterministic_for_identical_inputs(self):
    first = _assess(_ALL_KINDS, _all_timings(), 2 * _SECOND + 1)
    second = _assess(_ALL_KINDS, _all_timings(), 2 * _SECOND + 1)
    self.assertEqual(first, second)


class _LiveStore:
  """Live source store. Changes here must not reach a latched snapshot."""

  def __init__(self):
    self._lock = threading.Lock()
    self._revisions = {}
    self._observations = {}

  def set(self, kind, revision, observation):
    with self._lock:
      self._revisions[kind] = revision
      self._observations[kind] = observation

  def bump(self, kind, observation):
    with self._lock:
      self._revisions[kind] += 1
      self._observations[kind] = observation

  def revision_reader(self, kind):
    def read():
      with self._lock:
        return self._revisions[kind]

    return read

  def latch_timings(self, kinds):
    with self._lock:
      return tuple(
          _SKEW.ComponentTiming(kind, self._observations[kind], (10, 0), "live")
          for kind in kinds
      )


def _latch_descriptor(store, kinds):
  builder = world_snapshot_builder.WorldSnapshotBuilder()
  builder.set_state_epoch(5).set_creation_time((_BASE_SECONDS, 0))
  for kind in kinds:
    builder.add_component_revision_source(kind, store.revision_reader(kind))
  return builder.build()


def _assess_descriptor(descriptor, timings, query_offset_ns, policy):
  return world_snapshot_skew_assessor.assess_world_snapshot_descriptor_skew(
      descriptor, timings, _at(query_offset_ns), policy
  )


class LatchedSnapshotTest(unittest.TestCase):

  def test_latched_inputs_are_immutable_under_concurrent_source_changes(self):
    store = _LiveStore()
    for kind in _ALL_KINDS:
      store.set(kind, 1, _at(0))
    policy = _policy()
    query_offset_ns = 300 * _MS

    latched = _latch_descriptor(store, _ALL_KINDS)
    latched_timings = store.latch_timings(_ALL_KINDS)
    latched_bytes = latched.SerializeToString()
    expected = _assess_descriptor(
        latched, latched_timings, query_offset_ns, policy
    )
    self.assertIs(expected.status, _STATUS.FRESH)

    stop = threading.Event()

    def mutate():
      step = 0
      while not stop.is_set():
        step += 1
        for kind in _ALL_KINDS:
          store.bump(kind, _at(step * _SECOND if kind == _CONTACTS else 0))

    mutator = threading.Thread(target=mutate)
    mutator.start()
    try:
      results = [
          _assess_descriptor(latched, latched_timings, query_offset_ns, policy)
          for _ in range(500)
      ]
    finally:
      stop.set()
      mutator.join()
    self.assertTrue(all(result == expected for result in results))
    store.bump(_CONTACTS, _at(3 * _SECOND))

    self.assertEqual(latched.SerializeToString(), latched_bytes)
    self.assertEqual(
        _assess_descriptor(latched, latched_timings, query_offset_ns, policy),
        expected,
    )

    relatched = _latch_descriptor(store, _ALL_KINDS)
    relatched_timings = store.latch_timings(_ALL_KINDS)
    self.assertNotEqual(relatched.snapshot_id, latched.snapshot_id)
    updated = _assess_descriptor(
        relatched, relatched_timings, query_offset_ns, policy
    )
    self.assertNotEqual(updated, expected)
    self.assertIs(updated.status, _STATUS.EXCESSIVE_SKEW)

  def test_new_latch_after_observation_change_reflects_skew(self):
    kinds = (_BATHYMETRY, _CURRENT)
    store = _LiveStore()
    store.set(_BATHYMETRY, 1, _at(0))
    store.set(_CURRENT, 1, _at(0))
    policy = _policy()

    before = _latch_descriptor(store, kinds)
    before_timings = store.latch_timings(kinds)
    store.bump(_CURRENT, _at(250 * _MS))
    old_latch = _assess_descriptor(before, before_timings, 300 * _MS, policy)
    self.assertIs(old_latch.status, _STATUS.PARTIAL)

    after = _latch_descriptor(store, kinds)
    after_timings = store.latch_timings(kinds)
    new_latch = _assess_descriptor(after, after_timings, 300 * _MS, policy)
    self.assertIs(new_latch.status, _STATUS.EXCESSIVE_SKEW)
    self.assertNotEqual(before.snapshot_id, after.snapshot_id)

  def test_assessment_does_not_change_descriptor_or_digest(self):
    golden = "b43197bb590584583288de254822664ae70709d0b9103698d296b2bc7ef30f7e"
    builder = world_snapshot_builder.WorldSnapshotBuilder()
    builder.set_state_epoch(42)
    builder.add_component_revision(_CURRENT, 7)
    builder.add_component_revision(_BATHYMETRY, 3)
    builder.set_creation_time((_BASE_SECONDS, 250000000))
    descriptor = builder.build()
    before = descriptor.SerializeToString()
    result = _assess_descriptor(
        descriptor,
        [_timing(_BATHYMETRY, 0), _timing(_CURRENT, 0)],
        0,
        _policy(),
    )
    self.assertIs(result.status, _STATUS.PARTIAL)
    self.assertEqual(descriptor.SerializeToString(), before)
    self.assertEqual(descriptor.snapshot_id, golden)

  def test_proto_descriptor_with_structural_defect_is_rejected(self):
    empty = world_snapshot_descriptor_pb2.WorldSnapshotDescriptor()
    result = _assess_descriptor(empty, [], 0, _SKEW.WorldSnapshotSkewPolicy())
    self.assertIs(result.error, _ERROR.DESCRIPTOR)
    self.assertIs(result.descriptor_error, _SNAPSHOT.SnapshotError.SNAPSHOT_ID)
    self.assertFalse(result.accepted)


if __name__ == "__main__":
  unittest.main()
