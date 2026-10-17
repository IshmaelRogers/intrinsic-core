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

"""Plain-value tests for the EpisodeEvent host policy."""

import dataclasses
import unittest

from intrinsic_data.episode import episode_event_policy as policy

_Error = policy.EpisodeEventError
_Kind = policy.PayloadKind
_SNAPSHOT = "0123456789abcdef" * 4
_METADATA_TEXT = "marine_randomization_metadata_v1\nseed=1\n"


def _view(**overrides):
  fields = dict(
      header_present=True,
      source_id="episode_recorder",
      frame_id="world_enu",
      source_time_present=True,
      source_time=(1700000000, 0),
      receive_time_present=True,
      receive_time=(1700000000, 1000000),
      schema_version=1,
      snapshot_id=_SNAPSHOT,
      payload_case=policy.PAYLOAD_CASE_SAFETY_DECISION,
  )
  fields.update(overrides)
  return policy.EpisodeEventView(**fields)


def _metadata_view(**overrides):
  fields = dict(
      payload_case=policy.PAYLOAD_CASE_EPISODE_METADATA,
      config_digest=policy.compute_config_digest(_METADATA_TEXT),
      marine_randomization_metadata=_METADATA_TEXT,
  )
  fields.update(overrides)
  return _view(**fields)


def _error(view):
  return policy.assess_episode_event(view).error


class EpisodeEventPolicyTest(unittest.TestCase):

  def test_error_numbers_are_stable(self):
    self.assertEqual(
        {e.name: e.value for e in _Error},
        {
            "NONE": 0,
            "MISSING_HEADER": 1,
            "SOURCE_ID": 2,
            "MISSING_FRAME": 3,
            "SOURCE_TIME": 4,
            "RECEIVE_TIME": 5,
            "SCHEMA_VERSION": 6,
            "SNAPSHOT_ID": 7,
            "CONFIG_DIGEST": 8,
            "CONFIG_DIGEST_MISMATCH": 9,
            "MISSING_PAYLOAD": 10,
        },
    )

  def test_empty_view_is_not_engaged(self):
    assessment = policy.assess_episode_event(policy.EpisodeEventView())
    self.assertIs(assessment.error, _Error.NONE)
    self.assertIs(assessment.payload_kind, _Kind.NONE)
    self.assertFalse(assessment.skip)
    self.assertFalse(assessment.engaged)

  def test_any_single_signal_engages(self):
    signals = (
        dict(header_present=True),
        dict(source_id="s"),
        dict(frame_id="f"),
        dict(source_time_present=True),
        dict(receive_time_present=True),
        dict(schema_version=1),
        dict(snapshot_id=_SNAPSHOT),
        dict(config_digest=_SNAPSHOT),
        dict(provenance_present=True),
        dict(payload_case=policy.PAYLOAD_CASE_INFERENCE_RESULT),
        dict(has_unknown_fields=True),
    )
    for signal in signals:
      with self.subTest(signal=signal):
        view = policy.EpisodeEventView(**signal)
        self.assertTrue(policy.assess_episode_event(view).engaged)

  def test_nominal_events_are_accepted_for_every_known_case(self):
    cases = (
        (policy.PAYLOAD_CASE_SAFETY_DECISION, _Kind.SAFETY_DECISION),
        (policy.PAYLOAD_CASE_VEHICLE_TRAJECTORY, _Kind.VEHICLE_TRAJECTORY),
        (policy.PAYLOAD_CASE_INFERENCE_ENVELOPE, _Kind.INFERENCE_ENVELOPE),
        (policy.PAYLOAD_CASE_INFERENCE_RESULT, _Kind.INFERENCE_RESULT),
    )
    for payload_case, kind in cases:
      with self.subTest(payload_case=payload_case):
        assessment = policy.assess_episode_event(
            _view(payload_case=payload_case)
        )
        self.assertIs(assessment.error, _Error.NONE)
        self.assertIs(assessment.payload_kind, kind)
        self.assertFalse(assessment.skip)
        self.assertTrue(assessment.engaged)
    assessment = policy.assess_episode_event(_metadata_view())
    self.assertIs(assessment.error, _Error.NONE)
    self.assertIs(assessment.payload_kind, _Kind.EPISODE_METADATA)

  def test_empty_snapshot_and_digest_are_absent_not_errors(self):
    self.assertIs(_error(_view(snapshot_id="", config_digest="")), _Error.NONE)

  def test_every_error_row_has_exactly_one_mutation(self):
    self.assertIs(_error(_view(header_present=False)), _Error.MISSING_HEADER)
    self.assertIs(_error(_view(source_id="")), _Error.SOURCE_ID)
    self.assertIs(_error(_view(frame_id="")), _Error.MISSING_FRAME)
    self.assertIs(_error(_view(source_time_present=False)), _Error.SOURCE_TIME)
    for nanos in (-1, 1000000000, 2000000000):
      with self.subTest(nanos=nanos):
        self.assertIs(
            _error(_view(source_time=(1700000000, nanos))), _Error.SOURCE_TIME
        )
        self.assertIs(
            _error(_view(receive_time=(1700000000, nanos))),
            _Error.RECEIVE_TIME,
        )
    self.assertIs(
        _error(_view(receive_time_present=False)), _Error.RECEIVE_TIME
    )
    self.assertIs(_error(_view(schema_version=0)), _Error.SCHEMA_VERSION)
    assessment = policy.assess_episode_event(
        _view(payload_case=policy.PAYLOAD_CASE_NOT_SET)
    )
    self.assertIs(assessment.error, _Error.MISSING_PAYLOAD)
    self.assertIs(assessment.payload_kind, _Kind.NONE)
    self.assertFalse(assessment.skip)

  def test_times_are_not_ordered_and_sequence_is_not_checked(self):
    view = _view(source_time=(1700000010, 0), receive_time=(1700000000, 0))
    self.assertIs(_error(view), _Error.NONE)

  def test_newer_schema_versions_are_accepted(self):
    for version in (1, 2, 3, 4294967295):
      with self.subTest(version=version):
        self.assertIs(_error(_view(schema_version=version)), _Error.NONE)

  def test_hex_rules_apply_to_snapshot_and_config_digest(self):
    bad_values = (
        "A" * 64,
        "a" * 63,
        "a" * 65,
        "g" * 64,
        "sha256:" + "a" * 57,
    )
    for bad in bad_values:
      with self.subTest(bad=bad):
        self.assertIs(_error(_view(snapshot_id=bad)), _Error.SNAPSHOT_ID)
        self.assertIs(_error(_view(config_digest=bad)), _Error.CONFIG_DIGEST)

  def test_config_digest_must_match_metadata_text(self):
    self.assertIs(_error(_metadata_view()), _Error.NONE)
    for overrides in (
        dict(config_digest=policy.compute_config_digest("different")),
        dict(config_digest=""),
        dict(marine_randomization_metadata=""),
        dict(config_digest=_SNAPSHOT),
    ):
      with self.subTest(overrides=overrides):
        self.assertIs(
            _error(_metadata_view(**overrides)), _Error.CONFIG_DIGEST_MISMATCH
        )
    both_empty = _view(payload_case=policy.PAYLOAD_CASE_EPISODE_METADATA)
    self.assertIs(_error(both_empty), _Error.NONE)

  def test_digest_is_not_checked_for_other_payloads(self):
    view = _view(config_digest=policy.compute_config_digest("anything"))
    self.assertIs(_error(view), _Error.NONE)

  def test_compute_config_digest_known_vectors(self):
    self.assertEqual(
        policy.compute_config_digest("abc"),
        "ba7816bf8f01cfea414140de5dae2223b00361a396177a9cb410ff61f20015ad",
    )
    self.assertEqual(policy.compute_config_digest(""), "")
    self.assertTrue(
        policy.is_lowercase_sha256_hex(policy.compute_config_digest("abc"))
    )

  def test_unknown_payload_is_skipped_not_rejected(self):
    assessment = policy.assess_episode_event(
        _view(payload_case=policy.PAYLOAD_CASE_NOT_SET, has_unknown_fields=True)
    )
    self.assertIs(assessment.error, _Error.NONE)
    self.assertIs(assessment.payload_kind, _Kind.UNKNOWN)
    self.assertTrue(assessment.skip)

  def test_unknown_fields_do_not_change_a_known_payload(self):
    assessment = policy.assess_episode_event(_view(has_unknown_fields=True))
    self.assertIs(assessment.error, _Error.NONE)
    self.assertIs(assessment.payload_kind, _Kind.SAFETY_DECISION)
    self.assertFalse(assessment.skip)

  def test_invalid_envelope_is_never_skipped(self):
    assessment = policy.assess_episode_event(
        _view(
            payload_case=policy.PAYLOAD_CASE_NOT_SET,
            has_unknown_fields=True,
            frame_id="",
        )
    )
    self.assertIs(assessment.error, _Error.MISSING_FRAME)
    self.assertFalse(assessment.skip)

  def test_only_unknown_fields_fails_on_missing_header_first(self):
    assessment = policy.assess_episode_event(
        policy.EpisodeEventView(has_unknown_fields=True)
    )
    self.assertIs(assessment.error, _Error.MISSING_HEADER)
    self.assertFalse(assessment.skip)

  def test_first_failing_check_wins(self):
    view = policy.EpisodeEventView(
        provenance_present=True, snapshot_id="ZZ", config_digest="ZZ"
    )
    steps = (
        ({}, _Error.MISSING_HEADER),
        (dict(header_present=True), _Error.SOURCE_ID),
        (dict(source_id="s"), _Error.MISSING_FRAME),
        (dict(frame_id="f"), _Error.SOURCE_TIME),
        (dict(source_time_present=True), _Error.RECEIVE_TIME),
        (dict(receive_time_present=True), _Error.SCHEMA_VERSION),
        (dict(schema_version=1), _Error.SNAPSHOT_ID),
        (dict(snapshot_id=""), _Error.CONFIG_DIGEST),
        (dict(config_digest=""), _Error.MISSING_PAYLOAD),
    )
    for change, expected in steps:
      view = dataclasses.replace(view, **change)
      with self.subTest(expected=expected):
        self.assertIs(_error(view), expected)

  def test_well_formed_but_wrong_digest_is_a_mismatch(self):
    view = _metadata_view(config_digest=_SNAPSHOT, snapshot_id="")
    self.assertIs(_error(view), _Error.CONFIG_DIGEST_MISMATCH)

  def test_views_are_frozen(self):
    with self.assertRaises(dataclasses.FrozenInstanceError):
      _view().frame_id = "x"


if __name__ == "__main__":
  unittest.main()
