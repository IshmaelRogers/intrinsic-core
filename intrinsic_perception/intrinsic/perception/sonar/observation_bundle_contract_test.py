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

"""Validation tests for the multimodal observation bundle contract."""

import dataclasses
import unittest

from intrinsic.perception.sonar import observation_bundle_contract_policy as policy

from intrinsic.embodiment import frame_policy
from intrinsic.embodiment import stamped_header_policy

_Error = policy.ObservationBundleContractError
_Kind = stamped_header_policy.ValidityKind
_Modality = policy.ObservationModality
_Reason = policy.AbsentModalityReason
_SECONDS = 1700000000
_SKEW_NANOS = policy.FIXTURE_MAX_SKEW_NANOS
_SNAPSHOT = "0123456789abcdef" * 4


def _slot(reference_id, frame_id, nanos):
  return policy.ObservationSlotView(
      reference_id=reference_id,
      content_type="application/octet-stream",
      byte_size_present=True,
      byte_size=16,
      frame_id=frame_id,
      source_time_present=True,
      source_time=(_SECONDS, nanos),
      receive_time_present=True,
      receive_time=(_SECONDS, nanos),
      clock_domain="monotonic",
  )


def _state(frame_id, nanos, snapshot=""):
  return policy.StateReferenceView(
      message_set=True,
      frame_id=frame_id,
      source_time_present=True,
      source_time=(_SECONDS, nanos),
      receive_time_present=True,
      receive_time=(_SECONDS, nanos),
      clock_domain="monotonic",
      state_epoch=0,
      world_snapshot_id=snapshot,
  )


def _entry(modality, reason):
  return policy.AbsentModalityEntryView(
      modality=int(modality), reason=int(reason)
  )


def _except_fls():
  return (
      _entry(_Modality.SONAR_SSS, _Reason.NOT_CONFIGURED),
      _entry(_Modality.OPTICAL, _Reason.SENSOR_OFFLINE),
      _entry(_Modality.POINT_CLOUD, _Reason.OUT_OF_RANGE),
      _entry(_Modality.VEHICLE_STATE, _Reason.INTENTIONALLY_OMITTED),
  )


def _except_optical():
  return (
      _entry(_Modality.SONAR_FLS, _Reason.NOT_CONFIGURED),
      _entry(_Modality.SONAR_SSS, _Reason.SENSOR_OFFLINE),
      _entry(_Modality.POINT_CLOUD, _Reason.DROPPED_FOR_SKEW),
      _entry(_Modality.VEHICLE_STATE, _Reason.INTENTIONALLY_OMITTED),
  )


def _all_absent():
  return (
      _entry(_Modality.SONAR_FLS, _Reason.NOT_CONFIGURED),
      _entry(_Modality.SONAR_SSS, _Reason.SENSOR_OFFLINE),
      _entry(_Modality.OPTICAL, _Reason.OUT_OF_RANGE),
      _entry(_Modality.POINT_CLOUD, _Reason.DROPPED_FOR_SKEW),
      _entry(_Modality.VEHICLE_STATE, _Reason.INTENTIONALLY_OMITTED),
  )


def _base():
  return policy.ObservationBundleView(
      header_present=True,
      validity_present=True,
      validity_state=1,
      frame_id="sync",
      clock_domain="monotonic",
      source_time_present=True,
      source_time=(_SECONDS, 0),
      receive_time_present=True,
      receive_time=(_SECONDS, 100000000),
      max_skew_present=True,
      max_skew=(0, _SKEW_NANOS),
  )


def _assess(bundle):
  return policy.assess_multimodal_observation_bundle(bundle)


class ObservationBundleContractTest(unittest.TestCase):

  def test_error_and_enum_values_are_locked(self):
    self.assertEqual(int(_Error.NONE), 0)
    self.assertEqual(int(_Error.MISSING_FRAME), 1)
    self.assertEqual(int(_Error.MAX_SKEW), 2)
    self.assertEqual(int(_Error.SLOT), 3)
    self.assertEqual(int(_Error.ABSENT_LIST), 4)
    self.assertEqual(int(_Error.CLOCK_DOMAIN), 5)
    self.assertEqual(int(_Error.FRAME_MISMATCH), 6)
    self.assertEqual(int(_Error.EXCESSIVE_SKEW), 7)
    self.assertEqual(int(_Error.TIME_REVERSAL), 8)
    self.assertEqual(int(_Error.SNAPSHOT_ID), 9)
    self.assertEqual(int(_Error.REFERENCE), 10)
    self.assertEqual(int(_Modality.UNSPECIFIED), 0)
    self.assertEqual(int(_Modality.SONAR_FLS), 1)
    self.assertEqual(int(_Modality.SONAR_SSS), 2)
    self.assertEqual(int(_Modality.OPTICAL), 3)
    self.assertEqual(int(_Modality.POINT_CLOUD), 4)
    self.assertEqual(int(_Modality.VEHICLE_STATE), 5)
    self.assertEqual(int(_Reason.UNSPECIFIED), 0)
    self.assertEqual(int(_Reason.NOT_CONFIGURED), 1)
    self.assertEqual(int(_Reason.SENSOR_OFFLINE), 2)
    self.assertEqual(int(_Reason.OUT_OF_RANGE), 3)
    self.assertEqual(int(_Reason.DROPPED_FOR_SKEW), 4)
    self.assertEqual(int(_Reason.INTENTIONALLY_OMITTED), 5)
    self.assertEqual(policy.FIXTURE_MAX_SKEW_SECONDS, 0)
    self.assertEqual(policy.FIXTURE_MAX_SKEW_NANOS, 200000000)

  def test_empty_is_not_engaged(self):
    empty = policy.ObservationBundleView()
    self.assertFalse(policy.observation_bundle_engaged(empty))
    assessment = _assess(empty)
    self.assertEqual(assessment.error, _Error.NONE)
    self.assertEqual(assessment.validity, _Kind.ABSENT)
    self.assertFalse(assessment.accepted)
    self.assertFalse(assessment.measured_skew_present)
    self.assertEqual(assessment.measured_skew, (0, 0))

  def _expect_accepted(self, bundle):
    assessment = _assess(bundle)
    self.assertEqual(assessment.error, _Error.NONE)
    self.assertEqual(assessment.validity, _Kind.VALID)
    self.assertTrue(assessment.accepted)
    return assessment

  def _expect_error(self, bundle, error):
    assessment = _assess(bundle)
    self.assertEqual(assessment.error, error)
    self.assertEqual(assessment.validity, _Kind.VALID)
    self.assertFalse(assessment.accepted)
    return assessment

  def test_sonar_only_camera_only_and_all_modalities(self):
    sonar = dataclasses.replace(
        _base(),
        fls=_slot("fls-frame-0001", "fls_sonar", 0),
        absent=_except_fls(),
        metadata_present=True,
    )
    sonar_assessment = self._expect_accepted(sonar)
    self.assertTrue(sonar_assessment.measured_skew_present)
    self.assertEqual(sonar_assessment.measured_skew, (0, 0))

    camera = dataclasses.replace(
        _base(),
        optical=_slot("optical-frame-0001", "optical_cam", 50000000),
        absent=_except_optical(),
    )
    self._expect_accepted(camera)

    all_modalities = dataclasses.replace(
        _base(),
        fls=_slot("fls-frame-0001", "fls_sonar", 0),
        sss=_slot("sss-frame-0001", "sss_sonar", 50000000),
        optical=_slot("optical-frame-0001", "optical_cam", 100000000),
        point_cloud=_slot("points-0001", "points", 150000000),
        vehicle_state=dataclasses.replace(
            _state("body", 180000000, _SNAPSHOT), state_epoch=42
        ),
    )
    all_assessment = self._expect_accepted(all_modalities)
    self.assertTrue(all_assessment.measured_skew_present)
    self.assertEqual(all_assessment.measured_skew, (0, 180000000))

  def test_missing_modality_with_approved_reason(self):
    # Optical is not present and not listed: a subset, not a defect.
    bundle = dataclasses.replace(
        _base(),
        fls=_slot("fls-frame-0001", "fls_sonar", 0),
        sss=_slot("sss-frame-0001", "sss_sonar", 1000000),
        absent=_except_fls()[2:],
    )
    self._expect_accepted(bundle)

  def test_all_absent_with_reasons_is_accepted(self):
    bundle = dataclasses.replace(_base(), absent=_all_absent())
    self._expect_accepted(bundle)
    self.assertFalse(_assess(bundle).measured_skew_present)

  def test_engaged_with_nothing_present_or_absent_is_slot(self):
    self._expect_error(_base(), _Error.SLOT)
    self._expect_error(
        dataclasses.replace(_base(), metadata_present=True), _Error.SLOT
    )

  def test_absent_list_defects(self):
    overlap = dataclasses.replace(
        _base(),
        fls=_slot("fls-frame-0001", "fls_sonar", 0),
        absent=(
            _entry(_Modality.SONAR_FLS, _Reason.NOT_CONFIGURED),
            _except_fls()[0],
        ),
    )
    self._expect_error(overlap, _Error.ABSENT_LIST)

    duplicate = dataclasses.replace(
        _base(),
        fls=_slot("fls-frame-0001", "fls_sonar", 0),
        absent=(
            _entry(_Modality.OPTICAL, _Reason.SENSOR_OFFLINE),
            _entry(_Modality.OPTICAL, _Reason.OUT_OF_RANGE),
            _entry(_Modality.SONAR_SSS, _Reason.NOT_CONFIGURED),
            _entry(_Modality.POINT_CLOUD, _Reason.OUT_OF_RANGE),
            _entry(_Modality.VEHICLE_STATE, _Reason.INTENTIONALLY_OMITTED),
        ),
    )
    self._expect_error(duplicate, _Error.ABSENT_LIST)

    unspecified = dataclasses.replace(
        _base(),
        fls=_slot("fls-frame-0001", "fls_sonar", 0),
        absent=(_entry(_Modality.OPTICAL, _Reason.UNSPECIFIED),),
    )
    self._expect_error(unspecified, _Error.ABSENT_LIST)
    self._expect_error(
        dataclasses.replace(
            unspecified,
            absent=(_entry(_Modality.UNSPECIFIED, _Reason.NOT_CONFIGURED),),
        ),
        _Error.ABSENT_LIST,
    )
    self._expect_error(
        dataclasses.replace(
            unspecified,
            absent=(policy.AbsentModalityEntryView(modality=9, reason=1),),
        ),
        _Error.ABSENT_LIST,
    )

  def test_exact_skew_is_ok_and_one_nanosecond_fails(self):
    exact = dataclasses.replace(
        _base(),
        fls=_slot("fls-frame-0001", "fls_sonar", 0),
        sss=_slot("sss-frame-0001", "sss_sonar", _SKEW_NANOS),
        absent=_except_fls()[1:],
    )
    assessment = self._expect_accepted(exact)
    self.assertEqual(assessment.measured_skew, (0, _SKEW_NANOS))

    inside = dataclasses.replace(
        exact,
        sss=dataclasses.replace(
            exact.sss,
            source_time=(_SECONDS, _SKEW_NANOS - 1),
            receive_time=(_SECONDS, _SKEW_NANOS - 1),
        ),
    )
    self._expect_accepted(inside)

    outside = dataclasses.replace(
        exact,
        sss=dataclasses.replace(
            exact.sss,
            source_time=(_SECONDS, _SKEW_NANOS + 1),
            receive_time=(_SECONDS, _SKEW_NANOS + 1),
        ),
    )
    failed = self._expect_error(outside, _Error.EXCESSIVE_SKEW)
    self.assertTrue(failed.measured_skew_present)
    self.assertEqual(failed.measured_skew, (0, _SKEW_NANOS + 1))

  def test_zero_max_skew_allows_equal_times_only(self):
    equal = dataclasses.replace(
        _base(),
        max_skew=(0, 0),
        fls=_slot("fls-frame-0001", "fls_sonar", 0),
        sss=_slot("sss-frame-0001", "sss_sonar", 0),
        absent=_except_fls()[1:],
    )
    self._expect_accepted(equal)
    shifted = dataclasses.replace(
        equal,
        sss=dataclasses.replace(
            equal.sss,
            source_time=(_SECONDS, 1),
            receive_time=(_SECONDS, 1),
        ),
    )
    self._expect_error(shifted, _Error.EXCESSIVE_SKEW)

  def test_frame_mismatch_and_sensor_frames(self):
    rest = (
        _entry(_Modality.POINT_CLOUD, _Reason.NOT_CONFIGURED),
        _entry(_Modality.VEHICLE_STATE, _Reason.INTENTIONALLY_OMITTED),
    )
    mismatch = dataclasses.replace(
        _base(),
        fls=_slot("fls-frame-0001", frame_policy.WORLD_ENU_FRAME_ID, 0),
        optical=_slot(
            "optical-frame-0001", frame_policy.WORLD_NED_FRAME_ID, 1000
        ),
        absent=rest,
    )
    self._expect_error(mismatch, _Error.FRAME_MISMATCH)

    same_world = dataclasses.replace(
        mismatch,
        optical=dataclasses.replace(
            mismatch.optical, frame_id=frame_policy.WORLD_ENU_FRAME_ID
        ),
    )
    self._expect_accepted(same_world)

    sensors = dataclasses.replace(
        same_world,
        fls=dataclasses.replace(same_world.fls, frame_id="fls_sonar"),
        optical=dataclasses.replace(same_world.optical, frame_id="optical_cam"),
    )
    self._expect_accepted(sensors)

    state_world = dataclasses.replace(
        sensors,
        fls=dataclasses.replace(
            sensors.fls, frame_id=frame_policy.WORLD_ENU_FRAME_ID
        ),
        vehicle_state=_state(frame_policy.WORLD_NED_FRAME_ID, 2000),
        absent=(
            _entry(_Modality.SONAR_SSS, _Reason.NOT_CONFIGURED),
            _entry(_Modality.POINT_CLOUD, _Reason.NOT_CONFIGURED),
        ),
    )
    self._expect_accepted(state_world)

  def test_time_reversal_and_equal_receive(self):
    bundle = dataclasses.replace(
        _base(),
        fls=dataclasses.replace(
            _slot("fls-frame-0001", "fls_sonar", 100000000),
            receive_time=(_SECONDS, 50000000),
        ),
        absent=_except_fls(),
    )
    self._expect_error(bundle, _Error.TIME_REVERSAL)
    equal = dataclasses.replace(
        bundle,
        fls=dataclasses.replace(bundle.fls, receive_time=(_SECONDS, 100000000)),
    )
    self._expect_accepted(equal)
    unset_receive = dataclasses.replace(
        equal, fls=dataclasses.replace(equal.fls, receive_time_present=False)
    )
    self._expect_accepted(unset_receive)
    header_reversed = dataclasses.replace(
        equal,
        source_time=(_SECONDS, 1),
        receive_time=(_SECONDS, 0),
    )
    self._expect_error(header_reversed, _Error.TIME_REVERSAL)

  def test_snapshot_id_and_epoch(self):
    absent = (
        _entry(_Modality.SONAR_FLS, _Reason.NOT_CONFIGURED),
        _entry(_Modality.SONAR_SSS, _Reason.NOT_CONFIGURED),
        _entry(_Modality.OPTICAL, _Reason.SENSOR_OFFLINE),
        _entry(_Modality.POINT_CLOUD, _Reason.OUT_OF_RANGE),
    )
    bundle = dataclasses.replace(
        _base(),
        vehicle_state=_state("body", 0, _SNAPSHOT),
        absent=absent,
    )
    self._expect_accepted(bundle)
    self._expect_accepted(
        dataclasses.replace(
            bundle,
            vehicle_state=dataclasses.replace(
                bundle.vehicle_state, world_snapshot_id=""
            ),
        )
    )
    for bad in ("not-a-hex", "a" * 63, "a" * 65, "A" * 64, "g" * 64):
      failed = dataclasses.replace(
          bundle,
          vehicle_state=dataclasses.replace(
              bundle.vehicle_state, world_snapshot_id=bad
          ),
      )
      self._expect_error(failed, _Error.SNAPSHOT_ID)
    self.assertTrue(policy.is_lowercase_sha256_hex(_SNAPSHOT))
    self.assertTrue(policy.is_lowercase_sha256_hex("0" * 64))
    self.assertFalse(policy.is_lowercase_sha256_hex(""))

  def test_slot_and_reference_defects(self):
    bundle = dataclasses.replace(
        _base(),
        fls=dataclasses.replace(
            _slot("fls-frame-0001", "fls_sonar", 0), frame_id=""
        ),
        absent=_except_fls(),
    )
    self._expect_error(bundle, _Error.SLOT)
    missing_time = dataclasses.replace(
        bundle,
        fls=dataclasses.replace(
            _slot("fls-frame-0001", "fls_sonar", 0), source_time_present=False
        ),
    )
    self._expect_error(missing_time, _Error.SLOT)
    bad_nanos = dataclasses.replace(
        bundle,
        fls=dataclasses.replace(
            _slot("fls-frame-0001", "fls_sonar", 0),
            source_time=(_SECONDS, 1000000000),
        ),
    )
    self._expect_error(bad_nanos, _Error.SLOT)
    zero_size = dataclasses.replace(
        bundle,
        fls=dataclasses.replace(
            _slot("fls-frame-0001", "fls_sonar", 0), byte_size=0
        ),
    )
    self._expect_error(zero_size, _Error.REFERENCE)
    unset_size = dataclasses.replace(
        bundle,
        fls=dataclasses.replace(
            _slot("fls-frame-0001", "fls_sonar", 0),
            byte_size_present=False,
            content_type="",
        ),
    )
    self._expect_accepted(unset_size)

  def test_clock_domain_must_match(self):
    bundle = dataclasses.replace(
        _base(),
        fls=dataclasses.replace(
            _slot("fls-frame-0001", "fls_sonar", 0), clock_domain="utc"
        ),
        absent=_except_fls(),
    )
    self._expect_error(bundle, _Error.CLOCK_DOMAIN)
    empty_slot_clock = dataclasses.replace(
        bundle,
        fls=dataclasses.replace(bundle.fls, clock_domain=""),
    )
    self._expect_error(empty_slot_clock, _Error.CLOCK_DOMAIN)
    both_empty = dataclasses.replace(empty_slot_clock, clock_domain="")
    self._expect_accepted(both_empty)
    state_mismatch = dataclasses.replace(
        both_empty,
        vehicle_state=dataclasses.replace(
            _state("body", 0), clock_domain="monotonic"
        ),
        absent=_except_fls()[:3],
    )
    self._expect_error(state_mismatch, _Error.CLOCK_DOMAIN)

  def test_max_skew_and_missing_frame_order(self):
    skew_only = policy.ObservationBundleView(
        max_skew_present=True, max_skew=(1, 0)
    )
    assessment = _assess(skew_only)
    self.assertEqual(assessment.error, _Error.MISSING_FRAME)
    self.assertFalse(assessment.accepted)

    bundle = dataclasses.replace(_base(), max_skew_present=False)
    self._expect_error(bundle, _Error.MAX_SKEW)
    self._expect_error(
        dataclasses.replace(_base(), max_skew=(-1, 0)), _Error.MAX_SKEW
    )
    self._expect_error(
        dataclasses.replace(_base(), max_skew=(0, -1)), _Error.MAX_SKEW
    )
    self._expect_error(
        dataclasses.replace(_base(), max_skew=(0, 1000000000)), _Error.MAX_SKEW
    )
    missing_frame = dataclasses.replace(
        _base(),
        frame_id="",
        fls=_slot("fls-frame-0001", "fls_sonar", _SKEW_NANOS + 1),
    )
    self._expect_error(missing_frame, _Error.MISSING_FRAME)

  def test_first_defect_wins(self):
    bundle = dataclasses.replace(
        _base(),
        fls=dataclasses.replace(
            _slot("fls-frame-0001", "fls_sonar", 0), frame_id="", byte_size=0
        ),
        absent=_except_fls(),
    )
    self._expect_error(bundle, _Error.SLOT)
    reference = dataclasses.replace(
        bundle,
        fls=dataclasses.replace(
            _slot("fls-frame-0001", "fls_sonar", 0),
            byte_size=0,
            clock_domain="utc",
        ),
    )
    self._expect_error(reference, _Error.REFERENCE)
    without_optical = (
        _except_fls()[0],
        _except_fls()[2],
        _except_fls()[3],
    )
    clock = dataclasses.replace(
        reference,
        absent=without_optical,
        fls=dataclasses.replace(
            _slot("fls-frame-0001", frame_policy.WORLD_ENU_FRAME_ID, 0),
            clock_domain="utc",
        ),
        optical=_slot("optical-frame-0001", frame_policy.WORLD_NED_FRAME_ID, 0),
    )
    self._expect_error(clock, _Error.CLOCK_DOMAIN)
    frames = dataclasses.replace(
        clock,
        fls=dataclasses.replace(
            clock.fls,
            clock_domain="monotonic",
            source_time=(_SECONDS, 1),
            receive_time=(_SECONDS, 0),
        ),
        optical=dataclasses.replace(
            clock.optical,
            source_time=(_SECONDS, _SKEW_NANOS + 5),
            receive_time=(_SECONDS, _SKEW_NANOS + 5),
        ),
    )
    self._expect_error(frames, _Error.FRAME_MISMATCH)
    reversal = dataclasses.replace(
        frames,
        optical=dataclasses.replace(
            frames.optical, frame_id=frame_policy.WORLD_ENU_FRAME_ID
        ),
    )
    self._expect_error(reversal, _Error.TIME_REVERSAL)

  def test_half_filled_slot_is_not_present(self):
    bundle = dataclasses.replace(
        _base(),
        fls=policy.ObservationSlotView(
            frame_id="fls_sonar",
            source_time_present=True,
            source_time=(_SECONDS, 0),
            clock_domain="utc",
        ),
        absent=_except_fls(),
    )
    self._expect_accepted(bundle)
    self.assertFalse(policy.slot_present(bundle.fls))

  def test_state_without_frame_is_not_present(self):
    bundle = dataclasses.replace(
        _base(),
        fls=_slot("fls-frame-0001", "fls_sonar", 0),
        vehicle_state=policy.StateReferenceView(
            message_set=True,
            world_snapshot_id="not-a-hex",
            state_epoch=9,
        ),
        absent=_except_fls(),
    )
    self._expect_accepted(bundle)
    self.assertFalse(policy.state_present(bundle.vehicle_state))

  def test_validity_does_not_repair_or_replace_structure(self):
    bundle = dataclasses.replace(
        _base(),
        fls=_slot("fls-frame-0001", "fls_sonar", 0),
        absent=_except_fls(),
        validity_present=False,
    )
    absent_validity = _assess(bundle)
    self.assertEqual(absent_validity.error, _Error.NONE)
    self.assertEqual(absent_validity.validity, _Kind.ABSENT)
    self.assertFalse(absent_validity.accepted)

    invalid = _assess(
        dataclasses.replace(bundle, validity_present=True, validity_state=2)
    )
    self.assertEqual(invalid.error, _Error.NONE)
    self.assertEqual(invalid.validity, _Kind.INVALID)
    self.assertFalse(invalid.accepted)

    unknown = _assess(
        dataclasses.replace(bundle, validity_present=True, validity_state=99)
    )
    self.assertEqual(unknown.validity, _Kind.UNSPECIFIED)
    self.assertFalse(unknown.accepted)

    structural = _assess(
        dataclasses.replace(
            _base(),
            fls=dataclasses.replace(
                _slot("fls-frame-0001", "fls_sonar", 0), byte_size=0
            ),
            absent=_except_fls(),
        )
    )
    self.assertEqual(structural.error, _Error.REFERENCE)
    self.assertEqual(structural.validity, _Kind.VALID)
    self.assertFalse(structural.accepted)

  def test_determinism(self):
    first = dataclasses.replace(
        _base(),
        fls=_slot("fls-frame-0001", "fls_sonar", 0),
        absent=_except_fls(),
        metadata_present=True,
    )
    left = _assess(first)
    right = _assess(first)
    self.assertEqual(left, right)
    self.assertTrue(left.accepted)
    self.assertEqual(_assess(dataclasses.replace(first)), left)

  def test_zero_max_skew_alone_does_not_engage(self):
    bundle = policy.ObservationBundleView(
        max_skew_present=True, max_skew=(0, 0)
    )
    self.assertFalse(policy.observation_bundle_engaged(bundle))
    assessment = _assess(bundle)
    self.assertFalse(assessment.accepted)
    self.assertEqual(assessment.error, _Error.NONE)

  def test_point_cloud_only_and_state_only(self):
    points = dataclasses.replace(
        _base(),
        point_cloud=dataclasses.replace(
            _slot("points-0001", "points", 0), byte_size_present=False
        ),
        absent=(
            _entry(_Modality.SONAR_FLS, _Reason.NOT_CONFIGURED),
            _entry(_Modality.SONAR_SSS, _Reason.NOT_CONFIGURED),
            _entry(_Modality.OPTICAL, _Reason.SENSOR_OFFLINE),
            _entry(_Modality.VEHICLE_STATE, _Reason.INTENTIONALLY_OMITTED),
        ),
    )
    self._expect_accepted(points)
    state_only = dataclasses.replace(
        _base(),
        vehicle_state=dataclasses.replace(
            _state("body", 0, _SNAPSHOT), state_epoch=7
        ),
        absent=(
            _entry(_Modality.SONAR_FLS, _Reason.NOT_CONFIGURED),
            _entry(_Modality.SONAR_SSS, _Reason.SENSOR_OFFLINE),
            _entry(_Modality.OPTICAL, _Reason.OUT_OF_RANGE),
            _entry(_Modality.POINT_CLOUD, _Reason.DROPPED_FOR_SKEW),
        ),
    )
    self._expect_accepted(state_only)

  def test_metadata_alone_is_engaged(self):
    assessment = _assess(policy.ObservationBundleView(metadata_present=True))
    self.assertEqual(assessment.error, _Error.MISSING_FRAME)
    self.assertEqual(assessment.validity, _Kind.ABSENT)
    self.assertFalse(assessment.accepted)

  def test_state_time_participates_in_skew(self):
    bundle = dataclasses.replace(
        _base(),
        fls=_slot("fls-frame-0001", "fls_sonar", 0),
        vehicle_state=_state("body", _SKEW_NANOS),
        absent=(
            _entry(_Modality.SONAR_SSS, _Reason.NOT_CONFIGURED),
            _entry(_Modality.OPTICAL, _Reason.SENSOR_OFFLINE),
            _entry(_Modality.POINT_CLOUD, _Reason.OUT_OF_RANGE),
        ),
    )
    assessment = self._expect_accepted(bundle)
    self.assertEqual(assessment.measured_skew, (0, _SKEW_NANOS))
    outside = dataclasses.replace(
        bundle,
        vehicle_state=dataclasses.replace(
            bundle.vehicle_state,
            source_time=(_SECONDS, _SKEW_NANOS + 1),
            receive_time=(_SECONDS, _SKEW_NANOS + 1),
        ),
    )
    self._expect_error(outside, _Error.EXCESSIVE_SKEW)

  def test_absent_overlap_beats_a_bad_slot(self):
    bundle = dataclasses.replace(
        _base(),
        fls=_slot("fls-frame-0001", "", 0),
        absent=(_entry(_Modality.SONAR_FLS, _Reason.NOT_CONFIGURED),),
    )
    self._expect_error(bundle, _Error.ABSENT_LIST)

  def test_absent_entries_helper_copies_pairs(self):
    copied = policy.absent_entries(((1, 1), (3, 2)))
    self.assertEqual(
        copied,
        (
            policy.AbsentModalityEntryView(modality=1, reason=1),
            policy.AbsentModalityEntryView(modality=3, reason=2),
        ),
    )


if __name__ == "__main__":
  unittest.main()
