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

"""Validation tests for the forward-looking sonar frame contract."""

import dataclasses
import math
import unittest

from intrinsic.embodiment import stamped_header_policy
from intrinsic.perception.sonar import fls_frame_contract_policy as policy

_Error = policy.FlsFrameContractError
_Kind = stamped_header_policy.ValidityKind
_NAN = float("nan")
_INF = float("inf")


def _geometry(**overrides):
  values = dict(
      num_beams=4,
      num_range_bins=8,
      min_range_m=0.5,
      max_range_m=40.0,
      min_bearing_rad=-0.5,
      max_bearing_rad=0.5,
  )
  values.update(overrides)
  return policy.FlsGeometryView(**values)


def _calibration(**overrides):
  values = dict(
      present=True,
      parent_frame_id="body",
      sensor_frame_id="fls_sonar",
      pose_present=True,
      position=(1.5, 0.0, -0.25),
      orientation=(0.0, 0.0, 0.0, 1.0),
  )
  values.update(overrides)
  return policy.FlsCalibrationView(**values)


def _inline(**overrides):
  values = dict(
      header_present=True,
      validity_present=True,
      validity_state=1,
      frame_id="fls_sonar",
      geometry=_geometry(),
      calibration=_calibration(),
      sound_speed_m_s=1500.0,
      payload_mode=policy.FlsPayloadMode.INLINE,
      intensity=tuple(0.125 * i for i in range(32)),
      metadata_present=True,
  )
  values.update(overrides)
  return policy.FlsFrameView(**values)


def _blob(**overrides):
  values = dict(
      payload_mode=policy.FlsPayloadMode.BLOB,
      intensity=(),
      blob_id="fls-blob-0001",
      blob_byte_size_present=True,
      blob_byte_size=128,
  )
  values.update(overrides)
  return _inline(**values)


def _error_of(frame):
  return policy.assess_forward_looking_sonar_frame(frame).error


class FlsFrameContractTest(unittest.TestCase):

  def test_error_values_are_locked(self):
    self.assertEqual(
        {error.name: error.value for error in _Error},
        {
            "NONE": 0,
            "MISSING_FRAME": 1,
            "GEOMETRY": 2,
            "SOUND_SPEED": 3,
            "CALIBRATION": 4,
            "MISSING_PAYLOAD": 5,
            "PAYLOAD_MISMATCH": 6,
            "NON_FINITE": 7,
            "BLOB_REFERENCE": 8,
            "QUATERNION": 9,
        },
    )

  def test_empty_frame_is_not_engaged(self):
    empty = policy.FlsFrameView()
    assessment = policy.assess_forward_looking_sonar_frame(empty)
    self.assertIs(assessment.error, _Error.NONE)
    self.assertIs(assessment.validity, _Kind.ABSENT)
    self.assertFalse(assessment.accepted)
    self.assertFalse(policy.fls_frame_engaged(empty))

  def test_each_field_engages_the_frame(self):
    engaged = [
        policy.FlsFrameView(header_present=True),
        policy.FlsFrameView(geometry=policy.FlsGeometryView(num_beams=1)),
        policy.FlsFrameView(geometry=policy.FlsGeometryView(num_range_bins=1)),
        policy.FlsFrameView(geometry=policy.FlsGeometryView(min_range_m=1.0)),
        policy.FlsFrameView(geometry=policy.FlsGeometryView(max_range_m=1.0)),
        policy.FlsFrameView(
            geometry=policy.FlsGeometryView(min_bearing_rad=-1.0)
        ),
        policy.FlsFrameView(
            geometry=policy.FlsGeometryView(max_bearing_rad=1.0)
        ),
        policy.FlsFrameView(sound_speed_m_s=1500.0),
        policy.FlsFrameView(sound_speed_m_s=_NAN),
        policy.FlsFrameView(
            calibration=policy.FlsCalibrationView(present=True)
        ),
        policy.FlsFrameView(payload_mode=policy.FlsPayloadMode.INLINE),
        policy.FlsFrameView(payload_mode=policy.FlsPayloadMode.BLOB),
        policy.FlsFrameView(metadata_present=True),
    ]
    for frame in engaged:
      with self.subTest(frame=frame):
        self.assertTrue(policy.fls_frame_engaged(frame))
        assessment = policy.assess_forward_looking_sonar_frame(frame)
        self.assertIsNot(assessment.error, _Error.NONE)
        self.assertFalse(assessment.accepted)

  def test_nominal_inline_and_blob_are_accepted(self):
    for frame in (_inline(), _blob()):
      assessment = policy.assess_forward_looking_sonar_frame(frame)
      self.assertIs(assessment.error, _Error.NONE)
      self.assertIs(assessment.validity, _Kind.VALID)
      self.assertTrue(assessment.accepted)

  def test_calibration_and_metadata_are_optional(self):
    bare = _inline(
        calibration=policy.FlsCalibrationView(), metadata_present=False
    )
    self.assertTrue(policy.assess_forward_looking_sonar_frame(bare).accepted)
    bare_blob = _blob(
        calibration=policy.FlsCalibrationView(), metadata_present=False
    )
    self.assertTrue(
        policy.assess_forward_looking_sonar_frame(bare_blob).accepted
    )

  def test_metadata_never_changes_the_assessment(self):
    with_metadata = policy.assess_forward_looking_sonar_frame(_inline())
    without = policy.assess_forward_looking_sonar_frame(
        _inline(metadata_present=False)
    )
    self.assertEqual(with_metadata, without)
    defective = _inline(sound_speed_m_s=-1.0)
    self.assertIs(_error_of(defective), _Error.SOUND_SPEED)
    self.assertIs(
        _error_of(dataclasses.replace(defective, metadata_present=False)),
        _Error.SOUND_SPEED,
    )

  def test_validity_is_classified_and_does_not_repair_defects(self):
    cases = (
        (False, 0, _Kind.ABSENT, False),
        (True, 0, _Kind.UNSPECIFIED, False),
        (True, 1, _Kind.VALID, True),
        (True, 2, _Kind.INVALID, False),
        (True, 99, _Kind.UNSPECIFIED, False),
    )
    for present, state, kind, accepted in cases:
      with self.subTest(state=state, present=present):
        assessment = policy.assess_forward_looking_sonar_frame(
            _inline(validity_present=present, validity_state=state)
        )
        self.assertIs(assessment.error, _Error.NONE)
        self.assertIs(assessment.validity, kind)
        self.assertEqual(assessment.accepted, accepted)
    defective = policy.assess_forward_looking_sonar_frame(
        _inline(sound_speed_m_s=0.0)
    )
    self.assertIs(defective.validity, _Kind.VALID)
    self.assertIs(defective.error, _Error.SOUND_SPEED)
    self.assertFalse(defective.accepted)

  def test_missing_frame(self):
    self.assertIs(_error_of(_inline(frame_id="")), _Error.MISSING_FRAME)
    self.assertIs(_error_of(_blob(frame_id="")), _Error.MISSING_FRAME)
    self.assertIs(
        _error_of(_inline(header_present=False, frame_id="")),
        _Error.MISSING_FRAME,
    )
    for frame_id in ("world_enu", "world_ned", "anything"):
      self.assertIs(_error_of(_inline(frame_id=frame_id)), _Error.NONE)

  def test_bad_geometry(self):
    bad = (
        dict(num_beams=0),
        dict(num_range_bins=0),
        dict(min_range_m=-0.001),
        dict(min_range_m=_NAN),
        dict(min_range_m=-_INF),
        dict(max_range_m=_NAN),
        dict(max_range_m=_INF),
        dict(max_range_m=0.5),
        dict(max_range_m=0.25),
        dict(min_bearing_rad=_NAN),
        dict(min_bearing_rad=-_INF),
        dict(max_bearing_rad=_NAN),
        dict(max_bearing_rad=_INF),
        dict(min_bearing_rad=0.6),
    )
    for overrides in bad:
      with self.subTest(overrides=overrides):
        self.assertIs(
            _error_of(_blob(geometry=_geometry(**overrides))), _Error.GEOMETRY
        )

  def test_geometry_boundaries_are_accepted(self):
    good = (
        dict(min_range_m=0.0),
        dict(min_bearing_rad=0.5),
        dict(min_bearing_rad=0.0, max_bearing_rad=0.0),
        dict(max_range_m=0.5000001),
    )
    for overrides in good:
      with self.subTest(overrides=overrides):
        self.assertIs(
            _error_of(_blob(geometry=_geometry(**overrides))), _Error.NONE
        )

  def test_bad_sound_speed(self):
    for value in (0.0, -0.0, -1500.0, -1e-300, _NAN, _INF, -_INF):
      with self.subTest(value=value):
        self.assertIs(
            _error_of(_inline(sound_speed_m_s=value)), _Error.SOUND_SPEED
        )
        self.assertIs(
            _error_of(_blob(sound_speed_m_s=value)), _Error.SOUND_SPEED
        )
    self.assertIs(_error_of(_inline(sound_speed_m_s=1e-9)), _Error.NONE)
    self.assertIs(_error_of(_inline(sound_speed_m_s=1e9)), _Error.NONE)

  def test_missing_payload(self):
    frame = _inline(
        payload_mode=policy.FlsPayloadMode.NONE, intensity=(), blob_id=""
    )
    self.assertIs(_error_of(frame), _Error.MISSING_PAYLOAD)

  def test_inline_size_mismatch(self):
    samples = tuple(0.125 * i for i in range(32))
    for size in (0, 1, 8, 31, 33, 64):
      with self.subTest(size=size):
        frame = _inline(intensity=(samples * 3)[:size])
        self.assertIs(_error_of(frame), _Error.PAYLOAD_MISMATCH)

  def test_inline_empty_payload_is_a_mismatch(self):
    self.assertIs(_error_of(_inline(intensity=())), _Error.PAYLOAD_MISMATCH)

  def test_inline_single_sample_grid(self):
    geometry = _geometry(num_beams=1, num_range_bins=1)
    self.assertIs(
        _error_of(_inline(geometry=geometry, intensity=(0.0,))), _Error.NONE
    )
    self.assertIs(
        _error_of(_inline(geometry=geometry, intensity=(0.0, 0.0))),
        _Error.PAYLOAD_MISMATCH,
    )

  def test_inline_layout_uses_the_product_not_the_sum(self):
    geometry = _geometry(num_beams=3, num_range_bins=5)
    self.assertIs(
        _error_of(_inline(geometry=geometry, intensity=(0.0,) * 15)),
        _Error.NONE,
    )
    self.assertIs(
        _error_of(_inline(geometry=geometry, intensity=(0.0,) * 8)),
        _Error.PAYLOAD_MISMATCH,
    )

  def test_inline_huge_dimensions_do_not_overflow(self):
    huge = _geometry(num_beams=2**32 - 1, num_range_bins=2**32 - 1)
    self.assertIs(
        _error_of(_inline(geometry=huge, intensity=(0.0,) * 32)),
        _Error.PAYLOAD_MISMATCH,
    )
    overflow = _geometry(num_beams=2**33, num_range_bins=2**33)
    self.assertIsNone(policy.expected_sample_count(overflow))
    self.assertIs(
        _error_of(_inline(geometry=overflow, intensity=(0.0,) * 32)),
        _Error.PAYLOAD_MISMATCH,
    )

  def test_inline_non_finite_samples(self):
    for bad in (_NAN, _INF, -_INF):
      for index in (0, 17, 31):
        with self.subTest(bad=bad, index=index):
          samples = [0.5] * 32
          samples[index] = bad
          self.assertIs(
              _error_of(_inline(intensity=tuple(samples))), _Error.NON_FINITE
          )

  def test_size_mismatch_wins_over_non_finite_samples(self):
    samples = tuple([_NAN] * 31)
    self.assertIs(
        _error_of(_inline(intensity=samples)), _Error.PAYLOAD_MISMATCH
    )

  def test_zero_intensity_is_finite(self):
    self.assertIs(_error_of(_inline(intensity=(0.0,) * 32)), _Error.NONE)
    self.assertIs(_error_of(_inline(intensity=(-1.0,) * 32)), _Error.NONE)

  def test_blob_reference(self):
    self.assertIs(_error_of(_blob(blob_id="")), _Error.BLOB_REFERENCE)
    self.assertIs(
        _error_of(_blob(blob_byte_size_present=True, blob_byte_size=0)),
        _Error.BLOB_REFERENCE,
    )
    self.assertIs(
        _error_of(_blob(blob_byte_size_present=False, blob_byte_size=0)),
        _Error.NONE,
    )
    self.assertIs(
        _error_of(_blob(blob_byte_size_present=True, blob_byte_size=1)),
        _Error.NONE,
    )
    self.assertIs(
        _error_of(_blob(blob_byte_size_present=True, blob_byte_size=2**64 - 1)),
        _Error.NONE,
    )

  def test_blob_mode_ignores_stale_inline_values(self):
    frame = _blob(intensity=(_NAN,))
    self.assertIs(_error_of(frame), _Error.NONE)

  def test_inline_mode_ignores_stale_blob_values(self):
    frame = _inline(blob_id="", blob_byte_size_present=True, blob_byte_size=0)
    self.assertIs(_error_of(frame), _Error.NONE)

  def test_bad_calibration_frames(self):
    bad = (
        dict(parent_frame_id=""),
        dict(sensor_frame_id=""),
        dict(parent_frame_id="fls_sonar", sensor_frame_id="fls_sonar"),
        dict(pose_present=False),
    )
    for overrides in bad:
      with self.subTest(overrides=overrides):
        self.assertIs(
            _error_of(_inline(calibration=_calibration(**overrides))),
            _Error.CALIBRATION,
        )

  def test_present_empty_calibration_is_rejected(self):
    frame = _inline(calibration=policy.FlsCalibrationView(present=True))
    self.assertIs(_error_of(frame), _Error.CALIBRATION)

  def test_calibration_non_finite(self):
    bad = (
        dict(position=(_NAN, 0.0, 0.0)),
        dict(position=(0.0, _INF, 0.0)),
        dict(position=(0.0, 0.0, -_INF)),
        dict(orientation=(_NAN, 0.0, 0.0, 1.0)),
        dict(orientation=(0.0, 0.0, 0.0, _INF)),
    )
    for overrides in bad:
      with self.subTest(overrides=overrides):
        self.assertIs(
            _error_of(_inline(calibration=_calibration(**overrides))),
            _Error.NON_FINITE,
        )

  def test_calibration_quaternion(self):
    half = math.sqrt(2.0) / 2.0
    bad = (
        (0.0, 0.0, 0.0, 0.0),
        (0.0, 0.0, 0.0, 2.0),
        (1.0, 1.0, 1.0, 1.0),
        (0.0, 0.0, 0.0, 1.0 + 1e-6),
    )
    for orientation in bad:
      with self.subTest(orientation=orientation):
        self.assertIs(
            _error_of(
                _inline(calibration=_calibration(orientation=orientation))
            ),
            _Error.QUATERNION,
        )
    for orientation in (
        (0.0, 0.0, 0.0, 1.0),
        (0.0, 0.0, 0.0, -1.0),
        (half, 0.0, 0.0, half),
    ):
      with self.subTest(orientation=orientation):
        self.assertIs(
            _error_of(
                _inline(calibration=_calibration(orientation=orientation))
            ),
            _Error.NONE,
        )

  def test_calibration_is_checked_for_blob_mode_too(self):
    frame = _blob(calibration=_calibration(orientation=(0.0, 0.0, 0.0, 0.0)))
    self.assertIs(_error_of(frame), _Error.QUATERNION)

  def test_first_defect_wins(self):
    everything_wrong = dict(
        frame_id="",
        geometry=_geometry(num_beams=0),
        sound_speed_m_s=-1.0,
        payload_mode=policy.FlsPayloadMode.NONE,
        calibration=_calibration(parent_frame_id=""),
    )
    self.assertIs(_error_of(_inline(**everything_wrong)), _Error.MISSING_FRAME)
    everything_wrong.pop("frame_id")
    self.assertIs(_error_of(_inline(**everything_wrong)), _Error.GEOMETRY)
    everything_wrong.pop("geometry")
    self.assertIs(_error_of(_inline(**everything_wrong)), _Error.SOUND_SPEED)
    everything_wrong.pop("sound_speed_m_s")
    self.assertIs(
        _error_of(_inline(**everything_wrong)), _Error.MISSING_PAYLOAD
    )
    everything_wrong.pop("payload_mode")
    self.assertIs(
        _error_of(_inline(intensity=(), **everything_wrong)),
        _Error.PAYLOAD_MISMATCH,
    )
    self.assertIs(
        _error_of(_blob(blob_id="", **everything_wrong)), _Error.BLOB_REFERENCE
    )
    everything_wrong.pop("calibration")
    self.assertIs(
        _error_of(_inline(calibration=_calibration(parent_frame_id=""))),
        _Error.CALIBRATION,
    )

  def test_assessment_is_deterministic(self):
    first = policy.assess_forward_looking_sonar_frame(_inline())
    for _ in range(3):
      self.assertEqual(
          policy.assess_forward_looking_sonar_frame(_inline()), first
      )


if __name__ == "__main__":
  unittest.main()
