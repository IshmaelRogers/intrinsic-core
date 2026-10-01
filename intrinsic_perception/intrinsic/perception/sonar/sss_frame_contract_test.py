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

"""Validation tests for the side-scan sonar frame contract."""

import dataclasses
import math
import unittest

from intrinsic.embodiment import stamped_header_policy
from intrinsic.perception.sonar import sss_frame_contract_policy as policy

_Error = policy.SssFrameContractError
_Kind = stamped_header_policy.ValidityKind
_NAN = float("nan")
_INF = float("inf")


def _channel(**overrides):
  values = dict(num_samples=16, min_range_m=1.0, max_range_m=75.0)
  values.update(overrides)
  return policy.SssChannelGeometryView(**values)


def _geometry(**overrides):
  values = dict(port=_channel(), starboard=_channel())
  values.update(overrides)
  return policy.SssGeometryView(**values)


def _calibration(**overrides):
  values = dict(
      present=True,
      parent_frame_id="body",
      sensor_frame_id="sss_sonar",
      pose_present=True,
      position=(0.75, 0.0, -0.5),
      orientation=(0.0, 0.0, 0.0, 1.0),
  )
  values.update(overrides)
  return policy.SssCalibrationView(**values)


def _inline(**overrides):
  values = dict(
      header_present=True,
      validity_present=True,
      validity_state=1,
      frame_id="sss_sonar",
      geometry=_geometry(),
      calibration=_calibration(),
      sound_speed_m_s=1500.0,
      payload_mode=policy.SssPayloadMode.INLINE,
      port_intensity=tuple(0.125 * i for i in range(16)),
      starboard_intensity=tuple(0.25 + 0.125 * i for i in range(16)),
      metadata_present=True,
  )
  values.update(overrides)
  return policy.SssFrameView(**values)


def _blob(**overrides):
  values = dict(
      payload_mode=policy.SssPayloadMode.BLOB,
      port_intensity=(),
      starboard_intensity=(),
      blob_id="sss-blob-0001",
      blob_byte_size_present=True,
      blob_byte_size=256,
  )
  values.update(overrides)
  return _inline(**values)


def _error_of(frame):
  return policy.assess_side_scan_sonar_frame(frame).error


class SssFrameContractTest(unittest.TestCase):

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
    empty = policy.SssFrameView()
    assessment = policy.assess_side_scan_sonar_frame(empty)
    self.assertIs(assessment.error, _Error.NONE)
    self.assertIs(assessment.validity, _Kind.ABSENT)
    self.assertFalse(assessment.accepted)
    self.assertFalse(policy.sss_frame_engaged(empty))

  def test_each_field_engages_the_frame(self):
    channel_fields = ("num_samples", "min_range_m", "max_range_m")
    engaged = [policy.SssFrameView(header_present=True)]
    for side in ("port", "starboard"):
      for name in channel_fields:
        engaged.append(
            policy.SssFrameView(
                geometry=policy.SssGeometryView(
                    **{side: policy.SssChannelGeometryView(**{name: 1})}
                )
            )
        )
    engaged += [
        policy.SssFrameView(sound_speed_m_s=1500.0),
        policy.SssFrameView(sound_speed_m_s=_NAN),
        policy.SssFrameView(
            calibration=policy.SssCalibrationView(present=True)
        ),
        policy.SssFrameView(payload_mode=policy.SssPayloadMode.INLINE),
        policy.SssFrameView(payload_mode=policy.SssPayloadMode.BLOB),
        policy.SssFrameView(metadata_present=True),
    ]
    for frame in engaged:
      with self.subTest(frame=frame):
        self.assertTrue(policy.sss_frame_engaged(frame))
        assessment = policy.assess_side_scan_sonar_frame(frame)
        self.assertIsNot(assessment.error, _Error.NONE)
        self.assertFalse(assessment.accepted)

  def test_nominal_inline_and_blob_are_accepted(self):
    for frame in (_inline(), _blob()):
      assessment = policy.assess_side_scan_sonar_frame(frame)
      self.assertIs(assessment.error, _Error.NONE)
      self.assertIs(assessment.validity, _Kind.VALID)
      self.assertTrue(assessment.accepted)

  def test_calibration_and_metadata_are_optional(self):
    for build in (_inline, _blob):
      bare = build(
          calibration=policy.SssCalibrationView(), metadata_present=False
      )
      self.assertTrue(policy.assess_side_scan_sonar_frame(bare).accepted)

  def test_metadata_never_changes_the_assessment(self):
    with_metadata = policy.assess_side_scan_sonar_frame(_inline())
    without = policy.assess_side_scan_sonar_frame(
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
        assessment = policy.assess_side_scan_sonar_frame(
            _inline(validity_present=present, validity_state=state)
        )
        self.assertIs(assessment.error, _Error.NONE)
        self.assertIs(assessment.validity, kind)
        self.assertEqual(assessment.accepted, accepted)
    defective = policy.assess_side_scan_sonar_frame(
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

  def test_bad_geometry_on_either_channel(self):
    bad_channels = (
        dict(num_samples=0),
        dict(min_range_m=-0.5),
        dict(min_range_m=75.0),
        dict(min_range_m=80.0),
        dict(min_range_m=_NAN),
        dict(min_range_m=_INF),
        dict(min_range_m=-_INF),
        dict(max_range_m=_NAN),
        dict(max_range_m=_INF),
        dict(max_range_m=1.0),
        dict(max_range_m=0.0),
    )
    for side in ("port", "starboard"):
      for overrides in bad_channels:
        with self.subTest(side=side, overrides=overrides):
          frame = _inline(geometry=_geometry(**{side: _channel(**overrides)}))
          self.assertIs(_error_of(frame), _Error.GEOMETRY)

  def test_missing_channel_is_a_geometry_defect(self):
    empty = policy.SssChannelGeometryView()
    for geometry in (
        _geometry(port=empty),
        _geometry(starboard=empty),
        policy.SssGeometryView(),
    ):
      with self.subTest(geometry=geometry):
        self.assertIs(_error_of(_inline(geometry=geometry)), _Error.GEOMETRY)

  def test_geometry_boundaries_are_accepted(self):
    geometry = _geometry(
        port=_channel(num_samples=1, min_range_m=0.0, max_range_m=1e-9),
        starboard=_channel(num_samples=1, min_range_m=0.0, max_range_m=1e9),
    )
    frame = _inline(
        geometry=geometry, port_intensity=(0.5,), starboard_intensity=(0.5,)
    )
    self.assertTrue(policy.assess_side_scan_sonar_frame(frame).accepted)

  def test_channels_may_differ(self):
    geometry = _geometry(
        port=_channel(num_samples=3), starboard=_channel(num_samples=5)
    )
    frame = _inline(
        geometry=geometry,
        port_intensity=(0.0, 1.0, 2.0),
        starboard_intensity=(0.0, 1.0, 2.0, 3.0, 4.0),
    )
    self.assertTrue(policy.assess_side_scan_sonar_frame(frame).accepted)
    swapped = dataclasses.replace(
        frame,
        port_intensity=frame.starboard_intensity,
        starboard_intensity=frame.port_intensity,
    )
    self.assertIs(_error_of(swapped), _Error.PAYLOAD_MISMATCH)

  def test_bad_sound_speed(self):
    for value in (0.0, -1.0, -1500.0, _NAN, _INF, -_INF):
      with self.subTest(value=value):
        self.assertIs(
            _error_of(_inline(sound_speed_m_s=value)), _Error.SOUND_SPEED
        )
        self.assertIs(
            _error_of(_blob(sound_speed_m_s=value)), _Error.SOUND_SPEED
        )
    self.assertTrue(
        policy.assess_side_scan_sonar_frame(
            _inline(sound_speed_m_s=1e-300)
        ).accepted
    )

  def test_missing_payload(self):
    frame = _inline(payload_mode=policy.SssPayloadMode.NONE)
    self.assertIs(_error_of(frame), _Error.MISSING_PAYLOAD)

  def test_inline_size_mismatch_on_either_channel(self):
    ramp = tuple(0.5 for _ in range(16))
    cases = (
        dict(port_intensity=ramp[:15]),
        dict(port_intensity=ramp + (0.5,)),
        dict(port_intensity=()),
        dict(starboard_intensity=ramp[:15]),
        dict(starboard_intensity=ramp + (0.5,)),
        dict(starboard_intensity=()),
        dict(port_intensity=(), starboard_intensity=()),
    )
    for overrides in cases:
      with self.subTest(overrides=overrides):
        self.assertIs(_error_of(_inline(**overrides)), _Error.PAYLOAD_MISMATCH)

  def test_inline_sizes_are_not_pooled(self):
    frame = _inline(
        port_intensity=tuple(0.0 for _ in range(15)),
        starboard_intensity=tuple(0.0 for _ in range(17)),
    )
    self.assertIs(_error_of(frame), _Error.PAYLOAD_MISMATCH)

  def test_inline_non_finite_samples(self):
    good = tuple(0.5 for _ in range(16))
    for value in (_NAN, _INF, -_INF):
      for index in (0, 15):
        with self.subTest(value=value, index=index):
          bad = good[:index] + (value,) + good[index + 1 :]
          self.assertIs(
              _error_of(_inline(port_intensity=bad)), _Error.NON_FINITE
          )
          self.assertIs(
              _error_of(_inline(starboard_intensity=bad)), _Error.NON_FINITE
          )

  def test_size_mismatch_wins_over_non_finite_samples(self):
    frame = _inline(
        port_intensity=(_NAN,) * 16, starboard_intensity=(0.0,) * 15
    )
    self.assertIs(_error_of(frame), _Error.PAYLOAD_MISMATCH)
    frame = _inline(
        port_intensity=(0.0,) * 15, starboard_intensity=(_NAN,) * 16
    )
    self.assertIs(_error_of(frame), _Error.PAYLOAD_MISMATCH)

  def test_zero_and_negative_intensity_are_finite(self):
    frame = _inline(
        port_intensity=(0.0, -0.0) + (-3.5,) * 14,
        starboard_intensity=(0.0,) * 16,
    )
    self.assertTrue(policy.assess_side_scan_sonar_frame(frame).accepted)

  def test_blob_reference(self):
    self.assertIs(_error_of(_blob(blob_id="")), _Error.BLOB_REFERENCE)
    self.assertIs(
        _error_of(_blob(blob_byte_size_present=True, blob_byte_size=0)),
        _Error.BLOB_REFERENCE,
    )
    self.assertTrue(
        policy.assess_side_scan_sonar_frame(
            _blob(blob_byte_size_present=False, blob_byte_size=0)
        ).accepted
    )
    self.assertTrue(
        policy.assess_side_scan_sonar_frame(
            _blob(blob_byte_size_present=True, blob_byte_size=1)
        ).accepted
    )

  def test_blob_mode_ignores_stale_inline_values(self):
    frame = _blob(port_intensity=(_NAN,), starboard_intensity=())
    self.assertTrue(policy.assess_side_scan_sonar_frame(frame).accepted)

  def test_inline_mode_ignores_stale_blob_values(self):
    frame = _inline(blob_id="", blob_byte_size_present=True, blob_byte_size=0)
    self.assertTrue(policy.assess_side_scan_sonar_frame(frame).accepted)

  def test_bad_calibration_frames(self):
    cases = (
        dict(parent_frame_id=""),
        dict(sensor_frame_id=""),
        dict(parent_frame_id="x", sensor_frame_id="x"),
        dict(parent_frame_id="sss_sonar"),
    )
    for overrides in cases:
      with self.subTest(overrides=overrides):
        frame = _inline(calibration=_calibration(**overrides))
        self.assertIs(_error_of(frame), _Error.CALIBRATION)
        frame = _blob(calibration=_calibration(**overrides))
        self.assertIs(_error_of(frame), _Error.CALIBRATION)

  def test_unset_calibration_pose_is_a_calibration_defect(self):
    frame = _inline(calibration=_calibration(pose_present=False))
    self.assertIs(_error_of(frame), _Error.CALIBRATION)
    self.assertIs(
        _error_of(_blob(calibration=_calibration(pose_present=False))),
        _Error.CALIBRATION,
    )

  def test_unset_pose_wins_over_pose_values(self):
    frame = _inline(
        calibration=_calibration(
            pose_present=False,
            position=(_NAN, 0.0, 0.0),
            orientation=(0.0, 0.0, 0.0, 0.0),
        )
    )
    self.assertIs(_error_of(frame), _Error.CALIBRATION)

  def test_present_empty_calibration_is_rejected(self):
    frame = _inline(calibration=policy.SssCalibrationView(present=True))
    self.assertIs(_error_of(frame), _Error.CALIBRATION)

  def test_calibration_non_finite(self):
    for position in (
        (_NAN, 0.0, 0.0),
        (0.0, _INF, 0.0),
        (0.0, 0.0, -_INF),
    ):
      with self.subTest(position=position):
        frame = _inline(calibration=_calibration(position=position))
        self.assertIs(_error_of(frame), _Error.NON_FINITE)
    for orientation in (
        (_NAN, 0.0, 0.0, 1.0),
        (0.0, 0.0, 0.0, _INF),
    ):
      with self.subTest(orientation=orientation):
        frame = _inline(calibration=_calibration(orientation=orientation))
        self.assertIs(_error_of(frame), _Error.NON_FINITE)

  def test_calibration_quaternion(self):
    half = math.sqrt(0.5)
    for orientation in ((half, 0.0, 0.0, half), (0.0, 0.0, 0.0, -1.0)):
      with self.subTest(orientation=orientation):
        frame = _inline(calibration=_calibration(orientation=orientation))
        self.assertIs(_error_of(frame), _Error.NONE)
    for orientation in (
        (0.0, 0.0, 0.0, 0.0),
        (0.0, 0.0, 0.0, 2.0),
        (1.0, 1.0, 0.0, 0.0),
        (0.0, 0.0, 0.0, 0.5),
    ):
      with self.subTest(orientation=orientation):
        frame = _inline(calibration=_calibration(orientation=orientation))
        self.assertIs(_error_of(frame), _Error.QUATERNION)

  def test_first_defect_wins(self):
    frame = _inline(
        frame_id="",
        geometry=policy.SssGeometryView(),
        sound_speed_m_s=0.0,
        payload_mode=policy.SssPayloadMode.NONE,
        calibration=_calibration(parent_frame_id=""),
    )
    self.assertIs(_error_of(frame), _Error.MISSING_FRAME)
    frame = dataclasses.replace(frame, frame_id="sss_sonar")
    self.assertIs(_error_of(frame), _Error.GEOMETRY)
    frame = dataclasses.replace(frame, geometry=_geometry())
    self.assertIs(_error_of(frame), _Error.SOUND_SPEED)
    frame = dataclasses.replace(frame, sound_speed_m_s=1500.0)
    self.assertIs(_error_of(frame), _Error.MISSING_PAYLOAD)
    frame = dataclasses.replace(
        frame,
        payload_mode=policy.SssPayloadMode.BLOB,
        blob_id="",
    )
    self.assertIs(_error_of(frame), _Error.BLOB_REFERENCE)
    frame = dataclasses.replace(frame, blob_id="sss-blob-0001")
    self.assertIs(_error_of(frame), _Error.CALIBRATION)
    frame = dataclasses.replace(frame, calibration=_calibration())
    self.assertIs(_error_of(frame), _Error.NONE)

  def test_payload_defect_wins_over_calibration_defect(self):
    frame = _inline(
        port_intensity=(), calibration=_calibration(pose_present=False)
    )
    self.assertIs(_error_of(frame), _Error.PAYLOAD_MISMATCH)

  def test_assessment_is_deterministic(self):
    frame = _inline()
    self.assertEqual(
        policy.assess_side_scan_sonar_frame(frame),
        policy.assess_side_scan_sonar_frame(frame),
    )


if __name__ == "__main__":
  unittest.main()
