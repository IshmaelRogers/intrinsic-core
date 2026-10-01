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

"""Plain-value checks for ForwardLookingSonarFrame.

Policy: intrinsic_apis/intrinsic/perception/proto/sonar/README.md.

These helpers do not parse protobuf, do not fetch blobs, do not beamform or
detect, do not convert frames, and do not call World, ICON, or a HAL.
"""

from collections.abc import Sequence
from dataclasses import dataclass
import enum
import math

from intrinsic.embodiment import frame_policy
from intrinsic.embodiment import stamped_header_policy

_UINT64_MAX = 18446744073709551615


class FlsFrameContractError(enum.Enum):
  NONE = 0
  MISSING_FRAME = 1
  GEOMETRY = 2
  SOUND_SPEED = 3
  CALIBRATION = 4
  MISSING_PAYLOAD = 5
  PAYLOAD_MISMATCH = 6
  NON_FINITE = 7
  BLOB_REFERENCE = 8
  QUATERNION = 9


class FlsPayloadMode(enum.Enum):
  """Which arm of the payload oneof is set. NONE means the oneof is unset."""

  NONE = 0
  INLINE = 1
  BLOB = 2


@dataclass(frozen=True)
class FlsFrameContractAssessment:
  """accepted is true only for no error and STATE_VALID.

  An empty message is not accepted and is not an error.
  """

  error: FlsFrameContractError
  validity: stamped_header_policy.ValidityKind
  accepted: bool


@dataclass(frozen=True)
class FlsGeometryView:
  num_beams: int = 0
  num_range_bins: int = 0
  min_range_m: float = 0.0
  max_range_m: float = 0.0
  min_bearing_rad: float = 0.0
  max_bearing_rad: float = 0.0


@dataclass(frozen=True)
class FlsCalibrationView:
  """Pose is parent_from_sensor. pose_present is false for an unset pose."""

  present: bool = False
  parent_frame_id: str = ""
  sensor_frame_id: str = ""
  pose_present: bool = False
  position: frame_policy.Vec3 = (0.0, 0.0, 0.0)
  orientation: frame_policy.Quaternion = (0.0, 0.0, 0.0, 1.0)


@dataclass(frozen=True)
class FlsFrameView:
  """Plain values of one ForwardLookingSonarFrame."""

  header_present: bool = False
  validity_present: bool = False
  validity_state: int = 0
  frame_id: str = ""
  geometry: FlsGeometryView = FlsGeometryView()
  calibration: FlsCalibrationView = FlsCalibrationView()
  sound_speed_m_s: float = 0.0
  payload_mode: FlsPayloadMode = FlsPayloadMode.NONE
  # Inline mode.
  intensity: Sequence[float] = ()
  # Blob mode. The bytes behind blob_id are never fetched.
  blob_id: str = ""
  blob_byte_size_present: bool = False
  blob_byte_size: int = 0
  # True when the metadata map has any entry. Values are not inspected.
  metadata_present: bool = False


def fls_geometry_non_default(geometry: FlsGeometryView) -> bool:
  return (
      geometry.num_beams != 0
      or geometry.num_range_bins != 0
      or geometry.min_range_m != 0
      or geometry.max_range_m != 0
      or geometry.min_bearing_rad != 0
      or geometry.max_bearing_rad != 0
  )


def fls_frame_engaged(frame: FlsFrameView) -> bool:
  """Header, geometry, sound speed, calibration, payload, or metadata set.

  NaN compares unequal to zero, so a NaN field engages the frame.
  """
  return (
      frame.header_present
      or fls_geometry_non_default(frame.geometry)
      or frame.sound_speed_m_s != 0
      or frame.calibration.present
      or frame.payload_mode is not FlsPayloadMode.NONE
      or frame.metadata_present
  )


def fls_geometry_ok(geometry: FlsGeometryView) -> bool:
  return (
      geometry.num_beams >= 1
      and geometry.num_range_bins >= 1
      and math.isfinite(geometry.min_range_m)
      and math.isfinite(geometry.max_range_m)
      and math.isfinite(geometry.min_bearing_rad)
      and math.isfinite(geometry.max_bearing_rad)
      and geometry.min_range_m >= 0.0
      and geometry.max_range_m > geometry.min_range_m
      and geometry.max_bearing_rad >= geometry.min_bearing_rad
  )


def sound_speed_ok(sound_speed_m_s: float) -> bool:
  return math.isfinite(sound_speed_m_s) and sound_speed_m_s > 0.0


def expected_sample_count(geometry: FlsGeometryView) -> int | None:
  """num_beams * num_range_bins, or None when it exceeds 64 bits."""
  count = geometry.num_beams * geometry.num_range_bins
  if count > _UINT64_MAX:
    return None
  return count


def _assessment(
    error: FlsFrameContractError, validity: stamped_header_policy.ValidityKind
) -> FlsFrameContractAssessment:
  accepted = (
      error is FlsFrameContractError.NONE
      and stamped_header_policy.sample_accepted(validity, True)
  )
  return FlsFrameContractAssessment(error, validity, accepted)


def _assess_inline_payload(frame: FlsFrameView) -> FlsFrameContractError:
  """The size check comes first, then the sample scan."""
  expected = expected_sample_count(frame.geometry)
  if expected is None or len(frame.intensity) != expected:
    return FlsFrameContractError.PAYLOAD_MISMATCH
  if not all(math.isfinite(sample) for sample in frame.intensity):
    return FlsFrameContractError.NON_FINITE
  return FlsFrameContractError.NONE


def _assess_blob_payload(frame: FlsFrameView) -> FlsFrameContractError:
  if frame.blob_id == "" or (
      frame.blob_byte_size_present and frame.blob_byte_size == 0
  ):
    return FlsFrameContractError.BLOB_REFERENCE
  return FlsFrameContractError.NONE


def _assess_calibration(
    calibration: FlsCalibrationView,
) -> FlsFrameContractError:
  if (
      calibration.parent_frame_id == ""
      or calibration.sensor_frame_id == ""
      or calibration.parent_frame_id == calibration.sensor_frame_id
      or not calibration.pose_present
  ):
    return FlsFrameContractError.CALIBRATION
  if not frame_policy.is_finite_vec3(
      calibration.position
  ) or not frame_policy.is_finite_quaternion(calibration.orientation):
    return FlsFrameContractError.NON_FINITE
  if not frame_policy.is_normalized(calibration.orientation):
    return FlsFrameContractError.QUATERNION
  return FlsFrameContractError.NONE


def assess_forward_looking_sonar_frame(
    frame: FlsFrameView,
) -> FlsFrameContractAssessment:
  """First defect wins. An empty view is not engaged.

  Order: frame id, geometry, sound speed, payload presence, payload content
  (inline size then samples, or blob reference), then calibration when
  present. Metadata is never a defect.
  """
  if not fls_frame_engaged(frame):
    return FlsFrameContractAssessment(
        FlsFrameContractError.NONE,
        stamped_header_policy.ValidityKind.ABSENT,
        False,
    )
  validity = stamped_header_policy.classify_validity(
      frame.validity_present, frame.validity_state
  )
  if frame.frame_id == "":
    error = FlsFrameContractError.MISSING_FRAME
  elif not fls_geometry_ok(frame.geometry):
    error = FlsFrameContractError.GEOMETRY
  elif not sound_speed_ok(frame.sound_speed_m_s):
    error = FlsFrameContractError.SOUND_SPEED
  elif frame.payload_mode is FlsPayloadMode.NONE:
    error = FlsFrameContractError.MISSING_PAYLOAD
  elif frame.payload_mode is FlsPayloadMode.INLINE:
    error = _assess_inline_payload(frame)
  else:
    error = _assess_blob_payload(frame)
  if error is FlsFrameContractError.NONE and frame.calibration.present:
    error = _assess_calibration(frame.calibration)
  return _assessment(error, validity)
