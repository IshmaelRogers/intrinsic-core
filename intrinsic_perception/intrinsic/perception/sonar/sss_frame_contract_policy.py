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

"""Plain-value checks for SideScanSonarFrame.

Policy: intrinsic_apis/intrinsic/perception/proto/sonar/README.md.

These helpers do not parse protobuf, do not fetch blobs, do not build mosaics,
convert slant range, georeference, convert frames, or call World, ICON, or a
HAL.
"""

from collections.abc import Sequence
from dataclasses import dataclass
import enum
import math

from intrinsic.embodiment import frame_policy
from intrinsic.embodiment import stamped_header_policy


class SssFrameContractError(enum.Enum):
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


class SssPayloadMode(enum.Enum):
  """Which arm of the payload oneof is set. NONE means the oneof is unset."""

  NONE = 0
  INLINE = 1
  BLOB = 2


@dataclass(frozen=True)
class SssFrameContractAssessment:
  """accepted is true only for no error and STATE_VALID.

  An empty message is not accepted and is not an error.
  """

  error: SssFrameContractError
  validity: stamped_header_policy.ValidityKind
  accepted: bool


@dataclass(frozen=True)
class SssChannelGeometryView:
  """One across-track channel. An unset wire channel equals an all-zero one."""

  num_samples: int = 0
  min_range_m: float = 0.0
  max_range_m: float = 0.0


@dataclass(frozen=True)
class SssGeometryView:
  port: SssChannelGeometryView = SssChannelGeometryView()
  starboard: SssChannelGeometryView = SssChannelGeometryView()


@dataclass(frozen=True)
class SssCalibrationView:
  """Pose is parent_from_sensor. pose_present is false for an unset pose."""

  present: bool = False
  parent_frame_id: str = ""
  sensor_frame_id: str = ""
  pose_present: bool = False
  position: frame_policy.Vec3 = (0.0, 0.0, 0.0)
  orientation: frame_policy.Quaternion = (0.0, 0.0, 0.0, 1.0)


@dataclass(frozen=True)
class SssFrameView:
  """Plain values of one SideScanSonarFrame."""

  header_present: bool = False
  validity_present: bool = False
  validity_state: int = 0
  frame_id: str = ""
  geometry: SssGeometryView = SssGeometryView()
  calibration: SssCalibrationView = SssCalibrationView()
  sound_speed_m_s: float = 0.0
  payload_mode: SssPayloadMode = SssPayloadMode.NONE
  # Inline mode.
  port_intensity: Sequence[float] = ()
  starboard_intensity: Sequence[float] = ()
  # Blob mode. The bytes behind blob_id are never fetched.
  blob_id: str = ""
  blob_byte_size_present: bool = False
  blob_byte_size: int = 0
  # True when the metadata map has any entry. Values are not inspected.
  metadata_present: bool = False


def sss_channel_non_default(channel: SssChannelGeometryView) -> bool:
  return (
      channel.num_samples != 0
      or channel.min_range_m != 0
      or channel.max_range_m != 0
  )


def sss_geometry_non_default(geometry: SssGeometryView) -> bool:
  return sss_channel_non_default(geometry.port) or sss_channel_non_default(
      geometry.starboard
  )


def sss_frame_engaged(frame: SssFrameView) -> bool:
  """Header, geometry, sound speed, calibration, payload, or metadata set.

  NaN compares unequal to zero, so a NaN field engages the frame.
  """
  return (
      frame.header_present
      or sss_geometry_non_default(frame.geometry)
      or frame.sound_speed_m_s != 0
      or frame.calibration.present
      or frame.payload_mode is not SssPayloadMode.NONE
      or frame.metadata_present
  )


def sss_channel_ok(channel: SssChannelGeometryView) -> bool:
  return (
      channel.num_samples >= 1
      and math.isfinite(channel.min_range_m)
      and math.isfinite(channel.max_range_m)
      and channel.min_range_m >= 0.0
      and channel.max_range_m > channel.min_range_m
  )


def sss_geometry_ok(geometry: SssGeometryView) -> bool:
  return sss_channel_ok(geometry.port) and sss_channel_ok(geometry.starboard)


def sss_sound_speed_ok(sound_speed_m_s: float) -> bool:
  return math.isfinite(sound_speed_m_s) and sound_speed_m_s > 0.0


def _assessment(
    error: SssFrameContractError, validity: stamped_header_policy.ValidityKind
) -> SssFrameContractAssessment:
  accepted = (
      error is SssFrameContractError.NONE
      and stamped_header_policy.sample_accepted(validity, True)
  )
  return SssFrameContractAssessment(error, validity, accepted)


def _assess_inline_payload(frame: SssFrameView) -> SssFrameContractError:
  """Both size checks come first, then the sample scan (port, starboard)."""
  if (
      len(frame.port_intensity) != frame.geometry.port.num_samples
      or len(frame.starboard_intensity) != frame.geometry.starboard.num_samples
  ):
    return SssFrameContractError.PAYLOAD_MISMATCH
  if not all(
      math.isfinite(sample) for sample in frame.port_intensity
  ) or not all(math.isfinite(sample) for sample in frame.starboard_intensity):
    return SssFrameContractError.NON_FINITE
  return SssFrameContractError.NONE


def _assess_blob_payload(frame: SssFrameView) -> SssFrameContractError:
  if frame.blob_id == "" or (
      frame.blob_byte_size_present and frame.blob_byte_size == 0
  ):
    return SssFrameContractError.BLOB_REFERENCE
  return SssFrameContractError.NONE


def _assess_calibration(
    calibration: SssCalibrationView,
) -> SssFrameContractError:
  if (
      calibration.parent_frame_id == ""
      or calibration.sensor_frame_id == ""
      or calibration.parent_frame_id == calibration.sensor_frame_id
      or not calibration.pose_present
  ):
    return SssFrameContractError.CALIBRATION
  if not frame_policy.is_finite_vec3(
      calibration.position
  ) or not frame_policy.is_finite_quaternion(calibration.orientation):
    return SssFrameContractError.NON_FINITE
  if not frame_policy.is_normalized(calibration.orientation):
    return SssFrameContractError.QUATERNION
  return SssFrameContractError.NONE


def assess_side_scan_sonar_frame(
    frame: SssFrameView,
) -> SssFrameContractAssessment:
  """First defect wins. An empty view is not engaged.

  Order: frame id, geometry, sound speed, payload presence, payload content
  (inline sizes then samples, or blob reference), then calibration when
  present. Metadata is never a defect.
  """
  if not sss_frame_engaged(frame):
    return SssFrameContractAssessment(
        SssFrameContractError.NONE,
        stamped_header_policy.ValidityKind.ABSENT,
        False,
    )
  validity = stamped_header_policy.classify_validity(
      frame.validity_present, frame.validity_state
  )
  if frame.frame_id == "":
    error = SssFrameContractError.MISSING_FRAME
  elif not sss_geometry_ok(frame.geometry):
    error = SssFrameContractError.GEOMETRY
  elif not sss_sound_speed_ok(frame.sound_speed_m_s):
    error = SssFrameContractError.SOUND_SPEED
  elif frame.payload_mode is SssPayloadMode.NONE:
    error = SssFrameContractError.MISSING_PAYLOAD
  elif frame.payload_mode is SssPayloadMode.INLINE:
    error = _assess_inline_payload(frame)
  else:
    error = _assess_blob_payload(frame)
  if error is SssFrameContractError.NONE and frame.calibration.present:
    error = _assess_calibration(frame.calibration)
  return _assessment(error, validity)
