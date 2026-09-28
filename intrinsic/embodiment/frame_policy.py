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

"""ENU/NED world-frame adapter.

Policy: intrinsic_apis/intrinsic/embodiment/proto/README.md.

Quaternion storage is Hamilton (x, y, z, w), matching
intrinsic_proto.Quaternion. These helpers convert world frames only.
Body axes stay REP-103 (x forward, y left, z up).
"""

import math

WORLD_ENU_FRAME_ID = "world_enu"
WORLD_NED_FRAME_ID = "world_ned"

Vec3 = tuple[float, float, float]
Quaternion = tuple[float, float, float, float]

_HALF_SQRT2 = math.sqrt(2.0) / 2.0
_NED_FROM_ENU = (_HALF_SQRT2, _HALF_SQRT2, 0.0, 0.0)


def world_frame_id(frame: str) -> str:
  if frame == "enu":
    return WORLD_ENU_FRAME_ID
  if frame == "ned":
    return WORLD_NED_FRAME_ID
  raise ValueError("frame must be 'enu' or 'ned'")


def frame_id_matches(frame_id: str, expected: str) -> bool:
  """True only for an exact well-known world id. Does not infer a frame."""
  return frame_id == world_frame_id(expected)


def is_finite_vec3(vector: Vec3) -> bool:
  return all(math.isfinite(component) for component in vector)


def is_finite_quaternion(quaternion: Quaternion) -> bool:
  return all(math.isfinite(component) for component in quaternion)


def quaternion_norm(quaternion: Quaternion) -> float:
  return math.sqrt(sum(component * component for component in quaternion))


def is_normalized(quaternion: Quaternion, tolerance: float = 1e-9) -> bool:
  if not is_finite_quaternion(quaternion):
    return False
  return abs(quaternion_norm(quaternion) - 1.0) <= tolerance


def quaternions_equivalent(
    lhs: Quaternion, rhs: Quaternion, tolerance: float = 1e-9
) -> bool:
  same = all(abs(a - b) <= tolerance for a, b in zip(lhs, rhs))
  flipped = all(abs(a + b) <= tolerance for a, b in zip(lhs, rhs))
  return same or flipped


def hamilton_product(lhs: Quaternion, rhs: Quaternion) -> Quaternion:
  """q_a_from_c = hamilton_product(q_a_from_b, q_b_from_c)."""
  x1, y1, z1, w1 = lhs
  x2, y2, z2, w2 = rhs
  return (
      (w1 * x2) + (x1 * w2) + (y1 * z2) - (z1 * y2),
      (w1 * y2) - (x1 * z2) + (y1 * w2) + (z1 * x2),
      (w1 * z2) + (x1 * y2) - (y1 * x2) + (z1 * w2),
      (w1 * w2) - (x1 * x2) - (y1 * y2) - (z1 * z2),
  )


def rotate_vector(rotation: Quaternion, vector: Vec3) -> Vec3:
  """Active rotation. Caller supplies a normalized quaternion."""
  pure = (vector[0], vector[1], vector[2], 0.0)
  conjugate = (-rotation[0], -rotation[1], -rotation[2], rotation[3])
  rotated = hamilton_product(hamilton_product(rotation, pure), conjugate)
  return (rotated[0], rotated[1], rotated[2])


def world_vector_enu_to_ned(enu: Vec3) -> Vec3:
  """World ENU to world NED. The map is an involution."""
  return (enu[1], enu[0], -enu[2])


def world_vector_ned_to_enu(ned: Vec3) -> Vec3:
  return (ned[1], ned[0], -ned[2])


def ned_from_enu_world_rotation() -> Quaternion:
  """(x, y, z, w) = (sqrt(2)/2, sqrt(2)/2, 0, 0)."""
  return _NED_FROM_ENU


def world_orientation_enu_to_ned(q_enu_from_body: Quaternion) -> Quaternion:
  """World-from-body orientation. Body axes are unchanged."""
  return hamilton_product(_NED_FROM_ENU, q_enu_from_body)


def world_orientation_ned_to_enu(q_ned_from_body: Quaternion) -> Quaternion:
  return hamilton_product(_NED_FROM_ENU, q_ned_from_body)
