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

"""Tests for the ENU/NED adapter and stamped-header host policy."""

import math
import unittest

from intrinsic.embodiment import frame_policy
from intrinsic.embodiment import stamped_header_policy


def _near(actual, expected, tolerance=1e-12):
  return all(abs(a - b) <= tolerance for a, b in zip(actual, expected))


class FramePolicyTest(unittest.TestCase):

  def test_enu_ned_maps_cardinal_axes(self):
    self.assertEqual(
        frame_policy.world_vector_enu_to_ned((1.0, 0.0, 0.0)), (0.0, 1.0, 0.0)
    )
    self.assertEqual(
        frame_policy.world_vector_enu_to_ned((0.0, 1.0, 0.0)), (1.0, 0.0, 0.0)
    )
    self.assertEqual(
        frame_policy.world_vector_enu_to_ned((0.0, 0.0, 1.0)), (0.0, 0.0, -1.0)
    )

  def test_enu_ned_vector_is_an_involution(self):
    enu = (-1.5, 2.25, 0.5)
    ned = frame_policy.world_vector_enu_to_ned(enu)
    self.assertEqual(frame_policy.world_vector_ned_to_enu(ned), enu)
    self.assertEqual(frame_policy.world_vector_enu_to_ned(ned), enu)

  def test_world_adapter_is_not_body_flu_to_frd(self):
    east = frame_policy.world_vector_enu_to_ned((1.0, 0.0, 0.0))
    self.assertEqual(east, (0.0, 1.0, 0.0))

  def test_frame_id_is_explicit(self):
    self.assertTrue(frame_policy.frame_id_matches("world_enu", "enu"))
    self.assertTrue(frame_policy.frame_id_matches("world_ned", "ned"))
    self.assertFalse(frame_policy.frame_id_matches("world_enu", "ned"))
    for name in (
        "world",
        "map",
        "odom",
        "base_link",
        "WORLD_ENU",
        "StampedHeader",
        "",
    ):
      self.assertFalse(frame_policy.frame_id_matches(name, "enu"))

  def test_quaternion_storage_order_and_ned_from_enu(self):
    rotation = frame_policy.ned_from_enu_world_rotation()
    half_sqrt2 = math.sqrt(2.0) / 2.0
    self.assertTrue(_near(rotation, (half_sqrt2, half_sqrt2, 0.0, 0.0)))
    self.assertTrue(frame_policy.is_normalized(rotation))

  def test_orientation_round_trip_matches_up_to_sign(self):
    q_enu = (0.0, 0.0, 0.0, 1.0)
    q_ned = frame_policy.world_orientation_enu_to_ned(q_enu)
    self.assertTrue(
        frame_policy.quaternions_equivalent(
            q_ned, frame_policy.ned_from_enu_world_rotation()
        )
    )
    self.assertTrue(
        frame_policy.quaternions_equivalent(
            frame_policy.world_orientation_ned_to_enu(q_ned), q_enu
        )
    )

  def test_body_vector_agrees_after_world_conversion(self):
    half_sqrt2 = math.sqrt(2.0) / 2.0
    q_z90 = (0.0, 0.0, half_sqrt2, half_sqrt2)
    q_ned = frame_policy.world_orientation_enu_to_ned(q_z90)
    forward = (1.0, 0.0, 0.0)
    left = (0.0, 1.0, 0.0)
    self.assertTrue(
        _near(frame_policy.rotate_vector(q_z90, forward), (0.0, 1.0, 0.0))
    )
    self.assertTrue(
        _near(
            frame_policy.world_vector_enu_to_ned(
                frame_policy.rotate_vector(q_z90, forward)
            ),
            frame_policy.rotate_vector(q_ned, forward),
        )
    )
    self.assertTrue(
        _near(
            frame_policy.world_vector_enu_to_ned(
                frame_policy.rotate_vector(q_z90, left)
            ),
            frame_policy.rotate_vector(q_ned, left),
        )
    )
    self.assertTrue(frame_policy.is_normalized(q_ned))

  def test_helper_does_not_renormalize(self):
    doubled = (0.0, 0.0, 0.0, 2.0)
    self.assertFalse(frame_policy.is_normalized(doubled))
    converted = frame_policy.world_orientation_enu_to_ned(doubled)
    self.assertAlmostEqual(frame_policy.quaternion_norm(converted), 2.0)
    self.assertFalse(frame_policy.is_normalized(converted))

  def test_non_finite_stays_non_finite(self):
    mapped = frame_policy.world_vector_enu_to_ned((math.nan, 1.0, 2.0))
    self.assertFalse(frame_policy.is_finite_vec3(mapped))
    self.assertEqual(mapped[0], 1.0)
    self.assertFalse(
        frame_policy.is_finite_quaternion((math.nan, 0.0, 0.0, 1.0))
    )
    self.assertFalse(frame_policy.is_normalized((math.nan, 0.0, 0.0, 1.0)))


class StampedHeaderPolicyTest(unittest.TestCase):

  def test_validity_absent_is_not_invalid(self):
    kind = stamped_header_policy.ValidityKind
    self.assertIs(
        stamped_header_policy.classify_validity(False, 0), kind.ABSENT
    )
    self.assertIs(
        stamped_header_policy.classify_validity(True, 0), kind.UNSPECIFIED
    )
    self.assertIs(
        stamped_header_policy.classify_validity(True, 2), kind.INVALID
    )
    self.assertIs(stamped_header_policy.classify_validity(True, 1), kind.VALID)
    self.assertIs(
        stamped_header_policy.classify_validity(True, 99), kind.UNSPECIFIED
    )
    self.assertIsNot(
        stamped_header_policy.classify_validity(False, 0),
        stamped_header_policy.classify_validity(True, 2),
    )
    self.assertFalse(stamped_header_policy.sample_accepted(kind.ABSENT, True))
    self.assertFalse(stamped_header_policy.sample_accepted(kind.INVALID, True))
    self.assertFalse(stamped_header_policy.sample_accepted(kind.VALID, False))
    self.assertTrue(stamped_header_policy.sample_accepted(kind.VALID, True))

  def test_monotonic_age_ignores_wall_clock(self):
    source = (1700000000, 250000000)
    receive = (1700000001, 0)
    age = stamped_header_policy.monotonic_age_seconds(
        source, receive, stamped_header_policy.CLOCK_DOMAIN_MONOTONIC
    )
    self.assertEqual(age, 0.75)
    self.assertIsNone(
        stamped_header_policy.monotonic_age_seconds(
            source, receive, stamped_header_policy.CLOCK_DOMAIN_UTC
        )
    )
    self.assertIsNone(
        stamped_header_policy.monotonic_age_seconds(source, receive, "wall")
    )
    self.assertIsNone(
        stamped_header_policy.monotonic_age_seconds(source, receive, "")
    )
    self.assertIsNone(
        stamped_header_policy.monotonic_age_seconds(
            None, receive, stamped_header_policy.CLOCK_DOMAIN_MONOTONIC
        )
    )
    self.assertIsNone(
        stamped_header_policy.monotonic_age_seconds(
            receive, source, stamped_header_policy.CLOCK_DOMAIN_MONOTONIC
        )
    )
    self.assertIsNone(
        stamped_header_policy.monotonic_age_seconds(
            (1, -1), (2, 0), stamped_header_policy.CLOCK_DOMAIN_MONOTONIC
        )
    )
    self.assertEqual(
        stamped_header_policy.monotonic_age_seconds(
            source, source, stamped_header_policy.CLOCK_DOMAIN_MONOTONIC
        ),
        0.0,
    )

  def test_sequence_advances_strictly(self):
    self.assertTrue(stamped_header_policy.sequence_advances(41, 42))
    self.assertFalse(stamped_header_policy.sequence_advances(42, 42))
    self.assertFalse(stamped_header_policy.sequence_advances(42, 41))
    self.assertFalse(stamped_header_policy.sequence_advances((1 << 64) - 1, 0))


if __name__ == "__main__":
  unittest.main()
