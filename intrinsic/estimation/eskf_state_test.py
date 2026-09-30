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

"""Tests for ESKF layout. Mirrors eskf_state_test.cc."""

import unittest

from intrinsic.estimation import eskf_state as eskf


class EskfStateTest(unittest.TestCase):

  def test_dimensions(self):
    self.assertEqual(eskf.kNominalDim, 16)
    self.assertEqual(eskf.kErrorDim, 15)
    self.assertEqual(eskf.kCovDim, 15)
    self.assertEqual(len(eskf.EskfNominal().to_tuple()), 16)
    self.assertEqual(len(eskf.EskfError().to_tuple()), 15)
    self.assertEqual(len(eskf.EskfCovariance.identity().row_major), 225)

  def test_nominal_index_constants_and_units(self):
    self.assertEqual(
        [tuple(f) for f in eskf.NOMINAL_FIELDS],
        [
            ("p_enu", 0, 3, "m"),
            ("q_wxyz", 3, 4, "1"),
            ("v_body", 7, 3, "m/s"),
            ("b_a", 10, 3, "m/s^2"),
            ("b_g", 13, 3, "rad/s"),
        ],
    )

  def test_error_index_constants_and_units(self):
    self.assertEqual(
        [tuple(f) for f in eskf.ERROR_FIELDS],
        [
            ("dp", 0, 3, "m"),
            ("dtheta", 3, 3, "rad"),
            ("dv", 6, 3, "m/s"),
            ("dba", 9, 3, "m/s^2"),
            ("dbg", 12, 3, "rad/s"),
        ],
    )

  def test_nominal_defaults(self):
    self.assertEqual(
        eskf.EskfNominal().to_tuple(),
        (0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0),
    )

  def test_error_defaults_are_zero(self):
    self.assertEqual(eskf.EskfError().to_tuple(), (0.0,) * 15)

  def test_defaults_are_value_stable(self):
    self.assertEqual(eskf.EskfNominal(), eskf.EskfNominal())
    self.assertEqual(eskf.EskfError(), eskf.EskfError())
    self.assertEqual(
        eskf.EskfNominal().to_tuple(), eskf.EskfNominal().to_tuple()
    )

  def test_nominal_index_order_fixture(self):
    n = eskf.EskfNominal(
        p_enu=(0, 1, 2),
        q_wxyz=(3, 4, 5, 6),
        v_body=(7, 8, 9),
        b_a=(10, 11, 12),
        b_g=(13, 14, 15),
    )
    self.assertEqual(n.to_tuple(), tuple(range(16)))
    back = eskf.EskfNominal.from_sequence([100 + i for i in range(16)])
    self.assertEqual(back.p_enu[0], 100)
    self.assertEqual(back.q_wxyz, (103, 104, 105, 106))
    self.assertEqual(back.v_body[0], 107)
    self.assertEqual(back.b_a[0], 110)
    self.assertEqual(back.b_g[2], 115)

  def test_error_index_order_fixture(self):
    e = eskf.EskfError(
        dp=(0, 1, 2),
        dtheta=(3, 4, 5),
        dv=(6, 7, 8),
        dba=(9, 10, 11),
        dbg=(12, 13, 14),
    )
    self.assertEqual(e.to_tuple(), tuple(range(15)))
    back = eskf.EskfError.from_sequence([100 + i for i in range(15)])
    self.assertEqual(back.dp[0], 100)
    self.assertEqual(back.dtheta[0], 103)
    self.assertEqual(back.dv[0], 106)
    self.assertEqual(back.dba[0], 109)
    self.assertEqual(back.dbg[2], 114)

  def test_wrong_sizes_rejected(self):
    for n in (15, 17):
      with self.assertRaises(ValueError):
        eskf.EskfNominal.from_sequence([0.0] * n)
    for n in (14, 16):
      with self.assertRaises(ValueError):
        eskf.EskfError.from_sequence([0.0] * n)
    for n in (224, 226):
      with self.assertRaises(ValueError):
        eskf.EskfCovariance([0.0] * n)

  def test_covariance_default_is_unknown_not_zeros(self):
    self.assertFalse(eskf.EskfCovariance().has_value)
    with self.assertRaises(ValueError):
      _ = eskf.EskfCovariance().row_major
    self.assertTrue(eskf.EskfCovariance([0.0] * 225).has_value)

  def test_covariance_row_major_order(self):
    p = eskf.EskfCovariance([float(i) for i in range(225)])
    self.assertEqual(p.at(0, 1), 1)
    self.assertEqual(p.at(1, 0), 15)
    self.assertEqual(p.at(14, 14), 224)
    with self.assertRaises(IndexError):
      p.at(15, 0)

  def test_identity_is_known_diagonal(self):
    p = eskf.EskfCovariance.identity()
    self.assertTrue(p.has_value)
    for r in range(15):
      for c in range(15):
        self.assertEqual(p.at(r, c), 1.0 if r == c else 0.0)


if __name__ == "__main__":
  unittest.main()
