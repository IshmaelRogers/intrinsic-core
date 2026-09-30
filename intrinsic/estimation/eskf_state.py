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

"""ESKF layout only (#81). Mirrors eskf_state.h. See ESKF_LAYOUT.md.

No propagation, update, gating, injection, or reset.
"""

import dataclasses
from typing import NamedTuple, Optional, Sequence, Tuple

NOMINAL_DIM = 16
ERROR_DIM = 15
COV_DIM = 15

# Spelled as in the C++ header.
kNominalDim = NOMINAL_DIM
kErrorDim = ERROR_DIM
kCovDim = COV_DIM

NOMINAL_P_ENU = 0  # 0..2
NOMINAL_Q_WXYZ = 3  # 3..6
NOMINAL_V_BODY = 7  # 7..9
NOMINAL_B_A = 10  # 10..12
NOMINAL_B_G = 13  # 13..15

ERROR_DP = 0  # 0..2
ERROR_DTHETA = 3  # 3..5
ERROR_DV = 6  # 6..8
ERROR_DBA = 9  # 9..11
ERROR_DBG = 12  # 12..14


class EskfField(NamedTuple):
  symbol: str
  first: int
  count: int
  unit: str


NOMINAL_FIELDS = (
    EskfField("p_enu", NOMINAL_P_ENU, 3, "m"),
    EskfField("q_wxyz", NOMINAL_Q_WXYZ, 4, "1"),
    EskfField("v_body", NOMINAL_V_BODY, 3, "m/s"),
    EskfField("b_a", NOMINAL_B_A, 3, "m/s^2"),
    EskfField("b_g", NOMINAL_B_G, 3, "rad/s"),
)

ERROR_FIELDS = (
    EskfField("dp", ERROR_DP, 3, "m"),
    EskfField("dtheta", ERROR_DTHETA, 3, "rad"),
    EskfField("dv", ERROR_DV, 3, "m/s"),
    EskfField("dba", ERROR_DBA, 3, "m/s^2"),
    EskfField("dbg", ERROR_DBG, 3, "rad/s"),
)

_Vec3 = Tuple[float, float, float]


def _check_size(values: Sequence[float], expected: int, name: str) -> None:
  if len(values) != expected:
    raise ValueError(f"{name} needs {expected} values, got {len(values)}")


@dataclasses.dataclass(frozen=True)
class EskfNominal:
  """Nominal state, 16 scalars. Default: p=0, q=(1,0,0,0), rest 0."""

  p_enu: _Vec3 = (0.0, 0.0, 0.0)
  q_wxyz: Tuple[float, float, float, float] = (1.0, 0.0, 0.0, 0.0)
  v_body: _Vec3 = (0.0, 0.0, 0.0)
  b_a: _Vec3 = (0.0, 0.0, 0.0)
  b_g: _Vec3 = (0.0, 0.0, 0.0)

  def to_tuple(self) -> Tuple[float, ...]:
    return (
        tuple(self.p_enu)
        + tuple(self.q_wxyz)
        + tuple(self.v_body)
        + tuple(self.b_a)
        + tuple(self.b_g)
    )

  @classmethod
  def from_sequence(cls, values: Sequence[float]) -> "EskfNominal":
    """Raises ValueError unless len(values) == NOMINAL_DIM."""
    _check_size(values, NOMINAL_DIM, "EskfNominal")
    v = tuple(float(x) for x in values)
    return cls(
        p_enu=v[NOMINAL_P_ENU : NOMINAL_P_ENU + 3],
        q_wxyz=v[NOMINAL_Q_WXYZ : NOMINAL_Q_WXYZ + 4],
        v_body=v[NOMINAL_V_BODY : NOMINAL_V_BODY + 3],
        b_a=v[NOMINAL_B_A : NOMINAL_B_A + 3],
        b_g=v[NOMINAL_B_G : NOMINAL_B_G + 3],
    )


@dataclasses.dataclass(frozen=True)
class EskfError:
  """Error state, 15 scalars. Default: all zeros."""

  dp: _Vec3 = (0.0, 0.0, 0.0)
  dtheta: _Vec3 = (0.0, 0.0, 0.0)
  dv: _Vec3 = (0.0, 0.0, 0.0)
  dba: _Vec3 = (0.0, 0.0, 0.0)
  dbg: _Vec3 = (0.0, 0.0, 0.0)

  def to_tuple(self) -> Tuple[float, ...]:
    return (
        tuple(self.dp)
        + tuple(self.dtheta)
        + tuple(self.dv)
        + tuple(self.dba)
        + tuple(self.dbg)
    )

  @classmethod
  def from_sequence(cls, values: Sequence[float]) -> "EskfError":
    """Raises ValueError unless len(values) == ERROR_DIM."""
    _check_size(values, ERROR_DIM, "EskfError")
    v = tuple(float(x) for x in values)
    return cls(
        dp=v[ERROR_DP : ERROR_DP + 3],
        dtheta=v[ERROR_DTHETA : ERROR_DTHETA + 3],
        dv=v[ERROR_DV : ERROR_DV + 3],
        dba=v[ERROR_DBA : ERROR_DBA + 3],
        dbg=v[ERROR_DBG : ERROR_DBG + 3],
    )


class EskfCovariance:
  """15x15 row-major covariance P. Default is unknown (no storage)."""

  def __init__(self, row_major: Optional[Sequence[float]] = None):
    if row_major is None:
      self._data = None
      return
    _check_size(row_major, COV_DIM * COV_DIM, "EskfCovariance")
    self._data = tuple(float(x) for x in row_major)

  @classmethod
  def identity(cls) -> "EskfCovariance":
    """Known diagonal P for tests. Not the default."""
    return cls(
        [1.0 if r == c else 0.0 for r in range(COV_DIM) for c in range(COV_DIM)]
    )

  @property
  def has_value(self) -> bool:
    return self._data is not None

  @property
  def row_major(self) -> Tuple[float, ...]:
    if self._data is None:
      raise ValueError("covariance is unknown")
    return self._data

  def at(self, row: int, col: int) -> float:
    if not (0 <= row < COV_DIM and 0 <= col < COV_DIM):
      raise IndexError(f"({row}, {col}) outside {COV_DIM}x{COV_DIM}")
    return self.row_major[row * COV_DIM + col]
