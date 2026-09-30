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

"""Dive, DVL lock loss, recovery, resurface replay test (#90). Mirrors the .cc.

Test only. The replayer below calls the existing public estimation APIs and
nothing else: propagate_nominal, propagate_covariance, update_dvl_bottom_track,
update_dvl_water_track, update_depth, update_altitude, update_surface_position.
The phase tags in the fixture are labels, not a navigation-mode enum, and no
mode is decided here.
"""

import dataclasses
import math
import os
import struct
import unittest

from intrinsic.estimation import altitude_update
from intrinsic.estimation import depth_update
from intrinsic.estimation import dvl_bottom_track_update
from intrinsic.estimation import dvl_water_track_update
from intrinsic.estimation import eskf_cov_propagate
from intrinsic.estimation import eskf_propagate
from intrinsic.estimation import eskf_state
from intrinsic.estimation import surface_position_update

_FIXTURE = "dive_lock_loss_resurface.txt"
_N = eskf_state.COV_DIM
_TOL = 1e-9

_PHASES = (
    "SURFACE_FIX",
    "DIVE",
    "BOTTOM_LOCK",
    "LOCK_LOSS",
    "WATER_TRACK",
    "ALTITUDE",
    "RESURFACE",
)

# Error-state diagonal indices that each accepted update observes.
_OBSERVED = {
    "SURFACE": (0, 1),
    "DEPTH": (2,),
    "ALT": (2,),
    "DVL_BT": (6, 7, 8),
    "DVL_WT": (6, 7, 8),
}
# Pinned upper bounds on the observed posterior variances after an accept.
# Each is the measurement variance R of that sensor plus a small margin, since
# a posterior variance never exceeds R for a direct observation. The water
# track bound is looser because the current couples attitude into h(x).
_ACCEPT_CEILING = {
    "SURFACE": 0.04,
    "DEPTH": 0.01,
    "ALT": 0.01,
    "DVL_BT": 0.0004,
    "DVL_WT": 0.0009 + 1e-4,
}
_SLACK = 1e-12
# Pinned ceiling on the dv variance anywhere in LOCK_LOSS, (m/s)^2.
_LOCK_LOSS_DV_CEILING = 0.03
_DV_INDEX = 6

_EXPECTED_STATUSES = {
    "SURFACE": ("OK_ACCEPT", "SKIPPED_POLICY"),
    "DEPTH": ("OK_ACCEPT", "SKIPPED_INVALID"),
    "DVL_BT": ("OK_ACCEPT", "OK_REJECT", "SKIPPED_LOCK_LOSS"),
    "DVL_WT": ("OK_ACCEPT", "OK_REJECT", "SKIPPED_MISSING_CURRENT"),
    "ALT": ("OK_ACCEPT", "OK_REJECT", "SKIPPED_MISSING_SEAFLOOR"),
}

# Golden values from one replay of the fixture. Shared with the C++ test.
_EVENT_COUNT = 155
_MEASUREMENT_COUNT = 35
_ACCEPT_COUNT = 19
_STATUS_DIGEST = 0xE487EF7CDB87A065
_GOLDEN_NOMINAL = (
    73.02814147347405,
    20.684643554279752,
    -0.2714838825824188,
    0.9999999999994098,
    -8.693742969871378e-07,
    6.50407971566527e-07,
    3.9770524709302564e-08,
    0.5091360476894995,
    0.022370733591540008,
    0.0002562895657194696,
    -1.1704345690193751e-07,
    7.988003429889057e-08,
    -5.6663669078122895e-08,
    9.967584453900154e-09,
    -1.6367422251000055e-09,
    -5.227877923050971e-10,
)
_GOLDEN_P_DIAG = (
    0.05631013346235586,
    0.056313350579697066,
    0.015143689407485568,
    3.198633158779116e-07,
    3.198475535968472e-07,
    2.6552993267479133e-06,
    0.010023545828770708,
    0.010026539839451966,
    0.0008838142983284686,
    2.1994375822531817e-08,
    2.19943789658641e-08,
    2.1733812788215175e-08,
    5.3156397017609345e-11,
    5.3155556392633266e-11,
    1.0119195392779462e-10,
)
# Pinned tolerance of the final nominal against the fixture truth (m, m/s).
_TRUTH_P = (73.0, 20.7, -0.25)
_TRUTH_V = (0.52, 0.01, 0.0)
_TRUTH_P_TOL = 0.05
_TRUTH_V_TOL = 0.02


class FixtureError(Exception):
  pass


class ReplayError(Exception):
  pass


@dataclasses.dataclass(frozen=True)
class Event:
  kind: str
  t_ms: int
  phase: str
  payload: tuple
  expect: str


@dataclasses.dataclass(frozen=True)
class Fixture:
  chi2: tuple
  noise: eskf_cov_propagate.ProcessNoiseConfig
  init_t_ms: int
  init_x: eskf_state.EskfNominal
  p0_diag: tuple
  events: tuple


@dataclasses.dataclass(frozen=True)
class Step:
  index: int
  kind: str
  phase: str
  t_ms: int
  status: str
  x_before: tuple
  p_before: tuple
  x_after: tuple
  p_after: tuple


@dataclasses.dataclass(frozen=True)
class ReplayResult:
  x: eskf_state.EskfNominal
  p: eskf_state.EskfCovariance
  steps: tuple


def _floats(tokens):
  try:
    return tuple(float(t) for t in tokens)
  except ValueError as e:
    raise FixtureError(str(e)) from e


def _flag(token):
  if token not in ("0", "1"):
    raise FixtureError("bad flag " + token)
  return token == "1"


def _isotropic(r, n):
  return tuple(r if i // n == i % n else 0.0 for i in range(n * n))


def _parse_event(tokens):
  kind = tokens[0]
  widths = {
      "IMU": 9,
      "SURFACE": 10,
      "DEPTH": 8,
      "DVL_BT": 10,
      "DVL_WT": 13,
      "ALT": 9,
  }
  if kind not in widths:
    raise FixtureError("unknown event " + kind)
  if len(tokens) != widths[kind]:
    raise FixtureError(f"{kind} needs {widths[kind]} tokens")
  try:
    t_ms = int(tokens[1])
  except ValueError as e:
    raise FixtureError("bad time " + tokens[1]) from e
  phase = tokens[2]
  if phase not in _PHASES:
    raise FixtureError("unknown phase " + phase)
  a = tokens[3:]
  if kind == "IMU":
    f = _floats(a)
    payload = (eskf_propagate.ImuSample(accel_m_s2=f[0:3], gyro_rad_s=f[3:6]),)
    return Event(kind, t_ms, phase, payload, "OK")
  expect = a[-1]
  a = a[:-1]
  if kind == "SURFACE":
    e, n, r = _floats(a[3:6])
    payload = (
        surface_position_update.SurfacePositionSample(
            position_en_m=(e, n), R_en=_isotropic(r, 2), valid=_flag(a[2])
        ),
        surface_position_update.SurfaceFixPolicy(
            is_surfaced=_flag(a[0]), quality_ok=_flag(a[1])
        ),
    )
  elif kind == "DEPTH":
    depth, r, free_surface = _floats((a[0], a[1], a[3]))
    payload = (
        depth_update.DepthSample(
            depth_m=depth,
            R=r,
            valid=_flag(a[2]),
            free_surface_up_m=free_surface,
        ),
    )
  elif kind == "DVL_BT":
    f = _floats(a[0:4])
    payload = (
        dvl_bottom_track_update.DvlBottomTrackSample(
            velocity_body_m_s=f[0:3],
            R_body=_isotropic(f[3], 3),
            bottom_lock=_flag(a[4]),
            valid=_flag(a[5]),
        ),
    )
  elif kind == "DVL_WT":
    f = _floats(a[0:4])
    c = _floats(a[6:9])
    payload = (
        dvl_water_track_update.DvlWaterTrackSample(
            velocity_body_m_s=f[0:3],
            R_body=_isotropic(f[3], 3),
            valid=_flag(a[4]),
        ),
        dvl_water_track_update.WaterCurrentEstimate(
            present=_flag(a[5]), v_enu_m_s=c
        ),
    )
  else:
    altitude, r, floor = _floats((a[0], a[1], a[4]))
    payload = (
        altitude_update.AltitudeSample(
            altitude_m=altitude, R=r, valid=_flag(a[2])
        ),
        altitude_update.SeafloorContext(
            present=_flag(a[3]), seafloor_up_m=floor
        ),
    )
  return Event(kind, t_ms, phase, payload, expect)


def parse_fixture(text):
  """Parses the fixture text. Raises FixtureError on any malformed line."""
  config = init = p0 = None
  events = []
  for raw in text.splitlines():
    line = raw.strip()
    if not line or line.startswith("#"):
      continue
    tokens = line.split()
    kind = tokens[0]
    if kind == "CONFIG":
      if len(tokens) != 8:
        raise FixtureError("CONFIG needs 7 values")
      config = _floats(tokens[1:])
    elif kind == "INIT":
      if len(tokens) != 12:
        raise FixtureError("INIT needs 11 values")
      try:
        t_ms = int(tokens[1])
      except ValueError as e:
        raise FixtureError("bad time " + tokens[1]) from e
      f = _floats(tokens[2:])
      init = (
          t_ms,
          eskf_state.EskfNominal(p_enu=f[0:3], q_wxyz=f[3:7], v_body=f[7:10]),
      )
    elif kind == "P0_DIAG":
      if len(tokens) != 1 + _N:
        raise FixtureError("P0_DIAG needs 15 values")
      p0 = _floats(tokens[1:])
    else:
      events.append(_parse_event(tokens))
  if config is None or init is None or p0 is None:
    raise FixtureError("missing CONFIG, INIT, or P0_DIAG")
  return Fixture(
      chi2=config[0:3],
      noise=eskf_cov_propagate.ProcessNoiseConfig(
          sigma_accel=config[3],
          sigma_gyro=config[4],
          sigma_accel_bias_rw=config[5],
          sigma_gyro_bias_rw=config[6],
      ),
      init_t_ms=init[0],
      init_x=init[1],
      p0_diag=p0,
      events=tuple(events),
  )


def _diag_cov(diag):
  v = [0.0] * (_N * _N)
  for i in range(_N):
    v[i * _N + i] = diag[i]
  return eskf_state.EskfCovariance(v)


def _apply_measurement(ev, x, p, fixture):
  """Calls the public update for one event. Returns the update result."""
  chi2_1, chi2_2, chi2_3 = fixture.chi2
  if ev.kind == "SURFACE":
    return surface_position_update.update_surface_position(
        x, p, ev.payload[0], ev.payload[1], chi2_2
    )
  if ev.kind == "DEPTH":
    return depth_update.update_depth(x, p, ev.payload[0], chi2_1)
  if ev.kind == "DVL_BT":
    return dvl_bottom_track_update.update_dvl_bottom_track(
        x, p, ev.payload[0], chi2_3
    )
  if ev.kind == "DVL_WT":
    return dvl_water_track_update.update_dvl_water_track(
        x, p, ev.payload[0], ev.payload[1], chi2_3
    )
  return altitude_update.update_altitude(
      x, p, ev.payload[0], ev.payload[1], chi2_1
  )


def replay(fixture, num_events=None):
  """Replays the first num_events events (all when None) from the fixture."""
  x = fixture.init_x
  p = _diag_cov(fixture.p0_diag)
  t_ms = fixture.init_t_ms
  last_imu_ms = fixture.init_t_ms
  steps = []
  events = fixture.events if num_events is None else fixture.events[:num_events]
  for index, ev in enumerate(events):
    if ev.t_ms < t_ms:
      raise ReplayError(f"event {index} is out of order")
    t_ms = ev.t_ms
    x_before = x.to_tuple()
    p_before = p.row_major
    if ev.kind == "IMU":
      dt_s = (ev.t_ms - last_imu_ms) / 1000.0
      nominal = eskf_propagate.propagate_nominal(x, ev.payload[0], dt_s)
      cov = eskf_cov_propagate.propagate_covariance(
          x, ev.payload[0], dt_s, p, fixture.noise
      )
      if not nominal.ok or not cov.ok:
        raise ReplayError(f"event {index} propagation failed")
      x, p = nominal.nominal, cov.P
      last_imu_ms = ev.t_ms
      status = "OK"
    else:
      result = _apply_measurement(ev, x, p, fixture)
      status = result.status.name
      if result.accepted:
        x, p = result.nominal, result.P
      elif result.nominal is not None or result.P is not None:
        raise ReplayError(f"event {index} returned state without accept")
    steps.append(
        Step(
            index,
            ev.kind,
            ev.phase,
            ev.t_ms,
            status,
            x_before,
            p_before,
            x.to_tuple(),
            p.row_major,
        )
    )
  return ReplayResult(x=x, p=p, steps=tuple(steps))


def _fnv1a64(data, h=0xCBF29CE484222325):
  for b in data:
    h ^= b
    h = (h * 0x100000001B3) & 0xFFFFFFFFFFFFFFFF
  return h


def status_digest(result):
  """FNV-1a 64 over the ordered kind:status sequence. Exact across languages."""
  h = 0xCBF29CE484222325
  for s in result.steps:
    h = _fnv1a64(f"{s.kind}:{s.status}\n".encode("ascii"), h)
  return h


def full_digest(result):
  """FNV-1a 64 over final x, final P row-major, then the status sequence."""
  data = b"".join(struct.pack("<d", v) for v in result.x.to_tuple())
  data += b"".join(struct.pack("<d", v) for v in result.p.row_major)
  h = _fnv1a64(data)
  for s in result.steps:
    h = _fnv1a64(f"{s.kind}:{s.status}\n".encode("ascii"), h)
  return h


def _load_text():
  here = os.path.dirname(os.path.abspath(__file__))
  path = os.path.join(here, "testdata", _FIXTURE)
  if os.path.exists(path):
    with open(path, encoding="utf-8") as f:
      return f.read()
  root = os.environ.get("TEST_SRCDIR", "")
  for dirpath, _, filenames in os.walk(root):
    if _FIXTURE in filenames and os.path.basename(dirpath) == "testdata":
      with open(os.path.join(dirpath, _FIXTURE), encoding="utf-8") as f:
        return f.read()
  raise AssertionError("missing %s" % _FIXTURE)


def _diag(row_major):
  return [row_major[i * _N + i] for i in range(_N)]


class DiveLockLossResurfaceReplayTest(unittest.TestCase):

  @classmethod
  def setUpClass(cls):
    cls.fixture = parse_fixture(_load_text())
    cls.result = replay(cls.fixture)

  def _steps(self, phase=None, kind=None):
    return [
        s
        for s in self.result.steps
        if (phase is None or s.phase == phase)
        and (kind is None or s.kind == kind)
    ]

  def _measurements(self):
    return [s for s in self.result.steps if s.kind != "IMU"]

  def test_fixture_is_timestamped_and_ordered(self):
    events = self.fixture.events
    self.assertEqual(len(events), _EVENT_COUNT)
    times = [e.t_ms for e in events]
    self.assertEqual(times, sorted(times))
    self.assertGreaterEqual(times[0], self.fixture.init_t_ms)
    imu = [e.t_ms for e in events if e.kind == "IMU"]
    self.assertEqual(imu, list(range(1000, 1000 * len(imu) + 1, 1000)))
    order = [_PHASES.index(e.phase) for e in events]
    self.assertEqual(order, sorted(order))
    self.assertEqual(set(e.phase for e in events), set(_PHASES))

  def test_statuses_match_fixture_expectations(self):
    events = self.fixture.events
    self.assertEqual(len(self.result.steps), len(events))
    for ev, step in zip(events, self.result.steps):
      if ev.kind == "IMU":
        self.assertEqual(step.status, "OK")
      else:
        self.assertEqual(step.status, ev.expect, msg=f"{ev.kind}@{ev.t_ms}")
        self.assertIn(ev.expect, _EXPECTED_STATUSES[ev.kind])

  def test_phase_status_assertions(self):
    def statuses(phase, kind):
      return [s.status for s in self._steps(phase, kind)]

    self.assertEqual(
        statuses("SURFACE_FIX", "SURFACE"),
        ["OK_ACCEPT", "SKIPPED_POLICY", "OK_ACCEPT"],
    )
    self.assertEqual(statuses("SURFACE_FIX", "DEPTH"), ["OK_ACCEPT"] * 2)
    self.assertEqual(statuses("DIVE", "SURFACE"), ["SKIPPED_POLICY"] * 3)
    self.assertEqual(
        statuses("DIVE", "DEPTH"), ["OK_ACCEPT", "OK_ACCEPT", "SKIPPED_INVALID"]
    )
    self.assertEqual(
        statuses("BOTTOM_LOCK", "DVL_BT"),
        ["OK_ACCEPT", "OK_REJECT", "OK_ACCEPT"],
    )
    self.assertEqual(
        statuses("LOCK_LOSS", "DVL_BT"),
        [
            "SKIPPED_LOCK_LOSS",
            "SKIPPED_LOCK_LOSS",
            "SKIPPED_LOCK_LOSS",
            "SKIPPED_LOCK_LOSS",
        ],
    )
    self.assertEqual(
        statuses("WATER_TRACK", "DVL_WT"),
        ["SKIPPED_MISSING_CURRENT", "OK_ACCEPT", "OK_ACCEPT", "OK_REJECT"],
    )
    self.assertEqual(
        statuses("ALTITUDE", "ALT"),
        ["SKIPPED_MISSING_SEAFLOOR", "OK_ACCEPT", "OK_REJECT", "OK_ACCEPT"],
    )
    self.assertEqual(
        statuses("RESURFACE", "SURFACE"),
        ["SKIPPED_POLICY", "SKIPPED_POLICY", "OK_ACCEPT", "OK_ACCEPT"],
    )
    self.assertEqual(statuses("RESURFACE", "DEPTH"), ["OK_ACCEPT"] * 2)

  def test_skip_and_reject_leave_state_bit_identical(self):
    count = 0
    for s in self._measurements():
      if s.status == "OK_ACCEPT":
        continue
      count += 1
      self.assertEqual(s.x_after, s.x_before, msg=f"{s.kind}@{s.t_ms}")
      self.assertEqual(s.p_after, s.p_before, msg=f"{s.kind}@{s.t_ms}")
    self.assertEqual(count, _MEASUREMENT_COUNT - _ACCEPT_COUNT)

  def test_accept_changes_state_and_shrinks_observed_variance(self):
    accepts = [s for s in self._measurements() if s.status == "OK_ACCEPT"]
    self.assertEqual(len(accepts), _ACCEPT_COUNT)
    for s in accepts:
      self.assertNotEqual(s.p_after, s.p_before)
      before = _diag(s.p_before)
      after = _diag(s.p_after)
      for i in _OBSERVED[s.kind]:
        msg = f"{s.kind}@{s.t_ms} index {i}"
        self.assertLessEqual(after[i], before[i] + _SLACK, msg=msg)
        self.assertLessEqual(
            after[i], _ACCEPT_CEILING[s.kind] + _SLACK, msg=msg
        )
        self.assertGreater(after[i], 0.0, msg=msg)

  def test_covariance_stays_symmetric_positive_diagonal(self):
    for s in self.result.steps:
      p = s.p_after
      for i in range(_N):
        self.assertGreater(p[i * _N + i], 0.0, msg=f"{s.kind}@{s.t_ms}")
        for j in range(i + 1, _N):
          self.assertAlmostEqual(
              p[i * _N + j], p[j * _N + i], delta=1e-9, msg=f"{s.t_ms}"
          )

  def test_lock_loss_bounds_velocity_variance(self):
    lock = self._steps("LOCK_LOSS")
    for s in lock:
      self.assertLessEqual(
          _diag(s.p_after)[_DV_INDEX], _LOCK_LOSS_DV_CEILING, msg=f"{s.t_ms}"
      )
    start = _diag(self._steps("BOTTOM_LOCK")[-1].p_after)[_DV_INDEX]
    end = _diag(lock[-1].p_after)[_DV_INDEX]
    self.assertGreater(end, start)
    recovered = self._steps("WATER_TRACK", "DVL_WT")[1]
    self.assertEqual(recovered.status, "OK_ACCEPT")
    self.assertLess(_diag(recovered.p_after)[_DV_INDEX], end)

  def test_imu_propagation_continues_through_lock_loss(self):
    imu = self._steps("LOCK_LOSS", "IMU")
    self.assertEqual(len(imu), 20)
    for s in imu:
      self.assertNotEqual(s.p_after, s.p_before)
      self.assertEqual(s.status, "OK")

  def test_final_state_within_pinned_tolerances(self):
    x = self.result.x
    for got, want in zip(x.p_enu, _TRUTH_P):
      self.assertAlmostEqual(got, want, delta=_TRUTH_P_TOL)
    for got, want in zip(x.v_body, _TRUTH_V):
      self.assertAlmostEqual(got, want, delta=_TRUTH_V_TOL)
    self.assertAlmostEqual(
        math.sqrt(sum(q * q for q in x.q_wxyz)), 1.0, delta=1e-12
    )

  def test_golden_final_state_and_covariance(self):
    got = self.result.x.to_tuple()
    self.assertEqual(len(got), len(_GOLDEN_NOMINAL))
    for i, want in enumerate(_GOLDEN_NOMINAL):
      self.assertAlmostEqual(got[i], want, delta=_TOL, msg=str(i))
    diag = _diag(self.result.p.row_major)
    for i, want in enumerate(_GOLDEN_P_DIAG):
      self.assertAlmostEqual(diag[i], want, delta=_TOL, msg=str(i))

  def test_two_replays_have_identical_digest(self):
    a = replay(self.fixture)
    b = replay(parse_fixture(_load_text()))
    self.assertEqual(full_digest(a), full_digest(b))
    self.assertEqual(status_digest(a), status_digest(b))
    self.assertEqual(a.x.to_tuple(), b.x.to_tuple())
    self.assertEqual(a.p.row_major, b.p.row_major)
    self.assertEqual(full_digest(self.result), full_digest(a))

  def test_golden_status_digest(self):
    self.assertEqual(status_digest(self.result), _STATUS_DIGEST)

  def test_digest_detects_a_changed_status_or_state(self):
    base = full_digest(self.result)
    shorter = replay(self.fixture, _EVENT_COUNT - 1)
    self.assertNotEqual(full_digest(shorter), base)
    self.assertNotEqual(status_digest(shorter), status_digest(self.result))

  def test_empty_episode(self):
    r = replay(self.fixture, 0)
    self.assertEqual(r.steps, ())
    self.assertEqual(r.x, self.fixture.init_x)
    self.assertEqual(r.p.row_major, _diag_cov(self.fixture.p0_diag).row_major)
    self.assertEqual(full_digest(r), full_digest(replay(self.fixture, 0)))
    empty = Fixture(
        self.fixture.chi2,
        self.fixture.noise,
        self.fixture.init_t_ms,
        self.fixture.init_x,
        self.fixture.p0_diag,
        (),
    )
    self.assertEqual(replay(empty).steps, ())

  def test_one_event_episode(self):
    r = replay(self.fixture, 1)
    self.assertEqual(len(r.steps), 1)
    self.assertEqual(r.steps[0].kind, "IMU")
    self.assertNotEqual(r.x, self.fixture.init_x)
    first_measurement = next(
        i for i, e in enumerate(self.fixture.events) if e.kind != "IMU"
    )
    only = Fixture(
        self.fixture.chi2,
        self.fixture.noise,
        self.fixture.init_t_ms,
        self.fixture.init_x,
        self.fixture.p0_diag,
        (self.fixture.events[first_measurement],),
    )
    m = replay(only)
    self.assertEqual(m.steps[0].status, "OK_ACCEPT")

  def test_end_of_stream_prefix_matches_full_replay(self):
    n = len(self.fixture.events)
    for count in (1, 2, 40, 100, n - 1, n):
      prefix = replay(self.fixture, count)
      step = self.result.steps[count - 1]
      self.assertEqual(prefix.x.to_tuple(), step.x_after)
      self.assertEqual(prefix.p.row_major, step.p_after)
    past = replay(self.fixture, n + 50)
    self.assertEqual(len(past.steps), n)
    self.assertEqual(full_digest(past), full_digest(self.result))

  def test_unknown_and_truncated_events_are_typed_errors(self):
    text = _load_text()
    with self.assertRaises(FixtureError):
      parse_fixture(text + "SONAR 200000 DIVE 1\n")
    with self.assertRaises(FixtureError):
      parse_fixture(text + "IMU 200000 DIVE 0 0 9.8\n")
    with self.assertRaises(FixtureError):
      parse_fixture(text + "IMU 200000 NOT_A_PHASE 0 0 9.8 0 0 0\n")
    with self.assertRaises(FixtureError):
      parse_fixture("IMU 1000 DIVE 0 0 9.8 0 0 0\n")

  def test_out_of_order_timestamp_is_a_typed_error(self):
    events = list(self.fixture.events)
    events[0], events[3] = events[3], events[0]
    swapped = Fixture(
        self.fixture.chi2,
        self.fixture.noise,
        self.fixture.init_t_ms,
        self.fixture.init_x,
        self.fixture.p0_diag,
        tuple(events),
    )
    with self.assertRaises(ReplayError):
      replay(swapped)

  def test_replay_never_mutates_the_fixture(self):
    events = self.fixture.events
    before = [dataclasses.astuple(e) for e in events]
    replay(self.fixture)
    self.assertEqual([dataclasses.astuple(e) for e in events], before)


if __name__ == "__main__":
  unittest.main()
