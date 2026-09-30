// Copyright 2026 Intrinsic Innovation LLC
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     https://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

// Dive, DVL lock loss, recovery, resurface replay test (#90). Mirrors
// dive_lock_loss_resurface_replay_test.py.
//
// Test only. The replayer below calls the existing public estimation APIs and
// nothing else: PropagateNominal, PropagateCovariance, UpdateDvlBottomTrack,
// UpdateDvlWaterTrack, UpdateDepth, UpdateAltitude, UpdateSurfacePosition. The
// phase tags in the fixture are labels, not a navigation-mode enum, and no mode
// is decided here.

#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <limits>
#include <map>
#include <optional>
#include <set>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include "intrinsic/estimation/altitude_update.h"
#include "intrinsic/estimation/depth_update.h"
#include "intrinsic/estimation/dvl_bottom_track_update.h"
#include "intrinsic/estimation/dvl_water_track_update.h"
#include "intrinsic/estimation/eskf_cov_propagate.h"
#include "intrinsic/estimation/eskf_propagate.h"
#include "intrinsic/estimation/eskf_state.h"
#include "intrinsic/estimation/surface_position_update.h"

namespace intrinsic::estimation {
namespace {

constexpr int kN = kCovDim;
constexpr double kTol = 1e-9;
constexpr double kSlack = 1e-12;
const char kFixtureName[] = "dive_lock_loss_resurface.txt";

const std::vector<std::string> kPhases = {
    "SURFACE_FIX", "DIVE",     "BOTTOM_LOCK", "LOCK_LOSS",
    "WATER_TRACK", "ALTITUDE", "RESURFACE"};

// Error-state diagonal indices that each accepted update observes.
const std::map<std::string, std::vector<int>> kObserved = {
    {"SURFACE", {0, 1}},   {"DEPTH", {2}},        {"ALT", {2}},
    {"DVL_BT", {6, 7, 8}}, {"DVL_WT", {6, 7, 8}},
};
// Pinned upper bounds on the observed posterior variances after an accept.
// Each is the measurement variance R of that sensor plus a small margin, since
// a posterior variance never exceeds R for a direct observation. The water
// track bound is looser because the current couples attitude into h(x).
const std::map<std::string, double> kAcceptCeiling = {
    {"SURFACE", 0.04},  {"DEPTH", 0.01},           {"ALT", 0.01},
    {"DVL_BT", 0.0004}, {"DVL_WT", 0.0009 + 1e-4},
};
// Pinned ceiling on the dv variance anywhere in LOCK_LOSS, (m/s)^2.
constexpr double kLockLossDvCeiling = 0.03;
constexpr int kDvIndex = 6;

const std::map<std::string, std::set<std::string>> kExpectedStatuses = {
    {"SURFACE", {"OK_ACCEPT", "SKIPPED_POLICY"}},
    {"DEPTH", {"OK_ACCEPT", "SKIPPED_INVALID"}},
    {"DVL_BT", {"OK_ACCEPT", "OK_REJECT", "SKIPPED_LOCK_LOSS"}},
    {"DVL_WT", {"OK_ACCEPT", "OK_REJECT", "SKIPPED_MISSING_CURRENT"}},
    {"ALT", {"OK_ACCEPT", "OK_REJECT", "SKIPPED_MISSING_SEAFLOOR"}},
};

// Golden values from one replay of the fixture. Shared with the Python test.
constexpr size_t kEventCount = 155;
constexpr size_t kMeasurementCount = 35;
constexpr size_t kAcceptCount = 19;
constexpr uint64_t kStatusDigest = 0xe487ef7cdb87a065ULL;
constexpr double kGoldenNominal[kNominalDim] = {
    73.02814147347405,       20.684643554279752,      -0.2714838825824188,
    0.9999999999994098,      -8.693742969871378e-07,  6.50407971566527e-07,
    3.9770524709302564e-08,  0.5091360476894995,      0.022370733591540008,
    0.0002562895657194696,   -1.1704345690193751e-07, 7.988003429889057e-08,
    -5.6663669078122895e-08, 9.967584453900154e-09,   -1.6367422251000055e-09,
    -5.227877923050971e-10,
};
constexpr double kGoldenPDiag[kN] = {
    0.05631013346235586,    0.056313350579697066,   0.015143689407485568,
    3.198633158779116e-07,  3.198475535968472e-07,  2.6552993267479133e-06,
    0.010023545828770708,   0.010026539839451966,   0.0008838142983284686,
    2.1994375822531817e-08, 2.19943789658641e-08,   2.1733812788215175e-08,
    5.3156397017609345e-11, 5.3155556392633266e-11, 1.0119195392779462e-10,
};
// Pinned tolerance of the final nominal against the fixture truth (m, m/s).
constexpr double kTruthP[3] = {73.0, 20.7, -0.25};
constexpr double kTruthV[3] = {0.52, 0.01, 0.0};
constexpr double kTruthPTol = 0.05;
constexpr double kTruthVTol = 0.02;

using NominalArray = std::array<double, kNominalDim>;
using CovArray = std::array<double, kN * kN>;

struct Event {
  std::string kind;
  int64_t t_ms = 0;
  std::string phase;
  ImuSample imu;
  SurfacePositionSample surface;
  SurfaceFixPolicy policy;
  DepthSample depth;
  DvlBottomTrackSample bottom_track;
  DvlWaterTrackSample water_track;
  WaterCurrentEstimate current;
  AltitudeSample altitude;
  SeafloorContext seafloor;
  std::string expect;
};

struct Fixture {
  std::array<double, 3> chi2 = {0.0, 0.0, 0.0};
  ProcessNoiseConfig noise;
  int64_t init_t_ms = 0;
  EskfNominal init_x;
  std::array<double, kN> p0_diag = {};
  std::vector<Event> events;
};

struct Step {
  size_t index = 0;
  std::string kind;
  std::string phase;
  int64_t t_ms = 0;
  std::string status;
  NominalArray x_before;
  CovArray p_before;
  NominalArray x_after;
  CovArray p_after;
};

struct ReplayResult {
  bool ok = false;
  std::string error;
  EskfNominal x;
  EskfCovariance p;
  std::vector<Step> steps;
};

std::vector<std::string> Split(const std::string& line) {
  std::istringstream in(line);
  std::vector<std::string> tokens;
  std::string token;
  while (in >> token) tokens.push_back(token);
  return tokens;
}

bool ParseDouble(const std::string& token, double* out) {
  char* end = nullptr;
  *out = std::strtod(token.c_str(), &end);
  return end != token.c_str() && *end == '\0' && std::isfinite(*out);
}

bool ParseInt(const std::string& token, int64_t* out) {
  char* end = nullptr;
  *out = std::strtoll(token.c_str(), &end, 10);
  return end != token.c_str() && *end == '\0';
}

bool ParseFlag(const std::string& token, bool* out) {
  if (token != "0" && token != "1") return false;
  *out = token == "1";
  return true;
}

// Parses tokens[first, first + n) as doubles.
bool ParseDoubles(const std::vector<std::string>& tokens, size_t first,
                  size_t n, double* out) {
  for (size_t i = 0; i < n; ++i) {
    if (!ParseDouble(tokens[first + i], &out[i])) return false;
  }
  return true;
}

template <size_t M>
std::array<double, M * M> Isotropic(double r) {
  std::array<double, M * M> out = {};
  for (size_t i = 0; i < M; ++i) out[i * M + i] = r;
  return out;
}

bool ParseEvent(const std::vector<std::string>& tokens, Event* ev,
                std::string* error) {
  static const std::map<std::string, size_t> kWidths = {
      {"IMU", 9},     {"SURFACE", 10}, {"DEPTH", 8},
      {"DVL_BT", 10}, {"DVL_WT", 13},  {"ALT", 9}};
  const std::string& kind = tokens[0];
  const auto width = kWidths.find(kind);
  if (width == kWidths.end()) {
    *error = "unknown event " + kind;
    return false;
  }
  if (tokens.size() != width->second) {
    *error = kind + " has the wrong token count";
    return false;
  }
  ev->kind = kind;
  if (!ParseInt(tokens[1], &ev->t_ms)) {
    *error = "bad time " + tokens[1];
    return false;
  }
  ev->phase = tokens[2];
  if (std::find(kPhases.begin(), kPhases.end(), ev->phase) == kPhases.end()) {
    *error = "unknown phase " + ev->phase;
    return false;
  }
  bool ok = true;
  if (kind == "IMU") {
    double f[6];
    ok = ParseDoubles(tokens, 3, 6, f);
    ev->imu.accel_m_s2 = {f[0], f[1], f[2]};
    ev->imu.gyro_rad_s = {f[3], f[4], f[5]};
    ev->expect = "OK";
  } else {
    ev->expect = tokens.back();
    const std::vector<std::string> a(tokens.begin() + 3, tokens.end() - 1);
    if (kind == "SURFACE") {
      double f[3];
      ok = ParseFlag(a[0], &ev->policy.is_surfaced) &&
           ParseFlag(a[1], &ev->policy.quality_ok) &&
           ParseFlag(a[2], &ev->surface.valid) && ParseDoubles(a, 3, 3, f);
      ev->surface.position_en_m = {f[0], f[1]};
      ev->surface.R_en = Isotropic<2>(f[2]);
    } else if (kind == "DEPTH") {
      ok = ParseDouble(a[0], &ev->depth.depth_m) &&
           ParseDouble(a[1], &ev->depth.R) &&
           ParseFlag(a[2], &ev->depth.valid) &&
           ParseDouble(a[3], &ev->depth.free_surface_up_m);
    } else if (kind == "DVL_BT") {
      double f[4];
      ok = ParseDoubles(a, 0, 4, f) &&
           ParseFlag(a[4], &ev->bottom_track.bottom_lock) &&
           ParseFlag(a[5], &ev->bottom_track.valid);
      ev->bottom_track.velocity_body_m_s = {f[0], f[1], f[2]};
      ev->bottom_track.R_body = Isotropic<3>(f[3]);
    } else if (kind == "DVL_WT") {
      double f[4];
      double c[3];
      ok = ParseDoubles(a, 0, 4, f) &&
           ParseFlag(a[4], &ev->water_track.valid) &&
           ParseFlag(a[5], &ev->current.present) && ParseDoubles(a, 6, 3, c);
      ev->water_track.velocity_body_m_s = {f[0], f[1], f[2]};
      ev->water_track.R_body = Isotropic<3>(f[3]);
      ev->current.v_enu_m_s = {c[0], c[1], c[2]};
    } else {
      ok = ParseDouble(a[0], &ev->altitude.altitude_m) &&
           ParseDouble(a[1], &ev->altitude.R) &&
           ParseFlag(a[2], &ev->altitude.valid) &&
           ParseFlag(a[3], &ev->seafloor.present) &&
           ParseDouble(a[4], &ev->seafloor.seafloor_up_m);
    }
  }
  if (!ok) *error = "bad value in " + kind;
  return ok;
}

// Parses the fixture text. Returns false with `error` set on a malformed line.
bool ParseFixture(const std::string& text, Fixture* out, std::string* error) {
  bool have_config = false;
  bool have_init = false;
  bool have_p0 = false;
  std::istringstream in(text);
  std::string raw;
  while (std::getline(in, raw)) {
    const std::vector<std::string> tokens = Split(raw);
    if (tokens.empty() || tokens[0][0] == '#') continue;
    const std::string& kind = tokens[0];
    if (kind == "CONFIG") {
      double f[7];
      if (tokens.size() != 8 || !ParseDoubles(tokens, 1, 7, f)) {
        *error = "bad CONFIG";
        return false;
      }
      out->chi2 = {f[0], f[1], f[2]};
      out->noise.sigma_accel = f[3];
      out->noise.sigma_gyro = f[4];
      out->noise.sigma_accel_bias_rw = f[5];
      out->noise.sigma_gyro_bias_rw = f[6];
      have_config = true;
    } else if (kind == "INIT") {
      double f[10];
      if (tokens.size() != 12 || !ParseInt(tokens[1], &out->init_t_ms) ||
          !ParseDoubles(tokens, 2, 10, f)) {
        *error = "bad INIT";
        return false;
      }
      out->init_x.p_enu = {f[0], f[1], f[2]};
      out->init_x.q_wxyz = {f[3], f[4], f[5], f[6]};
      out->init_x.v_body = {f[7], f[8], f[9]};
      have_init = true;
    } else if (kind == "P0_DIAG") {
      if (tokens.size() != 1 + kN ||
          !ParseDoubles(tokens, 1, kN, out->p0_diag.data())) {
        *error = "bad P0_DIAG";
        return false;
      }
      have_p0 = true;
    } else {
      Event ev;
      if (!ParseEvent(tokens, &ev, error)) return false;
      out->events.push_back(ev);
    }
  }
  if (!have_config || !have_init || !have_p0) {
    *error = "missing CONFIG, INIT, or P0_DIAG";
    return false;
  }
  return true;
}

EskfCovariance DiagCov(const std::array<double, kN>& diag) {
  std::vector<double> v(kN * kN, 0.0);
  for (int i = 0; i < kN; ++i) v[i * kN + i] = diag[i];
  return *EskfCovariance::FromRowMajor(v);
}

const char* Name(DvlUpdateStatus s) {
  switch (s) {
    case DvlUpdateStatus::kOkAccept:
      return "OK_ACCEPT";
    case DvlUpdateStatus::kOkReject:
      return "OK_REJECT";
    case DvlUpdateStatus::kSkippedLockLoss:
      return "SKIPPED_LOCK_LOSS";
    case DvlUpdateStatus::kSkippedInvalid:
      return "SKIPPED_INVALID";
    case DvlUpdateStatus::kSingular:
      return "SINGULAR";
    case DvlUpdateStatus::kNonFinite:
      return "NON_FINITE";
  }
  return "UNKNOWN";
}

const char* Name(DvlWaterTrackStatus s) {
  switch (s) {
    case DvlWaterTrackStatus::kOkAccept:
      return "OK_ACCEPT";
    case DvlWaterTrackStatus::kOkReject:
      return "OK_REJECT";
    case DvlWaterTrackStatus::kSkippedMissingCurrent:
      return "SKIPPED_MISSING_CURRENT";
    case DvlWaterTrackStatus::kSkippedInvalid:
      return "SKIPPED_INVALID";
    case DvlWaterTrackStatus::kSingular:
      return "SINGULAR";
    case DvlWaterTrackStatus::kNonFinite:
      return "NON_FINITE";
  }
  return "UNKNOWN";
}

const char* Name(DepthUpdateStatus s) {
  switch (s) {
    case DepthUpdateStatus::kOkAccept:
      return "OK_ACCEPT";
    case DepthUpdateStatus::kOkReject:
      return "OK_REJECT";
    case DepthUpdateStatus::kSkippedInvalid:
      return "SKIPPED_INVALID";
    case DepthUpdateStatus::kSingular:
      return "SINGULAR";
    case DepthUpdateStatus::kNonFinite:
      return "NON_FINITE";
  }
  return "UNKNOWN";
}

const char* Name(AltitudeUpdateStatus s) {
  switch (s) {
    case AltitudeUpdateStatus::kOkAccept:
      return "OK_ACCEPT";
    case AltitudeUpdateStatus::kOkReject:
      return "OK_REJECT";
    case AltitudeUpdateStatus::kSkippedMissingSeafloor:
      return "SKIPPED_MISSING_SEAFLOOR";
    case AltitudeUpdateStatus::kSkippedInvalid:
      return "SKIPPED_INVALID";
    case AltitudeUpdateStatus::kSingular:
      return "SINGULAR";
    case AltitudeUpdateStatus::kNonFinite:
      return "NON_FINITE";
  }
  return "UNKNOWN";
}

const char* Name(SurfacePositionUpdateStatus s) {
  switch (s) {
    case SurfacePositionUpdateStatus::kOkAccept:
      return "OK_ACCEPT";
    case SurfacePositionUpdateStatus::kOkReject:
      return "OK_REJECT";
    case SurfacePositionUpdateStatus::kSkippedPolicy:
      return "SKIPPED_POLICY";
    case SurfacePositionUpdateStatus::kSkippedInvalid:
      return "SKIPPED_INVALID";
    case SurfacePositionUpdateStatus::kSingular:
      return "SINGULAR";
    case SurfacePositionUpdateStatus::kNonFinite:
      return "NON_FINITE";
  }
  return "UNKNOWN";
}

// What one public measurement update returned, reduced to what replay commits.
struct Applied {
  std::string status;
  bool accepted = false;
  bool has_state = false;
  std::optional<EskfNominal> nominal;
  std::optional<EskfCovariance> P;
};

template <typename Result, typename Status>
Applied Reduce(const Result& r, Status status) {
  Applied a;
  a.status = Name(status);
  a.accepted = r.accepted;
  a.has_state = r.nominal.has_value() || r.P.has_value();
  a.nominal = r.nominal;
  a.P = r.P;
  return a;
}

Applied ApplyMeasurement(const Event& ev, const EskfNominal& x,
                         const EskfCovariance& p, const Fixture& f) {
  const double chi2_1 = f.chi2[0];
  const double chi2_2 = f.chi2[1];
  const double chi2_3 = f.chi2[2];
  if (ev.kind == "SURFACE") {
    const auto r = UpdateSurfacePosition(x, p, ev.surface, ev.policy, chi2_2);
    return Reduce(r, r.status);
  }
  if (ev.kind == "DEPTH") {
    const auto r = UpdateDepth(x, p, ev.depth, chi2_1);
    return Reduce(r, r.status);
  }
  if (ev.kind == "DVL_BT") {
    const auto r = UpdateDvlBottomTrack(x, p, ev.bottom_track, chi2_3);
    return Reduce(r, r.status);
  }
  if (ev.kind == "DVL_WT") {
    const auto r =
        UpdateDvlWaterTrack(x, p, ev.water_track, ev.current, chi2_3);
    return Reduce(r, r.status);
  }
  const auto r = UpdateAltitude(x, p, ev.altitude, ev.seafloor, chi2_1);
  return Reduce(r, r.status);
}

CovArray CovRows(const EskfCovariance& p) { return p.row_major(); }

// Replays the first `num_events` events (all by default) from the fixture.
ReplayResult Replay(const Fixture& f,
                    size_t num_events = std::numeric_limits<size_t>::max()) {
  ReplayResult out;
  EskfNominal x = f.init_x;
  EskfCovariance p = DiagCov(f.p0_diag);
  int64_t t_ms = f.init_t_ms;
  int64_t last_imu_ms = f.init_t_ms;
  const size_t count = std::min(num_events, f.events.size());
  for (size_t index = 0; index < count; ++index) {
    const Event& ev = f.events[index];
    if (ev.t_ms < t_ms) {
      out.error = "event " + std::to_string(index) + " is out of order";
      return out;
    }
    t_ms = ev.t_ms;
    Step step;
    step.index = index;
    step.kind = ev.kind;
    step.phase = ev.phase;
    step.t_ms = ev.t_ms;
    step.x_before = x.ToArray();
    step.p_before = CovRows(p);
    if (ev.kind == "IMU") {
      const double dt_s = static_cast<double>(ev.t_ms - last_imu_ms) / 1000.0;
      const PropagateResult nominal = PropagateNominal(x, ev.imu, dt_s);
      const CovPropagateResult cov =
          PropagateCovariance(x, ev.imu, dt_s, p, f.noise);
      if (!nominal.ok || !cov.ok) {
        out.error = "event " + std::to_string(index) + " propagation failed";
        return out;
      }
      x = *nominal.nominal;
      p = *cov.P;
      last_imu_ms = ev.t_ms;
      step.status = "OK";
    } else {
      const Applied applied = ApplyMeasurement(ev, x, p, f);
      step.status = applied.status;
      if (applied.accepted) {
        x = *applied.nominal;
        p = *applied.P;
      } else if (applied.has_state) {
        out.error = "event " + std::to_string(index) + " returned state";
        return out;
      }
    }
    step.x_after = x.ToArray();
    step.p_after = CovRows(p);
    out.steps.push_back(std::move(step));
  }
  out.ok = true;
  out.x = x;
  out.p = p;
  return out;
}

uint64_t Fnv1a64(const void* data, size_t size, uint64_t h) {
  const unsigned char* bytes = static_cast<const unsigned char*>(data);
  for (size_t i = 0; i < size; ++i) {
    h ^= bytes[i];
    h *= 0x100000001B3ULL;
  }
  return h;
}

constexpr uint64_t kFnvBasis = 0xCBF29CE484222325ULL;

uint64_t HashStatuses(const ReplayResult& r, uint64_t h) {
  for (const Step& s : r.steps) {
    const std::string line = s.kind + ":" + s.status + "\n";
    h = Fnv1a64(line.data(), line.size(), h);
  }
  return h;
}

// FNV-1a 64 over the ordered kind:status sequence. Exact across languages.
uint64_t StatusDigest(const ReplayResult& r) {
  return HashStatuses(r, kFnvBasis);
}

// FNV-1a 64 over final x, final P row-major (little-endian IEEE-754 doubles),
// then the status sequence.
uint64_t FullDigest(const ReplayResult& r) {
  const NominalArray x = r.x.ToArray();
  const CovArray& p = r.p.row_major();
  uint64_t h = Fnv1a64(x.data(), sizeof(double) * x.size(), kFnvBasis);
  h = Fnv1a64(p.data(), sizeof(double) * p.size(), h);
  return HashStatuses(r, h);
}

std::string LoadFixtureText() {
  const std::string suffix =
      std::string("intrinsic/estimation/testdata/") + kFixtureName;
  std::vector<std::string> candidates;
  if (const char* src = std::getenv("TEST_SRCDIR")) {
    candidates.push_back(std::string(src) + "/_main/" + suffix);
    candidates.push_back(std::string(src) + "/" + suffix);
    if (const char* workspace = std::getenv("TEST_WORKSPACE")) {
      candidates.push_back(std::string(src) + "/" + workspace + "/" + suffix);
    }
  }
  candidates.push_back(suffix);
  candidates.push_back(std::string("testdata/") + kFixtureName);
  for (const std::string& path : candidates) {
    std::ifstream in(path);
    if (!in) continue;
    std::ostringstream buffer;
    buffer << in.rdbuf();
    return buffer.str();
  }
  return "";
}

Fixture LoadFixture() {
  Fixture f;
  std::string error;
  const std::string text = LoadFixtureText();
  EXPECT_FALSE(text.empty()) << "missing " << kFixtureName;
  EXPECT_TRUE(ParseFixture(text, &f, &error)) << error;
  return f;
}

std::array<double, kN> Diag(const CovArray& p) {
  std::array<double, kN> d;
  for (int i = 0; i < kN; ++i) d[i] = p[i * kN + i];
  return d;
}

class DiveLockLossResurfaceReplayTest : public ::testing::Test {
 protected:
  static void SetUpTestSuite() {
    fixture_ = new Fixture(LoadFixture());
    result_ = new ReplayResult(Replay(*fixture_));
  }
  static void TearDownTestSuite() {
    delete fixture_;
    delete result_;
  }

  static std::vector<const Step*> Steps(const std::string& phase = "",
                                        const std::string& kind = "") {
    std::vector<const Step*> out;
    for (const Step& s : result_->steps) {
      if ((phase.empty() || s.phase == phase) &&
          (kind.empty() || s.kind == kind)) {
        out.push_back(&s);
      }
    }
    return out;
  }

  static std::vector<std::string> Statuses(const std::string& phase,
                                           const std::string& kind) {
    std::vector<std::string> out;
    for (const Step* s : Steps(phase, kind)) out.push_back(s->status);
    return out;
  }

  static std::vector<const Step*> Measurements() {
    std::vector<const Step*> out;
    for (const Step& s : result_->steps) {
      if (s.kind != "IMU") out.push_back(&s);
    }
    return out;
  }

  static Fixture WithEvents(std::vector<Event> events) {
    Fixture f = *fixture_;
    f.events = std::move(events);
    return f;
  }

  static Fixture* fixture_;
  static ReplayResult* result_;
};

Fixture* DiveLockLossResurfaceReplayTest::fixture_ = nullptr;
ReplayResult* DiveLockLossResurfaceReplayTest::result_ = nullptr;

using Strings = std::vector<std::string>;

TEST_F(DiveLockLossResurfaceReplayTest, ReplayRan) {
  ASSERT_TRUE(result_->ok) << result_->error;
}

TEST_F(DiveLockLossResurfaceReplayTest, FixtureIsTimestampedAndOrdered) {
  const auto& events = fixture_->events;
  ASSERT_EQ(events.size(), kEventCount);
  for (size_t i = 1; i < events.size(); ++i) {
    EXPECT_LE(events[i - 1].t_ms, events[i].t_ms) << i;
  }
  EXPECT_GE(events[0].t_ms, fixture_->init_t_ms);
  int64_t expected_imu_ms = 1000;
  for (const Event& e : events) {
    if (e.kind != "IMU") continue;
    EXPECT_EQ(e.t_ms, expected_imu_ms);
    expected_imu_ms += 1000;
  }
  std::vector<size_t> order;
  std::set<std::string> seen;
  for (const Event& e : events) {
    order.push_back(std::find(kPhases.begin(), kPhases.end(), e.phase) -
                    kPhases.begin());
    seen.insert(e.phase);
  }
  EXPECT_TRUE(std::is_sorted(order.begin(), order.end()));
  EXPECT_EQ(seen, std::set<std::string>(kPhases.begin(), kPhases.end()));
}

TEST_F(DiveLockLossResurfaceReplayTest, StatusesMatchFixtureExpectations) {
  const auto& events = fixture_->events;
  ASSERT_EQ(result_->steps.size(), events.size());
  for (size_t i = 0; i < events.size(); ++i) {
    const Event& ev = events[i];
    const Step& step = result_->steps[i];
    if (ev.kind == "IMU") {
      EXPECT_EQ(step.status, "OK");
    } else {
      EXPECT_EQ(step.status, ev.expect) << ev.kind << "@" << ev.t_ms;
      EXPECT_EQ(kExpectedStatuses.at(ev.kind).count(ev.expect), 1u);
    }
  }
}

TEST_F(DiveLockLossResurfaceReplayTest, PhaseStatusAssertions) {
  EXPECT_EQ(Statuses("SURFACE_FIX", "SURFACE"),
            (Strings{"OK_ACCEPT", "SKIPPED_POLICY", "OK_ACCEPT"}));
  EXPECT_EQ(Statuses("SURFACE_FIX", "DEPTH"), (Strings(2, "OK_ACCEPT")));
  EXPECT_EQ(Statuses("DIVE", "SURFACE"), (Strings(3, "SKIPPED_POLICY")));
  EXPECT_EQ(Statuses("DIVE", "DEPTH"),
            (Strings{"OK_ACCEPT", "OK_ACCEPT", "SKIPPED_INVALID"}));
  EXPECT_EQ(Statuses("BOTTOM_LOCK", "DVL_BT"),
            (Strings{"OK_ACCEPT", "OK_REJECT", "OK_ACCEPT"}));
  EXPECT_EQ(Statuses("LOCK_LOSS", "DVL_BT"), (Strings(4, "SKIPPED_LOCK_LOSS")));
  EXPECT_EQ(Statuses("WATER_TRACK", "DVL_WT"),
            (Strings{"SKIPPED_MISSING_CURRENT", "OK_ACCEPT", "OK_ACCEPT",
                     "OK_REJECT"}));
  EXPECT_EQ(Statuses("ALTITUDE", "ALT"),
            (Strings{"SKIPPED_MISSING_SEAFLOOR", "OK_ACCEPT", "OK_REJECT",
                     "OK_ACCEPT"}));
  EXPECT_EQ(
      Statuses("RESURFACE", "SURFACE"),
      (Strings{"SKIPPED_POLICY", "SKIPPED_POLICY", "OK_ACCEPT", "OK_ACCEPT"}));
  EXPECT_EQ(Statuses("RESURFACE", "DEPTH"), (Strings(2, "OK_ACCEPT")));
}

TEST_F(DiveLockLossResurfaceReplayTest, SkipAndRejectLeaveStateBitIdentical) {
  size_t count = 0;
  for (const Step* s : Measurements()) {
    if (s->status == "OK_ACCEPT") continue;
    ++count;
    EXPECT_EQ(s->x_after, s->x_before) << s->kind << "@" << s->t_ms;
    EXPECT_EQ(s->p_after, s->p_before) << s->kind << "@" << s->t_ms;
  }
  EXPECT_EQ(count, kMeasurementCount - kAcceptCount);
}

TEST_F(DiveLockLossResurfaceReplayTest,
       AcceptChangesStateAndShrinksObservedVariance) {
  size_t accepts = 0;
  for (const Step* s : Measurements()) {
    if (s->status != "OK_ACCEPT") continue;
    ++accepts;
    EXPECT_NE(s->p_after, s->p_before);
    const auto before = Diag(s->p_before);
    const auto after = Diag(s->p_after);
    for (int i : kObserved.at(s->kind)) {
      EXPECT_LE(after[i], before[i] + kSlack) << s->kind << "@" << s->t_ms;
      EXPECT_LE(after[i], kAcceptCeiling.at(s->kind) + kSlack)
          << s->kind << "@" << s->t_ms;
      EXPECT_GT(after[i], 0.0) << s->kind << "@" << s->t_ms;
    }
  }
  EXPECT_EQ(accepts, kAcceptCount);
}

TEST_F(DiveLockLossResurfaceReplayTest,
       CovarianceStaysSymmetricPositiveDiagonal) {
  for (const Step& s : result_->steps) {
    for (int i = 0; i < kN; ++i) {
      EXPECT_GT(s.p_after[i * kN + i], 0.0) << s.kind << "@" << s.t_ms;
      for (int j = i + 1; j < kN; ++j) {
        EXPECT_NEAR(s.p_after[i * kN + j], s.p_after[j * kN + i], 1e-9)
            << s.t_ms;
      }
    }
  }
}

TEST_F(DiveLockLossResurfaceReplayTest, LockLossBoundsVelocityVariance) {
  const auto lock = Steps("LOCK_LOSS");
  ASSERT_FALSE(lock.empty());
  for (const Step* s : lock) {
    EXPECT_LE(Diag(s->p_after)[kDvIndex], kLockLossDvCeiling) << s->t_ms;
  }
  const double start = Diag(Steps("BOTTOM_LOCK").back()->p_after)[kDvIndex];
  const double end = Diag(lock.back()->p_after)[kDvIndex];
  EXPECT_GT(end, start);
  const Step* recovered = Steps("WATER_TRACK", "DVL_WT")[1];
  EXPECT_EQ(recovered->status, "OK_ACCEPT");
  EXPECT_LT(Diag(recovered->p_after)[kDvIndex], end);
}

TEST_F(DiveLockLossResurfaceReplayTest,
       ImuPropagationContinuesThroughLockLoss) {
  const auto imu = Steps("LOCK_LOSS", "IMU");
  EXPECT_EQ(imu.size(), 20u);
  for (const Step* s : imu) {
    EXPECT_NE(s->p_after, s->p_before);
    EXPECT_EQ(s->status, "OK");
  }
}

TEST_F(DiveLockLossResurfaceReplayTest, FinalStateWithinPinnedTolerances) {
  const EskfNominal& x = result_->x;
  for (int i = 0; i < 3; ++i) {
    EXPECT_NEAR(x.p_enu[i], kTruthP[i], kTruthPTol) << i;
    EXPECT_NEAR(x.v_body[i], kTruthV[i], kTruthVTol) << i;
  }
  double norm2 = 0.0;
  for (double q : x.q_wxyz) norm2 += q * q;
  EXPECT_NEAR(std::sqrt(norm2), 1.0, 1e-12);
}

TEST_F(DiveLockLossResurfaceReplayTest, GoldenFinalStateAndCovariance) {
  const NominalArray got = result_->x.ToArray();
  for (int i = 0; i < kNominalDim; ++i) {
    EXPECT_NEAR(got[i], kGoldenNominal[i], kTol) << i;
  }
  const auto diag = Diag(result_->p.row_major());
  for (int i = 0; i < kN; ++i) EXPECT_NEAR(diag[i], kGoldenPDiag[i], kTol) << i;
}

TEST_F(DiveLockLossResurfaceReplayTest, TwoReplaysHaveIdenticalDigest) {
  const ReplayResult a = Replay(*fixture_);
  Fixture reparsed;
  std::string error;
  ASSERT_TRUE(ParseFixture(LoadFixtureText(), &reparsed, &error)) << error;
  const ReplayResult b = Replay(reparsed);
  ASSERT_TRUE(a.ok);
  ASSERT_TRUE(b.ok);
  EXPECT_EQ(FullDigest(a), FullDigest(b));
  EXPECT_EQ(StatusDigest(a), StatusDigest(b));
  EXPECT_EQ(a.x.ToArray(), b.x.ToArray());
  EXPECT_EQ(a.p.row_major(), b.p.row_major());
  EXPECT_EQ(FullDigest(*result_), FullDigest(a));
}

TEST_F(DiveLockLossResurfaceReplayTest, GoldenStatusDigest) {
  EXPECT_EQ(StatusDigest(*result_), kStatusDigest);
}

TEST_F(DiveLockLossResurfaceReplayTest, DigestDetectsAChangedStatusOrState) {
  const ReplayResult shorter = Replay(*fixture_, kEventCount - 1);
  ASSERT_TRUE(shorter.ok);
  EXPECT_NE(FullDigest(shorter), FullDigest(*result_));
  EXPECT_NE(StatusDigest(shorter), StatusDigest(*result_));
}

TEST_F(DiveLockLossResurfaceReplayTest, EmptyEpisode) {
  const ReplayResult r = Replay(*fixture_, 0);
  ASSERT_TRUE(r.ok);
  EXPECT_TRUE(r.steps.empty());
  EXPECT_EQ(r.x.ToArray(), fixture_->init_x.ToArray());
  EXPECT_EQ(r.p.row_major(), DiagCov(fixture_->p0_diag).row_major());
  EXPECT_EQ(FullDigest(r), FullDigest(Replay(*fixture_, 0)));
  const ReplayResult none = Replay(WithEvents({}));
  ASSERT_TRUE(none.ok);
  EXPECT_TRUE(none.steps.empty());
}

TEST_F(DiveLockLossResurfaceReplayTest, OneEventEpisode) {
  const ReplayResult r = Replay(*fixture_, 1);
  ASSERT_TRUE(r.ok);
  ASSERT_EQ(r.steps.size(), 1u);
  EXPECT_EQ(r.steps[0].kind, "IMU");
  EXPECT_NE(r.x.ToArray(), fixture_->init_x.ToArray());
  const auto first =
      std::find_if(fixture_->events.begin(), fixture_->events.end(),
                   [](const Event& e) { return e.kind != "IMU"; });
  ASSERT_NE(first, fixture_->events.end());
  const ReplayResult m = Replay(WithEvents({*first}));
  ASSERT_TRUE(m.ok);
  EXPECT_EQ(m.steps[0].status, "OK_ACCEPT");
}

TEST_F(DiveLockLossResurfaceReplayTest, EndOfStreamPrefixMatchesFullReplay) {
  const size_t n = fixture_->events.size();
  for (size_t count :
       {size_t{1}, size_t{2}, size_t{40}, size_t{100}, n - 1, n}) {
    const ReplayResult prefix = Replay(*fixture_, count);
    ASSERT_TRUE(prefix.ok);
    const Step& step = result_->steps[count - 1];
    EXPECT_EQ(prefix.x.ToArray(), step.x_after) << count;
    EXPECT_EQ(prefix.p.row_major(), step.p_after) << count;
  }
  const ReplayResult past = Replay(*fixture_, n + 50);
  ASSERT_TRUE(past.ok);
  EXPECT_EQ(past.steps.size(), n);
  EXPECT_EQ(FullDigest(past), FullDigest(*result_));
}

TEST_F(DiveLockLossResurfaceReplayTest,
       UnknownAndTruncatedEventsAreTypedErrors) {
  const std::string text = LoadFixtureText();
  Fixture f;
  std::string error;
  EXPECT_FALSE(ParseFixture(text + "SONAR 200000 DIVE 1\n", &f, &error));
  EXPECT_FALSE(ParseFixture(text + "IMU 200000 DIVE 0 0 9.8\n", &f, &error));
  EXPECT_FALSE(ParseFixture(text + "IMU 200000 NOT_A_PHASE 0 0 9.8 0 0 0\n", &f,
                            &error));
  EXPECT_FALSE(ParseFixture("IMU 1000 DIVE 0 0 9.8 0 0 0\n", &f, &error));
}

TEST_F(DiveLockLossResurfaceReplayTest, OutOfOrderTimestampIsATypedError) {
  std::vector<Event> events = fixture_->events;
  std::swap(events[0], events[3]);
  const ReplayResult r = Replay(WithEvents(events));
  EXPECT_FALSE(r.ok);
  EXPECT_FALSE(r.error.empty());
}

TEST_F(DiveLockLossResurfaceReplayTest, ReplayNeverMutatesTheFixture) {
  const Fixture copy = *fixture_;
  Replay(*fixture_);
  ASSERT_EQ(copy.events.size(), fixture_->events.size());
  for (size_t i = 0; i < copy.events.size(); ++i) {
    const Event& a = copy.events[i];
    const Event& b = fixture_->events[i];
    EXPECT_EQ(a.kind, b.kind);
    EXPECT_EQ(a.t_ms, b.t_ms);
    EXPECT_EQ(a.imu.accel_m_s2, b.imu.accel_m_s2);
    EXPECT_EQ(a.depth.depth_m, b.depth.depth_m);
    EXPECT_EQ(a.surface.position_en_m, b.surface.position_en_m);
    EXPECT_EQ(a.bottom_track.velocity_body_m_s,
              b.bottom_track.velocity_body_m_s);
  }
}

}  // namespace
}  // namespace intrinsic::estimation
