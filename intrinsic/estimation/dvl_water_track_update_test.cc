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

// Mirrors dvl_water_track_update_test.py.

#include "intrinsic/estimation/dvl_water_track_update.h"

#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <limits>
#include <map>
#include <utility>
#include <vector>

#include "intrinsic/estimation/eskf_state.h"

namespace intrinsic::estimation {
namespace {

constexpr int kN = kCovDim;
constexpr double kTol = 1e-9;
constexpr double kChi2 = 7.815;
const double kNan = std::numeric_limits<double>::quiet_NaN();
const double kInf = std::numeric_limits<double>::infinity();
using Status = DvlWaterTrackStatus;
using Overrides = std::map<std::pair<int, int>, double>;

// P = value * I, with optional {(row, col): value} overrides.
EskfCovariance DiagP(double value, const Overrides &overrides = {}) {
  std::vector<double> v(kN * kN, 0.0);
  for (int i = 0; i < kN; ++i)
    v[i * kN + i] = value;
  for (const auto &[ij, o] : overrides)
    v[ij.first * kN + ij.second] = o;
  return *EskfCovariance::FromRowMajor(v);
}

std::array<double, 9> RDiag(double value) {
  return {value, 0.0, 0.0, 0.0, value, 0.0, 0.0, 0.0, value};
}

DvlWaterTrackSample Sample(std::array<double, 3> v = {0.0, 0.0, 0.0},
                           std::array<double, 9> r = {0.01, 0.0, 0.0, 0.0, 0.01,
                                                      0.0, 0.0, 0.0, 0.01},
                           bool valid = true) {
  DvlWaterTrackSample z;
  z.velocity_body_m_s = v;
  z.R_body = r;
  z.valid = valid;
  return z;
}

WaterCurrentEstimate Current(std::array<double, 3> v = {0.0, 0.0, 0.0},
                             bool present = true) {
  WaterCurrentEstimate c;
  c.present = present;
  c.v_enu_m_s = v;
  return c;
}

std::array<double, kN> Diag(const EskfCovariance &p) {
  std::array<double, kN> d;
  for (int i = 0; i < kN; ++i)
    d[i] = p.At(i, i);
  return d;
}

// Deliberately asymmetric, finite, deterministic.
EskfCovariance GoldenP() {
  std::vector<double> v(kN * kN);
  for (int i = 0; i < kN; ++i) {
    for (int j = 0; j < kN; ++j) {
      v[i * kN + j] =
          i == j ? 0.05 + 0.01 * i : 0.002 * (((i * 7 + j * 3) % 5) - 2);
    }
  }
  return *EskfCovariance::FromRowMajor(v);
}

EskfNominal GoldenX() {
  EskfNominal x;
  x.p_enu = {1.0, 2.0, -3.0};
  x.q_wxyz = {0.9, 0.1, -0.2, 0.3};
  x.v_body = {0.5, -0.1, 0.2};
  x.b_a = {0.01, -0.02, 0.03};
  x.b_g = {0.001, 0.002, -0.003};
  return x;
}

const std::array<double, 9> kGoldenR = {0.02,  0.004, 0.0,   0.004, 0.03,
                                        0.002, 0.0,   0.002, 0.025};
const std::array<double, 3> kGoldenZ = {0.15, 0.25, -0.05};
const std::array<double, 3> kGoldenC = {0.4, -0.3, 0.1};

// Independent numpy evaluation, same constants as the Python test.
constexpr double kGoldenD2 = 0.4719881583806437;
constexpr std::array<double, 9> kGoldenS = {
    0.15395940166204986,    0.014026326869806096, -0.0003772011080332405,
    0.014026326869806097,   0.1536128753462604,   0.00485063490304709,
    -0.0003772011080332404, 0.004850634903047091, 0.17945670470914127};
constexpr std::array<double, kNominalDim> kGoldenNominal = {
    1.0003685806512679,   2.0025635648950675,    -2.999355088831938,
    0.9311518839827754,   0.12870942214744951,   -0.19092067948327526,
    0.2827354731041758,   0.38113109515252125,   -0.18385691717479807,
    0.05853281176318678,  0.006613701232377509,  -0.019631419348732172,
    0.03256356489506766,  0.0016449111680622566, 0.0018092420532247464,
    -0.006386298767622491};
constexpr std::array<double, kN> kGoldenPDiag = {
    0.0499941380243371,   0.059907913801712745, 0.06987733146632756,
    0.07004439083278782,  0.0883869300385078,   0.08337714480699841,
    0.030114374741023367, 0.025420693106184777, 0.033258788189523665,
    0.13997524540177425,  0.14999413802433714,  0.15990791380171274,
    0.16987733146632755,  0.17994428989577507,  0.18997524540177424};

void ExpectUnchanged(const DvlWaterTrackResult &r, Status status,
                     bool evaluated = false) {
  EXPECT_EQ(r.status, status);
  EXPECT_EQ(r.evaluated, evaluated);
  EXPECT_FALSE(r.accepted);
  EXPECT_FALSE(r.nominal.has_value());
  EXPECT_FALSE(r.P.has_value());
  EXPECT_FALSE(r.delta_x.has_value());
  EXPECT_EQ(r.diagnostics.has_value(), evaluated);
}

DvlWaterTrackResult Update(const EskfNominal &x, const EskfCovariance &p,
                           const DvlWaterTrackSample &z,
                           const WaterCurrentEstimate &c, double t = kChi2) {
  return UpdateDvlWaterTrack(x, p, z, c, t);
}

TEST(DvlWaterTrackUpdateTest, AcceptHoverZeroCurrent) {
  // S = 0.04 + 0.01 = 0.05, K = 0.8 on dv, dv variance 0.04 -> 0.008.
  const EskfNominal x;
  const DvlWaterTrackResult r = Update(x, DiagP(0.04), Sample(), Current());
  ASSERT_EQ(r.status, Status::kOkAccept);
  EXPECT_TRUE(r.evaluated);
  EXPECT_TRUE(r.accepted);
  EXPECT_EQ(r.nominal->ToArray(), x.ToArray());
  EXPECT_EQ(r.delta_x->ToArray(), EskfError().ToArray());
  const auto diag = Diag(*r.P);
  for (int i = 0; i < kN; ++i) {
    EXPECT_NEAR(diag[i], (i >= 6 && i <= 8) ? 0.008 : 0.04, kTol) << i;
    for (int j = 0; j < kN; ++j) {
      if (i != j) {
        EXPECT_EQ(r.P->At(i, j), 0.0);
      }
    }
  }
  EXPECT_EQ(r.diagnostics->mahalanobis_sq, 0.0);
  EXPECT_EQ(r.diagnostics->innovation_norm, 0.0);
  EXPECT_EQ(r.diagnostics->threshold, kChi2);
  EXPECT_EQ(r.diagnostics->dof, 3);
}

TEST(DvlWaterTrackUpdateTest, NonzeroCurrentIdentityAttitudeZeroInnovation) {
  // c_b = c_enu = (0.5, 0, 0), h = (-0.5, 0, 0), z = h, so nu = 0.
  // H theta = [[0,0,0],[0,0,0.5],[0,-0.5,0]], S = diag(0.05, 0.06, 0.06).
  const EskfNominal x;
  const DvlWaterTrackResult r = Update(x, DiagP(0.04), Sample({-0.5, 0.0, 0.0}),
                                       Current({0.5, 0.0, 0.0}));
  ASSERT_EQ(r.status, Status::kOkAccept);
  EXPECT_LT(std::fabs(r.diagnostics->mahalanobis_sq), 1e-15);
  EXPECT_LT(r.diagnostics->innovation_norm, 1e-15);
  EXPECT_EQ(r.nominal->ToArray(), x.ToArray());
  EXPECT_EQ(r.delta_x->ToArray(), EskfError().ToArray());
  const std::array<double, 3> s_diag = {0.05, 0.06, 0.06};
  for (int i = 0; i < 3; ++i) {
    EXPECT_NEAR(r.diagnostics->S[i * 3 + i], s_diag[i], kTol);
  }
  std::array<double, kN> expected;
  expected.fill(0.04);
  expected[6] = 0.04 - 0.04 * 0.04 / 0.05;
  expected[7] = 0.04 - 0.04 * 0.04 / 0.06;
  expected[8] = 0.04 - 0.04 * 0.04 / 0.06;
  expected[4] = 0.04 - (0.04 * 0.5) * (0.04 * 0.5) / 0.06;
  expected[5] = 0.04 - (0.04 * 0.5) * (0.04 * 0.5) / 0.06;
  const auto diag = Diag(*r.P);
  for (int i = 0; i < kN; ++i)
    EXPECT_NEAR(diag[i], expected[i], kTol) << i;
}

TEST(DvlWaterTrackUpdateTest, AttitudeCouplingInjectsDtheta) {
  // nu = (0, 0.06, 0). K[dv_y] = 0.04 / 0.06, K[dtheta_z] = 0.02 / 0.06.
  const DvlWaterTrackResult r =
      Update(EskfNominal(), DiagP(0.04), Sample({-0.5, 0.06, 0.0}),
             Current({0.5, 0.0, 0.0}));
  ASSERT_EQ(r.status, Status::kOkAccept);
  EXPECT_NEAR(r.delta_x->dv[1], 0.04, kTol);
  EXPECT_NEAR(r.delta_x->dtheta[2], 0.02, kTol);
  EXPECT_NEAR(r.delta_x->dtheta[0], 0.0, kTol);
  EXPECT_NEAR(r.nominal->v_body[1], 0.04, kTol);
  const double norm = std::sqrt(1.0 + 0.01 * 0.01);
  EXPECT_NEAR(r.nominal->q_wxyz[0], 1.0 / norm, kTol);
  EXPECT_NEAR(r.nominal->q_wxyz[3], 0.01 / norm, kTol);
}

TEST(DvlWaterTrackUpdateTest, CurrentIsRotatedIntoBody) {
  // q = +90 deg about z maps body x to world y. c_enu = (0, 1, 0) is then
  // c_b = R^T c = (1, 0, 0), so z = -c_b gives nu = 0.
  EskfNominal x;
  const double s = std::sqrt(0.5);
  x.q_wxyz = {s, 0.0, 0.0, s};
  const DvlWaterTrackResult r = Update(x, DiagP(0.04), Sample({-1.0, 0.0, 0.0}),
                                       Current({0.0, 1.0, 0.0}));
  ASSERT_EQ(r.status, Status::kOkAccept);
  EXPECT_LT(r.diagnostics->innovation_norm, 1e-12);
}

TEST(DvlWaterTrackUpdateTest, MissingCurrentSkips) {
  const EskfNominal x = GoldenX();
  const EskfCovariance p = GoldenP();
  const auto x_before = x.ToArray();
  const auto p_before = p.row_major();
  const std::vector<WaterCurrentEstimate> cases = {
      Current({0.4, 0.0, 0.0}, /*present=*/false),
      Current({kNan, 0.0, 0.0}),
      Current({0.0, kInf, 0.0}),
      Current({0.0, 0.0, -kInf}),
  };
  for (const WaterCurrentEstimate &c : cases) {
    ExpectUnchanged(Update(x, p, Sample(kGoldenZ, kGoldenR), c),
                    Status::kSkippedMissingCurrent);
  }
  EXPECT_EQ(x.ToArray(), x_before);
  EXPECT_EQ(p.row_major(), p_before);
}

TEST(DvlWaterTrackUpdateTest, MissingCurrentWinsOverBadContents) {
  std::array<double, 9> r_nan;
  r_nan.fill(kNan);
  ExpectUnchanged(Update(EskfNominal(), DiagP(0.04),
                         Sample({kNan, 0.0, 0.0}, r_nan),
                         Current({0.0, 0.0, 0.0}, /*present=*/false), kNan),
                  Status::kSkippedMissingCurrent);
}

TEST(DvlWaterTrackUpdateTest, DefaultCurrentIsMissing) {
  ExpectUnchanged(
      Update(EskfNominal(), DiagP(0.04), Sample(), WaterCurrentEstimate()),
      Status::kSkippedMissingCurrent);
}

TEST(DvlWaterTrackUpdateTest, UnusedCurrentCovarianceDoesNotMatter) {
  const DvlWaterTrackSample z = Sample(kGoldenZ, kGoldenR);
  WaterCurrentEstimate c = Current(kGoldenC);
  const DvlWaterTrackResult base = Update(GoldenX(), GoldenP(), z, c);
  c.R_current_enu.fill(kNan);
  const DvlWaterTrackResult other = Update(GoldenX(), GoldenP(), z, c);
  EXPECT_EQ(base.nominal->ToArray(), other.nominal->ToArray());
  EXPECT_EQ(base.P->row_major(), other.P->row_major());
}

TEST(DvlWaterTrackUpdateTest, CovarianceIsSymmetricAndDoesNotGrow) {
  const DvlWaterTrackResult r = Update(
      GoldenX(), GoldenP(), Sample(kGoldenZ, kGoldenR), Current(kGoldenC));
  ASSERT_EQ(r.status, Status::kOkAccept);
  const EskfCovariance prior = GoldenP();
  for (int i = 0; i < kN; ++i) {
    for (int j = 0; j < kN; ++j)
      EXPECT_EQ(r.P->At(i, j), r.P->At(j, i));
    EXPECT_LE(r.P->At(i, i), prior.At(i, i) + 1e-15);
  }
}

TEST(DvlWaterTrackUpdateTest, AsymmetricPMatchesItsSymmetrization) {
  const EskfCovariance raw = GoldenP();
  std::vector<double> sym(kN * kN);
  for (int i = 0; i < kN; ++i) {
    for (int j = 0; j < kN; ++j) {
      sym[i * kN + j] = 0.5 * (raw.At(i, j) + raw.At(j, i));
    }
  }
  const DvlWaterTrackSample z = Sample(kGoldenZ, kGoldenR);
  const WaterCurrentEstimate c = Current(kGoldenC);
  const DvlWaterTrackResult a = Update(GoldenX(), raw, z, c);
  const DvlWaterTrackResult b =
      Update(GoldenX(), *EskfCovariance::FromRowMajor(sym), z, c);
  EXPECT_EQ(a.nominal->ToArray(), b.nominal->ToArray());
  EXPECT_EQ(a.P->row_major(), b.P->row_major());
}

TEST(DvlWaterTrackUpdateTest, DeepRejectLeavesEverythingUnchanged) {
  const EskfNominal x;
  const EskfCovariance p = DiagP(0.04);
  const auto x_before = x.ToArray();
  const auto p_before = p.row_major();
  // nu = (1.5, 0, 0), S_xx = 0.05, d^2 = 2.25 / 0.05 = 45.
  const DvlWaterTrackResult r =
      Update(x, p, Sample({1.0, 0.0, 0.0}), Current({0.5, 0.0, 0.0}));
  ExpectUnchanged(r, Status::kOkReject, /*evaluated=*/true);
  EXPECT_NEAR(r.diagnostics->mahalanobis_sq, 45.0, kTol);
  EXPECT_EQ(r.diagnostics->dof, 3);
  EXPECT_EQ(x.ToArray(), x_before);
  EXPECT_EQ(p.row_major(), p_before);
}

TEST(DvlWaterTrackUpdateTest, ThresholdBoundary) {
  const DvlWaterTrackSample z = Sample({0.1, 0.0, 0.0});
  EXPECT_EQ(Update(EskfNominal(), DiagP(0.04), z, Current(), 0.2 + 1e-6).status,
            Status::kOkAccept);
  EXPECT_EQ(Update(EskfNominal(), DiagP(0.04), z, Current(), 0.2 - 1e-6).status,
            Status::kOkReject);
}

TEST(DvlWaterTrackUpdateTest, InvalidSampleFlag) {
  ExpectUnchanged(Update(EskfNominal(), DiagP(0.04),
                         Sample({0.1, 0.0, 0.0}, RDiag(0.01), /*valid=*/false),
                         Current()),
                  Status::kSkippedInvalid);
}

TEST(DvlWaterTrackUpdateTest, InvalidVelocity) {
  for (const std::array<double, 3> &v : std::vector<std::array<double, 3>>{
           {kNan, 0.0, 0.0}, {0.0, kInf, 0.0}, {0.0, 0.0, -kInf}}) {
    ExpectUnchanged(Update(EskfNominal(), DiagP(0.04), Sample(v), Current()),
                    Status::kSkippedInvalid);
  }
}

TEST(DvlWaterTrackUpdateTest, InvalidR) {
  std::array<double, 9> bad_nan = RDiag(0.01);
  bad_nan[4] = kNan;
  std::array<double, 9> bad_inf = RDiag(0.01);
  bad_inf[0] = kInf;
  std::array<double, 9> asym = RDiag(0.01);
  asym[1] = 1e-11;
  std::array<double, 9> asym_big = RDiag(0.01);
  asym_big[2] = 0.25;
  asym_big[6] = -0.25;
  for (const auto &r : {bad_nan, bad_inf, asym, asym_big}) {
    ExpectUnchanged(
        Update(EskfNominal(), DiagP(0.04), Sample({0, 0, 0}, r), Current()),
        Status::kSkippedInvalid);
  }
}

TEST(DvlWaterTrackUpdateTest, RAsymmetryInsideToleranceEvaluates) {
  std::array<double, 9> r = RDiag(0.01);
  r[1] = 5e-13;
  EXPECT_EQ(Update(EskfNominal(), DiagP(0.04), Sample({0, 0, 0}, r), Current())
                .status,
            Status::kOkAccept);
}

TEST(DvlWaterTrackUpdateTest, InvalidThreshold) {
  for (double t : {0.0, -1.0, kNan, kInf}) {
    ExpectUnchanged(Update(EskfNominal(), DiagP(0.04), Sample(), Current(), t),
                    Status::kSkippedInvalid);
  }
}

TEST(DvlWaterTrackUpdateTest, InvalidP) {
  ExpectUnchanged(Update(EskfNominal(), EskfCovariance(), Sample(), Current()),
                  Status::kSkippedInvalid);
  ExpectUnchanged(
      Update(EskfNominal(), DiagP(0.04, {{{3, 4}, kNan}}), Sample(), Current()),
      Status::kSkippedInvalid);
}

TEST(DvlWaterTrackUpdateTest, InvalidNominalAndQuaternion) {
  std::vector<EskfNominal> bad(7);
  bad[0].p_enu = {kNan, 0.0, 0.0};
  bad[1].v_body = {0.0, kInf, 0.0};
  bad[2].b_a = {0.0, 0.0, kNan};
  bad[3].b_g = {kInf, 0.0, 0.0};
  bad[4].q_wxyz = {1.0, kNan, 0.0, 0.0};
  bad[5].q_wxyz = {0.0, 0.0, 0.0, 0.0};
  bad[6].q_wxyz = {1e-13, 0.0, 0.0, 0.0};
  for (const EskfNominal &x : bad) {
    ExpectUnchanged(Update(x, DiagP(0.04), Sample(), Current({1.0, 0.0, 0.0})),
                    Status::kSkippedInvalid);
  }
}

TEST(DvlWaterTrackUpdateTest, SingularR) {
  const std::array<double, 9> zero = {0, 0, 0, 0, 0, 0, 0, 0, 0};
  const std::array<double, 9> indefinite = {1, 2, 0, 2, 1, 0, 0, 0, 1};
  for (const auto &r : {zero, indefinite, RDiag(1e-12)}) {
    ExpectUnchanged(
        Update(EskfNominal(), DiagP(0.04), Sample({0, 0, 0}, r), Current()),
        Status::kSingular);
  }
}

TEST(DvlWaterTrackUpdateTest, RPivotJustAboveFloorEvaluates) {
  EXPECT_EQ(Update(EskfNominal(), DiagP(0.04), Sample({0, 0, 0}, RDiag(2e-12)),
                   Current())
                .status,
            Status::kOkAccept);
}

TEST(DvlWaterTrackUpdateTest, NonFiniteUpdateLeavesInputsUnchanged) {
  // d^2 = 4 / 2 = 2 passes the gate, but the Joseph product overflows:
  // K[0] = 1e200 / 2, and K[0] * P[6, 0] = 5e399.
  const EskfCovariance p = DiagP(1.0, {{{0, 6}, 1e200}, {{6, 0}, 1e200}});
  const EskfNominal x;
  const DvlWaterTrackResult r =
      Update(x, p, Sample({2.0, 0.0, 0.0}, RDiag(1.0)), Current());
  ExpectUnchanged(r, Status::kNonFinite, /*evaluated=*/true);
  EXPECT_EQ(r.diagnostics->dof, 3);
  EXPECT_EQ(x.ToArray(), EskfNominal().ToArray());
}

TEST(DvlWaterTrackUpdateTest, GateOverflowIsNonFinite) {
  ExpectUnchanged(Update(EskfNominal(), DiagP(0.04, {{{6, 6}, 1.5e308}}),
                         Sample({0, 0, 0}, RDiag(1.5e308)), Current()),
                  Status::kNonFinite);
}

TEST(DvlWaterTrackUpdateTest, HugeCurrentOverflowIsNonFinite) {
  EskfNominal x;
  x.q_wxyz = {0.9, 0.1, -0.2, 0.3};
  ExpectUnchanged(
      Update(x, DiagP(0.04), Sample(), Current({1.7e308, 1.7e308, 1.7e308})),
      Status::kNonFinite);
}

TEST(DvlWaterTrackUpdateTest, GoldenCase) {
  const DvlWaterTrackResult r = Update(
      GoldenX(), GoldenP(), Sample(kGoldenZ, kGoldenR), Current(kGoldenC));
  ASSERT_EQ(r.status, Status::kOkAccept);
  EXPECT_NEAR(r.diagnostics->mahalanobis_sq, kGoldenD2, kTol);
  for (int i = 0; i < 9; ++i) {
    EXPECT_NEAR(r.diagnostics->S[i], kGoldenS[i], kTol) << i;
  }
  const auto got = r.nominal->ToArray();
  for (int i = 0; i < kNominalDim; ++i) {
    EXPECT_NEAR(got[i], kGoldenNominal[i], kTol) << i;
  }
  const auto diag = Diag(*r.P);
  for (int i = 0; i < kN; ++i)
    EXPECT_NEAR(diag[i], kGoldenPDiag[i], kTol) << i;
}

TEST(DvlWaterTrackUpdateTest, InputsAreNotModified) {
  const EskfNominal x = GoldenX();
  const EskfCovariance p = GoldenP();
  const DvlWaterTrackSample z = Sample(kGoldenZ, kGoldenR);
  const WaterCurrentEstimate c = Current(kGoldenC);
  const auto x_before = x.ToArray();
  const auto p_before = p.row_major();
  Update(x, p, z, c);
  EXPECT_EQ(x.ToArray(), x_before);
  EXPECT_EQ(p.row_major(), p_before);
  EXPECT_EQ(z.velocity_body_m_s, kGoldenZ);
  EXPECT_EQ(z.R_body, kGoldenR);
  EXPECT_EQ(c.v_enu_m_s, kGoldenC);
}

TEST(DvlWaterTrackUpdateTest, Determinism) {
  const DvlWaterTrackSample z = Sample(kGoldenZ, kGoldenR);
  const WaterCurrentEstimate c = Current(kGoldenC);
  const DvlWaterTrackResult a = Update(GoldenX(), GoldenP(), z, c);
  const DvlWaterTrackResult b = Update(GoldenX(), GoldenP(), z, c);
  EXPECT_EQ(a.nominal->ToArray(), b.nominal->ToArray());
  EXPECT_EQ(a.P->row_major(), b.P->row_major());
  EXPECT_EQ(a.diagnostics->mahalanobis_sq, b.diagnostics->mahalanobis_sq);
  EXPECT_EQ(a.diagnostics->S, b.diagnostics->S);
}

} // namespace
} // namespace intrinsic::estimation
