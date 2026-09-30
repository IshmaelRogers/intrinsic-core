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

// Mirrors dvl_bottom_track_update_test.py.

#include "intrinsic/estimation/dvl_bottom_track_update.h"

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
using Status = DvlUpdateStatus;
using Overrides = std::map<std::pair<int, int>, double>;

// P = value * I, with optional {(row, col): value} overrides.
EskfCovariance DiagP(double value, const Overrides& overrides = {}) {
  std::vector<double> v(kN * kN, 0.0);
  for (int i = 0; i < kN; ++i) v[i * kN + i] = value;
  for (const auto& [ij, o] : overrides) v[ij.first * kN + ij.second] = o;
  return *EskfCovariance::FromRowMajor(v);
}

std::array<double, 9> RDiag(double value) {
  return {value, 0.0, 0.0, 0.0, value, 0.0, 0.0, 0.0, value};
}

DvlBottomTrackSample Sample(std::array<double, 3> v = {0.0, 0.0, 0.0},
                            std::array<double, 9> r = {0.01, 0.0, 0.0, 0.0,
                                                       0.01, 0.0, 0.0, 0.0,
                                                       0.01},
                            bool lock = true, bool valid = true) {
  DvlBottomTrackSample z;
  z.velocity_body_m_s = v;
  z.R_body = r;
  z.bottom_lock = lock;
  z.valid = valid;
  return z;
}

std::array<double, kN> Diag(const EskfCovariance& p) {
  std::array<double, kN> d;
  for (int i = 0; i < kN; ++i) d[i] = p.At(i, i);
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
const std::array<double, 3> kGoldenZ = {0.55, -0.12, 0.25};

// Independent numpy evaluation: K = sym(P) H^T S^-1, Joseph, right-error
// inject. Same constants as the Python test.
constexpr double kGoldenD2 = 0.03860645238264895;
constexpr std::array<double, kNominalDim> kGoldenNominal = {
    1.0005583105621352,    1.9986182873297131,    -2.998678051989587,
    0.9234058908586854,    0.10196666905428795,   -0.20512914674990052,
    0.3079714147175167,    0.5428508170289345,    -0.11761626070688205,
    0.24222961984665276,   0.01055831056213513,   -0.01944168943786487,
    0.028618287329713138,  0.0023219480104131624, 0.000943143535603439,
    -0.0024416894378648706};
constexpr std::array<double, kN> kGoldenPDiag = {
    0.049980021774181524, 0.0598614784238409,   0.06987602105831801,
    0.07988148283170414,  0.08998002177418153,  0.09998002177418154,
    0.01685309586108271,  0.023918402828986506, 0.020952066641589353,
    0.13998002177418153,  0.14998002177418154,  0.1598614784238409,
    0.169876021058318,    0.17988148283170413,  0.18998002177418152};

void ExpectUnchanged(const DvlUpdateResult& r, Status status,
                     bool evaluated = false) {
  EXPECT_EQ(r.status, status);
  EXPECT_EQ(r.evaluated, evaluated);
  EXPECT_FALSE(r.accepted);
  EXPECT_FALSE(r.nominal.has_value());
  EXPECT_FALSE(r.P.has_value());
  EXPECT_FALSE(r.delta_x.has_value());
  EXPECT_EQ(r.diagnostics.has_value(), evaluated);
}

DvlUpdateResult Update(const EskfNominal& x, const EskfCovariance& p,
                       const DvlBottomTrackSample& z, double t = kChi2) {
  return UpdateDvlBottomTrack(x, p, z, t);
}

TEST(DvlBottomTrackUpdateTest, AcceptHoverZeroInnovation) {
  // S = 0.04 + 0.01 = 0.05, K = 0.8 on dv, dv variance 0.04 -> 0.008.
  const EskfNominal x;
  const DvlUpdateResult r = Update(x, DiagP(0.04), Sample());
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

TEST(DvlBottomTrackUpdateTest, AcceptNonzeroInnovation) {
  // nu = (0.1, 0, 0), d^2 = 0.01 / 0.05 = 0.2, dv = 0.8 * 0.1 = 0.08.
  const DvlUpdateResult r =
      Update(EskfNominal(), DiagP(0.04), Sample({0.1, 0.0, 0.0}));
  ASSERT_EQ(r.status, Status::kOkAccept);
  EXPECT_NEAR(r.diagnostics->mahalanobis_sq, 0.2, kTol);
  EXPECT_NEAR(r.diagnostics->innovation_norm, 0.1, kTol);
  EXPECT_NEAR(r.nominal->v_body[0], 0.08, kTol);
  EXPECT_NEAR(r.nominal->v_body[1], 0.0, kTol);
  EXPECT_EQ(r.nominal->q_wxyz, (std::array<double, 4>{1.0, 0.0, 0.0, 0.0}));
  EXPECT_NEAR(r.P->At(6, 6), 0.008, kTol);
}

TEST(DvlBottomTrackUpdateTest, CorrelatedStateIsInjected) {
  // K[i, 0] = P[i, 6] / 0.05 and nu_x = 0.1.
  const EskfCovariance p = DiagP(0.04, {{{0, 6}, 0.02},
                                        {{6, 0}, 0.02},
                                        {{4, 6}, 0.01},
                                        {{6, 4}, 0.01},
                                        {{9, 6}, 0.005},
                                        {{6, 9}, 0.005},
                                        {{13, 6}, 0.0025},
                                        {{6, 13}, 0.0025}});
  const DvlUpdateResult r = Update(EskfNominal(), p, Sample({0.1, 0.0, 0.0}));
  ASSERT_EQ(r.status, Status::kOkAccept);
  const EskfNominal& n = *r.nominal;
  EXPECT_NEAR(n.p_enu[0], 0.04, kTol);
  EXPECT_NEAR(n.b_a[0], 0.01, kTol);
  EXPECT_NEAR(n.b_g[1], 0.005, kTol);
  // dtheta_y = 0.02 -> q = normalize(1, 0, 0.01, 0).
  const double norm = std::sqrt(1.0 + 0.01 * 0.01);
  EXPECT_NEAR(n.q_wxyz[0], 1.0 / norm, kTol);
  EXPECT_NEAR(n.q_wxyz[2], 0.01 / norm, kTol);
  EXPECT_NEAR(n.q_wxyz[1], 0.0, kTol);
  EXPECT_NEAR(n.q_wxyz[3], 0.0, kTol);
  double q_sq = 0.0;
  for (double c : n.q_wxyz) q_sq += c * c;
  EXPECT_NEAR(std::sqrt(q_sq), 1.0, kTol);
  EXPECT_NEAR(r.delta_x->dtheta[1], 0.02, kTol);
}

TEST(DvlBottomTrackUpdateTest, CovarianceIsSymmetricAndDoesNotGrow) {
  const DvlUpdateResult r =
      Update(GoldenX(), GoldenP(), Sample(kGoldenZ, kGoldenR));
  ASSERT_EQ(r.status, Status::kOkAccept);
  const EskfCovariance prior = GoldenP();
  for (int i = 0; i < kN; ++i) {
    for (int j = 0; j < kN; ++j) EXPECT_EQ(r.P->At(i, j), r.P->At(j, i));
    EXPECT_LE(r.P->At(i, i), prior.At(i, i) + 1e-15);
  }
}

TEST(DvlBottomTrackUpdateTest, AsymmetricPMatchesItsSymmetrization) {
  const EskfCovariance raw = GoldenP();
  std::vector<double> sym(kN * kN);
  for (int i = 0; i < kN; ++i) {
    for (int j = 0; j < kN; ++j) {
      sym[i * kN + j] = 0.5 * (raw.At(i, j) + raw.At(j, i));
    }
  }
  const DvlBottomTrackSample z = Sample(kGoldenZ, kGoldenR);
  const DvlUpdateResult a = Update(GoldenX(), raw, z);
  const DvlUpdateResult b =
      Update(GoldenX(), *EskfCovariance::FromRowMajor(sym), z);
  EXPECT_EQ(a.nominal->ToArray(), b.nominal->ToArray());
  EXPECT_EQ(a.P->row_major(), b.P->row_major());
}

TEST(DvlBottomTrackUpdateTest, DeepRejectLeavesEverythingUnchanged) {
  const EskfNominal x;
  const EskfCovariance p = DiagP(0.04);
  const auto x_before = x.ToArray();
  const auto p_before = p.row_major();
  const DvlUpdateResult r = Update(x, p, Sample({1.0, 0.0, 0.0}));
  ExpectUnchanged(r, Status::kOkReject, /*evaluated=*/true);
  EXPECT_NEAR(r.diagnostics->mahalanobis_sq, 20.0, kTol);
  EXPECT_EQ(r.diagnostics->dof, 3);
  EXPECT_EQ(x.ToArray(), x_before);
  EXPECT_EQ(p.row_major(), p_before);
}

TEST(DvlBottomTrackUpdateTest, ThresholdBoundary) {
  const DvlBottomTrackSample z = Sample({0.1, 0.0, 0.0});
  EXPECT_EQ(Update(EskfNominal(), DiagP(0.04), z, 0.2 + 1e-6).status,
            Status::kOkAccept);
  EXPECT_EQ(Update(EskfNominal(), DiagP(0.04), z, 0.2 - 1e-6).status,
            Status::kOkReject);
}

TEST(DvlBottomTrackUpdateTest, LockLossAndInvalidHealth) {
  for (const auto& [lock, valid] : std::vector<std::pair<bool, bool>>{
           {false, true}, {true, false}, {false, false}}) {
    const DvlUpdateResult r =
        Update(EskfNominal(), DiagP(0.04),
               Sample({0.1, 0.0, 0.0}, RDiag(0.01), lock, valid));
    ExpectUnchanged(r, Status::kSkippedLockLoss);
  }
}

TEST(DvlBottomTrackUpdateTest, LockLossWinsOverBadSampleContents) {
  std::array<double, 9> r_nan;
  r_nan.fill(kNan);
  ExpectUnchanged(Update(EskfNominal(), DiagP(0.04),
                         Sample({kNan, 0.0, 0.0}, r_nan, /*lock=*/false)),
                  Status::kSkippedLockLoss);
}

TEST(DvlBottomTrackUpdateTest, InvalidVelocity) {
  for (const std::array<double, 3>& v : std::vector<std::array<double, 3>>{
           {kNan, 0.0, 0.0}, {0.0, kInf, 0.0}, {0.0, 0.0, -kInf}}) {
    ExpectUnchanged(Update(EskfNominal(), DiagP(0.04), Sample(v)),
                    Status::kSkippedInvalid);
  }
}

TEST(DvlBottomTrackUpdateTest, InvalidR) {
  std::array<double, 9> bad_nan = RDiag(0.01);
  bad_nan[4] = kNan;
  std::array<double, 9> bad_inf = RDiag(0.01);
  bad_inf[0] = kInf;
  std::array<double, 9> asym = RDiag(0.01);
  asym[1] = 1e-11;
  std::array<double, 9> asym_big = RDiag(0.01);
  asym_big[2] = 0.25;
  asym_big[6] = -0.25;
  for (const auto& r : {bad_nan, bad_inf, asym, asym_big}) {
    ExpectUnchanged(Update(EskfNominal(), DiagP(0.04), Sample({0, 0, 0}, r)),
                    Status::kSkippedInvalid);
  }
}

TEST(DvlBottomTrackUpdateTest, RAsymmetryInsideToleranceEvaluates) {
  std::array<double, 9> r = RDiag(0.01);
  r[1] = 5e-13;
  EXPECT_EQ(Update(EskfNominal(), DiagP(0.04), Sample({0, 0, 0}, r)).status,
            Status::kOkAccept);
}

TEST(DvlBottomTrackUpdateTest, InvalidThreshold) {
  for (double t : {0.0, -1.0, kNan, kInf}) {
    ExpectUnchanged(Update(EskfNominal(), DiagP(0.04), Sample(), t),
                    Status::kSkippedInvalid);
  }
}

TEST(DvlBottomTrackUpdateTest, InvalidP) {
  ExpectUnchanged(Update(EskfNominal(), EskfCovariance(), Sample()),
                  Status::kSkippedInvalid);
  ExpectUnchanged(
      Update(EskfNominal(), DiagP(0.04, {{{3, 4}, kNan}}), Sample()),
      Status::kSkippedInvalid);
}

TEST(DvlBottomTrackUpdateTest, InvalidNominalAndQuaternion) {
  std::vector<EskfNominal> bad(7);
  bad[0].p_enu = {kNan, 0.0, 0.0};
  bad[1].v_body = {0.0, kInf, 0.0};
  bad[2].b_a = {0.0, 0.0, kNan};
  bad[3].b_g = {kInf, 0.0, 0.0};
  bad[4].q_wxyz = {1.0, kNan, 0.0, 0.0};
  bad[5].q_wxyz = {0.0, 0.0, 0.0, 0.0};
  bad[6].q_wxyz = {1e-13, 0.0, 0.0, 0.0};
  for (const EskfNominal& x : bad) {
    ExpectUnchanged(Update(x, DiagP(0.04), Sample()), Status::kSkippedInvalid);
  }
}

TEST(DvlBottomTrackUpdateTest, SingularR) {
  const std::array<double, 9> zero = {0, 0, 0, 0, 0, 0, 0, 0, 0};
  const std::array<double, 9> indefinite = {1, 2, 0, 2, 1, 0, 0, 0, 1};
  for (const auto& r : {zero, indefinite, RDiag(1e-12)}) {
    ExpectUnchanged(Update(EskfNominal(), DiagP(0.04), Sample({0, 0, 0}, r)),
                    Status::kSingular);
  }
}

TEST(DvlBottomTrackUpdateTest, RPivotJustAboveFloorEvaluates) {
  EXPECT_EQ(Update(EskfNominal(), DiagP(0.04), Sample({0, 0, 0}, RDiag(2e-12)))
                .status,
            Status::kOkAccept);
}

TEST(DvlBottomTrackUpdateTest, NonFiniteUpdateLeavesInputsUnchanged) {
  // d^2 = 4 / 2 = 2 passes the gate, but the Joseph product overflows:
  // K[0] = 1e200 / 2, and K[0] * P[6, 0] = 5e399.
  const EskfCovariance p = DiagP(1.0, {{{0, 6}, 1e200}, {{6, 0}, 1e200}});
  const EskfNominal x;
  const DvlUpdateResult r = Update(x, p, Sample({2.0, 0.0, 0.0}, RDiag(1.0)));
  ExpectUnchanged(r, Status::kNonFinite, /*evaluated=*/true);
  EXPECT_EQ(r.diagnostics->dof, 3);
  EXPECT_EQ(x.ToArray(), EskfNominal().ToArray());
}

TEST(DvlBottomTrackUpdateTest, GateOverflowIsNonFinite) {
  ExpectUnchanged(Update(EskfNominal(), DiagP(0.04, {{{6, 6}, 1.5e308}}),
                         Sample({0, 0, 0}, RDiag(1.5e308))),
                  Status::kNonFinite);
}

TEST(DvlBottomTrackUpdateTest, GoldenCase) {
  const DvlUpdateResult r =
      Update(GoldenX(), GoldenP(), Sample(kGoldenZ, kGoldenR));
  ASSERT_EQ(r.status, Status::kOkAccept);
  EXPECT_NEAR(r.diagnostics->mahalanobis_sq, kGoldenD2, kTol);
  const auto got = r.nominal->ToArray();
  for (int i = 0; i < kNominalDim; ++i) {
    EXPECT_NEAR(got[i], kGoldenNominal[i], kTol) << i;
  }
  const auto diag = Diag(*r.P);
  for (int i = 0; i < kN; ++i) EXPECT_NEAR(diag[i], kGoldenPDiag[i], kTol) << i;
}

TEST(DvlBottomTrackUpdateTest, InputsAreNotModified) {
  const EskfNominal x = GoldenX();
  const EskfCovariance p = GoldenP();
  const DvlBottomTrackSample z = Sample(kGoldenZ, kGoldenR);
  const auto x_before = x.ToArray();
  const auto p_before = p.row_major();
  const auto z_v = z.velocity_body_m_s;
  const auto z_r = z.R_body;
  Update(x, p, z);
  EXPECT_EQ(x.ToArray(), x_before);
  EXPECT_EQ(p.row_major(), p_before);
  EXPECT_EQ(z.velocity_body_m_s, z_v);
  EXPECT_EQ(z.R_body, z_r);
}

TEST(DvlBottomTrackUpdateTest, Determinism) {
  const DvlBottomTrackSample z = Sample(kGoldenZ, kGoldenR);
  const DvlUpdateResult a = Update(GoldenX(), GoldenP(), z);
  const DvlUpdateResult b = Update(GoldenX(), GoldenP(), z);
  EXPECT_EQ(a.nominal->ToArray(), b.nominal->ToArray());
  EXPECT_EQ(a.P->row_major(), b.P->row_major());
  EXPECT_EQ(a.diagnostics->mahalanobis_sq, b.diagnostics->mahalanobis_sq);
  EXPECT_EQ(a.diagnostics->S, b.diagnostics->S);
}

}  // namespace
}  // namespace intrinsic::estimation
