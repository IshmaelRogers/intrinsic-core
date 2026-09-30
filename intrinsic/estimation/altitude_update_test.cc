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

// Mirrors altitude_update_test.py.

#include "intrinsic/estimation/altitude_update.h"

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
constexpr int kDpZ = 2;
constexpr double kTol = 1e-9;
constexpr double kChi2 = 3.841;
const double kNan = std::numeric_limits<double>::quiet_NaN();
const double kInf = std::numeric_limits<double>::infinity();
using Status = AltitudeUpdateStatus;
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

AltitudeSample Sample(double altitude = 10.0, double r = 0.01,
                      bool valid = true) {
  AltitudeSample z;
  z.altitude_m = altitude;
  z.R = r;
  z.valid = valid;
  return z;
}

SeafloorContext Floor(double up = -20.0, bool present = true) {
  SeafloorContext s;
  s.present = present;
  s.seafloor_up_m = up;
  return s;
}

// Flat seafloor at -20 m, vehicle 10 m above it.
EskfNominal HoverX(double z = -10.0) {
  EskfNominal x;
  x.p_enu = {0.0, 0.0, z};
  return x;
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

// Seafloor at -5 gives h = 2, altitude 2.1 gives nu = 0.1.
AltitudeSample GoldenZ() { return Sample(2.1, 0.02); }
SeafloorContext GoldenFloor() { return Floor(-5.0); }

// Independent numpy evaluation: K = sym(P) H^T S^-1, Joseph, right-error
// inject. Same constants as the Python test.
constexpr double kGoldenD2 = 0.11111111111111112;
constexpr double kGoldenNu = 0.1;
constexpr std::array<double, kNominalDim> kGoldenNominal = {
    1.001111111111111,    2.0011111111111113,     -2.922222222222222,
    0.923266091844056,    0.10282578279334963,    -0.20456858893865398,
    0.3084773483800488,   0.5011111111111111,     -0.10444444444444445,
    0.20111111111111113,  0.011111111111111112,   -0.01888888888888889,
    0.03111111111111111,  -0.0034444444444444444, 0.0031111111111111114,
    -0.001888888888888889};
constexpr std::array<double, kN> kGoldenPDiag = {
    0.04998888888888889, 0.05998888888888889, 0.015555555555555559,
    0.07998888888888889, 0.08998888888888888, 0.09998888888888889,
    0.10998888888888889, 0.11982222222222223, 0.1299888888888889,
    0.1399888888888889,  0.14998888888888892, 0.1599888888888889,
    0.1698222222222222,  0.1799888888888889,  0.1899888888888889};

void ExpectUnchanged(const AltitudeUpdateResult &r, Status status,
                     bool evaluated = false) {
  EXPECT_EQ(r.status, status);
  EXPECT_EQ(r.evaluated, evaluated);
  EXPECT_FALSE(r.accepted);
  EXPECT_FALSE(r.nominal.has_value());
  EXPECT_FALSE(r.P.has_value());
  EXPECT_FALSE(r.delta_x.has_value());
  EXPECT_EQ(r.diagnostics.has_value(), evaluated);
}

AltitudeUpdateResult Update(const EskfNominal &x, const EskfCovariance &p,
                            const AltitudeSample &z,
                            const SeafloorContext &floor = Floor(),
                            double t = kChi2) {
  return UpdateAltitude(x, p, z, floor, t);
}

TEST(AltitudeUpdateTest, FlatSeafloorAccept) {
  // seafloor -20, p_z = -10 -> h = 10. S = 0.04 + 0.01 = 0.05,
  // K[dp_z] = +0.8, dp_z variance 0.04 -> 0.008.
  const EskfNominal x = HoverX();
  const AltitudeUpdateResult r = Update(x, DiagP(0.04), Sample());
  ASSERT_EQ(r.status, Status::kOkAccept);
  EXPECT_TRUE(r.evaluated);
  EXPECT_TRUE(r.accepted);
  EXPECT_EQ(r.nominal->ToArray(), x.ToArray());
  EXPECT_EQ(r.delta_x->ToArray(), EskfError().ToArray());
  const auto diag = Diag(*r.P);
  for (int i = 0; i < kN; ++i) {
    EXPECT_NEAR(diag[i], i == kDpZ ? 0.008 : 0.04, kTol) << i;
    for (int j = 0; j < kN; ++j) {
      if (i != j) {
        EXPECT_EQ(r.P->At(i, j), 0.0);
      }
    }
  }
  EXPECT_EQ(r.diagnostics->mahalanobis_sq, 0.0);
  EXPECT_EQ(r.diagnostics->innovation_norm, 0.0);
  EXPECT_EQ(r.diagnostics->threshold, kChi2);
  EXPECT_EQ(r.diagnostics->dof, 1);
  EXPECT_NEAR(r.diagnostics->S[0], 0.05, kTol);
}

TEST(AltitudeUpdateTest, OnlyDpZVarianceShrinks) {
  const AltitudeUpdateResult r = Update(HoverX(), DiagP(0.04), Sample());
  EXPECT_LT(r.P->At(kDpZ, kDpZ), 0.04);
  for (int i = 0; i < kN; ++i) {
    if (i != kDpZ) {
      EXPECT_EQ(r.P->At(i, i), 0.04);
    }
  }
}

TEST(AltitudeUpdateTest, HigherReadingMovesPUp) {
  // altitude = 10.1, nu = 0.1, d^2 = 0.2, dp_z = +0.8 * 0.1 = +0.08.
  const AltitudeUpdateResult r = Update(HoverX(), DiagP(0.04), Sample(10.1));
  ASSERT_EQ(r.status, Status::kOkAccept);
  EXPECT_NEAR(r.diagnostics->mahalanobis_sq, 0.2, kTol);
  EXPECT_NEAR(r.diagnostics->innovation_norm, 0.1, kTol);
  EXPECT_NEAR(r.nominal->p_enu[2], -9.92, kTol);
  EXPECT_NEAR(r.delta_x->dp[2], 0.08, kTol);
  EXPECT_EQ(r.nominal->p_enu[0], 0.0);
  EXPECT_EQ(r.nominal->p_enu[1], 0.0);
  EXPECT_EQ(r.nominal->q_wxyz, (std::array<double, 4>{1.0, 0.0, 0.0, 0.0}));
}

TEST(AltitudeUpdateTest, LowerReadingMovesPDown) {
  const AltitudeUpdateResult r = Update(HoverX(), DiagP(0.04), Sample(9.9));
  EXPECT_NEAR(r.nominal->p_enu[2], -10.08, kTol);
}

TEST(AltitudeUpdateTest, SeafloorUpShiftsPrediction) {
  // Seafloor at -18 and p_z = -8 give the same 10 m clearance as the hover.
  const EskfNominal x = HoverX(-8.0);
  AltitudeUpdateResult r = Update(x, DiagP(0.04), Sample(10.0), Floor(-18.0));
  ASSERT_EQ(r.status, Status::kOkAccept);
  EXPECT_EQ(r.diagnostics->innovation_norm, 0.0);
  EXPECT_EQ(r.nominal->ToArray(), x.ToArray());
  r = Update(x, DiagP(0.04), Sample(10.1), Floor(-18.0));
  EXPECT_NEAR(r.nominal->p_enu[2], -7.92, kTol);
}

TEST(AltitudeUpdateTest, ClearanceIsPositiveUp) {
  // Vehicle below a raised seafloor has negative clearance (finite, accepted).
  const EskfNominal x = HoverX(-10.0);
  const AltitudeUpdateResult r =
      Update(x, DiagP(0.04), Sample(-5.0), Floor(-5.0));
  ASSERT_EQ(r.status, Status::kOkAccept);
  EXPECT_EQ(r.diagnostics->innovation_norm, 0.0);
}

TEST(AltitudeUpdateTest, CorrelatedStateIsInjected) {
  // K[i] = +P[i, dp_z] / 0.05 and nu = 0.1.
  const EskfCovariance p = DiagP(0.04, {{{0, 2}, 0.02},
                                        {{2, 0}, 0.02},
                                        {{4, 2}, 0.01},
                                        {{2, 4}, 0.01},
                                        {{9, 2}, 0.005},
                                        {{2, 9}, 0.005},
                                        {{13, 2}, 0.0025},
                                        {{2, 13}, 0.0025}});
  const AltitudeUpdateResult r = Update(HoverX(), p, Sample(10.1));
  ASSERT_EQ(r.status, Status::kOkAccept);
  const EskfNominal &n = *r.nominal;
  EXPECT_NEAR(n.p_enu[0], 0.04, kTol);
  EXPECT_NEAR(n.b_a[0], 0.01, kTol);
  EXPECT_NEAR(n.b_g[1], 0.005, kTol);
  // dtheta_y = +0.02 -> q = normalize(1, 0, 0.01, 0).
  const double norm = std::sqrt(1.0 + 0.01 * 0.01);
  EXPECT_NEAR(n.q_wxyz[0], 1.0 / norm, kTol);
  EXPECT_NEAR(n.q_wxyz[2], 0.01 / norm, kTol);
  EXPECT_NEAR(n.q_wxyz[1], 0.0, kTol);
  EXPECT_NEAR(n.q_wxyz[3], 0.0, kTol);
  double q_sq = 0.0;
  for (double c : n.q_wxyz)
    q_sq += c * c;
  EXPECT_NEAR(std::sqrt(q_sq), 1.0, kTol);
  EXPECT_NEAR(r.delta_x->dtheta[1], 0.02, kTol);
}

TEST(AltitudeUpdateTest, CovarianceIsSymmetricAndDoesNotGrow) {
  const AltitudeUpdateResult r =
      Update(GoldenX(), GoldenP(), GoldenZ(), GoldenFloor());
  ASSERT_EQ(r.status, Status::kOkAccept);
  const EskfCovariance prior = GoldenP();
  for (int i = 0; i < kN; ++i) {
    for (int j = 0; j < kN; ++j)
      EXPECT_EQ(r.P->At(i, j), r.P->At(j, i));
    EXPECT_LE(r.P->At(i, i), prior.At(i, i) + 1e-15);
  }
}

TEST(AltitudeUpdateTest, AsymmetricPMatchesItsSymmetrization) {
  const EskfCovariance raw = GoldenP();
  std::vector<double> sym(kN * kN);
  for (int i = 0; i < kN; ++i) {
    for (int j = 0; j < kN; ++j) {
      sym[i * kN + j] = 0.5 * (raw.At(i, j) + raw.At(j, i));
    }
  }
  const AltitudeUpdateResult a =
      Update(GoldenX(), raw, GoldenZ(), GoldenFloor());
  const AltitudeUpdateResult b = Update(
      GoldenX(), *EskfCovariance::FromRowMajor(sym), GoldenZ(), GoldenFloor());
  EXPECT_EQ(a.nominal->ToArray(), b.nominal->ToArray());
  EXPECT_EQ(a.P->row_major(), b.P->row_major());
}

TEST(AltitudeUpdateTest, OutlierRejectLeavesEverythingUnchanged) {
  const EskfNominal x = HoverX();
  const EskfCovariance p = DiagP(0.04);
  const auto x_before = x.ToArray();
  const auto p_before = p.row_major();
  const AltitudeUpdateResult r = Update(x, p, Sample(11.0));
  ExpectUnchanged(r, Status::kOkReject, /*evaluated=*/true);
  EXPECT_NEAR(r.diagnostics->mahalanobis_sq, 20.0, kTol);
  EXPECT_EQ(r.diagnostics->dof, 1);
  EXPECT_EQ(x.ToArray(), x_before);
  EXPECT_EQ(p.row_major(), p_before);
}

TEST(AltitudeUpdateTest, ThresholdBoundary) {
  const AltitudeSample z = Sample(10.1);
  EXPECT_EQ(Update(HoverX(), DiagP(0.04), z, Floor(), 0.2 + 1e-6).status,
            Status::kOkAccept);
  EXPECT_EQ(Update(HoverX(), DiagP(0.04), z, Floor(), 0.2 - 1e-6).status,
            Status::kOkReject);
}

TEST(AltitudeUpdateTest, MissingSeafloorLeavesEverythingUnchanged) {
  const EskfNominal x = GoldenX();
  const EskfCovariance p = GoldenP();
  const auto x_before = x.ToArray();
  const auto p_before = p.row_major();
  for (const SeafloorContext &s :
       {Floor(-5.0, /*present=*/false), SeafloorContext(), Floor(kNan),
        Floor(kInf), Floor(-kInf)}) {
    ExpectUnchanged(Update(x, p, GoldenZ(), s),
                    Status::kSkippedMissingSeafloor);
  }
  EXPECT_EQ(x.ToArray(), x_before);
  EXPECT_EQ(p.row_major(), p_before);
}

TEST(AltitudeUpdateTest, InvalidSampleWinsOverMissingSeafloor) {
  ExpectUnchanged(Update(HoverX(), DiagP(0.04), Sample(10.0, 0.01, false),
                         Floor(-20.0, false)),
                  Status::kSkippedInvalid);
}

TEST(AltitudeUpdateTest, InvalidFlag) {
  ExpectUnchanged(Update(HoverX(), DiagP(0.04), Sample(10.0, 0.01, false)),
                  Status::kSkippedInvalid);
}

TEST(AltitudeUpdateTest, InvalidFlagWinsOverBadContents) {
  ExpectUnchanged(Update(HoverX(), DiagP(0.04), Sample(kNan, kNan, false)),
                  Status::kSkippedInvalid);
}

TEST(AltitudeUpdateTest, NonFiniteAltitudeOrR) {
  const std::vector<AltitudeSample> bad = {Sample(kNan), Sample(kInf),
                                           Sample(-kInf), Sample(10.0, kNan),
                                           Sample(10.0, kInf)};
  for (const AltitudeSample &z : bad) {
    ExpectUnchanged(Update(HoverX(), DiagP(0.04), z), Status::kSkippedInvalid);
  }
}

TEST(AltitudeUpdateTest, NonPositiveRIsSingular) {
  for (double r : {0.0, -0.01, -1e-300, 1e-12, 1e-13}) {
    ExpectUnchanged(Update(HoverX(), DiagP(0.04), Sample(10.0, r)),
                    Status::kSingular);
  }
}

TEST(AltitudeUpdateTest, RJustAboveFloorEvaluates) {
  EXPECT_EQ(Update(HoverX(), DiagP(0.04), Sample(10.0, 2e-12)).status,
            Status::kOkAccept);
}

TEST(AltitudeUpdateTest, InvalidThreshold) {
  for (double t : {0.0, -1.0, kNan, kInf}) {
    ExpectUnchanged(Update(HoverX(), DiagP(0.04), Sample(), Floor(), t),
                    Status::kSkippedInvalid);
  }
}

TEST(AltitudeUpdateTest, InvalidP) {
  ExpectUnchanged(Update(HoverX(), EskfCovariance(), Sample()),
                  Status::kSkippedInvalid);
  ExpectUnchanged(Update(HoverX(), DiagP(0.04, {{{3, 4}, kNan}}), Sample()),
                  Status::kSkippedInvalid);
}

TEST(AltitudeUpdateTest, InvalidNominalAndQuaternion) {
  std::vector<EskfNominal> bad(8);
  bad[0].p_enu = {kNan, 0.0, 0.0};
  bad[1].p_enu = {0.0, 0.0, kInf};
  bad[2].v_body = {0.0, kInf, 0.0};
  bad[3].b_a = {0.0, 0.0, kNan};
  bad[4].b_g = {kInf, 0.0, 0.0};
  bad[5].q_wxyz = {1.0, kNan, 0.0, 0.0};
  bad[6].q_wxyz = {0.0, 0.0, 0.0, 0.0};
  bad[7].q_wxyz = {1e-13, 0.0, 0.0, 0.0};
  for (const EskfNominal &x : bad) {
    ExpectUnchanged(Update(x, DiagP(0.04), Sample()), Status::kSkippedInvalid);
  }
}

TEST(AltitudeUpdateTest, NonFiniteUpdateLeavesInputsUnchanged) {
  // d^2 = 4 / 2 = 2 passes the gate, but K[0] = 1e200 / 2 and the Joseph
  // product K[0] * P[2, 0] overflows.
  const EskfCovariance p = DiagP(1.0, {{{0, 2}, 1e200}, {{2, 0}, 1e200}});
  const EskfNominal x = HoverX();
  const AltitudeUpdateResult r = Update(x, p, Sample(12.0, 1.0));
  ExpectUnchanged(r, Status::kNonFinite, /*evaluated=*/true);
  EXPECT_EQ(r.diagnostics->dof, 1);
  EXPECT_EQ(x.ToArray(), HoverX().ToArray());
}

TEST(AltitudeUpdateTest, GateOverflowIsNonFinite) {
  ExpectUnchanged(
      Update(HoverX(), DiagP(0.04, {{{2, 2}, 1.5e308}}), Sample(10.0, 1.5e308)),
      Status::kNonFinite);
}

TEST(AltitudeUpdateTest, PredictionOverflowIsNonFinite) {
  ExpectUnchanged(
      Update(HoverX(1.7e308), DiagP(0.04), Sample(10.0), Floor(-1.7e308)),
      Status::kNonFinite);
}

TEST(AltitudeUpdateTest, GoldenCase) {
  const AltitudeUpdateResult r =
      Update(GoldenX(), GoldenP(), GoldenZ(), GoldenFloor());
  ASSERT_EQ(r.status, Status::kOkAccept);
  EXPECT_NEAR(r.diagnostics->mahalanobis_sq, kGoldenD2, kTol);
  EXPECT_NEAR(r.diagnostics->innovation_norm, kGoldenNu, kTol);
  const auto got = r.nominal->ToArray();
  for (int i = 0; i < kNominalDim; ++i) {
    EXPECT_NEAR(got[i], kGoldenNominal[i], kTol) << i;
  }
  const auto diag = Diag(*r.P);
  for (int i = 0; i < kN; ++i)
    EXPECT_NEAR(diag[i], kGoldenPDiag[i], kTol) << i;
}

TEST(AltitudeUpdateTest, InputsAreNotModified) {
  const EskfNominal x = GoldenX();
  const EskfCovariance p = GoldenP();
  const AltitudeSample z = GoldenZ();
  const SeafloorContext s = GoldenFloor();
  const auto x_before = x.ToArray();
  const auto p_before = p.row_major();
  Update(x, p, z, s);
  EXPECT_EQ(x.ToArray(), x_before);
  EXPECT_EQ(p.row_major(), p_before);
  EXPECT_EQ(z.altitude_m, 2.1);
  EXPECT_EQ(z.R, 0.02);
  EXPECT_TRUE(s.present);
  EXPECT_EQ(s.seafloor_up_m, -5.0);
}

TEST(AltitudeUpdateTest, Determinism) {
  const AltitudeUpdateResult a =
      Update(GoldenX(), GoldenP(), GoldenZ(), GoldenFloor());
  const AltitudeUpdateResult b =
      Update(GoldenX(), GoldenP(), GoldenZ(), GoldenFloor());
  EXPECT_EQ(a.nominal->ToArray(), b.nominal->ToArray());
  EXPECT_EQ(a.P->row_major(), b.P->row_major());
  EXPECT_EQ(a.delta_x->ToArray(), b.delta_x->ToArray());
  EXPECT_EQ(a.diagnostics->mahalanobis_sq, b.diagnostics->mahalanobis_sq);
  EXPECT_EQ(a.diagnostics->S, b.diagnostics->S);
}

} // namespace
} // namespace intrinsic::estimation
