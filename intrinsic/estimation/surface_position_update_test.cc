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

// Mirrors surface_position_update_test.py.

#include "intrinsic/estimation/surface_position_update.h"

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
constexpr double kChi2 = 5.991;
const double kNan = std::numeric_limits<double>::quiet_NaN();
const double kInf = std::numeric_limits<double>::infinity();
using Status = SurfacePositionUpdateStatus;
using Overrides = std::map<std::pair<int, int>, double>;

// P = value * I, with optional {(row, col): value} overrides.
EskfCovariance DiagP(double value, const Overrides& overrides = {}) {
  std::vector<double> v(kN * kN, 0.0);
  for (int i = 0; i < kN; ++i) v[i * kN + i] = value;
  for (const auto& [ij, o] : overrides) v[ij.first * kN + ij.second] = o;
  return *EskfCovariance::FromRowMajor(v);
}

std::array<double, 4> DiagR(double r) { return {r, 0.0, 0.0, r}; }

SurfacePositionSample Sample(double e = 3.0, double n = 4.0, double r = 0.01,
                             bool valid = true) {
  SurfacePositionSample z;
  z.position_en_m = {e, n};
  z.R_en = DiagR(r);
  z.valid = valid;
  return z;
}

SurfacePositionSample SampleR(const std::array<double, 4>& r, double e = 3.0,
                              double n = 4.0) {
  SurfacePositionSample z = Sample(e, n);
  z.R_en = r;
  return z;
}

SurfaceFixPolicy Policy(bool is_surfaced = true, bool quality_ok = true) {
  SurfaceFixPolicy p;
  p.is_surfaced = is_surfaced;
  p.quality_ok = quality_ok;
  return p;
}

// Vehicle at rest at East 3, North 4, Up -10.
EskfNominal HoverX(double e = 3.0, double n = 4.0) {
  EskfNominal x;
  x.p_enu = {e, n, -10.0};
  return x;
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

// nu = (0.1, -0.05), correlated R.
SurfacePositionSample GoldenZ() {
  return SampleR({0.02, 0.004, 0.004, 0.03}, 1.1, 1.95);
}

// Independent numpy evaluation: K = sym(P) H^T S^-1, Joseph, right-error
// inject. Same constants as the Python test.
constexpr double kGoldenD2 = 0.17928286852589673;
constexpr double kGoldenNu = 0.11180339887498958;
constexpr std::array<double, 4> kGoldenS = {0.07, 0.005, 0.005, 0.09};
constexpr std::array<double, kNominalDim> kGoldenNominal = {
    1.0730677290836654,   1.9632270916334662,   -2.9991633466135457,
    0.9244238819514294,   0.10352514147585345,  -0.20434433105621927,
    0.3049039618067309,   0.5040239043824701,   -0.09916334661354582,
    0.2008366533864542,   0.010836653386454185, -0.026533864541832677,
    0.03402390438247012,  0.001836653386454184, 0.002836653386454184,
    -0.002163346613545816};
constexpr std::array<double, kN> kGoldenPDiag = {
    0.01421195219123506, 0.019921912350597606, 0.06997609561752989,
    0.07997609561752989, 0.08997609561752988,  0.09975298804780877,
    0.109800796812749,   0.1199760956175299,   0.12997609561752987,
    0.13997609561752988, 0.1497529880478088,   0.159800796812749,
    0.16997609561752985, 0.17997609561752986,  0.18997609561752987};

void ExpectUnchanged(const SurfacePositionUpdateResult& r, Status status,
                     bool evaluated = false) {
  EXPECT_EQ(r.status, status);
  EXPECT_EQ(r.evaluated, evaluated);
  EXPECT_FALSE(r.accepted);
  EXPECT_FALSE(r.nominal.has_value());
  EXPECT_FALSE(r.P.has_value());
  EXPECT_FALSE(r.delta_x.has_value());
  EXPECT_EQ(r.diagnostics.has_value(), evaluated);
}

SurfacePositionUpdateResult Update(const EskfNominal& x,
                                   const EskfCovariance& p,
                                   const SurfacePositionSample& z,
                                   const SurfaceFixPolicy& policy = Policy(),
                                   double t = kChi2) {
  return UpdateSurfacePosition(x, p, z, policy, t);
}

TEST(SurfacePositionUpdateTest, SurfacedAccept) {
  // Identity hover at (3, 4) with a matching fix. S = 0.05 I2, K = 0.8 on
  // dp_e / dp_n, variance 0.04 -> 0.008.
  const EskfNominal x = HoverX();
  const SurfacePositionUpdateResult r = Update(x, DiagP(0.04), Sample());
  ASSERT_EQ(r.status, Status::kOkAccept);
  EXPECT_TRUE(r.evaluated);
  EXPECT_TRUE(r.accepted);
  EXPECT_EQ(r.nominal->ToArray(), x.ToArray());
  EXPECT_EQ(r.delta_x->ToArray(), EskfError().ToArray());
  const auto diag = Diag(*r.P);
  for (int i = 0; i < kN; ++i) {
    EXPECT_NEAR(diag[i], i < 2 ? 0.008 : 0.04, kTol) << i;
    for (int j = 0; j < kN; ++j) {
      if (i != j) {
        EXPECT_EQ(r.P->At(i, j), 0.0);
      }
    }
  }
  EXPECT_EQ(r.diagnostics->mahalanobis_sq, 0.0);
  EXPECT_EQ(r.diagnostics->innovation_norm, 0.0);
  EXPECT_EQ(r.diagnostics->threshold, kChi2);
  EXPECT_EQ(r.diagnostics->dof, 2);
  ASSERT_EQ(r.diagnostics->S.size(), 4u);
  EXPECT_NEAR(r.diagnostics->S[0], 0.05, kTol);
  EXPECT_NEAR(r.diagnostics->S[1], 0.0, kTol);
  EXPECT_NEAR(r.diagnostics->S[2], 0.0, kTol);
  EXPECT_NEAR(r.diagnostics->S[3], 0.05, kTol);
}

TEST(SurfacePositionUpdateTest, OnlyHorizontalPositionVarianceShrinks) {
  const SurfacePositionUpdateResult r = Update(HoverX(), DiagP(0.04), Sample());
  EXPECT_LT(r.P->At(0, 0), 0.04);
  EXPECT_LT(r.P->At(1, 1), 0.04);
  for (int i = 2; i < kN; ++i) EXPECT_EQ(r.P->At(i, i), 0.04);
}

TEST(SurfacePositionUpdateTest, OffsetFixMovesHorizontalPosition) {
  // nu = (0.1, -0.05), d^2 = (0.01 + 0.0025) / 0.05 = 0.25,
  // dp = 0.8 * nu = (0.08, -0.04). Up is untouched.
  const SurfacePositionUpdateResult r =
      Update(HoverX(), DiagP(0.04), Sample(3.1, 3.95));
  ASSERT_EQ(r.status, Status::kOkAccept);
  EXPECT_NEAR(r.diagnostics->mahalanobis_sq, 0.25, kTol);
  EXPECT_NEAR(r.diagnostics->innovation_norm, std::sqrt(0.0125), kTol);
  EXPECT_NEAR(r.nominal->p_enu[0], 3.08, kTol);
  EXPECT_NEAR(r.nominal->p_enu[1], 3.96, kTol);
  EXPECT_EQ(r.nominal->p_enu[2], -10.0);
  EXPECT_NEAR(r.delta_x->dp[0], 0.08, kTol);
  EXPECT_NEAR(r.delta_x->dp[1], -0.04, kTol);
  EXPECT_EQ(r.delta_x->dp[2], 0.0);
  EXPECT_EQ(r.nominal->q_wxyz, (std::array<double, 4>{1.0, 0.0, 0.0, 0.0}));
}

TEST(SurfacePositionUpdateTest, EastAndNorthAreNotSwapped) {
  const SurfacePositionUpdateResult r =
      Update(HoverX(), DiagP(0.04), Sample(3.1, 4.0));
  ASSERT_EQ(r.status, Status::kOkAccept);
  EXPECT_NEAR(r.nominal->p_enu[0], 3.08, kTol);
  EXPECT_EQ(r.nominal->p_enu[1], 4.0);
}

TEST(SurfacePositionUpdateTest, CorrelatedStateIsInjected) {
  // nu = (0.1, 0), S = 0.05 I2, K[i, e] = P[i, e] / 0.05.
  const EskfCovariance p = DiagP(0.04, {{{0, 3}, 0.02},
                                        {{3, 0}, 0.02},
                                        {{9, 0}, 0.005},
                                        {{0, 9}, 0.005},
                                        {{13, 1}, 0.0025},
                                        {{1, 13}, 0.0025}});
  const SurfacePositionUpdateResult r = Update(HoverX(), p, Sample(3.1, 4.0));
  ASSERT_EQ(r.status, Status::kOkAccept);
  const EskfNominal& n = *r.nominal;
  EXPECT_NEAR(n.p_enu[0], 3.08, kTol);
  EXPECT_NEAR(n.b_a[0], 0.01, kTol);
  EXPECT_NEAR(n.b_g[0], 0.0, kTol);
  // dtheta_x = +0.04 -> q = normalize(1, 0.02, 0, 0).
  const double norm = std::sqrt(1.0 + 0.02 * 0.02);
  EXPECT_NEAR(n.q_wxyz[0], 1.0 / norm, kTol);
  EXPECT_NEAR(n.q_wxyz[1], 0.02 / norm, kTol);
  EXPECT_NEAR(n.q_wxyz[2], 0.0, kTol);
  EXPECT_NEAR(n.q_wxyz[3], 0.0, kTol);
  double q_sq = 0.0;
  for (double c : n.q_wxyz) q_sq += c * c;
  EXPECT_NEAR(std::sqrt(q_sq), 1.0, kTol);
  EXPECT_NEAR(r.delta_x->dtheta[0], 0.04, kTol);
}

TEST(SurfacePositionUpdateTest, CovarianceIsSymmetricAndDoesNotGrow) {
  const SurfacePositionUpdateResult r = Update(GoldenX(), GoldenP(), GoldenZ());
  ASSERT_EQ(r.status, Status::kOkAccept);
  const EskfCovariance prior = GoldenP();
  for (int i = 0; i < kN; ++i) {
    for (int j = 0; j < kN; ++j) EXPECT_EQ(r.P->At(i, j), r.P->At(j, i));
    EXPECT_LE(r.P->At(i, i), prior.At(i, i) + 1e-15);
  }
}

TEST(SurfacePositionUpdateTest, AsymmetricPMatchesItsSymmetrization) {
  const EskfCovariance raw = GoldenP();
  std::vector<double> sym(kN * kN);
  for (int i = 0; i < kN; ++i) {
    for (int j = 0; j < kN; ++j) {
      sym[i * kN + j] = 0.5 * (raw.At(i, j) + raw.At(j, i));
    }
  }
  const SurfacePositionUpdateResult a = Update(GoldenX(), raw, GoldenZ());
  const SurfacePositionUpdateResult b =
      Update(GoldenX(), *EskfCovariance::FromRowMajor(sym), GoldenZ());
  EXPECT_EQ(a.nominal->ToArray(), b.nominal->ToArray());
  EXPECT_EQ(a.P->row_major(), b.P->row_major());
}

TEST(SurfacePositionUpdateTest, OutlierRejectLeavesEverythingUnchanged) {
  const EskfNominal x = HoverX();
  const EskfCovariance p = DiagP(0.04);
  const auto x_before = x.ToArray();
  const auto p_before = p.row_major();
  // nu = (1, 0), d^2 = 1 / 0.05 = 20.
  const SurfacePositionUpdateResult r = Update(x, p, Sample(4.0, 4.0));
  ExpectUnchanged(r, Status::kOkReject, /*evaluated=*/true);
  EXPECT_NEAR(r.diagnostics->mahalanobis_sq, 20.0, kTol);
  EXPECT_EQ(r.diagnostics->dof, 2);
  EXPECT_EQ(x.ToArray(), x_before);
  EXPECT_EQ(p.row_major(), p_before);
}

TEST(SurfacePositionUpdateTest, ThresholdBoundary) {
  // nu = (0.1, 0), d^2 = 0.2.
  const SurfacePositionSample z = Sample(3.1, 4.0);
  EXPECT_EQ(Update(HoverX(), DiagP(0.04), z, Policy(), 0.2 + 1e-6).status,
            Status::kOkAccept);
  EXPECT_EQ(Update(HoverX(), DiagP(0.04), z, Policy(), 0.2 - 1e-6).status,
            Status::kOkReject);
}

TEST(SurfacePositionUpdateTest, SubmergedPolicySkipsBitIdentical) {
  const EskfNominal x = GoldenX();
  const EskfCovariance p = GoldenP();
  const auto x_before = x.ToArray();
  const auto p_before = p.row_major();
  ExpectUnchanged(Update(x, p, GoldenZ(), Policy(/*is_surfaced=*/false)),
                  Status::kSkippedPolicy);
  EXPECT_EQ(x.ToArray(), x_before);
  EXPECT_EQ(p.row_major(), p_before);
}

TEST(SurfacePositionUpdateTest, LowQualityPolicySkipsBitIdentical) {
  const EskfNominal x = GoldenX();
  const EskfCovariance p = GoldenP();
  const auto x_before = x.ToArray();
  const auto p_before = p.row_major();
  ExpectUnchanged(Update(x, p, GoldenZ(),
                         Policy(/*is_surfaced=*/true, /*quality_ok=*/false)),
                  Status::kSkippedPolicy);
  EXPECT_EQ(x.ToArray(), x_before);
  EXPECT_EQ(p.row_major(), p_before);
}

TEST(SurfacePositionUpdateTest, DefaultPolicyAndBothFlagsOffSkip) {
  ExpectUnchanged(Update(HoverX(), DiagP(0.04), Sample(), SurfaceFixPolicy()),
                  Status::kSkippedPolicy);
  ExpectUnchanged(Update(HoverX(), DiagP(0.04), Sample(), Policy(false, false)),
                  Status::kSkippedPolicy);
}

TEST(SurfacePositionUpdateTest, PolicyWinsOverEverythingElse) {
  // Garbage sample, threshold, nominal, and P are not inspected when the
  // policy forbids the fix.
  EskfNominal bad_x;
  bad_x.p_enu = {kNan, 0.0, 0.0};
  SurfacePositionSample bad_z = Sample(kNan, kNan, kNan, false);
  bad_z.R_en = {kNan, kNan, kNan, kNan};
  ExpectUnchanged(
      Update(bad_x, EskfCovariance(), bad_z, Policy(false, true), kNan),
      Status::kSkippedPolicy);
}

TEST(SurfacePositionUpdateTest, InvalidFlag) {
  ExpectUnchanged(Update(HoverX(), DiagP(0.04), Sample(3.0, 4.0, 0.01, false)),
                  Status::kSkippedInvalid);
}

TEST(SurfacePositionUpdateTest, InvalidFlagWinsOverBadContents) {
  SurfacePositionSample z = Sample(kNan, kNan, kNan, false);
  z.R_en = {kNan, 0.0, 0.0, -1.0};
  ExpectUnchanged(Update(HoverX(), DiagP(0.04), z), Status::kSkippedInvalid);
}

TEST(SurfacePositionUpdateTest, NonFinitePositionOrR) {
  std::vector<SurfacePositionSample> bad = {
      Sample(kNan, 4.0),  Sample(3.0, kNan),      Sample(kInf, 4.0),
      Sample(3.0, -kInf), Sample(3.0, 4.0, kNan), Sample(3.0, 4.0, kInf)};
  bad.push_back(SampleR({0.01, kNan, 0.0, 0.01}));
  for (const SurfacePositionSample& z : bad) {
    ExpectUnchanged(Update(HoverX(), DiagP(0.04), z), Status::kSkippedInvalid);
  }
}

TEST(SurfacePositionUpdateTest, AsymmetricRBeyondToleranceIsInvalid) {
  ExpectUnchanged(
      Update(HoverX(), DiagP(0.04), SampleR({0.01, 0.002, 0.001, 0.01})),
      Status::kSkippedInvalid);
  ExpectUnchanged(Update(HoverX(), DiagP(0.04),
                         SampleR({0.01, 0.002, 0.002 + 2e-12, 0.01})),
                  Status::kSkippedInvalid);
}

TEST(SurfacePositionUpdateTest, AsymmetricRWithinToleranceEvaluates) {
  EXPECT_EQ(
      Update(HoverX(), DiagP(0.04), SampleR({0.01, 0.002, 0.002 + 1e-13, 0.01}))
          .status,
      Status::kOkAccept);
}

TEST(SurfacePositionUpdateTest, NonPositiveDefiniteRIsSingular) {
  const std::vector<std::array<double, 4>> bad = {
      {0.0, 0.0, 0.0, 0.0},    {-0.01, 0.0, 0.0, 0.01},
      {0.01, 0.0, 0.0, -0.01}, {1e-12, 0.0, 0.0, 0.01},
      {0.01, 0.0, 0.0, 1e-13}, {0.01, 0.02, 0.02, 0.01},
      {0.01, 0.01, 0.01, 0.01}};
  for (const auto& r : bad) {
    ExpectUnchanged(Update(HoverX(), DiagP(0.04), SampleR(r)),
                    Status::kSingular);
  }
}

TEST(SurfacePositionUpdateTest, RJustAboveFloorEvaluates) {
  EXPECT_EQ(Update(HoverX(), DiagP(0.04), Sample(3.0, 4.0, 2e-12)).status,
            Status::kOkAccept);
}

TEST(SurfacePositionUpdateTest, InvalidThreshold) {
  for (double t : {0.0, -1.0, kNan, kInf}) {
    ExpectUnchanged(Update(HoverX(), DiagP(0.04), Sample(), Policy(), t),
                    Status::kSkippedInvalid);
  }
}

TEST(SurfacePositionUpdateTest, InvalidP) {
  ExpectUnchanged(Update(HoverX(), EskfCovariance(), Sample()),
                  Status::kSkippedInvalid);
  ExpectUnchanged(Update(HoverX(), DiagP(0.04, {{{3, 4}, kNan}}), Sample()),
                  Status::kSkippedInvalid);
}

TEST(SurfacePositionUpdateTest, InvalidNominalAndQuaternion) {
  std::vector<EskfNominal> bad(8);
  bad[0].p_enu = {kNan, 0.0, 0.0};
  bad[1].p_enu = {0.0, 0.0, kInf};
  bad[2].v_body = {0.0, kInf, 0.0};
  bad[3].b_a = {0.0, 0.0, kNan};
  bad[4].b_g = {kInf, 0.0, 0.0};
  bad[5].q_wxyz = {1.0, kNan, 0.0, 0.0};
  bad[6].q_wxyz = {0.0, 0.0, 0.0, 0.0};
  bad[7].q_wxyz = {1e-13, 0.0, 0.0, 0.0};
  for (const EskfNominal& x : bad) {
    ExpectUnchanged(Update(x, DiagP(0.04), Sample()), Status::kSkippedInvalid);
  }
}

TEST(SurfacePositionUpdateTest, NonFiniteUpdateLeavesInputsUnchanged) {
  // nu = (2, 0), S = 2 I2, d^2 = 2 passes the gate, but K[dp_z, e] = 1e200 / 2
  // and the Joseph product K * P[e, dp_z] overflows.
  const EskfCovariance p = DiagP(1.0, {{{0, 2}, 1e200}, {{2, 0}, 1e200}});
  const EskfNominal x = HoverX();
  const SurfacePositionUpdateResult r = Update(x, p, Sample(5.0, 4.0, 1.0));
  ExpectUnchanged(r, Status::kNonFinite, /*evaluated=*/true);
  EXPECT_EQ(r.diagnostics->dof, 2);
  EXPECT_EQ(x.ToArray(), HoverX().ToArray());
}

TEST(SurfacePositionUpdateTest, GateOverflowIsNonFinite) {
  ExpectUnchanged(Update(HoverX(), DiagP(0.04, {{{0, 0}, 1.5e308}}),
                         Sample(3.0, 4.0, 1.5e308)),
                  Status::kNonFinite);
}

TEST(SurfacePositionUpdateTest, InnovationOverflowIsNonFinite) {
  ExpectUnchanged(
      Update(HoverX(1.7e308, 4.0), DiagP(0.04), Sample(-1.7e308, 4.0)),
      Status::kNonFinite);
}

TEST(SurfacePositionUpdateTest, GoldenCase) {
  const SurfacePositionUpdateResult r = Update(GoldenX(), GoldenP(), GoldenZ());
  ASSERT_EQ(r.status, Status::kOkAccept);
  EXPECT_NEAR(r.diagnostics->mahalanobis_sq, kGoldenD2, kTol);
  EXPECT_NEAR(r.diagnostics->innovation_norm, kGoldenNu, kTol);
  EXPECT_EQ(r.diagnostics->dof, 2);
  for (int i = 0; i < 4; ++i) {
    EXPECT_NEAR(r.diagnostics->S[i], kGoldenS[i], kTol) << i;
  }
  const auto got = r.nominal->ToArray();
  for (int i = 0; i < kNominalDim; ++i) {
    EXPECT_NEAR(got[i], kGoldenNominal[i], kTol) << i;
  }
  const auto diag = Diag(*r.P);
  for (int i = 0; i < kN; ++i) EXPECT_NEAR(diag[i], kGoldenPDiag[i], kTol) << i;
}

TEST(SurfacePositionUpdateTest, InputsAreNotModified) {
  const EskfNominal x = GoldenX();
  const EskfCovariance p = GoldenP();
  const SurfacePositionSample z = GoldenZ();
  const SurfaceFixPolicy policy = Policy();
  const auto x_before = x.ToArray();
  const auto p_before = p.row_major();
  Update(x, p, z, policy);
  EXPECT_EQ(x.ToArray(), x_before);
  EXPECT_EQ(p.row_major(), p_before);
  EXPECT_EQ(z.position_en_m, (std::array<double, 2>{1.1, 1.95}));
  EXPECT_EQ(z.R_en, (std::array<double, 4>{0.02, 0.004, 0.004, 0.03}));
  EXPECT_TRUE(policy.is_surfaced);
  EXPECT_TRUE(policy.quality_ok);
}

TEST(SurfacePositionUpdateTest, Determinism) {
  const SurfacePositionUpdateResult a = Update(GoldenX(), GoldenP(), GoldenZ());
  const SurfacePositionUpdateResult b = Update(GoldenX(), GoldenP(), GoldenZ());
  EXPECT_EQ(a.nominal->ToArray(), b.nominal->ToArray());
  EXPECT_EQ(a.P->row_major(), b.P->row_major());
  EXPECT_EQ(a.delta_x->ToArray(), b.delta_x->ToArray());
  EXPECT_EQ(a.diagnostics->mahalanobis_sq, b.diagnostics->mahalanobis_sq);
  EXPECT_EQ(a.diagnostics->S, b.diagnostics->S);
}

TEST(SurfacePositionUpdateTest, StatusSetHasNoModeTransition) {
  // The status enum is the whole public outcome surface. There is no
  // navigation-mode output, so the six statuses are all there is.
  const std::vector<Status> all = {
      Status::kOkAccept,       Status::kOkReject, Status::kSkippedPolicy,
      Status::kSkippedInvalid, Status::kSingular, Status::kNonFinite};
  for (size_t i = 0; i < all.size(); ++i) {
    EXPECT_EQ(static_cast<int>(all[i]), static_cast<int>(i));
  }
}

}  // namespace
}  // namespace intrinsic::estimation
