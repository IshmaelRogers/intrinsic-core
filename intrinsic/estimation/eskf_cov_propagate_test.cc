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

// Mirrors eskf_cov_propagate_test.py.

#include "intrinsic/estimation/eskf_cov_propagate.h"

#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <limits>
#include <vector>

#include "intrinsic/estimation/eskf_propagate.h"
#include "intrinsic/estimation/eskf_state.h"

namespace intrinsic::estimation {
namespace {

constexpr int kN = kCovDim;
constexpr double kTol = 1e-9;
constexpr double kG = 9.80665;
const double kNan = std::numeric_limits<double>::quiet_NaN();
const double kInf = std::numeric_limits<double>::infinity();

ImuSample HoverImu(const std::array<double, 4> &q) {
  const std::array<double, 9> r = RotationBodyToWorld(q);
  ImuSample imu;
  for (int i = 0; i < 3; ++i) {
    imu.accel_m_s2[i] = -(r[i] * kGravityEnu[0] + r[3 + i] * kGravityEnu[1] +
                          r[6 + i] * kGravityEnu[2]);
  }
  return imu;
}

EskfNominal GoldenNominal() {
  EskfNominal x;
  x.p_enu = {1.0, 2.0, 3.0};
  x.q_wxyz = {0.9, 0.1, -0.2, 0.3};
  x.v_body = {0.5, -0.25, 0.1};
  x.b_a = {0.01, -0.02, 0.03};
  x.b_g = {0.001, 0.002, -0.003};
  return x;
}

ImuSample GoldenImu() {
  ImuSample imu;
  imu.accel_m_s2 = {0.3, -0.4, 9.5};
  imu.gyro_rad_s = {0.05, -0.1, 0.2};
  return imu;
}

// Deliberately asymmetric, finite, deterministic.
EskfCovariance GoldenP() {
  std::vector<double> v;
  for (int i = 0; i < kN; ++i) {
    for (int j = 0; j < kN; ++j) {
      v.push_back(i == j ? 1.0 + 0.1 * i : 0.01 * ((i * 7 + j * 3) % 5) - 0.02);
    }
  }
  return *EskfCovariance::FromRowMajor(v);
}

ProcessNoiseConfig Noise(double a, double g, double ba, double bg) {
  ProcessNoiseConfig n;
  n.sigma_accel = a;
  n.sigma_gyro = g;
  n.sigma_accel_bias_rw = ba;
  n.sigma_gyro_bias_rw = bg;
  return n;
}

const ProcessNoiseConfig kGoldenNoise = Noise(0.1, 0.01, 0.001, 0.0001);
constexpr double kGoldenDt = 0.01;

struct GoldenEntry {
  int row;
  int col;
  double value;
};

// From the Python implementation; pinned identically in the Python test.
const GoldenEntry kGolden[] = {
    {0, 0, 1.000244208930194},      {1, 1, 1.1001273573407204},
    {2, 2, 1.2002688521501386},     {3, 3, 1.3001585455659999},
    {4, 4, 1.4001204463729997},     {5, 5, 1.5001274996579999},
    {6, 6, 1.6119586155527448},     {7, 7, 1.7127169341409083},
    {8, 8, 1.8024498947138858},     {9, 9, 1.9000000099999999},
    {10, 10, 2.00000001},           {11, 11, 2.10000001},
    {12, 12, 2.2000000001},         {13, 13, 2.3000000001},
    {14, 14, 2.4000000001000004},   {0, 1, 0.004770390235457063},
    {1, 5, 0.012895720342105264},   {2, 9, 0.005146236842105263},
    {3, 13, -0.020034749999999997}, {4, 2, -0.0006276264576315776},
    {5, 6, -0.004031543281503161},  {6, 10, 0.005545342026315788},
    {7, 14, 0.016715360026315792},  {8, 3, -0.012235875194768944},
    {9, 7, 0.004615360026315791},   {10, 11, 0.004999999999999999},
    {11, 0, 0.004820763157894736},  {12, 4, 0.004942300000000002},
    {13, 8, -0.03191070205263158},  {14, 12, 0.005},
};

EskfCovariance Diagonal(const std::array<double, kN> &d) {
  std::vector<double> v(kN * kN, 0.0);
  for (int i = 0; i < kN; ++i)
    v[i * kN + i] = d[i];
  return *EskfCovariance::FromRowMajor(v);
}

EskfCovariance Scaled(double s) {
  std::array<double, kN> d;
  d.fill(s);
  return Diagonal(d);
}

EskfMatrix MatMul(const EskfMatrix &a, const EskfMatrix &b) {
  EskfMatrix c{};
  for (int i = 0; i < kN; ++i) {
    for (int j = 0; j < kN; ++j) {
      double s = 0.0;
      for (int k = 0; k < kN; ++k)
        s += a[i * kN + k] * b[k * kN + j];
      c[i * kN + j] = s;
    }
  }
  return c;
}

EskfMatrix Transpose(const EskfMatrix &a) {
  EskfMatrix t{};
  for (int i = 0; i < kN; ++i) {
    for (int j = 0; j < kN; ++j)
      t[i * kN + j] = a[j * kN + i];
  }
  return t;
}

double SymErr(const EskfMatrix &m) {
  double e = 0.0;
  for (int i = 0; i < kN; ++i) {
    for (int j = 0; j < kN; ++j) {
      e = std::max(e, std::abs(m[i * kN + j] - m[j * kN + i]));
    }
  }
  return e;
}

// Smallest eigenvalue of a symmetric matrix by cyclic Jacobi.
double MinEigenvalue(const EskfMatrix &m) {
  double a[kN][kN];
  for (int i = 0; i < kN; ++i) {
    for (int j = 0; j < kN; ++j)
      a[i][j] = m[i * kN + j];
  }
  for (int sweep = 0; sweep < 100; ++sweep) {
    double off = 0.0;
    for (int i = 0; i < kN; ++i) {
      for (int j = 0; j < kN; ++j) {
        if (i != j)
          off += a[i][j] * a[i][j];
      }
    }
    if (off < 1e-30)
      break;
    for (int p = 0; p < kN - 1; ++p) {
      for (int q = p + 1; q < kN; ++q) {
        if (std::abs(a[p][q]) < 1e-300)
          continue;
        const double theta = (a[q][q] - a[p][p]) / (2.0 * a[p][q]);
        const double t = std::copysign(1.0, theta) /
                         (std::abs(theta) + std::sqrt(theta * theta + 1.0));
        const double c = 1.0 / std::sqrt(t * t + 1.0);
        const double s = t * c;
        for (int k = 0; k < kN; ++k) {
          const double akp = a[k][p];
          const double akq = a[k][q];
          a[k][p] = c * akp - s * akq;
          a[k][q] = s * akp + c * akq;
        }
        for (int k = 0; k < kN; ++k) {
          const double apk = a[p][k];
          const double aqk = a[q][k];
          a[p][k] = c * apk - s * aqk;
          a[q][k] = s * apk + c * aqk;
        }
      }
    }
  }
  double lo = a[0][0];
  for (int i = 1; i < kN; ++i)
    lo = std::min(lo, a[i][i]);
  return lo;
}

void ExpectNear(const EskfMatrix &actual, const EskfMatrix &expected,
                double tol = kTol) {
  for (std::size_t i = 0; i < actual.size(); ++i) {
    EXPECT_NEAR(actual[i], expected[i], tol) << "index " << i;
  }
}

void ExpectRejected(const CovPropagateResult &r, CovPropagateStatus status) {
  EXPECT_FALSE(r.ok);
  EXPECT_EQ(r.status, status);
  EXPECT_FALSE(r.P.has_value());
}

CovPropagateResult Propagate(const EskfNominal &x, const ImuSample &imu,
                             double dt, const EskfCovariance &p,
                             const ProcessNoiseConfig &n) {
  return PropagateCovariance(x, imu, dt, p, n);
}

std::array<double, 9> Skew(const std::array<double, 3> &u) {
  return {0.0, -u[2], u[1], u[2], 0.0, -u[0], -u[1], u[0], 0.0};
}

std::array<double, 9> Mul3(const std::array<double, 9> &a,
                           const std::array<double, 9> &b) {
  std::array<double, 9> c{};
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      for (int k = 0; k < 3; ++k)
        c[3 * i + j] += a[3 * i + k] * b[3 * k + j];
    }
  }
  return c;
}

TEST(EskfCovPropagateTest, ZeroNoiseIdentityPIsPhiPhiT) {
  const EskfNominal x = GoldenNominal();
  const ImuSample imu = GoldenImu();
  const auto phi = BuildPhi(x, imu, 0.05);
  ASSERT_TRUE(phi.has_value());
  const CovPropagateResult r =
      Propagate(x, imu, 0.05, EskfCovariance::Identity(), ProcessNoiseConfig());
  ASSERT_TRUE(r.ok);
  EXPECT_EQ(r.status, CovPropagateStatus::kOk);
  const EskfMatrix pp = MatMul(*phi, Transpose(*phi));
  EskfMatrix sym{};
  for (int i = 0; i < kN; ++i) {
    for (int j = 0; j < kN; ++j) {
      sym[i * kN + j] = 0.5 * (pp[i * kN + j] + pp[j * kN + i]);
    }
  }
  ExpectNear(r.P->row_major(), sym);
}

TEST(EskfCovPropagateTest, PhiBlocksFollowTheContractTable) {
  const EskfNominal x = GoldenNominal();
  const ImuSample imu = GoldenImu();
  const double dt = 0.02;
  const auto phi = BuildPhi(x, imu, dt);
  ASSERT_TRUE(phi.has_value());
  const std::array<double, 9> r = RotationBodyToWorld(x.q_wxyz);
  const std::array<double, 3> omega = {imu.gyro_rad_s[0] - x.b_g[0],
                                       imu.gyro_rad_s[1] - x.b_g[1],
                                       imu.gyro_rad_s[2] - x.b_g[2]};
  const std::array<double, 3> g_b = {
      r[0] * kGravityEnu[0] + r[3] * kGravityEnu[1] + r[6] * kGravityEnu[2],
      r[1] * kGravityEnu[0] + r[4] * kGravityEnu[1] + r[7] * kGravityEnu[2],
      r[2] * kGravityEnu[0] + r[5] * kGravityEnu[1] + r[8] * kGravityEnu[2]};
  const std::array<double, 9> eye = {1, 0, 0, 0, 1, 0, 0, 0, 1};
  const std::array<double, 9> zero{};
  const std::array<double, 9> rv = Mul3(r, Skew(x.v_body));
  const std::array<double, 9> w_x = Skew(omega);
  const std::array<double, 9> g_x = Skew(g_b);
  const std::array<double, 9> v_x = Skew(x.v_body);

  auto scaled = [&](const std::array<double, 9> &m, double s) {
    std::array<double, 9> o;
    for (int i = 0; i < 9; ++i)
      o[i] = s * m[i];
    return o;
  };
  auto eye_plus = [&](const std::array<double, 9> &m, double s) {
    std::array<double, 9> o;
    for (int i = 0; i < 9; ++i)
      o[i] = eye[i] + s * m[i];
    return o;
  };
  // Expected 3x3 blocks, [block_row][block_col].
  const std::array<double, 9> expected[5][5] = {
      {eye, scaled(rv, -dt), scaled(r, dt), zero, zero},
      {zero, eye_plus(w_x, -dt), zero, zero, scaled(eye, -dt)},
      {zero, scaled(g_x, dt), eye_plus(w_x, -dt), scaled(eye, -dt),
       scaled(v_x, -dt)},
      {zero, zero, zero, eye, zero},
      {zero, zero, zero, zero, eye}};
  for (int br = 0; br < 5; ++br) {
    for (int bc = 0; bc < 5; ++bc) {
      for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
          EXPECT_NEAR((*phi)[(3 * br + i) * kN + 3 * bc + j],
                      expected[br][bc][3 * i + j], 1e-15)
              << br << "," << bc << "," << i << "," << j;
        }
      }
    }
  }
}

TEST(EskfCovPropagateTest, QdDiagonal) {
  const double dt = 0.25;
  const auto qd = BuildQd(Noise(2.0, 3.0, 5.0, 7.0), dt);
  ASSERT_TRUE(qd.has_value());
  EskfMatrix expected{};
  const std::pair<int, double> blocks[] = {
      {3, 3.0}, {6, 2.0}, {9, 5.0}, {12, 7.0}};
  for (const auto &[first, sigma] : blocks) {
    for (int i = 0; i < 3; ++i) {
      expected[(first + i) * kN + first + i] = sigma * sigma * dt;
    }
  }
  ExpectNear(*qd, expected, 1e-15);
  EXPECT_FALSE(BuildQd(Noise(0.0, -1.0, 0.0, 0.0), dt).has_value());
}

TEST(EskfCovPropagateTest, ZeroPReturnsQd) {
  const double dt = 0.1;
  const ProcessNoiseConfig noise = Noise(0.2, 0.03, 0.004, 0.0005);
  const CovPropagateResult r =
      Propagate(GoldenNominal(), GoldenImu(), dt, Scaled(0.0), noise);
  ASSERT_TRUE(r.ok);
  ExpectNear(r.P->row_major(), *BuildQd(noise, dt), 1e-15);
}

TEST(EskfCovPropagateTest, StationaryHoverAnalytic) {
  const double pp = 1.0, pth = 2.0, pv = 3.0, pba = 4.0, pbg = 5.0;
  const double dt = 0.1;
  std::array<double, kN> diag;
  for (int i = 0; i < 3; ++i) {
    diag[i] = pp;
    diag[3 + i] = pth;
    diag[6 + i] = pv;
    diag[9 + i] = pba;
    diag[12 + i] = pbg;
  }
  const EskfNominal x;
  const CovPropagateResult r = Propagate(x, HoverImu(x.q_wxyz), dt,
                                         Diagonal(diag), ProcessNoiseConfig());
  ASSERT_TRUE(r.ok);

  // [g_b x] with g_b = (0, 0, -g) is [[0, g, 0], [-g, 0, 0], [0, 0, 0]].
  const double s[9] = {0.0, kG, 0.0, -kG, 0.0, 0.0, 0.0, 0.0, 0.0};
  EskfMatrix e{};
  auto add = [&](int row, int col, double v) { e[row * kN + col] += v; };
  for (int i = 0; i < 3; ++i) {
    add(i, i, pp + pv * dt * dt);
    add(i, 6 + i, dt * pv);
    add(6 + i, i, dt * pv);
    add(3 + i, 3 + i, pth + pbg * dt * dt);
    add(3 + i, 12 + i, -dt * pbg);
    add(12 + i, 3 + i, -dt * pbg);
    add(6 + i, 9 + i, -dt * pba);
    add(9 + i, 6 + i, -dt * pba);
    add(9 + i, 9 + i, pba);
    add(12 + i, 12 + i, pbg);
    add(6 + i, 6 + i, pv + dt * dt * pba);
  }
  add(6, 6, dt * dt * pth * kG * kG);
  add(7, 7, dt * dt * pth * kG * kG);
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      add(6 + i, 3 + j, dt * pth * s[3 * i + j]);
      add(3 + j, 6 + i, dt * pth * s[3 * i + j]);
    }
  }
  ExpectNear(r.P->row_major(), e);
  EXPECT_NEAR(r.P->At(0, 0), 1.03, kTol);
  EXPECT_NEAR(r.P->At(6, 6), 3.0 + 0.01 * (2.0 * kG * kG + 4.0), kTol);
  EXPECT_NEAR(r.P->At(6, 4), 0.1 * 2.0 * kG, kTol);
  EXPECT_NEAR(r.P->At(7, 3), -0.1 * 2.0 * kG, kTol);
  EXPECT_NEAR(r.P->At(3, 12), -0.5, kTol);
}

TEST(EskfCovPropagateTest, SymmetryForAsymmetricP) {
  unsigned long long seed = 12345;
  std::vector<double> vals;
  for (int i = 0; i < kN * kN; ++i) {
    seed = (1103515245ULL * seed + 12345ULL) % (1ULL << 31);
    vals.push_back(static_cast<double>(seed) / static_cast<double>(1ULL << 31) -
                   0.5);
  }
  double asym = 0.0;
  for (int i = 0; i < kN; ++i) {
    for (int j = 0; j < kN; ++j) {
      asym = std::max(asym, std::abs(vals[i * kN + j] - vals[j * kN + i]));
    }
  }
  EXPECT_GT(asym, 0.1);
  const CovPropagateResult r =
      Propagate(GoldenNominal(), GoldenImu(), 0.05,
                *EskfCovariance::FromRowMajor(vals), kGoldenNoise);
  ASSERT_TRUE(r.ok);
  EXPECT_LE(SymErr(r.P->row_major()), 1e-12);
}

TEST(EskfCovPropagateTest, PsdFloor) {
  const EskfNominal level;
  CovPropagateResult r = Propagate(level, HoverImu(level.q_wxyz), 0.1,
                                   Scaled(1e-6), ProcessNoiseConfig());
  ASSERT_TRUE(r.ok);
  EXPECT_GE(MinEigenvalue(r.P->row_major()), -1e-9);

  r = Propagate(GoldenNominal(), GoldenImu(), kGoldenDt,
                EskfCovariance::Identity(), kGoldenNoise);
  ASSERT_TRUE(r.ok);
  EXPECT_GE(MinEigenvalue(r.P->row_major()), -1e-9);
}

TEST(EskfCovPropagateTest, DtRejects) {
  const EskfNominal x = GoldenNominal();
  const ImuSample imu = GoldenImu();
  const EskfCovariance p = EskfCovariance::Identity();
  for (double dt : {0.0, -0.01, kNan, kInf, -kInf, 1.0 + 1e-9}) {
    ExpectRejected(Propagate(x, imu, dt, p, ProcessNoiseConfig()),
                   CovPropagateStatus::kInvalidDt);
  }
  EXPECT_TRUE(Propagate(x, imu, 1.0, p, ProcessNoiseConfig()).ok);
}

TEST(EskfCovPropagateTest, ImuRejects) {
  const EskfNominal x = GoldenNominal();
  const EskfCovariance p = EskfCovariance::Identity();
  for (double bad : {kNan, kInf}) {
    for (int i = 0; i < 3; ++i) {
      ImuSample a;
      a.accel_m_s2[i] = bad;
      ImuSample g;
      g.gyro_rad_s[i] = bad;
      for (const ImuSample &imu : {a, g}) {
        ExpectRejected(Propagate(x, imu, 0.01, p, ProcessNoiseConfig()),
                       CovPropagateStatus::kInvalidImu);
      }
    }
  }
}

TEST(EskfCovPropagateTest, QuaternionAndNominalRejects) {
  const ImuSample imu = GoldenImu();
  const EskfCovariance p = EskfCovariance::Identity();
  EskfNominal x;
  x.q_wxyz = {0.0, 0.0, 0.0, 0.0};
  ExpectRejected(Propagate(x, imu, 0.01, p, ProcessNoiseConfig()),
                 CovPropagateStatus::kInvalidQuaternion);
  x.q_wxyz = {1e-13, 0.0, 0.0, 0.0};
  ExpectRejected(Propagate(x, imu, 0.01, p, ProcessNoiseConfig()),
                 CovPropagateStatus::kInvalidQuaternion);
  EskfNominal nan_v;
  nan_v.v_body = {kNan, 0.0, 0.0};
  ExpectRejected(Propagate(nan_v, imu, 0.01, p, ProcessNoiseConfig()),
                 CovPropagateStatus::kNonFinite);
}

TEST(EskfCovPropagateTest, PRejects) {
  const EskfNominal x = GoldenNominal();
  const ImuSample imu = GoldenImu();
  ExpectRejected(
      Propagate(x, imu, 0.01, EskfCovariance(), ProcessNoiseConfig()),
      CovPropagateStatus::kInvalidP);
  for (double bad : {kNan, kInf}) {
    std::vector<double> vals(kN * kN, 0.0);
    vals[17] = bad;
    ExpectRejected(Propagate(x, imu, 0.01, *EskfCovariance::FromRowMajor(vals),
                             ProcessNoiseConfig()),
                   CovPropagateStatus::kInvalidP);
  }
}

TEST(EskfCovPropagateTest, NoiseRejects) {
  const EskfNominal x = GoldenNominal();
  const ImuSample imu = GoldenImu();
  const EskfCovariance p = EskfCovariance::Identity();
  for (double bad : {-1e-12, kNan, kInf, -kInf}) {
    ProcessNoiseConfig n[4];
    n[0].sigma_accel = bad;
    n[1].sigma_gyro = bad;
    n[2].sigma_accel_bias_rw = bad;
    n[3].sigma_gyro_bias_rw = bad;
    for (const ProcessNoiseConfig &noise : n) {
      ExpectRejected(Propagate(x, imu, 0.01, p, noise),
                     CovPropagateStatus::kInvalidNoise);
    }
  }
}

TEST(EskfCovPropagateTest, RejectPrecedence) {
  const ProcessNoiseConfig bad_noise = Noise(-1.0, 0.0, 0.0, 0.0);
  ImuSample bad_imu;
  bad_imu.accel_m_s2[0] = kNan;
  EXPECT_EQ(
      Propagate(GoldenNominal(), bad_imu, 0.0, EskfCovariance(), bad_noise)
          .status,
      CovPropagateStatus::kInvalidDt);
  EXPECT_EQ(
      Propagate(GoldenNominal(), bad_imu, 0.01, EskfCovariance(), bad_noise)
          .status,
      CovPropagateStatus::kInvalidImu);
  EXPECT_EQ(
      Propagate(GoldenNominal(), GoldenImu(), 0.01, EskfCovariance(), bad_noise)
          .status,
      CovPropagateStatus::kInvalidP);
}

TEST(EskfCovPropagateTest, OverflowingOutputIsNonFinite) {
  EskfNominal x;
  x.v_body = {1e200, 0.0, 0.0};
  ExpectRejected(Propagate(x, GoldenImu(), 0.5, EskfCovariance::Identity(),
                           ProcessNoiseConfig()),
                 CovPropagateStatus::kNonFinite);
}

TEST(EskfCovPropagateTest, DeterministicAndGolden) {
  const CovPropagateResult first = Propagate(
      GoldenNominal(), GoldenImu(), kGoldenDt, GoldenP(), kGoldenNoise);
  const CovPropagateResult second = Propagate(
      GoldenNominal(), GoldenImu(), kGoldenDt, GoldenP(), kGoldenNoise);
  ASSERT_TRUE(first.ok);
  ASSERT_TRUE(second.ok);
  EXPECT_EQ(first.P->row_major(), second.P->row_major());
  for (const GoldenEntry &g : kGolden) {
    EXPECT_NEAR(first.P->At(g.row, g.col), g.value, kTol)
        << g.row << "," << g.col;
  }
}

} // namespace
} // namespace intrinsic::estimation
