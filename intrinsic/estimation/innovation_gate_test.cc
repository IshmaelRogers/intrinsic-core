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

// Mirrors innovation_gate_test.py.

#include "intrinsic/estimation/innovation_gate.h"

#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <limits>
#include <map>
#include <string>
#include <vector>

#include "intrinsic/estimation/eskf_state.h"

namespace intrinsic::estimation {
namespace {

constexpr int kN = kCovDim;
constexpr double kTol = 1e-9;
const double kNan = std::numeric_limits<double>::quiet_NaN();
const double kInf = std::numeric_limits<double>::infinity();

// H (rows.size() x 15) with a single 1 per row at the given column.
std::vector<double> RowSelector(const std::vector<int>& cols) {
  std::vector<double> h(cols.size() * kN, 0.0);
  for (size_t i = 0; i < cols.size(); ++i) h[i * kN + cols[i]] = 1.0;
  return h;
}

// P with the given {index: value} diagonal entries, all else zero.
EskfCovariance SparseP(const std::map<int, double>& diag) {
  std::vector<double> v(kN * kN, 0.0);
  for (const auto& [i, value] : diag) v[i * kN + i] = value;
  return *EskfCovariance::FromRowMajor(v);
}

// Analytic: S = diag(3 + 1, 0.5 + 0.5) = diag(4, 1), nu = (2, 1) gives
// d^2 = 4/4 + 1/1 = 2 exactly in binary floating point.
struct Inputs {
  std::vector<double> nu;
  std::vector<double> h;
  EskfCovariance p;
  std::vector<double> r;
};

Inputs BoundaryInputs() {
  return {{2.0, 1.0},
          RowSelector({0, 1}),
          SparseP({{0, 3.0}, {1, 0.5}}),
          {1.0, 0.0, 0.0, 0.5}};
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

Inputs GoldenInputs() {
  std::vector<double> h;
  for (int i = 0; i < 3; ++i) {
    for (int k = 0; k < kN; ++k)
      h.push_back(0.1 * ((i * 5 + k * 3) % 7) - 0.25);
  }
  return {{0.3, -0.2, 0.5},
          h,
          GoldenP(),
          {0.5, 0.1, 0.0, 0.1, 0.4, 0.05, 0.0, 0.05, 0.3}};
}

// Independent numpy evaluation of sym(S)^-1 with sym(P).
constexpr double kGoldenD2 = 0.3613017414836702;
constexpr double kGoldenNorm = 0.6164414002968976;
struct GoldenS {
  int row;
  int col;
  double value;
};
const GoldenS kGoldenS[] = {
    {0, 0, 1.6041000000000003},
    {0, 1, -0.21315},
    {1, 2, -0.0508499999999999},
    {2, 2, 1.2932000000000003},
};

GateResult Gate(const Inputs& in, double threshold) {
  return GateInnovation(in.nu, in.h, in.p, in.r, threshold);
}

void ExpectRejected(const GateResult& res, GateStatus status) {
  EXPECT_FALSE(res.ok);
  EXPECT_FALSE(res.evaluated);
  EXPECT_FALSE(res.accepted);
  EXPECT_EQ(res.status, status);
  EXPECT_FALSE(res.diagnostics.has_value());
}

TEST(InnovationGateTest, ExactBoundaryAccepts) {
  const GateResult res = Gate(BoundaryInputs(), 2.0);
  EXPECT_TRUE(res.ok);
  EXPECT_TRUE(res.evaluated);
  EXPECT_TRUE(res.accepted);
  EXPECT_EQ(res.status, GateStatus::kOkAccept);
  ASSERT_TRUE(res.diagnostics.has_value());
  EXPECT_EQ(res.diagnostics->mahalanobis_sq, 2.0);
}

TEST(InnovationGateTest, JustOverRejects) {
  Inputs in = BoundaryInputs();
  GateResult res = Gate(in, 2.0 - 1e-9);
  EXPECT_TRUE(res.ok);
  EXPECT_TRUE(res.evaluated);
  EXPECT_FALSE(res.accepted);
  EXPECT_EQ(res.status, GateStatus::kOkReject);
  ASSERT_TRUE(res.diagnostics.has_value());
  EXPECT_EQ(res.diagnostics->mahalanobis_sq, 2.0);
  in.nu = {2.0 * (1.0 + 1e-6), 1.0};
  res = Gate(in, 2.0);
  EXPECT_EQ(res.status, GateStatus::kOkReject);
  ASSERT_TRUE(res.diagnostics.has_value());
  EXPECT_GT(res.diagnostics->mahalanobis_sq, 2.0);
}

TEST(InnovationGateTest, DeepAcceptAndReject) {
  Inputs in = BoundaryInputs();
  in.nu = {0.01, 0.01};
  GateResult res = Gate(in, 5.991);
  EXPECT_EQ(res.status, GateStatus::kOkAccept);
  ASSERT_TRUE(res.diagnostics.has_value());
  EXPECT_LT(res.diagnostics->mahalanobis_sq, 1e-3);
  in.nu = {100.0, 100.0};
  res = Gate(in, 5.991);
  EXPECT_EQ(res.status, GateStatus::kOkReject);
  EXPECT_FALSE(res.accepted);
  ASSERT_TRUE(res.diagnostics.has_value());
  EXPECT_GT(res.diagnostics->mahalanobis_sq, 1e3);
}

TEST(InnovationGateTest, ScalarAnalytic) {
  const GateResult res =
      GateInnovation({2.0}, RowSelector({4}), SparseP({{4, 3.0}}), {1.0}, 1.0);
  EXPECT_EQ(res.status, GateStatus::kOkAccept);
  ASSERT_TRUE(res.diagnostics.has_value());
  EXPECT_EQ(res.diagnostics->mahalanobis_sq, 1.0);
  EXPECT_EQ(res.diagnostics->S, std::vector<double>({4.0}));
  EXPECT_EQ(res.diagnostics->dof, 1);
}

TEST(InnovationGateTest, SingularS) {
  GateResult res = GateInnovation({1.0}, std::vector<double>(kN, 0.0),
                                  EskfCovariance::Identity(), {0.0}, 3.841);
  ExpectRejected(res, GateStatus::kSingularS);
  res = GateInnovation({1.0, 1.0}, std::vector<double>(2 * kN, 0.0),
                       EskfCovariance::Identity(), std::vector<double>(4, 0.0),
                       5.991);
  ExpectRejected(res, GateStatus::kSingularS);
}

TEST(InnovationGateTest, IndefiniteSIsSingular) {
  const std::vector<double> h = RowSelector({0});
  ExpectRejected(
      GateInnovation({1.0}, h, EskfCovariance::Identity(), {-2.0}, 3.841),
      GateStatus::kSingularS);
  ExpectRejected(
      GateInnovation({1.0}, h, EskfCovariance::Identity(), {-1.0}, 3.841),
      GateStatus::kSingularS);
}

TEST(InnovationGateTest, PivotFloor) {
  const std::vector<double> h = RowSelector({0});
  ExpectRejected(GateInnovation({1.0}, h, SparseP({}), {1e-12}, 3.841),
                 GateStatus::kSingularS);
  EXPECT_EQ(GateInnovation({1.0}, h, SparseP({}), {2e-12}, 3.841).status,
            GateStatus::kOkReject);
}

TEST(InnovationGateTest, NonFiniteNu) {
  for (double bad : {kNan, kInf, -kInf}) {
    for (int i = 0; i < 2; ++i) {
      Inputs in = BoundaryInputs();
      in.nu[i] = bad;
      ExpectRejected(Gate(in, 2.0), GateStatus::kInvalidNu);
    }
  }
}

TEST(InnovationGateTest, NonFiniteAndUnknownP) {
  Inputs in = BoundaryInputs();
  in.p = EskfCovariance();
  ExpectRejected(Gate(in, 2.0), GateStatus::kInvalidP);
  for (double bad : {kNan, kInf, -kInf}) {
    std::vector<double> vals(kN * kN, 0.0);
    vals[17] = bad;
    in.p = *EskfCovariance::FromRowMajor(vals);
    ExpectRejected(Gate(in, 2.0), GateStatus::kInvalidP);
  }
}

TEST(InnovationGateTest, NonFiniteR) {
  for (double bad : {kNan, kInf, -kInf}) {
    for (int idx : {0, 1, 3}) {
      Inputs in = BoundaryInputs();
      in.r[idx] = bad;
      ExpectRejected(Gate(in, 2.0), GateStatus::kInvalidR);
    }
  }
}

TEST(InnovationGateTest, InvalidThreshold) {
  const Inputs in = BoundaryInputs();
  for (double bad : {kNan, kInf, -kInf, 0.0, -0.0, -1.0, -1e-300}) {
    ExpectRejected(Gate(in, bad), GateStatus::kInvalidThreshold);
  }
  EXPECT_TRUE(Gate(in, 1e-300).ok);
}

TEST(InnovationGateTest, NonFiniteH) {
  for (double bad : {kNan, kInf}) {
    Inputs in = BoundaryInputs();
    in.h[3] = bad;
    ExpectRejected(Gate(in, 2.0), GateStatus::kNonFinite);
  }
}

TEST(InnovationGateTest, OverflowIsNonFinite) {
  const std::vector<double> h = RowSelector({0});
  ExpectRejected(
      GateInnovation({1.0}, h, SparseP({{0, 1e308}}), {1e308}, 3.841),
      GateStatus::kNonFinite);
  ExpectRejected(
      GateInnovation({1e200}, h, EskfCovariance::Identity(), {1.0}, 3.841),
      GateStatus::kNonFinite);
}

TEST(InnovationGateTest, DimMismatch) {
  const Inputs base = BoundaryInputs();
  std::map<std::string, Inputs> cases;
  cases["empty nu"] = base;
  cases["empty nu"].nu = {};
  cases["empty nu"].h = {};
  cases["h short"] = base;
  cases["h short"].h.pop_back();
  cases["h long"] = base;
  cases["h long"].h.push_back(0.0);
  cases["h 14 cols"] = base;
  cases["h 14 cols"].h.assign(2 * 14, 0.0);
  cases["h 16 cols"] = base;
  cases["h 16 cols"].h.assign(2 * 16, 0.0);
  cases["r short"] = base;
  cases["r short"].r.pop_back();
  cases["r long"] = base;
  cases["r long"].r.push_back(0.0);
  cases["r 1x1 for m=2"] = base;
  cases["r 1x1 for m=2"].r = {1.0};
  for (const auto& [name, in] : cases) {
    SCOPED_TRACE(name);
    ExpectRejected(Gate(in, 2.0), GateStatus::kInvalidDim);
  }
}

TEST(InnovationGateTest, AsymmetricR) {
  Inputs in = BoundaryInputs();
  in.r = {1.0, 0.0, 1e-11, 0.5};
  ExpectRejected(Gate(in, 2.0), GateStatus::kInvalidR);
  in.r = {1.0, 0.25, -0.25, 0.5};
  ExpectRejected(Gate(in, 2.0), GateStatus::kInvalidR);
  in.r = {1.0, 0.0, 5e-13, 0.5};
  EXPECT_TRUE(Gate(in, 2.0).ok);
}

TEST(InnovationGateTest, RejectPrecedence) {
  const Inputs ok = BoundaryInputs();
  Inputs in = ok;
  in.h.pop_back();
  in.p = EskfCovariance();
  in.r = {kNan, 0.0, 0.0, 0.5};
  EXPECT_EQ(Gate(in, kNan).status, GateStatus::kInvalidDim);
  in.h = ok.h;
  in.nu = {kNan, 1.0};
  EXPECT_EQ(Gate(in, kNan).status, GateStatus::kInvalidP);
  in.p = ok.p;
  EXPECT_EQ(Gate(in, kNan).status, GateStatus::kInvalidR);
  in.r = ok.r;
  EXPECT_EQ(Gate(in, kNan).status, GateStatus::kInvalidNu);
  in.nu = ok.nu;
  EXPECT_EQ(Gate(in, kNan).status, GateStatus::kInvalidThreshold);
}

TEST(InnovationGateTest, DiagnosticsOnAcceptAndReject) {
  const Inputs in = BoundaryInputs();
  const std::pair<double, GateStatus> cases[] = {{2.0, GateStatus::kOkAccept},
                                                 {1.0, GateStatus::kOkReject}};
  for (const auto& [threshold, status] : cases) {
    const GateResult res = Gate(in, threshold);
    EXPECT_EQ(res.status, status);
    ASSERT_TRUE(res.diagnostics.has_value());
    const GateDiagnostics& diag = *res.diagnostics;
    EXPECT_EQ(diag.threshold, threshold);
    EXPECT_EQ(diag.dof, 2);
    EXPECT_NEAR(diag.innovation_norm, std::sqrt(5.0), kTol);
    EXPECT_EQ(diag.mahalanobis_sq, 2.0);
    EXPECT_EQ(diag.S, std::vector<double>({4.0, 0.0, 0.0, 1.0}));
  }
}

TEST(InnovationGateTest, PIsSymmetrized) {
  const Inputs base = BoundaryInputs();
  std::vector<double> asym(kN * kN, 0.0);
  asym[0] = 3.0;
  asym[kN + 1] = 0.5;
  std::vector<double> sym = asym;
  asym[0 * kN + 1] = 0.4;
  sym[0 * kN + 1] = 0.2;
  sym[1 * kN + 0] = 0.2;
  Inputs a = base;
  a.p = *EskfCovariance::FromRowMajor(asym);
  Inputs b = base;
  b.p = *EskfCovariance::FromRowMajor(sym);
  const GateResult ra = Gate(a, 2.0);
  const GateResult rb = Gate(b, 2.0);
  ASSERT_TRUE(ra.diagnostics.has_value());
  ASSERT_TRUE(rb.diagnostics.has_value());
  EXPECT_EQ(ra.diagnostics->mahalanobis_sq, rb.diagnostics->mahalanobis_sq);
  EXPECT_EQ(ra.diagnostics->S, rb.diagnostics->S);
  EXPECT_EQ(ra.diagnostics->S[1], ra.diagnostics->S[2]);
}

TEST(InnovationGateTest, DoesNotWriteInputs) {
  const Inputs in = GoldenInputs();
  const Inputs copy = in;
  Gate(in, 7.815);
  EXPECT_EQ(in.nu, copy.nu);
  EXPECT_EQ(in.h, copy.h);
  EXPECT_EQ(in.r, copy.r);
  EXPECT_EQ(in.p.row_major(), copy.p.row_major());
}

TEST(InnovationGateTest, DeterminismAndGolden) {
  const Inputs in = GoldenInputs();
  const GateResult first = Gate(in, 7.815);
  const GateResult second = Gate(in, 7.815);
  EXPECT_EQ(first.status, GateStatus::kOkAccept);
  ASSERT_TRUE(first.diagnostics.has_value());
  ASSERT_TRUE(second.diagnostics.has_value());
  EXPECT_EQ(first.diagnostics->mahalanobis_sq,
            second.diagnostics->mahalanobis_sq);
  EXPECT_EQ(first.diagnostics->S, second.diagnostics->S);
  EXPECT_EQ(first.diagnostics->innovation_norm,
            second.diagnostics->innovation_norm);
  const GateDiagnostics& diag = *first.diagnostics;
  EXPECT_NEAR(diag.mahalanobis_sq, kGoldenD2, kTol);
  EXPECT_NEAR(diag.innovation_norm, kGoldenNorm, kTol);
  EXPECT_EQ(diag.dof, 3);
  for (const GoldenS& e : kGoldenS) {
    EXPECT_NEAR(diag.S[e.row * 3 + e.col], e.value, kTol);
    EXPECT_NEAR(diag.S[e.col * 3 + e.row], e.value, kTol);
  }
}

TEST(InnovationGateTest, GoldenThresholdStraddle) {
  const Inputs in = GoldenInputs();
  EXPECT_EQ(Gate(in, kGoldenD2 + 1e-6).status, GateStatus::kOkAccept);
  EXPECT_EQ(Gate(in, kGoldenD2 - 1e-6).status, GateStatus::kOkReject);
}

}  // namespace
}  // namespace intrinsic::estimation
