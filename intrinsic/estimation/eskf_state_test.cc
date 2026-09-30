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

// Mirrors eskf_state_test.py.

#include "intrinsic/estimation/eskf_state.h"

#include <array>
#include <cstring>
#include <optional>
#include <string_view>
#include <vector>

#include <gtest/gtest.h>

namespace intrinsic::estimation {
namespace {

static_assert(kNominalDim == 16);
static_assert(kErrorDim == 15);
static_assert(kCovDim == 15);
static_assert(kNominalFields.size() == 5);
static_assert(kErrorFields.size() == 5);

TEST(EskfStateTest, Dimensions) {
  EXPECT_EQ(EskfNominal().ToArray().size(), 16u);
  EXPECT_EQ(EskfError().ToArray().size(), 15u);
  EXPECT_EQ(EskfCovariance::Identity().row_major().size(), 225u);
}

TEST(EskfStateTest, NominalIndexConstantsAndUnits) {
  EXPECT_EQ(kNominalPEnu, 0);
  EXPECT_EQ(kNominalQWxyz, 3);
  EXPECT_EQ(kNominalVBody, 7);
  EXPECT_EQ(kNominalBa, 10);
  EXPECT_EQ(kNominalBg, 13);
  struct Expected {
    std::string_view symbol;
    int first;
    int count;
    std::string_view unit;
  };
  const Expected expected[] = {{"p_enu", 0, 3, "m"},
                               {"q_wxyz", 3, 4, "1"},
                               {"v_body", 7, 3, "m/s"},
                               {"b_a", 10, 3, "m/s^2"},
                               {"b_g", 13, 3, "rad/s"}};
  for (size_t i = 0; i < 5; ++i) {
    EXPECT_EQ(kNominalFields[i].symbol, expected[i].symbol);
    EXPECT_EQ(kNominalFields[i].first, expected[i].first);
    EXPECT_EQ(kNominalFields[i].count, expected[i].count);
    EXPECT_EQ(kNominalFields[i].unit, expected[i].unit);
  }
}

TEST(EskfStateTest, ErrorIndexConstantsAndUnits) {
  EXPECT_EQ(kErrorDp, 0);
  EXPECT_EQ(kErrorDtheta, 3);
  EXPECT_EQ(kErrorDv, 6);
  EXPECT_EQ(kErrorDba, 9);
  EXPECT_EQ(kErrorDbg, 12);
  struct Expected {
    std::string_view symbol;
    int first;
    std::string_view unit;
  };
  const Expected expected[] = {{"dp", 0, "m"},
                               {"dtheta", 3, "rad"},
                               {"dv", 6, "m/s"},
                               {"dba", 9, "m/s^2"},
                               {"dbg", 12, "rad/s"}};
  for (size_t i = 0; i < 5; ++i) {
    EXPECT_EQ(kErrorFields[i].symbol, expected[i].symbol);
    EXPECT_EQ(kErrorFields[i].first, expected[i].first);
    EXPECT_EQ(kErrorFields[i].count, 3);
    EXPECT_EQ(kErrorFields[i].unit, expected[i].unit);
  }
}

TEST(EskfStateTest, NominalDefaults) {
  const auto a = EskfNominal().ToArray();
  const std::array<double, 16> want = {0, 0, 0, 1, 0, 0, 0, 0,
                                       0, 0, 0, 0, 0, 0, 0, 0};
  EXPECT_EQ(a, want);
}

TEST(EskfStateTest, ErrorDefaultsAreZero) {
  for (double v : EskfError().ToArray()) EXPECT_EQ(v, 0.0);
}

TEST(EskfStateTest, DefaultsAreBitStable) {
  const auto n1 = EskfNominal().ToArray();
  const auto n2 = EskfNominal().ToArray();
  EXPECT_EQ(std::memcmp(n1.data(), n2.data(), sizeof(n1)), 0);
  const auto e1 = EskfError().ToArray();
  const auto e2 = EskfError().ToArray();
  EXPECT_EQ(std::memcmp(e1.data(), e2.data(), sizeof(e1)), 0);
}

TEST(EskfStateTest, NominalIndexOrderFixture) {
  EskfNominal n;
  n.p_enu = {0, 1, 2};
  n.q_wxyz = {3, 4, 5, 6};
  n.v_body = {7, 8, 9};
  n.b_a = {10, 11, 12};
  n.b_g = {13, 14, 15};
  const auto a = n.ToArray();
  for (int i = 0; i < kNominalDim; ++i) EXPECT_EQ(a[i], i) << i;

  std::vector<double> marked(kNominalDim);
  for (int i = 0; i < kNominalDim; ++i) marked[i] = 100 + i;
  const auto back = EskfNominal::FromVector(marked);
  ASSERT_TRUE(back.has_value());
  EXPECT_EQ(back->p_enu[0], 100);
  EXPECT_EQ(back->q_wxyz[0], 103);
  EXPECT_EQ(back->q_wxyz[3], 106);
  EXPECT_EQ(back->v_body[0], 107);
  EXPECT_EQ(back->b_a[0], 110);
  EXPECT_EQ(back->b_g[2], 115);
  for (int i = 0; i < kNominalDim; ++i) EXPECT_EQ(back->ToArray()[i], 100 + i);
}

TEST(EskfStateTest, ErrorIndexOrderFixture) {
  EskfError e;
  e.dp = {0, 1, 2};
  e.dtheta = {3, 4, 5};
  e.dv = {6, 7, 8};
  e.dba = {9, 10, 11};
  e.dbg = {12, 13, 14};
  const auto a = e.ToArray();
  for (int i = 0; i < kErrorDim; ++i) EXPECT_EQ(a[i], i) << i;

  std::vector<double> marked(kErrorDim);
  for (int i = 0; i < kErrorDim; ++i) marked[i] = 100 + i;
  const auto back = EskfError::FromVector(marked);
  ASSERT_TRUE(back.has_value());
  EXPECT_EQ(back->dp[0], 100);
  EXPECT_EQ(back->dtheta[0], 103);
  EXPECT_EQ(back->dv[0], 106);
  EXPECT_EQ(back->dba[0], 109);
  EXPECT_EQ(back->dbg[2], 114);
}

TEST(EskfStateTest, WrongSizesRejected) {
  EXPECT_FALSE(EskfNominal::FromVector(std::vector<double>(15)).has_value());
  EXPECT_FALSE(EskfNominal::FromVector(std::vector<double>(17)).has_value());
  EXPECT_FALSE(EskfError::FromVector(std::vector<double>(16)).has_value());
  EXPECT_FALSE(EskfError::FromVector(std::vector<double>(14)).has_value());
  EXPECT_FALSE(
      EskfCovariance::FromRowMajor(std::vector<double>(224)).has_value());
  EXPECT_FALSE(
      EskfCovariance::FromRowMajor(std::vector<double>(226)).has_value());
}

TEST(EskfStateTest, CovarianceDefaultIsUnknownNotZeros) {
  EskfCovariance p;
  EXPECT_FALSE(p.has_value());
  const auto zeros =
      EskfCovariance::FromRowMajor(std::vector<double>(kCovDim * kCovDim, 0));
  ASSERT_TRUE(zeros.has_value());
  EXPECT_TRUE(zeros->has_value());
}

TEST(EskfStateTest, CovarianceRowMajorOrder) {
  std::vector<double> m(kCovDim * kCovDim);
  for (int i = 0; i < kCovDim * kCovDim; ++i) m[i] = i;
  const auto p = EskfCovariance::FromRowMajor(m);
  ASSERT_TRUE(p.has_value());
  EXPECT_EQ(p->At(0, 1), 1);
  EXPECT_EQ(p->At(1, 0), 15);
  EXPECT_EQ(p->At(14, 14), 224);
}

TEST(EskfStateTest, IdentityIsKnownDiagonal) {
  const auto p = EskfCovariance::Identity();
  ASSERT_TRUE(p.has_value());
  for (int r = 0; r < kCovDim; ++r) {
    for (int c = 0; c < kCovDim; ++c) EXPECT_EQ(p.At(r, c), r == c ? 1 : 0);
  }
}

}  // namespace
}  // namespace intrinsic::estimation
