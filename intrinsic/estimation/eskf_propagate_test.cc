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

// Mirrors eskf_propagate_test.py.

#include "intrinsic/estimation/eskf_propagate.h"

#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <cstddef>
#include <limits>

#include "intrinsic/estimation/eskf_state.h"

namespace intrinsic::estimation {
namespace {

constexpr double kTol = 1e-9;
const double kNan = std::numeric_limits<double>::quiet_NaN();
const double kInf = std::numeric_limits<double>::infinity();

static_assert(kGravityEnu[0] == 0.0 && kGravityEnu[1] == 0.0 &&
              kGravityEnu[2] == -9.80665);

// a = -R^T g, so the specific force cancels gravity in the body frame.
ImuSample HoverImu(const std::array<double, 4>& q,
                   const std::array<double, 3>& omega = {0.0, 0.0, 0.0}) {
  const std::array<double, 9> r = RotationBodyToWorld(q);
  ImuSample imu;
  imu.accel_m_s2 = {
      -(r[0] * kGravityEnu[0] + r[3] * kGravityEnu[1] + r[6] * kGravityEnu[2]),
      -(r[1] * kGravityEnu[0] + r[4] * kGravityEnu[1] + r[7] * kGravityEnu[2]),
      -(r[2] * kGravityEnu[0] + r[5] * kGravityEnu[1] + r[8] * kGravityEnu[2])};
  imu.gyro_rad_s = omega;
  return imu;
}

template <std::size_t N>
void ExpectNear(const std::array<double, N>& actual,
                const std::array<double, N>& expected, double tol = kTol) {
  for (std::size_t i = 0; i < N; ++i) {
    EXPECT_NEAR(actual[i], expected[i], tol) << "index " << i;
  }
}

void ExpectRejected(const PropagateResult& r, PropagateStatus status) {
  EXPECT_FALSE(r.ok);
  EXPECT_EQ(r.status, status);
  EXPECT_FALSE(r.nominal.has_value());
}

TEST(EskfPropagateTest, StationaryLevelIsUnchanged) {
  EskfNominal x;
  x.p_enu = {1.0, -2.0, 3.0};
  for (double dt : {1e-4, 0.01, 0.5, 1.0}) {
    const PropagateResult r = PropagateNominal(x, HoverImu(x.q_wxyz), dt);
    ASSERT_TRUE(r.ok);
    EXPECT_EQ(r.status, PropagateStatus::kOk);
    ExpectNear(r.nominal->ToArray(), x.ToArray());
  }
}

TEST(EskfPropagateTest, StationaryTiltedIsUnchanged) {
  const double s = std::sqrt(0.5);
  for (const std::array<double, 4>& q :
       {std::array<double, 4>{s, 0.0, 0.0, s},
        std::array<double, 4>{0.9, 0.1, -0.2, 0.3}}) {
    EskfNominal x;
    x.q_wxyz = q;
    const PropagateResult r = PropagateNominal(x, HoverImu(q), 0.02);
    ASSERT_TRUE(r.ok);
    ExpectNear(r.nominal->p_enu, x.p_enu);
    ExpectNear(r.nominal->v_body, x.v_body);
    // A non-unit input renormalizes, so compare against its normalized form.
    const double n =
        std::sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
    ExpectNear(r.nominal->q_wxyz, {q[0] / n, q[1] / n, q[2] / n, q[3] / n});
  }
}

TEST(EskfPropagateTest, ConstantRateAboutBodyZ) {
  const double wz = 0.5;
  const double dt = 0.01;
  const EskfNominal x;
  const PropagateResult r =
      PropagateNominal(x, HoverImu(x.q_wxyz, {0.0, 0.0, wz}), dt);
  ASSERT_TRUE(r.ok);
  ExpectNear(r.nominal->p_enu, {0.0, 0.0, 0.0});
  ExpectNear(r.nominal->v_body, {0.0, 0.0, 0.0});
  const double n = std::sqrt(1.0 + (0.5 * wz * dt) * (0.5 * wz * dt));
  ExpectNear(r.nominal->q_wxyz, {1.0 / n, 0.0, 0.0, 0.5 * wz * dt / n});
  const auto& q = r.nominal->q_wxyz;
  EXPECT_NEAR(2.0 * std::atan2(q[3], q[0]), wz * dt, 1e-7);
}

TEST(EskfPropagateTest, ConstantRateIntegratedYaw) {
  const double wz = 0.5;
  const double dt = 0.001;
  const int steps = 1000;
  EskfNominal x;
  for (int i = 0; i < steps; ++i) {
    const PropagateResult r =
        PropagateNominal(x, HoverImu(x.q_wxyz, {0.0, 0.0, wz}), dt);
    ASSERT_TRUE(r.ok);
    x = *r.nominal;
  }
  EXPECT_NEAR(2.0 * std::atan2(x.q_wxyz[3], x.q_wxyz[0]), wz * dt * steps,
              1e-4);
  ExpectNear(x.p_enu, {0.0, 0.0, 0.0});
}

TEST(EskfPropagateTest, InvalidDt) {
  const EskfNominal x;
  const ImuSample imu = HoverImu(x.q_wxyz);
  for (double dt : {0.0, -0.01, kNan, kInf, -kInf, 1.0000001, 2.0}) {
    ExpectRejected(PropagateNominal(x, imu, dt), PropagateStatus::kInvalidDt);
  }
}

TEST(EskfPropagateTest, InvalidImu) {
  const EskfNominal x;
  const ImuSample good = HoverImu(x.q_wxyz);
  for (double bad : {kNan, kInf, -kInf}) {
    for (int i = 0; i < 3; ++i) {
      ImuSample bad_accel = good;
      bad_accel.accel_m_s2[i] = bad;
      ExpectRejected(PropagateNominal(x, bad_accel, 0.01),
                     PropagateStatus::kInvalidImu);
      ImuSample bad_gyro = good;
      bad_gyro.gyro_rad_s[i] = bad;
      ExpectRejected(PropagateNominal(x, bad_gyro, 0.01),
                     PropagateStatus::kInvalidImu);
    }
  }
}

TEST(EskfPropagateTest, InvalidDtWinsOverInvalidImu) {
  ImuSample imu;
  imu.accel_m_s2[0] = kNan;
  ExpectRejected(PropagateNominal(EskfNominal(), imu, 0.0),
                 PropagateStatus::kInvalidDt);
}

TEST(EskfPropagateTest, NearZeroQuaternion) {
  for (const std::array<double, 4>& q :
       {std::array<double, 4>{0.0, 0.0, 0.0, 0.0},
        std::array<double, 4>{1e-13, 0.0, 0.0, 0.0},
        std::array<double, 4>{0.0, 0.0, 0.0, -1e-13}}) {
    EskfNominal x;
    x.q_wxyz = q;
    ExpectRejected(PropagateNominal(x, ImuSample(), 0.01),
                   PropagateStatus::kInvalidQuaternion);
  }
}

TEST(EskfPropagateTest, NonFiniteState) {
  EskfNominal p;
  p.p_enu[0] = kNan;
  EskfNominal q;
  q.q_wxyz[1] = kInf;
  EskfNominal v;
  v.v_body[1] = kNan;
  EskfNominal ba;
  ba.b_a[2] = kInf;
  EskfNominal bg;
  bg.b_g[0] = kNan;
  for (const EskfNominal& x : {p, q, v, ba, bg}) {
    ExpectRejected(PropagateNominal(x, ImuSample(), 0.01),
                   PropagateStatus::kNonFiniteState);
  }
}

TEST(EskfPropagateTest, OverflowOutputIsNonFiniteState) {
  EskfNominal x;
  x.v_body = {1e308, 0.0, 0.0};
  ImuSample imu;
  imu.accel_m_s2 = {1e308, 0.0, 0.0};
  ExpectRejected(PropagateNominal(x, imu, 1.0),
                 PropagateStatus::kNonFiniteState);
}

TEST(EskfPropagateTest, VelocityAndPositionUsePreUpdateAttitude) {
  const double dt = 0.1;
  EskfNominal x;
  x.v_body = {1.0, 0.0, 0.0};
  const PropagateResult r =
      PropagateNominal(x, HoverImu(x.q_wxyz, {0.0, 0.0, 1.0}), dt);
  ASSERT_TRUE(r.ok);
  ExpectNear(r.nominal->v_body, {1.0, -dt, 0.0});
  ExpectNear(r.nominal->p_enu, {dt, 0.0, 0.0});
}

TEST(EskfPropagateTest, BiasesAreSubtractedAndHeld) {
  EskfNominal x;
  x.b_a = {0.1, -0.2, 0.3};
  x.b_g = {0.01, 0.02, -0.03};
  ImuSample imu;
  imu.accel_m_s2 = {x.b_a[0], x.b_a[1], 9.80665 + x.b_a[2]};
  imu.gyro_rad_s = x.b_g;
  const PropagateResult r = PropagateNominal(x, imu, 0.05);
  ASSERT_TRUE(r.ok);
  EXPECT_EQ(r.nominal->b_a, x.b_a);
  EXPECT_EQ(r.nominal->b_g, x.b_g);
  ExpectNear(r.nominal->v_body, {0.0, 0.0, 0.0});
  ExpectNear(r.nominal->q_wxyz, {1.0, 0.0, 0.0, 0.0});
}

TEST(EskfPropagateTest, DtUpperBoundIsInclusive) {
  const EskfNominal x;
  EXPECT_TRUE(PropagateNominal(x, HoverImu(x.q_wxyz), 1.0).ok);
}

TEST(EskfPropagateTest, DeterministicAndGolden) {
  EskfNominal x;
  x.p_enu = {1.0, 2.0, 3.0};
  x.q_wxyz = {0.9, 0.1, -0.2, 0.3};
  x.v_body = {0.5, -0.25, 0.1};
  x.b_a = {0.01, -0.02, 0.03};
  x.b_g = {0.001, 0.002, -0.003};
  ImuSample imu;
  imu.accel_m_s2 = {0.3, -0.4, 9.5};
  imu.gyro_rad_s = {0.05, -0.1, 0.2};
  const PropagateResult first = PropagateNominal(x, imu, 0.01);
  const PropagateResult second = PropagateNominal(x, imu, 0.01);
  ASSERT_TRUE(first.ok);
  ASSERT_TRUE(second.ok);
  EXPECT_EQ(first.nominal->ToArray(), second.nominal->ToArray());
  ExpectNear(first.nominal->ToArray(),
             {1.004842105263158, 2.000342105263158, 3.0029473684210526,
              0.9229376970626407, 0.10277269512801493, -0.20569518299429798,
              0.3087284764477931, 0.45913878421052634, -0.2609596736842105,
              0.10656878947368423, 0.01, -0.02, 0.03, 0.001, 0.002, -0.003});
}

}  // namespace
}  // namespace intrinsic::estimation
