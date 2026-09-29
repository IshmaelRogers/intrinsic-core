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

#include "intrinsic/vehicle/dynamics/added_mass_coriolis_matrix.h"

#include <array>
#include <cmath>
#include <string_view>

#include "intrinsic/vehicle/dynamics/added_mass_matrix.h"

namespace intrinsic::vehicle::dynamics {
namespace {

using parameters::kInertiaDof;

static_assert(kSpatialDof == parameters::kSpatialDof);
static_assert(kInertiaDof == 3);
static_assert(kSpatialDof == 6);

constexpr std::string_view kBodyTwistNonFiniteMessage =
    "body_twist must be finite";
constexpr std::string_view kCoriolisMatrixNonFiniteMessage =
    "added-mass Coriolis matrix is not finite";

StatusOr<AddedMassCoriolisMatrix> Reject(std::string_view message) {
  return StatusOr<AddedMassCoriolisMatrix>::Failure(
      DynamicsStatus::InvalidArgument(message));
}

// -0 compares equal to 0 and is a different bit pattern. Stored zeros use
// +0 so a zero-velocity result is the zero matrix.
double PositiveZero(double value) { return value == 0.0 ? 0.0 : value; }

// -S(a) with S(a) b = a cross b.
void NegatedSkew(const double a[kInertiaDof],
                 double out[kInertiaDof][kInertiaDof]) {
  out[0][0] = 0.0;
  out[0][1] = a[2];
  out[0][2] = -a[1];
  out[1][0] = -a[2];
  out[1][1] = 0.0;
  out[1][2] = a[0];
  out[2][0] = a[1];
  out[2][1] = -a[0];
  out[2][2] = 0.0;
}

void WriteBlock(std::array<double, kSpatialDof * kSpatialDof>& coefficients,
                int row0, int col0,
                const double block[kInertiaDof][kInertiaDof]) {
  for (int row = 0; row < kInertiaDof; ++row) {
    for (int col = 0; col < kInertiaDof; ++col) {
      coefficients[(row0 + row) * kSpatialDof + (col0 + col)] =
          PositiveZero(block[row][col]);
    }
  }
}

// p1 and p2 are the linear and angular blocks of M_A ν. Rigid-body mass
// is not included.
void AddedMassMomentum(
    const std::array<double, kSpatialDof * kSpatialDof>& added_mass,
    const std::array<double, kSpatialDof>& twist, double linear[kInertiaDof],
    double angular[kInertiaDof]) {
  for (int row = 0; row < kInertiaDof; ++row) {
    double linear_sum = 0.0;
    double angular_sum = 0.0;
    for (int col = 0; col < kSpatialDof; ++col) {
      const double component = twist[col];
      linear_sum += added_mass[row * kSpatialDof + col] * component;
      angular_sum +=
          added_mass[(kInertiaDof + row) * kSpatialDof + col] * component;
    }
    linear[row] = linear_sum;
    angular[row] = angular_sum;
  }
}

}  // namespace

StatusOr<AddedMassCoriolisMatrix> ComputeAddedMassCoriolisMatrix(
    const parameters::AddedMass& added_mass,
    const std::array<double, kSpatialDof>& body_twist) {
  const StatusOr<AddedMassMatrix> mass = ComputeAddedMassMatrix(added_mass);
  if (!mass.ok()) {
    return StatusOr<AddedMassCoriolisMatrix>::Failure(mass.status());
  }

  for (double component : body_twist) {
    if (!std::isfinite(component)) {
      return Reject(kBodyTwistNonFiniteMessage);
    }
  }

  double linear[kInertiaDof] = {};
  double angular[kInertiaDof] = {};
  AddedMassMomentum(mass.value().coefficients, body_twist, linear, angular);

  double neg_linear[kInertiaDof][kInertiaDof] = {};
  double neg_angular[kInertiaDof][kInertiaDof] = {};
  NegatedSkew(linear, neg_linear);
  NegatedSkew(angular, neg_angular);

  // Upper-left 3x3 stays zero. C12 and C21 are both -S(p1). C22 is -S(p2).
  AddedMassCoriolisMatrix matrix;
  WriteBlock(matrix.coefficients, 0, kInertiaDof, neg_linear);
  WriteBlock(matrix.coefficients, kInertiaDof, 0, neg_linear);
  WriteBlock(matrix.coefficients, kInertiaDof, kInertiaDof, neg_angular);

  for (double entry : matrix.coefficients) {
    if (!std::isfinite(entry)) {
      return Reject(kCoriolisMatrixNonFiniteMessage);
    }
  }
  return StatusOr<AddedMassCoriolisMatrix>::Ok(matrix);
}

}  // namespace intrinsic::vehicle::dynamics
