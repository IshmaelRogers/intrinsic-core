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

// Hand-calculated analytic fixtures for MarineForceDynamics::Evaluate.
//
// The composition under test is the merged helper composition:
//
//   τ_hydro = -C_RB(ν) ν - C_A(ν_r) ν_r + τ_damp(ν_r) + τ_g
//
// with τ_damp = -D(ν_r) ν_r and τ_g the weight-plus-buoyancy body wrench.
// C_RB uses the body twist ν. C_A and damping use the current-relative
// twist ν_r. These fixtures do not form M and do not solve ν̇.
//
// Expected wrenches are derived below from those definitions. They are not
// copied from helper output. Component order is surge, sway, heave, roll,
// pitch, yaw. A zero entry is the documented zero for that component.
//
// Stable output digest of τ_hydro, exact for every value in this file:
//
//   digest = Σ_i (i + 1) τ_hydro[i]
//
// Tolerances are absolute. Zero and restoring cases use 1e-12. The
// Coriolis/damping cases are binary64-exact, so they use 1e-12, which is
// inside the approved 1e-9 bound.

#ifndef INTRINSIC_VEHICLE_DYNAMICS_MARINE_FORCE_ANALYTIC_FIXTURES_H_
#define INTRINSIC_VEHICLE_DYNAMICS_MARINE_FORCE_ANALYTIC_FIXTURES_H_

#include <array>
#include <limits>
#include <string_view>

#include "intrinsic/vehicle/dynamics/vehicle_dynamics.h"

namespace intrinsic::vehicle::dynamics {

// Neutral hull shared by the static and constant-velocity fixtures.
// m = 8 kg, g = 10 m/s^2, so W = m g = 80 N.
// ρ = 1024 kg/m^3, ∇ = 1/128 m^3, so B = ρ g ∇ = 80 N.
// Centers are the body origin unless a fixture overrides them.
// Inertia about the center of gravity is diag(4, 5, 6) kg·m².
// Added mass is diag(8, 6, 4, 1, 1, 2).
// Linear damping is diag(4, 5, 6, 1, 1, 2).
// Quadratic damping is (2, 0, 0, 0, 0, 4).
inline constexpr double kAnalyticNeutralMassKg = 8;
inline constexpr double kAnalyticNeutralGravityMps2 = 10;
inline constexpr double kAnalyticNeutralDensityKgM3 = 1024;
inline constexpr double kAnalyticNeutralVolumeM3 = 1.0 / 128.0;
inline constexpr std::array<double, 3> kAnalyticInertiaDiag = {4, 5, 6};
inline constexpr std::array<double, 6> kAnalyticAddedMassDiag = {8, 6, 4,
                                                                 1, 1, 2};
inline constexpr std::array<double, 6> kAnalyticLinearDampingDiag = {4, 5, 6,
                                                                     1, 1, 2};
inline constexpr std::array<double, 6> kAnalyticQuadraticDamping = {2, 0, 0,
                                                                    0, 0, 4};

// Heavy restoring hull. m = 150 kg, g = 8 m/s^2, W = m g = 1200 N.
// ρ = 1000 kg/m^3, ∇ = 1/8 m^3, B = ρ g ∇ = 1000 N. W − B = 200 N.
inline constexpr double kAnalyticHeavyMassKg = 150;
inline constexpr double kAnalyticHeavyGravityMps2 = 8;
inline constexpr double kAnalyticHeavyDensityKgM3 = 1000;
inline constexpr double kAnalyticHeavyVolumeM3 = 0.125;

inline constexpr double kAnalyticExactTolerance = 1e-12;

// Identity orientation for every fixture. Position is unused by Evaluate.
inline constexpr std::array<double, 4> kAnalyticIdentityQuaternion = {0, 0, 0,
                                                                      1};

struct AnalyticFixture {
  std::string_view name;
  bool expect_ok = true;
  // Empty on success. On failure, the existing DynamicsStatus message.
  std::string_view status_message;

  double mass_kg = kAnalyticNeutralMassKg;
  std::array<double, 3> inertia_diag = kAnalyticInertiaDiag;
  std::array<double, 3> cog_m = {};
  std::array<double, 3> cob_m = {};
  double displaced_volume_m3 = kAnalyticNeutralVolumeM3;
  std::array<double, 6> added_mass_diag = kAnalyticAddedMassDiag;
  std::array<double, 6> damping_linear_diag = kAnalyticLinearDampingDiag;
  std::array<double, 6> damping_quadratic = kAnalyticQuadraticDamping;

  FrameId pose_frame = FrameId::kWorldEnu;
  std::array<double, 6> body_twist = {};
  double dt_s = 0;

  double gravity_m_s2 = kAnalyticNeutralGravityMps2;
  double fluid_density_kg_m3 = kAnalyticNeutralDensityKgM3;
  std::array<double, 3> current_velocity_m_s = {};
  FrameId current_frame = FrameId::kBody;

  std::array<double, 3> input_force_n = {};
  std::array<double, 3> input_torque_n_m = {};

  std::array<double, 6> relative_twist = {};
  std::array<double, 6> rigid_body_coriolis = {};
  std::array<double, 6> added_mass_coriolis = {};
  std::array<double, 6> damping = {};
  std::array<double, 6> restoring = {};
  std::array<double, 6> hydrodynamic = {};
  std::array<double, 6> total = {};
  double tolerance = kAnalyticExactTolerance;
  double hydrodynamic_digest = 0;
  // ν_r · τ_damp. Zero when the relative twist or the damping wrench is zero.
  double damping_power = 0;

  // Constant-velocity witness. The sway rigid-body term must match -m u r
  // and must not match the value obtained by substituting ν_r. Added-mass
  // sway and damping surge must match ν_r and must not match ν.
  bool witness_twist_split = false;
  double rigid_sway_if_nu_r = 0;
  double added_sway_if_nu = 0;
  double damping_surge_if_nu = 0;
};

// static_neutral
// Identity pose, ν = 0, current = 0, τ_input = 0, W = B = 80 N.
// Every Coriolis product, damping product, and restoring wrench is zero,
// so τ_hydro = 0. Digest 0.
//
// restoring_heave_enu / restoring_heave_ned
// Identity orientation, ν = 0, current = 0, τ_input = 0.
// W = 1200 N, B = 1000 N, W − B = 200 N. Centers at the origin, so torque
// is zero. ENU down is −z, so the heave force is −(W − B) = −200 N.
// NED down is +z, so the heave force is +(W − B) = +200 N.
// Coriolis and damping stay zero. Digest is 3 * heave: −600 and +600.
//
// constant_velocity_coriolis_damping
// Centers at the origin. Body-frame current (0.5, 0, 0) m/s.
// ν = (2, 1, 0, 0, 0, 0.5), so ν_r = (1.5, 1, 0, 0, 0, 0.5).
// W = B, so τ_g = 0.
// With r_g = 0, p1 = m ν1 and p2 = I ν2. The body term is
//   −C_RB(ν) ν = (p1 × ν2, p1 × ν1 + p2 × ν2).
// p1 × ν1 = 0 and p2 is parallel to ν2, so the torque is zero.
// p1 × ν2 = (m v r, −m u r, 0) = (4, −8, 0).
// Substituting ν_r would give sway −m u_r r = −6, which is rejected.
// Added mass p1 = (8*1.5, 6*1, 0) = (12, 6, 0), p2 = (0, 0, 1).
//   −C_A(ν_r) ν_r = ((3, −6, 0), (0, 0, 3)).
// Substituting ν would give sway −8 and yaw torque 4, which is rejected.
// τ_damp = −(D_L + D_Q(|ν_r|)) ν_r
//   surge −(4*1.5 + 2*1.5*1.5) = −10.5
//   sway −(5*1) = −5
//   yaw −(2*0.5 + 4*0.5*0.5) = −2
// Substituting ν would give surge −16, which is rejected.
// τ_hydro = (−3.5, −19, 0, 0, 0, 1). Digest −35.5.
// ν_r · τ_damp = −21.75.
//
// reversed_velocity_odd_terms
// ν and the body current are negated, so ν_r is negated.
// −C(−ν)(−ν) = −C(ν)ν, so both Coriolis wrenches stay put.
// Damping is odd, so τ_damp changes sign.
// τ_hydro = (17.5, −9, 0, 0, 0, 5). Digest 29.5.
// ν_r · τ_damp stays −21.75.
//
// axis_surge
// ν = (1, 0, 0, 0, 0, 0), current = 0. Both Coriolis wrenches are zero
// because ν2 = 0 and p1 is parallel to ν1. τ_damp surge = −(4 + 2) = −6.
// τ_input surge = 2, so τ_total surge = −4. Digest −6. Power −6.
//
// axis_yaw
// ν = (0, 0, 0, 0, 0, 1), current = 0. p1 = 0 and p2 is parallel to ν2,
// so both Coriolis wrenches are zero. τ_damp yaw = −(2 + 4) = −6.
// Digest −36. Power −6.
//
// offset_gravity_enu / offset_gravity_ned
// Same heavy hull, ν = 0, current = 0. r_g = (0, 0.25, 0) m, r_b = 0.
// ENU: f_W = (0, 0, −1200) N, f_B = (0, 0, 1000) N.
// Force = (0, 0, −200) N.
// τ = r_g × f_W = (0.25*(−1200), 0, 0) = (−300, 0, 0) N·m.
// Positive-y center of gravity and ENU weight (−z) produce negative roll.
// NED flips both the heave force and the roll torque.
// Digests −1800 and +1800.
//
// offset_buoyancy_enu
// r_g = 0, r_b = (0.25, 0, 0) m. ENU f_B = (0, 0, 1000) N.
// τ = r_b × f_B = (0, −0.25*1000, 0) = (0, −250, 0) N·m.
// A forward center of buoyancy and upward buoyancy produce negative pitch.
// Force heave remains −200 N. Digest −1850.
//
// Failure fixtures reuse the constant-velocity inputs and one defect.
// ValidateEvaluationInputs rejects them before any term is formed.
// Status text is the existing contract. Results are finite zeros, and a
// non-finite time step is not echoed.
inline constexpr AnalyticFixture kMarineForceAnalyticFixtures[] = {
    {
        .name = "static_neutral",
        .dt_s = 0,
    },
    {
        .name = "restoring_heave_enu",
        .mass_kg = kAnalyticHeavyMassKg,
        .displaced_volume_m3 = kAnalyticHeavyVolumeM3,
        .dt_s = 0.5,
        .gravity_m_s2 = kAnalyticHeavyGravityMps2,
        .fluid_density_kg_m3 = kAnalyticHeavyDensityKgM3,
        .restoring = {0, 0, -200, 0, 0, 0},
        .hydrodynamic = {0, 0, -200, 0, 0, 0},
        .total = {0, 0, -200, 0, 0, 0},
        .hydrodynamic_digest = -600,
    },
    {
        .name = "restoring_heave_ned",
        .mass_kg = kAnalyticHeavyMassKg,
        .displaced_volume_m3 = kAnalyticHeavyVolumeM3,
        .pose_frame = FrameId::kWorldNed,
        .dt_s = 0.5,
        .gravity_m_s2 = kAnalyticHeavyGravityMps2,
        .fluid_density_kg_m3 = kAnalyticHeavyDensityKgM3,
        .restoring = {0, 0, 200, 0, 0, 0},
        .hydrodynamic = {0, 0, 200, 0, 0, 0},
        .total = {0, 0, 200, 0, 0, 0},
        .hydrodynamic_digest = 600,
    },
    {
        .name = "constant_velocity_coriolis_damping",
        .body_twist = {2, 1, 0, 0, 0, 0.5},
        .dt_s = 0.25,
        .current_velocity_m_s = {0.5, 0, 0},
        .relative_twist = {1.5, 1, 0, 0, 0, 0.5},
        .rigid_body_coriolis = {4, -8, 0, 0, 0, 0},
        .added_mass_coriolis = {3, -6, 0, 0, 0, 3},
        .damping = {-10.5, -5, 0, 0, 0, -2},
        .hydrodynamic = {-3.5, -19, 0, 0, 0, 1},
        .total = {-3.5, -19, 0, 0, 0, 1},
        .hydrodynamic_digest = -35.5,
        .damping_power = -21.75,
        .witness_twist_split = true,
        .rigid_sway_if_nu_r = -6,
        .added_sway_if_nu = -8,
        .damping_surge_if_nu = -16,
    },
    {
        .name = "reversed_velocity_odd_terms",
        .body_twist = {-2, -1, 0, 0, 0, -0.5},
        .dt_s = 0.25,
        .current_velocity_m_s = {-0.5, 0, 0},
        .relative_twist = {-1.5, -1, 0, 0, 0, -0.5},
        .rigid_body_coriolis = {4, -8, 0, 0, 0, 0},
        .added_mass_coriolis = {3, -6, 0, 0, 0, 3},
        .damping = {10.5, 5, 0, 0, 0, 2},
        .hydrodynamic = {17.5, -9, 0, 0, 0, 5},
        .total = {17.5, -9, 0, 0, 0, 5},
        .hydrodynamic_digest = 29.5,
        .damping_power = -21.75,
    },
    {
        .name = "axis_surge",
        .body_twist = {1, 0, 0, 0, 0, 0},
        .dt_s = 1,
        .input_force_n = {2, 0, 0},
        .relative_twist = {1, 0, 0, 0, 0, 0},
        .damping = {-6, 0, 0, 0, 0, 0},
        .hydrodynamic = {-6, 0, 0, 0, 0, 0},
        .total = {-4, 0, 0, 0, 0, 0},
        .hydrodynamic_digest = -6,
        .damping_power = -6,
    },
    {
        .name = "axis_yaw",
        .body_twist = {0, 0, 0, 0, 0, 1},
        .dt_s = 1,
        .relative_twist = {0, 0, 0, 0, 0, 1},
        .damping = {0, 0, 0, 0, 0, -6},
        .hydrodynamic = {0, 0, 0, 0, 0, -6},
        .total = {0, 0, 0, 0, 0, -6},
        .hydrodynamic_digest = -36,
        .damping_power = -6,
    },
    {
        .name = "offset_gravity_enu",
        .mass_kg = kAnalyticHeavyMassKg,
        .cog_m = {0, 0.25, 0},
        .displaced_volume_m3 = kAnalyticHeavyVolumeM3,
        .dt_s = 0.125,
        .gravity_m_s2 = kAnalyticHeavyGravityMps2,
        .fluid_density_kg_m3 = kAnalyticHeavyDensityKgM3,
        .restoring = {0, 0, -200, -300, 0, 0},
        .hydrodynamic = {0, 0, -200, -300, 0, 0},
        .total = {0, 0, -200, -300, 0, 0},
        .hydrodynamic_digest = -1800,
    },
    {
        .name = "offset_gravity_ned",
        .mass_kg = kAnalyticHeavyMassKg,
        .cog_m = {0, 0.25, 0},
        .displaced_volume_m3 = kAnalyticHeavyVolumeM3,
        .pose_frame = FrameId::kWorldNed,
        .dt_s = 0.125,
        .gravity_m_s2 = kAnalyticHeavyGravityMps2,
        .fluid_density_kg_m3 = kAnalyticHeavyDensityKgM3,
        .restoring = {0, 0, 200, 300, 0, 0},
        .hydrodynamic = {0, 0, 200, 300, 0, 0},
        .total = {0, 0, 200, 300, 0, 0},
        .hydrodynamic_digest = 1800,
    },
    {
        .name = "offset_buoyancy_enu",
        .mass_kg = kAnalyticHeavyMassKg,
        .cob_m = {0.25, 0, 0},
        .displaced_volume_m3 = kAnalyticHeavyVolumeM3,
        .dt_s = 0.125,
        .gravity_m_s2 = kAnalyticHeavyGravityMps2,
        .fluid_density_kg_m3 = kAnalyticHeavyDensityKgM3,
        .restoring = {0, 0, -200, 0, -250, 0},
        .hydrodynamic = {0, 0, -200, 0, -250, 0},
        .total = {0, 0, -200, 0, -250, 0},
        .hydrodynamic_digest = -1850,
    },
    {
        .name = "nonfinite_body_twist",
        .expect_ok = false,
        .status_message = "state value must be finite",
        .body_twist = {std::numeric_limits<double>::quiet_NaN(), 1, 0, 0, 0,
                       0.5},
        .dt_s = 0.25,
        .current_velocity_m_s = {0.5, 0, 0},
    },
    {
        .name = "nonfinite_time_step",
        .expect_ok = false,
        .status_message =
            "time step must be finite and greater than or equal to zero",
        .body_twist = {2, 1, 0, 0, 0, 0.5},
        .dt_s = std::numeric_limits<double>::quiet_NaN(),
        .current_velocity_m_s = {0.5, 0, 0},
    },
    {
        .name = "negative_time_step",
        .expect_ok = false,
        .status_message =
            "time step must be finite and greater than or equal to zero",
        .body_twist = {2, 1, 0, 0, 0, 0.5},
        .dt_s = -1,
        .current_velocity_m_s = {0.5, 0, 0},
    },
    {
        .name = "invalid_pose_frame",
        .expect_ok = false,
        .status_message = "state pose frame must be world_enu or world_ned",
        .pose_frame = FrameId::kBody,
        .body_twist = {2, 1, 0, 0, 0, 0.5},
        .dt_s = 0.25,
        .current_velocity_m_s = {0.5, 0, 0},
    },
    {
        .name = "invalid_current_frame",
        .expect_ok = false,
        .status_message =
            "environment current frame must be world_enu, world_ned, or body",
        .body_twist = {2, 1, 0, 0, 0, 0.5},
        .dt_s = 0.25,
        .current_velocity_m_s = {0.5, 0, 0},
        .current_frame = FrameId::kUnspecified,
    },
};

}  // namespace intrinsic::vehicle::dynamics

#endif  // INTRINSIC_VEHICLE_DYNAMICS_MARINE_FORCE_ANALYTIC_FIXTURES_H_
