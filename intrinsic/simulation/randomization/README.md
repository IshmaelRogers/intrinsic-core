# Seeded marine parameter sampler

Opt-in, plain C++ sampler that draws uniform values for a fixed set of
existing marine simulation parameters from a recorded seed. It adds no
protos, no simulator plugin, and no dynamics. Nothing runs unless a caller
builds a `MarineRandomizationConfig` and calls `SampleMarineParameters`.
`.github/baseline/manipulator_targets.tsv` does not list these targets.
Manipulator code is unchanged.

## Randomizable fields

Each field takes optional closed bounds `[lo, hi]`. A field without bounds
keeps its nominal value and is not sampled. A default config returns the
nominal parameters.

| Config field | Existing field |
| --- | --- |
| `mass_kg` | `MarineModel.mass_inertia.mass_kg` |
| `displaced_volume_m3` | `MarineModel.buoyancy.displaced_volume_m3` |
| `linear_damping_diagonal[6]` | `damping.linear_coefficients[i][i]` (off-diagonals stay nominal) |
| `quadratic_damping[6]` | `damping.quadratic_coefficients[i]` |
| `thruster_efficiency` | `ThrusterGeometry.efficiency`, one independent draw per thruster |
| `current_velocity_m_s[3]` | `environment.current_velocity_m_s` and `CurrentFieldView.velocity_m_s` |
| `imu_bias[6]` | the six `FakeImuConfig` bias fields |

## Random numbers

Draws reuse the SplitMix64 / 53-bit-fraction mix of `ImuSignedUnitNoise`
(`fake_imu.h`). There is no `absl::BitGen` and no `std::` distribution.
Draw `k` of a seed is `ImuSignedUnitNoise(seed + k * 0x9E3779B97F4A7C15)`,
which is the standard SplitMix64 sequence. Each field owns a fixed `k`
(`kMarineSlot*`), so enabling a bound on one field never changes another.
A value is `std::fma(hi - lo, u, lo)` with `u` in `[0, 1)`, a single
rounding that no compiler contraction setting can change; `lo == hi` returns
`lo` exactly.

## Validation

`ValidateMarineRandomization` returns `InvalidArgument` and never clamps for:
non-finite bounds, `lo > hi`, `hi - lo` overflow, an invalid nominal model,
or a model that fails `ValidateMarineModel` with every bounded field at its
`lo` corner or at its `hi` corner. Raising a diagonal entry of a
positive-definite damping matrix keeps it positive definite, so the corner
check covers every realized value.

## Metadata dump

`FormatMarineMetadata` writes the realized values as canonical text: a
version line, `seed=`, then `path=value` lines in fixed order, doubles as
`%.17g`. The golden fixture for seed 42 is
`testdata/marine_parameter_sampler_seed_42.golden.txt`.

## Known limitations

A bias-bearing `FakeImuConfig` yields a DEGRADED measurement by design, so
`AssessImu` reports no error but `accepted` is false. Off-diagonal damping,
inertia, added mass, centers, and thrust bounds are not randomized.
