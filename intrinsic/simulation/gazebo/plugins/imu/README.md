# IMU simulator adapter

Opt-in Gazebo **sensor** system plugin (`intrinsic::simulation::Imu`) and a
simulator-free adapter (`ImuAdapter`). Each simulation step the adapter reads
the target link's world pose, world angular velocity, and world linear
acceleration and returns
`std::optional<intrinsic_proto::hardware::marine::ImuMeasurement>`, the #79 HAL
type, by calling the existing `FakeImu::Measure`.

Sim and test only. UUV only. C++ only (the #122 hydrodynamics plugin and the
#124 DVL plugin have no Python mirror either). There is no gz-transport topic:
the output is the adapter return value, and `Imu::LastResultForTesting()` for
tests. A publication channel is a later decision.
`.github/baseline/manipulator_targets.tsv` does not list these targets, and
manipulator code is unchanged.

## Files

| File | Role |
| --- | --- |
| `imu_adapter.{h,cc}` | Pure adapter, no gz-sim include. SDF parsing, truth, noise, drift, dropout, latency. |
| `imu_plugin.{h,cc}` | Gazebo system. Copies pose, angular velocity, and linear acceleration, calls `Step`. |
| `imu_plugin_register.cc` | `GZ_ADD_STATIC_PLUGIN_ALIAS(..., "intrinsic::simulation::Imu")`. |
| `imu_adapter_test.cc` | Fixtures I1-I9 and the scope guard. |
| `imu_plugin_test.cc` | Fixtures P1-P4 and I10, headless gz. |
| `testdata/imu_adapter_seed_42.golden.txt` | Checked-in seed-42 trace. Tests read it and never rewrite it. |

## Reused, not redefined

`ImuMeasurement`, `FakeImu`, `FakeImuConfig`, `ImuTruth`,
`ImuSignedUnitNoise` (`fake_imu.h`); `AssessImu` and `ImuMeasurementView`
(`imu_policy.h`) as the conformance oracle in tests; `kGravityEnu`
(`intrinsic/estimation/eskf_propagate.h`); `embodiment::RotateVector` and
`kBodyFrameId` (`frame_policy.h`). `FakeImu` writes every field of the
measurement (health state, covariance, sources, frame, clock domain, stamps).
The adapter's only edits to a produced message are the receive stamp (see
Latency) and the noise-or-drift DEGRADED rule.

The SDF parse helpers (`std::from_chars` based), the status shape, the latency
FIFO, dropout windows, and the plugin shape are copied from `plugins/depth/`
and `plugins/dvl/` and not shared with them. The time split is named
`SplitImuSimTimeNanos` because `SplitSimTimeNanos` already exists in this
namespace in the DVL package and every sensor package is linked into the same
plugin set. Every other helper is in an anonymous namespace with an `Imu`
prefix.

## Plugin parameters

| Element | Rule |
| --- | --- |
| `<link_name>` | Optional. Default is the canonical link. |
| `<angular_bias_{x,y,z}_rad_s>` | Optional, finite. Default 0. |
| `<linear_bias_{x,y,z}_m_s2>` | Optional, finite. Default 0. |
| `<angular_noise_amplitude_rad_s>` | Optional, finite, `>= 0`. Default 0. |
| `<linear_noise_amplitude_m_s2>` | Optional, finite, `>= 0`. Default 0. |
| `<angular_drift_rad_s2>` | Optional, finite. Default 0. Linear ramp rate, same for all three gyro axes. |
| `<linear_drift_m_s3>` | Optional, finite. Default 0. Linear ramp rate, same for all three accelerometer axes. |
| `<latency_steps>` | Optional integer `0..1000`. Default 0. |
| `<dropout_window><start_step/><end_step/></dropout_window>` | 0 to 64 windows. Half-open `[start, end)` over the origination step, `0 <= start <= end`. |
| `<seed>` | Optional uint64. Default 42, the `FakeImu` default. |

Every numeric value must parse completely with `std::from_chars` (locale-free),
be finite, and satisfy its bound. A violation is
`ImuSimErrorCode::kInvalidConfig`. Values are never clamped and a bad value is
never replaced by a default. Unknown child elements are ignored. Invalid
configuration yields one de-duplicated `LOG(ERROR)` from the plugin and no
output on any step.

## Ground truth

The measurement point is the link origin with a zero lever arm. The body frame
is `body` (REP-103 FLU), so `ImuTruth.frame_id = "body"`. The Gazebo world is
ENU, so world z is up. Let `R` be the rotation of the link orientation `q`
(world-ENU-from-body, Hamilton, stored `x, y, z, w`).

* **Angular velocity:** `omega_b = R^T omega_w`.
* **Specific force:** `f_b = R^T (a_w - kGravityEnu)` with
  `kGravityEnu = (0, 0, -9.80665)`. A link at rest has `a_w = 0` and measures
  `f_b = (0, 0, +9.80665)`. A link in free fall has `a_w = kGravityEnu` and
  measures zero.
* **Orientation:** `q` itself, present, copied bit for bit. The adapter never
  normalises it.
* `ImuTruth{frame "body", source = t_k, rates and specific force as above,
  orientation present, quality 0.75, covariance present}`.

### Where the acceleration comes from

`a_w` is `Link::WorldLinearAcceleration`, which gz-sim fills from the physics
engine. The plugin enables the link's velocity and acceleration checks in
`Configure` and `Reset` (`Link::EnableVelocityChecks`,
`Link::EnableAccelerationChecks`). Those calls create components in the ECM
and are the only writes the plugin makes. Link poses are bit-identical with and
without the plugin (fixture P3). The adapter does not differentiate velocity
itself.

There is no gravity reconstruction beyond the one subtraction above. The
adapter is not told whether the simulated body is subject to gravity. See
[Open numerics question](#open-numerics-question-122-gravity).

## Per-step pipeline

Step index `k` counts non-paused `PostUpdate` calls since Configure or Reset,
starting at 0. `t_k` is `info.simTime` split into int64 seconds and nanos in
`[0, 1e9)`. The order is part of the contract:

1. **Truth.** An unresolved link, missing or non-finite kinematics, a
   non-unit quaternion (`| ||q|| - 1 | <= 1e-6`), bad time nanos, simulation
   time that runs backwards, or a derived body-frame value that is not finite
   gives `kInvalidState`, enqueues an empty slot, and emits nothing. `k` still
   advances. The step also drops the delayed sample that was due at `k`.
2. **Fault class.** None for the IMU.
3. **Noise and drift.** `u_{k,i} = ImuSignedUnitNoise(seed + (8k + i) *
   0x9E3779B97F4A7C15)` with uint64 wrap. Slots 0-2 are the gyro axes, slots
   3-5 the accelerometer axes, **slots 6 and 7 are reserved and neither drawn
   nor reused.** With `e_k` the elapsed simulation time in seconds, the
   effective bias of axis `i` is

       b'_i = std::fma(amp_i, u_{k,i}, std::fma(drift_i, e_k, bias_i))

   Drift and noise are folded into the bias, so `FakeImu` is called with zero
   drift and zero noise. `e_k` is `(t_k - t_0)` in nanoseconds divided by
   `1e9`, where `t_0` is the time of the first step with a valid time field
   since Configure or Reset. Reset restarts the ramp even though simulation
   time keeps counting. Draws are stateless, so dropout and latency never
   shift a later draw.
4. **Dropout.** If `k` is in a window, the fake is not called and an empty
   slot is enqueued.
5. **Produce.** `FakeImuConfig{seed + k, effective biases, drift 0, noise 0,
   delay 0, dropout false}`. If any noise amplitude or drift rate is non-zero
   and the fake returned `VALID` (only possible when every effective bias is
   exactly 0, for example drift at step 0), the state becomes `DEGRADED`.
6. **Latency.** A FIFO of depth `latency_steps`. Step `k` emits the item that
   originated at `k - latency_steps`, if one exists. Nothing is invented at
   startup. The receive time is set to `t_k` at emission. `receive >= source`
   always, so `kTimeReversal` cannot occur.

`header.sequence` is `seed + k` for the origination step.

### Drift time base

Fixture I4 states the ramp as `omega_x(k) = fma(1e-3, k * 0.01, bias)`. For a
10 ms step, `k * 0.01` and `(k * 10'000'000) / 1e9` are different doubles for
some `k`, because `0.01` is not exactly representable. The adapter derives time
from the integer nanosecond clock, which is the only exact quantity it has, so
the test compares bit for bit against `fma(1e-3, double(k * 10'000'000) / 1e9,
bias)` and checks the literal `k * 0.01` form to within `4e-18`.

## Validity semantics

| Case | Result |
| --- | --- |
| Nominal | `VALID`, `kNone`, accepted. |
| Bias, noise, or drift | `DEGRADED`, `kNone`, **not** accepted. Never rewritten to `INVALID`. |
| Latency | Judgment unchanged. The age shows in the stamps. |
| Dropout | No message. Absent is not an error. |
| Bad state | No message and `kInvalidState` from the adapter. |

## Verification

```
bazel test -c opt \
  //intrinsic/simulation/gazebo/plugins/imu:all \
  //intrinsic/simulation/gazebo/plugins/surface_fix:all \
  //intrinsic_hardware/intrinsic/hardware/marine:fake_imu_test \
  //intrinsic_hardware/intrinsic/hardware/marine:imu_policy_test \
  //intrinsic_hardware/intrinsic/hardware/marine:fake_surface_fix_test \
  //intrinsic_hardware/intrinsic/hardware/marine:surface_fix_policy_test \
  //intrinsic/simulation/gazebo/plugins/dvl:all \
  //intrinsic/simulation/gazebo/plugins/depth:all \
  //intrinsic/simulation/gazebo/plugins/altimeter:all \
  //intrinsic/simulation/gazebo/plugins/hydrodynamics:all
```

The golden trace scenario: 20 steps, `latency_steps = 2`,
`angular_bias_x_rad_s = 0.01`, `linear_bias_z_m_s2 = -0.02`, noise amplitudes
`0.02` and `0.04`, drift `1e-3` and `2e-3`, dropout `[5, 7)`, `dt = 10 ms`,
`q = (0.5, 0.5, 0.5, 0.5)` (120 degrees about `(1, 1, 1)`, exactly
representable, so no libm is involved), `omega_w = (0.01 k, -0.005 k, 0.1)`,
`a_w = (0.02 k, 0, -0.01 k)`. A change to the golden needs senior review.

## I10: gravity double-count guard

`imu_plugin_test.cc` drops a box on a ground plane in a real physics world and
checks that a resting link measures `+1 g` on the body z axis: not `0`
(gravity lost) and not `2 g` (gravity counted twice). It repeats the check
with the #122 hydrodynamics plugin attached, and characterises the engine's
acceleration output in free fall.

Observed with `-c opt` and the `bullet-featherstone` physics plugin that this
repository links (`@gz-physics//bullet-featherstone`):

| Scenario | `f_b` observed | Reading |
| --- | --- | --- |
| Box resting on a ground plane | norm `9.80665` (z within `1e-4`, lateral within `1e-4`) | `+1 g`: not `0`, not `2 g`. |
| Same box with `intrinsic::simulation::Hydrodynamics` attached | norm `9.80665` | `+1 g`: neither lost nor doubled. |
| Box in free fall (no ground), z falling | norm `9.80665` | **Wrong for a physical accelerometer, which reads `0`.** |

The first two rows pass for a reason that needs to be stated plainly: the
engine reports `WorldLinearAcceleration = 0` for every link, so the adapter
only ever sees `a_w = 0` and the result is `f_b = R^T (0 - kGravityEnu)`, which
is `+1 g` whether the link rests or falls. The free-fall row records that
limit as a test (`FreeFallCharacterizesTheEnginesAccelerationOutput`). It
fails loudly if an engine starts filling the acceleration, so that the
expectations here are reviewed instead of drifting.

Consequences: the adapter is correct for any engine that reports true link
acceleration (the unit fixture I1 pins the formula, including the free
fall case `a_w = kGravityEnu -> 0`). With bullet-featherstone as shipped, a
vehicle that accelerates or free-falls will not show that in the specific
force. A dynamics-aware source is a decision for the question below.

## Open numerics question (#122 gravity)

The #122 hydrodynamics plugin applies its own buoyancy and weight terms. This
adapter assumes the engine's `WorldLinearAcceleration` is the true coordinate
acceleration of the link, and subtracts `kGravityEnu` once. If an engine reports
zero acceleration for a body (see I10 results) or if a hydrodynamics force
model changes what "at rest" means for a neutrally buoyant vehicle, the
specific force is wrong by up to `1 g`. The adapter cannot detect that from one
sample. This needs a decision from senior numerics before any estimator
consumes the output. It is intentionally not worked around here.

## Known limitations

* Uniform noise on `[-a, a)`, not Gaussian.
* Fixed fake covariance. It is not derived from the noise amplitude.
* Drift is a linear ramp with one rate per sensor triple. No random walk and no
  temperature model.
* Zero lever arm. The IMU is at the link origin, so there is no centripetal or
  Euler term.
* Orientation is the link orientation, not a filtered estimate.
* The seed shares a space with the #123 sampler, DVL, depth, altimeter, and
  surface-fix seeds. Use distinct seeds, and not seeds that differ by a
  multiple of `0x9E3779B97F4A7C15`.
