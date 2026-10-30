# IMU simulator adapter

Opt-in Gazebo **sensor** system plugin (`intrinsic::simulation::Imu`) and a
simulator-free adapter (`ImuAdapter`). Each simulation step the adapter reads
the target link's world pose, world angular velocity, and world linear
acceleration (differenced from the link's world linear velocity, see
[Where the acceleration comes from](#where-the-acceleration-comes-from)) and returns
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
| `imu_plugin.{h,cc}` | Gazebo system. Copies pose and angular velocity, differences linear velocity into acceleration, calls `Step`. |
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

`a_w` is differenced from `Link::WorldLinearVelocity` by the plugin. The
engine's `Link::WorldLinearAcceleration` is not used, because
bullet-featherstone never fills it (it stays 0, even in free fall). The
adapter does not differentiate anything itself, and `ImuAdapter` and its API
are unchanged.

The plugin holds the last *fresh* velocity `v_f` and its time `t_f` (cleared
on Configure and Reset). On each non-paused `PostUpdate`:

* **Warm-up** (the first step after Configure or Reset): records `v_f, t_f`
  and sets `kinematics_ready = false`. The adapter reports `kInvalidState`
  with no sample, and `k` still advances. The plugin logs at most one ERROR
  line for it, through the de-duplicated `Report`.
* **Fresh step** (the link's world pose is not bit-identical to the previous
  step's): `a_w = (v_k - v_f) / (t_k - t_f)`, the true average acceleration
  since the last real sample, then `v_f, t_f <- v_k, t_k`. A semi-implicit
  step makes this equal to the applied acceleration. Free fall reads
  `f_b = 0`.
* **Gated step** (pose bit-identical): `a_w = 0`. A body truly at rest reads
  exactly `+g`. The angular velocity is the held engine value.

The pose comparison is exact `==` on every position and quaternion component.
`gz::math::Pose3d::operator==` has a tolerance, and the ECM `ComponentState`
flags depend on system ordering, so neither is used. `Imu::KinematicsFreshness()`
returns `{fresh, steps_since_fresh, gated_steps}` for the latest step. It is a
diagnostic: gated steps log nothing.

The plugin enables the link's velocity and acceleration checks in `Configure`
and `Reset`. Those calls create components in the ECM and are the only writes
the plugin makes. Link poses are bit-identical with and without the plugin
(fixture P3).

#### Known limitation: the 1e-6 pose gate

* **Mechanism.** gz-physics `bullet-featherstone/src/SimulationFeatures.cc`
  `Write(ChangedWorldPoses&)` reports a link only when its world position or
  rotation differs from the last reported one by more than 1e-6. gz-sim
  `Physics.cc` `ChangedLinks` and `UpdateSim` then publish `WorldPose`,
  `WorldLinearVelocity`, `WorldAngularVelocity` and acceleration only for those
  links. The published values are true, just infrequent. A fixed-joint child of
  the link is gated identically, so there is no per-step probe.
* **F1-B output.** On a gated step the plugin emits `a_w = 0`. Slow motion is
  therefore under-read by the gated fraction of the steps, and the next fresh
  step reports the average over the gap.
* **H1 numbers.** A hull accelerating at 0.049 m/s^2 from rest moves 1e-6 m
  per republish only near step 20 (`v dt` reaches about 1e-6 m there). In H1
  steps 1-19 read `gggggFggFgFgFgFgFgF` (`F` fresh, `g` gated) and every step
  from 19 on is fresh. Free fall is fresh every step. A body at rest is gated on almost every step (I10a: 294 of 299), which is exact because its acceleration is 0.
* **Follow-up.** The pose gate also makes the #122 Hydrodynamics plugin
  compute drag from stale velocity. A per-step engine read is a separate
  follow-up. It is not part of this change, and the issue is not filed.

There is no gravity reconstruction beyond the one subtraction above. The
adapter is not told whether the simulated body is subject to gravity.

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

## I10, H1, H2: true acceleration, gravity counted once

`imu_plugin_test.cc` runs real bullet-featherstone physics worlds (1 ms step,
hull mass 32, IMU seed 42, zero noise). The world rule for worlds that run
Physics and load `intrinsic::simulation::Hydrodynamics` (D2-A) is
`<gravity>0 0 0</gravity>`; see `../hydrodynamics/README.md`. Only H1 follows
it. I10b and H2 break it on purpose, as characterizations.

Warm-up (all runs): step 0 gives `kInvalidState`, no sample, at most one ERROR
line.

| Id | World | Measured |
| --- | --- | --- |
| I10a, box resting on ground, 300 steps | gravity, ground | Step 11 (landing) `f_z` = 10.2998, back to 9.80665 by step 12. Steps 20-299: max `\|f_z - g\|` = 1.5e-3. Last-100 mean `f_b` = (1.688e-6, 1.688e-6, 9.80665). 294 gated steps. |
| I10b, same with Hydrodynamics (zero current), 300 steps | gravity, ground | Step 11 `f_z` = 10.2973. Steps 20-299: max `\|f_z - g\|` = 1.5e-3. Last-100 mean `f_b` = (3.464e-6, -3.7e-8, 9.80665). Engine gravity and the plugin's weight both act, the contact carries the excess, so the true acceleration is about 0 and `f_b = (0, 0, g)`. |
| I10c, free fall, 200 steps | gravity, no ground | `\|f\|` = 0.0 on every step after warm-up, z falls 0.5 to 0.3029. No gated step. |
| H1, free hull + Hydrodynamics, 1000 steps | `0 0 0` | Steps 20-999: every step fresh. `a_w,z` within 1.2e-5 and `f_z` within 3.6e-5 of the drag closed form. `v_z(1 s)` = 0.029049 (closed form 0.029040), z 0.5 to 0.51735. Body pitch rate within 8.1e-6 of the restoring-torque model (step 999: -3.3127e-3 against -3.3208e-3). `\|omega_x\|`, `\|omega_z\|` <= 2.0e-8. |
| H2, same hull | `0 0 -9.80665` | Steps 1-50: every step fresh, `a_w,z` from -9.7467 (step 1) to -8.6408 (step 50), within 1.94e-3 of the closed form. This is about -0.995 g: gravity is counted twice. |

The drag closed form is `m dv/dt = F - c_l v - c_q v |v|` with `m = 32`,
`c_l = 35`, `c_q = 90` (heave), `F_H1 = B - W = +1.569064 N` and
`F_H2 = F_H1 - m g = -312.243736 N`, evaluated at the velocity at the start of
each step. The pitch model is `I theta'' + d_l theta' + K theta = tau0` with the
`six_thruster_uuv_example.cc` constants (`d_l = 1.5`, `K = 12.599584`,
`tau0 = -0.03138128`) and `I_yy = 5.4`; the constants and their sources are
named in `imu_plugin_test.cc`.

H1 steps 1-19 are not banded (the pose gate leaves them stale). In the test
they are only checked to be finite.

## Open numerics question (#122 gravity): closed

The question was whether the adapter's single `kGravityEnu` subtraction holds
when the #122 hydrodynamics plugin applies its own buoyancy and weight. It
does, given the true link acceleration. Source of that acceleration: the
differencing above. The remaining rule is the D2-A world rule. Under
bullet-featherstone, which ignores `GravityEnabledCmd`, a world with gravity
on and Hydrodynamics loaded counts gravity twice (H2), so such worlds must set
`<gravity>0 0 0</gravity>`.

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
