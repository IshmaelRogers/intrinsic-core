# Surface-fix simulator adapter

Opt-in Gazebo **sensor** system plugin (`intrinsic::simulation::SurfaceFix`)
and a simulator-free adapter (`SurfaceFixAdapter`). Each simulation step the
adapter reads the target link's world pose and returns
`std::optional<intrinsic_proto::hardware::marine::SurfaceFix>`, the #80 HAL
type, by calling the existing `FakeSurfaceFix::Measure`.

Sim and test only. UUV only. C++ only (the #122 hydrodynamics plugin and the
#124 DVL plugin have no Python mirror either). There is no gz-transport topic:
the output is the adapter return value, and `SurfaceFix::LastResultForTesting()`
for tests. A publication channel is a later decision.
`.github/baseline/manipulator_targets.tsv` does not list these targets, and
manipulator code is unchanged.

## Files

| File | Role |
| --- | --- |
| `surface_fix_adapter.{h,cc}` | Pure adapter, no gz-sim include. SDF parsing, truth, surface gate, noise, dropout, latency. |
| `surface_fix_plugin.{h,cc}` | Gazebo system. Copies the link world pose, calls `Step`. |
| `surface_fix_plugin_register.cc` | `GZ_ADD_STATIC_PLUGIN_ALIAS(..., "intrinsic::simulation::SurfaceFix")`. |
| `surface_fix_adapter_test.cc` | Fixtures S1-S9 and the scope guard. |
| `surface_fix_plugin_test.cc` | Fixtures P1-P4, headless gz. |
| `testdata/surface_fix_adapter_seed_42.golden.txt` | Checked-in seed-42 trace. Tests read it and never rewrite it. |

## Reused, not redefined

`SurfaceFix`, `FakeSurfaceFix`, `FakeSurfaceFixConfig`, `SurfaceFixTruth`
(`fake_surface_fix.h`); `AssessSurfaceFix` and `SurfaceFixView`
(`surface_fix_policy.h`) as the conformance oracle in tests;
`ImuSignedUnitNoise` (`fake_imu.h`); `embodiment::WorldFrameId` and
`WorldFrame::kEnu` (`frame_policy.h`). `FakeSurfaceFix` writes every field of
the measurement (health state, covariance, sources, frame, clock domain,
stamps). The adapter's only edits to a produced message are the receive stamp
(see Latency) and the noise-only DEGRADED rule.

The SDF parse helpers (`std::from_chars` based), the status shape, the latency
FIFO, dropout windows, and the plugin shape are copied from `plugins/depth/`
and `plugins/dvl/` and not shared with them. The time split is named
`SplitSurfaceFixSimTimeNanos` because `SplitSimTimeNanos` already exists in
this namespace in the DVL package and every sensor package is linked into the
same plugin set. Every other helper is in an anonymous namespace with a
`SurfaceFix` prefix.

## Plugin parameters

| Element | Rule |
| --- | --- |
| `<link_name>` | Optional. Default is the canonical link. |
| `<surface_z_m>` | **Required**, finite. World-ENU z of the flat surface plane. |
| `<max_fix_depth_m>` | **Required**, finite, `>= 0`. Deepest link origin below the plane that still gets a fix. |
| `<position_bias_{x,y,z}_m>` | Optional, finite. Default 0. |
| `<noise_amplitude_m>` | Optional, finite, `>= 0`. Default 0. |
| `<latency_steps>` | Optional integer `0..1000`. Default 0. |
| `<dropout_window><start_step/><end_step/></dropout_window>` | 0 to 64 windows. Half-open `[start, end)` over the origination step, `0 <= start <= end`. |
| `<seed>` | Optional uint64. Default 42, the `FakeSurfaceFix` default. |

`surface_z_m` and `max_fix_depth_m` have no default on purpose: the gate is the
whole point of this sensor, and a silently assumed plane would give a fix at
the wrong depth. Every numeric value must parse completely with
`std::from_chars` (locale-free), be finite, and satisfy its bound. A violation
is `SurfaceFixSimErrorCode::kInvalidConfig`. Values are never clamped and a bad
value is never replaced by a default. Unknown child elements are ignored.
Invalid configuration yields one de-duplicated `LOG(ERROR)` from the plugin and
no output on any step.

## Ground truth

The measurement point is the link origin with a zero lever arm. The position is
the link origin in **world ENU** and the header frame is
`WorldFrameId(WorldFrame::kEnu)` (`world_enu`). Source is `FIX_SOURCE_GNSS`
with 12 satellites. No beacon count, no accuracy, no velocity.

* **No geodetic conversion.** There is no latitude, longitude, WGS84, or NED
  step. Converting a local ENU position to a geodetic fix needs an origin and an
  approved model, neither of which exists here, and `SurfaceFix` carries
  position in the frame named in its header. `world_enu` is the frame a world
  author already uses for every other plugin.
* The orientation is not used by the truth. It must still be a finite unit
  quaternion (`| ||q|| - 1 | <= 1e-6`, never normalised), otherwise the step is
  `kInvalidState`.

## Surface gate

`d = surface_z_m - z_link` (positive is deeper).

* `d > max_fix_depth_m`: no sample is originated. That is an empty slot, the
  same as a dropout, and **not an error**. `d == max_fix_depth_m` emits.
* There is no hysteresis. The gate looks only at this step's pose.
* The noise draws are consumed on every step and are keyed by step index, so a
  gated step never shifts a later draw.

This is a **simulation gate, not a HAL policy**. The HAL's
`surface_fix_policy.h` explicitly does not decide whether the vehicle is
submerged, and `FakeSurfaceFix` does not gate on submersion. The gate lives in
the sim adapter because a sim vehicle that is 30 m deep must not be handed a
GNSS fix. It is a stand-in for physical reality, not a conformance rule.

PDR note for review: the gate is a plain depth threshold on the link origin. It
does not model antenna height, wave action, GNSS acquisition time after
surfacing, or loss of lock after a dive. A vehicle that dips below the limit
for one step loses that one fix and gets the next one immediately on return.
Whether the gate should become a policy in the HAL (with an acquisition delay)
is a design question for the PDR and is not decided here.

## Per-step pipeline

Step index `k` counts non-paused `PostUpdate` calls since Configure or Reset,
starting at 0. `t_k` is `info.simTime` split into int64 seconds and nanos in
`[0, 1e9)`. The order is part of the contract:

1. **Truth.** An unresolved link, a missing or non-finite pose, a non-unit
   quaternion, bad time nanos, or a derived position that is not finite gives
   `kInvalidState`, enqueues an empty slot, and emits nothing. `k` still
   advances. The step also drops the delayed sample that was due at `k`.
2. **Noise.** `u_{k,i} = ImuSignedUnitNoise(seed + (4k + i) *
   0x9E3779B97F4A7C15)` with uint64 wrap. Slots 0-2 are position x, y, z.
   **Slot 3 is reserved and neither drawn nor reused.** The effective bias of
   axis `i` is `b'_i = std::fma(amp, u_{k,i}, bias_i)`. Draws are stateless.
3. **Gate.** As above.
4. **Dropout.** If `k` is in a window, the fake is not called and an empty
   slot is enqueued.
5. **Produce.** `FakeSurfaceFixConfig{seed + k, effective biases, delay 0,
   dropout false, invalid_fix false}`. If `amp != 0` and the fake returned
   `VALID` (only possible when every `b'_i` is exactly 0), the state becomes
   `DEGRADED`.
6. **Latency.** A FIFO of depth `latency_steps`. Step `k` emits the item that
   originated at `k - latency_steps`, if one exists. Nothing is invented at
   startup. The receive time is set to `t_k` at emission, so `receive >=
   source` always and `kTimeReversal` cannot occur.

`header.sequence` is `seed + k` for the origination step.

## Validity semantics

| Case | Result |
| --- | --- |
| Surfaced, no bias or noise | `VALID`, `kNone`, accepted, source GNSS. |
| Bias or noise | `DEGRADED`, `kNone`, **not** accepted. Never rewritten to `INVALID`. |
| Too deep | No message. Absent is not an error. |
| Latency | Judgment unchanged. The age shows in the stamps. |
| Dropout | No message. |
| Bad state | No message and `kInvalidState` from the adapter. |

## Verification

```
bazel test -c opt \
  //intrinsic/simulation/gazebo/plugins/surface_fix:all \
  //intrinsic/simulation/gazebo/plugins/imu:all \
  //intrinsic_hardware/intrinsic/hardware/marine:fake_surface_fix_test \
  //intrinsic_hardware/intrinsic/hardware/marine:surface_fix_policy_test \
  //intrinsic_hardware/intrinsic/hardware/marine:fake_imu_test \
  //intrinsic_hardware/intrinsic/hardware/marine:imu_policy_test \
  //intrinsic/simulation/gazebo/plugins/dvl:all \
  //intrinsic/simulation/gazebo/plugins/depth:all \
  //intrinsic/simulation/gazebo/plugins/altimeter:all \
  //intrinsic/simulation/gazebo/plugins/hydrodynamics:all
```

The golden trace scenario: 20 steps, `surface_z_m = 0`, `max_fix_depth_m =
0.5`, `position_bias_x_m = 0.25`, `position_bias_z_m = -0.02`,
`noise_amplitude_m = 0.04`, `latency_steps = 2`, dropout `[5, 7)`,
`dt = 10 ms`, identity attitude. The link is at `z = 0.125` except for
`z = -0.5` at steps 8 and 9 (`d == max`, still a fix) and `z = -1` at steps 10
to 14 (gated). With latency 2 the emitted trace is absent for steps 0-1 (startup),
7-8 (dropout), and 12-16 (gate). A change to the golden needs senior review.

## Known limitations

* The surface is a flat plane. No wave, tide, or antenna offset.
* Noise is uniform on `[-a, a)`, not Gaussian.
* Covariance is the fake's fixed value. It is not derived from the noise
  amplitude, and no horizontal or vertical accuracy is reported.
* No geodetic output, no acquisition delay, and no loss of lock model.
* The plane and the depth limit must be kept consistent with other plugins (for
  example the depth plugin's `surface_z_m`) by the world author. The plugins do
  not share state.
* The seed shares a space with the #123 sampler, DVL, depth, altimeter, and IMU
  seeds. Use distinct seeds, and not seeds that differ by a multiple of
  `0x9E3779B97F4A7C15`.
