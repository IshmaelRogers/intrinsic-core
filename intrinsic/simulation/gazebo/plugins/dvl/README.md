# DVL simulator adapter

Opt-in Gazebo **sensor** system plugin (`intrinsic::simulation::Dvl`) and a
simulator-free adapter (`DvlAdapter`). Each simulation step the adapter reads
the target link's kinematics and returns
`std::optional<intrinsic_proto::hardware::marine::DvlMeasurement>`, the #76
HAL type, by calling the existing `FakeDvl::Measure`.

Sim and test only. UUV only. C++ only (the #122 hydrodynamics plugin has no
Python mirror either). There is no gz-transport topic: the output is the
adapter return value, and `Dvl::last_result()` for tests. A publication
channel is a later decision. `.github/baseline/manipulator_targets.tsv` does
not list these targets, and manipulator code is unchanged.

## Files

| File | Role |
| --- | --- |
| `dvl_adapter.{h,cc}` | Pure adapter, no gz-sim include. SDF parsing, truth, lock, noise, dropout, latency. |
| `dvl_plugin.{h,cc}` | Gazebo system. Copies pose and `Link::WorldLinearVelocity`, calls `Step`. |
| `dvl_plugin_register.cc` | `GZ_ADD_STATIC_PLUGIN_ALIAS(..., "intrinsic::simulation::Dvl")`. |
| `dvl_adapter_test.cc` | Fixtures 1-10 and the scope guard. |
| `dvl_plugin_test.cc` | Fixture 11, headless gz. |
| `testdata/dvl_adapter_seed_42.golden.txt` | Checked-in seed-42 trace. Tests read it and never rewrite it. |

## Reused, not redefined

`DvlMeasurement`, `FakeDvl`, `FakeDvlConfig`, `DvlTruth`, `AddClockDelay`
(`fake_dvl.h`); `AssessDvl` (`dvl_policy.h`) as the conformance oracle in
tests; `ImuSignedUnitNoise` (`fake_imu.h`); `embodiment::RotateVector` and
`WorldVectorNedToEnu` (`frame_policy.h`); `kBodyFrameId`; the #121
`<current_field>` shape validated with `world::AssessCurrentField` the same
way `HydrodynamicsAdapter::Configure` does. `FakeDvl` writes every field of
the measurement. The adapter's only edits to a produced message are the
receive stamp (see Latency) and the noise-only DEGRADED rule.

## Plugin parameters

| Element | Rule |
| --- | --- |
| `<link_name>` | Optional. Default is the canonical link. |
| `<mode>` | `bottom_track` (default) or `water_track`. Anything else is invalid. |
| `<seabed_z_m>` | **Required.** Finite. Flat seabed plane at world-ENU z. |
| `<min_altitude_m>` | Optional, default `0`. Finite. `0 <= min <= max`. |
| `<max_altitude_m>` | **Required.** Finite, `> 0`. |
| `<current_field>` | #121 shape. **Required for `water_track`**, validated whenever present, used only by water track. |
| `<bias_m_s>` | Optional `"x y z"`, finite. Default 0. |
| `<noise_amplitude_m_s>` | Optional, finite, `>= 0`. Default 0. |
| `<latency_steps>` | Optional integer `0..1000`. Default 0. |
| `<dropout_window><start_step/><end_step/></dropout_window>` | 0 to 64 windows. Half-open `[start, end)` over the origination step, `0 <= start <= end`. |
| `<seed>` | Optional uint64. Default 42, the `FakeDvl` default. |

Invalid configuration yields `DvlSimErrorCode::kInvalidConfig`, one
de-duplicated `LOG(ERROR)` from the plugin, and no output on any step.

## Ground truth

The measurement point is the link origin with a zero lever arm. The sensor
frame is `body` (REP-103 FLU), so `DvlTruth.frame_id = "body"`. With `q` the
link world pose (ENU from body) and `v_w` the link world linear velocity:

* **Bottom track:** `v = R(q)^T v_w`, as `RotateVector(conj(q), v_w)`. Velocity
  of the vehicle relative to the seabed (+x forward), the #85 `h(x) = v_body`.
* **Water track:** `v = R(q)^T v_w - c_b`. `world_enu`: `c_b = R(q)^T c`.
  `world_ned`: `c_b = R(q)^T WorldVectorNedToEnu(c)`. `body`: `c_b = c`. This
  is the #86 `h(x) = v_body - R(q)^T c_enu`.
* **Altitude:** `a = z_link_enu - seabed_z_m`, the vertical distance.
  `in_range = min <= a <= max` (closed interval).

## Per-step pipeline

Step index `k` counts non-paused `PostUpdate` calls since Configure or Reset,
starting at 0. `t_k` is `info.simTime` split into int64 seconds and nanos in
`[0, 1e9)`. The order is part of the contract:

1. **Truth.** Unresolved link or missing or non-finite kinematics gives
   `kInvalidState`, enqueues nothing, and emits nothing. `k` still advances.
2. **Lock.** `lock_loss = (mode == bottom_track && !in_range)`. Water track
   never sets it. `altitude_present = in_range`.
3. **Noise.** `u_{k,i} = ImuSignedUnitNoise(seed + (3k + i) * 0x9E3779B97F4A7C15)`
   for `i = 0, 1, 2`, with uint64 wrap, drawn on every step. The effective bias
   is `std::fma(amp, u_{k,i}, b_i)`, or `b_i` on lock-loss.
4. **Dropout.** If `k` is in a window the sample is absent and `FakeDvl` is
   not called.
5. **FakeDvl.** `FakeDvlConfig{seed + k, b', lock_loss, dropout = false}` and
   `DvlTruth{"body", t_k, v, mode, 0.75, in_range, a, covariance}`. If
   `amp != 0`, there is no lock-loss, and the fake returned `VALID` (only
   possible when `b'` is exactly 0), the state becomes `DEGRADED`.
6. **Latency.** A FIFO of depth `latency_steps`. Step `k` emits the item that
   originated at `k - latency_steps`, if one exists. Nothing is invented at
   startup. The emitted sample's receive time is `t_k`, computed with
   `AddClockDelay`, so `receive - source = latency_steps * dt` for a fixed `dt`.

Draws are keyed by origination step, so dropout, lock-loss, latency, and an
invalid-state step never shift a later draw. On lock-loss the sample has mode
bottom track, `bottom_lock = false`, health `INVALID`, no altitude, and
finite velocity (truth plus bias, no noise), so `AssessDvl` reports
`kNone` with state `kInvalid` and never `kNonFinite`. `header.sequence` is
`seed + k` for the origination step.

## Verification

```
bazel test -c opt \
  //intrinsic/simulation/gazebo/plugins/dvl:dvl_adapter_test \
  //intrinsic/simulation/gazebo/plugins/dvl:dvl_plugin_test \
  //intrinsic_hardware/intrinsic/hardware/marine:fake_dvl_test \
  //intrinsic_hardware/intrinsic/hardware/marine:dvl_policy_test \
  //intrinsic/simulation/gazebo/plugins/hydrodynamics:all \
  //intrinsic/simulation/randomization:marine_parameter_sampler_test
```

The golden trace scenario: 20 steps, `latency_steps = 2`, bias `(0.01, 0, 0)`,
`noise_amplitude_m_s = 0.02`, dropout `[5, 7)`, `z_k = -40 + k`,
`max_altitude_m = 25`, seabed `-50`, `min_altitude_m = 0.5`, `dt = 10 ms`,
world velocity `(0.5, -0.25, 0)`, identity attitude. Lock is lost from
origination step 16. A change to the golden needs senior review.

## Known limitations

* Vertical altitude only. No beams, ray casting, bathymetry, or lever arm.
* Noise is uniform on `[-a, a)`, not Gaussian.
* Covariance is the fake's fixed 0.25 m^2/s^2 diagonal. It is not derived from
  the noise amplitude.
* There is no velocity cap, lock reacquisition delay, or automatic
  bottom-to-water fallback.
* The water-track current must be kept equal to the Hydrodynamics plugin's
  `<current_field>` by the world author. The two plugins do not share state.
* The DVL seed shares a space with the #123 sampler seed. Use distinct seeds,
  and not seeds that differ by a multiple of `0x9E3779B97F4A7C15`.
