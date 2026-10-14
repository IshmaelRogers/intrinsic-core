# Depth simulator adapter

Opt-in Gazebo **sensor** system plugin (`intrinsic::simulation::Depth`) and a
simulator-free adapter (`DepthAdapter`). Each simulation step the adapter reads
the target link's world pose and returns
`std::optional<intrinsic_proto::hardware::marine::PressureDepthMeasurement>`,
the #77 HAL type, by calling the existing `FakePressureDepth::Measure`.

Sim and test only. UUV only. C++ only (the #122 hydrodynamics plugin and the
#124 DVL plugin have no Python mirror either). There is no gz-transport topic:
the output is the adapter return value, and `Depth::LastResultForTesting()`
for tests. A publication channel is a later decision.
`.github/baseline/manipulator_targets.tsv` does not list these targets, and
manipulator code is unchanged.

## Files

| File | Role |
| --- | --- |
| `depth_adapter.{h,cc}` | Pure adapter, no gz-sim include. SDF parsing, truth, fault class, noise, dropout, latency. |
| `depth_plugin.{h,cc}` | Gazebo system. Copies the link world pose, calls `Step`. |
| `depth_plugin_register.cc` | `GZ_ADD_STATIC_PLUGIN_ALIAS(..., "intrinsic::simulation::Depth")`. |
| `depth_adapter_test.cc` | Fixtures D1-D9 and the scope guard. |
| `depth_plugin_test.cc` | Fixtures P1-P4, headless gz. |
| `testdata/depth_adapter_seed_42.golden.txt` | Checked-in seed-42 trace. Tests read it and never rewrite it. |

## Reused, not redefined

`PressureDepthMeasurement`, `FakePressureDepth`, `FakePressureDepthConfig`,
`PressureDepthTruth`, `kCanonicalOutOfRangeDepthM` (`fake_pressure_depth.h`);
`AssessPressureDepth` and `PressureDepthMeasurementView`
(`pressure_depth_policy.h`) as the conformance oracle in tests;
`ImuSignedUnitNoise` (`fake_imu.h`); `kBodyFrameId`. `FakePressureDepth`
writes every field of the measurement (health state, covariance, sources,
frame, clock domain, stamps). The adapter's only edits to a produced message
are the receive stamp (see Latency) and the noise-only DEGRADED rule.

The SDF parse helpers (`std::from_chars` based), the time split, the status
shape, the latency FIFO, dropout windows, and the plugin shape are copied from
`plugins/dvl/` and not shared with it. The time split is named
`SplitDepthSimTimeNanos` because `SplitSimTimeNanos` already exists in this
namespace in the DVL package and both are linked into the same plugin set.

## Plugin parameters

| Element | Rule |
| --- | --- |
| `<link_name>` | Optional. Default is the canonical link. |
| `<surface_z_m>` | Optional, finite. Default `0`. Flat free-surface plane at world-ENU z. |
| `<max_depth_m>` | Optional, finite, `> 0`. Unset means no rating limit. |
| `<bias_m>` | Optional, finite. Default 0. |
| `<noise_amplitude_m>` | Optional, finite, `>= 0`. Default 0. |
| `<latency_steps>` | Optional integer `0..1000`. Default 0. |
| `<dropout_window><start_step/><end_step/></dropout_window>` | 0 to 64 windows. Half-open `[start, end)` over the origination step, `0 <= start <= end`. |
| `<seed>` | Optional uint64. Default 42, the `FakePressureDepth` default. |

Every numeric value must parse completely with `std::from_chars` (locale-free),
be finite, and satisfy its bound. A violation is
`DepthSimErrorCode::kInvalidConfig`. Values are never clamped and a bad value
is never replaced by a default. Unknown child elements are ignored. Invalid
configuration yields one de-duplicated `LOG(ERROR)` from the plugin and no
output on any step.

## Ground truth

The measurement point is the link origin with a zero lever arm. The frame is
`body` (REP-103 FLU), so `PressureDepthTruth.frame_id = "body"`, overriding the
fake's `sensor` default. The Gazebo world is ENU, so world z is up.

* **Depth:** `d = surface_z_m - z_link` in meters, positive deeper.
  `PressureDepthTruth{frame "body", source = t_k, pressure absent, depth
  present, provenance present = DIRECT, density absent, quality 0.75,
  covariance present}`.
* **No pressure is synthesized.** Hydrostatic `p = p0 + rho g d` is a new
  equation with no approved source in the repository, and the fake explicitly
  does not integrate hydrostatic pressure. A depth-only `DIRECT` message is a
  legal #77 payload that `AssessPressureDepth` accepts (density is not
  required for `DIRECT`). A pressure output needs senior navigation approval.
* `out_of_range = (d < 0) || (max_depth_m set && d > max_depth_m)`. Exactly
  `d == 0` and `d == max_depth_m` are in range.

The orientation is not used by the truth. It must still be a finite unit
quaternion (`| ||q|| - 1 | <= 1e-6`, never normalised), otherwise the step is
`kInvalidState`. That keeps a future slant-range leaf from silently accepting
bad poses.

## Per-step pipeline

Step index `k` counts non-paused `PostUpdate` calls since Configure or Reset,
starting at 0. `t_k` is `info.simTime` split into int64 seconds and nanos in
`[0, 1e9)`. The order is part of the contract:

1. **Truth.** An unresolved link, a missing or non-finite pose, a non-unit
   quaternion, or bad time nanos gives `kInvalidState`, enqueues an empty slot,
   and emits nothing. `k` still advances. The step also drops the delayed
   sample that was due at `k`, so one bad step means no sample at step `k` and
   none at `k + latency_steps`.
2. **Fault class.** `out_of_range` as above.
3. **Noise.** `u_{k,i} = ImuSignedUnitNoise(seed + (2k + i) *
   0x9E3779B97F4A7C15)` with uint64 wrap. Slot `i = 0` is depth. **Slot
   `i = 1` is reserved for a future pressure channel and is neither drawn nor
   reused.** The effective bias is `b' = std::fma(amp, u_{k,0}, bias_m)`. Draws
   are stateless, so dropout, out-of-range, and latency never shift a later
   draw.
4. **Dropout.** If `k` is in a window, the fake is not called and an empty
   slot is enqueued.
5. **Produce.** `FakePressureDepthConfig{seed + k, depth_bias_m = b',
   delay = 0, dropout = false, out_of_range}`. If `amp != 0`, the sample is
   not out of range, and the fake returned `VALID` (only possible when `b'` is
   exactly 0), the state becomes `DEGRADED`.
6. **Latency.** A FIFO of depth `latency_steps`. Step `k` emits the item that
   originated at `k - latency_steps`, if one exists. Nothing is invented at
   startup. The receive time is set to `t_k` at emission, so
   `receive - source = latency_steps * dt` for a fixed `dt`. `receive >=
   source` always, so `kTimeReversal` cannot occur.

`header.sequence` is `seed + k` for the origination step.

## Validity semantics

| Case | Result |
| --- | --- |
| Nominal | `VALID`, `kNone`, accepted, provenance `kDirect`, covariance slots 0 (1.0) and 7 (0.25) only. |
| Bias or noise | `DEGRADED`, `kNone`, **not** accepted. Never rewritten to `INVALID`. |
| Out of range | The fake writes `depth_m = -1` and `INVALID`, so `AssessPressureDepth` returns `kDepth`, not accepted. This is the HAL's canonical out-of-range. Bias and noise are not visible. |
| Latency | Judgment unchanged. The age shows in the stamps. |
| Dropout | No message. Absent is not an error. |

A noisy or biased depth near the surface can be negative. It is not repaired,
and the oracle reports `kDepth`.

## Verification

```
bazel test -c opt \
  //intrinsic/simulation/gazebo/plugins/depth:all \
  //intrinsic/simulation/gazebo/plugins/altimeter:all \
  //intrinsic_hardware/intrinsic/hardware/marine:fake_pressure_depth_test \
  //intrinsic_hardware/intrinsic/hardware/marine:pressure_depth_policy_test \
  //intrinsic_hardware/intrinsic/hardware/marine:fake_altimeter_test \
  //intrinsic_hardware/intrinsic/hardware/marine:altimeter_policy_test \
  //intrinsic/simulation/gazebo/plugins/dvl:all \
  //intrinsic/simulation/gazebo/plugins/hydrodynamics:all
```

The golden trace scenario: 20 steps, `latency_steps = 2`, `bias_m = 0.01`,
`noise_amplitude_m = 0.02`, dropout `[5, 7)`, `max_depth_m = 4`, surface `0`,
`z_k = -3 + 0.25 k`, `dt = 10 ms`, identity attitude. The depth is in range
for origination steps 0 to 12 (`d == 0` at step 12) and out of range from
step 13. A change to the golden needs senior review.

## Known limitations

* Vertical depth only. No wave, tide, or tilted-sensor correction. The surface
  is a flat plane.
* Noise is uniform on `[-a, a)`, not Gaussian.
* Covariance is the fake's fixed 0.25 m^2 depth variance. It is not derived
  from the noise amplitude.
* No pressure, density, or atmospheric model. `pressure_pa` and
  `fluid_density_kg_m3` are always absent.
* The surface plane must be kept consistent with other plugins (for example the
  altimeter's seabed plane and the hydrodynamics plugin) by the world author.
  The plugins do not share state.
* The depth seed shares a space with the #123 sampler and DVL seeds. Use
  distinct seeds, and not seeds that differ by a multiple of
  `0x9E3779B97F4A7C15`.
