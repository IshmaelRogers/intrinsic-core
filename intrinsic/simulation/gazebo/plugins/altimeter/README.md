# Altimeter simulator adapter

Opt-in Gazebo **sensor** system plugin (`intrinsic::simulation::Altimeter`) and
a simulator-free adapter (`AltimeterAdapter`). Each simulation step the adapter
reads the target link's world pose and returns
`std::optional<intrinsic_proto::hardware::marine::AltimeterMeasurement>`, the
#78 HAL type, by calling the existing `FakeAltimeter::Measure`.

Sim and test only. UUV only. C++ only (the #122 hydrodynamics plugin and the
#124 DVL plugin have no Python mirror either). There is no gz-transport topic:
the output is the adapter return value, and `Altimeter::LastResultForTesting()`
for tests. A publication channel is a later decision.
`.github/baseline/manipulator_targets.tsv` does not list these targets, and
manipulator code is unchanged.

## Files

| File | Role |
| --- | --- |
| `altimeter_adapter.{h,cc}` | Pure adapter, no gz-sim include. SDF parsing, truth, fault class, dropout, latency. |
| `altimeter_plugin.{h,cc}` | Gazebo system. Copies the link world pose, calls `Step`. |
| `altimeter_plugin_register.cc` | `GZ_ADD_STATIC_PLUGIN_ALIAS(..., "intrinsic::simulation::Altimeter")`. |
| `altimeter_adapter_test.cc` | Fixtures A1-A9 and the scope guard. |
| `altimeter_plugin_test.cc` | Fixtures P1-P4, headless gz. |
| `testdata/altimeter_adapter_seed_42.golden.txt` | Checked-in seed-42 trace. Tests read it and never rewrite it. |

## Reused, not redefined

`AltimeterMeasurement`, `FakeAltimeter`, `FakeAltimeterConfig`,
`AltimeterTruth`, `kCanonicalMinRangeM`, `kCanonicalMaxRangeM`,
`kCanonicalOutOfRangeRangeM`, `AltimeterSignedUnitNoise` (`fake_altimeter.h`);
`AssessAltimeter` and `AltimeterMeasurementView` (`altimeter_policy.h`) as the
conformance oracle in tests; `kBodyFrameId`. `FakeAltimeter` writes every field
of the measurement (health state, covariance, sources, frame, clock domain,
stamps) and draws the noise itself. The adapter's only edit to a produced
message is the receive stamp (see Latency).

The SDF parse helpers (`std::from_chars` based), the time split, the status
shape, the latency FIFO, dropout windows, and the plugin shape are copied from
`plugins/dvl/` and not shared with it. The time split is named
`SplitAltimeterSimTimeNanos` because `SplitSimTimeNanos` already exists in this
namespace in the DVL package and both are linked into the same plugin set.

## Plugin parameters

| Element | Rule |
| --- | --- |
| `<link_name>` | Optional. Default is the canonical link. |
| `<seabed_z_m>` | **Required.** Finite. Flat seabed plane at world-ENU z (same meaning as the DVL plugin). |
| `<noise_amplitude_m>` | Optional, finite, `>= 0`. Default 0. |
| `<latency_steps>` | Optional integer `0..1000`. Default 0. |
| `<dropout_window><start_step/><end_step/></dropout_window>` | 0 to 64 windows. Half-open `[start, end)` over the origination step, `0 <= start <= end`. |
| `<seed>` | Optional uint64. Default 42, the `FakeAltimeter` default. |

The range bounds are **fixed** at the fake's canonical `min = 0.5 m` and
`max = 100 m` and are not configurable in this leaf, so the fake's canonical
out-of-range output is self-consistent. Configurable bounds are a later senior
decision.

Every numeric value must parse completely with `std::from_chars` (locale-free),
be finite, and satisfy its bound. A violation is
`AltimeterSimErrorCode::kInvalidConfig`. Values are never clamped and a bad
value is never replaced by a default. Unknown child elements are ignored.
Invalid configuration yields one de-duplicated `LOG(ERROR)` from the plugin and
no output on any step.

## Ground truth

The measurement point is the link origin with a zero lever arm. The frame is
`body` (REP-103 FLU), so `AltimeterTruth.frame_id = "body"`, overriding the
fake's `sensor` default. The Gazebo world is ENU, so world z is up.

* **Altitude:** `a = z_link - seabed_z_m` in meters, vertical (the DVL altitude
  definition, not along a tilted beam).
  `AltimeterTruth{frame "body", source = t_k, range = a, beam "down", min 0.5,
  max 100, has_return true, quality 0.75, covariance present}`.
* `no_return = a < 0.5` (the blanking zone, including at or below the seabed,
  `a <= 0`).
* `out_of_range = a > 100`.
* The two are mutually exclusive by construction, so the fake's priority
  (dropout, then no-return, then out-of-range, then noise) is never contested.
  Exactly `a == 0.5` and `a == 100` are valid returns (closed interval).

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
2. **Fault class.** `no_return` or `out_of_range` as above.
3. **Noise.** Drawn inside the fake as
   `amp * AltimeterSignedUnitNoise(seed + k)`. The adapter passes `seed + k`
   and `amp`, and makes no second draw. The draw is keyed by `k` and
   stateless, so dropout, faults, and latency never shift a later draw.
4. **Dropout.** If `k` is in a window, the fake is not called and an empty
   slot is enqueued.
5. **Produce.** `FakeAltimeterConfig{seed + k, amp, delay = 0, dropout =
   false, no_return, out_of_range}`.
6. **Latency.** A FIFO of depth `latency_steps`. Step `k` emits the item that
   originated at `k - latency_steps`, if one exists. Nothing is invented at
   startup. The receive time is set to `t_k` at emission, so
   `receive - source = latency_steps * dt` for a fixed `dt`. `receive >=
   source` always, so `kTimeReversal` cannot occur.

`header.sequence` is `seed + k` for the origination step.

## Validity semantics

| Case | Result |
| --- | --- |
| Nominal | `VALID`, `kNone`, accepted, covariance slot 0 = 0.25 only. |
| Noise | `DEGRADED`, `kNone`, not accepted. The fake sets `DEGRADED` whenever the amplitude is not 0, so the adapter does not edit the state. A noisy range below 0 or outside the bounds is **not repaired**: the oracle then reports `kRange` or `kBounds`. |
| No return | `has_return = false`, `range_m` unset, bounds present, `INVALID`, `kNone`, state `kInvalid`, not accepted. Noise is not applied. |
| Out of range | `range_m = 101` (the fake's sentinel, not truth), min 0.5, max 100, `has_return = true`, `INVALID`, so `AssessAltimeter` returns `kBounds`, not accepted. Noise is not applied. |
| Latency | Judgment unchanged. The age shows in the stamps. |
| Dropout | No message. Absent is not an error. |

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

The golden trace scenario: 20 steps, `latency_steps = 1`,
`noise_amplitude_m = 0.05`, dropout `[8, 10)`, seabed `-50`,
`z_k = -49.8 + 6 k`, `dt = 10 ms`, identity attitude. Origination step 0 is a
no-return, steps 1 to 16 are valid returns (degraded by noise), and steps 17 to
19 are out of range (the last is emitted after the 20 steps). A change to the
golden needs senior review.

## Known limitations

* Vertical range only. No beam geometry, ray casting, bathymetry, or lever arm.
* Noise is uniform on `[-a, a)`, not Gaussian.
* Covariance is the fake's fixed 0.25 m^2 range variance. It is not derived
  from the noise amplitude.
* The out-of-range range is the sentinel 101 m, not a measured value.
* The seabed plane must be kept consistent with other plugins (the DVL plugin's
  `<seabed_z_m>`, the depth plugin's surface) by the world author. The plugins
  do not share state.
* Noise relies on the fake's non-fma multiply-add, `a + amp * u`. The golden is
  pinned under `-c opt` on CI's toolchain.
* The altimeter seed shares a space with the #123 sampler and DVL seeds. Use
  distinct seeds.
