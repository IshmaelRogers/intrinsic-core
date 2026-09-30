# intrinsic_estimation

Opt-in persistent estimation service contract (#29, ADR 0001 Phase 3).
`EstimatorService` publishes the authoritative #17
`intrinsic_proto.vehicle.VehicleState`. This package adds no state message, no filter
equations (#81 adds ESKF layout only, see `ESKF_LAYOUT.md`; #82 adds nominal propagation only, see `ESKF_PROPAGATE.md`; #83 adds covariance propagation only, see `ESKF_COV_PROPAGATE.md`; #84 adds a reusable innovation gate only, see `INNOVATION_GATE.md`; #85 adds the DVL bottom-track velocity update only, see `DVL_BOTTOM_TRACK_UPDATE.md`; #86 adds the DVL water-track velocity update only, see `DVL_WATER_TRACK_UPDATE.md`), no ICON wiring,
and no safety authority (#31 and #32). Manipulator contracts are unchanged, and these targets
are not listed in `.github/baseline/manipulator_targets.tsv`.

## Files

| File | Role |
| --- | --- |
| `estimator.proto` | `EstimatorConfig`, `MeasurementEnvelope`, `EstimatorResult`, `SourceStatus`, `GetStateResponse`. |
| `estimator_service.h`, `estimator_service.py` | Abstract `EstimatorService`. |
| `fake_estimator_service.{h,cc,py}` | Fixed-seed `FakeEstimatorService`. |
| `fake_estimator_service_test.{cc,py}` | Paired tests. Same cases in both languages. |
| `eskf_state.{h,cc,py}` | ESKF nominal (16), error (15), and covariance (15x15) layout (#81). See `ESKF_LAYOUT.md`. |
| `eskf_state_test.{cc,py}` | Paired layout tests: index order, dimensions, defaults. |
| `eskf_propagate.{h,cc,py}` | `PropagateNominal`: IMU nominal-state propagation only (#82). See `ESKF_PROPAGATE.md`. |
| `eskf_propagate_test.{cc,py}` | Paired propagation tests: stationary, constant rate, rejects, determinism. |
| `eskf_cov_propagate.{h,cc,py}` | `PropagateCovariance`: `Phi = I + F dt`, `Qd`, symmetrized `P` (#83). See `ESKF_COV_PROPAGATE.md`. |
| `eskf_cov_propagate_test.{cc,py}` | Paired tests: zero noise, hover, symmetry, PSD floor, rejects, golden. |
| `innovation_gate.{h,cc,py}` | `GateInnovation`: `S = H P Hᵀ + R`, Cholesky `d²`, accept iff `d² <= chi2` (#84). See `INNOVATION_GATE.md`. |
| `innovation_gate_test.{cc,py}` | Paired tests: boundary, reject, singular, typed errors, golden. |
| `dvl_bottom_track_update.{h,cc,py}` | `UpdateDvlBottomTrack`: gated body-velocity update with Joseph covariance and right-error attitude inject (#85). See `DVL_BOTTOM_TRACK_UPDATE.md`. |
| `dvl_bottom_track_update_test.{cc,py}` | Paired tests: accept, reject, lock loss, invalid, singular, non-finite, golden. |
| `DVL_BOTTOM_TRACK_UPDATE.md` | Locked model, statuses, update steps, fixtures. |
| `dvl_water_track_update.{h,cc,py}` | `UpdateDvlWaterTrack`: gated water-relative body-velocity update using an external `WaterCurrentEstimate`, Joseph covariance, right-error attitude inject (#86). See `DVL_WATER_TRACK_UPDATE.md`. |
| `dvl_water_track_update_test.{cc,py}` | Paired tests: accept, nonzero current, missing current, reject, invalid, singular, non-finite, golden. |
| `DVL_WATER_TRACK_UPDATE.md` | Locked model, statuses, update steps, fixtures. |
| `depth_update.{h,cc,py}` | `UpdateDepth`: gated scalar depth update (positive down from a free-surface ENU Up), Joseph covariance, right-error attitude inject (#87). See `DEPTH_UPDATE.md`. |
| `depth_update_test.{cc,py}` | Paired tests: hover accept, ENU/NED sign, outlier reject, invalid, singular, non-finite, golden. |
| `DEPTH_UPDATE.md` | Locked model, statuses, update steps, fixtures. |
| `altitude_update.{h,cc,py}` | `UpdateAltitude`: gated scalar terrain / seafloor altimeter clearance update (positive up above an external `SeafloorContext`), Joseph covariance, right-error attitude inject (#88). See `ALTITUDE_UPDATE.md`. |
| `altitude_update_test.{cc,py}` | Paired tests: flat seafloor accept, missing seafloor, outlier reject, invalid, singular, non-finite, golden. |
| `ALTITUDE_UPDATE.md` | Locked model, statuses, update steps, fixtures. |
| `INNOVATION_GATE.md` | Locked numerics, statuses, diagnostics, fixtures. |
| `ESKF_COV_PROPAGATE.md` | Locked `F` blocks, `Qd`, statuses, fixtures. |
| `ESKF_PROPAGATE.md` | Locked discretization order, statuses, helpers, fixtures. |
| `ESKF_LAYOUT.md` | Index tables, defaults, and the #17 `VehicleState` mapping. |
| `testdata/nominal_vehicle_state.textproto` | Golden nominal `VehicleState` from the fake. |

## Reuse of #17

- `VehicleState` is returned as is. `estimator_epoch` (field 9) already exists,
  so no metadata message was added for it.
- `NavigationMode` wire values stay `0..4` (`UNSPECIFIED`, `INITIALIZING`,
  `DEAD_RECKONING`, `AIDED`, `FAULTED`). The service never rewrites them.
  Unknown numbers classify as unknown through `ClassifyNavigationMode`.
- Frames, units, and validity follow #15 and #17. The published pose is in
  `world_enu` (REP-103), body quantities are REP-103, units are SI. An invalid
  sample is not an absent one. Non-finite values are invalid.
- `AssessVehicleState` decides whether a `Reset` prior or the initial pose is
  usable. Embodiment `Validity` and sensor `MeasurementHealth.state` are never
  rewritten.

## Operations

| Operation | Behavior |
| --- | --- |
| `Initialize(config)` | Loads the config, clears every buffer, bumps the epoch, sets mode `INITIALIZING`. Calling it again is a full reset. An invalid config (negative `max_future_skew`) returns `INVALID_CONFIG` and changes nothing. |
| `Ingest(envelope)` | Accepts or rejects one sample with a first-defect `RejectReason`. |
| `Predict(to_time)` | Advances the snapshot stamp and applies the mode rule below. Time never moves backward (`INVALID_TIME`). |
| `Reset(prior?)` | Clears buffers and bumps the epoch. A prior that passes `AssessVehicleState` accept becomes the body. Otherwise the body is empty. The mode is `INITIALIZING` either way. |
| `GetState()` | Returns the latched `VehicleState` and per-source status. `state` is unset before the first `Initialize`, which is absent and not an error. |

The epoch starts at 1 on the first `Initialize`, is a `uint64`, and increases by
one on every `Initialize` and `Reset`. It is never reused.

Config defaults: `estimator_id` `"estimator"`, `world_frame_id` `"world_enu"`,
`clock_domain` `"monotonic"`, `max_future_skew` 0,
`unhealthy_after_consecutive_rejects` 3.

## Mode transitions

One deterministic rule. Only `Initialize`, `Reset`, `Predict`, and fault
injection change the mode. `Ingest` never does.

| From | Event | To |
| --- | --- | --- |
| any | `Initialize`, `Reset` | `INITIALIZING` |
| `INITIALIZING` | `Predict`, state holds a pose that passes `AssessVehicleState` accept, no aiding accepted since the last `Predict` | `DEAD_RECKONING` |
| `INITIALIZING`, `DEAD_RECKONING`, `AIDED` | `Predict`, valid pose, at least one aiding sample accepted since the last `Predict` | `AIDED` |
| `AIDED` | `Predict`, no aiding sample accepted since the last `Predict` | `DEAD_RECKONING` |
| `INITIALIZING` | `Predict`, no valid pose | stays `INITIALIZING` |
| any | fault injection (fake) | `FAULTED` |
| `FAULTED` | `Predict` | stays `FAULTED` (stamp still advances) |
| `FAULTED` | `Initialize`, `Reset` | `INITIALIZING` |

- Aiding kinds: `DVL`, `PRESSURE`, `ALTIMETER`, `SURFACE_FIX`. `IMU`, `INS`, and
  `THRUSTER_FEEDBACK` are propagation-class and never mark `AIDED`.
- `Initialize` installs a seed-derived prior pose, so the first `Predict` leaves
  `INITIALIZING`. After `Reset` without an accepted prior there is no pose, so
  the mode stays `INITIALIZING` until `Initialize` or a `Reset` with a prior.
- `FAULTED` is latched. `Ingest` returns `ESTIMATOR_FAULTED` and changes
  nothing. Fault injection is a test hook only. It is not a safety transition.

## Ingest: out-of-order and future policy

Each source is keyed by `MeasurementEnvelope.source_id`. The first defect in
this order wins.

| Order | Case | Policy |
| --- | --- | --- |
| 1 | Not initialized | Reject `NOT_INITIALIZED`. |
| 2 | Estimator faulted | Reject `ESTIMATOR_FAULTED`. |
| 3 | Empty `source_id` | Reject `INVALID_ENVELOPE`. No bookkeeping is possible. |
| 4 | Unspecified or unknown `kind`, missing `source_time` or `receive_time`, nanos out of range, payload without `payload_type` | Reject `INVALID_ENVELOPE`. |
| 5 | `header.clock_domain` differs from the config | Reject `CLOCK_DOMAIN_MISMATCH`. |
| 6 | Header validity absent, `STATE_UNSPECIFIED`, or `STATE_INVALID` | Reject `INVALID_PAYLOAD`. Only `STATE_VALID` is usable (#15). |
| 7 | **Future:** `source_time > receive_time + max_future_skew` (default skew 0) | Reject `FUTURE_MEASUREMENT`. |
| 8 | **Out-of-order:** `source_time < last accepted source_time` of that source | Reject `OUT_OF_ORDER`. There is no reorder buffer. |
| 9 | **Equal timestamp:** `source_time` equals the last accepted one of that source | Reject `DUPLICATE_TIMESTAMP`. |
| 10 | Payload fails the validator (fake hook) | Reject `INVALID_PAYLOAD`. |
| 11 | **Delayed but in order:** `source_time` later than the last accepted one, even if `receive_time` is much later | Accept. The delay is not repaired. Age stays visible in the stamps. |

Accepted times are strictly increasing per source, so "already accepted" equals
"equal to the last accepted". Ordering is per source. Two sources do not
constrain each other. Bounded buffering is WP #30 and is not part of this leaf.

A reject never changes the published pose, twist, covariance, mode, epoch, or
header. It only updates the `SourceStatus` of the named source
(`last_reject_reason`, `consecutive_rejects`, `healthy`). A source appears in
`VehicleState.sources` only after its first accept, so a reject from an unknown
source changes `GetStateResponse.source_status` and nothing in `state`.

Payloads are opaque: `payload_type` (fully qualified HAL message name) plus
`payload` bytes. This package does not re-implement HAL validation. The fake
takes an optional payload validator so callers can apply the existing marine
HAL host policies (#76 through #80, #28).

## Source health

`SourceStatus` per source, in first-seen order: `healthy`,
`last_accept_source_time`, `last_reject_reason`, `consecutive_rejects`.

- `healthy` is false before the first accept, true after an accept, and false
  after `unhealthy_after_consecutive_rejects` consecutive rejects or after
  source fault injection. The next accept makes it healthy again.
- `VehicleState.sources` carries `STATE_VALID` for healthy and `STATE_INVALID`
  for unhealthy sources that have been accepted at least once.

## FakeEstimatorService

- Seeded. `EstimatorConfig.seed` drives SplitMix64 draws in 1/1024 steps, so the
  initial pose (`x`, `y` in `[0, 64)` m, `z` in `(-16, 0]` m, identity
  orientation) and surge `body_twist.linear_x_m_s` in `[0, 2)` m/s are exact in
  binary floating point and identical in C++ and Python.
- `publish_pose_covariance` true publishes a specified diagonal pose covariance
  (0.25 position, 0.0625 rotation). False leaves it unset, which is unknown and
  not the zero matrix. Twist covariance is always unknown.
- `Predict` holds pose, twist, and covariance. There is no process noise, no
  error state, and no integration.
- `StateDigest()` / `state_digest()` is FNV-1a 64 over the deterministic
  serialization of the state, for replay comparison.
- Hooks: `InjectFault`, `InjectSourceFault`, `SetPayloadValidator`.

## Tests and golden

`fake_estimator_service_test.cc` and `fake_estimator_service_test.py` cover
operation smoke, covariance present and absent, mode transitions, source
health, epoch bumps, future, out-of-order, duplicate, delayed-in-order, reject
isolation, fault injection, and replay determinism.

`testdata/nominal_vehicle_state.textproto` is the golden: seed 29,
`publish_pose_covariance`, `Initialize`, accepted DVL and pressure ingest,
`Predict(1700000001)`, giving mode `AIDED`, epoch 1, and two healthy sources.
Both tests parse it and compare serialized bytes and the digest. To refresh
it, run the nominal scenario from the Python test and update the digest constant
in both tests.

Run with Bazel:

```shell
bazel test //intrinsic/estimation:all
```
