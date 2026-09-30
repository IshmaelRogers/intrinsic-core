# Marine measurement health

Opt-in sensor-agnostic health envelope for one marine measurement sample.
The message is `MeasurementHealth` in
`intrinsic_proto.hardware.marine`. PDR §15 places sensor contracts in this
package. Common stamped provenance stays on embodiment `StampedHeader`.
Covariance and source health stay on the vehicle messages from #17.
`DvlMeasurement` and `PressureDepthMeasurement` embed this envelope.
Both are specified below.

This package does not add a robot-type enum, a platform-wide embodiment
switch, ICON feature interfaces, FlatBuffers, Gazebo plugins, filters, or
actuator commands. It does not append a value to
`intrinsic_proto.embodiment.Validity`.

Manipulator joint, Cartesian, kinematics, motion-planning, World, and Gazebo
contracts are unchanged. `.github/baseline/manipulator_targets.tsv` does not
list these targets. A default message serializes to zero bytes. Callers that
never set this message keep the previous manipulator behavior.

Host checks live in `measurement_health_policy.h` and
`measurement_health_policy.py`. They do not parse protobuf and they do not
convert frames.

## Messages

| Message | Role |
| --- | --- |
| `MeasurementHealth` | One sample's shared health envelope. |
| `MeasurementHealth.State` | Local judgment: `UNKNOWN=0`, `VALID=1`, `DEGRADED=2`, `INVALID=3`. |

Reused, not redefined:

| Type | Source |
| --- | --- |
| `intrinsic_proto.embodiment.StampedHeader` | #15. Timestamp, frame, clock, and the orthogonal Validity companion. |
| `intrinsic_proto.vehicle.Matrix6` | #17. Row-major 6x6 covariance values. |
| `intrinsic_proto.vehicle.SourceHealth` | #17. One producer id plus optional embodiment Validity. |

## State

`state` is `optional`. Unset and `UNKNOWN` are different wire values.

| Observation | Meaning |
| --- | --- |
| `state` field missing | Absent judgment. No classification was supplied. |
| `UNKNOWN` | Judgment present and unclassified. |
| `VALID` | Explicitly usable, still subject to the structural checks. |
| `DEGRADED` | Explicit degraded judgment. Recorded, not rewritten to invalid. |
| `INVALID` | Explicit rejection. |
| Any other number | Kept on the wire. Not accepted as valid. Not rewritten to invalid. |

`header.validity` uses embodiment `Validity` (`STATE_UNSPECIFIED=0`,
`STATE_VALID=1`, `STATE_INVALID=2` only). That companion does not gain
`DEGRADED`. Host checks classify it with the #15 helper and do not use it
as `MeasurementHealth.State`. A `STATE_VALID` header does not make a
measurement `INVALID` sample usable, and a `STATE_INVALID` header does not
rewrite `state`.

`accepted` on the host assessment requires measurement state `VALID` and no
structural defect in the fields that are present. `DEGRADED`, `UNKNOWN`,
`INVALID`, absent, and unrecognized are not accepted. An empty message is
not accepted and is not an error. These checks do not rewrite `state` or
the header stamp.

## Documented producer status

The DVL payload below is the first sensor message on this envelope.
Pressure and depth follow it in this package. Other sensor payloads are
later contracts. This envelope records the status those producers already
decided:

| Situation | What the producer sets | Host result |
| --- | --- | --- |
| Nominal sample | `VALID`, finite in-range quality, explicit frame, ordered stamps | `accepted` when no structural defect is present. |
| Dropout, no-return, or lock-loss | `INVALID`, or omit the message | `INVALID` is not accepted and is not a structural error. An omitted message is absent, not an error. |
| Bias or noise | `DEGRADED` | Not accepted. Not rewritten to `INVALID`. |
| Delay with receive time at or after source time | Keep the judgment. Stamps show the age. | Not a structural defect. This contract has no maximum age. |
| Source time after receive time | Stamps reversed | `kTimeReversal`. The judgment is not rewritten. |
| Quality outside `[0, 1]` | Present quality | `kQuality`. |
| Non-finite quality | Present quality | `kNonFinite`. `VALID` is not rewritten. |

## Quality

`quality` is `optional float`, dimensionless, closed interval `[0, 1]`.

| Wire | Meaning |
| --- | --- |
| Field unset | Absent. Not a defect. |
| Present `0` or `1` | Supplied endpoints. In range. |
| Present value outside `[0, 1]` | Rejected. |
| NaN or infinity | Rejected as non-finite. Not an out-of-range finite value. |

## Frames and time

`header.frame_id` is the frame of this sample. It is never inferred from
`MeasurementHealth`. When the sample is engaged, an empty frame id is
rejected. A caller may pass an expected frame id; the check is an exact
string match. `world_enu`, `world_ned`, `body`, and any other non-empty id
are legal when they are the stamped frame. This package does not call the
ENU↔NED helper.

`StampedHeader` time rules apply. `source_time` is measurement time.
`receive_time` is ingestion time. When both are present, receive time
strictly before source time is rejected. Equal timestamps are ordered.
`clock_domain` of `monotonic` is the only domain for which the existing age
helper returns a value. `utc` and every other domain do not drive a
watchdog. A large positive age is not a structural defect.

## Covariance

`covariance` reuses `Matrix6` and `AssessCovariance`.

| Wire | Meaning |
| --- | --- |
| Parent field unset | Unknown covariance. This is the only unknown encoding. |
| Present, length not 36 | Invalid shape. Not unknown. |
| Present, any NaN or infinity | Invalid. Not unknown. |
| Present, asymmetric beyond `1e-9` | Invalid. A covariance is symmetric. |
| Present, 36 zeros | Specified zero matrix. Not unknown. |

An unknown covariance does not by itself fail the structural check. A
symmetric finite matrix of length 36 does not fail it either. Positive
semidefinite checks stay with the estimator.

## Source health

`sources` reuses `SourceHealth`.

| Observation | Meaning |
| --- | --- |
| Entry absent | That producer was not reported. |
| Entry present, `validity` unset | No health judgment. Not `STATE_INVALID`. |
| Entry present, `STATE_INVALID` | Producer explicitly unhealthy on the #15 scale. |
| Entry present, empty `source_id` | Rejected. |

Duplicate source ids are preserved. An empty `header.source_id` is not this
check. The header id and a `SourceHealth.source_id` are different fields.

## Check order

The first defect wins:

1. Empty frame id.
2. Frame id different from a caller-supplied expected id.
3. Receive time strictly before source time, when both timestamps are present.
4. Non-finite quality, when quality is present.
5. Quality outside `[0, 1]`, when quality is present.
6. Covariance shape, when covariance is present. Absence is not a defect.
7. Empty `source_id` on a present source entry.

An unengaged view (no header, state, quality, covariance, sources, or
timestamps) is not a sample.

## Evolution

Append fields and enum values. Reserve removed tags and names. Preserve
unknown fields and unknown enum numbers. Golden bytes for one populated
`MeasurementHealth` are fixed in the C++ and Python serialization tests.
Clearing `sources` leaves a prefix of that golden. Field 100 is preserved.

## Examples

Text format examples:

- [`examples/measurement_health.textproto`](examples/measurement_health.textproto)
  sets `VALID`, quality, and a specified covariance.
- [`examples/measurement_health_unknown_covariance.textproto`](examples/measurement_health_unknown_covariance.textproto)
  omits `covariance`, which means unknown, not zero.

## Targets

These targets are separate from the protected manipulator baseline:

- `//intrinsic_hardware/intrinsic/hardware/marine:measurement_health_proto`
- `//intrinsic_hardware/intrinsic/hardware/marine:measurement_health_cc_proto`
- `//intrinsic_hardware/intrinsic/hardware/marine:measurement_health_py_pb2`
- `//intrinsic_hardware/intrinsic/hardware/marine:measurement_health_policy`
- `//intrinsic_hardware/intrinsic/hardware/marine:measurement_health_policy_py`
- `//intrinsic_hardware/intrinsic/hardware/marine:measurement_health_policy_test`
- `//intrinsic_hardware/intrinsic/hardware/marine:measurement_health_policy_test_py`
- `//intrinsic_hardware/intrinsic/hardware/marine:measurement_health_serialization_test`
- `//intrinsic_hardware/intrinsic/hardware/marine:measurement_health_serialization_test_py`
- `//intrinsic_hardware/intrinsic/hardware/marine:examples`

## DVL measurement

`DvlMeasurement` is one bottom-track or water-track Doppler velocity sample.
It embeds `MeasurementHealth` as `health`. The velocity frame is
`health.header.frame_id`. It is never inferred from `DvlMeasurement`. Units
are SI. Velocity is meters per second. Altitude is meters.

Host checks live in `dvl_policy.h` and `dvl_policy.py`. They do not parse
protobuf and they do not convert frames. `FakeDvl` is a deterministic
producer. There is no estimator adapter, ICON FlatBuffer, Gazebo plugin, or
real-hardware path.

| Message | Role |
| --- | --- |
| `DvlMeasurement` | One DVL sample: health plus track payload. |
| `DvlMeasurement.Mode` | `MODE_UNSPECIFIED=0`, `MODE_BOTTOM_TRACK=1`, `MODE_WATER_TRACK=2`. |

### Mode

`mode` is `optional`. Unset and `MODE_UNSPECIFIED` are different wire values.
This enum is not `NavigationMode`.

| Observation | Meaning |
| --- | --- |
| `mode` field missing | Absent. Not a track mode. |
| `MODE_UNSPECIFIED` | Judgment present and not a track mode. |
| `MODE_BOTTOM_TRACK` | Bottom track. |
| `MODE_WATER_TRACK` | Water track. |
| Any other number | Kept on the wire. Not a valid track mode. Not rewritten. |

An engaged sample with absent, unspecified, or unrecognized mode is rejected.

### Velocity

`velocity_x_m_s`, `velocity_y_m_s`, and `velocity_z_m_s` are optional doubles,
REP-103 axes in `health.header.frame_id`. An engaged sample requires all
three, and each present component must be finite. There is no magnitude cap.
A missing component is rejected before a non-finite component. Non-finite
velocity does not rewrite `health.state`. Zero velocity is a supplied sample.

### Covariance

Covariance is `health.covariance` (`Matrix6`), not a new field. Order is
linear x, y, z, then angular x, y, z. #17 shape, symmetry, and finiteness
rules apply. When the matrix is present and well formed, the three angular
variance slots must be exactly `0.0`:

| Slot | Row-major index |
| --- | --- |
| angular x variance | 21 |
| angular y variance | 28 |
| angular z variance | 35 |

Off-diagonal angular entries are not given a further ban here. Positive
semidefinite checks stay with the estimator. Unset covariance is unknown.
Thirty-six zeros are a specified zero matrix, and those three slots are zero.

### Bottom lock

`bottom_lock` is `optional bool`. Unset means absent. Present false is not
the same as unset.

| Combination | Host result |
| --- | --- |
| Bottom track, lock absent, state `VALID` | Not the lock-loss combo. Accepted when nothing else fails. |
| Bottom track, lock false, state `VALID` | Rejected. Inconsistent lock. |
| Bottom track, lock false, state `INVALID` | Lock-loss. Not a structural error. Not accepted. |
| Bottom track, lock false, state `DEGRADED` | Not the `VALID` combo. Not rewritten. Not accepted. |
| Water track, lock false, state `VALID` | Accepted when the rest of the sample is sound. |

### Altitude

`altitude_m` unset means unavailable. A present value must be finite and
`>= 0`. Zero is supplied. Water-track may carry altitude. A finite
non-negative altitude on water track is accepted. This leaf does not ban
that combination.

### Check order

The first defect wins:

1. Health missing on an engaged sample.
2. Empty frame id.
3. Frame id different from a caller-supplied expected id.
4. Receive time strictly before source time, when both timestamps are present.
5. Non-finite quality, when quality is present.
6. Quality outside `[0, 1]`, when quality is present.
7. Covariance shape, when covariance is present.
8. Empty `source_id` on a present source entry.
9. Angular variance slot not exactly zero, when covariance is present and well formed.
10. Mode absent, unspecified, or unrecognized.
11. Any velocity component absent.
12. Any velocity component non-finite.
13. Bottom track with explicit lock false and health state `VALID`.
14. Altitude non-finite, when altitude is present.
15. Altitude negative, when altitude is present.

An unengaged message (no health engagement, mode, velocity, lock, or
altitude) is not a sample. `accepted` requires health state `VALID`, a
bottom or water track mode, and no structural defect. `DEGRADED` is not
rewritten to `INVALID`.

### Fake

`FakeDvl` is a pure function of its config and truth. The same seed, bias,
delay, dropout, and lock-loss flags produce the same bytes. `seed` is
written to `health.header.sequence`. The fake does not draw noise.

Default config and truth are the nominal bottom-track fixture: sequence 42,
frame `sensor`, source time `1700000000.250000000`, delay `1s` plus
`-250000000` ns so receive time is `1700000001.0`, velocity `(0.5, -0.25,
0)` m/s, quality `0.75`, altitude `10` m, bottom lock true, and a specified
covariance with linear variance `0.25` and zero angular variances. Header
validity stays `STATE_VALID`. Sources are `primary` (valid) and `aiding`
(unset validity).

| Fault | What the fake emits |
| --- | --- |
| Dropout | No message. Checked before lock-loss. Absent, not an error. |
| Delay | `receive_time = source_time + delay`. A reversal is emitted and not repaired. |
| Bias | Each bias component is added to truth velocity. Any non-zero bias sets health state `DEGRADED` unless lock-loss applies. |
| Lock loss | Mode bottom track, `bottom_lock` false, health state `INVALID`. Bias still adds. |

A large positive delay is not a structural defect. This contract has no
maximum age.

### Evolution

Append fields and enum values. Reserve removed tags and names. Preserve
unknown fields and unknown mode numbers. Golden bytes for the nominal
bottom-track fixture are fixed in the C++ and Python serialization tests.
Clearing `altitude_m` leaves a prefix of that golden. Field 100 is preserved.

Text format example:
[`examples/dvl_bottom_track.textproto`](examples/dvl_bottom_track.textproto).

### DVL targets

These targets stay off `.github/baseline/manipulator_targets.tsv`:

- `//intrinsic_hardware/intrinsic/hardware/marine:dvl_proto`
- `//intrinsic_hardware/intrinsic/hardware/marine:dvl_cc_proto`
- `//intrinsic_hardware/intrinsic/hardware/marine:dvl_py_pb2`
- `//intrinsic_hardware/intrinsic/hardware/marine:dvl_policy`
- `//intrinsic_hardware/intrinsic/hardware/marine:dvl_policy_py`
- `//intrinsic_hardware/intrinsic/hardware/marine:fake_dvl`
- `//intrinsic_hardware/intrinsic/hardware/marine:fake_dvl_py`
- `//intrinsic_hardware/intrinsic/hardware/marine:dvl_policy_test`
- `//intrinsic_hardware/intrinsic/hardware/marine:dvl_policy_test_py`
- `//intrinsic_hardware/intrinsic/hardware/marine:dvl_serialization_test`
- `//intrinsic_hardware/intrinsic/hardware/marine:dvl_serialization_test_py`
- `//intrinsic_hardware/intrinsic/hardware/marine:fake_dvl_test`
- `//intrinsic_hardware/intrinsic/hardware/marine:fake_dvl_test_py`

## Pressure and depth measurement

`PressureDepthMeasurement` is one pressure sample, one depth sample, or both.
It embeds `MeasurementHealth` as `health`. The sample frame is
`health.header.frame_id`. It is never inferred from
`PressureDepthMeasurement`. Units are SI. Pressure is pascals. Depth is
meters, positive deeper (down from the free surface). Fluid density is
kilograms per cubic meter.

Host checks live in `pressure_depth_policy.h` and
`pressure_depth_policy.py`. They do not parse protobuf, do not convert
frames, and do not integrate hydrostatic pressure. `FakePressureDepth` is a
deterministic producer. There is no altitude field, estimator adapter, ICON
FlatBuffer, Gazebo plugin, or real-hardware path.

| Message | Role |
| --- | --- |
| `PressureDepthMeasurement` | One pressure and/or depth sample plus health. |
| `PressureDepthMeasurement.DepthProvenance` | `DEPTH_PROVENANCE_UNSPECIFIED=0`, `DEPTH_PROVENANCE_DIRECT=1`, `DEPTH_PROVENANCE_FROM_PRESSURE=2`. |

### Pressure and depth

`pressure_pa` and `depth_m` are separate optional doubles. Either may appear
alone. Both may appear together. Unset means absent. An engaged sample
needs at least one of them.

| Wire | Meaning |
| --- | --- |
| Pressure unset | Absent. Not a defect by itself. |
| Pressure present and finite, including zero and negative | Supplied. No absolute-pressure floor. |
| Pressure NaN or infinity | Rejected as non-finite. `health.state` is not rewritten. |
| Depth unset | Absent. Not a defect by itself. |
| Depth present and `0` | The free surface. |
| Depth present and `> 0` | Deeper than the surface. |
| Depth present and `< 0` | Out of range. Rejected. |
| Depth NaN or infinity | Rejected as non-finite, before the sign check. |

### Provenance and fluid density

`depth_provenance` is `optional`. Unset and `DEPTH_PROVENANCE_UNSPECIFIED`
are different wire values. Unknown numbers stay on the wire.

`fluid_density_kg_m3` is `optional`. Unset means absent. A present value
must be finite and `> 0`.

| Combination | Host result |
| --- | --- |
| Depth present, provenance absent, unspecified, or unrecognized | Rejected. Missing conversion provenance. |
| Depth present, `DIRECT`, density absent | Accepted when the rest of the sample is sound. |
| Depth present, `DIRECT`, density finite and `> 0` | Accepted when the rest of the sample is sound. |
| Depth present, `FROM_PRESSURE`, finite pressure, density finite and `> 0` | Accepted when the rest of the sample is sound. |
| `FROM_PRESSURE` without depth | Rejected. Inconsistent provenance. |
| `FROM_PRESSURE` without pressure | Rejected. The derived pair is incomplete. |
| `FROM_PRESSURE` without a finite density `> 0` | Rejected. |
| Depth absent, provenance absent, unspecified, or unrecognized | Not a provenance defect. Pressure may still be accepted. |
| Density present and non-finite, or `<= 0` | Rejected. Non-finite density is `kDensity`, not `kNonFinite`. |

This contract stores provenance. It does not compute depth from pressure.

### Covariance

Covariance is `health.covariance` (`Matrix6`), not a new field. #17 shape,
symmetry, and finiteness rules apply. When the matrix is present and well
formed, every entry except the two sensor diagonals must be exactly `0.0`:

| Slot | Row-major index | Unit |
| --- | --- | --- |
| pressure variance | 0 | Pa² |
| depth variance | 7 | m² |

The other 34 entries, including the remaining diagonals, are exactly zero.
Unset covariance is unknown. Thirty-six zeros are a specified zero matrix,
and both allowed slots are zero.

### Check order

The first defect wins:

1. Health missing on an engaged sample.
2. Empty frame id.
3. Frame id different from a caller-supplied expected id.
4. Receive time strictly before source time, when both timestamps are present.
5. Non-finite quality, when quality is present.
6. Quality outside `[0, 1]`, when quality is present.
7. Covariance shape, when covariance is present.
8. Empty `source_id` on a present source entry.
9. A covariance entry other than indices 0 and 7 is not exactly zero, when covariance is present and well formed.
10. Neither pressure nor depth present.
11. Pressure non-finite, when pressure is present.
12. Depth non-finite, when depth is present.
13. Depth negative, when depth is present.
14. Depth present with provenance absent, unspecified, or unrecognized.
15. `FROM_PRESSURE` without depth.
16. `FROM_PRESSURE` without pressure.
17. `FROM_PRESSURE` without a present finite density `> 0`.
18. Present density non-finite.
19. Present density `<= 0`.

An unengaged message (no health engagement, pressure, depth, provenance, or
density) is not a sample. `accepted` requires health state `VALID` and no
structural defect. `DEGRADED` is not rewritten to `INVALID`.

### Fake

`FakePressureDepth` is a pure function of its config and truth. The same
seed, bias, delay, dropout, and out-of-range flags produce the same bytes.
`seed` is written to `health.header.sequence`. The fake does not draw noise
and it does not integrate hydrostatic pressure.

Default config and truth are the nominal from-pressure fixture: sequence
42, frame `sensor`, source time `1700000000.250000000`, delay `1s` plus
`-250000000` ns so receive time is `1700000001.0`, pressure `200000` Pa,
depth `10` m, provenance `FROM_PRESSURE`, density `1025` kg/m³, quality
`0.75`, and a specified covariance with pressure variance `1.0` at index 0
and depth variance `0.25` at index 7. Header validity stays `STATE_VALID`.
Sources are `primary` (valid) and `aiding` (unset validity).

| Fault | What the fake emits |
| --- | --- |
| Dropout | No message. Checked before out-of-range. Absent, not an error. |
| Delay | `receive_time = source_time + delay`. A reversal is emitted and not repaired. |
| Bias | Each non-zero bias is added to the matching present truth field. Any non-zero bias sets health state `DEGRADED` unless out-of-range applies. |
| Out of range | Canonical fixture: `depth_m = -1`, health state `INVALID`. Pressure bias still applies when pressure is present. Depth bias is not applied. |

A large positive delay is not a structural defect. This contract has no
maximum age. A bias that drives depth below zero is emitted as that value
with `DEGRADED`. The host rejects the negative depth and does not rewrite
`health.state`.

### Evolution

Append fields and enum values. Reserve removed tags and names. Preserve
unknown fields and unknown provenance numbers. Golden bytes for the nominal
from-pressure fixture are fixed in the C++ and Python serialization tests.
Clearing `fluid_density_kg_m3` leaves a prefix of that golden. Field 100 is
preserved.

Text format example:
[`examples/pressure_depth_from_pressure.textproto`](examples/pressure_depth_from_pressure.textproto).

### Pressure and depth targets

These targets stay off `.github/baseline/manipulator_targets.tsv`:

- `//intrinsic_hardware/intrinsic/hardware/marine:pressure_depth_proto`
- `//intrinsic_hardware/intrinsic/hardware/marine:pressure_depth_cc_proto`
- `//intrinsic_hardware/intrinsic/hardware/marine:pressure_depth_py_pb2`
- `//intrinsic_hardware/intrinsic/hardware/marine:pressure_depth_policy`
- `//intrinsic_hardware/intrinsic/hardware/marine:pressure_depth_policy_py`
- `//intrinsic_hardware/intrinsic/hardware/marine:fake_pressure_depth`
- `//intrinsic_hardware/intrinsic/hardware/marine:fake_pressure_depth_py`
- `//intrinsic_hardware/intrinsic/hardware/marine:pressure_depth_policy_test`
- `//intrinsic_hardware/intrinsic/hardware/marine:pressure_depth_policy_test_py`
- `//intrinsic_hardware/intrinsic/hardware/marine:pressure_depth_serialization_test`
- `//intrinsic_hardware/intrinsic/hardware/marine:pressure_depth_serialization_test_py`
- `//intrinsic_hardware/intrinsic/hardware/marine:fake_pressure_depth_test`
- `//intrinsic_hardware/intrinsic/hardware/marine:fake_pressure_depth_test_py`
