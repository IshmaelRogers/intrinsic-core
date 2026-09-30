# Marine measurement health

Opt-in sensor-agnostic health envelope for one marine measurement sample.
The message is `MeasurementHealth` in
`intrinsic_proto.hardware.marine`. PDR §15 places sensor contracts in this
package. Common stamped provenance stays on embodiment `StampedHeader`.
Covariance and source health stay on the vehicle messages from #17.
`DvlMeasurement`, `PressureDepthMeasurement`, `AltimeterMeasurement`,
`ImuMeasurement`, `InsSolution`, and `SurfaceFix` embed this envelope. Each
is specified below.

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
Pressure and depth follow it. Altimeter range follows them. IMU and INS
follow the altimeter as separate messages. `SurfaceFix` is a lighter
position fix and is not an INS solution. There is no filter between
them, and this leaf does not apply a submerged-acceptance policy. Other
sensor payloads are later contracts. This envelope records the status those
producers already decided:

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

## Altimeter measurement

`AltimeterMeasurement` is one seafloor range sample along a single beam.
It embeds `MeasurementHealth` as `health`. The range frame is
`health.header.frame_id`. It is never inferred from `AltimeterMeasurement`.
Units are SI. Range and the sensor bounds are meters. This leaf does not
reuse DVL `altitude_m`.

Host checks live in `altimeter_policy.h` and `altimeter_policy.py`. They
do not parse protobuf, do not convert frames, and do not fuse bathymetry.
`FakeAltimeter` is a deterministic producer. There is no estimator adapter,
ICON FlatBuffer, Gazebo plugin, or real-hardware path.

| Message | Role |
| --- | --- |
| `AltimeterMeasurement` | One beam range sample plus health. |

### Range

`range_m` is an optional double, meters along the beam. Unset means absent.
An engaged sample needs a present range or an explicit no-return.

| Wire | Meaning |
| --- | --- |
| Range unset | Absent. Not a defect when `has_return` is present and false. |
| Range present and `0` | Contact. Legal. |
| Range present and `> 0` | Seafloor range along the beam. |
| Range present and `< 0` | Rejected. `health.state` is not rewritten. |
| Range NaN or infinity | Rejected as non-finite, before the sign check. |

`has_return` unset with a present finite in-bounds range is a range-only
sample and is accepted when the rest of the sample is sound.

### Beam and bounds

`beam_id` unset means absent. A present value must be non-empty. There is
no robot-type enum.

`min_range_m` and `max_range_m` unset means unspecified. A present bound
must be finite and `>= 0`. When both are present, `min_range_m` must be
`<= max_range_m`. A present range strictly below a present minimum, or
strictly above a present maximum, is a structural reject. Range equal to
a present bound is inside the window. `min_range_m == max_range_m` is
legal when the range equals that value.

### Return

`has_return` is `optional bool`. Unset means absent. Present false is
no-return, which is not the same as unset.

| Combination | Host result |
| --- | --- |
| `has_return` false, range absent, state `INVALID` | No-return. Not a structural error. Not accepted. |
| `has_return` false, range absent, state `VALID` | Rejected. Inconsistent no-return. |
| `has_return` false, range absent, state `DEGRADED` | Not the `VALID` combo. Not rewritten. Not accepted. |
| `has_return` false and range present | Rejected, after the range and bound checks. |
| `has_return` true, or unset, with a sound range | Accepted when nothing else fails. |

### Covariance

Covariance is `health.covariance` (`Matrix6`), not a new field. #17 shape,
symmetry, and finiteness rules apply. When the matrix is present and well
formed, every entry except diagonal index 0 must be exactly `0.0`:

| Slot | Row-major index | Unit |
| --- | --- | --- |
| range variance | 0 | m² |

The other 35 entries are exactly zero. Unset covariance is unknown.
Thirty-six zeros are a specified zero matrix, and the range slot is zero.

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
9. A covariance entry other than index 0 is not exactly zero, when covariance is present and well formed.
10. Neither `range_m` present nor explicit no-return (`has_return` present and false).
11. `range_m` non-finite, when range is present.
12. `range_m` negative, when range is present.
13. `min_range_m` non-finite, when min is present.
14. `min_range_m` negative, when min is present.
15. `max_range_m` non-finite, when max is present.
16. `max_range_m` negative, when max is present.
17. Both bounds present and `min_range_m` > `max_range_m`.
18. Present `range_m` strictly below present `min_range_m`.
19. Present `range_m` strictly above present `max_range_m`.
20. `has_return` false and health state `VALID`.
21. `has_return` false and `range_m` present.
22. `beam_id` present and empty.

An unengaged message (no health engagement, range, beam, bounds, or
`has_return`) is not a sample. `accepted` requires health state `VALID`
and no structural defect. `DEGRADED` is not rewritten to `INVALID`.

### Fake

`FakeAltimeter` is a pure function of its config and truth. The same seed,
noise amplitude, delay, dropout, no-return, and out-of-range flags produce
the same bytes. `seed` is written to `health.header.sequence`. It also
mixes the noise draw. The draw is SplitMix64 of the seed, mapped onto
`[-1, 1)` with a 53-bit fraction that is exact in binary64. C++ and Python
use the same mix. The fake does not fuse bathymetry.

Default config and truth are the nominal range fixture: sequence 42, frame
`sensor`, source time `1700000000.250000000`, delay `1s` plus `-250000000`
ns so receive time is `1700000001.0`, range `10` m, beam `down`, minimum
`0.5` m, maximum `100` m, `has_return` true, quality `0.75`, and a
specified covariance with range variance `0.25` at index 0. Header
validity stays `STATE_VALID`. Sources are `primary` (valid) and `aiding`
(unset validity). Noise amplitude defaults to `0`.

| Fault | What the fake emits |
| --- | --- |
| Dropout | No message. Checked before no-return and out-of-range. Absent, not an error. |
| Delay | `receive_time = source_time + delay`. A reversal is emitted and not repaired. |
| Noise | A non-zero amplitude adds `amplitude * signed_unit(seed)` to a present range and sets health state `DEGRADED`, unless no-return or out-of-range applies. A negative or out-of-bounds result is emitted and not repaired. |
| No-return | `has_return` false, range omitted, health state `INVALID`. Wins over out-of-range and noise. Beam and bounds stay on the truth. |
| Out of range | Canonical fixture: `range_m = 101`, `min_range_m = 0.5`, `max_range_m = 100`, `has_return` true, health state `INVALID`. Noise is not added. Truth bounds are replaced. |

A large positive delay is not a structural defect. This contract has no
maximum age. Noise that drives range below zero is emitted as that value
with `DEGRADED`. The host rejects the negative range and does not rewrite
`health.state`.

### Evolution

Append fields. Reserve removed tags and names. Preserve unknown fields.
Golden bytes for the nominal range fixture are fixed in the C++ and Python
serialization tests. Clearing `has_return` leaves a prefix of that golden.
Field 100 is preserved.

Text format example:
[`examples/altimeter_nominal.textproto`](examples/altimeter_nominal.textproto).

### Altimeter targets

These targets stay off `.github/baseline/manipulator_targets.tsv`:

- `//intrinsic_hardware/intrinsic/hardware/marine:altimeter_proto`
- `//intrinsic_hardware/intrinsic/hardware/marine:altimeter_cc_proto`
- `//intrinsic_hardware/intrinsic/hardware/marine:altimeter_py_pb2`
- `//intrinsic_hardware/intrinsic/hardware/marine:altimeter_policy`
- `//intrinsic_hardware/intrinsic/hardware/marine:altimeter_policy_py`
- `//intrinsic_hardware/intrinsic/hardware/marine:fake_altimeter`
- `//intrinsic_hardware/intrinsic/hardware/marine:fake_altimeter_py`
- `//intrinsic_hardware/intrinsic/hardware/marine:altimeter_policy_test`
- `//intrinsic_hardware/intrinsic/hardware/marine:altimeter_policy_test_py`
- `//intrinsic_hardware/intrinsic/hardware/marine:altimeter_serialization_test`
- `//intrinsic_hardware/intrinsic/hardware/marine:altimeter_serialization_test_py`
- `//intrinsic_hardware/intrinsic/hardware/marine:fake_altimeter_test`
- `//intrinsic_hardware/intrinsic/hardware/marine:fake_altimeter_test_py`

## IMU measurement

`ImuMeasurement` is one raw inertial sample. It embeds `MeasurementHealth`
as `health`. The sample frame is `health.header.frame_id`. It is never
inferred from `ImuMeasurement`. Axes are REP-103 in that frame. The host
does not convert ENU and NED. Units are SI: angular velocity is rad/s and
linear acceleration is m/s². This message is not an INS solution and it
does not propagate a filter.

Host checks live in `imu_policy.h` and `imu_policy.py`. They do not parse
protobuf, do not convert frames, do not renormalize quaternions, and do
not rewrite `health.state` or embodiment Validity.
`FakeImu` is a deterministic producer. There is no estimator adapter,
magnetometer field, ICON FlatBuffer, Gazebo plugin, or real-hardware path.

| Message | Role |
| --- | --- |
| `ImuMeasurement` | One raw ω and a sample plus health. |
| `ImuMeasurement.QuaternionXyzw` | Optional Hamilton attitude, stored x, y, z, w. |

### Rates and acceleration

An engaged sample requires both triples. Each component is an optional
double, so zero is a supplied value and is not the same as absent.

| Wire | Meaning |
| --- | --- |
| All six components present and finite | Required payload. Zero is legal. |
| Any component of either triple absent | Rejected. |
| Any present component NaN or infinity | Rejected as non-finite. `health.state` is not rewritten. |

There is no magnitude cap.

### Orientation

`orientation_xyzw` is optional. Unset means a raw IMU sample with no
attitude. When the nested message is present, all four components are
supplied (zero is a component). The quaternion must be finite and
unit-norm: `|‖q‖ − 1| ≤ 1e-6`. The host does not renormalize and does not
mutate the view. Identity `(0, 0, 0, 1)` and `(0, 0, 0, -1)` are unit.
A present non-unit quaternion is `kOrientation`.

### Covariance

Covariance is `health.covariance` (`Matrix6`), not a new field. #17 shape,
symmetry, and finiteness rules apply. When the matrix is present and well
formed, only these diagonal indices may be non-zero. Every other entry
must be exactly `0.0`:

| Slot | Row-major index | Unit |
| --- | --- | --- |
| ωx variance | 0 | (rad/s)² |
| ωy variance | 7 | (rad/s)² |
| ωz variance | 14 | (rad/s)² |
| ax variance | 21 | (m/s²)² |
| ay variance | 28 | (m/s²)² |
| az variance | 35 | (m/s²)² |

Orientation uncertainty is not a second matrix. Unset covariance is
unknown. Thirty-six zeros are a specified zero matrix. A negative diagonal
is structurally allowed. An off-diagonal that breaks #17 symmetry beyond
`1e-9` is a covariance defect. An entry that is within that symmetry
tolerance and is not exactly zero is a slot defect.

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
9. A covariance entry other than indices 0, 7, 14, 21, 28, and 35 is not exactly zero, when covariance is present and well formed.
10. Angular-velocity triple incomplete.
11. A present angular-velocity component is non-finite.
12. Linear-acceleration triple incomplete.
13. A present linear-acceleration component is non-finite.
14. A present orientation component is non-finite.
15. A present orientation is not unit-norm within `1e-6`.

An unengaged message (no health engagement and no rate, acceleration, or
orientation presence) is not a sample. `accepted` requires health state
`VALID` and no structural defect. `DEGRADED` and `INVALID` are not
rewritten. Header Validity does not select `accepted`.

### Fake

`FakeImu` is a pure function of its config and truth. The same seed, bias,
drift, noise, delay, dropout, and invalid-orientation flags produce the
same bytes. `seed` is written to `health.header.sequence`. It also mixes
the noise draw. The draw is SplitMix64 of the seed, mapped onto `[-1, 1)`
with a 53-bit fraction that is exact in binary64. C++ and Python use the
same mix. Angular noise uses `seed`. Linear-acceleration noise uses
`seed + 1`. The fake does not read an INS solution.

Default config and truth are the nominal fixture: sequence 42, frame
`imu`, source time `1700000000.250000000`, delay `1s` plus `-250000000` ns
so receive time is `1700000001.0`, angular velocity `(0.25, -0.5, 0.125)`
rad/s, linear acceleration `(0, 0, 8)` m/s², orientation identity
`(0, 0, 0, 1)`, quality `0.75`, and a specified covariance with diagonals
`0.25`, `0.5`, `0.125`, `1`, `2`, and `4` at indices 0, 7, 14, 21, 28, and
35. Header validity stays `STATE_VALID`. `source_id` is `nav_sensor`. The
clock domain is monotonic. Sources are `primary` (valid) and `aiding`
(unset validity). Noise, bias, and drift amplitudes default to `0`.

| Fault | What the fake emits |
| --- | --- |
| Dropout | No message. Checked first, including when invalid orientation is also set. Absent, not an error. |
| Delay | `receive_time = source_time + delay`. A reversal is emitted and not repaired. |
| Bias | Added per component to ω and/or a. Any non-zero component sets health state `DEGRADED`, unless invalid orientation applies. |
| Drift | `amplitude * seed` added to every component of that triple. A non-zero amplitude sets `DEGRADED`, unless invalid orientation applies. |
| Noise | A non-zero amplitude, including a non-finite amplitude, adds the signed-unit draw and sets `DEGRADED`, unless invalid orientation applies. The result is not repaired. |
| Invalid orientation | Canonical quaternion `(0, 0, 0, 2)`, health state `INVALID`. Bias, drift, and noise are not applied. Delay is still applied. |

A large positive delay is not a structural defect. This contract has no
maximum age. The canonical non-unit quaternion is the stronger `INVALID`
fixture. The host rejects it and does not rewrite `health.state` or the
header Validity companion.

### Evolution

Append fields. Reserve removed tags and names. Preserve unknown fields.
Golden bytes for the nominal fixture and the rates-only fixture (orientation
absent) are fixed in the C++ and Python serialization tests. Those bytes
were captured from a local C++ protobuf run of `FakeImu` because Bazel was
not available in the authoring environment. The rates-only encoding is a
prefix of the nominal encoding. Field 100 is preserved.

Text format examples:
[`examples/imu_nominal.textproto`](examples/imu_nominal.textproto) and
[`examples/imu_rates_only.textproto`](examples/imu_rates_only.textproto).

### IMU targets

These targets stay off `.github/baseline/manipulator_targets.tsv`:

- `//intrinsic_hardware/intrinsic/hardware/marine:imu_proto`
- `//intrinsic_hardware/intrinsic/hardware/marine:imu_cc_proto`
- `//intrinsic_hardware/intrinsic/hardware/marine:imu_py_pb2`
- `//intrinsic_hardware/intrinsic/hardware/marine:imu_policy`
- `//intrinsic_hardware/intrinsic/hardware/marine:imu_policy_py`
- `//intrinsic_hardware/intrinsic/hardware/marine:fake_imu`
- `//intrinsic_hardware/intrinsic/hardware/marine:fake_imu_py`
- `//intrinsic_hardware/intrinsic/hardware/marine:imu_policy_test`
- `//intrinsic_hardware/intrinsic/hardware/marine:imu_policy_test_py`
- `//intrinsic_hardware/intrinsic/hardware/marine:imu_serialization_test`
- `//intrinsic_hardware/intrinsic/hardware/marine:imu_serialization_test_py`
- `//intrinsic_hardware/intrinsic/hardware/marine:fake_imu_test`
- `//intrinsic_hardware/intrinsic/hardware/marine:fake_imu_test_py`

## INS solution

`InsSolution` is one optional vendor navigation solution. It embeds
`MeasurementHealth` as `health`. Position and attitude are in
`health.header.frame_id`. The frame is never inferred from `InsSolution`.
The host does not convert ENU and NED and does not apply a
submerged-acceptance policy. Position is meters. This message is not an
IMU sample and it does not propagate a filter from `ImuMeasurement`.

Host checks live in `ins_policy.h` and `ins_policy.py`. They do not parse
protobuf, do not convert frames, do not renormalize quaternions, do not
read IMU samples, and do not rewrite `health.state` or embodiment
Validity. `FakeIns` is a deterministic producer. There is no estimator
adapter, ICON FlatBuffer, Gazebo plugin, or real-hardware path.

| Message | Role |
| --- | --- |
| `InsSolution` | One position, attitude, and optional twist plus health. |
| `InsSolution.QuaternionXyzw` | Required Hamilton attitude when the sample is engaged. |
| `InsSolution.SourceKind` | Optional measurement-local source. |

### Position and orientation

An engaged sample requires a finite position triple and a present unit
quaternion. Each position component is an optional double, so zero and
negative positions are supplied values.

`orientation_xyzw` is a message, not `optional`. Presence of the nested
message is required when the sample is engaged. All four components are
supplied when it is present. The quaternion must be finite and unit-norm:
`|‖q‖ − 1| ≤ 1e-6`. The host does not renormalize. The canonical non-unit
fixture is `(0, 0, 0, 2)`.

### Twist

Linear velocity (m/s) and angular velocity (rad/s) are optional triples.
All three components unset means that twist was not reported. If any
component of a triple is present, all three must be present and finite.
Zero components of a present triple are supplied values.

### Source

`source` is `optional`. Unset is absent and is not `SOURCE_UNSPECIFIED`.

| Wire | Host classification |
| --- | --- |
| Field unset | Absent. Allowed on an otherwise sound sample. |
| `SOURCE_UNSPECIFIED` (`0`) | Explicit non-source. Rejected when the sample is engaged. Not rewritten. |
| `SOURCE_VENDOR_INS` (`1`) | Known vendor solution. |
| `SOURCE_EXTERNAL_NAV` (`2`) | Known external navigation source. |
| Any other number | Kept on the wire. Not accepted. Not rewritten. |

### Covariance

Covariance is `health.covariance` (`Matrix6`). #17 shape, symmetry, and
finiteness rules apply. When the matrix is present and well formed, only
these diagonal indices may be non-zero. Every other entry must be exactly
`0.0`:

| Slot | Row-major index | Unit |
| --- | --- | --- |
| px variance | 0 | m² |
| py variance | 7 | m² |
| pz variance | 14 | m² |
| rx placeholder attitude variance | 21 | rad² |
| ry placeholder attitude variance | 28 | rad² |
| rz placeholder attitude variance | 35 | rad² |

The attitude slots are placeholders. They are not a quaternion covariance.
Velocity covariance is not in this matrix. Unset covariance is unknown.
Thirty-six zeros are a specified zero matrix.

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
9. A covariance entry other than indices 0, 7, 14, 21, 28, and 35 is not exactly zero, when covariance is present and well formed.
10. Position triple incomplete.
11. A present position component is non-finite.
12. Orientation message absent.
13. A present orientation component is non-finite.
14. A present orientation is not unit-norm within `1e-6`.
15. Linear-velocity triple partial (one or two components).
16. A present linear-velocity component is non-finite.
17. Angular-velocity triple partial (one or two components).
18. A present angular-velocity component is non-finite.
19. `source` present and not `SOURCE_VENDOR_INS` or `SOURCE_EXTERNAL_NAV`.

An unengaged message is not a sample. `accepted` requires health state
`VALID` and no structural defect. Absent twist and absent source are not
defects. `DEGRADED` and `INVALID` are not rewritten. Header Validity does
not select `accepted`.

### Fake

`FakeIns` is a pure function of its config and truth. The same seed, bias,
drift, noise, delay, dropout, and invalid-orientation flags produce the
same bytes. `seed` is written to `health.header.sequence` and mixes the
noise draws. Position noise uses `seed`, linear-velocity noise uses
`seed + 1`, and angular-velocity noise uses `seed + 2`. The fake does not
read `FakeImu` outputs.

Default config and truth are the nominal fixture: sequence 42, frame
`ins`, source time `1700000000.250000000`, delay `1s` plus `-250000000` ns
so receive time is `1700000001.0`, position `(12, -4, 0.5)` m, orientation
identity `(0, 0, 0, 1)`, linear velocity `(1.5, 0, -0.25)` m/s, angular
velocity `(0, 0.125, 0)` rad/s, source `SOURCE_VENDOR_INS`, quality
`0.75`, and a specified covariance with diagonals `1`, `4`, `0.25`,
`0.0625`, `0.125`, and `0.5` at indices 0, 7, 14, 21, 28, and 35. Header
validity stays `STATE_VALID`. `source_id` is `nav_sensor`. The clock domain
is monotonic. Sources are `primary` (valid) and `aiding` (unset validity).

| Fault | What the fake emits |
| --- | --- |
| Dropout | No message. Checked first. Absent, not an error. |
| Delay | `receive_time = source_time + delay`. A reversal is emitted and not repaired. |
| Position bias or drift | Added to each position component. Drift is `amplitude * seed`. A non-zero value sets `DEGRADED`, unless invalid orientation applies. |
| Position noise | A non-zero amplitude adds `amplitude * signed_unit(seed)` and sets `DEGRADED`, unless invalid orientation applies. |
| Rate bias or noise | Applied only when that twist triple is present on the truth. A non-zero applicable amplitude sets `DEGRADED`. An absent triple is not invented, and an inapplicable rate fault does not set `DEGRADED`. |
| Invalid orientation | Canonical quaternion `(0, 0, 0, 2)`, health state `INVALID`. Bias, drift, and noise are not applied. Delay is still applied. |

### Evolution

Append fields. Reserve removed tags and names. Preserve unknown fields.
Golden bytes for the nominal solution are fixed in the C++ and Python
serialization tests. Those bytes were captured from a local C++ protobuf
run of `FakeIns` because Bazel was not available in the authoring
environment. Clearing `source` leaves a prefix of that golden. An unknown
source number is preserved and is not accepted. Field 100 is preserved.

Text format example:
[`examples/ins_nominal.textproto`](examples/ins_nominal.textproto).

### INS targets

These targets stay off `.github/baseline/manipulator_targets.tsv`:

- `//intrinsic_hardware/intrinsic/hardware/marine:ins_proto`
- `//intrinsic_hardware/intrinsic/hardware/marine:ins_cc_proto`
- `//intrinsic_hardware/intrinsic/hardware/marine:ins_py_pb2`
- `//intrinsic_hardware/intrinsic/hardware/marine:ins_policy`
- `//intrinsic_hardware/intrinsic/hardware/marine:ins_policy_py`
- `//intrinsic_hardware/intrinsic/hardware/marine:fake_ins`
- `//intrinsic_hardware/intrinsic/hardware/marine:fake_ins_py`
- `//intrinsic_hardware/intrinsic/hardware/marine:ins_policy_test`
- `//intrinsic_hardware/intrinsic/hardware/marine:ins_policy_test_py`
- `//intrinsic_hardware/intrinsic/hardware/marine:ins_serialization_test`
- `//intrinsic_hardware/intrinsic/hardware/marine:ins_serialization_test_py`
- `//intrinsic_hardware/intrinsic/hardware/marine:fake_ins_test`
- `//intrinsic_hardware/intrinsic/hardware/marine:fake_ins_test_py`

## Surface fix

`SurfaceFix` is one GNSS, acoustic, or other surface position fix. It embeds
`MeasurementHealth` as `health`. Position and velocity are in
`health.header.frame_id`. The frame is never inferred from `SurfaceFix`. The
host does not convert lat/lon, WGS84, ENU, or NED, and it does not decide
whether the vehicle is submerged. This message is not `InsSolution`: it has
no orientation and no angular velocity, and it does not reuse or extend the
INS message.

Host checks live in `surface_fix_policy.h` and `surface_fix_policy.py`. They
do not parse protobuf, do not convert frames or geodetic coordinates, do not
read INS solutions, and do not rewrite `health.state` or embodiment
Validity. `FakeSurfaceFix` is a deterministic producer. There is no
estimator, ICON FlatBuffer, Gazebo plugin, or real-hardware path.

| Message | Role |
| --- | --- |
| `SurfaceFix` | One position fix, source, and optional counts, accuracy, and velocity plus health. |
| `SurfaceFix.FixSource` | Measurement-local source: `FIX_SOURCE_UNSPECIFIED=0`, `FIX_SOURCE_GNSS=1`, `FIX_SOURCE_ACOUSTIC=2`, `FIX_SOURCE_OTHER=3`. |

### Position, quality, and optional fields

An engaged sample requires a finite position triple (`position_x_m`,
`position_y_m`, `position_z_m`, meters). Each component is an optional
double, so zero and negative positions are supplied values.

`health.quality` in `[0, 1]` is the fix quality. There is no second quality
field.

`satellite_count` and `beacon_count` are optional `int32`. Unset is absent.
A present value must be `>= 0`, and zero is a supplied count.

`horizontal_accuracy_m` and `vertical_accuracy_m` are optional doubles.
Unset is absent. A present value must be finite and `>= 0`. Zero is
supplied.

`velocity_x_m_s`, `velocity_y_m_s`, and `velocity_z_m_s` are an optional
linear velocity triple. If any component is present, all three must be
present and finite.

### Source

`source` is `optional`. Unset is absent and is not `FIX_SOURCE_UNSPECIFIED`.
An engaged sample needs a set, known source.

| Wire | Host classification |
| --- | --- |
| Field unset | Absent. Rejected when the sample is engaged. |
| `FIX_SOURCE_UNSPECIFIED` (`0`) | Explicit non-source. Rejected when the sample is engaged. Not rewritten. |
| `FIX_SOURCE_GNSS` (`1`) | Known GNSS fix. |
| `FIX_SOURCE_ACOUSTIC` (`2`) | Known acoustic positioning fix. |
| `FIX_SOURCE_OTHER` (`3`) | Known other source. |
| Any other number | Kept on the wire. Not accepted. Not rewritten. |

A producer `VALID` sample with a missing source is rejected by the host and
its `health.state` stays `VALID`.

### Covariance

Covariance is `health.covariance` (`Matrix6`). #17 shape, symmetry, and
finiteness rules apply. When the matrix is present and well formed, only
these diagonal indices may be non-zero. Every other entry must be exactly
`0.0`:

| Slot | Row-major index | Unit |
| --- | --- | --- |
| px variance | 0 | m² |
| py variance | 7 | m² |
| pz variance | 14 | m² |

Unset covariance is unknown. Thirty-six zeros are a specified zero matrix.

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
9. A covariance entry other than indices 0, 7, and 14 is not exactly zero, when covariance is present and well formed.
10. Position triple incomplete.
11. A position component is non-finite.
12. `source` absent, `FIX_SOURCE_UNSPECIFIED`, or an unknown number.
13. A present accuracy is non-finite (horizontal, then vertical), or negative.
14. `satellite_count` or `beacon_count` is present and negative.
15. Velocity triple partial (one or two components).
16. A present velocity component is non-finite.

An unengaged message is not a sample. `accepted` requires health state
`VALID` and no structural defect. Absent optional fields are not defects.
`DEGRADED` and `INVALID` are not rewritten. Header Validity does not select
`accepted`.

### Fake

`FakeSurfaceFix` is a pure function of its config and truth. The same seed,
bias, delay, dropout, and invalid-fix flags produce the same bytes. `seed`
is written to `health.header.sequence`. The fake does not read `FakeIns`
outputs and does not gate on submersion.

Default config and truth are the nominal GNSS fixture: sequence 42, frame
`gnss`, source time `1700000000.250000000`, delay `1s` plus `-250000000` ns
so receive time is `1700000001.0`, position `(12.5, -3.25, 1.0)` m, source
`FIX_SOURCE_GNSS`, `satellite_count` 12, quality `0.75`, and a specified
covariance with diagonals `1`, `4`, and `0.25` at indices 0, 7, and 14.
Header validity stays `STATE_VALID`. `source_id` is `nav_sensor`. The clock
domain is monotonic. Sources are `primary` (valid) and `aiding` (unset
validity).

| Fault | What the fake emits |
| --- | --- |
| Dropout | No message. Checked first. Absent, not an error. |
| Delay | `receive_time = source_time + delay`. A reversal is emitted and not repaired. |
| Bias | Added to each position component. Any non-zero component sets `DEGRADED`, unless an invalid fix applies. The result is not repaired. |
| Invalid fix | Canonical fixture: `source` is set to `FIX_SOURCE_UNSPECIFIED` and health state is `INVALID`. Bias is not applied. Delay is still applied. The host rejects it with a source defect and leaves the state `INVALID`. |

### Evolution

Append fields. Reserve removed tags and names. Preserve unknown fields.
Golden bytes for the nominal GNSS fix are fixed in the C++ and Python
serialization tests. Those bytes were captured from a local run of
`FakeSurfaceFix` and are identical in both languages. Clearing
`satellite_count` leaves a prefix of that golden. An unknown source number
is preserved and is not accepted. Field 100 is preserved.

Text format examples:
[`examples/surface_fix_gnss.textproto`](examples/surface_fix_gnss.textproto)
and
[`examples/surface_fix_acoustic.textproto`](examples/surface_fix_acoustic.textproto).

### Surface fix targets

These targets stay off `.github/baseline/manipulator_targets.tsv`:

- `//intrinsic_hardware/intrinsic/hardware/marine:surface_fix_proto`
- `//intrinsic_hardware/intrinsic/hardware/marine:surface_fix_cc_proto`
- `//intrinsic_hardware/intrinsic/hardware/marine:surface_fix_py_pb2`
- `//intrinsic_hardware/intrinsic/hardware/marine:surface_fix_policy`
- `//intrinsic_hardware/intrinsic/hardware/marine:surface_fix_policy_py`
- `//intrinsic_hardware/intrinsic/hardware/marine:fake_surface_fix`
- `//intrinsic_hardware/intrinsic/hardware/marine:fake_surface_fix_py`
- `//intrinsic_hardware/intrinsic/hardware/marine:surface_fix_policy_test`
- `//intrinsic_hardware/intrinsic/hardware/marine:surface_fix_policy_test_py`
- `//intrinsic_hardware/intrinsic/hardware/marine:surface_fix_serialization_test`
- `//intrinsic_hardware/intrinsic/hardware/marine:surface_fix_serialization_test_py`
- `//intrinsic_hardware/intrinsic/hardware/marine:fake_surface_fix_test`
- `//intrinsic_hardware/intrinsic/hardware/marine:fake_surface_fix_test_py`
