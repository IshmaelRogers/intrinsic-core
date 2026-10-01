# Forward-looking sonar frame contract

Opt-in non-real-time protobuf contract for one forward-looking sonar (FLS)
frame. It implements the "independent observation type with geometry,
calibration, sound-speed context, and blob reference" sketch of PDR §8 and
follows [ADR 0001](../../../../../docs/adr/0001-multi-embodiment-capability-architecture.md).

PDR §15 places sonar contracts in `intrinsic_perception/intrinsic/perception/sonar`.
The proto lives here, in a new `sonar` package. It does not extend the camera
`v1/` perception protos and it does not import the logging `BlobReference`.

Package: `intrinsic_proto.perception.sonar`. Imports `StampedHeader`
(`intrinsic/embodiment/proto/stamped_header.proto`) and `intrinsic_proto.Pose`
(`intrinsic/math/proto/pose.proto`).

## Out of scope

- Side-scan sonar (SSS) frames. They are a separate contract (#116).
- A multimodal observation bundle, absent-modality reasons, and
  synchronization tolerances (#117).
- Beamforming, matched filtering, detection, tracking, SLAM, occupancy
  fusion, and CFD. This package never turns intensity samples into ranges.
- ICON features, HAL FlatBuffers, Gazebo plugins, real hardware, World
  payloads, and any robot-type enum or `CapabilityDescriptor` change.

Manipulator joint, Cartesian, kinematics, motion-planning, World, and Gazebo
contracts are unchanged. `.github/baseline/manipulator_targets.tsv` does not
list these targets. A default message serializes to zero bytes. Callers that
never set these messages keep the previous manipulator behavior.

Host checks live in `intrinsic_perception/intrinsic/perception/sonar`
(`fls_frame_contract_policy.h`, `fls_frame_contract_policy.py`). They take
plain values, do not parse protobuf, do not fetch blobs, and do not convert
frames.

## Messages

| Message | Role |
| --- | --- |
| `FlsGeometry` | Polar beam by range-bin grid of one ping. |
| `FlsCalibration` | Mounting extrinsic, `parent_from_sensor`. |
| `FlsInlineSamples` | Inline intensity samples, beam-major. |
| `SonarPayloadBlobReference` | Opaque id of off-wire payload bytes. |
| `ForwardLookingSonarFrame` | One approved FLS frame. Not a real-time stream. |

`ForwardLookingSonarFrame.payload` is a oneof: `inline_samples` (tag 5) or
`blob_reference` (tag 6). Field numbers and names are locked. Evolution is
append-only. Reserve removed tags and names. `metadata` is tag 100.

## Units and frames

SI only. Field names carry a unit suffix.

| Field | Unit or rule |
| --- | --- |
| `FlsGeometry.min_range_m`, `max_range_m` | meters, finite, `0 <= min < max` |
| `FlsGeometry.min_bearing_rad`, `max_bearing_rad` | radians, finite, `min <= max`, sensor-forward is 0 |
| `sound_speed_m_s` | meters per second, finite and strictly `> 0` when engaged |
| `FlsInlineSamples.intensity` | dimensionless, finite, may be zero |
| `FlsCalibration.pose_parent_from_sensor` | meters, Hamilton quaternion stored x, y, z, w; identity is `w = 1`; helpers do not renormalize |
| `header.frame_id` | frame of this FLS observation; required non-empty when engaged |
| `calibration.parent_frame_id`, `sensor_frame_id` | explicit; both required, non-empty, and different when calibration is present |

Bearings are sensor-local polar coordinates. Left-handed versus right-handed
conventions are not converted, and this package does not convert ENU and NED.
A frame is never inferred from the message type. World ids `world_enu` and
`world_ned` are allowed in `header.frame_id` when the producer stamps in the
world, but a sensor frame id is typical.

Time uses `StampedHeader` unchanged: `sequence`, `source_time`,
`receive_time`, `source_id`, and `clock_domain` have the semantics in the
[embodiment README](../../../embodiment/proto/README.md). This package adds no
clock or tolerance rule and does not rewrite a stamp.

## Grid and payload layout

`num_beams` beams span `[min_bearing_rad, max_bearing_rad]`, with the first and
last beam centers on the bounds. `num_range_bins` bins span
`[min_range_m, max_range_m]`. Inline samples are beam-major, then range bin:
`index = beam * num_range_bins + bin`. The sample count must equal
`num_beams * num_range_bins`, computed in 64 bits. An overflowing product is a
mismatch.

Blob mode carries only `SonarPayloadBlobReference`. `blob_id` is an opaque id,
not a payload and not a covariance. `content_type` is an optional hint and may
be empty. `byte_size` is `optional`: unset is not supplied, and a present `0`
is rejected. The host never fetches the bytes behind `blob_id`.

## Validity

`Validity` is the embodiment companion on `header.validity`.

| Observation | Meaning |
| --- | --- |
| Validity field missing | Absent. No judgment was supplied. Not accepted. |
| `STATE_UNSPECIFIED` | Judgment present, unclassified. Not accepted. |
| `STATE_VALID` | Explicitly usable, still subject to every rule below. |
| `STATE_INVALID` | Explicit rejection. |
| Unknown state number | Kept on the wire. Classified unspecified. Not accepted. |

`STATE_VALID` does not repair a non-finite number or a geometry or payload
mismatch. The assessment reports the structural error and the validity
separately.

## Host validator

`AssessForwardLookingSonarFrame` (C++, namespace
`intrinsic::perception::sonar`) and `assess_forward_looking_sonar_frame`
(Python) take an `FlsFrameView` of plain values and return an error, the
header validity kind, and `accepted`. `accepted` is true only when the error
is `kNone` and the validity is `STATE_VALID`.

**Not engaged.** A view with no header, zero geometry dimensions and bounds,
zero sound speed, no calibration, no payload arm, and no metadata is not
engaged. It returns `kNone`, validity absent, and `accepted = false`. This is
the same rule as an empty `MarineComponentValidity` or `VehicleTrajectory`.

**Engaged.** Any of these engages the frame: header present, any geometry
dimension or bound non-zero, `sound_speed_m_s` non-zero (NaN counts), calibration
present, a payload arm set, or non-empty metadata.

Checks run in this order and the first defect wins:

| Order | Check | Error |
| --- | --- | --- |
| 1 | `header.frame_id` non-empty | `kMissingFrame` (1) |
| 2 | `num_beams >= 1`, `num_range_bins >= 1`, four doubles finite, `min_range_m >= 0`, `max_range_m > min_range_m`, `max_bearing_rad >= min_bearing_rad` | `kGeometry` (2) |
| 3 | `sound_speed_m_s` finite and `> 0` | `kSoundSpeed` (3) |
| 4 | a payload arm is set | `kMissingPayload` (5) |
| 5 | inline: sample count equals `num_beams * num_range_bins` | `kPayloadMismatch` (6) |
| 5 | inline: every sample finite, checked after the size | `kNonFinite` (7) |
| 6 | blob: `blob_id` non-empty; `byte_size`, when set, is `> 0` | `kBlobReference` (8) |
| 7 | calibration present: both frame ids non-empty and different, and the pose is set | `kCalibration` (4) |
| 7 | calibration present: position and quaternion finite | `kNonFinite` (7) |
| 7 | calibration present: quaternion normalized | `kQuaternion` (9) |

`metadata` is always tolerated. The validator never reads its keys or values.
Unknown protobuf fields round-trip and are not a defect. No check reads the
bytes behind a blob reference, and no check calls World, ICON, or a HAL.

## Examples

`examples/` holds text-format fixtures that the paired C++ and Python tests
parse and assess.

| Fixture | Result |
| --- | --- |
| `fls_frame_inline_nominal.textproto` | Accepted. 4 beams by 8 bins, 32 samples, 1500 m/s, calibration, metadata. Its wire bytes are the golden in the serialization tests. |
| `fls_frame_blob_nominal.textproto` | Accepted. Same geometry, blob reference, no inline samples. |
| `fls_frame_bad_sound_speed.textproto` | `kSoundSpeed` |
| `fls_frame_payload_mismatch.textproto` | `kPayloadMismatch` |
| `fls_frame_bad_frame.textproto` | `kMissingFrame` |
| `fls_frame_empty_blob_id.textproto` | `kBlobReference` |

Every invalid fixture keeps `STATE_VALID` on the stamp to show that validity
does not repair a structural defect.
