# Sonar frame contracts

This package holds three opt-in, non-real-time protobuf contracts:
forward-looking sonar (FLS, `fls_frame.proto`, #115), side-scan sonar (SSS,
`sss_frame.proto`, #116), and the multimodal observation bundle
(`observation_bundle.proto`, #117). The sections up to "Side-scan sonar (SSS)
frame contract" describe FLS; the SSS section follows; the multimodal section
is last. FLS and SSS are independent observation types and share only
`StampedHeader`, `Pose`, and `SonarPayloadBlobReference`. The bundle does not
import those frame messages. It references observations by opaque id.

## Forward-looking sonar frame contract

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

### Out of scope

- Side-scan sonar (SSS) frames. They are a separate contract (#116), described
  in the SSS section below.
- A multimodal observation bundle, absent-modality reasons, and
  synchronization tolerances. That contract is `observation_bundle.proto`,
  described in the multimodal section below (#117).
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

### Messages

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

### Units and frames

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

### Grid and payload layout

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

### Validity

`Validity` is the embodiment companion on `header.validity`. The SSS contract
uses the same table.

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

### Host validator

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

### Examples

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

## Side-scan sonar (SSS) frame contract

Opt-in non-real-time protobuf contract for one side-scan sonar frame
(`SideScanSonarFrame`, `sss_frame.proto`). It is the second sonar leaf of the
PDR §8 independent-observation sketch and follows
[ADR 0001](../../../../../docs/adr/0001-multi-embodiment-capability-architecture.md).
Package: `intrinsic_proto.perception.sonar`. It imports `StampedHeader`,
`intrinsic_proto.Pose`, and `fls_frame.proto`, and the last import is only for
`SonarPayloadBlobReference`. That message is reused, not redefined, and no FLS
field number or name changes.

A default message serializes to zero bytes. Callers that never set these
messages keep the previous manipulator behavior, and
`.github/baseline/manipulator_targets.tsv` does not list the SSS targets.

### SSS out of scope

- A multimodal observation bundle, absent-modality reasons, and
  synchronization tolerances. That contract is `observation_bundle.proto`,
  described in the multimodal section below (#117).
- Mosaics, along-track assembly, slant-to-ground conversion, georeferencing,
  detection, tracking, SLAM, occupancy fusion, and CFD. This package never
  turns intensity samples into anything else.
- ICON features, HAL FlatBuffers, Gazebo plugins, real hardware, World
  payloads, and any robot-type enum or `CapabilityDescriptor` change.
- Any change to the FLS messages or the FLS host policy.

### SSS messages

| Message | Role |
| --- | --- |
| `SssChannelGeometry` | Across-track sample window of one channel. |
| `SssGeometry` | Explicit `port` (1) and `starboard` (2) channels. |
| `SssCalibration` | Mounting extrinsic, `parent_from_sensor`. |
| `SssInlineSamples` | Inline intensity samples, one list per channel. |
| `SideScanSonarFrame` | One approved SSS frame. Not a real-time stream. |

`SideScanSonarFrame.payload` is a oneof: `inline_samples` (tag 5) or
`blob_reference` (tag 6, `SonarPayloadBlobReference` from `fls_frame.proto`).
`header` is tag 1, `geometry` 2, `calibration` 3, `sound_speed_m_s` 4, and
`metadata` 100. Field numbers and names are locked. Evolution is append-only.
Reserve removed tags and names.

### SSS units and frames

SI only. Field names carry a unit suffix.

| Field | Unit or rule |
| --- | --- |
| `SssChannelGeometry.min_range_m`, `max_range_m` | meters, across-track, finite, `0 <= min < max`, per channel |
| `SssChannelGeometry.num_samples` | `>= 1` on both channels when engaged |
| `sound_speed_m_s` | meters per second, finite and strictly `> 0` when engaged |
| `SssInlineSamples.port_intensity`, `starboard_intensity` | dimensionless, finite, may be zero |
| `SssCalibration.pose_parent_from_sensor` | meters, Hamilton quaternion stored x, y, z, w; identity is `w = 1`; helpers do not renormalize |
| `header.frame_id` | frame of this SSS observation (sensor acoustic frame for the ping); required non-empty when engaged |
| `calibration.parent_frame_id`, `sensor_frame_id` | explicit; both required, non-empty, and different when calibration is present |

Ranges are sensor-local across-track meters. This package does not convert
ENU and NED, and a frame is never inferred from the message type. Time uses
`StampedHeader` unchanged, with the semantics in the
[embodiment README](../../../embodiment/proto/README.md).

### SSS channels and payload layout

Both `port` and `starboard` are required when the frame is engaged. The
channels are independent: each has its own sample count and range window, and
they need not match. Samples run from `min_range_m` to `max_range_m` of their
channel. `port_intensity.size()` must equal `geometry.port.num_samples` and
`starboard_intensity.size()` must equal `geometry.starboard.num_samples`,
compared as 64-bit values. The sizes are not pooled.

Blob mode carries one `SonarPayloadBlobReference` for the whole frame. It is
opaque. `blob_id` is an id, not a payload. `content_type` is an optional hint
and may be empty. `byte_size` is `optional`: unset is not supplied, and a
present `0` is rejected. The host never fetches the bytes behind `blob_id`.

### SSS host validator

`AssessSideScanSonarFrame` (C++, namespace `intrinsic::perception::sonar`) and
`assess_side_scan_sonar_frame` (Python) take an `SssFrameView` of plain values
(`sss_frame_contract_policy.h`, `sss_frame_contract_policy.py` in
`intrinsic_perception/intrinsic/perception/sonar`) and return an error, the
header validity kind, and `accepted`. `accepted` is true only when the error
is `kNone` and the validity is `STATE_VALID`. The error values and the
`accepted` rule match the FLS contract.

**Not engaged.** A view with no header, both channels at zero samples and zero
ranges, zero sound speed, no calibration, no payload arm, and no metadata is
not engaged. It returns `kNone`, validity absent, and `accepted = false`.

**Engaged.** Any of these engages the frame: header present, any channel
dimension or range non-zero, `sound_speed_m_s` non-zero (NaN counts),
calibration present, a payload arm set, or non-empty metadata. An unset wire
channel and an all-zero channel are the same plain value, so a missing channel
is a `kGeometry` defect.

Checks run in this order and the first defect wins:

| Order | Check | Error |
| --- | --- | --- |
| 1 | `header.frame_id` non-empty | `kMissingFrame` (1) |
| 2 | each channel: `num_samples >= 1`, both ranges finite, `min_range_m >= 0`, `max_range_m > min_range_m` | `kGeometry` (2) |
| 3 | `sound_speed_m_s` finite and `> 0` | `kSoundSpeed` (3) |
| 4 | a payload arm is set | `kMissingPayload` (5) |
| 5 | inline: both channel sizes equal their `num_samples`, checked for both channels first | `kPayloadMismatch` (6) |
| 5 | inline: every sample finite, port scanned before starboard, after the sizes | `kNonFinite` (7) |
| 6 | blob: `blob_id` non-empty; `byte_size`, when set, is `> 0` | `kBlobReference` (8) |
| 7 | calibration present: both frame ids non-empty and different, and the pose is set | `kCalibration` (4) |
| 7 | calibration present: position and quaternion finite | `kNonFinite` (7) |
| 7 | calibration present: quaternion normalized | `kQuaternion` (9) |

An unset calibration pose is `kCalibration`. That check comes before the
finiteness and quaternion checks, so a pose that is missing is never reported
as a bad quaternion. `metadata` is always tolerated, and the validator never
reads its keys or values. Unknown protobuf fields round-trip and are not a
defect. No check reads the bytes behind a blob reference, and no check calls
World, ICON, or a HAL.

### SSS examples

`examples/` holds text-format fixtures that the paired C++ and Python tests
parse and assess.

| Fixture | Result |
| --- | --- |
| `sss_frame_inline_nominal.textproto` | Accepted. Port 16 and starboard 16 samples, 1500 m/s, calibration, metadata. Its wire bytes are the golden in the serialization tests. |
| `sss_frame_blob_nominal.textproto` | Accepted. Same geometry, blob reference, no inline samples. |
| `sss_frame_bad_sound_speed.textproto` | `kSoundSpeed` |
| `sss_frame_payload_mismatch.textproto` | `kPayloadMismatch` |
| `sss_frame_bad_frame.textproto` | `kMissingFrame` |
| `sss_frame_empty_blob_id.textproto` | `kBlobReference` |

Every invalid fixture keeps `STATE_VALID` on the stamp to show that validity
does not repair a structural defect.

## Multimodal observation bundle

Opt-in non-real-time protobuf contract for one `MultimodalObservationBundle`
(`observation_bundle.proto`). It is the third leaf of parent work package
#36 and follows
[ADR 0001](../../../../../docs/adr/0001-multi-embodiment-capability-architecture.md).
PDR §8 allows any subset of modalities and requires a reason when a modality
is stated as absent. Sonar-only is a valid bundle. PDR §15 places sonar and
multimodal contracts in `intrinsic_perception/intrinsic/perception/sonar`.

Package: `intrinsic_proto.perception.sonar`. Imports `StampedHeader` and
`google.protobuf.Duration`. `google.protobuf.Timestamp` is not imported: an
unused proto3 import is an error, and source and receive times live on
`StampedHeader`. The bundle does not import `fls_frame.proto`,
`sss_frame.proto`, or any camera `v1/` proto.

A default message serializes to zero bytes. Callers that never set this
message keep the previous manipulator behavior, and
`.github/baseline/manipulator_targets.tsv` does not list these targets.

### Bundle out of scope

- Inference, OIP, model weights, beamforming, mosaics, SLAM, and detection.
- Embedding FLS, SSS, camera, point-cloud, VehicleState, or World payloads.
  Slots carry an opaque `reference_id` only. The host does not fetch it.
- ICON features, HAL FlatBuffers, Gazebo plugins, and real hardware.
- Any change to FLS or SSS field numbers, the World descriptor digest, or
  camera `v1/` messages. There is no robot-type enum.

### Reference fields

`ObservationSlot` inlines the opaque reference at the locked field numbers:
`reference_id` (2), `content_type` (3), and optional `byte_size` (4). That is
the same discipline as `SonarPayloadBlobReference` (id, optional content type,
optional byte size). The FLS message names the id `blob_id`, and reusing it
would not match the locked slot fields, so this file does not import it and
does not add a third blob message.

A slot is present iff `reference_id` is non-empty. The header is then
required. A slot message with an empty `reference_id` is not present, even if
other slot fields are set.

`StateReference` is present iff the message is set and `header.frame_id` is
non-empty. `state_epoch` may be 0. An empty `world_snapshot_id` means there is
no World snapshot latch. A non-empty id must be 64 lowercase hex characters,
the form of `WorldSnapshotDescriptor.snapshot_id`.

### Messages

| Message | Role |
| --- | --- |
| `ObservationModality` | FLS, SSS, optical, point cloud, or vehicle state. |
| `AbsentModalityReason` | Why a modality is intentionally absent. |
| `AbsentModalityEntry` | One absent modality and its reason. |
| `ObservationSlot` | Stamped identity plus an opaque reference. |
| `StateReference` | VehicleState epoch and optional World snapshot id. |
| `MultimodalObservationBundle` | One approved bundle. Not a real-time stream. |

Field numbers and names are locked. Evolution is append-only. Reserve removed
tags and names. `metadata` is tag 100.

`header` is tag 1, `max_skew` 2, `fls` 3, `sss` 4, `optical` 5, `point_cloud`
6, `vehicle_state` 7, and `absent` 8.

### Sync, frames, and time

SI only. `max_skew` is a `Duration`. Fixture examples use 200ms. The host does
not fill that value in when the field is unset.

| Rule | Detail |
| --- | --- |
| Engaged | Any present slot, any absent entry, a non-zero `max_skew`, non-empty metadata, or a present bundle header. A present zero `max_skew` does not engage by itself. |
| Not engaged | Default or empty. Error `kNone`, validity absent, `accepted` false. |
| Bundle `header.frame_id` | Required and non-empty when engaged. It is a sync frame and need not equal a sensor frame. |
| `max_skew` | Required when engaged. Seconds `>= 0`. Nanos in `[0, 1e9)`. |
| Present slot | `header.frame_id` non-empty, `header.source_time` set with nanos in range, `reference_id` non-empty. `byte_size`, when set, is `> 0`. `content_type` may be empty. Slot validity is not assessed. |
| Present state | `header.frame_id` non-empty, `header.source_time` set with nanos in range. Snapshot id, when non-empty, is 64 lowercase hex. |
| Clock domain | Every present slot and the present state must use the same `clock_domain` string as the bundle header. Empty matches empty. |
| World frames | If two present observation slots both use `world_enu` or `world_ned` and those ids differ, that is a frame mismatch. Sensor-local frames may differ. `StateReference` is not an observation slot. |
| Skew set | `source_time` of every present observation slot and of the present state. The bundle header time is not in the set. |
| Skew | `measured_skew = max(source_time) - min(source_time)`. Equal to `max_skew` is within the limit. Strictly greater is excessive. One sample measures zero. Zero samples leave the measured skew unset at zero. |
| Source-time reversal | On the bundle header or any present slot or state, when both `source_time` and `receive_time` are set: `receive_time < source_time` is a reversal. A set timestamp whose nanos are outside `[0, 1e9)` cannot be ordered and is the same defect. Equal times are not a reversal. |
| Absent list | Each entry has a modality and a reason in the approved range (not UNSPECIFIED, not an unknown number). The same modality must not appear twice. A modality that is present must not also be absent. |
| Subset | A modality omitted from both the present slots and `absent` is allowed. Sonar-only and camera-only fixtures still list every other modality so a missing reason is visible in the examples. |
| All-absent | Engaged, no present slot, and a non-empty valid `absent` list can be accepted. Engaged with no present slot and an empty `absent` list is `kSlot`. |
| Validity | Only the bundle header `Validity` is classified. `accepted` is true only when the error is `kNone` and the state is `STATE_VALID`. |

### Host validator

`AssessMultimodalObservationBundle` (C++, namespace
`intrinsic::perception::sonar`) and
`assess_multimodal_observation_bundle` (Python) take an
`ObservationBundleView` of plain values
(`observation_bundle_contract_policy.h`,
`observation_bundle_contract_policy.py`) and return an error, the header
validity kind, `accepted`, and the measured skew as seconds and nanos.
`accepted` is true only when the error is `kNone` and the validity is
`STATE_VALID`.

`BuildMultimodalObservationBundle` and
`build_multimodal_observation_bundle` copy caller-supplied fields onto the
wire message. They do not fetch references and they do not rewrite ids, times,
or snapshot ids.

Checks run in this order and the first defect wins. A defect before the skew
step leaves the measured skew at zero.

| Order | Check | Error |
| --- | --- | --- |
| 1 | bundle `header.frame_id` non-empty | `kMissingFrame` (1) |
| 2 | `max_skew` present, seconds `>= 0`, nanos in range | `kMaxSkew` (2) |
| 3 | absent entries: known modality, known reason, no duplicate, no overlap with a present modality | `kAbsentList` (4) |
| 4 | at least one present slot or state, unless `absent` is non-empty | `kSlot` (3) |
| 4 | each present slot: frame, source time, then reference and `byte_size` | `kSlot` (3) or `kReference` (10) |
| 5 | present state: frame and source time, then snapshot id when set | `kSlot` (3) or `kSnapshotId` (9) |
| 6 | clock domains agree | `kClockDomain` (5) |
| 7 | well-known world ids on present slots agree | `kFrameMismatch` (6) |
| 8 | no source-time reversal | `kTimeReversal` (8) |
| 9 | measured skew `<= max_skew` | `kExcessiveSkew` (7) |

`metadata` is always tolerated. Unknown protobuf fields round-trip and are not
a defect. No check reads the bytes behind a reference, and no check calls
World, ICON, or a HAL. No check runs a model.

### Bundle examples

`examples/` holds text-format fixtures that the paired C++ and Python tests
parse and assess. Invalid fixtures keep `STATE_VALID` so validity does not
repair a structural defect.

| Fixture | Result |
| --- | --- |
| `observation_bundle_sonar_only.textproto` | Accepted. FLS present. SSS, optical, point cloud, and vehicle state absent with approved reasons. Its wire bytes are the golden in the serialization tests. |
| `observation_bundle_camera_only.textproto` | Accepted. Optical present. The other modalities are absent with approved reasons. |
| `observation_bundle_all_modalities.textproto` | Accepted. All five modalities present. Measured skew is 180ms. Snapshot id is 64 lowercase hex. |
| `observation_bundle_excessive_skew.textproto` | `kExcessiveSkew`. FLS and SSS differ by 200ms + 1ns. |
| `observation_bundle_frame_mismatch.textproto` | `kFrameMismatch`. Present slots use `world_enu` and `world_ned`. |
| `observation_bundle_time_reversal.textproto` | `kTimeReversal`. FLS `receive_time` is before `source_time`. |
| `observation_bundle_missing_absent_reason.textproto` | `kAbsentList`. FLS is present and also listed in `absent`. |
| `observation_bundle_bad_snapshot_id.textproto` | `kSnapshotId`. The snapshot id is not 64 lowercase hex. |

Unit tests, not these files, cover the exact `max_skew` boundary, the boundary
minus 1ns, a modality omitted from both sides, and an all-absent bundle.
