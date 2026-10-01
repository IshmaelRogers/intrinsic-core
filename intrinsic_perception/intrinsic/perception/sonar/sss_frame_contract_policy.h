// Copyright 2026 Intrinsic Innovation LLC
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     https://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef INTRINSIC_PERCEPTION_SONAR_SSS_FRAME_CONTRACT_POLICY_H_
#define INTRINSIC_PERCEPTION_SONAR_SSS_FRAME_CONTRACT_POLICY_H_

#include <cmath>
#include <cstdint>
#include <span>
#include <string_view>

#include "intrinsic/embodiment/frame_policy.h"
#include "intrinsic/embodiment/stamped_header_policy.h"

namespace intrinsic::perception::sonar {

// Policy: intrinsic_apis/intrinsic/perception/proto/sonar/README.md.
//
// Plain-value checks for SideScanSonarFrame. These functions do not parse
// protobuf, do not fetch blobs, do not build mosaics, convert slant range,
// georeference, convert frames, or call World, ICON, or a HAL.

enum class SssFrameContractError {
  kNone = 0,
  kMissingFrame = 1,
  kGeometry = 2,
  kSoundSpeed = 3,
  kCalibration = 4,
  kMissingPayload = 5,
  kPayloadMismatch = 6,
  kNonFinite = 7,
  kBlobReference = 8,
  kQuaternion = 9,
};

struct SssFrameContractAssessment {
  SssFrameContractError error = SssFrameContractError::kNone;
  embodiment::ValidityKind validity = embodiment::ValidityKind::kAbsent;
  // True only when error == kNone and the header validity is STATE_VALID.
  // An empty message is not accepted and is not an error.
  bool accepted = false;
};

// Which arm of the payload oneof is set. kNone means the oneof is unset.
enum class SssPayloadMode {
  kNone = 0,
  kInline = 1,
  kBlob = 2,
};

// One across-track channel. An unset wire channel and an all-zero channel are
// the same plain value.
struct SssChannelGeometryView {
  uint32_t num_samples = 0;
  double min_range_m = 0;
  double max_range_m = 0;
};

struct SssGeometryView {
  SssChannelGeometryView port;
  SssChannelGeometryView starboard;
};

// Pose is parent_from_sensor. pose_present is false when the wire pose
// message is unset.
struct SssCalibrationView {
  bool present = false;
  std::string_view parent_frame_id;
  std::string_view sensor_frame_id;
  bool pose_present = false;
  embodiment::Vec3 position;
  embodiment::Quaternion orientation;
};

struct SssFrameView {
  bool header_present = false;
  bool validity_present = false;
  int validity_state = 0;
  std::string_view frame_id;
  SssGeometryView geometry;
  SssCalibrationView calibration;
  double sound_speed_m_s = 0;
  SssPayloadMode payload_mode = SssPayloadMode::kNone;
  // Inline mode. Valid only while the caller's storage is alive.
  std::span<const float> port_intensity;
  std::span<const float> starboard_intensity;
  // Blob mode. The bytes behind blob_id are never fetched.
  std::string_view blob_id;
  bool blob_byte_size_present = false;
  uint64_t blob_byte_size = 0;
  // True when the metadata map has any entry. Values are not inspected.
  bool metadata_present = false;
};

inline bool SssChannelNonDefault(const SssChannelGeometryView &channel) {
  return channel.num_samples != 0 || channel.min_range_m != 0 ||
         channel.max_range_m != 0;
}

inline bool SssGeometryNonDefault(const SssGeometryView &geometry) {
  return SssChannelNonDefault(geometry.port) ||
         SssChannelNonDefault(geometry.starboard);
}

// Engaged when any of: header present, any channel dimension or range set,
// sound speed set, calibration present, payload oneof set, or metadata
// non-empty. NaN compares unequal to zero, so a NaN field engages the frame.
inline bool SssFrameEngaged(const SssFrameView &frame) {
  return frame.header_present || SssGeometryNonDefault(frame.geometry) ||
         frame.sound_speed_m_s != 0 || frame.calibration.present ||
         frame.payload_mode != SssPayloadMode::kNone || frame.metadata_present;
}

inline bool SssChannelOk(const SssChannelGeometryView &channel) {
  return channel.num_samples >= 1 &&
         embodiment::IsFinite(channel.min_range_m) &&
         embodiment::IsFinite(channel.max_range_m) &&
         channel.min_range_m >= 0.0 &&
         channel.max_range_m > channel.min_range_m;
}

inline bool SssGeometryOk(const SssGeometryView &geometry) {
  return SssChannelOk(geometry.port) && SssChannelOk(geometry.starboard);
}

inline bool SssSoundSpeedOk(double sound_speed_m_s) {
  return embodiment::IsFinite(sound_speed_m_s) && sound_speed_m_s > 0.0;
}

inline SssFrameContractAssessment
MakeSssAssessment(SssFrameContractError error,
                  embodiment::ValidityKind validity) {
  const bool accepted = error == SssFrameContractError::kNone &&
                        embodiment::SampleAccepted(validity, true);
  return SssFrameContractAssessment{error, validity, accepted};
}

inline bool SssSamplesFinite(std::span<const float> samples) {
  for (const float sample : samples) {
    if (!std::isfinite(sample)) {
      return false;
    }
  }
  return true;
}

// Inline mode: both size checks come first, then the sample scan (port, then
// starboard). Sizes are compared as 64-bit values.
inline SssFrameContractError AssessSssInlinePayload(const SssFrameView &frame) {
  if (static_cast<uint64_t>(frame.port_intensity.size()) !=
          static_cast<uint64_t>(frame.geometry.port.num_samples) ||
      static_cast<uint64_t>(frame.starboard_intensity.size()) !=
          static_cast<uint64_t>(frame.geometry.starboard.num_samples)) {
    return SssFrameContractError::kPayloadMismatch;
  }
  if (!SssSamplesFinite(frame.port_intensity) ||
      !SssSamplesFinite(frame.starboard_intensity)) {
    return SssFrameContractError::kNonFinite;
  }
  return SssFrameContractError::kNone;
}

inline SssFrameContractError AssessSssBlobPayload(const SssFrameView &frame) {
  if (frame.blob_id.empty() ||
      (frame.blob_byte_size_present && frame.blob_byte_size == 0)) {
    return SssFrameContractError::kBlobReference;
  }
  return SssFrameContractError::kNone;
}

inline SssFrameContractError
AssessSssCalibration(const SssCalibrationView &calibration) {
  if (calibration.parent_frame_id.empty() ||
      calibration.sensor_frame_id.empty() ||
      calibration.parent_frame_id == calibration.sensor_frame_id ||
      !calibration.pose_present) {
    return SssFrameContractError::kCalibration;
  }
  if (!embodiment::IsFinite(calibration.position) ||
      !embodiment::IsFinite(calibration.orientation)) {
    return SssFrameContractError::kNonFinite;
  }
  if (!embodiment::IsNormalized(calibration.orientation)) {
    return SssFrameContractError::kQuaternion;
  }
  return SssFrameContractError::kNone;
}

// First defect wins. Order: frame id, geometry, sound speed, payload
// presence, payload content (inline sizes then samples, or blob reference),
// then calibration when present. Metadata is never a defect. An empty view is
// not engaged.
inline SssFrameContractAssessment
AssessSideScanSonarFrame(const SssFrameView &frame) {
  if (!SssFrameEngaged(frame)) {
    return SssFrameContractAssessment{};
  }
  const embodiment::ValidityKind validity = embodiment::ClassifyValidity(
      frame.validity_present, frame.validity_state);
  SssFrameContractError error = SssFrameContractError::kNone;
  if (frame.frame_id.empty()) {
    error = SssFrameContractError::kMissingFrame;
  } else if (!SssGeometryOk(frame.geometry)) {
    error = SssFrameContractError::kGeometry;
  } else if (!SssSoundSpeedOk(frame.sound_speed_m_s)) {
    error = SssFrameContractError::kSoundSpeed;
  } else if (frame.payload_mode == SssPayloadMode::kNone) {
    error = SssFrameContractError::kMissingPayload;
  } else if (frame.payload_mode == SssPayloadMode::kInline) {
    error = AssessSssInlinePayload(frame);
  } else {
    error = AssessSssBlobPayload(frame);
  }
  if (error == SssFrameContractError::kNone && frame.calibration.present) {
    error = AssessSssCalibration(frame.calibration);
  }
  return MakeSssAssessment(error, validity);
}

} // namespace intrinsic::perception::sonar

#endif // INTRINSIC_PERCEPTION_SONAR_SSS_FRAME_CONTRACT_POLICY_H_
