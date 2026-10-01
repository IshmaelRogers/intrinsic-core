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

#ifndef INTRINSIC_PERCEPTION_SONAR_FLS_FRAME_CONTRACT_POLICY_H_
#define INTRINSIC_PERCEPTION_SONAR_FLS_FRAME_CONTRACT_POLICY_H_

#include <cmath>
#include <cstdint>
#include <limits>
#include <span>
#include <string_view>

#include "intrinsic/embodiment/frame_policy.h"
#include "intrinsic/embodiment/stamped_header_policy.h"

namespace intrinsic::perception::sonar {

// Policy: intrinsic_apis/intrinsic/perception/proto/sonar/README.md.
//
// Plain-value checks for ForwardLookingSonarFrame. These functions do not
// parse protobuf, do not fetch blobs, do not beamform or detect, do not
// convert frames, and do not call World, ICON, or a HAL.

enum class FlsFrameContractError {
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

struct FlsFrameContractAssessment {
  FlsFrameContractError error = FlsFrameContractError::kNone;
  embodiment::ValidityKind validity = embodiment::ValidityKind::kAbsent;
  // True only when error == kNone and the header validity is STATE_VALID.
  // An empty message is not accepted and is not an error.
  bool accepted = false;
};

// Which arm of the payload oneof is set. kNone means the oneof is unset.
enum class FlsPayloadMode {
  kNone = 0,
  kInline = 1,
  kBlob = 2,
};

struct FlsGeometryView {
  uint32_t num_beams = 0;
  uint32_t num_range_bins = 0;
  double min_range_m = 0;
  double max_range_m = 0;
  double min_bearing_rad = 0;
  double max_bearing_rad = 0;
};

// Pose is parent_from_sensor. pose_present is false when the wire pose
// message is unset.
struct FlsCalibrationView {
  bool present = false;
  std::string_view parent_frame_id;
  std::string_view sensor_frame_id;
  bool pose_present = false;
  embodiment::Vec3 position;
  embodiment::Quaternion orientation;
};

struct FlsFrameView {
  bool header_present = false;
  bool validity_present = false;
  int validity_state = 0;
  std::string_view frame_id;
  FlsGeometryView geometry;
  FlsCalibrationView calibration;
  double sound_speed_m_s = 0;
  FlsPayloadMode payload_mode = FlsPayloadMode::kNone;
  // Inline mode. Valid only while the caller's storage is alive.
  std::span<const float> intensity;
  // Blob mode. The bytes behind blob_id are never fetched.
  std::string_view blob_id;
  bool blob_byte_size_present = false;
  uint64_t blob_byte_size = 0;
  // True when the metadata map has any entry. Values are not inspected.
  bool metadata_present = false;
};

inline bool FlsGeometryNonDefault(const FlsGeometryView& geometry) {
  return geometry.num_beams != 0 || geometry.num_range_bins != 0 ||
         geometry.min_range_m != 0 || geometry.max_range_m != 0 ||
         geometry.min_bearing_rad != 0 || geometry.max_bearing_rad != 0;
}

// Engaged when any of: header present, geometry dimensions or bounds set,
// sound speed set, calibration present, payload oneof set, or metadata
// non-empty. NaN compares unequal to zero, so a NaN field engages the frame.
inline bool FlsFrameEngaged(const FlsFrameView& frame) {
  return frame.header_present || FlsGeometryNonDefault(frame.geometry) ||
         frame.sound_speed_m_s != 0 || frame.calibration.present ||
         frame.payload_mode != FlsPayloadMode::kNone || frame.metadata_present;
}

inline bool FlsGeometryOk(const FlsGeometryView& geometry) {
  return geometry.num_beams >= 1 && geometry.num_range_bins >= 1 &&
         embodiment::IsFinite(geometry.min_range_m) &&
         embodiment::IsFinite(geometry.max_range_m) &&
         embodiment::IsFinite(geometry.min_bearing_rad) &&
         embodiment::IsFinite(geometry.max_bearing_rad) &&
         geometry.min_range_m >= 0.0 &&
         geometry.max_range_m > geometry.min_range_m &&
         geometry.max_bearing_rad >= geometry.min_bearing_rad;
}

inline bool SoundSpeedOk(double sound_speed_m_s) {
  return embodiment::IsFinite(sound_speed_m_s) && sound_speed_m_s > 0.0;
}

// num_beams * num_range_bins in 64 bits. Returns false on overflow.
inline bool ExpectedSampleCount(const FlsGeometryView& geometry,
                                uint64_t* count) {
  const uint64_t beams = geometry.num_beams;
  const uint64_t bins = geometry.num_range_bins;
  if (beams != 0 && bins > std::numeric_limits<uint64_t>::max() / beams) {
    return false;
  }
  *count = beams * bins;
  return true;
}

inline FlsFrameContractAssessment MakeFlsAssessment(
    FlsFrameContractError error, embodiment::ValidityKind validity) {
  const bool accepted = error == FlsFrameContractError::kNone &&
                        embodiment::SampleAccepted(validity, true);
  return FlsFrameContractAssessment{error, validity, accepted};
}

// Inline mode: the size check comes first, then the sample scan.
inline FlsFrameContractError AssessInlinePayload(const FlsFrameView& frame) {
  uint64_t expected = 0;
  if (!ExpectedSampleCount(frame.geometry, &expected) ||
      static_cast<uint64_t>(frame.intensity.size()) != expected) {
    return FlsFrameContractError::kPayloadMismatch;
  }
  for (const float sample : frame.intensity) {
    if (!std::isfinite(sample)) {
      return FlsFrameContractError::kNonFinite;
    }
  }
  return FlsFrameContractError::kNone;
}

inline FlsFrameContractError AssessBlobPayload(const FlsFrameView& frame) {
  if (frame.blob_id.empty() ||
      (frame.blob_byte_size_present && frame.blob_byte_size == 0)) {
    return FlsFrameContractError::kBlobReference;
  }
  return FlsFrameContractError::kNone;
}

inline FlsFrameContractError AssessCalibration(
    const FlsCalibrationView& calibration) {
  if (calibration.parent_frame_id.empty() ||
      calibration.sensor_frame_id.empty() ||
      calibration.parent_frame_id == calibration.sensor_frame_id ||
      !calibration.pose_present) {
    return FlsFrameContractError::kCalibration;
  }
  if (!embodiment::IsFinite(calibration.position) ||
      !embodiment::IsFinite(calibration.orientation)) {
    return FlsFrameContractError::kNonFinite;
  }
  if (!embodiment::IsNormalized(calibration.orientation)) {
    return FlsFrameContractError::kQuaternion;
  }
  return FlsFrameContractError::kNone;
}

// First defect wins. Order: frame id, geometry, sound speed, payload
// presence, payload content (inline size then samples, or blob reference),
// then calibration when present. Metadata is never a defect. An empty view is
// not engaged.
inline FlsFrameContractAssessment AssessForwardLookingSonarFrame(
    const FlsFrameView& frame) {
  if (!FlsFrameEngaged(frame)) {
    return FlsFrameContractAssessment{};
  }
  const embodiment::ValidityKind validity = embodiment::ClassifyValidity(
      frame.validity_present, frame.validity_state);
  FlsFrameContractError error = FlsFrameContractError::kNone;
  if (frame.frame_id.empty()) {
    error = FlsFrameContractError::kMissingFrame;
  } else if (!FlsGeometryOk(frame.geometry)) {
    error = FlsFrameContractError::kGeometry;
  } else if (!SoundSpeedOk(frame.sound_speed_m_s)) {
    error = FlsFrameContractError::kSoundSpeed;
  } else if (frame.payload_mode == FlsPayloadMode::kNone) {
    error = FlsFrameContractError::kMissingPayload;
  } else if (frame.payload_mode == FlsPayloadMode::kInline) {
    error = AssessInlinePayload(frame);
  } else {
    error = AssessBlobPayload(frame);
  }
  if (error == FlsFrameContractError::kNone && frame.calibration.present) {
    error = AssessCalibration(frame.calibration);
  }
  return MakeFlsAssessment(error, validity);
}

}  // namespace intrinsic::perception::sonar

#endif  // INTRINSIC_PERCEPTION_SONAR_FLS_FRAME_CONTRACT_POLICY_H_
