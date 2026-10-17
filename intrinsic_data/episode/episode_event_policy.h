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

#ifndef INTRINSIC_DATA_EPISODE_EPISODE_EVENT_POLICY_H_
#define INTRINSIC_DATA_EPISODE_EPISODE_EVENT_POLICY_H_

#include <cstddef>
#include <cstdint>
#include <string>
#include <string_view>

#include "intrinsic/embodiment/stamped_header_policy.h"
#include "openssl/sha.h"

namespace intrinsic::episode {

// Policy: intrinsic_apis/intrinsic/episode/proto/README.md.
//
// Plain-value checks for EpisodeEvent. These functions do not parse protobuf,
// do not write, read, or replay a stream, do not check sequence advance, do
// not order times, and do not re-validate inner payloads. Consumers use the
// SafetyDecision, VehicleTrajectory, and Inference* policies for those.

// Payload oneof case numbers. They equal the EpisodeEvent field numbers. The
// serialization tests assert this against the generated enum.
inline constexpr int kPayloadCaseNotSet = 0;
inline constexpr int kPayloadCaseSafetyDecision = 10;
inline constexpr int kPayloadCaseVehicleTrajectory = 11;
inline constexpr int kPayloadCaseInferenceEnvelope = 12;
inline constexpr int kPayloadCaseInferenceResult = 13;
inline constexpr int kPayloadCaseEpisodeMetadata = 14;

inline constexpr size_t kSha256HexLength = 64;

// Append-only. Numbers are permanent.
enum class EpisodeEventError {
  kNone = 0,
  kMissingHeader = 1,
  kSourceId = 2,
  kMissingFrame = 3,
  kSourceTime = 4,
  kReceiveTime = 5,
  kSchemaVersion = 6,
  kSnapshotId = 7,
  kConfigDigest = 8,
  kConfigDigestMismatch = 9,
  kMissingPayload = 10,
};

enum class PayloadKind {
  // The payload oneof is not set and there are no unknown fields.
  kNone = 0,
  kSafetyDecision,
  kVehicleTrajectory,
  kInferenceEnvelope,
  kInferenceResult,
  kEpisodeMetadata,
  // The oneof is not set but the message carries unknown fields: a payload
  // case this reader does not know.
  kUnknown,
};

struct EpisodeEventAssessment {
  EpisodeEventError error = EpisodeEventError::kNone;
  PayloadKind payload_kind = PayloadKind::kNone;
  // True when the event is structurally valid and carries a payload this
  // reader does not know. The reader skips it and keeps the unknown bytes.
  bool skip = false;
  // False for an empty message: absent, not an error.
  bool engaged = false;
};

struct EpisodeEventView {
  bool header_present = false;
  std::string_view source_id;
  std::string_view frame_id;
  bool source_time_present = false;
  embodiment::ClockReading source_time;
  bool receive_time_present = false;
  embodiment::ClockReading receive_time;
  uint32_t schema_version = 0;
  std::string_view snapshot_id;
  std::string_view config_digest;
  bool provenance_present = false;
  // The oneof case number, 0 when not set.
  int payload_case = kPayloadCaseNotSet;
  // True when the message carries any unknown field (UnknownFieldSet).
  bool has_unknown_fields = false;
  // EpisodeMetadata.marine_randomization_metadata. Read only for case 14.
  std::string_view marine_randomization_metadata;
};

inline bool IsLowercaseSha256Hex(std::string_view value) {
  if (value.size() != kSha256HexLength) {
    return false;
  }
  for (const char c : value) {
    if (!((c >= '0' && c <= '9') || (c >= 'a' && c <= 'f'))) {
      return false;
    }
  }
  return true;
}

namespace internal {

inline std::string EpisodeSha256Hex(std::string_view input) {
  uint8_t digest[SHA256_DIGEST_LENGTH];
  SHA256(reinterpret_cast<const uint8_t*>(input.data()), input.size(), digest);
  static constexpr char kHex[] = "0123456789abcdef";
  std::string hex;
  hex.reserve(kSha256HexLength);
  for (const uint8_t byte : digest) {
    hex.push_back(kHex[byte >> 4]);
    hex.push_back(kHex[byte & 0x0f]);
  }
  return hex;
}

}  // namespace internal

// Lowercase hex SHA-256 over exactly the bytes of the text. No prefix, suffix,
// or normalization. Covers only the FormatMarineMetadata realized parameters.
// Empty text has no digest: the result is "" (no config recorded).
inline std::string ComputeConfigDigest(std::string_view marine_metadata_text) {
  if (marine_metadata_text.empty()) {
    return "";
  }
  return internal::EpisodeSha256Hex(marine_metadata_text);
}

inline PayloadKind ClassifyPayload(int payload_case, bool has_unknown_fields) {
  switch (payload_case) {
    case kPayloadCaseSafetyDecision:
      return PayloadKind::kSafetyDecision;
    case kPayloadCaseVehicleTrajectory:
      return PayloadKind::kVehicleTrajectory;
    case kPayloadCaseInferenceEnvelope:
      return PayloadKind::kInferenceEnvelope;
    case kPayloadCaseInferenceResult:
      return PayloadKind::kInferenceResult;
    case kPayloadCaseEpisodeMetadata:
      return PayloadKind::kEpisodeMetadata;
    default:
      return has_unknown_fields ? PayloadKind::kUnknown : PayloadKind::kNone;
  }
}

// Any signal engages. An empty message (0 bytes) is not engaged.
inline bool EpisodeEventEngaged(const EpisodeEventView& view) {
  return view.header_present || !view.source_id.empty() ||
         !view.frame_id.empty() || view.source_time_present ||
         view.receive_time_present || view.schema_version != 0 ||
         !view.snapshot_id.empty() || !view.config_digest.empty() ||
         view.provenance_present || view.payload_case != kPayloadCaseNotSet ||
         view.has_unknown_fields;
}

// First failing check wins. An empty message is not engaged: kNone, not an
// error, engaged = false.
inline EpisodeEventAssessment AssessEpisodeEvent(const EpisodeEventView& view) {
  EpisodeEventAssessment assessment;
  if (!EpisodeEventEngaged(view)) {
    return assessment;
  }
  assessment.engaged = true;
  assessment.payload_kind =
      ClassifyPayload(view.payload_case, view.has_unknown_fields);

  if (!view.header_present) {
    assessment.error = EpisodeEventError::kMissingHeader;
  } else if (view.source_id.empty()) {
    assessment.error = EpisodeEventError::kSourceId;
  } else if (view.frame_id.empty()) {
    assessment.error = EpisodeEventError::kMissingFrame;
  } else if (!view.source_time_present ||
             !embodiment::NanosInRange(view.source_time.nanos)) {
    assessment.error = EpisodeEventError::kSourceTime;
  } else if (!view.receive_time_present ||
             !embodiment::NanosInRange(view.receive_time.nanos)) {
    assessment.error = EpisodeEventError::kReceiveTime;
  } else if (view.schema_version < 1) {
    assessment.error = EpisodeEventError::kSchemaVersion;
  } else if (!view.snapshot_id.empty() &&
             !IsLowercaseSha256Hex(view.snapshot_id)) {
    assessment.error = EpisodeEventError::kSnapshotId;
  } else if (!view.config_digest.empty() &&
             !IsLowercaseSha256Hex(view.config_digest)) {
    assessment.error = EpisodeEventError::kConfigDigest;
  } else if (view.payload_case == kPayloadCaseEpisodeMetadata &&
             view.config_digest !=
                 ComputeConfigDigest(view.marine_randomization_metadata)) {
    assessment.error = EpisodeEventError::kConfigDigestMismatch;
  } else if (assessment.payload_kind == PayloadKind::kNone) {
    assessment.error = EpisodeEventError::kMissingPayload;
  } else if (assessment.payload_kind == PayloadKind::kUnknown) {
    assessment.skip = true;
  }
  return assessment;
}

}  // namespace intrinsic::episode

#endif  // INTRINSIC_DATA_EPISODE_EPISODE_EVENT_POLICY_H_
