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

#ifndef INTRINSIC_PERCEPTION_SONAR_OBSERVATION_BUNDLE_CONTRACT_POLICY_H_
#define INTRINSIC_PERCEPTION_SONAR_OBSERVATION_BUNDLE_CONTRACT_POLICY_H_

#include <cstdint>
#include <limits>
#include <span>
#include <string_view>

#include "intrinsic/embodiment/frame_policy.h"
#include "intrinsic/embodiment/stamped_header_policy.h"

namespace intrinsic::perception::sonar {

// Policy: intrinsic_apis/intrinsic/perception/proto/sonar/README.md.
//
// Plain-value checks for MultimodalObservationBundle. These functions do not
// parse protobuf, do not fetch references, do not run inference, do not
// beamform, and do not call World, ICON, or a HAL.

// Fixture examples use 200ms. The assessor does not apply this when max_skew
// is unset.
inline constexpr int64_t kFixtureMaxSkewSeconds = 0;
inline constexpr int32_t kFixtureMaxSkewNanos = 200000000;

enum class ObservationModality {
  kUnspecified = 0,
  kSonarFls = 1,
  kSonarSss = 2,
  kOptical = 3,
  kPointCloud = 4,
  kVehicleState = 5,
};

enum class AbsentModalityReason {
  kUnspecified = 0,
  kNotConfigured = 1,
  kSensorOffline = 2,
  kOutOfRange = 3,
  kDroppedForSkew = 4,
  kIntentionallyOmitted = 5,
};

enum class ObservationBundleContractError {
  kNone = 0,
  kMissingFrame = 1,
  kMaxSkew = 2,
  kSlot = 3,
  kAbsentList = 4,
  kClockDomain = 5,
  kFrameMismatch = 6,
  kExcessiveSkew = 7,
  kTimeReversal = 8,
  kSnapshotId = 9,
  kReference = 10,
};

// Seconds and nanos, matching google.protobuf.Duration on the wire.
struct DurationParts {
  int64_t seconds = 0;
  int32_t nanos = 0;
};

struct ObservationBundleContractAssessment {
  ObservationBundleContractError error = ObservationBundleContractError::kNone;
  embodiment::ValidityKind validity = embodiment::ValidityKind::kAbsent;
  // True only when error == kNone and the bundle header validity is
  // STATE_VALID. An empty message is not accepted and is not an error.
  bool accepted = false;
  // True when the skew set has at least one source_time and the check
  // reached the skew step. Earlier defects leave this false and the duration
  // at zero.
  bool measured_skew_present = false;
  DurationParts measured_skew;
};

struct ObservationSlotView {
  // Present iff reference_id is non-empty. The header is then required.
  std::string_view reference_id;
  std::string_view content_type;
  bool byte_size_present = false;
  uint64_t byte_size = 0;
  std::string_view frame_id;
  bool source_time_present = false;
  embodiment::ClockReading source_time;
  bool receive_time_present = false;
  embodiment::ClockReading receive_time;
  std::string_view clock_domain;
};

struct StateReferenceView {
  // Wire presence of the StateReference message. Present iff this is true
  // and frame_id is non-empty.
  bool message_set = false;
  std::string_view frame_id;
  bool source_time_present = false;
  embodiment::ClockReading source_time;
  bool receive_time_present = false;
  embodiment::ClockReading receive_time;
  std::string_view clock_domain;
  uint64_t state_epoch = 0;
  std::string_view world_snapshot_id;
};

struct AbsentModalityEntryView {
  int modality = 0;
  int reason = 0;
};

struct ObservationBundleView {
  bool header_present = false;
  bool validity_present = false;
  int validity_state = 0;
  std::string_view frame_id;
  std::string_view clock_domain;
  bool source_time_present = false;
  embodiment::ClockReading source_time;
  bool receive_time_present = false;
  embodiment::ClockReading receive_time;
  bool max_skew_present = false;
  int64_t max_skew_seconds = 0;
  int32_t max_skew_nanos = 0;
  ObservationSlotView fls;
  ObservationSlotView sss;
  ObservationSlotView optical;
  ObservationSlotView point_cloud;
  StateReferenceView vehicle_state;
  std::span<const AbsentModalityEntryView> absent;
  // True when the metadata map has any entry. Values are not inspected.
  bool metadata_present = false;
};

inline bool SlotPresent(const ObservationSlotView& slot) {
  return !slot.reference_id.empty();
}

inline bool StatePresent(const StateReferenceView& state) {
  return state.message_set && !state.frame_id.empty();
}

inline bool MaxSkewNonZero(const ObservationBundleView& bundle) {
  return bundle.max_skew_present &&
         (bundle.max_skew_seconds != 0 || bundle.max_skew_nanos != 0);
}

// Engaged when any slot is present, any absent entry is set, max_skew is
// set and non-zero, metadata is non-empty, or the bundle header is present.
// A present zero max_skew does not engage by itself.
inline bool ObservationBundleEngaged(const ObservationBundleView& bundle) {
  return SlotPresent(bundle.fls) || SlotPresent(bundle.sss) ||
         SlotPresent(bundle.optical) || SlotPresent(bundle.point_cloud) ||
         StatePresent(bundle.vehicle_state) || !bundle.absent.empty() ||
         MaxSkewNonZero(bundle) || bundle.metadata_present ||
         bundle.header_present;
}

inline bool KnownModality(int modality) {
  return modality >= static_cast<int>(ObservationModality::kSonarFls) &&
         modality <= static_cast<int>(ObservationModality::kVehicleState);
}

inline bool KnownAbsentReason(int reason) {
  return reason >= static_cast<int>(AbsentModalityReason::kNotConfigured) &&
         reason <=
             static_cast<int>(AbsentModalityReason::kIntentionallyOmitted);
}

inline bool IsLowercaseSha256Hex(std::string_view value) {
  if (value.size() != 64) {
    return false;
  }
  for (const char c : value) {
    const bool digit = c >= '0' && c <= '9';
    const bool hex = c >= 'a' && c <= 'f';
    if (!digit && !hex) {
      return false;
    }
  }
  return true;
}

inline bool IsWellKnownWorldFrame(std::string_view frame_id) {
  return frame_id == embodiment::kWorldEnuFrameId ||
         frame_id == embodiment::kWorldNedFrameId;
}

inline bool TimeBefore(embodiment::ClockReading a, embodiment::ClockReading b) {
  return a.seconds < b.seconds || (a.seconds == b.seconds && a.nanos < b.nanos);
}

inline bool DurationGreater(DurationParts value, int64_t seconds,
                            int32_t nanos) {
  return value.seconds > seconds ||
         (value.seconds == seconds && value.nanos > nanos);
}

// later - earlier for later >= earlier. Saturates when the difference does
// not fit in int64 seconds.
inline DurationParts NonNegativeDifference(embodiment::ClockReading later,
                                           embodiment::ClockReading earlier) {
  int64_t seconds = 0;
  if (__builtin_sub_overflow(later.seconds, earlier.seconds, &seconds)) {
    return {std::numeric_limits<int64_t>::max(), 999999999};
  }
  int32_t nanos = later.nanos - earlier.nanos;
  if (nanos < 0) {
    nanos += embodiment::kNanosPerSecond;
    if (seconds == std::numeric_limits<int64_t>::min()) {
      return {std::numeric_limits<int64_t>::max(), 999999999};
    }
    --seconds;
  }
  return {seconds, nanos};
}

inline bool MaxSkewOk(const ObservationBundleView& bundle) {
  return bundle.max_skew_present && bundle.max_skew_seconds >= 0 &&
         embodiment::NanosInRange(bundle.max_skew_nanos);
}

inline ObservationBundleContractAssessment MakeBundleAssessment(
    ObservationBundleContractError error, embodiment::ValidityKind validity,
    bool measured_skew_present, DurationParts measured_skew) {
  ObservationBundleContractAssessment assessment;
  assessment.error = error;
  assessment.validity = validity;
  assessment.accepted = error == ObservationBundleContractError::kNone &&
                        embodiment::SampleAccepted(validity, true);
  assessment.measured_skew_present = measured_skew_present;
  assessment.measured_skew = measured_skew;
  return assessment;
}

inline ObservationBundleContractError AssessSlot(
    const ObservationSlotView& slot) {
  if (slot.frame_id.empty() || !slot.source_time_present ||
      !embodiment::NanosInRange(slot.source_time.nanos)) {
    return ObservationBundleContractError::kSlot;
  }
  if (slot.reference_id.empty() ||
      (slot.byte_size_present && slot.byte_size == 0)) {
    return ObservationBundleContractError::kReference;
  }
  return ObservationBundleContractError::kNone;
}

inline ObservationBundleContractError AssessState(
    const StateReferenceView& state) {
  if (state.frame_id.empty() || !state.source_time_present ||
      !embodiment::NanosInRange(state.source_time.nanos)) {
    return ObservationBundleContractError::kSlot;
  }
  if (!state.world_snapshot_id.empty() &&
      !IsLowercaseSha256Hex(state.world_snapshot_id)) {
    return ObservationBundleContractError::kSnapshotId;
  }
  return ObservationBundleContractError::kNone;
}

// Modality outside 1..5, reason outside 1..5, a repeated modality, or an
// absent entry for a modality that is also present. A modality listed in
// neither place is a subset and is not a defect.
inline bool AbsentListOk(const ObservationBundleView& bundle) {
  bool present[6] = {};
  if (SlotPresent(bundle.fls)) {
    present[static_cast<int>(ObservationModality::kSonarFls)] = true;
  }
  if (SlotPresent(bundle.sss)) {
    present[static_cast<int>(ObservationModality::kSonarSss)] = true;
  }
  if (SlotPresent(bundle.optical)) {
    present[static_cast<int>(ObservationModality::kOptical)] = true;
  }
  if (SlotPresent(bundle.point_cloud)) {
    present[static_cast<int>(ObservationModality::kPointCloud)] = true;
  }
  if (StatePresent(bundle.vehicle_state)) {
    present[static_cast<int>(ObservationModality::kVehicleState)] = true;
  }
  bool seen[6] = {};
  for (const AbsentModalityEntryView& entry : bundle.absent) {
    if (!KnownModality(entry.modality) || !KnownAbsentReason(entry.reason) ||
        seen[entry.modality] || present[entry.modality]) {
      return false;
    }
    seen[entry.modality] = true;
  }
  return true;
}

inline bool AnyPresent(const ObservationBundleView& bundle) {
  return SlotPresent(bundle.fls) || SlotPresent(bundle.sss) ||
         SlotPresent(bundle.optical) || SlotPresent(bundle.point_cloud) ||
         StatePresent(bundle.vehicle_state);
}

inline ObservationBundleContractError AssessPresentSlots(
    const ObservationBundleView& bundle) {
  const ObservationSlotView* slots[] = {&bundle.fls, &bundle.sss,
                                        &bundle.optical, &bundle.point_cloud};
  for (const ObservationSlotView* slot : slots) {
    if (!SlotPresent(*slot)) {
      continue;
    }
    const ObservationBundleContractError error = AssessSlot(*slot);
    if (error != ObservationBundleContractError::kNone) {
      return error;
    }
  }
  if (StatePresent(bundle.vehicle_state)) {
    return AssessState(bundle.vehicle_state);
  }
  return ObservationBundleContractError::kNone;
}

// Every present slot and the present state must use the bundle clock domain.
// Empty matches empty. A non-empty domain that differs is kClockDomain.
inline bool ClocksAgree(const ObservationBundleView& bundle) {
  const std::string_view domain = bundle.clock_domain;
  const ObservationSlotView* slots[] = {&bundle.fls, &bundle.sss,
                                        &bundle.optical, &bundle.point_cloud};
  for (const ObservationSlotView* slot : slots) {
    if (SlotPresent(*slot) && slot->clock_domain != domain) {
      return false;
    }
  }
  if (StatePresent(bundle.vehicle_state) &&
      bundle.vehicle_state.clock_domain != domain) {
    return false;
  }
  return true;
}

// Two present observation slots that both use world_enu or world_ned and
// disagree. Sensor-local frames may differ. StateReference is not a slot.
// The bundle header frame is a sync frame and is not compared here.
inline bool WorldFramesAgree(const ObservationBundleView& bundle) {
  bool have_world = false;
  std::string_view world;
  const ObservationSlotView* slots[] = {&bundle.fls, &bundle.sss,
                                        &bundle.optical, &bundle.point_cloud};
  for (const ObservationSlotView* slot : slots) {
    if (!SlotPresent(*slot) || !IsWellKnownWorldFrame(slot->frame_id)) {
      continue;
    }
    if (have_world && slot->frame_id != world) {
      return false;
    }
    have_world = true;
    world = slot->frame_id;
  }
  return true;
}

// receive_time < source_time on the bundle header or any present slot or
// state, when both timestamps are set. A set timestamp whose nanos are
// outside [0, 1e9) cannot be ordered and is a reversal defect.
inline bool SourceTimeReversed(bool source_present,
                               embodiment::ClockReading source,
                               bool receive_present,
                               embodiment::ClockReading receive) {
  if (!source_present || !receive_present) {
    return false;
  }
  if (!embodiment::NanosInRange(source.nanos) ||
      !embodiment::NanosInRange(receive.nanos)) {
    return true;
  }
  return TimeBefore(receive, source);
}

inline bool AnySourceTimeReversed(const ObservationBundleView& bundle) {
  if (SourceTimeReversed(bundle.source_time_present, bundle.source_time,
                         bundle.receive_time_present, bundle.receive_time)) {
    return true;
  }
  const ObservationSlotView* slots[] = {&bundle.fls, &bundle.sss,
                                        &bundle.optical, &bundle.point_cloud};
  for (const ObservationSlotView* slot : slots) {
    if (SlotPresent(*slot) &&
        SourceTimeReversed(slot->source_time_present, slot->source_time,
                           slot->receive_time_present, slot->receive_time)) {
      return true;
    }
  }
  const StateReferenceView& state = bundle.vehicle_state;
  return StatePresent(state) &&
         SourceTimeReversed(state.source_time_present, state.source_time,
                            state.receive_time_present, state.receive_time);
}

// max(source_time) - min(source_time) over present slots and present state.
// One sample measures zero. Zero samples leave measured_skew_present false.
inline void MeasureSkew(const ObservationBundleView& bundle,
                        bool* measured_skew_present, DurationParts* skew) {
  bool have = false;
  embodiment::ClockReading earliest;
  embodiment::ClockReading latest;
  auto consider = [&](bool present, embodiment::ClockReading time) {
    if (!present) {
      return;
    }
    if (!have) {
      earliest = time;
      latest = time;
      have = true;
      return;
    }
    if (TimeBefore(time, earliest)) {
      earliest = time;
    }
    if (TimeBefore(latest, time)) {
      latest = time;
    }
  };
  if (SlotPresent(bundle.fls)) {
    consider(bundle.fls.source_time_present, bundle.fls.source_time);
  }
  if (SlotPresent(bundle.sss)) {
    consider(bundle.sss.source_time_present, bundle.sss.source_time);
  }
  if (SlotPresent(bundle.optical)) {
    consider(bundle.optical.source_time_present, bundle.optical.source_time);
  }
  if (SlotPresent(bundle.point_cloud)) {
    consider(bundle.point_cloud.source_time_present,
             bundle.point_cloud.source_time);
  }
  if (StatePresent(bundle.vehicle_state)) {
    consider(bundle.vehicle_state.source_time_present,
             bundle.vehicle_state.source_time);
  }
  *measured_skew_present = have;
  *skew = have ? NonNegativeDifference(latest, earliest) : DurationParts{};
}

// First defect wins. Order: frame id, max_skew, absent list, present slots
// and state, clock domain, world-frame disagreement, source-time reversal,
// excessive skew. Metadata is never a defect. An empty view is not engaged.
//
// Engaged with no present slot and no absent entry is kSlot. Engaged with no
// present slot and a non-empty valid absent list (all-absent) can be accepted.
inline ObservationBundleContractAssessment AssessMultimodalObservationBundle(
    const ObservationBundleView& bundle) {
  if (!ObservationBundleEngaged(bundle)) {
    return ObservationBundleContractAssessment{};
  }
  const embodiment::ValidityKind validity = embodiment::ClassifyValidity(
      bundle.validity_present, bundle.validity_state);
  auto fail = [&](ObservationBundleContractError error) {
    return MakeBundleAssessment(error, validity, false, {});
  };
  if (bundle.frame_id.empty()) {
    return fail(ObservationBundleContractError::kMissingFrame);
  }
  if (!MaxSkewOk(bundle)) {
    return fail(ObservationBundleContractError::kMaxSkew);
  }
  if (!AbsentListOk(bundle)) {
    return fail(ObservationBundleContractError::kAbsentList);
  }
  if (!AnyPresent(bundle)) {
    if (bundle.absent.empty()) {
      return fail(ObservationBundleContractError::kSlot);
    }
  } else {
    const ObservationBundleContractError slot_error =
        AssessPresentSlots(bundle);
    if (slot_error != ObservationBundleContractError::kNone) {
      return fail(slot_error);
    }
  }
  if (!ClocksAgree(bundle)) {
    return fail(ObservationBundleContractError::kClockDomain);
  }
  if (!WorldFramesAgree(bundle)) {
    return fail(ObservationBundleContractError::kFrameMismatch);
  }
  if (AnySourceTimeReversed(bundle)) {
    return fail(ObservationBundleContractError::kTimeReversal);
  }
  bool measured_skew_present = false;
  DurationParts measured_skew;
  MeasureSkew(bundle, &measured_skew_present, &measured_skew);
  if (measured_skew_present &&
      DurationGreater(measured_skew, bundle.max_skew_seconds,
                      bundle.max_skew_nanos)) {
    return MakeBundleAssessment(ObservationBundleContractError::kExcessiveSkew,
                                validity, true, measured_skew);
  }
  return MakeBundleAssessment(ObservationBundleContractError::kNone, validity,
                              measured_skew_present, measured_skew);
}

}  // namespace intrinsic::perception::sonar

#endif  // INTRINSIC_PERCEPTION_SONAR_OBSERVATION_BUNDLE_CONTRACT_POLICY_H_
