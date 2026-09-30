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

#ifndef INTRINSIC_HARDWARE_MARINE_THRUSTER_ARRAY_POLICY_H_
#define INTRINSIC_HARDWARE_MARINE_THRUSTER_ARRAY_POLICY_H_

#include <span>
#include <string_view>

#include "intrinsic/vehicle/vehicle_contract_policy.h"

namespace intrinsic::hardware::marine {

// Policy: intrinsic_hardware/intrinsic/hardware/marine/README.md.
//
// Plain-value checks for ThrusterArrayCommand and ThrusterArrayFeedback.
// These functions do not parse protobuf, do not call ICON, do not allocate
// thrust, do not integrate dynamics, and do not rewrite thruster health
// into embodiment Validity.
//
// Health integers match ThrusterHealthState: NOMINAL=0, DISABLED=1,
// DERATED=2, STUCK_OFF=3, FAILED=4. Unset health is absent. A present
// number outside 0..4 stays unrecognized: it is not accepted and it is
// not rewritten.
//
// `frame_id` must be the body frame. An empty id and any other id are
// defects. The message type does not supply a second body frame.

enum class ThrusterHealthKind {
  kAbsent = 0,
  kNominal = 1,
  kDisabled = 2,
  kDerated = 3,
  kStuckOff = 4,
  kFailed = 5,
  kUnrecognized = 6,
};

// field_present is the optional health field, not the enum value.
inline constexpr ThrusterHealthKind ClassifyThrusterHealth(bool field_present,
                                                           int health) {
  if (!field_present) {
    return ThrusterHealthKind::kAbsent;
  }
  switch (health) {
  case 0:
    return ThrusterHealthKind::kNominal;
  case 1:
    return ThrusterHealthKind::kDisabled;
  case 2:
    return ThrusterHealthKind::kDerated;
  case 3:
    return ThrusterHealthKind::kStuckOff;
  case 4:
    return ThrusterHealthKind::kFailed;
  default:
    return ThrusterHealthKind::kUnrecognized;
  }
}

enum class ThrusterArrayError {
  kNone = 0,
  kMissingHeader = 1,
  kMissingFrame = 2,
  kWrongFrame = 3,
  kTimeReversal = 4,
  kEmptyArray = 5,
  kEmptyName = 6,
  kMissingThrust = 7,
  kNonFinite = 8,
  kHealth = 9,
  kHealthDerate = 10,
  kEfficiency = 11,
};

struct ThrusterArrayAssessment {
  ThrusterArrayError error = ThrusterArrayError::kNone;
  // Orthogonal #15 companion. Does not by itself accept or reject.
  embodiment::ValidityKind header_validity = embodiment::ValidityKind::kAbsent;
  // True only for an engaged message with no structural defect. An empty
  // message is not accepted and is not an error.
  bool accepted = false;
};

struct ThrusterHeaderView {
  bool header_present = false;
  std::string_view frame_id;
  bool source_time_present = false;
  embodiment::ClockReading source_time;
  bool receive_time_present = false;
  embodiment::ClockReading receive_time;
  bool header_validity_present = false;
  int header_validity_state = 0;
};

struct ThrusterCommandElementView {
  bool name_present = false;
  std::string_view name;
  bool thrust_present = false;
  double thrust_n = 0;
  bool enable_present = false;
  bool enable = false;
};

struct ThrusterFeedbackElementView {
  bool name_present = false;
  std::string_view name;
  bool commanded_thrust_present = false;
  double commanded_thrust_n = 0;
  bool measured_thrust_present = false;
  double measured_thrust_n = 0;
  bool saturated_present = false;
  bool saturated = false;
  bool health_present = false;
  int health = 0;
  bool health_derate_present = false;
  double health_derate = 0;
  bool efficiency_present = false;
  double efficiency = 0;
};

struct ThrusterArrayCommandView {
  ThrusterHeaderView header;
  // Valid only while the caller's element storage is alive.
  std::span<const ThrusterCommandElementView> thrusters;
};

struct ThrusterArrayFeedbackView {
  ThrusterHeaderView header;
  // Valid only while the caller's element storage is alive.
  std::span<const ThrusterFeedbackElementView> thrusters;
};

// Unset enable is enabled. Explicit false is the neutralize path.
inline bool ThrusterCommandEnabled(const ThrusterCommandElementView &element) {
  return !element.enable_present || element.enable;
}

inline bool
ThrusterArrayCommandEngaged(const ThrusterArrayCommandView &sample) {
  return sample.header.header_present || !sample.thrusters.empty();
}

inline bool
ThrusterArrayFeedbackEngaged(const ThrusterArrayFeedbackView &sample) {
  return sample.header.header_present || !sample.thrusters.empty();
}

// True when `lhs` is strictly earlier than `rhs` in (seconds, nanos) order.
inline bool ThrusterTimestampPrecedes(embodiment::ClockReading lhs,
                                      embodiment::ClockReading rhs) {
  if (lhs.seconds != rhs.seconds) {
    return lhs.seconds < rhs.seconds;
  }
  return lhs.nanos < rhs.nanos;
}

// NOMINAL → 1, DERATED → (0, 1), DISABLED / STUCK_OFF / FAILED → 0.
// Comparison is exact. There is no tolerance.
inline bool ThrusterDerateMatchesHealth(ThrusterHealthKind kind,
                                        double derate) {
  switch (kind) {
  case ThrusterHealthKind::kNominal:
    return derate == 1.0;
  case ThrusterHealthKind::kDerated:
    return derate > 0.0 && derate < 1.0;
  case ThrusterHealthKind::kDisabled:
  case ThrusterHealthKind::kStuckOff:
  case ThrusterHealthKind::kFailed:
    return derate == 0.0;
  case ThrusterHealthKind::kAbsent:
  case ThrusterHealthKind::kUnrecognized:
    return false;
  }
  return false;
}

// Dimensionless (0, 1]. Caller has already required a finite value.
inline bool ThrusterEfficiencyInRange(double efficiency) {
  return efficiency > 0.0 && efficiency <= 1.0;
}

// Header order: missing header, empty frame, frame other than body, then
// receive time strictly before source time when both timestamps are present.
inline ThrusterArrayError
AssessThrusterHeader(const ThrusterHeaderView &header) {
  if (!header.header_present) {
    return ThrusterArrayError::kMissingHeader;
  }
  if (header.frame_id.empty()) {
    return ThrusterArrayError::kMissingFrame;
  }
  if (header.frame_id != vehicle::kBodyFrameId) {
    return ThrusterArrayError::kWrongFrame;
  }
  if (header.source_time_present && header.receive_time_present &&
      ThrusterTimestampPrecedes(header.receive_time, header.source_time)) {
    return ThrusterArrayError::kTimeReversal;
  }
  return ThrusterArrayError::kNone;
}

inline ThrusterArrayAssessment
MakeThrusterAssessment(ThrusterArrayError error,
                       embodiment::ValidityKind header_validity, bool engaged) {
  const bool accepted = engaged && error == ThrusterArrayError::kNone;
  return ThrusterArrayAssessment{error, header_validity, accepted};
}

// Check order: header defects, then an empty thruster list, then each
// element in order: empty name, missing thrust, non-finite thrust. The
// first defect wins. Unset name and unset enable are not defects. Zero
// thrust is a supplied value. `kMissingThrust` is command-only.
inline ThrusterArrayAssessment
AssessThrusterArrayCommand(const ThrusterArrayCommandView &sample) {
  const embodiment::ValidityKind header_validity =
      embodiment::ClassifyValidity(sample.header.header_validity_present,
                                   sample.header.header_validity_state);
  if (!ThrusterArrayCommandEngaged(sample)) {
    return MakeThrusterAssessment(ThrusterArrayError::kNone, header_validity,
                                  false);
  }
  ThrusterArrayError error = AssessThrusterHeader(sample.header);
  if (error == ThrusterArrayError::kNone && sample.thrusters.empty()) {
    error = ThrusterArrayError::kEmptyArray;
  }
  if (error == ThrusterArrayError::kNone) {
    for (const ThrusterCommandElementView &element : sample.thrusters) {
      if (element.name_present && element.name.empty()) {
        error = ThrusterArrayError::kEmptyName;
        break;
      }
      if (!element.thrust_present) {
        error = ThrusterArrayError::kMissingThrust;
        break;
      }
      if (!embodiment::IsFinite(element.thrust_n)) {
        error = ThrusterArrayError::kNonFinite;
        break;
      }
    }
  }
  return MakeThrusterAssessment(error, header_validity, true);
}

// Check order: header defects, then an empty thruster list, then each
// element in order: empty name, non-finite commanded thrust, non-finite
// measured thrust, non-finite derate, non-finite efficiency, unknown
// health, health/derate inconsistency, efficiency outside (0, 1]. The
// first defect wins. Absent optional fields are not defects. `saturated`
// is diagnostic and is not a defect.
inline ThrusterArrayAssessment
AssessThrusterArrayFeedback(const ThrusterArrayFeedbackView &sample) {
  const embodiment::ValidityKind header_validity =
      embodiment::ClassifyValidity(sample.header.header_validity_present,
                                   sample.header.header_validity_state);
  if (!ThrusterArrayFeedbackEngaged(sample)) {
    return MakeThrusterAssessment(ThrusterArrayError::kNone, header_validity,
                                  false);
  }
  ThrusterArrayError error = AssessThrusterHeader(sample.header);
  if (error == ThrusterArrayError::kNone && sample.thrusters.empty()) {
    error = ThrusterArrayError::kEmptyArray;
  }
  if (error == ThrusterArrayError::kNone) {
    for (const ThrusterFeedbackElementView &element : sample.thrusters) {
      if (element.name_present && element.name.empty()) {
        error = ThrusterArrayError::kEmptyName;
        break;
      }
      if (element.commanded_thrust_present &&
          !embodiment::IsFinite(element.commanded_thrust_n)) {
        error = ThrusterArrayError::kNonFinite;
        break;
      }
      if (element.measured_thrust_present &&
          !embodiment::IsFinite(element.measured_thrust_n)) {
        error = ThrusterArrayError::kNonFinite;
        break;
      }
      if (element.health_derate_present &&
          !embodiment::IsFinite(element.health_derate)) {
        error = ThrusterArrayError::kNonFinite;
        break;
      }
      if (element.efficiency_present &&
          !embodiment::IsFinite(element.efficiency)) {
        error = ThrusterArrayError::kNonFinite;
        break;
      }
      const ThrusterHealthKind health =
          ClassifyThrusterHealth(element.health_present, element.health);
      if (health == ThrusterHealthKind::kUnrecognized) {
        error = ThrusterArrayError::kHealth;
        break;
      }
      if (element.health_present &&
          (!element.health_derate_present ||
           !ThrusterDerateMatchesHealth(health, element.health_derate))) {
        error = ThrusterArrayError::kHealthDerate;
        break;
      }
      if (element.efficiency_present &&
          !ThrusterEfficiencyInRange(element.efficiency)) {
        error = ThrusterArrayError::kEfficiency;
        break;
      }
    }
  }
  return MakeThrusterAssessment(error, header_validity, true);
}

} // namespace intrinsic::hardware::marine

#endif // INTRINSIC_HARDWARE_MARINE_THRUSTER_ARRAY_POLICY_H_
