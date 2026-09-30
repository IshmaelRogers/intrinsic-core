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

#ifndef INTRINSIC_HARDWARE_MARINE_PRESSURE_DEPTH_POLICY_H_
#define INTRINSIC_HARDWARE_MARINE_PRESSURE_DEPTH_POLICY_H_

#include <span>

#include "intrinsic/hardware/marine/measurement_health_policy.h"
#include "intrinsic/vehicle/vehicle_contract_policy.h"

namespace intrinsic::hardware::marine {

// Policy: intrinsic_hardware/intrinsic/hardware/marine/README.md.
//
// Plain-value checks for PressureDepthMeasurement. These functions do not
// parse protobuf, do not convert ENU and NED, do not integrate hydrostatic
// pressure, and do not command hardware.
//
// `depth_provenance` is the measurement-local enum
// (DEPTH_PROVENANCE_UNSPECIFIED=0, DEPTH_PROVENANCE_DIRECT=1,
// DEPTH_PROVENANCE_FROM_PRESSURE=2). Unset provenance
// (`provenance_present == false`) is absent. A present 0 is UNSPECIFIED.
// Numbers other than 0, 1, and 2 stay unrecognized: they are not a usable
// conversion and they are not rewritten.
//
// Health checks are #75 AssessMeasurementHealth. This policy does not
// rewrite `health.state` or embodiment Validity.

enum class DepthProvenanceKind {
  kAbsent = 0,
  kUnspecified = 1,
  kDirect = 2,
  kFromPressure = 3,
  kUnrecognized = 4,
};

// field_present is depth_provenance presence, not the enum value.
inline constexpr DepthProvenanceKind ClassifyDepthProvenance(bool field_present,
                                                             int provenance) {
  if (!field_present) {
    return DepthProvenanceKind::kAbsent;
  }
  switch (provenance) {
    case 0:
      return DepthProvenanceKind::kUnspecified;
    case 1:
      return DepthProvenanceKind::kDirect;
    case 2:
      return DepthProvenanceKind::kFromPressure;
    default:
      return DepthProvenanceKind::kUnrecognized;
  }
}

enum class PressureDepthError {
  kNone = 0,
  kMissingHealth = 1,
  kMissingFrame = 2,
  kWrongFrame = 3,
  kTimeReversal = 4,
  kNonFinite = 5,
  kQuality = 6,
  kCovariance = 7,
  kSourceId = 8,
  kCovarianceSlots = 9,
  kPayload = 10,
  kDepth = 11,
  kProvenance = 12,
  kDensity = 13,
};

// Row-major Matrix6 diagonals. Index 0 is pressure variance (Pa^2).
// Index 7 is depth variance (m^2). The other 34 entries stay exactly 0
// when covariance is present.
inline constexpr int kPressureVarianceSlot = vehicle::CovarianceIndex(0, 0);
inline constexpr int kDepthVarianceSlot = vehicle::CovarianceIndex(1, 1);

struct PressureDepthAssessment {
  PressureDepthError error = PressureDepthError::kNone;
  MeasurementStateKind state = MeasurementStateKind::kAbsent;
  DepthProvenanceKind provenance = DepthProvenanceKind::kAbsent;
  // Orthogonal #15 companion. Does not by itself accept or reject.
  embodiment::ValidityKind header_validity = embodiment::ValidityKind::kAbsent;
  // True only for health state VALID, at least one payload field, and no
  // structural defect. An empty message is not accepted and is not an error.
  bool accepted = false;
};

struct PressureDepthMeasurementView {
  // Span inside `health.covariance` is valid only while caller storage lives.
  MeasurementHealthView health;
  bool pressure_present = false;
  double pressure_pa = 0;
  bool depth_present = false;
  double depth_m = 0;
  bool provenance_present = false;
  int depth_provenance = 0;
  bool density_present = false;
  double fluid_density_kg_m3 = 0;
};

inline bool PressureDepthEngaged(const PressureDepthMeasurementView& sample) {
  return MeasurementEngaged(sample.health) || sample.pressure_present ||
         sample.depth_present || sample.provenance_present ||
         sample.density_present;
}

inline bool UsableDepthProvenance(DepthProvenanceKind provenance) {
  return provenance == DepthProvenanceKind::kDirect ||
         provenance == DepthProvenanceKind::kFromPressure;
}

inline bool AllowedCovarianceSlotsOnly(std::span<const double> values) {
  if (static_cast<int>(values.size()) != vehicle::kCovarianceValues) {
    return false;
  }
  for (int index = 0; index < vehicle::kCovarianceValues; ++index) {
    if (index == kPressureVarianceSlot || index == kDepthVarianceSlot) {
      continue;
    }
    if (values[index] != 0.0) {
      return false;
    }
  }
  return true;
}

inline PressureDepthError ToPressureDepthError(MeasurementError error) {
  switch (error) {
    case MeasurementError::kNone:
      return PressureDepthError::kNone;
    case MeasurementError::kMissingFrame:
      return PressureDepthError::kMissingFrame;
    case MeasurementError::kWrongFrame:
      return PressureDepthError::kWrongFrame;
    case MeasurementError::kTimeReversal:
      return PressureDepthError::kTimeReversal;
    case MeasurementError::kNonFinite:
      return PressureDepthError::kNonFinite;
    case MeasurementError::kQuality:
      return PressureDepthError::kQuality;
    case MeasurementError::kCovariance:
      return PressureDepthError::kCovariance;
    case MeasurementError::kSourceId:
      return PressureDepthError::kSourceId;
  }
  return PressureDepthError::kNone;
}

// Check order: missing health, then #75 health order, then covariance
// slots, payload presence, pressure finiteness, depth finiteness, depth
// sign, provenance, FROM_PRESSURE consistency, density. The first defect
// wins. Absent optional fields are not defects. Health state is not
// rewritten. Pressure has no absolute floor beyond finiteness.
inline PressureDepthAssessment AssessPressureDepth(
    const PressureDepthMeasurementView& sample) {
  const embodiment::ValidityKind header_validity =
      embodiment::ClassifyValidity(sample.health.header_validity_present,
                                   sample.health.header_validity_state);
  const DepthProvenanceKind provenance = ClassifyDepthProvenance(
      sample.provenance_present, sample.depth_provenance);
  if (!PressureDepthEngaged(sample)) {
    return PressureDepthAssessment{
        PressureDepthError::kNone, MeasurementStateKind::kAbsent,
        DepthProvenanceKind::kAbsent, header_validity, false};
  }

  MeasurementStateKind state = MeasurementStateKind::kAbsent;
  PressureDepthError error = PressureDepthError::kNone;
  if (!MeasurementEngaged(sample.health)) {
    error = PressureDepthError::kMissingHealth;
  } else {
    const MeasurementAssessment health = AssessMeasurementHealth(sample.health);
    state = health.state;
    error = ToPressureDepthError(health.error);
    if (error == PressureDepthError::kNone &&
        sample.health.covariance_present &&
        vehicle::AssessCovariance(true, sample.health.covariance) ==
            vehicle::CovarianceError::kNone &&
        !AllowedCovarianceSlotsOnly(sample.health.covariance)) {
      error = PressureDepthError::kCovarianceSlots;
    }
  }
  if (error == PressureDepthError::kNone && !sample.pressure_present &&
      !sample.depth_present) {
    error = PressureDepthError::kPayload;
  }
  if (error == PressureDepthError::kNone && sample.pressure_present &&
      !embodiment::IsFinite(sample.pressure_pa)) {
    error = PressureDepthError::kNonFinite;
  }
  if (error == PressureDepthError::kNone && sample.depth_present) {
    if (!embodiment::IsFinite(sample.depth_m)) {
      error = PressureDepthError::kNonFinite;
    } else if (sample.depth_m < 0.0) {
      error = PressureDepthError::kDepth;
    }
  }
  if (error == PressureDepthError::kNone && sample.depth_present &&
      !UsableDepthProvenance(provenance)) {
    error = PressureDepthError::kProvenance;
  }
  if (error == PressureDepthError::kNone &&
      provenance == DepthProvenanceKind::kFromPressure &&
      !sample.depth_present) {
    error = PressureDepthError::kProvenance;
  }
  if (error == PressureDepthError::kNone &&
      provenance == DepthProvenanceKind::kFromPressure &&
      !sample.pressure_present) {
    error = PressureDepthError::kProvenance;
  }
  if (error == PressureDepthError::kNone &&
      provenance == DepthProvenanceKind::kFromPressure &&
      (!sample.density_present ||
       !embodiment::IsFinite(sample.fluid_density_kg_m3) ||
       sample.fluid_density_kg_m3 <= 0.0)) {
    error = PressureDepthError::kDensity;
  }
  if (error == PressureDepthError::kNone && sample.density_present) {
    if (!embodiment::IsFinite(sample.fluid_density_kg_m3)) {
      error = PressureDepthError::kDensity;
    } else if (sample.fluid_density_kg_m3 <= 0.0) {
      error = PressureDepthError::kDensity;
    }
  }
  const bool accepted = error == PressureDepthError::kNone &&
                        state == MeasurementStateKind::kValid;
  return PressureDepthAssessment{error, state, provenance, header_validity,
                                 accepted};
}

}  // namespace intrinsic::hardware::marine

#endif  // INTRINSIC_HARDWARE_MARINE_PRESSURE_DEPTH_POLICY_H_
