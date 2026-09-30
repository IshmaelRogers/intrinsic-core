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

#ifndef INTRINSIC_HARDWARE_MARINE_SURFACE_FIX_POLICY_H_
#define INTRINSIC_HARDWARE_MARINE_SURFACE_FIX_POLICY_H_

#include <cstdint>
#include <span>

#include "intrinsic/hardware/marine/measurement_health_policy.h"
#include "intrinsic/vehicle/vehicle_contract_policy.h"

namespace intrinsic::hardware::marine {

// Policy: intrinsic_hardware/intrinsic/hardware/marine/README.md.
//
// Plain-value checks for SurfaceFix. These functions do not parse protobuf,
// do not convert lat/lon, WGS84, ENU, or NED, do not decide whether the
// vehicle is submerged, do not read InsSolution, and do not command
// hardware.
//
// Health checks are #75 AssessMeasurementHealth. This policy does not
// rewrite `health.state` or embodiment Validity.
//
// `source` is the measurement-local enum (FIX_SOURCE_UNSPECIFIED=0,
// FIX_SOURCE_GNSS=1, FIX_SOURCE_ACOUSTIC=2, FIX_SOURCE_OTHER=3). Unset
// source (`source_present == false`) is absent. A present 0 is
// FIX_SOURCE_UNSPECIFIED. Other numbers stay unrecognized: they are not
// accepted and they are not rewritten. An engaged sample needs a known
// source.

inline constexpr int kSurfacePositionXVarianceSlot =
    vehicle::CovarianceIndex(0, 0);
inline constexpr int kSurfacePositionYVarianceSlot =
    vehicle::CovarianceIndex(1, 1);
inline constexpr int kSurfacePositionZVarianceSlot =
    vehicle::CovarianceIndex(2, 2);

inline constexpr int kSurfaceFixVarianceSlots[] = {
    kSurfacePositionXVarianceSlot,
    kSurfacePositionYVarianceSlot,
    kSurfacePositionZVarianceSlot,
};

enum class SurfaceFixSourceKind {
  kAbsent = 0,
  kUnspecified = 1,
  kGnss = 2,
  kAcoustic = 3,
  kOther = 4,
  kUnrecognized = 5,
};

// field_present is SurfaceFix.source presence, not the enum value.
inline constexpr SurfaceFixSourceKind ClassifySurfaceFixSource(
    bool field_present, int source) {
  if (!field_present) {
    return SurfaceFixSourceKind::kAbsent;
  }
  switch (source) {
    case 0:
      return SurfaceFixSourceKind::kUnspecified;
    case 1:
      return SurfaceFixSourceKind::kGnss;
    case 2:
      return SurfaceFixSourceKind::kAcoustic;
    case 3:
      return SurfaceFixSourceKind::kOther;
    default:
      return SurfaceFixSourceKind::kUnrecognized;
  }
}

enum class SurfaceFixError {
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
  kPosition = 10,
  kSource = 11,
  kAccuracy = 12,
  kCount = 13,
  kVelocity = 14,
};

struct SurfaceFixAssessment {
  SurfaceFixError error = SurfaceFixError::kNone;
  MeasurementStateKind state = MeasurementStateKind::kAbsent;
  SurfaceFixSourceKind source = SurfaceFixSourceKind::kAbsent;
  // Orthogonal #15 companion. Does not by itself accept or reject.
  embodiment::ValidityKind header_validity = embodiment::ValidityKind::kAbsent;
  // True only for health state VALID and no structural defect. An empty
  // message is not accepted and is not an error.
  bool accepted = false;
};

struct SurfaceFixView {
  // Span inside `health.covariance` is valid only while caller storage lives.
  MeasurementHealthView health;
  bool position_x_present = false;
  double position_x_m = 0;
  bool position_y_present = false;
  double position_y_m = 0;
  bool position_z_present = false;
  double position_z_m = 0;
  bool source_present = false;
  int source = 0;
  bool satellite_count_present = false;
  int32_t satellite_count = 0;
  bool beacon_count_present = false;
  int32_t beacon_count = 0;
  bool horizontal_accuracy_present = false;
  double horizontal_accuracy_m = 0;
  bool vertical_accuracy_present = false;
  double vertical_accuracy_m = 0;
  bool velocity_x_present = false;
  double velocity_x_m_s = 0;
  bool velocity_y_present = false;
  double velocity_y_m_s = 0;
  bool velocity_z_present = false;
  double velocity_z_m_s = 0;
};

inline bool SurfaceFixEngaged(const SurfaceFixView& sample) {
  return MeasurementEngaged(sample.health) || sample.position_x_present ||
         sample.position_y_present || sample.position_z_present ||
         sample.source_present || sample.satellite_count_present ||
         sample.beacon_count_present || sample.horizontal_accuracy_present ||
         sample.vertical_accuracy_present || sample.velocity_x_present ||
         sample.velocity_y_present || sample.velocity_z_present;
}

inline bool AllowedSurfaceFixCovarianceOnly(std::span<const double> values) {
  if (static_cast<int>(values.size()) != vehicle::kCovarianceValues) {
    return false;
  }
  for (int index = 0; index < vehicle::kCovarianceValues; ++index) {
    bool allowed = false;
    for (int slot : kSurfaceFixVarianceSlots) {
      if (index == slot) {
        allowed = true;
        break;
      }
    }
    if (!allowed && values[index] != 0.0) {
      return false;
    }
  }
  return true;
}

inline bool KnownSurfaceFixSource(SurfaceFixSourceKind source) {
  return source == SurfaceFixSourceKind::kGnss ||
         source == SurfaceFixSourceKind::kAcoustic ||
         source == SurfaceFixSourceKind::kOther;
}

inline SurfaceFixError ToSurfaceFixError(MeasurementError error) {
  switch (error) {
    case MeasurementError::kNone:
      return SurfaceFixError::kNone;
    case MeasurementError::kMissingFrame:
      return SurfaceFixError::kMissingFrame;
    case MeasurementError::kWrongFrame:
      return SurfaceFixError::kWrongFrame;
    case MeasurementError::kTimeReversal:
      return SurfaceFixError::kTimeReversal;
    case MeasurementError::kNonFinite:
      return SurfaceFixError::kNonFinite;
    case MeasurementError::kQuality:
      return SurfaceFixError::kQuality;
    case MeasurementError::kCovariance:
      return SurfaceFixError::kCovariance;
    case MeasurementError::kSourceId:
      return SurfaceFixError::kSourceId;
  }
  return SurfaceFixError::kNone;
}

inline int SurfaceFixPresentCount(bool x, bool y, bool z) {
  return static_cast<int>(x) + static_cast<int>(y) + static_cast<int>(z);
}

// Check order: missing health, then #75 health order, then covariance
// slots, position presence, position finiteness, source kind, accuracy
// finiteness and sign, satellite and beacon count sign, velocity partial
// triple, velocity finiteness. The first defect wins. Absent optional
// fields are not defects. Health state is not rewritten, so a producer
// VALID sample with a missing source is rejected and stays VALID.
inline SurfaceFixAssessment AssessSurfaceFix(const SurfaceFixView& sample) {
  const embodiment::ValidityKind header_validity =
      embodiment::ClassifyValidity(sample.health.header_validity_present,
                                   sample.health.header_validity_state);
  const SurfaceFixSourceKind source =
      ClassifySurfaceFixSource(sample.source_present, sample.source);
  if (!SurfaceFixEngaged(sample)) {
    return SurfaceFixAssessment{
        SurfaceFixError::kNone, MeasurementStateKind::kAbsent,
        SurfaceFixSourceKind::kAbsent, header_validity, false};
  }

  MeasurementStateKind state = MeasurementStateKind::kAbsent;
  SurfaceFixError error = SurfaceFixError::kNone;
  if (!MeasurementEngaged(sample.health)) {
    error = SurfaceFixError::kMissingHealth;
  } else {
    const MeasurementAssessment health = AssessMeasurementHealth(sample.health);
    state = health.state;
    error = ToSurfaceFixError(health.error);
    if (error == SurfaceFixError::kNone && sample.health.covariance_present &&
        vehicle::AssessCovariance(true, sample.health.covariance) ==
            vehicle::CovarianceError::kNone &&
        !AllowedSurfaceFixCovarianceOnly(sample.health.covariance)) {
      error = SurfaceFixError::kCovarianceSlots;
    }
  }
  if (error == SurfaceFixError::kNone &&
      !(sample.position_x_present && sample.position_y_present &&
        sample.position_z_present)) {
    error = SurfaceFixError::kPosition;
  }
  if (error == SurfaceFixError::kNone &&
      (!embodiment::IsFinite(sample.position_x_m) ||
       !embodiment::IsFinite(sample.position_y_m) ||
       !embodiment::IsFinite(sample.position_z_m))) {
    error = SurfaceFixError::kNonFinite;
  }
  if (error == SurfaceFixError::kNone && !KnownSurfaceFixSource(source)) {
    error = SurfaceFixError::kSource;
  }
  if (error == SurfaceFixError::kNone && sample.horizontal_accuracy_present) {
    if (!embodiment::IsFinite(sample.horizontal_accuracy_m)) {
      error = SurfaceFixError::kNonFinite;
    } else if (sample.horizontal_accuracy_m < 0.0) {
      error = SurfaceFixError::kAccuracy;
    }
  }
  if (error == SurfaceFixError::kNone && sample.vertical_accuracy_present) {
    if (!embodiment::IsFinite(sample.vertical_accuracy_m)) {
      error = SurfaceFixError::kNonFinite;
    } else if (sample.vertical_accuracy_m < 0.0) {
      error = SurfaceFixError::kAccuracy;
    }
  }
  if (error == SurfaceFixError::kNone && sample.satellite_count_present &&
      sample.satellite_count < 0) {
    error = SurfaceFixError::kCount;
  }
  if (error == SurfaceFixError::kNone && sample.beacon_count_present &&
      sample.beacon_count < 0) {
    error = SurfaceFixError::kCount;
  }
  const int velocity = SurfaceFixPresentCount(sample.velocity_x_present,
                                              sample.velocity_y_present,
                                              sample.velocity_z_present);
  if (error == SurfaceFixError::kNone && velocity != 0 && velocity != 3) {
    error = SurfaceFixError::kVelocity;
  }
  if (error == SurfaceFixError::kNone && velocity == 3 &&
      (!embodiment::IsFinite(sample.velocity_x_m_s) ||
       !embodiment::IsFinite(sample.velocity_y_m_s) ||
       !embodiment::IsFinite(sample.velocity_z_m_s))) {
    error = SurfaceFixError::kNonFinite;
  }
  const bool accepted =
      error == SurfaceFixError::kNone && state == MeasurementStateKind::kValid;
  return SurfaceFixAssessment{error, state, source, header_validity, accepted};
}

}  // namespace intrinsic::hardware::marine

#endif  // INTRINSIC_HARDWARE_MARINE_SURFACE_FIX_POLICY_H_
