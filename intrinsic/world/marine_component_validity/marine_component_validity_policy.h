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

#ifndef INTRINSIC_WORLD_MARINE_COMPONENT_VALIDITY_MARINE_COMPONENT_VALIDITY_POLICY_H_
#define INTRINSIC_WORLD_MARINE_COMPONENT_VALIDITY_MARINE_COMPONENT_VALIDITY_POLICY_H_

#include <cstdint>
#include <limits>
#include <string_view>

#include "intrinsic/embodiment/frame_policy.h"
#include "intrinsic/embodiment/stamped_header_policy.h"

namespace intrinsic::world {

// Policy: intrinsic_apis/intrinsic/world/proto/README.md.
//
// Plain-value checks for MarineComponentValidity. These functions do not
// parse protobuf, do not mutate World entities, and do not allocate a
// snapshot id. Expired is a host assessment. It is not a Validity.State.

struct TimeParts {
  int64_t seconds = 0;
  int32_t nanos = 0;
};

enum class ComponentValidityError {
  kNone = 0,
  kSourceId = 1,
  kObservationTime = 2,
  kHorizon = 3,
  kConfidence = 4,
};

// Unknown: the embodiment Validity field is unset, or observation_time /
// validity_horizon / query_time cannot form a deadline. Expired: both
// component times are usable and query_time is strictly after
// observation_time + validity_horizon. Equality with that deadline is fresh.
enum class ComponentFreshness {
  kUnknown = 0,
  kFresh = 1,
  kExpired = 2,
};

struct MarineComponentValidityView {
  // False when the caller is not validating a present component. An empty
  // message is not present.
  bool present = false;
  std::string_view source_id;
  bool observation_time_present = false;
  TimeParts observation_time;
  bool validity_horizon_present = false;
  TimeParts validity_horizon;
  bool confidence_present = false;
  double confidence = 0;
  // Opaque reference. Empty is allowed. The string is not a covariance.
  std::string_view uncertainty_reference;
  bool validity_present = false;
  int validity_state = 0;
};

struct MarineComponentAssessment {
  ComponentValidityError error = ComponentValidityError::kNone;
  embodiment::ValidityKind validity = embodiment::ValidityKind::kAbsent;
  ComponentFreshness freshness = ComponentFreshness::kUnknown;
  // True only for STATE_VALID, no structural defect, and a fresh deadline.
  bool accepted = false;
};

inline bool TimestampNanosInRange(int32_t nanos) {
  return embodiment::NanosInRange(nanos);
}

inline bool DurationNonNegative(TimeParts duration) {
  return duration.seconds >= 0 && TimestampNanosInRange(duration.nanos);
}

inline bool ConfidenceInRange(double confidence) {
  return embodiment::IsFinite(confidence) && confidence >= 0.0 &&
         confidence <= 1.0;
}

// Adds a non-negative duration onto a timestamp. Returns false when the
// inputs cannot form a deadline or the sum does not fit in int64 seconds.
inline bool AddNonNegativeDuration(TimeParts start, TimeParts duration,
                                   TimeParts* deadline) {
  if (deadline == nullptr || !TimestampNanosInRange(start.nanos) ||
      !DurationNonNegative(duration)) {
    return false;
  }
  int32_t nanos = start.nanos + duration.nanos;
  int64_t carry = 0;
  if (nanos >= embodiment::kNanosPerSecond) {
    nanos -= embodiment::kNanosPerSecond;
    carry = 1;
  }
  if (carry == 1 && duration.seconds == std::numeric_limits<int64_t>::max()) {
    return false;
  }
  const int64_t duration_with_carry = duration.seconds + carry;
  // A negative start plus a non-negative duration cannot overflow int64.
  if (start.seconds > 0 &&
      duration_with_carry >
          std::numeric_limits<int64_t>::max() - start.seconds) {
    return false;
  }
  deadline->seconds = start.seconds + duration_with_carry;
  deadline->nanos = nanos;
  return true;
}

inline bool TimeStrictlyAfter(TimeParts query, TimeParts deadline) {
  if (query.seconds != deadline.seconds) {
    return query.seconds > deadline.seconds;
  }
  return query.nanos > deadline.nanos;
}

inline ComponentFreshness AssessFreshness(bool validity_present,
                                          bool observation_present,
                                          TimeParts observation,
                                          bool horizon_present,
                                          TimeParts horizon, TimeParts query) {
  if (!validity_present || !observation_present || !horizon_present ||
      !TimestampNanosInRange(query.nanos)) {
    return ComponentFreshness::kUnknown;
  }
  TimeParts deadline;
  if (!AddNonNegativeDuration(observation, horizon, &deadline)) {
    return ComponentFreshness::kUnknown;
  }
  if (TimeStrictlyAfter(query, deadline)) {
    return ComponentFreshness::kExpired;
  }
  return ComponentFreshness::kFresh;
}

// Check order: source id, observation nanos, horizon, confidence. The first
// defect wins. Absent optional fields are not defects. An empty view is not
// a component. Freshness is reported even when a defect is present.
inline MarineComponentAssessment AssessMarineComponentValidity(
    const MarineComponentValidityView& component, TimeParts query) {
  if (!component.present) {
    return MarineComponentAssessment{};
  }
  const embodiment::ValidityKind validity = embodiment::ClassifyValidity(
      component.validity_present, component.validity_state);
  ComponentValidityError error = ComponentValidityError::kNone;
  if (component.source_id.empty()) {
    error = ComponentValidityError::kSourceId;
  } else if (component.observation_time_present &&
             !TimestampNanosInRange(component.observation_time.nanos)) {
    error = ComponentValidityError::kObservationTime;
  } else if (component.validity_horizon_present &&
             !DurationNonNegative(component.validity_horizon)) {
    error = ComponentValidityError::kHorizon;
  } else if (component.confidence_present &&
             !ConfidenceInRange(component.confidence)) {
    error = ComponentValidityError::kConfidence;
  }
  const ComponentFreshness freshness = AssessFreshness(
      component.validity_present, component.observation_time_present,
      component.observation_time, component.validity_horizon_present,
      component.validity_horizon, query);
  const bool accepted = error == ComponentValidityError::kNone &&
                        embodiment::SampleAccepted(validity, true) &&
                        freshness == ComponentFreshness::kFresh;
  return MarineComponentAssessment{error, validity, freshness, accepted};
}

}  // namespace intrinsic::world

#endif  // INTRINSIC_WORLD_MARINE_COMPONENT_VALIDITY_MARINE_COMPONENT_VALIDITY_POLICY_H_
