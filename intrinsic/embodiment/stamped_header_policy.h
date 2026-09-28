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

#ifndef INTRINSIC_EMBODIMENT_STAMPED_HEADER_POLICY_H_
#define INTRINSIC_EMBODIMENT_STAMPED_HEADER_POLICY_H_

#include <cstdint>
#include <optional>
#include <string_view>

namespace intrinsic::embodiment {

// Policy: intrinsic_apis/intrinsic/embodiment/proto/README.md.

inline constexpr std::string_view kClockDomainUtc = "utc";
inline constexpr std::string_view kClockDomainMonotonic = "monotonic";

inline constexpr int32_t kNanosPerSecond = 1000000000;

struct ClockReading {
  int64_t seconds = 0;
  int32_t nanos = 0;
};

enum class ValidityKind {
  kAbsent,
  kUnspecified,
  kValid,
  kInvalid,
};

// field_present is StampedHeader.validity presence, not the enum value.
// Unknown state numbers stay unspecified so they are not accepted as valid
// and are not collapsed into invalid.
inline ValidityKind ClassifyValidity(bool field_present, int state) {
  if (!field_present) {
    return ValidityKind::kAbsent;
  }
  switch (state) {
    case 1:
      return ValidityKind::kValid;
    case 2:
      return ValidityKind::kInvalid;
    default:
      return ValidityKind::kUnspecified;
  }
}

// A usable sample is explicitly valid and finite. Absence is not invalid.
inline bool SampleAccepted(ValidityKind kind, bool values_finite) {
  return kind == ValidityKind::kValid && values_finite;
}

inline bool IsUtcClockDomain(std::string_view clock_domain) {
  return clock_domain == kClockDomainUtc;
}

inline bool IsMonotonicClockDomain(std::string_view clock_domain) {
  return clock_domain == kClockDomainMonotonic;
}

inline bool SequenceAdvances(uint64_t previous, uint64_t current) {
  return current > previous;
}

inline bool NanosInRange(int32_t nanos) {
  return nanos >= 0 && nanos < kNanosPerSecond;
}

// Age in seconds for a monotonic clock only. utc and every other domain
// return nullopt so wall clock cannot feed a watchdog through this helper.
// Absent timestamps and a receive time before the source also return nullopt.
inline std::optional<double> MonotonicAgeSeconds(
    std::optional<ClockReading> source, std::optional<ClockReading> receive,
    std::string_view clock_domain) {
  if (!IsMonotonicClockDomain(clock_domain) || !source.has_value() ||
      !receive.has_value() || !NanosInRange(source->nanos) ||
      !NanosInRange(receive->nanos)) {
    return std::nullopt;
  }
  if (receive->seconds < source->seconds ||
      (receive->seconds == source->seconds && receive->nanos < source->nanos)) {
    return std::nullopt;
  }
  int64_t seconds = receive->seconds - source->seconds;
  int32_t nanos = receive->nanos - source->nanos;
  if (nanos < 0) {
    seconds -= 1;
    nanos += kNanosPerSecond;
  }
  return static_cast<double>(seconds) +
         (static_cast<double>(nanos) / static_cast<double>(kNanosPerSecond));
}

}  // namespace intrinsic::embodiment

#endif  // INTRINSIC_EMBODIMENT_STAMPED_HEADER_POLICY_H_
