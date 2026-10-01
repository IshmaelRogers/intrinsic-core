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

#include "intrinsic/safety/state_age_rule.h"

#include "intrinsic/embodiment/stamped_header_policy.h"

namespace intrinsic::safety {
namespace {

using embodiment::kNanosPerSecond;

constexpr std::string_view kUnusableSummary = "state age input is not usable";
constexpr std::string_view kExceededSummary = "state age exceeds max_age";

SafetyRuleResult AgeViolation(std::string_view summary) {
  return SafetyRuleResult{true, kStateAgeRuleId, kSeverityError, summary,
                          kDecisionKindReject};
}

bool StampNanosOk(int32_t nanos) { return embodiment::NanosInRange(nanos); }

bool MaxAgeOk(StateTime max_age) {
  return max_age.seconds >= 0 && StampNanosOk(max_age.nanos);
}

bool ObservationInFuture(StateTime query_time, StateTime observation_time) {
  if (observation_time.seconds != query_time.seconds) {
    return observation_time.seconds > query_time.seconds;
  }
  return observation_time.nanos > query_time.nanos;
}

}  // namespace

SafetyRuleResult EvaluateStateAgeRule(StateTime query_time,
                                      StateTime observation_time,
                                      StateTime max_age) {
  if (!StampNanosOk(query_time.nanos) ||
      !StampNanosOk(observation_time.nanos) || !MaxAgeOk(max_age)) {
    return AgeViolation(kUnusableSummary);
  }
  if (ObservationInFuture(query_time, observation_time)) {
    return AgeViolation(kUnusableSummary);
  }

  // query >= observation, so the mathematical age is non-negative. A
  // difference that does not fit in int64 seconds is older than any max_age
  // that itself fits in the StateTime seconds field.
  int64_t seconds = 0;
  if (__builtin_sub_overflow(query_time.seconds, observation_time.seconds,
                             &seconds)) {
    return AgeViolation(kExceededSummary);
  }
  int32_t nanos = query_time.nanos - observation_time.nanos;
  if (nanos < 0) {
    nanos += kNanosPerSecond;
    // seconds == 0 here would mean the observation is in the future. That
    // path already returned. A negative result fails closed.
    if (__builtin_sub_overflow(seconds, static_cast<int64_t>(1), &seconds) ||
        seconds < 0) {
      return AgeViolation(kUnusableSummary);
    }
  }
  const bool too_old = seconds > max_age.seconds ||
                       (seconds == max_age.seconds && nanos > max_age.nanos);
  if (too_old) {
    return AgeViolation(kExceededSummary);
  }
  return SafetyRuleResult{};
}

}  // namespace intrinsic::safety
