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

#ifndef INTRINSIC_SAFETY_STATE_AGE_RULE_H_
#define INTRINSIC_SAFETY_STATE_AGE_RULE_H_

#include <cstdint>
#include <string_view>

#include "intrinsic/safety/safety_rule_result.h"

namespace intrinsic::safety {

// Pure state-age gate. Times are injected by the caller. This function does
// not read a wall clock, World, ICON, a HAL, or the snapshot skew assessor.

inline constexpr std::string_view kStateAgeRuleId = "state.age";

// (seconds, nanos). `nanos` must be in [0, 1000000000) to be usable.
struct StateTime {
  int64_t seconds = 0;
  int32_t nanos = 0;
};

// Fixture default. An age equal to this bound is compliant.
inline constexpr StateTime kDefaultStateMaxAge{2, 0};

// `state.age` at ERROR, recommending REJECT, when the input is unusable or
// the age is strictly greater than `max_age`. Equality with `max_age` is
// compliant. A future observation stamp fails closed.
SafetyRuleResult EvaluateStateAgeRule(StateTime query_time,
                                      StateTime observation_time,
                                      StateTime max_age = kDefaultStateMaxAge);

}  // namespace intrinsic::safety

#endif  // INTRINSIC_SAFETY_STATE_AGE_RULE_H_
