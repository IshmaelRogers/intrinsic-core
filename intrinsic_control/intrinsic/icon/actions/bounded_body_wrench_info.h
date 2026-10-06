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

#ifndef INTRINSIC_ICON_ACTIONS_BOUNDED_BODY_WRENCH_INFO_H_
#define INTRINSIC_ICON_ACTIONS_BOUNDED_BODY_WRENCH_INFO_H_

#include "intrinsic/icon/actions/bounded_body_wrench.pb.h"
#include "intrinsic/vehicle/proto/vehicle_command.pb.h"

namespace intrinsic {
namespace icon {

// Streams body-frame wrench intents to a free-body vehicle part. The vehicle
// part's BodyWrenchCommand feature validates every wrench and rejects, never
// clamps, a wrench that violates its checks.
struct BoundedBodyWrenchInfo {
  static constexpr char kActionTypeName[] = "intrinsic.bounded_body_wrench";
  static constexpr char kActionDescription[] =
      "Forwards body-frame wrench intents to a free-body vehicle part. The "
      "wrench can be updated using non-realtime streaming Action inputs. The "
      "vehicle part rejects wrenches that are non-finite, stale, or outside "
      "its application or system limits.";
  static constexpr char kSlotName[] = "vehicle";
  static constexpr char kSlotDescription[] =
      "Free-body vehicle part exposing body wrench.";

  // Streaming input for updating the wrench intent. Type is the same as
  // `FixedParams`.
  static constexpr char kStreamingInputName[] = "streaming-body-wrench-command";

  using FixedParams =
      intrinsic_proto::icon::actions::proto::BoundedBodyWrenchParams;
};

// Returns params that carry `wrench`, used for both fixed and streaming input
// parameters.
BoundedBodyWrenchInfo::FixedParams GetBoundedBodyWrenchFixedParams(
    const intrinsic_proto::vehicle::BodyWrench& wrench);

}  // namespace icon
}  // namespace intrinsic

#endif  // INTRINSIC_ICON_ACTIONS_BOUNDED_BODY_WRENCH_INFO_H_
