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

#ifndef INTRINSIC_ICON_ACTIONS_BODY_WRENCH_ACTION_INFO_H_
#define INTRINSIC_ICON_ACTIONS_BODY_WRENCH_ACTION_INFO_H_

#include "intrinsic/vehicle/proto/vehicle_command.pb.h"

namespace intrinsic::icon {

// Opt-in action that latches a bounded body-frame wrench through the vehicle
// feature interfaces. It is not a thruster or actuator command, and it is not
// part of any manipulator default action set.
struct BodyWrenchActionInfo {
  static constexpr char kActionTypeName[] = "intrinsic.body_wrench";
  static constexpr char kActionDescription[] =
      "Forwards body-frame wrench commands (force in N and torque in N*m at "
      "the body origin) to the vehicle's BodyWrenchCommand feature interface. "
      "The feature interface rejects non-finite, wrong-frame, stale, and "
      "out-of-limit commands. This Action never clamps a command and has no "
      "access to thruster or actuator commands. This Action takes no fixed "
      "parameters; commands arrive on the streaming input.";
  static constexpr char kSlotName[] = "body";
  static constexpr char kSlotDescription[] =
      "The vehicle Part that receives the body wrench. It must provide the "
      "BodyState, VehicleLimits, and BodyWrench feature interfaces.";

  // Streaming input carrying one BodyWrench command per message. A default
  // (empty) message is not a command. A present all-zero wrench is the
  // neutral wrench.
  static constexpr char kStreamingInputName[] = "body-wrench-command";

  using StreamingInput = ::intrinsic_proto::vehicle::BodyWrench;
};

}  // namespace intrinsic::icon

#endif  // INTRINSIC_ICON_ACTIONS_BODY_WRENCH_ACTION_INFO_H_
