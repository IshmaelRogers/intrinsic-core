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

#ifndef INTRINSIC_ICON_CONTROL_ACTIONS_BODY_WRENCH_ACTION_H_
#define INTRINSIC_ICON_CONTROL_ACTIONS_BODY_WRENCH_ACTION_H_

#include <memory>

#include "absl/status/statusor.h"
#include "absl/strings/string_view.h"
#include "intrinsic/icon/actions/body_wrench_action_info.h"
#include "intrinsic/icon/control/action_factory_context.h"
#include "intrinsic/icon/control/parts/feature_interfaces.h"
#include "intrinsic/icon/control/rtcl_action.h"
#include "intrinsic/icon/control/slot_types.h"
#include "intrinsic/icon/control/streaming_io_types.h"
#include "intrinsic/icon/utils/realtime_status.h"
#include "intrinsic/icon/utils/realtime_status_or.h"

namespace intrinsic::icon {

// Forwards streamed body-frame wrench commands to the slot's
// BodyWrenchCommand feature interface.
//
// The Action is a thin forwarder. BodyWrenchCommand::SetBodyWrench is
// authoritative for finiteness, frame, command age, state age, and vehicle
// limits. This Action neither validates nor clamps a command, and it
// propagates the status that SetBodyWrench returns. It reaches the slot only
// through BodyState, VehicleLimitsInterface, and BodyWrenchCommand, so it has
// no handle to thrusters, actuators, or an allocator.
//
// BodyWrenchCommand holds the last accepted wrench while it is fresh, so the
// Action forwards each streamed command once. Forwarding the same command
// again would be rejected by the feature because its sequence does not advance.
class BodyWrenchAction final : public RtclActionInterface {
 public:
  // Non-real-time conversion of the streaming input. An empty message is not
  // a command and returns InvalidArgumentError. No other validation happens
  // here, so the vehicle body features stay the only authority for command
  // checks.
  static absl::StatusOr<BodyWrenchSample> ParseStreamingInput(
      const BodyWrenchActionInfo::StreamingInput& input);

  // Returns FailedPreconditionError naming the missing interface when the
  // slot's part does not provide BodyState, VehicleLimitsInterface, and
  // BodyWrenchCommand.
  static absl::StatusOr<std::unique_ptr<BodyWrenchAction>> Create(
      ActionFactoryContext& context);

  BodyWrenchAction(RealtimeSlotId slot_id, StreamingInputId streaming_input_id)
      : slot_id_(slot_id), streaming_input_id_(streaming_input_id) {}

  RealtimeStatus OnEnter(OnEnterParameters params) override;

  RealtimeStatus Sense(SenseParameters params) override;

  RealtimeStatus Control(ControlParameters params) override;

  RealtimeStatusOr<StateVariableValue> GetStateVariable(
      absl::string_view name) const override;

 private:
  RealtimeStatus CheckSlotInterfaces(const RealtimeSlotMap& slot_map) const;

  RealtimeSlotId slot_id_;
  StreamingInputId streaming_input_id_;
  // Latest streamed command that Control() has not forwarded yet.
  bool has_pending_command_ = false;
  BodyWrenchSample pending_command_;
};

}  // namespace intrinsic::icon

#endif  // INTRINSIC_ICON_CONTROL_ACTIONS_BODY_WRENCH_ACTION_H_
