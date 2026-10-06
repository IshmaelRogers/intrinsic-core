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

#ifndef INTRINSIC_ICON_CONTROL_ACTIONS_BOUNDED_BODY_WRENCH_ACTION_H_
#define INTRINSIC_ICON_CONTROL_ACTIONS_BOUNDED_BODY_WRENCH_ACTION_H_

#include <memory>

#include "absl/status/statusor.h"
#include "absl/strings/string_view.h"
#include "intrinsic/icon/actions/bounded_body_wrench_info.h"
#include "intrinsic/icon/cc_client/condition.h"
#include "intrinsic/icon/control/action_factory_context.h"
#include "intrinsic/icon/control/parts/feature_interfaces.h"
#include "intrinsic/icon/control/rtcl_action.h"
#include "intrinsic/icon/control/slot_types.h"
#include "intrinsic/icon/control/streaming_io_types.h"
#include "intrinsic/icon/utils/realtime_status.h"
#include "intrinsic/icon/utils/realtime_status_or.h"

namespace intrinsic::icon {

// Forwards body-frame wrench intents to the `BodyWrenchCommand` feature of a
// free-body vehicle part.
//
// The fixed parameters are forwarded once, in the first cycle after the first
// OnEnter(). Every streaming input is forwarded once, in the cycle after it
// arrives. The vehicle part requires the sequence of accepted wrenches to
// advance, so the action never re-sends a wrench.
//
// The action does not check finiteness, frame, validity, sequence, command age
// or vehicle limits. `BodyWrenchCommand::SetBodyWrench` owns those checks and
// its status is returned from Control() unchanged. The action reaches no
// thruster, actuator or allocator interface.
//
// Sense(), OnEnter() and Control() do not allocate.
class BoundedBodyWrenchAction final : public RtclActionInterface {
 public:
  BoundedBodyWrenchAction(RealtimeSlotId slot_id,
                          StreamingInputId streaming_input_id,
                          BodyWrenchSample initial_wrench);

  // Returns FailedPrecondition if the part in the `vehicle` slot does not
  // advertise BODY_STATE, BODY_WRENCH and VEHICLE_LIMITS.
  static absl::StatusOr<std::unique_ptr<BoundedBodyWrenchAction>> Create(
      BoundedBodyWrenchInfo::FixedParams params_proto,
      ActionFactoryContext& context);

  RealtimeStatus OnEnter(OnEnterParameters params) override;

  RealtimeStatus Sense(SenseParameters params) override;

  RealtimeStatus Control(ControlParameters params) override;

  RealtimeStatusOr<StateVariableValue> GetStateVariable(
      absl::string_view name) const override;

  // Converts the non-realtime proto into the realtime sample. Runs on the
  // non-realtime side, for both the fixed parameters and every streaming
  // input.
  //
  // Returns InvalidArgument if `params` carries no wrench (an unset or empty
  // `BodyWrench` is not a command) or if an id does not fit a FixedId64.
  // Values are copied as given. They are validated by the vehicle part.
  static absl::StatusOr<BodyWrenchSample> ParseStreamingInput(
      const BoundedBodyWrenchInfo::FixedParams& params);

 private:
  RealtimeSlotId slot_id_;
  StreamingInputId streaming_input_id_;
  BodyWrenchSample command_;
  bool pending_ = false;
  bool initial_forwarded_ = false;
};

}  // namespace intrinsic::icon

#endif  // INTRINSIC_ICON_CONTROL_ACTIONS_BOUNDED_BODY_WRENCH_ACTION_H_
