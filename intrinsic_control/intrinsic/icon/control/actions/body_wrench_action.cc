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

#include "intrinsic/icon/control/actions/body_wrench_action.h"

#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "absl/algorithm/container.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/str_cat.h"
#include "absl/strings/str_join.h"
#include "absl/strings/string_view.h"
#include "intrinsic/icon/actions/body_wrench_action_info.h"
#include "intrinsic/icon/actions/body_wrench_action_signature.h"
#include "intrinsic/icon/cc_client/condition.h"
#include "intrinsic/icon/control/action_factory_context.h"
#include "intrinsic/icon/control/parts/feature_interfaces.h"
#include "intrinsic/icon/control/realtime_slot_map.h"
#include "intrinsic/icon/control/slot_types.h"
#include "intrinsic/icon/control/streaming_io_realtime.h"
#include "intrinsic/icon/control/streaming_io_types.h"
#include "intrinsic/icon/proto/v1/types.pb.h"
#include "intrinsic/icon/utils/realtime_guard.h"
#include "intrinsic/icon/utils/realtime_status.h"
#include "intrinsic/icon/utils/realtime_status_macro.h"
#include "intrinsic/icon/utils/realtime_status_or.h"
#include "intrinsic/util/status/status_macros.h"

namespace intrinsic::icon {
namespace {

using ::intrinsic_proto::icon::v1::FeatureInterfaceTypes;

absl::Status AssignId(absl::string_view field_name, absl::string_view text,
                      FixedId64& out) {
  if (text.size() > sizeof(out.data)) {
    return absl::InvalidArgumentError(absl::StrCat(
        "BodyWrench.header.", field_name, " is ", text.size(),
        " bytes, which exceeds the ", sizeof(out.data), " byte limit."));
  }
  out = FixedId64{};
  out.length = static_cast<uint8_t>(text.size());
  for (size_t i = 0; i < text.size(); ++i) {
    out.data[i] = text[i];
  }
  return absl::OkStatus();
}

}  // namespace

// static
absl::StatusOr<BodyWrenchSample> BodyWrenchAction::ParseStreamingInput(
    const BodyWrenchActionInfo::StreamingInput& input) {
  INTRINSIC_ASSERT_NON_REALTIME();
  // A default message serializes to zero bytes and is not a command. A
  // present message with an all-zero wrench still carries a header, or at
  // least one non-default field, so it is not empty.
  if (input.ByteSizeLong() == 0) {
    return absl::InvalidArgumentError(
        "An empty BodyWrench is not a command. Send a BodyWrench with a "
        "header to command the neutral (zero) wrench.");
  }
  BodyWrenchSample sample;
  const auto& header = input.header();
  sample.sequence = header.sequence();
  if (header.has_source_time()) {
    sample.has_source_time = true;
    sample.source_time_seconds = header.source_time().seconds();
    sample.source_time_nanos = header.source_time().nanos();
  }
  if (header.has_receive_time()) {
    sample.has_receive_time = true;
    sample.receive_time_seconds = header.receive_time().seconds();
    sample.receive_time_nanos = header.receive_time().nanos();
  }
  INTR_RETURN_IF_ERROR(
      AssignId("source_id", header.source_id(), sample.source_id));
  INTR_RETURN_IF_ERROR(
      AssignId("frame_id", header.frame_id(), sample.frame_id));
  INTR_RETURN_IF_ERROR(
      AssignId("clock_domain", header.clock_domain(), sample.clock_domain));
  if (header.has_validity()) {
    sample.validity_present = true;
    sample.validity_state = static_cast<uint32_t>(header.validity().state());
  }
  sample.force_x_n = input.force_x_n();
  sample.force_y_n = input.force_y_n();
  sample.force_z_n = input.force_z_n();
  sample.torque_x_n_m = input.torque_x_n_m();
  sample.torque_y_n_m = input.torque_y_n_m();
  sample.torque_z_n_m = input.torque_z_n_m();
  return sample;
}

// static
absl::StatusOr<std::unique_ptr<BodyWrenchAction>> BodyWrenchAction::Create(
    ActionFactoryContext& context) {
  INTRINSIC_ASSERT_NON_REALTIME();
  INTR_ASSIGN_OR_RETURN(SlotInfo slot_info,
                        context.GetSlotInfo(BodyWrenchActionInfo::kSlotName));

  std::vector<std::string> missing;
  const auto require = [&](FeatureInterfaceTypes type, const char* name) {
    if (absl::c_find(slot_info.config.feature_interfaces(), type) ==
        slot_info.config.feature_interfaces().end()) {
      missing.push_back(name);
    }
  };
  require(FeatureInterfaceTypes::FEATURE_INTERFACE_BODY_STATE, "BodyState");
  require(FeatureInterfaceTypes::FEATURE_INTERFACE_VEHICLE_LIMITS,
          "VehicleLimitsInterface");
  require(FeatureInterfaceTypes::FEATURE_INTERFACE_BODY_WRENCH,
          "BodyWrenchCommand");
  if (!missing.empty()) {
    return absl::FailedPreconditionError(absl::StrCat(
        "BodyWrenchAction requires the feature interfaces BodyState, "
        "VehicleLimitsInterface, and BodyWrenchCommand. Part '",
        slot_info.config.name(), "' is missing: ", absl::StrJoin(missing, ", "),
        "."));
  }

  INTR_ASSIGN_OR_RETURN(
      StreamingInputId streaming_input_id,
      (context.AddStreamingInputParser<BodyWrenchActionInfo::StreamingInput,
                                       BodyWrenchSample>(
          BodyWrenchActionInfo::kStreamingInputName,
          &BodyWrenchAction::ParseStreamingInput)));
  return std::make_unique<BodyWrenchAction>(slot_info.slot_id,
                                            streaming_input_id);
}

RealtimeStatus BodyWrenchAction::CheckSlotInterfaces(
    const RealtimeSlotMap& slot_map) const {
  if (slot_map.GetInterfaceForSlot<BodyState>(slot_id_) == nullptr) {
    return FailedPreconditionError("Slot doesn't have BodyState.");
  }
  if (slot_map.GetInterfaceForSlot<VehicleLimitsInterface>(slot_id_) ==
      nullptr) {
    return FailedPreconditionError("Slot doesn't have VehicleLimitsInterface.");
  }
  if (slot_map.GetInterfaceForSlot<BodyWrenchCommand>(slot_id_) == nullptr) {
    return FailedPreconditionError("Slot doesn't have BodyWrenchCommand.");
  }
  return OkStatus();
}

RealtimeStatus BodyWrenchAction::OnEnter(OnEnterParameters params) {
  has_pending_command_ = false;
  return CheckSlotInterfaces(params.slot_map);
}

RealtimeStatus BodyWrenchAction::Sense(SenseParameters params) {
  INTRINSIC_RT_ASSIGN_OR_RETURN(
      const BodyWrenchSample* command,
      params.streaming_io_access.PollInput<BodyWrenchSample>(
          streaming_input_id_));
  if (command != nullptr) {
    pending_command_ = *command;
    has_pending_command_ = true;
  }
  return CheckSlotInterfaces(params.slot_map);
}

RealtimeStatus BodyWrenchAction::Control(ControlParameters params) {
  INTRINSIC_RT_RETURN_IF_ERROR(CheckSlotInterfaces(params.slot_map));
  if (!has_pending_command_) {
    return OkStatus();
  }
  has_pending_command_ = false;
  BodyWrenchCommand* wrench =
      params.slot_map.GetMutableInterfaceForSlot<BodyWrenchCommand>(slot_id_);
  return wrench->SetBodyWrench(pending_command_);
}

RealtimeStatusOr<StateVariableValue> BodyWrenchAction::GetStateVariable(
    absl::string_view name) const {
  return NotFoundError(
      RealtimeStatus::StrCat(name, " is not a registered state variable."));
}

}  // namespace intrinsic::icon
