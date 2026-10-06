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

#include "intrinsic/icon/control/actions/bounded_body_wrench_action.h"

#include <cstdint>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "absl/algorithm/container.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/str_cat.h"
#include "absl/strings/str_join.h"
#include "absl/strings/string_view.h"
#include "intrinsic/embodiment/proto/stamped_header.pb.h"
#include "intrinsic/icon/actions/bounded_body_wrench.pb.h"
#include "intrinsic/icon/actions/bounded_body_wrench_info.h"
#include "intrinsic/icon/cc_client/condition.h"
#include "intrinsic/icon/control/action_factory_context.h"
#include "intrinsic/icon/control/parts/feature_interfaces.h"
#include "intrinsic/icon/control/realtime_slot_map.h"
#include "intrinsic/icon/control/slot_types.h"
#include "intrinsic/icon/control/streaming_io_realtime.h"
#include "intrinsic/icon/control/streaming_io_types.h"
#include "intrinsic/icon/proto/v1/types.pb.h"
#include "intrinsic/icon/utils/fixed_str_cat.h"
#include "intrinsic/icon/utils/realtime_guard.h"
#include "intrinsic/icon/utils/realtime_status.h"
#include "intrinsic/icon/utils/realtime_status_macro.h"
#include "intrinsic/icon/utils/realtime_status_or.h"
#include "intrinsic/util/status/status_macros.h"
#include "intrinsic/vehicle/proto/vehicle_command.pb.h"

namespace intrinsic::icon {

namespace {

using ::intrinsic_proto::icon::v1::FeatureInterfaceTypes;

absl::Status CopyId(absl::string_view text, absl::string_view field_name,
                    FixedId64& out) {
  if (text.size() > sizeof(out.data)) {
    return absl::InvalidArgumentError(
        absl::StrCat("BodyWrench header ", field_name, " has ", text.size(),
                     " bytes, but at most ", sizeof(out.data), " fit."));
  }
  out = FixedId64{};
  out.length = static_cast<uint8_t>(text.size());
  for (size_t i = 0; i < text.size(); ++i) {
    out.data[i] = text[i];
  }
  return absl::OkStatus();
}

}  // namespace

BoundedBodyWrenchAction::BoundedBodyWrenchAction(
    RealtimeSlotId slot_id, StreamingInputId streaming_input_id,
    BodyWrenchSample initial_wrench)
    : slot_id_(slot_id),
      streaming_input_id_(streaming_input_id),
      command_(initial_wrench) {}

// static
absl::StatusOr<std::unique_ptr<BoundedBodyWrenchAction>>
BoundedBodyWrenchAction::Create(BoundedBodyWrenchInfo::FixedParams params_proto,
                                ActionFactoryContext& context) {
  INTRINSIC_ASSERT_NON_REALTIME();
  INTR_ASSIGN_OR_RETURN(SlotInfo vehicle_info,
                        context.GetSlotInfo(BoundedBodyWrenchInfo::kSlotName));

  std::vector<std::string> missing;
  for (const FeatureInterfaceTypes required : {
           FeatureInterfaceTypes::FEATURE_INTERFACE_BODY_STATE,
           FeatureInterfaceTypes::FEATURE_INTERFACE_BODY_WRENCH,
           FeatureInterfaceTypes::FEATURE_INTERFACE_VEHICLE_LIMITS,
       }) {
    if (!absl::c_linear_search(vehicle_info.config.feature_interfaces(),
                               static_cast<int>(required))) {
      missing.push_back(FeatureInterfaceTypes_Name(required));
    }
  }
  if (!missing.empty()) {
    return absl::FailedPreconditionError(absl::StrCat(
        "Slot '", BoundedBodyWrenchInfo::kSlotName, "' of Action type '",
        BoundedBodyWrenchInfo::kActionTypeName,
        "' is missing required feature interfaces: [",
        absl::StrJoin(missing, ", "), "]."));
  }

  INTR_ASSIGN_OR_RETURN(BodyWrenchSample initial_wrench,
                        ParseStreamingInput(params_proto));
  INTR_ASSIGN_OR_RETURN(
      StreamingInputId streaming_input_id,
      (context.AddStreamingInputParser<BoundedBodyWrenchInfo::FixedParams,
                                       BodyWrenchSample>(
          BoundedBodyWrenchInfo::kStreamingInputName, &ParseStreamingInput)));

  return std::make_unique<BoundedBodyWrenchAction>(
      vehicle_info.slot_id, streaming_input_id, initial_wrench);
}

RealtimeStatus BoundedBodyWrenchAction::OnEnter(OnEnterParameters params) {
  if (!initial_forwarded_) {
    initial_forwarded_ = true;
    pending_ = true;
  }
  return OkStatus();
}

RealtimeStatus BoundedBodyWrenchAction::Sense(SenseParameters params) {
  if (params.slot_map.GetInterfaceForSlot<BodyState>(slot_id_) == nullptr) {
    return InternalError("Slot doesn't have BodyState.");
  }
  if (params.slot_map.GetInterfaceForSlot<VehicleLimitsInterface>(slot_id_) ==
      nullptr) {
    return InternalError("Slot doesn't have VehicleLimitsInterface.");
  }

  INTRINSIC_RT_ASSIGN_OR_RETURN(
      const BodyWrenchSample* streamed,
      params.streaming_io_access.PollInput<BodyWrenchSample>(
          streaming_input_id_));
  if (streamed != nullptr) {
    command_ = *streamed;
    pending_ = true;
  }
  return OkStatus();
}

RealtimeStatus BoundedBodyWrenchAction::Control(ControlParameters params) {
  BodyWrenchCommand* body_wrench =
      params.slot_map.GetMutableInterfaceForSlot<BodyWrenchCommand>(slot_id_);
  if (body_wrench == nullptr) {
    return InternalError("Slot doesn't have BodyWrenchCommand.");
  }
  if (!pending_) {
    return OkStatus();
  }
  pending_ = false;
  return body_wrench->SetBodyWrench(command_);
}

RealtimeStatusOr<StateVariableValue> BoundedBodyWrenchAction::GetStateVariable(
    absl::string_view name) const {
  return NotFoundError(FixedStrCat<RealtimeStatus::kMaxMessageLength>(
      "BoundedBodyWrenchAction, state variable not found ", name));
}

// static
absl::StatusOr<BodyWrenchSample> BoundedBodyWrenchAction::ParseStreamingInput(
    const BoundedBodyWrenchInfo::FixedParams& params) {
  if (!params.has_wrench() || params.wrench().ByteSizeLong() == 0) {
    return absl::InvalidArgumentError(
        "The BoundedBodyWrenchParams must contain a non-empty `wrench`. An "
        "unset or empty BodyWrench is not a command.");
  }
  const intrinsic_proto::vehicle::BodyWrench& wrench = params.wrench();
  const intrinsic_proto::embodiment::StampedHeader& header = wrench.header();

  BodyWrenchSample sample;
  sample.sequence = header.sequence();
  sample.has_source_time = header.has_source_time();
  sample.source_time_seconds = header.source_time().seconds();
  sample.source_time_nanos = header.source_time().nanos();
  sample.has_receive_time = header.has_receive_time();
  sample.receive_time_seconds = header.receive_time().seconds();
  sample.receive_time_nanos = header.receive_time().nanos();
  INTR_RETURN_IF_ERROR(
      CopyId(header.source_id(), "source_id", sample.source_id));
  INTR_RETURN_IF_ERROR(CopyId(header.frame_id(), "frame_id", sample.frame_id));
  INTR_RETURN_IF_ERROR(
      CopyId(header.clock_domain(), "clock_domain", sample.clock_domain));
  sample.validity_present = header.has_validity();
  sample.validity_state = static_cast<uint32_t>(header.validity().state());
  sample.force_x_n = wrench.force_x_n();
  sample.force_y_n = wrench.force_y_n();
  sample.force_z_n = wrench.force_z_n();
  sample.torque_x_n_m = wrench.torque_x_n_m();
  sample.torque_y_n_m = wrench.torque_y_n_m();
  sample.torque_z_n_m = wrench.torque_z_n_m();
  return sample;
}

}  // namespace intrinsic::icon
