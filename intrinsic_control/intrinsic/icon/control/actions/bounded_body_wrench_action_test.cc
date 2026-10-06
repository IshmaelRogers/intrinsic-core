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

#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include "absl/container/flat_hash_map.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "gmock/gmock.h"
#include "gtest/gtest.h"
#include "intrinsic/embodiment/proto/stamped_header.pb.h"
#include "intrinsic/icon/actions/bounded_body_wrench_info.h"
#include "intrinsic/icon/actions/bounded_body_wrench_signature.h"
#include "intrinsic/icon/common/id_types.h"
#include "intrinsic/icon/control/action_factory_context.h"
#include "intrinsic/icon/control/actions/test_helpers.h"
#include "intrinsic/icon/control/parts/feature_interface_registry.h"
#include "intrinsic/icon/control/parts/feature_interfaces.h"
#include "intrinsic/icon/control/parts/testing/fake_vehicle_realtime_part.h"
#include "intrinsic/icon/control/realtime_signal_access.h"
#include "intrinsic/icon/control/realtime_signal_storage.h"
#include "intrinsic/icon/control/realtime_slot_map.h"
#include "intrinsic/icon/control/rtcl_action_factory_registry.h"
#include "intrinsic/icon/control/slot_types.h"
#include "intrinsic/icon/control/streaming_io_helpers.h"
#include "intrinsic/icon/control/streaming_io_realtime.h"
#include "intrinsic/icon/control/streaming_io_storage.h"
#include "intrinsic/icon/proto/v1/types.pb.h"
#include "intrinsic/icon/server/signature_utils.h"
#include "intrinsic/icon/testing/malloc_test.h"
#include "intrinsic/icon/utils/clock.h"
#include "intrinsic/icon/utils/realtime_status.h"
#include "intrinsic/icon/utils/realtime_status_matchers.h"
#include "intrinsic/kinematics/types/joint_trajectories.h"
#include "intrinsic/vehicle/proto/vehicle_command.pb.h"

namespace intrinsic::icon {
namespace {

using ::intrinsic_proto::icon::v1::FeatureInterfaceTypes;
using ::testing::HasSubstr;

#define ASSERT_STATUS_OK(expr)                                \
  do {                                                        \
    const absl::Status status_for_assert = (expr);            \
    ASSERT_TRUE(status_for_assert.ok()) << status_for_assert; \
  } while (false)

#define EXPECT_STATUS_OK(expr)                                \
  do {                                                        \
    const absl::Status status_for_expect = (expr);            \
    EXPECT_TRUE(status_for_expect.ok()) << status_for_expect; \
  } while (false)

class ManualClock : public Clock::IClockDriver {
 public:
  explicit ManualClock(Time start) : now_(start) {}

  Time now() const override { return now_; }

  void Advance(Duration step) { now_ += step; }

 private:
  Time now_;
};

class SingleSlotMap : public RealtimeSlotMapInterface {
 public:
  SingleSlotMap(RealtimeSlotId slot_id, FeatureInterfaceRegistry* registry)
      : slot_id_(slot_id), registry_(registry) {}

  FeatureInterfaceRegistry* GetMutableRegistryForSlot(
      RealtimeSlotId slot_id) override {
    return slot_id == slot_id_ ? registry_ : nullptr;
  }
  const FeatureInterfaceRegistry* GetRegistryForSlot(
      RealtimeSlotId slot_id) const override {
    return slot_id == slot_id_ ? registry_ : nullptr;
  }

 private:
  RealtimeSlotId slot_id_;
  FeatureInterfaceRegistry* registry_;
};

FixedId64 Id(std::string_view text) {
  FixedId64 id;
  EXPECT_LE(text.size(), sizeof(id.data));
  id.length = static_cast<uint8_t>(text.size());
  for (size_t i = 0; i < text.size(); ++i) {
    id.data[i] = text[i];
  }
  return id;
}

std::string_view View(const FixedId64& id) {
  return std::string_view(id.data, id.length);
}

VehicleLimitsSample ForceTorqueLimits(double force, double torque) {
  VehicleLimitsSample sample;
  sample.has_force_limits = true;
  sample.max_force_n = VehicleVector3{force, force, force};
  sample.has_torque_limits = true;
  sample.max_torque_n_m = VehicleVector3{torque, torque, torque};
  return sample;
}

BodyStateSample ValidState(int64_t monotonic_ns, uint64_t sequence) {
  BodyStateSample sample;
  sample.latched = true;
  sample.latched_monotonic_ns = monotonic_ns;
  sample.sequence = sequence;
  sample.has_source_time = true;
  sample.source_time_seconds = 1000;
  sample.has_receive_time = true;
  sample.receive_time_seconds = 1000;
  sample.source_id = Id("estimator");
  sample.pose_frame_id = Id("world_enu");
  sample.clock_domain = Id("monotonic");
  sample.validity_present = true;
  sample.validity_state = 1;
  sample.has_pose = true;
  sample.orientation_w = 1.0;
  sample.navigation_mode = 3;
  return sample;
}

// Wrench command proto with a fresh monotonic stamp (age `age_ms`).
BoundedBodyWrenchInfo::FixedParams WrenchParams(uint64_t sequence,
                                                double force_x,
                                                double torque_z = 0.0,
                                                int age_ms = 0) {
  intrinsic_proto::vehicle::BodyWrench wrench;
  auto* header = wrench.mutable_header();
  header->set_sequence(sequence);
  header->mutable_source_time()->set_seconds(1000);
  header->mutable_receive_time()->set_seconds(1000);
  header->mutable_receive_time()->set_nanos(age_ms * 1000000);
  header->set_source_id("planner");
  header->set_frame_id("body");
  header->set_clock_domain("monotonic");
  header->mutable_validity()->set_state(
      intrinsic_proto::embodiment::Validity::STATE_VALID);
  wrench.set_force_x_n(force_x);
  wrench.set_force_y_n(0.25);
  wrench.set_torque_z_n_m(torque_z);
  return GetBoundedBodyWrenchFixedParams(wrench);
}

// Everything an ICON server would provide to one BoundedBodyWrenchAction
// instance, with a FeatureInterfaceRegistry the test chooses.
class ActionHarness {
 public:
  ActionHarness(FeatureInterfaceRegistry* registry,
                intrinsic_proto::icon::v1::PartConfig part_config)
      : signature_(GetBoundedBodyWrenchSignature()),
        slot_map_impl_(RealtimeSlotId(0), registry),
        slot_map_(slot_map_impl_),
        storage_(ActionInstanceId(1), signature_) {
    SlotInfo slot_info;
    slot_info.config = std::move(part_config);
    slot_info.slot_id = RealtimeSlotId(0);
    slot_infos_[BoundedBodyWrenchInfo::kSlotName] = std::move(slot_info);
  }

  absl::Status Create(const BoundedBodyWrenchInfo::FixedParams& params) {
    auto signals = CreateRealtimeSignalStorage(signature_);
    if (!signals.ok()) return signals.status();
    signal_storage_ = std::make_unique<RealtimeSignalStorage>(
        std::move(signals->signal_storage));
    context_ = std::make_unique<ActionFactoryContext>(
        server_config_, signature_, slot_infos_, signals->signal_id_map,
        storage_, ActionInstanceId(1), trajectory_map_);
    auto action = BoundedBodyWrenchAction::Create(params, *context_);
    if (!action.ok()) return action.status();
    action_ = std::move(*action);
    rt_storage_ = std::make_unique<RealtimeStreamingIoStorage>(
        FromStreamingIoStorage(storage_));
    signal_access_ = std::make_unique<RealtimeSignalAccess>(*signal_storage_);
    return absl::OkStatus();
  }

  absl::Status Stream(const BoundedBodyWrenchInfo::FixedParams& params) {
    return WriteStreamingInput<BoundedBodyWrenchInfo::FixedParams>(
        BoundedBodyWrenchInfo::kStreamingInputName, params, storage_);
  }

  RealtimeStatus OnEnter() { return action_->OnEnter({.slot_map = slot_map_}); }

  RealtimeStatus Sense() {
    StreamingIoRealtimeAccess access(Clock::Now(), *rt_storage_);
    return action_->Sense({.slot_map = slot_map_,
                           .streaming_io_access = access,
                           .signal_access = *signal_access_});
  }

  RealtimeStatus Control() { return action_->Control({.slot_map = slot_map_}); }

  BoundedBodyWrenchAction& action() { return *action_; }

 private:
  const intrinsic_proto::icon::v1::ActionSignature signature_;
  const intrinsic_proto::icon::v1::ServerConfig server_config_;
  absl::flat_hash_map<std::string, SlotInfo> slot_infos_;
  absl::flat_hash_map<ActionInstanceId, JointTrajectoryPVA> trajectory_map_;
  SingleSlotMap slot_map_impl_;
  RealtimeSlotMap slot_map_;
  StreamingIoStorage storage_;
  std::unique_ptr<RealtimeSignalStorage> signal_storage_;
  std::unique_ptr<ActionFactoryContext> context_;
  std::unique_ptr<RealtimeStreamingIoStorage> rt_storage_;
  std::unique_ptr<RealtimeSignalAccess> signal_access_;
  std::unique_ptr<BoundedBodyWrenchAction> action_;
};

struct CycleResult {
  RealtimeStatus read;
  RealtimeStatus sense;
  RealtimeStatus control;
  RealtimeStatus apply;
};

class BoundedBodyWrenchActionTest : public ::testing::Test {
 protected:
  void SetUp() override {
    clock_ = std::make_shared<ManualClock>(Time(Seconds(1000)));
    Clock::setClockImpl(clock_);
    auto part = FakeVehicleRealtimePart::Create(ForceTorqueLimits(10.0, 5.0),
                                                ForceTorqueLimits(20.0, 8.0));
    ASSERT_TRUE(part.ok()) << part.status();
    part_ = std::move(*part);
    harness_ = std::make_unique<ActionHarness>(
        &part_->GetFeatureInterfaces(), PartConfigFromPart("vehicle", *part_));
  }

  void TearDown() override { Clock::setClockImpl(nullptr); }

  int64_t NowNs() const { return Clock::ToNSec(Clock::Now()); }

  // One ICON cycle in server order: part ReadStatus, Action Sense, Action
  // Control, part ApplyCommand. The body state is refreshed to the current
  // manual clock unless `state_age` moves the producer stamp into the past.
  CycleResult RunCycle(Duration state_age = Nanoseconds(0)) {
    CycleResult result;
    EXPECT_THAT(part_->SetBodyState(ValidState(
                    NowNs() - Clock::ToNSec(Time(state_age)), ++state_seq_)),
                RealtimeIsOk());
    result.read = part_->ReadStatusForTest();
    result.sense = harness_->Sense();
    result.control = harness_->Control();
    result.apply = part_->ApplyCommandForTest();
    return result;
  }

  void ExpectNeutralRecorded() {
    const BodyWrenchSample& recorded = part_->RecordedBodyWrench();
    EXPECT_DOUBLE_EQ(recorded.force_x_n, 0.0);
    EXPECT_DOUBLE_EQ(recorded.force_y_n, 0.0);
    EXPECT_DOUBLE_EQ(recorded.force_z_n, 0.0);
    EXPECT_DOUBLE_EQ(recorded.torque_x_n_m, 0.0);
    EXPECT_DOUBLE_EQ(recorded.torque_y_n_m, 0.0);
    EXPECT_DOUBLE_EQ(recorded.torque_z_n_m, 0.0);
    EXPECT_EQ(View(recorded.frame_id), "body");
  }

  std::shared_ptr<ManualClock> clock_;
  std::unique_ptr<FakeVehicleRealtimePart> part_;
  std::unique_ptr<ActionHarness> harness_;
  uint64_t state_seq_ = 0;
};

TEST_F(BoundedBodyWrenchActionTest, SignatureMatchesLockedContract) {
  const intrinsic_proto::icon::v1::ActionSignature signature =
      GetBoundedBodyWrenchSignature();
  EXPECT_EQ(signature.action_type_name(), "intrinsic.bounded_body_wrench");
  ASSERT_EQ(signature.part_slot_infos().size(), 1);
  const auto& slot = signature.part_slot_infos().at("vehicle");
  EXPECT_THAT(slot.required_feature_interfaces(),
              ::testing::UnorderedElementsAre(
                  FeatureInterfaceTypes::FEATURE_INTERFACE_BODY_STATE,
                  FeatureInterfaceTypes::FEATURE_INTERFACE_BODY_WRENCH,
                  FeatureInterfaceTypes::FEATURE_INTERFACE_VEHICLE_LIMITS));
  EXPECT_TRUE(slot.optional_feature_interfaces().empty());
  ASSERT_EQ(signature.streaming_input_infos_size(), 1);
  EXPECT_EQ(signature.streaming_input_infos(0).value_message_type(),
            "intrinsic_proto.icon.actions.proto.BoundedBodyWrenchParams");
  EXPECT_EQ(signature.fixed_parameters_message_type(),
            "intrinsic_proto.icon.actions.proto.BoundedBodyWrenchParams");
}

TEST_F(BoundedBodyWrenchActionTest, RegisterTargetRegistersTheActionType) {
  const auto signature = GetGlobalRtclActionFactoryRegistry().GetSignature(
      BoundedBodyWrenchInfo::kActionTypeName);
  ASSERT_TRUE(signature.ok()) << signature.status();
  EXPECT_EQ(signature->action_type_name(), "intrinsic.bounded_body_wrench");
}

TEST_F(BoundedBodyWrenchActionTest, FakeVehiclePartIsCompatibleWithSlot) {
  const auto config = PartConfigFromPart("vehicle", *part_);
  const auto signature = GetBoundedBodyWrenchSignature();
  const SlotPartCompatibility compat =
      PartCompatibleWithSlot(config, signature.part_slot_infos().at("vehicle"));
  EXPECT_TRUE(compat.Compatible()) << compat.Explain();
}

TEST_F(BoundedBodyWrenchActionTest, MissingCapabilityFailsCreation) {
  const FeatureInterfaceTypes required[] = {
      FeatureInterfaceTypes::FEATURE_INTERFACE_BODY_STATE,
      FeatureInterfaceTypes::FEATURE_INTERFACE_BODY_WRENCH,
      FeatureInterfaceTypes::FEATURE_INTERFACE_VEHICLE_LIMITS,
  };
  for (const FeatureInterfaceTypes missing : required) {
    intrinsic_proto::icon::v1::PartConfig config;
    config.set_name("vehicle");
    for (const FeatureInterfaceTypes present : required) {
      if (present != missing) config.add_feature_interfaces(present);
    }
    ActionHarness harness(&part_->GetFeatureInterfaces(), config);
    const absl::Status status = harness.Create(WrenchParams(1, 1.0));
    EXPECT_EQ(status.code(), absl::StatusCode::kFailedPrecondition)
        << FeatureInterfaceTypes_Name(missing) << ": " << status;
    EXPECT_THAT(std::string(status.message()),
                HasSubstr(FeatureInterfaceTypes_Name(missing)));

    const auto signature = GetBoundedBodyWrenchSignature();
    const SlotPartCompatibility compat = PartCompatibleWithSlot(
        config, signature.part_slot_infos().at("vehicle"));
    EXPECT_FALSE(compat.Compatible());
    EXPECT_THAT(compat.Explain(),
                HasSubstr(FeatureInterfaceTypes_Name(missing)));
  }

  ActionHarness bare(&part_->GetFeatureInterfaces(), {});
  EXPECT_EQ(bare.Create(WrenchParams(1, 1.0)).code(),
            absl::StatusCode::kFailedPrecondition);
}

TEST_F(BoundedBodyWrenchActionTest, ValidFixedParamsAreAppliedToTheHal) {
  ASSERT_STATUS_OK(harness_->Create(WrenchParams(1, 2.5, -0.1)));
  ASSERT_THAT(harness_->OnEnter(), RealtimeIsOk());

  const CycleResult cycle = RunCycle();
  EXPECT_THAT(cycle.read, RealtimeIsOk());
  EXPECT_THAT(cycle.sense, RealtimeIsOk());
  EXPECT_THAT(cycle.control, RealtimeIsOk());
  EXPECT_THAT(cycle.apply, RealtimeIsOk());

  const BodyWrenchSample& recorded = part_->RecordedBodyWrench();
  EXPECT_EQ(recorded.sequence, 1);
  EXPECT_DOUBLE_EQ(recorded.force_x_n, 2.5);
  EXPECT_DOUBLE_EQ(recorded.force_y_n, 0.25);
  EXPECT_DOUBLE_EQ(recorded.force_z_n, 0.0);
  EXPECT_DOUBLE_EQ(recorded.torque_x_n_m, 0.0);
  EXPECT_DOUBLE_EQ(recorded.torque_y_n_m, 0.0);
  EXPECT_DOUBLE_EQ(recorded.torque_z_n_m, -0.1);
  EXPECT_EQ(View(recorded.frame_id), "body");
  EXPECT_EQ(View(recorded.source_id), "planner");
  EXPECT_EQ(View(recorded.clock_domain), "monotonic");
  EXPECT_TRUE(recorded.validity_present);
  EXPECT_EQ(recorded.validity_state, 1);
  EXPECT_TRUE(recorded.has_source_time);
  EXPECT_EQ(recorded.source_time_seconds, 1000);
  EXPECT_TRUE(recorded.has_receive_time);
  EXPECT_EQ(recorded.receive_time_seconds, 1000);
  EXPECT_FALSE(recorded.applied);

  const BodyWrenchCommand* wrench =
      part_->GetFeatureInterfaces().GetInterface<BodyWrenchCommand>();
  ASSERT_NE(wrench, nullptr);
  EXPECT_TRUE(wrench->PreviousBodyWrench().applied);
}

TEST_F(BoundedBodyWrenchActionTest, FixedParamsAreForwardedOnce) {
  ASSERT_STATUS_OK(harness_->Create(WrenchParams(1, 2.5)));
  ASSERT_THAT(harness_->OnEnter(), RealtimeIsOk());
  ASSERT_THAT(RunCycle().control, RealtimeIsOk());

  // A second forward of sequence 1 would be rejected by the part. The action
  // must not re-send, so the following cycle stays OK and holds the wrench.
  const CycleResult held = RunCycle();
  EXPECT_THAT(held.control, RealtimeIsOk());
  EXPECT_THAT(held.apply, RealtimeIsOk());
  EXPECT_DOUBLE_EQ(part_->RecordedBodyWrench().force_x_n, 2.5);
  EXPECT_EQ(part_->RecordedBodyWrench().sequence, 1);
}

TEST_F(BoundedBodyWrenchActionTest, StreamingInputReplacesTheWrench) {
  ASSERT_STATUS_OK(harness_->Create(WrenchParams(1, 2.5)));
  ASSERT_THAT(harness_->OnEnter(), RealtimeIsOk());
  ASSERT_THAT(RunCycle().apply, RealtimeIsOk());

  ASSERT_STATUS_OK(harness_->Stream(WrenchParams(2, 3.5, 1.0)));
  const CycleResult cycle = RunCycle();
  EXPECT_THAT(cycle.sense, RealtimeIsOk());
  EXPECT_THAT(cycle.control, RealtimeIsOk());
  EXPECT_THAT(cycle.apply, RealtimeIsOk());
  EXPECT_EQ(part_->RecordedBodyWrench().sequence, 2);
  EXPECT_DOUBLE_EQ(part_->RecordedBodyWrench().force_x_n, 3.5);
  EXPECT_DOUBLE_EQ(part_->RecordedBodyWrench().torque_z_n_m, 1.0);
}

TEST_F(BoundedBodyWrenchActionTest, NonFiniteWrenchIsRejectedNotApplied) {
  ASSERT_STATUS_OK(harness_->Create(WrenchParams(1, 1.0)));
  ASSERT_THAT(harness_->OnEnter(), RealtimeIsOk());
  ASSERT_THAT(RunCycle().apply, RealtimeIsOk());

  const double kNan = std::numeric_limits<double>::quiet_NaN();
  const double kInf = std::numeric_limits<double>::infinity();

  ASSERT_STATUS_OK(harness_->Stream(WrenchParams(2, kNan)));
  CycleResult nan_cycle = RunCycle();
  EXPECT_EQ(nan_cycle.control.code(), absl::StatusCode::kInvalidArgument);
  EXPECT_EQ(nan_cycle.control.message(), "wrench is non-finite");
  EXPECT_THAT(nan_cycle.apply, RealtimeIsOk());
  EXPECT_EQ(part_->RecordedBodyWrench().sequence, 1);
  EXPECT_DOUBLE_EQ(part_->RecordedBodyWrench().force_x_n, 1.0);

  ASSERT_STATUS_OK(harness_->Stream(WrenchParams(3, 1.0, kInf)));
  CycleResult inf_cycle = RunCycle();
  EXPECT_EQ(inf_cycle.control.code(), absl::StatusCode::kInvalidArgument);
  EXPECT_EQ(inf_cycle.control.message(), "wrench is non-finite");
  EXPECT_TRUE(std::isfinite(part_->RecordedBodyWrench().torque_z_n_m));
  EXPECT_EQ(part_->RecordedBodyWrench().sequence, 1);
}

TEST_F(BoundedBodyWrenchActionTest, NonFiniteFirstWrenchRecordsNeutral) {
  ASSERT_STATUS_OK(harness_->Create(
      WrenchParams(1, std::numeric_limits<double>::quiet_NaN())));
  ASSERT_THAT(harness_->OnEnter(), RealtimeIsOk());
  const CycleResult cycle = RunCycle();
  EXPECT_EQ(cycle.control.code(), absl::StatusCode::kInvalidArgument);
  EXPECT_THAT(cycle.apply, RealtimeIsOk());
  ExpectNeutralRecorded();
}

TEST_F(BoundedBodyWrenchActionTest, StaleCommandIsRejected) {
  // Receive stamp 200 ms after the source stamp exceeds max_command_age.
  ASSERT_STATUS_OK(harness_->Create(WrenchParams(1, 2.0, 0.0, /*age_ms=*/200)));
  ASSERT_THAT(harness_->OnEnter(), RealtimeIsOk());
  const CycleResult cycle = RunCycle();
  EXPECT_EQ(cycle.control.code(), absl::StatusCode::kDeadlineExceeded);
  EXPECT_EQ(cycle.control.message(), "body wrench is stale");
  EXPECT_THAT(cycle.apply, RealtimeIsOk());
  ExpectNeutralRecorded();
}

TEST_F(BoundedBodyWrenchActionTest, StaleBodyStateRecordsNeutral) {
  ASSERT_STATUS_OK(harness_->Create(WrenchParams(1, 2.0)));
  ASSERT_THAT(harness_->OnEnter(), RealtimeIsOk());
  const CycleResult cycle = RunCycle(/*state_age=*/Milliseconds(51));
  EXPECT_THAT(cycle.sense, RealtimeIsOk());
  EXPECT_EQ(cycle.control.code(), absl::StatusCode::kDeadlineExceeded);
  EXPECT_EQ(cycle.control.message(), "body state is stale");
  EXPECT_EQ(cycle.apply.code(), absl::StatusCode::kDeadlineExceeded);
  EXPECT_EQ(cycle.apply.message(), "body state is stale");
  ExpectNeutralRecorded();
}

TEST_F(BoundedBodyWrenchActionTest, ApplicationLimitViolationIsRejected) {
  ASSERT_STATUS_OK(harness_->Create(WrenchParams(1, 11.0)));
  ASSERT_THAT(harness_->OnEnter(), RealtimeIsOk());
  CycleResult force = RunCycle();
  EXPECT_EQ(force.control.code(), absl::StatusCode::kOutOfRange);
  EXPECT_EQ(force.control.message(), "wrench exceeds application limits");
  EXPECT_THAT(force.apply, RealtimeIsOk());
  // Rejected, not clamped to the 10 N application limit.
  ExpectNeutralRecorded();

  ASSERT_STATUS_OK(harness_->Stream(WrenchParams(2, 1.0, /*torque_z=*/6.0)));
  CycleResult torque = RunCycle();
  EXPECT_EQ(torque.control.code(), absl::StatusCode::kOutOfRange);
  ExpectNeutralRecorded();
}

TEST_F(BoundedBodyWrenchActionTest, SystemLimitViolationIsRejected) {
  ASSERT_STATUS_OK(harness_->Create(WrenchParams(1, 21.0)));
  ASSERT_THAT(harness_->OnEnter(), RealtimeIsOk());
  const CycleResult cycle = RunCycle();
  EXPECT_EQ(cycle.control.code(), absl::StatusCode::kFailedPrecondition);
  EXPECT_EQ(cycle.control.message(), "wrench exceeds system limits");
  ExpectNeutralRecorded();
}

TEST_F(BoundedBodyWrenchActionTest, EmptyBodyWrenchIsNotACommand) {
  BoundedBodyWrenchInfo::FixedParams unset;
  EXPECT_EQ(harness_->Create(unset).code(), absl::StatusCode::kInvalidArgument);

  BoundedBodyWrenchInfo::FixedParams empty;
  empty.mutable_wrench();
  EXPECT_EQ(harness_->Create(empty).code(), absl::StatusCode::kInvalidArgument);

  ASSERT_STATUS_OK(harness_->Create(WrenchParams(1, 1.0)));
  EXPECT_EQ(harness_->Stream(unset).code(), absl::StatusCode::kInvalidArgument);
  EXPECT_EQ(harness_->Stream(empty).code(), absl::StatusCode::kInvalidArgument);

  ASSERT_THAT(harness_->OnEnter(), RealtimeIsOk());
  ASSERT_THAT(RunCycle().apply, RealtimeIsOk());
  // The rejected streaming inputs did not replace the applied wrench.
  EXPECT_DOUBLE_EQ(part_->RecordedBodyWrench().force_x_n, 1.0);
  const CycleResult after = RunCycle();
  EXPECT_THAT(after.control, RealtimeIsOk());
  EXPECT_EQ(part_->RecordedBodyWrench().sequence, 1);
}

TEST_F(BoundedBodyWrenchActionTest, PresentZeroWrenchIsTheNeutralCommand) {
  ASSERT_STATUS_OK(harness_->Create(WrenchParams(1, 4.0)));
  ASSERT_THAT(harness_->OnEnter(), RealtimeIsOk());
  ASSERT_THAT(RunCycle().apply, RealtimeIsOk());
  ASSERT_DOUBLE_EQ(part_->RecordedBodyWrench().force_x_n, 4.0);

  BoundedBodyWrenchInfo::FixedParams zero = WrenchParams(2, 0.0);
  zero.mutable_wrench()->set_force_y_n(0.0);
  ASSERT_STATUS_OK(harness_->Stream(zero));
  const CycleResult cycle = RunCycle();
  EXPECT_THAT(cycle.control, RealtimeIsOk());
  EXPECT_EQ(part_->RecordedBodyWrench().sequence, 2);
  ExpectNeutralRecorded();
}

TEST_F(BoundedBodyWrenchActionTest, OverlongIdIsRejectedWhenParsing) {
  BoundedBodyWrenchInfo::FixedParams params = WrenchParams(1, 1.0);
  params.mutable_wrench()->mutable_header()->set_source_id(
      std::string(65, 'x'));
  const auto parsed = BoundedBodyWrenchAction::ParseStreamingInput(params);
  EXPECT_EQ(parsed.status().code(), absl::StatusCode::kInvalidArgument);
}

TEST_F(BoundedBodyWrenchActionTest, IdenticalInputsGiveIdenticalOutputs) {
  struct Trace {
    std::vector<int> codes;
    std::vector<uint64_t> sequences;
    std::vector<double> forces;
  };
  const auto run = [this]() {
    harness_.reset();
    Clock::setClockImpl(nullptr);
    clock_ = std::make_shared<ManualClock>(Time(Seconds(1000)));
    Clock::setClockImpl(clock_);
    auto part = FakeVehicleRealtimePart::Create(ForceTorqueLimits(10.0, 5.0),
                                                ForceTorqueLimits(20.0, 8.0));
    EXPECT_TRUE(part.ok()) << part.status();
    part_ = std::move(*part);
    harness_ = std::make_unique<ActionHarness>(
        &part_->GetFeatureInterfaces(), PartConfigFromPart("vehicle", *part_));
    state_seq_ = 0;

    Trace trace;
    EXPECT_STATUS_OK(harness_->Create(WrenchParams(1, 1.0)));
    EXPECT_THAT(harness_->OnEnter(), RealtimeIsOk());
    const std::vector<BoundedBodyWrenchInfo::FixedParams> script = {
        WrenchParams(2, 2.0),
        WrenchParams(3, 11.0),
        WrenchParams(4, std::numeric_limits<double>::quiet_NaN()),
        WrenchParams(5, 3.0),
        WrenchParams(5, 4.0),
        WrenchParams(6, 21.0),
        WrenchParams(7, 5.0, 0.0, 200),
        WrenchParams(8, 6.0),
    };
    for (size_t i = 0; i <= script.size(); ++i) {
      if (i > 0) {
        EXPECT_STATUS_OK(harness_->Stream(script[i - 1]));
      }
      const CycleResult cycle = RunCycle();
      trace.codes.push_back(static_cast<int>(cycle.control.code()));
      trace.codes.push_back(static_cast<int>(cycle.apply.code()));
      trace.sequences.push_back(part_->RecordedBodyWrench().sequence);
      trace.forces.push_back(part_->RecordedBodyWrench().force_x_n);
      clock_->Advance(Milliseconds(10));
    }
    return trace;
  };

  const Trace first = run();
  const Trace second = run();
  EXPECT_EQ(first.codes, second.codes);
  EXPECT_EQ(first.sequences, second.sequences);
  EXPECT_EQ(first.forces, second.forces);
  EXPECT_FALSE(first.codes.empty());
}

class SpyVehicle final : public BodyState,
                         public BodyWrenchCommand,
                         public VehicleLimitsInterface {
 public:
  const BodyStateSample& LatchedBodyState() const override { return state_; }
  RealtimeStatus SetBodyWrench(const BodyWrenchSample& command) override {
    ++calls_;
    last_ = command;
    return result_;
  }
  const BodyWrenchSample& PreviousBodyWrench() const override { return last_; }
  const VehicleLimitsSample& GetApplicationLimits() const override {
    return limits_;
  }
  const VehicleLimitsSample& GetSystemLimits() const override {
    return limits_;
  }

  int calls_ = 0;
  BodyWrenchSample last_;
  RealtimeStatus result_ = OkStatus();

 private:
  BodyStateSample state_;
  VehicleLimitsSample limits_;
};

TEST_F(BoundedBodyWrenchActionTest, ControlOnlyForwardsToSetBodyWrench) {
  SpyVehicle spy;
  FeatureInterfaceRegistry registry;
  ASSERT_THAT(registry.RegisterInterface(static_cast<BodyState*>(&spy)),
              RealtimeIsOk());
  ASSERT_THAT(registry.RegisterInterface(static_cast<BodyWrenchCommand*>(&spy)),
              RealtimeIsOk());
  ASSERT_THAT(
      registry.RegisterInterface(static_cast<VehicleLimitsInterface*>(&spy)),
      RealtimeIsOk());
  ActionHarness harness(&registry, PartConfigFromPart("vehicle", *part_));

  // Values the part would reject. The action must hand them over unchanged and
  // must not check them itself.
  BoundedBodyWrenchInfo::FixedParams params = WrenchParams(
      9, std::numeric_limits<double>::quiet_NaN(), /*torque_z=*/1e9);
  params.mutable_wrench()->mutable_header()->set_frame_id("not_body");
  ASSERT_STATUS_OK(harness.Create(params));
  ASSERT_THAT(harness.OnEnter(), RealtimeIsOk());

  spy.result_ = DataLossError("spy status");
  EXPECT_THAT(harness.Sense(), RealtimeIsOk());
  const RealtimeStatus control = harness.Control();
  EXPECT_EQ(spy.calls_, 1);
  EXPECT_EQ(control.code(), absl::StatusCode::kDataLoss);
  EXPECT_EQ(control.message(), "spy status");
  EXPECT_EQ(spy.last_.sequence, 9);
  EXPECT_TRUE(std::isnan(spy.last_.force_x_n));
  EXPECT_DOUBLE_EQ(spy.last_.torque_z_n_m, 1e9);
  EXPECT_EQ(View(spy.last_.frame_id), "not_body");

  EXPECT_THAT(harness.Sense(), RealtimeIsOk());
  EXPECT_THAT(harness.Control(), RealtimeIsOk());
  EXPECT_EQ(spy.calls_, 1);
}

TEST_F(BoundedBodyWrenchActionTest, SenseAndControlReportMissingInterfaces) {
  FeatureInterfaceRegistry empty_registry;
  ActionHarness harness(&empty_registry, PartConfigFromPart("vehicle", *part_));
  ASSERT_STATUS_OK(harness.Create(WrenchParams(1, 1.0)));
  ASSERT_THAT(harness.OnEnter(), RealtimeIsOk());
  EXPECT_EQ(harness.Sense().code(), absl::StatusCode::kInternal);
  EXPECT_EQ(harness.Control().code(), absl::StatusCode::kInternal);
}

TEST_F(BoundedBodyWrenchActionTest, UnknownStateVariableIsNotFound) {
  ASSERT_STATUS_OK(harness_->Create(WrenchParams(1, 1.0)));
  EXPECT_EQ(harness_->action().GetStateVariable("anything").status().code(),
            absl::StatusCode::kNotFound);
}

TEST_F(BoundedBodyWrenchActionTest, CycleMethodsDoNotAllocate) {
  ASSERT_STATUS_OK(harness_->Create(WrenchParams(1, 1.0)));
  ASSERT_STATUS_OK(harness_->Stream(WrenchParams(2, 2.0)));
  ASSERT_THAT(part_->SetBodyState(ValidState(NowNs(), 1)), RealtimeIsOk());
  ASSERT_THAT(part_->ReadStatusForTest(), RealtimeIsOk());

  IF_INTRINSIC_MALLOC_TEST_INIT_COUNTER();
  const RealtimeStatus on_enter = harness_->OnEnter();
  const RealtimeStatus sense = harness_->Sense();
  const RealtimeStatus control = harness_->Control();
  IF_INTRINSIC_MALLOC_TEST_EXPECT_NO_ALLOCATIONS();

  EXPECT_THAT(on_enter, RealtimeIsOk());
  EXPECT_THAT(sense, RealtimeIsOk());
  EXPECT_THAT(control, RealtimeIsOk());

  ASSERT_THAT(part_->ApplyCommandForTest(), RealtimeIsOk());
  EXPECT_EQ(part_->RecordedBodyWrench().sequence, 2);
}

}  // namespace
}  // namespace intrinsic::icon
