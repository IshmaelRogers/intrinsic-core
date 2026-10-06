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

#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <string>
#include <string_view>
#include <vector>

#include "absl/container/fixed_array.h"
#include "absl/container/flat_hash_map.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "gmock/gmock.h"
#include "gtest/gtest.h"
#include "intrinsic/icon/actions/body_wrench_action_info.h"
#include "intrinsic/icon/actions/body_wrench_action_signature.h"
#include "intrinsic/icon/common/id_types.h"
#include "intrinsic/icon/control/action_factory_context.h"
#include "intrinsic/icon/control/actions/test_helpers.h"
#include "intrinsic/icon/control/parts/feature_interface_registry.h"
#include "intrinsic/icon/control/parts/feature_interfaces.h"
#include "intrinsic/icon/control/parts/testing/fake_vehicle_realtime_part.h"
#include "intrinsic/icon/control/realtime_signal_access.h"
#include "intrinsic/icon/control/realtime_signal_storage.h"
#include "intrinsic/icon/control/realtime_slot_map.h"
#include "intrinsic/icon/control/rtcl_action.h"
#include "intrinsic/icon/control/slot_types.h"
#include "intrinsic/icon/control/streaming_io_helpers.h"
#include "intrinsic/icon/control/streaming_io_realtime.h"
#include "intrinsic/icon/control/streaming_io_storage.h"
#include "intrinsic/icon/proto/v1/types.pb.h"
#include "intrinsic/icon/testing/malloc_test.h"
#include "intrinsic/icon/utils/clock.h"
#include "intrinsic/icon/utils/realtime_status.h"
#include "intrinsic/icon/utils/realtime_status_matchers.h"
#include "intrinsic/kinematics/types/joint_trajectories.h"
#include "intrinsic/vehicle/proto/vehicle_command.pb.h"

namespace intrinsic::icon {
namespace {

using ::intrinsic_proto::embodiment::Validity;
using ::intrinsic_proto::icon::v1::FeatureInterfaceTypes;
using ::intrinsic_proto::vehicle::BodyWrench;

const RealtimeSlotId kSlotId(7);
constexpr std::string_view kPartName = "vehicle";

class ManualClock : public Clock::IClockDriver {
 public:
  explicit ManualClock(Time start) : now_(start) {}

  Time now() const override { return now_; }

  void Advance(Duration step) { now_ += step; }

 private:
  Time now_;
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

std::string_view IdView(const FixedId64& id) {
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
  return sample;
}

// A well-formed command: body frame, monotonic stamps with zero age, valid.
BodyWrench Command(uint64_t sequence, double force_x, double torque_z = 0.0) {
  BodyWrench wrench;
  auto* header = wrench.mutable_header();
  header->set_sequence(sequence);
  header->mutable_source_time()->set_seconds(1000);
  header->mutable_receive_time()->set_seconds(1000);
  header->set_source_id("planner");
  header->set_frame_id("body");
  header->set_clock_domain("monotonic");
  header->mutable_validity()->set_state(Validity::STATE_VALID);
  wrench.set_force_x_n(force_x);
  wrench.set_force_y_n(0.25);
  wrench.set_torque_z_n_m(torque_z);
  return wrench;
}

class TestSlotMap final : public RealtimeSlotMapInterface {
 public:
  explicit TestSlotMap(FeatureInterfaceRegistry* registry)
      : registry_(registry) {}

  FeatureInterfaceRegistry* GetMutableRegistryForSlot(
      RealtimeSlotId slot_id) override {
    return slot_id == kSlotId ? registry_ : nullptr;
  }
  const FeatureInterfaceRegistry* GetRegistryForSlot(
      RealtimeSlotId slot_id) const override {
    return slot_id == kSlotId ? registry_ : nullptr;
  }

 private:
  FeatureInterfaceRegistry* registry_;
};

// Result of one control cycle: the action's Control() status and the fake
// part's ApplyCommand() status.
struct CycleResult {
  RealtimeStatus control = OkStatus();
  RealtimeStatus apply = OkStatus();
};

class BodyWrenchActionTest : public ::testing::Test {
 protected:
  void SetUp() override { ResetFixture(); }

  // Rebuilds the clock, the fake part, and all action state. Tests that
  // replay a scenario call this between runs.
  void ResetFixture() {
    action_.reset();
    slot_map_.reset();
    slot_map_impl_.reset();
    rt_storage_.reset();
    context_.reset();
    storage_.reset();
    part_.reset();
    state_sequence_ = 0;
    clock_ = std::make_shared<ManualClock>(Time(Seconds(1000)));
    Clock::setClockImpl(clock_);
    auto part = FakeVehicleRealtimePart::Create(ForceTorqueLimits(10.0, 5.0),
                                                ForceTorqueLimits(20.0, 8.0));
    ASSERT_TRUE(part.ok()) << part.status();
    part_ = std::move(*part);
  }

  void TearDown() override { Clock::setClockImpl(nullptr); }

  int64_t NowNs() const { return Clock::ToNSec(Clock::Now()); }

  // Builds the factory context for a part config with the given feature
  // interfaces and runs the factory.
  absl::StatusOr<std::unique_ptr<BodyWrenchAction>> CreateAction(
      const intrinsic_proto::icon::v1::PartConfig& part_config) {
    signature_ = GetBodyWrenchActionSignature();
    storage_ =
        std::make_unique<StreamingIoStorage>(ActionInstanceId(1), signature_);
    slot_infos_.clear();
    slot_infos_.emplace(BodyWrenchActionInfo::kSlotName,
                        SlotInfo{.config = part_config, .slot_id = kSlotId});
    context_ = std::make_unique<ActionFactoryContext>(
        server_config_, signature_, slot_infos_,
        absl::flat_hash_map<std::string, RealtimeSignalId>{}, *storage_,
        ActionInstanceId(1), trajectories_);
    auto action = BodyWrenchAction::Create(*context_);
    if (action.ok()) {
      EXPECT_TRUE(context_->Validate().ok());
    }
    return action;
  }

  intrinsic_proto::icon::v1::PartConfig FullPartConfig() const {
    return PartConfigFromPart(kPartName, *part_);
  }

  void StartAction() {
    auto action = CreateAction(FullPartConfig());
    ASSERT_TRUE(action.ok()) << action.status();
    action_ = std::move(*action);
    slot_map_impl_ =
        std::make_unique<TestSlotMap>(&part_->GetFeatureInterfaces());
    slot_map_ = std::make_unique<RealtimeSlotMap>(*slot_map_impl_);
    rt_storage_ = std::make_unique<RealtimeStreamingIoStorage>(
        FromStreamingIoStorage(*storage_));
    ASSERT_THAT(
        action_->OnEnter(RtclActionInterface::OnEnterParameters{*slot_map_}),
        RealtimeIsOk());
  }

  absl::Status Stream(const BodyWrench& command) {
    return WriteStreamingInput<BodyWrench>(
        BodyWrenchActionInfo::kStreamingInputName, command, *storage_);
  }

  RealtimeStatus SenseOnly() {
    StreamingIoRealtimeAccess streaming_access(Clock::Now(), *rt_storage_);
    RealtimeSignalStorage signal_storage{absl::FixedArray<SignalValue>(0)};
    RealtimeSignalAccess signal_access(signal_storage);
    return action_->Sense(RtclActionInterface::SenseParameters{
        *slot_map_, streaming_access, signal_access});
  }

  RealtimeStatus ControlOnly() {
    return action_->Control(RtclActionInterface::ControlParameters{*slot_map_});
  }

  // One full cycle in the documented ICON order: part ReadStatus, action
  // Sense and Control, part ApplyCommand. The body state is refreshed first so
  // it is never stale unless a test advances the clock afterwards.
  CycleResult Cycle(bool refresh_state = true) {
    if (refresh_state) {
      EXPECT_THAT(part_->SetBodyState(ValidState(NowNs(), ++state_sequence_)),
                  RealtimeIsOk());
    }
    EXPECT_THAT(part_->ReadStatusForTest(), RealtimeIsOk());
    EXPECT_THAT(SenseOnly(), RealtimeIsOk());
    CycleResult result;
    result.control = ControlOnly();
    result.apply = part_->ApplyCommandForTest();
    return result;
  }

  std::shared_ptr<ManualClock> clock_;
  std::unique_ptr<FakeVehicleRealtimePart> part_;
  intrinsic_proto::icon::v1::ServerConfig server_config_;
  intrinsic_proto::icon::v1::ActionSignature signature_;
  absl::flat_hash_map<std::string, SlotInfo> slot_infos_;
  absl::flat_hash_map<ActionInstanceId, JointTrajectoryPVA> trajectories_;
  std::unique_ptr<StreamingIoStorage> storage_;
  std::unique_ptr<ActionFactoryContext> context_;
  std::unique_ptr<BodyWrenchAction> action_;
  std::unique_ptr<TestSlotMap> slot_map_impl_;
  std::unique_ptr<RealtimeSlotMap> slot_map_;
  std::unique_ptr<RealtimeStreamingIoStorage> rt_storage_;
  uint64_t state_sequence_ = 0;
};

void ExpectNeutral(const BodyWrenchSample& wrench) {
  EXPECT_DOUBLE_EQ(wrench.force_x_n, 0.0);
  EXPECT_DOUBLE_EQ(wrench.force_y_n, 0.0);
  EXPECT_DOUBLE_EQ(wrench.force_z_n, 0.0);
  EXPECT_DOUBLE_EQ(wrench.torque_x_n_m, 0.0);
  EXPECT_DOUBLE_EQ(wrench.torque_y_n_m, 0.0);
  EXPECT_DOUBLE_EQ(wrench.torque_z_n_m, 0.0);
  EXPECT_EQ(IdView(wrench.frame_id), "body");
}

void ExpectForces(const BodyWrenchSample& wrench, double force_x,
                  double force_y, double torque_z) {
  EXPECT_DOUBLE_EQ(wrench.force_x_n, force_x);
  EXPECT_DOUBLE_EQ(wrench.force_y_n, force_y);
  EXPECT_DOUBLE_EQ(wrench.torque_z_n_m, torque_z);
}

// 1. Missing capability.

TEST_F(BodyWrenchActionTest, SignatureRequiresTheThreeVehicleInterfaces) {
  const auto signature = GetBodyWrenchActionSignature();
  const auto& slot =
      signature.part_slot_infos().at(BodyWrenchActionInfo::kSlotName);
  EXPECT_THAT(slot.required_feature_interfaces(),
              ::testing::UnorderedElementsAre(
                  FeatureInterfaceTypes::FEATURE_INTERFACE_BODY_STATE,
                  FeatureInterfaceTypes::FEATURE_INTERFACE_VEHICLE_LIMITS,
                  FeatureInterfaceTypes::FEATURE_INTERFACE_BODY_WRENCH));
  EXPECT_TRUE(signature.fixed_parameters_message_type().empty());
  ASSERT_EQ(signature.streaming_input_infos_size(), 1);
  EXPECT_EQ(signature.streaming_input_infos(0).value_message_type(),
            "intrinsic_proto.vehicle.BodyWrench");
}

TEST_F(BodyWrenchActionTest, CreateSucceedsWhenAllThreeInterfacesArePresent) {
  EXPECT_TRUE(CreateAction(FullPartConfig()).ok());
}

TEST_F(BodyWrenchActionTest, CreateFailsWhenAnyInterfaceIsMissing) {
  struct Case {
    FeatureInterfaceTypes missing;
    std::string_view name;
  };
  const Case cases[] = {
      {FeatureInterfaceTypes::FEATURE_INTERFACE_BODY_STATE, "BodyState"},
      {FeatureInterfaceTypes::FEATURE_INTERFACE_VEHICLE_LIMITS,
       "VehicleLimitsInterface"},
      {FeatureInterfaceTypes::FEATURE_INTERFACE_BODY_WRENCH,
       "BodyWrenchCommand"},
  };
  for (const Case& test_case : cases) {
    const intrinsic_proto::icon::v1::PartConfig full = FullPartConfig();
    intrinsic_proto::icon::v1::PartConfig config = full;
    config.clear_feature_interfaces();
    for (const auto type : full.feature_interfaces()) {
      if (type != test_case.missing) {
        config.add_feature_interfaces(static_cast<FeatureInterfaceTypes>(type));
      }
    }
    const auto action = CreateAction(config);
    ASSERT_FALSE(action.ok()) << test_case.name;
    EXPECT_EQ(action.status().code(), absl::StatusCode::kFailedPrecondition);
    EXPECT_THAT(std::string(action.status().message()),
                ::testing::HasSubstr(std::string(test_case.name)))
        << action.status();
  }
}

TEST_F(BodyWrenchActionTest, ManipulatorStylePartConfigIsRejected) {
  intrinsic_proto::icon::v1::PartConfig config;
  config.set_name("arm");
  config.add_feature_interfaces(
      FeatureInterfaceTypes::FEATURE_INTERFACE_JOINT_POSITION);
  const auto action = CreateAction(config);
  ASSERT_FALSE(action.ok());
  EXPECT_EQ(action.status().code(), absl::StatusCode::kFailedPrecondition);
}

TEST_F(BodyWrenchActionTest, OnEnterFailsWhenRegistryLacksTheInterfaces) {
  auto action = CreateAction(FullPartConfig());
  ASSERT_TRUE(action.ok()) << action.status();
  FeatureInterfaceRegistry empty;
  TestSlotMap empty_map_impl(&empty);
  RealtimeSlotMap empty_map(empty_map_impl);
  const RealtimeStatus status =
      (*action)->OnEnter(RtclActionInterface::OnEnterParameters{empty_map});
  EXPECT_EQ(status.code(), absl::StatusCode::kFailedPrecondition);
}

// 2. Valid command.

TEST_F(BodyWrenchActionTest, ValidCommandIsLatchedAndRecorded) {
  StartAction();
  ASSERT_TRUE(Stream(Command(1, 4.5, -0.5)).ok());
  const CycleResult result = Cycle();
  EXPECT_THAT(result.control, RealtimeIsOk());
  EXPECT_THAT(result.apply, RealtimeIsOk());
  const BodyWrenchSample& recorded = part_->RecordedBodyWrench();
  ExpectForces(recorded, 4.5, 0.25, -0.5);
  EXPECT_EQ(recorded.sequence, 1u);
  EXPECT_EQ(IdView(recorded.frame_id), "body");
  EXPECT_EQ(IdView(recorded.source_id), "planner");
}

TEST_F(BodyWrenchActionTest, CommandIsHeldWhileFreshAndNotForwardedTwice) {
  StartAction();
  ASSERT_TRUE(Stream(Command(1, 4.0)).ok());
  EXPECT_THAT(Cycle().control, RealtimeIsOk());
  clock_->Advance(Milliseconds(10));
  const CycleResult hold = Cycle();
  EXPECT_THAT(hold.control, RealtimeIsOk());
  EXPECT_THAT(hold.apply, RealtimeIsOk());
  ExpectForces(part_->RecordedBodyWrench(), 4.0, 0.25, 0.0);
  EXPECT_EQ(part_->RecordedBodyWrench().sequence, 1u);
}

TEST_F(BodyWrenchActionTest, NoCommandBeforeFirstStreamIsNeutral) {
  StartAction();
  const CycleResult result = Cycle();
  EXPECT_THAT(result.control, RealtimeIsOk());
  EXPECT_THAT(result.apply, RealtimeIsOk());
  ExpectNeutral(part_->RecordedBodyWrench());
}

TEST_F(BodyWrenchActionTest, PresentZeroWrenchIsTheNeutralCommand) {
  StartAction();
  ASSERT_TRUE(Stream(Command(1, 4.0)).ok());
  ASSERT_THAT(Cycle().control, RealtimeIsOk());
  BodyWrench zero = Command(2, 0.0);
  zero.set_force_y_n(0.0);
  ASSERT_TRUE(Stream(zero).ok());
  clock_->Advance(Milliseconds(10));
  const CycleResult result = Cycle();
  EXPECT_THAT(result.control, RealtimeIsOk());
  EXPECT_THAT(result.apply, RealtimeIsOk());
  ExpectNeutral(part_->RecordedBodyWrench());
  EXPECT_EQ(part_->RecordedBodyWrench().sequence, 2u);
}

// 3. Non-finite command.

TEST_F(BodyWrenchActionTest, NonFiniteComponentIsRejectedAndNeutral) {
  struct Case {
    const char* field;
    void (*set)(BodyWrench&, double);
  };
  const Case cases[] = {
      {"force_x_n", [](BodyWrench& w, double v) { w.set_force_x_n(v); }},
      {"force_y_n", [](BodyWrench& w, double v) { w.set_force_y_n(v); }},
      {"force_z_n", [](BodyWrench& w, double v) { w.set_force_z_n(v); }},
      {"torque_x_n_m", [](BodyWrench& w, double v) { w.set_torque_x_n_m(v); }},
      {"torque_y_n_m", [](BodyWrench& w, double v) { w.set_torque_y_n_m(v); }},
      {"torque_z_n_m", [](BodyWrench& w, double v) { w.set_torque_z_n_m(v); }},
  };
  for (const double bad : {std::numeric_limits<double>::quiet_NaN(),
                           std::numeric_limits<double>::infinity()}) {
    for (const Case& test_case : cases) {
      SCOPED_TRACE(test_case.field);
      ResetFixture();
      StartAction();
      BodyWrench command = Command(1, 1.0);
      test_case.set(command, bad);
      ASSERT_TRUE(Stream(command).ok());
      const CycleResult result = Cycle();
      EXPECT_EQ(result.control.code(), absl::StatusCode::kInvalidArgument);
      EXPECT_THAT(std::string(result.control.message()),
                  ::testing::HasSubstr("non-finite"));
      EXPECT_THAT(result.apply, RealtimeIsOk());
      ExpectNeutral(part_->RecordedBodyWrench());
    }
  }
}

TEST_F(BodyWrenchActionTest, RejectedCommandKeepsHoldingThePreviousWrench) {
  StartAction();
  ASSERT_TRUE(Stream(Command(1, 3.0)).ok());
  ASSERT_THAT(Cycle().control, RealtimeIsOk());
  BodyWrench bad = Command(2, std::numeric_limits<double>::quiet_NaN());
  ASSERT_TRUE(Stream(bad).ok());
  clock_->Advance(Milliseconds(10));
  const CycleResult result = Cycle();
  EXPECT_EQ(result.control.code(), absl::StatusCode::kInvalidArgument);
  EXPECT_THAT(result.apply, RealtimeIsOk());
  ExpectForces(part_->RecordedBodyWrench(), 3.0, 0.25, 0.0);
  EXPECT_EQ(part_->RecordedBodyWrench().sequence, 1u);
}

TEST_F(BodyWrenchActionTest, WrongFrameIsRejectedByTheFeature) {
  StartAction();
  BodyWrench command = Command(1, 1.0);
  command.mutable_header()->set_frame_id("world_enu");
  ASSERT_TRUE(Stream(command).ok());
  const CycleResult result = Cycle();
  EXPECT_EQ(result.control.code(), absl::StatusCode::kInvalidArgument);
  ExpectNeutral(part_->RecordedBodyWrench());
}

// 4. Stale command.

TEST_F(BodyWrenchActionTest, StaleCommandIsRejectedAndNeutral) {
  StartAction();
  BodyWrench stale = Command(1, 2.0);
  stale.mutable_header()->mutable_receive_time()->set_nanos(100'000'000);
  ASSERT_TRUE(Stream(stale).ok());
  const CycleResult result = Cycle();
  EXPECT_EQ(result.control.code(), absl::StatusCode::kDeadlineExceeded);
  EXPECT_THAT(std::string(result.control.message()),
              ::testing::HasSubstr("stale"));
  EXPECT_THAT(result.apply, RealtimeIsOk());
  ExpectNeutral(part_->RecordedBodyWrench());
}

TEST_F(BodyWrenchActionTest, StaleCommandHoldsAFreshPreviousWrench) {
  StartAction();
  ASSERT_TRUE(Stream(Command(1, 2.0)).ok());
  ASSERT_THAT(Cycle().control, RealtimeIsOk());
  BodyWrench stale = Command(2, 9.0);
  stale.mutable_header()->mutable_receive_time()->set_nanos(100'000'000);
  ASSERT_TRUE(Stream(stale).ok());
  clock_->Advance(Milliseconds(10));
  const CycleResult result = Cycle();
  EXPECT_EQ(result.control.code(), absl::StatusCode::kDeadlineExceeded);
  ExpectForces(part_->RecordedBodyWrench(), 2.0, 0.25, 0.0);
}

TEST_F(BodyWrenchActionTest, StaleBodyStateRejectsTheCommand) {
  StartAction();
  ASSERT_TRUE(Stream(Command(1, 2.0)).ok());
  ASSERT_THAT(part_->SetBodyState(ValidState(NowNs(), 1)), RealtimeIsOk());
  clock_->Advance(Milliseconds(60));
  const CycleResult result = Cycle(/*refresh_state=*/false);
  EXPECT_EQ(result.control.code(), absl::StatusCode::kDeadlineExceeded);
  EXPECT_THAT(std::string(result.control.message()),
              ::testing::HasSubstr("stale"));
  ExpectNeutral(part_->RecordedBodyWrench());
}

// 5. Limit violation.

TEST_F(BodyWrenchActionTest, ApplicationLimitViolationIsRejectedNotClamped) {
  StartAction();
  ASSERT_TRUE(Stream(Command(1, 12.0)).ok());
  const CycleResult result = Cycle();
  EXPECT_EQ(result.control.code(), absl::StatusCode::kOutOfRange);
  EXPECT_THAT(result.apply, RealtimeIsOk());
  ExpectNeutral(part_->RecordedBodyWrench());
  EXPECT_NE(part_->RecordedBodyWrench().force_x_n, 10.0);
}

TEST_F(BodyWrenchActionTest, TorqueLimitViolationIsRejectedNotClamped) {
  StartAction();
  ASSERT_TRUE(Stream(Command(1, 1.0, /*torque_z=*/-6.0)).ok());
  const CycleResult result = Cycle();
  EXPECT_EQ(result.control.code(), absl::StatusCode::kOutOfRange);
  ExpectNeutral(part_->RecordedBodyWrench());
}

TEST_F(BodyWrenchActionTest, SystemLimitViolationIsRejected) {
  StartAction();
  ASSERT_TRUE(Stream(Command(1, 25.0)).ok());
  const CycleResult result = Cycle();
  EXPECT_EQ(result.control.code(), absl::StatusCode::kFailedPrecondition);
  ExpectNeutral(part_->RecordedBodyWrench());
}

TEST_F(BodyWrenchActionTest, ExactApplicationLimitIsAccepted) {
  StartAction();
  ASSERT_TRUE(Stream(Command(1, 10.0, 5.0)).ok());
  const CycleResult result = Cycle();
  EXPECT_THAT(result.control, RealtimeIsOk());
  ExpectForces(part_->RecordedBodyWrench(), 10.0, 0.25, 5.0);
}

// 6. Empty BodyWrench.

TEST_F(BodyWrenchActionTest, EmptyMessageIsNotACommand) {
  StartAction();
  const absl::Status status = Stream(BodyWrench());
  EXPECT_EQ(status.code(), absl::StatusCode::kInvalidArgument);
  EXPECT_THAT(std::string(status.message()), ::testing::HasSubstr("empty"));
  const CycleResult result = Cycle();
  EXPECT_THAT(result.control, RealtimeIsOk());
  ExpectNeutral(part_->RecordedBodyWrench());
}

TEST_F(BodyWrenchActionTest, EmptyMessageDoesNotReplaceAPendingOrHeldCommand) {
  StartAction();
  ASSERT_TRUE(Stream(Command(1, 3.0)).ok());
  ASSERT_THAT(Cycle().control, RealtimeIsOk());
  EXPECT_FALSE(Stream(BodyWrench()).ok());
  clock_->Advance(Milliseconds(10));
  const CycleResult result = Cycle();
  EXPECT_THAT(result.control, RealtimeIsOk());
  ExpectForces(part_->RecordedBodyWrench(), 3.0, 0.25, 0.0);
}

TEST(BodyWrenchActionParseTest, ConvertsEveryField) {
  BodyWrench proto = Command(42, 1.5, -2.5);
  proto.set_force_z_n(3.5);
  proto.set_torque_x_n_m(0.5);
  proto.set_torque_y_n_m(-0.75);
  proto.mutable_header()->mutable_source_time()->set_nanos(5);
  proto.mutable_header()->mutable_receive_time()->set_nanos(9);
  const auto sample = BodyWrenchAction::ParseStreamingInput(proto);
  ASSERT_TRUE(sample.ok()) << sample.status();
  EXPECT_FALSE(sample->applied);
  EXPECT_EQ(sample->sequence, 42u);
  EXPECT_TRUE(sample->has_source_time);
  EXPECT_EQ(sample->source_time_seconds, 1000);
  EXPECT_EQ(sample->source_time_nanos, 5);
  EXPECT_TRUE(sample->has_receive_time);
  EXPECT_EQ(sample->receive_time_seconds, 1000);
  EXPECT_EQ(sample->receive_time_nanos, 9);
  EXPECT_EQ(IdView(sample->source_id), "planner");
  EXPECT_EQ(IdView(sample->frame_id), "body");
  EXPECT_EQ(IdView(sample->clock_domain), "monotonic");
  EXPECT_TRUE(sample->validity_present);
  EXPECT_EQ(sample->validity_state, 1u);
  EXPECT_DOUBLE_EQ(sample->force_x_n, 1.5);
  EXPECT_DOUBLE_EQ(sample->force_y_n, 0.25);
  EXPECT_DOUBLE_EQ(sample->force_z_n, 3.5);
  EXPECT_DOUBLE_EQ(sample->torque_x_n_m, 0.5);
  EXPECT_DOUBLE_EQ(sample->torque_y_n_m, -0.75);
  EXPECT_DOUBLE_EQ(sample->torque_z_n_m, -2.5);
}

TEST(BodyWrenchActionParseTest, AbsentOptionalHeaderFieldsStayAbsent) {
  BodyWrench proto;
  proto.mutable_header()->set_frame_id("body");
  const auto sample = BodyWrenchAction::ParseStreamingInput(proto);
  ASSERT_TRUE(sample.ok()) << sample.status();
  EXPECT_FALSE(sample->has_source_time);
  EXPECT_FALSE(sample->has_receive_time);
  EXPECT_FALSE(sample->validity_present);
  EXPECT_EQ(sample->source_id.length, 0);
}

TEST(BodyWrenchActionParseTest, OverlongIdIsInvalidArgument) {
  BodyWrench proto = Command(1, 1.0);
  proto.mutable_header()->set_frame_id(std::string(65, 'x'));
  const auto sample = BodyWrenchAction::ParseStreamingInput(proto);
  EXPECT_EQ(sample.status().code(), absl::StatusCode::kInvalidArgument);
  EXPECT_THAT(std::string(sample.status().message()),
              ::testing::HasSubstr("frame_id"));
}

TEST(BodyWrenchActionParseTest, ParserDoesNotPreValidateNonFiniteValues) {
  BodyWrench proto = Command(1, std::numeric_limits<double>::infinity());
  const auto sample = BodyWrenchAction::ParseStreamingInput(proto);
  ASSERT_TRUE(sample.ok()) << sample.status();
  EXPECT_TRUE(std::isinf(sample->force_x_n));
}

// 7. No heap allocation in the cycle methods.

TEST_F(BodyWrenchActionTest, CycleMethodsDoNotAllocate) {
  StartAction();
  ASSERT_TRUE(Stream(Command(1, 4.0)).ok());
  ASSERT_THAT(part_->SetBodyState(ValidState(NowNs(), 1)), RealtimeIsOk());
  ASSERT_THAT(part_->ReadStatusForTest(), RealtimeIsOk());
  StreamingIoRealtimeAccess streaming_access(Clock::Now(), *rt_storage_);
  RealtimeSignalStorage signal_storage{absl::FixedArray<SignalValue>(0)};
  RealtimeSignalAccess signal_access(signal_storage);
  const RtclActionInterface::SenseParameters sense{*slot_map_, streaming_access,
                                                   signal_access};
  const RtclActionInterface::ControlParameters control{*slot_map_};

  IF_INTRINSIC_MALLOC_TEST_INIT_COUNTER();
  const RealtimeStatus sensed = action_->Sense(sense);
  const RealtimeStatus controlled = action_->Control(control);
  // A cycle with no new command, and a second Control() with nothing pending.
  const RealtimeStatus sensed_idle = action_->Sense(sense);
  const RealtimeStatus controlled_idle = action_->Control(control);
  INTRINSIC_RT_EXPECT_OK(sensed);
  INTRINSIC_RT_EXPECT_OK(controlled);
  INTRINSIC_RT_EXPECT_OK(sensed_idle);
  INTRINSIC_RT_EXPECT_OK(controlled_idle);
  IF_INTRINSIC_MALLOC_TEST_EXPECT_NO_ALLOCATIONS();
  ASSERT_THAT(part_->ApplyCommandForTest(), RealtimeIsOk());
  ExpectForces(part_->RecordedBodyWrench(), 4.0, 0.25, 0.0);
}

TEST_F(BodyWrenchActionTest, RejectingCycleDoesNotAllocate) {
  StartAction();
  ASSERT_TRUE(Stream(Command(1, 99.0)).ok());
  ASSERT_THAT(part_->SetBodyState(ValidState(NowNs(), 1)), RealtimeIsOk());
  ASSERT_THAT(part_->ReadStatusForTest(), RealtimeIsOk());
  StreamingIoRealtimeAccess streaming_access(Clock::Now(), *rt_storage_);
  RealtimeSignalStorage signal_storage{absl::FixedArray<SignalValue>(0)};
  RealtimeSignalAccess signal_access(signal_storage);
  const RtclActionInterface::SenseParameters sense{*slot_map_, streaming_access,
                                                   signal_access};
  const RtclActionInterface::ControlParameters control{*slot_map_};

  IF_INTRINSIC_MALLOC_TEST_INIT_COUNTER();
  const RealtimeStatus sensed = action_->Sense(sense);
  const RealtimeStatus controlled = action_->Control(control);
  INTRINSIC_RT_EXPECT_OK(sensed);
  EXPECT_EQ(controlled.code(), absl::StatusCode::kFailedPrecondition);
  IF_INTRINSIC_MALLOC_TEST_EXPECT_NO_ALLOCATIONS();
}

// 8. Determinism.

struct Trace {
  std::vector<absl::StatusCode> control_codes;
  std::vector<std::string> control_messages;
  std::vector<absl::StatusCode> apply_codes;
  std::vector<double> force_x;
  std::vector<double> force_y;
  std::vector<double> torque_z;
  std::vector<uint64_t> sequence;
};

TEST_F(BodyWrenchActionTest, IdenticalInputSequencesGiveIdenticalOutputs) {
  const auto run = [this]() {
    ResetFixture();
    StartAction();
    Trace trace;
    BodyWrench stale = Command(5, 1.0);
    stale.mutable_header()->mutable_receive_time()->set_nanos(100'000'000);
    const std::vector<BodyWrench> inputs = {
        Command(1, 1.0),
        Command(2, 12.0),
        Command(3, std::numeric_limits<double>::quiet_NaN()),
        Command(4, -4.0, 2.0),
        stale,
        Command(6, 25.0),
        Command(7, 0.0),
    };
    for (const BodyWrench& input : inputs) {
      EXPECT_TRUE(Stream(input).ok());
      const CycleResult result = Cycle();
      trace.control_codes.push_back(result.control.code());
      trace.control_messages.emplace_back(result.control.message());
      trace.apply_codes.push_back(result.apply.code());
      const BodyWrenchSample& recorded = part_->RecordedBodyWrench();
      trace.force_x.push_back(recorded.force_x_n);
      trace.force_y.push_back(recorded.force_y_n);
      trace.torque_z.push_back(recorded.torque_z_n_m);
      trace.sequence.push_back(recorded.sequence);
      clock_->Advance(Milliseconds(10));
    }
    return trace;
  };
  const Trace first = run();
  const Trace second = run();
  EXPECT_EQ(first.control_codes, second.control_codes);
  EXPECT_EQ(first.control_messages, second.control_messages);
  EXPECT_EQ(first.apply_codes, second.apply_codes);
  EXPECT_EQ(first.force_x, second.force_x);
  EXPECT_EQ(first.force_y, second.force_y);
  EXPECT_EQ(first.torque_z, second.torque_z);
  EXPECT_EQ(first.sequence, second.sequence);
  EXPECT_EQ(first.control_codes[0], absl::StatusCode::kOk);
  EXPECT_EQ(first.control_codes[1], absl::StatusCode::kOutOfRange);
  EXPECT_EQ(first.control_codes[2], absl::StatusCode::kInvalidArgument);
  EXPECT_EQ(first.control_codes[3], absl::StatusCode::kOk);
  EXPECT_EQ(first.control_codes[4], absl::StatusCode::kDeadlineExceeded);
  EXPECT_EQ(first.control_codes[5], absl::StatusCode::kFailedPrecondition);
  EXPECT_EQ(first.control_codes[6], absl::StatusCode::kOk);
}

TEST_F(BodyWrenchActionTest, StateVariablesAreNotRegistered) {
  StartAction();
  EXPECT_EQ(action_->GetStateVariable("anything").status().code(),
            absl::StatusCode::kNotFound);
}

}  // namespace
}  // namespace intrinsic::icon
