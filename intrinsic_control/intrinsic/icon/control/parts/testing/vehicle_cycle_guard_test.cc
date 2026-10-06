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

#include "intrinsic/icon/control/parts/testing/vehicle_cycle_guard.h"

#include <stdint.h>

#include <memory>
#include <string>
#include <string_view>
#include <utility>

#include "absl/container/fixed_array.h"
#include "absl/container/flat_hash_map.h"
#include "absl/status/status.h"
#include "absl/strings/str_cat.h"
#include "absl/strings/string_view.h"
#include "gmock/gmock.h"
#include "gtest/gtest.h"
#include "intrinsic/embodiment/proto/stamped_header.pb.h"
#include "intrinsic/icon/actions/bounded_body_wrench_info.h"
#include "intrinsic/icon/actions/bounded_body_wrench_signature.h"
#include "intrinsic/icon/common/id_types.h"
#include "intrinsic/icon/common/part_properties.h"
#include "intrinsic/icon/control/action_factory_context.h"
#include "intrinsic/icon/control/actions/bounded_body_wrench_action.h"
#include "intrinsic/icon/control/parts/feature_interface_registry.h"
#include "intrinsic/icon/control/parts/feature_interfaces.h"
#include "intrinsic/icon/control/parts/realtime_part_interface.h"
#include "intrinsic/icon/control/parts/realtime_part_property_access.h"
#include "intrinsic/icon/control/parts/testing/fake_realtime_part.h"
#include "intrinsic/icon/control/parts/testing/fake_vehicle_realtime_part.h"
#include "intrinsic/icon/control/realtime_signal_access.h"
#include "intrinsic/icon/control/realtime_signal_storage.h"
#include "intrinsic/icon/control/realtime_slot_map.h"
#include "intrinsic/icon/control/slot_types.h"
#include "intrinsic/icon/control/streaming_io_helpers.h"
#include "intrinsic/icon/control/streaming_io_realtime.h"
#include "intrinsic/icon/control/streaming_io_storage.h"
#include "intrinsic/icon/proto/v1/types.pb.h"
#include "intrinsic/icon/utils/clock.h"
#include "intrinsic/icon/utils/realtime_status.h"
#include "intrinsic/kinematics/types/joint_trajectories.h"
#include "intrinsic/vehicle/proto/vehicle_command.pb.h"

namespace intrinsic::icon {
namespace {

using ::intrinsic_proto::icon::v1::FeatureInterfaceTypes;
using ::testing::HasSubstr;

constexpr int kCycles = 100;
constexpr Duration kCycleBudget = Milliseconds(1);
constexpr Duration kScriptedCycle = Microseconds(100);

class ManualClock : public Clock::IClockDriver {
 public:
  explicit ManualClock(Time start) : now_(start) {}

  Time now() const override { return now_; }

  void Advance(Duration step) { now_ += step; }

 private:
  Time now_;
};

VehicleCycleGuardConfig TestConfig() {
  VehicleCycleGuardConfig config;
  config.max_allocations = 0;
  config.max_blocking_waits = 0;
  config.max_cycle = kCycleBudget;
  return config;
}

void ExpectDiagnostics(const absl::Status& status, int cycle_index,
                       absl::string_view call_path, int64_t budget,
                       int64_t observed, absl::string_view kind) {
  ASSERT_FALSE(status.ok()) << status;
  const std::string message(status.message());
  EXPECT_THAT(message, HasSubstr(absl::StrCat("cycle=", cycle_index, " ")));
  EXPECT_THAT(message, HasSubstr(absl::StrCat(" path=", call_path, " ")));
  EXPECT_THAT(message, HasSubstr(absl::StrCat(" budget=", budget, " ")));
  EXPECT_THAT(message, HasSubstr(absl::StrCat(" observed=", observed, " ")));
  EXPECT_THAT(message, HasSubstr(absl::StrCat(" kind=", kind)));
}

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

BoundedBodyWrenchInfo::FixedParams WrenchParams(uint64_t sequence,
                                                double force_x) {
  intrinsic_proto::vehicle::BodyWrench wrench;
  auto* header = wrench.mutable_header();
  header->set_sequence(sequence);
  header->mutable_source_time()->set_seconds(1000);
  header->mutable_receive_time()->set_seconds(1000);
  header->set_source_id("planner");
  header->set_frame_id("body");
  header->set_clock_domain("monotonic");
  header->mutable_validity()->set_state(
      intrinsic_proto::embodiment::Validity::STATE_VALID);
  wrench.set_force_x_n(force_x);
  wrench.set_force_y_n(0.25);
  return GetBoundedBodyWrenchFixedParams(wrench);
}

intrinsic_proto::icon::v1::PartConfig VehiclePartConfig() {
  intrinsic_proto::icon::v1::PartConfig config;
  config.set_name("vehicle");
  config.add_feature_interfaces(
      FeatureInterfaceTypes::FEATURE_INTERFACE_BODY_STATE);
  config.add_feature_interfaces(
      FeatureInterfaceTypes::FEATURE_INTERFACE_BODY_WRENCH);
  config.add_feature_interfaces(
      FeatureInterfaceTypes::FEATURE_INTERFACE_VEHICLE_LIMITS);
  return config;
}

// ICON server pieces for one intrinsic.bounded_body_wrench instance.
class ActionHarness {
 public:
  explicit ActionHarness(FeatureInterfaceRegistry* registry)
      : signature_(GetBoundedBodyWrenchSignature()),
        slot_map_impl_(RealtimeSlotId(0), registry),
        slot_map_(slot_map_impl_),
        storage_(ActionInstanceId(1), signature_) {
    SlotInfo slot_info;
    slot_info.config = VehiclePartConfig();
    slot_info.slot_id = RealtimeSlotId(0);
    slot_infos_[BoundedBodyWrenchInfo::kSlotName] = std::move(slot_info);
  }

  absl::Status Create(const BoundedBodyWrenchInfo::FixedParams& params) {
    auto signals = CreateRealtimeSignalStorage(signature_);
    if (!signals.ok()) {
      return signals.status();
    }
    signal_storage_ = std::make_unique<RealtimeSignalStorage>(
        std::move(signals->signal_storage));
    context_ = std::make_unique<ActionFactoryContext>(
        server_config_, signature_, slot_infos_, signals->signal_id_map,
        storage_, ActionInstanceId(1), trajectory_map_);
    auto action = BoundedBodyWrenchAction::Create(params, *context_);
    if (!action.ok()) {
      return action.status();
    }
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

// Fake vehicle part plus the bounded body-wrench action. Property storage is
// built once so the guarded cycle does not construct a FixedArray.
class VehicleStack {
 public:
  VehicleStack()
      : properties_in_(0),
        properties_out_(0),
        access_(properties_in_, properties_out_) {}

  absl::Status Init() {
    auto part = FakeVehicleRealtimePart::Create(ForceTorqueLimits(10.0, 5.0),
                                                ForceTorqueLimits(20.0, 8.0));
    if (!part.ok()) {
      return part.status();
    }
    part_ = std::move(*part);
    harness_ = std::make_unique<ActionHarness>(&part_->GetFeatureInterfaces());
    if (const absl::Status status = harness_->Create(WrenchParams(1, 1.0));
        !status.ok()) {
      return status;
    }
    return harness_->OnEnter();
  }

  // Non-realtime setup for cycle `cycle_index`. Sequence numbers start at 1.
  absl::Status Prepare(int cycle_index) {
    const uint64_t sequence = static_cast<uint64_t>(cycle_index) + 1;
    const RealtimeStatus state =
        part_->SetBodyState(ValidState(Clock::ToNSec(Clock::Now()), sequence));
    if (!state.ok()) {
      return state;
    }
    return harness_->Stream(WrenchParams(sequence, 1.0));
  }

  RealtimeStatus GuardedCycle(ManualClock* clock) {
    const RealtimeStatus read = part_->ReadStatus({.part_properties = access_});
    if (!read.ok()) {
      return read;
    }
    const RealtimeStatus sense = harness_->Sense();
    if (!sense.ok()) {
      return sense;
    }
    const RealtimeStatus control = harness_->Control();
    if (!control.ok()) {
      return control;
    }
    clock->Advance(kScriptedCycle);
    return part_->ApplyCommand({.part_properties = access_});
  }

  FakeVehicleRealtimePart& part() { return *part_; }

 private:
  absl::FixedArray<PartPropertyValue> properties_in_;
  absl::FixedArray<PartPropertyValue> properties_out_;
  RealtimePartPropertyAccess access_;
  std::unique_ptr<FakeVehicleRealtimePart> part_;
  std::unique_ptr<ActionHarness> harness_;
};

absl::Status RunStreamingVehicleCycles(VehicleStack* stack, ManualClock* clock,
                                       VehicleCycleGuard* guard, int cycles,
                                       absl::string_view call_path) {
  return guard->RunRepeated(
      cycles, call_path,
      [stack](int cycle_index) { return stack->Prepare(cycle_index); },
      [stack, clock](int) { return stack->GuardedCycle(clock); });
}

class VehicleCycleGuardTest : public ::testing::Test {
 protected:
  void SetUp() override {
    clock_ = std::make_shared<ManualClock>(Time(Seconds(1000)));
    Clock::setClockImpl(clock_);
  }

  void TearDown() override { Clock::setClockImpl(nullptr); }

  std::shared_ptr<ManualClock> clock_;
  int* allocation_sink_ = nullptr;
};

TEST_F(VehicleCycleGuardTest, NominalFakeVehicleCyclePassesBudgets) {
  VehicleStack stack;
  const absl::Status init = stack.Init();
  ASSERT_TRUE(init.ok()) << init;
  VehicleCycleGuard guard(TestConfig());
  ASSERT_LT(kScriptedCycle, guard.config().max_cycle);
  ASSERT_GE(kCycles, 100);

  const absl::Status status = RunStreamingVehicleCycles(
      &stack, clock_.get(), &guard, kCycles, "vehicle_icon_cycle");
  EXPECT_TRUE(status.ok()) << status;
  EXPECT_EQ(stack.part().RecordedBodyWrench().sequence,
            static_cast<uint64_t>(kCycles));
  EXPECT_DOUBLE_EQ(stack.part().RecordedBodyWrench().force_x_n, 1.0);
  EXPECT_NE(stack.part().GetFeatureInterfaces().GetInterface<BodyState>(),
            nullptr);
  EXPECT_NE(
      stack.part().GetFeatureInterfaces().GetInterface<BodyWrenchCommand>(),
      nullptr);
  EXPECT_NE(stack.part()
                .GetFeatureInterfaces()
                .GetInterface<VehicleLimitsInterface>(),
            nullptr);
}

TEST_F(VehicleCycleGuardTest, DetectsIntentionalHeapAllocation) {
  VehicleCycleGuard guard(TestConfig());
  const absl::Status status = guard.RunRepeated(
      5, "heap_allocation", [](int) { return absl::OkStatus(); },
      [&](int cycle_index) {
        if (cycle_index == 2) {
          allocation_sink_ = new int(cycle_index);
          delete allocation_sink_;
          allocation_sink_ = nullptr;
        }
        return OkStatus();
      });
  if (VehicleCycleGuard::AllocationChecksEnabled()) {
    ExpectDiagnostics(status, /*cycle_index=*/2, "heap_allocation",
                      /*budget=*/0, /*observed=*/1, "allocation");
  } else {
    EXPECT_TRUE(status.ok()) << status;
  }
}

TEST_F(VehicleCycleGuardTest, DetectsIntentionalMutexAndConditionWait) {
  VehicleCycleGuard guard(TestConfig());
  VehicleCycleMutex mutex;
  const absl::Status mutex_status = guard.Measure(2, "mutex_wait", [&] {
    mutex.Wait();
    return OkStatus();
  });
  ExpectDiagnostics(mutex_status, /*cycle_index=*/2, "mutex_wait",
                    /*budget=*/0, /*observed=*/1, "blocking");

  VehicleCycleConditionVariable condition;
  const absl::Status condition_status = guard.Measure(3, "condition_wait", [&] {
    condition.Wait();
    return OkStatus();
  });
  ExpectDiagnostics(condition_status, /*cycle_index=*/3, "condition_wait",
                    /*budget=*/0, /*observed=*/1, "blocking");
}

TEST_F(VehicleCycleGuardTest, DetectsUnboundedQueueGrowth) {
  VehicleCycleGuard guard(TestConfig());
  VehicleCycleUnboundedQueue queue;
  ASSERT_EQ(queue.size(), 0);
  const absl::Status status = guard.Measure(6, "unbounded_queue_push", [&] {
    queue.Push(1);
    return OkStatus();
  });
  ExpectDiagnostics(status, /*cycle_index=*/6, "unbounded_queue_push",
                    /*budget=*/0, /*observed=*/1, "blocking");
  EXPECT_EQ(queue.size(), 1);
}

TEST_F(VehicleCycleGuardTest, DetectsDeadlineOverrun) {
  VehicleCycleGuard guard(TestConfig());
  const Duration overrun = kCycleBudget + Nanoseconds(1);
  const absl::Status status = guard.RunRepeated(
      8, "deadline", [](int) { return absl::OkStatus(); },
      [&](int cycle_index) {
        if (cycle_index == 4) {
          clock_->Advance(overrun);
        }
        return OkStatus();
      });
  ExpectDiagnostics(status, /*cycle_index=*/4, "deadline",
                    ToInt64Nanoseconds(kCycleBudget),
                    ToInt64Nanoseconds(overrun), "deadline");
}

TEST_F(VehicleCycleGuardTest, DeadlineEqualToBudgetPasses) {
  VehicleCycleGuard guard(TestConfig());
  const absl::Status status = guard.Measure(0, "deadline_boundary", [&] {
    clock_->Advance(kCycleBudget);
    return OkStatus();
  });
  EXPECT_TRUE(status.ok()) << status;
}

TEST_F(VehicleCycleGuardTest, BoundaryEmptyFullQueueAndCycleLoad) {
  VehicleCycleGuard guard(TestConfig());
  ASSERT_EQ(guard.config().min_cycle_load, kVehicleCycleMinLoad);
  ASSERT_EQ(guard.config().max_cycle_load, kVehicleCycleMaxLoad);
  ASSERT_EQ(guard.config().max_cycle_load, VehicleCycleBoundedQueue::kCapacity);

  VehicleCycleBoundedQueue empty_queue;
  int min_drained = 0;
  const absl::Status empty_status = guard.RunRepeated(
      kCycles, "bounded_queue_empty", [](int) { return absl::OkStatus(); },
      [&](int) {
        min_drained += empty_queue.Drain(guard.config().min_cycle_load);
        return OkStatus();
      });
  EXPECT_TRUE(empty_status.ok()) << empty_status;
  EXPECT_TRUE(empty_queue.empty());
  EXPECT_EQ(min_drained, 0);

  VehicleCycleBoundedQueue full_queue;
  ASSERT_EQ(full_queue.Fill(), VehicleCycleBoundedQueue::kCapacity);
  ASSERT_TRUE(full_queue.full());
  int rejected = 0;
  const absl::Status full_status = guard.RunRepeated(
      kCycles, "bounded_queue_full", [](int) { return absl::OkStatus(); },
      [&](int) {
        if (!full_queue.TryPush(1)) {
          ++rejected;
        }
        return OkStatus();
      });
  EXPECT_TRUE(full_status.ok()) << full_status;
  EXPECT_EQ(rejected, kCycles);
  EXPECT_TRUE(full_queue.full());
  EXPECT_EQ(full_queue.size(), full_queue.capacity());

  VehicleCycleBoundedQueue load_queue;
  int max_drained = 0;
  const absl::Status max_status = guard.RunRepeated(
      kCycles, "cycle_load_max",
      [&](int) {
        if (load_queue.empty()) {
          load_queue.Fill();
        }
        return absl::OkStatus();
      },
      [&](int) {
        max_drained += load_queue.Drain(guard.config().max_cycle_load);
        return OkStatus();
      });
  EXPECT_TRUE(max_status.ok()) << max_status;
  EXPECT_EQ(max_drained, kCycles * guard.config().max_cycle_load);
  EXPECT_TRUE(load_queue.empty());
}

TEST_F(VehicleCycleGuardTest, IsolationVehicleFeaturesEnabled) {
  VehicleStack stack;
  const absl::Status init = stack.Init();
  ASSERT_TRUE(init.ok()) << init;
  ASSERT_NE(stack.part().GetFeatureInterfaces().GetInterface<BodyState>(),
            nullptr);
  ASSERT_NE(
      stack.part().GetFeatureInterfaces().GetInterface<BodyWrenchCommand>(),
      nullptr);
  ASSERT_NE(stack.part()
                .GetFeatureInterfaces()
                .GetInterface<VehicleLimitsInterface>(),
            nullptr);

  VehicleCycleGuard guard(TestConfig());
  const absl::Status status = RunStreamingVehicleCycles(
      &stack, clock_.get(), &guard, kCycles, "vehicle_features");
  EXPECT_TRUE(status.ok()) << status;
  EXPECT_EQ(stack.part().RecordedBodyWrench().sequence,
            static_cast<uint64_t>(kCycles));
}

TEST_F(VehicleCycleGuardTest, IsolationManipulatorOnlyFake) {
  // Existing generic part double with no vehicle feature interfaces. This is
  // the manipulator-only case that can be built without editing manipulator
  // sources. HalArmPart and joint features are not constructed.
  FakeRealtimePart<> arm;
  EXPECT_EQ(arm.GetFeatureInterfaces().GetInterface<BodyState>(), nullptr);
  EXPECT_EQ(arm.GetFeatureInterfaces().GetInterface<BodyWrenchCommand>(),
            nullptr);
  EXPECT_EQ(arm.GetFeatureInterfaces().GetInterface<VehicleLimitsInterface>(),
            nullptr);

  absl::FixedArray<PartPropertyValue> properties_in(0);
  absl::FixedArray<PartPropertyValue> properties_out(0);
  RealtimePartPropertyAccess access(properties_in, properties_out);
  VehicleCycleGuard guard(TestConfig());
  const absl::Status status = guard.RunRepeated(
      kCycles, "manipulator_only", [](int) { return absl::OkStatus(); },
      [&](int) {
        const RealtimeStatus read = arm.ReadStatus({.part_properties = access});
        if (!read.ok()) {
          return read;
        }
        return arm.ApplyCommand({.part_properties = access});
      });
  EXPECT_TRUE(status.ok()) << status;
}

}  // namespace
}  // namespace intrinsic::icon
