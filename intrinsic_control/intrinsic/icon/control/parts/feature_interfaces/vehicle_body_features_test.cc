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

#include "intrinsic/icon/control/parts/feature_interfaces/vehicle_body_features.h"

#include <cstdint>
#include <limits>
#include <memory>
#include <optional>
#include <string_view>
#include <utility>

#include "absl/container/fixed_array.h"
#include "absl/status/status.h"
#include "gtest/gtest.h"
#include "intrinsic/icon/common/part_properties.h"
#include "intrinsic/icon/control/parts/feature_interface_registry.h"
#include "intrinsic/icon/control/parts/feature_interfaces.h"
#include "intrinsic/icon/control/parts/realtime_part_interface.h"
#include "intrinsic/icon/control/realtime_bridge_types.h"
#include "intrinsic/icon/hal/hardware_interface_registry.h"
#include "intrinsic/icon/hal/interfaces/vehicle_hal.fbs.h"
#include "intrinsic/icon/hal/interfaces/vehicle_hal_utils.h"
#include "intrinsic/icon/hal/interfaces/vehicle_hardware_interfaces.h"
#include "intrinsic/icon/interprocess/shared_memory_manager/shared_memory_manager.h"
#include "intrinsic/icon/proto/v1/types.pb.h"
#include "intrinsic/icon/testing/malloc_test.h"
#include "intrinsic/icon/utils/clock.h"
#include "intrinsic/icon/utils/realtime_status.h"
#include "intrinsic/icon/utils/realtime_status_matchers.h"

namespace intrinsic::icon {
namespace {

class ManualClock : public Clock::IClockDriver {
 public:
  explicit ManualClock(Time start) : now_(start) {}

  Time now() const override { return now_; }

  void Set(Time time) { now_ = time; }

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

BodyWrenchSample Wrench(double force_x, uint64_t sequence) {
  BodyWrenchSample sample;
  sample.sequence = sequence;
  sample.frame_id = Id("body");
  sample.clock_domain = Id("monotonic");
  sample.validity_present = true;
  sample.validity_state = 1;
  sample.force_x_n = force_x;
  return sample;
}

VehicleLimitsSample ForceTorqueLimits(double force, double torque) {
  VehicleLimitsSample sample;
  sample.has_force_limits = true;
  sample.max_force_n = VehicleVector3{force, force, force};
  sample.has_torque_limits = true;
  sample.max_torque_n_m = VehicleVector3{torque, torque, torque};
  return sample;
}

void FillValidState(intrinsic_fbs::BodyState* state) {
  state->mutate_sequence(4);
  state->mutate_has_source_time(true);
  state->mutable_source_time()->mutate_seconds(1000);
  state->mutable_source_time()->mutate_nanos(0);
  state->mutate_has_receive_time(true);
  state->mutable_receive_time()->mutate_seconds(1000);
  state->mutable_receive_time()->mutate_nanos(0);
  ASSERT_TRUE(intrinsic_fbs::AssignFixedString64(state->mutable_clock_domain(),
                                                 "monotonic"));
  ASSERT_TRUE(intrinsic_fbs::AssignFixedString64(state->mutable_pose_frame_id(),
                                                 "world_enu"));
  state->mutate_validity_present(true);
  state->mutate_validity_state(1);
  state->mutate_has_pose(true);
  state->mutable_pose_world_from_body()->mutable_position().mutate_x(1.5);
  state->mutable_pose_world_from_body()->mutable_position().mutate_y(-2.0);
  state->mutable_pose_world_from_body()->mutable_position().mutate_z(0.25);
  state->mutable_pose_world_from_body()->mutable_orientation().mutate_x(0.0);
  state->mutable_pose_world_from_body()->mutable_orientation().mutate_y(0.0);
  state->mutable_pose_world_from_body()->mutable_orientation().mutate_z(0.0);
  state->mutable_pose_world_from_body()->mutable_orientation().mutate_w(1.0);
  state->mutate_navigation_mode(3);
  state->mutate_estimator_epoch(7);
}

class VehicleBodyFeaturesTest : public ::testing::Test {
 protected:
  void SetUp() override {
    clock_ = std::make_shared<ManualClock>(Time(Seconds(1000)));
    Clock::setClockImpl(clock_);
    auto shm = SharedMemoryManager::Create("vehicle_body_features_test",
                                           "vehicle_body");
    ASSERT_TRUE(shm.ok()) << shm.status();
    shm_ = std::move(*shm);
    hw_ = std::make_unique<HardwareInterfaceRegistry>(*shm_);
    ASSERT_TRUE(
        hw_->AdvertiseMutableInterface<intrinsic_fbs::BodyState>("body_state")
            .ok());
    ASSERT_TRUE(
        hw_->AdvertiseMutableInterface<intrinsic_fbs::BodyWrench>("body_wrench")
            .ok());
    auto state_editor =
        hw_->GetMutableInterfaceHandle<intrinsic_fbs::BodyState>("body_state");
    ASSERT_TRUE(state_editor.ok()) << state_editor.status();
    state_editor_ = std::move(*state_editor);
    auto wrench_editor =
        hw_->GetMutableInterfaceHandle<intrinsic_fbs::BodyWrench>(
            "body_wrench");
    ASSERT_TRUE(wrench_editor.ok()) << wrench_editor.status();
    wrench_editor_ = std::move(*wrench_editor);
    FillValidState(**state_editor_);
    state_editor_->UpdatedAt(Clock::Now());

    auto state_reader =
        hw_->GetInterfaceHandle<intrinsic_fbs::BodyState>("body_state");
    ASSERT_TRUE(state_reader.ok()) << state_reader.status();
    auto state = BodyStateFeature::Create(std::move(*state_reader));
    ASSERT_TRUE(state.ok()) << state.status();
    state_.emplace(std::move(*state));

    auto limits = VehicleLimitsFeature::Create(ForceTorqueLimits(10.0, 5.0),
                                               ForceTorqueLimits(20.0, 8.0));
    ASSERT_TRUE(limits.ok()) << limits.status();
    limits_.emplace(std::move(*limits));

    auto wrench_writer =
        hw_->GetMutableInterfaceHandle<intrinsic_fbs::BodyWrench>(
            "body_wrench");
    ASSERT_TRUE(wrench_writer.ok()) << wrench_writer.status();
    VehicleFeatureCycleConfig config;
    config.max_command_age = Milliseconds(50);
    config.max_state_age = Milliseconds(50);
    auto wrench = BodyWrenchFeature::Create(std::move(*wrench_writer), &*state_,
                                            &*limits_, config);
    ASSERT_TRUE(wrench.ok()) << wrench.status();
    wrench_.emplace(std::move(*wrench));
  }

  void TearDown() override { Clock::setClockImpl(nullptr); }

  RealtimePartInterface::ReadStatusParameters ReadParams(
      SafetyStatus safety = {}) {
    access_.emplace(properties_in_, properties_out_);
    return RealtimePartInterface::ReadStatusParameters{*access_, safety};
  }

  RealtimePartInterface::ApplyCommandParameters ApplyParams(
      SafetyStatus safety = {}) {
    access_.emplace(properties_in_, properties_out_);
    return RealtimePartInterface::ApplyCommandParameters{*access_, safety};
  }

  void Prepare(SafetyStatus safety = {}) {
    const auto params = ReadParams(safety);
    ASSERT_THAT(state_->ReadStatus(params), RealtimeIsOk());
    ASSERT_THAT(wrench_->ReadStatus(params), RealtimeIsOk());
  }

  std::shared_ptr<ManualClock> clock_;
  std::unique_ptr<SharedMemoryManager> shm_;
  std::unique_ptr<HardwareInterfaceRegistry> hw_;
  std::optional<MutableHardwareInterfaceHandle<intrinsic_fbs::BodyState>>
      state_editor_;
  std::optional<MutableHardwareInterfaceHandle<intrinsic_fbs::BodyWrench>>
      wrench_editor_;
  absl::FixedArray<PartPropertyValue> properties_in_{0};
  absl::FixedArray<PartPropertyValue> properties_out_{0};
  std::optional<RealtimePartPropertyAccess> access_;
  std::optional<BodyStateFeature> state_;
  std::optional<VehicleLimitsFeature> limits_;
  std::optional<BodyWrenchFeature> wrench_;
};

TEST_F(VehicleBodyFeaturesTest, SuccessLatchesStateAndAppliesWrench) {
  Prepare();
  const BodyStateSample& state = state_->LatchedBodyState();
  EXPECT_TRUE(state.latched);
  EXPECT_EQ(state.sequence, 4);
  EXPECT_EQ(state.estimator_epoch, 7);
  EXPECT_DOUBLE_EQ(state.position_m.x, 1.5);
  EXPECT_DOUBLE_EQ(state.position_m.y, -2.0);
  EXPECT_EQ(limits_->GetApplicationLimits().max_force_n.x, 10.0);
  EXPECT_EQ(limits_->GetSystemLimits().max_force_n.x, 20.0);
  EXPECT_FALSE(wrench_->PreviousBodyWrench().applied);

  ASSERT_THAT(wrench_->SetBodyWrench(Wrench(2.5, 1)), RealtimeIsOk());
  ASSERT_THAT(wrench_->ApplyCommand(ApplyParams()), RealtimeIsOk());

  const BodyWrenchSample& previous = wrench_->PreviousBodyWrench();
  EXPECT_TRUE(previous.applied);
  EXPECT_DOUBLE_EQ(previous.force_x_n, 2.5);
  EXPECT_EQ(previous.sequence, 1);
  EXPECT_DOUBLE_EQ((*wrench_editor_)->force_x_n(), 2.5);
  EXPECT_EQ(intrinsic_fbs::ViewFixedString64(*(*wrench_editor_)->frame_id()),
            "body");
}

TEST_F(VehicleBodyFeaturesTest, RejectsNonFiniteAndOutOfLimitCommands) {
  Prepare();
  BodyWrenchSample non_finite = Wrench(1.0, 1);
  non_finite.force_y_n = std::numeric_limits<double>::quiet_NaN();
  const RealtimeStatus nan = wrench_->SetBodyWrench(non_finite);
  EXPECT_EQ(nan.code(), absl::StatusCode::kInvalidArgument);
  EXPECT_EQ(nan.message(), "wrench is non-finite");
  EXPECT_DOUBLE_EQ((*wrench_editor_)->force_y_n(), 0.0);

  BodyWrenchSample infinite = Wrench(1.0, 1);
  infinite.torque_z_n_m = std::numeric_limits<double>::infinity();
  EXPECT_EQ(wrench_->SetBodyWrench(infinite).code(),
            absl::StatusCode::kInvalidArgument);

  const RealtimeStatus application = wrench_->SetBodyWrench(Wrench(11.0, 1));
  EXPECT_EQ(application.code(), absl::StatusCode::kOutOfRange);
  EXPECT_EQ(application.message(), "wrench exceeds application limits");

  const RealtimeStatus system = wrench_->SetBodyWrench(Wrench(21.0, 1));
  EXPECT_EQ(system.code(), absl::StatusCode::kFailedPrecondition);
  EXPECT_EQ(system.message(), "wrench exceeds system limits");
  EXPECT_DOUBLE_EQ((*wrench_editor_)->force_x_n(), 0.0);
}

TEST_F(VehicleBodyFeaturesTest, AbsentForceLimitsDoNotBoundTheCommand) {
  wrench_.reset();
  auto open = VehicleLimitsFeature::Create(VehicleLimitsSample{},
                                           VehicleLimitsSample{});
  ASSERT_TRUE(open.ok()) << open.status();
  limits_.emplace(std::move(*open));
  auto writer =
      hw_->GetMutableInterfaceHandle<intrinsic_fbs::BodyWrench>("body_wrench");
  ASSERT_TRUE(writer.ok()) << writer.status();
  VehicleFeatureCycleConfig config;
  config.max_command_age = Milliseconds(50);
  config.max_state_age = Milliseconds(50);
  auto wrench = BodyWrenchFeature::Create(std::move(*writer), &*state_,
                                          &*limits_, config);
  ASSERT_TRUE(wrench.ok()) << wrench.status();
  wrench_.emplace(std::move(*wrench));

  Prepare();
  ASSERT_THAT(wrench_->SetBodyWrench(Wrench(1000.0, 1)), RealtimeIsOk());
  ASSERT_THAT(wrench_->ApplyCommand(ApplyParams()), RealtimeIsOk());
  EXPECT_DOUBLE_EQ((*wrench_editor_)->force_x_n(), 1000.0);
}

TEST_F(VehicleBodyFeaturesTest, RejectsStaleCommandAndRequiresReset) {
  Prepare();
  ASSERT_THAT(wrench_->SetBodyWrench(Wrench(1.0, 1)), RealtimeIsOk());
  clock_->Advance(Milliseconds(51));
  const RealtimeStatus applied = wrench_->ApplyCommand(ApplyParams());
  EXPECT_EQ(applied.code(), absl::StatusCode::kDeadlineExceeded);
  EXPECT_EQ(applied.message(), "command watchdog requires reset");
  EXPECT_DOUBLE_EQ((*wrench_editor_)->force_x_n(), 0.0);
  EXPECT_EQ(intrinsic_fbs::ViewFixedString64(*(*wrench_editor_)->frame_id()),
            "body");

  clock_->Set(Time(Seconds(1000)));
  state_editor_->UpdatedAt(Clock::Now());
  Prepare();
  EXPECT_EQ(wrench_->SetBodyWrench(Wrench(1.0, 1)).code(),
            absl::StatusCode::kDeadlineExceeded);

  ASSERT_THAT(wrench_->Reset(), RealtimeIsOk());
  Prepare();
  ASSERT_THAT(wrench_->SetBodyWrench(Wrench(1.5, 1)), RealtimeIsOk());
  ASSERT_THAT(wrench_->ApplyCommand(ApplyParams()), RealtimeIsOk());
  EXPECT_DOUBLE_EQ((*wrench_editor_)->force_x_n(), 1.5);
}

TEST_F(VehicleBodyFeaturesTest, MonotonicStampAgeRejectsAndUtcDoesNot) {
  Prepare();
  BodyWrenchSample stale = Wrench(1.0, 1);
  stale.has_source_time = true;
  stale.source_time_seconds = 0;
  stale.has_receive_time = true;
  stale.receive_time_seconds = 10;
  const RealtimeStatus rejected = wrench_->SetBodyWrench(stale);
  EXPECT_EQ(rejected.code(), absl::StatusCode::kDeadlineExceeded);
  EXPECT_EQ(rejected.message(), "body wrench is stale");

  BodyWrenchSample wall = stale;
  wall.clock_domain = Id("utc");
  ASSERT_THAT(wrench_->SetBodyWrench(wall), RealtimeIsOk());
  ASSERT_THAT(wrench_->ApplyCommand(ApplyParams()), RealtimeIsOk());
  EXPECT_DOUBLE_EQ((*wrench_editor_)->force_x_n(), 1.0);
}

TEST_F(VehicleBodyFeaturesTest, StaleStateNeutralizesWithoutLatchingWatchdog) {
  clock_->Advance(Seconds(1));
  Prepare();
  const RealtimeStatus rejected = wrench_->SetBodyWrench(Wrench(3.0, 1));
  EXPECT_EQ(rejected.code(), absl::StatusCode::kDeadlineExceeded);
  EXPECT_EQ(rejected.message(), "body state is stale");
  const RealtimeStatus applied = wrench_->ApplyCommand(ApplyParams());
  EXPECT_EQ(applied.code(), absl::StatusCode::kDeadlineExceeded);
  EXPECT_DOUBLE_EQ((*wrench_editor_)->force_x_n(), 0.0);

  clock_->Set(Time(Seconds(1000)));
  state_editor_->UpdatedAt(Clock::Now());
  Prepare();
  ASSERT_THAT(wrench_->SetBodyWrench(Wrench(3.0, 1)), RealtimeIsOk());
  ASSERT_THAT(wrench_->ApplyCommand(ApplyParams()), RealtimeIsOk());
  EXPECT_DOUBLE_EQ((*wrench_editor_)->force_x_n(), 3.0);
}

TEST_F(VehicleBodyFeaturesTest, InvalidStateAndSafetyFaultsPropagate) {
  Prepare();
  ASSERT_THAT(wrench_->SetBodyWrench(Wrench(2.0, 1)), RealtimeIsOk());
  ASSERT_THAT(wrench_->ApplyCommand(ApplyParams()), RealtimeIsOk());

  (*state_editor_)->mutate_validity_state(2);
  state_editor_->UpdatedAt(Clock::Now());
  Prepare();
  const RealtimeStatus invalid = wrench_->SetBodyWrench(Wrench(4.0, 2));
  EXPECT_EQ(invalid.code(), absl::StatusCode::kFailedPrecondition);
  EXPECT_EQ(invalid.message(), "body state is invalid");
  const RealtimeStatus applied = wrench_->ApplyCommand(ApplyParams());
  EXPECT_EQ(applied.message(), "body state is invalid");
  EXPECT_DOUBLE_EQ((*wrench_editor_)->force_x_n(), 0.0);
  EXPECT_EQ(intrinsic_fbs::ViewFixedString64(*(*wrench_editor_)->frame_id()),
            "body");

  (*state_editor_)->mutate_validity_state(1);
  state_editor_->UpdatedAt(Clock::Now());
  SafetyStatus emergency;
  emergency.estop_button_status = intrinsic_fbs::ButtonStatus::ENGAGED;
  emergency.requested_behavior = intrinsic_fbs::RequestedBehavior::SAFE_STOP_0;
  (*state_editor_)->mutate_validity_state(2);
  Prepare(emergency);
  const RealtimeStatus safety = wrench_->ApplyCommand(ApplyParams(emergency));
  EXPECT_EQ(safety.code(), absl::StatusCode::kFailedPrecondition);
  EXPECT_EQ(safety.message(), "safety emergency");
  EXPECT_DOUBLE_EQ((*wrench_editor_)->force_x_n(), 0.0);
}

TEST_F(VehicleBodyFeaturesTest, NonFiniteStateIsAFault) {
  (*state_editor_)
      ->mutable_pose_world_from_body()
      ->mutable_position()
      .mutate_z(std::numeric_limits<double>::quiet_NaN());
  state_editor_->UpdatedAt(Clock::Now());
  Prepare();
  const RealtimeStatus status = wrench_->SetBodyWrench(Wrench(1.0, 1));
  EXPECT_EQ(status.code(), absl::StatusCode::kFailedPrecondition);
  EXPECT_EQ(status.message(), "body state is non-finite");
  EXPECT_EQ(wrench_->ApplyCommand(ApplyParams()).message(),
            "body state is non-finite");
}

TEST_F(VehicleBodyFeaturesTest, PrepareAndApplyDoNotAllocate) {
  access_.emplace(properties_in_, properties_out_);
  const RealtimePartInterface::ReadStatusParameters read{*access_,
                                                         SafetyStatus{}};
  const RealtimePartInterface::ApplyCommandParameters apply{*access_,
                                                            SafetyStatus{}};
  const BodyWrenchSample command = Wrench(1.25, 1);
  IF_INTRINSIC_MALLOC_TEST_INIT_COUNTER();
  INTRINSIC_RT_EXPECT_OK(state_->ReadStatus(read));
  INTRINSIC_RT_EXPECT_OK(wrench_->ReadStatus(read));
  INTRINSIC_RT_EXPECT_OK(wrench_->SetBodyWrench(command));
  INTRINSIC_RT_EXPECT_OK(wrench_->ApplyCommand(apply));
  IF_INTRINSIC_MALLOC_TEST_EXPECT_NO_ALLOCATIONS();
  EXPECT_DOUBLE_EQ(state_->LatchedBodyState().position_m.x, 1.5);
  EXPECT_DOUBLE_EQ((*wrench_editor_)->force_x_n(), 1.25);
}

TEST_F(VehicleBodyFeaturesTest, RegistryIsOptInUntilTheFeaturesAreAdded) {
  FeatureInterfaceRegistry empty;
  EXPECT_EQ(empty.GetInterface<BodyState>(), nullptr);
  EXPECT_EQ(empty.GetInterface<BodyWrenchCommand>(), nullptr);
  EXPECT_EQ(empty.GetInterface<VehicleLimitsInterface>(), nullptr);
  EXPECT_FALSE(empty.SupportedFeatureInterfaceTypes().contains(
      intrinsic_proto::icon::v1::FEATURE_INTERFACE_BODY_STATE));

  ASSERT_THAT(empty.RegisterInterface<BodyState>(&*state_), RealtimeIsOk());
  ASSERT_THAT(empty.RegisterInterface<BodyWrenchCommand>(&*wrench_),
              RealtimeIsOk());
  ASSERT_THAT(empty.RegisterInterface<VehicleLimitsInterface>(&*limits_),
              RealtimeIsOk());
  EXPECT_EQ(empty.GetInterface<BodyState>(), &*state_);
  EXPECT_EQ(empty.GetInterface<BodyWrenchCommand>(), &*wrench_);
  EXPECT_EQ(empty.GetInterface<VehicleLimitsInterface>(), &*limits_);
  const auto supported = empty.SupportedFeatureInterfaceTypes();
  EXPECT_TRUE(supported.contains(
      intrinsic_proto::icon::v1::FEATURE_INTERFACE_BODY_STATE));
  EXPECT_TRUE(supported.contains(
      intrinsic_proto::icon::v1::FEATURE_INTERFACE_BODY_WRENCH));
  EXPECT_TRUE(supported.contains(
      intrinsic_proto::icon::v1::FEATURE_INTERFACE_VEHICLE_LIMITS));
}

TEST_F(VehicleBodyFeaturesTest, MissingCapabilityAndBadLimitsAreRejected) {
  auto writer = hw_->GetMutableInterfaceHandle<intrinsic_fbs::BodyWrench>(
      "body_wrench_missing");
  ASSERT_FALSE(writer.ok());
  auto real_writer =
      hw_->GetMutableInterfaceHandle<intrinsic_fbs::BodyWrench>("body_wrench");
  ASSERT_TRUE(real_writer.ok()) << real_writer.status();
  VehicleFeatureCycleConfig config;
  auto missing_state = BodyWrenchFeature::Create(std::move(*real_writer),
                                                 nullptr, &*limits_, config);
  EXPECT_EQ(missing_state.status().code(),
            absl::StatusCode::kFailedPrecondition);

  auto again =
      hw_->GetMutableInterfaceHandle<intrinsic_fbs::BodyWrench>("body_wrench");
  ASSERT_TRUE(again.ok()) << again.status();
  auto missing_limits =
      BodyWrenchFeature::Create(std::move(*again), &*state_, nullptr, config);
  EXPECT_EQ(missing_limits.status().code(),
            absl::StatusCode::kFailedPrecondition);

  auto wider = VehicleLimitsFeature::Create(ForceTorqueLimits(30.0, 1.0),
                                            ForceTorqueLimits(20.0, 8.0));
  EXPECT_EQ(wider.status().code(), absl::StatusCode::kFailedPrecondition);
}

}  // namespace
}  // namespace intrinsic::icon
