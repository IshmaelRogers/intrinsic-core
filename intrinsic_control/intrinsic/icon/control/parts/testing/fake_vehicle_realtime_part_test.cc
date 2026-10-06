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

#include "intrinsic/icon/control/parts/testing/fake_vehicle_realtime_part.h"

#include <cstdint>
#include <memory>
#include <string_view>

#include "absl/status/status.h"
#include "gtest/gtest.h"
#include "intrinsic/icon/control/parts/feature_interfaces.h"
#include "intrinsic/icon/control/realtime_operational_status.h"
#include "intrinsic/icon/hal/interfaces/vehicle_hal.fbs.h"
#include "intrinsic/icon/proto/v1/types.pb.h"
#include "intrinsic/icon/utils/clock.h"
#include "intrinsic/icon/utils/realtime_status.h"
#include "intrinsic/icon/utils/realtime_status_matchers.h"

namespace intrinsic::icon {
namespace {

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

void ExpectIdEq(const FixedId64& actual, const FixedId64& expected) {
  EXPECT_EQ(actual.length, expected.length);
  EXPECT_EQ(std::string_view(actual.data, actual.length),
            std::string_view(expected.data, expected.length));
}

void ExpectVecEq(const VehicleVector3& actual, const VehicleVector3& expected) {
  EXPECT_DOUBLE_EQ(actual.x, expected.x);
  EXPECT_DOUBLE_EQ(actual.y, expected.y);
  EXPECT_DOUBLE_EQ(actual.z, expected.z);
}

void ExpectBodyStateEq(const BodyStateSample& actual,
                       const BodyStateSample& expected) {
  EXPECT_EQ(actual.latched, expected.latched);
  EXPECT_EQ(actual.latched_monotonic_ns, expected.latched_monotonic_ns);
  EXPECT_EQ(actual.sequence, expected.sequence);
  EXPECT_EQ(actual.has_source_time, expected.has_source_time);
  EXPECT_EQ(actual.source_time_seconds, expected.source_time_seconds);
  EXPECT_EQ(actual.source_time_nanos, expected.source_time_nanos);
  EXPECT_EQ(actual.has_receive_time, expected.has_receive_time);
  EXPECT_EQ(actual.receive_time_seconds, expected.receive_time_seconds);
  EXPECT_EQ(actual.receive_time_nanos, expected.receive_time_nanos);
  ExpectIdEq(actual.source_id, expected.source_id);
  ExpectIdEq(actual.pose_frame_id, expected.pose_frame_id);
  ExpectIdEq(actual.clock_domain, expected.clock_domain);
  EXPECT_EQ(actual.validity_present, expected.validity_present);
  EXPECT_EQ(actual.validity_state, expected.validity_state);
  EXPECT_EQ(actual.has_pose, expected.has_pose);
  ExpectVecEq(actual.position_m, expected.position_m);
  EXPECT_DOUBLE_EQ(actual.orientation_x, expected.orientation_x);
  EXPECT_DOUBLE_EQ(actual.orientation_y, expected.orientation_y);
  EXPECT_DOUBLE_EQ(actual.orientation_z, expected.orientation_z);
  EXPECT_DOUBLE_EQ(actual.orientation_w, expected.orientation_w);
  EXPECT_EQ(actual.has_body_twist, expected.has_body_twist);
  EXPECT_DOUBLE_EQ(actual.linear_x_m_s, expected.linear_x_m_s);
  EXPECT_DOUBLE_EQ(actual.linear_y_m_s, expected.linear_y_m_s);
  EXPECT_DOUBLE_EQ(actual.linear_z_m_s, expected.linear_z_m_s);
  EXPECT_DOUBLE_EQ(actual.angular_x_rad_s, expected.angular_x_rad_s);
  EXPECT_DOUBLE_EQ(actual.angular_y_rad_s, expected.angular_y_rad_s);
  EXPECT_DOUBLE_EQ(actual.angular_z_rad_s, expected.angular_z_rad_s);
  EXPECT_EQ(actual.has_body_acceleration, expected.has_body_acceleration);
  EXPECT_DOUBLE_EQ(actual.linear_x_m_s2, expected.linear_x_m_s2);
  EXPECT_DOUBLE_EQ(actual.linear_y_m_s2, expected.linear_y_m_s2);
  EXPECT_DOUBLE_EQ(actual.linear_z_m_s2, expected.linear_z_m_s2);
  EXPECT_DOUBLE_EQ(actual.angular_x_rad_s2, expected.angular_x_rad_s2);
  EXPECT_DOUBLE_EQ(actual.angular_y_rad_s2, expected.angular_y_rad_s2);
  EXPECT_DOUBLE_EQ(actual.angular_z_rad_s2, expected.angular_z_rad_s2);
  EXPECT_EQ(actual.has_pose_covariance, expected.has_pose_covariance);
  EXPECT_EQ(actual.has_twist_covariance, expected.has_twist_covariance);
  for (int i = 0; i < 36; ++i) {
    EXPECT_DOUBLE_EQ(actual.pose_covariance[i], expected.pose_covariance[i])
        << i;
    EXPECT_DOUBLE_EQ(actual.twist_covariance[i], expected.twist_covariance[i])
        << i;
  }
  EXPECT_EQ(actual.navigation_mode, expected.navigation_mode);
  EXPECT_EQ(actual.estimator_epoch, expected.estimator_epoch);
}

void ExpectWrenchEq(const BodyWrenchSample& actual,
                    const BodyWrenchSample& expected) {
  EXPECT_EQ(actual.applied, expected.applied);
  EXPECT_EQ(actual.sequence, expected.sequence);
  EXPECT_EQ(actual.has_source_time, expected.has_source_time);
  EXPECT_EQ(actual.source_time_seconds, expected.source_time_seconds);
  EXPECT_EQ(actual.source_time_nanos, expected.source_time_nanos);
  EXPECT_EQ(actual.has_receive_time, expected.has_receive_time);
  EXPECT_EQ(actual.receive_time_seconds, expected.receive_time_seconds);
  EXPECT_EQ(actual.receive_time_nanos, expected.receive_time_nanos);
  ExpectIdEq(actual.source_id, expected.source_id);
  ExpectIdEq(actual.frame_id, expected.frame_id);
  ExpectIdEq(actual.clock_domain, expected.clock_domain);
  EXPECT_EQ(actual.validity_present, expected.validity_present);
  EXPECT_EQ(actual.validity_state, expected.validity_state);
  EXPECT_DOUBLE_EQ(actual.force_x_n, expected.force_x_n);
  EXPECT_DOUBLE_EQ(actual.force_y_n, expected.force_y_n);
  EXPECT_DOUBLE_EQ(actual.force_z_n, expected.force_z_n);
  EXPECT_DOUBLE_EQ(actual.torque_x_n_m, expected.torque_x_n_m);
  EXPECT_DOUBLE_EQ(actual.torque_y_n_m, expected.torque_y_n_m);
  EXPECT_DOUBLE_EQ(actual.torque_z_n_m, expected.torque_z_n_m);
}

void ExpectLimitsEq(const VehicleLimitsSample& actual,
                    const VehicleLimitsSample& expected) {
  EXPECT_EQ(actual.has_translational_position_limits,
            expected.has_translational_position_limits);
  ExpectVecEq(actual.min_translational_position_m,
              expected.min_translational_position_m);
  ExpectVecEq(actual.max_translational_position_m,
              expected.max_translational_position_m);
  EXPECT_EQ(actual.has_translational_velocity_limits,
            expected.has_translational_velocity_limits);
  ExpectVecEq(actual.min_translational_velocity_m_s,
              expected.min_translational_velocity_m_s);
  ExpectVecEq(actual.max_translational_velocity_m_s,
              expected.max_translational_velocity_m_s);
  EXPECT_EQ(actual.has_translational_acceleration_limits,
            expected.has_translational_acceleration_limits);
  ExpectVecEq(actual.min_translational_acceleration_m_s2,
              expected.min_translational_acceleration_m_s2);
  ExpectVecEq(actual.max_translational_acceleration_m_s2,
              expected.max_translational_acceleration_m_s2);
  EXPECT_EQ(actual.has_rotational_velocity_limit,
            expected.has_rotational_velocity_limit);
  EXPECT_DOUBLE_EQ(actual.max_rotational_velocity_rad_s,
                   expected.max_rotational_velocity_rad_s);
  EXPECT_EQ(actual.has_rotational_acceleration_limit,
            expected.has_rotational_acceleration_limit);
  EXPECT_DOUBLE_EQ(actual.max_rotational_acceleration_rad_s2,
                   expected.max_rotational_acceleration_rad_s2);
  EXPECT_EQ(actual.has_force_limits, expected.has_force_limits);
  ExpectVecEq(actual.max_force_n, expected.max_force_n);
  EXPECT_EQ(actual.has_torque_limits, expected.has_torque_limits);
  ExpectVecEq(actual.max_torque_n_m, expected.max_torque_n_m);
}

VehicleLimitsSample ForceTorqueLimits(double force, double torque) {
  VehicleLimitsSample sample;
  sample.has_force_limits = true;
  sample.max_force_n = VehicleVector3{force, force, force};
  sample.has_torque_limits = true;
  sample.max_torque_n_m = VehicleVector3{torque, torque, torque};
  return sample;
}

BodyStateSample ValidState(int64_t monotonic_ns, uint64_t sequence, double x) {
  BodyStateSample sample;
  sample.latched = true;
  sample.latched_monotonic_ns = monotonic_ns;
  sample.sequence = sequence;
  sample.has_source_time = true;
  sample.source_time_seconds = 1000;
  sample.source_time_nanos = 0;
  sample.has_receive_time = true;
  sample.receive_time_seconds = 1000;
  sample.receive_time_nanos = 0;
  sample.source_id = Id("estimator");
  sample.pose_frame_id = Id("world_enu");
  sample.clock_domain = Id("monotonic");
  sample.validity_present = true;
  sample.validity_state = 1;
  sample.has_pose = true;
  sample.position_m = VehicleVector3{x, -2.0, 0.25};
  sample.orientation_w = 1.0;
  sample.has_body_twist = true;
  sample.linear_x_m_s = 0.1;
  sample.navigation_mode = 3;
  sample.estimator_epoch = sequence;
  sample.has_pose_covariance = true;
  sample.pose_covariance[0] = 0.01;
  sample.pose_covariance[7] = 0.01;
  sample.pose_covariance[14] = 0.01;
  sample.pose_covariance[21] = 0.01;
  sample.pose_covariance[28] = 0.01;
  sample.pose_covariance[35] = 0.01;
  return sample;
}

BodyWrenchSample Wrench(double force_x, uint64_t sequence) {
  BodyWrenchSample sample;
  sample.sequence = sequence;
  sample.frame_id = Id("body");
  sample.clock_domain = Id("monotonic");
  sample.validity_present = true;
  sample.validity_state = 1;
  sample.force_x_n = force_x;
  sample.force_y_n = 0.25;
  sample.torque_z_n_m = -0.1;
  return sample;
}

// HAL image of the #19 neutral body wrench. `applied` is not a field of
// intrinsic_fbs::BodyWrench, so the recorded sample leaves it false.
BodyWrenchSample NeutralWrench() {
  BodyWrenchSample sample;
  sample.frame_id = Id("body");
  sample.clock_domain = Id("monotonic");
  sample.validity_present = true;
  sample.validity_state = 1;
  return sample;
}

class FakeVehicleRealtimePartTest : public ::testing::Test {
 protected:
  void SetUp() override {
    clock_ = std::make_shared<ManualClock>(Time(Seconds(1000)));
    Clock::setClockImpl(clock_);
    auto part = FakeVehicleRealtimePart::Create(ForceTorqueLimits(10.0, 5.0),
                                                ForceTorqueLimits(20.0, 8.0));
    ASSERT_TRUE(part.ok()) << part.status();
    part_ = std::move(*part);
  }

  void TearDown() override { Clock::setClockImpl(nullptr); }

  int64_t NowNs() const { return Clock::ToNSec(Clock::Now()); }

  BodyWrenchCommand* WrenchInterface() {
    return part_->GetFeatureInterfaces().GetInterface<BodyWrenchCommand>();
  }

  std::shared_ptr<ManualClock> clock_;
  std::unique_ptr<FakeVehicleRealtimePart> part_;
};

TEST_F(FakeVehicleRealtimePartTest, ReadStatusLatchesInjectedBodyState) {
  const BodyStateSample injected = ValidState(NowNs(), 4, 1.5);
  ASSERT_THAT(part_->SetBodyState(injected), RealtimeIsOk());
  ASSERT_THAT(part_->ReadStatusForTest(), RealtimeIsOk());

  const BodyState* state =
      part_->GetFeatureInterfaces().GetInterface<BodyState>();
  ASSERT_NE(state, nullptr);
  ExpectBodyStateEq(state->LatchedBodyState(), injected);
}

TEST_F(FakeVehicleRealtimePartTest, LimitsRoundTrip) {
  const VehicleLimitsInterface* limits =
      part_->GetFeatureInterfaces().GetInterface<VehicleLimitsInterface>();
  ASSERT_NE(limits, nullptr);
  ExpectLimitsEq(limits->GetApplicationLimits(), ForceTorqueLimits(10.0, 5.0));
  ExpectLimitsEq(limits->GetSystemLimits(), ForceTorqueLimits(20.0, 8.0));
  EXPECT_TRUE(part_->ApplicationLimitsHal().has_force_limits());
  EXPECT_DOUBLE_EQ(part_->ApplicationLimitsHal().max_force_n()->x(), 10.0);
  EXPECT_DOUBLE_EQ(part_->ApplicationLimitsHal().max_torque_n_m()->y(), 5.0);
  EXPECT_TRUE(part_->SystemLimitsHal().has_torque_limits());
  EXPECT_DOUBLE_EQ(part_->SystemLimitsHal().max_force_n()->z(), 20.0);
  EXPECT_DOUBLE_EQ(part_->SystemLimitsHal().max_torque_n_m()->z(), 8.0);
}

TEST_F(FakeVehicleRealtimePartTest, OperationalStatusInjectionIsUnchanged) {
  const auto initial = part_->GetOperationalStatus();
  ASSERT_THAT(initial, RealtimeIsOk());
  EXPECT_EQ((*initial).state, RealtimeOperationalState::kEnabled);
  EXPECT_TRUE((*initial).fault_reason.empty());

  RealtimeOperationalStatus injected;
  injected.state = RealtimeOperationalState::kFaultedConnected;
  injected.fault_reason =
      RealtimeOperationalStatus::FaultReasonString("thruster bus timeout");
  part_->SetOperationalStatus(injected);
  const auto got = part_->GetOperationalStatus();
  ASSERT_THAT(got, RealtimeIsOk());
  EXPECT_EQ((*got).state, injected.state);
  EXPECT_EQ((*got).fault_reason, injected.fault_reason);
}

TEST_F(FakeVehicleRealtimePartTest, HealthInjectionIsReadable) {
  const auto before = part_->LatchedActuatorHealth(0);
  ASSERT_THAT(before, RealtimeIsOk());
  EXPECT_EQ((*before).health, FakeThrusterHealth::kNominal);
  EXPECT_FALSE((*before).has_derate);

  const FakeThrusterHealth values[] = {
      FakeThrusterHealth::kNominal, FakeThrusterHealth::kDisabled,
      FakeThrusterHealth::kDerated, FakeThrusterHealth::kStuckOff,
      FakeThrusterHealth::kFailed,
  };
  for (int slot = 0; slot < 5; ++slot) {
    FakeThrusterActuatorHealth health;
    health.health = values[slot];
    if (values[slot] == FakeThrusterHealth::kDerated) {
      health.has_derate = true;
      health.derate = 0.4;
    }
    ASSERT_THAT(part_->SetActuatorHealth(slot, health), RealtimeIsOk());
    const auto staged = part_->ActuatorHealth(slot);
    ASSERT_THAT(staged, RealtimeIsOk());
    EXPECT_EQ((*staged).health, health.health);
    EXPECT_EQ((*staged).has_derate, health.has_derate);
    EXPECT_DOUBLE_EQ((*staged).derate, health.derate);
    const auto still_latched = part_->LatchedActuatorHealth(slot);
    ASSERT_THAT(still_latched, RealtimeIsOk());
    EXPECT_EQ((*still_latched).health, FakeThrusterHealth::kNominal);
  }

  ASSERT_THAT(part_->SetBodyState(ValidState(NowNs(), 1, 0.0)), RealtimeIsOk());
  ASSERT_THAT(part_->ReadStatusForTest(), RealtimeIsOk());
  const auto latched = part_->LatchedActuatorHealth(2);
  ASSERT_THAT(latched, RealtimeIsOk());
  EXPECT_EQ((*latched).health, FakeThrusterHealth::kDerated);
  EXPECT_TRUE((*latched).has_derate);
  EXPECT_DOUBLE_EQ((*latched).derate, 0.4);

  const RealtimeStatus negative = part_->SetActuatorHealth(-1, {});
  EXPECT_EQ(negative.code(), absl::StatusCode::kOutOfRange);
  EXPECT_EQ(negative.message(), "thruster slot is out of range");
  const RealtimeStatus past =
      part_->SetActuatorHealth(kFakeVehicleThrusterSlots, {});
  EXPECT_EQ(past.code(), absl::StatusCode::kOutOfRange);
  EXPECT_EQ(part_->ActuatorHealth(kFakeVehicleThrusterSlots).status().code(),
            absl::StatusCode::kOutOfRange);
  EXPECT_EQ(part_->LatchedActuatorHealth(-1).status().code(),
            absl::StatusCode::kOutOfRange);
  EXPECT_EQ((*part_->ActuatorHealth(0)).health, FakeThrusterHealth::kNominal);

  FakeThrusterActuatorHealth unrecognized;
  unrecognized.health = static_cast<FakeThrusterHealth>(99);
  const RealtimeStatus rejected = part_->SetActuatorHealth(0, unrecognized);
  EXPECT_EQ(rejected.code(), absl::StatusCode::kInvalidArgument);
  EXPECT_EQ(rejected.message(), "thruster health is unrecognized");
  EXPECT_EQ((*part_->ActuatorHealth(0)).health, FakeThrusterHealth::kNominal);

  const auto supported =
      part_->GetFeatureInterfaces().SupportedFeatureInterfaceTypes();
  EXPECT_EQ(supported.size(), 3);
  EXPECT_TRUE(supported.contains(
      intrinsic_proto::icon::v1::FEATURE_INTERFACE_BODY_STATE));
  EXPECT_TRUE(supported.contains(
      intrinsic_proto::icon::v1::FEATURE_INTERFACE_BODY_WRENCH));
  EXPECT_TRUE(supported.contains(
      intrinsic_proto::icon::v1::FEATURE_INTERFACE_VEHICLE_LIMITS));
}

TEST_F(FakeVehicleRealtimePartTest, ValidWrenchIsRecordedAfterApply) {
  ASSERT_THAT(part_->SetBodyState(ValidState(NowNs(), 4, 1.5)), RealtimeIsOk());
  ASSERT_THAT(part_->ReadStatusForTest(), RealtimeIsOk());
  BodyWrenchCommand* wrench = WrenchInterface();
  ASSERT_NE(wrench, nullptr);
  const BodyWrenchSample command = Wrench(2.5, 1);
  ASSERT_THAT(wrench->SetBodyWrench(command), RealtimeIsOk());
  ASSERT_THAT(part_->ApplyCommandForTest(), RealtimeIsOk());
  ExpectWrenchEq(part_->RecordedBodyWrench(), command);
}

TEST_F(FakeVehicleRealtimePartTest, StaleStateRecordsNeutralWrench) {
  ASSERT_THAT(part_->SetBodyState(ValidState(NowNs(), 4, 1.5)), RealtimeIsOk());
  clock_->Advance(Milliseconds(51));
  ASSERT_THAT(part_->ReadStatusForTest(), RealtimeIsOk());
  BodyWrenchCommand* wrench = WrenchInterface();
  ASSERT_NE(wrench, nullptr);
  const RealtimeStatus rejected = wrench->SetBodyWrench(Wrench(3.0, 1));
  EXPECT_EQ(rejected.code(), absl::StatusCode::kDeadlineExceeded);
  EXPECT_EQ(rejected.message(), "body state is stale");
  const RealtimeStatus applied = part_->ApplyCommandForTest();
  EXPECT_EQ(applied.code(), absl::StatusCode::kDeadlineExceeded);
  EXPECT_EQ(applied.message(), "body state is stale");
  ExpectWrenchEq(part_->RecordedBodyWrench(), NeutralWrench());
}

TEST_F(FakeVehicleRealtimePartTest,
       ReadStatusApplyCommandOrderingAcrossCycles) {
  FakeThrusterActuatorHealth derated;
  derated.health = FakeThrusterHealth::kDerated;
  derated.has_derate = true;
  derated.derate = 0.5;
  ASSERT_THAT(part_->SetActuatorHealth(1, derated), RealtimeIsOk());

  const BodyStateSample state1 = ValidState(NowNs(), 1, 1.0);
  ASSERT_THAT(part_->SetBodyState(state1), RealtimeIsOk());
  ASSERT_THAT(part_->ReadStatusForTest(), RealtimeIsOk());
  const BodyState* state =
      part_->GetFeatureInterfaces().GetInterface<BodyState>();
  ASSERT_NE(state, nullptr);
  ExpectBodyStateEq(state->LatchedBodyState(), state1);
  EXPECT_EQ((*part_->LatchedActuatorHealth(1)).health,
            FakeThrusterHealth::kDerated);

  BodyWrenchCommand* wrench = WrenchInterface();
  ASSERT_NE(wrench, nullptr);
  const BodyWrenchSample command1 = Wrench(1.0, 1);
  ASSERT_THAT(wrench->SetBodyWrench(command1), RealtimeIsOk());
  ASSERT_THAT(part_->ApplyCommandForTest(), RealtimeIsOk());
  ExpectWrenchEq(part_->RecordedBodyWrench(), command1);

  FakeThrusterActuatorHealth stuck;
  stuck.health = FakeThrusterHealth::kStuckOff;
  stuck.has_derate = true;
  stuck.derate = 0.0;
  ASSERT_THAT(part_->SetActuatorHealth(1, stuck), RealtimeIsOk());
  EXPECT_EQ((*part_->LatchedActuatorHealth(1)).health,
            FakeThrusterHealth::kDerated);

  const BodyStateSample state2 = ValidState(NowNs(), 2, 2.0);
  ASSERT_THAT(part_->SetBodyState(state2), RealtimeIsOk());
  ASSERT_THAT(part_->ReadStatusForTest(), RealtimeIsOk());
  ExpectBodyStateEq(state->LatchedBodyState(), state2);
  EXPECT_EQ((*part_->LatchedActuatorHealth(1)).health,
            FakeThrusterHealth::kStuckOff);

  const BodyWrenchSample command2 = Wrench(2.0, 2);
  ASSERT_THAT(wrench->SetBodyWrench(command2), RealtimeIsOk());
  ASSERT_THAT(part_->ApplyCommandForTest(), RealtimeIsOk());
  ExpectWrenchEq(part_->RecordedBodyWrench(), command2);

  const BodyWrenchSample dropped = Wrench(4.0, 3);
  ASSERT_THAT(wrench->SetBodyWrench(dropped), RealtimeIsOk());
  const BodyStateSample state3 = ValidState(NowNs(), 3, 3.0);
  ASSERT_THAT(part_->SetBodyState(state3), RealtimeIsOk());
  ASSERT_THAT(part_->ReadStatusForTest(), RealtimeIsOk());
  ASSERT_THAT(part_->ApplyCommandForTest(), RealtimeIsOk());
  ExpectBodyStateEq(state->LatchedBodyState(), state3);
  ExpectWrenchEq(part_->RecordedBodyWrench(), command2);
}

TEST(FakeVehicleRealtimePartCreateTest, RejectsApplicationLimitsOutsideSystem) {
  auto part = FakeVehicleRealtimePart::Create(ForceTorqueLimits(30.0, 1.0),
                                              ForceTorqueLimits(20.0, 8.0));
  EXPECT_FALSE(part.ok());
  EXPECT_EQ(part.status().code(), absl::StatusCode::kFailedPrecondition);
}

TEST(FakeVehicleRealtimePartCreateTest, CycleConfigIsPassedToTheFeature) {
  auto clock = std::make_shared<ManualClock>(Time(Seconds(1000)));
  Clock::setClockImpl(clock);
  VehicleFeatureCycleConfig config;
  config.max_state_age = Milliseconds(10);
  config.max_command_age = Milliseconds(50);
  auto part = FakeVehicleRealtimePart::Create(
      ForceTorqueLimits(10.0, 5.0), ForceTorqueLimits(20.0, 8.0), config);
  ASSERT_TRUE(part.ok()) << part.status();
  const int64_t stamped = Clock::ToNSec(Clock::Now());
  ASSERT_THAT((*part)->SetBodyState(ValidState(stamped, 4, 1.5)),
              RealtimeIsOk());
  clock->Advance(Milliseconds(11));
  ASSERT_THAT((*part)->ReadStatusForTest(), RealtimeIsOk());
  BodyWrenchCommand* wrench =
      (*part)->GetFeatureInterfaces().GetInterface<BodyWrenchCommand>();
  ASSERT_NE(wrench, nullptr);
  const RealtimeStatus rejected = wrench->SetBodyWrench(Wrench(1.0, 1));
  EXPECT_EQ(rejected.code(), absl::StatusCode::kDeadlineExceeded);
  EXPECT_EQ(rejected.message(), "body state is stale");
  Clock::setClockImpl(nullptr);
}

}  // namespace
}  // namespace intrinsic::icon
