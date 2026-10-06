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
#include <string_view>
#include <utility>

#include "absl/container/fixed_array.h"
#include "absl/memory/memory.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "intrinsic/icon/common/part_properties.h"
#include "intrinsic/icon/control/parts/realtime_part_property_access.h"
#include "intrinsic/icon/hal/hardware_interface_registry.h"
#include "intrinsic/icon/hal/interfaces/vehicle_hal.fbs.h"
#include "intrinsic/icon/hal/interfaces/vehicle_hal_utils.h"
#include "intrinsic/icon/hal/interfaces/vehicle_hardware_interfaces.h"
#include "intrinsic/icon/interprocess/shared_memory_manager/shared_memory_manager.h"
#include "intrinsic/icon/utils/clock.h"
#include "intrinsic/icon/utils/realtime_status.h"

namespace intrinsic::icon {
namespace {

constexpr int kMatrixN = 36;

RealtimeStatus AbsentField() {
  return FailedPreconditionError("vehicle hardware field is absent");
}

bool KnownHealth(FakeThrusterHealth health) {
  switch (health) {
    case FakeThrusterHealth::kNominal:
    case FakeThrusterHealth::kDisabled:
    case FakeThrusterHealth::kDerated:
    case FakeThrusterHealth::kStuckOff:
    case FakeThrusterHealth::kFailed:
      return true;
  }
  return false;
}

bool SlotInRange(int slot) {
  return slot >= 0 && slot < kFakeVehicleThrusterSlots;
}

RealtimeStatus CheckId(const FixedId64& id) {
  if (id.length > sizeof(id.data)) {
    return InvalidArgumentError("fixed id is longer than 64 bytes");
  }
  return OkStatus();
}

RealtimeStatus WriteId(intrinsic_fbs::FixedString64* out, const FixedId64& id) {
  if (const RealtimeStatus status = CheckId(id); !status.ok()) {
    return status;
  }
  if (out == nullptr) {
    return AbsentField();
  }
  if (!intrinsic_fbs::AssignFixedString64(
          out, std::string_view(id.data, id.length))) {
    return InvalidArgumentError("fixed id is longer than 64 bytes");
  }
  return OkStatus();
}

RealtimeStatus WriteVector(intrinsic_fbs::Vector3d* out,
                           const VehicleVector3& value) {
  if (out == nullptr) {
    return AbsentField();
  }
  out->mutate_x(value.x);
  out->mutate_y(value.y);
  out->mutate_z(value.z);
  return OkStatus();
}

RealtimeStatus WriteMatrix(intrinsic_fbs::Matrix6d* out, const double* values) {
  if (out == nullptr || out->mutable_values() == nullptr) {
    return AbsentField();
  }
  for (int i = 0; i < kMatrixN; ++i) {
    out->mutable_values()->Mutate(static_cast<uint16_t>(i), values[i]);
  }
  return OkStatus();
}

RealtimeStatus WriteVehicleLimits(intrinsic_fbs::VehicleLimits* out,
                                  const VehicleLimitsSample& sample) {
  if (out == nullptr) {
    return FailedPreconditionError(
        "vehicle limits hardware interface is empty");
  }
  out->mutate_has_translational_position_limits(
      sample.has_translational_position_limits);
  if (const RealtimeStatus status =
          WriteVector(out->mutable_min_translational_position_m(),
                      sample.min_translational_position_m);
      !status.ok()) {
    return status;
  }
  if (const RealtimeStatus status =
          WriteVector(out->mutable_max_translational_position_m(),
                      sample.max_translational_position_m);
      !status.ok()) {
    return status;
  }
  out->mutate_has_translational_velocity_limits(
      sample.has_translational_velocity_limits);
  if (const RealtimeStatus status =
          WriteVector(out->mutable_min_translational_velocity_m_s(),
                      sample.min_translational_velocity_m_s);
      !status.ok()) {
    return status;
  }
  if (const RealtimeStatus status =
          WriteVector(out->mutable_max_translational_velocity_m_s(),
                      sample.max_translational_velocity_m_s);
      !status.ok()) {
    return status;
  }
  out->mutate_has_translational_acceleration_limits(
      sample.has_translational_acceleration_limits);
  if (const RealtimeStatus status =
          WriteVector(out->mutable_min_translational_acceleration_m_s2(),
                      sample.min_translational_acceleration_m_s2);
      !status.ok()) {
    return status;
  }
  if (const RealtimeStatus status =
          WriteVector(out->mutable_max_translational_acceleration_m_s2(),
                      sample.max_translational_acceleration_m_s2);
      !status.ok()) {
    return status;
  }
  out->mutate_has_rotational_velocity_limit(
      sample.has_rotational_velocity_limit);
  out->mutate_max_rotational_velocity_rad_s(
      sample.max_rotational_velocity_rad_s);
  out->mutate_has_rotational_acceleration_limit(
      sample.has_rotational_acceleration_limit);
  out->mutate_max_rotational_acceleration_rad_s2(
      sample.max_rotational_acceleration_rad_s2);
  out->mutate_has_force_limits(sample.has_force_limits);
  if (const RealtimeStatus status =
          WriteVector(out->mutable_max_force_n(), sample.max_force_n);
      !status.ok()) {
    return status;
  }
  out->mutate_has_torque_limits(sample.has_torque_limits);
  return WriteVector(out->mutable_max_torque_n_m(), sample.max_torque_n_m);
}

RealtimeStatus WriteBodyState(
    MutableHardwareInterfaceHandle<intrinsic_fbs::BodyState>& editor,
    const BodyStateSample& sample) {
  intrinsic_fbs::BodyState* state = *editor;
  if (state == nullptr) {
    return FailedPreconditionError("body state hardware interface is empty");
  }
  state->mutate_sequence(sample.sequence);
  state->mutate_has_source_time(sample.has_source_time);
  if (state->mutable_source_time() == nullptr ||
      state->mutable_receive_time() == nullptr) {
    return AbsentField();
  }
  state->mutable_source_time()->mutate_seconds(sample.source_time_seconds);
  state->mutable_source_time()->mutate_nanos(sample.source_time_nanos);
  state->mutate_has_receive_time(sample.has_receive_time);
  state->mutable_receive_time()->mutate_seconds(sample.receive_time_seconds);
  state->mutable_receive_time()->mutate_nanos(sample.receive_time_nanos);
  if (const RealtimeStatus status =
          WriteId(state->mutable_source_id(), sample.source_id);
      !status.ok()) {
    return status;
  }
  if (const RealtimeStatus status =
          WriteId(state->mutable_pose_frame_id(), sample.pose_frame_id);
      !status.ok()) {
    return status;
  }
  if (const RealtimeStatus status =
          WriteId(state->mutable_clock_domain(), sample.clock_domain);
      !status.ok()) {
    return status;
  }
  state->mutate_validity_present(sample.validity_present);
  state->mutate_validity_state(sample.validity_state);
  state->mutate_has_pose(sample.has_pose);
  intrinsic_fbs::BodyPose* pose = state->mutable_pose_world_from_body();
  if (pose == nullptr) {
    return AbsentField();
  }
  pose->mutable_position().mutate_x(sample.position_m.x);
  pose->mutable_position().mutate_y(sample.position_m.y);
  pose->mutable_position().mutate_z(sample.position_m.z);
  pose->mutable_orientation().mutate_x(sample.orientation_x);
  pose->mutable_orientation().mutate_y(sample.orientation_y);
  pose->mutable_orientation().mutate_z(sample.orientation_z);
  pose->mutable_orientation().mutate_w(sample.orientation_w);
  state->mutate_has_body_twist(sample.has_body_twist);
  intrinsic_fbs::BodyTwistRt* twist = state->mutable_body_twist();
  if (twist == nullptr) {
    return AbsentField();
  }
  twist->mutate_linear_x_m_s(sample.linear_x_m_s);
  twist->mutate_linear_y_m_s(sample.linear_y_m_s);
  twist->mutate_linear_z_m_s(sample.linear_z_m_s);
  twist->mutate_angular_x_rad_s(sample.angular_x_rad_s);
  twist->mutate_angular_y_rad_s(sample.angular_y_rad_s);
  twist->mutate_angular_z_rad_s(sample.angular_z_rad_s);
  state->mutate_has_body_acceleration(sample.has_body_acceleration);
  intrinsic_fbs::BodyAccelerationRt* acceleration =
      state->mutable_body_acceleration();
  if (acceleration == nullptr) {
    return AbsentField();
  }
  acceleration->mutate_linear_x_m_s2(sample.linear_x_m_s2);
  acceleration->mutate_linear_y_m_s2(sample.linear_y_m_s2);
  acceleration->mutate_linear_z_m_s2(sample.linear_z_m_s2);
  acceleration->mutate_angular_x_rad_s2(sample.angular_x_rad_s2);
  acceleration->mutate_angular_y_rad_s2(sample.angular_y_rad_s2);
  acceleration->mutate_angular_z_rad_s2(sample.angular_z_rad_s2);
  state->mutate_has_pose_covariance(sample.has_pose_covariance);
  if (const RealtimeStatus status =
          WriteMatrix(state->mutable_pose_covariance(), sample.pose_covariance);
      !status.ok()) {
    return status;
  }
  state->mutate_has_twist_covariance(sample.has_twist_covariance);
  if (const RealtimeStatus status = WriteMatrix(
          state->mutable_twist_covariance(), sample.twist_covariance);
      !status.ok()) {
    return status;
  }
  state->mutate_navigation_mode(sample.navigation_mode);
  state->mutate_estimator_epoch(sample.estimator_epoch);
  return OkStatus();
}

void CopyId(FixedId64* out, const intrinsic_fbs::FixedString64* in) {
  *out = FixedId64{};
  if (in == nullptr) {
    return;
  }
  const std::string_view view = intrinsic_fbs::ViewFixedString64(*in);
  if (view.size() > sizeof(out->data)) {
    return;
  }
  out->length = static_cast<uint8_t>(view.size());
  for (size_t i = 0; i < view.size(); ++i) {
    out->data[i] = view[i];
  }
}

BodyWrenchSample CopyWrench(const intrinsic_fbs::BodyWrench& wrench) {
  BodyWrenchSample sample;
  sample.sequence = wrench.sequence();
  sample.has_source_time = wrench.has_source_time();
  if (wrench.source_time() != nullptr) {
    sample.source_time_seconds = wrench.source_time()->seconds();
    sample.source_time_nanos = wrench.source_time()->nanos();
  }
  sample.has_receive_time = wrench.has_receive_time();
  if (wrench.receive_time() != nullptr) {
    sample.receive_time_seconds = wrench.receive_time()->seconds();
    sample.receive_time_nanos = wrench.receive_time()->nanos();
  }
  CopyId(&sample.source_id, wrench.source_id());
  CopyId(&sample.frame_id, wrench.frame_id());
  CopyId(&sample.clock_domain, wrench.clock_domain());
  sample.validity_present = wrench.validity_present();
  sample.validity_state = wrench.validity_state();
  sample.force_x_n = wrench.force_x_n();
  sample.force_y_n = wrench.force_y_n();
  sample.force_z_n = wrench.force_z_n();
  sample.torque_x_n_m = wrench.torque_x_n_m();
  sample.torque_y_n_m = wrench.torque_y_n_m();
  sample.torque_z_n_m = wrench.torque_z_n_m();
  return sample;
}

}  // namespace

absl::StatusOr<std::unique_ptr<FakeVehicleRealtimePart>>
FakeVehicleRealtimePart::Create(VehicleLimitsSample application_limits,
                                VehicleLimitsSample system_limits,
                                VehicleFeatureCycleConfig cycle_config) {
  auto part = absl::WrapUnique(new FakeVehicleRealtimePart());
  if (const absl::Status status =
          part->Init(application_limits, system_limits, cycle_config);
      !status.ok()) {
    return status;
  }
  return part;
}

FakeVehicleRealtimePart::~FakeVehicleRealtimePart() = default;

absl::Status FakeVehicleRealtimePart::Init(
    VehicleLimitsSample application_limits, VehicleLimitsSample system_limits,
    VehicleFeatureCycleConfig cycle_config) {
  auto shm =
      SharedMemoryManager::Create("fake_vehicle_realtime_part", "fake_vehicle");
  if (!shm.ok()) {
    return shm.status();
  }
  shm_ = std::move(*shm);
  hw_ = std::make_unique<HardwareInterfaceRegistry>(*shm_);

  auto body_state =
      hw_->AdvertiseMutableInterface<intrinsic_fbs::BodyState>("body_state");
  if (!body_state.ok()) {
    return body_state.status();
  }
  body_state_editor_ = std::move(*body_state);
  auto state_reader =
      hw_->GetInterfaceHandle<intrinsic_fbs::BodyState>("body_state");
  if (!state_reader.ok()) {
    return state_reader.status();
  }

  auto wrench_writer =
      hw_->AdvertiseMutableInterface<intrinsic_fbs::BodyWrench>("body_wrench");
  if (!wrench_writer.ok()) {
    return wrench_writer.status();
  }
  auto wrench_reader =
      hw_->GetInterfaceHandle<intrinsic_fbs::BodyWrench>("body_wrench");
  if (!wrench_reader.ok()) {
    return wrench_reader.status();
  }
  body_wrench_reader_ = std::move(*wrench_reader);

  auto application =
      hw_->AdvertiseMutableInterface<intrinsic_fbs::VehicleLimits>(
          "application_limits");
  if (!application.ok()) {
    return application.status();
  }
  application_limits_hal_ = std::move(*application);
  auto system = hw_->AdvertiseMutableInterface<intrinsic_fbs::VehicleLimits>(
      "system_limits");
  if (!system.ok()) {
    return system.status();
  }
  system_limits_hal_ = std::move(*system);
  if (const RealtimeStatus status =
          WriteVehicleLimits(*application_limits_hal_, application_limits);
      !status.ok()) {
    return status;
  }
  if (const RealtimeStatus status =
          WriteVehicleLimits(*system_limits_hal_, system_limits);
      !status.ok()) {
    return status;
  }
  application_limits_hal_.UpdatedAt(Clock::Now());
  system_limits_hal_.UpdatedAt(Clock::Now());

  auto limits = VehicleLimitsFeature::Create(application_limits, system_limits);
  if (!limits.ok()) {
    return limits.status();
  }
  limits_.emplace(std::move(*limits));
  auto state = BodyStateFeature::Create(std::move(*state_reader));
  if (!state.ok()) {
    return state.status();
  }
  body_state_.emplace(std::move(*state));
  auto wrench = BodyWrenchFeature::Create(
      std::move(*wrench_writer), &*body_state_, &*limits_, cycle_config);
  if (!wrench.ok()) {
    return wrench.status();
  }
  wrench_.emplace(std::move(*wrench));

  if (const RealtimeStatus status =
          registry_.RegisterAsCompatibleInterfaces(&*body_state_);
      !status.ok()) {
    return status;
  }
  if (const RealtimeStatus status =
          registry_.RegisterAsCompatibleInterfaces(&*limits_);
      !status.ok()) {
    return status;
  }
  if (const RealtimeStatus status =
          registry_.RegisterAsCompatibleInterfaces(&*wrench_);
      !status.ok()) {
    return status;
  }
  return absl::OkStatus();
}

RealtimeStatus FakeVehicleRealtimePart::SetBodyState(
    const BodyStateSample& sample) {
  if (const RealtimeStatus status = CheckId(sample.source_id); !status.ok()) {
    return status;
  }
  if (const RealtimeStatus status = CheckId(sample.pose_frame_id);
      !status.ok()) {
    return status;
  }
  if (const RealtimeStatus status = CheckId(sample.clock_domain);
      !status.ok()) {
    return status;
  }
  injected_body_state_ = sample;
  return OkStatus();
}

void FakeVehicleRealtimePart::SetOperationalStatus(
    RealtimeOperationalStatus status) {
  operational_status_ = status;
}

RealtimeStatus FakeVehicleRealtimePart::SetActuatorHealth(
    int slot, FakeThrusterActuatorHealth health) {
  if (!SlotInRange(slot)) {
    return OutOfRangeError("thruster slot is out of range");
  }
  if (!KnownHealth(health.health)) {
    return InvalidArgumentError("thruster health is unrecognized");
  }
  health_[slot] = health;
  return OkStatus();
}

RealtimeStatusOr<FakeThrusterActuatorHealth>
FakeVehicleRealtimePart::ActuatorHealth(int slot) const {
  if (!SlotInRange(slot)) {
    return OutOfRangeError("thruster slot is out of range");
  }
  return health_[slot];
}

RealtimeStatusOr<FakeThrusterActuatorHealth>
FakeVehicleRealtimePart::LatchedActuatorHealth(int slot) const {
  if (!SlotInRange(slot)) {
    return OutOfRangeError("thruster slot is out of range");
  }
  return latched_health_[slot];
}

RealtimeStatus FakeVehicleRealtimePart::ReadStatusForTest(SafetyStatus safety) {
  absl::FixedArray<PartPropertyValue> properties_in(0);
  absl::FixedArray<PartPropertyValue> properties_out(0);
  RealtimePartPropertyAccess access(properties_in, properties_out);
  return ReadStatus({.part_properties = access, .safety_status = safety});
}

RealtimeStatus FakeVehicleRealtimePart::ApplyCommandForTest(
    SafetyStatus safety) {
  absl::FixedArray<PartPropertyValue> properties_in(0);
  absl::FixedArray<PartPropertyValue> properties_out(0);
  RealtimePartPropertyAccess access(properties_in, properties_out);
  return ApplyCommand({.part_properties = access, .safety_status = safety});
}

RealtimeStatusOr<RealtimeOperationalStatus>
FakeVehicleRealtimePart::GetOperationalStatus() const {
  return operational_status_;
}

HardwareGroupSet FakeVehicleRealtimePart::GetHardwareDependencies() const {
  return hardware_dependencies_;
}

RealtimeStatus FakeVehicleRealtimePart::ReadStatus(
    ReadStatusParameters params) {
  if (!body_state_.has_value() || !limits_.has_value() ||
      !wrench_.has_value()) {
    return FailedPreconditionError("vehicle fake is not initialized");
  }
  if (const RealtimeStatus status =
          WriteBodyState(body_state_editor_, injected_body_state_);
      !status.ok()) {
    return status;
  }
  body_state_editor_.UpdatedAt(
      Time(Nanoseconds(injected_body_state_.latched_monotonic_ns)));
  latched_health_ = health_;
  if (const RealtimeStatus status = body_state_->ReadStatus(params);
      !status.ok()) {
    return status;
  }
  if (const RealtimeStatus status = limits_->ReadStatus(params); !status.ok()) {
    return status;
  }
  return wrench_->ReadStatus(params);
}

RealtimeStatus FakeVehicleRealtimePart::ApplyCommand(
    ApplyCommandParameters params) {
  if (!limits_.has_value() || !wrench_.has_value() ||
      *body_wrench_reader_ == nullptr) {
    return FailedPreconditionError("vehicle fake is not initialized");
  }
  if (const RealtimeStatus status = limits_->ApplyCommand(params);
      !status.ok()) {
    return status;
  }
  const RealtimeStatus status = wrench_->ApplyCommand(params);
  recorded_wrench_ = CopyWrench(**body_wrench_reader_);
  return status;
}

FeatureInterfaceRegistry& FakeVehicleRealtimePart::GetFeatureInterfaces() {
  return registry_;
}

const FeatureInterfaceRegistry& FakeVehicleRealtimePart::GetFeatureInterfaces()
    const {
  return registry_;
}

}  // namespace intrinsic::icon
