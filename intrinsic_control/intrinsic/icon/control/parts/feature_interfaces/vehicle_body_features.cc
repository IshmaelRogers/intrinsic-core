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

#include <cmath>
#include <cstdint>
#include <optional>
#include <string_view>

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "intrinsic/embodiment/frame_policy.h"
#include "intrinsic/embodiment/stamped_header_policy.h"
#include "intrinsic/icon/control/parts/realtime_part_interface.h"
#include "intrinsic/icon/control/safety/safety_messages.fbs.h"
#include "intrinsic/icon/hal/interfaces/vehicle_hal.fbs.h"
#include "intrinsic/icon/hal/interfaces/vehicle_hal_utils.h"
#include "intrinsic/icon/utils/clock.h"
#include "intrinsic/icon/utils/realtime_status.h"

namespace intrinsic::icon {
namespace {

constexpr uint32_t kNavigationModeFaulted = 4;
constexpr int kMatrixN = 36;
constexpr double kCovarianceSymmetryTolerance = 1e-9;
constexpr std::string_view kBodyFrameId = "body";

using embodiment::ClassifyValidity;
using embodiment::ClockReading;
using embodiment::IsNormalized;
using embodiment::MonotonicAgeSeconds;
using embodiment::Quaternion;
using embodiment::SequenceAdvances;
using embodiment::ValidityKind;

bool Finite(double value) { return std::isfinite(value); }

bool Finite(const VehicleVector3& value) {
  return Finite(value.x) && Finite(value.y) && Finite(value.z);
}

bool Finite6(double a, double b, double c, double d, double e, double f) {
  return Finite(a) && Finite(b) && Finite(c) && Finite(d) && Finite(e) &&
         Finite(f);
}

std::string_view IdView(const FixedId64& id) {
  return std::string_view(id.data, id.length);
}

bool IdEquals(const FixedId64& id, std::string_view text) {
  return IdView(id) == text;
}

void CopyId(FixedId64* out, const intrinsic_fbs::FixedString64* in) {
  *out = FixedId64{};
  if (in == nullptr || in->data() == nullptr) {
    return;
  }
  const size_t cap = in->data()->size();
  size_t length = in->length();
  if (length > cap || length > sizeof(out->data)) {
    return;
  }
  out->length = static_cast<uint8_t>(length);
  for (size_t i = 0; i < length; ++i) {
    out->data[i] = static_cast<char>(in->data()->Get(static_cast<uint16_t>(i)));
  }
}

void CopyVector(VehicleVector3* out, const intrinsic_fbs::Vector3d* in) {
  if (in == nullptr) {
    *out = VehicleVector3{};
    return;
  }
  out->x = in->x();
  out->y = in->y();
  out->z = in->z();
}

void CopyMatrix(double* out, const intrinsic_fbs::Matrix6d* in) {
  for (int i = 0; i < kMatrixN; ++i) {
    out[i] = 0.0;
  }
  if (in == nullptr || in->values() == nullptr) {
    return;
  }
  const int n = static_cast<int>(in->values()->size());
  for (int i = 0; i < kMatrixN && i < n; ++i) {
    out[i] = in->values()->Get(static_cast<uint16_t>(i));
  }
}

bool MatrixFiniteAndSymmetric(const double* values) {
  for (int i = 0; i < kMatrixN; ++i) {
    if (!Finite(values[i])) {
      return false;
    }
  }
  for (int row = 0; row < 6; ++row) {
    for (int col = row + 1; col < 6; ++col) {
      const double delta =
          std::fabs(values[row * 6 + col] - values[col * 6 + row]);
      if (delta > kCovarianceSymmetryTolerance) {
        return false;
      }
    }
  }
  return true;
}

bool NonNegativeMax(double value) { return Finite(value) && value >= 0.0; }

bool NonNegativeMax(const VehicleVector3& value) {
  return NonNegativeMax(value.x) && NonNegativeMax(value.y) &&
         NonNegativeMax(value.z);
}

bool WindowOk(const VehicleVector3& min_value,
              const VehicleVector3& max_value) {
  return Finite(min_value) && Finite(max_value) && min_value.x <= max_value.x &&
         min_value.y <= max_value.y && min_value.z <= max_value.z;
}

bool WindowInside(const VehicleVector3& app_min, const VehicleVector3& app_max,
                  const VehicleVector3& sys_min,
                  const VehicleVector3& sys_max) {
  return app_min.x >= sys_min.x && app_max.x <= sys_max.x &&
         app_min.y >= sys_min.y && app_max.y <= sys_max.y &&
         app_min.z >= sys_min.z && app_max.z <= sys_max.z;
}

bool MaxInside(const VehicleVector3& application,
               const VehicleVector3& system) {
  return application.x <= system.x && application.y <= system.y &&
         application.z <= system.z;
}

absl::Status CheckSample(const VehicleLimitsSample& sample) {
  if (sample.has_translational_position_limits &&
      !WindowOk(sample.min_translational_position_m,
                sample.max_translational_position_m)) {
    return absl::InvalidArgumentError("vehicle limits are invalid");
  }
  if (sample.has_translational_velocity_limits &&
      !WindowOk(sample.min_translational_velocity_m_s,
                sample.max_translational_velocity_m_s)) {
    return absl::InvalidArgumentError("vehicle limits are invalid");
  }
  if (sample.has_translational_acceleration_limits &&
      !WindowOk(sample.min_translational_acceleration_m_s2,
                sample.max_translational_acceleration_m_s2)) {
    return absl::InvalidArgumentError("vehicle limits are invalid");
  }
  if (sample.has_rotational_velocity_limit &&
      !NonNegativeMax(sample.max_rotational_velocity_rad_s)) {
    return absl::InvalidArgumentError("vehicle limits are invalid");
  }
  if (sample.has_rotational_acceleration_limit &&
      !NonNegativeMax(sample.max_rotational_acceleration_rad_s2)) {
    return absl::InvalidArgumentError("vehicle limits are invalid");
  }
  if (sample.has_force_limits && !NonNegativeMax(sample.max_force_n)) {
    return absl::InvalidArgumentError("vehicle limits are invalid");
  }
  if (sample.has_torque_limits && !NonNegativeMax(sample.max_torque_n_m)) {
    return absl::InvalidArgumentError("vehicle limits are invalid");
  }
  return absl::OkStatus();
}

absl::Status CheckContained(const VehicleLimitsSample& application,
                            const VehicleLimitsSample& system) {
  if (application.has_translational_position_limits &&
      system.has_translational_position_limits &&
      !WindowInside(application.min_translational_position_m,
                    application.max_translational_position_m,
                    system.min_translational_position_m,
                    system.max_translational_position_m)) {
    return absl::FailedPreconditionError(
        "application limits exceed system limits");
  }
  if (application.has_translational_velocity_limits &&
      system.has_translational_velocity_limits &&
      !WindowInside(application.min_translational_velocity_m_s,
                    application.max_translational_velocity_m_s,
                    system.min_translational_velocity_m_s,
                    system.max_translational_velocity_m_s)) {
    return absl::FailedPreconditionError(
        "application limits exceed system limits");
  }
  if (application.has_translational_acceleration_limits &&
      system.has_translational_acceleration_limits &&
      !WindowInside(application.min_translational_acceleration_m_s2,
                    application.max_translational_acceleration_m_s2,
                    system.min_translational_acceleration_m_s2,
                    system.max_translational_acceleration_m_s2)) {
    return absl::FailedPreconditionError(
        "application limits exceed system limits");
  }
  if (application.has_rotational_velocity_limit &&
      system.has_rotational_velocity_limit &&
      application.max_rotational_velocity_rad_s >
          system.max_rotational_velocity_rad_s) {
    return absl::FailedPreconditionError(
        "application limits exceed system limits");
  }
  if (application.has_rotational_acceleration_limit &&
      system.has_rotational_acceleration_limit &&
      application.max_rotational_acceleration_rad_s2 >
          system.max_rotational_acceleration_rad_s2) {
    return absl::FailedPreconditionError(
        "application limits exceed system limits");
  }
  if (application.has_force_limits && system.has_force_limits &&
      !MaxInside(application.max_force_n, system.max_force_n)) {
    return absl::FailedPreconditionError(
        "application limits exceed system limits");
  }
  if (application.has_torque_limits && system.has_torque_limits &&
      !MaxInside(application.max_torque_n_m, system.max_torque_n_m)) {
    return absl::FailedPreconditionError(
        "application limits exceed system limits");
  }
  return absl::OkStatus();
}

BodyStateSample CopyState(const intrinsic_fbs::BodyState& state, Time updated) {
  BodyStateSample sample;
  sample.latched = true;
  sample.latched_monotonic_ns = Clock::ToNSec(updated);
  sample.sequence = state.sequence();
  sample.has_source_time = state.has_source_time();
  if (state.source_time() != nullptr) {
    sample.source_time_seconds = state.source_time()->seconds();
    sample.source_time_nanos = state.source_time()->nanos();
  }
  sample.has_receive_time = state.has_receive_time();
  if (state.receive_time() != nullptr) {
    sample.receive_time_seconds = state.receive_time()->seconds();
    sample.receive_time_nanos = state.receive_time()->nanos();
  }
  CopyId(&sample.source_id, state.source_id());
  CopyId(&sample.pose_frame_id, state.pose_frame_id());
  CopyId(&sample.clock_domain, state.clock_domain());
  sample.validity_present = state.validity_present();
  sample.validity_state = state.validity_state();
  sample.has_pose = state.has_pose();
  if (state.pose_world_from_body() != nullptr) {
    const intrinsic_fbs::BodyPose& pose = *state.pose_world_from_body();
    CopyVector(&sample.position_m, &pose.position());
    const intrinsic_fbs::BodyQuaternion& orientation = pose.orientation();
    sample.orientation_x = orientation.x();
    sample.orientation_y = orientation.y();
    sample.orientation_z = orientation.z();
    sample.orientation_w = orientation.w();
  }
  sample.has_body_twist = state.has_body_twist();
  if (state.body_twist() != nullptr) {
    const auto* twist = state.body_twist();
    sample.linear_x_m_s = twist->linear_x_m_s();
    sample.linear_y_m_s = twist->linear_y_m_s();
    sample.linear_z_m_s = twist->linear_z_m_s();
    sample.angular_x_rad_s = twist->angular_x_rad_s();
    sample.angular_y_rad_s = twist->angular_y_rad_s();
    sample.angular_z_rad_s = twist->angular_z_rad_s();
  }
  sample.has_body_acceleration = state.has_body_acceleration();
  if (state.body_acceleration() != nullptr) {
    const auto* acceleration = state.body_acceleration();
    sample.linear_x_m_s2 = acceleration->linear_x_m_s2();
    sample.linear_y_m_s2 = acceleration->linear_y_m_s2();
    sample.linear_z_m_s2 = acceleration->linear_z_m_s2();
    sample.angular_x_rad_s2 = acceleration->angular_x_rad_s2();
    sample.angular_y_rad_s2 = acceleration->angular_y_rad_s2();
    sample.angular_z_rad_s2 = acceleration->angular_z_rad_s2();
  }
  sample.has_pose_covariance = state.has_pose_covariance();
  CopyMatrix(sample.pose_covariance, state.pose_covariance());
  sample.has_twist_covariance = state.has_twist_covariance();
  CopyMatrix(sample.twist_covariance, state.twist_covariance());
  sample.navigation_mode = state.navigation_mode();
  sample.estimator_epoch = state.estimator_epoch();
  return sample;
}

RealtimeStatus StateUsable(const BodyStateSample& sample) {
  if (!sample.latched) {
    return FailedPreconditionError("body state is absent");
  }
  const ValidityKind kind =
      ClassifyValidity(sample.validity_present, sample.validity_state);
  if (kind == ValidityKind::kInvalid) {
    return FailedPreconditionError("body state is invalid");
  }
  if (kind == ValidityKind::kAbsent) {
    return FailedPreconditionError("body state validity is absent");
  }
  if (kind != ValidityKind::kValid) {
    return FailedPreconditionError("body state validity is unspecified");
  }
  if (sample.navigation_mode == kNavigationModeFaulted) {
    return FailedPreconditionError("navigation is faulted");
  }
  if (sample.has_pose) {
    if (sample.pose_frame_id.length == 0) {
      return FailedPreconditionError("pose frame is absent");
    }
    const Quaternion orientation{sample.orientation_x, sample.orientation_y,
                                 sample.orientation_z, sample.orientation_w};
    if (!Finite(sample.position_m) || !embodiment::IsFinite(orientation)) {
      return FailedPreconditionError("body state is non-finite");
    }
    if (!IsNormalized(orientation)) {
      return FailedPreconditionError("pose quaternion is not unit");
    }
  }
  if (sample.has_body_twist &&
      !Finite6(sample.linear_x_m_s, sample.linear_y_m_s, sample.linear_z_m_s,
               sample.angular_x_rad_s, sample.angular_y_rad_s,
               sample.angular_z_rad_s)) {
    return FailedPreconditionError("body state is non-finite");
  }
  if (sample.has_body_acceleration &&
      !Finite6(sample.linear_x_m_s2, sample.linear_y_m_s2, sample.linear_z_m_s2,
               sample.angular_x_rad_s2, sample.angular_y_rad_s2,
               sample.angular_z_rad_s2)) {
    return FailedPreconditionError("body state is non-finite");
  }
  if (sample.has_pose_covariance &&
      !MatrixFiniteAndSymmetric(sample.pose_covariance)) {
    return FailedPreconditionError("pose covariance is invalid");
  }
  if (sample.has_twist_covariance &&
      !MatrixFiniteAndSymmetric(sample.twist_covariance)) {
    return FailedPreconditionError("twist covariance is invalid");
  }
  return OkStatus();
}

std::optional<double> StampAge(bool has_source, int64_t source_seconds,
                               int32_t source_nanos, bool has_receive,
                               int64_t receive_seconds, int32_t receive_nanos,
                               const FixedId64& clock_domain) {
  std::optional<ClockReading> source;
  std::optional<ClockReading> receive;
  if (has_source) {
    source = ClockReading{source_seconds, source_nanos};
  }
  if (has_receive) {
    receive = ClockReading{receive_seconds, receive_nanos};
  }
  return MonotonicAgeSeconds(source, receive, IdView(clock_domain));
}

bool SafetyEmergency(const SafetyStatus& status) {
  if (status.estop_button_status == intrinsic_fbs::ButtonStatus::ENGAGED) {
    return true;
  }
  switch (status.requested_behavior) {
    case intrinsic_fbs::RequestedBehavior::SAFE_STOP_0:
    case intrinsic_fbs::RequestedBehavior::SAFE_STOP_1_TIME_MONITORED:
    case intrinsic_fbs::RequestedBehavior::SAFE_STOP_2_TIME_MONITORED:
      return true;
    default:
      return false;
  }
}

bool Exceeds(double value, double limit) { return std::fabs(value) > limit; }

bool Exceeds(double x, double y, double z, const VehicleVector3& limit) {
  return Exceeds(x, limit.x) || Exceeds(y, limit.y) || Exceeds(z, limit.z);
}

bool AgeExceeded(Time stamped, Time now, Duration limit) {
  if (now < stamped) {
    return true;
  }
  return (now - stamped) > limit;
}

}  // namespace

absl::StatusOr<BodyStateFeature> BodyStateFeature::Create(
    HardwareInterfaceHandle<intrinsic_fbs::BodyState> body_state) {
  if (*body_state == nullptr) {
    return absl::InvalidArgumentError("body state hardware interface is empty");
  }
  return BodyStateFeature(std::move(body_state));
}

BodyStateFeature::BodyStateFeature(
    HardwareInterfaceHandle<intrinsic_fbs::BodyState> body_state)
    : body_state_(std::move(body_state)) {}

RealtimeStatus BodyStateFeature::ReadStatus(
    RealtimePartInterface::ReadStatusParameters /*params*/) {
  if (*body_state_ == nullptr) {
    return FailedPreconditionError("body state hardware interface is empty");
  }
  latched_ = CopyState(**body_state_, body_state_.LastUpdatedTime());
  return OkStatus();
}

absl::StatusOr<VehicleLimitsFeature> VehicleLimitsFeature::Create(
    VehicleLimitsSample application, VehicleLimitsSample system) {
  if (const absl::Status application_status = CheckSample(application);
      !application_status.ok()) {
    return application_status;
  }
  if (const absl::Status system_status = CheckSample(system);
      !system_status.ok()) {
    return system_status;
  }
  if (const absl::Status contained = CheckContained(application, system);
      !contained.ok()) {
    return contained;
  }
  return VehicleLimitsFeature(application, system);
}

VehicleLimitsFeature::VehicleLimitsFeature(VehicleLimitsSample application,
                                           VehicleLimitsSample system)
    : application_(application), system_(system) {}

absl::StatusOr<BodyWrenchFeature> BodyWrenchFeature::Create(
    MutableHardwareInterfaceHandle<intrinsic_fbs::BodyWrench> wrench,
    const BodyState* body_state, const VehicleLimitsInterface* limits,
    VehicleFeatureCycleConfig config) {
  if (*wrench == nullptr) {
    return absl::InvalidArgumentError(
        "body wrench hardware interface is empty");
  }
  if (body_state == nullptr) {
    return absl::FailedPreconditionError("body state capability is absent");
  }
  if (limits == nullptr) {
    return absl::FailedPreconditionError("vehicle limits capability is absent");
  }
  if (config.max_command_age < Duration::zero() ||
      config.max_state_age < Duration::zero()) {
    return absl::InvalidArgumentError("command age limit is negative");
  }
  return BodyWrenchFeature(std::move(wrench), body_state, limits, config);
}

BodyWrenchFeature::BodyWrenchFeature(
    MutableHardwareInterfaceHandle<intrinsic_fbs::BodyWrench> wrench,
    const BodyState* body_state, const VehicleLimitsInterface* limits,
    VehicleFeatureCycleConfig config)
    : wrench_(std::move(wrench)),
      body_state_(body_state),
      limits_(limits),
      config_(config) {}

void BodyWrenchFeature::WriteSample(const BodyWrenchSample& sample) {
  intrinsic_fbs::BodyWrench* out = *wrench_;
  out->mutate_sequence(sample.sequence);
  out->mutate_has_source_time(sample.has_source_time);
  if (out->mutable_source_time() != nullptr) {
    out->mutable_source_time()->mutate_seconds(sample.source_time_seconds);
    out->mutable_source_time()->mutate_nanos(sample.source_time_nanos);
  }
  out->mutate_has_receive_time(sample.has_receive_time);
  if (out->mutable_receive_time() != nullptr) {
    out->mutable_receive_time()->mutate_seconds(sample.receive_time_seconds);
    out->mutable_receive_time()->mutate_nanos(sample.receive_time_nanos);
  }
  if (out->mutable_source_id() != nullptr) {
    intrinsic_fbs::AssignFixedString64(out->mutable_source_id(),
                                       IdView(sample.source_id));
  }
  if (out->mutable_frame_id() != nullptr) {
    intrinsic_fbs::AssignFixedString64(out->mutable_frame_id(),
                                       IdView(sample.frame_id));
  }
  if (out->mutable_clock_domain() != nullptr) {
    intrinsic_fbs::AssignFixedString64(out->mutable_clock_domain(),
                                       IdView(sample.clock_domain));
  }
  out->mutate_validity_present(sample.validity_present);
  out->mutate_validity_state(sample.validity_state);
  out->mutate_force_x_n(sample.force_x_n);
  out->mutate_force_y_n(sample.force_y_n);
  out->mutate_force_z_n(sample.force_z_n);
  out->mutate_torque_x_n_m(sample.torque_x_n_m);
  out->mutate_torque_y_n_m(sample.torque_y_n_m);
  out->mutate_torque_z_n_m(sample.torque_z_n_m);
  wrench_.UpdatedAt(Clock::Now());
}

BodyWrenchSample BodyWrenchFeature::NeutralSample() const {
  BodyWrenchSample sample;
  sample.applied = true;
  sample.sequence = have_sequence_ ? last_sequence_ : 0;
  sample.frame_id.length = static_cast<uint8_t>(kBodyFrameId.size());
  for (size_t i = 0; i < kBodyFrameId.size(); ++i) {
    sample.frame_id.data[i] = kBodyFrameId[i];
  }
  sample.clock_domain.length = 9;
  const char* domain = "monotonic";
  for (int i = 0; i < 9; ++i) {
    sample.clock_domain.data[i] = domain[i];
  }
  sample.validity_present = true;
  sample.validity_state = 1;
  return sample;
}

BodyWrenchFeature::CycleFault BodyWrenchFeature::ActiveFault() const {
  if (watchdog_latched_) {
    CycleFault latched;
    latched.rank = FaultRank::kCommandWatchdog;
    latched.status = DeadlineExceededError("command watchdog requires reset");
    if (cycle_fault_.rank > latched.rank) {
      return cycle_fault_;
    }
    return latched;
  }
  return cycle_fault_;
}

RealtimeStatus BodyWrenchFeature::ReadStatus(
    RealtimePartInterface::ReadStatusParameters params) {
  pending_ = false;
  cycle_fault_ = CycleFault{};
  if (SafetyEmergency(params.safety_status)) {
    cycle_fault_.rank = FaultRank::kSafety;
    cycle_fault_.status = FailedPreconditionError("safety emergency");
    return OkStatus();
  }
  if (body_state_ == nullptr) {
    cycle_fault_.rank = FaultRank::kInvalidState;
    cycle_fault_.status =
        FailedPreconditionError("body state capability is absent");
    return OkStatus();
  }
  const BodyStateSample& sample = body_state_->LatchedBodyState();
  const RealtimeStatus usable = StateUsable(sample);
  if (!usable.ok()) {
    cycle_fault_.rank = FaultRank::kInvalidState;
    cycle_fault_.status = usable;
    return OkStatus();
  }
  const Time stamped(Nanoseconds(sample.latched_monotonic_ns));
  if (AgeExceeded(stamped, Clock::Now(), config_.max_state_age)) {
    cycle_fault_.rank = FaultRank::kStateStale;
    cycle_fault_.status = DeadlineExceededError("body state is stale");
    return OkStatus();
  }
  if (embodiment::IsMonotonicClockDomain(IdView(sample.clock_domain)) &&
      sample.has_source_time && sample.has_receive_time) {
    const std::optional<double> age =
        StampAge(sample.has_source_time, sample.source_time_seconds,
                 sample.source_time_nanos, sample.has_receive_time,
                 sample.receive_time_seconds, sample.receive_time_nanos,
                 sample.clock_domain);
    if (!age.has_value()) {
      cycle_fault_.rank = FaultRank::kInvalidState;
      cycle_fault_.status =
          InvalidArgumentError("body state timestamps are inconsistent");
      return OkStatus();
    }
    if (*age > ToDoubleSeconds(config_.max_state_age)) {
      cycle_fault_.rank = FaultRank::kStateStale;
      cycle_fault_.status = DeadlineExceededError("body state is stale");
    }
  }
  return OkStatus();
}

RealtimeStatus BodyWrenchFeature::ApplyCommand(
    RealtimePartInterface::ApplyCommandParameters /*params*/) {
  const CycleFault fault = ActiveFault();
  if (fault.rank != FaultRank::kNone) {
    const BodyWrenchSample neutral = NeutralSample();
    WriteSample(neutral);
    previous_ = neutral;
    pending_ = false;
    return fault.status;
  }

  const Time now = Clock::Now();
  if (pending_) {
    if (AgeExceeded(accepted_at_, now, config_.max_command_age)) {
      watchdog_latched_ = true;
      pending_ = false;
      const BodyWrenchSample neutral = NeutralSample();
      WriteSample(neutral);
      previous_ = neutral;
      return DeadlineExceededError("command watchdog requires reset");
    }
    pending_command_.applied = true;
    WriteSample(pending_command_);
    previous_ = pending_command_;
    last_applied_at_ = now;
    last_sequence_ = pending_command_.sequence;
    have_sequence_ = true;
    command_stream_started_ = true;
    pending_ = false;
    return OkStatus();
  }

  if (command_stream_started_ && previous_.applied &&
      AgeExceeded(last_applied_at_, now, config_.max_command_age)) {
    watchdog_latched_ = true;
    const BodyWrenchSample neutral = NeutralSample();
    WriteSample(neutral);
    previous_ = neutral;
    return DeadlineExceededError("command watchdog requires reset");
  }

  if (previous_.applied) {
    WriteSample(previous_);
    return OkStatus();
  }

  const BodyWrenchSample neutral = NeutralSample();
  WriteSample(neutral);
  previous_ = neutral;
  return OkStatus();
}

RealtimeStatus BodyWrenchFeature::Reset() {
  pending_ = false;
  watchdog_latched_ = false;
  command_stream_started_ = false;
  cycle_fault_ = CycleFault{};
  return OkStatus();
}

RealtimeStatus BodyWrenchFeature::SetBodyWrench(
    const BodyWrenchSample& command) {
  const CycleFault fault = ActiveFault();
  if (fault.rank != FaultRank::kNone) {
    return fault.status;
  }
  if (!Finite6(command.force_x_n, command.force_y_n, command.force_z_n,
               command.torque_x_n_m, command.torque_y_n_m,
               command.torque_z_n_m)) {
    return InvalidArgumentError("wrench is non-finite");
  }
  if (!IdEquals(command.frame_id, kBodyFrameId)) {
    return InvalidArgumentError("wrench frame must be body");
  }
  const ValidityKind kind =
      ClassifyValidity(command.validity_present, command.validity_state);
  if (kind == ValidityKind::kInvalid) {
    return FailedPreconditionError("wrench validity is invalid");
  }
  if (kind != ValidityKind::kValid) {
    return InvalidArgumentError("wrench validity is absent");
  }
  if (have_sequence_ && !SequenceAdvances(last_sequence_, command.sequence)) {
    return InvalidArgumentError("wrench sequence did not advance");
  }
  if (embodiment::IsMonotonicClockDomain(IdView(command.clock_domain)) &&
      command.has_source_time && command.has_receive_time) {
    const std::optional<double> age =
        StampAge(command.has_source_time, command.source_time_seconds,
                 command.source_time_nanos, command.has_receive_time,
                 command.receive_time_seconds, command.receive_time_nanos,
                 command.clock_domain);
    if (!age.has_value()) {
      return InvalidArgumentError("wrench timestamps are inconsistent");
    }
    if (*age > ToDoubleSeconds(config_.max_command_age)) {
      return DeadlineExceededError("body wrench is stale");
    }
  }
  if (limits_ == nullptr) {
    return FailedPreconditionError("vehicle limits capability is absent");
  }
  const VehicleLimitsSample& system = limits_->GetSystemLimits();
  const VehicleLimitsSample& application = limits_->GetApplicationLimits();
  if ((system.has_force_limits &&
       Exceeds(command.force_x_n, command.force_y_n, command.force_z_n,
               system.max_force_n)) ||
      (system.has_torque_limits &&
       Exceeds(command.torque_x_n_m, command.torque_y_n_m, command.torque_z_n_m,
               system.max_torque_n_m))) {
    return FailedPreconditionError("wrench exceeds system limits");
  }
  if ((application.has_force_limits &&
       Exceeds(command.force_x_n, command.force_y_n, command.force_z_n,
               application.max_force_n)) ||
      (application.has_torque_limits &&
       Exceeds(command.torque_x_n_m, command.torque_y_n_m, command.torque_z_n_m,
               application.max_torque_n_m))) {
    return OutOfRangeError("wrench exceeds application limits");
  }

  pending_command_ = command;
  pending_command_.applied = false;
  accepted_at_ = Clock::Now();
  pending_ = true;
  return OkStatus();
}

}  // namespace intrinsic::icon
