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

#include "intrinsic/icon/hal/interfaces/vehicle_hal_utils.h"

#include <stddef.h>

#include "flatbuffers/flatbuffer_builder.h"
#include "intrinsic/icon/hal/interfaces/vehicle_hal.fbs.h"

namespace intrinsic_fbs {
namespace {

constexpr size_t kBuilderBytes = 4096;

static_assert(sizeof(FixedString64) == 65, "FixedString64 layout changed");
static_assert(alignof(FixedString64) == 1, "FixedString64 alignment changed");
static_assert(sizeof(TimestampRt) == 16, "TimestampRt layout changed");
static_assert(sizeof(Vector3d) == 24, "Vector3d layout changed");
static_assert(sizeof(BodyQuaternion) == 32, "BodyQuaternion layout changed");
static_assert(sizeof(BodyPose) == 56, "BodyPose layout changed");
static_assert(sizeof(BodyTwistRt) == 48, "BodyTwistRt layout changed");
static_assert(sizeof(BodyAccelerationRt) == 48,
              "BodyAccelerationRt layout changed");
static_assert(sizeof(Matrix6d) == 288, "Matrix6d layout changed");
static_assert(alignof(BodyTwistRt) == 8, "body twist alignment changed");
static_assert(alignof(Matrix6d) == 8, "matrix alignment changed");

}  // namespace

flatbuffers::DetachedBuffer BuildBodyState() {
  flatbuffers::FlatBufferBuilder builder(kBuilderBytes);
  builder.ForceDefaults(true);

  const FixedString64 source_id;
  const FixedString64 pose_frame_id;
  const FixedString64 clock_domain;
  const TimestampRt source_time;
  const TimestampRt receive_time;
  const BodyPose pose;
  const BodyTwistRt twist;
  const BodyAccelerationRt acceleration;
  const Matrix6d pose_covariance;
  const Matrix6d twist_covariance;
  const auto state = CreateBodyState(
      builder, /*sequence=*/0, /*has_source_time=*/false, &source_time,
      /*has_receive_time=*/false, &receive_time, &source_id, &pose_frame_id,
      &clock_domain, /*validity_present=*/false, /*validity_state=*/0,
      /*has_pose=*/false, &pose, /*has_body_twist=*/false, &twist,
      /*has_body_acceleration=*/false, &acceleration,
      /*has_pose_covariance=*/false, &pose_covariance,
      /*has_twist_covariance=*/false, &twist_covariance,
      /*navigation_mode=*/0, /*estimator_epoch=*/0);
  builder.Finish(state);
  return builder.Release();
}

flatbuffers::DetachedBuffer BuildBodyWrench() {
  flatbuffers::FlatBufferBuilder builder(kBuilderBytes);
  builder.ForceDefaults(true);

  const FixedString64 source_id;
  const FixedString64 frame_id;
  const FixedString64 clock_domain;
  const TimestampRt source_time;
  const TimestampRt receive_time;
  const auto wrench = CreateBodyWrench(
      builder, /*sequence=*/0, /*has_source_time=*/false, &source_time,
      /*has_receive_time=*/false, &receive_time, &source_id, &frame_id,
      &clock_domain, /*validity_present=*/false, /*validity_state=*/0,
      /*force_x_n=*/0.0, /*force_y_n=*/0.0, /*force_z_n=*/0.0,
      /*torque_x_n_m=*/0.0, /*torque_y_n_m=*/0.0, /*torque_z_n_m=*/0.0);
  builder.Finish(wrench);
  return builder.Release();
}

flatbuffers::DetachedBuffer BuildVehicleLimits() {
  flatbuffers::FlatBufferBuilder builder(kBuilderBytes);
  builder.ForceDefaults(true);

  const Vector3d zero;
  const auto limits = CreateVehicleLimits(
      builder, /*has_translational_position_limits=*/false, &zero, &zero,
      /*has_translational_velocity_limits=*/false, &zero, &zero,
      /*has_translational_acceleration_limits=*/false, &zero, &zero,
      /*has_rotational_velocity_limit=*/false,
      /*max_rotational_velocity_rad_s=*/0.0,
      /*has_rotational_acceleration_limit=*/false,
      /*max_rotational_acceleration_rad_s2=*/0.0, /*has_force_limits=*/false,
      &zero, /*has_torque_limits=*/false, &zero);
  builder.Finish(limits);
  return builder.Release();
}

}  // namespace intrinsic_fbs
