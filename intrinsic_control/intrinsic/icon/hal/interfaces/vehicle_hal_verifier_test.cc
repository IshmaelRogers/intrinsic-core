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
#include <stdint.h>
#include <string.h>

#include <cstdlib>
#include <iostream>
#include <limits>
#include <string>
#include <string_view>
#include <vector>

#include "flatbuffers/flatbuffers.h"
#include "intrinsic/icon/hal/interfaces/vehicle_hal.fbs.h"
#include "intrinsic/icon/hal/interfaces/vehicle_hardware_interfaces.h"

namespace {
int g_failures = 0;

void Expect(bool condition, const char* expression, int line) {
  if (!condition) {
    std::cerr << "vehicle_hal_verifier_test.cc:" << line << " " << expression
              << "\n";
    ++g_failures;
  }
}

#define EXPECT(condition) \
  Expect(static_cast<bool>(condition), #condition, __LINE__)

bool SameBytes(const flatbuffers::DetachedBuffer& left,
               const flatbuffers::DetachedBuffer& right) {
  return left.size() == right.size() &&
         memcmp(left.data(), right.data(), left.size()) == 0;
}

template <typename T>
bool Verifies(const flatbuffers::DetachedBuffer& buffer) {
  flatbuffers::Verifier verifier(buffer.data(), buffer.size());
  return verifier.VerifyBuffer<T>();
}

template <typename T>
bool VerifiesSpan(const uint8_t* data, size_t size) {
  flatbuffers::Verifier verifier(data, size);
  return verifier.VerifyBuffer<T>();
}

void ExpectLittleEndianDouble(const void* field, double value, int line) {
  uint64_t bits = 0;
  memcpy(&bits, &value, sizeof(bits));
  const auto* raw = static_cast<const uint8_t*>(field);
  for (int i = 0; i < 8; ++i) {
    const auto byte = static_cast<uint8_t>((bits >> (8 * i)) & 0xff);
    if (raw[i] != byte) {
      std::cerr << "vehicle_hal_verifier_test.cc:" << line
                << " little-endian byte " << i << "\n";
      ++g_failures;
    }
  }
}

flatbuffers::DetachedBuffer PopulatedBodyState() {
  flatbuffers::FlatBufferBuilder builder(4096);
  builder.ForceDefaults(true);

  intrinsic_fbs::FixedString64 source_id;
  intrinsic_fbs::FixedString64 pose_frame_id;
  intrinsic_fbs::FixedString64 clock_domain;
  EXPECT(intrinsic_fbs::AssignFixedString64(&source_id, "estimator"));
  EXPECT(intrinsic_fbs::AssignFixedString64(&pose_frame_id, "world_enu"));
  EXPECT(intrinsic_fbs::AssignFixedString64(&clock_domain, "monotonic"));

  const intrinsic_fbs::TimestampRt source_time(/*seconds=*/10, /*nanos=*/20);
  const intrinsic_fbs::TimestampRt receive_time(/*seconds=*/11, /*nanos=*/21);
  const intrinsic_fbs::Vector3d position(1.25, 2.5, -3.5);
  const intrinsic_fbs::BodyQuaternion orientation(0.0, 0.0, 0.1, 0.9);
  const intrinsic_fbs::BodyPose pose(position, orientation);
  const intrinsic_fbs::BodyTwistRt twist(0.5, -0.25, 0.0, 0.0, 0.125, -0.5);
  const intrinsic_fbs::BodyAccelerationRt acceleration(0.0, 0.0, 0.01, 0.0, 0.0,
                                                      0.0);
  intrinsic_fbs::Matrix6d pose_covariance;
  pose_covariance.mutable_values()->Mutate(8, 3.5);
  intrinsic_fbs::Matrix6d twist_covariance;
  twist_covariance.mutable_values()->Mutate(0, 9.0);

  const auto state = intrinsic_fbs::CreateBodyState(
      builder, /*sequence=*/4, /*has_source_time=*/true, &source_time,
      /*has_receive_time=*/true, &receive_time, &source_id, &pose_frame_id,
      &clock_domain, /*validity_present=*/true, /*validity_state=*/9,
      /*has_pose=*/true, &pose, /*has_body_twist=*/true, &twist,
      /*has_body_acceleration=*/true, &acceleration,
      /*has_pose_covariance=*/true, &pose_covariance,
      /*has_twist_covariance=*/false, &twist_covariance,
      /*navigation_mode=*/99, /*estimator_epoch=*/7);
  builder.Finish(state);
  return builder.Release();
}

void TestStructLayout() {
  EXPECT(sizeof(intrinsic_fbs::FixedString64) == 65);
  EXPECT(sizeof(intrinsic_fbs::TimestampRt) == 16);
  EXPECT(sizeof(intrinsic_fbs::Vector3d) == 24);
  EXPECT(sizeof(intrinsic_fbs::BodyQuaternion) == 32);
  EXPECT(sizeof(intrinsic_fbs::BodyPose) == 56);
  EXPECT(sizeof(intrinsic_fbs::BodyTwistRt) == 48);
  EXPECT(sizeof(intrinsic_fbs::BodyAccelerationRt) == 48);
  EXPECT(sizeof(intrinsic_fbs::Matrix6d) == 288);
  EXPECT(alignof(intrinsic_fbs::BodyTwistRt) == 8);
  EXPECT(alignof(intrinsic_fbs::FixedString64) == 1);
}

void TestNeutralBuffersVerifyAndRepeat() {
  const auto state_a = intrinsic_fbs::BuildBodyState();
  const auto state_b = intrinsic_fbs::BuildBodyState();
  EXPECT(Verifies<intrinsic_fbs::BodyState>(state_a));
  EXPECT(SameBytes(state_a, state_b));
  EXPECT(state_a.size() > 1);

  const auto* state =
      flatbuffers::GetRoot<intrinsic_fbs::BodyState>(state_a.data());
  EXPECT(state != nullptr);
  EXPECT(!state->has_pose());
  EXPECT(!state->has_body_twist());
  EXPECT(!state->has_body_acceleration());
  EXPECT(!state->has_pose_covariance());
  EXPECT(!state->has_twist_covariance());
  EXPECT(!state->validity_present());
  EXPECT(state->navigation_mode() == 0);
  EXPECT(state->estimator_epoch() == 0);
  EXPECT(state->pose_frame_id() != nullptr);
  EXPECT(state->pose_frame_id()->length() == 0);
  EXPECT(intrinsic_fbs::ViewFixedString64(*state->pose_frame_id()).empty());

  const auto wrench_a = intrinsic_fbs::BuildBodyWrench();
  const auto wrench_b = intrinsic_fbs::BuildBodyWrench();
  EXPECT(Verifies<intrinsic_fbs::BodyWrench>(wrench_a));
  EXPECT(SameBytes(wrench_a, wrench_b));
  const auto* wrench =
      flatbuffers::GetRoot<intrinsic_fbs::BodyWrench>(wrench_a.data());
  EXPECT(wrench->force_x_n() == 0.0);
  EXPECT(wrench->torque_z_n_m() == 0.0);
  EXPECT(wrench->frame_id()->length() == 0);

  const auto limits_a = intrinsic_fbs::BuildVehicleLimits();
  const auto limits_b = intrinsic_fbs::BuildVehicleLimits();
  EXPECT(Verifies<intrinsic_fbs::VehicleLimits>(limits_a));
  EXPECT(SameBytes(limits_a, limits_b));
  const auto* limits =
      flatbuffers::GetRoot<intrinsic_fbs::VehicleLimits>(limits_a.data());
  EXPECT(!limits->has_force_limits());
  EXPECT(!limits->has_translational_velocity_limits());
  EXPECT(limits->max_rotational_velocity_rad_s() == 0.0);
}

void TestPopulatedBodyState() {
  const auto buffer = PopulatedBodyState();
  const auto again = PopulatedBodyState();
  EXPECT(Verifies<intrinsic_fbs::BodyState>(buffer));
  EXPECT(SameBytes(buffer, again));
  EXPECT(!SameBytes(buffer, intrinsic_fbs::BuildBodyState()));

  const auto* state =
      flatbuffers::GetRoot<intrinsic_fbs::BodyState>(buffer.data());
  EXPECT(state->sequence() == 4);
  EXPECT(state->has_source_time());
  EXPECT(state->source_time()->seconds() == 10);
  EXPECT(state->source_time()->nanos() == 20);
  EXPECT(state->has_receive_time());
  EXPECT(state->receive_time()->seconds() == 11);
  EXPECT(intrinsic_fbs::ViewFixedString64(*state->source_id()) == "estimator");
  EXPECT(intrinsic_fbs::ViewFixedString64(*state->pose_frame_id()) ==
         "world_enu");
  EXPECT(intrinsic_fbs::ViewFixedString64(*state->clock_domain()) ==
         "monotonic");
  EXPECT(state->validity_present());
  EXPECT(state->validity_state() == 9);
  EXPECT(state->validity_state() != 1);
  EXPECT(state->validity_state() != 2);
  EXPECT(state->has_pose());
  EXPECT(state->pose_world_from_body()->position().x() == 1.25);
  EXPECT(state->pose_world_from_body()->position().z() == -3.5);
  EXPECT(state->pose_world_from_body()->orientation().z() == 0.1);
  EXPECT(state->pose_world_from_body()->orientation().w() == 0.9);
  EXPECT(state->has_body_twist());
  EXPECT(state->body_twist()->linear_x_m_s() == 0.5);
  EXPECT(state->body_twist()->linear_y_m_s() == -0.25);
  EXPECT(state->body_twist()->angular_y_rad_s() == 0.125);
  EXPECT(state->body_twist()->angular_z_rad_s() == -0.5);
  EXPECT(state->has_body_acceleration());
  EXPECT(state->body_acceleration()->linear_z_m_s2() == 0.01);
  EXPECT(state->has_pose_covariance());
  EXPECT(state->pose_covariance()->values()->Get(8) == 3.5);
  EXPECT(state->pose_covariance()->values()->Get(0) == 0.0);
  EXPECT(!state->has_twist_covariance());
  EXPECT(state->twist_covariance()->values()->Get(0) == 9.0);
  EXPECT(state->navigation_mode() == 99);
  EXPECT(state->navigation_mode() != 3);
  EXPECT(state->navigation_mode() != 4);
  EXPECT(state->estimator_epoch() == 7);

  ExpectLittleEndianDouble(state->body_twist(), 0.5, __LINE__);
  ExpectLittleEndianDouble(&state->pose_world_from_body()->position(), 1.25,
                           __LINE__);
  const double matrix_value = 3.5;
  const auto* matrix_bytes = reinterpret_cast<const uint8_t*>(
      state->pose_covariance()->values()->Data());
  ExpectLittleEndianDouble(matrix_bytes + (8 * sizeof(double)), matrix_value,
                           __LINE__);
}

void TestAbsentPoseIsNotZeroPose() {
  flatbuffers::FlatBufferBuilder absent_builder(4096);
  absent_builder.ForceDefaults(true);
  const intrinsic_fbs::BodyPose zeros;
  const auto absent = intrinsic_fbs::CreateBodyState(
      absent_builder, 0, false, nullptr, false, nullptr, nullptr, nullptr,
      nullptr, false, 0, /*has_pose=*/false, &zeros, false, nullptr, false,
      nullptr, false, nullptr, false, nullptr, 0, 0);
  absent_builder.Finish(absent);

  flatbuffers::FlatBufferBuilder present_builder(4096);
  present_builder.ForceDefaults(true);
  const auto present = intrinsic_fbs::CreateBodyState(
      present_builder, 0, false, nullptr, false, nullptr, nullptr, nullptr,
      nullptr, false, 0, /*has_pose=*/true, &zeros, false, nullptr, false,
      nullptr, false, nullptr, false, nullptr, 0, 0);
  present_builder.Finish(present);

  const auto absent_buffer = absent_builder.Release();
  const auto present_buffer = present_builder.Release();
  EXPECT(Verifies<intrinsic_fbs::BodyState>(absent_buffer));
  EXPECT(Verifies<intrinsic_fbs::BodyState>(present_buffer));
  EXPECT(!SameBytes(absent_buffer, present_buffer));
  EXPECT(!flatbuffers::GetRoot<intrinsic_fbs::BodyState>(absent_buffer.data())
              ->has_pose());
  EXPECT(flatbuffers::GetRoot<intrinsic_fbs::BodyState>(present_buffer.data())
             ->has_pose());
  EXPECT(flatbuffers::GetRoot<intrinsic_fbs::BodyState>(present_buffer.data())
             ->pose_world_from_body()
             ->position()
             .x() == 0.0);
}

void TestWrenchNeutralAndNan() {
  flatbuffers::FlatBufferBuilder builder(1024);
  builder.ForceDefaults(true);
  intrinsic_fbs::FixedString64 frame_id;
  EXPECT(intrinsic_fbs::AssignFixedString64(&frame_id, "body"));
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const auto wrench = intrinsic_fbs::CreateBodyWrench(
      builder, /*sequence=*/1, false, nullptr, false, nullptr, nullptr,
      &frame_id, nullptr, true, /*validity_state=*/1, nan, 0.0, -2.0, 0.0, 4.0,
      0.0);
  builder.Finish(wrench);
  const auto buffer = builder.Release();
  EXPECT(Verifies<intrinsic_fbs::BodyWrench>(buffer));
  const auto* value =
      flatbuffers::GetRoot<intrinsic_fbs::BodyWrench>(buffer.data());
  EXPECT(intrinsic_fbs::ViewFixedString64(*value->frame_id()) == "body");
  EXPECT(value->force_z_n() == -2.0);
  EXPECT(value->torque_y_n_m() == 4.0);
  EXPECT(value->validity_present());
  EXPECT(value->validity_state() == 1);
  double loaded = value->force_x_n();
  EXPECT(memcmp(&loaded, &nan, sizeof(double)) == 0);
}

void TestVehicleLimitFlag() {
  flatbuffers::FlatBufferBuilder unconfigured(1024);
  unconfigured.ForceDefaults(true);
  const intrinsic_fbs::Vector3d zero;
  const auto absent = intrinsic_fbs::CreateVehicleLimits(
      unconfigured, false, &zero, &zero, false, &zero, &zero, false, &zero,
      &zero, false, 0.0, false, 0.0, /*has_force_limits=*/false, &zero, false,
      &zero);
  unconfigured.Finish(absent);

  flatbuffers::FlatBufferBuilder configured(1024);
  configured.ForceDefaults(true);
  const intrinsic_fbs::Vector3d max_force(1.0, 2.0, 3.0);
  const auto present = intrinsic_fbs::CreateVehicleLimits(
      configured, false, &zero, &zero, false, &zero, &zero, false, &zero, &zero,
      false, 0.0, false, 0.0, /*has_force_limits=*/true, &max_force, false,
      &zero);
  configured.Finish(present);

  const auto absent_buffer = unconfigured.Release();
  const auto present_buffer = configured.Release();
  EXPECT(Verifies<intrinsic_fbs::VehicleLimits>(absent_buffer));
  EXPECT(Verifies<intrinsic_fbs::VehicleLimits>(present_buffer));
  EXPECT(!SameBytes(absent_buffer, present_buffer));
  const auto* limits =
      flatbuffers::GetRoot<intrinsic_fbs::VehicleLimits>(present_buffer.data());
  EXPECT(limits->has_force_limits());
  EXPECT(!limits->has_torque_limits());
  EXPECT(limits->max_force_n()->x() == 1.0);
  EXPECT(limits->max_force_n()->y() == 2.0);
  EXPECT(limits->max_force_n()->z() == 3.0);
  ExpectLittleEndianDouble(limits->max_force_n(), 1.0, __LINE__);
  const auto* absent_limits =
      flatbuffers::GetRoot<intrinsic_fbs::VehicleLimits>(absent_buffer.data());
  EXPECT(!absent_limits->has_force_limits());
}

void TestRejectedStringAndVerifierFailures() {
  intrinsic_fbs::FixedString64 text;
  EXPECT(text.length() == 0);
  const std::string too_long(65, 'a');
  EXPECT(!intrinsic_fbs::AssignFixedString64(&text, too_long));
  EXPECT(text.length() == 0);
  EXPECT(intrinsic_fbs::AssignFixedString64(&text, std::string(64, 'b')));
  EXPECT(text.length() == 64);
  EXPECT(intrinsic_fbs::ViewFixedString64(text).size() == 64);

  text.mutate_length(255);
  EXPECT(intrinsic_fbs::ViewFixedString64(text).empty());

  const auto buffer = intrinsic_fbs::BuildBodyState();
  EXPECT(!VerifiesSpan<intrinsic_fbs::BodyState>(buffer.data(), 0));
  EXPECT(!VerifiesSpan<intrinsic_fbs::BodyState>(buffer.data(),
                                                buffer.size() - 1));
  std::vector<uint8_t> storage(buffer.size() + 16);
  auto address = reinterpret_cast<uintptr_t>(storage.data());
  auto* aligned = reinterpret_cast<uint8_t*>((address + 15) & ~uintptr_t{15});
  memcpy(aligned, buffer.data(), buffer.size());
  aligned[0] = 0xff;
  aligned[1] = 0xff;
  aligned[2] = 0xff;
  aligned[3] = 0xff;
  EXPECT(!VerifiesSpan<intrinsic_fbs::BodyState>(aligned, buffer.size()));

  const auto wrench = intrinsic_fbs::BuildBodyWrench();
  EXPECT(!VerifiesSpan<intrinsic_fbs::BodyWrench>(wrench.data(),
                                                 wrench.size() - 1));
  const auto limits = intrinsic_fbs::BuildVehicleLimits();
  EXPECT(!VerifiesSpan<intrinsic_fbs::VehicleLimits>(limits.data(),
                                                    limits.size() - 1));
}

void TestTypeIds() {
  using intrinsic::icon::hardware_interface_traits::TypeID;
  EXPECT(std::string_view(intrinsic_fbs::kBodyStateTypeId) ==
         "intrinsic_fbs.BodyState");
  EXPECT(std::string_view(intrinsic_fbs::kBodyWrenchTypeId) ==
         "intrinsic_fbs.BodyWrench");
  EXPECT(std::string_view(intrinsic_fbs::kVehicleLimitsTypeId) ==
         "intrinsic_fbs.VehicleLimits");
  EXPECT(std::string_view(TypeID<intrinsic_fbs::BodyState>::kTypeString) ==
         intrinsic_fbs::kBodyStateTypeId);
  EXPECT(std::string_view(TypeID<intrinsic_fbs::BodyWrench>::kTypeString) ==
         intrinsic_fbs::kBodyWrenchTypeId);
  EXPECT(std::string_view(TypeID<intrinsic_fbs::VehicleLimits>::kTypeString) ==
         intrinsic_fbs::kVehicleLimitsTypeId);
  EXPECT(std::string_view(intrinsic_fbs::kBodyStateTypeId) !=
         "intrinsic_fbs.JointPositionState");
  EXPECT(std::string_view(intrinsic_fbs::kBodyWrenchTypeId) !=
         "intrinsic_fbs.Wrench");
  EXPECT(std::string_view(intrinsic_fbs::kVehicleLimitsTypeId) !=
         "intrinsic_fbs.JointLimits");
}

}  // namespace

int main() {
  TestStructLayout();
  TestNeutralBuffersVerifyAndRepeat();
  TestPopulatedBodyState();
  TestAbsentPoseIsNotZeroPose();
  TestWrenchNeutralAndNan();
  TestVehicleLimitFlag();
  TestRejectedStringAndVerifierFailures();
  TestTypeIds();
  if (g_failures != 0) {
    std::cerr << g_failures << " verifier check(s) failed\n";
    return EXIT_FAILURE;
  }
  return EXIT_SUCCESS;
}
