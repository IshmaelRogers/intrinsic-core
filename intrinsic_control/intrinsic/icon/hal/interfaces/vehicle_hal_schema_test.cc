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

#include <cmath>
#include <cstdint>
#include <limits>
#include <string>

#include "flatbuffers/buffer.h"
#include "flatbuffers/flatbuffer_builder.h"
#include "flatbuffers/verifier.h"
#include "gtest/gtest.h"
#include "intrinsic/icon/flatbuffers/transform_types.fbs.h"
#include "intrinsic/icon/hal/hardware_interface_traits.h"
#include "intrinsic/icon/hal/interfaces/body_state.fbs.h"
#include "intrinsic/icon/hal/interfaces/body_state_utils.h"
#include "intrinsic/icon/hal/interfaces/body_wrench.fbs.h"
#include "intrinsic/icon/hal/interfaces/body_wrench_utils.h"
#include "intrinsic/icon/hal/interfaces/vehicle_hal_types.fbs.h"
#include "intrinsic/icon/hal/interfaces/vehicle_hal_types_utils.h"
#include "intrinsic/icon/hal/interfaces/vehicle_hardware_interfaces.h"
#include "intrinsic/icon/hal/interfaces/vehicle_limits.fbs.h"
#include "intrinsic/icon/hal/interfaces/vehicle_limits_utils.h"
#include "intrinsic/icon/proto/v1/types.pb.h"
#include "intrinsic/icon/server/signature_utils.h"

namespace intrinsic_fbs {
namespace {

using intrinsic::icon::PartCompatibleWithSlot;
using intrinsic::icon::hardware_interface_traits::BuilderFunctions;
using intrinsic::icon::hardware_interface_traits::TypeID;
using intrinsic_proto::icon::v1::ActionSignature;
using intrinsic_proto::icon::v1::FeatureInterfaceTypes;
using intrinsic_proto::icon::v1::FeatureInterfaceTypes_IsValid;
using intrinsic_proto::icon::v1::PartConfig;

template <typename T>
bool Verifies(const uint8_t* data, size_t size) {
  flatbuffers::Verifier verifier(data, size);
  return verifier.VerifyBuffer<T>();
}

template <typename T>
bool Verifies(const flatbuffers::DetachedBuffer& buffer) {
  return Verifies<T>(buffer.data(), buffer.size());
}

bool FiniteTwist(const Twist& twist) {
  return std::isfinite(twist.x()) && std::isfinite(twist.y()) &&
         std::isfinite(twist.z()) && std::isfinite(twist.rx()) &&
         std::isfinite(twist.ry()) && std::isfinite(twist.rz());
}

Matrix6d ZeroMatrix() {
  Matrix6d matrix;
  for (int i = 0; i < kMatrix6dValues; ++i) {
    matrix.mutable_values()->Mutate(i, 0.0);
  }
  return matrix;
}

TEST(VehicleHalSchemaTest, DefaultBodyStateVerifiesAndIsAbsent) {
  const flatbuffers::DetachedBuffer buffer = BuildBodyState();
  ASSERT_TRUE(Verifies<BodyState>(buffer));
  const BodyState* state = flatbuffers::GetRoot<BodyState>(buffer.data());
  ASSERT_NE(state, nullptr);
  EXPECT_EQ(state->sequence(), 0u);
  EXPECT_FALSE(state->source_time_present());
  EXPECT_FALSE(state->receive_time_present());
  EXPECT_EQ(state->source_id(), nullptr);
  EXPECT_EQ(state->frame_id(), nullptr);
  EXPECT_EQ(state->clock_domain(), nullptr);
  EXPECT_FALSE(state->validity_present());
  EXPECT_EQ(state->validity_state(), 0);
  EXPECT_EQ(state->pose_world_from_body(), nullptr);
  EXPECT_EQ(state->body_twist(), nullptr);
  EXPECT_EQ(state->body_acceleration(), nullptr);
  EXPECT_EQ(state->pose_covariance(), nullptr);
  EXPECT_EQ(state->twist_covariance(), nullptr);
  EXPECT_EQ(state->navigation_mode(), 0);
  EXPECT_EQ(state->estimator_epoch(), 0u);
}

TEST(VehicleHalSchemaTest, OmittedScalarsStillReadAsDefaults) {
  flatbuffers::FlatBufferBuilder builder;
  builder.Finish(CreateBodyState(builder));
  const flatbuffers::DetachedBuffer buffer = builder.Release();
  ASSERT_TRUE(Verifies<BodyState>(buffer));
  const BodyState* state = flatbuffers::GetRoot<BodyState>(buffer.data());
  EXPECT_EQ(state->sequence(), 0u);
  EXPECT_FALSE(state->validity_present());
  EXPECT_EQ(state->body_twist(), nullptr);
  EXPECT_EQ(state->pose_covariance(), nullptr);
}

TEST(VehicleHalSchemaTest, TruncatedBodyStateFailsVerifier) {
  const flatbuffers::DetachedBuffer buffer = BuildBodyState();
  ASSERT_GT(buffer.size(), 1u);
  EXPECT_FALSE(Verifies<BodyState>(buffer.data(), 1));
}

TEST(VehicleHalSchemaTest, PresentZeroCovarianceIsNotUnknown) {
  const Matrix6d zeros = ZeroMatrix();
  flatbuffers::FlatBufferBuilder builder;
  builder.ForceDefaults(true);
  BodyStateBuilder state_builder(builder);
  state_builder.add_pose_covariance(&zeros);
  builder.Finish(state_builder.Finish());
  const flatbuffers::DetachedBuffer buffer = builder.Release();
  ASSERT_TRUE(Verifies<BodyState>(buffer));
  const BodyState* state = flatbuffers::GetRoot<BodyState>(buffer.data());
  ASSERT_NE(state->pose_covariance(), nullptr);
  EXPECT_EQ(state->twist_covariance(), nullptr);
  EXPECT_EQ(state->pose_covariance()->values()->Get(0), 0.0);
  EXPECT_EQ(state->pose_covariance()->values()->size(),
            static_cast<size_t>(kMatrix6dValues));
  const flatbuffers::DetachedBuffer absent_buffer = BuildBodyState();
  const BodyState* absent =
      flatbuffers::GetRoot<BodyState>(absent_buffer.data());
  EXPECT_EQ(absent->pose_covariance(), nullptr);
}

TEST(VehicleHalSchemaTest, NonFiniteTwistIsNotRepairedByValidState) {
  const Twist twist(std::numeric_limits<double>::quiet_NaN(), 0.0, 0.0, 0.0,
                    0.0, 0.0);
  flatbuffers::FlatBufferBuilder builder;
  builder.ForceDefaults(true);
  BodyStateBuilder state_builder(builder);
  state_builder.add_validity_present(true);
  state_builder.add_validity_state(1);
  state_builder.add_body_twist(&twist);
  builder.Finish(state_builder.Finish());
  const flatbuffers::DetachedBuffer buffer = builder.Release();
  ASSERT_TRUE(Verifies<BodyState>(buffer));
  const BodyState* state = flatbuffers::GetRoot<BodyState>(buffer.data());
  ASSERT_NE(state->body_twist(), nullptr);
  EXPECT_FALSE(FiniteTwist(*state->body_twist()));
  EXPECT_TRUE(state->validity_present());
  EXPECT_EQ(state->validity_state(), 1);
}

TEST(VehicleHalSchemaTest, QuaternionIsStoredWithoutRenormalizing) {
  const Rotation rotation(0.0, 0.0, 0.0, 2.0);
  const Transform pose(Point(1.0, 2.0, 3.0), rotation);
  flatbuffers::FlatBufferBuilder builder;
  BodyStateBuilder state_builder(builder);
  state_builder.add_pose_world_from_body(&pose);
  builder.Finish(state_builder.Finish());
  const flatbuffers::DetachedBuffer buffer = builder.Release();
  ASSERT_TRUE(Verifies<BodyState>(buffer));
  const BodyState* state = flatbuffers::GetRoot<BodyState>(buffer.data());
  ASSERT_NE(state->pose_world_from_body(), nullptr);
  EXPECT_EQ(state->pose_world_from_body()->rotation().qw(), 2.0);
  EXPECT_EQ(state->pose_world_from_body()->position().x(), 1.0);
}

TEST(VehicleHalSchemaTest, FrameIdIsNotInferredFromTheTable) {
  FixedText256 frame;
  ASSERT_TRUE(SetFixedText(&frame, "world_enu").ok());
  const Twist twist(0.1, 0.0, 0.0, 0.0, 0.0, 0.0);
  flatbuffers::FlatBufferBuilder builder;
  BodyStateBuilder state_builder(builder);
  state_builder.add_frame_id(&frame);
  state_builder.add_body_twist(&twist);
  builder.Finish(state_builder.Finish());
  const flatbuffers::DetachedBuffer buffer = builder.Release();
  ASSERT_TRUE(Verifies<BodyState>(buffer));
  const BodyState* state = flatbuffers::GetRoot<BodyState>(buffer.data());
  ASSERT_NE(state->frame_id(), nullptr);
  EXPECT_EQ(ReadFixedText(*state->frame_id()), "world_enu");
  ASSERT_NE(state->body_twist(), nullptr);
  EXPECT_EQ(state->body_twist()->x(), 0.1);

  flatbuffers::FlatBufferBuilder wrench_builder;
  BodyWrenchBuilder command(wrench_builder);
  const Wrench neutral(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
  command.add_wrench(&neutral);
  wrench_builder.Finish(command.Finish());
  const flatbuffers::DetachedBuffer wrench_buffer = wrench_builder.Release();
  ASSERT_TRUE(Verifies<BodyWrench>(wrench_buffer));
  const BodyWrench* wrench =
      flatbuffers::GetRoot<BodyWrench>(wrench_buffer.data());
  ASSERT_NE(wrench->wrench(), nullptr);
  EXPECT_EQ(wrench->frame_id(), nullptr);
  EXPECT_EQ(wrench->wrench()->x(), 0.0);
}

TEST(VehicleHalSchemaTest, FixedTextRejectsOverflowAndKeepsEmptyPresent) {
  EXPECT_FALSE(SetFixedText(nullptr, "body").ok());
  FixedText256 text;
  EXPECT_FALSE(
      SetFixedText(&text, std::string(kFixedText256Capacity, 'a')).ok());
  ASSERT_TRUE(
      SetFixedText(&text, std::string(kFixedText256MaxChars, 'b')).ok());
  EXPECT_EQ(ReadFixedText(text).size(),
            static_cast<size_t>(kFixedText256MaxChars));

  FixedText256 empty;
  ASSERT_TRUE(SetFixedText(&empty, "").ok());
  flatbuffers::FlatBufferBuilder builder;
  BodyStateBuilder state_builder(builder);
  state_builder.add_source_id(&empty);
  builder.Finish(state_builder.Finish());
  const flatbuffers::DetachedBuffer buffer = builder.Release();
  const BodyState* state = flatbuffers::GetRoot<BodyState>(buffer.data());
  ASSERT_NE(state->source_id(), nullptr);
  EXPECT_TRUE(ReadFixedText(*state->source_id()).empty());
  const flatbuffers::DetachedBuffer absent_buffer = BuildBodyState();
  EXPECT_EQ(flatbuffers::GetRoot<BodyState>(absent_buffer.data())->source_id(),
            nullptr);

  FixedText256 domain;
  ASSERT_TRUE(SetFixedText(&domain, "utc").ok());
  EXPECT_EQ(ReadFixedText(domain), "utc");
  FixedText256 unknown_domain;
  ASSERT_TRUE(SetFixedText(&unknown_domain, "ship").ok());
  EXPECT_EQ(ReadFixedText(unknown_domain), "ship");
}

TEST(VehicleHalSchemaTest, DefaultBodyWrenchIsNotACommand) {
  const flatbuffers::DetachedBuffer buffer = BuildBodyWrench();
  ASSERT_TRUE(Verifies<BodyWrench>(buffer));
  const BodyWrench* wrench = flatbuffers::GetRoot<BodyWrench>(buffer.data());
  EXPECT_EQ(wrench->wrench(), nullptr);
  EXPECT_EQ(wrench->frame_id(), nullptr);
  EXPECT_FALSE(wrench->validity_present());
  EXPECT_EQ(wrench->sequence(), 0u);
}

TEST(VehicleHalSchemaTest, PresentZeroWrenchIsNeutral) {
  const Wrench neutral(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
  FixedText256 frame;
  ASSERT_TRUE(SetFixedText(&frame, "body").ok());
  flatbuffers::FlatBufferBuilder builder;
  builder.ForceDefaults(true);
  BodyWrenchBuilder wrench_builder(builder);
  wrench_builder.add_frame_id(&frame);
  wrench_builder.add_wrench(&neutral);
  wrench_builder.add_validity_present(true);
  wrench_builder.add_validity_state(1);
  builder.Finish(wrench_builder.Finish());
  const flatbuffers::DetachedBuffer buffer = builder.Release();
  ASSERT_TRUE(Verifies<BodyWrench>(buffer));
  const BodyWrench* wrench = flatbuffers::GetRoot<BodyWrench>(buffer.data());
  ASSERT_NE(wrench->wrench(), nullptr);
  EXPECT_EQ(wrench->wrench()->x(), 0.0);
  EXPECT_EQ(wrench->wrench()->rz(), 0.0);
  ASSERT_NE(wrench->frame_id(), nullptr);
  EXPECT_EQ(ReadFixedText(*wrench->frame_id()), "body");
  const flatbuffers::DetachedBuffer absent = BuildBodyWrench();
  EXPECT_EQ(flatbuffers::GetRoot<BodyWrench>(absent.data())->wrench(), nullptr);
  EXPECT_NE(buffer.size(), absent.size());
}

TEST(VehicleHalSchemaTest, DefaultVehicleLimitsAreAbsent) {
  const flatbuffers::DetachedBuffer buffer = BuildVehicleLimits();
  ASSERT_TRUE(Verifies<VehicleLimits>(buffer));
  const VehicleLimits* limits =
      flatbuffers::GetRoot<VehicleLimits>(buffer.data());
  EXPECT_FALSE(limits->has_linear_speed_limit());
  EXPECT_EQ(limits->max_linear_speed_m_s(), nullptr);
  EXPECT_FALSE(limits->has_angular_speed_limit());
  EXPECT_EQ(limits->max_angular_speed_rad_s(), nullptr);
  EXPECT_FALSE(limits->has_linear_acceleration_limit());
  EXPECT_FALSE(limits->has_angular_acceleration_limit());
  EXPECT_FALSE(limits->has_force_limit());
  EXPECT_FALSE(limits->has_torque_limit());
  EXPECT_EQ(limits->max_force_n(), nullptr);
  EXPECT_EQ(limits->max_torque_n_m(), nullptr);
}

TEST(VehicleHalSchemaTest, ZeroSpeedLimitIsConfigured) {
  const Point zeros(0.0, 0.0, 0.0);
  flatbuffers::FlatBufferBuilder builder;
  builder.ForceDefaults(true);
  VehicleLimitsBuilder limits_builder(builder);
  limits_builder.add_has_linear_speed_limit(true);
  limits_builder.add_max_linear_speed_m_s(&zeros);
  builder.Finish(limits_builder.Finish());
  const flatbuffers::DetachedBuffer buffer = builder.Release();
  ASSERT_TRUE(Verifies<VehicleLimits>(buffer));
  const VehicleLimits* limits =
      flatbuffers::GetRoot<VehicleLimits>(buffer.data());
  EXPECT_TRUE(limits->has_linear_speed_limit());
  ASSERT_NE(limits->max_linear_speed_m_s(), nullptr);
  EXPECT_EQ(limits->max_linear_speed_m_s()->x(), 0.0);
  EXPECT_FALSE(limits->has_force_limit());
  EXPECT_FALSE(Verifies<VehicleLimits>(buffer.data(), 1));
}

TEST(VehicleHalSchemaTest, TypeIdsMatchTheBuffers) {
  static_assert(BuilderFunctions<BodyState>::value);
  static_assert(BuilderFunctions<BodyWrench>::value);
  static_assert(BuilderFunctions<VehicleLimits>::value);
  EXPECT_STREQ(TypeID<BodyState>::kTypeString, "intrinsic_fbs.BodyState");
  EXPECT_STREQ(TypeID<BodyWrench>::kTypeString, "intrinsic_fbs.BodyWrench");
  EXPECT_STREQ(TypeID<VehicleLimits>::kTypeString,
               "intrinsic_fbs.VehicleLimits");
  EXPECT_TRUE(Verifies<BodyState>(BuilderFunctions<BodyState>::kBuild()));
  EXPECT_TRUE(Verifies<BodyWrench>(BuilderFunctions<BodyWrench>::kBuild()));
  EXPECT_TRUE(
      Verifies<VehicleLimits>(BuilderFunctions<VehicleLimits>::kBuild()));
}

TEST(VehicleHalSchemaTest, FeatureIdsAreAppendedAfterJointAcceleration) {
  EXPECT_EQ(FeatureInterfaceTypes::FEATURE_INTERFACE_JOINT_ACCELERATION, 27);
  EXPECT_EQ(FeatureInterfaceTypes::FEATURE_INTERFACE_HOMING, 26);
  EXPECT_EQ(FeatureInterfaceTypes::FEATURE_INTERFACE_BODY_STATE, 28);
  EXPECT_EQ(FeatureInterfaceTypes::FEATURE_INTERFACE_BODY_WRENCH_COMMAND, 29);
  EXPECT_EQ(FeatureInterfaceTypes::FEATURE_INTERFACE_VEHICLE_LIMITS, 30);
  EXPECT_TRUE(FeatureInterfaceTypes_IsValid(
      FeatureInterfaceTypes::FEATURE_INTERFACE_BODY_STATE));
  EXPECT_TRUE(FeatureInterfaceTypes_IsValid(
      FeatureInterfaceTypes::FEATURE_INTERFACE_BODY_WRENCH_COMMAND));
  EXPECT_TRUE(FeatureInterfaceTypes_IsValid(
      FeatureInterfaceTypes::FEATURE_INTERFACE_VEHICLE_LIMITS));
  EXPECT_FALSE(FeatureInterfaceTypes_IsValid(31));
  EXPECT_FALSE(FeatureInterfaceTypes_IsValid(1000));
}

PartConfig ArmPart() {
  PartConfig part;
  part.set_name("arm");
  part.add_feature_interfaces(
      FeatureInterfaceTypes::FEATURE_INTERFACE_JOINT_POSITION);
  return part;
}

ActionSignature::PartSlotInfo JointSlot() {
  ActionSignature::PartSlotInfo slot;
  slot.add_required_feature_interfaces(
      FeatureInterfaceTypes::FEATURE_INTERFACE_JOINT_POSITION);
  return slot;
}

ActionSignature::PartSlotInfo BodyStateSlot() {
  ActionSignature::PartSlotInfo slot;
  slot.add_required_feature_interfaces(
      FeatureInterfaceTypes::FEATURE_INTERFACE_BODY_STATE);
  return slot;
}

TEST(VehicleHalSchemaTest, UnknownFeatureIdDoesNotBreakJointSlot) {
  PartConfig part = ArmPart();
  part.add_feature_interfaces(
      FeatureInterfaceTypes::FEATURE_INTERFACE_BODY_STATE);
  part.add_feature_interfaces(
      FeatureInterfaceTypes::FEATURE_INTERFACE_BODY_WRENCH_COMMAND);
  part.add_feature_interfaces(
      FeatureInterfaceTypes::FEATURE_INTERFACE_VEHICLE_LIMITS);
  part.add_feature_interfaces(static_cast<FeatureInterfaceTypes>(1000));
  EXPECT_TRUE(PartCompatibleWithSlot(part, JointSlot()).Compatible());

  const std::string bytes = part.SerializeAsString();
  PartConfig parsed;
  ASSERT_TRUE(parsed.ParseFromString(bytes));
  ASSERT_EQ(parsed.feature_interfaces_size(), 5);
  EXPECT_EQ(parsed.feature_interfaces(0),
            FeatureInterfaceTypes::FEATURE_INTERFACE_JOINT_POSITION);
  EXPECT_EQ(static_cast<int>(parsed.feature_interfaces(4)), 1000);
  EXPECT_FALSE(FeatureInterfaceTypes_IsValid(parsed.feature_interfaces(4)));
  EXPECT_TRUE(PartCompatibleWithSlot(parsed, JointSlot()).Compatible());
}

TEST(VehicleHalSchemaTest, AbsentVehicleFeatureLeavesJointSlotCompatible) {
  const PartConfig arm = ArmPart();
  const auto missing = PartCompatibleWithSlot(arm, BodyStateSlot());
  EXPECT_FALSE(missing.Compatible());
  bool names_body_state = false;
  for (const std::string& name : missing.missing_required_interfaces) {
    if (name == "FEATURE_INTERFACE_BODY_STATE") {
      names_body_state = true;
    }
  }
  EXPECT_TRUE(names_body_state);
  EXPECT_TRUE(PartCompatibleWithSlot(arm, JointSlot()).Compatible());
}

}  // namespace
}  // namespace intrinsic_fbs
