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

#include <array>
#include <cstdlib>
#include <fstream>
#include <optional>
#include <sstream>
#include <string>
#include <string_view>
#include <vector>

#include "google/protobuf/descriptor.h"
#include "google/protobuf/text_format.h"
#include "google/protobuf/unknown_field_set.h"
#include "gtest/gtest.h"
#include "intrinsic/embodiment/frame_policy.h"
#include "intrinsic/embodiment/proto/stamped_header.pb.h"
#include "intrinsic/embodiment/stamped_header_policy.h"
#include "intrinsic/vehicle/proto/vehicle_command.pb.h"
#include "intrinsic/vehicle/proto/vehicle_state.pb.h"
#include "intrinsic/vehicle/vehicle_contract_policy.h"

namespace intrinsic::vehicle {
namespace {

using intrinsic_proto::embodiment::Validity;
using intrinsic_proto::vehicle::BodyWrench;
using intrinsic_proto::vehicle::DesiredMotion;
using intrinsic_proto::vehicle::NavigationMode;
using intrinsic_proto::vehicle::VehicleState;

// Canonical serializations of the fillers below. Keep in sync with
// vehicle_contract_serialization_test.py.
constexpr std::string_view kVehicleStateGoldenHex =
    "0a37082a120b0880e2cfaa061080e59a771a060881e2cfaa06220465736b662a09776f72"
    "6c645f656e7532096d6f6e6f746f6e69633a02080112280a1b09000000000000f03f1100"
    "000000000000401900000000000008c0120921000000000000f03f1a1209000000000000"
    "e03f31000000000000c03f220919000000000000d03f2aa3020aa002000000000000d03f"
    "000000000000000000000000000000000000000000000000000000000000000000000000"
    "000000000000000000000000000000000000d03f00000000000000000000000000000000"
    "000000000000000000000000000000000000000000000000000000000000000000000000"
    "0000d03f0000000000000000000000000000000000000000000000000000000000000000"
    "00000000000000000000000000000000000000000000b03f000000000000000000000000"
    "000000000000000000000000000000000000000000000000000000000000000000000000"
    "000000000000b03f00000000000000000000000000000000000000000000000000000000"
    "0000000000000000000000000000000000000000000000000000b03f380342090a036476"
    "6c1202080142070a0564657074684803";
constexpr std::string_view kDesiredMotionGoldenHex =
    "0a390807120b0882e2cfaa061080e59a771a060883e2cfaa062206706f6c6963792a0977"
    "6f726c645f656e7532096d6f6e6f746f6e69633a02080112210a1f0a1209000000000000"
    "10401900000000000000c0120921000000000000f03f2a02080531000000000000e03f3a"
    "1c0a0a7575765f696e74656e74120276311a0a7368613235363a616263";
constexpr std::string_view kBodyWrenchGoldenHex =
    "0a360809120b0883e2cfaa061080e59a771a060884e2cfaa06220867756964616e63652a"
    "04626f647932096d6f6e6f746f6e69633a02080111000000000000f83f39000000000000"
    "d03f";

std::array<double, kCovarianceValues> PoseCovariance() {
  std::array<double, kCovarianceValues> values = {};
  values[0] = 0.25;
  values[7] = 0.25;
  values[14] = 0.25;
  values[21] = 0.0625;
  values[28] = 0.0625;
  values[35] = 0.0625;
  return values;
}

std::string ToHex(std::string_view bytes) {
  static constexpr char kDigits[] = "0123456789abcdef";
  std::string out(bytes.size() * 2, '\0');
  for (size_t i = 0; i < bytes.size(); ++i) {
    const unsigned char value = static_cast<unsigned char>(bytes[i]);
    out[i * 2] = kDigits[value >> 4];
    out[i * 2 + 1] = kDigits[value & 0x0f];
  }
  return out;
}

std::string FromHex(std::string_view hex) {
  std::string out;
  out.reserve(hex.size() / 2);
  auto nibble = [](char c) -> int {
    if (c >= '0' && c <= '9') {
      return c - '0';
    }
    return c - 'a' + 10;
  };
  for (size_t i = 0; i < hex.size(); i += 2) {
    out.push_back(
        static_cast<char>((nibble(hex[i]) << 4) | nibble(hex[i + 1])));
  }
  return out;
}

void FillHeader(intrinsic_proto::embodiment::StampedHeader* header,
                uint64_t sequence, int64_t seconds, std::string_view source_id,
                std::string_view frame_id) {
  header->set_sequence(sequence);
  header->mutable_source_time()->set_seconds(seconds);
  header->mutable_source_time()->set_nanos(250000000);
  header->mutable_receive_time()->set_seconds(seconds + 1);
  header->set_source_id(std::string(source_id));
  header->set_frame_id(std::string(frame_id));
  header->set_clock_domain(std::string(embodiment::kClockDomainMonotonic));
  header->mutable_validity()->set_state(Validity::STATE_VALID);
}

void FillVehicleState(VehicleState* state) {
  FillHeader(state->mutable_header(), 42, 1700000000, "eskf",
             embodiment::kWorldEnuFrameId);
  auto* pose = state->mutable_pose_world_from_body();
  pose->mutable_position()->set_x(1);
  pose->mutable_position()->set_y(2);
  pose->mutable_position()->set_z(-3);
  pose->mutable_orientation()->set_w(1);
  auto* twist = state->mutable_body_twist();
  twist->set_linear_x_m_s(0.5);
  twist->set_angular_z_rad_s(0.125);
  state->mutable_body_acceleration()->set_linear_z_m_s2(0.25);
  for (double value : PoseCovariance()) {
    state->mutable_pose_covariance()->add_values(value);
  }
  state->set_mode(intrinsic_proto::vehicle::NAVIGATION_MODE_AIDED);
  auto* dvl = state->add_sources();
  dvl->set_source_id("dvl");
  dvl->mutable_validity()->set_state(Validity::STATE_VALID);
  state->add_sources()->set_source_id("depth");
  state->set_estimator_epoch(3);
}

void FillDesiredMotionPose(DesiredMotion* motion) {
  FillHeader(motion->mutable_header(), 7, 1700000002, "policy",
             embodiment::kWorldEnuFrameId);
  auto* pose = motion->mutable_pose()->mutable_pose_world_from_body();
  pose->mutable_position()->set_x(4);
  pose->mutable_position()->set_z(-2);
  pose->mutable_orientation()->set_w(1);
  motion->mutable_horizon()->set_seconds(5);
  motion->set_confidence(0.5);
  motion->mutable_provenance()->set_model_id("uuv_intent");
  motion->mutable_provenance()->set_model_version("v1");
  motion->mutable_provenance()->set_digest("sha256:abc");
}

void FillDesiredMotionTwist(DesiredMotion* motion) {
  FillHeader(motion->mutable_header(), 8, 1700000002, "policy", kBodyFrameId);
  motion->mutable_twist()->mutable_body_twist()->set_linear_x_m_s(0.5);
  motion->mutable_horizon()->set_seconds(1);
  motion->set_confidence(0.5);
}

void FillBodyWrench(BodyWrench* wrench) {
  FillHeader(wrench->mutable_header(), 9, 1700000003, "guidance", kBodyFrameId);
  wrench->set_force_x_n(1.5);
  wrench->set_torque_z_n_m(0.25);
}

std::vector<double> CopyDoubles(
    const google::protobuf::RepeatedField<double>& field) {
  return std::vector<double>(field.begin(), field.end());
}

VehicleStateView ViewOf(const VehicleState& state,
                        std::vector<double>* pose_covariance,
                        std::vector<double>* twist_covariance,
                        std::vector<SourceView>* sources) {
  VehicleStateView view;
  view.validity_present = state.header().has_validity();
  view.validity_state = state.header().validity().state();
  view.frame_id = state.header().frame_id();
  view.pose_present = state.has_pose_world_from_body();
  if (view.pose_present) {
    const auto& pose = state.pose_world_from_body();
    view.position = embodiment::Vec3{pose.position().x(), pose.position().y(),
                                     pose.position().z()};
    view.orientation =
        embodiment::Quaternion{pose.orientation().x(), pose.orientation().y(),
                               pose.orientation().z(), pose.orientation().w()};
  }
  view.twist_present = state.has_body_twist();
  const auto& twist = state.body_twist();
  view.twist = BodyVector{twist.linear_x_m_s(),    twist.linear_y_m_s(),
                          twist.linear_z_m_s(),    twist.angular_x_rad_s(),
                          twist.angular_y_rad_s(), twist.angular_z_rad_s()};
  view.acceleration_present = state.has_body_acceleration();
  const auto& acceleration = state.body_acceleration();
  view.acceleration = BodyVector{
      acceleration.linear_x_m_s2(),    acceleration.linear_y_m_s2(),
      acceleration.linear_z_m_s2(),    acceleration.angular_x_rad_s2(),
      acceleration.angular_y_rad_s2(), acceleration.angular_z_rad_s2()};
  view.pose_covariance_present = state.has_pose_covariance();
  if (view.pose_covariance_present) {
    *pose_covariance = CopyDoubles(state.pose_covariance().values());
    view.pose_covariance = *pose_covariance;
  }
  view.twist_covariance_present = state.has_twist_covariance();
  if (view.twist_covariance_present) {
    *twist_covariance = CopyDoubles(state.twist_covariance().values());
    view.twist_covariance = *twist_covariance;
  }
  sources->clear();
  for (const auto& source : state.sources()) {
    sources->push_back(SourceView{source.source_id(), source.has_validity(),
                                  source.validity().state()});
  }
  view.sources = *sources;
  return view;
}

DesiredMotionView ViewOf(const DesiredMotion& motion) {
  DesiredMotionView view;
  view.header_present = motion.has_header();
  view.validity_present = motion.header().has_validity();
  view.validity_state = motion.header().validity().state();
  view.frame_id = motion.header().frame_id();
  switch (motion.objective_case()) {
    case DesiredMotion::kPose: {
      view.objective = ObjectiveKind::kPose;
      const auto& pose = motion.pose().pose_world_from_body();
      view.position = embodiment::Vec3{pose.position().x(), pose.position().y(),
                                       pose.position().z()};
      view.orientation = embodiment::Quaternion{
          pose.orientation().x(), pose.orientation().y(),
          pose.orientation().z(), pose.orientation().w()};
      break;
    }
    case DesiredMotion::kTwist: {
      view.objective = ObjectiveKind::kTwist;
      const auto& twist = motion.twist().body_twist();
      view.twist = BodyVector{twist.linear_x_m_s(),    twist.linear_y_m_s(),
                              twist.linear_z_m_s(),    twist.angular_x_rad_s(),
                              twist.angular_y_rad_s(), twist.angular_z_rad_s()};
      break;
    }
    case DesiredMotion::kTrajectory:
      view.objective = ObjectiveKind::kTrajectory;
      view.trajectory_id = motion.trajectory().trajectory_id();
      break;
    case DesiredMotion::OBJECTIVE_NOT_SET:
      view.objective = ObjectiveKind::kAbsent;
      break;
  }
  view.confidence_present = motion.has_confidence();
  view.confidence = motion.confidence();
  view.horizon_present = motion.has_horizon();
  if (view.horizon_present) {
    view.horizon_seconds = motion.horizon().seconds();
    view.horizon_nanos = motion.horizon().nanos();
  }
  view.provenance_present = motion.has_provenance();
  view.model_id = motion.provenance().model_id();
  return view;
}

BodyWrenchView ViewOf(const BodyWrench& wrench) {
  BodyWrenchView view;
  view.engaged = wrench.ByteSizeLong() != 0;
  view.validity_present = wrench.header().has_validity();
  view.validity_state = wrench.header().validity().state();
  view.frame_id = wrench.header().frame_id();
  view.force_torque = BodyVector{wrench.force_x_n(),    wrench.force_y_n(),
                                 wrench.force_z_n(),    wrench.torque_x_n_m(),
                                 wrench.torque_y_n_m(), wrench.torque_z_n_m()};
  return view;
}

std::string LoadExample(const std::string& name) {
  const char* src = std::getenv("TEST_SRCDIR");
  if (src == nullptr) {
    return "";
  }
  const std::string suffix = "intrinsic/vehicle/proto/examples/" + name;
  const char* workspace = std::getenv("TEST_WORKSPACE");
  // Bzlmod canonical repo name is intrinsic_apis+. The apparent name
  // intrinsic_apis is kept for layouts that symlink it.
  std::vector<std::string> candidates = {
      std::string(src) + "/intrinsic_apis+/" + suffix,
      std::string(src) + "/intrinsic_apis/" + suffix,
  };
  if (workspace != nullptr) {
    candidates.push_back(std::string(src) + "/" + workspace +
                         "/external/intrinsic_apis+/" + suffix);
    candidates.push_back(std::string(src) + "/" + workspace +
                         "/external/intrinsic_apis/" + suffix);
    candidates.push_back(std::string(src) + "/" + workspace + "/" + suffix);
  }
  for (const std::string& path : candidates) {
    std::ifstream in(path);
    if (!in) {
      continue;
    }
    std::ostringstream buffer;
    buffer << in.rdbuf();
    return buffer.str();
  }
  return "";
}

TEST(VehicleContractSerializationTest, NoRobotTypeOnTheContracts) {
  EXPECT_EQ(VehicleState::descriptor()->file()->package(),
            "intrinsic_proto.vehicle");
  EXPECT_EQ(VehicleState::descriptor()->FindFieldByName("robot_type"), nullptr);
  EXPECT_EQ(VehicleState::descriptor()->FindFieldByName("embodiment"), nullptr);
  EXPECT_EQ(DesiredMotion::descriptor()->FindFieldByName("robot_type"),
            nullptr);
  EXPECT_EQ(BodyWrench::descriptor()->FindFieldByName("actuator_command"),
            nullptr);
  EXPECT_EQ(VehicleState::descriptor()->file()->FindEnumTypeByName("RobotType"),
            nullptr);
  EXPECT_EQ(
      DesiredMotion::descriptor()->file()->FindEnumTypeByName("RobotType"),
      nullptr);
  EXPECT_EQ(VehicleState::descriptor()->file()->FindMessageTypeByName("Uuv"),
            nullptr);
}

TEST(VehicleContractSerializationTest, DefaultsAreEmptyAndOptIn) {
  EXPECT_EQ(VehicleState().SerializeAsString(), "");
  EXPECT_EQ(DesiredMotion().SerializeAsString(), "");
  EXPECT_EQ(BodyWrench().SerializeAsString(), "");
  EXPECT_FALSE(VehicleState().has_pose_world_from_body());
  EXPECT_FALSE(VehicleState().has_body_twist());
  EXPECT_FALSE(VehicleState().has_pose_covariance());
  EXPECT_FALSE(DesiredMotion().has_confidence());
  EXPECT_FALSE(DesiredMotion().has_horizon());
  EXPECT_EQ(DesiredMotion().objective_case(), DesiredMotion::OBJECTIVE_NOT_SET);
}

TEST(VehicleContractSerializationTest, VehicleStateGoldenRoundTrip) {
  VehicleState state;
  FillVehicleState(&state);
  const std::string bytes = state.SerializeAsString();
  ASSERT_EQ(ToHex(bytes), kVehicleStateGoldenHex);
  const std::string golden = FromHex(kVehicleStateGoldenHex);
  VehicleState parsed;
  ASSERT_TRUE(parsed.ParseFromString(golden));
  EXPECT_EQ(parsed.SerializeAsString(), golden);
  EXPECT_EQ(parsed.header().frame_id(), embodiment::kWorldEnuFrameId);
  EXPECT_EQ(parsed.pose_world_from_body().orientation().w(), 1);
  EXPECT_EQ(parsed.body_twist().linear_x_m_s(), 0.5);
  EXPECT_EQ(parsed.body_twist().angular_z_rad_s(), 0.125);
  EXPECT_EQ(parsed.body_acceleration().linear_z_m_s2(), 0.25);
  EXPECT_EQ(parsed.pose_covariance().values_size(), kCovarianceValues);
  EXPECT_FALSE(parsed.has_twist_covariance());
  EXPECT_EQ(parsed.mode(), intrinsic_proto::vehicle::NAVIGATION_MODE_AIDED);
  ASSERT_EQ(parsed.sources_size(), 2);
  EXPECT_TRUE(parsed.sources(0).has_validity());
  EXPECT_FALSE(parsed.sources(1).has_validity());
  EXPECT_EQ(parsed.estimator_epoch(), 3u);

  std::vector<double> pose_covariance;
  std::vector<double> twist_covariance;
  std::vector<SourceView> sources;
  const ContractAssessment assessment = AssessVehicleState(
      ViewOf(parsed, &pose_covariance, &twist_covariance, &sources));
  EXPECT_TRUE(assessment.accepted);
  EXPECT_FALSE(CovarianceIsUnknown(
      AssessCovariance(parsed.has_pose_covariance(), pose_covariance)));
  EXPECT_TRUE(CovarianceIsUnknown(
      AssessCovariance(parsed.has_twist_covariance(), twist_covariance)));
}

TEST(VehicleContractSerializationTest, EpochClearIsAPrefixOfTheGolden) {
  VehicleState state;
  FillVehicleState(&state);
  const std::string full = state.SerializeAsString();
  state.clear_estimator_epoch();
  EXPECT_TRUE(full.starts_with(state.SerializeAsString()));
  EXPECT_NE(full, state.SerializeAsString());
}

TEST(VehicleContractSerializationTest, PresentZeroTwistIsNotAbsence) {
  VehicleState state;
  state.mutable_body_twist();
  EXPECT_TRUE(state.has_body_twist());
  EXPECT_GT(state.ByteSizeLong(), 0u);
  VehicleState parsed;
  ASSERT_TRUE(parsed.ParseFromString(state.SerializeAsString()));
  EXPECT_TRUE(parsed.has_body_twist());
  EXPECT_EQ(parsed.body_twist().linear_x_m_s(), 0);
}

TEST(VehicleContractSerializationTest, ZeroCovarianceIsNotTheUnknownEncoding) {
  VehicleState absent;
  VehicleState zeros;
  for (int i = 0; i < kCovarianceValues; ++i) {
    zeros.mutable_pose_covariance()->add_values(0);
  }
  EXPECT_FALSE(absent.has_pose_covariance());
  EXPECT_TRUE(zeros.has_pose_covariance());
  EXPECT_NE(absent.SerializeAsString(), zeros.SerializeAsString());
  EXPECT_TRUE(
      IsAllZeroCovariance(CopyDoubles(zeros.pose_covariance().values())));
  EXPECT_FALSE(CovarianceIsUnknown(
      AssessCovariance(true, CopyDoubles(zeros.pose_covariance().values()))));
}

TEST(VehicleContractSerializationTest, UnknownFieldAndUnknownModeAreKept) {
  VehicleState state;
  FillVehicleState(&state);
  const std::string golden = state.SerializeAsString();
  const std::string with_unknown = golden + std::string("\xA0\x06\x07", 3);
  VehicleState parsed;
  ASSERT_TRUE(parsed.ParseFromString(with_unknown));
  EXPECT_EQ(parsed.estimator_epoch(), 3u);
  const google::protobuf::UnknownFieldSet& unknown =
      parsed.GetReflection()->GetUnknownFields(parsed);
  ASSERT_EQ(unknown.field_count(), 1);
  EXPECT_EQ(unknown.field(0).number(), 100);
  EXPECT_EQ(unknown.field(0).varint(), 7u);
  EXPECT_EQ(parsed.SerializeAsString(), with_unknown);

  parsed.set_mode(static_cast<NavigationMode>(100));
  const std::string mode_bytes = parsed.SerializeAsString();
  VehicleState reparsed;
  ASSERT_TRUE(reparsed.ParseFromString(mode_bytes));
  EXPECT_EQ(static_cast<int>(reparsed.mode()), 100);
  EXPECT_EQ(ClassifyNavigationMode(static_cast<int>(reparsed.mode())),
            NavigationModeKind::kUnknown);
  EXPECT_NE(ClassifyNavigationMode(static_cast<int>(reparsed.mode())),
            NavigationModeKind::kFaulted);
}

TEST(VehicleContractSerializationTest, DesiredMotionAndWrenchGoldens) {
  DesiredMotion motion;
  FillDesiredMotionPose(&motion);
  ASSERT_EQ(ToHex(motion.SerializeAsString()), kDesiredMotionGoldenHex);
  DesiredMotion parsed_motion;
  ASSERT_TRUE(parsed_motion.ParseFromString(FromHex(kDesiredMotionGoldenHex)));
  EXPECT_EQ(parsed_motion.objective_case(), DesiredMotion::kPose);
  EXPECT_TRUE(parsed_motion.has_confidence());
  EXPECT_EQ(parsed_motion.confidence(), 0.5);
  EXPECT_EQ(parsed_motion.provenance().model_id(), "uuv_intent");
  EXPECT_TRUE(AssessDesiredMotion(ViewOf(parsed_motion)).accepted);

  BodyWrench wrench;
  FillBodyWrench(&wrench);
  ASSERT_EQ(ToHex(wrench.SerializeAsString()), kBodyWrenchGoldenHex);
  BodyWrench parsed_wrench;
  ASSERT_TRUE(parsed_wrench.ParseFromString(FromHex(kBodyWrenchGoldenHex)));
  EXPECT_EQ(parsed_wrench.header().frame_id(), kBodyFrameId);
  EXPECT_EQ(parsed_wrench.force_x_n(), 1.5);
  EXPECT_EQ(parsed_wrench.torque_z_n_m(), 0.25);
  EXPECT_TRUE(AssessBodyWrench(ViewOf(parsed_wrench)).accepted);
}

TEST(VehicleContractSerializationTest, OneofKeepsTheLastObjective) {
  DesiredMotion motion;
  motion.mutable_pose()
      ->mutable_pose_world_from_body()
      ->mutable_orientation()
      ->set_w(1);
  motion.mutable_twist()->mutable_body_twist()->set_linear_x_m_s(0.5);
  EXPECT_EQ(motion.objective_case(), DesiredMotion::kTwist);
  EXPECT_FALSE(motion.has_pose());
  DesiredMotion parsed;
  ASSERT_TRUE(parsed.ParseFromString(motion.SerializeAsString()));
  EXPECT_FALSE(parsed.has_pose());
  EXPECT_TRUE(parsed.has_twist());
}

TEST(VehicleContractSerializationTest, ZeroConfidenceIsPresent) {
  DesiredMotion motion;
  EXPECT_FALSE(motion.has_confidence());
  motion.set_confidence(0);
  EXPECT_TRUE(motion.has_confidence());
  const std::string with_zero = motion.SerializeAsString();
  motion.clear_confidence();
  EXPECT_FALSE(motion.has_confidence());
  EXPECT_NE(motion.SerializeAsString(), with_zero);
}

TEST(VehicleContractSerializationTest, StampUsesTheExistingTimePolicy) {
  VehicleState state;
  FillVehicleState(&state);
  const std::optional<double> age = embodiment::MonotonicAgeSeconds(
      embodiment::ClockReading{state.header().source_time().seconds(),
                               state.header().source_time().nanos()},
      embodiment::ClockReading{state.header().receive_time().seconds(),
                               state.header().receive_time().nanos()},
      state.header().clock_domain());
  ASSERT_TRUE(age.has_value());
  EXPECT_DOUBLE_EQ(*age, 0.75);
  EXPECT_FALSE(
      embodiment::MonotonicAgeSeconds(
          embodiment::ClockReading{state.header().source_time().seconds(),
                                   state.header().source_time().nanos()},
          embodiment::ClockReading{state.header().receive_time().seconds(),
                                   state.header().receive_time().nanos()},
          embodiment::kClockDomainUtc)
          .has_value());
  EXPECT_TRUE(embodiment::SequenceAdvances(2, 3));
  EXPECT_FALSE(embodiment::SequenceAdvances(3, 3));
  EXPECT_TRUE(embodiment::FrameIdMatches(state.header().frame_id(),
                                         embodiment::WorldFrame::kEnu));
}

TEST(VehicleContractSerializationTest, TextprotoExamplesMatchTheFixtures) {
  const std::string state_text = LoadExample("vehicle_state.textproto");
  ASSERT_FALSE(state_text.empty()) << "vehicle_state.textproto";
  VehicleState parsed_state;
  ASSERT_TRUE(
      google::protobuf::TextFormat::ParseFromString(state_text, &parsed_state));
  VehicleState coded_state;
  FillVehicleState(&coded_state);
  EXPECT_EQ(parsed_state.SerializeAsString(), coded_state.SerializeAsString());

  const std::string unknown_text =
      LoadExample("vehicle_state_unknown_covariance.textproto");
  ASSERT_FALSE(unknown_text.empty());
  VehicleState unknown_state;
  ASSERT_TRUE(google::protobuf::TextFormat::ParseFromString(unknown_text,
                                                            &unknown_state));
  EXPECT_FALSE(unknown_state.has_pose_covariance());
  EXPECT_FALSE(unknown_state.has_twist_covariance());
  std::vector<double> pose_covariance;
  std::vector<double> twist_covariance;
  std::vector<SourceView> sources;
  const ContractAssessment unknown_assessment = AssessVehicleState(
      ViewOf(unknown_state, &pose_covariance, &twist_covariance, &sources));
  EXPECT_TRUE(unknown_assessment.accepted);
  EXPECT_TRUE(CovarianceIsUnknown(AssessCovariance(false, {})));

  const std::string pose_text = LoadExample("desired_motion_pose.textproto");
  ASSERT_FALSE(pose_text.empty());
  DesiredMotion parsed_pose;
  ASSERT_TRUE(
      google::protobuf::TextFormat::ParseFromString(pose_text, &parsed_pose));
  DesiredMotion coded_pose;
  FillDesiredMotionPose(&coded_pose);
  EXPECT_EQ(parsed_pose.SerializeAsString(), coded_pose.SerializeAsString());

  const std::string twist_text = LoadExample("desired_motion_twist.textproto");
  ASSERT_FALSE(twist_text.empty());
  DesiredMotion parsed_twist;
  ASSERT_TRUE(
      google::protobuf::TextFormat::ParseFromString(twist_text, &parsed_twist));
  DesiredMotion coded_twist;
  FillDesiredMotionTwist(&coded_twist);
  EXPECT_EQ(parsed_twist.SerializeAsString(), coded_twist.SerializeAsString());
  EXPECT_EQ(parsed_twist.header().frame_id(), kBodyFrameId);
  EXPECT_TRUE(AssessDesiredMotion(ViewOf(parsed_twist)).accepted);

  const std::string wrench_text = LoadExample("body_wrench.textproto");
  ASSERT_FALSE(wrench_text.empty());
  BodyWrench parsed_wrench;
  ASSERT_TRUE(google::protobuf::TextFormat::ParseFromString(wrench_text,
                                                            &parsed_wrench));
  BodyWrench coded_wrench;
  FillBodyWrench(&coded_wrench);
  EXPECT_EQ(parsed_wrench.SerializeAsString(),
            coded_wrench.SerializeAsString());
}

}  // namespace
}  // namespace intrinsic::vehicle
