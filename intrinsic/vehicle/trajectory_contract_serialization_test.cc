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

#include <cstdlib>
#include <fstream>
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
#include "intrinsic/vehicle/proto/vehicle_trajectory.pb.h"
#include "intrinsic/vehicle/trajectory_contract_policy.h"

namespace intrinsic::vehicle {
namespace {

using intrinsic_proto::embodiment::Validity;
using intrinsic_proto::vehicle::VehicleTrajectory;

// Canonical serialization of FillTwoSample. Keep in sync with
// trajectory_contract_serialization_test.py.
constexpr std::string_view kVehicleTrajectoryGoldenHex =
    "0a3a080b120b088ae2cfaa061080e59a771a06088be2cfaa062207706c616e6e65722a09776f"
    "726c645f656e7532096d6f6e6f746f6e69633a020801120a7472616a5f616c7068611a460a06"
    "088ae2cfaa0612280a1b09000000000000f03f1100000000000000401900000000000008c012"
    "0921000000000000f03f1a1209000000000000e03f31000000000000c03f1a430a0c088be2cf"
    "aa061080cab5ee0112280a1b09000000000000f83f1100000000000000401900000000000008"
    "c0120921000000000000f03f1a0909000000000000e03f421e0a0b7575765f706c616e6e6572"
    "120276311a0b7368613235363a7472616a";

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

void FillSample(intrinsic_proto::vehicle::TrajectorySample* sample,
                int64_t seconds, int32_t nanos, double x, bool angular_z) {
  sample->mutable_time()->set_seconds(seconds);
  if (nanos != 0) {
    sample->mutable_time()->set_nanos(nanos);
  }
  auto* pose = sample->mutable_pose_world_from_body();
  pose->mutable_position()->set_x(x);
  pose->mutable_position()->set_y(2);
  pose->mutable_position()->set_z(-3);
  pose->mutable_orientation()->set_w(1);
  sample->mutable_body_twist()->set_linear_x_m_s(0.5);
  if (angular_z) {
    sample->mutable_body_twist()->set_angular_z_rad_s(0.125);
  }
}

void FillTwoSample(VehicleTrajectory* trajectory) {
  FillHeader(trajectory->mutable_header(), 11, 1700000010, "planner",
             embodiment::kWorldEnuFrameId);
  trajectory->set_trajectory_id("traj_alpha");
  FillSample(trajectory->add_samples(), 1700000010, 0, 1, true);
  FillSample(trajectory->add_samples(), 1700000011, 500000000, 1.5, false);
  trajectory->mutable_provenance()->set_model_id("uuv_planner");
  trajectory->mutable_provenance()->set_model_version("v1");
  trajectory->mutable_provenance()->set_digest("sha256:traj");
}

struct TrajectoryStorage {
  std::vector<TrajectorySampleView> samples;
  std::vector<double> uncertainty;
};

VehicleTrajectoryView ViewOf(const VehicleTrajectory& message,
                             TrajectoryStorage* storage) {
  storage->samples.clear();
  storage->samples.reserve(message.samples_size());
  for (const auto& sample : message.samples()) {
    TrajectorySampleView view;
    view.time_present = sample.has_time();
    view.seconds = sample.time().seconds();
    view.nanos = sample.time().nanos();
    const auto& pose = sample.pose_world_from_body();
    view.position = embodiment::Vec3{pose.position().x(), pose.position().y(),
                                     pose.position().z()};
    view.orientation =
        embodiment::Quaternion{pose.orientation().x(), pose.orientation().y(),
                               pose.orientation().z(), pose.orientation().w()};
    view.twist_present = sample.has_body_twist();
    const auto& twist = sample.body_twist();
    view.twist = BodyVector{twist.linear_x_m_s(),    twist.linear_y_m_s(),
                            twist.linear_z_m_s(),    twist.angular_x_rad_s(),
                            twist.angular_y_rad_s(), twist.angular_z_rad_s()};
    view.acceleration_present = sample.has_body_acceleration();
    const auto& acceleration = sample.body_acceleration();
    view.acceleration = BodyVector{
        acceleration.linear_x_m_s2(),    acceleration.linear_y_m_s2(),
        acceleration.linear_z_m_s2(),    acceleration.angular_x_rad_s2(),
        acceleration.angular_y_rad_s2(), acceleration.angular_z_rad_s2()};
    storage->samples.push_back(view);
  }
  VehicleTrajectoryView view;
  view.header_present = message.has_header();
  view.validity_present = message.header().has_validity();
  view.validity_state = message.header().validity().state();
  view.frame_id = message.header().frame_id();
  view.trajectory_id = message.trajectory_id();
  view.samples = storage->samples;
  view.tolerances_present = message.has_tolerances();
  if (view.tolerances_present) {
    view.position_tolerance_m = message.tolerances().position_m();
    view.orientation_tolerance_rad = message.tolerances().orientation_rad();
    view.linear_velocity_tolerance_m_s =
        message.tolerances().linear_velocity_m_s();
    view.angular_velocity_tolerance_rad_s =
        message.tolerances().angular_velocity_rad_s();
  }
  view.cost_present = message.has_cost();
  view.cost = message.cost();
  view.risk_present = message.has_risk();
  view.risk = message.risk();
  view.uncertainty_present = message.has_uncertainty();
  if (view.uncertainty_present) {
    storage->uncertainty.assign(message.uncertainty().values().begin(),
                                message.uncertainty().values().end());
    view.uncertainty = storage->uncertainty;
  }
  view.provenance_present = message.has_provenance();
  view.model_id = message.provenance().model_id();
  view.metadata_present = message.metadata_size() > 0;
  return view;
}

std::string LoadExample(const std::string& name) {
  const char* src = std::getenv("TEST_SRCDIR");
  if (src == nullptr) {
    return "";
  }
  const std::string suffix = "intrinsic/vehicle/proto/examples/" + name;
  const char* workspace = std::getenv("TEST_WORKSPACE");
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

TrajectoryContractAssessment AssessText(const std::string& name) {
  const std::string text = LoadExample(name);
  EXPECT_FALSE(text.empty()) << name;
  VehicleTrajectory parsed;
  EXPECT_TRUE(google::protobuf::TextFormat::ParseFromString(text, &parsed))
      << name;
  TrajectoryStorage storage;
  return AssessVehicleTrajectory(ViewOf(parsed, &storage));
}

TEST(TrajectoryContractSerializationTest, NoRobotTypeAndLockedFieldNumbers) {
  const auto* descriptor = VehicleTrajectory::descriptor();
  EXPECT_EQ(descriptor->file()->package(), "intrinsic_proto.vehicle");
  EXPECT_EQ(descriptor->FindFieldByName("robot_type"), nullptr);
  EXPECT_EQ(descriptor->file()->FindEnumTypeByName("RobotType"), nullptr);
  EXPECT_EQ(descriptor->FindFieldByName("header")->number(), 1);
  EXPECT_EQ(descriptor->FindFieldByName("trajectory_id")->number(), 2);
  EXPECT_EQ(descriptor->FindFieldByName("samples")->number(), 3);
  EXPECT_EQ(descriptor->FindFieldByName("tolerances")->number(), 4);
  EXPECT_EQ(descriptor->FindFieldByName("cost")->number(), 5);
  EXPECT_EQ(descriptor->FindFieldByName("risk")->number(), 6);
  EXPECT_EQ(descriptor->FindFieldByName("uncertainty")->number(), 7);
  EXPECT_EQ(descriptor->FindFieldByName("provenance")->number(), 8);
  EXPECT_EQ(descriptor->FindFieldByName("metadata")->number(), 100);
  const auto* sample =
      descriptor->file()->FindMessageTypeByName("TrajectorySample");
  ASSERT_NE(sample, nullptr);
  EXPECT_EQ(sample->FindFieldByName("time")->number(), 1);
  EXPECT_EQ(sample->FindFieldByName("pose_world_from_body")->number(), 2);
  EXPECT_EQ(sample->FindFieldByName("body_twist")->number(), 3);
  EXPECT_EQ(sample->FindFieldByName("body_acceleration")->number(), 4);
}

TEST(TrajectoryContractSerializationTest, DefaultIsZeroBytesAndNotAccepted) {
  const VehicleTrajectory empty;
  EXPECT_EQ(empty.SerializeAsString(), "");
  EXPECT_EQ(empty.ByteSizeLong(), 0u);
  TrajectoryStorage storage;
  const TrajectoryContractAssessment assessment =
      AssessVehicleTrajectory(ViewOf(empty, &storage));
  EXPECT_EQ(assessment.error, TrajectoryContractError::kNone);
  EXPECT_FALSE(assessment.accepted);
  EXPECT_FALSE(empty.has_cost());
  EXPECT_FALSE(empty.has_risk());
  EXPECT_FALSE(empty.has_uncertainty());
  EXPECT_FALSE(empty.has_tolerances());
}

TEST(TrajectoryContractSerializationTest, GoldenRoundTrip) {
  VehicleTrajectory trajectory;
  FillTwoSample(&trajectory);
  const std::string bytes = trajectory.SerializeAsString();
  ASSERT_EQ(ToHex(bytes), kVehicleTrajectoryGoldenHex) << ToHex(bytes);
  VehicleTrajectory parsed;
  ASSERT_TRUE(parsed.ParseFromString(FromHex(kVehicleTrajectoryGoldenHex)));
  EXPECT_EQ(parsed.SerializeAsString(), bytes);
  EXPECT_EQ(parsed.trajectory_id(), "traj_alpha");
  EXPECT_EQ(parsed.header().frame_id(), embodiment::kWorldEnuFrameId);
  ASSERT_EQ(parsed.samples_size(), 2);
  EXPECT_EQ(parsed.samples(0).time().seconds(), 1700000010);
  EXPECT_EQ(parsed.samples(1).time().nanos(), 500000000);
  EXPECT_FALSE(parsed.has_uncertainty());
  EXPECT_EQ(parsed.provenance().model_id(), "uuv_planner");
  TrajectoryStorage storage;
  EXPECT_TRUE(AssessVehicleTrajectory(ViewOf(parsed, &storage)).accepted);
}

TEST(TrajectoryContractSerializationTest, UnknownFieldIsPreserved) {
  VehicleTrajectory trajectory;
  FillTwoSample(&trajectory);
  const std::string golden = trajectory.SerializeAsString();
  // Field 101, varint 7. Field 100 is metadata, so this tag is unknown.
  const std::string with_unknown = golden + std::string("\xA8\x06\x07", 3);
  VehicleTrajectory parsed;
  ASSERT_TRUE(parsed.ParseFromString(with_unknown));
  EXPECT_EQ(parsed.trajectory_id(), "traj_alpha");
  const google::protobuf::UnknownFieldSet& unknown =
      parsed.GetReflection()->GetUnknownFields(parsed);
  ASSERT_EQ(unknown.field_count(), 1);
  EXPECT_EQ(unknown.field(0).number(), 101);
  EXPECT_EQ(unknown.field(0).varint(), 7u);
  EXPECT_EQ(parsed.SerializeAsString(), with_unknown);
  TrajectoryStorage storage;
  EXPECT_TRUE(AssessVehicleTrajectory(ViewOf(parsed, &storage)).accepted);
}

TEST(TrajectoryContractSerializationTest, MetadataShuffleStaysAccepted) {
  VehicleTrajectory first;
  FillTwoSample(&first);
  (*first.mutable_metadata())["planner"] = "direct";
  (*first.mutable_metadata())["note"] = "";
  VehicleTrajectory second;
  FillTwoSample(&second);
  (*second.mutable_metadata())["note"] = "";
  (*second.mutable_metadata())["planner"] = "direct";
  EXPECT_EQ(first.SerializeAsString(), second.SerializeAsString());
  TrajectoryStorage first_storage;
  TrajectoryStorage second_storage;
  const TrajectoryContractAssessment first_assessment =
      AssessVehicleTrajectory(ViewOf(first, &first_storage));
  const TrajectoryContractAssessment second_assessment =
      AssessVehicleTrajectory(ViewOf(second, &second_storage));
  EXPECT_TRUE(first_assessment.accepted);
  EXPECT_TRUE(second_assessment.accepted);
  EXPECT_EQ(first_assessment.error, second_assessment.error);

  (*first.mutable_metadata())[""] = "nan";
  TrajectoryStorage empty_key_storage;
  EXPECT_TRUE(
      AssessVehicleTrajectory(ViewOf(first, &empty_key_storage)).accepted);
}

TEST(TrajectoryContractSerializationTest, ZeroCostIsPresent) {
  VehicleTrajectory trajectory;
  EXPECT_FALSE(trajectory.has_cost());
  trajectory.set_cost(0);
  EXPECT_TRUE(trajectory.has_cost());
  const std::string with_zero = trajectory.SerializeAsString();
  trajectory.clear_cost();
  EXPECT_FALSE(trajectory.has_cost());
  EXPECT_NE(trajectory.SerializeAsString(), with_zero);
}

TEST(TrajectoryContractSerializationTest, PresentZeroTwistIsNotAbsence) {
  VehicleTrajectory trajectory;
  trajectory.set_trajectory_id("traj_alpha");
  auto* sample = trajectory.add_samples();
  sample->mutable_time()->set_seconds(1);
  sample->mutable_pose_world_from_body()->mutable_orientation()->set_w(1);
  sample->mutable_body_twist();
  EXPECT_TRUE(sample->has_body_twist());
  VehicleTrajectory parsed;
  ASSERT_TRUE(parsed.ParseFromString(trajectory.SerializeAsString()));
  ASSERT_EQ(parsed.samples_size(), 1);
  EXPECT_TRUE(parsed.samples(0).has_body_twist());
  EXPECT_EQ(parsed.samples(0).body_twist().linear_x_m_s(), 0);
}

TEST(TrajectoryContractSerializationTest, TextprotoFixtures) {
  const std::string two_sample =
      LoadExample("vehicle_trajectory_two_sample.textproto");
  ASSERT_FALSE(two_sample.empty());
  VehicleTrajectory parsed_two_sample;
  ASSERT_TRUE(google::protobuf::TextFormat::ParseFromString(
      two_sample, &parsed_two_sample));
  VehicleTrajectory coded;
  FillTwoSample(&coded);
  EXPECT_EQ(parsed_two_sample.SerializeAsString(), coded.SerializeAsString());
  TrajectoryStorage storage;
  EXPECT_TRUE(
      AssessVehicleTrajectory(ViewOf(parsed_two_sample, &storage)).accepted);

  const std::string with_metadata =
      LoadExample("vehicle_trajectory_with_metadata.textproto");
  ASSERT_FALSE(with_metadata.empty());
  VehicleTrajectory parsed_metadata;
  ASSERT_TRUE(google::protobuf::TextFormat::ParseFromString(with_metadata,
                                                            &parsed_metadata));
  EXPECT_FALSE(parsed_metadata.has_uncertainty());
  EXPECT_EQ(parsed_metadata.metadata().at("planner"), "direct");
  EXPECT_EQ(parsed_metadata.metadata().at("note"), "");
  TrajectoryStorage metadata_storage;
  EXPECT_TRUE(
      AssessVehicleTrajectory(ViewOf(parsed_metadata, &metadata_storage))
          .accepted);

  const TrajectoryContractAssessment non_monotonic =
      AssessText("vehicle_trajectory_non_monotonic.textproto");
  EXPECT_EQ(non_monotonic.error, TrajectoryContractError::kNonMonotonicTime);
  EXPECT_EQ(non_monotonic.validity, embodiment::ValidityKind::kValid);
  EXPECT_FALSE(non_monotonic.accepted);

  const TrajectoryContractAssessment bad_frame =
      AssessText("vehicle_trajectory_bad_frame.textproto");
  EXPECT_EQ(bad_frame.error, TrajectoryContractError::kMissingFrame);

  const TrajectoryContractAssessment non_finite =
      AssessText("vehicle_trajectory_non_finite.textproto");
  EXPECT_EQ(non_finite.error, TrajectoryContractError::kNonFinite);
  EXPECT_EQ(non_finite.validity, embodiment::ValidityKind::kValid);

  const TrajectoryContractAssessment empty_id =
      AssessText("vehicle_trajectory_empty_id.textproto");
  EXPECT_EQ(empty_id.error, TrajectoryContractError::kTrajectoryId);
  EXPECT_EQ(empty_id.validity, embodiment::ValidityKind::kValid);
}

}  // namespace
}  // namespace intrinsic::vehicle
