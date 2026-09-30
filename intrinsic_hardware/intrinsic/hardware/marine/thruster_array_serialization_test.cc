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
#include <filesystem>
#include <fstream>
#include <optional>
#include <sstream>
#include <string>
#include <string_view>
#include <vector>

#include "google/protobuf/descriptor.h"
#include "google/protobuf/text_format.h"
#include "google/protobuf/unknown_field_set.h"
#include "intrinsic/embodiment/proto/stamped_header.pb.h"
#include "intrinsic/hardware/marine/fake_thruster_array.h"
#include "intrinsic/hardware/marine/thruster_array.pb.h"
#include "intrinsic/hardware/marine/thruster_array_policy.h"
#include "intrinsic/vehicle/parameters/marine_model.h"
#include "gtest/gtest.h"

namespace intrinsic::hardware::marine {
namespace {

using intrinsic::vehicle::parameters::ThrusterHealthState;
using intrinsic_proto::embodiment::Validity;
using intrinsic_proto::hardware::marine::ThrusterArrayCommand;
using intrinsic_proto::hardware::marine::ThrusterArrayFeedback;
using intrinsic_proto::hardware::marine::ThrusterHealth;

// Canonical serialization of the nominal command and of
// FakeThrusterArray().Apply(that command) with the default config.
// Produced by a local C++ protobuf run. Bazel was not available in the
// authoring environment. Keep in sync with
// thruster_array_serialization_test.py.
constexpr std::string_view kNominalCommandGoldenHex =
    "0a3c082a120b0880e2cfaa061080e59a771a060881e2cfaa06220e74687275737465"
    "725f61727261792a04626f647932096d6f6e6f746f6e69633a02080112150a0a7375"
    "7267655f706f7274110000000000002440121a0a0f73757267655f73746172626f61"
    "72641100000000000014c012140a09737761795f666f726511000000000000000012"
    "130a08737761795f61667411000000000000104012150a0a68656176655f666f7265"
    "11000000000000204012140a0968656176655f6166741100000000000000c0";
constexpr std::string_view kNominalFeedbackGoldenHex =
    "0a3c082a120b0880e2cfaa061080e59a771a060881e2cfaa06220e74687275737465"
    "725f61727261792a04626f647932096d6f6e6f746f6e69633a02080112340a0a7375"
    "7267655f706f72741100000000000024401900000000000024402000280031000000"
    "000000f03f39000000000000f03f12390a0f73757267655f73746172626f61726411"
    "00000000000014c01900000000000014c02000280031000000000000f03f39000000"
    "000000f03f12330a09737761795f666f726511000000000000000019000000000000"
    "00002000280031000000000000f03f39000000000000f03f12320a08737761795f61"
    "66741100000000000010401900000000000010402000280031000000000000f03f39"
    "000000000000f03f12340a0a68656176655f666f7265110000000000002040190000"
    "0000000020402000280031000000000000f03f39000000000000f03f12330a096865"
    "6176655f6166741100000000000000c01900000000000000c0200028003100000000"
    "0000f03f39000000000000f03f";

const char *kSlotNames[] = {
    "surge_port", "surge_starboard", "sway_fore",
    "sway_aft",   "heave_fore",      "heave_aft",
};
const double kSlotThrustN[] = {10.0, -5.0, 0.0, 4.0, 8.0, -2.0};

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

void FillNominalCommand(ThrusterArrayCommand *command) {
  auto *header = command->mutable_header();
  header->set_sequence(42);
  header->mutable_source_time()->set_seconds(1700000000);
  header->mutable_source_time()->set_nanos(250000000);
  header->mutable_receive_time()->set_seconds(1700000001);
  header->set_source_id("thruster_array");
  header->set_frame_id("body");
  header->set_clock_domain("monotonic");
  header->mutable_validity()->set_state(Validity::STATE_VALID);
  for (int i = 0; i < 6; ++i) {
    auto *element = command->add_thrusters();
    element->set_name(kSlotNames[i]);
    element->set_thrust_n(kSlotThrustN[i]);
  }
}

ThrusterHeaderView
HeaderView(const intrinsic_proto::embodiment::StampedHeader &header,
           bool present) {
  ThrusterHeaderView view;
  view.header_present = present;
  view.frame_id = header.frame_id();
  view.source_time_present = header.has_source_time();
  if (view.source_time_present) {
    view.source_time = embodiment::ClockReading{header.source_time().seconds(),
                                                header.source_time().nanos()};
  }
  view.receive_time_present = header.has_receive_time();
  if (view.receive_time_present) {
    view.receive_time = embodiment::ClockReading{
        header.receive_time().seconds(), header.receive_time().nanos()};
  }
  view.header_validity_present = header.has_validity();
  view.header_validity_state = header.validity().state();
  return view;
}

ThrusterArrayAssessment AssessCommand(const ThrusterArrayCommand &message) {
  std::vector<ThrusterCommandElementView> elements;
  for (const auto &element : message.thrusters()) {
    ThrusterCommandElementView view;
    view.name_present = element.has_name();
    view.name = element.name();
    view.thrust_present = element.has_thrust_n();
    view.thrust_n = element.thrust_n();
    view.enable_present = element.has_enable();
    view.enable = element.enable();
    elements.push_back(view);
  }
  ThrusterArrayCommandView sample;
  sample.header = HeaderView(message.header(), message.has_header());
  sample.thrusters = elements;
  return AssessThrusterArrayCommand(sample);
}

ThrusterArrayAssessment AssessFeedback(const ThrusterArrayFeedback &message) {
  std::vector<ThrusterFeedbackElementView> elements;
  for (const auto &element : message.thrusters()) {
    ThrusterFeedbackElementView view;
    view.name_present = element.has_name();
    view.name = element.name();
    view.commanded_thrust_present = element.has_commanded_thrust_n();
    view.commanded_thrust_n = element.commanded_thrust_n();
    view.measured_thrust_present = element.has_measured_thrust_n();
    view.measured_thrust_n = element.measured_thrust_n();
    view.saturated_present = element.has_saturated();
    view.saturated = element.saturated();
    view.health_present = element.has_health();
    view.health = element.health();
    view.health_derate_present = element.has_health_derate();
    view.health_derate = element.health_derate();
    view.efficiency_present = element.has_efficiency();
    view.efficiency = element.efficiency();
    elements.push_back(view);
  }
  ThrusterArrayFeedbackView sample;
  sample.header = HeaderView(message.header(), message.has_header());
  sample.thrusters = elements;
  return AssessThrusterArrayFeedback(sample);
}

std::string LoadExample(const std::string &name) {
  const char *src = std::getenv("TEST_SRCDIR");
  if (src == nullptr) {
    return "";
  }
  std::error_code error;
  std::string best;
  for (const auto &entry :
       std::filesystem::recursive_directory_iterator(src, error)) {
    if (error || !entry.is_regular_file() || entry.path().filename() != name) {
      continue;
    }
    const std::string path = entry.path().string();
    if (path.find("/examples/") == std::string::npos) {
      continue;
    }
    if (best.empty() || path.size() < best.size()) {
      best = path;
    }
  }
  if (best.empty()) {
    return "";
  }
  std::ifstream in(best);
  std::ostringstream buffer;
  buffer << in.rdbuf();
  return buffer.str();
}

TEST(ThrusterArraySerializationTest, ContractShapeAndHealthWireNumbers) {
  const google::protobuf::Descriptor *command =
      ThrusterArrayCommand::descriptor();
  const google::protobuf::Descriptor *feedback =
      ThrusterArrayFeedback::descriptor();
  EXPECT_EQ(command->file()->package(), "intrinsic_proto.hardware.marine");
  EXPECT_EQ(command->FindFieldByName("header")->number(), 1);
  EXPECT_EQ(command->FindFieldByName("thrusters")->number(), 2);
  EXPECT_EQ(feedback->FindFieldByName("header")->number(), 1);
  EXPECT_EQ(feedback->FindFieldByName("thrusters")->number(), 2);
  EXPECT_EQ(command->FindFieldByName("wrench"), nullptr);
  EXPECT_EQ(feedback->FindFieldByName("wrench"), nullptr);
  EXPECT_EQ(command->FindFieldByName("header")->message_type()->full_name(),
            "intrinsic_proto.embodiment.StampedHeader");

  const google::protobuf::Descriptor *command_element =
      command->FindFieldByName("thrusters")->message_type();
  for (const char *name : {"name", "thrust_n", "enable"}) {
    const google::protobuf::FieldDescriptor *field =
        command_element->FindFieldByName(name);
    ASSERT_NE(field, nullptr) << name;
    EXPECT_TRUE(field->has_presence()) << name;
  }
  const google::protobuf::Descriptor *feedback_element =
      feedback->FindFieldByName("thrusters")->message_type();
  for (const char *name :
       {"name", "commanded_thrust_n", "measured_thrust_n", "saturated",
        "health", "health_derate", "efficiency"}) {
    const google::protobuf::FieldDescriptor *field =
        feedback_element->FindFieldByName(name);
    ASSERT_NE(field, nullptr) << name;
    EXPECT_TRUE(field->has_presence()) << name;
  }

  const google::protobuf::EnumDescriptor *health =
      intrinsic_proto::hardware::marine::ThrusterHealth_descriptor();
  ASSERT_EQ(health->value_count(), 5);
  EXPECT_EQ(health->FindValueByName("THRUSTER_HEALTH_UNSPECIFIED"), nullptr);
  EXPECT_EQ(health->FindValueByName("THRUSTER_HEALTH_NOMINAL")->number(), 0);
  EXPECT_EQ(health->FindValueByName("THRUSTER_HEALTH_DISABLED")->number(), 1);
  EXPECT_EQ(health->FindValueByName("THRUSTER_HEALTH_DERATED")->number(), 2);
  EXPECT_EQ(health->FindValueByName("THRUSTER_HEALTH_STUCK_OFF")->number(), 3);
  EXPECT_EQ(health->FindValueByName("THRUSTER_HEALTH_FAILED")->number(), 4);
  EXPECT_EQ(static_cast<int>(ThrusterHealth::THRUSTER_HEALTH_NOMINAL),
            static_cast<int>(ThrusterHealthState::kNominal));
  EXPECT_EQ(static_cast<int>(ThrusterHealth::THRUSTER_HEALTH_DISABLED),
            static_cast<int>(ThrusterHealthState::kDisabled));
  EXPECT_EQ(static_cast<int>(ThrusterHealth::THRUSTER_HEALTH_DERATED),
            static_cast<int>(ThrusterHealthState::kDerated));
  EXPECT_EQ(static_cast<int>(ThrusterHealth::THRUSTER_HEALTH_STUCK_OFF),
            static_cast<int>(ThrusterHealthState::kStuckOff));
  EXPECT_EQ(static_cast<int>(ThrusterHealth::THRUSTER_HEALTH_FAILED),
            static_cast<int>(ThrusterHealthState::kFailed));

  const google::protobuf::EnumDescriptor *validity =
      Validity::State_descriptor();
  ASSERT_EQ(validity->value_count(), 3);
  EXPECT_EQ(validity->FindValueByName("STATE_DEGRADED"), nullptr);
}

TEST(ThrusterArraySerializationTest, DefaultsAreEmptyAndOptIn) {
  EXPECT_EQ(ThrusterArrayCommand().SerializeAsString(), "");
  EXPECT_EQ(ThrusterArrayFeedback().SerializeAsString(), "");
  const ThrusterArrayCommand command;
  EXPECT_FALSE(command.has_header());
  EXPECT_EQ(command.thrusters_size(), 0);
}

TEST(ThrusterArraySerializationTest, PresenceIsDistinctFromDefaultValues) {
  intrinsic_proto::hardware::marine::ThrusterCommandElement thrust;
  EXPECT_FALSE(thrust.has_thrust_n());
  thrust.set_thrust_n(0.0);
  EXPECT_TRUE(thrust.has_thrust_n());
  EXPECT_NE(thrust.SerializeAsString(), "");

  intrinsic_proto::hardware::marine::ThrusterCommandElement enable;
  EXPECT_FALSE(enable.has_enable());
  enable.set_enable(false);
  EXPECT_TRUE(enable.has_enable());
  EXPECT_NE(enable.SerializeAsString(), "");

  intrinsic_proto::hardware::marine::ThrusterFeedbackElement health;
  EXPECT_FALSE(health.has_health());
  health.set_health(ThrusterHealth::THRUSTER_HEALTH_NOMINAL);
  EXPECT_TRUE(health.has_health());
  EXPECT_NE(health.SerializeAsString(), "");

  intrinsic_proto::hardware::marine::ThrusterFeedbackElement saturated;
  saturated.set_saturated(false);
  EXPECT_TRUE(saturated.has_saturated());
  EXPECT_NE(saturated.SerializeAsString(), "");
}

TEST(ThrusterArraySerializationTest, NominalRoundTripMatchesGoldenAndFake) {
  ThrusterArrayCommand command;
  FillNominalCommand(&command);
  const std::string command_hex = ToHex(command.SerializeAsString());
  ASSERT_EQ(command_hex, kNominalCommandGoldenHex) << command_hex;
  ThrusterArrayCommand parsed_command;
  ASSERT_TRUE(
      parsed_command.ParseFromString(FromHex(kNominalCommandGoldenHex)));
  EXPECT_EQ(parsed_command.SerializeAsString(),
            FromHex(kNominalCommandGoldenHex));
  EXPECT_TRUE(AssessCommand(parsed_command).accepted);

  const std::optional<ThrusterArrayFeedback> feedback =
      FakeThrusterArray().Apply(command);
  ASSERT_TRUE(feedback.has_value());
  const std::string feedback_hex = ToHex(feedback->SerializeAsString());
  ASSERT_EQ(feedback_hex, kNominalFeedbackGoldenHex) << feedback_hex;
  ThrusterArrayFeedback parsed_feedback;
  ASSERT_TRUE(
      parsed_feedback.ParseFromString(FromHex(kNominalFeedbackGoldenHex)));
  EXPECT_EQ(parsed_feedback.header().sequence(), 42u);
  EXPECT_EQ(parsed_feedback.header().frame_id(), "body");
  ASSERT_EQ(parsed_feedback.thrusters_size(), 6);
  EXPECT_DOUBLE_EQ(parsed_feedback.thrusters(2).measured_thrust_n(), 0.0);
  EXPECT_EQ(parsed_feedback.thrusters(0).health(),
            ThrusterHealth::THRUSTER_HEALTH_NOMINAL);
  EXPECT_TRUE(AssessFeedback(parsed_feedback).accepted);
}

TEST(ThrusterArraySerializationTest, UnknownFieldAndHealthArePreserved) {
  ThrusterArrayCommand command;
  FillNominalCommand(&command);
  const std::string golden = command.SerializeAsString();
  const std::string with_unknown = golden + std::string("\xA0\x06\x07", 3);
  ThrusterArrayCommand parsed;
  ASSERT_TRUE(parsed.ParseFromString(with_unknown));
  EXPECT_EQ(parsed.thrusters(0).name(), "surge_port");
  const google::protobuf::UnknownFieldSet &unknown =
      parsed.GetReflection()->GetUnknownFields(parsed);
  ASSERT_EQ(unknown.field_count(), 1);
  EXPECT_EQ(unknown.field(0).number(), 100);
  EXPECT_EQ(unknown.field(0).varint(), 7u);
  EXPECT_EQ(parsed.SerializeAsString(), with_unknown);

  ThrusterArrayFeedback feedback;
  ASSERT_TRUE(FakeThrusterArray().Apply(command).has_value());
  feedback = *FakeThrusterArray().Apply(command);
  feedback.mutable_thrusters(0)->set_health(static_cast<ThrusterHealth>(99));
  ThrusterArrayFeedback reparsed;
  ASSERT_TRUE(reparsed.ParseFromString(feedback.SerializeAsString()));
  EXPECT_EQ(static_cast<int>(reparsed.thrusters(0).health()), 99);
  EXPECT_EQ(ClassifyThrusterHealth(true, reparsed.thrusters(0).health()),
            ThrusterHealthKind::kUnrecognized);
  const ThrusterArrayAssessment assessment = AssessFeedback(reparsed);
  EXPECT_EQ(assessment.error, ThrusterArrayError::kHealth);
  EXPECT_FALSE(assessment.accepted);
}

TEST(ThrusterArraySerializationTest, TextprotoExamplesMatchTheFixture) {
  const std::string command_text =
      LoadExample("thruster_array_command_nominal.textproto");
  ASSERT_FALSE(command_text.empty());
  ThrusterArrayCommand parsed_command;
  ASSERT_TRUE(google::protobuf::TextFormat::ParseFromString(command_text,
                                                            &parsed_command));
  ThrusterArrayCommand coded;
  FillNominalCommand(&coded);
  EXPECT_EQ(parsed_command.SerializeAsString(), coded.SerializeAsString());
  EXPECT_TRUE(AssessCommand(parsed_command).accepted);

  const std::string feedback_text =
      LoadExample("thruster_array_feedback_nominal.textproto");
  ASSERT_FALSE(feedback_text.empty());
  ThrusterArrayFeedback parsed_feedback;
  ASSERT_TRUE(google::protobuf::TextFormat::ParseFromString(feedback_text,
                                                            &parsed_feedback));
  const std::optional<ThrusterArrayFeedback> faked =
      FakeThrusterArray().Apply(coded);
  ASSERT_TRUE(faked.has_value());
  EXPECT_EQ(parsed_feedback.SerializeAsString(), faked->SerializeAsString());
  EXPECT_TRUE(AssessFeedback(parsed_feedback).accepted);
}

} // namespace
} // namespace intrinsic::hardware::marine
