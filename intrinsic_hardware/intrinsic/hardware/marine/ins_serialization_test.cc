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
#include <sstream>
#include <string>
#include <string_view>
#include <vector>

#include "google/protobuf/descriptor.h"
#include "google/protobuf/text_format.h"
#include "google/protobuf/unknown_field_set.h"
#include "gtest/gtest.h"
#include "intrinsic/embodiment/proto/stamped_header.pb.h"
#include "intrinsic/embodiment/stamped_header_policy.h"
#include "intrinsic/hardware/marine/fake_ins.h"
#include "intrinsic/hardware/marine/ins.pb.h"
#include "intrinsic/hardware/marine/ins_policy.h"
#include "intrinsic/vehicle/vehicle_contract_policy.h"

namespace intrinsic::hardware::marine {
namespace {

using intrinsic_proto::embodiment::Validity;
using intrinsic_proto::hardware::marine::InsSolution;
using intrinsic_proto::hardware::marine::MeasurementHealth;

// Canonical serialization of the default FakeIns / FillNominal().
// Produced by a local C++ protobuf run of that fixture. Bazel was not
// available in the authoring environment. Keep in sync with
// ins_serialization_test.py.
constexpr std::string_view kNominalGoldenHex =
    "0aff020a37082a120b0880e2cfaa061080e59a771a060881e2cfaa06220a6e61765f"
    "73656e736f722a03696e7332096d6f6e6f746f6e69633a02080110011d0000403f22"
    "a3020aa002000000000000f03f000000000000000000000000000000000000000000"
    "00000000000000000000000000000000000000000000000000000000000000000010"
    "40000000000000000000000000000000000000000000000000000000000000000000"
    "000000000000000000000000000000000000000000d03f0000000000000000000000"
    "00000000000000000000000000000000000000000000000000000000000000000000"
    "000000000000000000b03f0000000000000000000000000000000000000000000000"
    "00000000000000000000000000000000000000000000000000000000000000c03f00"
    "00000000000000000000000000000000000000000000000000000000000000000000"
    "00000000000000000000000000000000000000e03f2a0d0a077072696d6172791202"
    "08012a080a06616964696e671100000000000028401900000000000010c021000000"
    "000000e03f2a0921000000000000f03f31000000000000f83f390000000000000000"
    "41000000000000d0bf49000000000000000051000000000000c03f59000000000000"
    "00006001";

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

void FillInsCovariance(intrinsic_proto::vehicle::Matrix6* matrix) {
  matrix->mutable_values()->Resize(vehicle::kCovarianceValues, 0.0);
  matrix->set_values(kPositionXVarianceSlot, 1.0);
  matrix->set_values(kPositionYVarianceSlot, 4.0);
  matrix->set_values(kPositionZVarianceSlot, 0.25);
  matrix->set_values(kAttitudeXVarianceSlot, 0.0625);
  matrix->set_values(kAttitudeYVarianceSlot, 0.125);
  matrix->set_values(kAttitudeZVarianceSlot, 0.5);
}

void FillNominal(InsSolution* sample) {
  MeasurementHealth* health = sample->mutable_health();
  auto* header = health->mutable_header();
  header->set_sequence(42);
  header->mutable_source_time()->set_seconds(1700000000);
  header->mutable_source_time()->set_nanos(250000000);
  header->mutable_receive_time()->set_seconds(1700000001);
  header->set_source_id("nav_sensor");
  header->set_frame_id("ins");
  header->set_clock_domain(std::string(embodiment::kClockDomainMonotonic));
  header->mutable_validity()->set_state(Validity::STATE_VALID);
  health->set_state(MeasurementHealth::VALID);
  health->set_quality(0.75f);
  FillInsCovariance(health->mutable_covariance());
  auto* primary = health->add_sources();
  primary->set_source_id("primary");
  primary->mutable_validity()->set_state(Validity::STATE_VALID);
  health->add_sources()->set_source_id("aiding");
  sample->set_position_x_m(12.0);
  sample->set_position_y_m(-4.0);
  sample->set_position_z_m(0.5);
  auto* orientation = sample->mutable_orientation_xyzw();
  orientation->set_x(0.0);
  orientation->set_y(0.0);
  orientation->set_z(0.0);
  orientation->set_w(1.0);
  sample->set_linear_velocity_x_m_s(1.5);
  sample->set_linear_velocity_y_m_s(0.0);
  sample->set_linear_velocity_z_m_s(-0.25);
  sample->set_angular_velocity_x_rad_s(0.0);
  sample->set_angular_velocity_y_rad_s(0.125);
  sample->set_angular_velocity_z_rad_s(0.0);
  sample->set_source(InsSolution::SOURCE_VENDOR_INS);
}

InsSolutionView ViewOf(const InsSolution& sample,
                       std::vector<double>* covariance,
                       std::vector<SourceHealthView>* sources) {
  InsSolutionView view;
  const MeasurementHealth& health = sample.health();
  view.health.header_present = health.has_header();
  view.health.frame_id = health.header().frame_id();
  view.health.source_time_present = health.header().has_source_time();
  if (view.health.source_time_present) {
    view.health.source_time =
        embodiment::ClockReading{health.header().source_time().seconds(),
                                 health.header().source_time().nanos()};
  }
  view.health.receive_time_present = health.header().has_receive_time();
  if (view.health.receive_time_present) {
    view.health.receive_time =
        embodiment::ClockReading{health.header().receive_time().seconds(),
                                 health.header().receive_time().nanos()};
  }
  view.health.header_validity_present = health.header().has_validity();
  view.health.header_validity_state = health.header().validity().state();
  view.health.state_present = health.has_state();
  view.health.state = health.state();
  view.health.quality_present = health.has_quality();
  view.health.quality = health.quality();
  view.health.covariance_present = health.has_covariance();
  if (view.health.covariance_present) {
    covariance->assign(health.covariance().values().begin(),
                       health.covariance().values().end());
    view.health.covariance = *covariance;
  }
  sources->clear();
  for (const auto& source : health.sources()) {
    sources->push_back(SourceHealthView{
        source.source_id(), source.has_validity(), source.validity().state()});
  }
  view.health.sources = *sources;
  view.position_x_present = sample.has_position_x_m();
  view.position_x_m = sample.position_x_m();
  view.position_y_present = sample.has_position_y_m();
  view.position_y_m = sample.position_y_m();
  view.position_z_present = sample.has_position_z_m();
  view.position_z_m = sample.position_z_m();
  view.orientation_present = sample.has_orientation_xyzw();
  view.orientation_x = sample.orientation_xyzw().x();
  view.orientation_y = sample.orientation_xyzw().y();
  view.orientation_z = sample.orientation_xyzw().z();
  view.orientation_w = sample.orientation_xyzw().w();
  view.linear_velocity_x_present = sample.has_linear_velocity_x_m_s();
  view.linear_velocity_x_m_s = sample.linear_velocity_x_m_s();
  view.linear_velocity_y_present = sample.has_linear_velocity_y_m_s();
  view.linear_velocity_y_m_s = sample.linear_velocity_y_m_s();
  view.linear_velocity_z_present = sample.has_linear_velocity_z_m_s();
  view.linear_velocity_z_m_s = sample.linear_velocity_z_m_s();
  view.angular_velocity_x_present = sample.has_angular_velocity_x_rad_s();
  view.angular_velocity_x_rad_s = sample.angular_velocity_x_rad_s();
  view.angular_velocity_y_present = sample.has_angular_velocity_y_rad_s();
  view.angular_velocity_y_rad_s = sample.angular_velocity_y_rad_s();
  view.angular_velocity_z_present = sample.has_angular_velocity_z_rad_s();
  view.angular_velocity_z_rad_s = sample.angular_velocity_z_rad_s();
  view.source_present = sample.has_source();
  view.source = sample.source();
  return view;
}

std::string LoadExample(const std::string& name) {
  const char* src = std::getenv("TEST_SRCDIR");
  if (src == nullptr) {
    return "";
  }
  std::error_code error;
  std::string best;
  for (const auto& entry :
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

TEST(InsSerializationTest, ContractShapeAndUnchangedValidity) {
  const google::protobuf::Descriptor* descriptor = InsSolution::descriptor();
  EXPECT_EQ(descriptor->file()->package(), "intrinsic_proto.hardware.marine");
  EXPECT_EQ(descriptor->FindFieldByName("health")->message_type()->full_name(),
            "intrinsic_proto.hardware.marine.MeasurementHealth");
  EXPECT_TRUE(descriptor->FindFieldByName("position_x_m")->has_presence());
  EXPECT_TRUE(descriptor->FindFieldByName("orientation_xyzw")->has_presence());
  EXPECT_TRUE(
      descriptor->FindFieldByName("linear_velocity_x_m_s")->has_presence());
  EXPECT_TRUE(
      descriptor->FindFieldByName("angular_velocity_z_rad_s")->has_presence());
  EXPECT_TRUE(descriptor->FindFieldByName("source")->has_presence());
  EXPECT_EQ(descriptor->FindFieldByName("magnetic_field_x"), nullptr);
  const google::protobuf::EnumDescriptor* validity =
      Validity::State_descriptor();
  ASSERT_EQ(validity->value_count(), 3);
  EXPECT_EQ(validity->FindValueByName("STATE_DEGRADED"), nullptr);
  const google::protobuf::EnumDescriptor* source =
      descriptor->FindEnumTypeByName("SourceKind");
  ASSERT_NE(source, nullptr);
  EXPECT_EQ(source->FindValueByName("SOURCE_UNSPECIFIED")->number(), 0);
  EXPECT_EQ(source->FindValueByName("SOURCE_VENDOR_INS")->number(), 1);
  EXPECT_EQ(source->FindValueByName("SOURCE_EXTERNAL_NAV")->number(), 2);
}

TEST(InsSerializationTest, DefaultsAreEmptyAndOptIn) {
  EXPECT_EQ(InsSolution().SerializeAsString(), "");
  EXPECT_FALSE(InsSolution().has_health());
  EXPECT_FALSE(InsSolution().has_position_x_m());
  EXPECT_FALSE(InsSolution().has_orientation_xyzw());
  EXPECT_FALSE(InsSolution().has_linear_velocity_x_m_s());
  EXPECT_FALSE(InsSolution().has_angular_velocity_x_rad_s());
  EXPECT_FALSE(InsSolution().has_source());
}

TEST(InsSerializationTest, NominalRoundTripMatchesGoldenAndFake) {
  InsSolution sample;
  FillNominal(&sample);
  const std::optional<InsSolution> faked = FakeIns().Measure();
  ASSERT_TRUE(faked.has_value());
  EXPECT_EQ(sample.SerializeAsString(), faked->SerializeAsString());
  const std::string hex = ToHex(sample.SerializeAsString());
  ASSERT_EQ(hex, kNominalGoldenHex) << hex;
  InsSolution parsed;
  ASSERT_TRUE(parsed.ParseFromString(FromHex(kNominalGoldenHex)));
  EXPECT_EQ(parsed.SerializeAsString(), FromHex(kNominalGoldenHex));
  EXPECT_EQ(parsed.health().header().frame_id(), "ins");
  EXPECT_EQ(parsed.health().header().sequence(), 42u);
  EXPECT_DOUBLE_EQ(parsed.position_x_m(), 12.0);
  EXPECT_DOUBLE_EQ(parsed.position_y_m(), -4.0);
  EXPECT_DOUBLE_EQ(parsed.position_z_m(), 0.5);
  EXPECT_DOUBLE_EQ(parsed.orientation_xyzw().w(), 1.0);
  EXPECT_DOUBLE_EQ(parsed.linear_velocity_x_m_s(), 1.5);
  EXPECT_DOUBLE_EQ(parsed.angular_velocity_y_rad_s(), 0.125);
  EXPECT_EQ(parsed.source(), InsSolution::SOURCE_VENDOR_INS);
  EXPECT_DOUBLE_EQ(parsed.health().covariance().values(kPositionXVarianceSlot),
                   1.0);
  EXPECT_DOUBLE_EQ(parsed.health().covariance().values(kAttitudeZVarianceSlot),
                   0.5);
  EXPECT_DOUBLE_EQ(parsed.health().covariance().values(1), 0.0);

  std::vector<double> covariance;
  std::vector<SourceHealthView> sources;
  const InsAssessment assessment =
      AssessIns(ViewOf(parsed, &covariance, &sources));
  EXPECT_TRUE(assessment.accepted);
  EXPECT_EQ(assessment.source, InsSourceKind::kVendorIns);
}

TEST(InsSerializationTest, PresenceIsDistinctFromDefaultValues) {
  InsSolution position;
  EXPECT_FALSE(position.has_position_z_m());
  position.set_position_z_m(0.0);
  EXPECT_TRUE(position.has_position_z_m());
  EXPECT_NE(position.SerializeAsString(), InsSolution().SerializeAsString());

  InsSolution source;
  EXPECT_FALSE(source.has_source());
  source.set_source(InsSolution::SOURCE_UNSPECIFIED);
  EXPECT_TRUE(source.has_source());
  EXPECT_EQ(source.source(), InsSolution::SOURCE_UNSPECIFIED);
  EXPECT_NE(source.SerializeAsString(), InsSolution().SerializeAsString());

  InsSolution zeros;
  for (int i = 0; i < vehicle::kCovarianceValues; ++i) {
    zeros.mutable_health()->mutable_covariance()->add_values(0.0);
  }
  EXPECT_NE(zeros.SerializeAsString(), InsSolution().SerializeAsString());
}

TEST(InsSerializationTest, ClearingSourceIsAPrefixOfTheGolden) {
  InsSolution sample;
  FillNominal(&sample);
  const std::string full = sample.SerializeAsString();
  sample.clear_source();
  EXPECT_TRUE(full.starts_with(sample.SerializeAsString()));
  EXPECT_NE(full, sample.SerializeAsString());
}

TEST(InsSerializationTest, UnknownFieldAndSourceArePreserved) {
  InsSolution sample;
  FillNominal(&sample);
  const std::string golden = sample.SerializeAsString();
  const std::string with_unknown = golden + std::string("\xA0\x06\x07", 3);
  InsSolution parsed;
  ASSERT_TRUE(parsed.ParseFromString(with_unknown));
  EXPECT_EQ(parsed.source(), InsSolution::SOURCE_VENDOR_INS);
  const google::protobuf::UnknownFieldSet& unknown =
      parsed.GetReflection()->GetUnknownFields(parsed);
  ASSERT_EQ(unknown.field_count(), 1);
  EXPECT_EQ(unknown.field(0).number(), 100);
  EXPECT_EQ(unknown.field(0).varint(), 7u);
  EXPECT_EQ(parsed.SerializeAsString(), with_unknown);

  parsed.set_source(static_cast<InsSolution::SourceKind>(100));
  InsSolution reparsed;
  ASSERT_TRUE(reparsed.ParseFromString(parsed.SerializeAsString()));
  EXPECT_EQ(static_cast<int>(reparsed.source()), 100);
  EXPECT_EQ(ClassifyInsSource(reparsed.has_source(), reparsed.source()),
            InsSourceKind::kUnrecognized);
}

TEST(InsSerializationTest, TextprotoExampleMatchesTheFixture) {
  const std::string text = LoadExample("ins_nominal.textproto");
  ASSERT_FALSE(text.empty()) << "ins_nominal.textproto";
  InsSolution parsed;
  ASSERT_TRUE(google::protobuf::TextFormat::ParseFromString(text, &parsed));
  InsSolution coded;
  FillNominal(&coded);
  EXPECT_EQ(parsed.SerializeAsString(), coded.SerializeAsString());
  EXPECT_EQ(parsed.SerializeAsString(), FromHex(kNominalGoldenHex));
}

}  // namespace
}  // namespace intrinsic::hardware::marine
