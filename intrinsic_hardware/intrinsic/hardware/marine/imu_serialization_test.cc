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
#include "intrinsic/hardware/marine/fake_imu.h"
#include "intrinsic/hardware/marine/imu.pb.h"
#include "intrinsic/hardware/marine/imu_policy.h"
#include "intrinsic/vehicle/vehicle_contract_policy.h"

namespace intrinsic::hardware::marine {
namespace {

using intrinsic_proto::embodiment::Validity;
using intrinsic_proto::hardware::marine::ImuMeasurement;
using intrinsic_proto::hardware::marine::MeasurementHealth;

// Canonical serialization of the default FakeImu / FillNominal().
// Produced by a local C++ protobuf run of that fixture. Bazel was not
// available in the authoring environment. Keep in sync with
// imu_serialization_test.py.
constexpr std::string_view kNominalGoldenHex =
    "0aff020a37082a120b0880e2cfaa061080e59a771a060881e2cfaa06220a6e61765f"
    "73656e736f722a03696d7532096d6f6e6f746f6e69633a02080110011d0000403f22"
    "a3020aa002000000000000d03f000000000000000000000000000000000000000000"
    "000000000000000000000000000000000000000000000000000000000000000000e0"
    "3f000000000000000000000000000000000000000000000000000000000000000000"
    "000000000000000000000000000000000000000000c03f0000000000000000000000"
    "00000000000000000000000000000000000000000000000000000000000000000000"
    "000000000000000000f03f0000000000000000000000000000000000000000000000"
    "00000000000000000000000000000000000000000000000000000000000000004000"
    "00000000000000000000000000000000000000000000000000000000000000000000"
    "0000000000000000000000000000000000000010402a0d0a077072696d6172791202"
    "08012a080a06616964696e6711000000000000d03f19000000000000e0bf21000000"
    "000000c03f2900000000000000003100000000000000003900000000000020404209"
    "21000000000000f03f";

// Same fixture with orientation absent.
constexpr std::string_view kRatesOnlyGoldenHex =
    "0aff020a37082a120b0880e2cfaa061080e59a771a060881e2cfaa06220a6e61765f"
    "73656e736f722a03696d7532096d6f6e6f746f6e69633a02080110011d0000403f22"
    "a3020aa002000000000000d03f000000000000000000000000000000000000000000"
    "000000000000000000000000000000000000000000000000000000000000000000e0"
    "3f000000000000000000000000000000000000000000000000000000000000000000"
    "000000000000000000000000000000000000000000c03f0000000000000000000000"
    "00000000000000000000000000000000000000000000000000000000000000000000"
    "000000000000000000f03f0000000000000000000000000000000000000000000000"
    "00000000000000000000000000000000000000000000000000000000000000004000"
    "00000000000000000000000000000000000000000000000000000000000000000000"
    "0000000000000000000000000000000000000010402a0d0a077072696d6172791202"
    "08012a080a06616964696e6711000000000000d03f19000000000000e0bf21000000"
    "000000c03f290000000000000000310000000000000000390000000000002040";

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

void FillImuCovariance(intrinsic_proto::vehicle::Matrix6* matrix) {
  matrix->mutable_values()->Resize(vehicle::kCovarianceValues, 0.0);
  matrix->set_values(kAngularVelocityXVarianceSlot, 0.25);
  matrix->set_values(kAngularVelocityYVarianceSlot, 0.5);
  matrix->set_values(kAngularVelocityZVarianceSlot, 0.125);
  matrix->set_values(kLinearAccelerationXVarianceSlot, 1.0);
  matrix->set_values(kLinearAccelerationYVarianceSlot, 2.0);
  matrix->set_values(kLinearAccelerationZVarianceSlot, 4.0);
}

void FillHealth(ImuMeasurement* sample) {
  MeasurementHealth* health = sample->mutable_health();
  auto* header = health->mutable_header();
  header->set_sequence(42);
  header->mutable_source_time()->set_seconds(1700000000);
  header->mutable_source_time()->set_nanos(250000000);
  header->mutable_receive_time()->set_seconds(1700000001);
  header->set_source_id("nav_sensor");
  header->set_frame_id("imu");
  header->set_clock_domain(std::string(embodiment::kClockDomainMonotonic));
  header->mutable_validity()->set_state(Validity::STATE_VALID);
  health->set_state(MeasurementHealth::VALID);
  health->set_quality(0.75f);
  FillImuCovariance(health->mutable_covariance());
  auto* primary = health->add_sources();
  primary->set_source_id("primary");
  primary->mutable_validity()->set_state(Validity::STATE_VALID);
  health->add_sources()->set_source_id("aiding");
}

void FillRates(ImuMeasurement* sample) {
  sample->set_angular_velocity_x_rad_s(0.25);
  sample->set_angular_velocity_y_rad_s(-0.5);
  sample->set_angular_velocity_z_rad_s(0.125);
  sample->set_linear_acceleration_x_m_s2(0.0);
  sample->set_linear_acceleration_y_m_s2(0.0);
  sample->set_linear_acceleration_z_m_s2(8.0);
}

void FillNominal(ImuMeasurement* sample) {
  FillHealth(sample);
  FillRates(sample);
  auto* orientation = sample->mutable_orientation_xyzw();
  orientation->set_x(0.0);
  orientation->set_y(0.0);
  orientation->set_z(0.0);
  orientation->set_w(1.0);
}

void FillRatesOnly(ImuMeasurement* sample) {
  FillHealth(sample);
  FillRates(sample);
}

ImuMeasurementView ViewOf(const ImuMeasurement& sample,
                          std::vector<double>* covariance,
                          std::vector<SourceHealthView>* sources) {
  ImuMeasurementView view;
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
  view.angular_velocity_x_present = sample.has_angular_velocity_x_rad_s();
  view.angular_velocity_x_rad_s = sample.angular_velocity_x_rad_s();
  view.angular_velocity_y_present = sample.has_angular_velocity_y_rad_s();
  view.angular_velocity_y_rad_s = sample.angular_velocity_y_rad_s();
  view.angular_velocity_z_present = sample.has_angular_velocity_z_rad_s();
  view.angular_velocity_z_rad_s = sample.angular_velocity_z_rad_s();
  view.linear_acceleration_x_present = sample.has_linear_acceleration_x_m_s2();
  view.linear_acceleration_x_m_s2 = sample.linear_acceleration_x_m_s2();
  view.linear_acceleration_y_present = sample.has_linear_acceleration_y_m_s2();
  view.linear_acceleration_y_m_s2 = sample.linear_acceleration_y_m_s2();
  view.linear_acceleration_z_present = sample.has_linear_acceleration_z_m_s2();
  view.linear_acceleration_z_m_s2 = sample.linear_acceleration_z_m_s2();
  view.orientation_present = sample.has_orientation_xyzw();
  view.orientation_x = sample.orientation_xyzw().x();
  view.orientation_y = sample.orientation_xyzw().y();
  view.orientation_z = sample.orientation_xyzw().z();
  view.orientation_w = sample.orientation_xyzw().w();
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

TEST(ImuSerializationTest, ContractShapeAndUnchangedValidity) {
  const google::protobuf::Descriptor* descriptor = ImuMeasurement::descriptor();
  EXPECT_EQ(descriptor->file()->package(), "intrinsic_proto.hardware.marine");
  EXPECT_EQ(descriptor->FindFieldByName("health")->message_type()->full_name(),
            "intrinsic_proto.hardware.marine.MeasurementHealth");
  EXPECT_TRUE(
      descriptor->FindFieldByName("angular_velocity_x_rad_s")->has_presence());
  EXPECT_TRUE(descriptor->FindFieldByName("linear_acceleration_z_m_s2")
                  ->has_presence());
  EXPECT_TRUE(descriptor->FindFieldByName("orientation_xyzw")->has_presence());
  EXPECT_EQ(descriptor->FindFieldByName("magnetic_field_x"), nullptr);
  EXPECT_EQ(descriptor->FindFieldByName("altitude_m"), nullptr);
  const google::protobuf::EnumDescriptor* validity =
      Validity::State_descriptor();
  ASSERT_EQ(validity->value_count(), 3);
  EXPECT_EQ(validity->FindValueByName("STATE_DEGRADED"), nullptr);
  EXPECT_EQ(validity->FindValueByName("DEGRADED"), nullptr);
}

TEST(ImuSerializationTest, DefaultsAreEmptyAndOptIn) {
  EXPECT_EQ(ImuMeasurement().SerializeAsString(), "");
  EXPECT_FALSE(ImuMeasurement().has_health());
  EXPECT_FALSE(ImuMeasurement().has_angular_velocity_x_rad_s());
  EXPECT_FALSE(ImuMeasurement().has_linear_acceleration_x_m_s2());
  EXPECT_FALSE(ImuMeasurement().has_orientation_xyzw());
}

TEST(ImuSerializationTest, NominalRoundTripMatchesGoldenAndFake) {
  ImuMeasurement sample;
  FillNominal(&sample);
  const std::optional<ImuMeasurement> faked = FakeImu().Measure();
  ASSERT_TRUE(faked.has_value());
  EXPECT_EQ(sample.SerializeAsString(), faked->SerializeAsString());
  const std::string hex = ToHex(sample.SerializeAsString());
  ASSERT_EQ(hex, kNominalGoldenHex) << hex;
  ImuMeasurement parsed;
  ASSERT_TRUE(parsed.ParseFromString(FromHex(kNominalGoldenHex)));
  EXPECT_EQ(parsed.SerializeAsString(), FromHex(kNominalGoldenHex));
  EXPECT_EQ(parsed.health().header().frame_id(), "imu");
  EXPECT_EQ(parsed.health().header().sequence(), 42u);
  EXPECT_DOUBLE_EQ(parsed.angular_velocity_x_rad_s(), 0.25);
  EXPECT_DOUBLE_EQ(parsed.angular_velocity_y_rad_s(), -0.5);
  EXPECT_DOUBLE_EQ(parsed.linear_acceleration_z_m_s2(), 8.0);
  EXPECT_DOUBLE_EQ(parsed.orientation_xyzw().w(), 1.0);
  EXPECT_EQ(parsed.health().covariance().values_size(),
            vehicle::kCovarianceValues);
  EXPECT_DOUBLE_EQ(
      parsed.health().covariance().values(kAngularVelocityXVarianceSlot), 0.25);
  EXPECT_DOUBLE_EQ(
      parsed.health().covariance().values(kLinearAccelerationZVarianceSlot),
      4.0);
  EXPECT_DOUBLE_EQ(parsed.health().covariance().values(1), 0.0);

  std::vector<double> covariance;
  std::vector<SourceHealthView> sources;
  EXPECT_TRUE(AssessImu(ViewOf(parsed, &covariance, &sources)).accepted);
}

TEST(ImuSerializationTest, RatesOnlyRoundTripMatchesGoldenAndFake) {
  ImuMeasurement sample;
  FillRatesOnly(&sample);
  ImuTruth truth;
  truth.orientation_present = false;
  const std::optional<ImuMeasurement> faked = FakeImu().Measure(truth);
  ASSERT_TRUE(faked.has_value());
  EXPECT_EQ(sample.SerializeAsString(), faked->SerializeAsString());
  const std::string hex = ToHex(sample.SerializeAsString());
  ASSERT_EQ(hex, kRatesOnlyGoldenHex) << hex;
  EXPECT_TRUE(std::string(kNominalGoldenHex).starts_with(kRatesOnlyGoldenHex));

  ImuMeasurement parsed;
  ASSERT_TRUE(parsed.ParseFromString(FromHex(kRatesOnlyGoldenHex)));
  EXPECT_FALSE(parsed.has_orientation_xyzw());
  std::vector<double> covariance;
  std::vector<SourceHealthView> sources;
  EXPECT_TRUE(AssessImu(ViewOf(parsed, &covariance, &sources)).accepted);
}

TEST(ImuSerializationTest, PresenceIsDistinctFromDefaultValues) {
  ImuMeasurement rate;
  EXPECT_FALSE(rate.has_angular_velocity_x_rad_s());
  rate.set_angular_velocity_x_rad_s(0.0);
  EXPECT_TRUE(rate.has_angular_velocity_x_rad_s());
  EXPECT_NE(rate.SerializeAsString(), ImuMeasurement().SerializeAsString());

  ImuMeasurement accel;
  accel.set_linear_acceleration_y_m_s2(0.0);
  EXPECT_TRUE(accel.has_linear_acceleration_y_m_s2());
  EXPECT_NE(accel.SerializeAsString(), ImuMeasurement().SerializeAsString());

  ImuMeasurement zeros;
  for (int i = 0; i < vehicle::kCovarianceValues; ++i) {
    zeros.mutable_health()->mutable_covariance()->add_values(0.0);
  }
  EXPECT_NE(zeros.SerializeAsString(), ImuMeasurement().SerializeAsString());
}

TEST(ImuSerializationTest, ClearingOrientationIsAPrefixOfTheGolden) {
  ImuMeasurement sample;
  FillNominal(&sample);
  const std::string full = sample.SerializeAsString();
  sample.clear_orientation_xyzw();
  EXPECT_TRUE(full.starts_with(sample.SerializeAsString()));
  EXPECT_NE(full, sample.SerializeAsString());
  EXPECT_EQ(sample.SerializeAsString(), FromHex(kRatesOnlyGoldenHex));
}

TEST(ImuSerializationTest, UnknownFieldIsPreserved) {
  ImuMeasurement sample;
  FillNominal(&sample);
  const std::string golden = sample.SerializeAsString();
  const std::string with_unknown = golden + std::string("\xA0\x06\x07", 3);
  ImuMeasurement parsed;
  ASSERT_TRUE(parsed.ParseFromString(with_unknown));
  EXPECT_DOUBLE_EQ(parsed.angular_velocity_x_rad_s(), 0.25);
  EXPECT_DOUBLE_EQ(parsed.orientation_xyzw().w(), 1.0);
  const google::protobuf::UnknownFieldSet& unknown =
      parsed.GetReflection()->GetUnknownFields(parsed);
  ASSERT_EQ(unknown.field_count(), 1);
  EXPECT_EQ(unknown.field(0).number(), 100);
  EXPECT_EQ(unknown.field(0).varint(), 7u);
  EXPECT_EQ(parsed.SerializeAsString(), with_unknown);
}

TEST(ImuSerializationTest, TextprotoExamplesMatchTheFixtures) {
  const std::string nominal_text = LoadExample("imu_nominal.textproto");
  ASSERT_FALSE(nominal_text.empty()) << "imu_nominal.textproto";
  ImuMeasurement parsed;
  ASSERT_TRUE(
      google::protobuf::TextFormat::ParseFromString(nominal_text, &parsed));
  ImuMeasurement coded;
  FillNominal(&coded);
  EXPECT_EQ(parsed.SerializeAsString(), coded.SerializeAsString());
  EXPECT_EQ(parsed.SerializeAsString(), FromHex(kNominalGoldenHex));

  const std::string rates_text = LoadExample("imu_rates_only.textproto");
  ASSERT_FALSE(rates_text.empty()) << "imu_rates_only.textproto";
  ImuMeasurement rates;
  ASSERT_TRUE(
      google::protobuf::TextFormat::ParseFromString(rates_text, &rates));
  ImuMeasurement rates_coded;
  FillRatesOnly(&rates_coded);
  EXPECT_EQ(rates.SerializeAsString(), rates_coded.SerializeAsString());
  EXPECT_EQ(rates.SerializeAsString(), FromHex(kRatesOnlyGoldenHex));
}

}  // namespace
}  // namespace intrinsic::hardware::marine
