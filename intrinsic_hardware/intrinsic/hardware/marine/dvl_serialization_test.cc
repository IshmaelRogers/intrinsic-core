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
#include "intrinsic/hardware/marine/dvl.pb.h"
#include "intrinsic/hardware/marine/dvl_policy.h"
#include "intrinsic/hardware/marine/fake_dvl.h"
#include "intrinsic/vehicle/vehicle_contract_policy.h"

namespace intrinsic::hardware::marine {
namespace {

using intrinsic_proto::embodiment::Validity;
using intrinsic_proto::hardware::marine::DvlMeasurement;
using intrinsic_proto::hardware::marine::MeasurementHealth;

// Canonical serialization of the default FakeDvl / FillNominal(). Keep in
// sync with dvl_serialization_test.py.
constexpr std::string_view kNominalGoldenHex =
    "0a82030a3a082a120b0880e2cfaa061080e59a771a060881e2cfaa06220a6e61765f7365"
    "6e736f722a0673656e736f7232096d6f6e6f746f6e69633a02080110011d0000403f22a3"
    "020aa002000000000000d03f000000000000000000000000000000000000000000000000"
    "000000000000000000000000000000000000000000000000000000000000d03f00000000"
    "000000000000000000000000000000000000000000000000000000000000000000000000"
    "0000000000000000000000000000d03f0000000000000000000000000000000000000000"
    "000000000000000000000000000000000000000000000000000000000000000000000000"
    "000000000000000000000000000000000000000000000000000000000000000000000000"
    "000000000000000000000000000000000000000000000000000000000000000000000000"
    "000000000000000000000000000000000000000000000000000000000000000000000000"
    "000000002a0d0a077072696d617279120208012a080a06616964696e6710011900000000"
    "0000e03f21000000000000d0bf2900000000000000003001390000000000002440";

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

void FillLinearCovariance(intrinsic_proto::vehicle::Matrix6* matrix) {
  matrix->mutable_values()->Resize(vehicle::kCovarianceValues, 0.0);
  matrix->set_values(vehicle::CovarianceIndex(0, 0), 0.25);
  matrix->set_values(vehicle::CovarianceIndex(1, 1), 0.25);
  matrix->set_values(vehicle::CovarianceIndex(2, 2), 0.25);
}

void FillNominal(DvlMeasurement* sample) {
  MeasurementHealth* health = sample->mutable_health();
  auto* header = health->mutable_header();
  header->set_sequence(42);
  header->mutable_source_time()->set_seconds(1700000000);
  header->mutable_source_time()->set_nanos(250000000);
  header->mutable_receive_time()->set_seconds(1700000001);
  header->set_source_id("nav_sensor");
  header->set_frame_id("sensor");
  header->set_clock_domain(std::string(embodiment::kClockDomainMonotonic));
  header->mutable_validity()->set_state(Validity::STATE_VALID);
  health->set_state(MeasurementHealth::VALID);
  health->set_quality(0.75f);
  FillLinearCovariance(health->mutable_covariance());
  auto* primary = health->add_sources();
  primary->set_source_id("primary");
  primary->mutable_validity()->set_state(Validity::STATE_VALID);
  health->add_sources()->set_source_id("aiding");
  sample->set_mode(DvlMeasurement::MODE_BOTTOM_TRACK);
  sample->set_velocity_x_m_s(0.5);
  sample->set_velocity_y_m_s(-0.25);
  sample->set_velocity_z_m_s(0.0);
  sample->set_bottom_lock(true);
  sample->set_altitude_m(10.0);
}

DvlMeasurementView ViewOf(const DvlMeasurement& sample,
                          std::vector<double>* covariance,
                          std::vector<SourceHealthView>* sources) {
  DvlMeasurementView view;
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
  view.mode_present = sample.has_mode();
  view.mode = sample.mode();
  view.velocity_x_present = sample.has_velocity_x_m_s();
  view.velocity_x_m_s = sample.velocity_x_m_s();
  view.velocity_y_present = sample.has_velocity_y_m_s();
  view.velocity_y_m_s = sample.velocity_y_m_s();
  view.velocity_z_present = sample.has_velocity_z_m_s();
  view.velocity_z_m_s = sample.velocity_z_m_s();
  view.bottom_lock_present = sample.has_bottom_lock();
  view.bottom_lock = sample.bottom_lock();
  view.altitude_present = sample.has_altitude_m();
  view.altitude_m = sample.altitude_m();
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

TEST(DvlSerializationTest, ContractShapeAndUnchangedValidity) {
  const google::protobuf::Descriptor* descriptor = DvlMeasurement::descriptor();
  EXPECT_EQ(descriptor->file()->package(), "intrinsic_proto.hardware.marine");
  EXPECT_EQ(descriptor->FindFieldByName("health")->message_type()->full_name(),
            "intrinsic_proto.hardware.marine.MeasurementHealth");
  EXPECT_TRUE(descriptor->FindFieldByName("mode")->has_presence());
  EXPECT_TRUE(descriptor->FindFieldByName("velocity_x_m_s")->has_presence());
  EXPECT_TRUE(descriptor->FindFieldByName("velocity_y_m_s")->has_presence());
  EXPECT_TRUE(descriptor->FindFieldByName("velocity_z_m_s")->has_presence());
  EXPECT_TRUE(descriptor->FindFieldByName("bottom_lock")->has_presence());
  EXPECT_TRUE(descriptor->FindFieldByName("altitude_m")->has_presence());
  EXPECT_EQ(DvlMeasurement::MODE_UNSPECIFIED, 0);
  EXPECT_EQ(DvlMeasurement::MODE_BOTTOM_TRACK, 1);
  EXPECT_EQ(DvlMeasurement::MODE_WATER_TRACK, 2);
  const google::protobuf::EnumDescriptor* validity =
      Validity::State_descriptor();
  ASSERT_EQ(validity->value_count(), 3);
  EXPECT_EQ(validity->FindValueByName("STATE_DEGRADED"), nullptr);
  EXPECT_EQ(validity->FindValueByName("DEGRADED"), nullptr);
}

TEST(DvlSerializationTest, DefaultsAreEmptyAndOptIn) {
  EXPECT_EQ(DvlMeasurement().SerializeAsString(), "");
  EXPECT_FALSE(DvlMeasurement().has_health());
  EXPECT_FALSE(DvlMeasurement().has_mode());
  EXPECT_FALSE(DvlMeasurement().has_velocity_x_m_s());
  EXPECT_FALSE(DvlMeasurement().has_bottom_lock());
  EXPECT_FALSE(DvlMeasurement().has_altitude_m());
}

TEST(DvlSerializationTest, NominalRoundTripMatchesGoldenAndFake) {
  DvlMeasurement sample;
  FillNominal(&sample);
  const std::optional<DvlMeasurement> faked = FakeDvl().Measure();
  ASSERT_TRUE(faked.has_value());
  EXPECT_EQ(sample.SerializeAsString(), faked->SerializeAsString());
  const std::string hex = ToHex(sample.SerializeAsString());
  ASSERT_EQ(hex, kNominalGoldenHex) << hex;
  DvlMeasurement parsed;
  ASSERT_TRUE(parsed.ParseFromString(FromHex(kNominalGoldenHex)));
  EXPECT_EQ(parsed.SerializeAsString(), FromHex(kNominalGoldenHex));
  EXPECT_EQ(parsed.health().header().frame_id(), "sensor");
  EXPECT_EQ(parsed.health().header().sequence(), 42u);
  EXPECT_EQ(parsed.mode(), DvlMeasurement::MODE_BOTTOM_TRACK);
  EXPECT_DOUBLE_EQ(parsed.velocity_x_m_s(), 0.5);
  EXPECT_DOUBLE_EQ(parsed.velocity_y_m_s(), -0.25);
  EXPECT_DOUBLE_EQ(parsed.velocity_z_m_s(), 0.0);
  EXPECT_TRUE(parsed.bottom_lock());
  EXPECT_DOUBLE_EQ(parsed.altitude_m(), 10.0);
  EXPECT_EQ(parsed.health().covariance().values_size(),
            vehicle::kCovarianceValues);
  EXPECT_DOUBLE_EQ(parsed.health().covariance().values(21), 0.0);
  EXPECT_DOUBLE_EQ(parsed.health().covariance().values(28), 0.0);
  EXPECT_DOUBLE_EQ(parsed.health().covariance().values(35), 0.0);

  std::vector<double> covariance;
  std::vector<SourceHealthView> sources;
  const DvlAssessment assessment =
      AssessDvl(ViewOf(parsed, &covariance, &sources));
  EXPECT_TRUE(assessment.accepted);
  EXPECT_EQ(assessment.mode, DvlModeKind::kBottomTrack);
}

TEST(DvlSerializationTest, PresenceIsDistinctFromDefaultValues) {
  DvlMeasurement unset;
  DvlMeasurement specified;
  specified.set_mode(DvlMeasurement::MODE_UNSPECIFIED);
  EXPECT_FALSE(unset.has_mode());
  EXPECT_TRUE(specified.has_mode());
  EXPECT_NE(unset.SerializeAsString(), specified.SerializeAsString());

  DvlMeasurement locked;
  EXPECT_FALSE(locked.has_bottom_lock());
  locked.set_bottom_lock(false);
  EXPECT_TRUE(locked.has_bottom_lock());
  const std::string with_false = locked.SerializeAsString();
  locked.clear_bottom_lock();
  EXPECT_NE(locked.SerializeAsString(), with_false);

  DvlMeasurement altitude;
  EXPECT_FALSE(altitude.has_altitude_m());
  altitude.set_altitude_m(0.0);
  EXPECT_TRUE(altitude.has_altitude_m());
  EXPECT_NE(altitude.SerializeAsString(), DvlMeasurement().SerializeAsString());

  DvlMeasurement zeros;
  for (int i = 0; i < vehicle::kCovarianceValues; ++i) {
    zeros.mutable_health()->mutable_covariance()->add_values(0.0);
  }
  EXPECT_NE(zeros.SerializeAsString(), DvlMeasurement().SerializeAsString());
}

TEST(DvlSerializationTest, ClearingAltitudeIsAPrefixOfTheGolden) {
  DvlMeasurement sample;
  FillNominal(&sample);
  const std::string full = sample.SerializeAsString();
  sample.clear_altitude_m();
  EXPECT_TRUE(full.starts_with(sample.SerializeAsString()));
  EXPECT_NE(full, sample.SerializeAsString());
}

TEST(DvlSerializationTest, UnknownFieldAndModeArePreserved) {
  DvlMeasurement sample;
  FillNominal(&sample);
  const std::string golden = sample.SerializeAsString();
  const std::string with_unknown = golden + std::string("\xA0\x06\x07", 3);
  DvlMeasurement parsed;
  ASSERT_TRUE(parsed.ParseFromString(with_unknown));
  EXPECT_EQ(parsed.mode(), DvlMeasurement::MODE_BOTTOM_TRACK);
  const google::protobuf::UnknownFieldSet& unknown =
      parsed.GetReflection()->GetUnknownFields(parsed);
  ASSERT_EQ(unknown.field_count(), 1);
  EXPECT_EQ(unknown.field(0).number(), 100);
  EXPECT_EQ(unknown.field(0).varint(), 7u);
  EXPECT_EQ(parsed.SerializeAsString(), with_unknown);

  parsed.set_mode(static_cast<DvlMeasurement::Mode>(100));
  DvlMeasurement reparsed;
  ASSERT_TRUE(reparsed.ParseFromString(parsed.SerializeAsString()));
  EXPECT_EQ(static_cast<int>(reparsed.mode()), 100);
  EXPECT_EQ(ClassifyDvlMode(reparsed.has_mode(), reparsed.mode()),
            DvlModeKind::kUnrecognized);
}

TEST(DvlSerializationTest, TextprotoExampleMatchesTheFixture) {
  const std::string text = LoadExample("dvl_bottom_track.textproto");
  ASSERT_FALSE(text.empty()) << "dvl_bottom_track.textproto";
  DvlMeasurement parsed;
  ASSERT_TRUE(google::protobuf::TextFormat::ParseFromString(text, &parsed));
  DvlMeasurement coded;
  FillNominal(&coded);
  EXPECT_EQ(parsed.SerializeAsString(), coded.SerializeAsString());
  EXPECT_EQ(parsed.SerializeAsString(), FromHex(kNominalGoldenHex));
}

}  // namespace
}  // namespace intrinsic::hardware::marine
