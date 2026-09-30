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
#include "gtest/gtest.h"
#include "intrinsic/embodiment/proto/stamped_header.pb.h"
#include "intrinsic/embodiment/stamped_header_policy.h"
#include "intrinsic/hardware/marine/fake_surface_fix.h"
#include "intrinsic/hardware/marine/measurement_health.pb.h"
#include "intrinsic/hardware/marine/surface_fix.pb.h"
#include "intrinsic/hardware/marine/surface_fix_policy.h"
#include "intrinsic/vehicle/vehicle_contract_policy.h"

namespace intrinsic::hardware::marine {
namespace {

using intrinsic_proto::embodiment::Validity;
using intrinsic_proto::hardware::marine::MeasurementHealth;
using intrinsic_proto::hardware::marine::SurfaceFix;

// Canonical serialization of the default FakeSurfaceFix / FillNominal().
// Produced by a local C++ protobuf run of that fixture. Bazel was not
// available in the authoring environment. Keep in sync with
// surface_fix_serialization_test.py.
constexpr std::string_view kNominalGoldenHex =
    "0a80030a38082a120b0880e2cfaa061080e59a771a060881e2cfaa06220a6e61765f7365"
    "6e736f722a04676e737332096d6f6e6f746f6e69633a02080110011d0000403f22a3020a"
    "a002000000000000f03f0000000000000000000000000000000000000000000000000000"
    "000000000000000000000000000000000000000000000000000000001040000000000000"
    "000000000000000000000000000000000000000000000000000000000000000000000000"
    "000000000000000000000000d03f00000000000000000000000000000000000000000000"
    "000000000000000000000000000000000000000000000000000000000000000000000000"
    "000000000000000000000000000000000000000000000000000000000000000000000000"
    "000000000000000000000000000000000000000000000000000000000000000000000000"
    "000000000000000000000000000000000000000000000000000000000000000000000000"
    "00002a0d0a077072696d617279120208012a080a06616964696e67110000000000002940"
    "190000000000000ac021000000000000f03f2801300c";

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

void FillSurfaceFixCovariance(intrinsic_proto::vehicle::Matrix6* matrix) {
  matrix->mutable_values()->Resize(vehicle::kCovarianceValues, 0.0);
  matrix->set_values(kSurfacePositionXVarianceSlot, 1.0);
  matrix->set_values(kSurfacePositionYVarianceSlot, 4.0);
  matrix->set_values(kSurfacePositionZVarianceSlot, 0.25);
}

void FillNominal(SurfaceFix* sample) {
  MeasurementHealth* health = sample->mutable_health();
  auto* header = health->mutable_header();
  header->set_sequence(42);
  header->mutable_source_time()->set_seconds(1700000000);
  header->mutable_source_time()->set_nanos(250000000);
  header->mutable_receive_time()->set_seconds(1700000001);
  header->set_source_id("nav_sensor");
  header->set_frame_id("gnss");
  header->set_clock_domain(std::string(embodiment::kClockDomainMonotonic));
  header->mutable_validity()->set_state(Validity::STATE_VALID);
  health->set_state(MeasurementHealth::VALID);
  health->set_quality(0.75f);
  FillSurfaceFixCovariance(health->mutable_covariance());
  auto* primary = health->add_sources();
  primary->set_source_id("primary");
  primary->mutable_validity()->set_state(Validity::STATE_VALID);
  health->add_sources()->set_source_id("aiding");
  sample->set_position_x_m(12.5);
  sample->set_position_y_m(-3.25);
  sample->set_position_z_m(1.0);
  sample->set_source(SurfaceFix::FIX_SOURCE_GNSS);
  sample->set_satellite_count(12);
}

SurfaceFixView ViewOf(const SurfaceFix& sample, std::vector<double>* covariance,
                      std::vector<SourceHealthView>* sources) {
  SurfaceFixView view;
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
  view.source_present = sample.has_source();
  view.source = sample.source();
  view.satellite_count_present = sample.has_satellite_count();
  view.satellite_count = sample.satellite_count();
  view.beacon_count_present = sample.has_beacon_count();
  view.beacon_count = sample.beacon_count();
  view.horizontal_accuracy_present = sample.has_horizontal_accuracy_m();
  view.horizontal_accuracy_m = sample.horizontal_accuracy_m();
  view.vertical_accuracy_present = sample.has_vertical_accuracy_m();
  view.vertical_accuracy_m = sample.vertical_accuracy_m();
  view.velocity_x_present = sample.has_velocity_x_m_s();
  view.velocity_x_m_s = sample.velocity_x_m_s();
  view.velocity_y_present = sample.has_velocity_y_m_s();
  view.velocity_y_m_s = sample.velocity_y_m_s();
  view.velocity_z_present = sample.has_velocity_z_m_s();
  view.velocity_z_m_s = sample.velocity_z_m_s();
  return view;
}

SurfaceFixAssessment Assess(const SurfaceFix& sample) {
  std::vector<double> covariance;
  std::vector<SourceHealthView> sources;
  return AssessSurfaceFix(ViewOf(sample, &covariance, &sources));
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

TEST(SurfaceFixSerializationTest, ContractShapeAndUnchangedValidity) {
  const google::protobuf::Descriptor* descriptor = SurfaceFix::descriptor();
  EXPECT_EQ(descriptor->file()->package(), "intrinsic_proto.hardware.marine");
  EXPECT_EQ(descriptor->name(), "SurfaceFix");
  EXPECT_EQ(descriptor->FindFieldByName("health")->message_type()->full_name(),
            "intrinsic_proto.hardware.marine.MeasurementHealth");
  for (const char* name :
       {"position_x_m", "position_y_m", "position_z_m", "source",
        "satellite_count", "beacon_count", "horizontal_accuracy_m",
        "vertical_accuracy_m", "velocity_x_m_s", "velocity_y_m_s",
        "velocity_z_m_s"}) {
    const google::protobuf::FieldDescriptor* field =
        descriptor->FindFieldByName(name);
    ASSERT_NE(field, nullptr) << name;
    EXPECT_TRUE(field->has_presence()) << name;
  }
  EXPECT_EQ(descriptor->FindFieldByName("orientation_xyzw"), nullptr);
  EXPECT_EQ(descriptor->FindFieldByName("angular_velocity_x_rad_s"), nullptr);
  const google::protobuf::EnumDescriptor* validity =
      Validity::State_descriptor();
  ASSERT_EQ(validity->value_count(), 3);
  EXPECT_EQ(validity->FindValueByName("STATE_DEGRADED"), nullptr);
  const google::protobuf::EnumDescriptor* source =
      descriptor->FindEnumTypeByName("FixSource");
  ASSERT_NE(source, nullptr);
  EXPECT_EQ(source->value_count(), 4);
  EXPECT_EQ(source->FindValueByName("FIX_SOURCE_UNSPECIFIED")->number(), 0);
  EXPECT_EQ(source->FindValueByName("FIX_SOURCE_GNSS")->number(), 1);
  EXPECT_EQ(source->FindValueByName("FIX_SOURCE_ACOUSTIC")->number(), 2);
  EXPECT_EQ(source->FindValueByName("FIX_SOURCE_OTHER")->number(), 3);
}

TEST(SurfaceFixSerializationTest, DefaultsAreEmptyAndOptIn) {
  const SurfaceFix sample;
  EXPECT_EQ(sample.SerializeAsString(), "");
  EXPECT_FALSE(sample.has_health());
  EXPECT_FALSE(sample.has_position_x_m());
  EXPECT_FALSE(sample.has_source());
  EXPECT_FALSE(sample.has_satellite_count());
  EXPECT_FALSE(sample.has_beacon_count());
  EXPECT_FALSE(sample.has_horizontal_accuracy_m());
  EXPECT_FALSE(sample.has_vertical_accuracy_m());
  EXPECT_FALSE(sample.has_velocity_x_m_s());
}

TEST(SurfaceFixSerializationTest, NominalRoundTripMatchesGoldenAndFake) {
  SurfaceFix sample;
  FillNominal(&sample);
  const std::optional<SurfaceFix> faked = FakeSurfaceFix().Measure();
  ASSERT_TRUE(faked.has_value());
  EXPECT_EQ(sample.SerializeAsString(), faked->SerializeAsString());
  const std::string hex = ToHex(sample.SerializeAsString());
  ASSERT_EQ(hex, kNominalGoldenHex) << hex;
  SurfaceFix parsed;
  ASSERT_TRUE(parsed.ParseFromString(FromHex(kNominalGoldenHex)));
  EXPECT_EQ(parsed.SerializeAsString(), FromHex(kNominalGoldenHex));
  EXPECT_EQ(parsed.health().header().frame_id(), "gnss");
  EXPECT_DOUBLE_EQ(parsed.position_x_m(), 12.5);
  EXPECT_DOUBLE_EQ(parsed.position_y_m(), -3.25);
  EXPECT_DOUBLE_EQ(parsed.position_z_m(), 1.0);
  EXPECT_EQ(parsed.source(), SurfaceFix::FIX_SOURCE_GNSS);
  EXPECT_EQ(parsed.satellite_count(), 12);
  EXPECT_FLOAT_EQ(parsed.health().quality(), 0.75f);
  EXPECT_DOUBLE_EQ(
      parsed.health().covariance().values(kSurfacePositionXVarianceSlot), 1.0);
  EXPECT_DOUBLE_EQ(
      parsed.health().covariance().values(kSurfacePositionYVarianceSlot), 4.0);
  EXPECT_DOUBLE_EQ(
      parsed.health().covariance().values(kSurfacePositionZVarianceSlot), 0.25);
  EXPECT_TRUE(Assess(parsed).accepted);
}

TEST(SurfaceFixSerializationTest, PresenceIsDistinctFromDefaultValues) {
  SurfaceFix position;
  EXPECT_FALSE(position.has_position_x_m());
  position.set_position_x_m(0.0);
  EXPECT_TRUE(position.has_position_x_m());
  EXPECT_NE(position.SerializeAsString(), SurfaceFix().SerializeAsString());

  SurfaceFix source;
  EXPECT_FALSE(source.has_source());
  source.set_source(SurfaceFix::FIX_SOURCE_UNSPECIFIED);
  EXPECT_TRUE(source.has_source());
  EXPECT_NE(source.SerializeAsString(), SurfaceFix().SerializeAsString());

  SurfaceFix satellites;
  satellites.set_satellite_count(0);
  EXPECT_TRUE(satellites.has_satellite_count());
  EXPECT_NE(satellites.SerializeAsString(), SurfaceFix().SerializeAsString());

  SurfaceFix accuracy;
  accuracy.set_horizontal_accuracy_m(0.0);
  EXPECT_TRUE(accuracy.has_horizontal_accuracy_m());
  EXPECT_NE(accuracy.SerializeAsString(), SurfaceFix().SerializeAsString());

  SurfaceFix zeros;
  for (int i = 0; i < vehicle::kCovarianceValues; ++i) {
    zeros.mutable_health()->mutable_covariance()->add_values(0.0);
  }
  EXPECT_NE(zeros.SerializeAsString(), SurfaceFix().SerializeAsString());
}

TEST(SurfaceFixSerializationTest, ClearingSatelliteCountIsAPrefixOfTheGolden) {
  SurfaceFix sample;
  FillNominal(&sample);
  const std::string full = sample.SerializeAsString();
  sample.clear_satellite_count();
  EXPECT_TRUE(full.starts_with(sample.SerializeAsString()));
  EXPECT_NE(full, sample.SerializeAsString());
}

TEST(SurfaceFixSerializationTest, UnknownFieldAndSourceArePreserved) {
  SurfaceFix sample;
  FillNominal(&sample);
  const std::string golden = sample.SerializeAsString();
  const std::string with_unknown = golden + std::string("\xA0\x06\x07", 3);
  SurfaceFix parsed;
  ASSERT_TRUE(parsed.ParseFromString(with_unknown));
  EXPECT_EQ(parsed.source(), SurfaceFix::FIX_SOURCE_GNSS);
  const google::protobuf::UnknownFieldSet& unknown =
      parsed.GetReflection()->GetUnknownFields(parsed);
  ASSERT_EQ(unknown.field_count(), 1);
  EXPECT_EQ(unknown.field(0).number(), 100);
  EXPECT_EQ(unknown.field(0).varint(), 7u);
  EXPECT_EQ(parsed.SerializeAsString(), with_unknown);

  parsed.set_source(static_cast<SurfaceFix::FixSource>(99));
  SurfaceFix reparsed;
  ASSERT_TRUE(reparsed.ParseFromString(parsed.SerializeAsString()));
  EXPECT_EQ(static_cast<int>(reparsed.source()), 99);
  EXPECT_EQ(ClassifySurfaceFixSource(reparsed.has_source(), reparsed.source()),
            SurfaceFixSourceKind::kUnrecognized);
  const SurfaceFixAssessment assessment = Assess(reparsed);
  EXPECT_EQ(assessment.error, SurfaceFixError::kSource);
  EXPECT_FALSE(assessment.accepted);
}

TEST(SurfaceFixSerializationTest, TextprotoGnssExampleMatchesTheFixture) {
  const std::string text = LoadExample("surface_fix_gnss.textproto");
  ASSERT_FALSE(text.empty()) << "surface_fix_gnss.textproto";
  SurfaceFix parsed;
  ASSERT_TRUE(google::protobuf::TextFormat::ParseFromString(text, &parsed));
  SurfaceFix coded;
  FillNominal(&coded);
  EXPECT_EQ(parsed.SerializeAsString(), coded.SerializeAsString());
  EXPECT_EQ(parsed.SerializeAsString(), FromHex(kNominalGoldenHex));
}

TEST(SurfaceFixSerializationTest, TextprotoAcousticExampleIsAccepted) {
  const std::string text = LoadExample("surface_fix_acoustic.textproto");
  ASSERT_FALSE(text.empty()) << "surface_fix_acoustic.textproto";
  SurfaceFix parsed;
  ASSERT_TRUE(google::protobuf::TextFormat::ParseFromString(text, &parsed));
  EXPECT_EQ(parsed.source(), SurfaceFix::FIX_SOURCE_ACOUSTIC);
  EXPECT_EQ(parsed.beacon_count(), 4);
  EXPECT_FALSE(parsed.has_satellite_count());
  EXPECT_DOUBLE_EQ(parsed.horizontal_accuracy_m(), 0.5);
  EXPECT_DOUBLE_EQ(parsed.velocity_z_m_s(), -0.125);
  EXPECT_TRUE(Assess(parsed).accepted);

  SurfaceFixTruth truth;
  truth.frame_id = "acoustic_array";
  truth.source = 2;
  truth.satellite_count_present = false;
  truth.beacon_count_present = true;
  truth.beacon_count = 4;
  truth.horizontal_accuracy_present = true;
  truth.horizontal_accuracy_m = 0.5;
  truth.vertical_accuracy_present = true;
  truth.vertical_accuracy_m = 1.0;
  truth.velocity_present = true;
  truth.velocity_x_m_s = 0.5;
  truth.velocity_y_m_s = 0.0;
  truth.velocity_z_m_s = -0.125;
  const std::optional<SurfaceFix> faked = FakeSurfaceFix().Measure(truth);
  ASSERT_TRUE(faked.has_value());
  EXPECT_EQ(parsed.SerializeAsString(), faked->SerializeAsString());
}

}  // namespace
}  // namespace intrinsic::hardware::marine
