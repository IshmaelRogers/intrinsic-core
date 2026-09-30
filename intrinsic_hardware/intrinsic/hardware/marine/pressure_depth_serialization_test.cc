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
#include "intrinsic/hardware/marine/fake_pressure_depth.h"
#include "intrinsic/hardware/marine/pressure_depth.pb.h"
#include "intrinsic/hardware/marine/pressure_depth_policy.h"
#include "intrinsic/vehicle/vehicle_contract_policy.h"

namespace intrinsic::hardware::marine {
namespace {

using intrinsic_proto::embodiment::Validity;
using intrinsic_proto::hardware::marine::MeasurementHealth;
using intrinsic_proto::hardware::marine::PressureDepthMeasurement;

// Canonical serialization of the default FakePressureDepth / FillNominal().
// Keep in sync with pressure_depth_serialization_test.py.
constexpr std::string_view kNominalGoldenHex =
    "0a82030a3a082a120b0880e2cfaa061080e59a771a060881e2cfaa06220a6e61765f7365"
    "6e736f722a0673656e736f7232096d6f6e6f746f6e69633a02080110011d0000403f22a3"
    "020aa002000000000000f03f000000000000000000000000000000000000000000000000"
    "000000000000000000000000000000000000000000000000000000000000d03f00000000"
    "000000000000000000000000000000000000000000000000000000000000000000000000"
    "000000000000000000000000000000000000000000000000000000000000000000000000"
    "000000000000000000000000000000000000000000000000000000000000000000000000"
    "000000000000000000000000000000000000000000000000000000000000000000000000"
    "000000000000000000000000000000000000000000000000000000000000000000000000"
    "000000000000000000000000000000000000000000000000000000000000000000000000"
    "000000002a0d0a077072696d617279120208012a080a06616964696e671100000000006a"
    "08411900000000000024402002290000000000049040";

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

void FillPressureDepthCovariance(intrinsic_proto::vehicle::Matrix6* matrix) {
  matrix->mutable_values()->Resize(vehicle::kCovarianceValues, 0.0);
  matrix->set_values(kPressureVarianceSlot, 1.0);
  matrix->set_values(kDepthVarianceSlot, 0.25);
}

void FillNominal(PressureDepthMeasurement* sample) {
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
  FillPressureDepthCovariance(health->mutable_covariance());
  auto* primary = health->add_sources();
  primary->set_source_id("primary");
  primary->mutable_validity()->set_state(Validity::STATE_VALID);
  health->add_sources()->set_source_id("aiding");
  sample->set_pressure_pa(200000.0);
  sample->set_depth_m(10.0);
  sample->set_depth_provenance(
      PressureDepthMeasurement::DEPTH_PROVENANCE_FROM_PRESSURE);
  sample->set_fluid_density_kg_m3(1025.0);
}

PressureDepthMeasurementView ViewOf(const PressureDepthMeasurement& sample,
                                    std::vector<double>* covariance,
                                    std::vector<SourceHealthView>* sources) {
  PressureDepthMeasurementView view;
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
  view.pressure_present = sample.has_pressure_pa();
  view.pressure_pa = sample.pressure_pa();
  view.depth_present = sample.has_depth_m();
  view.depth_m = sample.depth_m();
  view.provenance_present = sample.has_depth_provenance();
  view.depth_provenance = sample.depth_provenance();
  view.density_present = sample.has_fluid_density_kg_m3();
  view.fluid_density_kg_m3 = sample.fluid_density_kg_m3();
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

TEST(PressureDepthSerializationTest, ContractShapeAndUnchangedValidity) {
  const google::protobuf::Descriptor* descriptor =
      PressureDepthMeasurement::descriptor();
  EXPECT_EQ(descriptor->file()->package(), "intrinsic_proto.hardware.marine");
  EXPECT_EQ(descriptor->FindFieldByName("health")->message_type()->full_name(),
            "intrinsic_proto.hardware.marine.MeasurementHealth");
  EXPECT_TRUE(descriptor->FindFieldByName("pressure_pa")->has_presence());
  EXPECT_TRUE(descriptor->FindFieldByName("depth_m")->has_presence());
  EXPECT_TRUE(descriptor->FindFieldByName("depth_provenance")->has_presence());
  EXPECT_TRUE(
      descriptor->FindFieldByName("fluid_density_kg_m3")->has_presence());
  EXPECT_EQ(descriptor->FindFieldByName("altitude_m"), nullptr);
  EXPECT_EQ(PressureDepthMeasurement::DEPTH_PROVENANCE_UNSPECIFIED, 0);
  EXPECT_EQ(PressureDepthMeasurement::DEPTH_PROVENANCE_DIRECT, 1);
  EXPECT_EQ(PressureDepthMeasurement::DEPTH_PROVENANCE_FROM_PRESSURE, 2);
  const google::protobuf::EnumDescriptor* validity =
      Validity::State_descriptor();
  ASSERT_EQ(validity->value_count(), 3);
  EXPECT_EQ(validity->FindValueByName("STATE_DEGRADED"), nullptr);
  EXPECT_EQ(validity->FindValueByName("DEGRADED"), nullptr);
}

TEST(PressureDepthSerializationTest, DefaultsAreEmptyAndOptIn) {
  EXPECT_EQ(PressureDepthMeasurement().SerializeAsString(), "");
  EXPECT_FALSE(PressureDepthMeasurement().has_health());
  EXPECT_FALSE(PressureDepthMeasurement().has_pressure_pa());
  EXPECT_FALSE(PressureDepthMeasurement().has_depth_m());
  EXPECT_FALSE(PressureDepthMeasurement().has_depth_provenance());
  EXPECT_FALSE(PressureDepthMeasurement().has_fluid_density_kg_m3());
}

TEST(PressureDepthSerializationTest, NominalRoundTripMatchesGoldenAndFake) {
  PressureDepthMeasurement sample;
  FillNominal(&sample);
  const std::optional<PressureDepthMeasurement> faked =
      FakePressureDepth().Measure();
  ASSERT_TRUE(faked.has_value());
  EXPECT_EQ(sample.SerializeAsString(), faked->SerializeAsString());
  const std::string hex = ToHex(sample.SerializeAsString());
  ASSERT_EQ(hex, kNominalGoldenHex) << hex;
  PressureDepthMeasurement parsed;
  ASSERT_TRUE(parsed.ParseFromString(FromHex(kNominalGoldenHex)));
  EXPECT_EQ(parsed.SerializeAsString(), FromHex(kNominalGoldenHex));
  EXPECT_EQ(parsed.health().header().frame_id(), "sensor");
  EXPECT_EQ(parsed.health().header().sequence(), 42u);
  EXPECT_DOUBLE_EQ(parsed.pressure_pa(), 200000.0);
  EXPECT_DOUBLE_EQ(parsed.depth_m(), 10.0);
  EXPECT_EQ(parsed.depth_provenance(),
            PressureDepthMeasurement::DEPTH_PROVENANCE_FROM_PRESSURE);
  EXPECT_DOUBLE_EQ(parsed.fluid_density_kg_m3(), 1025.0);
  EXPECT_EQ(parsed.health().covariance().values_size(),
            vehicle::kCovarianceValues);
  EXPECT_DOUBLE_EQ(parsed.health().covariance().values(kPressureVarianceSlot),
                   1.0);
  EXPECT_DOUBLE_EQ(parsed.health().covariance().values(kDepthVarianceSlot),
                   0.25);
  EXPECT_DOUBLE_EQ(parsed.health().covariance().values(14), 0.0);

  std::vector<double> covariance;
  std::vector<SourceHealthView> sources;
  const PressureDepthAssessment assessment =
      AssessPressureDepth(ViewOf(parsed, &covariance, &sources));
  EXPECT_TRUE(assessment.accepted);
  EXPECT_EQ(assessment.provenance, DepthProvenanceKind::kFromPressure);
}

TEST(PressureDepthSerializationTest, PresenceIsDistinctFromDefaultValues) {
  PressureDepthMeasurement unset;
  PressureDepthMeasurement specified;
  specified.set_depth_provenance(
      PressureDepthMeasurement::DEPTH_PROVENANCE_UNSPECIFIED);
  EXPECT_FALSE(unset.has_depth_provenance());
  EXPECT_TRUE(specified.has_depth_provenance());
  EXPECT_NE(unset.SerializeAsString(), specified.SerializeAsString());

  PressureDepthMeasurement pressure;
  EXPECT_FALSE(pressure.has_pressure_pa());
  pressure.set_pressure_pa(0.0);
  EXPECT_TRUE(pressure.has_pressure_pa());
  EXPECT_NE(pressure.SerializeAsString(),
            PressureDepthMeasurement().SerializeAsString());

  PressureDepthMeasurement depth;
  depth.set_depth_m(0.0);
  EXPECT_TRUE(depth.has_depth_m());
  EXPECT_NE(depth.SerializeAsString(),
            PressureDepthMeasurement().SerializeAsString());

  PressureDepthMeasurement density;
  density.set_fluid_density_kg_m3(0.0);
  EXPECT_TRUE(density.has_fluid_density_kg_m3());
  EXPECT_NE(density.SerializeAsString(),
            PressureDepthMeasurement().SerializeAsString());

  PressureDepthMeasurement zeros;
  for (int i = 0; i < vehicle::kCovarianceValues; ++i) {
    zeros.mutable_health()->mutable_covariance()->add_values(0.0);
  }
  EXPECT_NE(zeros.SerializeAsString(),
            PressureDepthMeasurement().SerializeAsString());
}

TEST(PressureDepthSerializationTest, ClearingDensityIsAPrefixOfTheGolden) {
  PressureDepthMeasurement sample;
  FillNominal(&sample);
  const std::string full = sample.SerializeAsString();
  sample.clear_fluid_density_kg_m3();
  EXPECT_TRUE(full.starts_with(sample.SerializeAsString()));
  EXPECT_NE(full, sample.SerializeAsString());
}

TEST(PressureDepthSerializationTest, UnknownFieldAndProvenanceArePreserved) {
  PressureDepthMeasurement sample;
  FillNominal(&sample);
  const std::string golden = sample.SerializeAsString();
  const std::string with_unknown = golden + std::string("\xA0\x06\x07", 3);
  PressureDepthMeasurement parsed;
  ASSERT_TRUE(parsed.ParseFromString(with_unknown));
  EXPECT_EQ(parsed.depth_provenance(),
            PressureDepthMeasurement::DEPTH_PROVENANCE_FROM_PRESSURE);
  const google::protobuf::UnknownFieldSet& unknown =
      parsed.GetReflection()->GetUnknownFields(parsed);
  ASSERT_EQ(unknown.field_count(), 1);
  EXPECT_EQ(unknown.field(0).number(), 100);
  EXPECT_EQ(unknown.field(0).varint(), 7u);
  EXPECT_EQ(parsed.SerializeAsString(), with_unknown);

  parsed.set_depth_provenance(
      static_cast<PressureDepthMeasurement::DepthProvenance>(100));
  PressureDepthMeasurement reparsed;
  ASSERT_TRUE(reparsed.ParseFromString(parsed.SerializeAsString()));
  EXPECT_EQ(static_cast<int>(reparsed.depth_provenance()), 100);
  EXPECT_EQ(ClassifyDepthProvenance(reparsed.has_depth_provenance(),
                                    reparsed.depth_provenance()),
            DepthProvenanceKind::kUnrecognized);
}

TEST(PressureDepthSerializationTest, TextprotoExampleMatchesTheFixture) {
  const std::string text =
      LoadExample("pressure_depth_from_pressure.textproto");
  ASSERT_FALSE(text.empty()) << "pressure_depth_from_pressure.textproto";
  PressureDepthMeasurement parsed;
  ASSERT_TRUE(google::protobuf::TextFormat::ParseFromString(text, &parsed));
  PressureDepthMeasurement coded;
  FillNominal(&coded);
  EXPECT_EQ(parsed.SerializeAsString(), coded.SerializeAsString());
  EXPECT_EQ(parsed.SerializeAsString(), FromHex(kNominalGoldenHex));
}

}  // namespace
}  // namespace intrinsic::hardware::marine
