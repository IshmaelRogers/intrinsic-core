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
#include "intrinsic/hardware/marine/altimeter.pb.h"
#include "intrinsic/hardware/marine/altimeter_policy.h"
#include "intrinsic/hardware/marine/fake_altimeter.h"
#include "intrinsic/vehicle/vehicle_contract_policy.h"

namespace intrinsic::hardware::marine {
namespace {

using intrinsic_proto::embodiment::Validity;
using intrinsic_proto::hardware::marine::AltimeterMeasurement;
using intrinsic_proto::hardware::marine::MeasurementHealth;

// Canonical serialization of the default FakeAltimeter / FillNominal().
// Keep in sync with altimeter_serialization_test.py.
constexpr std::string_view kNominalGoldenHex =
    "0a82030a3a082a120b0880e2cfaa061080e59a771a060881e2cfaa06220a6e61765f7365"
    "6e736f722a0673656e736f7232096d6f6e6f746f6e69633a02080110011d0000403f22a3"
    "020aa002000000000000d03f000000000000000000000000000000000000000000000000"
    "000000000000000000000000000000000000000000000000000000000000000000000000"
    "000000000000000000000000000000000000000000000000000000000000000000000000"
    "000000000000000000000000000000000000000000000000000000000000000000000000"
    "000000000000000000000000000000000000000000000000000000000000000000000000"
    "000000000000000000000000000000000000000000000000000000000000000000000000"
    "000000000000000000000000000000000000000000000000000000000000000000000000"
    "000000000000000000000000000000000000000000000000000000000000000000000000"
    "000000002a0d0a077072696d617279120208012a080a06616964696e6711000000000000"
    "24401a04646f776e21000000000000e03f2900000000000059403001";

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

void FillRangeCovariance(intrinsic_proto::vehicle::Matrix6* matrix) {
  matrix->mutable_values()->Resize(vehicle::kCovarianceValues, 0.0);
  matrix->set_values(kRangeVarianceSlot, 0.25);
}

void FillNominal(AltimeterMeasurement* sample) {
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
  FillRangeCovariance(health->mutable_covariance());
  auto* primary = health->add_sources();
  primary->set_source_id("primary");
  primary->mutable_validity()->set_state(Validity::STATE_VALID);
  health->add_sources()->set_source_id("aiding");
  sample->set_range_m(10.0);
  sample->set_beam_id("down");
  sample->set_min_range_m(0.5);
  sample->set_max_range_m(100.0);
  sample->set_has_return(true);
}

AltimeterMeasurementView ViewOf(const AltimeterMeasurement& sample,
                                std::vector<double>* covariance,
                                std::vector<SourceHealthView>* sources) {
  AltimeterMeasurementView view;
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
  view.range_present = sample.has_range_m();
  view.range_m = sample.range_m();
  view.beam_present = sample.has_beam_id();
  view.beam_id = sample.beam_id();
  view.min_present = sample.has_min_range_m();
  view.min_range_m = sample.min_range_m();
  view.max_present = sample.has_max_range_m();
  view.max_range_m = sample.max_range_m();
  view.has_return_present = sample.has_has_return();
  view.has_return = sample.has_return();
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

TEST(AltimeterSerializationTest, ContractShapeAndUnchangedValidity) {
  const google::protobuf::Descriptor* descriptor =
      AltimeterMeasurement::descriptor();
  EXPECT_EQ(descriptor->file()->package(), "intrinsic_proto.hardware.marine");
  EXPECT_EQ(descriptor->FindFieldByName("health")->message_type()->full_name(),
            "intrinsic_proto.hardware.marine.MeasurementHealth");
  EXPECT_TRUE(descriptor->FindFieldByName("range_m")->has_presence());
  EXPECT_TRUE(descriptor->FindFieldByName("beam_id")->has_presence());
  EXPECT_TRUE(descriptor->FindFieldByName("min_range_m")->has_presence());
  EXPECT_TRUE(descriptor->FindFieldByName("max_range_m")->has_presence());
  EXPECT_TRUE(descriptor->FindFieldByName("has_return")->has_presence());
  EXPECT_EQ(descriptor->FindFieldByName("altitude_m"), nullptr);
  const google::protobuf::EnumDescriptor* validity =
      Validity::State_descriptor();
  ASSERT_EQ(validity->value_count(), 3);
  EXPECT_EQ(validity->FindValueByName("STATE_DEGRADED"), nullptr);
  EXPECT_EQ(validity->FindValueByName("DEGRADED"), nullptr);
}

TEST(AltimeterSerializationTest, DefaultsAreEmptyAndOptIn) {
  EXPECT_EQ(AltimeterMeasurement().SerializeAsString(), "");
  EXPECT_FALSE(AltimeterMeasurement().has_health());
  EXPECT_FALSE(AltimeterMeasurement().has_range_m());
  EXPECT_FALSE(AltimeterMeasurement().has_beam_id());
  EXPECT_FALSE(AltimeterMeasurement().has_min_range_m());
  EXPECT_FALSE(AltimeterMeasurement().has_max_range_m());
  EXPECT_FALSE(AltimeterMeasurement().has_has_return());
}

TEST(AltimeterSerializationTest, NominalRoundTripMatchesGoldenAndFake) {
  AltimeterMeasurement sample;
  FillNominal(&sample);
  const std::optional<AltimeterMeasurement> faked = FakeAltimeter().Measure();
  ASSERT_TRUE(faked.has_value());
  EXPECT_EQ(sample.SerializeAsString(), faked->SerializeAsString());
  const std::string hex = ToHex(sample.SerializeAsString());
  ASSERT_EQ(hex, kNominalGoldenHex) << hex;
  AltimeterMeasurement parsed;
  ASSERT_TRUE(parsed.ParseFromString(FromHex(kNominalGoldenHex)));
  EXPECT_EQ(parsed.SerializeAsString(), FromHex(kNominalGoldenHex));
  EXPECT_EQ(parsed.health().header().frame_id(), "sensor");
  EXPECT_EQ(parsed.health().header().sequence(), 42u);
  EXPECT_DOUBLE_EQ(parsed.range_m(), 10.0);
  EXPECT_EQ(parsed.beam_id(), "down");
  EXPECT_DOUBLE_EQ(parsed.min_range_m(), 0.5);
  EXPECT_DOUBLE_EQ(parsed.max_range_m(), 100.0);
  EXPECT_TRUE(parsed.has_return());
  EXPECT_EQ(parsed.health().covariance().values_size(),
            vehicle::kCovarianceValues);
  EXPECT_DOUBLE_EQ(parsed.health().covariance().values(kRangeVarianceSlot),
                   0.25);
  EXPECT_DOUBLE_EQ(parsed.health().covariance().values(7), 0.0);

  std::vector<double> covariance;
  std::vector<SourceHealthView> sources;
  const AltimeterAssessment assessment =
      AssessAltimeter(ViewOf(parsed, &covariance, &sources));
  EXPECT_TRUE(assessment.accepted);
}

TEST(AltimeterSerializationTest, PresenceIsDistinctFromDefaultValues) {
  AltimeterMeasurement range;
  EXPECT_FALSE(range.has_range_m());
  range.set_range_m(0.0);
  EXPECT_TRUE(range.has_range_m());
  EXPECT_NE(range.SerializeAsString(),
            AltimeterMeasurement().SerializeAsString());

  AltimeterMeasurement beam;
  beam.set_beam_id("");
  EXPECT_TRUE(beam.has_beam_id());
  EXPECT_NE(beam.SerializeAsString(),
            AltimeterMeasurement().SerializeAsString());

  AltimeterMeasurement no_return;
  EXPECT_FALSE(no_return.has_has_return());
  no_return.set_has_return(false);
  EXPECT_TRUE(no_return.has_has_return());
  EXPECT_FALSE(no_return.has_return());
  EXPECT_NE(no_return.SerializeAsString(),
            AltimeterMeasurement().SerializeAsString());

  AltimeterMeasurement zeros;
  for (int i = 0; i < vehicle::kCovarianceValues; ++i) {
    zeros.mutable_health()->mutable_covariance()->add_values(0.0);
  }
  EXPECT_NE(zeros.SerializeAsString(),
            AltimeterMeasurement().SerializeAsString());
}

TEST(AltimeterSerializationTest, ClearingHasReturnIsAPrefixOfTheGolden) {
  AltimeterMeasurement sample;
  FillNominal(&sample);
  const std::string full = sample.SerializeAsString();
  sample.clear_has_return();
  EXPECT_TRUE(full.starts_with(sample.SerializeAsString()));
  EXPECT_NE(full, sample.SerializeAsString());
}

TEST(AltimeterSerializationTest, UnknownFieldIsPreserved) {
  AltimeterMeasurement sample;
  FillNominal(&sample);
  const std::string golden = sample.SerializeAsString();
  const std::string with_unknown = golden + std::string("\xA0\x06\x07", 3);
  AltimeterMeasurement parsed;
  ASSERT_TRUE(parsed.ParseFromString(with_unknown));
  EXPECT_DOUBLE_EQ(parsed.range_m(), 10.0);
  EXPECT_TRUE(parsed.has_return());
  const google::protobuf::UnknownFieldSet& unknown =
      parsed.GetReflection()->GetUnknownFields(parsed);
  ASSERT_EQ(unknown.field_count(), 1);
  EXPECT_EQ(unknown.field(0).number(), 100);
  EXPECT_EQ(unknown.field(0).varint(), 7u);
  EXPECT_EQ(parsed.SerializeAsString(), with_unknown);
}

TEST(AltimeterSerializationTest, TextprotoExampleMatchesTheFixture) {
  const std::string text = LoadExample("altimeter_nominal.textproto");
  ASSERT_FALSE(text.empty()) << "altimeter_nominal.textproto";
  AltimeterMeasurement parsed;
  ASSERT_TRUE(google::protobuf::TextFormat::ParseFromString(text, &parsed));
  AltimeterMeasurement coded;
  FillNominal(&coded);
  EXPECT_EQ(parsed.SerializeAsString(), coded.SerializeAsString());
  EXPECT_EQ(parsed.SerializeAsString(), FromHex(kNominalGoldenHex));
}

}  // namespace
}  // namespace intrinsic::hardware::marine
