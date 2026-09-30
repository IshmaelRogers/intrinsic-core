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
#include "intrinsic/embodiment/frame_policy.h"
#include "intrinsic/embodiment/proto/stamped_header.pb.h"
#include "intrinsic/embodiment/stamped_header_policy.h"
#include "intrinsic/hardware/marine/measurement_health.pb.h"
#include "intrinsic/hardware/marine/measurement_health_policy.h"
#include "intrinsic/vehicle/vehicle_contract_policy.h"

namespace intrinsic::hardware::marine {
namespace {

using intrinsic_proto::embodiment::Validity;
using intrinsic_proto::hardware::marine::MeasurementHealth;

// Canonical serialization of FillNominal(). Keep in sync with
// measurement_health_serialization_test.py.
constexpr std::string_view kNominalGoldenHex =
    "0a3a082a120b0880e2cfaa061080e59a771a060881e2cfaa06220a6e61765f73656e736f"
    "722a0673656e736f7232096d6f6e6f746f6e69633a02080110011d0000403f22a3020aa0"
    "02000000000000d03f000000000000000000000000000000000000000000000000000000"
    "000000000000000000000000000000000000000000000000000000d03f00000000000000"
    "000000000000000000000000000000000000000000000000000000000000000000000000"
    "0000000000000000000000d03f0000000000000000000000000000000000000000000000"
    "00000000000000000000000000000000000000000000000000000000000000b03f000000"
    "000000000000000000000000000000000000000000000000000000000000000000000000"
    "000000000000000000000000000000b03f00000000000000000000000000000000000000"
    "0000000000000000000000000000000000000000000000000000000000000000000000b0"
    "3f2a0d0a077072696d617279120208012a080a06616964696e67";

std::array<double, vehicle::kCovarianceValues> NominalCovariance() {
  std::array<double, vehicle::kCovarianceValues> values = {};
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

void FillHeader(intrinsic_proto::embodiment::StampedHeader* header) {
  header->set_sequence(42);
  header->mutable_source_time()->set_seconds(1700000000);
  header->mutable_source_time()->set_nanos(250000000);
  header->mutable_receive_time()->set_seconds(1700000001);
  header->set_source_id("nav_sensor");
  header->set_frame_id("sensor");
  header->set_clock_domain(std::string(embodiment::kClockDomainMonotonic));
  header->mutable_validity()->set_state(Validity::STATE_VALID);
}

void FillNominal(MeasurementHealth* sample) {
  FillHeader(sample->mutable_header());
  sample->set_state(MeasurementHealth::VALID);
  sample->set_quality(0.75f);
  for (double value : NominalCovariance()) {
    sample->mutable_covariance()->add_values(value);
  }
  auto* primary = sample->add_sources();
  primary->set_source_id("primary");
  primary->mutable_validity()->set_state(Validity::STATE_VALID);
  sample->add_sources()->set_source_id("aiding");
}

std::vector<double> CopyDoubles(
    const google::protobuf::RepeatedField<double>& field) {
  return std::vector<double>(field.begin(), field.end());
}

MeasurementHealthView ViewOf(const MeasurementHealth& sample,
                             std::vector<double>* covariance,
                             std::vector<SourceHealthView>* sources) {
  MeasurementHealthView view;
  view.header_present = sample.has_header();
  view.frame_id = sample.header().frame_id();
  view.source_time_present = sample.header().has_source_time();
  if (view.source_time_present) {
    view.source_time =
        embodiment::ClockReading{sample.header().source_time().seconds(),
                                 sample.header().source_time().nanos()};
  }
  view.receive_time_present = sample.header().has_receive_time();
  if (view.receive_time_present) {
    view.receive_time =
        embodiment::ClockReading{sample.header().receive_time().seconds(),
                                 sample.header().receive_time().nanos()};
  }
  view.header_validity_present = sample.header().has_validity();
  view.header_validity_state = sample.header().validity().state();
  view.state_present = sample.has_state();
  view.state = sample.state();
  view.quality_present = sample.has_quality();
  view.quality = sample.quality();
  view.covariance_present = sample.has_covariance();
  if (view.covariance_present) {
    *covariance = CopyDoubles(sample.covariance().values());
    view.covariance = *covariance;
  }
  sources->clear();
  for (const auto& source : sample.sources()) {
    sources->push_back(SourceHealthView{
        source.source_id(), source.has_validity(), source.validity().state()});
  }
  view.sources = *sources;
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

TEST(MeasurementHealthSerializationTest, NoSensorPayloadOnTheContract) {
  const google::protobuf::Descriptor* descriptor =
      MeasurementHealth::descriptor();
  EXPECT_EQ(descriptor->file()->package(), "intrinsic_proto.hardware.marine");
  EXPECT_EQ(descriptor->file()->message_type_count(), 1);
  EXPECT_EQ(descriptor->FindFieldByName("header")->message_type()->full_name(),
            "intrinsic_proto.embodiment.StampedHeader");
  EXPECT_EQ(
      descriptor->FindFieldByName("covariance")->message_type()->full_name(),
      "intrinsic_proto.vehicle.Matrix6");
  EXPECT_EQ(descriptor->FindFieldByName("sources")->message_type()->full_name(),
            "intrinsic_proto.vehicle.SourceHealth");
  EXPECT_TRUE(descriptor->FindFieldByName("state")->has_presence());
  EXPECT_TRUE(descriptor->FindFieldByName("quality")->has_presence());
  EXPECT_EQ(descriptor->FindFieldByName("quality")->cpp_type(),
            google::protobuf::FieldDescriptor::CPPTYPE_FLOAT);
  for (const char* name :
       {"dvl", "imu", "ins", "depth", "pressure", "altitude", "altimeter",
        "gnss", "gps", "sonar", "robot_type", "embodiment", "payload"}) {
    EXPECT_EQ(descriptor->FindFieldByName(name), nullptr) << name;
  }
  const google::protobuf::EnumDescriptor* validity =
      Validity::State_descriptor();
  ASSERT_EQ(validity->value_count(), 3);
  EXPECT_EQ(validity->value(0)->number(), 0);
  EXPECT_EQ(validity->value(1)->number(), 1);
  EXPECT_EQ(validity->value(2)->number(), 2);
  EXPECT_EQ(validity->FindValueByName("STATE_DEGRADED"), nullptr);
  EXPECT_EQ(validity->FindValueByName("DEGRADED"), nullptr);
  EXPECT_EQ(MeasurementHealth::UNKNOWN, 0);
  EXPECT_EQ(MeasurementHealth::VALID, 1);
  EXPECT_EQ(MeasurementHealth::DEGRADED, 2);
  EXPECT_EQ(MeasurementHealth::INVALID, 3);
}

TEST(MeasurementHealthSerializationTest, DefaultsAreEmptyAndOptIn) {
  EXPECT_EQ(MeasurementHealth().SerializeAsString(), "");
  EXPECT_FALSE(MeasurementHealth().has_header());
  EXPECT_FALSE(MeasurementHealth().has_state());
  EXPECT_FALSE(MeasurementHealth().has_quality());
  EXPECT_FALSE(MeasurementHealth().has_covariance());
  EXPECT_EQ(MeasurementHealth().sources_size(), 0);
}

TEST(MeasurementHealthSerializationTest, NominalRoundTripIsStable) {
  MeasurementHealth sample;
  FillNominal(&sample);
  const std::string bytes = sample.SerializeAsString();
  const std::string hex = ToHex(bytes);
  ASSERT_EQ(hex, kNominalGoldenHex) << hex;
  const std::string golden = FromHex(kNominalGoldenHex);
  MeasurementHealth parsed;
  ASSERT_TRUE(parsed.ParseFromString(golden));
  EXPECT_EQ(parsed.SerializeAsString(), golden);
  EXPECT_EQ(parsed.header().sequence(), 42u);
  EXPECT_EQ(parsed.header().frame_id(), "sensor");
  EXPECT_EQ(parsed.header().source_id(), "nav_sensor");
  EXPECT_EQ(parsed.header().clock_domain(), embodiment::kClockDomainMonotonic);
  EXPECT_EQ(parsed.header().source_time().seconds(), 1700000000);
  EXPECT_EQ(parsed.header().source_time().nanos(), 250000000);
  EXPECT_EQ(parsed.header().receive_time().seconds(), 1700000001);
  EXPECT_TRUE(parsed.header().has_validity());
  EXPECT_EQ(parsed.header().validity().state(), Validity::STATE_VALID);
  EXPECT_TRUE(parsed.has_state());
  EXPECT_EQ(parsed.state(), MeasurementHealth::VALID);
  EXPECT_FLOAT_EQ(parsed.quality(), 0.75f);
  EXPECT_EQ(parsed.covariance().values_size(), vehicle::kCovarianceValues);
  ASSERT_EQ(parsed.sources_size(), 2);
  EXPECT_EQ(parsed.sources(0).source_id(), "primary");
  EXPECT_TRUE(parsed.sources(0).has_validity());
  EXPECT_FALSE(parsed.sources(1).has_validity());

  std::vector<double> covariance;
  std::vector<SourceHealthView> sources;
  const MeasurementAssessment assessment =
      AssessMeasurementHealth(ViewOf(parsed, &covariance, &sources));
  EXPECT_TRUE(assessment.accepted);
  EXPECT_FALSE(vehicle::CovarianceIsUnknown(
      vehicle::AssessCovariance(parsed.has_covariance(), covariance)));
}

TEST(MeasurementHealthSerializationTest, ClearingSourcesIsAPrefixOfTheGolden) {
  MeasurementHealth sample;
  FillNominal(&sample);
  const std::string full = sample.SerializeAsString();
  sample.clear_sources();
  EXPECT_TRUE(full.starts_with(sample.SerializeAsString()));
  EXPECT_NE(full, sample.SerializeAsString());
}

TEST(MeasurementHealthSerializationTest, UnknownStateAndZeroQualityArePresent) {
  MeasurementHealth unset;
  MeasurementHealth unknown;
  unknown.set_state(MeasurementHealth::UNKNOWN);
  EXPECT_FALSE(unset.has_state());
  EXPECT_TRUE(unknown.has_state());
  EXPECT_EQ(unknown.state(), MeasurementHealth::UNKNOWN);
  EXPECT_NE(unset.SerializeAsString(), unknown.SerializeAsString());
  MeasurementHealth parsed;
  ASSERT_TRUE(parsed.ParseFromString(unknown.SerializeAsString()));
  EXPECT_TRUE(parsed.has_state());
  EXPECT_EQ(ClassifyMeasurementState(parsed.has_state(), parsed.state()),
            MeasurementStateKind::kUnknown);
  EXPECT_EQ(ClassifyMeasurementState(false, 0), MeasurementStateKind::kAbsent);

  MeasurementHealth zero_quality;
  EXPECT_FALSE(zero_quality.has_quality());
  zero_quality.set_quality(0.0f);
  EXPECT_TRUE(zero_quality.has_quality());
  const std::string with_zero = zero_quality.SerializeAsString();
  zero_quality.clear_quality();
  EXPECT_NE(zero_quality.SerializeAsString(), with_zero);
}

TEST(MeasurementHealthSerializationTest,
     ZeroCovarianceIsNotTheUnknownEncoding) {
  MeasurementHealth absent;
  MeasurementHealth zeros;
  for (int i = 0; i < vehicle::kCovarianceValues; ++i) {
    zeros.mutable_covariance()->add_values(0);
  }
  EXPECT_FALSE(absent.has_covariance());
  EXPECT_TRUE(zeros.has_covariance());
  EXPECT_NE(absent.SerializeAsString(), zeros.SerializeAsString());
  EXPECT_TRUE(
      vehicle::IsAllZeroCovariance(CopyDoubles(zeros.covariance().values())));
  EXPECT_FALSE(vehicle::CovarianceIsUnknown(vehicle::AssessCovariance(
      true, CopyDoubles(zeros.covariance().values()))));
  EXPECT_TRUE(
      vehicle::CovarianceIsUnknown(vehicle::AssessCovariance(false, {})));
}

TEST(MeasurementHealthSerializationTest, UnknownFieldIsPreserved) {
  MeasurementHealth sample;
  FillNominal(&sample);
  const std::string golden = sample.SerializeAsString();
  const std::string with_unknown = golden + std::string("\xA0\x06\x07", 3);
  MeasurementHealth parsed;
  ASSERT_TRUE(parsed.ParseFromString(with_unknown));
  EXPECT_EQ(parsed.state(), MeasurementHealth::VALID);
  const google::protobuf::UnknownFieldSet& unknown =
      parsed.GetReflection()->GetUnknownFields(parsed);
  ASSERT_EQ(unknown.field_count(), 1);
  EXPECT_EQ(unknown.field(0).number(), 100);
  EXPECT_EQ(unknown.field(0).varint(), 7u);
  EXPECT_EQ(parsed.SerializeAsString(), with_unknown);

  parsed.set_state(static_cast<MeasurementHealth::State>(100));
  MeasurementHealth reparsed;
  ASSERT_TRUE(reparsed.ParseFromString(parsed.SerializeAsString()));
  EXPECT_EQ(static_cast<int>(reparsed.state()), 100);
  EXPECT_EQ(ClassifyMeasurementState(reparsed.has_state(), reparsed.state()),
            MeasurementStateKind::kUnrecognized);
  EXPECT_NE(ClassifyMeasurementState(reparsed.has_state(), reparsed.state()),
            MeasurementStateKind::kInvalid);
}

TEST(MeasurementHealthSerializationTest, StampUsesTheExistingTimePolicy) {
  MeasurementHealth sample;
  FillNominal(&sample);
  const std::optional<double> age = embodiment::MonotonicAgeSeconds(
      embodiment::ClockReading{sample.header().source_time().seconds(),
                               sample.header().source_time().nanos()},
      embodiment::ClockReading{sample.header().receive_time().seconds(),
                               sample.header().receive_time().nanos()},
      sample.header().clock_domain());
  ASSERT_TRUE(age.has_value());
  EXPECT_DOUBLE_EQ(*age, 0.75);
  EXPECT_FALSE(
      embodiment::MonotonicAgeSeconds(
          embodiment::ClockReading{sample.header().source_time().seconds(),
                                   sample.header().source_time().nanos()},
          embodiment::ClockReading{sample.header().receive_time().seconds(),
                                   sample.header().receive_time().nanos()},
          embodiment::kClockDomainUtc)
          .has_value());

  sample.mutable_header()->mutable_receive_time()->set_seconds(1700000000);
  sample.mutable_header()->mutable_receive_time()->set_nanos(0);
  std::vector<double> covariance;
  std::vector<SourceHealthView> sources;
  EXPECT_EQ(
      AssessMeasurementHealth(ViewOf(sample, &covariance, &sources)).error,
      MeasurementError::kTimeReversal);
  EXPECT_TRUE(embodiment::SequenceAdvances(41, 42));
  EXPECT_FALSE(embodiment::SequenceAdvances(42, 42));
  EXPECT_FALSE(embodiment::FrameIdMatches(sample.header().frame_id(),
                                          embodiment::WorldFrame::kEnu));
}

TEST(MeasurementHealthSerializationTest, TextprotoExamplesMatchTheFixtures) {
  const std::string nominal_text = LoadExample("measurement_health.textproto");
  ASSERT_FALSE(nominal_text.empty()) << "measurement_health.textproto";
  MeasurementHealth parsed;
  ASSERT_TRUE(
      google::protobuf::TextFormat::ParseFromString(nominal_text, &parsed));
  MeasurementHealth coded;
  FillNominal(&coded);
  EXPECT_EQ(parsed.SerializeAsString(), coded.SerializeAsString());

  const std::string unknown_text =
      LoadExample("measurement_health_unknown_covariance.textproto");
  ASSERT_FALSE(unknown_text.empty());
  MeasurementHealth unknown;
  ASSERT_TRUE(
      google::protobuf::TextFormat::ParseFromString(unknown_text, &unknown));
  EXPECT_FALSE(unknown.has_covariance());
  MeasurementHealth coded_unknown;
  FillNominal(&coded_unknown);
  coded_unknown.clear_covariance();
  EXPECT_EQ(unknown.SerializeAsString(), coded_unknown.SerializeAsString());
  std::vector<double> covariance;
  std::vector<SourceHealthView> sources;
  const MeasurementAssessment assessment =
      AssessMeasurementHealth(ViewOf(unknown, &covariance, &sources));
  EXPECT_TRUE(assessment.accepted);
  EXPECT_TRUE(
      vehicle::CovarianceIsUnknown(vehicle::AssessCovariance(false, {})));
}

}  // namespace
}  // namespace intrinsic::hardware::marine
