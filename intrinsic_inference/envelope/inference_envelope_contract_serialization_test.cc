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

#include <cstdint>
#include <cstdlib>
#include <fstream>
#include <map>
#include <sstream>
#include <string>
#include <string_view>
#include <vector>

#include "google/protobuf/descriptor.h"
#include "google/protobuf/text_format.h"
#include "gtest/gtest.h"
#include "intrinsic/embodiment/stamped_header_policy.h"
#include "intrinsic/inference/proto/inference_envelope.pb.h"
#include "intrinsic_inference/envelope/inference_envelope_contract_policy.h"

namespace intrinsic::inference {
namespace {

using embodiment::ValidityKind;
using intrinsic_proto::inference::InferenceEnvelope;
using intrinsic_proto::inference::InferenceResult;
using intrinsic_proto::inference::OipModelIdentifier;
using intrinsic_proto::inference::OipRequestIdentifier;
using intrinsic_proto::inference::OipResultIdentifier;
using Error = InferenceEnvelopeContractError;

// Canonical serialization of examples/inference_envelope_nominal.textproto.
// Keep in sync with inference_envelope_contract_serialization_test.py.
constexpr std::string_view kNominalEnvelopeGoldenHex =
    "0a3b081512060880e2cfaa061a0a0880e2cfaa0610c0843d2209706c616e6e6572"
    "5f302a09776f726c645f656e7532096d6f6e6f746f6e69633a020801121f0a130a"
    "0e706f73655f657374696d61746f7212013312087265712d30303031182a224030"
    "313233343536373839616263646566303132333435363738396162636465663031"
    "3233343536373839616263646566303132333435363738396162636465662a0c08"
    "80e2cfaa061080cab5ee013202080239cdccccccccccec3f419a9999999999a93f"
    "4a477368613235363a616261626162616261626162616261626162616261626162"
    "616261626162616261626162616261626162616261626162616261626162616261"
    "62616261626162525c0a0e706f73655f657374696d61746f721201331a47736861"
    "3235363a6566656665666566656665666566656665666566656665666566656665"
    "666566656665666566656665666566656665666566656665666566656665666566"
    "6566a206130a0870726f6475636572120766697874757265";

std::string FromHex(std::string_view hex) {
  std::string out;
  out.reserve(hex.size() / 2);
  auto nibble = [](char c) -> int {
    if (c >= '0' && c <= '9') {
      return c - '0';
    }
    return c - 'a' + 10;
  };
  for (size_t i = 0; i + 1 < hex.size(); i += 2) {
    out.push_back(
        static_cast<char>((nibble(hex[i]) << 4) | nibble(hex[i + 1])));
  }
  return out;
}

std::string ToHex(const std::string& bytes) {
  static constexpr char kDigits[] = "0123456789abcdef";
  std::string out;
  out.reserve(bytes.size() * 2);
  for (const unsigned char c : bytes) {
    out.push_back(kDigits[c >> 4]);
    out.push_back(kDigits[c & 0xf]);
  }
  return out;
}

InferenceCommonView CommonViewOf(const InferenceEnvelope& message) {
  const auto& header = message.header();
  InferenceCommonView common;
  common.header_present = message.has_header();
  common.validity_present = header.has_validity();
  common.validity_state = static_cast<int>(header.validity().state());
  common.frame_id = header.frame_id();
  common.source_time_present = header.has_source_time();
  common.source_time = {header.source_time().seconds(),
                        header.source_time().nanos()};
  common.state_epoch = message.state_epoch();
  common.world_snapshot_id = message.world_snapshot_id();
  common.deadline_present = message.has_deadline();
  common.deadline = {message.deadline().seconds(), message.deadline().nanos()};
  common.validity_horizon_present = message.has_validity_horizon();
  common.validity_horizon = {message.validity_horizon().seconds(),
                             message.validity_horizon().nanos()};
  common.confidence_present = message.has_confidence();
  common.confidence = message.confidence();
  common.uncertainty_present = message.has_uncertainty();
  common.uncertainty = message.uncertainty();
  common.input_digest = message.input_digest();
  common.provenance_present = message.has_provenance();
  common.provenance_model_id = message.provenance().model_id();
  common.metadata_present = !message.metadata().empty();
  return common;
}

InferenceCommonView CommonViewOf(const InferenceResult& message) {
  const auto& header = message.header();
  InferenceCommonView common;
  common.header_present = message.has_header();
  common.validity_present = header.has_validity();
  common.validity_state = static_cast<int>(header.validity().state());
  common.frame_id = header.frame_id();
  common.source_time_present = header.has_source_time();
  common.source_time = {header.source_time().seconds(),
                        header.source_time().nanos()};
  common.state_epoch = message.state_epoch();
  common.world_snapshot_id = message.world_snapshot_id();
  common.deadline_present = message.has_deadline();
  common.deadline = {message.deadline().seconds(), message.deadline().nanos()};
  common.validity_horizon_present = message.has_validity_horizon();
  common.validity_horizon = {message.validity_horizon().seconds(),
                             message.validity_horizon().nanos()};
  common.confidence_present = message.has_confidence();
  common.confidence = message.confidence();
  common.uncertainty_present = message.has_uncertainty();
  common.uncertainty = message.uncertainty();
  common.input_digest = message.input_digest();
  common.provenance_present = message.has_provenance();
  common.provenance_model_id = message.provenance().model_id();
  common.metadata_present = !message.metadata().empty();
  return common;
}

InferenceEnvelopeContractAssessment Assess(const InferenceEnvelope& message) {
  InferenceEnvelopeView view;
  view.common = CommonViewOf(message);
  view.oip_request.message_set = message.has_oip_request();
  view.oip_request.model_name = message.oip_request().model().model_name();
  view.oip_request.model_version =
      message.oip_request().model().model_version();
  view.oip_request.request_id = message.oip_request().request_id();
  return AssessInferenceEnvelope(view);
}

InferenceEnvelopeContractAssessment Assess(const InferenceResult& message) {
  InferenceResultView view;
  view.common = CommonViewOf(message);
  view.oip_result.message_set = message.has_oip_result();
  view.oip_result.model_name = message.oip_result().model().model_name();
  view.oip_result.model_version = message.oip_result().model().model_version();
  view.oip_result.request_id = message.oip_result().request_id();
  view.output_digest = message.output_digest();
  return AssessInferenceResult(view);
}

std::string LoadExample(const std::string& name) {
  const char* src = std::getenv("TEST_SRCDIR");
  if (src == nullptr) {
    return "";
  }
  const std::string suffix = "intrinsic/inference/proto/examples/" + name;
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

template <typename Message>
Message ParseExample(const std::string& name) {
  const std::string text = LoadExample(name);
  EXPECT_FALSE(text.empty()) << name;
  Message parsed;
  EXPECT_TRUE(google::protobuf::TextFormat::ParseFromString(text, &parsed))
      << name;
  return parsed;
}

InferenceEnvelope ParseEnvelope(const std::string& stem) {
  return ParseExample<InferenceEnvelope>(stem + ".textproto");
}

TEST(InferenceEnvelopeFixtureTest, NominalEnvelopeIsAccepted) {
  const auto assessment = Assess(ParseEnvelope("inference_envelope_nominal"));
  EXPECT_EQ(assessment.error, Error::kNone);
  EXPECT_EQ(assessment.validity, ValidityKind::kValid);
  EXPECT_TRUE(assessment.accepted);
}

TEST(InferenceEnvelopeFixtureTest, NominalResultIsAccepted) {
  const auto assessment = Assess(
      ParseExample<InferenceResult>("inference_result_nominal.textproto"));
  EXPECT_EQ(assessment.error, Error::kNone);
  EXPECT_EQ(assessment.validity, ValidityKind::kValid);
  EXPECT_TRUE(assessment.accepted);
}

TEST(InferenceEnvelopeFixtureTest, DeadlineEqualCreationIsAccepted) {
  const InferenceEnvelope message =
      ParseEnvelope("inference_envelope_deadline_equal_creation");
  EXPECT_EQ(message.deadline().seconds(),
            message.header().source_time().seconds());
  EXPECT_EQ(message.deadline().nanos(), message.header().source_time().nanos());
  const auto assessment = Assess(message);
  EXPECT_EQ(assessment.error, Error::kNone);
  EXPECT_EQ(assessment.validity, ValidityKind::kValid);
  EXPECT_TRUE(assessment.accepted);
}

TEST(InferenceEnvelopeFixtureTest, InvalidFixturesReportLockedErrors) {
  const std::map<std::string, Error> cases = {
      {"inference_envelope_deadline_before_creation", Error::kDeadline},
      {"inference_envelope_bad_confidence", Error::kConfidence},
      {"inference_envelope_bad_uncertainty", Error::kUncertainty},
      {"inference_envelope_bad_snapshot_id", Error::kSnapshotId},
      {"inference_envelope_empty_oip_request_id", Error::kOipIdentifier},
  };
  for (const auto& [name, expected] : cases) {
    const auto assessment = Assess(ParseEnvelope(name));
    EXPECT_EQ(assessment.error, expected) << name;
    // STATE_VALID does not repair a structural defect.
    EXPECT_EQ(assessment.validity, ValidityKind::kValid) << name;
    EXPECT_FALSE(assessment.accepted) << name;
  }
}

TEST(InferenceEnvelopeFixtureTest, FixturesSurviveAWireRoundTrip) {
  for (const char* name : {"inference_envelope_nominal",
                           "inference_envelope_deadline_equal_creation",
                           "inference_envelope_deadline_before_creation",
                           "inference_envelope_bad_confidence",
                           "inference_envelope_bad_uncertainty",
                           "inference_envelope_bad_snapshot_id",
                           "inference_envelope_empty_oip_request_id"}) {
    const InferenceEnvelope original = ParseEnvelope(name);
    InferenceEnvelope reparsed;
    ASSERT_TRUE(reparsed.ParseFromString(original.SerializeAsString())) << name;
    EXPECT_EQ(original.SerializeAsString(), reparsed.SerializeAsString())
        << name;
    EXPECT_EQ(Assess(original).error, Assess(reparsed).error) << name;
    EXPECT_EQ(Assess(original).accepted, Assess(reparsed).accepted) << name;
  }
}

std::map<std::string, int> FieldNumbers(
    const google::protobuf::Descriptor* descriptor) {
  std::map<std::string, int> numbers;
  for (int i = 0; i < descriptor->field_count(); ++i) {
    numbers[std::string(descriptor->field(i)->name())] =
        descriptor->field(i)->number();
  }
  return numbers;
}

TEST(InferenceEnvelopeWireTest, LockedFieldNumbers) {
  EXPECT_EQ(FieldNumbers(InferenceEnvelope::descriptor()),
            (std::map<std::string, int>{{"header", 1},
                                        {"oip_request", 2},
                                        {"state_epoch", 3},
                                        {"world_snapshot_id", 4},
                                        {"deadline", 5},
                                        {"validity_horizon", 6},
                                        {"confidence", 7},
                                        {"uncertainty", 8},
                                        {"input_digest", 9},
                                        {"provenance", 10},
                                        {"metadata", 100}}));
  EXPECT_EQ(FieldNumbers(InferenceResult::descriptor()),
            (std::map<std::string, int>{{"header", 1},
                                        {"oip_result", 2},
                                        {"state_epoch", 3},
                                        {"world_snapshot_id", 4},
                                        {"deadline", 5},
                                        {"validity_horizon", 6},
                                        {"confidence", 7},
                                        {"uncertainty", 8},
                                        {"input_digest", 9},
                                        {"output_digest", 10},
                                        {"provenance", 11},
                                        {"metadata", 100}}));
  const std::map<std::string, int> identifier = {{"model", 1},
                                                 {"request_id", 2}};
  EXPECT_EQ(FieldNumbers(OipRequestIdentifier::descriptor()), identifier);
  EXPECT_EQ(FieldNumbers(OipResultIdentifier::descriptor()), identifier);
  EXPECT_EQ(
      FieldNumbers(OipModelIdentifier::descriptor()),
      (std::map<std::string, int>{{"model_name", 1}, {"model_version", 2}}));
}

TEST(InferenceEnvelopeWireTest, PackageAndImportsExcludeOipPayload) {
  const google::protobuf::FileDescriptor* file =
      InferenceEnvelope::descriptor()->file();
  EXPECT_EQ(std::string(file->package()), "intrinsic_proto.inference");
  std::vector<std::string> dependencies;
  for (int i = 0; i < file->dependency_count(); ++i) {
    dependencies.emplace_back(file->dependency(i)->name());
  }
  EXPECT_EQ(
      dependencies,
      (std::vector<std::string>{
          "google/protobuf/duration.proto", "google/protobuf/timestamp.proto",
          "intrinsic/embodiment/proto/stamped_header.proto",
          "intrinsic/embodiment/proto/model_provenance.proto"}));
}

TEST(InferenceEnvelopeWireTest, DefaultMessagesSerializeToZeroBytes) {
  EXPECT_EQ(InferenceEnvelope().SerializeAsString(), "");
  EXPECT_EQ(InferenceResult().SerializeAsString(), "");
  EXPECT_EQ(OipModelIdentifier().ByteSizeLong(), 0u);
  EXPECT_EQ(OipRequestIdentifier().ByteSizeLong(), 0u);
  EXPECT_EQ(OipResultIdentifier().ByteSizeLong(), 0u);
}

TEST(InferenceEnvelopeWireTest, DefaultMessagesAreNotEngaged) {
  const auto envelope = Assess(InferenceEnvelope());
  EXPECT_EQ(envelope.error, Error::kNone);
  EXPECT_FALSE(envelope.accepted);
  const auto result = Assess(InferenceResult());
  EXPECT_EQ(result.error, Error::kNone);
  EXPECT_FALSE(result.accepted);
}

TEST(InferenceEnvelopeWireTest, OptionalDoublesTrackPresence) {
  InferenceEnvelope message;
  EXPECT_FALSE(message.has_confidence());
  EXPECT_FALSE(message.has_uncertainty());
  message.set_confidence(0.0);
  message.set_uncertainty(0.0);
  EXPECT_TRUE(message.has_confidence());
  EXPECT_TRUE(message.has_uncertainty());
  // Tag plus fixed64 each. A set zero is not an unset field.
  EXPECT_EQ(message.ByteSizeLong(), 18u);
  InferenceEnvelope reparsed;
  ASSERT_TRUE(reparsed.ParseFromString(message.SerializeAsString()));
  EXPECT_TRUE(reparsed.has_confidence());
  EXPECT_TRUE(reparsed.has_uncertainty());
  reparsed.clear_confidence();
  EXPECT_FALSE(reparsed.has_confidence());
}

TEST(InferenceEnvelopeWireTest, UnknownFieldsRoundTrip) {
  const InferenceEnvelope original =
      ParseEnvelope("inference_envelope_nominal");
  // Field 900 varint = 7, then field 901 length-delimited = "ext".
  const std::string wire = original.SerializeAsString() + FromHex("a03807") +
                           FromHex("aa3803657874");
  InferenceEnvelope parsed;
  ASSERT_TRUE(parsed.ParseFromString(wire));
  EXPECT_EQ(parsed.input_digest(), original.input_digest());
  EXPECT_EQ(parsed.SerializeAsString(), wire);
  EXPECT_TRUE(Assess(parsed).accepted);
}

TEST(InferenceEnvelopeWireTest, UnknownFieldsInNestedMessagesRoundTrip) {
  InferenceEnvelope nested = ParseEnvelope("inference_envelope_nominal");
  ASSERT_TRUE(nested.mutable_oip_request()->mutable_model()->MergeFromString(
      FromHex("a03807")));
  ASSERT_TRUE(nested.mutable_provenance()->MergeFromString(FromHex("b03809")));
  const std::string wire = nested.SerializeAsString();
  InferenceEnvelope reparsed;
  ASSERT_TRUE(reparsed.ParseFromString(wire));
  EXPECT_EQ(reparsed.SerializeAsString(), wire);
  EXPECT_TRUE(Assess(reparsed).accepted);
}

TEST(InferenceEnvelopeWireTest, UnknownFieldsRoundTripOnResult) {
  const InferenceResult original =
      ParseExample<InferenceResult>("inference_result_nominal.textproto");
  const std::string wire = original.SerializeAsString() + FromHex("a03807");
  InferenceResult parsed;
  ASSERT_TRUE(parsed.ParseFromString(wire));
  EXPECT_EQ(parsed.SerializeAsString(), wire);
  EXPECT_TRUE(Assess(parsed).accepted);
}

TEST(InferenceEnvelopeWireTest, MetadataRoundTrips) {
  InferenceEnvelope original = ParseEnvelope("inference_envelope_nominal");
  (*original.mutable_metadata())["second"] = "value";
  InferenceEnvelope reparsed;
  ASSERT_TRUE(reparsed.ParseFromString(original.SerializeAsString()));
  EXPECT_EQ(reparsed.metadata().size(), original.metadata().size());
  EXPECT_EQ(reparsed.metadata().at("second"), "value");
  EXPECT_TRUE(Assess(reparsed).accepted);
}

TEST(InferenceEnvelopeWireTest, MetadataAloneEngages) {
  InferenceEnvelope message;
  (*message.mutable_metadata())["k"] = "v";
  const auto assessment = Assess(message);
  EXPECT_EQ(assessment.error, Error::kMissingFrame);
  EXPECT_FALSE(assessment.accepted);
}

TEST(InferenceEnvelopeWireTest, NominalEnvelopeGoldenBytes) {
  const InferenceEnvelope message = ParseEnvelope("inference_envelope_nominal");
  EXPECT_EQ(ToHex(message.SerializeAsString()), kNominalEnvelopeGoldenHex);
  InferenceEnvelope parsed;
  ASSERT_TRUE(parsed.ParseFromString(FromHex(kNominalEnvelopeGoldenHex)));
  EXPECT_EQ(parsed.SerializeAsString(), message.SerializeAsString());
}

}  // namespace
}  // namespace intrinsic::inference
