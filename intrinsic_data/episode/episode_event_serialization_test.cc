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

// Serialization, golden, and compatibility tests for EpisodeEvent.
// Keep in sync with episode_event_serialization_test.py.

#include <cctype>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <fstream>
#include <sstream>
#include <string>
#include <string_view>
#include <vector>

#include "google/protobuf/io/coded_stream.h"
#include "google/protobuf/io/zero_copy_stream_impl_lite.h"
#include "google/protobuf/message.h"
#include "google/protobuf/text_format.h"
#include "google/protobuf/unknown_field_set.h"
#include "gtest/gtest.h"
#include "intrinsic/episode/proto/episode_event.pb.h"
#include "intrinsic_data/episode/episode_event_policy.h"

namespace intrinsic::episode {
namespace {

using Error = EpisodeEventError;
using intrinsic_proto::episode::EpisodeEvent;
using intrinsic_proto::episode::RecordedComponentTiming;

constexpr std::string_view kPinnedSeed42Digest =
    "93150ec8e5fd8f6dcc2d9cae50d84de9b89287b187edfa4a807b0da0e8eb5d6a";
constexpr size_t kSeed42TextBytes = 1530;
constexpr std::string_view kSnapshot =
    "6ceb1af0dca7d2b01d4e876fe699fb8c5d5709d5a4938ea3f46e8d355d3b33de";
constexpr std::string_view kV2Name =
    "episode_event_v2_additive_unknown_payload.binpb";
constexpr uint64_t kUnknownPayloadTag = 536870000;
constexpr uint64_t kUnknownEnvelopeTag = 536870001;

struct PayloadCase {
  const char* name;
  PayloadKind kind;
};

constexpr PayloadCase kPayloads[] = {
    {"safety_decision", PayloadKind::kSafetyDecision},
    {"vehicle_trajectory", PayloadKind::kVehicleTrajectory},
    {"inference_envelope", PayloadKind::kInferenceEnvelope},
    {"inference_result", PayloadKind::kInferenceResult},
    {"metadata", PayloadKind::kEpisodeMetadata},
};

// Returns the file bytes, or fails the test when no candidate exists.
std::string ReadRunfile(const std::string& relative) {
  const char* src = std::getenv("TEST_SRCDIR");
  const char* workspace = std::getenv("TEST_WORKSPACE");
  std::vector<std::string> candidates;
  if (src != nullptr) {
    const std::string root(src);
    if (workspace != nullptr) {
      candidates.push_back(root + "/" + workspace + "/" + relative);
    }
    candidates.push_back(root + "/_main/" + relative);
    candidates.push_back(root + "/intrinsic_apis+/" + relative);
    candidates.push_back(root + "/intrinsic_apis/" + relative);
    if (workspace != nullptr) {
      candidates.push_back(root + "/" + workspace +
                           "/external/intrinsic_apis+/" + relative);
    }
  }
  for (const std::string& path : candidates) {
    std::ifstream in(path, std::ios::binary);
    if (!in) {
      continue;
    }
    std::ostringstream buffer;
    buffer << in.rdbuf();
    return buffer.str();
  }
  ADD_FAILURE() << "missing runfile " << relative;
  return "";
}

std::string TestdataBytes(const std::string& name) {
  return ReadRunfile("intrinsic_data/episode/testdata/" + name);
}

std::string GoldenBytes(const std::string& payload) {
  return TestdataBytes("episode_event_v1_" + payload + ".binpb");
}

std::string GoldenText(const std::string& payload) {
  return TestdataBytes("episode_event_v1_" + payload + ".textproto");
}

std::string ExampleText(const std::string& payload) {
  return ReadRunfile("intrinsic/episode/proto/examples/episode_event_v1_" +
                     payload + ".textproto");
}

std::string Serialize(const EpisodeEvent& message) {
  std::string out;
  {
    google::protobuf::io::StringOutputStream stream(&out);
    google::protobuf::io::CodedOutputStream coded(&stream);
    coded.SetSerializationDeterministic(true);
    EXPECT_TRUE(message.SerializeToCodedStream(&coded));
  }
  return out;
}

EpisodeEvent Parse(const std::string& bytes) {
  EpisodeEvent message;
  EXPECT_TRUE(message.ParseFromString(bytes));
  return message;
}

EpisodeEvent Golden(const std::string& payload) {
  return Parse(GoldenBytes(payload));
}

void AppendVarint(uint64_t value, std::string* out) {
  while (value >= 0x80) {
    out->push_back(static_cast<char>((value & 0x7f) | 0x80));
    value >>= 7;
  }
  out->push_back(static_cast<char>(value));
}

EpisodeEventView ViewOf(const EpisodeEvent& message) {
  const auto& header = message.header();
  EpisodeEventView view;
  view.header_present = message.has_header();
  view.source_id = header.source_id();
  view.frame_id = header.frame_id();
  view.source_time_present = header.has_source_time();
  view.source_time = {header.source_time().seconds(),
                      header.source_time().nanos()};
  view.receive_time_present = header.has_receive_time();
  view.receive_time = {header.receive_time().seconds(),
                       header.receive_time().nanos()};
  view.schema_version = message.schema_version();
  view.snapshot_id = message.snapshot_id();
  view.config_digest = message.config_digest();
  view.provenance_present = message.has_provenance();
  view.payload_case = static_cast<int>(message.payload_case());
  view.has_unknown_fields =
      !message.GetReflection()->GetUnknownFields(message).empty();
  if (message.has_episode_metadata()) {
    view.marine_randomization_metadata =
        message.episode_metadata().marine_randomization_metadata();
  }
  return view;
}

EpisodeEventAssessment Assess(const EpisodeEvent& message) {
  return AssessEpisodeEvent(ViewOf(message));
}

template <typename Edit>
Error ErrorAfter(const std::string& payload, Edit edit) {
  EpisodeEvent message = Golden(payload);
  edit(message);
  return Assess(message).error;
}

TEST(EpisodeEventSerializationTest, PayloadCaseNumbersMatchTheGeneratedSchema) {
  EXPECT_EQ(static_cast<int>(EpisodeEvent::PAYLOAD_NOT_SET),
            kPayloadCaseNotSet);
  EXPECT_EQ(static_cast<int>(EpisodeEvent::kSafetyDecision),
            kPayloadCaseSafetyDecision);
  EXPECT_EQ(static_cast<int>(EpisodeEvent::kVehicleTrajectory),
            kPayloadCaseVehicleTrajectory);
  EXPECT_EQ(static_cast<int>(EpisodeEvent::kInferenceEnvelope),
            kPayloadCaseInferenceEnvelope);
  EXPECT_EQ(static_cast<int>(EpisodeEvent::kInferenceResult),
            kPayloadCaseInferenceResult);
  EXPECT_EQ(static_cast<int>(EpisodeEvent::kEpisodeMetadata),
            kPayloadCaseEpisodeMetadata);
  EXPECT_EQ(static_cast<int>(EpisodeEvent::kRecordedMessage),
            kPayloadCaseRecordedMessage);
  EXPECT_EQ(static_cast<int>(EpisodeEvent::kWorldSnapshotRecord),
            kPayloadCaseWorldSnapshotRecord);
  EXPECT_EQ(static_cast<int>(EpisodeEvent::kDesiredMotion),
            kPayloadCaseDesiredMotion);
}

static_assert(static_cast<int>(EpisodeEvent::kRecordedMessage) ==
              kPayloadCaseRecordedMessage);
static_assert(static_cast<int>(EpisodeEvent::kWorldSnapshotRecord) ==
              kPayloadCaseWorldSnapshotRecord);
static_assert(static_cast<int>(EpisodeEvent::kDesiredMotion) ==
              kPayloadCaseDesiredMotion);

TEST(EpisodeEventSerializationTest, E1RoundTripPerPayload) {
  for (const PayloadCase& payload : kPayloads) {
    const std::string bytes = GoldenBytes(payload.name);
    const EpisodeEvent message = Parse(bytes);
    const EpisodeEventAssessment assessment = Assess(message);
    EXPECT_EQ(assessment.error, Error::kNone) << payload.name;
    EXPECT_EQ(assessment.payload_kind, payload.kind) << payload.name;
    EXPECT_FALSE(assessment.skip) << payload.name;
    EXPECT_EQ(Serialize(message), bytes) << payload.name;
  }
}

TEST(EpisodeEventSerializationTest, E1TextFixtureSerializesToTheGoldenBytes) {
  for (const PayloadCase& payload : kPayloads) {
    EpisodeEvent message;
    ASSERT_TRUE(google::protobuf::TextFormat::ParseFromString(
        GoldenText(payload.name), &message))
        << payload.name;
    EXPECT_EQ(Serialize(message), GoldenBytes(payload.name)) << payload.name;
  }
}

TEST(EpisodeEventSerializationTest,
     E1ProtoPackageExamplesMatchTheTestdataText) {
  for (const PayloadCase& payload : kPayloads) {
    EXPECT_EQ(ExampleText(payload.name), GoldenText(payload.name))
        << payload.name;
  }
}

TEST(EpisodeEventSerializationTest, E2MetadataEventPreservesEveryField) {
  const EpisodeEvent message = Parse(Serialize(Golden("metadata")));
  const auto& header = message.header();
  EXPECT_EQ(header.sequence(), 5u);
  EXPECT_EQ(header.source_time().seconds(), 1700000105);
  EXPECT_EQ(header.source_time().nanos(), 250000000);
  EXPECT_EQ(header.receive_time().seconds(), 1700000105);
  EXPECT_EQ(header.receive_time().nanos(), 251000000);
  EXPECT_EQ(header.source_id(), "episode_recorder");
  EXPECT_EQ(header.frame_id(), "world_enu");
  EXPECT_EQ(header.clock_domain(), "monotonic");
  EXPECT_EQ(static_cast<int>(header.validity().state()), 1);
  EXPECT_EQ(message.schema_version(), 1u);
  EXPECT_EQ(message.snapshot_id(), kSnapshot);
  EXPECT_EQ(message.config_digest(), kPinnedSeed42Digest);
  ASSERT_TRUE(message.has_provenance());
  EXPECT_EQ(message.provenance().model_id(), "marine_parameter_sampler");
  EXPECT_EQ(message.provenance().model_version(), "v1");
  EXPECT_EQ(message.provenance().digest(),
            "sha256:" + std::string(kPinnedSeed42Digest));
  EXPECT_EQ(message.payload_case(), EpisodeEvent::kEpisodeMetadata);
}

TEST(EpisodeEventSerializationTest, E2UnsetProvenanceStaysUnset) {
  for (const PayloadCase& payload : kPayloads) {
    if (std::string_view(payload.name) == "metadata") {
      continue;
    }
    const EpisodeEvent message = Parse(Serialize(Golden(payload.name)));
    EXPECT_FALSE(message.has_provenance()) << payload.name;
  }
}

TEST(EpisodeEventSerializationTest, E3EmptyMessageIsNotEngaged) {
  const EpisodeEvent message = Parse("");
  EXPECT_EQ(message.ByteSizeLong(), 0u);
  EXPECT_EQ(Serialize(message), "");
  const EpisodeEventAssessment assessment = Assess(message);
  EXPECT_EQ(assessment.error, Error::kNone);
  EXPECT_FALSE(assessment.engaged);
  EXPECT_FALSE(assessment.skip);
}

TEST(EpisodeEventSerializationTest, E3SingleEventIsAccepted) {
  const EpisodeEventAssessment assessment = Assess(Golden("safety_decision"));
  EXPECT_EQ(assessment.error, Error::kNone);
  EXPECT_TRUE(assessment.engaged);
}

TEST(EpisodeEventSerializationTest, E4EveryErrorRow) {
  EXPECT_EQ(
      ErrorAfter("safety_decision", [](EpisodeEvent& m) { m.clear_header(); }),
      Error::kMissingHeader);
  EXPECT_EQ(ErrorAfter(
                "safety_decision",
                [](EpisodeEvent& m) { m.mutable_header()->clear_source_id(); }),
            Error::kSourceId);
  EXPECT_EQ(
      ErrorAfter("safety_decision",
                 [](EpisodeEvent& m) { m.mutable_header()->clear_frame_id(); }),
      Error::kMissingFrame);
  EXPECT_EQ(ErrorAfter("safety_decision",
                       [](EpisodeEvent& m) {
                         m.mutable_header()->clear_source_time();
                       }),
            Error::kSourceTime);
  EXPECT_EQ(ErrorAfter("safety_decision",
                       [](EpisodeEvent& m) {
                         m.mutable_header()->clear_receive_time();
                       }),
            Error::kReceiveTime);
  for (const int32_t nanos : {-1, 1000000000}) {
    EXPECT_EQ(ErrorAfter("safety_decision",
                         [nanos](EpisodeEvent& m) {
                           m.mutable_header()->mutable_source_time()->set_nanos(
                               nanos);
                         }),
              Error::kSourceTime)
        << nanos;
    EXPECT_EQ(
        ErrorAfter("safety_decision",
                   [nanos](EpisodeEvent& m) {
                     m.mutable_header()->mutable_receive_time()->set_nanos(
                         nanos);
                   }),
        Error::kReceiveTime)
        << nanos;
  }
  EXPECT_EQ(ErrorAfter("safety_decision",
                       [](EpisodeEvent& m) { m.set_schema_version(0); }),
            Error::kSchemaVersion);
  const std::string upper = [] {
    std::string s(kSnapshot);
    for (char& c : s) {
      c = static_cast<char>(std::toupper(static_cast<unsigned char>(c)));
    }
    return s;
  }();
  const std::string short63(kSnapshot.substr(0, 63));
  const std::string long65 = std::string(kSnapshot) + "a";
  const std::string non_hex(64, 'g');
  for (const std::string& bad : {upper, short63, long65, non_hex}) {
    EXPECT_EQ(ErrorAfter("safety_decision",
                         [&bad](EpisodeEvent& m) { m.set_snapshot_id(bad); }),
              Error::kSnapshotId)
        << bad;
    EXPECT_EQ(ErrorAfter("safety_decision",
                         [&bad](EpisodeEvent& m) { m.set_config_digest(bad); }),
              Error::kConfigDigest)
        << bad;
  }
  EXPECT_EQ(
      ErrorAfter("safety_decision", [](EpisodeEvent& m) { m.clear_payload(); }),
      Error::kMissingPayload);
}

TEST(EpisodeEventSerializationTest, E4MetadataDigestMismatch) {
  EXPECT_EQ(
      ErrorAfter(
          "metadata",
          [](EpisodeEvent& m) {
            std::string text =
                m.episode_metadata().marine_randomization_metadata();
            const size_t index = text.find("seed=42") + 5;
            text[index] = '3';
            m.mutable_episode_metadata()->set_marine_randomization_metadata(
                text);
          }),
      Error::kConfigDigestMismatch);
  EXPECT_EQ(
      ErrorAfter("metadata", [](EpisodeEvent& m) { m.clear_config_digest(); }),
      Error::kConfigDigestMismatch);
  EXPECT_EQ(
      ErrorAfter(
          "metadata",
          [](EpisodeEvent& m) {
            m.mutable_episode_metadata()->clear_marine_randomization_metadata();
          }),
      Error::kConfigDigestMismatch);
}

TEST(EpisodeEventSerializationTest, E5TruncatedBytesReportAParseFailure) {
  for (const PayloadCase& payload : kPayloads) {
    const std::string bytes = GoldenBytes(payload.name);
    int failures = 0;
    for (size_t cut = 1; cut <= bytes.size(); ++cut) {
      EpisodeEvent message;
      if (!message.ParseFromString(bytes.substr(0, bytes.size() - cut))) {
        ++failures;
        continue;
      }
      Assess(message);
    }
    EXPECT_GT(failures, 0) << payload.name;
    EpisodeEvent message;
    EXPECT_FALSE(message.ParseFromString(bytes.substr(0, bytes.size() - 1)))
        << payload.name;
  }
}

TEST(EpisodeEventSerializationTest, E6V2AdditiveGoldenIsSkippedAndPreserved) {
  const std::string bytes = TestdataBytes(std::string(kV2Name));
  const EpisodeEvent message = Parse(bytes);
  const EpisodeEventView view = ViewOf(message);
  EXPECT_EQ(view.payload_case, kPayloadCaseNotSet);
  EXPECT_TRUE(view.has_unknown_fields);
  const EpisodeEventAssessment assessment = AssessEpisodeEvent(view);
  EXPECT_EQ(assessment.error, Error::kNone);
  EXPECT_EQ(assessment.payload_kind, PayloadKind::kUnknown);
  EXPECT_TRUE(assessment.skip);
  std::string reserialized;
  ASSERT_TRUE(message.SerializeToString(&reserialized));
  EXPECT_EQ(reserialized, bytes);
  EXPECT_EQ(Serialize(message), bytes);
}

TEST(EpisodeEventSerializationTest,
     E6V2GoldenIsTheV1MetadataEnvelopePlusUnknownFields) {
  EpisodeEvent expected = Golden("metadata");
  expected.clear_episode_metadata();
  std::string bytes = Serialize(expected);
  AppendVarint((kUnknownPayloadTag << 3) | 2, &bytes);
  constexpr std::string_view kFuture = "future_payload";
  AppendVarint(kFuture.size(), &bytes);
  bytes.append(kFuture);
  AppendVarint((kUnknownEnvelopeTag << 3) | 0, &bytes);
  AppendVarint(7, &bytes);
  EXPECT_EQ(TestdataBytes(std::string(kV2Name)), bytes);
}

TEST(EpisodeEventSerializationTest, E6UnknownEnvelopeFieldKeepsAKnownPayload) {
  std::string bytes = GoldenBytes("metadata");
  AppendVarint((kUnknownEnvelopeTag << 3) | 0, &bytes);
  AppendVarint(7, &bytes);
  const EpisodeEvent message = Parse(bytes);
  const EpisodeEventAssessment assessment = Assess(message);
  EXPECT_EQ(assessment.error, Error::kNone);
  EXPECT_EQ(assessment.payload_kind, PayloadKind::kEpisodeMetadata);
  EXPECT_FALSE(assessment.skip);
  std::string reserialized;
  ASSERT_TRUE(message.SerializeToString(&reserialized));
  EXPECT_EQ(reserialized, bytes);
}

TEST(EpisodeEventSerializationTest,
     E7V1GoldensParseIdenticallyFromTextAndBytes) {
  for (const PayloadCase& payload : kPayloads) {
    EpisodeEvent from_text;
    ASSERT_TRUE(google::protobuf::TextFormat::ParseFromString(
        GoldenText(payload.name), &from_text));
    EXPECT_EQ(Serialize(from_text), Serialize(Golden(payload.name)))
        << payload.name;
  }
}

TEST(EpisodeEventSerializationTest,
     E7NewerSchemaVersionOnAV1PayloadIsAccepted) {
  EpisodeEvent message = Golden("safety_decision");
  message.set_schema_version(2);
  const EpisodeEvent reparsed = Parse(Serialize(message));
  EXPECT_EQ(Assess(reparsed).error, Error::kNone);
  EXPECT_EQ(reparsed.schema_version(), 2u);
}

TEST(EpisodeEventSerializationTest, E8PinnedDigestOfTheSeed42Text) {
  const EpisodeEvent message = Golden("metadata");
  const std::string& text =
      message.episode_metadata().marine_randomization_metadata();
  EXPECT_EQ(text.size(), kSeed42TextBytes);
  EXPECT_EQ(text.rfind("marine_randomization_metadata_v1\n", 0), 0u);
  EXPECT_EQ(ComputeConfigDigest(text), kPinnedSeed42Digest);
  EXPECT_EQ(message.config_digest(), kPinnedSeed42Digest);
}

TEST(EpisodeEventSerializationTest, E8OneByteTextChangeIsAMismatch) {
  EpisodeEvent message = Golden("metadata");
  const std::string text =
      message.episode_metadata().marine_randomization_metadata();
  std::string changed = text;
  changed.back() = 'x';
  message.mutable_episode_metadata()->set_marine_randomization_metadata(
      changed);
  EXPECT_EQ(Assess(message).error, Error::kConfigDigestMismatch);
  message.mutable_episode_metadata()->set_marine_randomization_metadata(text +
                                                                        "\n");
  EXPECT_EQ(Assess(message).error, Error::kConfigDigestMismatch);
}

// #244: replay payloads. These goldens carry the metadata envelope identity
// (provenance and config digest) with their own sequence and times, so they
// are kept apart from kPayloads.
constexpr PayloadCase kReplayPayloads[] = {
    {"recorded_message", PayloadKind::kRecordedMessage},
    {"world_snapshot_record", PayloadKind::kWorldSnapshotRecord},
    {"desired_motion", PayloadKind::kDesiredMotion},
};

struct ReplayWireCase {
  const char* name;
  uint32_t tag;
  int kind_number;
};

constexpr ReplayWireCase kReplayWire[] = {
    {"recorded_message", 16, 8},
    {"world_snapshot_record", 17, 9},
    {"desired_motion", 18, 10},
};

// The fixed MeasurementEnvelope listed byte by byte in the testdata README.
constexpr uint8_t kMeasurementEnvelopeBytes[] = {
    0x0a, 0x38, 0x08, 0x01, 0x12, 0x0b, 0x08, 0xe4, 0xe2, 0xcf, 0xaa, 0x06,
    0x10, 0x80, 0xe5, 0x9a, 0x77, 0x1a, 0x0b, 0x08, 0xe4, 0xe2, 0xcf, 0xaa,
    0x06, 0x10, 0xc0, 0xe9, 0xd7, 0x77, 0x22, 0x05, 0x64, 0x76, 0x6c, 0x5f,
    0x30, 0x2a, 0x04, 0x62, 0x6f, 0x64, 0x79, 0x32, 0x09, 0x6d, 0x6f, 0x6e,
    0x6f, 0x74, 0x6f, 0x6e, 0x69, 0x63, 0x3a, 0x02, 0x08, 0x01, 0x12, 0x05,
    0x64, 0x76, 0x6c, 0x5f, 0x30, 0x18, 0x03, 0x22, 0x21, 0x69, 0x6e, 0x74,
    0x72, 0x69, 0x6e, 0x73, 0x69, 0x63, 0x5f, 0x70, 0x72, 0x6f, 0x74, 0x6f,
    0x2e, 0x76, 0x65, 0x68, 0x69, 0x63, 0x6c, 0x65, 0x2e, 0x42, 0x6f, 0x64,
    0x79, 0x54, 0x77, 0x69, 0x73, 0x74, 0x2a, 0x09, 0x09, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0xe0, 0x3f};

struct WireField {
  uint32_t number;
  uint32_t wire_type;
};

// Top-level fields of one serialized message, in wire order.
std::vector<WireField> TopLevelFields(const std::string& bytes) {
  std::vector<WireField> fields;
  google::protobuf::io::CodedInputStream in(
      reinterpret_cast<const uint8_t*>(bytes.data()),
      static_cast<int>(bytes.size()));
  for (uint32_t tag = in.ReadTag(); tag != 0; tag = in.ReadTag()) {
    fields.push_back({tag >> 3, tag & 7});
    switch (tag & 7) {
      case 0: {
        uint64_t ignored = 0;
        EXPECT_TRUE(in.ReadVarint64(&ignored));
        break;
      }
      case 1:
        EXPECT_TRUE(in.Skip(8));
        break;
      case 2: {
        uint32_t length = 0;
        EXPECT_TRUE(in.ReadVarint32(&length));
        EXPECT_TRUE(in.Skip(static_cast<int>(length)));
        break;
      }
      case 5:
        EXPECT_TRUE(in.Skip(4));
        break;
      default:
        ADD_FAILURE() << "unexpected wire type " << (tag & 7);
        return fields;
    }
  }
  return fields;
}

TEST(EpisodeEventSerializationTest, S1ReplayGoldensRoundTripByteExactly) {
  for (const PayloadCase& payload : kReplayPayloads) {
    const std::string bytes = GoldenBytes(payload.name);
    ASSERT_FALSE(bytes.empty()) << payload.name;
    const EpisodeEvent message = Parse(bytes);
    EXPECT_EQ(Serialize(message), bytes) << payload.name;
    std::string reserialized;
    ASSERT_TRUE(message.SerializeToString(&reserialized));
    EXPECT_EQ(reserialized, bytes) << payload.name;
    EXPECT_TRUE(message.GetReflection()->GetUnknownFields(message).empty())
        << payload.name;

    EpisodeEvent from_text;
    ASSERT_TRUE(google::protobuf::TextFormat::ParseFromString(
        GoldenText(payload.name), &from_text))
        << payload.name;
    EXPECT_EQ(Serialize(from_text), bytes) << payload.name;
    EXPECT_EQ(ExampleText(payload.name), GoldenText(payload.name))
        << payload.name;
  }
}

TEST(EpisodeEventSerializationTest, S1ReplayGoldensShareTheMetadataIdentity) {
  const EpisodeEvent metadata = Golden("metadata");
  for (const PayloadCase& payload : kReplayPayloads) {
    const EpisodeEvent message = Golden(payload.name);
    EXPECT_EQ(message.schema_version(), 1u) << payload.name;
    EXPECT_EQ(message.snapshot_id(), metadata.snapshot_id()) << payload.name;
    EXPECT_EQ(message.config_digest(), kPinnedSeed42Digest) << payload.name;
    EXPECT_EQ(message.provenance().SerializeAsString(),
              metadata.provenance().SerializeAsString())
        << payload.name;
    EXPECT_EQ(message.header().source_id(), "episode_recorder");
    EXPECT_EQ(message.header().frame_id(), "world_enu");
    EXPECT_EQ(message.header().clock_domain(), "monotonic");
  }
}

TEST(EpisodeEventSerializationTest, S1RecordedMessageCarriesTheFixedBytes) {
  const EpisodeEvent message = Golden("recorded_message");
  ASSERT_EQ(message.payload_case(), EpisodeEvent::kRecordedMessage);
  EXPECT_EQ(message.recorded_message().message_type(),
            "intrinsic_proto.estimation.MeasurementEnvelope");
  EXPECT_EQ(
      message.recorded_message().message_bytes(),
      std::string(reinterpret_cast<const char*>(kMeasurementEnvelopeBytes),
                  sizeof(kMeasurementEnvelopeBytes)));
  EXPECT_EQ(sizeof(kMeasurementEnvelopeBytes), 113u);
}

TEST(EpisodeEventSerializationTest, S1WorldSnapshotRecordMatchesItsTimings) {
  const EpisodeEvent message = Golden("world_snapshot_record");
  ASSERT_EQ(message.payload_case(), EpisodeEvent::kWorldSnapshotRecord);
  const auto& record = message.world_snapshot_record();
  EXPECT_EQ(record.snapshot().snapshot_id(), message.snapshot_id());
  ASSERT_EQ(record.snapshot().components_size(), 2);
  ASSERT_EQ(record.timings_size(), 2);
  for (int i = 0; i < 2; ++i) {
    EXPECT_EQ(record.timings(i).component_kind(),
              record.snapshot().components(i).component_kind())
        << i;
    EXPECT_TRUE(record.timings(i).has_observation_time()) << i;
    EXPECT_TRUE(record.timings(i).has_validity_horizon()) << i;
    EXPECT_FALSE(record.timings(i).source_id().empty()) << i;
  }
}

TEST(EpisodeEventSerializationTest, S1DesiredMotionHasATwistAndAHorizon) {
  const EpisodeEvent message = Golden("desired_motion");
  ASSERT_EQ(message.payload_case(), EpisodeEvent::kDesiredMotion);
  const auto& motion = message.desired_motion();
  EXPECT_TRUE(motion.has_twist());
  EXPECT_DOUBLE_EQ(motion.twist().body_twist().linear_x_m_s(), 0.5);
  EXPECT_DOUBLE_EQ(motion.twist().body_twist().angular_z_rad_s(), 0.125);
  EXPECT_TRUE(motion.has_horizon());
  EXPECT_EQ(motion.horizon().seconds(), 2);
  EXPECT_EQ(motion.header().frame_id(), "body");
  EXPECT_NE(motion.header().SerializeAsString(),
            message.header().SerializeAsString())
      << "the inner header is recorded verbatim, not cross-checked";
}

TEST(EpisodeEventSerializationTest, S2ReplayPayloadsAreEngagedAndAccepted) {
  for (const PayloadCase& payload : kReplayPayloads) {
    const EpisodeEventAssessment assessment = Assess(Golden(payload.name));
    EXPECT_EQ(assessment.error, Error::kNone) << payload.name;
    EXPECT_NE(assessment.error, Error::kMissingPayload) << payload.name;
    EXPECT_EQ(assessment.payload_kind, payload.kind) << payload.name;
    EXPECT_FALSE(assessment.skip) << payload.name;
    EXPECT_TRUE(assessment.engaged) << payload.name;
  }
}

TEST(EpisodeEventSerializationTest, S3ReplayPayloadsAreLengthDelimitedTags) {
  for (const ReplayWireCase& c : kReplayWire) {
    const std::vector<WireField> fields = TopLevelFields(GoldenBytes(c.name));
    int payload_fields = 0;
    for (const WireField& field : fields) {
      if (field.number >= 10) {
        ++payload_fields;
        EXPECT_EQ(field.number, c.tag) << c.name;
        EXPECT_EQ(field.wire_type, 2u) << c.name;
      }
    }
    EXPECT_EQ(payload_fields, 1) << c.name;
    EXPECT_EQ(static_cast<int>(ClassifyPayload(static_cast<int>(c.tag), false)),
              c.kind_number)
        << c.name;
  }
}

TEST(EpisodeEventSerializationTest, S3PreSchemaViewClassifiesAsUnknownSkip) {
  for (const PayloadCase& payload : kReplayPayloads) {
    EpisodeEventView view = ViewOf(Golden(payload.name));
    view.payload_case = kPayloadCaseNotSet;
    view.has_unknown_fields = true;
    const EpisodeEventAssessment assessment = AssessEpisodeEvent(view);
    EXPECT_EQ(assessment.error, Error::kNone) << payload.name;
    EXPECT_EQ(assessment.payload_kind, PayloadKind::kUnknown) << payload.name;
    EXPECT_TRUE(assessment.skip) << payload.name;
    EXPECT_TRUE(assessment.engaged) << payload.name;
  }
}

TEST(EpisodeEventSerializationTest, S3ReplayPayloadsDoNotMoveTheV2Boundary) {
  EXPECT_LT(static_cast<uint64_t>(kPayloadCaseDesiredMotion),
            kUnknownPayloadTag);
  EXPECT_EQ(Assess(Parse(TestdataBytes(std::string(kV2Name)))).payload_kind,
            PayloadKind::kUnknown);
}

TEST(EpisodeEventSerializationTest, S3SnapshotFieldIsNotNamedDescriptor) {
  // The field is named snapshot: a field named descriptor would collide with
  // Message::descriptor().
  const RecordedComponentTiming timing;
  EXPECT_EQ(timing.GetDescriptor()->full_name(),
            "intrinsic_proto.episode.RecordedComponentTiming");
  EXPECT_NE(intrinsic_proto::episode::WorldSnapshotRecord::descriptor()
                ->FindFieldByName("snapshot"),
            nullptr);
  EXPECT_EQ(intrinsic_proto::episode::WorldSnapshotRecord::descriptor()
                ->FindFieldByName("descriptor"),
            nullptr);
}

}  // namespace
}  // namespace intrinsic::episode
