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
#include "intrinsic/safety/proto/safety_decision.pb.h"
#include "intrinsic/safety/safety_decision_assessor.h"
#include "intrinsic/safety/safety_decision_policy.h"
#include "intrinsic/vehicle/proto/vehicle_command.pb.h"

namespace intrinsic::safety {
namespace {

using intrinsic_proto::embodiment::Validity;
using intrinsic_proto::safety::SafetyDecision;
using intrinsic_proto::safety::SafetyFinding;
using intrinsic_proto::safety::SafetyFindingSeverity;
using intrinsic_proto::vehicle::DesiredMotion;

constexpr std::string_view kDigest =
    "5c12e76d610e5cf3f5b0d6c402bcfef05db2ec4deb5faacd03cfecdd2f3e34ae";
constexpr std::string_view kSnapshotId =
    "6ceb1af0dca7d2b01d4e876fe699fb8c5d5709d5a4938ea3f46e8d355d3b33de";

// Canonical serializations of the fillers below. Keep in sync with
// safety_decision_serialization_test.py.
constexpr std::string_view kProjectGoldenHex =
    "0a36080b120c088ae2cfaa061080cab5ee011a06088be2cfaa06220d7361666574795f66"
    "696c74657232096d6f6e6f746f6e69633a02080110021a40356331326537366436313065"
    "356366336635623064366334303262636665663035646232656334646562356661616364"
    "3033636665636464326633653334616522580a3c080c120c088ae2cfaa061080cab5ee01"
    "1a06088be2cfaa06220d7361666574795f66696c7465722a04626f647932096d6f6e6f74"
    "6f6e69633a0208011a0b0a0909000000000000d03f2a02080131000000000000e03f2a40"
    "366365623161663064636137643262303164346538373666653639396662386335643537"
    "30396435613439333865613366343665386433353564336233336465323a0a0b73706565"
    "645f6c696d697410021a16466f727761726420737065656420726564756365642e221152"
    "657175657374656420302e35206d2f733802";
constexpr std::string_view kAppliedIntentDigest =
    "f800488ed44145af782a5d1ac6c9ca7fff4d548b03fa591c7b363fc7216f8d7f";

std::string FromHex(std::string_view hex) {
  std::string bytes;
  for (size_t i = 0; i + 1 < hex.size(); i += 2) {
    bytes.push_back(static_cast<char>(
        std::stoi(std::string(hex.substr(i, 2)), nullptr, 16)));
  }
  return bytes;
}

void FillHeader(intrinsic_proto::embodiment::StampedHeader* header,
                uint64_t sequence, const std::string& frame_id) {
  header->set_sequence(sequence);
  header->mutable_source_time()->set_seconds(1700000010);
  header->mutable_source_time()->set_nanos(500000000);
  header->mutable_receive_time()->set_seconds(1700000011);
  header->set_source_id("safety_filter");
  header->set_frame_id(frame_id);
  header->set_clock_domain("monotonic");
  header->mutable_validity()->set_state(Validity::STATE_VALID);
}

DesiredMotion TwistMotion() {
  DesiredMotion motion;
  FillHeader(motion.mutable_header(), 12, "body");
  motion.mutable_twist()->mutable_body_twist()->set_linear_x_m_s(0.25);
  motion.mutable_horizon()->set_seconds(1);
  motion.set_confidence(0.5);
  return motion;
}

SafetyDecision ProjectDecision() {
  SafetyDecision decision;
  FillHeader(decision.mutable_header(), 11, "");
  decision.set_kind(intrinsic_proto::safety::SAFETY_DECISION_KIND_PROJECT);
  decision.set_original_intent_digest(std::string(kDigest));
  *decision.mutable_applied_intent() = TwistMotion();
  decision.set_snapshot_id(std::string(kSnapshotId));
  SafetyFinding* finding = decision.add_findings();
  finding->set_rule_id("speed_limit");
  finding->set_severity(
      intrinsic_proto::safety::SAFETY_FINDING_SEVERITY_WARNING);
  finding->set_summary("Forward speed reduced.");
  finding->set_detail("Requested 0.5 m/s");
  decision.set_decision_epoch(2);
  return decision;
}

std::string LoadExample(const std::string& name) {
  const char* src = std::getenv("TEST_SRCDIR");
  if (src == nullptr) {
    return "";
  }
  const std::string suffix = "intrinsic/safety/proto/examples/" + name;
  const char* workspace = std::getenv("TEST_WORKSPACE");
  // Bzlmod canonical repo name is intrinsic_apis+. The apparent name
  // intrinsic_apis is kept for layouts that symlink it.
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

SafetyDecision LoadDecision(const std::string& name) {
  const std::string text = LoadExample(name);
  EXPECT_FALSE(text.empty()) << name;
  SafetyDecision decision;
  EXPECT_TRUE(google::protobuf::TextFormat::ParseFromString(text, &decision))
      << name;
  return decision;
}

TEST(SafetyDecisionSerializationTest, EnumNumbersAreLocked) {
  EXPECT_EQ(intrinsic_proto::safety::SAFETY_DECISION_KIND_UNSPECIFIED, 0);
  EXPECT_EQ(intrinsic_proto::safety::SAFETY_DECISION_KIND_ACCEPT, 1);
  EXPECT_EQ(intrinsic_proto::safety::SAFETY_DECISION_KIND_PROJECT, 2);
  EXPECT_EQ(intrinsic_proto::safety::SAFETY_DECISION_KIND_REJECT, 3);
  EXPECT_EQ(intrinsic_proto::safety::SAFETY_DECISION_KIND_ABORT, 4);
  EXPECT_EQ(intrinsic_proto::safety::SAFETY_DECISION_KIND_SURFACE, 5);
  EXPECT_EQ(intrinsic_proto::safety::SAFETY_FINDING_SEVERITY_UNSPECIFIED, 0);
  EXPECT_EQ(intrinsic_proto::safety::SAFETY_FINDING_SEVERITY_INFO, 1);
  EXPECT_EQ(intrinsic_proto::safety::SAFETY_FINDING_SEVERITY_WARNING, 2);
  EXPECT_EQ(intrinsic_proto::safety::SAFETY_FINDING_SEVERITY_ERROR, 3);
  EXPECT_EQ(intrinsic_proto::safety::SAFETY_FINDING_SEVERITY_CRITICAL, 4);
  const google::protobuf::EnumDescriptor* kinds =
      intrinsic_proto::safety::SafetyDecisionKind_descriptor();
  EXPECT_EQ(kinds->value_count(), 6);
  const google::protobuf::EnumDescriptor* severities =
      intrinsic_proto::safety::SafetyFindingSeverity_descriptor();
  EXPECT_EQ(severities->value_count(), 5);
}

TEST(SafetyDecisionSerializationTest, FieldNumbersAreLocked) {
  const google::protobuf::Descriptor* decision = SafetyDecision::descriptor();
  const std::vector<std::pair<std::string, int>> decision_fields = {
      {"header", 1},         {"kind", 2},        {"original_intent_digest", 3},
      {"applied_intent", 4}, {"snapshot_id", 5}, {"findings", 6},
      {"decision_epoch", 7}};
  EXPECT_EQ(decision->field_count(), 7);
  for (const auto& [name, number] : decision_fields) {
    const google::protobuf::FieldDescriptor* field =
        decision->FindFieldByName(name);
    ASSERT_NE(field, nullptr) << name;
    EXPECT_EQ(field->number(), number) << name;
  }
  EXPECT_TRUE(decision->FindFieldByName("decision_epoch")->has_presence());
  EXPECT_TRUE(decision->FindFieldByName("findings")->is_repeated());

  const google::protobuf::Descriptor* finding = SafetyFinding::descriptor();
  const std::vector<std::pair<std::string, int>> finding_fields = {
      {"rule_id", 1}, {"severity", 2}, {"summary", 3}, {"detail", 4}};
  EXPECT_EQ(finding->field_count(), 4);
  for (const auto& [name, number] : finding_fields) {
    const google::protobuf::FieldDescriptor* field =
        finding->FindFieldByName(name);
    ASSERT_NE(field, nullptr) << name;
    EXPECT_EQ(field->number(), number) << name;
  }
  EXPECT_TRUE(finding->FindFieldByName("detail")->has_presence());
}

TEST(SafetyDecisionSerializationTest, EmptyDecisionSerializesToZeroBytes) {
  SafetyDecision decision;
  EXPECT_EQ(decision.ByteSizeLong(), 0u);
  EXPECT_EQ(decision.SerializeAsString(), "");
  const SafetyDecisionAssessment assessment =
      AssessSafetyDecisionProto(decision);
  EXPECT_FALSE(assessment.engaged);
  EXPECT_EQ(assessment.error, SafetyDecisionError::kNone);
  EXPECT_FALSE(assessment.accepted);
}

TEST(SafetyDecisionSerializationTest, ProjectGoldenBytes) {
  const SafetyDecision decision = ProjectDecision();
  EXPECT_EQ(decision.SerializeAsString(), FromHex(kProjectGoldenHex));
  SafetyDecision parsed;
  ASSERT_TRUE(parsed.ParseFromString(FromHex(kProjectGoldenHex)));
  EXPECT_EQ(parsed.SerializeAsString(), FromHex(kProjectGoldenHex));
  EXPECT_TRUE(AssessSafetyDecisionProto(parsed).accepted);
}

TEST(SafetyDecisionSerializationTest, ClearingEpochLeavesAPrefixOfTheGolden) {
  SafetyDecision decision = ProjectDecision();
  decision.clear_decision_epoch();
  const std::string golden = FromHex(kProjectGoldenHex);
  EXPECT_EQ(decision.SerializeAsString(), golden.substr(0, golden.size() - 2));
  EXPECT_EQ(decision.decision_epoch(), 0u);
  EXPECT_TRUE(AssessSafetyDecisionProto(decision).accepted);
}

TEST(SafetyDecisionSerializationTest, OriginalIntentDigestIsSha256OfMotion) {
  const DesiredMotion motion = TwistMotion();
  const std::string digest = ComputeOriginalIntentDigest(motion);
  EXPECT_EQ(digest, kAppliedIntentDigest);
  EXPECT_TRUE(IsLowercaseSha256Hex(digest));
}

TEST(SafetyDecisionSerializationTest, UnknownKindRoundTripsAndIsNotRewritten) {
  SafetyDecision decision = ProjectDecision();
  decision.clear_applied_intent();
  decision.set_kind(
      static_cast<intrinsic_proto::safety::SafetyDecisionKind>(99));
  const std::string wire = decision.SerializeAsString();

  SafetyDecision parsed;
  ASSERT_TRUE(parsed.ParseFromString(wire));
  EXPECT_EQ(static_cast<int>(parsed.kind()), 99);
  EXPECT_EQ(parsed.SerializeAsString(), wire);

  const SafetyDecisionAssessment assessment = AssessSafetyDecisionProto(parsed);
  EXPECT_EQ(assessment.error, SafetyDecisionError::kNone);
  EXPECT_EQ(assessment.kind, SafetyDecisionKind::kUnknown);
  EXPECT_FALSE(assessment.accepted);
  EXPECT_NE(assessment.kind, ClassifySafetyDecisionKind(3));
  EXPECT_NE(assessment.kind, ClassifySafetyDecisionKind(4));
  EXPECT_EQ(static_cast<int>(parsed.kind()), 99);
  EXPECT_EQ(parsed.SerializeAsString(), wire);
}

TEST(SafetyDecisionSerializationTest, UnknownKindFromHandBuiltWireIsKept) {
  SafetyDecision decision = ProjectDecision();
  decision.clear_applied_intent();
  decision.set_kind(intrinsic_proto::safety::SAFETY_DECISION_KIND_UNSPECIFIED);
  // Field 2 (kind) as varint 99, appended after the known fields.
  const std::string wire =
      decision.SerializeAsString() + std::string("\x10\x63", 2);
  SafetyDecision parsed;
  ASSERT_TRUE(parsed.ParseFromString(wire));
  EXPECT_EQ(static_cast<int>(parsed.kind()), 99);
  EXPECT_FALSE(AssessSafetyDecisionProto(parsed).accepted);
  EXPECT_EQ(AssessSafetyDecisionProto(parsed).error,
            SafetyDecisionError::kNone);
}

TEST(SafetyDecisionSerializationTest, UnknownSeverityRoundTripsAndIsKept) {
  SafetyDecision decision = ProjectDecision();
  decision.mutable_findings(0)->set_severity(
      static_cast<SafetyFindingSeverity>(77));
  const std::string wire = decision.SerializeAsString();
  SafetyDecision parsed;
  ASSERT_TRUE(parsed.ParseFromString(wire));
  EXPECT_EQ(static_cast<int>(parsed.findings(0).severity()), 77);
  EXPECT_EQ(parsed.SerializeAsString(), wire);
  EXPECT_TRUE(AssessSafetyDecisionProto(parsed).accepted);
  EXPECT_EQ(static_cast<int>(parsed.findings(0).severity()), 77);
  EXPECT_EQ(ClassifySafetyFindingSeverity(77),
            SafetyFindingSeverityKind::kUnknown);
}

TEST(SafetyDecisionSerializationTest, UnknownFieldIsKept) {
  const std::string with_unknown =
      FromHex(kProjectGoldenHex) + std::string("\xA0\x06\x07", 3);
  SafetyDecision parsed;
  ASSERT_TRUE(parsed.ParseFromString(with_unknown));
  const google::protobuf::UnknownFieldSet& unknown =
      parsed.GetReflection()->GetUnknownFields(parsed);
  ASSERT_EQ(unknown.field_count(), 1);
  EXPECT_EQ(unknown.field(0).number(), 100);
  EXPECT_EQ(parsed.SerializeAsString(), with_unknown);
  EXPECT_TRUE(AssessSafetyDecisionProto(parsed).accepted);
}

TEST(SafetyDecisionSerializationTest, DuplicateRuleIdsAreNotRewritten) {
  SafetyDecision decision = ProjectDecision();
  SafetyFinding* duplicate = decision.add_findings();
  duplicate->set_rule_id("speed_limit");
  duplicate->set_severity(
      intrinsic_proto::safety::SAFETY_FINDING_SEVERITY_CRITICAL);
  SafetyDecision parsed;
  ASSERT_TRUE(parsed.ParseFromString(decision.SerializeAsString()));
  ASSERT_EQ(parsed.findings_size(), 2);
  EXPECT_EQ(parsed.findings(0).rule_id(), parsed.findings(1).rule_id());
  EXPECT_TRUE(AssessSafetyDecisionProto(parsed).accepted);
}

TEST(SafetyDecisionSerializationTest, TextprotoExamples) {
  const SafetyDecision accept = LoadDecision("accept.textproto");
  EXPECT_FALSE(accept.has_applied_intent());
  SafetyDecisionAssessment assessment = AssessSafetyDecisionProto(accept);
  EXPECT_EQ(assessment.kind, SafetyDecisionKind::kAccept);
  EXPECT_TRUE(assessment.accepted);

  const SafetyDecision project = LoadDecision("project.textproto");
  EXPECT_TRUE(project.has_applied_intent());
  assessment = AssessSafetyDecisionProto(project);
  EXPECT_EQ(assessment.kind, SafetyDecisionKind::kProject);
  EXPECT_TRUE(assessment.accepted);

  const SafetyDecision reject = LoadDecision("reject.textproto");
  EXPECT_FALSE(reject.has_applied_intent());
  EXPECT_EQ(reject.findings_size(), 2);
  assessment = AssessSafetyDecisionProto(reject);
  EXPECT_EQ(assessment.kind, SafetyDecisionKind::kReject);
  EXPECT_TRUE(assessment.accepted);

  const SafetyDecision abort = LoadDecision("abort.textproto");
  EXPECT_FALSE(abort.has_applied_intent());
  assessment = AssessSafetyDecisionProto(abort);
  EXPECT_EQ(assessment.kind, SafetyDecisionKind::kAbort);
  EXPECT_TRUE(assessment.accepted);

  const SafetyDecision surface = LoadDecision("surface.textproto");
  EXPECT_TRUE(surface.has_applied_intent());
  EXPECT_TRUE(surface.applied_intent().has_pose());
  assessment = AssessSafetyDecisionProto(surface);
  EXPECT_EQ(assessment.kind, SafetyDecisionKind::kSurface);
  EXPECT_TRUE(assessment.accepted);
}

TEST(SafetyDecisionSerializationTest, TextprotoExamplesRoundTripThroughWire) {
  for (const char* name :
       {"accept.textproto", "project.textproto", "reject.textproto",
        "abort.textproto", "surface.textproto"}) {
    const SafetyDecision example = LoadDecision(name);
    SafetyDecision parsed;
    ASSERT_TRUE(parsed.ParseFromString(example.SerializeAsString())) << name;
    EXPECT_EQ(parsed.SerializeAsString(), example.SerializeAsString()) << name;
    EXPECT_TRUE(AssessSafetyDecisionProto(parsed).accepted) << name;
  }
}

}  // namespace
}  // namespace intrinsic::safety
