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

#include "intrinsic_data/episode/episode_event_policy.h"

#include <cstdint>
#include <string>
#include <string_view>

#include "gtest/gtest.h"

namespace intrinsic::episode {
namespace {

using Error = EpisodeEventError;

constexpr std::string_view kSnapshot =
    "0123456789abcdef0123456789abcdef0123456789abcdef0123456789abcdef";
constexpr std::string_view kMetadataText =
    "marine_randomization_metadata_v1\nseed=1\n";

EpisodeEventView NominalView() {
  EpisodeEventView view;
  view.header_present = true;
  view.source_id = "episode_recorder";
  view.frame_id = "world_enu";
  view.source_time_present = true;
  view.source_time = {1700000000, 0};
  view.receive_time_present = true;
  view.receive_time = {1700000000, 1000000};
  view.schema_version = 1;
  view.snapshot_id = kSnapshot;
  view.payload_case = kPayloadCaseSafetyDecision;
  return view;
}

// The digest string must outlive the view that points at it.
struct MetadataFixture {
  std::string digest = ComputeConfigDigest(kMetadataText);
  EpisodeEventView view;

  MetadataFixture() {
    view = NominalView();
    view.payload_case = kPayloadCaseEpisodeMetadata;
    view.config_digest = digest;
    view.marine_randomization_metadata = kMetadataText;
  }
};

TEST(EpisodeEventPolicyTest, ErrorNumbersAreStable) {
  EXPECT_EQ(static_cast<int>(Error::kNone), 0);
  EXPECT_EQ(static_cast<int>(Error::kMissingHeader), 1);
  EXPECT_EQ(static_cast<int>(Error::kSourceId), 2);
  EXPECT_EQ(static_cast<int>(Error::kMissingFrame), 3);
  EXPECT_EQ(static_cast<int>(Error::kSourceTime), 4);
  EXPECT_EQ(static_cast<int>(Error::kReceiveTime), 5);
  EXPECT_EQ(static_cast<int>(Error::kSchemaVersion), 6);
  EXPECT_EQ(static_cast<int>(Error::kSnapshotId), 7);
  EXPECT_EQ(static_cast<int>(Error::kConfigDigest), 8);
  EXPECT_EQ(static_cast<int>(Error::kConfigDigestMismatch), 9);
  EXPECT_EQ(static_cast<int>(Error::kMissingPayload), 10);
}

TEST(EpisodeEventPolicyTest, EmptyViewIsNotEngaged) {
  const EpisodeEventAssessment assessment = AssessEpisodeEvent({});
  EXPECT_EQ(assessment.error, Error::kNone);
  EXPECT_EQ(assessment.payload_kind, PayloadKind::kNone);
  EXPECT_FALSE(assessment.skip);
  EXPECT_FALSE(assessment.engaged);
}

TEST(EpisodeEventPolicyTest, AnySingleSignalEngages) {
  const auto engaged = [](EpisodeEventView view) {
    return AssessEpisodeEvent(view).engaged;
  };
  EXPECT_TRUE(engaged([] {
    EpisodeEventView v;
    v.header_present = true;
    return v;
  }()));
  EXPECT_TRUE(engaged([] {
    EpisodeEventView v;
    v.schema_version = 1;
    return v;
  }()));
  EXPECT_TRUE(engaged([] {
    EpisodeEventView v;
    v.snapshot_id = kSnapshot;
    return v;
  }()));
  EXPECT_TRUE(engaged([] {
    EpisodeEventView v;
    v.config_digest = kSnapshot;
    return v;
  }()));
  EXPECT_TRUE(engaged([] {
    EpisodeEventView v;
    v.provenance_present = true;
    return v;
  }()));
  EXPECT_TRUE(engaged([] {
    EpisodeEventView v;
    v.payload_case = kPayloadCaseInferenceResult;
    return v;
  }()));
  EXPECT_TRUE(engaged([] {
    EpisodeEventView v;
    v.has_unknown_fields = true;
    v.unknown_payload_candidate = true;
    return v;
  }()));
}

TEST(EpisodeEventPolicyTest, NominalEventsAreAcceptedForEveryKnownCase) {
  struct Case {
    int payload_case;
    PayloadKind kind;
  };
  const Case cases[] = {
      {kPayloadCaseSafetyDecision, PayloadKind::kSafetyDecision},
      {kPayloadCaseVehicleTrajectory, PayloadKind::kVehicleTrajectory},
      {kPayloadCaseInferenceEnvelope, PayloadKind::kInferenceEnvelope},
      {kPayloadCaseInferenceResult, PayloadKind::kInferenceResult},
      {kPayloadCaseRecordedMessage, PayloadKind::kRecordedMessage},
      {kPayloadCaseWorldSnapshotRecord, PayloadKind::kWorldSnapshotRecord},
      {kPayloadCaseDesiredMotion, PayloadKind::kDesiredMotion},
  };
  for (const Case& c : cases) {
    EpisodeEventView view = NominalView();
    view.payload_case = c.payload_case;
    const EpisodeEventAssessment assessment = AssessEpisodeEvent(view);
    EXPECT_EQ(assessment.error, Error::kNone) << c.payload_case;
    EXPECT_EQ(assessment.payload_kind, c.kind) << c.payload_case;
    EXPECT_FALSE(assessment.skip) << c.payload_case;
    EXPECT_TRUE(assessment.engaged) << c.payload_case;
  }
  const MetadataFixture metadata;
  const EpisodeEventAssessment assessment = AssessEpisodeEvent(metadata.view);
  EXPECT_EQ(assessment.error, Error::kNone);
  EXPECT_EQ(assessment.payload_kind, PayloadKind::kEpisodeMetadata);
}

TEST(EpisodeEventPolicyTest, EmptySnapshotAndDigestAreAbsentNotErrors) {
  EpisodeEventView view = NominalView();
  view.snapshot_id = "";
  view.config_digest = "";
  EXPECT_EQ(AssessEpisodeEvent(view).error, Error::kNone);
}

TEST(EpisodeEventPolicyTest, EveryErrorRowHasExactlyOneMutation) {
  {
    EpisodeEventView view = NominalView();
    view.header_present = false;
    EXPECT_EQ(AssessEpisodeEvent(view).error, Error::kMissingHeader);
  }
  {
    EpisodeEventView view = NominalView();
    view.source_id = "";
    EXPECT_EQ(AssessEpisodeEvent(view).error, Error::kSourceId);
  }
  {
    EpisodeEventView view = NominalView();
    view.frame_id = "";
    EXPECT_EQ(AssessEpisodeEvent(view).error, Error::kMissingFrame);
  }
  {
    EpisodeEventView view = NominalView();
    view.source_time_present = false;
    EXPECT_EQ(AssessEpisodeEvent(view).error, Error::kSourceTime);
  }
  for (const int32_t nanos : {-1, 1000000000, 2000000000}) {
    EpisodeEventView view = NominalView();
    view.source_time = {1700000000, nanos};
    EXPECT_EQ(AssessEpisodeEvent(view).error, Error::kSourceTime) << nanos;
  }
  {
    EpisodeEventView view = NominalView();
    view.receive_time_present = false;
    EXPECT_EQ(AssessEpisodeEvent(view).error, Error::kReceiveTime);
  }
  for (const int32_t nanos : {-1, 1000000000, 2000000000}) {
    EpisodeEventView view = NominalView();
    view.receive_time = {1700000000, nanos};
    EXPECT_EQ(AssessEpisodeEvent(view).error, Error::kReceiveTime) << nanos;
  }
  {
    EpisodeEventView view = NominalView();
    view.schema_version = 0;
    EXPECT_EQ(AssessEpisodeEvent(view).error, Error::kSchemaVersion);
  }
  {
    EpisodeEventView view = NominalView();
    view.payload_case = kPayloadCaseNotSet;
    EXPECT_EQ(AssessEpisodeEvent(view).error, Error::kMissingPayload);
    EXPECT_EQ(AssessEpisodeEvent(view).payload_kind, PayloadKind::kNone);
    EXPECT_FALSE(AssessEpisodeEvent(view).skip);
  }
}

static_assert(kPayloadCaseEpisodeLoss == 15);
static_assert(kPayloadCaseRecordedMessage == 16);
static_assert(kPayloadCaseWorldSnapshotRecord == 17);
static_assert(kPayloadCaseDesiredMotion == 18);

TEST(EpisodeEventPolicyTest, PayloadKindNumbersAreStable) {
  EXPECT_EQ(static_cast<int>(PayloadKind::kNone), 0);
  EXPECT_EQ(static_cast<int>(PayloadKind::kSafetyDecision), 1);
  EXPECT_EQ(static_cast<int>(PayloadKind::kVehicleTrajectory), 2);
  EXPECT_EQ(static_cast<int>(PayloadKind::kInferenceEnvelope), 3);
  EXPECT_EQ(static_cast<int>(PayloadKind::kInferenceResult), 4);
  EXPECT_EQ(static_cast<int>(PayloadKind::kEpisodeMetadata), 5);
  EXPECT_EQ(static_cast<int>(PayloadKind::kUnknown), 6);
  EXPECT_EQ(static_cast<int>(PayloadKind::kEpisodeLoss), 7);
  EXPECT_EQ(static_cast<int>(PayloadKind::kRecordedMessage), 8);
  EXPECT_EQ(static_cast<int>(PayloadKind::kWorldSnapshotRecord), 9);
  EXPECT_EQ(static_cast<int>(PayloadKind::kDesiredMotion), 10);
}

TEST(EpisodeEventPolicyTest, ReplayPayloadsClassifyToTheirKinds) {
  struct Case {
    int payload_case;
    PayloadKind kind;
    int number;
  };
  const Case cases[] = {
      {kPayloadCaseRecordedMessage, PayloadKind::kRecordedMessage, 8},
      {kPayloadCaseWorldSnapshotRecord, PayloadKind::kWorldSnapshotRecord, 9},
      {kPayloadCaseDesiredMotion, PayloadKind::kDesiredMotion, 10},
  };
  for (const Case& c : cases) {
    for (const bool has_unknown : {false, true}) {
      EXPECT_EQ(ClassifyPayload(c.payload_case, has_unknown), c.kind)
          << c.payload_case;
      EXPECT_EQ(static_cast<int>(c.kind), c.number);
      EpisodeEventView view = NominalView();
      view.payload_case = c.payload_case;
      view.has_unknown_fields = has_unknown;
      view.unknown_payload_candidate = has_unknown;
      const EpisodeEventAssessment assessment = AssessEpisodeEvent(view);
      EXPECT_EQ(assessment.error, Error::kNone) << c.payload_case;
      EXPECT_EQ(assessment.payload_kind, c.kind) << c.payload_case;
      EXPECT_FALSE(assessment.skip) << c.payload_case;
      EXPECT_TRUE(assessment.engaged) << c.payload_case;
    }
  }
}

TEST(EpisodeEventPolicyTest, CasesPastTheKnownRangeStayUnknown) {
  for (const int payload_case : {9, 19, 99, 536870000}) {
    EXPECT_EQ(ClassifyPayload(payload_case, true), PayloadKind::kUnknown)
        << payload_case;
    EXPECT_EQ(ClassifyPayload(payload_case, false), PayloadKind::kNone)
        << payload_case;
  }
}

TEST(EpisodeEventPolicyTest, TimesAreNotOrderedAndSequenceIsNotChecked) {
  EpisodeEventView view = NominalView();
  view.source_time = {1700000010, 0};
  view.receive_time = {1700000000, 0};
  EXPECT_EQ(AssessEpisodeEvent(view).error, Error::kNone);
}

TEST(EpisodeEventPolicyTest, NewerSchemaVersionsAreAccepted) {
  for (const uint32_t version : {1u, 2u, 3u, 4294967295u}) {
    EpisodeEventView view = NominalView();
    view.schema_version = version;
    EXPECT_EQ(AssessEpisodeEvent(view).error, Error::kNone) << version;
  }
}

TEST(EpisodeEventPolicyTest, HexRulesApplyToSnapshotAndConfigDigest) {
  const std::string upper(64, 'A');
  const std::string short63(63, 'a');
  const std::string long65(65, 'a');
  const std::string non_hex(64, 'g');
  const std::string prefixed = "sha256:" + std::string(57, 'a');
  for (const std::string& bad : {upper, short63, long65, non_hex, prefixed}) {
    {
      EpisodeEventView view = NominalView();
      view.snapshot_id = bad;
      EXPECT_EQ(AssessEpisodeEvent(view).error, Error::kSnapshotId) << bad;
    }
    {
      EpisodeEventView view = NominalView();
      view.config_digest = bad;
      EXPECT_EQ(AssessEpisodeEvent(view).error, Error::kConfigDigest) << bad;
    }
  }
}

TEST(EpisodeEventPolicyTest, ConfigDigestMustMatchMetadataText) {
  {
    const MetadataFixture fixture;
    EXPECT_EQ(AssessEpisodeEvent(fixture.view).error, Error::kNone);
  }
  {
    MetadataFixture fixture;
    const std::string other = ComputeConfigDigest("different");
    fixture.view.config_digest = other;
    EXPECT_EQ(AssessEpisodeEvent(fixture.view).error,
              Error::kConfigDigestMismatch);
  }
  {
    MetadataFixture fixture;
    fixture.view.config_digest = "";
    EXPECT_EQ(AssessEpisodeEvent(fixture.view).error,
              Error::kConfigDigestMismatch);
  }
  {
    MetadataFixture fixture;
    fixture.view.marine_randomization_metadata = "";
    EXPECT_EQ(AssessEpisodeEvent(fixture.view).error,
              Error::kConfigDigestMismatch);
  }
  {
    EpisodeEventView view = NominalView();
    view.payload_case = kPayloadCaseEpisodeMetadata;
    EXPECT_EQ(AssessEpisodeEvent(view).error, Error::kNone)
        << "both empty is allowed";
  }
}

TEST(EpisodeEventPolicyTest, DigestIsNotCheckedForOtherPayloads) {
  EpisodeEventView view = NominalView();
  const std::string digest = ComputeConfigDigest("anything");
  view.config_digest = digest;
  EXPECT_EQ(AssessEpisodeEvent(view).error, Error::kNone);
}

TEST(EpisodeEventPolicyTest, ComputeConfigDigestKnownVectors) {
  EXPECT_EQ(ComputeConfigDigest("abc"),
            "ba7816bf8f01cfea414140de5dae2223b00361a396177a9cb410ff61f20015ad");
  EXPECT_EQ(ComputeConfigDigest(""), "");
  EXPECT_TRUE(IsLowercaseSha256Hex(ComputeConfigDigest("abc")));
}

TEST(EpisodeEventPolicyTest, UnknownPayloadIsSkippedNotRejected) {
  EpisodeEventView view = NominalView();
  view.payload_case = kPayloadCaseNotSet;
  view.has_unknown_fields = true;
  view.unknown_payload_candidate = true;
  const EpisodeEventAssessment assessment = AssessEpisodeEvent(view);
  EXPECT_EQ(assessment.error, Error::kNone);
  EXPECT_EQ(assessment.payload_kind, PayloadKind::kUnknown);
  EXPECT_TRUE(assessment.skip);
}

TEST(EpisodeEventPolicyTest, UnknownFieldsDoNotChangeAKnownPayload) {
  EpisodeEventView view = NominalView();
  view.has_unknown_fields = true;
  view.unknown_payload_candidate = true;
  const EpisodeEventAssessment assessment = AssessEpisodeEvent(view);
  EXPECT_EQ(assessment.error, Error::kNone);
  EXPECT_EQ(assessment.payload_kind, PayloadKind::kSafetyDecision);
  EXPECT_FALSE(assessment.skip);
}

TEST(EpisodeEventPolicyTest, InvalidEnvelopeIsNeverSkipped) {
  EpisodeEventView view = NominalView();
  view.payload_case = kPayloadCaseNotSet;
  view.has_unknown_fields = true;
  view.frame_id = "";
  const EpisodeEventAssessment assessment = AssessEpisodeEvent(view);
  EXPECT_EQ(assessment.error, Error::kMissingFrame);
  EXPECT_FALSE(assessment.skip);
}

TEST(EpisodeEventPolicyTest, OnlyUnknownFieldsFailsOnMissingHeaderFirst) {
  EpisodeEventView view;
  view.has_unknown_fields = true;
  const EpisodeEventAssessment assessment = AssessEpisodeEvent(view);
  EXPECT_EQ(assessment.error, Error::kMissingHeader);
  EXPECT_FALSE(assessment.skip);
}

TEST(EpisodeEventPolicyTest, FirstFailingCheckWins) {
  EpisodeEventView view;
  view.provenance_present = true;
  view.payload_case = kPayloadCaseNotSet;
  const std::string bad = "ZZ";
  view.snapshot_id = bad;
  view.config_digest = bad;
  EXPECT_EQ(AssessEpisodeEvent(view).error, Error::kMissingHeader);
  view.header_present = true;
  EXPECT_EQ(AssessEpisodeEvent(view).error, Error::kSourceId);
  view.source_id = "s";
  EXPECT_EQ(AssessEpisodeEvent(view).error, Error::kMissingFrame);
  view.frame_id = "f";
  EXPECT_EQ(AssessEpisodeEvent(view).error, Error::kSourceTime);
  view.source_time_present = true;
  EXPECT_EQ(AssessEpisodeEvent(view).error, Error::kReceiveTime);
  view.receive_time_present = true;
  EXPECT_EQ(AssessEpisodeEvent(view).error, Error::kSchemaVersion);
  view.schema_version = 1;
  EXPECT_EQ(AssessEpisodeEvent(view).error, Error::kSnapshotId);
  view.snapshot_id = "";
  EXPECT_EQ(AssessEpisodeEvent(view).error, Error::kConfigDigest);
  view.config_digest = "";
  EXPECT_EQ(AssessEpisodeEvent(view).error, Error::kMissingPayload);
}

TEST(EpisodeEventPolicyTest, WellFormedButWrongDigestIsAMismatch) {
  MetadataFixture fixture;
  fixture.view.config_digest = kSnapshot;
  fixture.view.snapshot_id = "";
  EXPECT_EQ(AssessEpisodeEvent(fixture.view).error,
            Error::kConfigDigestMismatch);
}

TEST(EpisodeEventPolicyTest, TagNamespaceConstantsArePermanent) {
  EXPECT_EQ(kFirstFuturePayloadTag, 19);
  EXPECT_EQ(kFirstFuturePayloadTag, kPayloadCaseDesiredMotion + 1);
  EXPECT_EQ(kMaxFieldNumber, 536870911);
  EXPECT_EQ(kMaxFieldNumber, (1 << 29) - 1);
  EXPECT_EQ(kExcludedFieldNumberFirst, 19000);
  EXPECT_EQ(kExcludedFieldNumberLast, 19999);
  EXPECT_EQ(kWireTypeLengthDelimited, 2);
}

TEST(EpisodeEventPolicyTest, B15B16B19UnknownPayloadCandidateBoundaries) {
  // Known payload tags are never candidates.
  for (int tag = kPayloadCaseSafetyDecision; tag <= kPayloadCaseDesiredMotion;
       ++tag) {
    EXPECT_FALSE(IsUnknownPayloadCandidate(tag, 2)) << tag;
  }
  EXPECT_FALSE(IsUnknownPayloadCandidate(15, 2));
  EXPECT_FALSE(IsUnknownPayloadCandidate(18, 2));
  // Envelope tags are never candidates.
  for (int tag = 1; tag <= 9; ++tag) {
    EXPECT_FALSE(IsUnknownPayloadCandidate(tag, 2)) << tag;
  }
  EXPECT_FALSE(IsUnknownPayloadCandidate(6, 2));
  EXPECT_FALSE(IsUnknownPayloadCandidate(0, 2));
  EXPECT_FALSE(IsUnknownPayloadCandidate(-1, 2));
  // Lowest candidate.
  EXPECT_TRUE(IsUnknownPayloadCandidate(19, 2));
  EXPECT_TRUE(IsUnknownPayloadCandidate(20, 2));
  EXPECT_TRUE(IsUnknownPayloadCandidate(99, 2));
  EXPECT_TRUE(IsUnknownPayloadCandidate(100, 2));
  EXPECT_TRUE(IsUnknownPayloadCandidate(536870000, 2));
  // The excluded range.
  EXPECT_TRUE(IsUnknownPayloadCandidate(18999, 2));
  EXPECT_FALSE(IsUnknownPayloadCandidate(19000, 2));
  EXPECT_FALSE(IsUnknownPayloadCandidate(19500, 2));
  EXPECT_FALSE(IsUnknownPayloadCandidate(19999, 2));
  EXPECT_TRUE(IsUnknownPayloadCandidate(20000, 2));
  // The maximum field number.
  EXPECT_TRUE(IsUnknownPayloadCandidate(536870911, 2));
  EXPECT_FALSE(IsUnknownPayloadCandidate(536870912, 2));
}

TEST(EpisodeEventPolicyTest, OnlyLengthDelimitedWireTypeIsACandidate) {
  for (const int number : {19, 99, 18999, 20000, 536870911}) {
    for (const int wire_type : {0, 1, 3, 4, 5, 6, 7}) {
      EXPECT_FALSE(IsUnknownPayloadCandidate(number, wire_type))
          << number << " wire " << wire_type;
    }
    EXPECT_TRUE(IsUnknownPayloadCandidate(number, 2)) << number;
  }
}

TEST(EpisodeEventPolicyTest, UnknownFieldsWithoutACandidateAreMissingPayload) {
  EpisodeEventView view = NominalView();
  view.payload_case = kPayloadCaseNotSet;
  view.has_unknown_fields = true;
  view.unknown_payload_candidate = false;
  const EpisodeEventAssessment assessment = AssessEpisodeEvent(view);
  EXPECT_EQ(assessment.error, Error::kMissingPayload);
  EXPECT_EQ(assessment.payload_kind, PayloadKind::kNone);
  EXPECT_FALSE(assessment.skip);
  EXPECT_TRUE(assessment.engaged);
  EXPECT_EQ(ClassifyPayload(kPayloadCaseNotSet, false), PayloadKind::kNone);
  EXPECT_EQ(ClassifyPayload(kPayloadCaseNotSet, true), PayloadKind::kUnknown);
}

TEST(EpisodeEventPolicyTest, EpisodeLossCase15IsKnownAndNeverSkipped) {
  EXPECT_EQ(kPayloadCaseEpisodeLoss, 15);
  EXPECT_EQ(static_cast<int>(PayloadKind::kEpisodeLoss), 7);
  for (const bool has_unknown : {false, true}) {
    for (const bool candidate : {false, true}) {
      EXPECT_EQ(ClassifyPayload(kPayloadCaseEpisodeLoss, candidate),
                PayloadKind::kEpisodeLoss);
      EpisodeEventView view = NominalView();
      view.payload_case = kPayloadCaseEpisodeLoss;
      view.has_unknown_fields = has_unknown;
      view.unknown_payload_candidate = candidate;
      const EpisodeEventAssessment assessment = AssessEpisodeEvent(view);
      EXPECT_EQ(assessment.error, Error::kNone);
      EXPECT_EQ(assessment.payload_kind, PayloadKind::kEpisodeLoss);
      EXPECT_FALSE(assessment.skip);
      EXPECT_TRUE(assessment.engaged);
    }
  }
}
}  // namespace
}  // namespace intrinsic::episode
