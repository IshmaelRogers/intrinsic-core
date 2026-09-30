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

#include "intrinsic/safety/safety_decision_policy.h"

#include <array>
#include <limits>
#include <string>

#include "gtest/gtest.h"
#include "intrinsic/embodiment/frame_policy.h"
#include "intrinsic/embodiment/stamped_header_policy.h"
#include "intrinsic/vehicle/vehicle_contract_policy.h"

namespace intrinsic::safety {
namespace {

using embodiment::ValidityKind;

const std::string kDigest(64, 'a');
const std::string kSnapshotId(64, '0');

vehicle::DesiredMotionView ValidTwistMotion() {
  vehicle::DesiredMotionView motion;
  motion.header_present = true;
  motion.validity_present = true;
  motion.validity_state = 1;
  motion.frame_id = vehicle::kBodyFrameId;
  motion.objective = vehicle::ObjectiveKind::kTwist;
  motion.twist = vehicle::BodyVector{0.25, 0, 0, 0, 0, 0};
  return motion;
}

SafetyDecisionView ValidDecision(int kind) {
  SafetyDecisionView decision;
  decision.header_present = true;
  decision.header_validity_present = true;
  decision.header_validity_state = 1;
  decision.kind = kind;
  decision.original_intent_digest = kDigest;
  decision.snapshot_id = kSnapshotId;
  return decision;
}

SafetyDecisionView ValidProject() {
  SafetyDecisionView decision = ValidDecision(2);
  decision.applied_intent_present = true;
  decision.applied_intent = ValidTwistMotion();
  return decision;
}

TEST(SafetyDecisionPolicyTest, EmptyDecisionIsNotEngagedAndNotAnError) {
  const SafetyDecisionAssessment assessment =
      AssessSafetyDecision(SafetyDecisionView{});
  EXPECT_FALSE(assessment.engaged);
  EXPECT_EQ(assessment.error, SafetyDecisionError::kNone);
  EXPECT_FALSE(assessment.accepted);
  EXPECT_EQ(assessment.finding_index, -1);
}

TEST(SafetyDecisionPolicyTest, AnyFieldEngagesTheDecision) {
  SafetyDecisionView kind_only;
  kind_only.kind = 3;
  SafetyDecisionView digest_only;
  digest_only.original_intent_digest = kDigest;
  SafetyDecisionView epoch_only;
  epoch_only.decision_epoch = 1;
  SafetyDecisionView zero_epoch_present;
  zero_epoch_present.decision_epoch_present = true;
  const std::array<SafetyFindingView, 1> findings = {SafetyFindingView{"r", 1}};
  SafetyDecisionView findings_only;
  findings_only.findings = findings;
  for (const SafetyDecisionView& decision :
       {kind_only, digest_only, epoch_only, zero_epoch_present,
        findings_only}) {
    const SafetyDecisionAssessment assessment = AssessSafetyDecision(decision);
    EXPECT_TRUE(assessment.engaged);
    EXPECT_EQ(assessment.error, SafetyDecisionError::kMissingHeader);
    EXPECT_FALSE(assessment.accepted);
  }
}

TEST(SafetyDecisionPolicyTest, AcceptWithoutAppliedIntentIsAccepted) {
  const SafetyDecisionAssessment assessment =
      AssessSafetyDecision(ValidDecision(1));
  EXPECT_TRUE(assessment.engaged);
  EXPECT_EQ(assessment.error, SafetyDecisionError::kNone);
  EXPECT_EQ(assessment.kind, SafetyDecisionKind::kAccept);
  EXPECT_EQ(assessment.header_validity, ValidityKind::kValid);
  EXPECT_TRUE(assessment.accepted);
}

TEST(SafetyDecisionPolicyTest, ProjectAndSurfaceWithValidAppliedIntent) {
  EXPECT_TRUE(AssessSafetyDecision(ValidProject()).accepted);
  SafetyDecisionView surface = ValidProject();
  surface.kind = 5;
  const SafetyDecisionAssessment assessment = AssessSafetyDecision(surface);
  EXPECT_EQ(assessment.error, SafetyDecisionError::kNone);
  EXPECT_EQ(assessment.kind, SafetyDecisionKind::kSurface);
  EXPECT_TRUE(assessment.accepted);
}

TEST(SafetyDecisionPolicyTest, RejectAndAbortAreAcceptedWithoutAppliedIntent) {
  for (int kind : {3, 4}) {
    const SafetyDecisionAssessment assessment =
        AssessSafetyDecision(ValidDecision(kind));
    EXPECT_EQ(assessment.error, SafetyDecisionError::kNone) << kind;
    EXPECT_TRUE(assessment.accepted) << kind;
  }
}

TEST(SafetyDecisionPolicyTest, AppliedIntentPresenceRulesPerKind) {
  for (int kind : {2, 5}) {
    SafetyDecisionView missing = ValidDecision(kind);
    const SafetyDecisionAssessment assessment = AssessSafetyDecision(missing);
    EXPECT_EQ(assessment.error, SafetyDecisionError::kAppliedIntentMissing)
        << kind;
    EXPECT_FALSE(assessment.accepted) << kind;
  }
  for (int kind : {1, 3, 4}) {
    SafetyDecisionView unexpected = ValidProject();
    unexpected.kind = kind;
    const SafetyDecisionAssessment assessment =
        AssessSafetyDecision(unexpected);
    EXPECT_EQ(assessment.error, SafetyDecisionError::kAppliedIntentUnexpected)
        << kind;
    EXPECT_FALSE(assessment.accepted) << kind;
  }
}

TEST(SafetyDecisionPolicyTest, EmptyAppliedIntentMessageIsStillPresent) {
  SafetyDecisionView accept = ValidDecision(1);
  accept.applied_intent_present = true;
  EXPECT_EQ(AssessSafetyDecision(accept).error,
            SafetyDecisionError::kAppliedIntentUnexpected);
  SafetyDecisionView project = ValidDecision(2);
  project.applied_intent_present = true;
  const SafetyDecisionAssessment assessment = AssessSafetyDecision(project);
  EXPECT_EQ(assessment.error, SafetyDecisionError::kAppliedIntentInvalid);
  EXPECT_FALSE(assessment.accepted);
}

TEST(SafetyDecisionPolicyTest, AppliedIntentDelegatesToDesiredMotion) {
  SafetyDecisionView bad_frame = ValidProject();
  bad_frame.applied_intent.frame_id = "world_enu";
  SafetyDecisionAssessment assessment = AssessSafetyDecision(bad_frame);
  EXPECT_EQ(assessment.error, SafetyDecisionError::kAppliedIntentInvalid);
  EXPECT_EQ(assessment.applied_intent_error,
            vehicle::ContractError::kBodyFrame);
  EXPECT_FALSE(assessment.accepted);

  SafetyDecisionView non_finite = ValidProject();
  non_finite.applied_intent.twist.linear_x =
      std::numeric_limits<double>::quiet_NaN();
  assessment = AssessSafetyDecision(non_finite);
  EXPECT_EQ(assessment.error, SafetyDecisionError::kAppliedIntentInvalid);
  EXPECT_EQ(assessment.applied_intent_error,
            vehicle::ContractError::kNonFinite);

  SafetyDecisionView infinite = ValidProject();
  infinite.applied_intent.twist.angular_z =
      std::numeric_limits<double>::infinity();
  EXPECT_EQ(AssessSafetyDecision(infinite).applied_intent_error,
            vehicle::ContractError::kNonFinite);

  SafetyDecisionView not_valid = ValidProject();
  not_valid.applied_intent.validity_state = 2;
  EXPECT_EQ(AssessSafetyDecision(not_valid).error,
            SafetyDecisionError::kAppliedIntentInvalid);
}

TEST(SafetyDecisionPolicyTest, DigestAndSnapshotIdFormat) {
  EXPECT_TRUE(IsLowercaseSha256Hex(std::string(64, 'f')));
  EXPECT_TRUE(IsLowercaseSha256Hex(std::string(64, '9')));
  EXPECT_FALSE(IsLowercaseSha256Hex(""));
  EXPECT_FALSE(IsLowercaseSha256Hex(std::string(63, 'a')));
  EXPECT_FALSE(IsLowercaseSha256Hex(std::string(65, 'a')));
  EXPECT_FALSE(IsLowercaseSha256Hex(std::string(64, 'A')));
  EXPECT_FALSE(IsLowercaseSha256Hex(std::string(64, 'g')));
  EXPECT_FALSE(IsLowercaseSha256Hex("sha256:" + std::string(57, 'a')));

  for (const std::string& bad : {std::string(), std::string(63, 'a'),
                                 std::string(65, 'a'), std::string(64, 'A')}) {
    SafetyDecisionView digest = ValidDecision(1);
    digest.original_intent_digest = bad;
    EXPECT_EQ(AssessSafetyDecision(digest).error,
              SafetyDecisionError::kOriginalIntentDigest);
    SafetyDecisionView snapshot = ValidDecision(1);
    snapshot.snapshot_id = bad;
    const SafetyDecisionAssessment assessment = AssessSafetyDecision(snapshot);
    EXPECT_EQ(assessment.error, SafetyDecisionError::kSnapshotId);
    EXPECT_FALSE(assessment.accepted);
  }
}

TEST(SafetyDecisionPolicyTest, FindingsRequireRuleIdAndAllowDuplicates) {
  const std::array<SafetyFindingView, 2> duplicates = {
      SafetyFindingView{"speed_limit", 2}, SafetyFindingView{"speed_limit", 3}};
  SafetyDecisionView decision = ValidDecision(3);
  decision.findings = duplicates;
  EXPECT_TRUE(AssessSafetyDecision(decision).accepted);

  const std::array<SafetyFindingView, 3> with_empty = {
      SafetyFindingView{"a", 1}, SafetyFindingView{"", 1},
      SafetyFindingView{"", 2}};
  decision.findings = with_empty;
  const SafetyDecisionAssessment assessment = AssessSafetyDecision(decision);
  EXPECT_EQ(assessment.error, SafetyDecisionError::kFindingRuleId);
  EXPECT_EQ(assessment.finding_index, 1);
  EXPECT_FALSE(assessment.accepted);
}

TEST(SafetyDecisionPolicyTest, UnknownSeverityIsNotADefect) {
  const std::array<SafetyFindingView, 2> findings = {
      SafetyFindingView{"a", 99}, SafetyFindingView{"b", -1}};
  SafetyDecisionView decision = ValidDecision(4);
  decision.findings = findings;
  EXPECT_TRUE(AssessSafetyDecision(decision).accepted);
  EXPECT_EQ(ClassifySafetyFindingSeverity(99),
            SafetyFindingSeverityKind::kUnknown);
  EXPECT_EQ(ClassifySafetyFindingSeverity(-1),
            SafetyFindingSeverityKind::kUnknown);
  EXPECT_EQ(ClassifySafetyFindingSeverity(0),
            SafetyFindingSeverityKind::kUnspecified);
  EXPECT_EQ(ClassifySafetyFindingSeverity(1), SafetyFindingSeverityKind::kInfo);
  EXPECT_EQ(ClassifySafetyFindingSeverity(2),
            SafetyFindingSeverityKind::kWarning);
  EXPECT_EQ(ClassifySafetyFindingSeverity(3),
            SafetyFindingSeverityKind::kError);
  EXPECT_EQ(ClassifySafetyFindingSeverity(4),
            SafetyFindingSeverityKind::kCritical);
}

TEST(SafetyDecisionPolicyTest, UnspecifiedKindIsRejectedAsStructuralDefect) {
  const SafetyDecisionAssessment assessment =
      AssessSafetyDecision(ValidDecision(0));
  EXPECT_EQ(assessment.error, SafetyDecisionError::kUnspecifiedKind);
  EXPECT_EQ(assessment.kind, SafetyDecisionKind::kUnspecified);
  EXPECT_FALSE(assessment.accepted);
}

TEST(SafetyDecisionPolicyTest,
     UnknownKindIsNotAStructuralDefectAndNotAccepted) {
  for (int kind : {6, 99, -1}) {
    const SafetyDecisionAssessment assessment =
        AssessSafetyDecision(ValidDecision(kind));
    EXPECT_TRUE(assessment.engaged);
    EXPECT_EQ(assessment.error, SafetyDecisionError::kNone) << kind;
    EXPECT_EQ(assessment.kind, SafetyDecisionKind::kUnknown) << kind;
    EXPECT_FALSE(assessment.accepted) << kind;
  }
  EXPECT_EQ(ClassifySafetyDecisionKind(99), SafetyDecisionKind::kUnknown);
  EXPECT_NE(ClassifySafetyDecisionKind(99), SafetyDecisionKind::kReject);
  EXPECT_NE(ClassifySafetyDecisionKind(99), SafetyDecisionKind::kAbort);
}

TEST(SafetyDecisionPolicyTest, UnknownKindStillReportsOtherDefects) {
  SafetyDecisionView decision = ValidDecision(99);
  decision.snapshot_id = "";
  EXPECT_EQ(AssessSafetyDecision(decision).error,
            SafetyDecisionError::kSnapshotId);
  SafetyDecisionView with_intent = ValidProject();
  with_intent.kind = 99;
  const SafetyDecisionAssessment assessment = AssessSafetyDecision(with_intent);
  EXPECT_EQ(assessment.error, SafetyDecisionError::kNone);
  EXPECT_FALSE(assessment.accepted);
}

TEST(SafetyDecisionPolicyTest, KindClassificationIsExhaustiveForKnownValues) {
  EXPECT_EQ(ClassifySafetyDecisionKind(0), SafetyDecisionKind::kUnspecified);
  EXPECT_EQ(ClassifySafetyDecisionKind(1), SafetyDecisionKind::kAccept);
  EXPECT_EQ(ClassifySafetyDecisionKind(2), SafetyDecisionKind::kProject);
  EXPECT_EQ(ClassifySafetyDecisionKind(3), SafetyDecisionKind::kReject);
  EXPECT_EQ(ClassifySafetyDecisionKind(4), SafetyDecisionKind::kAbort);
  EXPECT_EQ(ClassifySafetyDecisionKind(5), SafetyDecisionKind::kSurface);
  EXPECT_EQ(ClassifySafetyDecisionKind(6), SafetyDecisionKind::kUnknown);
}

TEST(SafetyDecisionPolicyTest, HeaderRules) {
  SafetyDecisionView no_header = ValidDecision(1);
  no_header.header_present = false;
  EXPECT_EQ(AssessSafetyDecision(no_header).error,
            SafetyDecisionError::kMissingHeader);

  SafetyDecisionView invalid = ValidDecision(1);
  invalid.header_validity_state = 2;
  SafetyDecisionAssessment assessment = AssessSafetyDecision(invalid);
  EXPECT_EQ(assessment.error, SafetyDecisionError::kHeaderInvalid);
  EXPECT_EQ(assessment.header_validity, ValidityKind::kInvalid);
  EXPECT_FALSE(assessment.accepted);

  SafetyDecisionView bad_nanos = ValidDecision(1);
  bad_nanos.header_source_time_present = true;
  bad_nanos.header_source_time_nanos = 1000000000;
  EXPECT_EQ(AssessSafetyDecision(bad_nanos).error,
            SafetyDecisionError::kHeaderTime);
  SafetyDecisionView bad_receive = ValidDecision(1);
  bad_receive.header_receive_time_present = true;
  bad_receive.header_receive_time_nanos = -1;
  EXPECT_EQ(AssessSafetyDecision(bad_receive).error,
            SafetyDecisionError::kHeaderTime);
}

TEST(SafetyDecisionPolicyTest, HeaderWithoutValidValidityIsNotAccepted) {
  for (bool present : {false, true}) {
    SafetyDecisionView decision = ValidDecision(3);
    decision.header_validity_present = present;
    decision.header_validity_state = 0;
    const SafetyDecisionAssessment assessment = AssessSafetyDecision(decision);
    EXPECT_EQ(assessment.error, SafetyDecisionError::kNone);
    EXPECT_FALSE(assessment.accepted);
  }
  SafetyDecisionView unknown_state = ValidDecision(3);
  unknown_state.header_validity_state = 99;
  EXPECT_FALSE(AssessSafetyDecision(unknown_state).accepted);
  EXPECT_EQ(AssessSafetyDecision(unknown_state).header_validity,
            ValidityKind::kUnspecified);
}

TEST(SafetyDecisionPolicyTest, FirstDefectWins) {
  SafetyDecisionView decision;
  decision.kind = 0;
  decision.original_intent_digest = "bad";
  decision.snapshot_id = "bad";
  decision.applied_intent_present = true;
  const std::array<SafetyFindingView, 1> findings = {SafetyFindingView{"", 1}};
  decision.findings = findings;
  EXPECT_EQ(AssessSafetyDecision(decision).error,
            SafetyDecisionError::kMissingHeader);
  decision.header_present = true;
  EXPECT_EQ(AssessSafetyDecision(decision).error,
            SafetyDecisionError::kUnspecifiedKind);
  decision.kind = 1;
  EXPECT_EQ(AssessSafetyDecision(decision).error,
            SafetyDecisionError::kOriginalIntentDigest);
  decision.original_intent_digest = kDigest;
  EXPECT_EQ(AssessSafetyDecision(decision).error,
            SafetyDecisionError::kSnapshotId);
  decision.snapshot_id = kSnapshotId;
  EXPECT_EQ(AssessSafetyDecision(decision).error,
            SafetyDecisionError::kAppliedIntentUnexpected);
  decision.applied_intent_present = false;
  EXPECT_EQ(AssessSafetyDecision(decision).error,
            SafetyDecisionError::kFindingRuleId);
}

TEST(SafetyDecisionPolicyTest, ErrorNamesAreStable) {
  EXPECT_STREQ(SafetyDecisionErrorName(SafetyDecisionError::kNone), "none");
  EXPECT_STREQ(SafetyDecisionErrorName(SafetyDecisionError::kSnapshotId),
               "snapshot_id");
  EXPECT_STREQ(SafetyDecisionErrorName(SafetyDecisionError::kFindingRuleId),
               "finding_rule_id");
}

}  // namespace
}  // namespace intrinsic::safety
