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

#include "intrinsic/safety/safety_decision_assessor.h"

#include <cmath>
#include <limits>
#include <string>

#include "gtest/gtest.h"
#include "intrinsic/embodiment/proto/stamped_header.pb.h"
#include "intrinsic/safety/proto/safety_decision.pb.h"
#include "intrinsic/safety/safety_decision_policy.h"
#include "intrinsic/vehicle/proto/vehicle_command.pb.h"
#include "intrinsic/vehicle/vehicle_contract_policy.h"
#include "intrinsic/world/world_snapshot/world_snapshot_policy.h"

namespace intrinsic::safety {
namespace {

using intrinsic_proto::embodiment::StampedHeader;
using intrinsic_proto::embodiment::Validity;
using intrinsic_proto::safety::SafetyDecision;
using intrinsic_proto::vehicle::DesiredMotion;

void FillHeader(StampedHeader* header, const std::string& frame_id) {
  header->set_sequence(1);
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
  FillHeader(motion.mutable_header(), "body");
  motion.mutable_twist()->mutable_body_twist()->set_linear_x_m_s(0.25);
  return motion;
}

// Ascent intent: a world-frame pose that is shallower than the vehicle.
DesiredMotion AscentPoseMotion() {
  DesiredMotion motion;
  FillHeader(motion.mutable_header(), "world_enu");
  auto* pose = motion.mutable_pose()->mutable_pose_world_from_body();
  pose->mutable_position()->set_x(4);
  pose->mutable_position()->set_z(0);
  pose->mutable_orientation()->set_w(1);
  return motion;
}

SafetyDecision Decision(intrinsic_proto::safety::SafetyDecisionKind kind,
                        const DesiredMotion& original) {
  SafetyDecision decision;
  FillHeader(decision.mutable_header(), "");
  decision.set_kind(kind);
  decision.set_original_intent_digest(ComputeOriginalIntentDigest(original));
  decision.set_snapshot_id(std::string(64, '0'));
  return decision;
}

TEST(SafetyDecisionAssessorTest, EmptyDecisionIsNotEngaged) {
  const SafetyDecisionAssessment assessment =
      AssessSafetyDecisionProto(SafetyDecision());
  EXPECT_FALSE(assessment.engaged);
  EXPECT_EQ(assessment.error, SafetyDecisionError::kNone);
  EXPECT_FALSE(assessment.accepted);
}

TEST(SafetyDecisionAssessorTest,
     AcceptHasDigestAndSnapshotIdAndNoAppliedIntent) {
  const SafetyDecision decision = Decision(
      intrinsic_proto::safety::SAFETY_DECISION_KIND_ACCEPT, TwistMotion());
  EXPECT_FALSE(decision.has_applied_intent());
  const SafetyDecisionAssessment assessment =
      AssessSafetyDecisionProto(decision);
  EXPECT_EQ(assessment.error, SafetyDecisionError::kNone);
  EXPECT_EQ(assessment.kind, SafetyDecisionKind::kAccept);
  EXPECT_TRUE(assessment.accepted);
}

TEST(SafetyDecisionAssessorTest, ProjectWithValidAppliedIntentIsAccepted) {
  SafetyDecision decision = Decision(
      intrinsic_proto::safety::SAFETY_DECISION_KIND_PROJECT, TwistMotion());
  *decision.mutable_applied_intent() = TwistMotion();
  decision.mutable_applied_intent()
      ->mutable_twist()
      ->mutable_body_twist()
      ->set_linear_x_m_s(0.1);
  const SafetyDecisionAssessment assessment =
      AssessSafetyDecisionProto(decision);
  EXPECT_EQ(assessment.error, SafetyDecisionError::kNone);
  EXPECT_TRUE(assessment.accepted);
}

TEST(SafetyDecisionAssessorTest, RejectAndAbortWithoutAppliedIntent) {
  for (auto kind : {intrinsic_proto::safety::SAFETY_DECISION_KIND_REJECT,
                    intrinsic_proto::safety::SAFETY_DECISION_KIND_ABORT}) {
    const SafetyDecision decision = Decision(kind, TwistMotion());
    const SafetyDecisionAssessment assessment =
        AssessSafetyDecisionProto(decision);
    EXPECT_EQ(assessment.error, SafetyDecisionError::kNone) << kind;
    EXPECT_TRUE(assessment.accepted) << kind;
  }
}

TEST(SafetyDecisionAssessorTest, SurfaceWithPoseAndTwistAscentFixtures) {
  for (const DesiredMotion& ascent : {AscentPoseMotion(), TwistMotion()}) {
    SafetyDecision decision = Decision(
        intrinsic_proto::safety::SAFETY_DECISION_KIND_SURFACE, TwistMotion());
    *decision.mutable_applied_intent() = ascent;
    const SafetyDecisionAssessment assessment =
        AssessSafetyDecisionProto(decision);
    EXPECT_EQ(assessment.error, SafetyDecisionError::kNone);
    EXPECT_EQ(assessment.kind, SafetyDecisionKind::kSurface);
    EXPECT_TRUE(assessment.accepted);
  }
}

TEST(SafetyDecisionAssessorTest, InvalidCombinations) {
  SafetyDecision project_missing = Decision(
      intrinsic_proto::safety::SAFETY_DECISION_KIND_PROJECT, TwistMotion());
  EXPECT_EQ(AssessSafetyDecisionProto(project_missing).error,
            SafetyDecisionError::kAppliedIntentMissing);

  SafetyDecision accept_with_intent = Decision(
      intrinsic_proto::safety::SAFETY_DECISION_KIND_ACCEPT, TwistMotion());
  *accept_with_intent.mutable_applied_intent() = TwistMotion();
  EXPECT_EQ(AssessSafetyDecisionProto(accept_with_intent).error,
            SafetyDecisionError::kAppliedIntentUnexpected);

  SafetyDecision empty_applied = Decision(
      intrinsic_proto::safety::SAFETY_DECISION_KIND_PROJECT, TwistMotion());
  empty_applied.mutable_applied_intent();
  EXPECT_EQ(AssessSafetyDecisionProto(empty_applied).error,
            SafetyDecisionError::kAppliedIntentInvalid);

  SafetyDecision bad_digest = Decision(
      intrinsic_proto::safety::SAFETY_DECISION_KIND_ACCEPT, TwistMotion());
  bad_digest.set_original_intent_digest("ABC");
  EXPECT_EQ(AssessSafetyDecisionProto(bad_digest).error,
            SafetyDecisionError::kOriginalIntentDigest);

  SafetyDecision upper_digest = Decision(
      intrinsic_proto::safety::SAFETY_DECISION_KIND_ACCEPT, TwistMotion());
  std::string upper = upper_digest.original_intent_digest();
  upper[0] = 'F';
  upper_digest.set_original_intent_digest(upper);
  EXPECT_EQ(AssessSafetyDecisionProto(upper_digest).error,
            SafetyDecisionError::kOriginalIntentDigest);

  SafetyDecision short_snapshot = Decision(
      intrinsic_proto::safety::SAFETY_DECISION_KIND_ACCEPT, TwistMotion());
  short_snapshot.set_snapshot_id(std::string(63, '0'));
  EXPECT_EQ(AssessSafetyDecisionProto(short_snapshot).error,
            SafetyDecisionError::kSnapshotId);

  SafetyDecision upper_snapshot = Decision(
      intrinsic_proto::safety::SAFETY_DECISION_KIND_ACCEPT, TwistMotion());
  upper_snapshot.set_snapshot_id(std::string(64, 'A'));
  EXPECT_EQ(AssessSafetyDecisionProto(upper_snapshot).error,
            SafetyDecisionError::kSnapshotId);

  SafetyDecision empty_rule = Decision(
      intrinsic_proto::safety::SAFETY_DECISION_KIND_REJECT, TwistMotion());
  empty_rule.add_findings()->set_rule_id("ok");
  empty_rule.add_findings()->set_summary("no rule id");
  const SafetyDecisionAssessment assessment =
      AssessSafetyDecisionProto(empty_rule);
  EXPECT_EQ(assessment.error, SafetyDecisionError::kFindingRuleId);
  EXPECT_EQ(assessment.finding_index, 1);
  EXPECT_FALSE(assessment.accepted);

  SafetyDecision no_header = Decision(
      intrinsic_proto::safety::SAFETY_DECISION_KIND_REJECT, TwistMotion());
  no_header.clear_header();
  EXPECT_EQ(AssessSafetyDecisionProto(no_header).error,
            SafetyDecisionError::kMissingHeader);

  SafetyDecision unspecified = Decision(
      intrinsic_proto::safety::SAFETY_DECISION_KIND_UNSPECIFIED, TwistMotion());
  EXPECT_EQ(AssessSafetyDecisionProto(unspecified).error,
            SafetyDecisionError::kUnspecifiedKind);
}

TEST(SafetyDecisionAssessorTest, AppliedIntentMustPassDesiredMotionAssessment) {
  SafetyDecision non_finite = Decision(
      intrinsic_proto::safety::SAFETY_DECISION_KIND_PROJECT, TwistMotion());
  *non_finite.mutable_applied_intent() = TwistMotion();
  non_finite.mutable_applied_intent()
      ->mutable_twist()
      ->mutable_body_twist()
      ->set_linear_x_m_s(std::numeric_limits<double>::quiet_NaN());
  SafetyDecisionAssessment assessment = AssessSafetyDecisionProto(non_finite);
  EXPECT_EQ(assessment.error, SafetyDecisionError::kAppliedIntentInvalid);
  EXPECT_EQ(assessment.applied_intent_error,
            vehicle::ContractError::kNonFinite);
  EXPECT_FALSE(assessment.accepted);

  SafetyDecision bad_quaternion = Decision(
      intrinsic_proto::safety::SAFETY_DECISION_KIND_SURFACE, TwistMotion());
  *bad_quaternion.mutable_applied_intent() = AscentPoseMotion();
  bad_quaternion.mutable_applied_intent()
      ->mutable_pose()
      ->mutable_pose_world_from_body()
      ->mutable_orientation()
      ->set_w(2);
  assessment = AssessSafetyDecisionProto(bad_quaternion);
  EXPECT_EQ(assessment.applied_intent_error,
            vehicle::ContractError::kQuaternion);

  SafetyDecision wrong_frame = Decision(
      intrinsic_proto::safety::SAFETY_DECISION_KIND_PROJECT, TwistMotion());
  *wrong_frame.mutable_applied_intent() = TwistMotion();
  wrong_frame.mutable_applied_intent()->mutable_header()->set_frame_id(
      "world_enu");
  assessment = AssessSafetyDecisionProto(wrong_frame);
  EXPECT_EQ(assessment.applied_intent_error,
            vehicle::ContractError::kBodyFrame);
}

TEST(SafetyDecisionAssessorTest, UnknownKindIsNotAcceptedAndNotRewritten) {
  SafetyDecision decision =
      Decision(static_cast<intrinsic_proto::safety::SafetyDecisionKind>(99),
               TwistMotion());
  const std::string before = decision.SerializeAsString();
  const SafetyDecisionAssessment assessment =
      AssessSafetyDecisionProto(decision);
  EXPECT_EQ(assessment.error, SafetyDecisionError::kNone);
  EXPECT_EQ(assessment.kind, SafetyDecisionKind::kUnknown);
  EXPECT_FALSE(assessment.accepted);
  EXPECT_EQ(static_cast<int>(decision.kind()), 99);
  EXPECT_EQ(decision.SerializeAsString(), before);
}

TEST(SafetyDecisionAssessorTest, SnapshotIdFromWorldSnapshotPolicyIsAccepted) {
  const std::string snapshot_id = world::ComputeSnapshotId(
      3, {{"current_field", 7}, {"occupancy_reference", 1}});
  ASSERT_EQ(snapshot_id.size(), kSha256HexLength);
  SafetyDecision decision = Decision(
      intrinsic_proto::safety::SAFETY_DECISION_KIND_ACCEPT, TwistMotion());
  decision.set_snapshot_id(snapshot_id);
  EXPECT_TRUE(AssessSafetyDecisionProto(decision).accepted);
}

TEST(SafetyDecisionAssessorTest, OriginalIntentDigestIsStableAndSensitive) {
  const DesiredMotion motion = TwistMotion();
  const std::string digest = ComputeOriginalIntentDigest(motion);
  EXPECT_EQ(digest, ComputeOriginalIntentDigest(TwistMotion()));
  EXPECT_TRUE(IsLowercaseSha256Hex(digest));
  DesiredMotion changed = motion;
  changed.mutable_twist()->mutable_body_twist()->set_linear_x_m_s(0.5);
  EXPECT_NE(digest, ComputeOriginalIntentDigest(changed));
}

}  // namespace
}  // namespace intrinsic::safety
