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

#include "intrinsic/safety/reference_safety_filter.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <string>
#include <vector>

#include "intrinsic/embodiment/proto/stamped_header.pb.h"
#include "intrinsic/safety/proto/authority_mode.pb.h"
#include "intrinsic/safety/proto/safety_decision.pb.h"
#include "intrinsic/safety/safety_decision_assessor.h"
#include "intrinsic/safety/safety_decision_policy.h"
#include "intrinsic/vehicle/proto/vehicle_command.pb.h"
#include "intrinsic/vehicle/proto/vehicle_state.pb.h"
#include "intrinsic/world/proto/world_snapshot_descriptor.pb.h"
#include "gtest/gtest.h"

namespace intrinsic::safety {
namespace {

using intrinsic_proto::embodiment::StampedHeader;
using intrinsic_proto::embodiment::Validity;
using ProtoAuthorityMode = intrinsic_proto::safety::AuthorityMode;
using intrinsic_proto::safety::SafetyDecision;
using ProtoDecisionKind = intrinsic_proto::safety::SafetyDecisionKind;
using intrinsic_proto::vehicle::DesiredMotion;
using intrinsic_proto::vehicle::VehicleState;
using intrinsic_proto::world::WorldSnapshotDescriptor;

constexpr double kPi = 3.14159265358979323846;
constexpr StateTime kQueryTime{1700000010, 250000000};
constexpr ProtoAuthorityMode kConstrained =
    intrinsic_proto::safety::AUTHORITY_MODE_CONSTRAINED;
constexpr ProtoAuthorityMode kShadow =
    intrinsic_proto::safety::AUTHORITY_MODE_SHADOW;
constexpr ProtoAuthorityMode kRecommend =
    intrinsic_proto::safety::AUTHORITY_MODE_RECOMMEND;
constexpr ProtoAuthorityMode kRevoked =
    intrinsic_proto::safety::AUTHORITY_MODE_REVOKED;
constexpr ProtoAuthorityMode kEmergency =
    intrinsic_proto::safety::AUTHORITY_MODE_EMERGENCY;

constexpr ProtoDecisionKind kAccept =
    intrinsic_proto::safety::SAFETY_DECISION_KIND_ACCEPT;
constexpr ProtoDecisionKind kProject =
    intrinsic_proto::safety::SAFETY_DECISION_KIND_PROJECT;
constexpr ProtoDecisionKind kReject =
    intrinsic_proto::safety::SAFETY_DECISION_KIND_REJECT;
constexpr ProtoDecisionKind kSurface =
    intrinsic_proto::safety::SAFETY_DECISION_KIND_SURFACE;

// The fence strings are string literals, so they outlive every filter.
ReferenceSafetyFilterConfig Config() {
  ReferenceSafetyFilterConfig config;
  config.surface_z_m = 0.0;
  config.fence = AabbGeofence{"world_enu", "reef", -100.0, -100.0,
                              -500.0,      100.0,  100.0,  10.0};
  return config;
}

std::string SnapshotId(char fill = 'a') { return std::string(64, fill); }

WorldSnapshotDescriptor Snapshot(const std::string &id) {
  WorldSnapshotDescriptor snapshot;
  snapshot.set_snapshot_id(id);
  return snapshot;
}

void FillHeader(StampedHeader *header, const std::string &frame_id,
                StateTime time) {
  header->set_sequence(1);
  header->mutable_source_time()->set_seconds(time.seconds);
  header->mutable_source_time()->set_nanos(time.nanos);
  header->set_source_id("test");
  header->set_frame_id(frame_id);
  header->set_clock_domain("sim");
  header->mutable_validity()->set_state(Validity::STATE_VALID);
}

// Single-axis or ZYX (yaw 0) rotation.
void SetOrientation(DesiredMotion *motion, double roll, double pitch) {
  const double cr = std::cos(roll / 2.0);
  const double sr = std::sin(roll / 2.0);
  const double cp = std::cos(pitch / 2.0);
  const double sp = std::sin(pitch / 2.0);
  auto *q = motion->mutable_pose()
                ->mutable_pose_world_from_body()
                ->mutable_orientation();
  q->set_w(cr * cp);
  q->set_x(sr * cp);
  q->set_y(cr * sp);
  q->set_z(-sr * sp);
}

DesiredMotion PoseIntent(double x, double y, double z, double roll = 0.0,
                         double pitch = 0.0,
                         const std::string &frame = "world_enu") {
  DesiredMotion motion;
  FillHeader(motion.mutable_header(), frame, kQueryTime);
  auto *position =
      motion.mutable_pose()->mutable_pose_world_from_body()->mutable_position();
  position->set_x(x);
  position->set_y(y);
  position->set_z(z);
  SetOrientation(&motion, roll, pitch);
  return motion;
}

DesiredMotion TwistIntent(double lx, double ly, double lz, double ax = 0.0,
                          double ay = 0.0, double az = 0.0) {
  DesiredMotion motion;
  FillHeader(motion.mutable_header(), "body", kQueryTime);
  auto *twist = motion.mutable_twist()->mutable_body_twist();
  twist->set_linear_x_m_s(lx);
  twist->set_linear_y_m_s(ly);
  twist->set_linear_z_m_s(lz);
  twist->set_angular_x_rad_s(ax);
  twist->set_angular_y_rad_s(ay);
  twist->set_angular_z_rad_s(az);
  return motion;
}

DesiredMotion TrajectoryIntent() {
  DesiredMotion motion;
  FillHeader(motion.mutable_header(), "world_enu", kQueryTime);
  motion.mutable_trajectory()->set_trajectory_id("trajectory-1");
  return motion;
}

VehicleState FreshState(StateTime age_origin = kQueryTime,
                        int64_t age_seconds = 1) {
  VehicleState state;
  FillHeader(state.mutable_header(), "world_enu",
             StateTime{age_origin.seconds - age_seconds, age_origin.nanos});
  state.mutable_pose_world_from_body()->mutable_orientation()->set_w(1.0);
  state.mutable_body_twist();
  return state;
}

std::vector<std::string> Ids(const SafetyDecision &decision) {
  std::vector<std::string> ids;
  for (const auto &finding : decision.findings()) {
    ids.push_back(finding.rule_id());
  }
  return ids;
}

// Every returned decision must pass the structural assessor.
void ExpectAssessed(const ReferenceSafetyFilterResult &result) {
  ASSERT_EQ(result.error, ReferenceSafetyFilterError::kOk);
  const SafetyDecisionAssessment assessment =
      AssessSafetyDecisionProto(result.decision);
  EXPECT_TRUE(assessment.accepted);
  EXPECT_EQ(assessment.error, SafetyDecisionError::kNone);
}

ReferenceSafetyFilterResult
Eval(ReferenceSafetyFilter &filter, const DesiredMotion &requested,
     ProtoAuthorityMode held = kConstrained,
     const VehicleState &state = FreshState(),
     const std::string &snapshot_id = SnapshotId()) {
  return filter.Evaluate(requested, state, Snapshot(snapshot_id), held,
                         kQueryTime);
}

void ExpectEmptyError(const ReferenceSafetyFilterResult &result,
                      ReferenceSafetyFilterError error) {
  EXPECT_EQ(result.error, error);
  EXPECT_EQ(result.decision.ByteSizeLong(), 0u);
}

TEST(ReferenceSafetyFilterTest, F1CompliantPoseIsAccepted) {
  ReferenceSafetyFilter filter(Config());
  const DesiredMotion requested = PoseIntent(5.0, 5.0, -10.0);
  const auto result = Eval(filter, requested);
  ExpectAssessed(result);
  EXPECT_EQ(result.decision.kind(), kAccept);
  // ACCEPT carries no applied_intent: the applied intent is the requested one.
  EXPECT_FALSE(result.decision.has_applied_intent());
  EXPECT_TRUE(result.decision.findings().empty());
  EXPECT_EQ(result.decision.decision_epoch(), 1u);
  EXPECT_EQ(filter.next_epoch(), 2u);
  EXPECT_EQ(result.decision.header().validity().state(), Validity::STATE_VALID);
  EXPECT_EQ(result.decision.header().source_time().seconds(),
            kQueryTime.seconds);
  EXPECT_EQ(result.decision.header().source_time().nanos(), kQueryTime.nanos);
  EXPECT_EQ(result.decision.header().frame_id(), "world_enu");
}

TEST(ReferenceSafetyFilterTest, F2CompliantTwistIsAccepted) {
  ReferenceSafetyFilter filter(Config());
  const auto result = Eval(filter, TwistIntent(1.0, 0.0, 0.0, 0.0, 0.0, 0.25));
  ExpectAssessed(result);
  EXPECT_EQ(result.decision.kind(), kAccept);
  EXPECT_FALSE(result.decision.has_applied_intent());
  EXPECT_TRUE(result.decision.findings().empty());
  EXPECT_EQ(result.decision.header().frame_id(), "body");
}

TEST(ReferenceSafetyFilterTest, F3DeepPoseIsProjectedToMaxDepth) {
  ReferenceSafetyFilterConfig config = Config();
  config.surface_z_m = 2.0;
  ReferenceSafetyFilter filter(config);
  const DesiredMotion requested = PoseIntent(5.0, -6.0, -150.0);
  const auto result = Eval(filter, requested);
  ExpectAssessed(result);
  EXPECT_EQ(result.decision.kind(), kProject);
  ASSERT_TRUE(result.decision.has_applied_intent());
  const auto &applied =
      result.decision.applied_intent().pose().pose_world_from_body();
  EXPECT_DOUBLE_EQ(applied.position().z(), 2.0 - 100.0);
  EXPECT_DOUBLE_EQ(applied.position().x(), 5.0);
  EXPECT_DOUBLE_EQ(applied.position().y(), -6.0);
  EXPECT_DOUBLE_EQ(applied.orientation().w(), 1.0);
  EXPECT_EQ(Ids(result.decision), std::vector<std::string>{"depth.max"});
}

TEST(ReferenceSafetyFilterTest, F4FastTwistIsProjectedToMaxSpeed) {
  ReferenceSafetyFilter filter(Config());
  const DesiredMotion requested = TwistIntent(1.8, 2.4, 0.0, 0.0, 0.0, 0.1);
  const auto result = Eval(filter, requested);
  ExpectAssessed(result);
  EXPECT_EQ(result.decision.kind(), kProject);
  ASSERT_TRUE(result.decision.has_applied_intent());
  const auto &twist = result.decision.applied_intent().twist().body_twist();
  EXPECT_NEAR(twist.linear_x_m_s(), 0.9, 1e-12);
  EXPECT_NEAR(twist.linear_y_m_s(), 1.2, 1e-12);
  EXPECT_DOUBLE_EQ(twist.linear_z_m_s(), 0.0);
  EXPECT_DOUBLE_EQ(twist.angular_z_rad_s(), 0.1);
  EXPECT_DOUBLE_EQ(twist.angular_x_rad_s(), 0.0);
  EXPECT_DOUBLE_EQ(twist.angular_y_rad_s(), 0.0);
  const double speed = std::sqrt(twist.linear_x_m_s() * twist.linear_x_m_s() +
                                 twist.linear_y_m_s() * twist.linear_y_m_s() +
                                 twist.linear_z_m_s() * twist.linear_z_m_s());
  EXPECT_NEAR(speed, 1.5, 1e-12);
  EXPECT_EQ(Ids(result.decision), std::vector<std::string>{"speed.linear.max"});
}

TEST(ReferenceSafetyFilterTest, F5PoseOutsideFenceIsRejected) {
  ReferenceSafetyFilter filter(Config());
  const auto result = Eval(filter, PoseIntent(500.0, 0.0, -10.0));
  ExpectAssessed(result);
  EXPECT_EQ(result.decision.kind(), kReject);
  EXPECT_FALSE(result.decision.has_applied_intent());
  EXPECT_EQ(Ids(result.decision), std::vector<std::string>{"geofence.aabb"});
}

TEST(ReferenceSafetyFilterTest, F6SteepPitchIsRejectedNotProjected) {
  ReferenceSafetyFilter filter(Config());
  const auto result = Eval(filter, PoseIntent(0.0, 0.0, -10.0, 0.0, kPi / 4.0));
  ExpectAssessed(result);
  EXPECT_EQ(result.decision.kind(), kReject);
  EXPECT_FALSE(result.decision.has_applied_intent());
  EXPECT_EQ(Ids(result.decision),
            std::vector<std::string>{"attitude.pitch.max"});
}

TEST(ReferenceSafetyFilterTest, F6bSteepRollIsRejected) {
  ReferenceSafetyFilter filter(Config());
  const auto result = Eval(filter, PoseIntent(0.0, 0.0, -10.0, kPi / 4.0, 0.0));
  ExpectAssessed(result);
  EXPECT_EQ(result.decision.kind(), kReject);
  EXPECT_FALSE(result.decision.has_applied_intent());
  EXPECT_EQ(Ids(result.decision),
            std::vector<std::string>{"attitude.roll.max"});
}

TEST(ReferenceSafetyFilterTest, F7FastTurnIsRejected) {
  ReferenceSafetyFilter filter(Config());
  const auto result = Eval(filter, TwistIntent(0.0, 0.0, 0.0, 0.0, 0.0, 1.0));
  ExpectAssessed(result);
  EXPECT_EQ(result.decision.kind(), kReject);
  EXPECT_EQ(Ids(result.decision), std::vector<std::string>{"rate.angular.max"});
}

TEST(ReferenceSafetyFilterTest, F8HeaveRatesAreRejected) {
  ReferenceSafetyFilter filter(Config());
  const auto descent = Eval(filter, TwistIntent(0.0, 0.0, -1.0));
  ExpectAssessed(descent);
  EXPECT_EQ(descent.decision.kind(), kReject);
  EXPECT_EQ(Ids(descent.decision),
            std::vector<std::string>{"rate.descent.max"});

  const auto ascent = Eval(filter, TwistIntent(0.0, 0.0, 1.0));
  ExpectAssessed(ascent);
  EXPECT_EQ(ascent.decision.kind(), kReject);
  EXPECT_EQ(Ids(ascent.decision), std::vector<std::string>{"rate.ascent.max"});
}

TEST(ReferenceSafetyFilterTest, F9ConflictingProjectionsAreRejected) {
  ReferenceSafetyFilter filter(Config());
  const auto result =
      Eval(filter, PoseIntent(0.0, 0.0, -150.0, 0.0, kPi / 4.0));
  ExpectAssessed(result);
  EXPECT_EQ(result.decision.kind(), kReject);
  EXPECT_FALSE(result.decision.has_applied_intent());
  EXPECT_EQ(Ids(result.decision),
            (std::vector<std::string>{"attitude.pitch.max", "depth.max"}));
}

TEST(ReferenceSafetyFilterTest, F10NonFiniteIntentIsRejected) {
  ReferenceSafetyFilter filter(Config());
  DesiredMotion requested = PoseIntent(0.0, 0.0, -10.0);
  requested.mutable_pose()
      ->mutable_pose_world_from_body()
      ->mutable_position()
      ->set_x(std::numeric_limits<double>::quiet_NaN());
  const auto result = Eval(filter, requested);
  ExpectAssessed(result);
  EXPECT_EQ(result.decision.kind(), kReject);
  EXPECT_FALSE(result.decision.has_applied_intent());
  EXPECT_EQ(
      Ids(result.decision),
      (std::vector<std::string>{"filter.intent.invalid", "state.non_finite"}));
  EXPECT_TRUE(IsLowercaseSha256Hex(result.decision.original_intent_digest()));
}

TEST(ReferenceSafetyFilterTest, F10bNonFiniteStateIsRejected) {
  ReferenceSafetyFilter filter(Config());
  VehicleState state = FreshState();
  state.mutable_body_twist()->set_linear_x_m_s(
      std::numeric_limits<double>::infinity());
  const auto result =
      Eval(filter, PoseIntent(0.0, 0.0, -10.0), kConstrained, state);
  ExpectAssessed(result);
  EXPECT_EQ(result.decision.kind(), kReject);
  EXPECT_EQ(Ids(result.decision), std::vector<std::string>{"state.non_finite"});
}

TEST(ReferenceSafetyFilterTest, F11StaleStateIsRejected) {
  ReferenceSafetyFilter filter(Config());
  const auto result = Eval(filter, PoseIntent(0.0, 0.0, -10.0), kConstrained,
                           FreshState(kQueryTime, /*age_seconds=*/3));
  ExpectAssessed(result);
  EXPECT_EQ(result.decision.kind(), kReject);
  EXPECT_EQ(Ids(result.decision), std::vector<std::string>{"state.age"});
}

TEST(ReferenceSafetyFilterTest, F12TrajectoryObjectiveIsRejected) {
  ReferenceSafetyFilter filter(Config());
  const auto result = Eval(filter, TrajectoryIntent());
  ExpectAssessed(result);
  EXPECT_EQ(result.decision.kind(), kReject);
  EXPECT_EQ(Ids(result.decision),
            std::vector<std::string>{"filter.objective.unsupported"});
}

TEST(ReferenceSafetyFilterTest, F13PoseInOtherFrameIsRejected) {
  ReferenceSafetyFilter filter(Config());
  const auto result =
      Eval(filter, PoseIntent(0.0, 0.0, -10.0, 0.0, 0.0, "map"));
  ExpectAssessed(result);
  EXPECT_EQ(result.decision.kind(), kReject);
  EXPECT_EQ(Ids(result.decision),
            std::vector<std::string>{"filter.frame.unsupported"});
  EXPECT_EQ(result.decision.header().frame_id(), "map");
}

TEST(ReferenceSafetyFilterTest, F14ShadowAndRecommendUseConstrainedRow) {
  ReferenceSafetyFilter filter(Config());
  for (ProtoAuthorityMode held : {kShadow, kRecommend, kConstrained}) {
    const auto result = Eval(filter, PoseIntent(1.0, 1.0, -10.0), held);
    ExpectAssessed(result);
    EXPECT_EQ(result.decision.kind(), kAccept) << held;
    EXPECT_TRUE(result.decision.findings().empty()) << held;
  }
  ReferenceSafetyFilter projecting(Config());
  const auto projected =
      Eval(projecting, PoseIntent(0.0, 0.0, -150.0), kShadow);
  ExpectAssessed(projected);
  EXPECT_EQ(projected.decision.kind(), kProject);
  EXPECT_TRUE(projected.decision.has_applied_intent());
}

TEST(ReferenceSafetyFilterTest, F15EmergencyAndRevokedRejectEverythingLive) {
  ReferenceSafetyFilter filter(Config());
  const auto emergency = Eval(filter, PoseIntent(1.0, 1.0, -10.0), kEmergency);
  ExpectAssessed(emergency);
  EXPECT_EQ(emergency.decision.kind(), kReject);
  EXPECT_FALSE(emergency.decision.has_applied_intent());
  EXPECT_EQ(Ids(emergency.decision),
            std::vector<std::string>{"authority.emergency"});
  EXPECT_EQ(emergency.decision.findings(0).severity(),
            intrinsic_proto::safety::SAFETY_FINDING_SEVERITY_ERROR);

  const auto revoked = Eval(filter, PoseIntent(1.0, 1.0, -10.0), kRevoked);
  ExpectAssessed(revoked);
  EXPECT_EQ(revoked.decision.kind(), kReject);
  EXPECT_EQ(Ids(revoked.decision),
            std::vector<std::string>{"authority.revoked"});

  const auto projected_in_emergency =
      Eval(filter, PoseIntent(0.0, 0.0, -150.0), kEmergency);
  ExpectAssessed(projected_in_emergency);
  EXPECT_EQ(projected_in_emergency.decision.kind(), kReject);
  EXPECT_FALSE(projected_in_emergency.decision.has_applied_intent());
  EXPECT_EQ(Ids(projected_in_emergency.decision),
            (std::vector<std::string>{"authority.emergency", "depth.max"}));
}

TEST(ReferenceSafetyFilterTest, F16SurfaceBuildsSurfacePose) {
  ReferenceSafetyFilterConfig config = Config();
  config.surface_z_m = 2.0;
  const DesiredMotion requested = PoseIntent(3.0, 4.0, -20.0);
  for (SurfaceTrigger trigger :
       {SurfaceTrigger::kPlannerNoSolution, SurfaceTrigger::kDeadReckoning}) {
    for (ProtoAuthorityMode held : {kConstrained, kShadow, kEmergency}) {
      ReferenceSafetyFilter filter(config);
      const auto result =
          filter.Surface(trigger, requested, 12.5, -7.5, Snapshot(SnapshotId()),
                         held, kQueryTime);
      ExpectAssessed(result);
      EXPECT_EQ(result.decision.kind(), kSurface) << held;
      ASSERT_TRUE(result.decision.has_applied_intent());
      const auto &applied = result.decision.applied_intent();
      EXPECT_EQ(applied.header().frame_id(), "world_enu");
      EXPECT_EQ(applied.header().validity().state(), Validity::STATE_VALID);
      EXPECT_EQ(applied.header().source_time().seconds(), kQueryTime.seconds);
      EXPECT_EQ(applied.header().source_time().nanos(), kQueryTime.nanos);
      const auto &pose = applied.pose().pose_world_from_body();
      EXPECT_DOUBLE_EQ(pose.position().x(), 12.5);
      EXPECT_DOUBLE_EQ(pose.position().y(), -7.5);
      EXPECT_DOUBLE_EQ(pose.position().z(), 2.0 - 0.25);
      EXPECT_DOUBLE_EQ(pose.orientation().x(), 0.0);
      EXPECT_DOUBLE_EQ(pose.orientation().y(), 0.0);
      EXPECT_DOUBLE_EQ(pose.orientation().z(), 0.0);
      EXPECT_DOUBLE_EQ(pose.orientation().w(), 1.0);
      ASSERT_EQ(result.decision.findings_size(), 1);
      EXPECT_EQ(result.decision.findings(0).rule_id(),
                trigger == SurfaceTrigger::kPlannerNoSolution
                    ? "fallback.planner_no_solution"
                    : "fallback.dead_reckoning");
      EXPECT_EQ(result.decision.findings(0).severity(),
                intrinsic_proto::safety::SAFETY_FINDING_SEVERITY_CRITICAL);
      EXPECT_EQ(result.decision.original_intent_digest(),
                ComputeOriginalIntentDigest(requested));
    }
  }
}

TEST(ReferenceSafetyFilterTest, F16bSurfaceWithNonFiniteTruthIsRejected) {
  ReferenceSafetyFilter filter(Config());
  const auto result = filter.Surface(
      SurfaceTrigger::kDeadReckoning, PoseIntent(0.0, 0.0, -10.0),
      std::numeric_limits<double>::quiet_NaN(), 0.0, Snapshot(SnapshotId()),
      kConstrained, kQueryTime);
  ExpectAssessed(result);
  EXPECT_EQ(result.decision.kind(), kReject);
  EXPECT_FALSE(result.decision.has_applied_intent());
  EXPECT_EQ(Ids(result.decision),
            (std::vector<std::string>{"fallback.dead_reckoning",
                                      "filter.intent.invalid"}));
}

TEST(ReferenceSafetyFilterTest, F17EpochAdvancesOnlyOnReturnedDecisions) {
  ReferenceSafetyFilter filter(Config());
  EXPECT_EQ(filter.next_epoch(), 1u);
  const auto first = Eval(filter, PoseIntent(0.0, 0.0, -10.0));
  ExpectAssessed(first);
  EXPECT_EQ(first.decision.decision_epoch(), 1u);

  const auto error =
      Eval(filter, PoseIntent(0.0, 0.0, -10.0), kConstrained, FreshState(), "");
  ExpectEmptyError(error, ReferenceSafetyFilterError::kInvalidSnapshot);
  EXPECT_EQ(filter.next_epoch(), 2u);

  const auto second = Eval(filter, PoseIntent(0.0, 0.0, -150.0));
  ExpectAssessed(second);
  EXPECT_EQ(second.decision.decision_epoch(), 2u);

  const auto third = filter.Surface(
      SurfaceTrigger::kPlannerNoSolution, PoseIntent(0.0, 0.0, -10.0), 0.0, 0.0,
      Snapshot(SnapshotId()), kConstrained, kQueryTime);
  ExpectAssessed(third);
  EXPECT_EQ(third.decision.decision_epoch(), 3u);
  EXPECT_EQ(filter.next_epoch(), 4u);
}

TEST(ReferenceSafetyFilterTest, F18DigestAndSnapshotIdAreFilled) {
  ReferenceSafetyFilter filter(Config());
  const std::string snapshot_id =
      "0123456789abcdef0123456789abcdef0123456789abcdef0123456789abcdef";
  const DesiredMotion accepted = PoseIntent(0.0, 0.0, -10.0);
  const DesiredMotion projected = PoseIntent(0.0, 0.0, -150.0);
  const DesiredMotion rejected = PoseIntent(500.0, 0.0, -10.0);
  for (const DesiredMotion *requested : {&accepted, &projected, &rejected}) {
    const auto result =
        Eval(filter, *requested, kConstrained, FreshState(), snapshot_id);
    ExpectAssessed(result);
    EXPECT_EQ(result.decision.original_intent_digest(),
              ComputeOriginalIntentDigest(*requested));
    EXPECT_EQ(result.decision.snapshot_id(), snapshot_id);
  }
}

TEST(ReferenceSafetyFilterTest, F18bSnapshotIdThatIsNotHexIsNotCommitted) {
  // The assessor requires a 64-character lowercase hex snapshot_id, and a
  // decision that fails it is an error, not a decision (lock section 4.9).
  ReferenceSafetyFilter filter(Config());
  const auto result = Eval(filter, PoseIntent(0.0, 0.0, -10.0), kConstrained,
                           FreshState(), "snapshot-\xC3\xA9\xC3\xA8");
  ExpectEmptyError(result, ReferenceSafetyFilterError::kDecisionInvalid);
  EXPECT_EQ(filter.next_epoch(), 1u);
}

TEST(ReferenceSafetyFilterTest, F19InvalidInputsReturnErrors) {
  const std::vector<ReferenceSafetyFilterConfig> bad_configs = [] {
    std::vector<ReferenceSafetyFilterConfig> configs;
    const double nan = std::numeric_limits<double>::quiet_NaN();
    for (double ReferenceSafetyFilterConfig::*field :
         {&ReferenceSafetyFilterConfig::surface_z_m,
          &ReferenceSafetyFilterConfig::max_depth_m,
          &ReferenceSafetyFilterConfig::max_linear_speed_m_s,
          &ReferenceSafetyFilterConfig::max_pitch_rad,
          &ReferenceSafetyFilterConfig::max_roll_rad,
          &ReferenceSafetyFilterConfig::max_angular_rate_rad_s,
          &ReferenceSafetyFilterConfig::max_descent_rate_m_s,
          &ReferenceSafetyFilterConfig::max_ascent_rate_m_s}) {
      ReferenceSafetyFilterConfig config = Config();
      config.*field = nan;
      configs.push_back(config);
    }
    ReferenceSafetyFilterConfig zero_limit = Config();
    zero_limit.max_depth_m = 0.0;
    configs.push_back(zero_limit);
    ReferenceSafetyFilterConfig bad_frame = Config();
    bad_frame.fence.frame_id = "map";
    configs.push_back(bad_frame);
    ReferenceSafetyFilterConfig no_region = Config();
    no_region.fence.region_id = "";
    configs.push_back(no_region);
    ReferenceSafetyFilterConfig inverted = Config();
    inverted.fence.min_x = 200.0;
    configs.push_back(inverted);
    ReferenceSafetyFilterConfig non_finite_fence = Config();
    non_finite_fence.fence.max_z = nan;
    configs.push_back(non_finite_fence);
    ReferenceSafetyFilterConfig negative_age = Config();
    negative_age.state_max_age = StateTime{-1, 0};
    configs.push_back(negative_age);
    return configs;
  }();
  for (const ReferenceSafetyFilterConfig &config : bad_configs) {
    ReferenceSafetyFilter filter(config);
    ExpectEmptyError(Eval(filter, PoseIntent(0.0, 0.0, -10.0)),
                     ReferenceSafetyFilterError::kInvalidConfig);
    ExpectEmptyError(filter.Surface(SurfaceTrigger::kDeadReckoning,
                                    PoseIntent(0.0, 0.0, -10.0), 0.0, 0.0,
                                    Snapshot(SnapshotId()), kConstrained,
                                    kQueryTime),
                     ReferenceSafetyFilterError::kInvalidConfig);
    EXPECT_EQ(filter.next_epoch(), 1u);
  }

  ReferenceSafetyFilter filter(Config());
  ExpectEmptyError(
      Eval(filter, PoseIntent(0.0, 0.0, -10.0), kConstrained, FreshState(), ""),
      ReferenceSafetyFilterError::kInvalidSnapshot);
  for (ProtoAuthorityMode held :
       {intrinsic_proto::safety::AUTHORITY_MODE_UNSPECIFIED,
        static_cast<ProtoAuthorityMode>(99)}) {
    ExpectEmptyError(Eval(filter, PoseIntent(0.0, 0.0, -10.0), held),
                     ReferenceSafetyFilterError::kNonLiveAuthority);
    ExpectEmptyError(filter.Surface(SurfaceTrigger::kPlannerNoSolution,
                                    PoseIntent(0.0, 0.0, -10.0), 0.0, 0.0,
                                    Snapshot(SnapshotId()), held, kQueryTime),
                     ReferenceSafetyFilterError::kNonLiveAuthority);
  }
  ExpectEmptyError(
      filter.Surface(SurfaceTrigger::kDeadReckoning, DesiredMotion(), 0.0, 0.0,
                     Snapshot(SnapshotId()), kConstrained, kQueryTime),
      ReferenceSafetyFilterError::kMissingTriggerIntent);
  ExpectEmptyError(filter.Surface(SurfaceTrigger::kDeadReckoning,
                                  PoseIntent(0.0, 0.0, -10.0), 0.0, 0.0,
                                  Snapshot(""), kConstrained, kQueryTime),
                   ReferenceSafetyFilterError::kInvalidSnapshot);
  EXPECT_EQ(filter.next_epoch(), 1u);
}

TEST(ReferenceSafetyFilterTest, InvalidRequestedIntentIsRejectedNotAnError) {
  ReferenceSafetyFilter filter(Config());
  DesiredMotion requested = PoseIntent(0.0, 0.0, -10.0);
  requested.mutable_header()->mutable_validity()->set_state(
      Validity::STATE_INVALID);
  const auto invalid_header = Eval(filter, requested);
  ExpectAssessed(invalid_header);
  EXPECT_EQ(invalid_header.decision.kind(), kReject);
  EXPECT_EQ(Ids(invalid_header.decision),
            std::vector<std::string>{"filter.intent.invalid"});

  const auto empty = Eval(filter, DesiredMotion());
  ExpectAssessed(empty);
  EXPECT_EQ(empty.decision.kind(), kReject);
  EXPECT_EQ(Ids(empty.decision),
            std::vector<std::string>{"filter.intent.invalid"});
}

TEST(ReferenceSafetyFilterTest, EqualityAtTheLimitIsCompliant) {
  ReferenceSafetyFilter filter(Config());
  const auto depth = Eval(filter, PoseIntent(0.0, 0.0, -100.0));
  ExpectAssessed(depth);
  EXPECT_EQ(depth.decision.kind(), kAccept);
  const auto speed = Eval(filter, TwistIntent(1.5, 0.0, 0.0));
  ExpectAssessed(speed);
  EXPECT_EQ(speed.decision.kind(), kAccept);
}

std::vector<std::string> RunSequence(std::vector<std::string> *digests) {
  ReferenceSafetyFilter filter(Config());
  std::vector<std::string> serialized;
  auto record = [&](const ReferenceSafetyFilterResult &result) {
    EXPECT_EQ(result.error, ReferenceSafetyFilterError::kOk);
    serialized.push_back(result.decision.SerializeAsString());
    digests->push_back(result.decision.original_intent_digest());
  };
  record(Eval(filter, PoseIntent(0.0, 0.0, -10.0)));
  record(Eval(filter, PoseIntent(0.0, 0.0, -150.0)));
  record(Eval(filter, PoseIntent(500.0, 0.0, -10.0)));
  record(Eval(filter, PoseIntent(0.0, 0.0, -10.0, 0.0, kPi / 4.0)));
  record(Eval(filter, TwistIntent(1.8, 2.4, 0.0, 0.0, 0.0, 0.1)));
  record(Eval(filter, TwistIntent(0.0, 0.0, -1.0)));
  record(Eval(filter, TrajectoryIntent()));
  record(Eval(filter, PoseIntent(1.0, 1.0, -10.0), kEmergency));
  record(Eval(filter, PoseIntent(0.0, 0.0, -10.0), kConstrained,
              FreshState(kQueryTime, 3)));
  record(filter.Surface(SurfaceTrigger::kPlannerNoSolution,
                        PoseIntent(0.0, 0.0, -10.0), 1.0, 2.0,
                        Snapshot(SnapshotId()), kShadow, kQueryTime));
  return serialized;
}

TEST(ReferenceSafetyFilterTest, F20RepeatedSequenceIsBitIdentical) {
  std::vector<std::string> digests_a;
  std::vector<std::string> digests_b;
  const std::vector<std::string> a = RunSequence(&digests_a);
  const std::vector<std::string> b = RunSequence(&digests_b);
  ASSERT_EQ(a.size(), 10u);
  EXPECT_EQ(a, b);
  EXPECT_EQ(digests_a, digests_b);
}

} // namespace
} // namespace intrinsic::safety
