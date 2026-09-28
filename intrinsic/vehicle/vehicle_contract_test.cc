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
#include <limits>

#include "gtest/gtest.h"
#include "intrinsic/embodiment/capability_policy.h"
#include "intrinsic/embodiment/frame_policy.h"
#include "intrinsic/vehicle/vehicle_contract_policy.h"

namespace intrinsic::vehicle {
namespace {

using embodiment::ValidityKind;

VehicleStateView ValidState() {
  VehicleStateView state;
  state.validity_present = true;
  state.validity_state = 1;
  state.frame_id = "world_enu";
  state.pose_present = true;
  state.position = embodiment::Vec3{1, 2, -3};
  state.orientation = embodiment::Quaternion{0, 0, 0, 1};
  state.twist_present = true;
  state.twist = BodyVector{0.5, 0, 0, 0, 0, 0.125};
  return state;
}

DesiredMotionView ValidPoseMotion() {
  DesiredMotionView motion;
  motion.header_present = true;
  motion.validity_present = true;
  motion.validity_state = 1;
  motion.frame_id = "world_enu";
  motion.objective = ObjectiveKind::kPose;
  motion.position = embodiment::Vec3{4, 0, -2};
  motion.orientation = embodiment::Quaternion{0, 0, 0, 1};
  motion.confidence_present = true;
  motion.confidence = 0.5;
  motion.horizon_present = true;
  motion.horizon_seconds = 5;
  motion.provenance_present = true;
  motion.model_id = "uuv_intent";
  return motion;
}

BodyWrenchView ValidWrench() {
  BodyWrenchView wrench;
  wrench.engaged = true;
  wrench.validity_present = true;
  wrench.validity_state = 1;
  wrench.frame_id = kBodyFrameId;
  wrench.force_torque = BodyVector{1.5, 0, 0, 0, 0, 0.25};
  return wrench;
}

TEST(VehicleContractTest, EmptyStateIsOptInAndNotAccepted) {
  const ContractAssessment assessment = AssessVehicleState(VehicleStateView{});
  EXPECT_EQ(assessment.error, ContractError::kNone);
  EXPECT_EQ(assessment.validity, ValidityKind::kAbsent);
  EXPECT_FALSE(assessment.accepted);
}

TEST(VehicleContractTest, ValidStateIsAccepted) {
  const ContractAssessment assessment = AssessVehicleState(ValidState());
  EXPECT_EQ(assessment.error, ContractError::kNone);
  EXPECT_EQ(assessment.validity, ValidityKind::kValid);
  EXPECT_TRUE(assessment.accepted);
}

TEST(VehicleContractTest, ValidStampDoesNotRepairNanOrInfinity) {
  VehicleStateView nan_twist = ValidState();
  nan_twist.twist.linear_x = std::numeric_limits<double>::quiet_NaN();
  const ContractAssessment nan_assessment = AssessVehicleState(nan_twist);
  EXPECT_EQ(nan_assessment.error, ContractError::kNonFinite);
  EXPECT_EQ(nan_assessment.validity, ValidityKind::kValid);
  EXPECT_FALSE(nan_assessment.accepted);

  VehicleStateView infinite_accel = ValidState();
  infinite_accel.acceleration_present = true;
  infinite_accel.acceleration.linear_z =
      std::numeric_limits<double>::infinity();
  EXPECT_EQ(AssessVehicleState(infinite_accel).error,
            ContractError::kNonFinite);
}

TEST(VehicleContractTest, InvalidStampIsNotRewrittenAndIsNotAccepted) {
  VehicleStateView state = ValidState();
  state.validity_state = 2;
  const ContractAssessment assessment = AssessVehicleState(state);
  EXPECT_EQ(assessment.error, ContractError::kNone);
  EXPECT_EQ(assessment.validity, ValidityKind::kInvalid);
  EXPECT_FALSE(assessment.accepted);
}

TEST(VehicleContractTest, AbsentValidityIsNotInvalid) {
  VehicleStateView state = ValidState();
  state.validity_present = false;
  const ContractAssessment assessment = AssessVehicleState(state);
  EXPECT_EQ(assessment.validity, ValidityKind::kAbsent);
  EXPECT_FALSE(assessment.accepted);
  EXPECT_NE(assessment.validity, ValidityKind::kInvalid);
}

TEST(VehicleContractTest, NonUnitQuaternionIsRejectedAndNotRenormalized) {
  VehicleStateView state = ValidState();
  state.orientation = embodiment::Quaternion{0, 0, 0, 2};
  EXPECT_EQ(AssessVehicleState(state).error, ContractError::kQuaternion);
  EXPECT_FALSE(embodiment::IsNormalized(state.orientation));

  state.orientation =
      embodiment::Quaternion{std::numeric_limits<double>::quiet_NaN(), 0, 0, 1};
  EXPECT_EQ(AssessVehicleState(state).error, ContractError::kNonFinite);
}

TEST(VehicleContractTest, PresentPoseRequiresAnExplicitFrame) {
  VehicleStateView state = ValidState();
  state.frame_id = "";
  EXPECT_EQ(AssessVehicleState(state).error, ContractError::kMissingFrame);
}

TEST(VehicleContractTest, BodyTwistIsNotConvertedWithThePoseFrame) {
  const embodiment::Vec3 body{1, 2, 3};
  const embodiment::Vec3 ned = embodiment::WorldVectorEnuToNed(body);
  EXPECT_NE(ned.x, body.x);
  VehicleStateView state = ValidState();
  state.frame_id = embodiment::kWorldEnuFrameId;
  state.twist = BodyVector{body.x, body.y, body.z, 0, 0, 0};
  const ContractAssessment assessment = AssessVehicleState(state);
  EXPECT_TRUE(assessment.accepted);
  EXPECT_EQ(state.twist.linear_x, body.x);
  EXPECT_EQ(state.twist.linear_y, body.y);
  EXPECT_EQ(state.twist.linear_z, body.z);
  EXPECT_FALSE(
      embodiment::FrameIdMatches("VehicleState", embodiment::WorldFrame::kEnu));
  EXPECT_FALSE(
      embodiment::FrameIdMatches(kBodyFrameId, embodiment::WorldFrame::kEnu));
}

TEST(VehicleContractTest, ValidHeaderWithoutPoseIsNotAStructuralDefect) {
  VehicleStateView state;
  state.validity_present = true;
  state.validity_state = 1;
  EXPECT_TRUE(AssessVehicleState(state).accepted);
}

TEST(VehicleContractTest, PresentZeroTwistIsFinite) {
  VehicleStateView state = ValidState();
  state.twist = BodyVector{};
  EXPECT_TRUE(IsFinite(state.twist));
  EXPECT_TRUE(AssessVehicleState(state).accepted);
}

TEST(VehicleContractTest, CovarianceUnknownIsAbsenceNotZeros) {
  EXPECT_EQ(AssessCovariance(false, {}), CovarianceError::kAbsent);
  EXPECT_TRUE(CovarianceIsUnknown(CovarianceError::kAbsent));

  const std::array<double, kCovarianceValues> zeros = {};
  EXPECT_EQ(AssessCovariance(true, zeros), CovarianceError::kNone);
  EXPECT_TRUE(IsAllZeroCovariance(zeros));
  EXPECT_FALSE(CovarianceIsUnknown(AssessCovariance(true, zeros)));

  EXPECT_EQ(AssessCovariance(true, {}), CovarianceError::kWrongLength);
  EXPECT_FALSE(CovarianceIsUnknown(CovarianceError::kWrongLength));
}

TEST(VehicleContractTest, CovarianceRejectsNonFiniteWrongShapeAndAsymmetry) {
  std::array<double, kCovarianceValues> values = {};
  values[0] = std::numeric_limits<double>::quiet_NaN();
  EXPECT_EQ(AssessCovariance(true, values), CovarianceError::kNonFinite);

  values[0] = 0.25;
  values[1] = 1.0;
  EXPECT_EQ(AssessCovariance(true, values), CovarianceError::kAsymmetric);

  values[1] = 1e-12;
  EXPECT_EQ(AssessCovariance(true, values), CovarianceError::kNone);
  values[1] = 1e-6;
  EXPECT_EQ(AssessCovariance(true, values), CovarianceError::kAsymmetric);

  const std::array<double, 35> short_values = {};
  EXPECT_EQ(AssessCovariance(true, short_values),
            CovarianceError::kWrongLength);
  EXPECT_EQ(CovarianceIndex(1, 0), 6);
  EXPECT_EQ(CovarianceIndex(5, 5), 35);
}

TEST(VehicleContractTest, BadCovarianceFailsTheStateAndDoesNotMeanUnknown) {
  VehicleStateView state = ValidState();
  const std::array<double, 3> short_values = {};
  state.pose_covariance_present = true;
  state.pose_covariance = short_values;
  EXPECT_EQ(AssessVehicleState(state).error, ContractError::kPoseCovariance);

  state = ValidState();
  state.twist_covariance_present = true;
  state.twist_covariance = short_values;
  EXPECT_EQ(AssessVehicleState(state).error, ContractError::kTwistCovariance);
}

TEST(VehicleContractTest, EmptySourceIdIsRejectedAndAbsentHealthIsNotInvalid) {
  VehicleStateView state = ValidState();
  const SourceView sources[] = {
      SourceView{"dvl", true, 1},
      SourceView{"", false, 0},
  };
  state.sources = sources;
  EXPECT_EQ(AssessVehicleState(state).error, ContractError::kSourceId);

  const SourceView depth[] = {SourceView{"depth", false, 0}};
  state.sources = depth;
  const ContractAssessment assessment = AssessVehicleState(state);
  EXPECT_TRUE(assessment.accepted);
  EXPECT_EQ(embodiment::ClassifyValidity(depth[0].validity_present,
                                         depth[0].validity_state),
            ValidityKind::kAbsent);
}

TEST(VehicleContractTest, UnknownNavigationModeIsNotFaulted) {
  EXPECT_EQ(ClassifyNavigationMode(0), NavigationModeKind::kUnspecified);
  EXPECT_EQ(ClassifyNavigationMode(2), NavigationModeKind::kDeadReckoning);
  EXPECT_EQ(ClassifyNavigationMode(3), NavigationModeKind::kAided);
  EXPECT_EQ(ClassifyNavigationMode(4), NavigationModeKind::kFaulted);
  EXPECT_EQ(ClassifyNavigationMode(100), NavigationModeKind::kUnknown);
  EXPECT_NE(ClassifyNavigationMode(100), NavigationModeKind::kFaulted);
  EXPECT_TRUE(AssessVehicleState(ValidState()).accepted);
}

TEST(VehicleContractTest, ManipulatorCapabilityFixtureStaysOptIn) {
  EXPECT_FALSE(
      embodiment::IsWellKnownCapabilityId("ai.intrinsic.capability.vehicle"));
  const auto declarations = embodiment::ManipulatorCapabilityDeclarations();
  EXPECT_EQ(embodiment::AssessCapabilityDeclarations(declarations).error,
            embodiment::DeclarationError::kNone);
  EXPECT_FALSE(embodiment::DeclaresCapability(
      declarations, "ai.intrinsic.capability.vehicle"));
}

TEST(VehicleContractTest, EmptyMotionAndWrenchAreNotCommands) {
  const ContractAssessment motion = AssessDesiredMotion(DesiredMotionView{});
  EXPECT_EQ(motion.error, ContractError::kNone);
  EXPECT_FALSE(motion.accepted);
  const ContractAssessment wrench = AssessBodyWrench(BodyWrenchView{});
  EXPECT_EQ(wrench.error, ContractError::kNone);
  EXPECT_EQ(wrench.validity, ValidityKind::kAbsent);
  EXPECT_FALSE(wrench.accepted);
}

TEST(VehicleContractTest, PoseIntentTwistIntentAndNeutralWrenchAreAccepted) {
  EXPECT_TRUE(AssessDesiredMotion(ValidPoseMotion()).accepted);

  DesiredMotionView twist = ValidPoseMotion();
  twist.objective = ObjectiveKind::kTwist;
  twist.frame_id = kBodyFrameId;
  twist.twist = BodyVector{0.5, 0, 0, 0, 0, 0};
  twist.provenance_present = false;
  EXPECT_TRUE(AssessDesiredMotion(twist).accepted);

  BodyWrenchView neutral = ValidWrench();
  neutral.force_torque = BodyVector{};
  EXPECT_TRUE(AssessBodyWrench(neutral).accepted);
  EXPECT_TRUE(AssessBodyWrench(ValidWrench()).accepted);
}

TEST(VehicleContractTest, HeaderWithoutObjectiveIsNotAnIntent) {
  DesiredMotionView motion;
  motion.header_present = true;
  motion.validity_present = true;
  motion.validity_state = 1;
  EXPECT_EQ(AssessDesiredMotion(motion).error, ContractError::kObjective);
}

TEST(VehicleContractTest, TwistObjectiveRejectsWorldFrameAndNonFinite) {
  DesiredMotionView motion = ValidPoseMotion();
  motion.objective = ObjectiveKind::kTwist;
  motion.frame_id = embodiment::kWorldEnuFrameId;
  EXPECT_EQ(AssessDesiredMotion(motion).error, ContractError::kBodyFrame);

  motion.frame_id = "";
  EXPECT_EQ(AssessDesiredMotion(motion).error, ContractError::kBodyFrame);

  motion.frame_id = kBodyFrameId;
  motion.twist.linear_x = std::numeric_limits<double>::infinity();
  EXPECT_EQ(AssessDesiredMotion(motion).error, ContractError::kNonFinite);
}

TEST(VehicleContractTest, PoseObjectiveRejectsEmptyFrameAndBadQuaternion) {
  DesiredMotionView motion = ValidPoseMotion();
  motion.frame_id = "";
  EXPECT_EQ(AssessDesiredMotion(motion).error, ContractError::kMissingFrame);
  motion.frame_id = kBodyFrameId;
  EXPECT_TRUE(AssessDesiredMotion(motion).accepted);
  motion.orientation = embodiment::Quaternion{0, 0, 0, 2};
  EXPECT_EQ(AssessDesiredMotion(motion).error, ContractError::kQuaternion);
}

TEST(VehicleContractTest, ConfidenceHorizonProvenanceAndTrajectoryRules) {
  DesiredMotionView motion = ValidPoseMotion();
  motion.confidence = 1.5;
  EXPECT_EQ(AssessDesiredMotion(motion).error, ContractError::kConfidence);
  motion.confidence = std::numeric_limits<double>::quiet_NaN();
  EXPECT_EQ(AssessDesiredMotion(motion).error, ContractError::kConfidence);

  motion.confidence_present = false;
  EXPECT_TRUE(AssessDesiredMotion(motion).accepted);
  motion.confidence_present = true;
  motion.confidence = 0;
  EXPECT_TRUE(AssessDesiredMotion(motion).accepted);

  motion.horizon_seconds = -1;
  EXPECT_EQ(AssessDesiredMotion(motion).error, ContractError::kHorizon);
  motion.horizon_seconds = 0;
  motion.horizon_nanos = embodiment::kNanosPerSecond;
  EXPECT_EQ(AssessDesiredMotion(motion).error, ContractError::kHorizon);
  motion.horizon_nanos = -1;
  EXPECT_EQ(AssessDesiredMotion(motion).error, ContractError::kHorizon);
  motion.horizon_present = false;
  EXPECT_TRUE(AssessDesiredMotion(motion).accepted);

  motion.model_id = "";
  EXPECT_EQ(AssessDesiredMotion(motion).error, ContractError::kProvenance);
  motion.provenance_present = false;
  EXPECT_TRUE(AssessDesiredMotion(motion).accepted);

  motion.objective = ObjectiveKind::kTrajectory;
  motion.trajectory_id = "";
  EXPECT_EQ(AssessDesiredMotion(motion).error, ContractError::kTrajectoryId);
  motion.trajectory_id = "traj-1";
  motion.frame_id = "";
  EXPECT_TRUE(AssessDesiredMotion(motion).accepted);
}

TEST(VehicleContractTest, WrenchFrameAndNonFiniteRules) {
  BodyWrenchView wrench = ValidWrench();
  wrench.frame_id = embodiment::kWorldEnuFrameId;
  EXPECT_EQ(AssessBodyWrench(wrench).error, ContractError::kBodyFrame);
  wrench.frame_id = "";
  EXPECT_EQ(AssessBodyWrench(wrench).error, ContractError::kBodyFrame);
  wrench.frame_id = kBodyFrameId;
  wrench.force_torque.linear_x = std::numeric_limits<double>::quiet_NaN();
  EXPECT_EQ(AssessBodyWrench(wrench).error, ContractError::kNonFinite);
  wrench.force_torque = BodyVector{};
  wrench.force_torque.angular_z = std::numeric_limits<double>::infinity();
  EXPECT_EQ(AssessBodyWrench(wrench).error, ContractError::kNonFinite);

  wrench = ValidWrench();
  wrench.validity_present = false;
  const ContractAssessment absent = AssessBodyWrench(wrench);
  EXPECT_EQ(absent.error, ContractError::kNone);
  EXPECT_EQ(absent.validity, ValidityKind::kAbsent);
  EXPECT_FALSE(absent.accepted);
}
}  // namespace
}  // namespace intrinsic::vehicle
