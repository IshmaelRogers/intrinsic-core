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
#include "intrinsic/embodiment/frame_policy.h"
#include "intrinsic/vehicle/trajectory_contract_policy.h"

namespace intrinsic::vehicle {
namespace {

using embodiment::ValidityKind;

TrajectorySampleView SampleAt(int64_t seconds, int32_t nanos, double x) {
  TrajectorySampleView sample;
  sample.time_present = true;
  sample.seconds = seconds;
  sample.nanos = nanos;
  sample.position = embodiment::Vec3{x, 2, -3};
  sample.orientation = embodiment::Quaternion{0, 0, 0, 1};
  sample.twist_present = true;
  sample.twist = BodyVector{0.5, 0, 0, 0, 0, 0.125};
  return sample;
}

VehicleTrajectoryView ValidTrajectory(TrajectorySampleView (&samples)[2]) {
  samples[0] = SampleAt(1700000010, 0, 1);
  samples[1] = SampleAt(1700000011, 500000000, 1.5);
  VehicleTrajectoryView trajectory;
  trajectory.header_present = true;
  trajectory.validity_present = true;
  trajectory.validity_state = 1;
  trajectory.frame_id = embodiment::kWorldEnuFrameId;
  trajectory.trajectory_id = "traj_alpha";
  trajectory.samples = samples;
  trajectory.provenance_present = true;
  trajectory.model_id = "uuv_planner";
  return trajectory;
}

TEST(TrajectoryContractTest, EmptyTrajectoryIsNotEngaged) {
  const TrajectoryContractAssessment assessment =
      AssessVehicleTrajectory(VehicleTrajectoryView{});
  EXPECT_EQ(assessment.error, TrajectoryContractError::kNone);
  EXPECT_EQ(assessment.validity, ValidityKind::kAbsent);
  EXPECT_FALSE(assessment.accepted);
  EXPECT_FALSE(TrajectoryEngaged(VehicleTrajectoryView{}));
}

TEST(TrajectoryContractTest, TwoSampleTrajectoryIsAccepted) {
  TrajectorySampleView samples[2];
  const VehicleTrajectoryView trajectory = ValidTrajectory(samples);
  const TrajectoryContractAssessment assessment =
      AssessVehicleTrajectory(trajectory);
  EXPECT_EQ(assessment.error, TrajectoryContractError::kNone);
  EXPECT_EQ(assessment.validity, ValidityKind::kValid);
  EXPECT_TRUE(assessment.accepted);
}

TEST(TrajectoryContractTest, MetadataPresenceDoesNotChangeAcceptance) {
  TrajectorySampleView samples[2];
  VehicleTrajectoryView with_metadata = ValidTrajectory(samples);
  with_metadata.metadata_present = true;
  VehicleTrajectoryView without_metadata = with_metadata;
  without_metadata.metadata_present = false;
  EXPECT_TRUE(AssessVehicleTrajectory(with_metadata).accepted);
  EXPECT_TRUE(AssessVehicleTrajectory(without_metadata).accepted);
  EXPECT_EQ(AssessVehicleTrajectory(with_metadata).error,
            AssessVehicleTrajectory(without_metadata).error);
}

TEST(TrajectoryContractTest, SingleSampleIsMonotonic) {
  TrajectorySampleView samples[1] = {SampleAt(10, 0, 1)};
  VehicleTrajectoryView trajectory;
  trajectory.header_present = true;
  trajectory.validity_present = true;
  trajectory.validity_state = 1;
  trajectory.frame_id = "world_enu";
  trajectory.trajectory_id = "traj_alpha";
  trajectory.samples = samples;
  EXPECT_TRUE(AssessVehicleTrajectory(trajectory).accepted);
}

TEST(TrajectoryContractTest, EqualAndDecreasingTimesAreNonMonotonic) {
  TrajectorySampleView samples[2];
  VehicleTrajectoryView equal_times = ValidTrajectory(samples);
  samples[1].seconds = samples[0].seconds;
  samples[1].nanos = samples[0].nanos;
  EXPECT_EQ(AssessVehicleTrajectory(equal_times).error,
            TrajectoryContractError::kNonMonotonicTime);

  samples[1].seconds = samples[0].seconds - 1;
  samples[1].nanos = 999999999;
  EXPECT_EQ(AssessVehicleTrajectory(equal_times).error,
            TrajectoryContractError::kNonMonotonicTime);

  samples[1].seconds = samples[0].seconds;
  samples[1].nanos = samples[0].nanos + 1;
  EXPECT_TRUE(AssessVehicleTrajectory(equal_times).accepted);
}

TEST(TrajectoryContractTest, MissingFrameEmptyIdAndEmptySamples) {
  TrajectorySampleView samples[2];
  VehicleTrajectoryView missing_frame = ValidTrajectory(samples);
  missing_frame.frame_id = "";
  EXPECT_EQ(AssessVehicleTrajectory(missing_frame).error,
            TrajectoryContractError::kMissingFrame);

  VehicleTrajectoryView empty_id = ValidTrajectory(samples);
  empty_id.trajectory_id = "";
  EXPECT_EQ(AssessVehicleTrajectory(empty_id).error,
            TrajectoryContractError::kTrajectoryId);

  VehicleTrajectoryView no_samples = ValidTrajectory(samples);
  no_samples.samples = {};
  EXPECT_EQ(AssessVehicleTrajectory(no_samples).error,
            TrajectoryContractError::kEmptySamples);

  VehicleTrajectoryView header_only;
  header_only.header_present = true;
  header_only.validity_present = true;
  header_only.validity_state = 1;
  header_only.frame_id = "world_enu";
  EXPECT_EQ(AssessVehicleTrajectory(header_only).error,
            TrajectoryContractError::kTrajectoryId);
}

TEST(TrajectoryContractTest, SampleTimeMustBePresentAndInRange) {
  TrajectorySampleView samples[2];
  VehicleTrajectoryView trajectory = ValidTrajectory(samples);
  samples[0].time_present = false;
  EXPECT_EQ(AssessVehicleTrajectory(trajectory).error,
            TrajectoryContractError::kSampleTime);

  samples[0].time_present = true;
  samples[1].nanos = -1;
  EXPECT_EQ(AssessVehicleTrajectory(trajectory).error,
            TrajectoryContractError::kSampleTime);

  samples[1].nanos = embodiment::kNanosPerSecond;
  EXPECT_EQ(AssessVehicleTrajectory(trajectory).error,
            TrajectoryContractError::kSampleTime);

  samples[1].nanos = embodiment::kNanosPerSecond - 1;
  EXPECT_TRUE(AssessVehicleTrajectory(trajectory).accepted);
}

TEST(TrajectoryContractTest, NonFinitePoseTwistAndAcceleration) {
  TrajectorySampleView samples[2];
  VehicleTrajectoryView trajectory = ValidTrajectory(samples);
  samples[0].position.x = std::numeric_limits<double>::quiet_NaN();
  const TrajectoryContractAssessment nan_pose =
      AssessVehicleTrajectory(trajectory);
  EXPECT_EQ(nan_pose.error, TrajectoryContractError::kNonFinite);
  EXPECT_EQ(nan_pose.validity, ValidityKind::kValid);
  EXPECT_FALSE(nan_pose.accepted);

  samples[0].position.x = 1;
  samples[0].twist.linear_z = std::numeric_limits<double>::infinity();
  EXPECT_EQ(AssessVehicleTrajectory(trajectory).error,
            TrajectoryContractError::kNonFinite);

  samples[0].twist.linear_z = 0;
  samples[1].acceleration_present = true;
  samples[1].acceleration.angular_x = std::numeric_limits<double>::quiet_NaN();
  EXPECT_EQ(AssessVehicleTrajectory(trajectory).error,
            TrajectoryContractError::kNonFinite);

  samples[1].acceleration_present = false;
  EXPECT_TRUE(AssessVehicleTrajectory(trajectory).accepted);
}

TEST(TrajectoryContractTest, BadQuaternionIsNotRenormalized) {
  TrajectorySampleView samples[2];
  VehicleTrajectoryView trajectory = ValidTrajectory(samples);
  samples[0].orientation = embodiment::Quaternion{0, 0, 0, 2};
  EXPECT_EQ(AssessVehicleTrajectory(trajectory).error,
            TrajectoryContractError::kQuaternion);
  EXPECT_FALSE(embodiment::IsNormalized(samples[0].orientation));

  samples[0].orientation = embodiment::Quaternion{0, 0, 0, 0};
  EXPECT_EQ(AssessVehicleTrajectory(trajectory).error,
            TrajectoryContractError::kQuaternion);

  samples[0].orientation =
      embodiment::Quaternion{std::numeric_limits<double>::quiet_NaN(), 0, 0, 1};
  EXPECT_EQ(AssessVehicleTrajectory(trajectory).error,
            TrajectoryContractError::kNonFinite);
}

TEST(TrajectoryContractTest, FirstDefectFollowsTheLockedOrder) {
  TrajectorySampleView samples[2];
  VehicleTrajectoryView trajectory = ValidTrajectory(samples);
  trajectory.trajectory_id = "";
  trajectory.samples = {};
  EXPECT_EQ(AssessVehicleTrajectory(trajectory).error,
            TrajectoryContractError::kTrajectoryId);

  trajectory = ValidTrajectory(samples);
  samples[1].time_present = false;
  samples[0].orientation = embodiment::Quaternion{0, 0, 0, 2};
  trajectory.frame_id = "";
  EXPECT_EQ(AssessVehicleTrajectory(trajectory).error,
            TrajectoryContractError::kSampleTime);

  trajectory = ValidTrajectory(samples);
  samples[1].seconds = samples[0].seconds;
  samples[1].nanos = samples[0].nanos;
  samples[0].orientation = embodiment::Quaternion{0, 0, 0, 2};
  EXPECT_EQ(AssessVehicleTrajectory(trajectory).error,
            TrajectoryContractError::kNonMonotonicTime);

  trajectory = ValidTrajectory(samples);
  trajectory.frame_id = "";
  samples[0].orientation = embodiment::Quaternion{0, 0, 0, 2};
  samples[0].twist.linear_x = std::numeric_limits<double>::quiet_NaN();
  EXPECT_EQ(AssessVehicleTrajectory(trajectory).error,
            TrajectoryContractError::kMissingFrame);

  trajectory = ValidTrajectory(samples);
  samples[0].orientation = embodiment::Quaternion{0, 0, 0, 2};
  samples[0].twist.linear_x = std::numeric_limits<double>::quiet_NaN();
  EXPECT_EQ(AssessVehicleTrajectory(trajectory).error,
            TrajectoryContractError::kQuaternion);
}

TEST(TrajectoryContractTest, TolerancesCostRiskUncertaintyAndProvenance) {
  TrajectorySampleView samples[2];
  VehicleTrajectoryView trajectory = ValidTrajectory(samples);
  trajectory.tolerances_present = true;
  EXPECT_TRUE(AssessVehicleTrajectory(trajectory).accepted);
  trajectory.position_tolerance_m = -1;
  EXPECT_EQ(AssessVehicleTrajectory(trajectory).error,
            TrajectoryContractError::kTolerance);
  trajectory.position_tolerance_m = 0;
  trajectory.orientation_tolerance_rad =
      std::numeric_limits<double>::infinity();
  EXPECT_EQ(AssessVehicleTrajectory(trajectory).error,
            TrajectoryContractError::kTolerance);
  trajectory.tolerances_present = false;

  trajectory.cost_present = true;
  trajectory.cost = -3;
  EXPECT_TRUE(AssessVehicleTrajectory(trajectory).accepted);
  trajectory.cost = std::numeric_limits<double>::quiet_NaN();
  EXPECT_EQ(AssessVehicleTrajectory(trajectory).error,
            TrajectoryContractError::kCost);
  trajectory.cost_present = false;

  trajectory.risk_present = true;
  trajectory.risk = 0;
  EXPECT_TRUE(AssessVehicleTrajectory(trajectory).accepted);
  trajectory.risk = 1;
  EXPECT_TRUE(AssessVehicleTrajectory(trajectory).accepted);
  trajectory.risk = 1.1;
  EXPECT_EQ(AssessVehicleTrajectory(trajectory).error,
            TrajectoryContractError::kRisk);
  trajectory.risk = -0.1;
  EXPECT_EQ(AssessVehicleTrajectory(trajectory).error,
            TrajectoryContractError::kRisk);
  trajectory.risk = std::numeric_limits<double>::infinity();
  EXPECT_EQ(AssessVehicleTrajectory(trajectory).error,
            TrajectoryContractError::kRisk);
  trajectory.risk_present = false;

  const std::array<double, kCovarianceValues> zeros = {};
  trajectory.uncertainty_present = true;
  trajectory.uncertainty = zeros;
  EXPECT_TRUE(AssessVehicleTrajectory(trajectory).accepted);
  const std::array<double, 3> short_values = {};
  trajectory.uncertainty = short_values;
  EXPECT_EQ(AssessVehicleTrajectory(trajectory).error,
            TrajectoryContractError::kUncertainty);
  std::array<double, kCovarianceValues> nan_values = {};
  nan_values[0] = std::numeric_limits<double>::quiet_NaN();
  trajectory.uncertainty = nan_values;
  EXPECT_EQ(AssessVehicleTrajectory(trajectory).error,
            TrajectoryContractError::kUncertainty);
  std::array<double, kCovarianceValues> asymmetric = {};
  asymmetric[1] = 1;
  trajectory.uncertainty = asymmetric;
  EXPECT_EQ(AssessVehicleTrajectory(trajectory).error,
            TrajectoryContractError::kUncertainty);
  trajectory.uncertainty_present = false;
  EXPECT_TRUE(AssessVehicleTrajectory(trajectory).accepted);

  trajectory.model_id = "";
  EXPECT_EQ(AssessVehicleTrajectory(trajectory).error,
            TrajectoryContractError::kProvenance);
  trajectory.provenance_present = false;
  EXPECT_TRUE(AssessVehicleTrajectory(trajectory).accepted);
}

TEST(TrajectoryContractTest, InvalidStampIsNotRewrittenAndIsNotAccepted) {
  TrajectorySampleView samples[2];
  VehicleTrajectoryView trajectory = ValidTrajectory(samples);
  trajectory.validity_state = 2;
  const TrajectoryContractAssessment assessment =
      AssessVehicleTrajectory(trajectory);
  EXPECT_EQ(assessment.error, TrajectoryContractError::kNone);
  EXPECT_EQ(assessment.validity, ValidityKind::kInvalid);
  EXPECT_FALSE(assessment.accepted);
}

TEST(TrajectoryContractTest, AbsentValidityIsNotInvalid) {
  TrajectorySampleView samples[2];
  VehicleTrajectoryView trajectory = ValidTrajectory(samples);
  trajectory.validity_present = false;
  const TrajectoryContractAssessment assessment =
      AssessVehicleTrajectory(trajectory);
  EXPECT_EQ(assessment.error, TrajectoryContractError::kNone);
  EXPECT_EQ(assessment.validity, ValidityKind::kAbsent);
  EXPECT_FALSE(assessment.accepted);
  EXPECT_NE(assessment.validity, ValidityKind::kInvalid);
}

TEST(TrajectoryContractTest, BodyTwistIsNotConvertedWithThePoseFrame) {
  TrajectorySampleView samples[2];
  VehicleTrajectoryView trajectory = ValidTrajectory(samples);
  const embodiment::Vec3 body{1, 2, 3};
  const embodiment::Vec3 ned = embodiment::WorldVectorEnuToNed(body);
  EXPECT_NE(ned.x, body.x);
  samples[0].twist = BodyVector{body.x, body.y, body.z, 0, 0, 0};
  EXPECT_TRUE(AssessVehicleTrajectory(trajectory).accepted);
  EXPECT_EQ(samples[0].twist.linear_x, body.x);
  EXPECT_EQ(samples[0].twist.linear_y, body.y);
  EXPECT_EQ(samples[0].twist.linear_z, body.z);
  EXPECT_FALSE(embodiment::FrameIdMatches("VehicleTrajectory",
                                          embodiment::WorldFrame::kEnu));
}

TEST(TrajectoryContractTest, MetadataAloneEngagesAndNeedsAnId) {
  VehicleTrajectoryView trajectory;
  trajectory.metadata_present = true;
  EXPECT_TRUE(TrajectoryEngaged(trajectory));
  EXPECT_EQ(AssessVehicleTrajectory(trajectory).error,
            TrajectoryContractError::kTrajectoryId);
}

}  // namespace
}  // namespace intrinsic::vehicle
