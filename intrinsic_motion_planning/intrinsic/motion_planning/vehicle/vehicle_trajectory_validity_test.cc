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

#include "intrinsic/motion_planning/vehicle/vehicle_trajectory_validity.h"

#include <limits>
#include <string>
#include <string_view>
#include <vector>

#include "gtest/gtest.h"
#include "intrinsic/motion_planning/vehicle/vehicle_primitive_propagation.h"
#include "intrinsic/motion_planning/vehicle/vehicle_state_space.h"
#include "intrinsic/safety/clearance_rule.h"
#include "intrinsic/safety/geofence_rule.h"
#include "intrinsic/world/world_snapshot/world_snapshot_policy.h"
#include "intrinsic/world/world_snapshot/world_snapshot_skew_policy.h"

namespace intrinsic::motion_planning::vehicle {
namespace {

using ::intrinsic::safety::ClearanceSample;
using ::intrinsic::safety::ClearanceSource;
using ::intrinsic::world::ComponentRevisionView;
using ::intrinsic::world::ComponentTiming;
using ::intrinsic::world::WorldSnapshotView;

constexpr std::string_view kFrame = "world_enu";
constexpr std::string_view kRegion = "ops-box";
constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();
constexpr double kInf = std::numeric_limits<double>::infinity();

std::vector<ComponentRevisionView> Components() {
  return {{std::string(::intrinsic::world::kOccupancyReferenceKind), 1}};
}

WorldSnapshotView View() {
  WorldSnapshotView view;
  view.present = true;
  view.components = Components();
  view.state_epoch = 1;
  view.snapshot_id = ::intrinsic::world::ComputeSnapshotId(1, view.components);
  view.creation_time_present = true;
  view.creation_time = {100, 0};
  return view;
}

PropagationSample Sample(double t, double x, double y = 0.0, double z = -2.0) {
  PropagationSample sample;
  sample.time_s = t;
  sample.state.position = {x, y, z};
  return sample;
}

ClearanceSample Clear(double clearance_m = 2.0,
                      ClearanceSource source = ClearanceSource::kObstacle,
                      bool snapshot_usable = true) {
  ClearanceSample sample;
  sample.frame_id = kFrame;
  sample.source = source;
  sample.clearance_m = clearance_m;
  sample.snapshot_usable = snapshot_usable;
  return sample;
}

// The request borrows the snapshot id from `view`, so `view` must outlive it.
TrajectoryValidityRequest Request(const WorldSnapshotView& view) {
  TrajectoryValidityRequest request;
  request.snapshot_id = view.snapshot_id;
  request.descriptor = view;
  request.samples = {Sample(0.0, 0.0), Sample(0.5, 1.0), Sample(1.0, 2.0)};
  request.bounds.position_limits_present = true;
  request.bounds.position_min = {-50, -50, -50};
  request.bounds.position_max = {50, 50, 50};
  request.fence.frame_id = kFrame;
  request.fence.region_id = kRegion;
  request.fence.min_x = request.fence.min_y = request.fence.min_z = -10.0;
  request.fence.max_x = request.fence.max_y = request.fence.max_z = 10.0;
  request.clearance_samples = {Clear(), Clear(), Clear()};
  return request;
}

ComponentTiming OccupancyTiming(int64_t observed_s, int64_t horizon_s) {
  ComponentTiming timing;
  timing.component_kind =
      std::string(::intrinsic::world::kOccupancyReferenceKind);
  timing.observation_time = {observed_s, 0};
  timing.validity_horizon = {horizon_s, 0};
  return timing;
}

::intrinsic::world::WorldSnapshotSkewPolicy RequireOccupancy() {
  ::intrinsic::world::WorldSnapshotSkewPolicy policy;
  policy.required_kinds = {
      std::string(::intrinsic::world::kOccupancyReferenceKind)};
  return policy;
}

void ExpectResult(const TrajectoryValidityResult& result,
                  TrajectoryValidityError error, int index,
                  std::string_view rule) {
  EXPECT_EQ(result.error, error);
  EXPECT_EQ(result.first_invalid_sample, index);
  EXPECT_EQ(result.failed_rule, rule);
}

TEST(TrajectoryValidityTest, EnumWireValues) {
  EXPECT_EQ(static_cast<int>(TrajectoryValidityError::kOk), 0);
  EXPECT_EQ(static_cast<int>(TrajectoryValidityError::kBadRequest), 1);
  EXPECT_EQ(static_cast<int>(TrajectoryValidityError::kStaleSnapshot), 2);
  EXPECT_EQ(static_cast<int>(TrajectoryValidityError::kFrameError), 3);
  EXPECT_EQ(static_cast<int>(TrajectoryValidityError::kBounds), 4);
  EXPECT_EQ(static_cast<int>(TrajectoryValidityError::kGeofence), 5);
  EXPECT_EQ(static_cast<int>(TrajectoryValidityError::kClearance), 6);
}

TEST(TrajectoryValidityTest, DefaultResultIsBadRequest) {
  const TrajectoryValidityResult result;
  ExpectResult(result, TrajectoryValidityError::kBadRequest, -1, "");
}

TEST(TrajectoryValidityTest, SnapshotIdIsSha256Hex) {
  EXPECT_EQ(View().snapshot_id.size(),
            ::intrinsic::world::kSnapshotIdHexLength);
}

TEST(TrajectoryValidityTest, FreeTrajectoryIsOk) {
  const WorldSnapshotView view = View();
  ExpectResult(CheckTrajectoryValidity(Request(view)),
               TrajectoryValidityError::kOk, -1, "");
}

TEST(TrajectoryValidityTest, FreeWithFreshSkewIsOk) {
  const WorldSnapshotView view = View();
  TrajectoryValidityRequest request = Request(view);
  request.assess_skew = true;
  request.timings = {OccupancyTiming(100, 10)};
  request.query_time = {101, 0};
  request.skew_policy = RequireOccupancy();
  ExpectResult(CheckTrajectoryValidity(request), TrajectoryValidityError::kOk,
               -1, "");
}

TEST(TrajectoryValidityTest, ClearanceExactlyAtMinimumIsOk) {
  const WorldSnapshotView view = View();
  TrajectoryValidityRequest request = Request(view);
  request.clearance_samples = {Clear(0.5), Clear(0.5), Clear(0.5)};
  ExpectResult(CheckTrajectoryValidity(request), TrajectoryValidityError::kOk,
               -1, "");
}

TEST(TrajectoryValidityTest, CollisionReportsFirstSampleBelowMinimum) {
  const WorldSnapshotView view = View();
  TrajectoryValidityRequest request = Request(view);
  request.clearance_samples = {Clear(), Clear(0.4), Clear(0.01)};
  ExpectResult(CheckTrajectoryValidity(request),
               TrajectoryValidityError::kClearance, 1, "clearance.min");
}

TEST(TrajectoryValidityTest, LaterWorseSamplesDoNotMoveFirstIndex) {
  const WorldSnapshotView view = View();
  TrajectoryValidityRequest shallow = Request(view);
  shallow.clearance_samples = {Clear(), Clear(0.4), Clear()};
  TrajectoryValidityRequest deep = Request(view);
  deep.clearance_samples = {Clear(), Clear(0.4), Clear(-3.0)};
  const TrajectoryValidityResult a = CheckTrajectoryValidity(shallow);
  const TrajectoryValidityResult b = CheckTrajectoryValidity(deep);
  ExpectResult(a, TrajectoryValidityError::kClearance, 1, "clearance.min");
  EXPECT_EQ(a.error, b.error);
  EXPECT_EQ(a.first_invalid_sample, b.first_invalid_sample);
  EXPECT_EQ(a.failed_rule, b.failed_rule);
}

TEST(TrajectoryValidityTest, LowAltitudeSeafloorAtFirstSample) {
  const WorldSnapshotView view = View();
  TrajectoryValidityRequest request = Request(view);
  request.clearance_samples = {Clear(0.1, ClearanceSource::kSeafloor), Clear(),
                               Clear()};
  ExpectResult(CheckTrajectoryValidity(request),
               TrajectoryValidityError::kClearance, 0, "clearance.min");
}

TEST(TrajectoryValidityTest, UnknownMapIsClearanceFailure) {
  const WorldSnapshotView view = View();
  TrajectoryValidityRequest request = Request(view);
  request.clearance_samples = {Clear(), Clear(),
                               Clear(9.0, ClearanceSource::kUnknownMap)};
  ExpectResult(CheckTrajectoryValidity(request),
               TrajectoryValidityError::kClearance, 2, "clearance.min");
}

TEST(TrajectoryValidityTest, CustomMinClearanceIsApplied) {
  const WorldSnapshotView view = View();
  TrajectoryValidityRequest request = Request(view);
  request.min_clearance_m = 3.0;
  ExpectResult(CheckTrajectoryValidity(request),
               TrajectoryValidityError::kClearance, 0, "clearance.min");
}

TEST(TrajectoryValidityTest, StaleSkewWithholdsBeforeSamples) {
  const WorldSnapshotView view = View();
  TrajectoryValidityRequest request = Request(view);
  request.assess_skew = true;
  request.timings = {OccupancyTiming(0, 1)};
  request.query_time = {5, 0};
  request.skew_policy = RequireOccupancy();
  request.bounds.position_min = {100, 100, 100};
  request.bounds.position_max = {101, 101, 101};
  ExpectResult(CheckTrajectoryValidity(request),
               TrajectoryValidityError::kStaleSnapshot, -1, "snapshot");
}

TEST(TrajectoryValidityTest, MissingTimingForRequiredKindIsStale) {
  const WorldSnapshotView view = View();
  TrajectoryValidityRequest request = Request(view);
  request.assess_skew = true;
  request.skew_policy = RequireOccupancy();
  ExpectResult(CheckTrajectoryValidity(request),
               TrajectoryValidityError::kStaleSnapshot, -1, "snapshot");
}

TEST(TrajectoryValidityTest, StaleSkewIgnoredWhenNotAssessed) {
  const WorldSnapshotView view = View();
  TrajectoryValidityRequest request = Request(view);
  request.assess_skew = false;
  request.timings = {OccupancyTiming(0, 1)};
  request.query_time = {5, 0};
  request.skew_policy = RequireOccupancy();
  ExpectResult(CheckTrajectoryValidity(request), TrajectoryValidityError::kOk,
               -1, "");
}

TEST(TrajectoryValidityTest, SampleLocalSnapshotUnusableIsStale) {
  const WorldSnapshotView view = View();
  TrajectoryValidityRequest request = Request(view);
  request.clearance_samples = {
      Clear(), Clear(2.0, ClearanceSource::kObstacle, false), Clear()};
  ExpectResult(CheckTrajectoryValidity(request),
               TrajectoryValidityError::kStaleSnapshot, 1, "clearance.min");
}

TEST(TrajectoryValidityTest, FrameErrorFromPoseFrameOverride) {
  const WorldSnapshotView view = View();
  TrajectoryValidityRequest request = Request(view);
  request.pose_frame = "world_ned";
  ExpectResult(CheckTrajectoryValidity(request),
               TrajectoryValidityError::kFrameError, 0, "geofence.aabb");
}

TEST(TrajectoryValidityTest, PoseFrameEqualToFenceFrameIsOk) {
  const WorldSnapshotView view = View();
  TrajectoryValidityRequest request = Request(view);
  request.pose_frame = kFrame;
  ExpectResult(CheckTrajectoryValidity(request), TrajectoryValidityError::kOk,
               -1, "");
}

TEST(TrajectoryValidityTest, InvalidFenceIsBadRequestAtSample) {
  const WorldSnapshotView view = View();
  TrajectoryValidityRequest request = Request(view);
  request.fence.min_x = 10.0;
  request.fence.max_x = -10.0;
  ExpectResult(CheckTrajectoryValidity(request),
               TrajectoryValidityError::kBadRequest, 0, "geofence.aabb");
}

TEST(TrajectoryValidityTest, EmptySamplesIsBadRequest) {
  const WorldSnapshotView view = View();
  TrajectoryValidityRequest request = Request(view);
  request.samples.clear();
  request.clearance_samples.clear();
  ExpectResult(CheckTrajectoryValidity(request),
               TrajectoryValidityError::kBadRequest, -1, "");
}

TEST(TrajectoryValidityTest, ClearanceLengthMismatchIsBadRequest) {
  const WorldSnapshotView view = View();
  TrajectoryValidityRequest request = Request(view);
  request.clearance_samples = {Clear(), Clear()};
  ExpectResult(CheckTrajectoryValidity(request),
               TrajectoryValidityError::kBadRequest, -1, "");
}

TEST(TrajectoryValidityTest, EmptySnapshotIdIsBadRequest) {
  const WorldSnapshotView view = View();
  TrajectoryValidityRequest request = Request(view);
  request.snapshot_id = {};
  ExpectResult(CheckTrajectoryValidity(request),
               TrajectoryValidityError::kBadRequest, -1, "snapshot");
}

TEST(TrajectoryValidityTest, SnapshotIdMismatchIsBadRequest) {
  const WorldSnapshotView view = View();
  const std::string other =
      ::intrinsic::world::ComputeSnapshotId(2, Components());
  TrajectoryValidityRequest request = Request(view);
  request.snapshot_id = other;
  ExpectResult(CheckTrajectoryValidity(request),
               TrajectoryValidityError::kBadRequest, -1, "snapshot");
}

TEST(TrajectoryValidityTest, AbsentDescriptorIsBadRequest) {
  const WorldSnapshotView view = View();
  TrajectoryValidityRequest request = Request(view);
  request.descriptor = WorldSnapshotView{};
  ExpectResult(CheckTrajectoryValidity(request),
               TrajectoryValidityError::kBadRequest, -1, "snapshot");
}

TEST(TrajectoryValidityTest, StructurallyDefectiveDescriptorIsBadRequest) {
  const WorldSnapshotView view = View();
  TrajectoryValidityRequest request = Request(view);
  request.descriptor.creation_time_present = false;
  ExpectResult(CheckTrajectoryValidity(request),
               TrajectoryValidityError::kBadRequest, -1, "snapshot");
}

TEST(TrajectoryValidityTest, DuplicateComponentKindIsBadRequest) {
  const WorldSnapshotView view = View();
  TrajectoryValidityRequest request = Request(view);
  request.descriptor.components.push_back(request.descriptor.components[0]);
  ExpectResult(CheckTrajectoryValidity(request),
               TrajectoryValidityError::kBadRequest, -1, "snapshot");
}

TEST(TrajectoryValidityTest, ShapeDefectsPrecedeSnapshotDefects) {
  const WorldSnapshotView view = View();
  TrajectoryValidityRequest empty = Request(view);
  empty.samples.clear();
  empty.snapshot_id = {};
  ExpectResult(CheckTrajectoryValidity(empty),
               TrajectoryValidityError::kBadRequest, -1, "");
  TrajectoryValidityRequest mismatch = Request(view);
  mismatch.snapshot_id = {};
  mismatch.clearance_samples = {Clear()};
  ExpectResult(CheckTrajectoryValidity(mismatch),
               TrajectoryValidityError::kBadRequest, -1, "");
}

TEST(TrajectoryValidityTest, SnapshotDefectPrecedesSkewWithhold) {
  const WorldSnapshotView view = View();
  TrajectoryValidityRequest request = Request(view);
  request.snapshot_id = {};
  request.assess_skew = true;
  request.timings = {OccupancyTiming(0, 1)};
  request.query_time = {5, 0};
  request.skew_policy = RequireOccupancy();
  ExpectResult(CheckTrajectoryValidity(request),
               TrajectoryValidityError::kBadRequest, -1, "snapshot");
}

TEST(TrajectoryValidityTest, BoundsFailurePositionBox) {
  const WorldSnapshotView view = View();
  TrajectoryValidityRequest request = Request(view);
  request.bounds.position_min = {-1.5, -1.0, -3.0};
  request.bounds.position_max = {1.5, 1.0, -1.0};
  ExpectResult(CheckTrajectoryValidity(request),
               TrajectoryValidityError::kBounds, 2, "bounds");
}

TEST(TrajectoryValidityTest, BoundsFailureNonFiniteState) {
  const WorldSnapshotView view = View();
  TrajectoryValidityRequest request = Request(view);
  request.samples[1] = Sample(0.5, kNaN);
  ExpectResult(CheckTrajectoryValidity(request),
               TrajectoryValidityError::kBounds, 1, "bounds");
}

TEST(TrajectoryValidityTest, BoundsFailureBadBoundsConfiguration) {
  const WorldSnapshotView view = View();
  TrajectoryValidityRequest request = Request(view);
  request.bounds = VehicleStateBounds{};
  request.bounds.max_linear_speed_present = true;
  request.bounds.max_linear_speed_m_s = -1.0;
  ExpectResult(CheckTrajectoryValidity(request),
               TrajectoryValidityError::kBounds, 0, "bounds");
}

TEST(TrajectoryValidityTest, GeofenceOutside) {
  const WorldSnapshotView view = View();
  TrajectoryValidityRequest request = Request(view);
  request.samples[1] = Sample(0.5, 10.5);
  ExpectResult(CheckTrajectoryValidity(request),
               TrajectoryValidityError::kGeofence, 1, "geofence.aabb");
}

TEST(TrajectoryValidityTest, GeofenceBoundaryIsInside) {
  const WorldSnapshotView view = View();
  TrajectoryValidityRequest request = Request(view);
  request.samples[0] = Sample(0.0, 10.0);
  request.samples[1] = Sample(0.5, -10.0);
  ExpectResult(CheckTrajectoryValidity(request), TrajectoryValidityError::kOk,
               -1, "");
}

TEST(TrajectoryValidityTest, BoundsPrecedeGeofenceAndClearanceAtSameSample) {
  const WorldSnapshotView view = View();
  TrajectoryValidityRequest request = Request(view);
  request.samples[1] = Sample(0.5, 20.0);
  request.bounds.position_min = {-5, -5, -5};
  request.bounds.position_max = {5, 5, 5};
  request.clearance_samples = {Clear(), Clear(0.0), Clear()};
  ExpectResult(CheckTrajectoryValidity(request),
               TrajectoryValidityError::kBounds, 1, "bounds");
}

TEST(TrajectoryValidityTest, GeofencePrecedesClearanceAtSameSample) {
  const WorldSnapshotView view = View();
  TrajectoryValidityRequest request = Request(view);
  request.samples[1] = Sample(0.5, 20.0);
  request.clearance_samples = {Clear(), Clear(0.0), Clear()};
  ExpectResult(CheckTrajectoryValidity(request),
               TrajectoryValidityError::kGeofence, 1, "geofence.aabb");
}

TEST(TrajectoryValidityTest, EarlierClearanceFailureWinsOverLaterBounds) {
  const WorldSnapshotView view = View();
  TrajectoryValidityRequest request = Request(view);
  request.samples[2] = Sample(1.0, kInf);
  request.clearance_samples = {Clear(0.1), Clear(), Clear()};
  ExpectResult(CheckTrajectoryValidity(request),
               TrajectoryValidityError::kClearance, 0, "clearance.min");
}

TEST(TrajectoryValidityTest, DeterministicRepeat) {
  const WorldSnapshotView view = View();
  TrajectoryValidityRequest request = Request(view);
  request.clearance_samples = {Clear(), Clear(0.4), Clear()};
  const TrajectoryValidityResult first = CheckTrajectoryValidity(request);
  for (int i = 0; i < 5; ++i) {
    const TrajectoryValidityResult again = CheckTrajectoryValidity(request);
    EXPECT_EQ(again.error, first.error);
    EXPECT_EQ(again.first_invalid_sample, first.first_invalid_sample);
    EXPECT_EQ(again.failed_rule, first.failed_rule);
  }
}

}  // namespace
}  // namespace intrinsic::motion_planning::vehicle
