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

#include "intrinsic/safety/geofence_rule.h"

#include <limits>
#include <string>
#include <string_view>

#include "gtest/gtest.h"
#include "intrinsic/safety/safety_decision_policy.h"

namespace intrinsic::safety {
namespace {

constexpr double kInf = std::numeric_limits<double>::infinity();
constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();
constexpr double kEps = 1e-6;

AabbGeofence Fence() {
  return AabbGeofence{"map", "dock_area", -1.0, -2.0, -3.0, 1.0, 2.0, 3.0};
}

GeofencePose Pose(double x, double y, double z) {
  return GeofencePose{"map", x, y, z};
}

void ExpectCompliant(const SafetyRuleResult& result) {
  EXPECT_FALSE(result.violated);
  EXPECT_TRUE(result.rule_id.empty());
  EXPECT_EQ(result.severity, 0);
  EXPECT_TRUE(result.summary.empty());
  EXPECT_EQ(result.recommended_kind, 0);
  EXPECT_FALSE(result.has_projected_value);
  EXPECT_EQ(result.projected_value, 0.0);
}

void ExpectRejectWithSeverity(const SafetyRuleResult& result, int severity) {
  EXPECT_TRUE(result.violated);
  EXPECT_EQ(result.rule_id, "geofence.aabb");
  EXPECT_EQ(result.rule_id, kGeofenceAabbRuleId);
  EXPECT_EQ(result.severity, severity);
  EXPECT_FALSE(result.summary.empty());
  EXPECT_EQ(result.recommended_kind, 3);
  EXPECT_EQ(result.recommended_kind, kDecisionKindReject);
  EXPECT_EQ(result.recommended_kind,
            static_cast<int>(SafetyDecisionKind::kReject));
  EXPECT_FALSE(result.has_projected_value);
  EXPECT_EQ(result.projected_value, 0.0);
}

void ExpectOutside(const SafetyRuleResult& result) {
  ExpectRejectWithSeverity(result, 3);
  EXPECT_EQ(result.severity,
            static_cast<int>(SafetyFindingSeverityKind::kError));
  EXPECT_NE(result.summary.find("dock_area"), std::string_view::npos)
      << result.summary;
}

void ExpectCritical(const SafetyRuleResult& result) {
  ExpectRejectWithSeverity(result, 4);
  EXPECT_EQ(result.severity,
            static_cast<int>(SafetyFindingSeverityKind::kCritical));
}

TEST(GeofenceAabbRuleTest, RuleIdIsLocked) {
  EXPECT_EQ(kGeofenceAabbRuleId, "geofence.aabb");
}

TEST(GeofenceAabbRuleTest, InsideCenterIsCompliant) {
  ExpectCompliant(EvaluateGeofenceAabbRule(Fence(), Pose(0.0, 0.0, 0.0)));
  ExpectCompliant(EvaluateGeofenceAabbRule(Fence(), Pose(0.5, -1.5, 2.5)));
}

TEST(GeofenceAabbRuleTest, FacesAreInclusive) {
  for (double v : {-1.0, 1.0}) {
    ExpectCompliant(EvaluateGeofenceAabbRule(Fence(), Pose(v, 0.0, 0.0)));
  }
  for (double v : {-2.0, 2.0}) {
    ExpectCompliant(EvaluateGeofenceAabbRule(Fence(), Pose(0.0, v, 0.0)));
  }
  for (double v : {-3.0, 3.0}) {
    ExpectCompliant(EvaluateGeofenceAabbRule(Fence(), Pose(0.0, 0.0, v)));
  }
}

TEST(GeofenceAabbRuleTest, EdgesAreInclusive) {
  for (double x : {-1.0, 1.0}) {
    for (double y : {-2.0, 2.0}) {
      ExpectCompliant(EvaluateGeofenceAabbRule(Fence(), Pose(x, y, 0.0)));
    }
    for (double z : {-3.0, 3.0}) {
      ExpectCompliant(EvaluateGeofenceAabbRule(Fence(), Pose(x, 0.0, z)));
    }
  }
  for (double y : {-2.0, 2.0}) {
    for (double z : {-3.0, 3.0}) {
      ExpectCompliant(EvaluateGeofenceAabbRule(Fence(), Pose(0.0, y, z)));
    }
  }
}

TEST(GeofenceAabbRuleTest, CornersAreInclusive) {
  for (double x : {-1.0, 1.0}) {
    for (double y : {-2.0, 2.0}) {
      for (double z : {-3.0, 3.0}) {
        ExpectCompliant(EvaluateGeofenceAabbRule(Fence(), Pose(x, y, z)));
      }
    }
  }
}

TEST(GeofenceAabbRuleTest, JustOutsideEachAxisIsRejected) {
  ExpectOutside(EvaluateGeofenceAabbRule(Fence(), Pose(1.0 + kEps, 0, 0)));
  ExpectOutside(EvaluateGeofenceAabbRule(Fence(), Pose(-1.0 - kEps, 0, 0)));
  ExpectOutside(EvaluateGeofenceAabbRule(Fence(), Pose(0, 2.0 + kEps, 0)));
  ExpectOutside(EvaluateGeofenceAabbRule(Fence(), Pose(0, -2.0 - kEps, 0)));
  ExpectOutside(EvaluateGeofenceAabbRule(Fence(), Pose(0, 0, 3.0 + kEps)));
  ExpectOutside(EvaluateGeofenceAabbRule(Fence(), Pose(0, 0, -3.0 - kEps)));
}

TEST(GeofenceAabbRuleTest, JustOutsideCombinationsAreRejected) {
  ExpectOutside(
      EvaluateGeofenceAabbRule(Fence(), Pose(1.0 + kEps, 2.0 + kEps, 0)));
  ExpectOutside(
      EvaluateGeofenceAabbRule(Fence(), Pose(1.0 + kEps, 0, 3.0 + kEps)));
  ExpectOutside(
      EvaluateGeofenceAabbRule(Fence(), Pose(0, 2.0 + kEps, 3.0 + kEps)));
  ExpectOutside(EvaluateGeofenceAabbRule(
      Fence(), Pose(1.0 + kEps, 2.0 + kEps, 3.0 + kEps)));
  ExpectOutside(EvaluateGeofenceAabbRule(Fence(), Pose(100.0, -100.0, 100.0)));
}

TEST(GeofenceAabbRuleTest, OutsideSummaryNamesRegionAndNeverProjects) {
  const SafetyRuleResult result =
      EvaluateGeofenceAabbRule(Fence(), Pose(5.0, 0.0, 0.0));
  EXPECT_EQ(result.summary, "geofence outside region=dock_area");
  EXPECT_NE(result.recommended_kind, kDecisionKindProject);
  EXPECT_FALSE(result.has_projected_value);
  EXPECT_EQ(result.projected_value, 0.0);
}

TEST(GeofenceAabbRuleTest, OutsideSummaryOutlivesCallerRegionString) {
  SafetyRuleResult result;
  {
    const std::string region = "temporary_region";
    AabbGeofence fence = Fence();
    fence.region_id = region;
    result = EvaluateGeofenceAabbRule(fence, Pose(5.0, 0.0, 0.0));
  }
  EXPECT_EQ(result.summary, "geofence outside region=temporary_region");
}

TEST(GeofenceAabbRuleTest, FrameMismatchIsCriticalEvenWhenCoordinatesInside) {
  const SafetyRuleResult result = EvaluateGeofenceAabbRule(
      Fence(), GeofencePose{"odom", 0.0, 0.0, 0.0});
  ExpectCritical(result);
  EXPECT_NE(result.summary.find("frame mismatch"), std::string_view::npos);
}

TEST(GeofenceAabbRuleTest, FrameMatchIsCaseSensitive) {
  ExpectCritical(
      EvaluateGeofenceAabbRule(Fence(), GeofencePose{"Map", 0.0, 0.0, 0.0}));
  ExpectCritical(
      EvaluateGeofenceAabbRule(Fence(), GeofencePose{"map ", 0.0, 0.0, 0.0}));
}

TEST(GeofenceAabbRuleTest, MinGreaterThanMaxOnAnyAxisIsCritical) {
  AabbGeofence fence = Fence();
  fence.min_x = 2.0;
  ExpectCritical(EvaluateGeofenceAabbRule(fence, Pose(0.0, 0.0, 0.0)));
  fence = Fence();
  fence.min_y = 3.0;
  ExpectCritical(EvaluateGeofenceAabbRule(fence, Pose(0.0, 0.0, 0.0)));
  fence = Fence();
  fence.max_z = -4.0;
  ExpectCritical(EvaluateGeofenceAabbRule(fence, Pose(0.0, 0.0, 0.0)));
}

TEST(GeofenceAabbRuleTest, DegenerateBoxIsAValidPoint) {
  AabbGeofence fence{"map", "pin", 1.0, 1.0, 1.0, 1.0, 1.0, 1.0};
  ExpectCompliant(EvaluateGeofenceAabbRule(fence, Pose(1.0, 1.0, 1.0)));
}

TEST(GeofenceAabbRuleTest, EmptyIdsAreCritical) {
  AabbGeofence fence = Fence();
  fence.frame_id = "";
  ExpectCritical(EvaluateGeofenceAabbRule(fence, GeofencePose{"", 0, 0, 0}));
  ExpectCritical(EvaluateGeofenceAabbRule(fence, Pose(0.0, 0.0, 0.0)));
  fence = Fence();
  fence.region_id = "";
  ExpectCritical(EvaluateGeofenceAabbRule(fence, Pose(0.0, 0.0, 0.0)));
  ExpectCritical(
      EvaluateGeofenceAabbRule(Fence(), GeofencePose{"", 0.0, 0.0, 0.0}));
}

TEST(GeofenceAabbRuleTest, NonFinitePoseIsCritical) {
  ExpectCritical(EvaluateGeofenceAabbRule(Fence(), Pose(kNaN, 0.0, 0.0)));
  ExpectCritical(EvaluateGeofenceAabbRule(Fence(), Pose(0.0, kNaN, 0.0)));
  ExpectCritical(EvaluateGeofenceAabbRule(Fence(), Pose(0.0, 0.0, kNaN)));
  ExpectCritical(EvaluateGeofenceAabbRule(Fence(), Pose(kInf, 0.0, 0.0)));
  ExpectCritical(EvaluateGeofenceAabbRule(Fence(), Pose(0.0, -kInf, 0.0)));
}

TEST(GeofenceAabbRuleTest, NonFiniteBoundIsCritical) {
  double AabbGeofence::*bounds[] = {&AabbGeofence::min_x, &AabbGeofence::min_y,
                                    &AabbGeofence::min_z, &AabbGeofence::max_x,
                                    &AabbGeofence::max_y, &AabbGeofence::max_z};
  for (auto bound : bounds) {
    for (double bad : {kNaN, kInf, -kInf}) {
      AabbGeofence fence = Fence();
      fence.*bound = bad;
      ExpectCritical(EvaluateGeofenceAabbRule(fence, Pose(0.0, 0.0, 0.0)));
    }
  }
}

TEST(GeofenceAabbSegmentRuleTest, BothInsideIsCompliant) {
  ExpectCompliant(EvaluateGeofenceAabbSegmentRule(
      Fence(), Pose(-1.0, -2.0, -3.0), Pose(1.0, 2.0, 3.0)));
  ExpectCompliant(EvaluateGeofenceAabbSegmentRule(Fence(), Pose(0, 0, 0),
                                                  Pose(0, 0, 0)));
}

TEST(GeofenceAabbSegmentRuleTest, StartOutsideIsRejected) {
  const SafetyRuleResult result = EvaluateGeofenceAabbSegmentRule(
      Fence(), Pose(5.0, 0.0, 0.0), Pose(0.0, 0.0, 0.0));
  ExpectOutside(result);
  EXPECT_EQ(result.summary,
            "geofence segment outside region=dock_area end=start");
}

TEST(GeofenceAabbSegmentRuleTest, EndOutsideIsRejected) {
  const SafetyRuleResult result = EvaluateGeofenceAabbSegmentRule(
      Fence(), Pose(0.0, 0.0, 0.0), Pose(0.0, 0.0, 5.0));
  ExpectOutside(result);
  EXPECT_EQ(result.summary,
            "geofence segment outside region=dock_area end=end");
}

TEST(GeofenceAabbSegmentRuleTest, BothOutsideIsRejected) {
  // Crosses the box, but endpoint-only checking rejects it by design.
  const SafetyRuleResult crossing = EvaluateGeofenceAabbSegmentRule(
      Fence(), Pose(-5.0, 0.0, 0.0), Pose(5.0, 0.0, 0.0));
  ExpectOutside(crossing);
  EXPECT_EQ(crossing.summary,
            "geofence segment outside region=dock_area end=both");
  ExpectOutside(EvaluateGeofenceAabbSegmentRule(
      Fence(), Pose(5.0, 5.0, 5.0), Pose(6.0, 6.0, 6.0)));
}

TEST(GeofenceAabbSegmentRuleTest, InvalidInputsAreCritical) {
  ExpectCritical(EvaluateGeofenceAabbSegmentRule(
      Fence(), GeofencePose{"odom", 0, 0, 0}, Pose(0, 0, 0)));
  ExpectCritical(EvaluateGeofenceAabbSegmentRule(
      Fence(), Pose(0, 0, 0), GeofencePose{"odom", 0, 0, 0}));
  ExpectCritical(EvaluateGeofenceAabbSegmentRule(Fence(), Pose(kNaN, 0, 0),
                                                 Pose(0, 0, 0)));
  ExpectCritical(EvaluateGeofenceAabbSegmentRule(Fence(), Pose(0, 0, 0),
                                                 Pose(0, kInf, 0)));
  ExpectCritical(EvaluateGeofenceAabbSegmentRule(
      Fence(), GeofencePose{"", 0, 0, 0}, Pose(0, 0, 0)));
}

TEST(GeofenceAabbSegmentRuleTest, InvalidPoseBeatsOutsideEndpoint) {
  ExpectCritical(EvaluateGeofenceAabbSegmentRule(
      Fence(), Pose(5.0, 0.0, 0.0), GeofencePose{"odom", 0, 0, 0}));
}

TEST(GeofenceAabbSegmentRuleTest, BadFenceIsCritical) {
  AabbGeofence fence = Fence();
  fence.min_x = 2.0;
  ExpectCritical(
      EvaluateGeofenceAabbSegmentRule(fence, Pose(0, 0, 0), Pose(0, 0, 0)));
  fence = Fence();
  fence.region_id = "";
  ExpectCritical(
      EvaluateGeofenceAabbSegmentRule(fence, Pose(0, 0, 0), Pose(0, 0, 0)));
  fence = Fence();
  fence.max_y = kNaN;
  ExpectCritical(
      EvaluateGeofenceAabbSegmentRule(fence, Pose(0, 0, 0), Pose(0, 0, 0)));
}

}  // namespace
}  // namespace intrinsic::safety
