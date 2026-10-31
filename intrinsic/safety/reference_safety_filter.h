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

#ifndef INTRINSIC_SAFETY_REFERENCE_SAFETY_FILTER_H_
#define INTRINSIC_SAFETY_REFERENCE_SAFETY_FILTER_H_

#include <cstdint>

#include "intrinsic/safety/angular_rate_max_rule.h"
#include "intrinsic/safety/attitude_bound_rule.h"
#include "intrinsic/safety/depth_max_rule.h"
#include "intrinsic/safety/geofence_rule.h"
#include "intrinsic/safety/heave_rate_bound_rule.h"
#include "intrinsic/safety/proto/authority_mode.pb.h"
#include "intrinsic/safety/proto/safety_decision.pb.h"
#include "intrinsic/safety/speed_max_rule.h"
#include "intrinsic/safety/state_age_rule.h"
#include "intrinsic/vehicle/proto/vehicle_command.pb.h"
#include "intrinsic/vehicle/proto/vehicle_state.pb.h"
#include "intrinsic/world/proto/world_snapshot_descriptor.pb.h"

namespace intrinsic::safety {

// Reference safety filter (L43-1, issue #261). Turns one requested
// DesiredMotion into one committed intrinsic_proto.safety.SafetyDecision by
// composing the existing rule functions, AggregateSafetyRuleResults, the
// authority gate and AssessSafetyDecisionProto. Contract: the
// `LOCK l43-1-reference-safety-filter-v1` comment on issue #261, ADR 0002
// D43-7 A, and S43-1 (#259).
//
// Pure and deterministic. No clock, I/O, threads or globals. Sim and test use
// only.

struct ReferenceSafetyFilterConfig {
  // world_enu z of the surface. Depth is surface_z_m - z.
  double surface_z_m = 0.0;
  double max_depth_m = kDefaultMaxDepth;
  double max_linear_speed_m_s = kDefaultMaxLinearSpeed;
  double max_pitch_rad = kDefaultMaxPitch;
  double max_roll_rad = kDefaultMaxRoll;
  double max_angular_rate_rad_s = kDefaultMaxAngularRate;
  double max_descent_rate_m_s = kDefaultMaxDescentRate;
  double max_ascent_rate_m_s = kDefaultMaxAscentRate;
  // frame_id must be "world_enu". The string views are borrowed: the strings
  // they point to must outlive the filter.
  AabbGeofence fence;
  StateTime state_max_age = kDefaultStateMaxAge;
};

enum class ReferenceSafetyFilterError {
  kOk,
  kInvalidConfig,
  kInvalidSnapshot,
  kNonLiveAuthority,
  kMissingTriggerIntent,
  kDecisionInvalid,
};

enum class SurfaceTrigger { kPlannerNoSolution, kDeadReckoning };

struct ReferenceSafetyFilterResult {
  ReferenceSafetyFilterError error = ReferenceSafetyFilterError::kInvalidConfig;
  // Empty unless error == kOk. Every decision returned with kOk has passed
  // AssessSafetyDecisionProto.
  intrinsic_proto::safety::SafetyDecision decision;
};

// Finding ids added by the filter itself, besides the rule ids.
//   filter.intent.invalid        CRITICAL  the requested intent is unusable
//   filter.projection.invalid    CRITICAL  the projected intent failed checks
//   filter.frame.unsupported     ERROR     a pose intent is not in world_enu
//   filter.objective.unsupported ERROR     a trajectory objective
//   authority.<mode>             ERROR     the authority gate denied the kind
//   fallback.planner_no_solution CRITICAL  Surface(kPlannerNoSolution)
//   fallback.dead_reckoning      CRITICAL  Surface(kDeadReckoning)
//
// Not thread-safe: use one caller per instance. The decision epoch is per
// instance. It starts at 1 and advances only when a decision is returned.
class ReferenceSafetyFilter {
 public:
  explicit ReferenceSafetyFilter(const ReferenceSafetyFilterConfig& config);

  // Judges `requested`. `state` supplies the state-age and non-finite checks.
  // The heave rate assumes a level body: depth_rate = -body_twist.linear.z.
  // `held` must be a live authority mode. `query_time` is simulation time.
  ReferenceSafetyFilterResult Evaluate(
      const intrinsic_proto::vehicle::DesiredMotion& requested,
      const intrinsic_proto::vehicle::VehicleState& state,
      const intrinsic_proto::world::WorldSnapshotDescriptor& snapshot,
      intrinsic_proto::safety::AuthorityMode held, StateTime query_time);

  // Builds a SURFACE decision for a fallback trigger. `requested` is the
  // intent that was superseded: it must have its objective set, and its digest
  // is recorded. truth_x_m and truth_y_m come from the caller.
  ReferenceSafetyFilterResult Surface(
      SurfaceTrigger trigger,
      const intrinsic_proto::vehicle::DesiredMotion& requested,
      double truth_x_m, double truth_y_m,
      const intrinsic_proto::world::WorldSnapshotDescriptor& snapshot,
      intrinsic_proto::safety::AuthorityMode held, StateTime query_time);

  // The epoch the next returned decision will carry. For tests.
  uint64_t next_epoch() const;

 private:
  ReferenceSafetyFilterConfig config_;
  uint64_t epoch_ = 1;
};

}  // namespace intrinsic::safety

#endif  // INTRINSIC_SAFETY_REFERENCE_SAFETY_FILTER_H_
