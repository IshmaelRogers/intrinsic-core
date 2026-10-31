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
#include <cstddef>
#include <cstdint>
#include <span>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include "intrinsic/embodiment/proto/stamped_header.pb.h"
#include "intrinsic/safety/angular_rate_max_rule.h"
#include "intrinsic/safety/attitude_bound_rule.h"
#include "intrinsic/safety/authority_transition_assessor.h"
#include "intrinsic/safety/authority_transition_policy.h"
#include "intrinsic/safety/depth_max_rule.h"
#include "intrinsic/safety/geofence_rule.h"
#include "intrinsic/safety/heave_rate_bound_rule.h"
#include "intrinsic/safety/non_finite_rule.h"
#include "intrinsic/safety/proto/authority_mode.pb.h"
#include "intrinsic/safety/proto/safety_decision.pb.h"
#include "intrinsic/safety/rule_aggregation.h"
#include "intrinsic/safety/safety_decision_assessor.h"
#include "intrinsic/safety/safety_decision_policy.h"
#include "intrinsic/safety/safety_rule_result.h"
#include "intrinsic/safety/speed_max_rule.h"
#include "intrinsic/safety/state_age_rule.h"
#include "intrinsic/vehicle/proto/vehicle_command.pb.h"
#include "intrinsic/vehicle/proto/vehicle_state.pb.h"
#include "intrinsic/vehicle/vehicle_contract_policy.h"
#include "intrinsic/world/proto/world_snapshot_descriptor.pb.h"

namespace intrinsic::safety {
namespace {

using intrinsic_proto::embodiment::Validity;
using ProtoAuthorityMode = intrinsic_proto::safety::AuthorityMode;
using intrinsic_proto::safety::SafetyDecision;
using ProtoDecisionKind = intrinsic_proto::safety::SafetyDecisionKind;
using intrinsic_proto::safety::SafetyFinding;
using intrinsic_proto::safety::SafetyFindingSeverity;
using intrinsic_proto::vehicle::DesiredMotion;
using intrinsic_proto::vehicle::VehicleState;
using intrinsic_proto::world::WorldSnapshotDescriptor;

constexpr std::string_view kWorldEnuFrameId = "world_enu";
constexpr double kSurfaceDepthOffsetM = 0.25;
constexpr double kMinQuaternionNorm = 1e-9;
constexpr int32_t kNanosPerSecond = 1000000000;

constexpr std::string_view kIntentInvalidId = "filter.intent.invalid";
constexpr std::string_view kProjectionInvalidId = "filter.projection.invalid";
constexpr std::string_view kFrameUnsupportedId = "filter.frame.unsupported";
constexpr std::string_view kObjectiveUnsupportedId =
    "filter.objective.unsupported";
constexpr std::string_view kPlannerNoSolutionId =
    "fallback.planner_no_solution";
constexpr std::string_view kDeadReckoningId = "fallback.dead_reckoning";

constexpr std::string_view kIntentInvalidSummary =
    "requested intent is not usable";
constexpr std::string_view kProjectionInvalidSummary =
    "projected intent failed its checks";
constexpr std::string_view kFrameUnsupportedSummary =
    "pose intent is not in world_enu";
constexpr std::string_view kObjectiveUnsupportedSummary =
    "trajectory objective is not supported";
constexpr std::string_view kFallbackSummary = "surface fallback requested";
constexpr std::string_view kAuthoritySummary =
    "decision kind is not permitted under the held authority mode";

bool PositiveFinite(double value) {
  return std::isfinite(value) && value > 0.0;
}

bool FenceValid(const AabbGeofence& fence) {
  return fence.frame_id == kWorldEnuFrameId && !fence.region_id.empty() &&
         std::isfinite(fence.min_x) && std::isfinite(fence.min_y) &&
         std::isfinite(fence.min_z) && std::isfinite(fence.max_x) &&
         std::isfinite(fence.max_y) && std::isfinite(fence.max_z) &&
         fence.min_x <= fence.max_x && fence.min_y <= fence.max_y &&
         fence.min_z <= fence.max_z;
}

bool ConfigValid(const ReferenceSafetyFilterConfig& config) {
  return std::isfinite(config.surface_z_m) &&
         PositiveFinite(config.max_depth_m) &&
         PositiveFinite(config.max_linear_speed_m_s) &&
         PositiveFinite(config.max_pitch_rad) &&
         PositiveFinite(config.max_roll_rad) &&
         PositiveFinite(config.max_angular_rate_rad_s) &&
         PositiveFinite(config.max_descent_rate_m_s) &&
         PositiveFinite(config.max_ascent_rate_m_s) &&
         FenceValid(config.fence) && config.state_max_age.seconds >= 0 &&
         config.state_max_age.nanos >= 0 &&
         config.state_max_age.nanos < kNanosPerSecond;
}

// A violated finding with owned strings. The geofence rule keeps its summary in
// a thread-local ring buffer, so every summary is copied on arrival.
struct Finding {
  std::string rule_id;
  int severity = 0;
  std::string summary;
  int recommended_kind = 0;
  bool has_projected_value = false;
  double projected_value = 0.0;
};

bool FindingLess(const Finding& a, const Finding& b) {
  if (a.rule_id != b.rule_id) return a.rule_id < b.rule_id;
  if (a.severity != b.severity) return a.severity < b.severity;
  return a.summary < b.summary;
}

class Findings {
 public:
  void Add(const SafetyRuleResult& result) {
    if (!result.violated) return;
    items_.push_back(
        Finding{std::string(result.rule_id), result.severity,
                std::string(result.summary), result.recommended_kind,
                result.has_projected_value, result.projected_value});
  }

  void AddReject(std::string_view rule_id, int severity,
                 std::string_view summary) {
    items_.push_back(Finding{std::string(rule_id), severity,
                             std::string(summary), kDecisionKindReject});
  }

  // The views point into this object. They are valid until the next Add.
  std::vector<SafetyRuleResult> RuleResults() const {
    std::vector<SafetyRuleResult> results;
    results.reserve(items_.size());
    for (const Finding& item : items_) {
      results.push_back(SafetyRuleResult{true, item.rule_id, item.severity,
                                         item.summary, item.recommended_kind,
                                         item.has_projected_value,
                                         item.projected_value});
    }
    return results;
  }

  std::vector<Finding> Sorted() const {
    std::vector<Finding> sorted = items_;
    std::sort(sorted.begin(), sorted.end(), FindingLess);
    return sorted;
  }

 private:
  std::vector<Finding> items_;
};

// Maps the state proto onto the plain view. String views and spans point into
// `state` and `sources`, which must outlive the view.
vehicle::VehicleStateView VehicleStateViewFromProto(
    const VehicleState& state, std::vector<vehicle::SourceView>* sources) {
  vehicle::VehicleStateView view;
  view.validity_present = state.header().has_validity();
  view.validity_state = state.header().validity().state();
  view.frame_id = state.header().frame_id();
  view.pose_present = state.has_pose_world_from_body();
  const auto& pose = state.pose_world_from_body();
  view.position = {pose.position().x(), pose.position().y(),
                   pose.position().z()};
  view.orientation = {pose.orientation().x(), pose.orientation().y(),
                      pose.orientation().z(), pose.orientation().w()};
  view.twist_present = state.has_body_twist();
  const auto& twist = state.body_twist();
  view.twist =
      vehicle::BodyVector{twist.linear_x_m_s(),    twist.linear_y_m_s(),
                          twist.linear_z_m_s(),    twist.angular_x_rad_s(),
                          twist.angular_y_rad_s(), twist.angular_z_rad_s()};
  view.acceleration_present = state.has_body_acceleration();
  const auto& acceleration = state.body_acceleration();
  view.acceleration = vehicle::BodyVector{
      acceleration.linear_x_m_s2(),    acceleration.linear_y_m_s2(),
      acceleration.linear_z_m_s2(),    acceleration.angular_x_rad_s2(),
      acceleration.angular_y_rad_s2(), acceleration.angular_z_rad_s2()};
  view.pose_covariance_present = state.has_pose_covariance();
  view.pose_covariance =
      std::span<const double>(state.pose_covariance().values().data(),
                              state.pose_covariance().values_size());
  view.twist_covariance_present = state.has_twist_covariance();
  view.twist_covariance =
      std::span<const double>(state.twist_covariance().values().data(),
                              state.twist_covariance().values_size());
  sources->clear();
  sources->reserve(state.sources_size());
  for (const auto& source : state.sources()) {
    sources->push_back(
        vehicle::SourceView{source.source_id(), source.has_validity(),
                            static_cast<int>(source.validity().state())});
  }
  view.sources = *sources;
  return view;
}

// ZYX roll and pitch from a quaternion. The quaternion is normalised first.
// Returns false when its norm is too small to normalise.
bool RollPitchFromQuaternion(double x, double y, double z, double w,
                             double* roll, double* pitch) {
  const double norm = std::sqrt(x * x + y * y + z * z + w * w);
  if (!std::isfinite(norm) || norm < kMinQuaternionNorm) return false;
  x /= norm;
  y /= norm;
  z /= norm;
  w /= norm;
  *pitch = std::asin(std::clamp(2.0 * (w * y - z * x), -1.0, 1.0));
  *roll = std::atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y));
  return true;
}

double Magnitude(double x, double y, double z) {
  return std::sqrt(x * x + y * y + z * z);
}

template <typename Timestamp>
void SetTime(Timestamp* stamp, StateTime time) {
  stamp->set_seconds(time.seconds);
  stamp->set_nanos(time.nanos);
}

bool IntentPasses(const DesiredMotion& intent) {
  const vehicle::DesiredMotionView view = DesiredMotionViewFromProto(intent);
  return vehicle::AssessDesiredMotion(view).accepted &&
         !EvaluateNonFiniteRule(view).violated;
}

std::string_view AuthorityName(ProtoAuthorityMode mode) {
  switch (mode) {
    case intrinsic_proto::safety::AUTHORITY_MODE_SHADOW:
      return "authority.shadow";
    case intrinsic_proto::safety::AUTHORITY_MODE_RECOMMEND:
      return "authority.recommend";
    case intrinsic_proto::safety::AUTHORITY_MODE_REVOKED:
      return "authority.revoked";
    case intrinsic_proto::safety::AUTHORITY_MODE_EMERGENCY:
      return "authority.emergency";
    default:
      return "authority.constrained";
  }
}

// D43-8 A. SHADOW, RECOMMEND and CONSTRAINED use the CONSTRAINED row. A denied
// kind becomes REJECT and its applied intent is dropped.
void GateAuthority(ProtoAuthorityMode held, ProtoDecisionKind* kind,
                   bool* has_applied, Findings* findings) {
  const bool uses_own_row =
      held == intrinsic_proto::safety::AUTHORITY_MODE_REVOKED ||
      held == intrinsic_proto::safety::AUTHORITY_MODE_EMERGENCY;
  const ProtoAuthorityMode row =
      uses_own_row ? held : intrinsic_proto::safety::AUTHORITY_MODE_CONSTRAINED;
  if (AssessDecisionKindUnderAuthority(row, *kind)) return;
  *kind = intrinsic_proto::safety::SAFETY_DECISION_KIND_REJECT;
  *has_applied = false;
  findings->AddReject(AuthorityName(held), kSeverityError, kAuthoritySummary);
}

ReferenceSafetyFilterResult BuildDecision(
    ProtoDecisionKind kind, const DesiredMotion* applied,
    const Findings& findings, const DesiredMotion& requested,
    const WorldSnapshotDescriptor& snapshot, StateTime query_time,
    uint64_t epoch) {
  ReferenceSafetyFilterResult result;
  SafetyDecision decision;
  auto* header = decision.mutable_header();
  header->mutable_validity()->set_state(Validity::STATE_VALID);
  SetTime(header->mutable_source_time(), query_time);
  header->set_frame_id(requested.header().frame_id());
  decision.set_kind(kind);
  decision.set_original_intent_digest(ComputeOriginalIntentDigest(requested));
  const bool needs_applied = KindRequiresAppliedIntent(
      ClassifySafetyDecisionKind(static_cast<int>(kind)));
  if (needs_applied != (applied != nullptr)) {
    result.error = ReferenceSafetyFilterError::kDecisionInvalid;
    return result;
  }
  if (applied != nullptr) {
    *decision.mutable_applied_intent() = *applied;
  }
  decision.set_snapshot_id(snapshot.snapshot_id());
  for (const Finding& finding : findings.Sorted()) {
    SafetyFinding* out = decision.add_findings();
    out->set_rule_id(finding.rule_id);
    out->set_severity(static_cast<SafetyFindingSeverity>(finding.severity));
    out->set_summary(finding.summary);
  }
  decision.set_decision_epoch(epoch);
  if (!AssessSafetyDecisionProto(decision).accepted) {
    result.error = ReferenceSafetyFilterError::kDecisionInvalid;
    return result;
  }
  result.error = ReferenceSafetyFilterError::kOk;
  result.decision = std::move(decision);
  return result;
}

}  // namespace

ReferenceSafetyFilter::ReferenceSafetyFilter(
    const ReferenceSafetyFilterConfig& config)
    : config_(config) {}

uint64_t ReferenceSafetyFilter::next_epoch() const { return epoch_; }

ReferenceSafetyFilterResult ReferenceSafetyFilter::Evaluate(
    const DesiredMotion& requested, const VehicleState& state,
    const WorldSnapshotDescriptor& snapshot, ProtoAuthorityMode held,
    StateTime query_time) {
  ReferenceSafetyFilterResult result;
  if (!ConfigValid(config_)) {
    result.error = ReferenceSafetyFilterError::kInvalidConfig;
    return result;
  }
  if (snapshot.snapshot_id().empty()) {
    result.error = ReferenceSafetyFilterError::kInvalidSnapshot;
    return result;
  }
  if (!IsLiveAuthorityMode(ClassifyAuthorityMode(static_cast<int>(held)))) {
    result.error = ReferenceSafetyFilterError::kNonLiveAuthority;
    return result;
  }

  Findings findings;
  const vehicle::DesiredMotionView intent =
      DesiredMotionViewFromProto(requested);
  const SafetyRuleResult intent_non_finite = EvaluateNonFiniteRule(intent);
  findings.Add(intent_non_finite);
  std::vector<vehicle::SourceView> sources;
  findings.Add(
      EvaluateNonFiniteRule(VehicleStateViewFromProto(state, &sources)));
  findings.Add(
      EvaluateStateAgeRule(query_time,
                           StateTime{state.header().source_time().seconds(),
                                     state.header().source_time().nanos()},
                           config_.state_max_age));

  double roll = 0.0;
  double pitch = 0.0;
  bool intent_usable = vehicle::AssessDesiredMotion(intent).accepted &&
                       !intent_non_finite.violated;
  if (intent_usable && intent.objective == vehicle::ObjectiveKind::kPose) {
    intent_usable = RollPitchFromQuaternion(
        intent.orientation.x, intent.orientation.y, intent.orientation.z,
        intent.orientation.w, &roll, &pitch);
  }

  if (!intent_usable) {
    findings.AddReject(kIntentInvalidId, kSeverityCritical,
                       kIntentInvalidSummary);
  } else if (intent.objective == vehicle::ObjectiveKind::kPose) {
    if (intent.frame_id != kWorldEnuFrameId) {
      findings.AddReject(kFrameUnsupportedId, kSeverityError,
                         kFrameUnsupportedSummary);
    } else {
      findings.Add(EvaluateDepthMaxRule(config_.surface_z_m - intent.position.z,
                                        config_.max_depth_m));
      findings.Add(EvaluateGeofenceAabbRule(
          config_.fence, GeofencePose{kWorldEnuFrameId, intent.position.x,
                                      intent.position.y, intent.position.z}));
      findings.Add(EvaluateAttitudePitchMaxRule(pitch, config_.max_pitch_rad));
      findings.Add(EvaluateAttitudeRollMaxRule(roll, config_.max_roll_rad));
    }
  } else if (intent.objective == vehicle::ObjectiveKind::kTwist) {
    const vehicle::BodyVector& twist = intent.twist;
    findings.Add(EvaluateSpeedMaxRule(
        Magnitude(twist.linear_x, twist.linear_y, twist.linear_z),
        config_.max_linear_speed_m_s));
    findings.Add(EvaluateAngularRateMaxRule(
        Magnitude(twist.angular_x, twist.angular_y, twist.angular_z),
        config_.max_angular_rate_rad_s));
    // Level-body assumption: ENU z is up, so depth_rate = -linear z.
    findings.Add(EvaluateHeaveRateBoundRule(-twist.linear_z,
                                            config_.max_descent_rate_m_s,
                                            config_.max_ascent_rate_m_s));
  } else {
    findings.AddReject(kObjectiveUnsupportedId, kSeverityError,
                       kObjectiveUnsupportedSummary);
  }

  const std::vector<SafetyRuleResult> results = findings.RuleResults();
  const AggregatedSafetyResult aggregate = AggregateSafetyRuleResults(results);
  const std::string primary_rule_id(aggregate.primary_rule_id);

  ProtoDecisionKind kind = intrinsic_proto::safety::SAFETY_DECISION_KIND_REJECT;
  DesiredMotion applied;
  bool has_applied = false;
  if (!aggregate.violated) {
    kind = intrinsic_proto::safety::SAFETY_DECISION_KIND_ACCEPT;
  } else if (aggregate.recommended_kind == kDecisionKindProject &&
             !aggregate.projection_conflict) {
    if (primary_rule_id == kDepthMaxRuleId) {
      applied = requested;
      applied.mutable_pose()
          ->mutable_pose_world_from_body()
          ->mutable_position()
          ->set_z(config_.surface_z_m - config_.max_depth_m);
      has_applied = true;
    } else if (primary_rule_id == kSpeedMaxRuleId) {
      const vehicle::BodyVector& twist = intent.twist;
      const double speed =
          Magnitude(twist.linear_x, twist.linear_y, twist.linear_z);
      const double scale = config_.max_linear_speed_m_s / speed;
      applied = requested;
      auto* body_twist = applied.mutable_twist()->mutable_body_twist();
      body_twist->set_linear_x_m_s(twist.linear_x * scale);
      body_twist->set_linear_y_m_s(twist.linear_y * scale);
      body_twist->set_linear_z_m_s(twist.linear_z * scale);
      has_applied = true;
    }
    if (has_applied) {
      if (IntentPasses(applied)) {
        kind = intrinsic_proto::safety::SAFETY_DECISION_KIND_PROJECT;
      } else {
        has_applied = false;
        findings.AddReject(kProjectionInvalidId, kSeverityCritical,
                           kProjectionInvalidSummary);
      }
    }
  }

  GateAuthority(held, &kind, &has_applied, &findings);
  result = BuildDecision(kind, has_applied ? &applied : nullptr, findings,
                         requested, snapshot, query_time, epoch_);
  if (result.error == ReferenceSafetyFilterError::kOk) ++epoch_;
  return result;
}

ReferenceSafetyFilterResult ReferenceSafetyFilter::Surface(
    SurfaceTrigger trigger, const DesiredMotion& requested, double truth_x_m,
    double truth_y_m, const WorldSnapshotDescriptor& snapshot,
    ProtoAuthorityMode held, StateTime query_time) {
  ReferenceSafetyFilterResult result;
  if (!ConfigValid(config_)) {
    result.error = ReferenceSafetyFilterError::kInvalidConfig;
    return result;
  }
  if (snapshot.snapshot_id().empty()) {
    result.error = ReferenceSafetyFilterError::kInvalidSnapshot;
    return result;
  }
  if (!IsLiveAuthorityMode(ClassifyAuthorityMode(static_cast<int>(held)))) {
    result.error = ReferenceSafetyFilterError::kNonLiveAuthority;
    return result;
  }
  if (requested.objective_case() == DesiredMotion::OBJECTIVE_NOT_SET) {
    result.error = ReferenceSafetyFilterError::kMissingTriggerIntent;
    return result;
  }

  Findings findings;
  findings.AddReject(trigger == SurfaceTrigger::kPlannerNoSolution
                         ? kPlannerNoSolutionId
                         : kDeadReckoningId,
                     kSeverityCritical, kFallbackSummary);

  ProtoDecisionKind kind = intrinsic_proto::safety::SAFETY_DECISION_KIND_REJECT;
  DesiredMotion applied;
  bool has_applied = false;
  if (!std::isfinite(truth_x_m) || !std::isfinite(truth_y_m)) {
    findings.AddReject(kIntentInvalidId, kSeverityCritical,
                       kIntentInvalidSummary);
  } else {
    auto* header = applied.mutable_header();
    header->mutable_validity()->set_state(Validity::STATE_VALID);
    SetTime(header->mutable_source_time(), query_time);
    header->set_frame_id(std::string(kWorldEnuFrameId));
    auto* pose = applied.mutable_pose()->mutable_pose_world_from_body();
    pose->mutable_position()->set_x(truth_x_m);
    pose->mutable_position()->set_y(truth_y_m);
    pose->mutable_position()->set_z(config_.surface_z_m - kSurfaceDepthOffsetM);
    pose->mutable_orientation()->set_x(0.0);
    pose->mutable_orientation()->set_y(0.0);
    pose->mutable_orientation()->set_z(0.0);
    pose->mutable_orientation()->set_w(1.0);
    if (IntentPasses(applied)) {
      kind = intrinsic_proto::safety::SAFETY_DECISION_KIND_SURFACE;
      has_applied = true;
    } else {
      findings.AddReject(kProjectionInvalidId, kSeverityCritical,
                         kProjectionInvalidSummary);
    }
  }

  GateAuthority(held, &kind, &has_applied, &findings);
  result = BuildDecision(kind, has_applied ? &applied : nullptr, findings,
                         requested, snapshot, query_time, epoch_);
  if (result.error == ReferenceSafetyFilterError::kOk) ++epoch_;
  return result;
}

}  // namespace intrinsic::safety
