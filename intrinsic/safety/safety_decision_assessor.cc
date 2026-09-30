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

#include <cstdint>
#include <string>
#include <vector>

#include "intrinsic/embodiment/frame_policy.h"
#include "intrinsic/safety/proto/safety_decision.pb.h"
#include "intrinsic/safety/safety_decision_policy.h"
#include "intrinsic/vehicle/proto/vehicle_command.pb.h"
#include "intrinsic/vehicle/vehicle_contract_policy.h"
#include "openssl/sha.h"

namespace intrinsic::safety {

using intrinsic_proto::safety::SafetyDecision;
using intrinsic_proto::safety::SafetyFinding;
using intrinsic_proto::vehicle::DesiredMotion;

vehicle::DesiredMotionView DesiredMotionViewFromProto(
    const DesiredMotion& motion) {
  vehicle::DesiredMotionView view;
  view.header_present = motion.has_header();
  view.validity_present = motion.header().has_validity();
  view.validity_state = motion.header().validity().state();
  view.frame_id = motion.header().frame_id();
  switch (motion.objective_case()) {
    case DesiredMotion::kPose: {
      view.objective = vehicle::ObjectiveKind::kPose;
      const auto& pose = motion.pose().pose_world_from_body();
      view.position = embodiment::Vec3{pose.position().x(), pose.position().y(),
                                       pose.position().z()};
      view.orientation = embodiment::Quaternion{
          pose.orientation().x(), pose.orientation().y(),
          pose.orientation().z(), pose.orientation().w()};
      break;
    }
    case DesiredMotion::kTwist: {
      view.objective = vehicle::ObjectiveKind::kTwist;
      const auto& twist = motion.twist().body_twist();
      view.twist =
          vehicle::BodyVector{twist.linear_x_m_s(),    twist.linear_y_m_s(),
                              twist.linear_z_m_s(),    twist.angular_x_rad_s(),
                              twist.angular_y_rad_s(), twist.angular_z_rad_s()};
      break;
    }
    case DesiredMotion::kTrajectory:
      view.objective = vehicle::ObjectiveKind::kTrajectory;
      view.trajectory_id = motion.trajectory().trajectory_id();
      break;
    case DesiredMotion::OBJECTIVE_NOT_SET:
      view.objective = vehicle::ObjectiveKind::kAbsent;
      break;
  }
  view.confidence_present = motion.has_confidence();
  view.confidence = motion.confidence();
  view.horizon_present = motion.has_horizon();
  if (view.horizon_present) {
    view.horizon_seconds = motion.horizon().seconds();
    view.horizon_nanos = motion.horizon().nanos();
  }
  view.provenance_present = motion.has_provenance();
  view.model_id = motion.provenance().model_id();
  return view;
}

SafetyDecisionView SafetyDecisionViewFromProto(
    const SafetyDecision& decision,
    std::vector<SafetyFindingView>* findings_storage) {
  SafetyDecisionView view;
  const auto& header = decision.header();
  view.header_present = decision.has_header();
  view.header_validity_present = header.has_validity();
  view.header_validity_state = header.validity().state();
  view.header_source_time_present = header.has_source_time();
  view.header_source_time_nanos = header.source_time().nanos();
  view.header_receive_time_present = header.has_receive_time();
  view.header_receive_time_nanos = header.receive_time().nanos();
  view.kind = static_cast<int>(decision.kind());
  view.original_intent_digest = decision.original_intent_digest();
  view.applied_intent_present = decision.has_applied_intent();
  if (view.applied_intent_present) {
    view.applied_intent = DesiredMotionViewFromProto(decision.applied_intent());
  }
  view.snapshot_id = decision.snapshot_id();
  findings_storage->clear();
  findings_storage->reserve(decision.findings_size());
  for (const SafetyFinding& finding : decision.findings()) {
    findings_storage->push_back(SafetyFindingView{
        finding.rule_id(), static_cast<int>(finding.severity())});
  }
  view.findings = *findings_storage;
  view.decision_epoch_present = decision.has_decision_epoch();
  view.decision_epoch = decision.decision_epoch();
  return view;
}

SafetyDecisionAssessment AssessSafetyDecisionProto(
    const SafetyDecision& decision) {
  std::vector<SafetyFindingView> findings;
  return AssessSafetyDecision(SafetyDecisionViewFromProto(decision, &findings));
}

std::string ComputeOriginalIntentDigest(const DesiredMotion& motion) {
  const std::string bytes = motion.SerializeAsString();
  uint8_t digest[SHA256_DIGEST_LENGTH];
  SHA256(reinterpret_cast<const uint8_t*>(bytes.data()), bytes.size(), digest);
  static constexpr char kHex[] = "0123456789abcdef";
  std::string hex;
  hex.reserve(kSha256HexLength);
  for (uint8_t byte : digest) {
    hex.push_back(kHex[byte >> 4]);
    hex.push_back(kHex[byte & 0x0f]);
  }
  return hex;
}

}  // namespace intrinsic::safety
