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

#include "intrinsic_data/episode/planning_replay_adapter.h"

#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "google/protobuf/io/coded_stream.h"
#include "google/protobuf/io/zero_copy_stream_impl_lite.h"
#include "google/protobuf/message_lite.h"
#include "intrinsic/safety/safety_decision_assessor.h"
#include "intrinsic/vehicle/proto/vehicle_command.pb.h"
#include "intrinsic/vehicle/proto/vehicle_trajectory.pb.h"
#include "intrinsic/vehicle/trajectory_contract_policy.h"
#include "intrinsic/vehicle/vehicle_contract_policy.h"
#include "intrinsic_data/episode/episode_event_policy.h"
#include "intrinsic_data/episode/episode_replay.h"
#include "intrinsic_data/episode/episode_replay_adapter_common.h"

namespace intrinsic::episode {
namespace {

using intrinsic_proto::episode::EpisodeEvent;
using intrinsic_proto::vehicle::VehicleTrajectory;

bool CompatibilityValid(const ReplayCompatibilityConfig& config) {
  return config.expected_config_digest.empty() ||
         IsLowercaseSha256Hex(config.expected_config_digest);
}

std::string DeterministicSha256Hex(
    const google::protobuf::MessageLite& message) {
  std::string bytes;
  {
    google::protobuf::io::StringOutputStream stream(&bytes);
    google::protobuf::io::CodedOutputStream coded(&stream);
    coded.SetSerializationDeterministic(true);
    message.SerializeToCodedStream(&coded);
  }
  return internal::EpisodeSha256Hex(bytes);
}

struct TrajectoryStorage {
  std::vector<vehicle::TrajectorySampleView> samples;
  std::vector<double> uncertainty;
};

// Field for field into the plain view. Presence flags come from the proto
// has_*. String views point into `message`, which must outlive the view.
vehicle::VehicleTrajectoryView VehicleTrajectoryViewFromProto(
    const VehicleTrajectory& message, TrajectoryStorage* storage) {
  storage->samples.clear();
  storage->samples.reserve(message.samples_size());
  for (const auto& sample : message.samples()) {
    vehicle::TrajectorySampleView view;
    view.time_present = sample.has_time();
    view.seconds = sample.time().seconds();
    view.nanos = sample.time().nanos();
    const auto& pose = sample.pose_world_from_body();
    view.position = embodiment::Vec3{pose.position().x(), pose.position().y(),
                                     pose.position().z()};
    view.orientation =
        embodiment::Quaternion{pose.orientation().x(), pose.orientation().y(),
                               pose.orientation().z(), pose.orientation().w()};
    view.twist_present = sample.has_body_twist();
    const auto& twist = sample.body_twist();
    view.twist =
        vehicle::BodyVector{twist.linear_x_m_s(),    twist.linear_y_m_s(),
                            twist.linear_z_m_s(),    twist.angular_x_rad_s(),
                            twist.angular_y_rad_s(), twist.angular_z_rad_s()};
    view.acceleration_present = sample.has_body_acceleration();
    const auto& acceleration = sample.body_acceleration();
    view.acceleration = vehicle::BodyVector{
        acceleration.linear_x_m_s2(),    acceleration.linear_y_m_s2(),
        acceleration.linear_z_m_s2(),    acceleration.angular_x_rad_s2(),
        acceleration.angular_y_rad_s2(), acceleration.angular_z_rad_s2()};
    storage->samples.push_back(view);
  }
  vehicle::VehicleTrajectoryView view;
  view.header_present = message.has_header();
  view.validity_present = message.header().has_validity();
  view.validity_state = message.header().validity().state();
  view.frame_id = message.header().frame_id();
  view.trajectory_id = message.trajectory_id();
  view.samples = storage->samples;
  view.tolerances_present = message.has_tolerances();
  if (view.tolerances_present) {
    view.position_tolerance_m = message.tolerances().position_m();
    view.orientation_tolerance_rad = message.tolerances().orientation_rad();
    view.linear_velocity_tolerance_m_s =
        message.tolerances().linear_velocity_m_s();
    view.angular_velocity_tolerance_rad_s =
        message.tolerances().angular_velocity_rad_s();
  }
  view.cost_present = message.has_cost();
  view.cost = message.cost();
  view.risk_present = message.has_risk();
  view.risk = message.risk();
  view.uncertainty_present = message.has_uncertainty();
  if (view.uncertainty_present) {
    storage->uncertainty.assign(message.uncertainty().values().begin(),
                                message.uncertainty().values().end());
    view.uncertainty = storage->uncertainty;
  }
  view.provenance_present = message.has_provenance();
  view.model_id = message.provenance().model_id();
  view.metadata_present = message.metadata_size() > 0;
  return view;
}

}  // namespace

PlanningReplayAdapter::PlanningReplayAdapter(PlanningReplayConfig config)
    : config_(std::move(config)) {}

std::unique_ptr<PlanningReplayAdapter> PlanningReplayAdapter::Create(
    const PlanningReplayConfig& config, ReplayAdapterStatus* status) {
  ReplayAdapterStatus local_status = ReplayAdapterStatus::kInvalidConfig;
  ReplayAdapterStatus* out = status != nullptr ? status : &local_status;
  *out = ReplayAdapterStatus::kInvalidConfig;
  if (!CompatibilityValid(config.compatibility)) {
    return nullptr;
  }
  *out = ReplayAdapterStatus::kOk;
  return std::unique_ptr<PlanningReplayAdapter>(
      new PlanningReplayAdapter(config));
}

PlanningReplayStep PlanningReplayAdapter::Feed(
    const EpisodeReplayEvent& event) {
  PlanningReplayStep step;
  if (finished_) {
    step.status = ReplayAdapterStatus::kFinished;
    return step;
  }
  step.writer_index = event.writer_index;
  ++stats_.fed;
  if (latched_) {
    ++stats_.after_latch;
    step.status = ReplayAdapterStatus::kLatched;
    return step;
  }
  if (event.loss != nullptr) {
    ++stats_.gaps;
    stats_.gap_dropped += event.loss->dropped_count;
    degraded_ = true;
    step.status = ReplayAdapterStatus::kLossRecord;
    return step;
  }
  EpisodeEvent parsed;
  if (!ParseReplayRecord(event, &parsed)) {
    ++stats_.invalid_records;
    degraded_ = true;
    step.status = ReplayAdapterStatus::kInvalidRecord;
    return step;
  }
  const ReplayAdapterStatus compatibility =
      CheckCompatibility(parsed, config_.compatibility);
  if (compatibility != ReplayAdapterStatus::kOk) {
    latched_ = true;
    latched_status_ = compatibility;
    step.status = compatibility;
    return step;
  }
  const PayloadKind kind = event.record->payload_kind;
  if (kind == PayloadKind::kVehicleTrajectory) {
    const VehicleTrajectory& trajectory = parsed.vehicle_trajectory();
    TrajectoryStorage storage;
    const vehicle::TrajectoryContractAssessment assessment =
        vehicle::AssessVehicleTrajectory(
            VehicleTrajectoryViewFromProto(trajectory, &storage));
    step.payload_kind = kind;
    step.error = static_cast<int>(assessment.error);
    step.accepted = assessment.accepted;
    step.payload_sha256 = DeterministicSha256Hex(trajectory);
    ++stats_.trajectories;
    if (step.accepted) {
      ++stats_.trajectories_accepted;
      plan_chain_input_ += step.payload_sha256;
      plan_chain_input_ += '\n';
    }
  } else if (kind == PayloadKind::kDesiredMotion) {
    const auto& motion = parsed.desired_motion();
    const vehicle::ContractAssessment assessment = vehicle::AssessDesiredMotion(
        safety::DesiredMotionViewFromProto(motion));
    step.payload_kind = kind;
    step.error = static_cast<int>(assessment.error);
    step.accepted = assessment.accepted;
    step.payload_sha256 = DeterministicSha256Hex(motion);
    ++stats_.motions;
    if (step.accepted) {
      ++stats_.motions_accepted;
      intent_chain_input_ += step.payload_sha256;
      intent_chain_input_ += '\n';
    }
  } else {
    ++stats_.ignored;
    step.status = ReplayAdapterStatus::kIgnoredPayload;
    return step;
  }
  step.status = ReplayAdapterStatus::kOk;
  trace_.push_back(PlanningTraceEntry{event.writer_index, step.payload_kind,
                                      step.error, step.accepted,
                                      step.payload_sha256});
  return step;
}

PlanningReplaySummary PlanningReplayAdapter::Finish() {
  if (finished_) {
    return summary_;
  }
  finished_ = true;
  summary_.status = latched_ ? latched_status_ : ReplayAdapterStatus::kOk;
  summary_.degraded = degraded_;
  summary_.plan_chain =
      internal::EpisodeSha256Hex(std::string("v1\n") + plan_chain_input_);
  summary_.intent_chain =
      internal::EpisodeSha256Hex(std::string("v1\n") + intent_chain_input_);
  summary_.trace = trace_;
  summary_.stats = stats_;
  return summary_;
}

}  // namespace intrinsic::episode
