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

#include "intrinsic/estimation/fake_estimator_service.h"

#include <algorithm>
#include <cstdint>
#include <optional>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include "google/protobuf/io/coded_stream.h"
#include "google/protobuf/io/zero_copy_stream_impl_lite.h"
#include "google/protobuf/timestamp.pb.h"
#include "intrinsic/embodiment/frame_policy.h"
#include "intrinsic/embodiment/stamped_header_policy.h"
#include "intrinsic/estimation/estimator.pb.h"
#include "intrinsic/vehicle/proto/vehicle_state.pb.h"
#include "intrinsic/vehicle/vehicle_contract_policy.h"

namespace intrinsic::estimation {
namespace {

using intrinsic_proto::embodiment::Validity;
using intrinsic_proto::estimation::EstimatorConfig;
using intrinsic_proto::estimation::EstimatorResult;
using intrinsic_proto::estimation::GetStateResponse;
using intrinsic_proto::estimation::MeasurementEnvelope;
using intrinsic_proto::estimation::MeasurementKind;
using intrinsic_proto::estimation::RejectReason;
using intrinsic_proto::estimation::SourceStatus;
using intrinsic_proto::vehicle::NavigationMode;
using intrinsic_proto::vehicle::VehicleState;

constexpr std::string_view kDefaultEstimatorId = "estimator";
constexpr std::string_view kDefaultClockDomain =
    embodiment::kClockDomainMonotonic;
constexpr uint32_t kDefaultUnhealthyAfter = 3;

__int128 ToNanos(const google::protobuf::Timestamp& time) {
  return static_cast<__int128>(time.seconds()) * embodiment::kNanosPerSecond +
         time.nanos();
}

bool TimestampValid(const google::protobuf::Timestamp& time) {
  return embodiment::NanosInRange(time.nanos());
}

// SplitMix64. The Python mirror uses the same constants.
uint64_t SplitMix64(uint64_t* state) {
  *state += 0x9E3779B97F4A7C15ULL;
  uint64_t z = *state;
  z = (z ^ (z >> 30)) * 0xBF58476D1CE4E5B9ULL;
  z = (z ^ (z >> 27)) * 0x94D049BB133111EBULL;
  return z ^ (z >> 31);
}

// A multiple of 1/1024 in [0, 1), exact in binary floating point.
double Draw(uint64_t* state) {
  return static_cast<double>(SplitMix64(state) >> 54) / 1024.0;
}

bool IsAidingKind(MeasurementKind kind) {
  switch (kind) {
    case intrinsic_proto::estimation::MEASUREMENT_KIND_DVL:
    case intrinsic_proto::estimation::MEASUREMENT_KIND_PRESSURE:
    case intrinsic_proto::estimation::MEASUREMENT_KIND_ALTIMETER:
    case intrinsic_proto::estimation::MEASUREMENT_KIND_SURFACE_FIX:
      return true;
    default:
      return false;
  }
}

bool ValidKind(int kind) {
  return kind != intrinsic_proto::estimation::MEASUREMENT_KIND_UNSPECIFIED &&
         intrinsic_proto::estimation::MeasurementKind_IsValid(kind);
}

// True when the state passes the #17 AssessVehicleState accept path and holds
// a pose.
bool HasAcceptedPose(const VehicleState& state) {
  vehicle::VehicleStateView view;
  view.validity_present = state.header().has_validity();
  view.validity_state = state.header().validity().state();
  view.frame_id = state.header().frame_id();
  view.pose_present = state.has_pose_world_from_body();
  if (view.pose_present) {
    const auto& pose = state.pose_world_from_body();
    view.position = embodiment::Vec3{pose.position().x(), pose.position().y(),
                                     pose.position().z()};
    view.orientation =
        embodiment::Quaternion{pose.orientation().x(), pose.orientation().y(),
                               pose.orientation().z(), pose.orientation().w()};
  }
  view.twist_present = state.has_body_twist();
  const auto& twist = state.body_twist();
  view.twist =
      vehicle::BodyVector{twist.linear_x_m_s(),    twist.linear_y_m_s(),
                          twist.linear_z_m_s(),    twist.angular_x_rad_s(),
                          twist.angular_y_rad_s(), twist.angular_z_rad_s()};
  view.acceleration_present = state.has_body_acceleration();
  const auto& accel = state.body_acceleration();
  view.acceleration =
      vehicle::BodyVector{accel.linear_x_m_s2(),    accel.linear_y_m_s2(),
                          accel.linear_z_m_s2(),    accel.angular_x_rad_s2(),
                          accel.angular_y_rad_s2(), accel.angular_z_rad_s2()};
  std::vector<vehicle::SourceView> sources;
  for (const auto& source : state.sources()) {
    sources.push_back(vehicle::SourceView{
        source.source_id(), source.has_validity(), source.validity().state()});
  }
  view.sources = sources;
  view.pose_covariance_present = state.has_pose_covariance();
  view.pose_covariance = {
      state.pose_covariance().values().data(),
      static_cast<size_t>(state.pose_covariance().values_size())};
  view.twist_covariance_present = state.has_twist_covariance();
  view.twist_covariance = {
      state.twist_covariance().values().data(),
      static_cast<size_t>(state.twist_covariance().values_size())};
  return view.pose_present && vehicle::AssessVehicleState(view).accepted;
}

void FillIdentityHeader(const EstimatorConfig& config, uint64_t sequence,
                        VehicleState* state) {
  auto* header = state->mutable_header();
  header->set_sequence(sequence);
  header->set_source_id(config.estimator_id());
  header->set_frame_id(config.world_frame_id());
  header->set_clock_domain(config.clock_domain());
}

}  // namespace

EstimatorResult FakeEstimatorService::Result(RejectReason reason) const {
  EstimatorResult result;
  result.set_ok(reason == intrinsic_proto::estimation::REJECT_REASON_NONE);
  result.set_reason(reason);
  result.set_estimator_epoch(epoch_);
  return result;
}

void FakeEstimatorService::SetPayloadValidator(PayloadValidator validator) {
  validator_ = std::move(validator);
}

void FakeEstimatorService::ResetBuffers() {
  faulted_ = false;
  aided_since_predict_ = false;
  sources_.clear();
  last_predict_ns_.reset();
  state_ = VehicleState();
}

EstimatorResult FakeEstimatorService::Initialize(
    const EstimatorConfig& config) {
  const auto& skew = config.max_future_skew();
  if (!vehicle::DurationNonNegative(skew.seconds(), skew.nanos())) {
    return Result(intrinsic_proto::estimation::REJECT_REASON_INVALID_CONFIG);
  }
  config_ = config;
  if (config_.estimator_id().empty()) {
    config_.set_estimator_id(std::string(kDefaultEstimatorId));
  }
  if (config_.world_frame_id().empty()) {
    config_.set_world_frame_id(std::string(embodiment::kWorldEnuFrameId));
  }
  if (config_.clock_domain().empty()) {
    config_.set_clock_domain(std::string(kDefaultClockDomain));
  }
  if (config_.unhealthy_after_consecutive_rejects() == 0) {
    config_.set_unhealthy_after_consecutive_rejects(kDefaultUnhealthyAfter);
  }
  max_future_skew_ns_ =
      static_cast<__int128>(skew.seconds()) * embodiment::kNanosPerSecond +
      skew.nanos();

  ResetBuffers();
  initialized_ = true;
  ++epoch_;
  ++sequence_;

  uint64_t rng = config_.seed();
  const double x = Draw(&rng) * 64.0;
  const double y = Draw(&rng) * 64.0;
  const double z = -Draw(&rng) * 16.0;
  const double surge = Draw(&rng) * 2.0;

  FillIdentityHeader(config_, sequence_, &state_);
  state_.mutable_header()->mutable_validity()->set_state(Validity::STATE_VALID);
  auto* pose = state_.mutable_pose_world_from_body();
  pose->mutable_position()->set_x(x);
  pose->mutable_position()->set_y(y);
  pose->mutable_position()->set_z(z);
  pose->mutable_orientation()->set_w(1);
  state_.mutable_body_twist()->set_linear_x_m_s(surge);
  if (config_.publish_pose_covariance()) {
    auto* covariance = state_.mutable_pose_covariance();
    for (int i = 0; i < vehicle::kCovarianceValues; ++i) {
      covariance->add_values(0.0);
    }
    for (int i = 0; i < vehicle::kSpatialDof; ++i) {
      covariance->set_values(vehicle::CovarianceIndex(i, i),
                             i < 3 ? 0.25 : 0.0625);
    }
  }
  state_.set_mode(intrinsic_proto::vehicle::NAVIGATION_MODE_INITIALIZING);
  state_.set_estimator_epoch(epoch_);
  return Result(intrinsic_proto::estimation::REJECT_REASON_NONE);
}

FakeEstimatorService::Source* FakeEstimatorService::FindOrAddSource(
    const std::string& source_id) {
  for (Source& source : sources_) {
    if (source.status.source_id() == source_id) {
      return &source;
    }
  }
  sources_.emplace_back();
  sources_.back().status.set_source_id(source_id);
  return &sources_.back();
}

EstimatorResult FakeEstimatorService::Reject(Source* source,
                                             RejectReason reason) {
  if (source != nullptr) {
    SourceStatus& status = source->status;
    status.set_last_reject_reason(reason);
    status.set_consecutive_rejects(status.consecutive_rejects() + 1);
    if (status.consecutive_rejects() >=
        config_.unhealthy_after_consecutive_rejects()) {
      status.set_healthy(false);
    }
  }
  return Result(reason);
}

EstimatorResult FakeEstimatorService::Ingest(
    const MeasurementEnvelope& envelope) {
  if (!initialized_) {
    return Result(intrinsic_proto::estimation::REJECT_REASON_NOT_INITIALIZED);
  }
  if (faulted_) {
    return Result(intrinsic_proto::estimation::REJECT_REASON_ESTIMATOR_FAULTED);
  }
  if (envelope.source_id().empty()) {
    return Result(intrinsic_proto::estimation::REJECT_REASON_INVALID_ENVELOPE);
  }
  Source* source = FindOrAddSource(envelope.source_id());
  const auto& header = envelope.header();

  if (!ValidKind(envelope.kind()) || !header.has_source_time() ||
      !header.has_receive_time() || !TimestampValid(header.source_time()) ||
      !TimestampValid(header.receive_time()) ||
      (!envelope.payload().empty() && envelope.payload_type().empty())) {
    return Reject(source,
                  intrinsic_proto::estimation::REJECT_REASON_INVALID_ENVELOPE);
  }
  if (header.clock_domain() != config_.clock_domain()) {
    return Reject(
        source,
        intrinsic_proto::estimation::REJECT_REASON_CLOCK_DOMAIN_MISMATCH);
  }
  if (!embodiment::SampleAccepted(
          embodiment::ClassifyValidity(header.has_validity(),
                                       header.validity().state()),
          true)) {
    return Reject(source,
                  intrinsic_proto::estimation::REJECT_REASON_INVALID_PAYLOAD);
  }

  const __int128 source_ns = ToNanos(header.source_time());
  if (source_ns > ToNanos(header.receive_time()) + max_future_skew_ns_) {
    return Reject(
        source, intrinsic_proto::estimation::REJECT_REASON_FUTURE_MEASUREMENT);
  }
  if (source->accepted_once) {
    if (source_ns < source->last_accept_ns) {
      return Reject(source,
                    intrinsic_proto::estimation::REJECT_REASON_OUT_OF_ORDER);
    }
    if (source_ns == source->last_accept_ns) {
      return Reject(
          source,
          intrinsic_proto::estimation::REJECT_REASON_DUPLICATE_TIMESTAMP);
    }
  }
  if (validator_ && !validator_(envelope)) {
    return Reject(source,
                  intrinsic_proto::estimation::REJECT_REASON_INVALID_PAYLOAD);
  }

  source->accepted_once = true;
  source->last_accept_ns = source_ns;
  source->status.set_healthy(true);
  source->status.set_consecutive_rejects(0);
  *source->status.mutable_last_accept_source_time() = header.source_time();
  if (IsAidingKind(envelope.kind())) {
    aided_since_predict_ = true;
  }
  return Result(intrinsic_proto::estimation::REJECT_REASON_NONE);
}

EstimatorResult FakeEstimatorService::Predict(
    const google::protobuf::Timestamp& to_time) {
  if (!initialized_) {
    return Result(intrinsic_proto::estimation::REJECT_REASON_NOT_INITIALIZED);
  }
  if (!TimestampValid(to_time) ||
      (last_predict_ns_.has_value() && ToNanos(to_time) < *last_predict_ns_)) {
    return Result(intrinsic_proto::estimation::REJECT_REASON_INVALID_TIME);
  }
  last_predict_ns_ = ToNanos(to_time);
  auto* header = state_.mutable_header();
  header->set_sequence(++sequence_);
  *header->mutable_source_time() = to_time;
  *header->mutable_receive_time() = to_time;

  if (faulted_) {
    state_.set_mode(intrinsic_proto::vehicle::NAVIGATION_MODE_FAULTED);
  } else if (HasAcceptedPose(state_)) {
    state_.set_mode(
        aided_since_predict_
            ? intrinsic_proto::vehicle::NAVIGATION_MODE_AIDED
            : intrinsic_proto::vehicle::NAVIGATION_MODE_DEAD_RECKONING);
  }
  aided_since_predict_ = false;
  return Result(intrinsic_proto::estimation::REJECT_REASON_NONE);
}

EstimatorResult FakeEstimatorService::Reset(
    const std::optional<VehicleState>& prior) {
  if (!initialized_) {
    return Result(intrinsic_proto::estimation::REJECT_REASON_NOT_INITIALIZED);
  }
  ResetBuffers();
  ++epoch_;
  ++sequence_;
  const bool prior_accepted = prior.has_value() && HasAcceptedPose(*prior);
  if (prior_accepted) {
    state_ = *prior;
    state_.clear_sources();
    sequence_ = std::max(sequence_, state_.header().sequence());
    if (state_.header().has_source_time() &&
        TimestampValid(state_.header().source_time())) {
      last_predict_ns_ = ToNanos(state_.header().source_time());
    }
  } else {
    FillIdentityHeader(config_, sequence_, &state_);
  }
  state_.set_mode(intrinsic_proto::vehicle::NAVIGATION_MODE_INITIALIZING);
  state_.set_estimator_epoch(epoch_);
  EstimatorResult result =
      Result(intrinsic_proto::estimation::REJECT_REASON_NONE);
  result.set_prior_accepted(prior_accepted);
  return result;
}

bool FakeEstimatorService::InjectFault() {
  if (!initialized_) {
    return false;
  }
  faulted_ = true;
  state_.set_mode(intrinsic_proto::vehicle::NAVIGATION_MODE_FAULTED);
  return true;
}

bool FakeEstimatorService::InjectSourceFault(std::string_view source_id) {
  for (Source& source : sources_) {
    if (source.status.source_id() == source_id) {
      source.status.set_healthy(false);
      return true;
    }
  }
  return false;
}

GetStateResponse FakeEstimatorService::GetState() const {
  GetStateResponse response;
  if (!initialized_) {
    return response;
  }
  VehicleState* state = response.mutable_state();
  *state = state_;
  for (const Source& source : sources_) {
    *response.add_source_status() = source.status;
    if (source.accepted_once) {
      auto* health = state->add_sources();
      health->set_source_id(source.status.source_id());
      health->mutable_validity()->set_state(source.status.healthy()
                                                ? Validity::STATE_VALID
                                                : Validity::STATE_INVALID);
    }
  }
  return response;
}

uint64_t FakeEstimatorService::StateDigest() const {
  const GetStateResponse response = GetState();
  if (!response.has_state()) {
    return 0;
  }
  std::string bytes;
  {
    google::protobuf::io::StringOutputStream stream(&bytes);
    google::protobuf::io::CodedOutputStream coded(&stream);
    coded.SetSerializationDeterministic(true);
    response.state().SerializeToCodedStream(&coded);
  }
  uint64_t hash = 0xCBF29CE484222325ULL;
  for (unsigned char byte : bytes) {
    hash = (hash ^ byte) * 0x100000001B3ULL;
  }
  return hash;
}

}  // namespace intrinsic::estimation
