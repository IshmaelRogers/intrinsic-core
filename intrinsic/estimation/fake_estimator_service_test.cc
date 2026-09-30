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

// Mirrors fake_estimator_service_test.py.

#include "intrinsic/estimation/fake_estimator_service.h"

#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <fstream>
#include <limits>
#include <optional>
#include <sstream>
#include <string>
#include <vector>

#include "google/protobuf/io/coded_stream.h"
#include "google/protobuf/io/zero_copy_stream_impl_lite.h"
#include "google/protobuf/text_format.h"
#include "google/protobuf/timestamp.pb.h"
#include "gtest/gtest.h"
#include "intrinsic/estimation/estimator.pb.h"
#include "intrinsic/vehicle/proto/vehicle_state.pb.h"
#include "intrinsic/vehicle/vehicle_contract_policy.h"

namespace intrinsic::estimation {
namespace {

using intrinsic_proto::embodiment::Validity;
using intrinsic_proto::estimation::EstimatorConfig;
using intrinsic_proto::estimation::EstimatorResult;
using intrinsic_proto::estimation::MeasurementEnvelope;
using intrinsic_proto::estimation::MeasurementKind;
using intrinsic_proto::vehicle::NavigationMode;
using intrinsic_proto::vehicle::VehicleState;
namespace pb = intrinsic_proto::estimation;

// FNV-1a 64 of the deterministic serialization of the nominal snapshot. Keep
// in sync with fake_estimator_service_test.py.
constexpr uint64_t kNominalDigest = 0x603D34D9F8B840DDULL;

google::protobuf::Timestamp Ts(int64_t seconds, int32_t nanos = 0) {
  google::protobuf::Timestamp time;
  time.set_seconds(seconds);
  time.set_nanos(nanos);
  return time;
}

bool SameTime(const google::protobuf::Timestamp& lhs,
              const google::protobuf::Timestamp& rhs) {
  return lhs.seconds() == rhs.seconds() && lhs.nanos() == rhs.nanos();
}

struct EnvelopeOptions {
  std::optional<Validity::State> validity = Validity::STATE_VALID;
  std::string clock_domain = "monotonic";
  std::optional<google::protobuf::Timestamp> receive_time;
};

MeasurementEnvelope Envelope(const std::string& source_id, int kind,
                             google::protobuf::Timestamp source_time,
                             EnvelopeOptions options = EnvelopeOptions()) {
  MeasurementEnvelope envelope;
  envelope.set_source_id(source_id);
  envelope.set_kind(static_cast<MeasurementKind>(kind));
  envelope.set_payload_type("intrinsic_proto.marine.Sample");
  envelope.set_payload("\x08\x01");
  auto* header = envelope.mutable_header();
  header->set_source_id(source_id);
  header->set_clock_domain(options.clock_domain);
  *header->mutable_source_time() = source_time;
  *header->mutable_receive_time() =
      options.receive_time.has_value() ? *options.receive_time : source_time;
  if (options.validity.has_value()) {
    header->mutable_validity()->set_state(*options.validity);
  }
  return envelope;
}

EnvelopeOptions Received(google::protobuf::Timestamp receive_time) {
  EnvelopeOptions options;
  options.receive_time = receive_time;
  return options;
}

std::string Bytes(const VehicleState& state) {
  std::string bytes;
  {
    google::protobuf::io::StringOutputStream stream(&bytes);
    google::protobuf::io::CodedOutputStream coded(&stream);
    coded.SetSerializationDeterministic(true);
    state.SerializeToCodedStream(&coded);
  }
  return bytes;
}

bool SameMessage(const google::protobuf::Message& lhs,
                 const google::protobuf::Message& rhs) {
  return lhs.GetTypeName() == rhs.GetTypeName() &&
         lhs.SerializeAsString() == rhs.SerializeAsString();
}

std::string LoadGolden(const std::string& name) {
  const char* src = std::getenv("TEST_SRCDIR");
  if (src == nullptr) {
    return "";
  }
  const std::string suffix = "intrinsic/estimation/testdata/" + name;
  const char* workspace = std::getenv("TEST_WORKSPACE");
  std::vector<std::string> candidates = {
      std::string(src) + "/_main/" + suffix,
      std::string(src) + "/" + suffix,
  };
  if (workspace != nullptr) {
    candidates.push_back(std::string(src) + "/" + workspace + "/" + suffix);
  }
  for (const std::string& path : candidates) {
    std::ifstream in(path);
    if (!in) {
      continue;
    }
    std::ostringstream buffer;
    buffer << in.rdbuf();
    return buffer.str();
  }
  return "";
}

EstimatorConfig Config(uint64_t seed = 0, bool covariance = false) {
  EstimatorConfig config;
  config.set_seed(seed);
  config.set_publish_pose_covariance(covariance);
  return config;
}

FakeEstimatorService Ready(const EstimatorConfig& config) {
  FakeEstimatorService service;
  EXPECT_TRUE(service.Initialize(config).ok());
  return service;
}

void RunNominal(FakeEstimatorService* service) {
  ASSERT_TRUE(service->Initialize(Config(29, true)).ok());
  ASSERT_TRUE(service
                  ->Ingest(Envelope("dvl", pb::MEASUREMENT_KIND_DVL,
                                    Ts(1700000000, 250000000),
                                    Received(Ts(1700000000, 300000000))))
                  .ok());
  ASSERT_TRUE(service
                  ->Ingest(Envelope("depth", pb::MEASUREMENT_KIND_PRESSURE,
                                    Ts(1700000000, 500000000)))
                  .ok());
  ASSERT_TRUE(service->Predict(Ts(1700000001)).ok());
}

TEST(FakeEstimatorServiceTest, NeverInitializedStateIsAbsent) {
  FakeEstimatorService service;
  EXPECT_FALSE(service.GetState().has_state());
  EXPECT_EQ(service.StateDigest(), 0u);
  EXPECT_EQ(
      service.Ingest(Envelope("dvl", pb::MEASUREMENT_KIND_DVL, Ts(1))).reason(),
      pb::REJECT_REASON_NOT_INITIALIZED);
  EXPECT_EQ(service.Predict(Ts(1)).reason(), pb::REJECT_REASON_NOT_INITIALIZED);
  EXPECT_EQ(service.Reset(std::nullopt).reason(),
            pb::REJECT_REASON_NOT_INITIALIZED);
  EXPECT_FALSE(service.InjectFault());
}

TEST(FakeEstimatorServiceTest, InitializePublishesInitializingWithEpochOne) {
  FakeEstimatorService service;
  const EstimatorResult result = service.Initialize(Config(7));
  EXPECT_TRUE(result.ok());
  EXPECT_EQ(result.estimator_epoch(), 1u);
  const VehicleState state = service.GetState().state();
  EXPECT_EQ(state.mode(),
            intrinsic_proto::vehicle::NAVIGATION_MODE_INITIALIZING);
  EXPECT_EQ(state.estimator_epoch(), 1u);
  EXPECT_EQ(state.header().source_id(), "estimator");
  EXPECT_EQ(state.header().frame_id(), "world_enu");
  EXPECT_EQ(state.header().clock_domain(), "monotonic");
  EXPECT_TRUE(state.has_pose_world_from_body());
}

TEST(FakeEstimatorServiceTest, InvalidConfigChangesNothing) {
  FakeEstimatorService service = Ready(Config(1));
  const uint64_t before = service.StateDigest();
  EstimatorConfig bad;
  bad.mutable_max_future_skew()->set_seconds(-1);
  EXPECT_EQ(service.Initialize(bad).reason(), pb::REJECT_REASON_INVALID_CONFIG);
  EXPECT_EQ(service.GetState().state().estimator_epoch(), 1u);
  EXPECT_EQ(service.StateDigest(), before);
}

TEST(FakeEstimatorServiceTest, ReinitializeIsAFullResetWithEpochBump) {
  FakeEstimatorService service = Ready(Config(1));
  service.Ingest(Envelope("dvl", pb::MEASUREMENT_KIND_DVL, Ts(10)));
  service.Predict(Ts(11));
  EXPECT_EQ(service.Initialize(Config(1)).estimator_epoch(), 2u);
  const auto response = service.GetState();
  EXPECT_EQ(response.state().mode(),
            intrinsic_proto::vehicle::NAVIGATION_MODE_INITIALIZING);
  EXPECT_FALSE(response.state().header().has_source_time());
  EXPECT_EQ(response.source_status_size(), 0);
  EXPECT_EQ(response.state().sources_size(), 0);
  EXPECT_TRUE(
      service.Ingest(Envelope("dvl", pb::MEASUREMENT_KIND_DVL, Ts(10))).ok());
}

TEST(FakeEstimatorServiceTest, CovariancePresentOrAbsent) {
  FakeEstimatorService with = Ready(Config(3, true));
  const VehicleState state = with.GetState().state();
  ASSERT_TRUE(state.has_pose_covariance());
  EXPECT_EQ(
      vehicle::AssessCovariance(
          true, {state.pose_covariance().values().data(),
                 static_cast<size_t>(state.pose_covariance().values_size())}),
      vehicle::CovarianceError::kNone);
  EXPECT_FALSE(state.has_twist_covariance());
  FakeEstimatorService without = Ready(Config(3));
  EXPECT_FALSE(without.GetState().state().has_pose_covariance());
  EXPECT_FALSE(without.GetState().state().has_twist_covariance());
}

TEST(FakeEstimatorServiceTest,
     ModeInitializingThenDeadReckoningOnFirstPredict) {
  FakeEstimatorService service = Ready(Config(9));
  ASSERT_TRUE(service.Predict(Ts(100)).ok());
  VehicleState state = service.GetState().state();
  EXPECT_EQ(state.mode(),
            intrinsic_proto::vehicle::NAVIGATION_MODE_DEAD_RECKONING);
  EXPECT_EQ(state.header().source_time().seconds(), 100);
  EXPECT_EQ(state.header().receive_time().seconds(), 100);
  ASSERT_TRUE(service.Predict(Ts(101)).ok());
  EXPECT_EQ(service.GetState().state().mode(),
            intrinsic_proto::vehicle::NAVIGATION_MODE_DEAD_RECKONING);
}

TEST(FakeEstimatorServiceTest, AcceptedAidingIngestMakesNextPredictAidedOnce) {
  FakeEstimatorService service = Ready(Config(9));
  ASSERT_TRUE(
      service.Ingest(Envelope("dvl", pb::MEASUREMENT_KIND_DVL, Ts(10))).ok());
  EXPECT_EQ(service.GetState().state().mode(),
            intrinsic_proto::vehicle::NAVIGATION_MODE_INITIALIZING);
  service.Predict(Ts(11));
  EXPECT_EQ(service.GetState().state().mode(),
            intrinsic_proto::vehicle::NAVIGATION_MODE_AIDED);
  service.Predict(Ts(12));
  EXPECT_EQ(service.GetState().state().mode(),
            intrinsic_proto::vehicle::NAVIGATION_MODE_DEAD_RECKONING);
}

TEST(FakeEstimatorServiceTest, PropagationKindsDoNotAid) {
  FakeEstimatorService service = Ready(Config(9));
  int index = 0;
  for (int kind : {pb::MEASUREMENT_KIND_IMU, pb::MEASUREMENT_KIND_INS,
                   pb::MEASUREMENT_KIND_THRUSTER_FEEDBACK}) {
    EXPECT_TRUE(
        service.Ingest(Envelope("src" + std::to_string(index++), kind, Ts(10)))
            .ok());
  }
  service.Predict(Ts(11));
  EXPECT_EQ(service.GetState().state().mode(),
            intrinsic_proto::vehicle::NAVIGATION_MODE_DEAD_RECKONING);
}

TEST(FakeEstimatorServiceTest, PredictHoldsKinematicsAndRejectsTimeRegression) {
  FakeEstimatorService service = Ready(Config(9, true));
  const VehicleState before = service.GetState().state();
  service.Predict(Ts(50));
  const VehicleState after = service.GetState().state();
  EXPECT_TRUE(
      SameMessage(after.pose_world_from_body(), before.pose_world_from_body()));
  EXPECT_TRUE(SameMessage(after.body_twist(), before.body_twist()));
  EXPECT_TRUE(SameMessage(after.pose_covariance(), before.pose_covariance()));
  EXPECT_GT(after.header().sequence(), before.header().sequence());
  EXPECT_EQ(service.Predict(Ts(49)).reason(), pb::REJECT_REASON_INVALID_TIME);
  EXPECT_EQ(service.GetState().state().header().source_time().seconds(), 50);
  EXPECT_TRUE(service.Predict(Ts(50)).ok());
  EXPECT_EQ(service.Predict(Ts(60, 1000000000)).reason(),
            pb::REJECT_REASON_INVALID_TIME);
}

TEST(FakeEstimatorServiceTest, ResetBumpsEpochAndEmptiesTheBodyWithoutPrior) {
  FakeEstimatorService service = Ready(Config(9));
  service.Predict(Ts(5));
  const EstimatorResult result = service.Reset(std::nullopt);
  EXPECT_TRUE(result.ok());
  EXPECT_FALSE(result.prior_accepted());
  EXPECT_EQ(result.estimator_epoch(), 2u);
  VehicleState state = service.GetState().state();
  EXPECT_FALSE(state.has_pose_world_from_body());
  EXPECT_FALSE(state.has_body_twist());
  EXPECT_EQ(state.mode(),
            intrinsic_proto::vehicle::NAVIGATION_MODE_INITIALIZING);
  EXPECT_EQ(state.estimator_epoch(), 2u);
  service.Predict(Ts(6));
  EXPECT_EQ(service.GetState().state().mode(),
            intrinsic_proto::vehicle::NAVIGATION_MODE_INITIALIZING);
  EXPECT_EQ(service.Reset(std::nullopt).estimator_epoch(), 3u);
}

TEST(FakeEstimatorServiceTest, ResetWithAcceptedPriorBecomesTheBody) {
  FakeEstimatorService service = Ready(Config(9));
  VehicleState prior;
  ASSERT_TRUE(google::protobuf::TextFormat::ParseFromString(
      R"pb(
        header {
          sequence: 500
          frame_id: "world_enu"
          clock_domain: "monotonic"
          source_time { seconds: 40 }
          validity { state: STATE_VALID }
        }
        pose_world_from_body {
          position { x: 1 y: 2 z: -3 }
          orientation { w: 1 }
        }
        mode: NAVIGATION_MODE_AIDED
        sources { source_id: "stale" }
        estimator_epoch: 99
      )pb",
      &prior));
  const EstimatorResult result = service.Reset(prior);
  EXPECT_TRUE(result.prior_accepted());
  VehicleState state = service.GetState().state();
  EXPECT_TRUE(
      SameMessage(state.pose_world_from_body(), prior.pose_world_from_body()));
  EXPECT_EQ(state.mode(),
            intrinsic_proto::vehicle::NAVIGATION_MODE_INITIALIZING);
  EXPECT_EQ(state.estimator_epoch(), 2u);
  EXPECT_EQ(state.sources_size(), 0);
  EXPECT_EQ(service.Predict(Ts(39)).reason(), pb::REJECT_REASON_INVALID_TIME);
  EXPECT_TRUE(service.Predict(Ts(41)).ok());
  state = service.GetState().state();
  EXPECT_EQ(state.mode(),
            intrinsic_proto::vehicle::NAVIGATION_MODE_DEAD_RECKONING);
  EXPECT_GT(state.header().sequence(), 500u);
}

TEST(FakeEstimatorServiceTest, ResetWithRejectedPriorIsEmpty) {
  FakeEstimatorService service = Ready(Config(9));
  VehicleState prior;
  prior.mutable_header()->set_frame_id("world_enu");
  prior.mutable_header()->mutable_validity()->set_state(Validity::STATE_VALID);
  prior.mutable_pose_world_from_body()->mutable_position()->set_x(
      std::numeric_limits<double>::quiet_NaN());
  prior.mutable_pose_world_from_body()->mutable_orientation()->set_w(1);
  const EstimatorResult result = service.Reset(prior);
  EXPECT_TRUE(result.ok());
  EXPECT_FALSE(result.prior_accepted());
  EXPECT_FALSE(service.GetState().state().has_pose_world_from_body());
  VehicleState no_validity;
  no_validity.mutable_header()->set_frame_id("world_enu");
  no_validity.mutable_pose_world_from_body()->mutable_orientation()->set_w(1);
  EXPECT_FALSE(service.Reset(no_validity).prior_accepted());
}

TEST(FakeEstimatorServiceTest, IngestAcceptAndSourceHealth) {
  FakeEstimatorService service = Ready(Config(2));
  const EstimatorResult result =
      service.Ingest(Envelope("dvl", pb::MEASUREMENT_KIND_DVL, Ts(10, 5)));
  EXPECT_TRUE(result.ok());
  EXPECT_EQ(result.reason(), pb::REJECT_REASON_NONE);
  const auto response = service.GetState();
  ASSERT_EQ(response.source_status_size(), 1);
  const auto& status = response.source_status(0);
  EXPECT_EQ(status.source_id(), "dvl");
  EXPECT_TRUE(status.healthy());
  EXPECT_TRUE(SameTime(status.last_accept_source_time(), Ts(10, 5)));
  EXPECT_FALSE(status.has_last_reject_reason());
  ASSERT_EQ(response.state().sources_size(), 1);
  EXPECT_EQ(response.state().sources(0).source_id(), "dvl");
  EXPECT_EQ(response.state().sources(0).validity().state(),
            Validity::STATE_VALID);
}

TEST(FakeEstimatorServiceTest, RejectLeavesPublishedStateUnchanged) {
  EstimatorConfig config = Config(2, true);
  config.set_unhealthy_after_consecutive_rejects(10);
  FakeEstimatorService service = Ready(config);
  service.Ingest(Envelope("dvl", pb::MEASUREMENT_KIND_DVL, Ts(10)));
  service.Predict(Ts(11));
  const std::string before = Bytes(service.GetState().state());
  const uint64_t digest = service.StateDigest();
  EXPECT_FALSE(
      service.Ingest(Envelope("dvl", pb::MEASUREMENT_KIND_DVL, Ts(9))).ok());
  EXPECT_FALSE(
      service.Ingest(Envelope("dvl", pb::MEASUREMENT_KIND_DVL, Ts(10))).ok());
  EXPECT_FALSE(service
                   .Ingest(Envelope("dvl", pb::MEASUREMENT_KIND_DVL, Ts(20),
                                    Received(Ts(12))))
                   .ok());
  EXPECT_EQ(Bytes(service.GetState().state()), before);
  EXPECT_EQ(service.StateDigest(), digest);
  service.Ingest(
      Envelope("ghost", pb::MEASUREMENT_KIND_DVL, Ts(99), Received(Ts(1))));
  EXPECT_EQ(Bytes(service.GetState().state()), before);
  EXPECT_EQ(service.GetState().source_status_size(), 2);
}

TEST(FakeEstimatorServiceTest, FutureMeasurementRejectedWithDefaultSkewZero) {
  FakeEstimatorService service = Ready(Config(2));
  EXPECT_EQ(service
                .Ingest(Envelope("dvl", pb::MEASUREMENT_KIND_DVL, Ts(10, 1),
                                 Received(Ts(10))))
                .reason(),
            pb::REJECT_REASON_FUTURE_MEASUREMENT);
  EXPECT_TRUE(service
                  .Ingest(Envelope("dvl", pb::MEASUREMENT_KIND_DVL, Ts(10),
                                   Received(Ts(10))))
                  .ok());
}

TEST(FakeEstimatorServiceTest, FutureSkewIsHonored) {
  EstimatorConfig config = Config(2);
  config.mutable_max_future_skew()->set_nanos(500000000);
  FakeEstimatorService service = Ready(config);
  EXPECT_TRUE(service
                  .Ingest(Envelope("dvl", pb::MEASUREMENT_KIND_DVL,
                                   Ts(10, 500000000), Received(Ts(10))))
                  .ok());
  EXPECT_EQ(service
                .Ingest(Envelope("dvl", pb::MEASUREMENT_KIND_DVL,
                                 Ts(11, 500000001), Received(Ts(11))))
                .reason(),
            pb::REJECT_REASON_FUTURE_MEASUREMENT);
}

TEST(FakeEstimatorServiceTest, OutOfOrderAndDuplicateAreRejectedPerSource) {
  FakeEstimatorService service = Ready(Config(2));
  EXPECT_TRUE(
      service.Ingest(Envelope("dvl", pb::MEASUREMENT_KIND_DVL, Ts(10))).ok());
  EXPECT_EQ(
      service
          .Ingest(Envelope("dvl", pb::MEASUREMENT_KIND_DVL, Ts(9, 999999999)))
          .reason(),
      pb::REJECT_REASON_OUT_OF_ORDER);
  EXPECT_EQ(service.Ingest(Envelope("dvl", pb::MEASUREMENT_KIND_DVL, Ts(10)))
                .reason(),
            pb::REJECT_REASON_DUPLICATE_TIMESTAMP);
  EXPECT_TRUE(
      service.Ingest(Envelope("depth", pb::MEASUREMENT_KIND_PRESSURE, Ts(5)))
          .ok());
  const auto response = service.GetState();
  const auto& dvl = response.source_status(0);
  ASSERT_EQ(dvl.source_id(), "dvl");
  EXPECT_EQ(dvl.last_reject_reason(), pb::REJECT_REASON_DUPLICATE_TIMESTAMP);
  EXPECT_TRUE(SameTime(dvl.last_accept_source_time(), Ts(10)));
}

TEST(FakeEstimatorServiceTest, DelayedButInOrderIsAccepted) {
  FakeEstimatorService service = Ready(Config(2));
  service.Predict(Ts(100));
  EXPECT_TRUE(service
                  .Ingest(Envelope("dvl", pb::MEASUREMENT_KIND_DVL, Ts(10),
                                   Received(Ts(99))))
                  .ok());
  const auto response = service.GetState();
  EXPECT_EQ(response.state().header().source_time().seconds(), 100);
  EXPECT_TRUE(
      SameTime(response.source_status(0).last_accept_source_time(), Ts(10)));
}

TEST(FakeEstimatorServiceTest, InvalidEnvelopeAndPayloadAreRejected) {
  FakeEstimatorService service = Ready(Config(2));
  const int kind = pb::MEASUREMENT_KIND_DVL;
  EXPECT_EQ(service.Ingest(Envelope("", kind, Ts(1))).reason(),
            pb::REJECT_REASON_INVALID_ENVELOPE);
  EXPECT_EQ(
      service.Ingest(Envelope("a", pb::MEASUREMENT_KIND_UNSPECIFIED, Ts(1)))
          .reason(),
      pb::REJECT_REASON_INVALID_ENVELOPE);
  EXPECT_EQ(service.Ingest(Envelope("a", 99, Ts(1))).reason(),
            pb::REJECT_REASON_INVALID_ENVELOPE);
  MeasurementEnvelope no_time = Envelope("a", kind, Ts(1));
  no_time.mutable_header()->clear_receive_time();
  EXPECT_EQ(service.Ingest(no_time).reason(),
            pb::REJECT_REASON_INVALID_ENVELOPE);
  MeasurementEnvelope untyped = Envelope("a", kind, Ts(1));
  untyped.clear_payload_type();
  EXPECT_EQ(service.Ingest(untyped).reason(),
            pb::REJECT_REASON_INVALID_ENVELOPE);
  EnvelopeOptions utc;
  utc.clock_domain = "utc";
  EXPECT_EQ(service.Ingest(Envelope("a", kind, Ts(1), utc)).reason(),
            pb::REJECT_REASON_CLOCK_DOMAIN_MISMATCH);
  for (std::optional<Validity::State> validity :
       {std::optional<Validity::State>(),
        std::optional(Validity::STATE_INVALID),
        std::optional(Validity::STATE_UNSPECIFIED)}) {
    EnvelopeOptions options;
    options.validity = validity;
    EXPECT_EQ(service.Ingest(Envelope("a", kind, Ts(1), options)).reason(),
              pb::REJECT_REASON_INVALID_PAYLOAD);
  }
  service.SetPayloadValidator([](const MeasurementEnvelope& envelope) {
    return envelope.payload().size() > 4;
  });
  EXPECT_EQ(service.Ingest(Envelope("a", kind, Ts(1))).reason(),
            pb::REJECT_REASON_INVALID_PAYLOAD);
  service.SetPayloadValidator(nullptr);
  EXPECT_TRUE(service.Ingest(Envelope("a", kind, Ts(1))).ok());
}

TEST(FakeEstimatorServiceTest, SourceBecomesUnhealthyAfterConsecutiveRejects) {
  EstimatorConfig config = Config(2);
  config.set_unhealthy_after_consecutive_rejects(2);
  FakeEstimatorService service = Ready(config);
  const int kind = pb::MEASUREMENT_KIND_DVL;
  service.Ingest(Envelope("dvl", kind, Ts(10)));
  service.Ingest(Envelope("dvl", kind, Ts(9)));
  EXPECT_TRUE(service.GetState().source_status(0).healthy());
  EXPECT_EQ(service.GetState().state().sources(0).validity().state(),
            Validity::STATE_VALID);
  service.Ingest(Envelope("dvl", kind, Ts(8)));
  auto status = service.GetState().source_status(0);
  EXPECT_FALSE(status.healthy());
  EXPECT_EQ(status.consecutive_rejects(), 2u);
  EXPECT_EQ(service.GetState().state().sources(0).validity().state(),
            Validity::STATE_INVALID);
  EXPECT_TRUE(service.Ingest(Envelope("dvl", kind, Ts(11))).ok());
  status = service.GetState().source_status(0);
  EXPECT_TRUE(status.healthy());
  EXPECT_EQ(status.consecutive_rejects(), 0u);
  EXPECT_EQ(status.last_reject_reason(), pb::REJECT_REASON_OUT_OF_ORDER);
}

TEST(FakeEstimatorServiceTest, DefaultUnhealthyThresholdIsThree) {
  FakeEstimatorService service = Ready(Config(2));
  const int kind = pb::MEASUREMENT_KIND_DVL;
  service.Ingest(Envelope("dvl", kind, Ts(10)));
  service.Ingest(Envelope("dvl", kind, Ts(1)));
  service.Ingest(Envelope("dvl", kind, Ts(1)));
  EXPECT_TRUE(service.GetState().source_status(0).healthy());
  service.Ingest(Envelope("dvl", kind, Ts(1)));
  EXPECT_FALSE(service.GetState().source_status(0).healthy());
}

TEST(FakeEstimatorServiceTest, SourceFaultInjection) {
  FakeEstimatorService service = Ready(Config(2));
  const int kind = pb::MEASUREMENT_KIND_DVL;
  service.Ingest(Envelope("dvl", kind, Ts(10)));
  EXPECT_FALSE(service.InjectSourceFault("missing"));
  EXPECT_TRUE(service.InjectSourceFault("dvl"));
  EXPECT_EQ(service.GetState().state().sources(0).validity().state(),
            Validity::STATE_INVALID);
  service.Ingest(Envelope("dvl", kind, Ts(11)));
  EXPECT_EQ(service.GetState().state().sources(0).validity().state(),
            Validity::STATE_VALID);
}

TEST(FakeEstimatorServiceTest, FaultInjectionForcesFaultedUntilResetOrInit) {
  FakeEstimatorService service = Ready(Config(2));
  ASSERT_TRUE(service.InjectFault());
  EXPECT_EQ(service.GetState().state().mode(),
            intrinsic_proto::vehicle::NAVIGATION_MODE_FAULTED);
  EXPECT_EQ(
      service.Ingest(Envelope("dvl", pb::MEASUREMENT_KIND_DVL, Ts(1))).reason(),
      pb::REJECT_REASON_ESTIMATOR_FAULTED);
  EXPECT_TRUE(service.Predict(Ts(5)).ok());
  VehicleState state = service.GetState().state();
  EXPECT_EQ(state.mode(), intrinsic_proto::vehicle::NAVIGATION_MODE_FAULTED);
  EXPECT_EQ(state.header().source_time().seconds(), 5);
  service.Reset(std::nullopt);
  EXPECT_EQ(service.GetState().state().mode(),
            intrinsic_proto::vehicle::NAVIGATION_MODE_INITIALIZING);
  service.InjectFault();
  service.Initialize(Config(2));
  EXPECT_EQ(service.GetState().state().mode(),
            intrinsic_proto::vehicle::NAVIGATION_MODE_INITIALIZING);
}

TEST(FakeEstimatorServiceTest, NavigationModeWireValuesAreThe17Values) {
  EXPECT_EQ(intrinsic_proto::vehicle::NAVIGATION_MODE_UNSPECIFIED, 0);
  EXPECT_EQ(intrinsic_proto::vehicle::NAVIGATION_MODE_INITIALIZING, 1);
  EXPECT_EQ(intrinsic_proto::vehicle::NAVIGATION_MODE_DEAD_RECKONING, 2);
  EXPECT_EQ(intrinsic_proto::vehicle::NAVIGATION_MODE_AIDED, 3);
  EXPECT_EQ(intrinsic_proto::vehicle::NAVIGATION_MODE_FAULTED, 4);
  FakeEstimatorService service = Ready(Config(2));
  for (int step = 1; step <= 3; ++step) {
    service.Predict(Ts(step));
    EXPECT_NE(
        vehicle::ClassifyNavigationMode(service.GetState().state().mode()),
        vehicle::NavigationModeKind::kUnknown);
  }
}

TEST(FakeEstimatorServiceTest, EpochIsMonotonicAcrossInitAndReset) {
  FakeEstimatorService service;
  EXPECT_EQ(service.Initialize(Config()).estimator_epoch(), 1u);
  EXPECT_EQ(service.Reset(std::nullopt).estimator_epoch(), 2u);
  EXPECT_EQ(service.Initialize(Config()).estimator_epoch(), 3u);
  EXPECT_EQ(service.Reset(std::nullopt).estimator_epoch(), 4u);
  EXPECT_EQ(service.GetState().state().estimator_epoch(), 4u);
}

TEST(FakeEstimatorServiceTest, ReplayDigestsRepeatForAFixedSeed) {
  FakeEstimatorService first;
  FakeEstimatorService second;
  RunNominal(&first);
  RunNominal(&second);
  EXPECT_EQ(first.StateDigest(), second.StateDigest());
  EXPECT_NE(first.StateDigest(), 0u);
  FakeEstimatorService other;
  other.Initialize(Config(30, true));
  other.Predict(Ts(1700000001));
  EXPECT_NE(first.StateDigest(), other.StateDigest());
}

TEST(FakeEstimatorServiceTest, NominalSnapshotMatchesTheGolden) {
  FakeEstimatorService service;
  RunNominal(&service);
  const std::string text = LoadGolden("nominal_vehicle_state.textproto");
  ASSERT_FALSE(text.empty());
  VehicleState expected;
  ASSERT_TRUE(google::protobuf::TextFormat::ParseFromString(text, &expected));
  const VehicleState actual = service.GetState().state();
  EXPECT_EQ(Bytes(actual), Bytes(expected));
  EXPECT_EQ(service.StateDigest(), kNominalDigest);
  EXPECT_EQ(actual.mode(), intrinsic_proto::vehicle::NAVIGATION_MODE_AIDED);
  EXPECT_EQ(actual.estimator_epoch(), 1u);
  EXPECT_EQ(actual.sources_size(), 2);
  EXPECT_FALSE(actual.has_twist_covariance());
}

}  // namespace
}  // namespace intrinsic::estimation
