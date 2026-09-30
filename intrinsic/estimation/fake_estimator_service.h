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

#ifndef INTRINSIC_ESTIMATION_FAKE_ESTIMATOR_SERVICE_H_
#define INTRINSIC_ESTIMATION_FAKE_ESTIMATOR_SERVICE_H_

#include <cstdint>
#include <functional>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

#include "google/protobuf/timestamp.pb.h"
#include "intrinsic/estimation/estimator.pb.h"
#include "intrinsic/estimation/estimator_service.h"
#include "intrinsic/vehicle/proto/vehicle_state.pb.h"

namespace intrinsic::estimation {

// Deterministic test double. The initial pose and twist come from
// EstimatorConfig.seed. Predict holds the kinematics and covariance and only
// advances the stamp. No process noise, no error state, no integration.
class FakeEstimatorService : public EstimatorService {
 public:
  // Returns true when the envelope payload is usable. The default accepts
  // every payload.
  using PayloadValidator = std::function<bool(
      const intrinsic_proto::estimation::MeasurementEnvelope&)>;

  FakeEstimatorService() = default;

  intrinsic_proto::estimation::EstimatorResult Initialize(
      const intrinsic_proto::estimation::EstimatorConfig& config) override;
  intrinsic_proto::estimation::EstimatorResult Ingest(
      const intrinsic_proto::estimation::MeasurementEnvelope& envelope)
      override;
  intrinsic_proto::estimation::EstimatorResult Predict(
      const google::protobuf::Timestamp& to_time) override;
  intrinsic_proto::estimation::EstimatorResult Reset(
      const std::optional<intrinsic_proto::vehicle::VehicleState>& prior)
      override;
  intrinsic_proto::estimation::GetStateResponse GetState() const override;

  void SetPayloadValidator(PayloadValidator validator);

  // Forces FAULTED until the next Initialize or Reset. Returns false when the
  // service was never initialized.
  bool InjectFault();

  // Marks one known source unhealthy until its next accept. Returns false for
  // an unknown source.
  bool InjectSourceFault(std::string_view source_id);

  // FNV-1a 64 of the deterministic serialization of GetState().state. Zero
  // when the state is unset.
  uint64_t StateDigest() const;

 private:
  struct Source {
    intrinsic_proto::estimation::SourceStatus status;
    __int128 last_accept_ns = 0;
    bool accepted_once = false;
  };

  intrinsic_proto::estimation::EstimatorResult Result(
      intrinsic_proto::estimation::RejectReason reason) const;
  intrinsic_proto::estimation::EstimatorResult Reject(
      Source* source, intrinsic_proto::estimation::RejectReason reason);
  Source* FindOrAddSource(const std::string& source_id);
  void ResetBuffers();

  bool initialized_ = false;
  bool faulted_ = false;
  bool aided_since_predict_ = false;
  uint64_t epoch_ = 0;
  uint64_t sequence_ = 0;
  intrinsic_proto::estimation::EstimatorConfig config_;
  __int128 max_future_skew_ns_ = 0;
  intrinsic_proto::vehicle::VehicleState state_;
  std::vector<Source> sources_;
  std::optional<__int128> last_predict_ns_;
  PayloadValidator validator_;
};

}  // namespace intrinsic::estimation

#endif  // INTRINSIC_ESTIMATION_FAKE_ESTIMATOR_SERVICE_H_
