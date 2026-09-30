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

#ifndef INTRINSIC_ESTIMATION_ESTIMATOR_SERVICE_H_
#define INTRINSIC_ESTIMATION_ESTIMATOR_SERVICE_H_

#include <optional>

#include "google/protobuf/timestamp.pb.h"
#include "intrinsic/estimation/estimator.pb.h"
#include "intrinsic/vehicle/proto/vehicle_state.pb.h"

namespace intrinsic::estimation {

// Policy: intrinsic/estimation/README.md.
//
// Persistent estimation service that publishes the #17 VehicleState. This
// interface has no filter equations and no ICON or safety wiring.
class EstimatorService {
 public:
  virtual ~EstimatorService() = default;

  // Loads config, resets every buffer, bumps the epoch, and sets mode
  // INITIALIZING. Calling it again is a full reset. An invalid config
  // changes nothing.
  virtual intrinsic_proto::estimation::EstimatorResult Initialize(
      const intrinsic_proto::estimation::EstimatorConfig& config) = 0;

  // A reject changes no published kinematics, mode, or epoch. It only updates
  // the source bookkeeping of a named source.
  virtual intrinsic_proto::estimation::EstimatorResult Ingest(
      const intrinsic_proto::estimation::MeasurementEnvelope& envelope) = 0;

  // Advances the snapshot clock to `to_time` and applies the mode rule in the
  // README. Time never moves backward.
  virtual intrinsic_proto::estimation::EstimatorResult Predict(
      const google::protobuf::Timestamp& to_time) = 0;

  // Clears buffers and bumps the epoch. A prior that passes
  // AssessVehicleState becomes the next body. Otherwise the published body is
  // empty. Either way the mode is INITIALIZING.
  virtual intrinsic_proto::estimation::EstimatorResult Reset(
      const std::optional<intrinsic_proto::vehicle::VehicleState>& prior) = 0;

  // `state` is unset when the service was never initialized.
  virtual intrinsic_proto::estimation::GetStateResponse GetState() const = 0;
};

}  // namespace intrinsic::estimation

#endif  // INTRINSIC_ESTIMATION_ESTIMATOR_SERVICE_H_
