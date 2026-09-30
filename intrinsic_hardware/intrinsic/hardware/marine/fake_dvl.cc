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

#include "intrinsic/hardware/marine/fake_dvl.h"

#include "intrinsic/embodiment/proto/stamped_header.pb.h"
#include "intrinsic/embodiment/stamped_header_policy.h"
#include "intrinsic/hardware/marine/measurement_health.pb.h"
#include "intrinsic/vehicle/proto/vehicle_state.pb.h"
#include "intrinsic/vehicle/vehicle_contract_policy.h"

namespace intrinsic::hardware::marine {
namespace {

using intrinsic_proto::embodiment::Validity;
using intrinsic_proto::hardware::marine::DvlMeasurement;
using intrinsic_proto::hardware::marine::MeasurementHealth;

bool BiasEngaged(const FakeDvlConfig& config) {
  return config.bias_x_m_s != 0.0 || config.bias_y_m_s != 0.0 ||
         config.bias_z_m_s != 0.0;
}

void FillLinearCovariance(intrinsic_proto::vehicle::Matrix6* matrix) {
  matrix->mutable_values()->Resize(vehicle::kCovarianceValues, 0.0);
  matrix->set_values(vehicle::CovarianceIndex(0, 0), 0.25);
  matrix->set_values(vehicle::CovarianceIndex(1, 1), 0.25);
  matrix->set_values(vehicle::CovarianceIndex(2, 2), 0.25);
}

}  // namespace

void AddClockDelay(int64_t* seconds, int32_t* nanos, int64_t delay_seconds,
                   int32_t delay_nanos) {
  constexpr int64_t kNanosPerSecond = 1000000000;
  int64_t total_nanos =
      static_cast<int64_t>(*nanos) + static_cast<int64_t>(delay_nanos);
  int64_t total_seconds = *seconds + delay_seconds;
  int64_t carry = total_nanos / kNanosPerSecond;
  total_nanos %= kNanosPerSecond;
  if (total_nanos < 0) {
    total_nanos += kNanosPerSecond;
    carry -= 1;
  }
  total_seconds += carry;
  *seconds = total_seconds;
  *nanos = static_cast<int32_t>(total_nanos);
}

std::optional<DvlMeasurement> FakeDvl::Measure(const DvlTruth& truth) const {
  if (config_.dropout) {
    return std::nullopt;
  }
  DvlMeasurement sample;
  MeasurementHealth* health = sample.mutable_health();
  intrinsic_proto::embodiment::StampedHeader* header = health->mutable_header();
  header->set_sequence(config_.seed);
  header->mutable_source_time()->set_seconds(truth.source_seconds);
  header->mutable_source_time()->set_nanos(truth.source_nanos);
  int64_t receive_seconds = truth.source_seconds;
  int32_t receive_nanos = truth.source_nanos;
  AddClockDelay(&receive_seconds, &receive_nanos, config_.delay_seconds,
                config_.delay_nanos);
  header->mutable_receive_time()->set_seconds(receive_seconds);
  header->mutable_receive_time()->set_nanos(receive_nanos);
  header->set_source_id("nav_sensor");
  header->set_frame_id(truth.frame_id);
  header->set_clock_domain(std::string(embodiment::kClockDomainMonotonic));
  header->mutable_validity()->set_state(Validity::STATE_VALID);

  if (config_.lock_loss) {
    health->set_state(MeasurementHealth::INVALID);
  } else if (BiasEngaged(config_)) {
    health->set_state(MeasurementHealth::DEGRADED);
  } else {
    health->set_state(MeasurementHealth::VALID);
  }
  health->set_quality(static_cast<float>(truth.quality));
  if (truth.covariance_present) {
    FillLinearCovariance(health->mutable_covariance());
  }
  auto* primary = health->add_sources();
  primary->set_source_id("primary");
  primary->mutable_validity()->set_state(Validity::STATE_VALID);
  health->add_sources()->set_source_id("aiding");

  const int mode = config_.lock_loss
                       ? static_cast<int>(DvlMeasurement::MODE_BOTTOM_TRACK)
                       : truth.mode;
  sample.set_mode(static_cast<DvlMeasurement::Mode>(mode));
  sample.set_velocity_x_m_s(truth.velocity_x_m_s + config_.bias_x_m_s);
  sample.set_velocity_y_m_s(truth.velocity_y_m_s + config_.bias_y_m_s);
  sample.set_velocity_z_m_s(truth.velocity_z_m_s + config_.bias_z_m_s);
  if (config_.lock_loss) {
    sample.set_bottom_lock(false);
  } else if (mode == DvlMeasurement::MODE_BOTTOM_TRACK) {
    sample.set_bottom_lock(true);
  }
  if (truth.altitude_present) {
    sample.set_altitude_m(truth.altitude_m);
  }
  return sample;
}

}  // namespace intrinsic::hardware::marine
