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

#include "intrinsic/hardware/marine/fake_surface_fix.h"

#include <cstdint>
#include <optional>
#include <string>

#include "intrinsic/embodiment/proto/stamped_header.pb.h"
#include "intrinsic/embodiment/stamped_header_policy.h"
#include "intrinsic/hardware/marine/measurement_health.pb.h"
#include "intrinsic/hardware/marine/surface_fix.pb.h"
#include "intrinsic/hardware/marine/surface_fix_policy.h"
#include "intrinsic/vehicle/proto/vehicle_state.pb.h"
#include "intrinsic/vehicle/vehicle_contract_policy.h"

namespace intrinsic::hardware::marine {
namespace {

using intrinsic_proto::embodiment::Validity;
using intrinsic_proto::hardware::marine::MeasurementHealth;
using intrinsic_proto::hardware::marine::SurfaceFix;

bool BiasEngaged(const FakeSurfaceFixConfig& config) {
  return config.position_bias_x_m != 0.0 || config.position_bias_y_m != 0.0 ||
         config.position_bias_z_m != 0.0;
}

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

void FillSurfaceFixCovariance(intrinsic_proto::vehicle::Matrix6* matrix) {
  matrix->mutable_values()->Resize(vehicle::kCovarianceValues, 0.0);
  matrix->set_values(kSurfacePositionXVarianceSlot, 1.0);
  matrix->set_values(kSurfacePositionYVarianceSlot, 4.0);
  matrix->set_values(kSurfacePositionZVarianceSlot, 0.25);
}

}  // namespace

std::optional<SurfaceFix> FakeSurfaceFix::Measure(
    const SurfaceFixTruth& truth) const {
  if (config_.dropout) {
    return std::nullopt;
  }
  SurfaceFix sample;
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

  if (config_.invalid_fix) {
    health->set_state(MeasurementHealth::INVALID);
  } else if (BiasEngaged(config_)) {
    health->set_state(MeasurementHealth::DEGRADED);
  } else {
    health->set_state(MeasurementHealth::VALID);
  }
  health->set_quality(static_cast<float>(truth.quality));
  if (truth.covariance_present) {
    FillSurfaceFixCovariance(health->mutable_covariance());
  }
  auto* primary = health->add_sources();
  primary->set_source_id("primary");
  primary->mutable_validity()->set_state(Validity::STATE_VALID);
  health->add_sources()->set_source_id("aiding");

  double position_x = truth.position_x_m;
  double position_y = truth.position_y_m;
  double position_z = truth.position_z_m;
  if (!config_.invalid_fix) {
    position_x += config_.position_bias_x_m;
    position_y += config_.position_bias_y_m;
    position_z += config_.position_bias_z_m;
  }
  sample.set_position_x_m(position_x);
  sample.set_position_y_m(position_y);
  sample.set_position_z_m(position_z);
  if (config_.invalid_fix) {
    sample.set_source(SurfaceFix::FIX_SOURCE_UNSPECIFIED);
  } else if (truth.source_present) {
    sample.set_source(static_cast<SurfaceFix::FixSource>(truth.source));
  }
  if (truth.satellite_count_present) {
    sample.set_satellite_count(truth.satellite_count);
  }
  if (truth.beacon_count_present) {
    sample.set_beacon_count(truth.beacon_count);
  }
  if (truth.horizontal_accuracy_present) {
    sample.set_horizontal_accuracy_m(truth.horizontal_accuracy_m);
  }
  if (truth.vertical_accuracy_present) {
    sample.set_vertical_accuracy_m(truth.vertical_accuracy_m);
  }
  if (truth.velocity_present) {
    sample.set_velocity_x_m_s(truth.velocity_x_m_s);
    sample.set_velocity_y_m_s(truth.velocity_y_m_s);
    sample.set_velocity_z_m_s(truth.velocity_z_m_s);
  }
  return sample;
}

}  // namespace intrinsic::hardware::marine
