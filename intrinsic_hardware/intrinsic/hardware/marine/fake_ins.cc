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

#include "intrinsic/hardware/marine/fake_ins.h"

#include "intrinsic/embodiment/proto/stamped_header.pb.h"
#include "intrinsic/embodiment/stamped_header_policy.h"
#include "intrinsic/hardware/marine/ins_policy.h"
#include "intrinsic/hardware/marine/measurement_health.pb.h"
#include "intrinsic/vehicle/proto/vehicle_state.pb.h"
#include "intrinsic/vehicle/vehicle_contract_policy.h"

namespace intrinsic::hardware::marine {
namespace {

using intrinsic_proto::embodiment::Validity;
using intrinsic_proto::hardware::marine::InsSolution;
using intrinsic_proto::hardware::marine::MeasurementHealth;

bool PositionBiasEngaged(const FakeInsConfig& config) {
  return config.position_bias_x_m != 0.0 || config.position_bias_y_m != 0.0 ||
         config.position_bias_z_m != 0.0;
}

bool LinearBiasEngaged(const FakeInsConfig& config) {
  return config.linear_velocity_bias_x_m_s != 0.0 ||
         config.linear_velocity_bias_y_m_s != 0.0 ||
         config.linear_velocity_bias_z_m_s != 0.0;
}

bool AngularBiasEngaged(const FakeInsConfig& config) {
  return config.angular_velocity_bias_x_rad_s != 0.0 ||
         config.angular_velocity_bias_y_rad_s != 0.0 ||
         config.angular_velocity_bias_z_rad_s != 0.0;
}

bool Degraded(const FakeInsConfig& config, const InsTruth& truth) {
  if (PositionBiasEngaged(config) || config.position_drift_m_per_seed != 0.0 ||
      config.position_noise_amplitude_m != 0.0) {
    return true;
  }
  if (truth.linear_velocity_present &&
      (LinearBiasEngaged(config) ||
       config.linear_velocity_noise_amplitude_m_s != 0.0)) {
    return true;
  }
  if (truth.angular_velocity_present &&
      (AngularBiasEngaged(config) ||
       config.angular_velocity_noise_amplitude_rad_s != 0.0)) {
    return true;
  }
  return false;
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

void FillInsCovariance(intrinsic_proto::vehicle::Matrix6* matrix) {
  matrix->mutable_values()->Resize(vehicle::kCovarianceValues, 0.0);
  matrix->set_values(kPositionXVarianceSlot, 1.0);
  matrix->set_values(kPositionYVarianceSlot, 4.0);
  matrix->set_values(kPositionZVarianceSlot, 0.25);
  matrix->set_values(kAttitudeXVarianceSlot, 0.0625);
  matrix->set_values(kAttitudeYVarianceSlot, 0.125);
  matrix->set_values(kAttitudeZVarianceSlot, 0.5);
}

void SetOrientation(InsSolution* sample, double x, double y, double z,
                    double w) {
  InsSolution::QuaternionXyzw* orientation = sample->mutable_orientation_xyzw();
  orientation->set_x(x);
  orientation->set_y(y);
  orientation->set_z(z);
  orientation->set_w(w);
}

}  // namespace

std::optional<InsSolution> FakeIns::Measure(const InsTruth& truth) const {
  if (config_.dropout) {
    return std::nullopt;
  }
  InsSolution sample;
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

  if (config_.invalid_orientation) {
    health->set_state(MeasurementHealth::INVALID);
  } else if (Degraded(config_, truth)) {
    health->set_state(MeasurementHealth::DEGRADED);
  } else {
    health->set_state(MeasurementHealth::VALID);
  }
  health->set_quality(static_cast<float>(truth.quality));
  if (truth.covariance_present) {
    FillInsCovariance(health->mutable_covariance());
  }
  auto* primary = health->add_sources();
  primary->set_source_id("primary");
  primary->mutable_validity()->set_state(Validity::STATE_VALID);
  health->add_sources()->set_source_id("aiding");

  double position_x = truth.position_x_m;
  double position_y = truth.position_y_m;
  double position_z = truth.position_z_m;
  double linear_x = truth.linear_velocity_x_m_s;
  double linear_y = truth.linear_velocity_y_m_s;
  double linear_z = truth.linear_velocity_z_m_s;
  double angular_x = truth.angular_velocity_x_rad_s;
  double angular_y = truth.angular_velocity_y_rad_s;
  double angular_z = truth.angular_velocity_z_rad_s;
  if (!config_.invalid_orientation) {
    const double seed = static_cast<double>(config_.seed);
    position_x += config_.position_bias_x_m;
    position_y += config_.position_bias_y_m;
    position_z += config_.position_bias_z_m;
    position_x += config_.position_drift_m_per_seed * seed;
    position_y += config_.position_drift_m_per_seed * seed;
    position_z += config_.position_drift_m_per_seed * seed;
    if (config_.position_noise_amplitude_m != 0.0) {
      const double unit = InsSignedUnitNoise(config_.seed);
      position_x += config_.position_noise_amplitude_m * unit;
      position_y += config_.position_noise_amplitude_m * unit;
      position_z += config_.position_noise_amplitude_m * unit;
    }
    if (truth.linear_velocity_present) {
      linear_x += config_.linear_velocity_bias_x_m_s;
      linear_y += config_.linear_velocity_bias_y_m_s;
      linear_z += config_.linear_velocity_bias_z_m_s;
      if (config_.linear_velocity_noise_amplitude_m_s != 0.0) {
        const double unit = InsSignedUnitNoise(config_.seed + 1);
        linear_x += config_.linear_velocity_noise_amplitude_m_s * unit;
        linear_y += config_.linear_velocity_noise_amplitude_m_s * unit;
        linear_z += config_.linear_velocity_noise_amplitude_m_s * unit;
      }
    }
    if (truth.angular_velocity_present) {
      angular_x += config_.angular_velocity_bias_x_rad_s;
      angular_y += config_.angular_velocity_bias_y_rad_s;
      angular_z += config_.angular_velocity_bias_z_rad_s;
      if (config_.angular_velocity_noise_amplitude_rad_s != 0.0) {
        const double unit = InsSignedUnitNoise(config_.seed + 2);
        angular_x += config_.angular_velocity_noise_amplitude_rad_s * unit;
        angular_y += config_.angular_velocity_noise_amplitude_rad_s * unit;
        angular_z += config_.angular_velocity_noise_amplitude_rad_s * unit;
      }
    }
  }
  sample.set_position_x_m(position_x);
  sample.set_position_y_m(position_y);
  sample.set_position_z_m(position_z);
  if (config_.invalid_orientation) {
    SetOrientation(&sample, kInsCanonicalNonUnitQuaternionX,
                   kInsCanonicalNonUnitQuaternionY,
                   kInsCanonicalNonUnitQuaternionZ,
                   kInsCanonicalNonUnitQuaternionW);
  } else {
    SetOrientation(&sample, truth.orientation_x, truth.orientation_y,
                   truth.orientation_z, truth.orientation_w);
  }
  if (truth.linear_velocity_present) {
    sample.set_linear_velocity_x_m_s(linear_x);
    sample.set_linear_velocity_y_m_s(linear_y);
    sample.set_linear_velocity_z_m_s(linear_z);
  }
  if (truth.angular_velocity_present) {
    sample.set_angular_velocity_x_rad_s(angular_x);
    sample.set_angular_velocity_y_rad_s(angular_y);
    sample.set_angular_velocity_z_rad_s(angular_z);
  }
  if (truth.source_present) {
    sample.set_source(static_cast<InsSolution::SourceKind>(truth.source));
  }
  return sample;
}

}  // namespace intrinsic::hardware::marine
