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

#include "intrinsic/hardware/marine/fake_imu.h"

#include "intrinsic/embodiment/proto/stamped_header.pb.h"
#include "intrinsic/embodiment/stamped_header_policy.h"
#include "intrinsic/hardware/marine/imu_policy.h"
#include "intrinsic/hardware/marine/measurement_health.pb.h"
#include "intrinsic/vehicle/proto/vehicle_state.pb.h"
#include "intrinsic/vehicle/vehicle_contract_policy.h"

namespace intrinsic::hardware::marine {
namespace {

using intrinsic_proto::embodiment::Validity;
using intrinsic_proto::hardware::marine::ImuMeasurement;
using intrinsic_proto::hardware::marine::MeasurementHealth;

bool BiasEngaged(const FakeImuConfig& config) {
  return config.angular_velocity_bias_x_rad_s != 0.0 ||
         config.angular_velocity_bias_y_rad_s != 0.0 ||
         config.angular_velocity_bias_z_rad_s != 0.0 ||
         config.linear_acceleration_bias_x_m_s2 != 0.0 ||
         config.linear_acceleration_bias_y_m_s2 != 0.0 ||
         config.linear_acceleration_bias_z_m_s2 != 0.0;
}

bool DriftEngaged(const FakeImuConfig& config) {
  return config.angular_drift_rad_s_per_seed != 0.0 ||
         config.linear_drift_m_s2_per_seed != 0.0;
}

bool NoiseEngaged(const FakeImuConfig& config) {
  return config.angular_noise_amplitude_rad_s != 0.0 ||
         config.linear_noise_amplitude_m_s2 != 0.0;
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

void FillImuCovariance(intrinsic_proto::vehicle::Matrix6* matrix) {
  matrix->mutable_values()->Resize(vehicle::kCovarianceValues, 0.0);
  matrix->set_values(kAngularVelocityXVarianceSlot, 0.25);
  matrix->set_values(kAngularVelocityYVarianceSlot, 0.5);
  matrix->set_values(kAngularVelocityZVarianceSlot, 0.125);
  matrix->set_values(kLinearAccelerationXVarianceSlot, 1.0);
  matrix->set_values(kLinearAccelerationYVarianceSlot, 2.0);
  matrix->set_values(kLinearAccelerationZVarianceSlot, 4.0);
}

void SetOrientation(ImuMeasurement* sample, double x, double y, double z,
                    double w) {
  ImuMeasurement::QuaternionXyzw* orientation =
      sample->mutable_orientation_xyzw();
  orientation->set_x(x);
  orientation->set_y(y);
  orientation->set_z(z);
  orientation->set_w(w);
}

}  // namespace

std::optional<ImuMeasurement> FakeImu::Measure(const ImuTruth& truth) const {
  if (config_.dropout) {
    return std::nullopt;
  }
  ImuMeasurement sample;
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

  const bool degraded =
      BiasEngaged(config_) || DriftEngaged(config_) || NoiseEngaged(config_);
  if (config_.invalid_orientation) {
    health->set_state(MeasurementHealth::INVALID);
  } else if (degraded) {
    health->set_state(MeasurementHealth::DEGRADED);
  } else {
    health->set_state(MeasurementHealth::VALID);
  }
  health->set_quality(static_cast<float>(truth.quality));
  if (truth.covariance_present) {
    FillImuCovariance(health->mutable_covariance());
  }
  auto* primary = health->add_sources();
  primary->set_source_id("primary");
  primary->mutable_validity()->set_state(Validity::STATE_VALID);
  health->add_sources()->set_source_id("aiding");

  double angular_x = truth.angular_velocity_x_rad_s;
  double angular_y = truth.angular_velocity_y_rad_s;
  double angular_z = truth.angular_velocity_z_rad_s;
  double linear_x = truth.linear_acceleration_x_m_s2;
  double linear_y = truth.linear_acceleration_y_m_s2;
  double linear_z = truth.linear_acceleration_z_m_s2;
  if (!config_.invalid_orientation) {
    const double seed = static_cast<double>(config_.seed);
    angular_x += config_.angular_velocity_bias_x_rad_s;
    angular_y += config_.angular_velocity_bias_y_rad_s;
    angular_z += config_.angular_velocity_bias_z_rad_s;
    linear_x += config_.linear_acceleration_bias_x_m_s2;
    linear_y += config_.linear_acceleration_bias_y_m_s2;
    linear_z += config_.linear_acceleration_bias_z_m_s2;
    angular_x += config_.angular_drift_rad_s_per_seed * seed;
    angular_y += config_.angular_drift_rad_s_per_seed * seed;
    angular_z += config_.angular_drift_rad_s_per_seed * seed;
    linear_x += config_.linear_drift_m_s2_per_seed * seed;
    linear_y += config_.linear_drift_m_s2_per_seed * seed;
    linear_z += config_.linear_drift_m_s2_per_seed * seed;
    if (config_.angular_noise_amplitude_rad_s != 0.0) {
      const double unit = ImuSignedUnitNoise(config_.seed);
      angular_x += config_.angular_noise_amplitude_rad_s * unit;
      angular_y += config_.angular_noise_amplitude_rad_s * unit;
      angular_z += config_.angular_noise_amplitude_rad_s * unit;
    }
    if (config_.linear_noise_amplitude_m_s2 != 0.0) {
      const double unit = ImuSignedUnitNoise(config_.seed + 1);
      linear_x += config_.linear_noise_amplitude_m_s2 * unit;
      linear_y += config_.linear_noise_amplitude_m_s2 * unit;
      linear_z += config_.linear_noise_amplitude_m_s2 * unit;
    }
  }
  sample.set_angular_velocity_x_rad_s(angular_x);
  sample.set_angular_velocity_y_rad_s(angular_y);
  sample.set_angular_velocity_z_rad_s(angular_z);
  sample.set_linear_acceleration_x_m_s2(linear_x);
  sample.set_linear_acceleration_y_m_s2(linear_y);
  sample.set_linear_acceleration_z_m_s2(linear_z);
  if (config_.invalid_orientation) {
    SetOrientation(&sample, kImuCanonicalNonUnitQuaternionX,
                   kImuCanonicalNonUnitQuaternionY,
                   kImuCanonicalNonUnitQuaternionZ,
                   kImuCanonicalNonUnitQuaternionW);
  } else if (truth.orientation_present) {
    SetOrientation(&sample, truth.orientation_x, truth.orientation_y,
                   truth.orientation_z, truth.orientation_w);
  }
  return sample;
}

}  // namespace intrinsic::hardware::marine
