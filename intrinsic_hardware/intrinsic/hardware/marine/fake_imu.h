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

#ifndef INTRINSIC_HARDWARE_MARINE_FAKE_IMU_H_
#define INTRINSIC_HARDWARE_MARINE_FAKE_IMU_H_

#include <cstdint>
#include <optional>
#include <string>

#include "intrinsic/hardware/marine/imu.pb.h"

namespace intrinsic::hardware::marine {

// Deterministic raw-IMU producer. Same config and truth yield the same
// bytes. `seed` is written to `health.header.sequence` and mixes the noise
// draw. There is no filter, no magnetometer, and no INS coupling.
//
// Dropout omits the sample and is checked first. Invalid orientation
// replaces the quaternion with the canonical non-unit fixture and sets
// health state INVALID. It wins over bias, drift, and noise: those
// additions are not applied. Delay is still applied.
//
// Otherwise a non-zero bias component, drift amplitude, or noise amplitude
// sets health state DEGRADED. Bias is added per component. Drift adds
// `amplitude * seed` to every component of that triple. Noise adds
// `amplitude * ImuSignedUnitNoise(seed)` to angular velocity and
// `amplitude * ImuSignedUnitNoise(seed + 1)` to linear acceleration.
// Results are not repaired. Delay is added to source time to form receive
// time and is not repaired when it reverses the stamps.

// Canonical non-unit quaternion. Norm is 2, so |‖q‖ − 1| is 1.
inline constexpr double kImuCanonicalNonUnitQuaternionX = 0.0;
inline constexpr double kImuCanonicalNonUnitQuaternionY = 0.0;
inline constexpr double kImuCanonicalNonUnitQuaternionZ = 0.0;
inline constexpr double kImuCanonicalNonUnitQuaternionW = 2.0;

// SplitMix64 of `seed`, mapped onto [-1, 1). The 53-bit fraction is exact
// in binary64. The Python fake uses the same mix.
inline double ImuSignedUnitNoise(uint64_t seed) {
  uint64_t mixed = seed + 0x9E3779B97F4A7C15ULL;
  mixed = (mixed ^ (mixed >> 30)) * 0xBF58476D1CE4E5B9ULL;
  mixed = (mixed ^ (mixed >> 27)) * 0x94D049BB133111EBULL;
  mixed ^= mixed >> 31;
  const double unit = static_cast<double>(mixed >> 11) * 0x1p-53;
  return unit * 2.0 - 1.0;
}

struct FakeImuConfig {
  uint64_t seed = 42;
  double angular_velocity_bias_x_rad_s = 0;
  double angular_velocity_bias_y_rad_s = 0;
  double angular_velocity_bias_z_rad_s = 0;
  double linear_acceleration_bias_x_m_s2 = 0;
  double linear_acceleration_bias_y_m_s2 = 0;
  double linear_acceleration_bias_z_m_s2 = 0;
  // Added as amplitude * seed on every component of that triple.
  double angular_drift_rad_s_per_seed = 0;
  double linear_drift_m_s2_per_seed = 0;
  // Exactly 0 disables that draw. Any other value, including a non-finite
  // amplitude, engages noise.
  double angular_noise_amplitude_rad_s = 0;
  double linear_noise_amplitude_m_s2 = 0;
  // receive_time = source_time + delay. Nanos may be negative.
  int64_t delay_seconds = 1;
  int32_t delay_nanos = -250000000;
  bool dropout = false;
  bool invalid_orientation = false;
};

struct ImuTruth {
  std::string frame_id = "imu";
  int64_t source_seconds = 1700000000;
  int32_t source_nanos = 250000000;
  double angular_velocity_x_rad_s = 0.25;
  double angular_velocity_y_rad_s = -0.5;
  double angular_velocity_z_rad_s = 0.125;
  double linear_acceleration_x_m_s2 = 0;
  double linear_acceleration_y_m_s2 = 0;
  double linear_acceleration_z_m_s2 = 8;
  bool orientation_present = true;
  double orientation_x = 0;
  double orientation_y = 0;
  double orientation_z = 0;
  double orientation_w = 1;
  double quality = 0.75;
  bool covariance_present = true;
};

class FakeImu {
 public:
  explicit FakeImu(FakeImuConfig config = {}) : config_(config) {}

  const FakeImuConfig& config() const { return config_; }

  // Absent when dropout is set. Otherwise one ImuMeasurement.
  std::optional<intrinsic_proto::hardware::marine::ImuMeasurement> Measure(
      const ImuTruth& truth = ImuTruth()) const;

 private:
  FakeImuConfig config_;
};

}  // namespace intrinsic::hardware::marine

#endif  // INTRINSIC_HARDWARE_MARINE_FAKE_IMU_H_
