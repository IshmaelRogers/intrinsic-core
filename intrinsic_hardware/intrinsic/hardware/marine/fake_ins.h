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

#ifndef INTRINSIC_HARDWARE_MARINE_FAKE_INS_H_
#define INTRINSIC_HARDWARE_MARINE_FAKE_INS_H_

#include <cstdint>
#include <optional>
#include <string>

#include "intrinsic/hardware/marine/ins.pb.h"

namespace intrinsic::hardware::marine {

// Deterministic vendor-INS producer. Same config and truth yield the same
// bytes. `seed` is written to `health.header.sequence` and mixes the noise
// draw. This producer does not read FakeImu and does not propagate a filter.
//
// Dropout omits the sample and is checked first. Invalid orientation
// replaces the quaternion with the canonical non-unit fixture and sets
// health state INVALID. It wins over bias, drift, and noise: those
// additions are not applied. Delay is still applied.
//
// Otherwise health state is DEGRADED when any position bias component, any
// applicable rate bias, the position drift amplitude, or any applicable
// noise amplitude is non-zero. Position bias, drift, and noise always apply.
// Drift adds `amplitude * seed` to each position component. Rate bias and
// rate noise apply only when that twist triple is present on the truth.
// They do not invent an absent triple, and an inapplicable rate fault does
// not set DEGRADED. Delay is not repaired when it reverses the stamps.

// Canonical non-unit quaternion. Norm is 2, so |‖q‖ − 1| is 1.
inline constexpr double kInsCanonicalNonUnitQuaternionX = 0.0;
inline constexpr double kInsCanonicalNonUnitQuaternionY = 0.0;
inline constexpr double kInsCanonicalNonUnitQuaternionZ = 0.0;
inline constexpr double kInsCanonicalNonUnitQuaternionW = 2.0;

// SplitMix64 of `seed`, mapped onto [-1, 1). The 53-bit fraction is exact
// in binary64. The Python fake uses the same mix.
inline double InsSignedUnitNoise(uint64_t seed) {
  uint64_t mixed = seed + 0x9E3779B97F4A7C15ULL;
  mixed = (mixed ^ (mixed >> 30)) * 0xBF58476D1CE4E5B9ULL;
  mixed = (mixed ^ (mixed >> 27)) * 0x94D049BB133111EBULL;
  mixed ^= mixed >> 31;
  const double unit = static_cast<double>(mixed >> 11) * 0x1p-53;
  return unit * 2.0 - 1.0;
}

struct FakeInsConfig {
  uint64_t seed = 42;
  double position_bias_x_m = 0;
  double position_bias_y_m = 0;
  double position_bias_z_m = 0;
  double linear_velocity_bias_x_m_s = 0;
  double linear_velocity_bias_y_m_s = 0;
  double linear_velocity_bias_z_m_s = 0;
  double angular_velocity_bias_x_rad_s = 0;
  double angular_velocity_bias_y_rad_s = 0;
  double angular_velocity_bias_z_rad_s = 0;
  // Added as amplitude * seed on every position component.
  double position_drift_m_per_seed = 0;
  // Exactly 0 disables that draw. Any other value, including a non-finite
  // amplitude, engages that draw when its triple is present.
  double position_noise_amplitude_m = 0;
  double linear_velocity_noise_amplitude_m_s = 0;
  double angular_velocity_noise_amplitude_rad_s = 0;
  // receive_time = source_time + delay. Nanos may be negative.
  int64_t delay_seconds = 1;
  int32_t delay_nanos = -250000000;
  bool dropout = false;
  bool invalid_orientation = false;
};

struct InsTruth {
  std::string frame_id = "ins";
  int64_t source_seconds = 1700000000;
  int32_t source_nanos = 250000000;
  double position_x_m = 12;
  double position_y_m = -4;
  double position_z_m = 0.5;
  double orientation_x = 0;
  double orientation_y = 0;
  double orientation_z = 0;
  double orientation_w = 1;
  bool linear_velocity_present = true;
  double linear_velocity_x_m_s = 1.5;
  double linear_velocity_y_m_s = 0;
  double linear_velocity_z_m_s = -0.25;
  bool angular_velocity_present = true;
  double angular_velocity_x_rad_s = 0;
  double angular_velocity_y_rad_s = 0.125;
  double angular_velocity_z_rad_s = 0;
  // Wire source. 1 is SOURCE_VENDOR_INS. 2 is SOURCE_EXTERNAL_NAV.
  bool source_present = true;
  int source = 1;
  double quality = 0.75;
  bool covariance_present = true;
};

class FakeIns {
 public:
  explicit FakeIns(FakeInsConfig config = {}) : config_(config) {}

  const FakeInsConfig& config() const { return config_; }

  // Absent when dropout is set. Otherwise one InsSolution.
  std::optional<intrinsic_proto::hardware::marine::InsSolution> Measure(
      const InsTruth& truth = InsTruth()) const;

 private:
  FakeInsConfig config_;
};

}  // namespace intrinsic::hardware::marine

#endif  // INTRINSIC_HARDWARE_MARINE_FAKE_INS_H_
