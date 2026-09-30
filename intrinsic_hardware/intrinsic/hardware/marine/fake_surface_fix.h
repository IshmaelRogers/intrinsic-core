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

#ifndef INTRINSIC_HARDWARE_MARINE_FAKE_SURFACE_FIX_H_
#define INTRINSIC_HARDWARE_MARINE_FAKE_SURFACE_FIX_H_

#include <cstdint>
#include <optional>
#include <string>

#include "intrinsic/hardware/marine/surface_fix.pb.h"

namespace intrinsic::hardware::marine {

// Deterministic surface-fix producer. Same config and truth yield the same
// bytes. `seed` is written to `health.header.sequence`. This producer does
// not read FakeIns, does not gate on submersion, and does not convert
// geodetic coordinates.
//
// Dropout omits the sample and is checked first. An invalid fix sets the
// source to FIX_SOURCE_UNSPECIFIED (a set field, not an absent one) and
// health state INVALID. It wins over bias: bias is not applied. Delay is
// still applied.
//
// Otherwise health state is DEGRADED when any position bias component is
// non-zero, and VALID when all are zero. Bias is added to the position
// components and is not repaired. Delay is added to source time to form
// receive time and is not repaired when it reverses the stamps.

struct FakeSurfaceFixConfig {
  uint64_t seed = 42;
  double position_bias_x_m = 0;
  double position_bias_y_m = 0;
  double position_bias_z_m = 0;
  // receive_time = source_time + delay. Nanos may be negative.
  int64_t delay_seconds = 1;
  int32_t delay_nanos = -250000000;
  bool dropout = false;
  bool invalid_fix = false;
};

struct SurfaceFixTruth {
  std::string frame_id = "gnss";
  int64_t source_seconds = 1700000000;
  int32_t source_nanos = 250000000;
  double position_x_m = 12.5;
  double position_y_m = -3.25;
  double position_z_m = 1.0;
  // Wire source. 1 is FIX_SOURCE_GNSS. 2 is FIX_SOURCE_ACOUSTIC.
  bool source_present = true;
  int source = 1;
  bool satellite_count_present = true;
  int32_t satellite_count = 12;
  bool beacon_count_present = false;
  int32_t beacon_count = 0;
  bool horizontal_accuracy_present = false;
  double horizontal_accuracy_m = 0;
  bool vertical_accuracy_present = false;
  double vertical_accuracy_m = 0;
  bool velocity_present = false;
  double velocity_x_m_s = 0;
  double velocity_y_m_s = 0;
  double velocity_z_m_s = 0;
  double quality = 0.75;
  bool covariance_present = true;
};

class FakeSurfaceFix {
 public:
  explicit FakeSurfaceFix(FakeSurfaceFixConfig config = {}) : config_(config) {}

  const FakeSurfaceFixConfig& config() const { return config_; }

  // Absent when dropout is set. Otherwise one SurfaceFix.
  std::optional<intrinsic_proto::hardware::marine::SurfaceFix> Measure(
      const SurfaceFixTruth& truth = SurfaceFixTruth()) const;

 private:
  FakeSurfaceFixConfig config_;
};

}  // namespace intrinsic::hardware::marine

#endif  // INTRINSIC_HARDWARE_MARINE_FAKE_SURFACE_FIX_H_
