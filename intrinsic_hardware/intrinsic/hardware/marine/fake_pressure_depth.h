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

#ifndef INTRINSIC_HARDWARE_MARINE_FAKE_PRESSURE_DEPTH_H_
#define INTRINSIC_HARDWARE_MARINE_FAKE_PRESSURE_DEPTH_H_

#include <cstdint>
#include <optional>
#include <string>

#include "intrinsic/hardware/marine/pressure_depth.pb.h"

namespace intrinsic::hardware::marine {

// Deterministic pressure/depth producer. Same config and truth yield the
// same bytes. `seed` is written to `health.header.sequence`. It does not
// draw noise and it does not integrate hydrostatic pressure.
//
// Dropout omits the sample. Otherwise out-of-range forces depth_m to -1
// and health state INVALID. A non-zero bias without out-of-range sets
// health state DEGRADED and adds the bias to each present truth field.
// Delay is added to source time to form receive time and is not repaired
// when it reverses the stamps.

inline constexpr double kCanonicalOutOfRangeDepthM = -1.0;

struct FakePressureDepthConfig {
  uint64_t seed = 42;
  double pressure_bias_pa = 0;
  double depth_bias_m = 0;
  // receive_time = source_time + delay. Nanos may be negative.
  int64_t delay_seconds = 1;
  int32_t delay_nanos = -250000000;
  bool dropout = false;
  bool out_of_range = false;
};

struct PressureDepthTruth {
  std::string frame_id = "sensor";
  int64_t source_seconds = 1700000000;
  int32_t source_nanos = 250000000;
  bool pressure_present = true;
  double pressure_pa = 200000;
  bool depth_present = true;
  double depth_m = 10;
  bool provenance_present = true;
  // Wire provenance. 1 is direct. 2 is from pressure.
  int depth_provenance = 2;
  bool density_present = true;
  double fluid_density_kg_m3 = 1025;
  double quality = 0.75;
  bool covariance_present = true;
};

class FakePressureDepth {
 public:
  explicit FakePressureDepth(FakePressureDepthConfig config = {})
      : config_(config) {}

  const FakePressureDepthConfig& config() const { return config_; }

  // Absent when dropout is set. Otherwise one PressureDepthMeasurement.
  std::optional<intrinsic_proto::hardware::marine::PressureDepthMeasurement>
  Measure(const PressureDepthTruth& truth = PressureDepthTruth()) const;

 private:
  FakePressureDepthConfig config_;
};

}  // namespace intrinsic::hardware::marine

#endif  // INTRINSIC_HARDWARE_MARINE_FAKE_PRESSURE_DEPTH_H_
