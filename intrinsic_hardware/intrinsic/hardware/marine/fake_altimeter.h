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

#ifndef INTRINSIC_HARDWARE_MARINE_FAKE_ALTIMETER_H_
#define INTRINSIC_HARDWARE_MARINE_FAKE_ALTIMETER_H_

#include <cstdint>
#include <optional>
#include <string>

#include "intrinsic/hardware/marine/altimeter.pb.h"

namespace intrinsic::hardware::marine {

// Deterministic altimeter producer. Same config and truth yield the same
// bytes. `seed` is written to `health.header.sequence` and mixes the noise
// draw. There is no bathymetry fusion and no DVL altitude reuse.
//
// Dropout omits the sample and is checked before no-return and
// out-of-range. No-return sets has_return false, omits range, and sets
// health state INVALID. It wins over out-of-range and noise. Out-of-range
// replaces range, bounds, and has_return with the canonical fixture and
// sets INVALID. Noise is not added on either INVALID fault.
//
// A non-zero noise amplitude otherwise adds
// `noise_amplitude_m * AltimeterSignedUnitNoise(seed)` to a present range
// and sets health state DEGRADED. The numeric result is not repaired when
// it is negative or outside the bounds. Delay is added to source time to
// form receive time and is not repaired when it reverses the stamps.

// Canonical out-of-range fixture. 101 m is strictly above max 100 m and
// strictly above min 0.5 m.
inline constexpr double kCanonicalMinRangeM = 0.5;
inline constexpr double kCanonicalMaxRangeM = 100.0;
inline constexpr double kCanonicalOutOfRangeRangeM = 101.0;

// SplitMix64 of `seed`, mapped onto [-1, 1). The 53-bit fraction is exact
// in binary64. The Python fake uses the same mix.
inline double AltimeterSignedUnitNoise(uint64_t seed) {
  uint64_t mixed = seed + 0x9E3779B97F4A7C15ULL;
  mixed = (mixed ^ (mixed >> 30)) * 0xBF58476D1CE4E5B9ULL;
  mixed = (mixed ^ (mixed >> 27)) * 0x94D049BB133111EBULL;
  mixed ^= mixed >> 31;
  const double unit = static_cast<double>(mixed >> 11) * 0x1p-53;
  return unit * 2.0 - 1.0;
}

struct FakeAltimeterConfig {
  uint64_t seed = 42;
  // Meters. Exactly 0 disables the draw. Any other value, including a
  // non-finite amplitude, engages noise.
  double noise_amplitude_m = 0;
  // receive_time = source_time + delay. Nanos may be negative.
  int64_t delay_seconds = 1;
  int32_t delay_nanos = -250000000;
  bool dropout = false;
  bool no_return = false;
  bool out_of_range = false;
};

struct AltimeterTruth {
  std::string frame_id = "sensor";
  int64_t source_seconds = 1700000000;
  int32_t source_nanos = 250000000;
  bool range_present = true;
  double range_m = 10;
  bool beam_present = true;
  std::string beam_id = "down";
  bool min_present = true;
  double min_range_m = kCanonicalMinRangeM;
  bool max_present = true;
  double max_range_m = kCanonicalMaxRangeM;
  bool has_return_present = true;
  bool has_return = true;
  double quality = 0.75;
  bool covariance_present = true;
};

class FakeAltimeter {
 public:
  explicit FakeAltimeter(FakeAltimeterConfig config = {}) : config_(config) {}

  const FakeAltimeterConfig& config() const { return config_; }

  // Absent when dropout is set. Otherwise one AltimeterMeasurement.
  std::optional<intrinsic_proto::hardware::marine::AltimeterMeasurement>
  Measure(const AltimeterTruth& truth = AltimeterTruth()) const;

 private:
  FakeAltimeterConfig config_;
};

}  // namespace intrinsic::hardware::marine

#endif  // INTRINSIC_HARDWARE_MARINE_FAKE_ALTIMETER_H_
