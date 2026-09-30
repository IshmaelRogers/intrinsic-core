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

#ifndef INTRINSIC_HARDWARE_MARINE_FAKE_DVL_H_
#define INTRINSIC_HARDWARE_MARINE_FAKE_DVL_H_

#include <cstdint>
#include <optional>
#include <string>

#include "intrinsic/hardware/marine/dvl.pb.h"

namespace intrinsic::hardware::marine {

// Deterministic DVL producer. Same config and truth yield the same bytes.
// `seed` is written to `health.header.sequence`. It does not draw noise.
//
// Dropout omits the sample. Otherwise lock-loss forces bottom track, lock
// false, and health state INVALID. A non-zero bias without lock-loss sets
// health state DEGRADED and adds the bias to the truth velocity. Delay is
// added to source time to form receive time and is not repaired when it
// reverses the stamps.

struct FakeDvlConfig {
  uint64_t seed = 42;
  double bias_x_m_s = 0;
  double bias_y_m_s = 0;
  double bias_z_m_s = 0;
  // receive_time = source_time + delay. Nanos may be negative.
  int64_t delay_seconds = 1;
  int32_t delay_nanos = -250000000;
  bool dropout = false;
  bool lock_loss = false;
};

struct DvlTruth {
  std::string frame_id = "sensor";
  int64_t source_seconds = 1700000000;
  int32_t source_nanos = 250000000;
  double velocity_x_m_s = 0.5;
  double velocity_y_m_s = -0.25;
  double velocity_z_m_s = 0;
  // Wire mode. 1 is bottom track. 2 is water track.
  int mode = 1;
  double quality = 0.75;
  bool altitude_present = true;
  double altitude_m = 10;
  bool covariance_present = true;
};

// Normalize (seconds, nanos) after adding a delay. Nanos land in [0, 1e9).
void AddClockDelay(int64_t* seconds, int32_t* nanos, int64_t delay_seconds,
                   int32_t delay_nanos);

class FakeDvl {
 public:
  explicit FakeDvl(FakeDvlConfig config = FakeDvlConfig()) : config_(config) {}

  const FakeDvlConfig& config() const { return config_; }

  // Absent when dropout is set. Otherwise one DvlMeasurement.
  std::optional<intrinsic_proto::hardware::marine::DvlMeasurement> Measure(
      const DvlTruth& truth = DvlTruth()) const;

 private:
  FakeDvlConfig config_;
};

}  // namespace intrinsic::hardware::marine

#endif  // INTRINSIC_HARDWARE_MARINE_FAKE_DVL_H_
