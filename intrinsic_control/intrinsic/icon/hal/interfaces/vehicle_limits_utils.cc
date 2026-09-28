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

#include "intrinsic/icon/hal/interfaces/vehicle_limits_utils.h"

#include "flatbuffers/detached_buffer.h"
#include "flatbuffers/flatbuffer_builder.h"
#include "intrinsic/icon/hal/interfaces/vehicle_limits.fbs.h"

namespace intrinsic_fbs {

flatbuffers::DetachedBuffer BuildVehicleLimits() {
  flatbuffers::FlatBufferBuilder builder;
  builder.ForceDefaults(true);
  VehicleLimitsBuilder limits_builder(builder);
  limits_builder.add_has_linear_speed_limit(false);
  limits_builder.add_has_angular_speed_limit(false);
  limits_builder.add_has_linear_acceleration_limit(false);
  limits_builder.add_has_angular_acceleration_limit(false);
  limits_builder.add_has_force_limit(false);
  limits_builder.add_has_torque_limit(false);
  builder.Finish(limits_builder.Finish());
  return builder.Release();
}

}  // namespace intrinsic_fbs
