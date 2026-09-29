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

#ifndef INTRINSIC_VEHICLE_PARAMETERS_SIX_THRUSTER_UUV_EXAMPLE_H_
#define INTRINSIC_VEHICLE_PARAMETERS_SIX_THRUSTER_UUV_EXAMPLE_H_

#include <string_view>

#include "intrinsic/vehicle/parameters/marine_model.h"

namespace intrinsic::vehicle::parameters {

// Calm-water inspection UUV used as a validated parameter example.
// Six fixed thrusters: surge port/starboard, sway fore/aft, heave fore/aft.
// Offsets are part of the example so the force axes span body wrenches.
// Each thruster is in the body frame. Slew, efficiency, and nominal health
// are populated. This is not a system-identification result, a thruster
// allocator, or a dynamics evaluation.
inline constexpr std::string_view kSixThrusterUuvModelId =
    "example_six_thruster_uuv";
inline constexpr int kSixThrusterUuvThrusterCount = 6;

MarineModel MakeSixThrusterUuvExample();

}  // namespace intrinsic::vehicle::parameters

#endif  // INTRINSIC_VEHICLE_PARAMETERS_SIX_THRUSTER_UUV_EXAMPLE_H_
