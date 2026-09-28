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

#ifndef INTRINSIC_ICON_HAL_INTERFACES_VEHICLE_HARDWARE_INTERFACES_H_
#define INTRINSIC_ICON_HAL_INTERFACES_VEHICLE_HARDWARE_INTERFACES_H_

#include "intrinsic/icon/hal/hardware_interface_traits.h"
#include "intrinsic/icon/hal/interfaces/body_state.fbs.h"
#include "intrinsic/icon/hal/interfaces/body_state_utils.h"
#include "intrinsic/icon/hal/interfaces/body_wrench.fbs.h"
#include "intrinsic/icon/hal/interfaces/body_wrench_utils.h"
#include "intrinsic/icon/hal/interfaces/vehicle_limits.fbs.h"
#include "intrinsic/icon/hal/interfaces/vehicle_limits_utils.h"

namespace intrinsic::icon {
namespace hardware_interface_traits {

// Opt-in vehicle HAL buffers. Manipulator modules do not include this header.
INTRINSIC_ADD_HARDWARE_INTERFACE(intrinsic_fbs::BodyState,
                                 intrinsic_fbs::BuildBodyState,
                                 "intrinsic_fbs.BodyState")

INTRINSIC_ADD_HARDWARE_INTERFACE(intrinsic_fbs::BodyWrench,
                                 intrinsic_fbs::BuildBodyWrench,
                                 "intrinsic_fbs.BodyWrench")

INTRINSIC_ADD_HARDWARE_INTERFACE(intrinsic_fbs::VehicleLimits,
                                 intrinsic_fbs::BuildVehicleLimits,
                                 "intrinsic_fbs.VehicleLimits")

}  // namespace hardware_interface_traits
}  // namespace intrinsic::icon

#endif  // INTRINSIC_ICON_HAL_INTERFACES_VEHICLE_HARDWARE_INTERFACES_H_
