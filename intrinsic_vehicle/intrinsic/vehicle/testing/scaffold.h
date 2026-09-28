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

#ifndef INTRINSIC_VEHICLE_TESTING_SCAFFOLD_H_
#define INTRINSIC_VEHICLE_TESTING_SCAFFOLD_H_

#include "intrinsic/vehicle/allocation/allocation.h"
#include "intrinsic/vehicle/control/control.h"
#include "intrinsic/vehicle/dynamics/dynamics.h"
#include "intrinsic/vehicle/guidance/guidance.h"
#include "intrinsic/vehicle/parameters/parameters.h"
#include "intrinsic/vehicle/state/state.h"

namespace intrinsic::vehicle::testing {

// Test-only umbrella over the vehicle package scaffold.
// Production targets must not depend on this library.
inline constexpr char kScaffoldPackages[] =
    "allocation,control,dynamics,guidance,parameters,state";

}  // namespace intrinsic::vehicle::testing

#endif  // INTRINSIC_VEHICLE_TESTING_SCAFFOLD_H_
