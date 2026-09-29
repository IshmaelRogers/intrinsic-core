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

#ifndef INTRINSIC_VEHICLE_DYNAMICS_DYNAMICS_H_
#define INTRINSIC_VEHICLE_DYNAMICS_DYNAMICS_H_

#include <string_view>

namespace intrinsic::vehicle::dynamics {

// Soft-real-time package scaffold. VehicleDynamics lives in
// vehicle_dynamics.h and is evaluated outside the ICON cycle.
// This header is the package marker. It does not evaluate forces.
// Marine model parameters are validated in the parameters package.
// Timing, allocation, and thread safety: README.md in this tree.
inline constexpr std::string_view kPackageName =
    "intrinsic_vehicle/intrinsic/vehicle/dynamics";

}  // namespace intrinsic::vehicle::dynamics

#endif  // INTRINSIC_VEHICLE_DYNAMICS_DYNAMICS_H_
