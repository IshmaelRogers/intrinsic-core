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

#ifndef INTRINSIC_VEHICLE_CONTROL_CONTROL_H_
#define INTRINSIC_VEHICLE_CONTROL_CONTROL_H_

#include <string_view>

namespace intrinsic::vehicle::control {

// Soft-real-time package scaffold. Reference control belongs here,
// outside the ICON cycle. ReferenceController in reference_control.h
// is the reference-to-body-wrench boundary. ZeroWrenchController is
// the deterministic neutral-wrench fake. ReferenceDepthController is
// the heave depth law. ReferenceHeadingController is the yaw heading
// law. ReferenceForwardSpeedController is the surge speed law. Scalar
// heading wrap, the bounded integrator, and back-calculation live in
// control_math.h. This header defines no control law and writes no
// actuator command.
// Timing and ownership: README.md in this tree.
inline constexpr std::string_view kPackageName =
    "intrinsic_vehicle/intrinsic/vehicle/control";

}  // namespace intrinsic::vehicle::control

#endif  // INTRINSIC_VEHICLE_CONTROL_CONTROL_H_
