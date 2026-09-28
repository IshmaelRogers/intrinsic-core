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

#ifndef INTRINSIC_EMBODIMENT_CAPABILITY_POLICY_H_
#define INTRINSIC_EMBODIMENT_CAPABILITY_POLICY_H_

#include <array>
#include <cstddef>
#include <span>
#include <string_view>

namespace intrinsic::embodiment {

// Policy: intrinsic_apis/intrinsic/embodiment/proto/README.md.
//
// Well-known capability ids. Comparison is exact. Unknown strings are kept
// by the descriptor and are not errors. Order is the manipulator fixture
// order: state, command, sensor, actuator, planning, simulation.

inline constexpr std::string_view kCapabilityState =
    "ai.intrinsic.capability.state";
inline constexpr std::string_view kCapabilityCommand =
    "ai.intrinsic.capability.command";
inline constexpr std::string_view kCapabilitySensor =
    "ai.intrinsic.capability.sensor";
inline constexpr std::string_view kCapabilityActuator =
    "ai.intrinsic.capability.actuator";
inline constexpr std::string_view kCapabilityPlanning =
    "ai.intrinsic.capability.planning";
inline constexpr std::string_view kCapabilitySimulation =
    "ai.intrinsic.capability.simulation";

inline constexpr std::array<std::string_view, 6> kWellKnownCapabilityIds = {
    kCapabilityState,    kCapabilityCommand,  kCapabilitySensor,
    kCapabilityActuator, kCapabilityPlanning, kCapabilitySimulation,
};

// Opaque fixture name for the existing manipulator compatibility profile.
// Callers must not select behavior from this string.
inline constexpr std::string_view kManipulatorResourceId =
    "ai.intrinsic.compatibility_profile.manipulator";

struct CapabilityDeclarationView {
  std::string_view id;
  std::string_view interface_id;
};

enum class DeclarationError {
  kNone = 0,
  kEmptyId = 1,
  kDuplicate = 2,
  kConflict = 3,
};

struct DeclarationAssessment {
  DeclarationError error = DeclarationError::kNone;
  // Points at an id in `declarations` when error is kDuplicate or kConflict.
  // Empty for kNone and kEmptyId. Valid while that span's data is alive.
  std::string_view capability_id;
};

// The six category ids, each with an empty interface binding. Describes the
// existing manipulator surface. Does not change joint, Cartesian, ICON,
// kinematics, motion-planning, World, or Gazebo contracts.
inline constexpr std::array<CapabilityDeclarationView, 6>
ManipulatorCapabilityDeclarations() {
  std::array<CapabilityDeclarationView, 6> declarations = {};
  for (std::size_t i = 0; i < kWellKnownCapabilityIds.size(); ++i) {
    declarations[i] = CapabilityDeclarationView{kWellKnownCapabilityIds[i], ""};
  }
  return declarations;
}

inline constexpr bool IsWellKnownCapabilityId(std::string_view id) {
  for (std::string_view known : kWellKnownCapabilityIds) {
    if (id == known) {
      return true;
    }
  }
  return false;
}

inline constexpr bool DeclaresCapability(
    std::span<const CapabilityDeclarationView> declarations,
    std::string_view id) {
  for (const CapabilityDeclarationView& declaration : declarations) {
    if (declaration.id == id) {
      return true;
    }
  }
  return false;
}

// Empty id is reported first. A repeated id with one interface_id is a
// duplicate. The same id with two interface_id values is a conflict, and
// that result wins when the list also contains a duplicate. Unknown ids are
// accepted. An empty span is valid and advertises nothing.
inline constexpr DeclarationAssessment AssessCapabilityDeclarations(
    std::span<const CapabilityDeclarationView> declarations) {
  for (const CapabilityDeclarationView& declaration : declarations) {
    if (declaration.id.empty()) {
      return DeclarationAssessment{DeclarationError::kEmptyId, {}};
    }
  }
  DeclarationAssessment duplicate{DeclarationError::kNone, {}};
  for (std::size_t i = 0; i < declarations.size(); ++i) {
    for (std::size_t j = 0; j < i; ++j) {
      if (declarations[i].id != declarations[j].id) {
        continue;
      }
      if (declarations[i].interface_id != declarations[j].interface_id) {
        return DeclarationAssessment{DeclarationError::kConflict,
                                     declarations[i].id};
      }
      if (duplicate.error == DeclarationError::kNone) {
        duplicate = DeclarationAssessment{DeclarationError::kDuplicate,
                                          declarations[i].id};
      }
    }
  }
  return duplicate;
}

}  // namespace intrinsic::embodiment

#endif  // INTRINSIC_EMBODIMENT_CAPABILITY_POLICY_H_
