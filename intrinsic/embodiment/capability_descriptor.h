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

#ifndef INTRINSIC_EMBODIMENT_CAPABILITY_DESCRIPTOR_H_
#define INTRINSIC_EMBODIMENT_CAPABILITY_DESCRIPTOR_H_

#include <cstddef>
#include <initializer_list>
#include <iterator>
#include <span>
#include <string_view>

namespace intrinsic::embodiment {

// Policy: intrinsic_apis/intrinsic/embodiment/proto/README.md.
//
// Category numbers match intrinsic_proto.embodiment.CapabilityCategory.
// This header does not depend on the generated proto and does not switch on
// a robot type.

inline constexpr int kCapabilityCategoryUnspecified = 0;
inline constexpr int kCapabilityCategoryState = 1;
inline constexpr int kCapabilityCategoryCommand = 2;
inline constexpr int kCapabilityCategorySensor = 3;
inline constexpr int kCapabilityCategoryActuator = 4;
inline constexpr int kCapabilityCategoryPlanning = 5;
inline constexpr int kCapabilityCategorySimulation = 6;

inline constexpr std::string_view kCapabilityCategoryIdState = "state";
inline constexpr std::string_view kCapabilityCategoryIdCommand = "command";
inline constexpr std::string_view kCapabilityCategoryIdSensor = "sensor";
inline constexpr std::string_view kCapabilityCategoryIdActuator = "actuator";
inline constexpr std::string_view kCapabilityCategoryIdPlanning = "planning";
inline constexpr std::string_view kCapabilityCategoryIdSimulation =
    "simulation";

inline constexpr std::string_view kManipulatorResourceId = "manipulator";

// One declaration. string_view fields point at caller storage (literals, or
// strings owned by a proto that outlives the view).
struct CapabilityDeclarationView {
  std::string_view id;
  int category = kCapabilityCategoryUnspecified;
  std::string_view interface_name;
};

enum class DeclarationStatus {
  kOk = 0,
  kDuplicateId,
  kConflictingDeclaration,
  kEmptyId,
  kUnspecifiedCategory,
};

struct DeclarationProblem {
  DeclarationStatus status = DeclarationStatus::kOk;
  // Index of the first offending declaration. Zero when status is kOk.
  std::size_t index = 0;
};

// Empty string when category is unspecified or unknown to this revision.
inline std::string_view CategoryStableId(int category) {
  switch (category) {
    case kCapabilityCategoryState:
      return kCapabilityCategoryIdState;
    case kCapabilityCategoryCommand:
      return kCapabilityCategoryIdCommand;
    case kCapabilityCategorySensor:
      return kCapabilityCategoryIdSensor;
    case kCapabilityCategoryActuator:
      return kCapabilityCategoryIdActuator;
    case kCapabilityCategoryPlanning:
      return kCapabilityCategoryIdPlanning;
    case kCapabilityCategorySimulation:
      return kCapabilityCategoryIdSimulation;
    default:
      return {};
  }
}

inline bool IsUnspecifiedCategory(int category) {
  return category == kCapabilityCategoryUnspecified;
}

inline bool IsKnownCategory(int category) {
  return !CategoryStableId(category).empty();
}

// First problem in declaration order. Does not modify `declarations`.
// Unknown ids and unknown category numbers other than 0 are accepted.
// `Views` is a contiguous indexable range (array, vector, or span).
template <typename Views>
inline DeclarationProblem ValidateDeclarations(const Views& declarations) {
  const std::size_t count = std::size(declarations);
  for (std::size_t index = 0; index < count; ++index) {
    const CapabilityDeclarationView& declaration = declarations[index];
    if (declaration.id.empty()) {
      return DeclarationProblem{DeclarationStatus::kEmptyId, index};
    }
    if (IsUnspecifiedCategory(declaration.category)) {
      return DeclarationProblem{DeclarationStatus::kUnspecifiedCategory, index};
    }
    for (std::size_t earlier = 0; earlier < index; ++earlier) {
      const CapabilityDeclarationView& previous = declarations[earlier];
      if (previous.id != declaration.id) {
        continue;
      }
      if (previous.category != declaration.category ||
          previous.interface_name != declaration.interface_name) {
        return DeclarationProblem{DeclarationStatus::kConflictingDeclaration,
                                  index};
      }
      return DeclarationProblem{DeclarationStatus::kDuplicateId, index};
    }
  }
  return DeclarationProblem{};
}

inline DeclarationProblem ValidateDeclarations(
    std::initializer_list<CapabilityDeclarationView> declarations) {
  const std::span<const CapabilityDeclarationView> view(declarations.begin(),
                                                        declarations.size());
  return ValidateDeclarations(view);
}

// Null when `id` is absent. Does not insert a capability and does not
// require a vehicle capability to be present. The returned pointer addresses
// an element of `declarations` and is valid only as long as that storage is.
template <typename Views>
inline const CapabilityDeclarationView* FindCapability(
    const Views& declarations, std::string_view id) {
  for (const CapabilityDeclarationView& declaration : declarations) {
    if (declaration.id == id) {
      return &declaration;
    }
  }
  return nullptr;
}

// Industrial compatibility profile, in FeatureInterfaceTypes numeric order,
// then motion_planner, then simulator. Keep in sync with
// capability_descriptor.py and the README table. Not a live registration.
inline constexpr CapabilityDeclarationView kManipulatorCapabilities[] = {
    {"joint_position", kCapabilityCategoryCommand, "JointPosition"},
    {"joint_velocity", kCapabilityCategoryCommand, "JointVelocity"},
    {"joint_position_sensor", kCapabilityCategorySensor, "JointPositionSensor"},
    {"joint_velocity_estimator", kCapabilityCategoryState,
     "JointVelocityEstimator"},
    {"joint_acceleration_estimator", kCapabilityCategoryState,
     "JointAccelerationEstimator"},
    {"joint_limits", kCapabilityCategoryState, "JointLimitsInterface"},
    {"cartesian_limits", kCapabilityCategoryState, "CartesianLimitsInterface"},
    {"simple_gripper", kCapabilityCategoryActuator, "SimpleGripper"},
    {"adio", kCapabilityCategorySensor, "ADIO"},
    {"range_finder", kCapabilityCategorySensor, "RangeFinder"},
    {"manipulator_kinematics", kCapabilityCategoryPlanning,
     "ManipulatorKinematics"},
    {"joint_torque", kCapabilityCategoryCommand, "JointTorque"},
    {"joint_torque_sensor", kCapabilityCategorySensor, "JointTorqueSensor"},
    {"dynamics", kCapabilityCategoryPlanning, "Dynamics"},
    {"force_torque_sensor", kCapabilityCategorySensor, "ForceTorqueSensor"},
    {"linear_gripper", kCapabilityCategoryActuator, "LinearGripper"},
    {"hand_guiding", kCapabilityCategoryCommand, "HandGuiding"},
    {"control_mode_exporter", kCapabilityCategoryState, "ControlModeExporter"},
    {"move_ok", kCapabilityCategoryState, "MoveOk"},
    {"imu", kCapabilityCategorySensor, "InertialMeasurementUnit"},
    {"standalone_force_torque_sensor", kCapabilityCategorySensor,
     "StandaloneForceTorqueSensor"},
    {"process_wrench_at_endeffector", kCapabilityCategoryCommand,
     "ProcessWrenchAtEndeffector"},
    {"payload", kCapabilityCategoryCommand, "Payload"},
    {"payload_state", kCapabilityCategoryState, "PayloadState"},
    {"cartesian_position_state", kCapabilityCategoryState,
     "CartesianPositionState"},
    {"homing", kCapabilityCategoryCommand, "Homing"},
    {"joint_acceleration", kCapabilityCategoryCommand, "JointAcceleration"},
    {"motion_planner", kCapabilityCategoryPlanning, "MotionPlannerInterface"},
    {"simulator", kCapabilityCategorySimulation, "Simulator"},
};

inline std::span<const CapabilityDeclarationView> ManipulatorCapabilities() {
  return kManipulatorCapabilities;
}

// False for unknown ids, including vehicle ids that this profile does not
// advertise. False does not delete the declaration.
inline bool IsManipulatorCapabilityId(std::string_view id) {
  return FindCapability(ManipulatorCapabilities(), id) != nullptr;
}

}  // namespace intrinsic::embodiment

#endif  // INTRINSIC_EMBODIMENT_CAPABILITY_DESCRIPTOR_H_
