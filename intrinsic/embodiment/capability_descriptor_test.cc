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

#include <array>
#include <cstddef>
#include <span>
#include <string_view>

#include "gtest/gtest.h"
#include "intrinsic/embodiment/capability_policy.h"

namespace intrinsic::embodiment {
namespace {

using View = CapabilityDeclarationView;

TEST(CapabilityDescriptorTest, RejectsDuplicateDeclarations) {
  constexpr std::array<View, 2> declarations = {{
      {kCapabilityCommand, "binding.a"},
      {kCapabilityCommand, "binding.a"},
  }};
  const DeclarationAssessment assessment =
      AssessCapabilityDeclarations(declarations);
  EXPECT_EQ(assessment.error, DeclarationError::kDuplicate);
  EXPECT_EQ(assessment.capability_id, kCapabilityCommand);
}

TEST(CapabilityDescriptorTest, RejectsConflictingInterfaceBindings) {
  constexpr std::array<View, 2> declarations = {{
      {kCapabilityState, ""},
      {kCapabilityState, "binding.b"},
  }};
  const DeclarationAssessment assessment =
      AssessCapabilityDeclarations(declarations);
  EXPECT_EQ(assessment.error, DeclarationError::kConflict);
  EXPECT_EQ(assessment.capability_id, kCapabilityState);
}

TEST(CapabilityDescriptorTest, ConflictOutranksDuplicate) {
  constexpr std::array<View, 3> declarations = {{
      {kCapabilitySensor, "binding.a"},
      {kCapabilitySensor, "binding.a"},
      {kCapabilitySensor, "binding.b"},
  }};
  const DeclarationAssessment assessment =
      AssessCapabilityDeclarations(declarations);
  EXPECT_EQ(assessment.error, DeclarationError::kConflict);
  EXPECT_EQ(assessment.capability_id, kCapabilitySensor);
}

TEST(CapabilityDescriptorTest, RejectsEmptyCapabilityId) {
  constexpr std::array<View, 2> declarations = {{
      {kCapabilityActuator, ""},
      {"", ""},
  }};
  const DeclarationAssessment assessment =
      AssessCapabilityDeclarations(declarations);
  EXPECT_EQ(assessment.error, DeclarationError::kEmptyId);
  EXPECT_TRUE(assessment.capability_id.empty());
}

TEST(CapabilityDescriptorTest, PreservesUnknownCapabilityId) {
  constexpr std::string_view kUnknown = "ai.intrinsic.capability.extension";
  constexpr std::array<View, 2> declarations = {{
      {kCapabilityPlanning, ""},
      {kUnknown, ""},
  }};
  const DeclarationAssessment assessment =
      AssessCapabilityDeclarations(declarations);
  EXPECT_EQ(assessment.error, DeclarationError::kNone);
  EXPECT_FALSE(IsWellKnownCapabilityId(kUnknown));
  EXPECT_TRUE(DeclaresCapability(declarations, kUnknown));
  EXPECT_TRUE(IsWellKnownCapabilityId(kCapabilityPlanning));
  EXPECT_TRUE(DeclaresCapability(declarations, kCapabilityPlanning));
}

TEST(CapabilityDescriptorTest, RejectsDuplicateUnknownCapabilityId) {
  constexpr std::string_view kUnknown = "ai.intrinsic.capability.extension";
  constexpr std::array<View, 2> declarations = {{
      {kUnknown, ""},
      {kUnknown, ""},
  }};
  const DeclarationAssessment assessment =
      AssessCapabilityDeclarations(declarations);
  EXPECT_EQ(assessment.error, DeclarationError::kDuplicate);
  EXPECT_EQ(assessment.capability_id, kUnknown);
}

TEST(CapabilityDescriptorTest, ManipulatorDescriptorListsStableIds) {
  constexpr std::array<View, 6> declarations =
      ManipulatorCapabilityDeclarations();
  EXPECT_EQ(AssessCapabilityDeclarations(declarations).error,
            DeclarationError::kNone);
  ASSERT_EQ(declarations.size(), kWellKnownCapabilityIds.size());
  for (std::size_t i = 0; i < declarations.size(); ++i) {
    EXPECT_EQ(declarations[i].id, kWellKnownCapabilityIds[i]);
    EXPECT_TRUE(declarations[i].interface_id.empty());
    EXPECT_TRUE(IsWellKnownCapabilityId(declarations[i].id));
    EXPECT_TRUE(DeclaresCapability(declarations, declarations[i].id));
  }
  EXPECT_FALSE(IsWellKnownCapabilityId(kManipulatorResourceId));
  EXPECT_FALSE(DeclaresCapability(declarations, kManipulatorResourceId));
}

TEST(CapabilityDescriptorTest,
     MissingVehicleCapabilityLeavesManipulatorDescriptorValid) {
  constexpr std::string_view kVehicle = "ai.intrinsic.capability.vehicle";
  constexpr std::array<View, 6> declarations =
      ManipulatorCapabilityDeclarations();
  EXPECT_EQ(AssessCapabilityDeclarations(declarations).error,
            DeclarationError::kNone);
  EXPECT_FALSE(IsWellKnownCapabilityId(kVehicle));
  EXPECT_FALSE(DeclaresCapability(declarations, kVehicle));
  EXPECT_TRUE(DeclaresCapability(declarations, kCapabilityState));
  EXPECT_TRUE(DeclaresCapability(declarations, kCapabilityCommand));
  EXPECT_TRUE(DeclaresCapability(declarations, kCapabilitySensor));
  EXPECT_TRUE(DeclaresCapability(declarations, kCapabilityActuator));
  EXPECT_TRUE(DeclaresCapability(declarations, kCapabilityPlanning));
  EXPECT_TRUE(DeclaresCapability(declarations, kCapabilitySimulation));
}

TEST(CapabilityDescriptorTest, EmptyDeclarationsArePriorBehavior) {
  const std::span<const View> declarations;
  const DeclarationAssessment assessment =
      AssessCapabilityDeclarations(declarations);
  EXPECT_EQ(assessment.error, DeclarationError::kNone);
  EXPECT_TRUE(assessment.capability_id.empty());
  EXPECT_FALSE(DeclaresCapability(declarations, kCapabilityState));
  EXPECT_FALSE(DeclaresCapability(declarations, kCapabilitySimulation));
}

}  // namespace
}  // namespace intrinsic::embodiment
