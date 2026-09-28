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

#include "intrinsic/embodiment/capability_descriptor.h"

#include <cstddef>
#include <string>
#include <string_view>
#include <vector>

#include "google/protobuf/unknown_field_set.h"
#include "gtest/gtest.h"
#include "intrinsic/embodiment/capability_descriptor_fixture.h"
#include "intrinsic/embodiment/proto/capability_descriptor.pb.h"

namespace intrinsic::embodiment {
namespace {

using intrinsic_proto::embodiment::CapabilityCategory;
using intrinsic_proto::embodiment::CapabilityDeclaration;
using intrinsic_proto::embodiment::EmbodimentDescriptor;

// Canonical bytes of FillManipulatorDescriptor(). Keep in sync with
// capability_descriptor_test.py.
constexpr std::string_view kManipulatorGoldenHex =
    "0a0b6d616e6970756c61746f7212210a0e6a6f696e745f706f736974696f6e10021a"
    "0d4a6f696e74506f736974696f6e12210a0e6a6f696e745f76656c6f636974791002"
    "1a0d4a6f696e7456656c6f63697479122e0a156a6f696e745f706f736974696f6e5f"
    "73656e736f7210031a134a6f696e74506f736974696f6e53656e736f7212340a186a"
    "6f696e745f76656c6f636974795f657374696d61746f7210011a164a6f696e745665"
    "6c6f63697479457374696d61746f72123c0a1c6a6f696e745f616363656c65726174"
    "696f6e5f657374696d61746f7210011a1a4a6f696e74416363656c65726174696f6e"
    "457374696d61746f7212260a0c6a6f696e745f6c696d69747310011a144a6f696e74"
    "4c696d697473496e74657266616365122e0a1063617274657369616e5f6c696d6974"
    "7310011a1843617274657369616e4c696d697473496e7465726661636512210a0e73"
    "696d706c655f6772697070657210041a0d53696d706c6547726970706572120e0a04"
    "6164696f10031a044144494f121d0a0c72616e67655f66696e64657210031a0b5261"
    "6e676546696e64657212310a166d616e6970756c61746f725f6b696e656d61746963"
    "7310051a154d616e6970756c61746f724b696e656d6174696373121d0a0c6a6f696e"
    "745f746f7271756510021a0b4a6f696e74546f72717565122a0a136a6f696e745f74"
    "6f727175655f73656e736f7210031a114a6f696e74546f7271756553656e736f7212"
    "160a0864796e616d69637310051a0844796e616d696373122a0a13666f7263655f74"
    "6f727175655f73656e736f7210031a11466f726365546f7271756553656e736f7212"
    "210a0e6c696e6561725f6772697070657210041a0d4c696e65617247726970706572"
    "121d0a0c68616e645f67756964696e6710021a0b48616e6447756964696e67122e0a"
    "15636f6e74726f6c5f6d6f64655f6578706f7274657210011a13436f6e74726f6c4d"
    "6f64654578706f7274657212130a076d6f76655f6f6b10011a064d6f76654f6b1220"
    "0a03696d7510031a17496e65727469616c4d6561737572656d656e74556e6974123f"
    "0a1e7374616e64616c6f6e655f666f7263655f746f727175655f73656e736f721003"
    "1a1b5374616e64616c6f6e65466f726365546f7271756553656e736f72123d0a1d70"
    "726f636573735f7772656e63685f61745f656e646566666563746f7210021a1a5072"
    "6f636573735772656e63684174456e646566666563746f7212140a077061796c6f61"
    "6410021a075061796c6f6164121f0a0d7061796c6f61645f737461746510011a0c50"
    "61796c6f6164537461746512340a1863617274657369616e5f706f736974696f6e5f"
    "737461746510011a1643617274657369616e506f736974696f6e537461746512120a"
    "06686f6d696e6710021a06486f6d696e6712290a126a6f696e745f616363656c6572"
    "6174696f6e10021a114a6f696e74416363656c65726174696f6e122a0a0e6d6f7469"
    "6f6e5f706c616e6e657210051a164d6f74696f6e506c616e6e6572496e7465726661"
    "636512180a0973696d756c61746f7210061a0953696d756c61746f72";

// resource_id "example", one joint_position command. Not the manipulator
// fixture. Keep in sync with capability_descriptor_test.py.
constexpr std::string_view kExampleGoldenHex =
    "0a076578616d706c6512210a0e6a6f696e745f706f736974696f6e10021a0d4a6f69"
    "6e74506f736974696f6e";

// id future.range_observation, category 100, interface RangeObservation.
constexpr std::string_view kUnknownCategoryHex =
    "0a186675747572652e72616e67655f6f62736572766174696f6e10641a1052616e67"
    "654f62736572766174696f6e";

constexpr std::string_view kAbsentIds[] = {
    "BodyState",      "BodyWrenchCommand",   "VehicleLimits",
    "StateSpace",     "DynamicsModel",       "RangeObservation",
    "SafetyRule",     "VehicleState",        "DesiredMotion",
    "body_state",     "body_wrench_command", "vehicle_limits",
    "dynamics_model", "range_observation",   "safety_rule",
    "vehicle_state",  "desired_motion",
};

std::string FromHex(std::string_view hex) {
  std::string out;
  out.reserve(hex.size() / 2);
  auto nibble = [](char c) -> int {
    if (c >= '0' && c <= '9') {
      return c - '0';
    }
    return c - 'a' + 10;
  };
  for (size_t i = 0; i < hex.size(); i += 2) {
    out.push_back(
        static_cast<char>((nibble(hex[i]) << 4) | nibble(hex[i + 1])));
  }
  return out;
}

std::vector<CapabilityDeclarationView> ViewsOf(
    const EmbodimentDescriptor& descriptor) {
  std::vector<CapabilityDeclarationView> views;
  views.reserve(static_cast<std::size_t>(descriptor.capabilities_size()));
  for (const auto& capability : descriptor.capabilities()) {
    views.push_back(CapabilityDeclarationView{
        capability.id(), static_cast<int>(capability.category()),
        capability.interface_name()});
  }
  return views;
}

template <typename Views>
bool CoversKnownCategories(const Views& declarations) {
  bool seen[kCapabilityCategorySimulation + 1] = {};
  for (const CapabilityDeclarationView& declaration : declarations) {
    if (IsKnownCategory(declaration.category)) {
      seen[declaration.category] = true;
    }
  }
  for (int category = kCapabilityCategoryState;
       category <= kCapabilityCategorySimulation; ++category) {
    if (!seen[category]) {
      return false;
    }
  }
  return true;
}

TEST(CapabilityDescriptorTest, CategoryNumbersAndStableIds) {
  EXPECT_EQ(
      static_cast<int>(CapabilityCategory::CAPABILITY_CATEGORY_UNSPECIFIED),
      kCapabilityCategoryUnspecified);
  EXPECT_EQ(static_cast<int>(CapabilityCategory::CAPABILITY_CATEGORY_STATE),
            kCapabilityCategoryState);
  EXPECT_EQ(static_cast<int>(CapabilityCategory::CAPABILITY_CATEGORY_COMMAND),
            kCapabilityCategoryCommand);
  EXPECT_EQ(static_cast<int>(CapabilityCategory::CAPABILITY_CATEGORY_SENSOR),
            kCapabilityCategorySensor);
  EXPECT_EQ(static_cast<int>(CapabilityCategory::CAPABILITY_CATEGORY_ACTUATOR),
            kCapabilityCategoryActuator);
  EXPECT_EQ(static_cast<int>(CapabilityCategory::CAPABILITY_CATEGORY_PLANNING),
            kCapabilityCategoryPlanning);
  EXPECT_EQ(
      static_cast<int>(CapabilityCategory::CAPABILITY_CATEGORY_SIMULATION),
      kCapabilityCategorySimulation);
  EXPECT_EQ(CategoryStableId(kCapabilityCategoryState),
            kCapabilityCategoryIdState);
  EXPECT_EQ(CategoryStableId(kCapabilityCategoryCommand),
            kCapabilityCategoryIdCommand);
  EXPECT_EQ(CategoryStableId(kCapabilityCategorySensor),
            kCapabilityCategoryIdSensor);
  EXPECT_EQ(CategoryStableId(kCapabilityCategoryActuator),
            kCapabilityCategoryIdActuator);
  EXPECT_EQ(CategoryStableId(kCapabilityCategoryPlanning),
            kCapabilityCategoryIdPlanning);
  EXPECT_EQ(CategoryStableId(kCapabilityCategorySimulation),
            kCapabilityCategoryIdSimulation);
  EXPECT_TRUE(CategoryStableId(kCapabilityCategoryUnspecified).empty());
  EXPECT_TRUE(CategoryStableId(100).empty());
  EXPECT_TRUE(IsUnspecifiedCategory(0));
  EXPECT_FALSE(IsUnspecifiedCategory(100));
  EXPECT_FALSE(IsKnownCategory(0));
  EXPECT_FALSE(IsKnownCategory(100));
  EXPECT_TRUE(IsKnownCategory(kCapabilityCategoryState));
  EXPECT_EQ(intrinsic_proto::embodiment::CapabilityCategory_descriptor()
                ->value_count(),
            7);
}

TEST(CapabilityDescriptorTest, SchemaHasNoRobotTypeField) {
  const google::protobuf::Descriptor* descriptor =
      EmbodimentDescriptor::descriptor();
  ASSERT_EQ(descriptor->field_count(), 2);
  EXPECT_EQ(descriptor->field(0)->name(), "resource_id");
  EXPECT_EQ(descriptor->field(1)->name(), "capabilities");
  EXPECT_EQ(descriptor->FindFieldByName("robot_type"), nullptr);
  EXPECT_EQ(descriptor->FindFieldByName("embodiment"), nullptr);

  const google::protobuf::Descriptor* declaration =
      CapabilityDeclaration::descriptor();
  ASSERT_EQ(declaration->field_count(), 3);
  EXPECT_EQ(declaration->field(0)->name(), "id");
  EXPECT_EQ(declaration->field(1)->name(), "category");
  EXPECT_EQ(declaration->field(2)->name(), "interface_name");
  EXPECT_EQ(declaration->FindFieldByName("robot_type"), nullptr);
}

TEST(CapabilityDescriptorTest, ManipulatorGoldenRoundTrip) {
  EmbodimentDescriptor descriptor;
  FillManipulatorDescriptor(&descriptor);
  const std::string golden = FromHex(kManipulatorGoldenHex);
  EXPECT_EQ(descriptor.SerializeAsString(), golden);

  EmbodimentDescriptor parsed;
  ASSERT_TRUE(parsed.ParseFromString(golden));
  EXPECT_EQ(parsed.resource_id(), kManipulatorResourceId);
  ASSERT_EQ(parsed.capabilities_size(),
            static_cast<int>(std::size(kManipulatorCapabilities)));
  for (int i = 0; i < parsed.capabilities_size(); ++i) {
    const auto& got = parsed.capabilities(i);
    const CapabilityDeclarationView& want = kManipulatorCapabilities[i];
    EXPECT_EQ(got.id(), want.id);
    EXPECT_EQ(static_cast<int>(got.category()), want.category);
    EXPECT_EQ(got.interface_name(), want.interface_name);
    EXPECT_NE(got.id(), CategoryStableId(want.category));
  }
  EXPECT_EQ(parsed.SerializeAsString(), golden);

  EmbodimentDescriptor again;
  FillManipulatorDescriptor(&again);
  EXPECT_EQ(again.SerializeAsString(), golden);

  const std::vector<CapabilityDeclarationView> views = ViewsOf(parsed);
  EXPECT_EQ(ValidateDeclarations(views).status, DeclarationStatus::kOk);
  EXPECT_TRUE(CoversKnownCategories(views));
  const CapabilityDeclarationView* joint =
      FindCapability(views, "joint_position");
  ASSERT_NE(joint, nullptr);
  EXPECT_EQ(joint->category, kCapabilityCategoryCommand);
  EXPECT_EQ(joint->interface_name, "JointPosition");
  const CapabilityDeclarationView* dynamics = FindCapability(views, "dynamics");
  ASSERT_NE(dynamics, nullptr);
  EXPECT_EQ(dynamics->interface_name, "Dynamics");
  EXPECT_EQ(dynamics->category, kCapabilityCategoryPlanning);
  EXPECT_TRUE(IsManipulatorCapabilityId("joint_position"));
  EXPECT_TRUE(IsManipulatorCapabilityId("simulator"));
}

TEST(CapabilityDescriptorTest, MissingVehicleCapabilityLeavesManipulatorBytes) {
  EmbodimentDescriptor descriptor;
  FillManipulatorDescriptor(&descriptor);
  const std::string before = descriptor.SerializeAsString();
  const std::vector<CapabilityDeclarationView> views = ViewsOf(descriptor);
  for (const std::string_view absent : kAbsentIds) {
    EXPECT_EQ(FindCapability(views, absent), nullptr) << absent;
    EXPECT_FALSE(IsManipulatorCapabilityId(absent)) << absent;
    for (const CapabilityDeclarationView& capability : views) {
      EXPECT_NE(capability.interface_name, absent) << absent;
    }
  }
  EXPECT_EQ(descriptor.SerializeAsString(), before);
  EXPECT_EQ(ValidateDeclarations(views).status, DeclarationStatus::kOk);
}

TEST(CapabilityDescriptorTest, DefaultDescriptorIsEmptyAndOptIn) {
  const EmbodimentDescriptor descriptor;
  EXPECT_EQ(descriptor.SerializeAsString(), "");
  EXPECT_EQ(descriptor.resource_id(), "");
  EXPECT_EQ(descriptor.capabilities_size(), 0);
  EmbodimentDescriptor parsed;
  ASSERT_TRUE(parsed.ParseFromString(""));
  EXPECT_EQ(parsed.SerializeAsString(), "");
  const std::vector<CapabilityDeclarationView> none;
  EXPECT_EQ(ValidateDeclarations(none).status, DeclarationStatus::kOk);
  EXPECT_EQ(FindCapability(none, "joint_position"), nullptr);
  EXPECT_EQ(FindCapability(none, "BodyState"), nullptr);
  EXPECT_FALSE(IsManipulatorCapabilityId("BodyState"));
}

TEST(CapabilityDescriptorTest, UnknownFieldIsPreserved) {
  const std::string golden = FromHex(kExampleGoldenHex);
  const std::string with_unknown = golden + std::string("\xA0\x06\x07", 3);
  EmbodimentDescriptor parsed;
  ASSERT_TRUE(parsed.ParseFromString(with_unknown));
  EXPECT_EQ(parsed.resource_id(), "example");
  ASSERT_EQ(parsed.capabilities_size(), 1);
  EXPECT_EQ(parsed.capabilities(0).id(), "joint_position");
  EXPECT_EQ(parsed.capabilities(0).category(),
            CapabilityCategory::CAPABILITY_CATEGORY_COMMAND);
  EXPECT_EQ(parsed.capabilities(0).interface_name(), "JointPosition");
  const google::protobuf::UnknownFieldSet& unknown =
      parsed.GetReflection()->GetUnknownFields(parsed);
  ASSERT_EQ(unknown.field_count(), 1);
  EXPECT_EQ(unknown.field(0).number(), 100);
  EXPECT_EQ(unknown.field(0).varint(), 7u);
  EXPECT_EQ(parsed.SerializeAsString(), with_unknown);
  EXPECT_EQ(ValidateDeclarations(ViewsOf(parsed)).status,
            DeclarationStatus::kOk);

  CapabilityDeclaration declaration;
  declaration.set_id("joint_position");
  declaration.set_category(CapabilityCategory::CAPABILITY_CATEGORY_COMMAND);
  declaration.set_interface_name("JointPosition");
  const std::string declaration_unknown =
      declaration.SerializeAsString() + std::string("\xA0\x06\x07", 3);
  CapabilityDeclaration parsed_declaration;
  ASSERT_TRUE(parsed_declaration.ParseFromString(declaration_unknown));
  EXPECT_EQ(parsed_declaration.id(), "joint_position");
  EXPECT_EQ(parsed_declaration.SerializeAsString(), declaration_unknown);
}

TEST(CapabilityDescriptorTest, UnknownCategoryAndUnknownIdArePreserved) {
  const std::string unknown_category = FromHex(kUnknownCategoryHex);
  CapabilityDeclaration parsed;
  ASSERT_TRUE(parsed.ParseFromString(unknown_category));
  EXPECT_EQ(parsed.id(), "future.range_observation");
  EXPECT_EQ(static_cast<int>(parsed.category()), 100);
  EXPECT_EQ(parsed.interface_name(), "RangeObservation");
  EXPECT_EQ(parsed.SerializeAsString(), unknown_category);
  EXPECT_TRUE(CategoryStableId(static_cast<int>(parsed.category())).empty());
  EXPECT_FALSE(IsKnownCategory(static_cast<int>(parsed.category())));
  EXPECT_FALSE(IsManipulatorCapabilityId(parsed.id()));

  const CapabilityDeclarationView unknown_view{
      parsed.id(), static_cast<int>(parsed.category()),
      parsed.interface_name()};
  const DeclarationProblem accepted = ValidateDeclarations({unknown_view});
  EXPECT_EQ(accepted.status, DeclarationStatus::kOk);

  CapabilityDeclaration known_category;
  known_category.set_id("future.range_observation");
  known_category.set_category(CapabilityCategory::CAPABILITY_CATEGORY_SENSOR);
  known_category.set_interface_name("RangeObservation");
  const std::string known_bytes = known_category.SerializeAsString();
  CapabilityDeclaration round_trip;
  ASSERT_TRUE(round_trip.ParseFromString(known_bytes));
  EXPECT_EQ(round_trip.SerializeAsString(), known_bytes);
  EXPECT_FALSE(IsManipulatorCapabilityId(round_trip.id()));
  const CapabilityDeclarationView known_view{
      round_trip.id(), static_cast<int>(round_trip.category()),
      round_trip.interface_name()};
  EXPECT_EQ(ValidateDeclarations({known_view}).status, DeclarationStatus::kOk);

  EmbodimentDescriptor manipulator;
  FillManipulatorDescriptor(&manipulator);
  const std::string before = manipulator.SerializeAsString();
  EXPECT_EQ(FindCapability(ViewsOf(manipulator), round_trip.id()), nullptr);
  EXPECT_EQ(manipulator.SerializeAsString(), before);
}

TEST(CapabilityDescriptorTest, RejectsDuplicateAndConflictWithoutRewriting) {
  const CapabilityDeclarationView joint{
      "joint_position", kCapabilityCategoryCommand, "JointPosition"};
  const DeclarationProblem duplicate = ValidateDeclarations({joint, joint});
  EXPECT_EQ(duplicate.status, DeclarationStatus::kDuplicateId);
  EXPECT_EQ(duplicate.index, 1u);

  const CapabilityDeclarationView third = joint;
  const DeclarationProblem first_duplicate =
      ValidateDeclarations({joint, third, joint});
  EXPECT_EQ(first_duplicate.status, DeclarationStatus::kDuplicateId);
  EXPECT_EQ(first_duplicate.index, 1u);

  const CapabilityDeclarationView other_category{
      "joint_position", kCapabilityCategorySensor, "JointPosition"};
  const DeclarationProblem category_conflict =
      ValidateDeclarations({joint, other_category});
  EXPECT_EQ(category_conflict.status,
            DeclarationStatus::kConflictingDeclaration);
  EXPECT_EQ(category_conflict.index, 1u);

  const CapabilityDeclarationView other_interface{
      "joint_position", kCapabilityCategoryCommand, "JointVelocity"};
  const DeclarationProblem interface_conflict =
      ValidateDeclarations({joint, other_interface});
  EXPECT_EQ(interface_conflict.status,
            DeclarationStatus::kConflictingDeclaration);
  EXPECT_EQ(interface_conflict.index, 1u);

  const CapabilityDeclarationView alias{
      "joint_position_alias", kCapabilityCategoryCommand, "JointPosition"};
  EXPECT_EQ(ValidateDeclarations({joint, alias}).status,
            DeclarationStatus::kOk);

  const CapabilityDeclarationView empty_interface{"custom_state",
                                                  kCapabilityCategoryState, ""};
  EXPECT_EQ(ValidateDeclarations({empty_interface}).status,
            DeclarationStatus::kOk);

  const CapabilityDeclarationView empty_id{"", kCapabilityCategoryState,
                                           "JointPosition"};
  const DeclarationProblem empty = ValidateDeclarations({joint, empty_id});
  EXPECT_EQ(empty.status, DeclarationStatus::kEmptyId);
  EXPECT_EQ(empty.index, 1u);

  const CapabilityDeclarationView unspecified{
      "custom_state", kCapabilityCategoryUnspecified, "Custom"};
  const DeclarationProblem unspecified_problem =
      ValidateDeclarations({unspecified, joint});
  EXPECT_EQ(unspecified_problem.status,
            DeclarationStatus::kUnspecifiedCategory);
  EXPECT_EQ(unspecified_problem.index, 0u);

  const CapabilityDeclarationView unknown{"future.range_observation", 100,
                                          "RangeObservation"};
  const DeclarationProblem unknown_duplicate =
      ValidateDeclarations({unknown, unknown});
  EXPECT_EQ(unknown_duplicate.status, DeclarationStatus::kDuplicateId);
  EXPECT_EQ(unknown_duplicate.index, 1u);
  const CapabilityDeclarationView unknown_other{unknown.id, 101,
                                                unknown.interface_name};
  EXPECT_EQ(ValidateDeclarations({unknown, unknown_other}).status,
            DeclarationStatus::kConflictingDeclaration);

  EmbodimentDescriptor descriptor;
  FillManipulatorDescriptor(&descriptor);
  auto* extra = descriptor.add_capabilities();
  extra->CopyFrom(descriptor.capabilities(0));
  const std::string with_unknown =
      descriptor.SerializeAsString() + std::string("\xA0\x06\x07", 3);
  EmbodimentDescriptor parsed;
  ASSERT_TRUE(parsed.ParseFromString(with_unknown));
  const DeclarationProblem problem = ValidateDeclarations(ViewsOf(parsed));
  EXPECT_EQ(problem.status, DeclarationStatus::kDuplicateId);
  EXPECT_EQ(parsed.SerializeAsString(), with_unknown);
  EXPECT_EQ(parsed.resource_id(), kManipulatorResourceId);
}

TEST(CapabilityDescriptorTest, ResourceIdIsNotAValidationSwitch) {
  EmbodimentDescriptor manipulator;
  FillManipulatorDescriptor(&manipulator);
  EmbodimentDescriptor other;
  FillManipulatorDescriptor(&other);
  other.set_resource_id("other-resource");
  EXPECT_EQ(ValidateDeclarations(ViewsOf(manipulator)).status,
            DeclarationStatus::kOk);
  EXPECT_EQ(ValidateDeclarations(ViewsOf(other)).status,
            DeclarationStatus::kOk);
  EXPECT_NE(manipulator.SerializeAsString(), other.SerializeAsString());
  EXPECT_EQ(other.capabilities_size(), manipulator.capabilities_size());
}

TEST(CapabilityDescriptorTest, ExampleGoldenRoundTrip) {
  const std::string golden = FromHex(kExampleGoldenHex);
  EmbodimentDescriptor parsed;
  ASSERT_TRUE(parsed.ParseFromString(golden));
  EXPECT_EQ(parsed.resource_id(), "example");
  ASSERT_EQ(parsed.capabilities_size(), 1);
  EXPECT_EQ(parsed.capabilities(0).id(), "joint_position");
  EXPECT_EQ(parsed.capabilities(0).category(),
            CapabilityCategory::CAPABILITY_CATEGORY_COMMAND);
  EXPECT_EQ(parsed.capabilities(0).interface_name(), "JointPosition");
  EXPECT_EQ(parsed.SerializeAsString(), golden);
  EXPECT_NE(parsed.resource_id(), kManipulatorResourceId);
}

}  // namespace
}  // namespace intrinsic::embodiment
