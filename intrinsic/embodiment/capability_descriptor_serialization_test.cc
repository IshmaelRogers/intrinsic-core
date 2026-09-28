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

#include <cstddef>
#include <string>
#include <string_view>
#include <vector>

#include "google/protobuf/unknown_field_set.h"
#include "gtest/gtest.h"
#include "intrinsic/embodiment/capability_policy.h"
#include "intrinsic/embodiment/proto/capability_descriptor.pb.h"

namespace intrinsic::embodiment {
namespace {

using intrinsic_proto::embodiment::CapabilityDescriptor;

// Canonical serialization of FillManipulator(). Keep in sync with
// capability_descriptor_serialization_test.py.
constexpr std::string_view kManipulatorGoldenHex =
    "0a2e61692e696e7472696e7369632e636f6d7061746962696c6974795f70726f66696c652e"
    "6d616e6970756c61746f72121f0a1d61692e696e7472696e7369632e6361706162696c6974"
    "792e737461746512210a1f61692e696e7472696e7369632e6361706162696c6974792e636f"
    "6d6d616e6412200a1e61692e696e7472696e7369632e6361706162696c6974792e73656e73"
    "6f7212220a2061692e696e7472696e7369632e6361706162696c6974792e6163747561746f"
    "7212220a2061692e696e7472696e7369632e6361706162696c6974792e706c616e6e696e67"
    "12240a2261692e696e7472696e7369632e6361706162696c6974792e73696d756c6174696f"
    "6e";

// One extra CapabilityDeclaration for ai.intrinsic.capability.extension.
// Appended, so the manipulator golden stays a prefix.
constexpr std::string_view kExtensionDeclarationHex =
    "12230a2161692e696e7472696e7369632e6361706162696c6974792e657874656e73696f6"
    "e";

constexpr std::string_view kExtensionId = "ai.intrinsic.capability.extension";

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

void FillManipulator(CapabilityDescriptor* descriptor) {
  descriptor->set_resource_id(std::string(kManipulatorResourceId));
  for (const CapabilityDeclarationView& declaration :
       ManipulatorCapabilityDeclarations()) {
    auto* capability = descriptor->add_capabilities();
    capability->set_id(std::string(declaration.id));
  }
}

std::vector<CapabilityDeclarationView> ViewsOf(
    const CapabilityDescriptor& descriptor) {
  std::vector<CapabilityDeclarationView> views;
  views.reserve(descriptor.capabilities_size());
  for (const auto& capability : descriptor.capabilities()) {
    views.push_back(
        CapabilityDeclarationView{capability.id(), capability.interface_id()});
  }
  return views;
}

TEST(CapabilityDescriptorSerializationTest, ManipulatorGoldenRoundTrip) {
  CapabilityDescriptor descriptor;
  FillManipulator(&descriptor);
  const std::string golden = FromHex(kManipulatorGoldenHex);
  EXPECT_EQ(descriptor.SerializeAsString(), golden);

  CapabilityDescriptor parsed;
  ASSERT_TRUE(parsed.ParseFromString(golden));
  EXPECT_EQ(parsed.resource_id(), kManipulatorResourceId);
  ASSERT_EQ(parsed.capabilities_size(), 6);
  const auto declarations = ManipulatorCapabilityDeclarations();
  for (int i = 0; i < parsed.capabilities_size(); ++i) {
    EXPECT_EQ(parsed.capabilities(i).id(), declarations[i].id);
    EXPECT_EQ(parsed.capabilities(i).interface_id(), "");
  }
  const std::vector<CapabilityDeclarationView> views = ViewsOf(parsed);
  EXPECT_EQ(AssessCapabilityDeclarations(views).error, DeclarationError::kNone);
  EXPECT_EQ(parsed.SerializeAsString(), golden);
}

TEST(CapabilityDescriptorSerializationTest, DefaultDescriptorIsEmptyAndOptIn) {
  const CapabilityDescriptor descriptor;
  EXPECT_EQ(descriptor.SerializeAsString(), "");
  EXPECT_EQ(descriptor.resource_id(), "");
  EXPECT_EQ(descriptor.capabilities_size(), 0);
  CapabilityDescriptor parsed;
  ASSERT_TRUE(parsed.ParseFromString(""));
  EXPECT_EQ(parsed.SerializeAsString(), "");
  EXPECT_EQ(AssessCapabilityDeclarations(ViewsOf(parsed)).error,
            DeclarationError::kNone);
  EXPECT_FALSE(DeclaresCapability(ViewsOf(parsed), kCapabilityState));
}

TEST(CapabilityDescriptorSerializationTest, UnknownFieldIsPreserved) {
  const std::string golden = FromHex(kManipulatorGoldenHex);
  const std::string with_unknown = golden + std::string("\xA0\x06\x07", 3);
  CapabilityDescriptor parsed;
  ASSERT_TRUE(parsed.ParseFromString(with_unknown));
  EXPECT_EQ(parsed.resource_id(), kManipulatorResourceId);
  EXPECT_EQ(parsed.capabilities_size(), 6);
  const google::protobuf::UnknownFieldSet& unknown =
      parsed.GetReflection()->GetUnknownFields(parsed);
  ASSERT_EQ(unknown.field_count(), 1);
  EXPECT_EQ(unknown.field(0).number(), 100);
  EXPECT_EQ(unknown.field(0).varint(), 7u);
  EXPECT_EQ(parsed.SerializeAsString(), with_unknown);
}

TEST(CapabilityDescriptorSerializationTest,
     UnknownCapabilityIdIsPrefixPreserved) {
  const std::string golden = FromHex(kManipulatorGoldenHex);
  const std::string with_extension = golden + FromHex(kExtensionDeclarationHex);
  CapabilityDescriptor parsed;
  ASSERT_TRUE(parsed.ParseFromString(with_extension));
  ASSERT_EQ(parsed.capabilities_size(), 7);
  EXPECT_EQ(parsed.capabilities(6).id(), kExtensionId);
  EXPECT_FALSE(IsWellKnownCapabilityId(parsed.capabilities(6).id()));
  EXPECT_EQ(parsed.capabilities(0).id(), kCapabilityState);
  const std::vector<CapabilityDeclarationView> views = ViewsOf(parsed);
  EXPECT_EQ(AssessCapabilityDeclarations(views).error, DeclarationError::kNone);
  EXPECT_TRUE(DeclaresCapability(views, kExtensionId));
  EXPECT_TRUE(with_extension.starts_with(golden));
  EXPECT_EQ(parsed.SerializeAsString(), with_extension);
}

TEST(CapabilityDescriptorSerializationTest,
     DuplicateDeclarationsRoundTripAndAreRejected) {
  CapabilityDescriptor descriptor;
  descriptor.set_resource_id("resource");
  descriptor.add_capabilities()->set_id(std::string(kCapabilityActuator));
  descriptor.add_capabilities()->set_id(std::string(kCapabilityActuator));
  const std::string bytes = descriptor.SerializeAsString();
  CapabilityDescriptor parsed;
  ASSERT_TRUE(parsed.ParseFromString(bytes));
  EXPECT_EQ(parsed.SerializeAsString(), bytes);
  const std::vector<CapabilityDeclarationView> views = ViewsOf(parsed);
  const DeclarationAssessment assessment = AssessCapabilityDeclarations(views);
  EXPECT_EQ(assessment.error, DeclarationError::kDuplicate);
  EXPECT_EQ(assessment.capability_id, kCapabilityActuator);
}

}  // namespace
}  // namespace intrinsic::embodiment
