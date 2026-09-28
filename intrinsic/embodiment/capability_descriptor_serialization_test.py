# Copyright 2026 Intrinsic Innovation LLC
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     https://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Python serialization tests for CapabilityDescriptor."""

import unittest

from intrinsic.embodiment import capability_policy
from intrinsic.embodiment.proto import capability_descriptor_pb2

# Canonical serialization of _fill_manipulator(). Keep in sync with
# capability_descriptor_serialization_test.cc.
_MANIPULATOR_GOLDEN_HEX = (
    "0a2e61692e696e7472696e7369632e636f6d7061746962696c6974795f70726f66696c652e"
    "6d616e6970756c61746f72121f0a1d61692e696e7472696e7369632e6361706162696c6974"
    "792e737461746512210a1f61692e696e7472696e7369632e6361706162696c6974792e636f"
    "6d6d616e6412200a1e61692e696e7472696e7369632e6361706162696c6974792e73656e73"
    "6f7212220a2061692e696e7472696e7369632e6361706162696c6974792e6163747561746f"
    "7212220a2061692e696e7472696e7369632e6361706162696c6974792e706c616e6e696e67"
    "12240a2261692e696e7472696e7369632e6361706162696c6974792e73696d756c6174696f"
    "6e"
)

# One extra CapabilityDeclaration for ai.intrinsic.capability.extension.
_EXTENSION_DECLARATION_HEX = (
    "12230a2161692e696e7472696e7369632e6361706162696c6974792e657874656e73696f6e"
)
_EXTENSION_ID = "ai.intrinsic.capability.extension"


def _fill_manipulator():
  descriptor = capability_descriptor_pb2.CapabilityDescriptor()
  descriptor.resource_id = capability_policy.MANIPULATOR_RESOURCE_ID
  for (
      capability_id,
      interface_id,
  ) in capability_policy.MANIPULATOR_CAPABILITY_DECLARATIONS:
    declaration = descriptor.capabilities.add()
    declaration.id = capability_id
    if interface_id:
      declaration.interface_id = interface_id
  return descriptor


def _views(descriptor):
  return tuple(
      (capability.id, capability.interface_id)
      for capability in descriptor.capabilities
  )


class CapabilityDescriptorSerializationTest(unittest.TestCase):

  def test_manipulator_golden_round_trip(self):
    descriptor = _fill_manipulator()
    golden = bytes.fromhex(_MANIPULATOR_GOLDEN_HEX)
    self.assertEqual(descriptor.SerializeToString(), golden)
    parsed = capability_descriptor_pb2.CapabilityDescriptor()
    parsed.ParseFromString(golden)
    self.assertEqual(
        parsed.resource_id, capability_policy.MANIPULATOR_RESOURCE_ID
    )
    self.assertEqual(len(parsed.capabilities), 6)
    for index, (capability_id, interface_id) in enumerate(
        capability_policy.MANIPULATOR_CAPABILITY_DECLARATIONS
    ):
      self.assertEqual(parsed.capabilities[index].id, capability_id)
      self.assertEqual(parsed.capabilities[index].interface_id, interface_id)
    self.assertIs(
        capability_policy.assess_declarations(_views(parsed)).error,
        capability_policy.DeclarationError.NONE,
    )
    self.assertEqual(parsed.SerializeToString(), golden)

  def test_default_descriptor_is_empty_and_opt_in(self):
    descriptor = capability_descriptor_pb2.CapabilityDescriptor()
    self.assertEqual(descriptor.SerializeToString(), b"")
    self.assertEqual(descriptor.resource_id, "")
    self.assertEqual(len(descriptor.capabilities), 0)
    parsed = capability_descriptor_pb2.CapabilityDescriptor()
    parsed.ParseFromString(b"")
    self.assertEqual(parsed.SerializeToString(), b"")
    self.assertIs(
        capability_policy.assess_declarations(_views(parsed)).error,
        capability_policy.DeclarationError.NONE,
    )
    self.assertFalse(
        capability_policy.declares_capability(
            _views(parsed), capability_policy.CAPABILITY_STATE
        )
    )

  def test_unknown_field_is_preserved(self):
    golden = bytes.fromhex(_MANIPULATOR_GOLDEN_HEX)
    with_unknown = golden + bytes((0xA0, 0x06, 0x07))
    parsed = capability_descriptor_pb2.CapabilityDescriptor()
    parsed.ParseFromString(with_unknown)
    self.assertEqual(
        parsed.resource_id, capability_policy.MANIPULATOR_RESOURCE_ID
    )
    self.assertEqual(len(parsed.capabilities), 6)
    self.assertEqual(parsed.SerializeToString(), with_unknown)

  def test_unknown_capability_id_is_prefix_preserved(self):
    golden = bytes.fromhex(_MANIPULATOR_GOLDEN_HEX)
    with_extension = golden + bytes.fromhex(_EXTENSION_DECLARATION_HEX)
    parsed = capability_descriptor_pb2.CapabilityDescriptor()
    parsed.ParseFromString(with_extension)
    self.assertEqual(len(parsed.capabilities), 7)
    self.assertEqual(parsed.capabilities[6].id, _EXTENSION_ID)
    self.assertFalse(
        capability_policy.is_well_known_capability_id(_EXTENSION_ID)
    )
    self.assertEqual(
        parsed.capabilities[0].id, capability_policy.CAPABILITY_STATE
    )
    views = _views(parsed)
    self.assertIs(
        capability_policy.assess_declarations(views).error,
        capability_policy.DeclarationError.NONE,
    )
    self.assertTrue(capability_policy.declares_capability(views, _EXTENSION_ID))
    self.assertTrue(with_extension.startswith(golden))
    self.assertEqual(parsed.SerializeToString(), with_extension)

  def test_duplicate_declarations_round_trip_and_are_rejected(self):
    descriptor = capability_descriptor_pb2.CapabilityDescriptor()
    descriptor.resource_id = "resource"
    descriptor.capabilities.add().id = capability_policy.CAPABILITY_ACTUATOR
    descriptor.capabilities.add().id = capability_policy.CAPABILITY_ACTUATOR
    payload = descriptor.SerializeToString()
    parsed = capability_descriptor_pb2.CapabilityDescriptor()
    parsed.ParseFromString(payload)
    self.assertEqual(parsed.SerializeToString(), payload)
    assessment = capability_policy.assess_declarations(_views(parsed))
    self.assertIs(
        assessment.error, capability_policy.DeclarationError.DUPLICATE
    )
    self.assertEqual(
        assessment.capability_id, capability_policy.CAPABILITY_ACTUATOR
    )


if __name__ == "__main__":
  unittest.main()
