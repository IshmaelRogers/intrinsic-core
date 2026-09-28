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

"""Python tests for the embodiment capability descriptor."""

import unittest

from intrinsic.embodiment import capability_descriptor
from intrinsic.embodiment.proto import capability_descriptor_pb2

# Canonical bytes of manipulator_descriptor(). Keep in sync with
# capability_descriptor_test.cc.
_MANIPULATOR_GOLDEN_HEX = (
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
    "636512180a0973696d756c61746f7210061a0953696d756c61746f72"
)

_EXAMPLE_GOLDEN_HEX = (
    "0a076578616d706c6512210a0e6a6f696e745f706f736974696f6e10021a0d4a6f69"
    "6e74506f736974696f6e"
)

_UNKNOWN_CATEGORY_HEX = (
    "0a186675747572652e72616e67655f6f62736572766174696f6e10641a1052616e67"
    "654f62736572766174696f6e"
)

_ABSENT_IDS = (
    "BodyState",
    "BodyWrenchCommand",
    "VehicleLimits",
    "StateSpace",
    "DynamicsModel",
    "RangeObservation",
    "SafetyRule",
    "VehicleState",
    "DesiredMotion",
    "body_state",
    "body_wrench_command",
    "vehicle_limits",
    "dynamics_model",
    "range_observation",
    "safety_rule",
    "vehicle_state",
    "desired_motion",
)

_CATEGORY = capability_descriptor_pb2.CapabilityCategory


def _covers_known_categories(declarations):
  seen = set()
  for _, category, _ in declarations:
    if capability_descriptor.is_known_category(category):
      seen.add(category)
  return seen == {
      capability_descriptor.CAPABILITY_CATEGORY_STATE,
      capability_descriptor.CAPABILITY_CATEGORY_COMMAND,
      capability_descriptor.CAPABILITY_CATEGORY_SENSOR,
      capability_descriptor.CAPABILITY_CATEGORY_ACTUATOR,
      capability_descriptor.CAPABILITY_CATEGORY_PLANNING,
      capability_descriptor.CAPABILITY_CATEGORY_SIMULATION,
  }


class CapabilityDescriptorTest(unittest.TestCase):

  def test_category_numbers_and_stable_ids(self):
    self.assertEqual(
        _CATEGORY.CAPABILITY_CATEGORY_UNSPECIFIED,
        capability_descriptor.CAPABILITY_CATEGORY_UNSPECIFIED,
    )
    self.assertEqual(
        _CATEGORY.CAPABILITY_CATEGORY_STATE,
        capability_descriptor.CAPABILITY_CATEGORY_STATE,
    )
    self.assertEqual(
        _CATEGORY.CAPABILITY_CATEGORY_COMMAND,
        capability_descriptor.CAPABILITY_CATEGORY_COMMAND,
    )
    self.assertEqual(
        _CATEGORY.CAPABILITY_CATEGORY_SENSOR,
        capability_descriptor.CAPABILITY_CATEGORY_SENSOR,
    )
    self.assertEqual(
        _CATEGORY.CAPABILITY_CATEGORY_ACTUATOR,
        capability_descriptor.CAPABILITY_CATEGORY_ACTUATOR,
    )
    self.assertEqual(
        _CATEGORY.CAPABILITY_CATEGORY_PLANNING,
        capability_descriptor.CAPABILITY_CATEGORY_PLANNING,
    )
    self.assertEqual(
        _CATEGORY.CAPABILITY_CATEGORY_SIMULATION,
        capability_descriptor.CAPABILITY_CATEGORY_SIMULATION,
    )
    self.assertEqual(
        capability_descriptor.category_stable_id(
            capability_descriptor.CAPABILITY_CATEGORY_STATE
        ),
        "state",
    )
    self.assertEqual(
        capability_descriptor.category_stable_id(
            capability_descriptor.CAPABILITY_CATEGORY_COMMAND
        ),
        "command",
    )
    self.assertEqual(
        capability_descriptor.category_stable_id(
            capability_descriptor.CAPABILITY_CATEGORY_SENSOR
        ),
        "sensor",
    )
    self.assertEqual(
        capability_descriptor.category_stable_id(
            capability_descriptor.CAPABILITY_CATEGORY_ACTUATOR
        ),
        "actuator",
    )
    self.assertEqual(
        capability_descriptor.category_stable_id(
            capability_descriptor.CAPABILITY_CATEGORY_PLANNING
        ),
        "planning",
    )
    self.assertEqual(
        capability_descriptor.category_stable_id(
            capability_descriptor.CAPABILITY_CATEGORY_SIMULATION
        ),
        "simulation",
    )
    self.assertEqual(capability_descriptor.category_stable_id(0), "")
    self.assertEqual(capability_descriptor.category_stable_id(100), "")
    self.assertTrue(capability_descriptor.is_unspecified_category(0))
    self.assertFalse(capability_descriptor.is_unspecified_category(100))
    self.assertFalse(capability_descriptor.is_known_category(0))
    self.assertFalse(capability_descriptor.is_known_category(100))
    self.assertEqual(len(_CATEGORY.values()), 7)

  def test_schema_has_no_robot_type_field(self):
    descriptor = capability_descriptor_pb2.EmbodimentDescriptor.DESCRIPTOR
    self.assertEqual(
        [field.name for field in descriptor.fields],
        ["resource_id", "capabilities"],
    )
    declaration = capability_descriptor_pb2.CapabilityDeclaration.DESCRIPTOR
    self.assertEqual(
        [field.name for field in declaration.fields],
        ["id", "category", "interface_name"],
    )

  def test_manipulator_golden_round_trip(self):
    descriptor = capability_descriptor.manipulator_descriptor()
    golden = bytes.fromhex(_MANIPULATOR_GOLDEN_HEX)
    self.assertEqual(descriptor.SerializeToString(), golden)
    parsed = capability_descriptor_pb2.EmbodimentDescriptor()
    parsed.ParseFromString(golden)
    self.assertEqual(parsed.resource_id, "manipulator")
    self.assertEqual(
        len(parsed.capabilities),
        len(capability_descriptor.MANIPULATOR_CAPABILITIES),
    )
    for got, want in zip(
        parsed.capabilities, capability_descriptor.MANIPULATOR_CAPABILITIES
    ):
      self.assertEqual(got.id, want[0])
      self.assertEqual(int(got.category), want[1])
      self.assertEqual(got.interface_name, want[2])
      self.assertNotEqual(
          got.id, capability_descriptor.category_stable_id(want[1])
      )
    self.assertEqual(parsed.SerializeToString(), golden)
    self.assertEqual(
        capability_descriptor.manipulator_descriptor().SerializeToString(),
        golden,
    )
    views = capability_descriptor.views_of(parsed)
    status, index = capability_descriptor.validate_declarations(views)
    self.assertIs(status, capability_descriptor.DeclarationStatus.OK)
    self.assertEqual(index, 0)
    self.assertTrue(_covers_known_categories(views))
    joint = capability_descriptor.find_capability(views, "joint_position")
    self.assertEqual(
        joint,
        (
            "joint_position",
            capability_descriptor.CAPABILITY_CATEGORY_COMMAND,
            "JointPosition",
        ),
    )
    dynamics = capability_descriptor.find_capability(views, "dynamics")
    self.assertEqual(
        dynamics,
        (
            "dynamics",
            capability_descriptor.CAPABILITY_CATEGORY_PLANNING,
            "Dynamics",
        ),
    )
    self.assertTrue(
        capability_descriptor.is_manipulator_capability_id("joint_position")
    )
    self.assertTrue(
        capability_descriptor.is_manipulator_capability_id("simulator")
    )

  def test_missing_vehicle_capability_leaves_manipulator_bytes(self):
    descriptor = capability_descriptor.manipulator_descriptor()
    before = descriptor.SerializeToString()
    views = capability_descriptor.views_of(descriptor)
    for absent in _ABSENT_IDS:
      self.assertIsNone(
          capability_descriptor.find_capability(views, absent), absent
      )
      self.assertFalse(
          capability_descriptor.is_manipulator_capability_id(absent), absent
      )
      for _, _, interface_name in views:
        self.assertNotEqual(interface_name, absent)
    self.assertEqual(descriptor.SerializeToString(), before)
    status, _ = capability_descriptor.validate_declarations(views)
    self.assertIs(status, capability_descriptor.DeclarationStatus.OK)

  def test_default_descriptor_is_empty_and_opt_in(self):
    descriptor = capability_descriptor_pb2.EmbodimentDescriptor()
    self.assertEqual(descriptor.SerializeToString(), b"")
    self.assertEqual(descriptor.resource_id, "")
    self.assertEqual(len(descriptor.capabilities), 0)
    parsed = capability_descriptor_pb2.EmbodimentDescriptor()
    parsed.ParseFromString(b"")
    self.assertEqual(parsed.SerializeToString(), b"")
    status, index = capability_descriptor.validate_declarations(())
    self.assertIs(status, capability_descriptor.DeclarationStatus.OK)
    self.assertEqual(index, 0)
    self.assertIsNone(
        capability_descriptor.find_capability((), "joint_position")
    )
    self.assertIsNone(capability_descriptor.find_capability((), "BodyState"))
    self.assertFalse(
        capability_descriptor.is_manipulator_capability_id("BodyState")
    )

  def test_unknown_field_is_preserved(self):
    golden = bytes.fromhex(_EXAMPLE_GOLDEN_HEX)
    with_unknown = golden + bytes((0xA0, 0x06, 0x07))
    parsed = capability_descriptor_pb2.EmbodimentDescriptor()
    parsed.ParseFromString(with_unknown)
    self.assertEqual(parsed.resource_id, "example")
    self.assertEqual(len(parsed.capabilities), 1)
    self.assertEqual(parsed.capabilities[0].id, "joint_position")
    self.assertEqual(
        parsed.capabilities[0].category, _CATEGORY.CAPABILITY_CATEGORY_COMMAND
    )
    self.assertEqual(parsed.capabilities[0].interface_name, "JointPosition")
    self.assertEqual(parsed.SerializeToString(), with_unknown)
    status, _ = capability_descriptor.validate_declarations(
        capability_descriptor.views_of(parsed)
    )
    self.assertIs(status, capability_descriptor.DeclarationStatus.OK)

    declaration = capability_descriptor_pb2.CapabilityDeclaration()
    declaration.id = "joint_position"
    declaration.category = _CATEGORY.CAPABILITY_CATEGORY_COMMAND
    declaration.interface_name = "JointPosition"
    declaration_unknown = declaration.SerializeToString() + bytes(
        (0xA0, 0x06, 0x07)
    )
    parsed_declaration = capability_descriptor_pb2.CapabilityDeclaration()
    parsed_declaration.ParseFromString(declaration_unknown)
    self.assertEqual(parsed_declaration.id, "joint_position")
    self.assertEqual(
        parsed_declaration.SerializeToString(), declaration_unknown
    )

  def test_unknown_category_and_unknown_id_are_preserved(self):
    unknown_category = bytes.fromhex(_UNKNOWN_CATEGORY_HEX)
    parsed = capability_descriptor_pb2.CapabilityDeclaration()
    parsed.ParseFromString(unknown_category)
    self.assertEqual(parsed.id, "future.range_observation")
    self.assertEqual(int(parsed.category), 100)
    self.assertEqual(parsed.interface_name, "RangeObservation")
    self.assertEqual(parsed.SerializeToString(), unknown_category)
    self.assertEqual(capability_descriptor.category_stable_id(100), "")
    self.assertFalse(capability_descriptor.is_known_category(100))
    self.assertFalse(
        capability_descriptor.is_manipulator_capability_id(parsed.id)
    )
    status, index = capability_descriptor.validate_declarations(
        [(parsed.id, int(parsed.category), parsed.interface_name)]
    )
    self.assertIs(status, capability_descriptor.DeclarationStatus.OK)
    self.assertEqual(index, 0)

    known_category = capability_descriptor_pb2.CapabilityDeclaration()
    known_category.id = "future.range_observation"
    known_category.category = _CATEGORY.CAPABILITY_CATEGORY_SENSOR
    known_category.interface_name = "RangeObservation"
    known_bytes = known_category.SerializeToString()
    round_trip = capability_descriptor_pb2.CapabilityDeclaration()
    round_trip.ParseFromString(known_bytes)
    self.assertEqual(round_trip.SerializeToString(), known_bytes)
    self.assertFalse(
        capability_descriptor.is_manipulator_capability_id(round_trip.id)
    )
    status, _ = capability_descriptor.validate_declarations(
        [(round_trip.id, int(round_trip.category), round_trip.interface_name)]
    )
    self.assertIs(status, capability_descriptor.DeclarationStatus.OK)

    manipulator = capability_descriptor.manipulator_descriptor()
    before = manipulator.SerializeToString()
    self.assertIsNone(
        capability_descriptor.find_capability(
            capability_descriptor.views_of(manipulator), round_trip.id
        )
    )
    self.assertEqual(manipulator.SerializeToString(), before)

  def test_rejects_duplicate_and_conflict_without_rewriting(self):
    joint = (
        "joint_position",
        capability_descriptor.CAPABILITY_CATEGORY_COMMAND,
        "JointPosition",
    )
    status, index = capability_descriptor.validate_declarations((joint, joint))
    self.assertIs(status, capability_descriptor.DeclarationStatus.DUPLICATE_ID)
    self.assertEqual(index, 1)

    status, index = capability_descriptor.validate_declarations(
        (joint, joint, joint)
    )
    self.assertIs(status, capability_descriptor.DeclarationStatus.DUPLICATE_ID)
    self.assertEqual(index, 1)

    other_category = (
        "joint_position",
        capability_descriptor.CAPABILITY_CATEGORY_SENSOR,
        "JointPosition",
    )
    status, index = capability_descriptor.validate_declarations(
        (joint, other_category)
    )
    self.assertIs(
        status, capability_descriptor.DeclarationStatus.CONFLICTING_DECLARATION
    )
    self.assertEqual(index, 1)

    other_interface = (
        "joint_position",
        capability_descriptor.CAPABILITY_CATEGORY_COMMAND,
        "JointVelocity",
    )
    status, index = capability_descriptor.validate_declarations(
        (joint, other_interface)
    )
    self.assertIs(
        status, capability_descriptor.DeclarationStatus.CONFLICTING_DECLARATION
    )
    self.assertEqual(index, 1)

    alias = (
        "joint_position_alias",
        capability_descriptor.CAPABILITY_CATEGORY_COMMAND,
        "JointPosition",
    )
    status, _ = capability_descriptor.validate_declarations((joint, alias))
    self.assertIs(status, capability_descriptor.DeclarationStatus.OK)

    empty_interface = (
        "custom_state",
        capability_descriptor.CAPABILITY_CATEGORY_STATE,
        "",
    )
    status, _ = capability_descriptor.validate_declarations((empty_interface,))
    self.assertIs(status, capability_descriptor.DeclarationStatus.OK)

    empty_id = (
        "",
        capability_descriptor.CAPABILITY_CATEGORY_STATE,
        "JointPosition",
    )
    status, index = capability_descriptor.validate_declarations(
        (joint, empty_id)
    )
    self.assertIs(status, capability_descriptor.DeclarationStatus.EMPTY_ID)
    self.assertEqual(index, 1)

    unspecified = (
        "custom_state",
        capability_descriptor.CAPABILITY_CATEGORY_UNSPECIFIED,
        "Custom",
    )
    status, index = capability_descriptor.validate_declarations(
        (unspecified, joint)
    )
    self.assertIs(
        status, capability_descriptor.DeclarationStatus.UNSPECIFIED_CATEGORY
    )
    self.assertEqual(index, 0)

    unknown = ("future.range_observation", 100, "RangeObservation")
    status, index = capability_descriptor.validate_declarations(
        (unknown, unknown)
    )
    self.assertIs(status, capability_descriptor.DeclarationStatus.DUPLICATE_ID)
    self.assertEqual(index, 1)
    status, _ = capability_descriptor.validate_declarations(
        (unknown, ("future.range_observation", 101, "RangeObservation"))
    )
    self.assertIs(
        status, capability_descriptor.DeclarationStatus.CONFLICTING_DECLARATION
    )

    descriptor = capability_descriptor.manipulator_descriptor()
    extra = descriptor.capabilities.add()
    extra.CopyFrom(descriptor.capabilities[0])
    with_unknown = descriptor.SerializeToString() + bytes((0xA0, 0x06, 0x07))
    parsed = capability_descriptor_pb2.EmbodimentDescriptor()
    parsed.ParseFromString(with_unknown)
    status, _ = capability_descriptor.validate_declarations(
        capability_descriptor.views_of(parsed)
    )
    self.assertIs(status, capability_descriptor.DeclarationStatus.DUPLICATE_ID)
    self.assertEqual(parsed.SerializeToString(), with_unknown)
    self.assertEqual(parsed.resource_id, "manipulator")

  def test_resource_id_is_not_a_validation_switch(self):
    manipulator = capability_descriptor.manipulator_descriptor()
    other = capability_descriptor.manipulator_descriptor()
    other.resource_id = "other-resource"
    manipulator_status, _ = capability_descriptor.validate_declarations(
        capability_descriptor.views_of(manipulator)
    )
    other_status, _ = capability_descriptor.validate_declarations(
        capability_descriptor.views_of(other)
    )
    self.assertIs(
        manipulator_status, capability_descriptor.DeclarationStatus.OK
    )
    self.assertIs(other_status, capability_descriptor.DeclarationStatus.OK)
    self.assertNotEqual(
        manipulator.SerializeToString(), other.SerializeToString()
    )
    self.assertEqual(len(other.capabilities), len(manipulator.capabilities))

  def test_example_golden_round_trip(self):
    golden = bytes.fromhex(_EXAMPLE_GOLDEN_HEX)
    parsed = capability_descriptor_pb2.EmbodimentDescriptor()
    parsed.ParseFromString(golden)
    self.assertEqual(parsed.resource_id, "example")
    self.assertEqual(len(parsed.capabilities), 1)
    self.assertEqual(parsed.capabilities[0].id, "joint_position")
    self.assertEqual(
        parsed.capabilities[0].category, _CATEGORY.CAPABILITY_CATEGORY_COMMAND
    )
    self.assertEqual(parsed.capabilities[0].interface_name, "JointPosition")
    self.assertEqual(parsed.SerializeToString(), golden)
    self.assertNotEqual(parsed.resource_id, "manipulator")


if __name__ == "__main__":
  unittest.main()
