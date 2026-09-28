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

"""Declaration checks for the embodiment capability descriptor."""

import unittest

from intrinsic.embodiment import capability_policy

_UNKNOWN = "ai.intrinsic.capability.extension"
_VEHICLE = "ai.intrinsic.capability.vehicle"


class CapabilityDescriptorTest(unittest.TestCase):

  def test_rejects_duplicate_declarations(self):
    assessment = capability_policy.assess_declarations(
        (
            (capability_policy.CAPABILITY_COMMAND, "binding.a"),
            (capability_policy.CAPABILITY_COMMAND, "binding.a"),
        )
    )
    self.assertIs(
        assessment.error, capability_policy.DeclarationError.DUPLICATE
    )
    self.assertEqual(
        assessment.capability_id, capability_policy.CAPABILITY_COMMAND
    )

  def test_rejects_conflicting_interface_bindings(self):
    assessment = capability_policy.assess_declarations(
        (
            (capability_policy.CAPABILITY_STATE, ""),
            (capability_policy.CAPABILITY_STATE, "binding.b"),
        )
    )
    self.assertIs(assessment.error, capability_policy.DeclarationError.CONFLICT)
    self.assertEqual(
        assessment.capability_id, capability_policy.CAPABILITY_STATE
    )

  def test_conflict_outranks_duplicate(self):
    assessment = capability_policy.assess_declarations(
        (
            (capability_policy.CAPABILITY_SENSOR, "binding.a"),
            (capability_policy.CAPABILITY_SENSOR, "binding.a"),
            (capability_policy.CAPABILITY_SENSOR, "binding.b"),
        )
    )
    self.assertIs(assessment.error, capability_policy.DeclarationError.CONFLICT)
    self.assertEqual(
        assessment.capability_id, capability_policy.CAPABILITY_SENSOR
    )

  def test_rejects_empty_capability_id(self):
    assessment = capability_policy.assess_declarations(
        (
            (capability_policy.CAPABILITY_ACTUATOR, ""),
            ("", ""),
        )
    )
    self.assertIs(assessment.error, capability_policy.DeclarationError.EMPTY_ID)
    self.assertEqual(assessment.capability_id, "")

  def test_preserves_unknown_capability_id(self):
    declarations = (
        (capability_policy.CAPABILITY_PLANNING, ""),
        (_UNKNOWN, ""),
    )
    assessment = capability_policy.assess_declarations(declarations)
    self.assertIs(assessment.error, capability_policy.DeclarationError.NONE)
    self.assertFalse(capability_policy.is_well_known_capability_id(_UNKNOWN))
    self.assertTrue(
        capability_policy.declares_capability(declarations, _UNKNOWN)
    )
    self.assertTrue(
        capability_policy.is_well_known_capability_id(
            capability_policy.CAPABILITY_PLANNING
        )
    )

  def test_rejects_duplicate_unknown_capability_id(self):
    assessment = capability_policy.assess_declarations(
        (
            (_UNKNOWN, ""),
            (_UNKNOWN, ""),
        )
    )
    self.assertIs(
        assessment.error, capability_policy.DeclarationError.DUPLICATE
    )
    self.assertEqual(assessment.capability_id, _UNKNOWN)

  def test_manipulator_descriptor_lists_stable_ids(self):
    declarations = capability_policy.MANIPULATOR_CAPABILITY_DECLARATIONS
    self.assertIs(
        capability_policy.assess_declarations(declarations).error,
        capability_policy.DeclarationError.NONE,
    )
    self.assertEqual(
        declarations,
        tuple(
            (capability_id, "")
            for capability_id in capability_policy.WELL_KNOWN_CAPABILITY_IDS
        ),
    )
    self.assertFalse(
        capability_policy.is_well_known_capability_id(
            capability_policy.MANIPULATOR_RESOURCE_ID
        )
    )
    self.assertFalse(
        capability_policy.declares_capability(
            declarations, capability_policy.MANIPULATOR_RESOURCE_ID
        )
    )

  def test_missing_vehicle_capability_leaves_manipulator_descriptor_valid(self):
    declarations = capability_policy.MANIPULATOR_CAPABILITY_DECLARATIONS
    self.assertIs(
        capability_policy.assess_declarations(declarations).error,
        capability_policy.DeclarationError.NONE,
    )
    self.assertFalse(capability_policy.is_well_known_capability_id(_VEHICLE))
    self.assertFalse(
        capability_policy.declares_capability(declarations, _VEHICLE)
    )
    for capability_id in capability_policy.WELL_KNOWN_CAPABILITY_IDS:
      self.assertTrue(
          capability_policy.declares_capability(declarations, capability_id)
      )

  def test_empty_declarations_are_prior_behavior(self):
    assessment = capability_policy.assess_declarations(())
    self.assertIs(assessment.error, capability_policy.DeclarationError.NONE)
    self.assertEqual(assessment.capability_id, "")
    self.assertFalse(
        capability_policy.declares_capability(
            (), capability_policy.CAPABILITY_STATE
        )
    )
    self.assertFalse(
        capability_policy.declares_capability(
            (), capability_policy.CAPABILITY_SIMULATION
        )
    )


if __name__ == "__main__":
  unittest.main()
