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

"""Capability ids and declaration checks for CapabilityDescriptor.

Policy: intrinsic_apis/intrinsic/embodiment/proto/README.md.
"""

from collections.abc import Sequence
import enum
from typing import NamedTuple

CAPABILITY_STATE = "ai.intrinsic.capability.state"
CAPABILITY_COMMAND = "ai.intrinsic.capability.command"
CAPABILITY_SENSOR = "ai.intrinsic.capability.sensor"
CAPABILITY_ACTUATOR = "ai.intrinsic.capability.actuator"
CAPABILITY_PLANNING = "ai.intrinsic.capability.planning"
CAPABILITY_SIMULATION = "ai.intrinsic.capability.simulation"

WELL_KNOWN_CAPABILITY_IDS = (
    CAPABILITY_STATE,
    CAPABILITY_COMMAND,
    CAPABILITY_SENSOR,
    CAPABILITY_ACTUATOR,
    CAPABILITY_PLANNING,
    CAPABILITY_SIMULATION,
)

# Opaque fixture name for the existing manipulator compatibility profile.
# Callers must not select behavior from this string.
MANIPULATOR_RESOURCE_ID = "ai.intrinsic.compatibility_profile.manipulator"

# (capability id, interface id). Empty interface id means the capability id
# is the requested interface. Order matches WELL_KNOWN_CAPABILITY_IDS.
MANIPULATOR_CAPABILITY_DECLARATIONS: tuple[tuple[str, str], ...] = tuple(
    (capability_id, "") for capability_id in WELL_KNOWN_CAPABILITY_IDS
)


class DeclarationError(enum.Enum):
  NONE = 0
  EMPTY_ID = 1
  DUPLICATE = 2
  CONFLICT = 3


class DeclarationAssessment(NamedTuple):
  error: DeclarationError
  capability_id: str = ""


def is_well_known_capability_id(capability_id: str) -> bool:
  return capability_id in WELL_KNOWN_CAPABILITY_IDS


def declares_capability(
    declarations: Sequence[tuple[str, str]], capability_id: str
) -> bool:
  return any(declared_id == capability_id for declared_id, _ in declarations)


def assess_declarations(
    declarations: Sequence[tuple[str, str]],
) -> DeclarationAssessment:
  """Reject empty ids, duplicates, and conflicting interface bindings.

  A duplicate is the same capability id more than once with the same
  interface_id. A conflict is the same id with two interface_id values,
  including empty versus non-empty, and it wins over a duplicate in the
  same list. Unknown ids are accepted. An empty list advertises nothing.
  """
  for capability_id, _interface_id in declarations:
    if capability_id == "":
      return DeclarationAssessment(DeclarationError.EMPTY_ID, "")
  duplicate = DeclarationAssessment(DeclarationError.NONE, "")
  for index, (capability_id, interface_id) in enumerate(declarations):
    for earlier_id, earlier_interface in declarations[:index]:
      if capability_id != earlier_id:
        continue
      if interface_id != earlier_interface:
        return DeclarationAssessment(DeclarationError.CONFLICT, capability_id)
      if duplicate.error is DeclarationError.NONE:
        duplicate = DeclarationAssessment(
            DeclarationError.DUPLICATE, capability_id
        )
  return duplicate
