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

"""Capability discovery for an embodiment resource.

Policy: intrinsic_apis/intrinsic/embodiment/proto/README.md.

Category numbers match intrinsic_proto.embodiment.CapabilityCategory.
Helpers do not switch on a robot type. Keep MANIPULATOR_CAPABILITIES in
sync with capability_descriptor.h and the README table.
"""

from collections.abc import Sequence
import enum

from intrinsic.embodiment.proto import capability_descriptor_pb2

CAPABILITY_CATEGORY_UNSPECIFIED = 0
CAPABILITY_CATEGORY_STATE = 1
CAPABILITY_CATEGORY_COMMAND = 2
CAPABILITY_CATEGORY_SENSOR = 3
CAPABILITY_CATEGORY_ACTUATOR = 4
CAPABILITY_CATEGORY_PLANNING = 5
CAPABILITY_CATEGORY_SIMULATION = 6

CAPABILITY_CATEGORY_ID_STATE = "state"
CAPABILITY_CATEGORY_ID_COMMAND = "command"
CAPABILITY_CATEGORY_ID_SENSOR = "sensor"
CAPABILITY_CATEGORY_ID_ACTUATOR = "actuator"
CAPABILITY_CATEGORY_ID_PLANNING = "planning"
CAPABILITY_CATEGORY_ID_SIMULATION = "simulation"

MANIPULATOR_RESOURCE_ID = "manipulator"

# id, category number, interface name.
Declaration = tuple[str, int, str]

_CATEGORY_STABLE_IDS = {
    CAPABILITY_CATEGORY_STATE: CAPABILITY_CATEGORY_ID_STATE,
    CAPABILITY_CATEGORY_COMMAND: CAPABILITY_CATEGORY_ID_COMMAND,
    CAPABILITY_CATEGORY_SENSOR: CAPABILITY_CATEGORY_ID_SENSOR,
    CAPABILITY_CATEGORY_ACTUATOR: CAPABILITY_CATEGORY_ID_ACTUATOR,
    CAPABILITY_CATEGORY_PLANNING: CAPABILITY_CATEGORY_ID_PLANNING,
    CAPABILITY_CATEGORY_SIMULATION: CAPABILITY_CATEGORY_ID_SIMULATION,
}


class DeclarationStatus(enum.Enum):
  OK = 0
  DUPLICATE_ID = 1
  CONFLICTING_DECLARATION = 2
  EMPTY_ID = 3
  UNSPECIFIED_CATEGORY = 4


# FeatureInterfaceTypes numeric order, then motion_planner, then simulator.
MANIPULATOR_CAPABILITIES: tuple[Declaration, ...] = (
    ("joint_position", CAPABILITY_CATEGORY_COMMAND, "JointPosition"),
    ("joint_velocity", CAPABILITY_CATEGORY_COMMAND, "JointVelocity"),
    (
        "joint_position_sensor",
        CAPABILITY_CATEGORY_SENSOR,
        "JointPositionSensor",
    ),
    (
        "joint_velocity_estimator",
        CAPABILITY_CATEGORY_STATE,
        "JointVelocityEstimator",
    ),
    (
        "joint_acceleration_estimator",
        CAPABILITY_CATEGORY_STATE,
        "JointAccelerationEstimator",
    ),
    ("joint_limits", CAPABILITY_CATEGORY_STATE, "JointLimitsInterface"),
    (
        "cartesian_limits",
        CAPABILITY_CATEGORY_STATE,
        "CartesianLimitsInterface",
    ),
    ("simple_gripper", CAPABILITY_CATEGORY_ACTUATOR, "SimpleGripper"),
    ("adio", CAPABILITY_CATEGORY_SENSOR, "ADIO"),
    ("range_finder", CAPABILITY_CATEGORY_SENSOR, "RangeFinder"),
    (
        "manipulator_kinematics",
        CAPABILITY_CATEGORY_PLANNING,
        "ManipulatorKinematics",
    ),
    ("joint_torque", CAPABILITY_CATEGORY_COMMAND, "JointTorque"),
    ("joint_torque_sensor", CAPABILITY_CATEGORY_SENSOR, "JointTorqueSensor"),
    ("dynamics", CAPABILITY_CATEGORY_PLANNING, "Dynamics"),
    ("force_torque_sensor", CAPABILITY_CATEGORY_SENSOR, "ForceTorqueSensor"),
    ("linear_gripper", CAPABILITY_CATEGORY_ACTUATOR, "LinearGripper"),
    ("hand_guiding", CAPABILITY_CATEGORY_COMMAND, "HandGuiding"),
    (
        "control_mode_exporter",
        CAPABILITY_CATEGORY_STATE,
        "ControlModeExporter",
    ),
    ("move_ok", CAPABILITY_CATEGORY_STATE, "MoveOk"),
    ("imu", CAPABILITY_CATEGORY_SENSOR, "InertialMeasurementUnit"),
    (
        "standalone_force_torque_sensor",
        CAPABILITY_CATEGORY_SENSOR,
        "StandaloneForceTorqueSensor",
    ),
    (
        "process_wrench_at_endeffector",
        CAPABILITY_CATEGORY_COMMAND,
        "ProcessWrenchAtEndeffector",
    ),
    ("payload", CAPABILITY_CATEGORY_COMMAND, "Payload"),
    ("payload_state", CAPABILITY_CATEGORY_STATE, "PayloadState"),
    (
        "cartesian_position_state",
        CAPABILITY_CATEGORY_STATE,
        "CartesianPositionState",
    ),
    ("homing", CAPABILITY_CATEGORY_COMMAND, "Homing"),
    ("joint_acceleration", CAPABILITY_CATEGORY_COMMAND, "JointAcceleration"),
    (
        "motion_planner",
        CAPABILITY_CATEGORY_PLANNING,
        "MotionPlannerInterface",
    ),
    ("simulator", CAPABILITY_CATEGORY_SIMULATION, "Simulator"),
)


def category_stable_id(category: int) -> str:
  """Stable string id for a known category.

  Empty when the category is unknown or unspecified.
  """
  return _CATEGORY_STABLE_IDS.get(category, "")


def is_unspecified_category(category: int) -> bool:
  return category == CAPABILITY_CATEGORY_UNSPECIFIED


def is_known_category(category: int) -> bool:
  return category in _CATEGORY_STABLE_IDS


def validate_declarations(
    declarations: Sequence[Declaration],
) -> tuple[DeclarationStatus, int]:
  """Returns the first problem as (status, index).

  index is 0 when status is OK. Does not modify declarations. Unknown ids
  and unknown category numbers other than 0 are accepted.
  """
  for index, (capability_id, category, interface_name) in enumerate(
      declarations
  ):
    if capability_id == "":
      return (DeclarationStatus.EMPTY_ID, index)
    if is_unspecified_category(category):
      return (DeclarationStatus.UNSPECIFIED_CATEGORY, index)
    earlier = declarations[:index]
    for earlier_id, earlier_category, earlier_interface in earlier:
      if earlier_id != capability_id:
        continue
      if earlier_category != category or earlier_interface != interface_name:
        return (DeclarationStatus.CONFLICTING_DECLARATION, index)
      return (DeclarationStatus.DUPLICATE_ID, index)
  return (DeclarationStatus.OK, 0)


def find_capability(
    declarations: Sequence[Declaration], capability_id: str
) -> Declaration | None:
  """None when the id is absent. Does not insert a capability."""
  for declaration in declarations:
    if declaration[0] == capability_id:
      return declaration
  return None


def is_manipulator_capability_id(capability_id: str) -> bool:
  """False for ids this profile does not advertise, including vehicle ids.

  False does not delete a declaration.
  """
  return find_capability(MANIPULATOR_CAPABILITIES, capability_id) is not None


def views_of(
    descriptor: capability_descriptor_pb2.EmbodimentDescriptor,
) -> list[Declaration]:
  return [
      (capability.id, int(capability.category), capability.interface_name)
      for capability in descriptor.capabilities
  ]


def manipulator_descriptor() -> capability_descriptor_pb2.EmbodimentDescriptor:
  """Industrial manipulator profile.

  Pure data. Does not read hardware, register ICON features, start a
  simulator, or command motion. Vehicle capabilities are absent.
  """
  descriptor = capability_descriptor_pb2.EmbodimentDescriptor()
  descriptor.resource_id = MANIPULATOR_RESOURCE_ID
  for capability_id, category, interface_name in MANIPULATOR_CAPABILITIES:
    added = descriptor.capabilities.add()
    added.id = capability_id
    added.category = category
    added.interface_name = interface_name
  return descriptor
