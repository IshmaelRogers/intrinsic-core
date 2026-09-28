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

"""Unknown-feature checks for the vehicle HAL type ids.

Existing FeatureInterfaceTypes numbers stay put. The three vehicle ids are
appended. A number that is not in the table is not rewritten to
FEATURE_INTERFACE_INVALID, and the manipulator factory still treats these
ids as an unknown feature. The feature-interface registry lists the C++
interfaces. HalArmPart does not register them.
"""

from __future__ import annotations

import importlib.util
import json
from pathlib import Path
import re
import unittest


def _repo_root() -> Path:
  here = Path(__file__).resolve()
  for parent in here.parents:
    if (parent / "MODULE.bazel").is_file():
      return parent
  raise RuntimeError("repository root was not found")


_REPO = _repo_root()
_TYPES = _REPO / "intrinsic_apis/intrinsic/icon/proto/v1/types.proto"
_FACTORY = (
    _REPO
    / "intrinsic_control/intrinsic/icon/control/parts"
    / "realtime_part_factory_common.cc"
)
_REGISTRY = (
    _REPO
    / "intrinsic_control/intrinsic/icon/control/parts"
    / "feature_interface_registry.cc"
)
_DEFAULT_INTERFACES = (
    _REPO / "intrinsic_control/intrinsic/icon/hal/default_hardware_interfaces.h"
)
_ARM = (
    _REPO
    / "intrinsic_control/intrinsic/icon/control/parts/hal/arm_part"
    / "hal_arm_part.cc"
)
_GOLDEN = (
    _REPO
    / "intrinsic_control/intrinsic/icon/control/parts/hal/arm_part"
    / "testdata/arm_feature_registration.golden.json"
)
_TSV = _REPO / ".github/baseline/manipulator_targets.tsv"
_VEHICLE_HEADER = (
    _REPO
    / "intrinsic_control/intrinsic/icon/hal/interfaces"
    / "vehicle_hardware_interfaces.h"
)
_BASELINE_INVENTORY = (
    _REPO
    / "intrinsic/tools/manipulator_schema_baseline"
    / "manipulator_public_schema_inventory.json"
)
_EXTRACTOR = (
    _REPO
    / "intrinsic/tools/manipulator_schema_baseline"
    / "extract_public_schema_inventory.py"
)
_COMPARER = (
    _REPO
    / "intrinsic/tools/manipulator_schema_compat"
    / "compare_schema_inventory.py"
)

# Frozen manipulator ids. 27 is the highest id that existed before this leaf.
_BASELINE_FEATURE_IDS = {
    "FEATURE_INTERFACE_INVALID": 0,
    "FEATURE_INTERFACE_JOINT_POSITION": 1,
    "FEATURE_INTERFACE_JOINT_VELOCITY": 2,
    "FEATURE_INTERFACE_JOINT_POSITION_SENSOR": 3,
    "FEATURE_INTERFACE_JOINT_VELOCITY_ESTIMATOR": 4,
    "FEATURE_INTERFACE_JOINT_ACCELERATION_ESTIMATOR": 5,
    "FEATURE_INTERFACE_JOINT_LIMITS": 6,
    "FEATURE_INTERFACE_CARTESIAN_LIMITS": 7,
    "FEATURE_INTERFACE_SIMPLE_GRIPPER": 8,
    "FEATURE_INTERFACE_ADIO": 9,
    "FEATURE_INTERFACE_RANGE_FINDER": 10,
    "FEATURE_INTERFACE_MANIPULATOR_KINEMATICS": 11,
    "FEATURE_INTERFACE_JOINT_TORQUE": 12,
    "FEATURE_INTERFACE_JOINT_TORQUE_SENSOR": 13,
    "FEATURE_INTERFACE_DYNAMICS": 14,
    "FEATURE_INTERFACE_FORCE_TORQUE_SENSOR": 15,
    "FEATURE_INTERFACE_LINEAR_GRIPPER": 16,
    "FEATURE_INTERFACE_HAND_GUIDING": 17,
    "FEATURE_INTERFACE_CONTROL_MODE_EXPORTER": 18,
    "FEATURE_INTERFACE_MOVE_OK": 19,
    "FEATURE_INTERFACE_IMU": 20,
    "FEATURE_INTERFACE_STANDALONE_FORCE_TORQUE_SENSOR": 21,
    "FEATURE_INTERFACE_PROCESS_WRENCH_AT_ENDEFFECTOR": 22,
    "FEATURE_INTERFACE_PAYLOAD": 23,
    "FEATURE_INTERFACE_PAYLOAD_STATE": 24,
    "FEATURE_INTERFACE_CARTESIAN_POSITION_STATE": 25,
    "FEATURE_INTERFACE_HOMING": 26,
    "FEATURE_INTERFACE_JOINT_ACCELERATION": 27,
}

_VEHICLE_FEATURE_IDS = {
    "FEATURE_INTERFACE_BODY_STATE": 28,
    "FEATURE_INTERFACE_BODY_WRENCH": 29,
    "FEATURE_INTERFACE_VEHICLE_LIMITS": 30,
}

_TYPE_IDS = (
    "intrinsic_fbs.BodyState",
    "intrinsic_fbs.BodyWrench",
    "intrinsic_fbs.VehicleLimits",
)

_JOINT_SCHEMAS = (
    "intrinsic_control/intrinsic/icon/hal/interfaces/joint_command.fbs",
    "intrinsic_control/intrinsic/icon/hal/interfaces/joint_limits.fbs",
    "intrinsic_control/intrinsic/icon/hal/interfaces/joint_state.fbs",
)

_VEHICLE_SCHEMA = (
    "intrinsic_control/intrinsic/icon/hal/interfaces/vehicle_hal.fbs"
)


def _load_module(name: str, path: Path):
  spec = importlib.util.spec_from_file_location(name, path)
  if spec is None or spec.loader is None:
    raise RuntimeError(f"could not load {path}")
  module = importlib.util.module_from_spec(spec)
  spec.loader.exec_module(module)
  return module


def _feature_ids(text: str) -> dict[str, int]:
  start = text.find("enum FeatureInterfaceTypes")
  if start < 0:
    raise ValueError("FeatureInterfaceTypes was not found")
  brace = text.find("{", start)
  depth = 0
  end = None
  for index in range(brace, len(text)):
    char = text[index]
    if char == "{":
      depth += 1
    elif char == "}":
      depth -= 1
      if depth == 0:
        end = index
        break
  if end is None:
    raise ValueError("FeatureInterfaceTypes was not closed")
  body = text[brace + 1 : end]
  found = {
      match.group(1): int(match.group(2))
      for match in re.finditer(
          r"(FEATURE_INTERFACE_\w+)\s*=\s*(\d+)\s*;", body
      )
  }
  if not found:
    raise ValueError("FeatureInterfaceTypes has no values")
  return found


class VehicleHalUnknownFeatureTest(unittest.TestCase):

  def test_feature_ids_are_appended(self) -> None:
    ids = _feature_ids(_TYPES.read_text(encoding="utf-8"))
    self.assertEqual(
        {name: ids[name] for name in _BASELINE_FEATURE_IDS},
        _BASELINE_FEATURE_IDS,
    )
    self.assertEqual(
        {name: ids[name] for name in _VEHICLE_FEATURE_IDS},
        _VEHICLE_FEATURE_IDS,
    )
    self.assertEqual(
        set(ids),
        set(_BASELINE_FEATURE_IDS) | set(_VEHICLE_FEATURE_IDS),
    )
    numbers = list(ids.values())
    self.assertEqual(len(numbers), len(set(numbers)))
    by_number = {number: name for name, number in ids.items()}
    self.assertEqual(by_number[28], "FEATURE_INTERFACE_BODY_STATE")
    self.assertEqual(by_number[29], "FEATURE_INTERFACE_BODY_WRENCH")
    self.assertEqual(by_number[30], "FEATURE_INTERFACE_VEHICLE_LIMITS")
    self.assertNotIn(31, by_number)
    self.assertNotIn(999, by_number)
    self.assertNotEqual(by_number.get(999), "FEATURE_INTERFACE_INVALID")
    self.assertNotEqual(by_number.get(31), "FEATURE_INTERFACE_INVALID")

  def test_unknown_feature_stays_on_the_default_path(self) -> None:
    factory = _FACTORY.read_text(encoding="utf-8")
    registry = _REGISTRY.read_text(encoding="utf-8")
    self.assertIn("Encountered unknown Feature Interface type.", factory)
    self.assertIn("default:", factory)
    for name in _VEHICLE_FEATURE_IDS:
      # The manipulator factory has no generic-config case, so a claim of
      # these ids still takes the default FailedPrecondition path.
      self.assertNotIn(name, factory)
      # The C++ feature interfaces are registered for an opt-in part.
      self.assertIn(name, registry)
    for name, number in _BASELINE_FEATURE_IDS.items():
      if name == "FEATURE_INTERFACE_INVALID":
        continue
      self.assertIn(name, factory)
      parsed = _feature_ids(_TYPES.read_text(encoding="utf-8"))
      self.assertEqual(parsed[name], number)

  def test_arm_does_not_register_vehicle_features(self) -> None:
    golden = _GOLDEN.read_text(encoding="utf-8")
    record = json.loads(golden)
    names = {row["feature_type"] for row in record["supported_feature_types"]}
    for name in _VEHICLE_FEATURE_IDS:
      self.assertNotIn(name, names)
      self.assertNotIn(name, golden)
    self.assertNotIn("vehicle", golden.lower())
    arm = _ARM.read_text(encoding="utf-8")
    default_interfaces = _DEFAULT_INTERFACES.read_text(encoding="utf-8")
    for token in (
        "BodyState",
        "BodyWrench",
        "VehicleLimits",
        "vehicle_hal.fbs",
    ):
      self.assertNotIn(token, arm)
      self.assertNotIn(token, default_interfaces)
    for type_id in _TYPE_IDS:
      self.assertNotIn(type_id, default_interfaces)
    tsv = _TSV.read_text(encoding="utf-8")
    self.assertNotIn("vehicle_hal", tsv)
    self.assertNotIn("vehicle_hardware_interfaces", tsv)

  def test_string_type_ids_are_new_and_stable(self) -> None:
    header = _VEHICLE_HEADER.read_text(encoding="utf-8")
    for type_id in _TYPE_IDS:
      # The constexpr and the hardware-interface macro each carry the literal.
      self.assertEqual(header.count(f'"{type_id}"'), 2)
      self.assertIn(
          f'k{type_id.split(".")[-1]}TypeId[] = "{type_id}"', header
      )
    seen: set[str] = set()
    pattern = re.compile(r"intrinsic_fbs\.[A-Za-z0-9_]+")
    roots = (
        _REPO / "intrinsic_control",
        _REPO / "intrinsic_hardware",
        _REPO / "third_party",
    )
    own = {
        _VEHICLE_HEADER.resolve(),
        Path(__file__).resolve(),
    }
    for root in roots:
      for path in root.rglob("*"):
        if path.suffix not in {".h", ".cc"} or path.resolve() in own:
          continue
        if "vehicle_hal" in path.name:
          continue
        text = path.read_text(encoding="utf-8", errors="ignore")
        seen.update(pattern.findall(text))
    for type_id in _TYPE_IDS:
      self.assertNotIn(type_id, seen)
    self.assertIn("intrinsic_fbs.JointPositionState", seen)
    self.assertIn("intrinsic_fbs.JointLimits", seen)
    self.assertIn("intrinsic_fbs.Wrench", seen)

  def test_schema_compat_keeps_joint_symbols(self) -> None:
    extractor = _load_module("schema_extract", _EXTRACTOR)
    comparer = _load_module("schema_compare", _COMPARER)
    baseline = json.loads(_BASELINE_INVENTORY.read_text(encoding="utf-8"))
    candidate = extractor.build_inventory(_REPO)
    notes = comparer.compare_inventories(baseline, candidate, strict=False)
    incompatible = [note for note in notes if note.startswith("incompatible:")]
    self.assertEqual(incompatible, [], "\n".join(notes))
    added = [
        note for note in notes if note.startswith("additive: added schema ")
    ]
    self.assertEqual(added, [f"additive: added schema {_VEHICLE_SCHEMA}"])
    old = {item["path"]: item for item in baseline["schemas"]}
    new = {item["path"]: item for item in candidate["schemas"]}
    for path in _JOINT_SCHEMAS:
      self.assertEqual(old[path], new[path])
    self.assertNotIn(_VEHICLE_SCHEMA, old)
    vehicle = new[_VEHICLE_SCHEMA]
    tables = {
        item["name"]: item
        for item in vehicle["messages"]
        if item["layout"] == "table"
    }
    self.assertEqual(
        set(tables), {"BodyState", "BodyWrench", "VehicleLimits"}
    )
    self.assertEqual(
        [field["number"] for field in tables["BodyState"]["fields"]],
        list(range(22)),
    )
    self.assertEqual(
        [field["number"] for field in tables["BodyWrench"]["fields"]],
        list(range(16)),
    )
    self.assertEqual(
        [field["number"] for field in tables["VehicleLimits"]["fields"]],
        list(range(17)),
    )
    twist = next(
        item
        for item in vehicle["messages"]
        if item["name"] == "BodyTwistRt"
    )
    self.assertEqual(twist["layout"], "struct")
    self.assertEqual(
        [field["name"] for field in twist["fields"]],
        [
            "linear_x_m_s",
            "linear_y_m_s",
            "linear_z_m_s",
            "angular_x_rad_s",
            "angular_y_rad_s",
            "angular_z_rad_s",
        ],
    )
    self.assertEqual(
        [field["number"] for field in twist["fields"]], list(range(6))
    )
    joined = "\n".join(notes)
    for name, number in _VEHICLE_FEATURE_IDS.items():
      self.assertIn(name, joined)
      self.assertIn(str(number), joined)
    for name in _BASELINE_FEATURE_IDS:
      self.assertNotIn(f"removed enum value {name}", joined)
      self.assertNotIn(f"renumbered", joined)


if __name__ == "__main__":
  unittest.main()
