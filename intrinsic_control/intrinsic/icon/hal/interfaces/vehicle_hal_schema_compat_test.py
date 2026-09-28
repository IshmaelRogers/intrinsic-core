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

"""Verifier fixture for the vehicle HAL schemas against the manipulator gate.

A fresh public-schema extract must stay additive against the checked-in
manipulator inventory. Joint FlatBuffer records stay identical. The checked-in
inventory and the protected target list are not rewritten.
"""

from __future__ import annotations

import importlib.util
from pathlib import Path
import unittest


def _repo_root() -> Path:
  for parent in Path(__file__).resolve().parents:
    if (parent / "MODULE.bazel").is_file():
      return parent
  raise RuntimeError("repository root was not found")


def _load_module(path: Path, name: str):
  spec = importlib.util.spec_from_file_location(name, path)
  if spec is None or spec.loader is None:
    raise RuntimeError(f"could not load {path}")
  module = importlib.util.module_from_spec(spec)
  spec.loader.exec_module(module)
  return module


_REPO = _repo_root()
_EXTRACTOR = _load_module(
    _REPO
    / "intrinsic/tools/manipulator_schema_baseline"
    / "extract_public_schema_inventory.py",
    "extract_public_schema_inventory",
)
_COMPARE = _load_module(
    _REPO
    / "intrinsic/tools/manipulator_schema_compat"
    / "compare_schema_inventory.py",
    "compare_schema_inventory",
)

_BASELINE = (
    _REPO
    / "intrinsic/tools/manipulator_schema_baseline"
    / "manipulator_public_schema_inventory.json"
)

_JOINT_SCHEMAS = (
    "intrinsic_control/intrinsic/icon/flatbuffers/transform_types.fbs",
    "intrinsic_control/intrinsic/icon/hal/interfaces/force_torque.fbs",
    "intrinsic_control/intrinsic/icon/hal/interfaces/joint_command.fbs",
    "intrinsic_control/intrinsic/icon/hal/interfaces/joint_limits.fbs",
    "intrinsic_control/intrinsic/icon/hal/interfaces/joint_state.fbs",
)

_NEW_SCHEMAS = (
    "intrinsic_control/intrinsic/icon/hal/interfaces/body_state.fbs",
    "intrinsic_control/intrinsic/icon/hal/interfaces/body_wrench.fbs",
    "intrinsic_control/intrinsic/icon/hal/interfaces/vehicle_hal_types.fbs",
    "intrinsic_control/intrinsic/icon/hal/interfaces/vehicle_limits.fbs",
)

_BODY_STATE_FIELDS = (
    ("sequence", "ulong", "0"),
    ("source_time_present", "bool", "false"),
    ("source_time_seconds", "long", "0"),
    ("source_time_nanos", "int", "0"),
    ("receive_time_present", "bool", "false"),
    ("receive_time_seconds", "long", "0"),
    ("receive_time_nanos", "int", "0"),
    ("source_id", "FixedText256", ""),
    ("frame_id", "FixedText256", ""),
    ("clock_domain", "FixedText256", ""),
    ("validity_present", "bool", "false"),
    ("validity_state", "int", "0"),
    ("pose_world_from_body", "intrinsic_fbs.Transform", ""),
    ("body_twist", "intrinsic_fbs.Twist", ""),
    ("body_acceleration", "intrinsic_fbs.Acceleration", ""),
    ("pose_covariance", "Matrix6d", ""),
    ("twist_covariance", "Matrix6d", ""),
    ("navigation_mode", "int", "0"),
    ("estimator_epoch", "ulong", "0"),
)

_BODY_WRENCH_FIELDS = (
    ("sequence", "ulong", "0"),
    ("source_time_present", "bool", "false"),
    ("source_time_seconds", "long", "0"),
    ("source_time_nanos", "int", "0"),
    ("receive_time_present", "bool", "false"),
    ("receive_time_seconds", "long", "0"),
    ("receive_time_nanos", "int", "0"),
    ("source_id", "FixedText256", ""),
    ("frame_id", "FixedText256", ""),
    ("clock_domain", "FixedText256", ""),
    ("validity_present", "bool", "false"),
    ("validity_state", "int", "0"),
    ("wrench", "intrinsic_fbs.Wrench", ""),
)

_VEHICLE_LIMITS_FIELDS = (
    ("has_linear_speed_limit", "bool", "false"),
    ("max_linear_speed_m_s", "intrinsic_fbs.Point", ""),
    ("has_angular_speed_limit", "bool", "false"),
    ("max_angular_speed_rad_s", "intrinsic_fbs.Point", ""),
    ("has_linear_acceleration_limit", "bool", "false"),
    ("max_linear_acceleration_m_s2", "intrinsic_fbs.Point", ""),
    ("has_angular_acceleration_limit", "bool", "false"),
    ("max_angular_acceleration_rad_s2", "intrinsic_fbs.Point", ""),
    ("has_force_limit", "bool", "false"),
    ("max_force_n", "intrinsic_fbs.Point", ""),
    ("has_torque_limit", "bool", "false"),
    ("max_torque_n_m", "intrinsic_fbs.Point", ""),
)


def _schema(inventory: dict, path: str) -> dict:
  for schema in inventory["schemas"]:
    if schema["path"] == path:
      return schema
  raise AssertionError(f"missing schema {path}")


def _message(schema: dict, name: str) -> dict:
  for message in schema["messages"]:
    if message["name"] == name:
      return message
  raise AssertionError(f"missing message {name} in {schema['path']}")


def _assert_table_fields(test: unittest.TestCase, message: dict, expected):
  test.assertEqual(message["layout"], "table")
  fields = sorted(message["fields"], key=lambda item: item["number"])
  test.assertEqual(len(fields), len(expected))
  for index, (name, type_name, default) in enumerate(expected):
    field = fields[index]
    test.assertEqual(field["name"], name)
    test.assertEqual(field["number"], index)
    test.assertEqual(field["type"], type_name)
    test.assertEqual(field["default"], default)
    test.assertEqual(field["id_kind"], "vtable")


class VehicleHalSchemaCompatTest(unittest.TestCase):

  @classmethod
  def setUpClass(cls):
    cls.baseline = _COMPARE.load_inventory(_BASELINE)
    cls.candidate = _EXTRACTOR.build_inventory(_REPO)
    cls.notes = _COMPARE.compare_inventories(cls.baseline, cls.candidate)

  def test_fresh_extract_is_additive_only(self):
    incompatible = [
        note for note in self.notes if note.startswith("incompatible:")
    ]
    self.assertEqual(incompatible, [])
    for path in _NEW_SCHEMAS:
      self.assertIn(f"additive: added schema {path}", self.notes)
    types = "intrinsic_apis/intrinsic/icon/proto/v1/types.proto"
    self.assertIn(
        f"additive: {types} enum FeatureInterfaceTypes added value "
        "FEATURE_INTERFACE_BODY_STATE number 28",
        self.notes,
    )
    self.assertIn(
        f"additive: {types} enum FeatureInterfaceTypes added value "
        "FEATURE_INTERFACE_BODY_WRENCH_COMMAND number 29",
        self.notes,
    )
    self.assertIn(
        f"additive: {types} enum FeatureInterfaceTypes added value "
        "FEATURE_INTERFACE_VEHICLE_LIMITS number 30",
        self.notes,
    )

  def test_joint_schemas_are_unchanged(self):
    for path in _JOINT_SCHEMAS:
      self.assertEqual(
          _schema(self.baseline, path), _schema(self.candidate, path)
      )

  def test_new_tables_keep_declared_ids(self):
    _assert_table_fields(
        self,
        _message(_schema(self.candidate, _NEW_SCHEMAS[0]), "BodyState"),
        _BODY_STATE_FIELDS,
    )
    _assert_table_fields(
        self,
        _message(_schema(self.candidate, _NEW_SCHEMAS[1]), "BodyWrench"),
        _BODY_WRENCH_FIELDS,
    )
    _assert_table_fields(
        self,
        _message(_schema(self.candidate, _NEW_SCHEMAS[3]), "VehicleLimits"),
        _VEHICLE_LIMITS_FIELDS,
    )
    types = _schema(
        self.candidate,
        "intrinsic_control/intrinsic/icon/hal/interfaces/vehicle_hal_types.fbs",
    )
    text = _message(types, "FixedText256")
    matrix = _message(types, "Matrix6d")
    self.assertEqual(text["layout"], "struct")
    self.assertEqual(text["fields"][0]["type"], "[uint8:256]")
    self.assertEqual(text["fields"][0]["id_kind"], "positional")
    self.assertEqual(matrix["layout"], "struct")
    self.assertEqual(matrix["fields"][0]["type"], "[double:36]")
    self.assertEqual(matrix["fields"][0]["number"], 0)

  def test_protected_inventory_and_baseline_tsv_are_untouched(self):
    baseline_text = _BASELINE.read_text(encoding="utf-8")
    self.assertNotIn("BodyState", baseline_text)
    self.assertNotIn("FEATURE_INTERFACE_BODY_STATE", baseline_text)
    tsv = (_REPO / ".github/baseline/manipulator_targets.tsv").read_text(
        encoding="utf-8"
    )
    for label in (
        "body_state_fbs",
        "body_wrench_fbs",
        "vehicle_limits_fbs",
        "vehicle_hal_types_fbs",
        "vehicle_hardware_interfaces",
        "vehicle_hal_schema_test",
    ):
      self.assertNotIn(label, tsv)
    registry = (
        _REPO
        / "intrinsic_control/intrinsic/icon/control/parts"
        / "feature_interface_registry.cc"
    ).read_text(encoding="utf-8")
    for name in (
        "FEATURE_INTERFACE_BODY_STATE",
        "FEATURE_INTERFACE_BODY_WRENCH_COMMAND",
        "FEATURE_INTERFACE_VEHICLE_LIMITS",
    ):
      self.assertNotIn(name, registry)
    default_header = (
        _REPO
        / "intrinsic_control/intrinsic/icon/hal/default_hardware_interfaces.h"
    ).read_text(encoding="utf-8")
    self.assertNotIn("body_state", default_header)
    self.assertNotIn("BodyWrench", default_header)
    self.assertNotIn("VehicleLimits", default_header)


if __name__ == "__main__":
  unittest.main()
