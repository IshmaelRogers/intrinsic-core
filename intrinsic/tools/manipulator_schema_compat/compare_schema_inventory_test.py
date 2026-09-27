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

"""Fixture tests for the manipulator schema compatibility comparer."""

from __future__ import annotations

import copy
import importlib.util
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest


def _load_module(path: Path, name: str):
  spec = importlib.util.spec_from_file_location(name, path)
  if spec is None or spec.loader is None:
    raise RuntimeError(f"could not load {path}")
  module = importlib.util.module_from_spec(spec)
  spec.loader.exec_module(module)
  return module


_DIR = Path(__file__).resolve().parent
_TOOL = _DIR / "compare_schema_inventory.py"
_TESDATA = _DIR / "testdata"
_BASELINE_FIXTURE = _TESDATA / "baseline.json"
_REAL_BASELINE = (
    _DIR.parent
    / "manipulator_schema_baseline"
    / "manipulator_public_schema_inventory.json"
)
_EXTRACTOR = (
    _DIR.parent
    / "manipulator_schema_baseline"
    / "extract_public_schema_inventory.py"
)
_COMPAT = _load_module(_TOOL, "compare_schema_inventory")
_EXTRACTOR_MOD = _load_module(_EXTRACTOR, "extract_public_schema_inventory")

_GOAL_PATH = "intrinsic_apis/intrinsic/icon/actions/point_to_point_move.proto"
_GOAL_MESSAGE = "PointToPointMoveFixedParams"
_LIMITS_PATH = (
    "intrinsic_control/intrinsic/icon/hal/interfaces/joint_limits.fbs"
)

# Exact phrases the definition of done requires the diagnostics to name.
_CASE_PHRASES = {
    "compatible_additive.json": (
        0,
        (
            "added field z proto tag 3",
            "added value HOLD number 2",
            "added field max_velocity flatbuffer id 2",
            "added value WARN number 2",
        ),
    ),
    "incompatible_removed_symbol.json": (
        1,
        (
            "removed message Pose",
            "removed enum Mode",
            "removed rpc Move",
        ),
    ),
    "incompatible_removed_field.json": (
        1,
        (
            "removed field y proto tag 2",
            "removed field max_position flatbuffer id 1",
        ),
    ),
    "incompatible_reused_proto_tag.json": (
        1,
        ("proto tag 2 reused by yaw (was y)",),
    ),
    "incompatible_flatbuffer_id.json": (
        1,
        (
            "field min_position flatbuffer id 0 -> 3",
            "field x flatbuffer id 0 -> 1",
            "field y flatbuffer id 1 -> 0",
        ),
    ),
    "incompatible_enum_renumber.json": (
        1,
        (
            "value ON number 1 -> 4",
            "value TRIP number 1 -> 5",
        ),
    ),
    "incompatible_enum_number_reuse.json": (
        1,
        ("value number 1 reused by PAUSED (was ON)",),
    ),
    "incompatible_reserved_tag_reuse.json": (
        1,
        ("field revived proto tag 8 reuses reserved range 8:9",),
    ),
}


def _load(path: Path) -> dict:
  return json.loads(path.read_text(encoding="utf-8"))


def _run_tool(*args: str) -> subprocess.CompletedProcess[str]:
  return subprocess.run(
      [sys.executable, str(_TOOL), *args],
      check=False,
      capture_output=True,
      text=True,
  )


def _find_message(inventory: dict, path: str, qualified: str) -> dict:
  for schema in inventory["schemas"]:
    if schema["path"] != path:
      continue
    for message in schema["messages"]:
      if message["qualified_name"] == qualified:
        return message
  raise AssertionError(f"missing {qualified} in {path}")


def _find_field(message: dict, name: str) -> dict:
  for field in message["fields"]:
    if field["name"] == name:
      return field
  raise AssertionError(f"missing field {name}")


class CompareSchemaInventoryTest(unittest.TestCase):

  def test_fixture_reports_match_expected_files(self) -> None:
    baseline = _load(_BASELINE_FIXTURE)
    for name, (exit_code, phrases) in _CASE_PHRASES.items():
      with self.subTest(name=name):
        candidate_path = _TESDATA / name
        expected_path = candidate_path.with_suffix(".expected.txt")
        candidate = _load(candidate_path)
        notes = _COMPAT.compare_inventories(baseline, candidate)
        report = _COMPAT.format_report(notes)
        self.assertEqual(expected_path.read_text(encoding="utf-8"), report)
        self.assertEqual(report, "".join(f"{note}\n" for note in notes))
        self.assertEqual(notes, sorted(notes))
        incompatible = _COMPAT.has_incompatible(notes)
        self.assertEqual(incompatible, exit_code == 1)
        for phrase in phrases:
          self.assertIn(phrase, report)
        if exit_code == 0:
          self.assertTrue(notes)
          self.assertTrue(all(note.startswith("additive:") for note in notes))
        else:
          self.assertTrue(
              all(note.startswith("incompatible:") for note in notes)
          )

  def test_cli_matches_fixtures_and_exit_codes(self) -> None:
    for name, (exit_code, _) in _CASE_PHRASES.items():
      with self.subTest(name=name):
        candidate = _TESDATA / name
        expected = candidate.with_suffix(".expected.txt").read_text(
            encoding="utf-8"
        )
        result = _run_tool(str(_BASELINE_FIXTURE), str(candidate))
        self.assertEqual(result.returncode, exit_code)
        self.assertEqual(result.stdout, expected)
        self.assertEqual(result.stderr, "")

  def test_strict_cli_rejects_additive_fixture(self) -> None:
    result = _run_tool(
        str(_BASELINE_FIXTURE),
        str(_TESDATA / "compatible_additive.json"),
        "--strict",
    )
    self.assertEqual(result.returncode, 1)
    lines = result.stdout.splitlines()
    self.assertTrue(lines)
    self.assertTrue(
        all(line.startswith("incompatible: strict additive:") for line in lines)
    )

  def test_checked_in_baseline_matches_itself(self) -> None:
    before = _REAL_BASELINE.read_bytes()
    inventory = _load(_REAL_BASELINE)
    self.assertEqual(inventory["selection_rule_id"], _COMPAT.SELECTION_RULE_ID)
    self.assertEqual(_COMPAT.compare_inventories(inventory, inventory), [])
    one_arg = _run_tool(str(_REAL_BASELINE))
    two_arg = _run_tool(str(_REAL_BASELINE), str(_REAL_BASELINE))
    self.assertEqual(one_arg.returncode, 0)
    self.assertEqual(one_arg.stdout, "identical\n")
    self.assertEqual(two_arg.returncode, 0)
    self.assertEqual(two_arg.stdout, "identical\n")
    self.assertEqual(_REAL_BASELINE.read_bytes(), before)

  def test_checked_in_scope_is_not_expanded(self) -> None:
    inventory = _load(_REAL_BASELINE)
    schema_paths = {schema["path"] for schema in inventory["schemas"]}
    outside = inventory["out_of_scope_imports"]
    self.assertIn("intrinsic/skills/proto/skills.proto", outside)
    self.assertIn("google/protobuf/empty.proto", outside)
    self.assertTrue(outside)
    for import_path in outside:
      self.assertNotIn(import_path, schema_paths)
    mutated = copy.deepcopy(inventory)
    mutated["out_of_scope_imports"] = sorted(
        set(outside) | {"not/in/the/protected/set.proto"}
    )
    mutated["selection_rule"] = "prose-only change"
    mutated["summary"]["schema_count"] = 0
    self.assertEqual(_COMPAT.compare_inventories(inventory, mutated), [])

  def test_checked_in_baseline_proto_tag_change_fails(self) -> None:
    inventory = _load(_REAL_BASELINE)
    mutated = copy.deepcopy(inventory)
    message = _find_message(mutated, _GOAL_PATH, _GOAL_MESSAGE)
    field = _find_field(message, "goal_position")
    self.assertEqual(field["number"], 1)
    self.assertEqual(field["id_kind"], "tag")
    field["number"] = 1001
    notes = _COMPAT.compare_inventories(inventory, mutated)
    self.assertEqual(
        notes,
        [
            "incompatible: "
            f"{_GOAL_PATH} message {_GOAL_MESSAGE} field goal_position "
            "proto tag 1 -> 1001"
        ],
    )

  def test_checked_in_baseline_flatbuffer_id_change_fails(self) -> None:
    inventory = _load(_REAL_BASELINE)
    mutated = copy.deepcopy(inventory)
    message = _find_message(mutated, _LIMITS_PATH, "JointLimits")
    field = _find_field(message, "min_position")
    self.assertEqual(field["number"], 0)
    self.assertEqual(field["id_kind"], "vtable")
    used = {item["number"] for item in message["fields"]}
    fresh = max(used) + 1
    field["number"] = fresh
    notes = _COMPAT.compare_inventories(inventory, mutated)
    self.assertEqual(
        notes,
        [
            "incompatible: "
            f"{_LIMITS_PATH} message JointLimits field min_position "
            f"flatbuffer id 0 -> {fresh}"
        ],
    )

  def test_checked_in_baseline_allows_additive_field(self) -> None:
    inventory = _load(_REAL_BASELINE)
    mutated = copy.deepcopy(inventory)
    message = _find_message(mutated, _GOAL_PATH, _GOAL_MESSAGE)
    message["fields"].append(
        {
            "default": "",
            "deprecated": False,
            "extra_attributes": [],
            "id_kind": "tag",
            "json_name": "",
            "label": "optional",
            "name": "compat_probe",
            "number": 500001,
            "oneof": "",
            "packed": None,
            "required": False,
            "type": "bool",
        }
    )
    notes = _COMPAT.compare_inventories(inventory, mutated)
    self.assertEqual(
        notes,
        [
            "additive: "
            f"{_GOAL_PATH} message {_GOAL_MESSAGE} added field "
            "compat_probe proto tag 500001"
        ],
    )
    self.assertFalse(_COMPAT.has_incompatible(notes))

  def test_report_is_independent_of_symbol_order(self) -> None:
    baseline = _load(_BASELINE_FIXTURE)
    candidate = _load(_TESDATA / "incompatible_flatbuffer_id.json")
    expected = _COMPAT.compare_inventories(baseline, candidate)
    scrambled = copy.deepcopy(candidate)
    scrambled["schemas"].reverse()
    for schema in scrambled["schemas"]:
      schema["messages"].reverse()
      schema["enums"].reverse()
      for message in schema["messages"]:
        message["fields"].reverse()
      for enum in schema["enums"]:
        enum["values"].reverse()
    self.assertEqual(_COMPAT.compare_inventories(baseline, scrambled), expected)
    again = _COMPAT.compare_inventories(baseline, candidate)
    self.assertEqual(again, expected)
    self.assertEqual(
        _COMPAT.format_report(again), _COMPAT.format_report(expected)
    )

  def test_field_attribute_changes_are_incompatible(self) -> None:
    baseline = _load(_BASELINE_FIXTURE)
    changes = {
        "type": "float",
        "label": "repeated",
        "oneof": "choice",
        "required": True,
        "default": "1",
        "packed": True,
        "id_kind": "vtable",
    }
    for key, value in changes.items():
      with self.subTest(key=key):
        mutated = copy.deepcopy(baseline)
        message = _find_message(mutated, "demo/manipulator.proto", "Pose")
        _find_field(message, "x")[key] = value
        notes = _COMPAT.compare_inventories(baseline, mutated)
        self.assertTrue(_COMPAT.has_incompatible(notes))
        self.assertTrue(any(f"field x {key} " in note for note in notes))

  def test_dropped_reserved_name_and_range_are_incompatible(self) -> None:
    baseline = _load(_BASELINE_FIXTURE)
    mutated = copy.deepcopy(baseline)
    message = _find_message(mutated, "demo/manipulator.proto", "Pose")
    message["reserved_names"] = []
    message["reserved_ranges"] = []
    notes = _COMPAT.compare_inventories(baseline, mutated)
    self.assertIn(
        "incompatible: demo/manipulator.proto message Pose "
        "dropped reserved name old_goal",
        notes,
    )
    self.assertIn(
        "incompatible: demo/manipulator.proto message Pose "
        "dropped reserved range 8:9",
        notes,
    )

  def test_deprecation_and_json_name_are_not_wire_breaks(self) -> None:
    baseline = _load(_BASELINE_FIXTURE)
    mutated = copy.deepcopy(baseline)
    field = _find_field(
        _find_message(mutated, "demo/manipulator.proto", "Pose"), "x"
    )
    field["deprecated"] = True
    field["json_name"] = "ex"
    self.assertEqual(_COMPAT.compare_inventories(baseline, mutated), [])

  def test_rpc_signature_change_is_incompatible(self) -> None:
    baseline = _load(_BASELINE_FIXTURE)
    mutated = copy.deepcopy(baseline)
    method = mutated["schemas"][1]["services"][0]["methods"][0]
    self.assertEqual(method["name"], "Move")
    method["request_type"] = "OtherRequest"
    method["client_streaming"] = True
    notes = _COMPAT.compare_inventories(baseline, mutated)
    self.assertIn(
        "incompatible: demo/manipulator.proto service Manipulator rpc Move "
        "request_type MoveRequest -> OtherRequest",
        notes,
    )
    self.assertIn(
        "incompatible: demo/manipulator.proto service Manipulator rpc Move "
        "client_streaming False -> True",
        notes,
    )

  def test_added_and_removed_schema(self) -> None:
    baseline = _load(_BASELINE_FIXTURE)
    removed = copy.deepcopy(baseline)
    removed["schemas"] = [
        schema
        for schema in removed["schemas"]
        if schema["path"] != "demo/joint_limits.fbs"
    ]
    notes = _COMPAT.compare_inventories(baseline, removed)
    self.assertIn("incompatible: removed schema demo/joint_limits.fbs", notes)
    added = copy.deepcopy(baseline)
    extra = copy.deepcopy(added["schemas"][0])
    extra["path"] = "demo/extra.proto"
    extra["kind"] = "protobuf"
    extra["messages"] = []
    extra["enums"] = []
    added["schemas"].append(extra)
    added_notes = _COMPAT.compare_inventories(baseline, added)
    self.assertEqual(added_notes, ["additive: added schema demo/extra.proto"])

  def test_duplicate_proto_tag_is_incompatible(self) -> None:
    baseline = _load(_BASELINE_FIXTURE)
    mutated = copy.deepcopy(baseline)
    message = _find_message(mutated, "demo/manipulator.proto", "Pose")
    message["fields"].append(
        {
            "default": "",
            "deprecated": False,
            "extra_attributes": [],
            "id_kind": "tag",
            "json_name": "",
            "label": "singular",
            "name": "yaw",
            "number": 1,
            "oneof": "",
            "packed": None,
            "required": False,
            "type": "double",
        }
    )
    notes = _COMPAT.compare_inventories(baseline, mutated)
    self.assertTrue(_COMPAT.has_incompatible(notes))
    self.assertTrue(any("reused by yaw (was x)" in note for note in notes))
    self.assertTrue(any("duplicate proto tag 1" in note for note in notes))
    self.assertFalse(any(note.startswith("additive:") for note in notes))

  def test_identity_changes_are_incompatible(self) -> None:
    baseline = _load(_BASELINE_FIXTURE)
    rule = copy.deepcopy(baseline)
    rule["selection_rule_id"] = "other-rule"
    self.assertEqual(
        _COMPAT.compare_inventories(baseline, rule),
        [
            "incompatible: selection_rule_id "
            f"{_COMPAT.SELECTION_RULE_ID} -> other-rule"
        ],
    )
    version = copy.deepcopy(baseline)
    version["inventory_version"] = 2
    self.assertEqual(
        _COMPAT.compare_inventories(baseline, version),
        ["incompatible: inventory_version 1 -> 2"],
    )

  def test_union_and_extend_renumber(self) -> None:
    baseline = _load(_BASELINE_FIXTURE)
    mutated = copy.deepcopy(baseline)
    proto = next(
        schema
        for schema in mutated["schemas"]
        if schema["path"] == "demo/manipulator.proto"
    )
    proto["unions"] = [
        {"members": [{"name": "Pose", "number": 2}], "name": "Payload"}
    ]
    proto["extends"] = [
        {
            "extendee": "google.protobuf.FieldOptions",
            "fields": [
                {
                    "default": "",
                    "deprecated": False,
                    "extra_attributes": [],
                    "id_kind": "tag",
                    "json_name": "",
                    "label": "optional",
                    "name": "unit",
                    "number": 50001,
                    "oneof": "",
                    "packed": None,
                    "required": False,
                    "type": "string",
                }
            ],
        }
    ]
    added = _COMPAT.compare_inventories(baseline, mutated)
    self.assertIn("additive: demo/manipulator.proto added union Payload", added)
    self.assertIn(
        "additive: demo/manipulator.proto union Payload added member Pose "
        "number 2",
        added,
    )
    self.assertFalse(_COMPAT.has_incompatible(added))
    renumbered = copy.deepcopy(mutated)
    renumbered_proto = next(
        schema
        for schema in renumbered["schemas"]
        if schema["path"] == "demo/manipulator.proto"
    )
    renumbered_proto["unions"][0]["members"][0]["number"] = 3
    renumbered_proto["extends"][0]["fields"][0]["number"] = 50002
    notes = _COMPAT.compare_inventories(mutated, renumbered)
    self.assertIn(
        "incompatible: demo/manipulator.proto union Payload member Pose "
        "number 2 -> 3",
        notes,
    )
    self.assertIn(
        "incompatible: demo/manipulator.proto extend "
        "google.protobuf.FieldOptions field unit proto tag 50001 -> 50002",
        notes,
    )

  def test_malformed_inventory_exits_nonzero_without_a_wire_report(
      self,
  ) -> None:
    with tempfile.TemporaryDirectory() as tmp:
      directory = Path(tmp)
      bad_json = directory / "bad.json"
      bad_json.write_text("{", encoding="utf-8")
      missing = directory / "missing.json"
      not_object = directory / "list.json"
      not_object.write_text("[]\n", encoding="utf-8")
      for path in (bad_json, missing, not_object):
        with self.subTest(path=path.name):
          result = _run_tool(str(_BASELINE_FIXTURE), str(path))
          self.assertEqual(result.returncode, 2)
          self.assertEqual(result.stdout, "")
          self.assertIn("error:", result.stderr)
    baseline = _load(_BASELINE_FIXTURE)
    broken = copy.deepcopy(baseline)
    message = _find_message(broken, "demo/manipulator.proto", "Pose")
    del _find_field(message, "x")["type"]
    with self.assertRaises(_COMPAT.InventoryError):
      _COMPAT.compare_inventories(baseline, broken)

  def test_fixtures_are_canonical_json(self) -> None:
    for path in sorted(_TESDATA.glob("*.json")):
      with self.subTest(path=path.name):
        data = json.loads(path.read_text(encoding="utf-8"))
        canonical = (
            json.dumps(data, ensure_ascii=True, indent=2, sort_keys=True) + "\n"
        )
        self.assertEqual(path.read_text(encoding="utf-8"), canonical)
        self.assertEqual(data["selection_rule_id"], _COMPAT.SELECTION_RULE_ID)

  def test_extractor_verdict_agrees_on_shared_cases(self) -> None:
    baseline = _load(_BASELINE_FIXTURE)
    shared = [
        "compatible_additive.json",
        "incompatible_removed_symbol.json",
        "incompatible_removed_field.json",
        "incompatible_reused_proto_tag.json",
        "incompatible_flatbuffer_id.json",
        "incompatible_enum_renumber.json",
        "incompatible_enum_number_reuse.json",
    ]
    self.assertEqual(
        _EXTRACTOR_MOD.compare_inventories(baseline, baseline, strict=False),
        [],
    )
    for name in shared:
      with self.subTest(name=name):
        candidate = _load(_TESDATA / name)
        ours = _COMPAT.has_incompatible(
            _COMPAT.compare_inventories(baseline, candidate)
        )
        theirs = _EXTRACTOR_MOD._has_incompatible(
            _EXTRACTOR_MOD.compare_inventories(
                baseline, candidate, strict=False
            )
        )
        self.assertEqual(ours, theirs)

  def test_extractor_self_check_remains_green(self) -> None:
    result = subprocess.run(
        [sys.executable, str(_EXTRACTOR), "--self-check"],
        check=False,
        capture_output=True,
        text=True,
    )
    self.assertEqual(result.returncode, 0, result.stderr)
    self.assertIn("self-check: ok", result.stdout)


if __name__ == "__main__":
  unittest.main()
