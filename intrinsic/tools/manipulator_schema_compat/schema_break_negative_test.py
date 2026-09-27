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

"""Negative test for an intentional proto and FlatBuffer schema break.

The sources under ``testdata/schema_break/`` are parsed by the public-schema
extractor and compared by the compatibility gate. They are not production
schemas.
"""

from __future__ import annotations

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
_EXTRACTOR_PATH = (
    _DIR.parent
    / "manipulator_schema_baseline"
    / "extract_public_schema_inventory.py"
)
_REAL_BASELINE = (
    _DIR.parent
    / "manipulator_schema_baseline"
    / "manipulator_public_schema_inventory.json"
)
_FIXTURE = _DIR / "testdata" / "schema_break"
_BASELINE_DIR = _FIXTURE / "baseline"
_BROKEN_DIR = _FIXTURE / "broken"
_EXPECTED = _FIXTURE / "expected.txt"

_LOGICAL_PROTO = (
    "intrinsic/tools/manipulator_schema_compat/testdata/"
    "schema_break/pose.proto"
)
_LOGICAL_FBS = (
    "intrinsic/tools/manipulator_schema_compat/testdata/"
    "schema_break/joint_limits.fbs"
)

# Exact comparer categories. ``_id_word`` emits these phrases.
_PROTO_CATEGORY = "proto tag"
_FLATBUFFER_CATEGORY = "flatbuffer id"
_EXPECTED_BY_CATEGORY = {
    _FLATBUFFER_CATEGORY: (
        "incompatible: "
        f"{_LOGICAL_FBS} message JointLimits field max_position "
        "flatbuffer id 1 -> 4"
    ),
    _PROTO_CATEGORY: (
        "incompatible: "
        f"{_LOGICAL_PROTO} message Pose field y proto tag 2 -> 9"
    ),
}

_COMPAT = _load_module(_TOOL, "compare_schema_inventory_for_schema_break")
_EXTRACTOR = _load_module(
    _EXTRACTOR_PATH, "extract_public_schema_inventory_for_schema_break"
)


def _repo_root() -> Path:
  for parent in _DIR.parents:
    if (parent / "MODULE.bazel").is_file():
      return parent
  raise RuntimeError("could not locate the repository root")


def _category(note: str) -> str:
  """Return the single wire-break category named by ``note``."""
  found = [
      name
      for name in (_FLATBUFFER_CATEGORY, _PROTO_CATEGORY)
      if f" {name} " in note
  ]
  if len(found) != 1:
    raise AssertionError(f"expected one diagnostic category in: {note}")
  return found[0]


def _inventory_from(directory: Path) -> dict:
  proto = _EXTRACTOR.parse_proto(
      (directory / "pose.proto").read_text(encoding="utf-8"),
      _LOGICAL_PROTO,
  )
  flatbuffer = _EXTRACTOR.parse_fbs(
      (directory / "joint_limits.fbs").read_text(encoding="utf-8"),
      _LOGICAL_FBS,
  )
  schemas = sorted((proto, flatbuffer), key=lambda item: item["path"])
  return {
      "inventory_version": _EXTRACTOR.INVENTORY_VERSION,
      "schemas": schemas,
      "selection_rule_id": _EXTRACTOR.SELECTION_RULE_ID,
  }


def _run_tool(*args: str) -> subprocess.CompletedProcess[str]:
  return subprocess.run(
      [sys.executable, str(_TOOL), *args],
      check=False,
      capture_output=True,
      text=True,
  )


class SchemaBreakNegativeTest(unittest.TestCase):

  def test_broken_sources_report_exact_categories(self) -> None:
    baseline = _inventory_from(_BASELINE_DIR)
    broken = _inventory_from(_BROKEN_DIR)
    notes = _COMPAT.compare_inventories(baseline, broken)
    report = _COMPAT.format_report(notes)
    self.assertEqual(_EXPECTED.read_text(encoding="utf-8"), report)
    self.assertEqual(len(notes), 2)
    self.assertEqual(
        {_category(note): note for note in notes},
        _EXPECTED_BY_CATEGORY,
    )
    self.assertEqual(
        sorted(_EXPECTED_BY_CATEGORY),
        [_FLATBUFFER_CATEGORY, _PROTO_CATEGORY],
    )
    self.assertTrue(_COMPAT.has_incompatible(notes))
    self.assertTrue(all(note.startswith("incompatible:") for note in notes))
    self.assertFalse(any(note.startswith("additive:") for note in notes))

  def test_cli_rejects_broken_fixture_and_accepts_baseline(self) -> None:
    baseline = _inventory_from(_BASELINE_DIR)
    broken = _inventory_from(_BROKEN_DIR)
    expected = _EXPECTED.read_text(encoding="utf-8")
    with tempfile.TemporaryDirectory() as tmp:
      directory = Path(tmp)
      baseline_path = directory / "baseline.json"
      broken_path = directory / "broken.json"
      baseline_path.write_text(
          _EXTRACTOR.dump_inventory(baseline), encoding="utf-8"
      )
      broken_path.write_text(
          _EXTRACTOR.dump_inventory(broken), encoding="utf-8"
      )
      rejected = _run_tool(str(baseline_path), str(broken_path))
      accepted = _run_tool(str(baseline_path), str(baseline_path))
    self.assertEqual(rejected.returncode, 1)
    self.assertEqual(rejected.stdout, expected)
    self.assertEqual(rejected.stderr, "")
    self.assertEqual(accepted.returncode, 0)
    self.assertEqual(accepted.stdout, "identical\n")
    self.assertEqual(accepted.stderr, "")

  def test_baseline_fixture_parses_cleanly_and_is_deterministic(self) -> None:
    first = _inventory_from(_BASELINE_DIR)
    second = _inventory_from(_BASELINE_DIR)
    self.assertEqual(_COMPAT.compare_inventories(first, second), [])
    self.assertEqual(
        _EXTRACTOR.dump_inventory(first),
        _EXTRACTOR.dump_inventory(second),
    )
    proto = next(
        schema
        for schema in first["schemas"]
        if schema["path"] == _LOGICAL_PROTO
    )
    flatbuffer = next(
        schema for schema in first["schemas"] if schema["path"] == _LOGICAL_FBS
    )
    self.assertEqual(proto["kind"], "protobuf")
    self.assertEqual(flatbuffer["kind"], "flatbuffer")
    self.assertEqual(first["selection_rule_id"], _COMPAT.SELECTION_RULE_ID)

  def test_fixtures_are_not_production_targets(self) -> None:
    repo = _repo_root()
    before = _REAL_BASELINE.read_bytes()
    inventory = json.loads(_REAL_BASELINE.read_text(encoding="utf-8"))
    protected_paths = {schema["path"] for schema in inventory["schemas"]}
    self.assertEqual(
        inventory["selection_rule_id"], _COMPAT.SELECTION_RULE_ID
    )
    for logical in (_LOGICAL_PROTO, _LOGICAL_FBS):
      self.assertNotIn(logical, protected_paths)
      relative = Path(logical)
      self.assertIn("testdata", relative.parts)
      self.assertTrue(_EXTRACTOR._is_excluded(relative))
      self.assertTrue(_EXTRACTOR._is_excluded(repo / logical))

    discovered = {
        _EXTRACTOR._repo_relative(repo, path)
        for path in _EXTRACTOR.discover_schema_files(repo)
    }
    self.assertNotIn(_LOGICAL_PROTO, discovered)
    self.assertNotIn(_LOGICAL_FBS, discovered)

    build_files = [
        path
        for path in _DIR.rglob("BUILD*")
        if path.name in ("BUILD", "BUILD.bazel")
    ]
    self.assertEqual(build_files, [])
    referenced = subprocess.run(
        [
            "git",
            "grep",
            "-n",
            "-e",
            "schema_break/",
            "--",
            ":(glob)BUILD",
            ":(glob)BUILD.bazel",
        ],
        check=False,
        capture_output=True,
        cwd=repo,
        text=True,
    )
    self.assertEqual(referenced.returncode, 1, referenced.stdout)
    self.assertEqual(referenced.stdout, "")
    self.assertEqual(_REAL_BASELINE.read_bytes(), before)

  def test_protected_baseline_still_compares_identical(self) -> None:
    before = _REAL_BASELINE.read_bytes()
    result = _run_tool(str(_REAL_BASELINE))
    self_check = subprocess.run(
        [sys.executable, str(_EXTRACTOR_PATH), "--self-check"],
        check=False,
        capture_output=True,
        text=True,
    )
    self.assertEqual(result.returncode, 0, result.stderr)
    self.assertEqual(result.stdout, "identical\n")
    self.assertEqual(self_check.returncode, 0, self_check.stderr)
    self.assertIn("self-check: ok", self_check.stdout)
    self.assertEqual(_REAL_BASELINE.read_bytes(), before)


if __name__ == "__main__":
  unittest.main()
