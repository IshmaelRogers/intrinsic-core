#!/usr/bin/env python3
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

"""Golden for one manipulator MotionPlanningRequest.

Reads ``CreateMotionPlanningRequestFromPointPath`` and the protos it fills.
Compares serialization tags and the function's validation results to a
checked-in fixture. Does not plan a trajectory and does not edit production
sources. Rewriting the fixture requires ``--update-golden`` (see README.md).
"""

from __future__ import annotations

import argparse
import contextlib
import difflib
import importlib.util
import io
import json
from pathlib import Path
import re
import sys
import unittest

SCHEMA = "manipulator-planning-request-v1"

# Case-insensitive scan of the rendered fixture. A manipulator planning
# request must not pick up a vehicle / UUV feature name.
FORBIDDEN_TOKENS = ("bathymetry", "marine", "thruster", "uuv", "vehicle")

# One 6-DOF joint target already used by the manipulator service tests.
_ROBOT_NAME = "agilus_04"
_JOINTS = [-0.1, 0.0, -1.36, -0.77, 0.0, -1.0]

# Inputs for the factory. The empty path is the same length as its motion-type
# list, so it passes the size check and hits the empty-path check.
REPRESENTATIVE_CASES = [
    {
        "motion_types": ["JOINT"],
        "name": "valid_joint_segment",
        "point_path": [_JOINTS],
        "robot_name": _ROBOT_NAME,
    },
    {
        "motion_types": ["JOINT", "LINEAR"],
        "name": "invalid_size_mismatch",
        "point_path": [_JOINTS],
        "robot_name": _ROBOT_NAME,
    },
    {
        "motion_types": [],
        "name": "invalid_empty_point_path",
        "point_path": [],
        "robot_name": _ROBOT_NAME,
    },
]

_DIR = Path(__file__).resolve().parent
_GOLDEN = _DIR / "testdata" / "manipulator_planning_request.golden.json"

_SETTERS = (
    "add_motion_segments",
    "mutable_by_name",
    "mutable_joint_position",
    "mutable_joints",
    "mutable_object_id",
    "mutable_robot_reference",
    "mutable_robot_specification",
    "mutable_target",
    "set_motion_type",
    "set_object_name",
)


def _repo_root() -> Path:
  for parent in _DIR.parents:
    if (parent / "MODULE.bazel").is_file():
      return parent
  raise RuntimeError("repository root was not found")


_REPO = _repo_root()
_UTILS_CC = _DIR / "motion_planner_service_utils.cc"
_UTILS_H = _DIR / "motion_planner_service_utils.h"
_SCHEMA_TOOL = (
    _REPO / "intrinsic/tools/manipulator_schema_baseline/"
    "extract_public_schema_inventory.py"
)

_PROTO_PATHS = {
    "geometric_constraints": (
        "intrinsic_apis/intrinsic/motion_planning/proto/v1/"
        "geometric_constraints.proto"
    ),
    "joint_space": "intrinsic_apis/intrinsic/icon/proto/joint_space.proto",
    "motion_planner_service": (
        "intrinsic_motion_planning/intrinsic/motion_planning/proto/v1/"
        "motion_planner_service.proto"
    ),
    "motion_specification": (
        "intrinsic_motion_planning/intrinsic/motion_planning/proto/v1/"
        "motion_specification.proto"
    ),
    "object_world_refs": (
        "intrinsic_apis/intrinsic/world/proto/object_world_refs.proto"
    ),
    "robot_specification": (
        "intrinsic_apis/intrinsic/motion_planning/proto/v1/"
        "robot_specification.proto"
    ),
}


def _read(path: Path) -> str:
  return path.read_text(encoding="utf-8")


def _strip_comments(text: str) -> str:
  text = re.sub(r"/\*.*?\*/", "", text, flags=re.S)
  return re.sub(r"//.*?$", "", text, flags=re.M)


def _matching_delimiter(
    text: str, open_index: int, open_ch: str, close_ch: str
) -> int:
  if text[open_index] != open_ch:
    raise ValueError(f"expected {open_ch} at {open_index}")
  depth = 0
  i = open_index
  while i < len(text):
    ch = text[i]
    if ch == '"':
      i += 1
      while i < len(text):
        if text[i] == "\\":
          i += 2
          continue
        if text[i] == '"':
          break
        i += 1
    elif ch == open_ch:
      depth += 1
    elif ch == close_ch:
      depth -= 1
      if depth == 0:
        return i
    i += 1
  raise ValueError(f"unbalanced {open_ch}{close_ch} starting at {open_index}")


def _decode_string(literal: str) -> str:
  """Decode one StrCat argument, including adjacent C++ string literals."""
  text = literal.strip()
  if not text.startswith('"'):
    raise ValueError(f"expected a string literal, got {literal!r}")
  parts: list[str] = []
  i = 0
  while i < len(text):
    while i < len(text) and text[i].isspace():
      i += 1
    if i >= len(text):
      break
    if text[i] != '"':
      raise ValueError(f"expected a string literal, got {literal!r}")
    i += 1
    chars: list[str] = []
    while i < len(text):
      if text[i] == "\\" and i + 1 < len(text):
        chars.append(text[i + 1])
        i += 2
        continue
      if text[i] == '"':
        break
      chars.append(text[i])
      i += 1
    if i >= len(text) or text[i] != '"':
      raise ValueError(f"unterminated string literal in {literal!r}")
    parts.append("".join(chars))
    i += 1
  return "".join(parts)


def _split_args(text: str) -> list[str]:
  args: list[str] = []
  start = 0
  depth = 0
  i = 0
  while i < len(text):
    ch = text[i]
    if ch == '"':
      i += 1
      while i < len(text):
        if text[i] == "\\":
          i += 2
          continue
        if text[i] == '"':
          break
        i += 1
    elif ch == "(":
      depth += 1
    elif ch == ")":
      depth -= 1
    elif ch == "," and depth == 0:
      args.append(text[start:i].strip())
      start = i + 1
    i += 1
  tail = text[start:].strip()
  if tail:
    args.append(tail)
  return args


def _function_body(source: str, name: str) -> str:
  idx = source.find(name)
  if idx < 0:
    raise ValueError(f"{name} was not found")
  brace = source.find("{", idx)
  end = _matching_delimiter(source, brace, "{", "}")
  return source[brace + 1 : end]


def _status_name(token: str) -> str:
  if token.startswith("k"):
    token = token[1:]
  if token.endswith("Error"):
    token = token[: -len("Error")]
  return re.sub(r"(?<!^)(?=[A-Z])", "_", token).upper()


def _documented_status(header: str) -> str:
  signature = "CreateMotionPlanningRequestFromPointPath("
  idx = header.find(signature)
  if idx < 0:
    raise ValueError("factory declaration was not found")
  lines = header[:idx].splitlines()
  comment_lines: list[str] = []
  for line in reversed(lines):
    stripped = line.strip()
    if stripped.startswith("//"):
      comment_lines.append(stripped)
      continue
    if not stripped:
      if comment_lines:
        break
      continue
    if comment_lines:
      break
  if not comment_lines:
    raise ValueError("factory comment was not found")
  comment = "\n".join(reversed(comment_lines))
  codes = re.findall(r"`(k[A-Z][A-Za-z0-9]*)`", comment)
  if len(codes) < 2:
    raise ValueError("factory comment does not name two status codes")
  names = {_status_name(code) for code in codes}
  if names != {"INVALID_ARGUMENT"}:
    raise ValueError(f"unexpected documented status codes: {sorted(names)}")
  return "INVALID_ARGUMENT"


def _condition_kind(condition: str) -> str:
  normalized = re.sub(r"\s+", "", condition)
  if normalized == "point_path.size()!=motion_types.size()":
    return "size_mismatch"
  if normalized == "point_path.empty()":
    return "empty_point_path"
  raise ValueError(f"unexpected validation condition: {condition}")


def _message_pieces(block: str) -> list[dict[str, str]]:
  marker = "InvalidArgumentError"
  idx = block.find(marker)
  if idx < 0:
    raise ValueError("InvalidArgumentError was not found")
  paren = block.find("(", idx)
  end = _matching_delimiter(block, paren, "(", ")")
  argument = block[paren + 1 : end].strip()
  if argument.startswith("absl::StrCat"):
    inner_paren = argument.find("(")
    inner_end = _matching_delimiter(argument, inner_paren, "(", ")")
    raw_args = _split_args(argument[inner_paren + 1 : inner_end])
  else:
    raw_args = [argument]
  pieces: list[dict[str, str]] = []
  for raw in raw_args:
    if raw.startswith('"'):
      pieces.append({"kind": "literal", "text": _decode_string(raw)})
    elif re.sub(r"\s+", "", raw) == "point_path.size()":
      pieces.append({"kind": "point_path_size"})
    elif re.sub(r"\s+", "", raw) == "motion_types.size()":
      pieces.append({"kind": "motion_types_size"})
    else:
      raise ValueError(f"unexpected status message piece: {raw}")
  return _merge_literals(pieces)


def _merge_literals(pieces: list[dict[str, str]]) -> list[dict[str, str]]:
  merged: list[dict[str, str]] = []
  for piece in pieces:
    if (
        merged
        and piece["kind"] == "literal"
        and merged[-1]["kind"] == "literal"
    ):
      merged[-1] = {
          "kind": "literal",
          "text": merged[-1]["text"] + piece["text"],
      }
      continue
    merged.append(dict(piece))
  return merged


def _template(pieces: list[dict[str, str]]) -> str:
  parts: list[str] = []
  for piece in pieces:
    if piece["kind"] == "literal":
      parts.append(piece["text"])
    elif piece["kind"] == "point_path_size":
      parts.append("{point_path_size}")
    elif piece["kind"] == "motion_types_size":
      parts.append("{motion_types_size}")
    else:
      raise ValueError(f"unexpected message piece {piece['kind']}")
  return "".join(parts)


def _validation_rules(body: str, documented: str) -> list[dict]:
  rules: list[dict] = []
  cursor = 0
  while True:
    match = re.search(r"\bif\s*\(", body[cursor:])
    if match is None:
      break
    start = cursor + match.start()
    if "MotionPlanningRequest request" in body[:start]:
      break
    paren = body.find("(", start)
    paren_end = _matching_delimiter(body, paren, "(", ")")
    brace = body.find("{", paren_end)
    brace_end = _matching_delimiter(body, brace, "{", "}")
    condition = body[paren + 1 : paren_end].strip()
    block = body[brace + 1 : brace_end]
    factory = "InvalidArgumentError"
    if factory not in block:
      raise ValueError("validation branch does not return InvalidArgumentError")
    if _status_name(factory) != documented:
      raise ValueError("return status does not match the header comment")
    pieces = _message_pieces(block)
    rules.append(
        {
            "factory": factory,
            "kind": _condition_kind(condition),
            "message_template": _template(pieces),
            "status_code": documented,
        }
    )
    cursor = brace_end + 1
  kinds = [rule["kind"] for rule in rules]
  if kinds != ["size_mismatch", "empty_point_path"]:
    raise ValueError(f"unexpected validation order: {kinds}")
  if not re.search(r"\breturn\s+request\s*;", body):
    raise ValueError("success path does not return the request")
  for setter in _SETTERS:
    if setter not in body:
      raise ValueError(f"factory no longer writes via {setter}")
  return rules


def _load_schema_tool():
  spec = importlib.util.spec_from_file_location(
      "extract_public_schema_inventory", _SCHEMA_TOOL
  )
  if spec is None or spec.loader is None:
    raise RuntimeError("schema inventory tool was not found")
  module = importlib.util.module_from_spec(spec)
  spec.loader.exec_module(module)
  return module


def _parse_protos(tool) -> dict[str, dict]:
  parsed: dict[str, dict] = {}
  for key, relative in _PROTO_PATHS.items():
    path = _REPO / relative
    parsed[key] = tool.parse_proto(_read(path), relative)
  return parsed


def _message(schema: dict, name: str) -> dict:
  for message in schema["messages"]:
    if message["name"] == name:
      return message
  raise ValueError(f"message {name} was not found in {schema.get('path')}")


def _field(schema: dict, message_name: str, field_name: str) -> dict:
  message = _message(schema, message_name)
  for field in message["fields"]:
    if field["name"] == field_name:
      return field
  raise ValueError(f"{message_name}.{field_name} was not found")


def _enum_values(schema: dict, qualified: str) -> list[dict[str, int | str]]:
  for enum in schema["enums"]:
    if enum["qualified_name"] == qualified:
      return [
          {"name": value["name"], "number": value["number"]}
          for value in enum["values"]
      ]
  raise ValueError(f"enum {qualified} was not found")


def _step(
    schemas: dict[str, dict], proto_key: str, message: str, name: str
) -> dict:
  field = _field(schemas[proto_key], message, name)
  step = {
      "label": field["label"],
      "message": message,
      "name": name,
      "number": field["number"],
      "type": field["type"],
  }
  if field["oneof"]:
    step["oneof"] = field["oneof"]
  return step


def _serialization(schemas: dict[str, dict]) -> dict:
  motion_type = _enum_values(
      schemas["motion_specification"], "MotionSegment.MotionType"
  )
  leaves = [
      {
          "leaf": "object_name",
          "steps": [
              _step(
                  schemas,
                  "motion_planner_service",
                  "MotionPlanningRequest",
                  "robot_specification",
              ),
              _step(
                  schemas,
                  "robot_specification",
                  "RobotSpecification",
                  "robot_reference",
              ),
              _step(
                  schemas, "robot_specification", "RobotReference", "object_id"
              ),
              _step(schemas, "object_world_refs", "ObjectReference", "by_name"),
              _step(
                  schemas,
                  "object_world_refs",
                  "ObjectReferenceByName",
                  "object_name",
              ),
          ],
      },
      {
          "leaf": "motion_type",
          "steps": [
              _step(
                  schemas,
                  "motion_planner_service",
                  "MotionPlanningRequest",
                  "motion_specification",
              ),
              _step(
                  schemas,
                  "motion_specification",
                  "MotionSpecification",
                  "motion_segments",
              ),
              _step(
                  schemas,
                  "motion_specification",
                  "MotionSegment",
                  "motion_type",
              ),
          ],
      },
      {
          "leaf": "joints",
          "steps": [
              _step(
                  schemas,
                  "motion_planner_service",
                  "MotionPlanningRequest",
                  "motion_specification",
              ),
              _step(
                  schemas,
                  "motion_specification",
                  "MotionSpecification",
                  "motion_segments",
              ),
              _step(schemas, "motion_specification", "MotionSegment", "target"),
              _step(
                  schemas,
                  "geometric_constraints",
                  "GeometricConstraint",
                  "joint_position",
              ),
              _step(schemas, "joint_space", "JointVec", "joints"),
          ],
      },
  ]
  request_fields = _message(
      schemas["motion_planner_service"], "MotionPlanningRequest"
  )["fields"]
  written = {"motion_specification", "robot_specification"}
  unset = [
      {
          "label": field["label"],
          "name": field["name"],
          "number": field["number"],
          "type": field["type"],
      }
      for field in sorted(request_fields, key=lambda item: item["number"])
      if field["name"] not in written
  ]
  return {
      "fields_left_unset": unset,
      "fields_written": leaves,
      "message": "intrinsic_proto.motion_planning.v1.MotionPlanningRequest",
      "motion_type_values": motion_type,
      "proto": _PROTO_PATHS["motion_planner_service"],
  }


def _fill_message(template: str, point_path: list, motion_types: list) -> str:
  return template.format(
      motion_types_size=len(motion_types),
      point_path_size=len(point_path),
  )


def _evaluate(rules: list[dict], case: dict) -> dict:
  point_path = case["point_path"]
  motion_types = case["motion_types"]
  for rule in rules:
    fired = False
    if rule["kind"] == "size_mismatch":
      fired = len(point_path) != len(motion_types)
    elif rule["kind"] == "empty_point_path":
      fired = len(point_path) == 0
    else:
      raise ValueError(f"unknown rule {rule['kind']}")
    if fired:
      return {
          "motion_segment_count": 0,
          "motion_type": None,
          "motion_type_number": None,
          "request_textproto": None,
          "robot_object_name": None,
          "status_code": rule["status_code"],
          "status_message": _fill_message(
              rule["message_template"], point_path, motion_types
          ),
          "target_joints": None,
      }
  return {
      "motion_segment_count": len(point_path),
      "motion_type": None,
      "motion_type_number": None,
      "request_textproto": None,
      "robot_object_name": case["robot_name"],
      "status_code": "OK",
      "status_message": "",
      "target_joints": point_path,
  }


def _format_double(value: float) -> str:
  text = format(value, ".12g")
  if "e" in text or "E" in text:
    text = format(value, ".12f").rstrip("0").rstrip(".")
  if "." not in text:
    text += ".0"
  return text


def _textproto(case: dict) -> str:
  joint_lines = "\n".join(
      f"        joints: {_format_double(value)}"
      for value in case["point_path"][0]
  )
  motion_type = case["motion_types"][0]
  return (
      "robot_specification {\n"
      "  robot_reference {\n"
      "    object_id {\n"
      "      by_name {\n"
      f'        object_name: "{case["robot_name"]}"\n'
      "      }\n"
      "    }\n"
      "  }\n"
      "}\n"
      "motion_specification {\n"
      "  motion_segments {\n"
      "    target {\n"
      "      joint_position {\n"
      f"{joint_lines}\n"
      "      }\n"
      "    }\n"
      f"    motion_type: {motion_type}\n"
      "  }\n"
      "}\n"
  )


def _motion_type_number(serialization: dict, name: str) -> int:
  for value in serialization["motion_type_values"]:
    if value["name"] == name:
      return int(value["number"])
  raise ValueError(f"motion type {name} is not in MotionSegment.MotionType")


def build_record(cases: list[dict] | None = None) -> dict:
  """Build the planning-request record from the current factory and protos."""
  cases = REPRESENTATIVE_CASES if cases is None else cases
  body = _function_body(
      _strip_comments(_read(_UTILS_CC)),
      "CreateMotionPlanningRequestFromPointPath",
  )
  documented = _documented_status(_read(_UTILS_H))
  rules = _validation_rules(body, documented)
  schemas = _parse_protos(_load_schema_tool())
  serialization = _serialization(schemas)
  rendered_cases = []
  for case in cases:
    for motion_type in case["motion_types"]:
      _motion_type_number(serialization, motion_type)
    result = _evaluate(rules, case)
    if result["status_code"] == "OK":
      if len(case["motion_types"]) != 1:
        raise ValueError("the valid request is one motion segment")
      result["motion_type"] = case["motion_types"][0]
      result["motion_type_number"] = _motion_type_number(
          serialization, case["motion_types"][0]
      )
      result["request_textproto"] = _textproto(case).splitlines()
    rendered_cases.append(
        {
            "input": {
                "motion_types": case["motion_types"],
                "point_path": case["point_path"],
                "robot_name": case["robot_name"],
            },
            "name": case["name"],
            "result": result,
        }
    )
  return {
      "cases": rendered_cases,
      "factory": {
          "documented_status_code": documented,
          "function": "CreateMotionPlanningRequestFromPointPath",
          "header": _UTILS_H.relative_to(_REPO).as_posix(),
          "source": _UTILS_CC.relative_to(_REPO).as_posix(),
          "success_status_code": "OK",
      },
      "schema": SCHEMA,
      "serialization": serialization,
      "validation_rules": rules,
  }


def render_golden(cases: list[dict] | None = None) -> str:
  return (
      json.dumps(
          build_record(cases), ensure_ascii=True, indent=2, sort_keys=True
      )
      + "\n"
  )


def assert_no_vehicle_feature(text: str) -> None:
  lowered = text.lower()
  found = [token for token in FORBIDDEN_TOKENS if token in lowered]
  if found:
    raise AssertionError(
        "manipulator planning-request fixture must not reference vehicle "
        "features: " + ", ".join(found)
    )


def compatibility_report(baseline: dict, candidate: dict) -> list[str]:
  """Name status, message, and serialization differences."""
  lines: list[str] = []
  if baseline.get("serialization") != candidate.get("serialization"):
    lines.append("changed serialization fields")
  if baseline.get("validation_rules") != candidate.get("validation_rules"):
    lines.append("changed validation rules")

  def by_name(record: dict) -> dict[str, dict]:
    return {case["name"]: case for case in record.get("cases", [])}

  base_cases = by_name(baseline)
  cand_cases = by_name(candidate)
  for name in sorted(set(base_cases) - set(cand_cases)):
    lines.append(f"removed request {name}")
  for name in sorted(set(cand_cases) - set(base_cases)):
    lines.append(f"added request {name}")
  for name in sorted(set(base_cases) & set(cand_cases)):
    base_result = base_cases[name]["result"]
    cand_result = cand_cases[name]["result"]
    if base_result["status_code"] != cand_result["status_code"]:
      lines.append(
          f"changed status code for {name}: "
          f"{base_result['status_code']} -> {cand_result['status_code']}"
      )
    if base_result["status_message"] != cand_result["status_message"]:
      lines.append(f"changed status message for {name}")
    if base_result.get("request_textproto") != cand_result.get(
        "request_textproto"
    ):
      lines.append(f"changed request textproto for {name}")
    stable = (
        "motion_segment_count",
        "motion_type",
        "motion_type_number",
        "robot_object_name",
        "target_joints",
    )
    if any(base_result.get(key) != cand_result.get(key) for key in stable):
      lines.append(f"changed stable output fields for {name}")
  return lines


class ManipulatorPlanningRequestGoldenTest(unittest.TestCase):
  """Checks the checked-in manipulator planning-request golden."""

  def test_golden_matches_current_factory(self) -> None:
    first = render_golden()
    second = render_golden()
    self.assertEqual(first, second)
    assert_no_vehicle_feature(first)
    expected = _GOLDEN.read_text(encoding="utf-8")
    if expected != first:
      report = compatibility_report(json.loads(expected), json.loads(first))
      diff = "\n".join(
          difflib.unified_diff(
              expected.splitlines(),
              first.splitlines(),
              fromfile=str(_GOLDEN.relative_to(_REPO)),
              tofile="current planning request",
              lineterm="",
          )
      )
      self.fail(
          "manipulator planning-request golden does not match the current "
          "factory.\nUpdating the fixture requires an explicit flag and "
          "senior review:\n"
          "  python3 intrinsic_motion_planning/intrinsic/motion_planning/"
          "service/planning_request_golden_test.py --update-golden\n"
          + "\n".join(report)
          + "\n"
          + diff
      )

  def test_compare_does_not_rewrite_golden(self) -> None:
    before = _GOLDEN.read_bytes()
    render_golden()
    self.assertEqual(before, _GOLDEN.read_bytes())

  def test_fixture_covers_one_valid_and_two_invalid(self) -> None:
    record = json.loads(_GOLDEN.read_text(encoding="utf-8"))
    self.assertEqual(record["schema"], SCHEMA)
    self.assertEqual(len(record["cases"]), 3)
    codes = [case["result"]["status_code"] for case in record["cases"]]
    self.assertEqual(codes.count("OK"), 1)
    self.assertEqual(codes.count("INVALID_ARGUMENT"), 2)
    names = [case["name"] for case in record["cases"]]
    self.assertEqual(
        names,
        [
            "valid_joint_segment",
            "invalid_size_mismatch",
            "invalid_empty_point_path",
        ],
    )

  def test_status_codes_and_stable_fields(self) -> None:
    record = build_record()
    by_name = {case["name"]: case for case in record["cases"]}
    valid = by_name["valid_joint_segment"]["result"]
    self.assertEqual(valid["status_code"], "OK")
    self.assertEqual(valid["status_message"], "")
    self.assertEqual(valid["robot_object_name"], "agilus_04")
    self.assertEqual(valid["motion_type"], "JOINT")
    self.assertEqual(valid["motion_type_number"], 2)
    self.assertEqual(valid["motion_segment_count"], 1)
    self.assertEqual(valid["target_joints"], [_JOINTS])
    textproto = "\n".join(valid["request_textproto"])
    self.assertIn('object_name: "agilus_04"', textproto)
    self.assertIn("motion_type: JOINT", textproto)
    self.assertNotIn("world_id", textproto)

    mismatch = by_name["invalid_size_mismatch"]["result"]
    self.assertEqual(mismatch["status_code"], "INVALID_ARGUMENT")
    self.assertEqual(
        mismatch["status_message"],
        "The size of `point_path` and `motion_types` must be the "
        "same, but got 1 and 2, respectively.",
    )
    self.assertIsNone(mismatch["request_textproto"])
    self.assertIsNone(mismatch["target_joints"])

    empty = by_name["invalid_empty_point_path"]["result"]
    self.assertEqual(empty["status_code"], "INVALID_ARGUMENT")
    self.assertEqual(
        empty["status_message"],
        "The size of `point_path` must be greater than 0.",
    )
    self.assertIsNone(empty["request_textproto"])
    self.assertEqual(empty["motion_segment_count"], 0)

    written = {
        leaf["leaf"]: [step["number"] for step in leaf["steps"]]
        for leaf in record["serialization"]["fields_written"]
    }
    self.assertEqual(written["object_name"], [2, 1, 1, 2, 1])
    self.assertEqual(written["motion_type"], [3, 1, 15])
    self.assertEqual(written["joints"], [3, 1, 5, 13, 1])
    unset = {
        field["name"] for field in record["serialization"]["fields_left_unset"]
    }
    self.assertIn("world_id", unset)
    self.assertNotIn("robot_specification", unset)
    self.assertNotIn("motion_specification", unset)

  def test_no_vehicle_feature_in_fixture(self) -> None:
    text = _GOLDEN.read_text(encoding="utf-8")
    assert_no_vehicle_feature(text)
    record = json.loads(text)
    self.assertEqual(record["cases"][0]["input"]["robot_name"], "agilus_04")

  def test_changed_status_is_reported(self) -> None:
    record = build_record()
    mutated = json.loads(json.dumps(record))
    mutated["cases"][1]["result"]["status_code"] = "INTERNAL"
    mutated["cases"][1]["result"]["status_message"] = "changed"
    mutated["validation_rules"][0]["status_code"] = "INTERNAL"
    report = compatibility_report(record, mutated)
    self.assertTrue(any("changed status code" in line for line in report))
    self.assertTrue(any("changed status message" in line for line in report))
    self.assertIn("changed validation rules", report)

  def test_changed_field_number_is_reported(self) -> None:
    record = build_record()
    mutated = json.loads(json.dumps(record))
    mutated["serialization"]["fields_written"][2]["steps"][-1]["number"] = 99
    mutated["cases"][0]["result"]["request_textproto"] = "mutated"
    report = compatibility_report(record, mutated)
    self.assertIn("changed serialization fields", report)
    self.assertTrue(any("changed request textproto" in line for line in report))

  def test_update_command_is_documented(self) -> None:
    readme = (_DIR / "README.md").read_text(encoding="utf-8")
    self.assertIn("--update-golden", readme)
    self.assertIn("planning_request_golden_test.py", readme)

  def test_update_requires_explicit_flag(self) -> None:
    before = _GOLDEN.read_bytes()
    with contextlib.redirect_stderr(io.StringIO()):
      with self.assertRaises(SystemExit) as raised:
        main([])
    self.assertEqual(raised.exception.code, 2)
    self.assertEqual(before, _GOLDEN.read_bytes())

  def test_update_refuses_vehicle_feature(self) -> None:
    before = _GOLDEN.read_bytes()
    poisoned = json.loads(json.dumps(REPRESENTATIVE_CASES))
    poisoned[0]["robot_name"] = "uuv_arm"
    with self.assertRaises(AssertionError):
      update_golden(text=render_golden(poisoned))
    self.assertEqual(before, _GOLDEN.read_bytes())


def update_golden(path: Path = _GOLDEN, text: str | None = None) -> None:
  if text is None:
    text = render_golden()
    if text != render_golden():
      raise RuntimeError("planning-request render is not deterministic")
  assert_no_vehicle_feature(text)
  path.parent.mkdir(parents=True, exist_ok=True)
  path.write_text(text, encoding="utf-8")


def main(argv: list[str] | None = None) -> int:
  parser = argparse.ArgumentParser(description=__doc__)
  parser.add_argument(
      "--update-golden",
      action="store_true",
      help=(
          "Rewrite testdata/manipulator_planning_request.golden.json from the "
          "current factory and protos. This flag is required; comparison is "
          "the unittest module."
      ),
  )
  args = parser.parse_args(argv)
  if not args.update_golden:
    parser.error(
        "refusing to update the golden without --update-golden; "
        "run the unittest module to compare"
    )
  update_golden()
  print(f"updated {_GOLDEN.relative_to(_REPO)}")
  return 0


if __name__ == "__main__":
  if "--update-golden" in sys.argv:
    sys.exit(main(sys.argv[1:]))
  unittest.main()
