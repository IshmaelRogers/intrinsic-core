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

"""Golden for HalArmPart feature-interface registration.

Reads the current arm part and feature-interface registry sources and compares
them to a checked-in fixture. Production arm code is not modified. Rewriting
the fixture requires ``--update-golden`` (see README.md).
"""

from __future__ import annotations

import argparse
import contextlib
import difflib
import io
import json
from pathlib import Path
import re
import sys
import unittest

SCHEMA = "arm-feature-registration-v1"

# The fixture must stay free of vehicle / UUV feature names. Matching is
# case-insensitive against the rendered JSON.
FORBIDDEN_TOKENS = ("bathymetry", "marine", "thruster", "uuv", "vehicle")

_DIR = Path(__file__).resolve().parent
_GOLDEN = _DIR / "testdata" / "arm_feature_registration.golden.json"


def _repo_root() -> Path:
  for parent in _DIR.parents:
    if (parent / "MODULE.bazel").is_file():
      return parent
  raise RuntimeError("repository root was not found")


_REPO = _repo_root()

_ARM_CC = _DIR / "hal_arm_part.cc"
_ARM_H = _DIR / "hal_arm_part.h"
_ARM_REGISTER = _DIR / "hal_arm_part_register.cc"
_KINEMATICS_H = _DIR.parents[1] / "new_manipulator_kinematics.h"
_FEATURE_HEADERS = _DIR.parents[1] / "feature_interfaces"
_REGISTRY_CC = _DIR.parents[1] / "feature_interface_registry.cc"
_FACTORY_CC = _DIR.parents[1] / "realtime_part_from_proto_factory_registry.cc"
_STATUS_CC = _REPO / "intrinsic_control/intrinsic/icon/utils/realtime_status.cc"
_STATUS_H = _REPO / "intrinsic_control/intrinsic/icon/utils/realtime_status.h"
_TYPES_PROTO = _REPO / "intrinsic_apis/intrinsic/icon/proto/v1/types.proto"

_NOT_A_FEATURE_BASE = frozenset(
    {"HalFeatureInterfaceBase", "HalRealtimePartBase"}
)


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


def _string_literals(text: str) -> list[str]:
  literals: list[str] = []
  i = 0
  while i < len(text):
    if text[i] != '"':
      i += 1
      continue
    i += 1
    chars: list[str] = []
    while i < len(text):
      if text[i] == "\\":
        if i + 1 >= len(text):
          break
        chars.append(text[i + 1])
        i += 2
        continue
      if text[i] == '"':
        break
      chars.append(text[i])
      i += 1
    literals.append("".join(chars))
    i += 1
  return literals


def _camel_to_status(name: str) -> str:
  if name.startswith("k"):
    name = name[1:]
  return re.sub(r"(?<!^)(?=[A-Z])", "_", name).upper()


def _status_code_by_factory(status_cc: str) -> dict[str, str]:
  """Map RealtimeStatus factory names to canonical status codes."""
  codes: dict[str, str] = {}
  pattern = re.compile(
      r"RealtimeStatus\s+(\w+)\(absl::string_view message\)\s*\{\s*"
      r"return\s*\{\s*absl::StatusCode::(k\w+)\s*,",
      re.S,
  )
  for match in pattern.finditer(status_cc):
    codes[match.group(1)] = _camel_to_status(match.group(2))
  if "InvalidArgumentError" not in codes or "AlreadyExistsError" not in codes:
    raise ValueError("realtime status factories were not found")
  return codes


def _ok_status_code(status_h: str) -> str:
  match = re.search(
      r"absl::StatusCode\s+code_\s*=\s*absl::StatusCode::(k\w+)\s*;", status_h
  )
  if match is None:
    raise ValueError("default RealtimeStatus code was not found")
  return _camel_to_status(match.group(1))


def _feature_enum_numbers(types_proto: str) -> dict[str, int]:
  start = types_proto.find("enum FeatureInterfaceTypes")
  if start < 0:
    raise ValueError("FeatureInterfaceTypes enum was not found")
  brace = types_proto.find("{", start)
  end = _matching_delimiter(types_proto, brace, "{", "}")
  body = types_proto[brace + 1 : end]
  numbers: dict[str, int] = {}
  for match in re.finditer(r"(FEATURE_INTERFACE_\w+)\s*=\s*(\d+)\s*;", body):
    numbers[match.group(1)] = int(match.group(2))
  if not numbers:
    raise ValueError("FeatureInterfaceTypes has no values")
  return numbers


def _registry_bindings(
    registry_cc: str, status_codes: dict[str, str], ok_code: str
) -> dict[str, dict]:
  """Map a C++ feature-interface type to its enum and status messages."""
  fn_start = registry_cc.find("SupportedFeatureInterfaceTypes()")
  if fn_start < 0:
    raise ValueError("SupportedFeatureInterfaceTypes was not found")
  brace = registry_cc.find("{", fn_start)
  end = _matching_delimiter(registry_cc, brace, "{", "}")
  supported = registry_cc[brace + 1 : end]
  member_to_enum: dict[str, str] = {}
  member_pattern = re.compile(
      r"if\s*\(\s*(\w+)\s*!=\s*nullptr\s*\)\s*\{\s*"
      r"interfaces\.insert\(\s*FeatureInterfaceTypes::\s*(\w+)\s*\)\s*;",
      re.S,
  )
  for match in member_pattern.finditer(supported):
    member_to_enum[match.group(1)] = match.group(2)

  bindings: dict[str, dict] = {}
  cursor = 0
  needle = "RegisterInterface("
  while True:
    idx = registry_cc.find(needle, cursor)
    if idx < 0:
      break
    open_paren = registry_cc.find("(", idx)
    close_paren = _matching_delimiter(registry_cc, open_paren, "(", ")")
    signature = registry_cc[idx : close_paren + 1]
    typed = re.search(r"RegisterInterface\(\s*(\w+)\s*\*\s*v\s*\)", signature)
    if typed is None:
      cursor = close_paren + 1
      continue
    brace = registry_cc.find("{", close_paren)
    between = registry_cc[close_paren + 1 : brace]
    # Declarations end in `;` before any body. Specializations open a body.
    if brace < 0 or ";" in between:
      cursor = close_paren + 1
      continue
    body_end = _matching_delimiter(registry_cc, brace, "{", "}")
    body = registry_cc[brace + 1 : body_end]
    cursor = body_end + 1
    if "return OkStatus();" not in body:
      continue
    cpp_type = typed.group(1)
    member_match = re.search(r"(\w+)\s*=\s*v\s*;", body)
    if member_match is None:
      raise ValueError(f"{cpp_type} registration does not assign a member")
    member = member_match.group(1)
    feature_type = member_to_enum.get(member)
    if feature_type is None:
      raise ValueError(f"{cpp_type} member {member} has no feature type")
    bindings[cpp_type] = {
        "feature_type": feature_type,
        "statuses": {
            "already_registered": {
                "code": status_codes["AlreadyExistsError"],
                "message": _call_message(body, "AlreadyExistsError"),
            },
            "nullptr": {
                "code": status_codes["InvalidArgumentError"],
                "message": _call_message(body, "InvalidArgumentError"),
            },
            "ok": {"code": ok_code, "message": ""},
        },
    }
  if not bindings:
    raise ValueError("no feature-interface registrations were parsed")
  return bindings


def _call_message(body: str, function_name: str) -> str:
  idx = body.find(function_name + "(")
  if idx < 0:
    raise ValueError(f"{function_name} was not found")
  open_paren = body.find("(", idx)
  close_paren = _matching_delimiter(body, open_paren, "(", ")")
  literals = _string_literals(body[open_paren + 1 : close_paren])
  if not literals:
    raise ValueError(f"{function_name} has no message literal")
  return "".join(literals)


def _class_bases(headers: list[tuple[str, str]]) -> dict[str, list[str]]:
  found: dict[str, list[str]] = {}
  for _path, text in headers:
    stripped = _strip_comments(text)
    for match in re.finditer(
        r"\bclass\s+(\w+)\s+(?:final\s+)?:\s*(.*?)\s*\{",
        stripped,
        re.S,
    ):
      name = match.group(1)
      bases = re.findall(r"\bpublic\s+(\w+)", match.group(2))
      if name in found and found[name] != bases:
        raise ValueError(f"conflicting bases for {name}")
      found[name] = bases
  return found


def _arm_classes(arm_cc: str) -> dict[str, int]:
  """Count construction sites that HalArmPart then registers."""
  counts: dict[str, int] = {}

  def add(name: str, amount: int) -> None:
    if amount:
      counts[name] = counts.get(name, 0) + amount

  for match in re.finditer(
      r"Map(?:Mutable)?HardwareInterfaceToFeatureInterface<"
      r"\s*[\w:]+\s*,\s*(\w+)\s*>",
      arm_cc,
  ):
    add(match.group(1), 1)
  for match in re.finditer(r"\b(\w+Feature)::Create\s*\(", arm_cc):
    add(match.group(1), 1)
  for match in re.finditer(r"std::make_unique<(\w+)>", arm_cc):
    name = match.group(1)
    if name in ("NewManipulatorKinematicsImpl", "DynamicsImpl"):
      add(name, 1)
  if not counts:
    raise ValueError("HalArmPart registers no feature classes")
  return counts


def _part_identity(header: str, register_cc: str) -> dict[str, str]:
  name_match = re.search(r'kPartTypeName\[\]\s*=\s*"([^"]+)"', header)
  if name_match is None:
    raise ValueError("kPartTypeName was not found")
  reg_match = re.search(
      r"RegisterTyped<\s*([^>\s]+)\s*>\(\s*"
      r"HalArmPart::kPartTypeName\s*,\s*&([^)\s]+)\s*\)",
      register_cc,
  )
  if reg_match is None:
    raise ValueError("HalArmPart RegisterTyped call was not found")
  return {
      "config_message": reg_match.group(1).replace("::", "."),
      "factory": reg_match.group(2),
      "part_type_name": name_match.group(1),
      "registry_call": "RegisterTyped",
  }


def _part_factory_statuses(factory_cc: str) -> list[dict[str, str]]:
  register_at = factory_cc.find(
      "RealtimePartFromProtoFactoryRegistry::Register("
  )
  if register_at < 0:
    raise ValueError("part factory Register was not found")
  brace = factory_cc.find("{", register_at)
  end = _matching_delimiter(factory_cc, brace, "{", "}")
  body = factory_cc[brace + 1 : end]
  duplicate = re.search(
      r"if\s*\(\s*registry_\.contains\(part_type_name\)\s*\)\s*"
      r"return\s+(true|false)\s*;",
      body,
  )
  inserted = re.search(
      r"registry_\.emplace\([^;]+;\s*return\s+(true|false)\s*;",
      body,
  )
  get_at = factory_cc.find("RealtimePartFromProtoFactoryRegistry::Get(")
  if get_at < 0 or duplicate is None or inserted is None:
    raise ValueError("part factory status returns were not found")
  get_brace = factory_cc.find("{", get_at)
  get_end = _matching_delimiter(factory_cc, get_brace, "{", "}")
  get_body = factory_cc[get_brace + 1 : get_end]
  if "return {};" not in get_body:
    raise ValueError(
        "missing part-type lookup does not return an empty factory"
    )
  return [
      {
          "condition": "part_type_already_registered",
          "result": duplicate.group(1),
      },
      {"condition": "part_type_inserted", "result": inserted.group(1)},
      {"condition": "part_type_not_found", "result": "empty_factory"},
  ]


def _from_proto_statuses(arm_cc: str) -> list[dict]:
  statuses: list[dict] = []
  cursor = 0
  while True:
    match = re.search(r"absl::(\w+)Error\s*\(", arm_cc[cursor:])
    if match is None:
      break
    name = match.group(1)
    open_paren = cursor + match.end() - 1
    close_paren = _matching_delimiter(arm_cc, open_paren, "(", ")")
    literals = _string_literals(arm_cc[open_paren + 1 : close_paren])
    if not literals:
      raise ValueError(f"absl::{name}Error has no message literal")
    statuses.append(
        {
            "code": _camel_to_status(name),
            "message_literals": literals,
        }
    )
    cursor = close_paren + 1
  unique = {json.dumps(item, sort_keys=True): item for item in statuses}
  return [unique[key] for key in sorted(unique)]


def _load_sources() -> dict[str, str]:
  headers = [
      (path.relative_to(_REPO).as_posix(), _read(path))
      for path in sorted(_FEATURE_HEADERS.glob("*.h"))
  ]
  headers.append(
      (_KINEMATICS_H.relative_to(_REPO).as_posix(), _read(_KINEMATICS_H))
  )
  headers.append((_ARM_H.relative_to(_REPO).as_posix(), _read(_ARM_H)))
  return {
      "arm_cc": _strip_comments(_read(_ARM_CC)),
      "arm_h": _read(_ARM_H),
      "factory_cc": _read(_FACTORY_CC),
      "headers": headers,
      "register_cc": _read(_ARM_REGISTER),
      "registry_cc": _read(_REGISTRY_CC),
      "status_cc": _read(_STATUS_CC),
      "status_h": _read(_STATUS_H),
      "types_proto": _read(_TYPES_PROTO),
  }


def build_record(sources: dict | None = None) -> dict:
  """Build the registration record from the current arm sources."""
  sources = sources if sources is not None else _load_sources()
  status_codes = _status_code_by_factory(sources["status_cc"])
  ok_code = _ok_status_code(sources["status_h"])
  enum_numbers = _feature_enum_numbers(sources["types_proto"])
  bindings = _registry_bindings(sources["registry_cc"], status_codes, ok_code)
  bases = _class_bases(sources["headers"])
  class_counts = _arm_classes(sources["arm_cc"])
  registrations: list[dict] = []
  for feature_class, call_sites in sorted(class_counts.items()):
    if feature_class not in bases:
      raise ValueError(f"{feature_class} bases were not found")
    interfaces = [
        base
        for base in bases[feature_class]
        if base not in _NOT_A_FEATURE_BASE and base in bindings
    ]
    if not interfaces:
      raise ValueError(f"{feature_class} registers no known feature interface")
    for cpp_interface in interfaces:
      binding = bindings[cpp_interface]
      feature_type = binding["feature_type"]
      if feature_type not in enum_numbers:
        raise ValueError(f"{feature_type} is not in FeatureInterfaceTypes")
      registrations.append(
          {
              "call_sites": call_sites,
              "cpp_interface": cpp_interface,
              "feature_class": feature_class,
              "feature_type": feature_type,
              "feature_type_number": enum_numbers[feature_type],
              "statuses": binding["statuses"],
          }
      )
  registrations.sort(
      key=lambda row: (
          row["feature_type_number"],
          row["feature_class"],
          row["cpp_interface"],
      )
  )
  supported = []
  seen: set[str] = set()
  for row in registrations:
    if row["feature_type"] in seen:
      continue
    seen.add(row["feature_type"])
    supported.append(
        {
            "feature_type": row["feature_type"],
            "feature_type_number": row["feature_type_number"],
        }
    )
  part = _part_identity(sources["arm_h"], sources["register_cc"])
  part["part_factory_statuses"] = _part_factory_statuses(sources["factory_cc"])
  return {
      "from_proto_status_values": _from_proto_statuses(sources["arm_cc"]),
      "part": part,
      "registrations": registrations,
      "schema": SCHEMA,
      "supported_feature_types": supported,
  }


def render_golden(sources: dict | None = None) -> str:
  record = build_record(sources)
  return json.dumps(record, ensure_ascii=True, indent=2, sort_keys=True) + "\n"


def assert_no_vehicle_feature(text: str) -> None:
  lowered = text.lower()
  found = [token for token in FORBIDDEN_TOKENS if token in lowered]
  if found:
    raise AssertionError(
        "arm feature-registration fixture must not reference vehicle features: "
        + ", ".join(found)
    )


def compatibility_report(baseline: dict, candidate: dict) -> list[str]:
  """Name feature-type and status differences between two records."""
  lines: list[str] = []
  base_types = {
      item["feature_type"] for item in baseline["supported_feature_types"]
  }
  cand_types = {
      item["feature_type"] for item in candidate["supported_feature_types"]
  }
  for name in sorted(base_types - cand_types):
    lines.append(f"removed feature type {name}")
  for name in sorted(cand_types - base_types):
    lines.append(f"added feature type {name}")
  base_numbers = {
      item["feature_type"]: item["feature_type_number"]
      for item in baseline["supported_feature_types"]
  }
  cand_numbers = {
      item["feature_type"]: item["feature_type_number"]
      for item in candidate["supported_feature_types"]
  }
  for name in sorted(base_types & cand_types):
    if base_numbers[name] != cand_numbers[name]:
      lines.append(
          f"renumbered feature type {name} "
          f"{base_numbers[name]} -> {cand_numbers[name]}"
      )

  def status_map(record: dict) -> dict[tuple[str, str], dict]:
    return {
        (row["feature_class"], row["cpp_interface"]): row["statuses"]
        for row in record["registrations"]
    }

  base_status = status_map(baseline)
  cand_status = status_map(candidate)
  for key in sorted(set(base_status) & set(cand_status)):
    if base_status[key] != cand_status[key]:
      lines.append(f"changed status values for {key[0]} {key[1]}")
  for key in sorted(set(base_status) - set(cand_status)):
    lines.append(f"removed registration {key[0]} {key[1]}")
  for key in sorted(set(cand_status) - set(base_status)):
    lines.append(f"added registration {key[0]} {key[1]}")
  if baseline.get("part", {}).get("part_type_name") != candidate.get(
      "part", {}
  ).get("part_type_name"):
    lines.append("changed part type name")
  if baseline.get("from_proto_status_values") != candidate.get(
      "from_proto_status_values"
  ):
    lines.append("changed from_proto status values")
  return lines


class ArmFeatureRegistrationGoldenTest(unittest.TestCase):
  """Checks the checked-in arm registration golden."""

  def test_golden_matches_current_registration(self) -> None:
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
              tofile="current arm registration",
              lineterm="",
          )
      )
      self.fail(
          "arm feature-registration golden does not match the current arm.\n"
          "Updating the fixture requires an explicit flag and senior review:\n"
          "  python3 intrinsic_control/intrinsic/icon/control/parts/hal/"
          "arm_part/feature_registration_golden_test.py --update-golden\n"
          + "\n".join(report)
          + "\n"
          + diff
      )

  def test_compare_does_not_rewrite_golden(self) -> None:
    before = _GOLDEN.read_bytes()
    render_golden()
    self.assertEqual(before, _GOLDEN.read_bytes())

  def test_no_vehicle_feature_in_fixture(self) -> None:
    assert_no_vehicle_feature(_GOLDEN.read_text(encoding="utf-8"))
    record = json.loads(_GOLDEN.read_text(encoding="utf-8"))
    for row in record["supported_feature_types"]:
      assert_no_vehicle_feature(row["feature_type"])
    self.assertEqual(record["schema"], SCHEMA)
    self.assertEqual(record["part"]["part_type_name"], "HalArmPart")

  def test_supported_types_have_status_values(self) -> None:
    record = build_record()
    supported = {
        item["feature_type"] for item in record["supported_feature_types"]
    }
    registered = {row["feature_type"] for row in record["registrations"]}
    self.assertEqual(supported, registered)
    for row in record["registrations"]:
      statuses = row["statuses"]
      self.assertEqual(statuses["ok"]["code"], "OK")
      self.assertEqual(statuses["nullptr"]["code"], "INVALID_ARGUMENT")
      self.assertEqual(statuses["already_registered"]["code"], "ALREADY_EXISTS")
      self.assertIn(row["cpp_interface"], statuses["nullptr"]["message"])
      self.assertGreaterEqual(row["call_sites"], 1)
      self.assertGreater(row["feature_type_number"], 0)

  def test_removed_feature_type_is_reported(self) -> None:
    record = build_record()
    mutated = json.loads(json.dumps(record))
    removed = mutated["supported_feature_types"].pop(0)
    mutated["registrations"] = [
        row
        for row in mutated["registrations"]
        if row["feature_type"] != removed["feature_type"]
    ]
    report = compatibility_report(record, mutated)
    self.assertIn(f"removed feature type {removed['feature_type']}", report)

  def test_changed_status_is_reported(self) -> None:
    record = build_record()
    mutated = json.loads(json.dumps(record))
    mutated["registrations"][0]["statuses"]["ok"]["code"] = "INTERNAL"
    mutated["from_proto_status_values"] = []
    report = compatibility_report(record, mutated)
    self.assertTrue(
        any(line.startswith("changed status values") for line in report)
    )
    self.assertIn("changed from_proto status values", report)

  def test_update_command_is_documented(self) -> None:
    readme = (_DIR / "README.md").read_text(encoding="utf-8")
    self.assertIn("--update-golden", readme)
    self.assertIn("feature_registration_golden_test.py", readme)

  def test_update_requires_explicit_flag(self) -> None:
    before = _GOLDEN.read_bytes()
    with contextlib.redirect_stderr(io.StringIO()):
      with self.assertRaises(SystemExit) as raised:
        main([])
    self.assertEqual(raised.exception.code, 2)
    self.assertEqual(before, _GOLDEN.read_bytes())


def update_golden(path: Path = _GOLDEN) -> None:
  first = render_golden()
  second = render_golden()
  if first != second:
    raise RuntimeError("arm feature-registration render is not deterministic")
  assert_no_vehicle_feature(first)
  path.parent.mkdir(parents=True, exist_ok=True)
  path.write_text(first, encoding="utf-8")


def main(argv: list[str] | None = None) -> int:
  parser = argparse.ArgumentParser(description=__doc__)
  parser.add_argument(
      "--update-golden",
      action="store_true",
      help=(
          "Rewrite testdata/arm_feature_registration.golden.json from the "
          "current arm sources. This flag is required; comparison is the "
          "unittest module."
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
