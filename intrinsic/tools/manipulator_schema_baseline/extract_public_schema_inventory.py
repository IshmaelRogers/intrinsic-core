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

"""Read-only inventory of public manipulator protobuf and FlatBuffer symbols.

Scans existing ``.proto`` and ``.fbs`` sources. Does not edit schemas, generate
stubs, or change runtime behavior. The selection rule is
``manipulator-public-wire-v1`` (see README.md).
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import re
import sys
from typing import Any

SELECTION_RULE_ID = "manipulator-public-wire-v1"

SELECTION_RULE = (
    "Inventory existing .proto and .fbs sources on the public manipulator "
    "wire surface only. Include intrinsic_apis packages for motion planning, "
    "ICON, kinematics, math, world, and geometry; skill motion targets and "
    "skill-parameter field metadata; ICON action and skill parameter protos "
    "under intrinsic_control; Cartesian force payload protos referenced by "
    "those actions; SDK service protos for ObjectWorld, RobotComponent, "
    "GenericAction, MotionPlanner, and move_robot (intrinsic_sdk has no "
    ".proto files); and ICON HAL FlatBuffers plus the shared flatbuffer "
    "types they include. Skip test fixtures. Record imports outside this "
    "allowlist without expanding them. Do not invent schemas."
)

DIRECTORY_ALLOWLIST = (
    "intrinsic_apis/intrinsic/geometry",
    "intrinsic_apis/intrinsic/icon",
    "intrinsic_apis/intrinsic/kinematics",
    "intrinsic_apis/intrinsic/math",
    "intrinsic_apis/intrinsic/motion_planning",
    "intrinsic_apis/intrinsic/world/proto",
    "intrinsic_control/intrinsic/icon/actions",
    "intrinsic_control/intrinsic/icon/flatbuffers",
    "intrinsic_control/intrinsic/icon/hal/interfaces",
    "intrinsic_control/intrinsic/icon/skills",
)

FILE_ALLOWLIST = (
    "intrinsic/manipulation/skills/force/contact_stiffness.proto",
    "intrinsic/world/proto/generic_action.proto",
    "intrinsic/world/proto/object_world_service.proto",
    "intrinsic/world/proto/object_world_updates.proto",
    "intrinsic/world/proto/robot_component.proto",
    "intrinsic_apis/intrinsic/skills/proto/motion_targets.proto",
    "intrinsic_apis/intrinsic/skills/proto/skill_parameter_metadata.proto",
    "intrinsic_control/intrinsic/icon/control/primitives/force_control/"
    "proto/controller_params.proto",
    "intrinsic_control/intrinsic/icon/control/primitives/force_control/"
    "proto/force_primitives.proto",
    "intrinsic_motion_planning/intrinsic/motion_planning/proto/v1/"
    "motion_planner_service.proto",
    "intrinsic_motion_planning/intrinsic/motion_planning/proto/v1/"
    "motion_specification.proto",
    "intrinsic_motion_planning/intrinsic/motion_planning/skills/"
    "move_robot.proto",
)

EXCLUDED_DIR_NAMES = frozenset(
    {"fixtures", "test", "testdata", "testing", "tests"}
)

INVENTORY_VERSION = 1


class ParseError(Exception):
  """A schema file could not be inventoried."""


class Token:
  """A lexer token with a 1-based source line."""

  def __init__(self, kind: str, text: str, line: int):
    self.kind = kind
    self.text = text
    self.line = line

  def __repr__(self) -> str:
    return f"Token({self.kind}, {self.text!r}, line={self.line})"


def _tokenize(text: str) -> list[Token]:
  """Tokenize proto or FlatBuffer source, dropping comments."""
  tokens: list[Token] = []
  i = 0
  line = 1
  n = len(text)
  while i < n:
    ch = text[i]
    if ch == "\n":
      line += 1
      i += 1
      continue
    if ch in " \t\r":
      i += 1
      continue
    if ch == "/" and i + 1 < n and text[i + 1] == "/":
      i += 2
      while i < n and text[i] != "\n":
        i += 1
      continue
    if ch == "/" and i + 1 < n and text[i + 1] == "*":
      i += 2
      while i + 1 < n and not (text[i] == "*" and text[i + 1] == "/"):
        if text[i] == "\n":
          line += 1
        i += 1
      if i + 1 >= n:
        raise ParseError(f"unterminated block comment at line {line}")
      i += 2
      continue
    if ch in "\"'":
      quote = ch
      i += 1
      buf: list[str] = []
      while i < n and text[i] != quote:
        if text[i] == "\n":
          line += 1
        if text[i] == "\\" and i + 1 < n:
          buf.append(text[i + 1])
          if text[i + 1] == "\n":
            line += 1
          i += 2
          continue
        buf.append(text[i])
        i += 1
      if i >= n:
        raise ParseError(f"unterminated string at line {line}")
      i += 1
      tokens.append(Token("string", "".join(buf), line))
      continue
    if text[i : i + 2] in ("0x", "0X"):
      start = i
      i += 2
      while i < n and text[i] in "0123456789abcdefABCDEF":
        i += 1
      tokens.append(Token("number", text[start:i], line))
      continue
    if ch.isdigit() or (ch == "." and i + 1 < n and text[i + 1].isdigit()):
      start = i
      if ch != ".":
        while i < n and text[i].isdigit():
          i += 1
      if i < n and text[i] == "." and not (i + 1 < n and text[i + 1].isalpha()):
        i += 1
        while i < n and text[i].isdigit():
          i += 1
      if i < n and text[i] in "eE":
        j = i + 1
        if j < n and text[j] in "+-":
          j += 1
        if j < n and text[j].isdigit():
          i = j
          while i < n and text[i].isdigit():
            i += 1
      tokens.append(Token("number", text[start:i], line))
      continue
    if ch.isalpha() or ch == "_":
      start = i
      i += 1
      while i < n and (text[i].isalnum() or text[i] == "_"):
        i += 1
      tokens.append(Token("ident", text[start:i], line))
      continue
    tokens.append(Token("punct", ch, line))
    i += 1
  tokens.append(Token("eof", "", line))
  return tokens


def _strip_for_counts(text: str) -> str:
  """Replace comments and strings with spaces, preserving newlines."""
  out: list[str] = []
  i = 0
  n = len(text)
  while i < n:
    ch = text[i]
    if ch == "/" and i + 1 < n and text[i + 1] == "/":
      while i < n and text[i] != "\n":
        out.append(" ")
        i += 1
      continue
    if ch == "/" and i + 1 < n and text[i + 1] == "*":
      out.extend((" ", " "))
      i += 2
      while i + 1 < n and not (text[i] == "*" and text[i + 1] == "/"):
        out.append("\n" if text[i] == "\n" else " ")
        i += 1
      if i + 1 < n:
        out.extend((" ", " "))
        i += 2
      continue
    if ch in "\"'":
      quote = ch
      out.append(" ")
      i += 1
      while i < n and text[i] != quote:
        if text[i] == "\\" and i + 1 < n:
          out.append(" ")
          out.append("\n" if text[i + 1] == "\n" else " ")
          i += 2
          continue
        out.append("\n" if text[i] == "\n" else " ")
        i += 1
      if i < n:
        out.append(" ")
        i += 1
      continue
    out.append(ch)
    i += 1
  return "".join(out)


def _count_decl(stripped: str, keyword: str) -> int:
  return len(re.findall(rf"\b{keyword}\s+[A-Za-z_]", stripped))


def _int_literal(text: str) -> int:
  return int(text, 0)


class _Cursor:
  """Token cursor shared by the proto and FlatBuffer parsers."""

  def __init__(self, tokens: list[Token], path: str):
    self.tokens = tokens
    self.path = path
    self.i = 0

  def peek(self) -> Token:
    return self.tokens[self.i]

  def advance(self) -> Token:
    tok = self.tokens[self.i]
    if tok.kind != "eof":
      self.i += 1
    return tok

  def fail(self, message: str) -> ParseError:
    tok = self.peek()
    return ParseError(f"{self.path}:{tok.line}: {message} (at {tok!r})")

  def expect_punct(self, text: str) -> None:
    tok = self.peek()
    if tok.kind != "punct" or tok.text != text:
      raise self.fail(f"expected {text!r}")
    self.advance()

  def expect_ident(self, text: str | None = None) -> str:
    tok = self.peek()
    if tok.kind != "ident" or (text is not None and tok.text != text):
      expected = text if text is not None else "identifier"
      raise self.fail(f"expected {expected}")
    self.advance()
    return tok.text

  def peek_is_ident(self, text: str | None = None) -> bool:
    tok = self.peek()
    if tok.kind != "ident":
      return False
    return text is None or tok.text == text

  def peek_is_punct(self, text: str) -> bool:
    tok = self.peek()
    return tok.kind == "punct" and tok.text == text

  def skip_until_semi(self) -> None:
    """Skip a statement, honoring nested braces, including its semicolon."""
    depth = 0
    while True:
      tok = self.peek()
      if tok.kind == "eof":
        raise self.fail("unterminated statement")
      self.advance()
      if tok.kind == "punct" and tok.text == "{":
        depth += 1
      elif tok.kind == "punct" and tok.text == "}":
        if depth == 0:
          raise self.fail("unexpected '}' while skipping a statement")
        depth -= 1
      elif tok.kind == "punct" and tok.text == ";" and depth == 0:
        return

  def skip_brace_block(self) -> None:
    """Consume a ``{ ... }`` block, including the braces."""
    self.expect_punct("{")
    depth = 1
    while depth:
      tok = self.peek()
      if tok.kind == "eof":
        raise self.fail("unterminated brace block")
      self.advance()
      if tok.kind == "punct" and tok.text == "{":
        depth += 1
      elif tok.kind == "punct" and tok.text == "}":
        depth -= 1

  def parse_type_name(self) -> str:
    leading = False
    if self.peek_is_punct("."):
      leading = True
      self.advance()
    parts = [self.expect_ident()]
    while self.peek_is_punct("."):
      self.advance()
      parts.append(self.expect_ident())
    name = ".".join(parts)
    if leading:
      return "." + name
    return name

  def parse_integer(self) -> int:
    sign = 1
    if self.peek_is_punct("-"):
      sign = -1
      self.advance()
    tok = self.peek()
    if tok.kind != "number":
      raise self.fail("expected integer")
    self.advance()
    if any(mark in tok.text for mark in (".", "e", "E")):
      raise self.fail("expected integer")
    return sign * _int_literal(tok.text)

  def parse_literal_text(self) -> str:
    """Return the source spelling of a scalar literal."""
    if self.peek_is_punct("-"):
      self.advance()
      tok = self.peek()
      if tok.kind != "number":
        raise self.fail("expected number after '-'")
      self.advance()
      return "-" + tok.text
    tok = self.peek()
    if tok.kind in ("ident", "number", "string"):
      self.advance()
      return tok.text
    raise self.fail("expected literal")


def _field(
    name: str,
    number: int,
    type_name: str,
    *,
    label: str = "",
    oneof: str = "",
    deprecated: bool = False,
    json_name: str = "",
    default: str = "",
    packed: bool | None = None,
    required: bool = False,
    id_kind: str = "tag",
    extra_attributes: list[str] | None = None,
) -> dict[str, Any]:
  return {
      "default": default,
      "deprecated": deprecated,
      "extra_attributes": sorted(extra_attributes or []),
      "id_kind": id_kind,
      "json_name": json_name,
      "label": label,
      "name": name,
      "number": number,
      "oneof": oneof,
      "packed": packed,
      "required": required,
      "type": type_name,
  }


def _empty_symbol(name: str, qualified: str, layout: str) -> dict[str, Any]:
  return {
      "deprecated": False,
      "extension_ranges": [],
      "fields": [],
      "layout": layout,
      "name": name,
      "qualified_name": qualified,
      "reserved_names": [],
      "reserved_ranges": [],
  }


def _sort_fields(fields: list[dict[str, Any]]) -> list[dict[str, Any]]:
  return sorted(fields, key=lambda item: (item["number"], item["name"]))


def _sort_ranges(ranges: list[dict[str, Any]]) -> list[dict[str, Any]]:
  def _key(item: dict[str, Any]) -> tuple[int, str]:
    end = item["end"]
    end_text = "max" if end == "max" else f"{int(end):010d}"
    return (int(item["start"]), end_text)

  return sorted(ranges, key=_key)


def _reject_duplicate_numbers(
    path: str, qualified: str, fields: list[dict[str, Any]]
) -> None:
  seen: dict[int, str] = {}
  for field in fields:
    previous = seen.get(field["number"])
    if previous is not None:
      raise ParseError(
          f"{path}: {qualified} reuses field number {field['number']} "
          f"({previous} and {field['name']})"
      )
    seen[field["number"]] = field["name"]


def _blank_schema(path: str, kind: str) -> dict[str, Any]:
  return {
      "enums": [],
      "extends": [],
      "file_extension": "",
      "file_identifier": "",
      "imports": [],
      "includes": [],
      "kind": kind,
      "messages": [],
      "namespace": "",
      "package": "",
      "path": path,
      "root_type": "",
      "services": [],
      "syntax": "",
      "unions": [],
  }


class ProtoParser:
  """Extract messages, enums, services, and extensions from one .proto."""

  def __init__(self, text: str, path: str):
    self.path = path
    self.cur = _Cursor(_tokenize(text), path)
    self.schema = _blank_schema(path, "protobuf")
    self.enum_count = 0
    self.extend_count = 0
    self.message_count = 0
    self.oneof_count = 0
    self.rpc_count = 0
    self.service_count = 0
    self.map_count = 0

  def parse(self) -> dict[str, Any]:
    cur = self.cur
    schema = self.schema
    while cur.peek().kind != "eof":
      if cur.peek_is_punct(";"):
        cur.advance()
        continue
      keyword = cur.expect_ident()
      if keyword == "syntax":
        cur.expect_punct("=")
        if cur.peek().kind != "string":
          raise cur.fail("expected syntax string")
        schema["syntax"] = cur.advance().text
        cur.expect_punct(";")
      elif keyword == "edition":
        cur.expect_punct("=")
        if cur.peek().kind != "string":
          raise cur.fail("expected edition string")
        schema["syntax"] = "edition " + cur.advance().text
        cur.expect_punct(";")
      elif keyword == "package":
        schema["package"] = cur.parse_type_name()
        cur.expect_punct(";")
      elif keyword == "import":
        if cur.peek_is_ident("public") or cur.peek_is_ident("weak"):
          cur.advance()
        if cur.peek().kind != "string":
          raise cur.fail("expected import path")
        schema["imports"].append(cur.advance().text)
        cur.expect_punct(";")
      elif keyword == "option":
        cur.skip_until_semi()
      elif keyword == "message":
        schema["messages"].append(self._parse_message(""))
      elif keyword == "enum":
        schema["enums"].append(self._parse_enum(""))
      elif keyword == "service":
        schema["services"].append(self._parse_service())
      elif keyword == "extend":
        schema["extends"].append(self._parse_extend())
      else:
        raise cur.fail(f"unexpected file-level keyword {keyword!r}")
    schema["imports"] = sorted(set(schema["imports"]))
    schema["messages"] = sorted(
        schema["messages"], key=lambda item: item["qualified_name"]
    )
    schema["enums"] = sorted(
        schema["enums"], key=lambda item: item["qualified_name"]
    )
    schema["services"] = sorted(
        schema["services"], key=lambda item: item["name"]
    )
    schema["extends"] = sorted(
        schema["extends"],
        key=lambda item: (
            item["extendee"],
            item["fields"][0]["number"] if item["fields"] else -1,
        ),
    )
    return schema

  def _qualify(self, prefix: str, name: str) -> str:
    if not prefix:
      return name
    return f"{prefix}.{name}"

  def _parse_message(self, prefix: str) -> dict[str, Any]:
    cur = self.cur
    name = cur.expect_ident()
    qualified = self._qualify(prefix, name)
    message = _empty_symbol(name, qualified, "message")
    self.message_count += 1
    cur.expect_punct("{")
    while not cur.peek_is_punct("}"):
      if cur.peek_is_punct(";"):
        cur.advance()
        continue
      if not cur.peek_is_ident():
        raise cur.fail("expected message member")
      keyword = cur.peek().text
      if keyword == "message":
        cur.advance()
        self.schema["messages"].append(self._parse_message(qualified))
        continue
      if keyword == "enum":
        cur.advance()
        self.schema["enums"].append(self._parse_enum(qualified))
        continue
      if keyword == "extend":
        cur.advance()
        self.schema["extends"].append(self._parse_extend())
        continue
      if keyword == "oneof":
        cur.advance()
        self._parse_oneof(message)
        continue
      if keyword == "option":
        cur.advance()
        if cur.peek_is_ident("deprecated"):
          message["deprecated"] = self._parse_bool_assignment()
        else:
          cur.skip_until_semi()
        continue
      if keyword == "reserved":
        cur.advance()
        names, ranges = self._parse_reserved()
        message["reserved_names"].extend(names)
        message["reserved_ranges"].extend(ranges)
        continue
      if keyword == "extensions":
        cur.advance()
        _, ranges = self._parse_reserved()
        message["extension_ranges"].extend(ranges)
        continue
      if keyword == "map":
        message["fields"].append(self._parse_map_field(""))
        continue
      if keyword in ("optional", "repeated", "required"):
        cur.advance()
        label = keyword
      else:
        label = "singular"
      message["fields"].append(self._parse_plain_field(label, ""))
    cur.expect_punct("}")
    message["fields"] = _sort_fields(message["fields"])
    message["reserved_names"] = sorted(set(message["reserved_names"]))
    message["reserved_ranges"] = _sort_ranges(message["reserved_ranges"])
    message["extension_ranges"] = _sort_ranges(message["extension_ranges"])
    _reject_duplicate_numbers(self.path, qualified, message["fields"])
    return message

  def _parse_oneof(self, message: dict[str, Any]) -> None:
    cur = self.cur
    oneof_name = cur.expect_ident()
    self.oneof_count += 1
    cur.expect_punct("{")
    while not cur.peek_is_punct("}"):
      if cur.peek_is_punct(";"):
        cur.advance()
        continue
      if cur.peek_is_ident("option"):
        cur.advance()
        cur.skip_until_semi()
        continue
      if cur.peek_is_ident("map"):
        message["fields"].append(self._parse_map_field(oneof_name))
        continue
      label = "singular"
      if cur.peek_is_ident("optional") or cur.peek_is_ident("repeated"):
        label = cur.advance().text
      message["fields"].append(self._parse_plain_field(label, oneof_name))
    cur.expect_punct("}")

  def _parse_bool_assignment(self) -> bool:
    """Parse ``deprecated = true;`` after the name has not yet been consumed."""
    self.cur.expect_ident("deprecated")
    self.cur.expect_punct("=")
    value = self.cur.expect_ident()
    if value not in ("false", "true"):
      raise self.cur.fail("expected true or false")
    self.cur.expect_punct(";")
    return value == "true"

  def _parse_reserved(self) -> tuple[list[str], list[dict[str, Any]]]:
    cur = self.cur
    names: list[str] = []
    ranges: list[dict[str, Any]] = []
    while not cur.peek_is_punct(";"):
      if cur.peek().kind == "string":
        names.append(cur.advance().text)
      else:
        start = cur.parse_integer()
        end: int | str = start
        if cur.peek_is_ident("to"):
          cur.advance()
          if cur.peek_is_ident("max"):
            cur.advance()
            end = "max"
          else:
            end = cur.parse_integer()
        ranges.append({"end": end, "start": start})
      if cur.peek_is_punct(","):
        cur.advance()
    cur.expect_punct(";")
    return names, ranges

  def _parse_map_field(self, oneof: str) -> dict[str, Any]:
    cur = self.cur
    cur.expect_ident("map")
    cur.expect_punct("<")
    key = cur.parse_type_name()
    cur.expect_punct(",")
    value = cur.parse_type_name()
    cur.expect_punct(">")
    name = cur.expect_ident()
    cur.expect_punct("=")
    number = cur.parse_integer()
    options = self._parse_field_options()
    cur.expect_punct(";")
    self.map_count += 1
    return _field(
        name,
        number,
        f"map<{key}, {value}>",
        label="map",
        oneof=oneof,
        deprecated=options["deprecated"],
        json_name=options["json_name"],
        default=options["default"],
        packed=options["packed"],
    )

  def _parse_plain_field(self, label: str, oneof: str) -> dict[str, Any]:
    cur = self.cur
    type_name = cur.parse_type_name()
    name = cur.expect_ident()
    cur.expect_punct("=")
    number = cur.parse_integer()
    options = self._parse_field_options()
    if cur.peek_is_punct("{"):
      raise cur.fail("proto2 groups are not supported by this inventory")
    cur.expect_punct(";")
    return _field(
        name,
        number,
        type_name,
        label=label,
        oneof=oneof,
        deprecated=options["deprecated"],
        json_name=options["json_name"],
        default=options["default"],
        packed=options["packed"],
    )

  def _parse_field_options(self) -> dict[str, Any]:
    found = {
        "default": "",
        "deprecated": False,
        "json_name": "",
        "packed": None,
    }
    cur = self.cur
    if not cur.peek_is_punct("["):
      return found
    cur.advance()
    while not cur.peek_is_punct("]"):
      if cur.peek_is_punct(","):
        cur.advance()
        continue
      if cur.peek_is_punct("("):
        self._skip_custom_option()
        continue
      name = cur.expect_ident()
      if cur.peek_is_punct("="):
        cur.advance()
        if name == "deprecated":
          found["deprecated"] = cur.expect_ident() == "true"
        elif name == "json_name":
          if cur.peek().kind != "string":
            raise cur.fail("expected json_name string")
          found["json_name"] = cur.advance().text
        elif name == "packed":
          found["packed"] = cur.expect_ident() == "true"
        elif name == "default":
          found["default"] = self._parse_option_value()
        else:
          self._parse_option_value()
      if cur.peek_is_punct(","):
        cur.advance()
    cur.expect_punct("]")
    return found

  def _skip_custom_option(self) -> None:
    cur = self.cur
    cur.expect_punct("(")
    depth = 1
    while depth:
      tok = cur.peek()
      if tok.kind == "eof":
        raise cur.fail("unterminated custom option")
      cur.advance()
      if tok.kind == "punct" and tok.text == "(":
        depth += 1
      elif tok.kind == "punct" and tok.text == ")":
        depth -= 1
    while cur.peek_is_punct("."):
      cur.advance()
      cur.expect_ident()
    if cur.peek_is_punct("="):
      cur.advance()
      self._parse_option_value()

  def _parse_option_value(self) -> str:
    cur = self.cur
    if cur.peek_is_punct("{"):
      cur.skip_brace_block()
      return ""
    if cur.peek_is_punct("["):
      cur.advance()
      depth = 1
      while depth:
        tok = cur.peek()
        if tok.kind == "eof":
          raise cur.fail("unterminated option list")
        cur.advance()
        if tok.kind == "punct" and tok.text == "[":
          depth += 1
        elif tok.kind == "punct" and tok.text == "]":
          depth -= 1
      return ""
    return cur.parse_literal_text()

  def _parse_enum(self, prefix: str) -> dict[str, Any]:
    cur = self.cur
    name = cur.expect_ident()
    qualified = self._qualify(prefix, name)
    enum = {
        "allow_alias": False,
        "name": name,
        "qualified_name": qualified,
        "reserved_names": [],
        "reserved_ranges": [],
        "underlying_type": "",
        "values": [],
    }
    self.enum_count += 1
    cur.expect_punct("{")
    while not cur.peek_is_punct("}"):
      if cur.peek_is_punct(";"):
        cur.advance()
        continue
      if cur.peek_is_ident("option"):
        cur.advance()
        if cur.peek_is_ident("allow_alias"):
          cur.expect_ident("allow_alias")
          cur.expect_punct("=")
          enum["allow_alias"] = cur.expect_ident() == "true"
          cur.expect_punct(";")
        elif cur.peek_is_ident("deprecated"):
          self._parse_bool_assignment()
        else:
          cur.skip_until_semi()
        continue
      if cur.peek_is_ident("reserved"):
        cur.advance()
        names, ranges = self._parse_reserved()
        enum["reserved_names"].extend(names)
        enum["reserved_ranges"].extend(ranges)
        continue
      value_name = cur.expect_ident()
      cur.expect_punct("=")
      number = cur.parse_integer()
      options = self._parse_field_options()
      if cur.peek_is_punct(";"):
        cur.advance()
      enum["values"].append(
          {
              "deprecated": options["deprecated"],
              "name": value_name,
              "number": number,
          }
      )
    cur.expect_punct("}")
    enum["values"] = sorted(
        enum["values"], key=lambda item: (item["number"], item["name"])
    )
    enum["reserved_names"] = sorted(set(enum["reserved_names"]))
    enum["reserved_ranges"] = _sort_ranges(enum["reserved_ranges"])
    if not enum["allow_alias"]:
      seen: dict[int, str] = {}
      for value in enum["values"]:
        previous = seen.get(value["number"])
        if previous is not None:
          raise ParseError(
              f"{self.path}: enum {qualified} reuses number "
              f"{value['number']} ({previous} and {value['name']})"
          )
        seen[value["number"]] = value["name"]
    return enum

  def _parse_service(self) -> dict[str, Any]:
    cur = self.cur
    name = cur.expect_ident()
    self.service_count += 1
    methods = []
    cur.expect_punct("{")
    while not cur.peek_is_punct("}"):
      if cur.peek_is_punct(";"):
        cur.advance()
        continue
      if cur.peek_is_ident("option"):
        cur.advance()
        cur.skip_until_semi()
        continue
      cur.expect_ident("rpc")
      methods.append(self._parse_rpc())
    cur.expect_punct("}")
    methods = sorted(methods, key=lambda item: item["name"])
    return {"methods": methods, "name": name}

  def _parse_rpc(self) -> dict[str, Any]:
    cur = self.cur
    name = cur.expect_ident()
    cur.expect_punct("(")
    client_streaming = False
    if cur.peek_is_ident("stream"):
      cur.advance()
      client_streaming = True
    request_type = cur.parse_type_name()
    cur.expect_punct(")")
    cur.expect_ident("returns")
    cur.expect_punct("(")
    server_streaming = False
    if cur.peek_is_ident("stream"):
      cur.advance()
      server_streaming = True
    response_type = cur.parse_type_name()
    cur.expect_punct(")")
    if cur.peek_is_punct("{"):
      cur.skip_brace_block()
    if cur.peek_is_punct(";"):
      cur.advance()
    self.rpc_count += 1
    return {
        "client_streaming": client_streaming,
        "name": name,
        "request_type": request_type,
        "response_type": response_type,
        "server_streaming": server_streaming,
    }

  def _parse_extend(self) -> dict[str, Any]:
    cur = self.cur
    extendee = cur.parse_type_name()
    fields = []
    self.extend_count += 1
    cur.expect_punct("{")
    while not cur.peek_is_punct("}"):
      if cur.peek_is_punct(";"):
        cur.advance()
        continue
      if cur.peek_is_ident("option"):
        cur.advance()
        cur.skip_until_semi()
        continue
      label = "singular"
      if cur.peek_is_ident() and cur.peek().text in (
          "optional",
          "repeated",
          "required",
      ):
        label = cur.advance().text
      fields.append(self._parse_plain_field(label, ""))
    cur.expect_punct("}")
    fields = _sort_fields(fields)
    _reject_duplicate_numbers(self.path, extendee, fields)
    return {"extendee": extendee, "fields": fields}


class FbsParser:
  """Extract tables, structs, enums, and unions from one .fbs file."""

  def __init__(self, text: str, path: str):
    self.path = path
    self.cur = _Cursor(_tokenize(text), path)
    self.schema = _blank_schema(path, "flatbuffer")
    self.enum_count = 0
    self.struct_count = 0
    self.table_count = 0
    self.union_count = 0

  def parse(self) -> dict[str, Any]:
    cur = self.cur
    schema = self.schema
    schema["syntax"] = "fbs"
    while cur.peek().kind != "eof":
      if cur.peek_is_punct(";"):
        cur.advance()
        continue
      keyword = cur.expect_ident()
      if keyword == "include":
        if cur.peek().kind != "string":
          raise cur.fail("expected include path")
        schema["includes"].append(cur.advance().text)
        cur.expect_punct(";")
      elif keyword == "namespace":
        schema["namespace"] = cur.parse_type_name()
        cur.expect_punct(";")
      elif keyword == "attribute":
        if cur.peek().kind != "string":
          raise cur.fail("expected attribute name")
        cur.advance()
        cur.expect_punct(";")
      elif keyword == "root_type":
        schema["root_type"] = cur.parse_type_name()
        cur.expect_punct(";")
      elif keyword == "file_identifier":
        if cur.peek().kind != "string":
          raise cur.fail("expected file_identifier")
        schema["file_identifier"] = cur.advance().text
        cur.expect_punct(";")
      elif keyword == "file_extension":
        if cur.peek().kind != "string":
          raise cur.fail("expected file_extension")
        schema["file_extension"] = cur.advance().text
        cur.expect_punct(";")
      elif keyword == "table":
        schema["messages"].append(self._parse_object("table"))
      elif keyword == "struct":
        schema["messages"].append(self._parse_object("struct"))
      elif keyword == "enum":
        schema["enums"].append(self._parse_enum())
      elif keyword == "union":
        schema["unions"].append(self._parse_union())
      elif keyword == "rpc_service":
        raise cur.fail("rpc_service is not part of the HAL inventory parser")
      else:
        raise cur.fail(f"unexpected FlatBuffer keyword {keyword!r}")
    schema["includes"] = sorted(set(schema["includes"]))
    schema["messages"] = sorted(
        schema["messages"], key=lambda item: item["qualified_name"]
    )
    schema["enums"] = sorted(
        schema["enums"], key=lambda item: item["qualified_name"]
    )
    schema["unions"] = sorted(schema["unions"], key=lambda item: item["name"])
    return schema

  def _parse_fbs_type(self) -> str:
    cur = self.cur
    if cur.peek_is_punct("["):
      cur.advance()
      inner = self._parse_fbs_type()
      length = ""
      if cur.peek_is_punct(":"):
        cur.advance()
        length = str(cur.parse_integer())
      cur.expect_punct("]")
      if length:
        return f"[{inner}:{length}]"
      return f"[{inner}]"
    return cur.parse_type_name()

  def _parse_object(self, layout: str) -> dict[str, Any]:
    cur = self.cur
    name = cur.expect_ident()
    symbol = _empty_symbol(name, name, layout)
    if layout == "table":
      self.table_count += 1
      id_kind = "vtable"
    else:
      self.struct_count += 1
      id_kind = "positional"
    cur.expect_punct("{")
    index = 0
    while not cur.peek_is_punct("}"):
      if cur.peek_is_punct(";"):
        cur.advance()
        continue
      field_name = cur.expect_ident()
      cur.expect_punct(":")
      type_name = self._parse_fbs_type()
      default = ""
      if cur.peek_is_punct("="):
        cur.advance()
        default = cur.parse_literal_text()
      attributes = self._parse_fbs_attributes()
      if cur.peek_is_punct(";"):
        cur.advance()
      number = index
      if attributes["id"] is not None:
        number = attributes["id"]
      symbol["fields"].append(
          _field(
              field_name,
              number,
              type_name,
              default=default,
              deprecated=attributes["deprecated"],
              required=attributes["required"],
              id_kind=id_kind,
              extra_attributes=attributes["extra"],
          )
      )
      index += 1
    cur.expect_punct("}")
    symbol["fields"] = _sort_fields(symbol["fields"])
    _reject_duplicate_numbers(self.path, name, symbol["fields"])
    return symbol

  def _parse_fbs_attributes(self) -> dict[str, Any]:
    found: dict[str, Any] = {
        "deprecated": False,
        "extra": [],
        "id": None,
        "required": False,
    }
    cur = self.cur
    if not cur.peek_is_punct("("):
      return found
    cur.advance()
    while not cur.peek_is_punct(")"):
      if cur.peek_is_punct(","):
        cur.advance()
        continue
      name = cur.expect_ident()
      if cur.peek_is_punct(":"):
        cur.advance()
        value = cur.parse_literal_text()
        if name == "id":
          found["id"] = int(value, 0)
        else:
          found["extra"].append(f"{name}:{value}")
      elif name == "deprecated":
        found["deprecated"] = True
      elif name == "required":
        found["required"] = True
      else:
        found["extra"].append(name)
    cur.expect_punct(")")
    return found

  def _parse_enum(self) -> dict[str, Any]:
    cur = self.cur
    name = cur.expect_ident()
    cur.expect_punct(":")
    underlying = self._parse_fbs_type()
    enum = {
        "allow_alias": False,
        "name": name,
        "qualified_name": name,
        "reserved_names": [],
        "reserved_ranges": [],
        "underlying_type": underlying,
        "values": [],
    }
    self.enum_count += 1
    cur.expect_punct("{")
    # FlatBuffers assigns omitted values sequentially, starting at 0.
    next_number = 0
    while not cur.peek_is_punct("}"):
      if cur.peek_is_punct(",") or cur.peek_is_punct(";"):
        cur.advance()
        continue
      value_name = cur.expect_ident()
      if cur.peek_is_punct("="):
        cur.advance()
        number = cur.parse_integer()
      else:
        number = next_number
      next_number = number + 1
      deprecated = False
      if cur.peek_is_punct("("):
        attributes = self._parse_fbs_attributes()
        deprecated = attributes["deprecated"]
      if cur.peek_is_punct(",") or cur.peek_is_punct(";"):
        cur.advance()
      enum["values"].append(
          {
              "deprecated": deprecated,
              "name": value_name,
              "number": number,
          }
      )
    cur.expect_punct("}")
    enum["values"] = sorted(
        enum["values"], key=lambda item: (item["number"], item["name"])
    )
    seen: dict[int, str] = {}
    for value in enum["values"]:
      previous = seen.get(value["number"])
      if previous is not None and previous != value["name"]:
        raise ParseError(
            f"{self.path}: enum {name} reuses number {value['number']} "
            f"({previous} and {value['name']})"
        )
      seen[value["number"]] = value["name"]
    return enum

  def _parse_union(self) -> dict[str, Any]:
    cur = self.cur
    name = cur.expect_ident()
    self.union_count += 1
    members = []
    cur.expect_punct("{")
    index = 1
    while not cur.peek_is_punct("}"):
      if cur.peek_is_punct(",") or cur.peek_is_punct(";"):
        cur.advance()
        continue
      member = cur.parse_type_name()
      number = index
      if cur.peek_is_punct("="):
        cur.advance()
        number = cur.parse_integer()
      if cur.peek_is_punct(",") or cur.peek_is_punct(";"):
        cur.advance()
      members.append({"name": member, "number": number})
      index += 1
    cur.expect_punct("}")
    members = sorted(members, key=lambda item: (item["number"], item["name"]))
    return {"members": members, "name": name}


def _check_proto_counts(text: str, parser: ProtoParser) -> None:
  stripped = _strip_for_counts(text)
  expected = {
      "enum": parser.enum_count,
      "extend": parser.extend_count,
      "message": parser.message_count,
      "oneof": parser.oneof_count,
      "rpc": _count_decl(stripped, "rpc"),
      "service": parser.service_count,
  }
  for keyword in ("enum", "extend", "message", "oneof", "service"):
    expected[keyword] = _count_decl(stripped, keyword)
  actual = {
      "enum": parser.enum_count,
      "extend": parser.extend_count,
      "message": parser.message_count,
      "oneof": parser.oneof_count,
      "rpc": parser.rpc_count,
      "service": parser.service_count,
  }
  map_actual = parser.map_count
  map_expected = len(re.findall(r"\bmap\s*<", stripped))
  if map_actual != map_expected:
    raise ParseError(
        f"{parser.path}: map count {map_actual} != source count {map_expected}"
    )
  for keyword, count in expected.items():
    if actual[keyword] != count:
      raise ParseError(
          f"{parser.path}: {keyword} count {actual[keyword]} != "
          f"source count {count}"
      )


def _check_fbs_counts(text: str, parser: FbsParser) -> None:
  stripped = _strip_for_counts(text)
  pairs = (
      ("enum", parser.enum_count),
      ("struct", parser.struct_count),
      ("table", parser.table_count),
      ("union", parser.union_count),
  )
  for keyword, actual in pairs:
    expected = _count_decl(stripped, keyword)
    if actual != expected:
      raise ParseError(
          f"{parser.path}: {keyword} count {actual} != source count {expected}"
      )


def parse_proto(text: str, path: str) -> dict[str, Any]:
  parser = ProtoParser(text, path)
  schema = parser.parse()
  _check_proto_counts(text, parser)
  return schema


def parse_fbs(text: str, path: str) -> dict[str, Any]:
  parser = FbsParser(text, path)
  schema = parser.parse()
  _check_fbs_counts(text, parser)
  return schema


def _is_excluded(path: Path) -> bool:
  if any(part in EXCLUDED_DIR_NAMES for part in path.parts):
    return True
  name = path.name
  return name.endswith("_test.proto") or name.endswith("_test.fbs")


def discover_schema_files(repo_root: Path) -> list[Path]:
  """Return allowlisted schema paths in deterministic order."""
  found: list[Path] = []
  for prefix in DIRECTORY_ALLOWLIST:
    directory = repo_root / prefix
    if not directory.is_dir():
      raise ParseError(f"allowlisted directory is missing: {prefix}")
    for path in directory.rglob("*"):
      if path.suffix not in (".fbs", ".proto"):
        continue
      if path.is_file() and not _is_excluded(path):
        found.append(path)
  for relative in FILE_ALLOWLIST:
    path = repo_root / relative
    if not path.is_file():
      raise ParseError(f"allowlisted file is missing: {relative}")
    if not _is_excluded(path):
      found.append(path)
  unique = sorted({path.resolve() for path in found})
  return unique


def _repo_relative(repo_root: Path, path: Path) -> str:
  return path.resolve().relative_to(repo_root.resolve()).as_posix()


def _import_is_in_scope(import_path: str, schema_paths: set[str]) -> bool:
  normalized = import_path.replace("\\", "/")
  if normalized in schema_paths:
    return True
  suffix = "/" + normalized
  return any(path.endswith(suffix) for path in schema_paths)


def build_inventory(repo_root: Path) -> dict[str, Any]:
  """Parse the allowlisted schemas into a deterministic inventory document."""
  schemas = []
  for path in discover_schema_files(repo_root):
    relative = _repo_relative(repo_root, path)
    text = path.read_text(encoding="utf-8")
    if path.suffix == ".proto":
      schemas.append(parse_proto(text, relative))
    else:
      schemas.append(parse_fbs(text, relative))
  schemas.sort(key=lambda item: item["path"])
  schema_paths = {item["path"] for item in schemas}
  outside: set[str] = set()
  for schema in schemas:
    for import_path in schema["imports"] + schema["includes"]:
      if not _import_is_in_scope(import_path, schema_paths):
        outside.add(import_path.replace("\\", "/"))
  inventory = {
      "inventory_version": INVENTORY_VERSION,
      "out_of_scope_imports": sorted(outside),
      "schemas": schemas,
      "selection_rule": SELECTION_RULE,
      "selection_rule_id": SELECTION_RULE_ID,
      "summary": _summary(schemas),
  }
  _verify_anchors(inventory)
  return inventory


def _summary(schemas: list[dict[str, Any]]) -> dict[str, int]:
  field_count = 0
  enum_value_count = 0
  rpc_count = 0
  for schema in schemas:
    for message in schema["messages"]:
      field_count += len(message["fields"])
    for extend in schema["extends"]:
      field_count += len(extend["fields"])
    for enum in schema["enums"]:
      enum_value_count += len(enum["values"])
    for service in schema["services"]:
      rpc_count += len(service["methods"])
  return {
      "enum_count": sum(len(item["enums"]) for item in schemas),
      "enum_value_count": enum_value_count,
      "extend_count": sum(len(item["extends"]) for item in schemas),
      "field_count": field_count,
      "flatbuffer_count": sum(
          1 for item in schemas if item["kind"] == "flatbuffer"
      ),
      "message_count": sum(
          1
          for item in schemas
          for message in item["messages"]
          if message["layout"] == "message"
      ),
      "protobuf_count": sum(
          1 for item in schemas if item["kind"] == "protobuf"
      ),
      "rpc_count": rpc_count,
      "schema_count": len(schemas),
      "service_count": sum(len(item["services"]) for item in schemas),
      "struct_count": sum(
          1
          for item in schemas
          for message in item["messages"]
          if message["layout"] == "struct"
      ),
      "table_count": sum(
          1
          for item in schemas
          for message in item["messages"]
          if message["layout"] == "table"
      ),
      "union_count": sum(len(item["unions"]) for item in schemas),
  }


def _find_message(
    inventory: dict[str, Any], path: str, qualified: str
) -> dict[str, Any]:
  for schema in inventory["schemas"]:
    if schema["path"] != path:
      continue
    for message in schema["messages"]:
      if message["qualified_name"] == qualified:
        return message
  raise ParseError(f"anchor message {qualified} missing from {path}")


def _find_field(message: dict[str, Any], name: str) -> dict[str, Any]:
  for field in message["fields"]:
    if field["name"] == name:
      return field
  raise ParseError(
      f"anchor field {name} missing from {message['qualified_name']}"
  )


def _verify_anchors(inventory: dict[str, Any]) -> None:
  """Fail if known public symbols were dropped or renumbered by the parser."""
  joint_limits = _find_message(
      inventory,
      "intrinsic_control/intrinsic/icon/hal/interfaces/joint_limits.fbs",
      "JointLimits",
  )
  min_position = _find_field(joint_limits, "min_position")
  if min_position["number"] != 0 or min_position["id_kind"] != "vtable":
    raise ParseError("JointLimits.min_position anchor failed")
  if joint_limits["layout"] != "table":
    raise ParseError("JointLimits layout anchor failed")

  state = _find_message(
      inventory,
      "intrinsic_control/intrinsic/icon/hal/interfaces/"
      "hardware_module_state.fbs",
      "HardwareModuleState",
  )
  fault = _find_field(state, "message")
  if fault["type"] != "[uint8:256]" or fault["id_kind"] != "positional":
    raise ParseError("HardwareModuleState.message anchor failed")

  move = _find_message(
      inventory,
      "intrinsic_apis/intrinsic/icon/actions/point_to_point_move.proto",
      "PointToPointMoveFixedParams",
  )
  goal = _find_field(move, "goal_position")
  if goal["number"] != 1 or goal["label"] != "singular":
    raise ParseError("PointToPointMoveFixedParams.goal_position anchor failed")

  limits = _find_field(move, "joint_limits")
  if limits["number"] != 3 or limits["label"] != "optional":
    raise ParseError("PointToPointMoveFixedParams.joint_limits anchor failed")

  for schema in inventory["schemas"]:
    if schema["path"].endswith("skill_parameter_metadata.proto"):
      fields = schema["extends"][0]["fields"]
      if (
          fields[0]["name"] != "skill_parameter_metadata"
          or fields[0]["number"] != 91335
      ):
        raise ParseError("skill_parameter_metadata extension anchor failed")
      break
  else:
    raise ParseError("skill_parameter_metadata.proto missing from inventory")

  for schema in inventory["schemas"]:
    if not schema["path"].endswith("service.proto"):
      continue
    if schema["path"] != "intrinsic_apis/intrinsic/icon/proto/v1/service.proto":
      continue
    methods = {item["name"]: item for item in schema["services"][0]["methods"]}
    opened = methods["OpenSession"]
    if not opened["client_streaming"] or not opened["server_streaming"]:
      raise ParseError("OpenSession streaming anchor failed")
    break
  else:
    raise ParseError("ICON service.proto missing from inventory")


def dump_inventory(inventory: dict[str, Any]) -> str:
  """Serialize an inventory with stable key order and a trailing newline."""
  return (
      json.dumps(inventory, ensure_ascii=True, indent=2, sort_keys=True) + "\n"
  )


def _index_schemas(
    inventory: dict[str, Any],
) -> dict[str, dict[str, Any]]:
  return {schema["path"]: schema for schema in inventory["schemas"]}


def _range_key(item: dict[str, Any]) -> str:
  return f"{item['start']}:{item['end']}"


def compare_inventories(
    baseline: dict[str, Any], candidate: dict[str, Any], strict: bool
) -> list[str]:
  """Return sorted diagnostics.

  Lines for wire breaks start with ``incompatible:``.
  """
  notes: list[str] = []
  if baseline.get("selection_rule_id") != candidate.get("selection_rule_id"):
    notes.append(
        "incompatible: selection_rule_id "
        f"{baseline.get('selection_rule_id')} -> "
        f"{candidate.get('selection_rule_id')}"
    )
  old_schemas = _index_schemas(baseline)
  new_schemas = _index_schemas(candidate)
  for path in sorted(set(old_schemas) - set(new_schemas)):
    notes.append(f"incompatible: removed schema {path}")
  for path in sorted(set(new_schemas) - set(old_schemas)):
    notes.append(f"additive: added schema {path}")
  for path in sorted(set(old_schemas) & set(new_schemas)):
    notes.extend(_compare_schema(path, old_schemas[path], new_schemas[path]))
  if strict:
    promoted = []
    for note in notes:
      if note.startswith("additive: "):
        promoted.append("incompatible: strict " + note)
      else:
        promoted.append(note)
    notes = promoted
  return sorted(notes)


def _compare_schema(
    path: str, old: dict[str, Any], new: dict[str, Any]
) -> list[str]:
  notes: list[str] = []
  if old.get("package") != new.get("package"):
    notes.append(
        f"incompatible: {path} package "
        f"{old.get('package')} -> {new.get('package')}"
    )
  if old.get("namespace") != new.get("namespace"):
    notes.append(
        f"incompatible: {path} namespace "
        f"{old.get('namespace')} -> {new.get('namespace')}"
    )
  notes.extend(
      _compare_named_group(
          path, "message", old["messages"], new["messages"], "qualified_name"
      )
  )
  notes.extend(_compare_enums(path, old["enums"], new["enums"]))
  notes.extend(_compare_services(path, old["services"], new["services"]))
  notes.extend(_compare_extends(path, old["extends"], new["extends"]))
  notes.extend(_compare_unions(path, old["unions"], new["unions"]))
  return notes


def _by_key(items: list[dict[str, Any]], key: str) -> dict[str, dict[str, Any]]:
  return {item[key]: item for item in items}


def _compare_named_group(
    path: str,
    kind: str,
    old_items: list[dict[str, Any]],
    new_items: list[dict[str, Any]],
    key: str,
) -> list[str]:
  notes: list[str] = []
  old_map = _by_key(old_items, key)
  new_map = _by_key(new_items, key)
  for name in sorted(set(old_map) - set(new_map)):
    notes.append(f"incompatible: {path} removed {kind} {name}")
  for name in sorted(set(new_map) - set(old_map)):
    notes.append(f"additive: {path} added {kind} {name}")
  for name in sorted(set(old_map) & set(new_map)):
    old_item = old_map[name]
    new_item = new_map[name]
    if old_item.get("layout") != new_item.get("layout"):
      notes.append(
          f"incompatible: {path} {kind} {name} layout "
          f"{old_item.get('layout')} -> {new_item.get('layout')}"
      )
    notes.extend(
        _compare_fields(
            path, f"{kind} {name}", old_item["fields"], new_item["fields"]
        )
    )
    notes.extend(_compare_reserved(path, f"{kind} {name}", old_item, new_item))
  return notes


def _compare_fields(
    path: str,
    owner: str,
    old_fields: list[dict[str, Any]],
    new_fields: list[dict[str, Any]],
) -> list[str]:
  notes: list[str] = []
  old_by_name = _by_key(old_fields, "name")
  new_by_name = _by_key(new_fields, "name")
  old_by_number = {field["number"]: field["name"] for field in old_fields}
  new_by_number = {field["number"]: field["name"] for field in new_fields}
  for name in sorted(set(old_by_name) - set(new_by_name)):
    number = old_by_name[name]["number"]
    notes.append(
        f"incompatible: {path} {owner} removed field {name} number {number}"
    )
  for name in sorted(set(new_by_name) - set(old_by_name)):
    number = new_by_name[name]["number"]
    previous = old_by_number.get(number)
    if previous is not None and previous != name:
      notes.append(
          f"incompatible: {path} {owner} field number {number} reused "
          f"by {name} (was {previous})"
      )
    else:
      notes.append(
          f"additive: {path} {owner} added field {name} number {number}"
      )
  for name in sorted(set(old_by_name) & set(new_by_name)):
    old_field = old_by_name[name]
    new_field = new_by_name[name]
    if old_field["number"] != new_field["number"]:
      notes.append(
          f"incompatible: {path} {owner} field {name} number "
          f"{old_field['number']} -> {new_field['number']}"
      )
    if old_field["type"] != new_field["type"]:
      notes.append(
          f"incompatible: {path} {owner} field {name} type "
          f"{old_field['type']} -> {new_field['type']}"
      )
    if old_field["label"] != new_field["label"]:
      notes.append(
          f"incompatible: {path} {owner} field {name} label "
          f"{old_field['label']} -> {new_field['label']}"
      )
    if old_field["id_kind"] != new_field["id_kind"]:
      notes.append(
          f"incompatible: {path} {owner} field {name} id_kind "
          f"{old_field['id_kind']} -> {new_field['id_kind']}"
      )
    if old_field["oneof"] != new_field["oneof"]:
      notes.append(
          f"incompatible: {path} {owner} field {name} oneof "
          f"{old_field['oneof']!r} -> {new_field['oneof']!r}"
      )
    if old_field["required"] != new_field["required"]:
      notes.append(
          f"incompatible: {path} {owner} field {name} required "
          f"{old_field['required']} -> {new_field['required']}"
      )
    if old_field["default"] != new_field["default"]:
      notes.append(
          f"incompatible: {path} {owner} field {name} default "
          f"{old_field['default']!r} -> {new_field['default']!r}"
      )
  for number, name in sorted(new_by_number.items()):
    previous = old_by_number.get(number)
    if (
        previous is not None
        and previous != name
        and previous not in new_by_name
        and name in old_by_name
    ):
      notes.append(
          f"incompatible: {path} {owner} field number {number} reused "
          f"by {name} (was {previous})"
      )
  return notes


def _compare_reserved(
    path: str, owner: str, old_item: dict[str, Any], new_item: dict[str, Any]
) -> list[str]:
  notes: list[str] = []
  old_names = set(old_item.get("reserved_names", []))
  new_names = set(new_item.get("reserved_names", []))
  for name in sorted(old_names - new_names):
    notes.append(f"incompatible: {path} {owner} dropped reserved name {name}")
  for name in sorted(new_names - old_names):
    notes.append(f"additive: {path} {owner} reserved name {name}")
  old_ranges = {
      _range_key(item) for item in old_item.get("reserved_ranges", [])
  }
  new_ranges = {
      _range_key(item) for item in new_item.get("reserved_ranges", [])
  }
  for key in sorted(old_ranges - new_ranges):
    notes.append(f"incompatible: {path} {owner} dropped reserved range {key}")
  for key in sorted(new_ranges - old_ranges):
    notes.append(f"additive: {path} {owner} reserved range {key}")
  return notes


def _compare_enums(
    path: str, old_enums: list[dict[str, Any]], new_enums: list[dict[str, Any]]
) -> list[str]:
  notes: list[str] = []
  old_map = _by_key(old_enums, "qualified_name")
  new_map = _by_key(new_enums, "qualified_name")
  for name in sorted(set(old_map) - set(new_map)):
    notes.append(f"incompatible: {path} removed enum {name}")
  for name in sorted(set(new_map) - set(old_map)):
    notes.append(f"additive: {path} added enum {name}")
  for name in sorted(set(old_map) & set(new_map)):
    old_enum = old_map[name]
    new_enum = new_map[name]
    if old_enum["underlying_type"] != new_enum["underlying_type"]:
      notes.append(
          f"incompatible: {path} enum {name} underlying_type "
          f"{old_enum['underlying_type']} -> {new_enum['underlying_type']}"
      )
    old_values = _by_key(old_enum["values"], "name")
    new_values = _by_key(new_enum["values"], "name")
    for value_name in sorted(set(old_values) - set(new_values)):
      number = old_values[value_name]["number"]
      notes.append(
          f"incompatible: {path} enum {name} removed value "
          f"{value_name} number {number}"
      )
    for value_name in sorted(set(new_values) - set(old_values)):
      number = new_values[value_name]["number"]
      notes.append(
          f"additive: {path} enum {name} added value "
          f"{value_name} number {number}"
      )
    for value_name in sorted(set(old_values) & set(new_values)):
      old_number = old_values[value_name]["number"]
      new_number = new_values[value_name]["number"]
      if old_number != new_number:
        notes.append(
            f"incompatible: {path} enum {name} value {value_name} "
            f"number {old_number} -> {new_number}"
        )
    notes.extend(_compare_reserved(path, f"enum {name}", old_enum, new_enum))
  return notes


def _compare_services(
    path: str,
    old_services: list[dict[str, Any]],
    new_services: list[dict[str, Any]],
) -> list[str]:
  notes: list[str] = []
  old_map = _by_key(old_services, "name")
  new_map = _by_key(new_services, "name")
  for name in sorted(set(old_map) - set(new_map)):
    notes.append(f"incompatible: {path} removed service {name}")
  for name in sorted(set(new_map) - set(old_map)):
    notes.append(f"additive: {path} added service {name}")
  for name in sorted(set(old_map) & set(new_map)):
    old_methods = _by_key(old_map[name]["methods"], "name")
    new_methods = _by_key(new_map[name]["methods"], "name")
    for method_name in sorted(set(old_methods) - set(new_methods)):
      notes.append(
          f"incompatible: {path} service {name} removed rpc {method_name}"
      )
    for method_name in sorted(set(new_methods) - set(old_methods)):
      notes.append(f"additive: {path} service {name} added rpc {method_name}")
    for method_name in sorted(set(old_methods) & set(new_methods)):
      old_method = old_methods[method_name]
      new_method = new_methods[method_name]
      for slot in (
          "request_type",
          "response_type",
          "client_streaming",
          "server_streaming",
      ):
        if old_method[slot] != new_method[slot]:
          notes.append(
              f"incompatible: {path} service {name} rpc {method_name} "
              f"{slot} {old_method[slot]} -> {new_method[slot]}"
          )
  return notes


def _compare_extends(
    path: str, old_items: list[dict[str, Any]], new_items: list[dict[str, Any]]
) -> list[str]:
  def _key(item: dict[str, Any]) -> str:
    numbers = ",".join(str(field["number"]) for field in item["fields"])
    return f"{item['extendee']}#{numbers}"

  old_map = {_key(item): item for item in old_items}
  new_map = {_key(item): item for item in new_items}
  notes: list[str] = []
  # Compare by extendee first so a renumbered extension is visible.
  old_by_extendee: dict[str, list[dict[str, Any]]] = {}
  new_by_extendee: dict[str, list[dict[str, Any]]] = {}
  for item in old_items:
    old_by_extendee.setdefault(item["extendee"], []).append(item)
  for item in new_items:
    new_by_extendee.setdefault(item["extendee"], []).append(item)
  for extendee in sorted(set(old_by_extendee) - set(new_by_extendee)):
    notes.append(f"incompatible: {path} removed extend {extendee}")
  for extendee in sorted(set(new_by_extendee) - set(old_by_extendee)):
    notes.append(f"additive: {path} added extend {extendee}")
  for extendee in sorted(set(old_by_extendee) & set(new_by_extendee)):
    old_fields = [
        field for item in old_by_extendee[extendee] for field in item["fields"]
    ]
    new_fields = [
        field for item in new_by_extendee[extendee] for field in item["fields"]
    ]
    notes.extend(
        _compare_fields(path, f"extend {extendee}", old_fields, new_fields)
    )
  del old_map, new_map
  return notes


def _compare_unions(
    path: str,
    old_unions: list[dict[str, Any]],
    new_unions: list[dict[str, Any]],
) -> list[str]:
  notes: list[str] = []
  old_map = _by_key(old_unions, "name")
  new_map = _by_key(new_unions, "name")
  for name in sorted(set(old_map) - set(new_map)):
    notes.append(f"incompatible: {path} removed union {name}")
  for name in sorted(set(new_map) - set(old_map)):
    notes.append(f"additive: {path} added union {name}")
  for name in sorted(set(old_map) & set(new_map)):
    old_members = _by_key(old_map[name]["members"], "name")
    new_members = _by_key(new_map[name]["members"], "name")
    for member in sorted(set(old_members) - set(new_members)):
      notes.append(f"incompatible: {path} union {name} removed member {member}")
    for member in sorted(set(new_members) - set(old_members)):
      notes.append(f"additive: {path} union {name} added member {member}")
    for member in sorted(set(old_members) & set(new_members)):
      old_number = old_members[member]["number"]
      new_number = new_members[member]["number"]
      if old_number != new_number:
        notes.append(
            f"incompatible: {path} union {name} member {member} "
            f"number {old_number} -> {new_number}"
        )
  return notes


def _load_inventory(path: Path) -> dict[str, Any]:
  return json.loads(path.read_text(encoding="utf-8"))


def _has_incompatible(notes: list[str]) -> bool:
  return any(note.startswith("incompatible:") for note in notes)


_SELF_CHECK_PROTO = """
syntax = "proto3";
package demo.v1;
import "google/protobuf/empty.proto";
import public "demo/other.proto";

message Outer {
  enum Status {
    UNKNOWN = 0;
    READY = 1 [deprecated = true];
    reserved 3;
    reserved "GONE";
  }
  message Inner {
    string name = 1;
  }
  oneof choice {
    Inner inner = 2;
    int32 raw = 3;
  }
  map<string, Inner> values = 4;
  optional double margin = 5 [json_name = "edge", deprecated = true];
  reserved 8 to 9;
  reserved "old_field";
  option deprecated = true;
}

service Demo {
  option (demo.http) = {
    get: "/v1/{name}"
  };
  rpc Get(GetRequest) returns (google.protobuf.Empty);
  rpc Watch(stream WatchRequest) returns (stream WatchResponse);
}

extend google.protobuf.FieldOptions {
  optional string unit = 50001 [deprecated = true];
}
"""

_SELF_CHECK_FBS = """
namespace demo;
include "other.fbs";
enum Mode : uint8 {
  OFF = 0,
  ON = 0x10,
  FAULT = -1
}
enum Implicit : int {
  A,
  B = 4,
  C
}
table Sample {
  value:double = 0.0;
  flag:bool = false;
  mode:Mode = OFF;
}
struct Fixed {
  message:[uint8:256];
  code:Mode = OFF;
}
"""


def _self_check() -> None:
  first = parse_proto(_SELF_CHECK_PROTO, "self_check.proto")
  second = parse_proto(_SELF_CHECK_PROTO, "self_check.proto")
  if dump_inventory({"schemas": [first]}) != dump_inventory(
      {"schemas": [second]}
  ):
    raise ParseError("proto self-check is not deterministic")
  outer = next(item for item in first["messages"] if item["name"] == "Outer")
  if outer["deprecated"] is not True:
    raise ParseError("self-check missed message deprecated option")
  margin = next(item for item in outer["fields"] if item["name"] == "margin")
  if margin["number"] != 5 or margin["json_name"] != "edge":
    raise ParseError("self-check missed field options")
  if not any(
      item["start"] == 8 and item["end"] == 9
      for item in outer["reserved_ranges"]
  ):
    raise ParseError("self-check missed reserved range")
  service = first["services"][0]
  methods = {item["name"]: item for item in service["methods"]}
  if (
      not methods["Watch"]["client_streaming"]
      or not methods["Watch"]["server_streaming"]
  ):
    raise ParseError("self-check missed streaming rpc")
  extension = first["extends"][0]["fields"][0]
  if extension["number"] != 50001:
    raise ParseError("self-check missed extension tag")

  fbs_one = parse_fbs(_SELF_CHECK_FBS, "self_check.fbs")
  fbs_two = parse_fbs(_SELF_CHECK_FBS, "self_check.fbs")
  if dump_inventory({"schemas": [fbs_one]}) != dump_inventory(
      {"schemas": [fbs_two]}
  ):
    raise ParseError("flatbuffer self-check is not deterministic")
  mode = next(item for item in fbs_one["enums"] if item["name"] == "Mode")
  values = {item["name"]: item["number"] for item in mode["values"]}
  if values["ON"] != 16 or values["FAULT"] != -1:
    raise ParseError("self-check missed enum numbers")
  implicit = next(
      item for item in fbs_one["enums"] if item["name"] == "Implicit"
  )
  implicit_values = {
      item["name"]: item["number"] for item in implicit["values"]
  }
  if implicit_values != {"A": 0, "B": 4, "C": 5}:
    raise ParseError("self-check missed implicit enum values")
  fixed = next(item for item in fbs_one["messages"] if item["name"] == "Fixed")
  fault_text = next(
      item for item in fixed["fields"] if item["name"] == "message"
  )
  if fault_text["type"] != "[uint8:256]" or fault_text["number"] != 0:
    raise ParseError("self-check missed fixed array field")

  baseline = {"schemas": [first], "selection_rule_id": SELECTION_RULE_ID}
  same = {
      "schemas": [json.loads(json.dumps(first))],
      "selection_rule_id": SELECTION_RULE_ID,
  }
  if compare_inventories(baseline, same, strict=False):
    raise ParseError("identical inventories did not compare clean")
  mutated = json.loads(json.dumps(baseline))
  mutated["schemas"][0]["messages"][0]["fields"][0]["number"] = 99
  notes = compare_inventories(baseline, mutated, strict=False)
  if not _has_incompatible(notes):
    raise ParseError("renumbered field was not reported as incompatible")
  added = json.loads(json.dumps(baseline))
  added["schemas"][0]["messages"][0]["fields"].append(
      _field("extra", 70, "bool", label="singular")
  )
  added_notes = compare_inventories(baseline, added, strict=False)
  if _has_incompatible(added_notes):
    raise ParseError("additive field was reported as incompatible")
  if not any(note.startswith("additive:") for note in added_notes):
    raise ParseError("additive field was not reported")
  strict_notes = compare_inventories(baseline, added, strict=True)
  if not _has_incompatible(strict_notes):
    raise ParseError("strict compare did not reject an additive field")


def _default_repo_root() -> Path:
  here = Path(__file__).resolve()
  for parent in here.parents:
    if (parent / "MODULE.bazel").is_file() and (
        parent / "intrinsic_apis"
    ).is_dir():
      return parent
  raise ParseError("could not locate the repository root; pass --repo-root")


def _default_output() -> Path:
  return (
      Path(__file__).resolve().parent
      / "manipulator_public_schema_inventory.json"
  )


def main(argv: list[str] | None = None) -> int:
  parser = argparse.ArgumentParser(description=__doc__)
  parser.add_argument(
      "--repo-root",
      type=Path,
      default=None,
      help="Repository root. Defaults to the tree that contains this script.",
  )
  parser.add_argument(
      "--output",
      type=Path,
      default=None,
      help="Inventory JSON path. Defaults to the file next to this script.",
  )
  parser.add_argument(
      "--compare",
      nargs=2,
      metavar=("BASELINE", "CANDIDATE"),
      help="Compare two inventory JSON files. Additive symbols are allowed.",
  )
  parser.add_argument(
      "--strict",
      action="store_true",
      help="With --compare, also fail when symbols were added.",
  )
  parser.add_argument(
      "--self-check",
      action="store_true",
      help="Run in-memory parser, determinism, and compare checks.",
  )
  args = parser.parse_args(argv)
  try:
    if args.self_check:
      _self_check()
      print("self-check: ok")
      return 0
    if args.compare:
      baseline = _load_inventory(Path(args.compare[0]))
      candidate = _load_inventory(Path(args.compare[1]))
      notes = compare_inventories(baseline, candidate, strict=args.strict)
      if not notes:
        print("identical")
        return 0
      for note in notes:
        print(note)
      return 1 if _has_incompatible(notes) else 0
    repo_root = (args.repo_root or _default_repo_root()).resolve()
    inventory = build_inventory(repo_root)
    text = dump_inventory(inventory)
    again = dump_inventory(build_inventory(repo_root))
    if text != again:
      raise ParseError("two consecutive inventories were not byte-identical")
    output = args.output or _default_output()
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(text, encoding="utf-8", newline="\n")
    summary = inventory["summary"]
    print(
        f"wrote {output} schemas={summary['schema_count']} "
        f"fields={summary['field_count']} enums={summary['enum_count']}"
    )
    return 0
  except ParseError as exc:
    print(f"error: {exc}", file=sys.stderr)
    return 2


if __name__ == "__main__":
  sys.exit(main())
