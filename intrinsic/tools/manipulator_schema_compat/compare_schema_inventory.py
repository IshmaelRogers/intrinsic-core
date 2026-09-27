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

"""Compare a schema inventory to the protected manipulator baseline.

The input format is the JSON written by
``extract_public_schema_inventory.py`` (``manipulator-public-wire-v1``).
This tool does not parse ``.proto`` or ``.fbs`` sources, does not expand
``out_of_scope_imports``, and does not edit the checked-in baseline.

Wire policy:

* Removing a schema, message, field, enum, enum value, service, RPC, extend,
  or union is incompatible.
* Reusing a proto tag or FlatBuffer id, including a number that still sits in
  a baseline reserved range, is incompatible.
* Changing a proto tag, a FlatBuffer vtable id, or a struct's positional id
  is incompatible. Enum value renumbering is incompatible.
* Changing a field type, label, oneof, default, packed flag, required bit, or
  id kind is incompatible.
* Adding a field or enum value with a fresh number is additive and allowed.
  ``--strict`` treats additive lines as failures.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys
from typing import Any

SELECTION_RULE_ID = "manipulator-public-wire-v1"

_FBS_ID_KINDS = frozenset({"positional", "vtable"})

_FIELD_KEYS = (
    "default",
    "id_kind",
    "label",
    "name",
    "number",
    "oneof",
    "required",
    "type",
)


class InventoryError(Exception):
  """An inventory JSON file is missing or not in the baseline shape."""


def load_inventory(path: Path) -> dict[str, Any]:
  """Load one inventory document."""
  try:
    text = path.read_text(encoding="utf-8")
  except OSError as exc:
    raise InventoryError(str(exc)) from exc
  try:
    data = json.loads(text)
  except json.JSONDecodeError as exc:
    raise InventoryError(f"{path}: {exc}") from exc
  if not isinstance(data, dict):
    raise InventoryError(f"{path}: inventory is not a JSON object")
  if not isinstance(data.get("schemas"), list):
    raise InventoryError(f"{path}: schemas is missing or not a list")
  return data


def compare_inventories(
    baseline: dict[str, Any],
    candidate: dict[str, Any],
    strict: bool = False,
) -> list[str]:
  """Return sorted diagnostics for ``candidate`` against ``baseline``.

  Incompatible lines start with ``incompatible:``. Additive lines start with
  ``additive:``. With ``strict``, every additive line is rewritten so it
  starts with ``incompatible:``.
  """
  notes: list[str] = []
  notes.extend(_compare_identity(baseline, candidate))
  old_schemas = _index_schemas(baseline)
  new_schemas = _index_schemas(candidate)
  for path in sorted(set(old_schemas) - set(new_schemas)):
    notes.append(f"incompatible: removed schema {path}")
  for path in sorted(set(new_schemas) - set(old_schemas)):
    notes.append(f"additive: added schema {path}")
  for path in sorted(set(old_schemas) & set(new_schemas)):
    notes.extend(_compare_schema(old_schemas[path], new_schemas[path]))
  if strict:
    notes = [_promote_strict(note) for note in notes]
  return sorted(set(notes))


def has_incompatible(notes: list[str]) -> bool:
  return any(note.startswith("incompatible:") for note in notes)


def format_report(notes: list[str]) -> str:
  """Return the CLI report, including a trailing newline."""
  if not notes:
    return "identical\n"
  return "".join(f"{note}\n" for note in notes)


def _promote_strict(note: str) -> str:
  if note.startswith("additive: "):
    return "incompatible: strict " + note
  return note


def _compare_identity(
    baseline: dict[str, Any], candidate: dict[str, Any]
) -> list[str]:
  notes: list[str] = []
  old_rule = _rule_id(baseline)
  new_rule = _rule_id(candidate)
  if old_rule != new_rule:
    notes.append(f"incompatible: selection_rule_id {old_rule} -> {new_rule}")
  old_version = _version(baseline)
  new_version = _version(candidate)
  if old_version != new_version:
    notes.append(
        f"incompatible: inventory_version {old_version} -> {new_version}"
    )
  return notes


def _rule_id(inventory: dict[str, Any]) -> str:
  value = inventory.get("selection_rule_id")
  if isinstance(value, str) and value:
    return value
  return "missing"


def _version(inventory: dict[str, Any]) -> str:
  value = inventory.get("inventory_version")
  if isinstance(value, bool) or not isinstance(value, int):
    return "missing"
  return str(value)


def _index_schemas(inventory: dict[str, Any]) -> dict[str, dict[str, Any]]:
  found: dict[str, dict[str, Any]] = {}
  for index, schema in enumerate(inventory["schemas"]):
    if not isinstance(schema, dict):
      raise InventoryError(f"schemas[{index}] is not an object")
    path = schema.get("path")
    if not isinstance(path, str) or not path:
      raise InventoryError(f"schemas[{index}] is missing path")
    if path in found:
      raise InventoryError(f"duplicate schema path {path}")
    _require_list(schema, "messages", path)
    _require_list(schema, "enums", path)
    _require_list(schema, "services", path)
    _require_list(schema, "extends", path)
    _require_list(schema, "unions", path)
    found[path] = schema
  return found


def _require_list(record: dict[str, Any], key: str, context: str) -> list[Any]:
  value = record.get(key)
  if not isinstance(value, list):
    raise InventoryError(f"{context} is missing {key}")
  return value


def _compare_schema(old: dict[str, Any], new: dict[str, Any]) -> list[str]:
  path = old["path"]
  notes: list[str] = []
  for key in ("kind", "syntax", "package", "namespace"):
    old_value = old.get(key, "")
    new_value = new.get(key, "")
    if old_value != new_value:
      notes.append(f"incompatible: {path} {key} {old_value} -> {new_value}")
  for key in ("root_type", "file_identifier"):
    old_value = old.get(key, "")
    new_value = new.get(key, "")
    if old_value != new_value:
      notes.append(f"incompatible: {path} {key} {old_value!r} -> {new_value!r}")
  notes.extend(_compare_messages(path, old["messages"], new["messages"]))
  notes.extend(_compare_enums(path, old["enums"], new["enums"]))
  notes.extend(_compare_services(path, old["services"], new["services"]))
  notes.extend(_compare_extends(path, old["extends"], new["extends"]))
  notes.extend(_compare_unions(path, old["unions"], new["unions"]))
  return notes


def _index_named(
    items: list[Any], key: str, context: str, kind: str
) -> tuple[dict[str, dict[str, Any]], list[str]]:
  found: dict[str, dict[str, Any]] = {}
  notes: list[str] = []
  for item in items:
    if not isinstance(item, dict):
      raise InventoryError(f"{context} has a non-object {kind}")
    name = item.get(key)
    if not isinstance(name, str) or not name:
      raise InventoryError(f"{context} {kind} is missing {key}")
    if name in found:
      notes.append(f"incompatible: {context} duplicate {kind} {name}")
    found[name] = item
  return found, notes


def _compare_messages(
    path: str, old_items: list[Any], new_items: list[Any]
) -> list[str]:
  old_map, notes = _index_named(old_items, "qualified_name", path, "message")
  new_map, new_notes = _index_named(
      new_items, "qualified_name", path, "message"
  )
  notes.extend(new_notes)
  for name in sorted(set(old_map) - set(new_map)):
    notes.append(f"incompatible: {path} removed message {name}")
  for name in sorted(set(new_map) - set(old_map)):
    notes.append(f"additive: {path} added message {name}")
  for name in sorted(set(old_map) & set(new_map)):
    old_item = old_map[name]
    new_item = new_map[name]
    old_layout = old_item.get("layout", "")
    new_layout = new_item.get("layout", "")
    if old_layout != new_layout:
      notes.append(
          f"incompatible: {path} message {name} layout "
          f"{old_layout} -> {new_layout}"
      )
    old_fields = _require_list(old_item, "fields", f"{path} message {name}")
    new_fields = _require_list(new_item, "fields", f"{path} message {name}")
    notes.extend(
        _compare_fields(
            path,
            f"message {name}",
            old_fields,
            new_fields,
            _ranges(old_item, "reserved_ranges", f"{path} message {name}"),
        )
    )
    notes.extend(_compare_reserved(path, f"message {name}", old_item, new_item))
    notes.extend(
        _compare_range_group(
            path,
            f"message {name}",
            "extension range",
            _ranges(old_item, "extension_ranges", f"{path} message {name}"),
            _ranges(new_item, "extension_ranges", f"{path} message {name}"),
        )
    )
  return notes


def _ranges(record: dict[str, Any], key: str, context: str) -> list[Any]:
  value = record.get(key, [])
  if not isinstance(value, list):
    raise InventoryError(f"{context} {key} is not a list")
  return value


def _compare_fields(
    path: str,
    owner: str,
    old_fields: list[Any],
    new_fields: list[Any],
    old_reserved: list[Any],
) -> list[str]:
  notes: list[str] = []
  old_checked = _check_fields(path, owner, old_fields, "baseline")
  new_checked = _check_fields(path, owner, new_fields, "candidate")
  notes.extend(old_checked[0])
  notes.extend(new_checked[0])
  old_by_name = old_checked[1]
  new_by_name = new_checked[1]
  old_by_number = {field["number"]: field["name"] for field in old_fields}
  new_by_number = {field["number"]: field["name"] for field in new_fields}

  for name in sorted(set(old_by_name) - set(new_by_name)):
    field = old_by_name[name]
    word = _id_word(field["id_kind"])
    notes.append(
        f"incompatible: {path} {owner} removed field {name} "
        f"{word} {field['number']}"
    )
  for name in sorted(set(new_by_name) - set(old_by_name)):
    field = new_by_name[name]
    notes.extend(
        _added_number(
            path,
            owner,
            name,
            field["number"],
            _id_word(field["id_kind"]),
            old_by_number,
            old_reserved,
            added_prefix="added field",
        )
    )
  for name in sorted(set(old_by_name) & set(new_by_name)):
    notes.extend(
        _compare_existing_field(
            path,
            owner,
            old_by_name[name],
            new_by_name[name],
            old_by_number,
            new_by_name,
            old_reserved,
        )
    )
  return notes


def _check_fields(
    path: str, owner: str, fields: list[Any], side: str
) -> tuple[list[str], dict[str, dict[str, Any]]]:
  notes: list[str] = []
  by_name: dict[str, dict[str, Any]] = {}
  seen_numbers: dict[int, str] = {}
  context = f"{path} {owner}"
  for field in fields:
    _check_field(field, context)
    name = field["name"]
    if name in by_name:
      notes.append(f"incompatible: {context} duplicate field name {name}")
    by_name[name] = field
    number = field["number"]
    previous = seen_numbers.get(number)
    if previous is not None and previous != name:
      word = _id_word(field["id_kind"])
      notes.append(
          f"incompatible: {context} {side} duplicate {word} "
          f"{number} ({previous} and {name})"
      )
    else:
      seen_numbers[number] = name
  return notes, by_name


def _check_field(field: Any, context: str) -> None:
  if not isinstance(field, dict):
    raise InventoryError(f"{context} has a non-object field")
  missing = [key for key in _FIELD_KEYS if key not in field]
  if missing:
    name = field.get("name", "<unnamed>")
    joined = ", ".join(missing)
    raise InventoryError(f"{context} field {name} missing {joined}")
  number = field["number"]
  if isinstance(number, bool) or not isinstance(number, int):
    raise InventoryError(
        f"{context} field {field['name']} number is not an integer"
    )
  if not isinstance(field["name"], str) or not field["name"]:
    raise InventoryError(f"{context} has a field with an empty name")
  if not isinstance(field["id_kind"], str):
    raise InventoryError(
        f"{context} field {field['name']} id_kind is not a string"
    )


def _added_number(
    path: str,
    owner: str,
    name: str,
    number: int,
    word: str,
    old_by_number: dict[int, str],
    old_reserved: list[Any],
    added_prefix: str,
) -> list[str]:
  """Classify a new field or enum value number."""
  notes: list[str] = []
  previous = old_by_number.get(number)
  reserved_key = _range_covering(old_reserved, number)
  reused = previous is not None and previous != name
  if reused:
    notes.append(
        f"incompatible: {path} {owner} {word} {number} reused "
        f"by {name} (was {previous})"
    )
  if reserved_key is not None:
    notes.append(
        f"incompatible: {path} {owner} field {name} {word} {number} "
        f"reuses reserved range {reserved_key}"
    )
  if not reused and reserved_key is None:
    notes.append(
        f"additive: {path} {owner} {added_prefix} {name} {word} {number}"
    )
  return notes


def _compare_existing_field(
    path: str,
    owner: str,
    old_field: dict[str, Any],
    new_field: dict[str, Any],
    old_by_number: dict[int, str],
    new_by_name: dict[str, dict[str, Any]],
    old_reserved: list[Any],
) -> list[str]:
  notes: list[str] = []
  name = old_field["name"]
  word = _id_word_for(old_field, new_field)
  old_number = old_field["number"]
  new_number = new_field["number"]
  if old_number != new_number:
    notes.append(
        f"incompatible: {path} {owner} field {name} {word} "
        f"{old_number} -> {new_number}"
    )
    previous = old_by_number.get(new_number)
    if (
        previous is not None
        and previous != name
        and previous not in new_by_name
    ):
      notes.append(
          f"incompatible: {path} {owner} {word} {new_number} reused "
          f"by {name} (was {previous})"
      )
    reserved_key = _range_covering(old_reserved, new_number)
    if reserved_key is not None:
      notes.append(
          f"incompatible: {path} {owner} field {name} {word} "
          f"{new_number} reuses reserved range {reserved_key}"
      )
  notes.extend(_compare_field_attrs(path, owner, old_field, new_field))
  return notes


def _compare_field_attrs(
    path: str,
    owner: str,
    old_field: dict[str, Any],
    new_field: dict[str, Any],
) -> list[str]:
  notes: list[str] = []
  name = old_field["name"]
  pairs = (
      ("type", False),
      ("label", False),
      ("id_kind", False),
      ("oneof", True),
      ("required", False),
      ("default", True),
  )
  for key, use_repr in pairs:
    old_value = old_field[key]
    new_value = new_field[key]
    if old_value != new_value:
      old_text = repr(old_value) if use_repr else str(old_value)
      new_text = repr(new_value) if use_repr else str(new_value)
      notes.append(
          f"incompatible: {path} {owner} field {name} {key} "
          f"{old_text} -> {new_text}"
      )
  old_packed = old_field.get("packed", None)
  new_packed = new_field.get("packed", None)
  if old_packed != new_packed:
    notes.append(
        f"incompatible: {path} {owner} field {name} packed "
        f"{_packed_text(old_packed)} -> {_packed_text(new_packed)}"
    )
  old_extra = tuple(sorted(old_field.get("extra_attributes") or []))
  new_extra = tuple(sorted(new_field.get("extra_attributes") or []))
  if old_extra != new_extra:
    notes.append(
        f"incompatible: {path} {owner} field {name} extra_attributes "
        f"{list(old_extra)!r} -> {list(new_extra)!r}"
    )
  return notes


def _packed_text(value: Any) -> str:
  if value is None:
    return "unset"
  if value is True:
    return "true"
  if value is False:
    return "false"
  return repr(value)


def _id_word(id_kind: str) -> str:
  if id_kind in _FBS_ID_KINDS:
    return "flatbuffer id"
  return "proto tag"


def _id_word_for(old_field: dict[str, Any], new_field: dict[str, Any]) -> str:
  if (
      old_field["id_kind"] in _FBS_ID_KINDS
      or new_field["id_kind"] in _FBS_ID_KINDS
  ):
    return "flatbuffer id"
  return "proto tag"


def _range_key(item: dict[str, Any]) -> str:
  return f"{item['start']}:{item['end']}"


def _range_covering(ranges: list[Any], number: int) -> str | None:
  parsed = [_check_range(item) for item in ranges]
  parsed.sort(key=lambda item: (item["start"], str(item["end"])))
  for item in parsed:
    start = item["start"]
    end = item["end"]
    if end == "max":
      if number >= start:
        return _range_key(item)
      continue
    if start <= number <= end:
      return _range_key(item)
  return None


def _check_range(item: Any) -> dict[str, Any]:
  if not isinstance(item, dict):
    raise InventoryError(f"reserved range is not an object: {item!r}")
  start = item.get("start")
  end = item.get("end")
  if isinstance(start, bool) or not isinstance(start, int):
    raise InventoryError(f"reserved range start is not an integer: {item!r}")
  if end != "max" and (isinstance(end, bool) or not isinstance(end, int)):
    raise InventoryError(
        f"reserved range end is not an integer or max: {item!r}"
    )
  return item


def _compare_reserved(
    path: str,
    owner: str,
    old_item: dict[str, Any],
    new_item: dict[str, Any],
) -> list[str]:
  notes: list[str] = []
  old_names = set(_name_list(old_item, "reserved_names", f"{path} {owner}"))
  new_names = set(_name_list(new_item, "reserved_names", f"{path} {owner}"))
  for name in sorted(old_names - new_names):
    notes.append(f"incompatible: {path} {owner} dropped reserved name {name}")
  for name in sorted(new_names - old_names):
    notes.append(f"additive: {path} {owner} reserved name {name}")
  notes.extend(
      _compare_range_group(
          path,
          owner,
          "reserved range",
          _ranges(old_item, "reserved_ranges", f"{path} {owner}"),
          _ranges(new_item, "reserved_ranges", f"{path} {owner}"),
      )
  )
  return notes


def _name_list(record: dict[str, Any], key: str, context: str) -> list[str]:
  value = record.get(key, [])
  if not isinstance(value, list):
    raise InventoryError(f"{context} {key} is not a list")
  for item in value:
    if not isinstance(item, str):
      raise InventoryError(f"{context} {key} contains a non-string")
  return value


def _compare_range_group(
    path: str,
    owner: str,
    label: str,
    old_ranges: list[Any],
    new_ranges: list[Any],
) -> list[str]:
  notes: list[str] = []
  old_keys = {_range_key(_check_range(item)) for item in old_ranges}
  new_keys = {_range_key(_check_range(item)) for item in new_ranges}
  for key in sorted(old_keys - new_keys):
    notes.append(f"incompatible: {path} {owner} dropped {label} {key}")
  for key in sorted(new_keys - old_keys):
    notes.append(f"additive: {path} {owner} {label} {key}")
  return notes


def _compare_enums(
    path: str, old_items: list[Any], new_items: list[Any]
) -> list[str]:
  old_map, notes = _index_named(old_items, "qualified_name", path, "enum")
  new_map, new_notes = _index_named(new_items, "qualified_name", path, "enum")
  notes.extend(new_notes)
  for name in sorted(set(old_map) - set(new_map)):
    notes.append(f"incompatible: {path} removed enum {name}")
  for name in sorted(set(new_map) - set(old_map)):
    notes.append(f"additive: {path} added enum {name}")
  for name in sorted(set(old_map) & set(new_map)):
    old_enum = old_map[name]
    new_enum = new_map[name]
    old_type = old_enum.get("underlying_type", "")
    new_type = new_enum.get("underlying_type", "")
    if old_type != new_type:
      notes.append(
          f"incompatible: {path} enum {name} underlying_type "
          f"{old_type} -> {new_type}"
      )
    old_alias = bool(old_enum.get("allow_alias", False))
    new_alias = bool(new_enum.get("allow_alias", False))
    if old_alias != new_alias:
      notes.append(
          f"incompatible: {path} enum {name} allow_alias "
          f"{old_alias} -> {new_alias}"
      )
    notes.extend(_compare_enum_values(path, name, old_enum, new_enum))
    notes.extend(_compare_reserved(path, f"enum {name}", old_enum, new_enum))
  return notes


def _enum_values(enum: dict[str, Any], context: str) -> list[dict[str, Any]]:
  values = _require_list(enum, "values", context)
  checked: list[dict[str, Any]] = []
  for value in values:
    if not isinstance(value, dict):
      raise InventoryError(f"{context} has a non-object enum value")
    name = value.get("name")
    number = value.get("number")
    if not isinstance(name, str) or not name:
      raise InventoryError(f"{context} has an enum value with an empty name")
    if isinstance(number, bool) or not isinstance(number, int):
      raise InventoryError(f"{context} value {name} number is not an integer")
    checked.append(value)
  return checked


def _compare_enum_values(
    path: str,
    enum_name: str,
    old_enum: dict[str, Any],
    new_enum: dict[str, Any],
) -> list[str]:
  owner = f"enum {enum_name}"
  context = f"{path} {owner}"
  old_values = _enum_values(old_enum, context)
  new_values = _enum_values(new_enum, context)
  notes: list[str] = []
  old_alias = bool(old_enum.get("allow_alias", False))
  new_alias = bool(new_enum.get("allow_alias", False))
  notes.extend(
      _duplicate_enum_numbers(
          path, enum_name, old_values, "baseline", allow_alias=old_alias
      )
  )
  notes.extend(
      _duplicate_enum_numbers(
          path, enum_name, new_values, "candidate", allow_alias=new_alias
      )
  )
  old_by_name = {item["name"]: item for item in old_values}
  new_by_name = {item["name"]: item for item in new_values}
  old_by_number = {item["number"]: item["name"] for item in old_values}
  old_reserved = _ranges(old_enum, "reserved_ranges", context)
  for name in sorted(set(old_by_name) - set(new_by_name)):
    number = old_by_name[name]["number"]
    notes.append(
        f"incompatible: {path} {owner} removed value {name} number {number}"
    )
  for name in sorted(set(new_by_name) - set(old_by_name)):
    number = new_by_name[name]["number"]
    notes.extend(
        _added_enum_value(
            path, owner, name, number, old_by_number, old_reserved
        )
    )
  for name in sorted(set(old_by_name) & set(new_by_name)):
    old_number = old_by_name[name]["number"]
    new_number = new_by_name[name]["number"]
    if old_number == new_number:
      continue
    notes.append(
        f"incompatible: {path} {owner} value {name} number "
        f"{old_number} -> {new_number}"
    )
    previous = old_by_number.get(new_number)
    if (
        previous is not None
        and previous != name
        and previous not in new_by_name
    ):
      notes.append(
          f"incompatible: {path} {owner} value number {new_number} "
          f"reused by {name} (was {previous})"
      )
    reserved_key = _range_covering(old_reserved, new_number)
    if reserved_key is not None:
      notes.append(
          f"incompatible: {path} {owner} value {name} number "
          f"{new_number} reuses reserved range {reserved_key}"
      )
  return notes


def _duplicate_enum_numbers(
    path: str,
    enum_name: str,
    values: list[dict[str, Any]],
    side: str,
    allow_alias: bool,
) -> list[str]:
  notes: list[str] = []
  seen: dict[int, str] = {}
  seen_names: set[str] = set()
  for value in values:
    name = value["name"]
    if name in seen_names:
      notes.append(
          f"incompatible: {path} enum {enum_name} duplicate value name {name}"
      )
    seen_names.add(name)
    number = value["number"]
    previous = seen.get(number)
    if previous is not None and previous != name and not allow_alias:
      notes.append(
          f"incompatible: {path} enum {enum_name} {side} duplicate number "
          f"{number} ({previous} and {name})"
      )
    else:
      seen[number] = name
  return notes


def _added_enum_value(
    path: str,
    owner: str,
    name: str,
    number: int,
    old_by_number: dict[int, str],
    old_reserved: list[Any],
) -> list[str]:
  notes: list[str] = []
  previous = old_by_number.get(number)
  reserved_key = _range_covering(old_reserved, number)
  reused = previous is not None and previous != name
  if reused:
    notes.append(
        f"incompatible: {path} {owner} value number {number} reused "
        f"by {name} (was {previous})"
    )
  if reserved_key is not None:
    notes.append(
        f"incompatible: {path} {owner} value {name} number {number} "
        f"reuses reserved range {reserved_key}"
    )
  if not reused and reserved_key is None:
    notes.append(f"additive: {path} {owner} added value {name} number {number}")
  return notes


def _compare_services(
    path: str, old_items: list[Any], new_items: list[Any]
) -> list[str]:
  old_map, notes = _index_named(old_items, "name", path, "service")
  new_map, new_notes = _index_named(new_items, "name", path, "service")
  notes.extend(new_notes)
  for name in sorted(set(old_map) - set(new_map)):
    notes.append(f"incompatible: {path} removed service {name}")
  for name in sorted(set(new_map) - set(old_map)):
    notes.append(f"additive: {path} added service {name}")
  for name in sorted(set(old_map) & set(new_map)):
    old_methods = _methods(old_map[name], f"{path} service {name}")
    new_methods = _methods(new_map[name], f"{path} service {name}")
    old_by_name, old_notes = _index_named(
        old_methods, "name", f"{path} service {name}", "rpc"
    )
    new_by_name, new_notes = _index_named(
        new_methods, "name", f"{path} service {name}", "rpc"
    )
    notes.extend(old_notes)
    notes.extend(new_notes)
    for method_name in sorted(set(old_by_name) - set(new_by_name)):
      notes.append(
          f"incompatible: {path} service {name} removed rpc {method_name}"
      )
    for method_name in sorted(set(new_by_name) - set(old_by_name)):
      notes.append(f"additive: {path} service {name} added rpc {method_name}")
    for method_name in sorted(set(old_by_name) & set(new_by_name)):
      notes.extend(
          _compare_rpc(
              path,
              name,
              method_name,
              old_by_name[method_name],
              new_by_name[method_name],
          )
      )
  return notes


def _methods(service: dict[str, Any], context: str) -> list[Any]:
  return _require_list(service, "methods", context)


def _compare_rpc(
    path: str,
    service: str,
    method: str,
    old_method: dict[str, Any],
    new_method: dict[str, Any],
) -> list[str]:
  notes: list[str] = []
  for slot in (
      "request_type",
      "response_type",
      "client_streaming",
      "server_streaming",
  ):
    if slot not in old_method or slot not in new_method:
      raise InventoryError(
          f"{path} service {service} rpc {method} is missing {slot}"
      )
    old_value = old_method[slot]
    new_value = new_method[slot]
    if old_value != new_value:
      notes.append(
          f"incompatible: {path} service {service} rpc {method} "
          f"{slot} {old_value} -> {new_value}"
      )
  return notes


def _compare_extends(
    path: str, old_items: list[Any], new_items: list[Any]
) -> list[str]:
  old_by_extendee = _extends_by_extendee(old_items, path)
  new_by_extendee = _extends_by_extendee(new_items, path)
  notes: list[str] = []
  for extendee in sorted(set(old_by_extendee) - set(new_by_extendee)):
    notes.append(f"incompatible: {path} removed extend {extendee}")
    notes.extend(
        _compare_fields(
            path,
            f"extend {extendee}",
            old_by_extendee[extendee],
            [],
            [],
        )
    )
  for extendee in sorted(set(new_by_extendee) - set(old_by_extendee)):
    notes.append(f"additive: {path} added extend {extendee}")
    notes.extend(
        _compare_fields(
            path,
            f"extend {extendee}",
            [],
            new_by_extendee[extendee],
            [],
        )
    )
  for extendee in sorted(set(old_by_extendee) & set(new_by_extendee)):
    notes.extend(
        _compare_fields(
            path,
            f"extend {extendee}",
            old_by_extendee[extendee],
            new_by_extendee[extendee],
            [],
        )
    )
  return notes


def _extends_by_extendee(items: list[Any], path: str) -> dict[str, list[Any]]:
  found: dict[str, list[Any]] = {}
  for item in items:
    if not isinstance(item, dict):
      raise InventoryError(f"{path} has a non-object extend")
    extendee = item.get("extendee")
    if not isinstance(extendee, str) or not extendee:
      raise InventoryError(f"{path} extend is missing extendee")
    fields = _require_list(item, "fields", f"{path} extend {extendee}")
    found.setdefault(extendee, []).extend(fields)
  return found


def _compare_unions(
    path: str, old_items: list[Any], new_items: list[Any]
) -> list[str]:
  old_map, notes = _index_named(old_items, "name", path, "union")
  new_map, new_notes = _index_named(new_items, "name", path, "union")
  notes.extend(new_notes)
  for name in sorted(set(old_map) - set(new_map)):
    notes.append(f"incompatible: {path} removed union {name}")
    notes.extend(
        _compare_union_members(
            path,
            name,
            _union_members(old_map[name], f"{path} union {name}"),
            [],
        )
    )
  for name in sorted(set(new_map) - set(old_map)):
    notes.append(f"additive: {path} added union {name}")
    notes.extend(
        _compare_union_members(
            path,
            name,
            [],
            _union_members(new_map[name], f"{path} union {name}"),
        )
    )
  for name in sorted(set(old_map) & set(new_map)):
    old_members = _union_members(old_map[name], f"{path} union {name}")
    new_members = _union_members(new_map[name], f"{path} union {name}")
    notes.extend(_compare_union_members(path, name, old_members, new_members))
  return notes


def _union_members(union: dict[str, Any], context: str) -> list[dict[str, Any]]:
  members = _require_list(union, "members", context)
  checked: list[dict[str, Any]] = []
  for member in members:
    if not isinstance(member, dict):
      raise InventoryError(f"{context} has a non-object member")
    name = member.get("name")
    number = member.get("number")
    if not isinstance(name, str) or not name:
      raise InventoryError(f"{context} has a member with an empty name")
    if isinstance(number, bool) or not isinstance(number, int):
      raise InventoryError(f"{context} member {name} number is not an integer")
    checked.append(member)
  return checked


def _compare_union_members(
    path: str,
    union_name: str,
    old_members: list[dict[str, Any]],
    new_members: list[dict[str, Any]],
) -> list[str]:
  notes: list[str] = []
  old_by_name = {item["name"]: item for item in old_members}
  new_by_name = {item["name"]: item for item in new_members}
  old_by_number = {item["number"]: item["name"] for item in old_members}
  for name in sorted(set(old_by_name) - set(new_by_name)):
    number = old_by_name[name]["number"]
    notes.append(
        f"incompatible: {path} union {union_name} removed member "
        f"{name} number {number}"
    )
  for name in sorted(set(new_by_name) - set(old_by_name)):
    number = new_by_name[name]["number"]
    previous = old_by_number.get(number)
    if previous is not None and previous != name:
      notes.append(
          f"incompatible: {path} union {union_name} member number "
          f"{number} reused by {name} (was {previous})"
      )
    else:
      notes.append(
          f"additive: {path} union {union_name} added member "
          f"{name} number {number}"
      )
  for name in sorted(set(old_by_name) & set(new_by_name)):
    old_number = old_by_name[name]["number"]
    new_number = new_by_name[name]["number"]
    if old_number != new_number:
      notes.append(
          f"incompatible: {path} union {union_name} member {name} "
          f"number {old_number} -> {new_number}"
      )
  return notes


def _default_baseline() -> Path:
  return (
      Path(__file__).resolve().parent.parent
      / "manipulator_schema_baseline"
      / "manipulator_public_schema_inventory.json"
  )


def main(argv: list[str] | None = None) -> int:
  parser = argparse.ArgumentParser(
      description=(
          "Compare a schema inventory to the protected manipulator baseline."
      )
  )
  parser.add_argument(
      "paths",
      nargs="+",
      type=Path,
      help=(
          "CANDIDATE compares against the protected baseline. "
          "BASELINE CANDIDATE compares two inventory files."
      ),
  )
  parser.add_argument(
      "--strict",
      action="store_true",
      help="Also fail when symbols were only added.",
  )
  args = parser.parse_args(argv)
  if len(args.paths) == 1:
    baseline_path = _default_baseline()
    candidate_path = args.paths[0]
  elif len(args.paths) == 2:
    baseline_path, candidate_path = args.paths
  else:
    parser.error("expected CANDIDATE or BASELINE CANDIDATE")
  try:
    baseline = load_inventory(baseline_path)
    candidate = load_inventory(candidate_path)
    notes = compare_inventories(baseline, candidate, strict=args.strict)
  except InventoryError as exc:
    print(f"error: {exc}", file=sys.stderr)
    return 2
  sys.stdout.write(format_report(notes))
  return 1 if has_incompatible(notes) else 0


if __name__ == "__main__":
  sys.exit(main())
