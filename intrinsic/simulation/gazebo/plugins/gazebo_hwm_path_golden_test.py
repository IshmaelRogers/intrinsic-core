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

"""Golden for the simulated manipulator Gazebo hardware-module path.

Loads the repository-owned world fixture headlessly (XML only, rendering off)
against the existing world template and UR5e asset. Compares part and
interface discovery plus command/status behavior to a checked-in fixture.
Does not start Gazebo and does not edit production sources. Rewriting the
fixture requires ``--update-golden`` (see README.md).
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
import time
import unittest
import xml.etree.ElementTree as ET

SCHEMA = "gazebo-hwm-path-v1"

# The unittest compare must finish inside this budget. It does not sleep or
# start a simulator. README.md documents the same number.
CI_RUNTIME_BUDGET_SECONDS = 5.0

# Case-insensitive scan of the rendered fixture and the launched XML. A
# manipulator hardware-module path must not pick up these names.
FORBIDDEN_TOKENS = ("bathymetry", "marine", "thruster", "uuv", "vehicle")

_ACTUATED_JOINT_TYPES = {"prismatic": "PRISMATIC", "revolute": "REVOLUTE"}

_DIR = Path(__file__).resolve().parent
_GOLDEN = _DIR / "testdata" / "gazebo_hwm_path.golden.json"
_WORLD = _DIR / "testdata" / "gazebo_hwm_manipulator.world.sdf"


def _repo_root() -> Path:
  for parent in _DIR.parents:
    if (parent / "MODULE.bazel").is_file():
      return parent
  raise RuntimeError("repository root was not found")


_REPO = _repo_root()
_HWM_CC = _DIR / "gazebo_hwm.cc"
_HWM_H = _DIR / "gazebo_hwm.h"
_INFER_CC = _DIR / "infer_hardware_interfaces.cc"
_INFER_H = _DIR / "infer_hardware_interfaces.h"
_DATA_CC = _DIR / "sim_hardware_interface_data.cc"
_REGISTER_CC = _DIR / "hardware_module_launcher_register.cc"
_SERVER_CC = _REPO / "intrinsic/simulation/gazebo/server_main_impl.cc"
_TEMPLATE = (
    _REPO / "intrinsic/simulation/gazebo/world_templates/default.sdf.tpl"
)
_BASELINE = _REPO / ".github/baseline/manipulator_targets.tsv"


def _read(path: Path) -> str:
  return path.read_text(encoding="utf-8")


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


def _function_body(text: str, signature: str) -> str:
  start = text.find(signature)
  if start < 0:
    raise ValueError(f"{signature} was not found")
  brace = text.find("{", start)
  end = _matching_delimiter(text, brace, "{", "}")
  return text[brace + 1 : end]


def _constexpr_strings(header: str) -> dict[str, str]:
  names: dict[str, str] = {}
  pattern = re.compile(r'constexpr char (k\w+)\[\]\s*=\s*"([^"]*)"\s*;')
  for match in pattern.finditer(header):
    names[match.group(1)] = match.group(2)
  if "kAutomaticJointGroupName" not in names:
    raise ValueError("automatic interface names were not found")
  return names


def _counted_joint_types(infer_cc: str) -> list[str]:
  body = _function_body(infer_cc, "CountRevoluteAndPrismaticJoints(")
  found = re.findall(r"JointType::(\w+)", body)
  if found != ["REVOLUTE", "PRISMATIC"]:
    raise ValueError(f"unexpected counted joint types: {found}")
  return ["prismatic", "revolute"]


def _attr(tag: str, name: str) -> str:
  match = re.search(rf'(?<![\w]){name}\s*=\s*"([^"]*)"', tag)
  if match is None:
    match = re.search(rf"(?<![\w]){name}\s*=\s*'([^']*)'", tag)
  return "" if match is None else match.group(1)


def _plugins(xml_text: str, source: str) -> list[dict]:
  # The world template uses an unbound gz: prefix, so plugin tags are read
  # from the text instead of a namespace-aware XML tree.
  plugins = []
  for match in re.finditer(r"<plugin\b(.*?)>", xml_text, re.S):
    tag = match.group(1)
    plugins.append(
        {
            "filename": _attr(tag, "filename"),
            "name": _attr(tag, "name"),
            "source": source,
        }
    )
  return plugins


def _plugin_alias(register_cc: str) -> str:
  match = re.search(
      r'GZ_ADD_STATIC_PLUGIN_ALIAS\(\s*HardwareModuleLauncher\s*,\s*"([^"]+)"',
      register_cc,
  )
  if match is None:
    raise ValueError("HardwareModuleLauncher alias was not found")
  return match.group(1)


def _server_launch(server_cc: str) -> dict:
  body = _function_body(server_cc, "DefaultServerConfig()")
  if "SetHeadlessRendering(true)" not in body:
    raise ValueError("headless rendering is not set")
  engines = re.findall(r'SetPhysicsEngine\(\s*"([^"]+)"\s*\)', body)
  if len(engines) != 2:
    raise ValueError(f"expected two physics engines, found {engines}")
  return {
      "headless_rendering": True,
      "physics_engine": engines[1],
      "physics_engine_when_use_dart": engines[0],
  }


def _resolve_model(uri: str) -> tuple[str, Path]:
  prefix = "model://"
  if not uri.startswith(prefix):
    raise ValueError(f"model uri must use {prefix}: {uri}")
  model_dir = _REPO / uri[len(prefix) :]
  config_path = model_dir / "model.config"
  if not config_path.is_file():
    raise ValueError(f"existing model.config was not found for {uri}")
  root = ET.fromstring(_read(config_path))
  sdf = root.find("sdf")
  if sdf is None or not (sdf.text or "").strip():
    raise ValueError(f"{config_path} does not name an sdf file")
  model_path = model_dir / sdf.text.strip()
  if not model_path.is_file():
    raise ValueError(f"existing model file was not found: {model_path}")
  return uri, model_path


def _joints(model_path: Path, counted_types: list[str]) -> list[dict]:
  root = ET.fromstring(_read(model_path))
  allowed = set(counted_types)
  joints = []
  for joint in root.iter("joint"):
    joint_type = joint.attrib.get("type", "")
    if joint_type not in allowed:
      continue
    name = joint.attrib.get("name", "")
    if not name:
      raise ValueError("actuated joint is missing a name")
    joints.append({"name": name, "type": joint_type})
  if not joints:
    raise ValueError(f"{model_path} has no actuated joints")
  return joints


def _sensor_counts(model_text: str) -> dict[str, int]:
  root = ET.fromstring(model_text)
  ft_count = 0
  rangefinder_count = 0
  for sensor in root.iter("sensor"):
    sensor_type = sensor.attrib.get("type", "")
    if sensor_type == "force_torque":
      ft_count += 1
    if sensor_type == "gpu_lidar":
      rangefinder_count += 1
  return {
      "force_torque": ft_count,
      "rangefinder": rangefinder_count,
  }


def _inferred_interfaces(infer_cc: str, names: dict[str, str]) -> list[dict]:
  body = _function_body(infer_cc, "PopulateInferredJointGroupAndInterfaces(")
  pattern = re.compile(
      r"\[automatic_interface_names::(kAutomatic\w+)\]\s*"
      r"\.mutable_(\w+)\(\)(.*?);",
      re.S,
  )
  interfaces = []
  for match in pattern.finditer(body):
    constant = match.group(1)
    field = match.group(2)
    tail = match.group(3)
    if constant not in names:
      raise ValueError(f"unknown interface constant {constant}")
    group_match = re.search(r"automatic_interface_names::(kAutomatic\w+)", tail)
    if group_match is None:
      raise ValueError(f"{constant} does not name a joint group")
    group_constant = group_match.group(1)
    interfaces.append(
        {
            "field": field,
            "joint_group": names[group_constant],
            "name": names[constant],
            "strict": "set_strict(true)" in tail,
        }
    )
  if len(interfaces) < 11:
    raise ValueError("inferred joint interfaces were not found")
  return interfaces


def _case_registry(data_cc: str) -> dict[str, str]:
  """Map a proto field name to the data type used when strict is unset."""
  body = _function_body(data_cc, "BuildHardwareInterfaceData(")
  registry: dict[str, str] = {}
  cursor = 0
  needle = "case "
  while True:
    start = body.find(needle, cursor)
    if start < 0:
      break
    brace = body.find("{", start)
    if brace < 0:
      break
    end = _matching_delimiter(body, brace, "{", "}")
    block = body[start:end]
    cursor = end + 1
    label = re.search(r"\b(k\w+)\s*:", block)
    if label is None:
      continue
    field_match = re.search(r"interface_config\.(\w+)\(", block)
    if field_match is None:
      continue
    field = field_match.group(1)
    getters = re.findall(r"\b(Get\w+FromEcm)\(", block)
    if not getters:
      continue
    if re.search(r"if\s*\(\s*interface(?:_config)?\.strict\(\)\s*\)", block):
      else_at = block.find("else")
      if else_at < 0:
        raise ValueError(f"{field} has a strict check and no else")
      else_getters = re.findall(r"\b(Get\w+FromEcm)\(", block[else_at:])
      if len(else_getters) != 1:
        raise ValueError(f"{field} else branch getters: {else_getters}")
      getter = else_getters[0]
    else:
      if len(getters) != 1:
        raise ValueError(f"{field} getters: {getters}")
      getter = getters[0]
    registry[field] = _getter_return_type(data_cc, getter)
  if "joint_position_command" not in registry:
    raise ValueError("joint position command was not mapped")
  return registry


def _getter_return_type(data_cc: str, getter: str) -> str:
  match = re.search(rf"absl::StatusOr<(\w+)>\s*{re.escape(getter)}\(", data_cc)
  if match is None:
    raise ValueError(f"return type for {getter} was not found")
  return match.group(1)


def _advertise_by_type(hwm_cc: str) -> dict[str, dict]:
  visitor = _function_body(hwm_cc, "struct RegisterInterfaceVisitor")
  found: dict[str, dict] = {}
  cursor = 0
  needle = "operator()"
  while True:
    start = visitor.find(needle, cursor)
    if start < 0:
      break
    signature = re.search(
        r"hardware_interface_data::(\w+)\s*&", visitor[start : start + 400]
    )
    brace = visitor.find("{", start)
    if signature is None or brace < 0:
      cursor = start + len(needle)
      continue
    end = _matching_delimiter(visitor, brace, "{", "}")
    body = visitor[brace + 1 : end]
    cursor = end + 1
    advertise = re.search(r"\.(Advertise\w+)<\s*intrinsic_fbs::(\w+)\s*>", body)
    if advertise is None:
      raise ValueError(f"no advertise call for {signature.group(1)}")
    data_type = signature.group(1)
    call = advertise.group(1)
    strict = call == "AdvertiseStrictInterface"
    if data_type.startswith("NonStrict"):
      strict = False
    elif data_type.startswith("Strict"):
      strict = True
    if call == "AdvertiseMutableInterface":
      direction = "status"
    elif call in ("AdvertiseInterface", "AdvertiseStrictInterface"):
      direction = "command"
    else:
      raise ValueError(f"unknown advertise call {call}")
    found[data_type] = {
        "advertise": call,
        "direction": direction,
        "flatbuffer": advertise.group(2),
        "strict": strict,
        "tracks_joint_count": "LookupNumJoints" in body,
    }
  if "NonStrictJointPositionCommandData" not in found:
    raise ValueError("joint position advertise path was not found")
  return found


def _safety_interface(hwm_cc: str) -> dict:
  body = _function_body(hwm_cc, "RegisterInterfaces(")
  match = re.search(
      r'interface_handle_map\["([^"]+)"\]\s*,\s*'
      r"interface_registry\s*\.\s*(Advertise\w+)<\s*intrinsic_fbs::(\w+)\s*>\(\s*"
      r'"([^"]+)"',
      body,
      re.S,
  )
  if match is None:
    raise ValueError("safety interface registration was not found")
  map_name, call, flatbuffer, arg_name = match.groups()
  if map_name != arg_name:
    raise ValueError("safety interface name is inconsistent")
  if call != "AdvertiseMutableInterface":
    raise ValueError(f"safety interface uses {call}")
  return {
      "advertise": call,
      "direction": "status",
      "flatbuffer": flatbuffer,
      "joint_group": None,
      "name": arg_name,
      "strict": False,
      "tracks_joint_count": False,
  }


def _shared_inference_cases(infer_cc: str) -> list[str]:
  body = _function_body(infer_cc, "InferSimHardwareModuleConfig(")
  match = re.search(
      r"HardwareInterfacesCase::(HARDWARE_INTERFACES_NOT_SET)\s*:\s*"
      r"case\s+::intrinsic_proto::sim::SimHardwareModuleConfig::\s*"
      r"HardwareInterfacesCase::(kFromSelf)\s*:",
      body,
  )
  if match is None:
    raise ValueError("from_self inference case was not found")
  return [match.group(1), match.group(2)]


def _command_facts(hwm_cc: str, hwm_h: str) -> dict:
  request = _function_body(hwm_cc, "GazeboHardwareModule::RequestIconTick(")
  read = _function_body(hwm_cc, "GazeboHardwareModule::ReadStatus(")
  apply_cmd = _function_body(hwm_cc, "GazeboHardwareModule::ApplyCommand(")
  wait = _function_body(
      hwm_cc, "GazeboHardwareModule::WaitForIconTicksToFinish("
  )
  prepare = _function_body(hwm_cc, "GazeboHardwareModule::Prepare(")
  activate = _function_body(hwm_cc, "GazeboHardwareModule::Activate(")
  timeout = re.search(
      r"kClockQueueTimeout\s*=\s*absl::Seconds\((\d+)\)", hwm_cc
  )
  if timeout is None:
    raise ValueError("clock queue timeout was not found")
  if "std::atomic_bool active_ = false" not in hwm_h:
    raise ValueError("module does not start inactive")
  if "num_expected_read_status_calls_" not in hwm_h or "= 0;" not in hwm_h:
    raise ValueError("expected read-status counter does not start at zero")
  increment_at = request.find("++num_expected_read_status_calls_")
  inactive_at = request.find("if (!active_)")
  if increment_at < 0 or inactive_at < 0 or inactive_at > increment_at:
    raise ValueError("inactive tick no-op was not found before the increment")
  if "--num_expected_read_status_calls_" not in read:
    raise ValueError("ReadStatus does not consume an expected call")
  if "AwaitWithTimeout" not in read:
    raise ValueError("ReadStatus does not wait for a tick")
  if "return icon::OkStatus();" not in apply_cmd:
    raise ValueError("ApplyCommand does not return OK")
  if "NoMoreExpectedReadStatusCallsOrShutdown" not in wait:
    raise ValueError("WaitForIconTicksToFinish does not wait for reads")
  if "StateCode::kPrepared" not in prepare:
    raise ValueError("Prepare does not enter kPrepared")
  if "active_ = true" not in activate:
    raise ValueError("Activate does not set active")
  queues_without_clock = False
  clock_at = request.find("if (realtime_clock_ != nullptr)")
  push_at = request.find("sim_time_queue_.push")
  if clock_at < 0 or push_at < 0 or push_at < clock_at:
    raise ValueError("sim-time queue is not guarded by the realtime clock")
  return {
      "apply_blocks": "AwaitWithTimeout" in apply_cmd,
      "clock_queue_timeout_seconds": int(timeout.group(1)),
      "queues_sim_time_without_realtime_clock": queues_without_clock,
      "read_blocks_until_tick": True,
      "request_increments": True,
  }


def _simulate_command_status(facts: dict, interface_count: int) -> dict:
  """Replay the null-clock path from the extracted source facts."""
  expected = 0
  active = False
  steps = [
      {
          "active_after": active,
          "call": "Init",
          "expected_read_status_calls": expected,
          "interfaces_registered": interface_count,
          "status": "OK",
      },
      {
          "active_after": active,
          "call": "Prepare",
          "expected_read_status_calls": expected,
          "state": "kPrepared",
          "status": "OK",
      },
  ]
  # RequestIconTick before Activate returns without arming ReadStatus.
  steps.append(
      {
          "active_after": active,
          "call": "RequestIconTick",
          "expected_read_status_calls": expected,
          "inactive_noop": True,
          "status": "OK",
      }
  )
  active = True
  steps.append(
      {
          "active_after": active,
          "call": "Activate",
          "expected_read_status_calls": expected,
          "status": "OK",
      }
  )
  if facts["request_increments"]:
    expected += 1
  steps.append(
      {
          "active_after": active,
          "call": "RequestIconTick",
          "expected_read_status_calls": expected,
          "status": "OK",
      }
  )
  if expected > 0:
    expected -= 1
  steps.append(
      {
          "active_after": active,
          "call": "ReadStatus",
          "expected_read_status_calls": expected,
          "status": "OK",
      }
  )
  steps.append(
      {
          "active_after": active,
          "blocks": facts["apply_blocks"],
          "call": "ApplyCommand",
          "expected_read_status_calls": expected,
          "status": "OK",
      }
  )
  wait_status = "OK" if expected <= 0 else "BLOCKED"
  steps.append(
      {
          "active_after": active,
          "call": "WaitForIconTicksToFinish",
          "expected_read_status_calls": expected,
          "status": wait_status,
      }
  )
  return {
      "apply_command_blocks": facts["apply_blocks"],
      "clock_queue_timeout_seconds": facts["clock_queue_timeout_seconds"],
      "queues_sim_time_without_realtime_clock": facts[
          "queues_sim_time_without_realtime_clock"
      ],
      "read_status_blocks_until_tick": facts["read_blocks_until_tick"],
      "realtime_clock": None,
      "steps": steps,
  }


def _scan_forbidden(text: str) -> list[str]:
  lowered = text.lower()
  return [token for token in FORBIDDEN_TOKENS if token in lowered]


def assert_no_vehicle_feature(text: str) -> None:
  found = _scan_forbidden(text)
  if found:
    raise AssertionError(
        "gazebo hardware-module fixture must not reference vehicle "
        "features: " + ", ".join(found)
    )


def _assert_launched_documents_clean(documents: list[str]) -> None:
  for document in documents:
    found = _scan_forbidden(document)
    if found:
      raise AssertionError(
          "refusing to launch a fixture that names a forbidden plugin or "
          "component: " + ", ".join(found)
      )


def launch_headless(world_text: str | None = None) -> dict:
  """Load the fixture and existing assets without a display or simulator."""
  if world_text is None:
    world_text = _read(_WORLD)
  template_text = _read(_TEMPLATE)
  _assert_launched_documents_clean([world_text, template_text])
  fixture_plugins = _plugins(world_text, "fixture")
  template_plugins = _plugins(template_text, "world_template")
  alias = _plugin_alias(_read(_REGISTER_CC))
  loaded_names = {plugin["name"] for plugin in fixture_plugins}
  if alias not in loaded_names:
    raise AssertionError(
        "fixture does not load the registered hardware-module launcher "
        f"plugin {alias}"
    )
  root = ET.fromstring(world_text)
  include = root.find(".//include/uri")
  if include is None or not (include.text or "").strip():
    raise ValueError("fixture does not include a model uri")
  model_uri = include.text.strip()
  _uri, model_path = _resolve_model(model_uri)
  model_text = _read(model_path)
  _assert_launched_documents_clean([model_text])
  model_element = root.find("world/model")
  if model_element is None:
    model_element = root.find(".//model")
  model_name = (
      "" if model_element is None else model_element.attrib.get("name", "")
  )
  server = _server_launch(_read(_SERVER_CC))
  counted = _counted_joint_types(_read(_INFER_CC))
  joints = _joints(model_path, counted)
  sensors = _sensor_counts(model_text)
  if sensors["force_torque"] or sensors["rangefinder"]:
    raise ValueError("this manipulator asset is joint-only")
  return {
      "counted_joint_types": counted,
      "fixture": _WORLD.relative_to(_REPO).as_posix(),
      "headless_rendering": server["headless_rendering"],
      "joints": joints,
      "model_file": model_path.relative_to(_REPO).as_posix(),
      "model_name": model_name,
      "model_uri": model_uri,
      "physics_engine": server["physics_engine"],
      "physics_engine_when_use_dart": server["physics_engine_when_use_dart"],
      "plugins": template_plugins + fixture_plugins,
      "rendering": "headless",
      "sensors": sensors,
      "world_template": _TEMPLATE.relative_to(_REPO).as_posix(),
  }


def _discover_interfaces(launch: dict) -> list[dict]:
  names = _constexpr_strings(_read(_INFER_H))
  inferred = _inferred_interfaces(_read(_INFER_CC), names)
  registry = _case_registry(_read(_DATA_CC))
  advertise = _advertise_by_type(_read(_HWM_CC))
  rows = []
  for item in inferred:
    data_type = registry[item["field"]]
    registered = advertise[data_type]
    if registered["strict"] != item["strict"]:
      raise ValueError(
          f"{item['name']} strict flag does not match the advertise call"
      )
    rows.append(
        {
            "advertise": registered["advertise"],
            "direction": registered["direction"],
            "flatbuffer": registered["flatbuffer"],
            "joint_group": item["joint_group"],
            "name": item["name"],
            "strict": item["strict"],
            "tracks_joint_count": registered["tracks_joint_count"],
        }
    )
  safety = _safety_interface(_read(_HWM_CC))
  rows.append(safety)
  return rows


def build_record(world_text: str | None = None) -> dict:
  """Build the hardware-module path record from the current sources."""
  launch = launch_headless(world_text)
  interfaces = _discover_interfaces(launch)
  facts = _command_facts(_read(_HWM_CC), _read(_HWM_H))
  cases = _shared_inference_cases(_read(_INFER_CC))
  group_names = {row["joint_group"] for row in interfaces if row["joint_group"]}
  if group_names != {"auto_joint_group"}:
    raise ValueError(f"unexpected joint groups: {sorted(group_names)}")
  return {
      "ci_runtime_budget_seconds": CI_RUNTIME_BUDGET_SECONDS,
      "command_status": _simulate_command_status(facts, len(interfaces)),
      "discovery": {
          "counted_joint_types": launch["counted_joint_types"],
          "hardware_interfaces_case": cases[0],
          "inference_branch": "from_self",
          "interfaces": interfaces,
          "joint_group": "auto_joint_group",
          "safety_interface": "safety_status",
          "shared_cases": cases,
      },
      "launch": {
          "fixture": launch["fixture"],
          "headless_rendering": launch["headless_rendering"],
          "model_file": launch["model_file"],
          "model_name": launch["model_name"],
          "model_uri": launch["model_uri"],
          "physics_engine": launch["physics_engine"],
          "physics_engine_when_use_dart": launch[
              "physics_engine_when_use_dart"
          ],
          "plugins": launch["plugins"],
          "rendering": launch["rendering"],
          "sensors": launch["sensors"],
          "world_template": launch["world_template"],
      },
      "part": {
          "joint_count": len(launch["joints"]),
          "joint_names": [joint["name"] for joint in launch["joints"]],
          "joint_types": [joint["type"] for joint in launch["joints"]],
          "name": launch["model_name"],
      },
      "schema": SCHEMA,
  }


def render_golden(world_text: str | None = None) -> str:
  return (
      json.dumps(
          build_record(world_text),
          ensure_ascii=True,
          indent=2,
          sort_keys=True,
      )
      + "\n"
  )


def _interface_key(row: dict) -> tuple:
  return (
      row.get("advertise"),
      row.get("direction"),
      row.get("flatbuffer"),
      row.get("joint_group"),
      row.get("name"),
      row.get("strict"),
      row.get("tracks_joint_count"),
  )


def compatibility_report(baseline: dict, candidate: dict) -> list[str]:
  """Name launch, discovery, and command/status differences."""
  lines: list[str] = []
  base_launch = baseline.get("launch", {})
  cand_launch = candidate.get("launch", {})
  if base_launch.get("headless_rendering") != cand_launch.get(
      "headless_rendering"
  ):
    lines.append("changed headless launch")
  if base_launch.get("plugins") != cand_launch.get("plugins"):
    lines.append("changed loaded plugins")
  if base_launch.get("model_uri") != cand_launch.get("model_uri"):
    lines.append("changed model asset")
  if baseline.get("part") != candidate.get("part"):
    lines.append("changed part joints")

  def by_name(record: dict) -> dict[str, dict]:
    interfaces = record.get("discovery", {}).get("interfaces", [])
    return {row["name"]: row for row in interfaces}

  base_rows = by_name(baseline)
  cand_rows = by_name(candidate)
  for name in sorted(set(base_rows) - set(cand_rows)):
    lines.append(f"removed interface {name}")
  for name in sorted(set(cand_rows) - set(base_rows)):
    lines.append(f"added interface {name}")
  for name in sorted(set(base_rows) & set(cand_rows)):
    if _interface_key(base_rows[name]) != _interface_key(cand_rows[name]):
      lines.append(f"changed interface {name}")

  def steps(record: dict) -> list[dict]:
    return record.get("command_status", {}).get("steps", [])

  base_steps = steps(baseline)
  cand_steps = steps(candidate)
  if len(base_steps) != len(cand_steps):
    lines.append("changed command/status step count")
  for base_step, cand_step in zip(base_steps, cand_steps):
    call = base_step.get("call", cand_step.get("call"))
    if base_step.get("status") != cand_step.get("status"):
      lines.append(f"changed command/status status for {call}")
    if base_step.get("expected_read_status_calls") != cand_step.get(
        "expected_read_status_calls"
    ):
      lines.append(f"changed command/status count for {call}")
    if base_step.get("call") != cand_step.get("call"):
      lines.append(f"changed command/status step {call}")
  if baseline.get("command_status", {}).get(
      "read_status_blocks_until_tick"
  ) != candidate.get("command_status", {}).get("read_status_blocks_until_tick"):
    lines.append("changed read-status blocking")
  if baseline.get("command_status", {}).get(
      "apply_command_blocks"
  ) != candidate.get("command_status", {}).get("apply_command_blocks"):
    lines.append("changed apply-command blocking")
  return lines


class GazeboHwmPathGoldenTest(unittest.TestCase):
  """Checks the checked-in Gazebo hardware-module path golden."""

  def test_golden_matches_current_path(self) -> None:
    started = time.perf_counter()
    first = render_golden()
    second = render_golden()
    elapsed = time.perf_counter() - started
    self.assertLess(elapsed, CI_RUNTIME_BUDGET_SECONDS)
    self.assertEqual(first, second)
    assert_no_vehicle_feature(first)
    assert_no_vehicle_feature(_read(_WORLD))
    expected = _GOLDEN.read_text(encoding="utf-8")
    if expected != first:
      report = compatibility_report(json.loads(expected), json.loads(first))
      diff = "\n".join(
          difflib.unified_diff(
              expected.splitlines(),
              first.splitlines(),
              fromfile=str(_GOLDEN.relative_to(_REPO)),
              tofile="current gazebo hardware-module path",
              lineterm="",
          )
      )
      self.fail(
          "gazebo hardware-module path golden does not match the current "
          "path.\nUpdating the fixture requires an explicit flag and senior "
          "review:\n"
          "  python3 intrinsic/simulation/gazebo/plugins/"
          "gazebo_hwm_path_golden_test.py --update-golden\n"
          + "\n".join(report)
          + "\n"
          + diff
      )

  def test_compare_does_not_rewrite_golden(self) -> None:
    before = _GOLDEN.read_bytes()
    world_before = _WORLD.read_bytes()
    render_golden()
    self.assertEqual(before, _GOLDEN.read_bytes())
    self.assertEqual(world_before, _WORLD.read_bytes())

  def test_launch_is_headless_and_uses_existing_assets(self) -> None:
    record = build_record()
    launch = record["launch"]
    self.assertTrue(launch["headless_rendering"])
    self.assertEqual(launch["rendering"], "headless")
    self.assertEqual(launch["model_name"], "ur5e")
    self.assertEqual(
        launch["model_uri"], "model://intrinsic/robot_definitions/ur/ur5e"
    )
    self.assertTrue((_REPO / launch["model_file"]).is_file())
    self.assertTrue((_REPO / launch["world_template"]).is_file())
    self.assertTrue((_REPO / launch["fixture"]).is_file())
    self.assertEqual(record["part"]["joint_count"], 6)
    self.assertEqual(
        record["part"]["joint_names"],
        [
            "shoulder_pan_joint",
            "shoulder_lift_joint",
            "elbow_joint",
            "wrist_1_joint",
            "wrist_2_joint",
            "wrist_3_joint",
        ],
    )
    plugin_names = [plugin["name"] for plugin in launch["plugins"]]
    self.assertIn("intrinsic::simulation::HardwareModuleLauncher", plugin_names)
    self.assertIn("gz::sim::systems::Physics", plugin_names)

  def test_discovery_and_command_status(self) -> None:
    record = build_record()
    self.assertEqual(record["schema"], SCHEMA)
    interfaces = record["discovery"]["interfaces"]
    by_name = {row["name"]: row for row in interfaces}
    self.assertIn("safety_status", by_name)
    self.assertEqual(by_name["safety_status"]["direction"], "status")
    self.assertEqual(by_name["joint_position_command"]["direction"], "command")
    self.assertEqual(
        by_name["joint_position_command"]["advertise"], "AdvertiseInterface"
    )
    self.assertFalse(by_name["joint_position_command"]["strict"])
    self.assertTrue(by_name["joint_position_command"]["tracks_joint_count"])
    self.assertEqual(by_name["joint_position_state"]["direction"], "status")
    self.assertEqual(
        by_name["joint_position_state"]["advertise"],
        "AdvertiseMutableInterface",
    )
    self.assertEqual(
        by_name["joint_position_command"]["joint_group"], "auto_joint_group"
    )
    self.assertEqual(len(interfaces), 12)
    self.assertEqual(
        record["discovery"]["hardware_interfaces_case"],
        "HARDWARE_INTERFACES_NOT_SET",
    )
    self.assertEqual(record["discovery"]["inference_branch"], "from_self")
    command = record["command_status"]
    self.assertIsNone(command["realtime_clock"])
    self.assertTrue(command["read_status_blocks_until_tick"])
    self.assertFalse(command["apply_command_blocks"])
    self.assertFalse(command["queues_sim_time_without_realtime_clock"])
    calls = [step["call"] for step in command["steps"]]
    self.assertEqual(
        calls,
        [
            "Init",
            "Prepare",
            "RequestIconTick",
            "Activate",
            "RequestIconTick",
            "ReadStatus",
            "ApplyCommand",
            "WaitForIconTicksToFinish",
        ],
    )
    self.assertTrue(command["steps"][2]["inactive_noop"])
    self.assertEqual(command["steps"][2]["expected_read_status_calls"], 0)
    self.assertEqual(command["steps"][4]["expected_read_status_calls"], 1)
    self.assertEqual(command["steps"][5]["expected_read_status_calls"], 0)
    self.assertEqual(command["steps"][-1]["status"], "OK")
    self.assertEqual(command["steps"][0]["interfaces_registered"], 12)

  def test_no_forbidden_plugin_or_component(self) -> None:
    launched = launch_headless()
    blob = json.dumps(launched)
    assert_no_vehicle_feature(blob)
    assert_no_vehicle_feature(_read(_WORLD))
    for plugin in launched["plugins"]:
      assert_no_vehicle_feature(plugin["name"])
      assert_no_vehicle_feature(plugin["filename"])
    self.assertEqual(launched["sensors"]["force_torque"], 0)
    self.assertEqual(launched["sensors"]["rangefinder"], 0)

  def test_runtime_budget_is_documented(self) -> None:
    readme = _read(_DIR / "README.md")
    self.assertIn("5 seconds", readme)
    self.assertIn(str(int(CI_RUNTIME_BUDGET_SECONDS)), readme)
    record = json.loads(_GOLDEN.read_text(encoding="utf-8"))
    self.assertEqual(
        record["ci_runtime_budget_seconds"], CI_RUNTIME_BUDGET_SECONDS
    )
    started = time.perf_counter()
    render_golden()
    self.assertLess(time.perf_counter() - started, CI_RUNTIME_BUDGET_SECONDS)

  def test_protected_gazebo_hwm_target_remains(self) -> None:
    text = _read(_BASELINE)
    self.assertIn(
        "//intrinsic/simulation/gazebo/plugins:gazebo_hwm\t"
        "gazebo_hwm\tbuild\t",
        text,
    )
    self.assertIn(
        "//intrinsic/simulation/gazebo/plugins:gazebo_hwm_test\t"
        "gazebo_hwm\ttest\t",
        text,
    )

  def test_changed_interface_is_reported(self) -> None:
    record = build_record()
    mutated = json.loads(json.dumps(record))
    mutated["discovery"]["interfaces"][0]["direction"] = "status"
    mutated["discovery"]["interfaces"].pop()
    report = compatibility_report(record, mutated)
    changed = mutated["discovery"]["interfaces"][0]["name"]
    self.assertIn(f"changed interface {changed}", report)
    self.assertTrue(
        any(line.startswith("removed interface ") for line in report)
    )

  def test_changed_status_is_reported(self) -> None:
    record = build_record()
    mutated = json.loads(json.dumps(record))
    mutated["command_status"]["steps"][-1]["status"] = "INTERNAL"
    mutated["command_status"]["steps"][4]["expected_read_status_calls"] = 9
    mutated["command_status"]["read_status_blocks_until_tick"] = False
    mutated["launch"]["headless_rendering"] = False
    report = compatibility_report(record, mutated)
    self.assertTrue(
        any("changed command/status status" in line for line in report)
    )
    self.assertTrue(
        any("changed command/status count" in line for line in report)
    )
    self.assertIn("changed read-status blocking", report)
    self.assertIn("changed headless launch", report)

  def test_changed_plugin_is_reported(self) -> None:
    record = build_record()
    mutated = json.loads(json.dumps(record))
    mutated["launch"]["plugins"].append(
        {
            "filename": "static://example::NotRegistered",
            "name": "example::NotRegistered",
            "source": "fixture",
        }
    )
    mutated["part"]["joint_names"] = []
    report = compatibility_report(record, mutated)
    self.assertIn("changed loaded plugins", report)
    self.assertIn("changed part joints", report)

  def test_update_command_is_documented(self) -> None:
    readme = _read(_DIR / "README.md")
    self.assertIn("--update-golden", readme)
    self.assertIn("gazebo_hwm_path_golden_test.py", readme)

  def test_update_requires_explicit_flag(self) -> None:
    before = _GOLDEN.read_bytes()
    with contextlib.redirect_stderr(io.StringIO()):
      with self.assertRaises(SystemExit) as raised:
        main([])
    self.assertEqual(raised.exception.code, 2)
    self.assertEqual(before, _GOLDEN.read_bytes())

  def test_update_refuses_vehicle_feature(self) -> None:
    before = _GOLDEN.read_bytes()
    world_before = _WORLD.read_bytes()
    with self.assertRaises(AssertionError):
      update_golden(text='{"plugin": "thruster_plugin"}\n')
    extra = _read(_WORLD).replace(
        "</world>",
        '<plugin filename="static://example::thruster_plugin" '
        'name="example::thruster_plugin"/>\n  </world>',
    )
    with self.assertRaises(AssertionError):
      render_golden(extra)
    self.assertEqual(before, _GOLDEN.read_bytes())
    self.assertEqual(world_before, _WORLD.read_bytes())


def update_golden(path: Path = _GOLDEN, text: str | None = None) -> None:
  if text is None:
    text = render_golden()
    if text != render_golden():
      raise RuntimeError("gazebo hardware-module render is not deterministic")
  assert_no_vehicle_feature(text)
  path.parent.mkdir(parents=True, exist_ok=True)
  path.write_text(text, encoding="utf-8")


def main(argv: list[str] | None = None) -> int:
  parser = argparse.ArgumentParser(description=__doc__)
  parser.add_argument(
      "--update-golden",
      action="store_true",
      help=(
          "Rewrite testdata/gazebo_hwm_path.golden.json from the current "
          "headless launch. This flag is required; comparison is the "
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
