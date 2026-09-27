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

"""Write the protected manipulator Bazel target manifest.

Runs a fixed `bazel query` and classifies existing targets. The script only
writes the manifest path it is given. It does not edit BUILD files.
"""

from __future__ import annotations

import argparse
import os
from pathlib import Path
import re
import subprocess
import sys
import tempfile

# Packages queried with `//pkg/...` or `//pkg:*`. Order is documentation only;
# the manifest is sorted after classification.
QUERY_SCOPES = (
    "//intrinsic_kinematics/...",
    "//intrinsic_motion_planning/...",
    "//intrinsic/manipulation/...",
    "//intrinsic_control/intrinsic/icon/actions/...",
    "//intrinsic_control/intrinsic/icon/control/algorithms/...",
    "//intrinsic_control/intrinsic/icon/control/collision/...",
    "//intrinsic_control/intrinsic/icon/control/parts/...",
    "//intrinsic_control/intrinsic/icon/control/primitives/...",
    "//intrinsic_control/intrinsic/icon/control:*",
    "//intrinsic_control/intrinsic/icon/hal/interfaces:*",
    "//intrinsic_control/intrinsic/icon/hardware_modules/...",
    "//intrinsic_control/intrinsic/icon/reflexxes/...",
    "//intrinsic_control/intrinsic/models/robot_definitions/...",
    "//intrinsic/simulation/gazebo/plugins/...",
    "//third_party/abb_egm/...",
    "//third_party/abb_hardware_module/...",
)

# Longer prefixes must precede the packages that contain them.
SUBSYSTEM_PREFIXES = (
    ("//intrinsic_kinematics/", "kinematics"),
    ("//intrinsic_motion_planning/", "motion_planning"),
    ("//intrinsic/manipulation/", "manipulation_skills"),
    (
        "//intrinsic_control/intrinsic/icon/control/parts/hal/arm_part",
        "icon_arm_part",
    ),
    (
        "//intrinsic_control/intrinsic/icon/control/parts/hal/"
        "linear_gripper_part",
        "icon_gripper",
    ),
    (
        "//intrinsic_control/intrinsic/icon/control/parts/hal/"
        "force_torque_sensor_part",
        "icon_force_torque",
    ),
    (
        "//intrinsic_control/intrinsic/icon/control/parts/feature_interfaces",
        "icon_feature_interfaces",
    ),
    ("//intrinsic_control/intrinsic/icon/control/parts", "icon_part_registry"),
    ("//intrinsic_control/intrinsic/icon/actions", "icon_actions"),
    (
        "//intrinsic_control/intrinsic/icon/control/primitives",
        "icon_control_primitives",
    ),
    (
        "//intrinsic_control/intrinsic/icon/control/algorithms",
        "icon_control_algorithms",
    ),
    (
        "//intrinsic_control/intrinsic/icon/control/collision",
        "icon_collision",
    ),
    ("//intrinsic_control/intrinsic/icon/control:", "icon_joint_commands"),
    (
        "//intrinsic_control/intrinsic/icon/hal/interfaces",
        "icon_hal_interfaces",
    ),
    ("//intrinsic_control/intrinsic/icon/reflexxes", "icon_reflexxes"),
    (
        "//intrinsic_control/intrinsic/icon/hardware_modules/gazebo_hwm_stub",
        "gazebo_hwm_stub",
    ),
    (
        "//intrinsic_control/intrinsic/icon/hardware_modules/",
        "icon_hardware_module",
    ),
    ("//intrinsic/simulation/gazebo/plugins/grippers", "gazebo_gripper"),
    ("//intrinsic/simulation/gazebo/plugins", "gazebo_hwm"),
    (
        "//intrinsic_control/intrinsic/models/robot_definitions/",
        "robot_definitions",
    ),
    ("//third_party/abb_hardware_module/", "abb_hardware_module"),
    ("//third_party/abb_egm", "abb_egm"),
)

EXCLUDED_PACKAGE_PREFIXES = (
    "//intrinsic_control/intrinsic/icon/control/parts/hal/adio_part",
    "//intrinsic_control/intrinsic/icon/control/parts/hal/imu_part",
    "//intrinsic_control/intrinsic/icon/control/parts/hal/laser_tracker_part",
    "//intrinsic_control/intrinsic/icon/control/parts/hal/rangefinder_part",
    "//intrinsic/simulation/gazebo/plugins/cameras",
    "//intrinsic/simulation/gazebo/plugins/generic_action",
    "//intrinsic/simulation/gazebo/plugins/gpio",
    "//intrinsic/simulation/gazebo/plugins/sim_inputs",
)

# Exact package -> target names kept. Other targets in the package are dropped.
PACKAGE_NAME_ALLOWLIST = {
    "//intrinsic_control/intrinsic/icon/control": frozenset(
        {
            "joint_acceleration_command",
            "joint_position_command",
        }
    ),
    "//intrinsic_control/intrinsic/icon/control/parts": frozenset(
        {
            "fake_feature_interfaces",
            "feature_interface_registry",
            "feature_interfaces",
            "move_checker",
            "new_manipulator_kinematics",
            "payload_property",
            "plane_checker",
            "realtime_robot_payload",
            "register_default_parts",
        }
    ),
    "//intrinsic/simulation/gazebo/plugins": frozenset(
        {
            "add_hardware_module_launchers_system",
            "aggregated_resource_health_cc_grpc_proto",
            "aggregated_resource_health_cc_proto",
            "aggregated_resource_health_proto",
            "device_util",
            "ecm_hardware_interface_conversion",
            "gazebo_hwm",
            "gazebo_hwm_test",
            "gravity_compensator",
            "hardware_module_launcher",
            "hardware_module_launcher_register",
            "infer_hardware_interfaces",
            "sim_hardware_interface_data",
        }
    ),
}

PACKAGE_NAME_DENYLIST = {
    "//intrinsic_control/intrinsic/icon/actions": frozenset(
        {
            "adio_info",
            "adio_utils",
        }
    ),
    "//intrinsic_control/intrinsic/icon/control/parts/feature_interfaces": (
        frozenset(
            {
                "imu_state",
                "laser_tracker_position_state",
                "rangefinder_state",
            }
        )
    ),
}

# Substrings that mark manipulator joint/Cartesian/gripper interfaces inside
# the mixed HAL FlatBuffers package.
HAL_INTERFACE_PACKAGE = "//intrinsic_control/intrinsic/icon/hal/interfaces"
HAL_INTERFACE_MARKERS = (
    "control_mode",
    "force_sensor",
    "force_torque",
    "gripper",
    "joint_",
    "payload_",
    "robot_controller",
    "robot_payload",
)

BUILD_KINDS = frozenset(
    {
        "_generate_pb2_grpc_src",
        "_intrinsic_skill_rule",
        "_sdf_scene_object",
        "alias",
        "cc_binary",
        "cc_grpc_library",
        "cc_library",
        "cc_proto_library",
        "flatbuffers_library",
        "go_binary",
        "go_library",
        "go_proto_library",
        "hardware_module_manifest",
        "intrinsic_hardware_device",
        "intrinsic_scene_object",
        "intrinsic_service",
        "proto_library",
        "py_binary",
        "py_library",
        "py_proto_library",
        "skill_manifest",
        "xacro_file",
    }
)
TEST_KINDS = frozenset(
    {
        "cc_test",
        "go_test",
        "importpath_matches_go_package_test",
        "py_test",
        "sh_test",
        "test_suite",
    }
)

HEADER = "label\tsubsystem\tbuild_vs_test\towner\n"
MANIFEST_NAME = ".github/baseline/manipulator_targets.tsv"


def repo_root() -> Path:
  return Path(__file__).resolve().parents[1]


def package_of(label: str) -> str:
  return label.split(":", 1)[0]


def target_name(label: str) -> str:
  return label.split(":", 1)[1]


def excluded_package(package: str) -> bool:
  return any(package.startswith(prefix) for prefix in EXCLUDED_PACKAGE_PREFIXES)


def name_selected(package: str, name: str) -> bool:
  if name.startswith("_"):
    return False
  allow = PACKAGE_NAME_ALLOWLIST.get(package)
  if allow is not None and name not in allow:
    return False
  deny = PACKAGE_NAME_DENYLIST.get(package)
  if deny is not None and name in deny:
    return False
  if package == HAL_INTERFACE_PACKAGE:
    return any(marker in name for marker in HAL_INTERFACE_MARKERS)
  return True


def subsystem_for(label: str) -> str:
  for prefix, subsystem in SUBSYSTEM_PREFIXES:
    if label.startswith(prefix):
      return subsystem
  raise ValueError(f"No subsystem rule for {label}")


def classification_for(kind: str) -> str | None:
  if kind in TEST_KINDS:
    return "test"
  if kind in BUILD_KINDS:
    return "build"
  return None


def load_codeowners(root: Path) -> list[tuple[str, tuple[str, ...]]]:
  """Return CODEOWNERS rules. Last matching rule wins, per GitHub."""
  for relative in ("CODEOWNERS", ".github/CODEOWNERS", "docs/CODEOWNERS"):
    path = root / relative
    if path.is_file():
      return _parse_codeowners(path)
  return []


def _parse_codeowners(path: Path) -> list[tuple[str, tuple[str, ...]]]:
  rules: list[tuple[str, tuple[str, ...]]] = []
  for raw in path.read_text(encoding="utf-8").splitlines():
    line = raw.split("#", 1)[0].strip()
    if not line:
      continue
    bits = line.split()
    if len(bits) < 2:
      continue
    rules.append((bits[0], tuple(bits[1:])))
  return rules


def _codeowners_regex(pattern: str) -> re.Pattern[str]:
  anchored = pattern.startswith("/")
  body = pattern[1:] if anchored else pattern
  regex = ["^"] if anchored else "(^|/)"
  i = 0
  while i < len(body):
    if body.startswith("**", i):
      regex.append(".*")
      i += 2
      continue
    char = body[i]
    if char == "*":
      regex.append("[^/]*")
    elif char == "?":
      regex.append("[^/]")
    else:
      regex.append(re.escape(char))
    i += 1
  regex.append("$")
  return re.compile("".join(regex))


def owner_for(package: str, rules: list[tuple[str, tuple[str, ...]]]) -> str:
  if not rules:
    return package
  relative = package[2:] if package.startswith("//") else package
  matched: tuple[str, ...] | None = None
  for pattern, owners in rules:
    if _codeowners_regex(pattern).search(relative):
      matched = owners
  if not matched:
    return package
  return ",".join(matched)


def bazel_query(root: Path, bazel: str, query: str, output: str) -> str:
  with tempfile.NamedTemporaryFile(
      mode="w", encoding="utf-8", suffix=".query", delete=False
  ) as handle:
    handle.write(query)
    if not query.endswith("\n"):
      handle.write("\n")
    query_path = handle.name
  cmd = [
      bazel,
      "query",
      "--noshow_progress",
      "--noshow_loading_progress",
      "--ui_event_filters=-info,-progress,-debug",
      "--order_output=full",
      f"--output={output}",
      f"--query_file={query_path}",
  ]
  try:
    completed = subprocess.run(
        cmd,
        cwd=root,
        check=False,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
    )
  finally:
    os.unlink(query_path)
  if completed.returncode != 0:
    sys.stderr.write(completed.stderr)
    raise SystemExit(completed.returncode)
  if completed.stderr:
    sys.stderr.write(completed.stderr)
  return completed.stdout


def parse_label_kind(text: str) -> list[tuple[str, str]]:
  rows: list[tuple[str, str]] = []
  for line in text.splitlines():
    if not line.strip():
      continue
    parts = line.split()
    if len(parts) < 3 or parts[1] != "rule":
      raise SystemExit(f"Unexpected bazel query line: {line}")
    rows.append((parts[0], parts[2]))
  return rows


def discover(root: Path, bazel: str) -> list[tuple[str, str, str, str]]:
  query = 'kind("rule", ' + " + ".join(QUERY_SCOPES) + ")"
  parsed = parse_label_kind(bazel_query(root, bazel, query, "label_kind"))
  rules = load_codeowners(root)
  skipped: dict[str, int] = {}
  rows: list[tuple[str, str, str, str]] = []
  for kind, label in parsed:
    package = package_of(label)
    name = target_name(label)
    if excluded_package(package) or not name_selected(package, name):
      continue
    build_vs_test = classification_for(kind)
    if build_vs_test is None:
      skipped[kind] = skipped.get(kind, 0) + 1
      continue
    rows.append(
        (label, subsystem_for(label), build_vs_test, owner_for(package, rules))
    )
  rows.sort()
  labels = [row[0] for row in rows]
  if len(labels) != len(set(labels)):
    raise SystemExit("Duplicate labels in inventory")
  if not rows:
    raise SystemExit("Inventory is empty")
  if skipped:
    summary = ", ".join(f"{kind}={skipped[kind]}" for kind in sorted(skipped))
    print(f"Skipped non-product kinds: {summary}", file=sys.stderr)
  print(f"Inventoried {len(rows)} targets", file=sys.stderr)
  return rows


def render(rows: list[tuple[str, str, str, str]]) -> str:
  lines = [HEADER]
  for label, subsystem, build_vs_test, owner in rows:
    lines.append(f"{label}\t{subsystem}\t{build_vs_test}\t{owner}\n")
  return "".join(lines)


def read_manifest(path: Path) -> list[str]:
  text = path.read_text(encoding="utf-8")
  if not text.startswith(HEADER):
    raise SystemExit(f"{path} is missing the expected header")
  if not text.endswith("\n"):
    raise SystemExit(f"{path} must end with a newline")
  labels: list[str] = []
  for line_number, line in enumerate(text.splitlines()[1:], start=2):
    fields = line.split("\t")
    if len(fields) != 4 or any(field == "" for field in fields):
      raise SystemExit(f"{path}:{line_number}: expected 4 columns")
    labels.append(fields[0])
  if labels != sorted(labels):
    raise SystemExit(f"{path} is not sorted by label")
  if len(labels) != len(set(labels)):
    raise SystemExit(f"{path} contains duplicate labels")
  return labels


def verify(root: Path, bazel: str, manifest: Path) -> None:
  labels = read_manifest(manifest)
  query = "set(\n" + "\n".join(labels) + "\n)\n"
  resolved = [
      line.strip()
      for line in bazel_query(root, bazel, query, "label").splitlines()
      if line.strip()
  ]
  missing = sorted(set(labels) - set(resolved))
  extra = sorted(set(resolved) - set(labels))
  if missing or extra:
    if missing:
      print("Labels that did not resolve:", file=sys.stderr)
      for label in missing:
        print(f"  {label}", file=sys.stderr)
    if extra:
      print("Unexpected query results:", file=sys.stderr)
      for label in extra:
        print(f"  {label}", file=sys.stderr)
    raise SystemExit(1)
  print(f"Resolved {len(labels)} labels", file=sys.stderr)


def main() -> None:
  parser = argparse.ArgumentParser(description=__doc__)
  parser.add_argument(
      "--bazel",
      default=os.environ.get("BAZEL", "bazel"),
      help="Bazel binary. Defaults to $BAZEL or 'bazel'.",
  )
  parser.add_argument(
      "--output",
      type=Path,
      default=repo_root() / MANIFEST_NAME,
      help="Manifest path to write.",
  )
  parser.add_argument(
      "--verify",
      type=Path,
      help="Check that every label in this manifest resolves. Does not write.",
  )
  args = parser.parse_args()
  root = repo_root()
  if args.verify is not None:
    verify(root, args.bazel, args.verify)
    return
  text = render(discover(root, args.bazel))
  args.output.parent.mkdir(parents=True, exist_ok=True)
  args.output.write_bytes(text.encode("utf-8"))


if __name__ == "__main__":
  main()
