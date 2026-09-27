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

"""Build protected manipulator targets from the checked-in manifest.

Reads `.github/baseline/manipulator_targets.tsv` and builds every row whose
`build_vs_test` column is `build`. The Bazel invocation does not pass
`--config` or any other flag that would enable a UUV configuration. On
failure, stderr ends with `First failing target: <label>`.
"""

from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import re
import subprocess
import sys
import tempfile

HEADER = "label\tsubsystem\tbuild_vs_test\towner\n"
DEFAULT_MANIFEST = Path(".github/baseline/manipulator_targets.tsv")
_LFS_POINTER_PREFIX = b"version https://git-lfs.github.com/spec/v1"
# Pointers are about 130 bytes. These meshes are much larger once smudged.
_MIN_SAMPLED_MESH_BYTES = 1024
# CI samples these paths after `git lfs pull`. The ABB glb is not under
# third_party/; robot-definition meshes are included in the same pull.
SAMPLED_LFS_MESHES = (
    "third_party/kuka/kr10_r1100_2/visual/base_link.dae",
    "third_party/kuka/kr16_r2010_2/visual/base.stl",
    "intrinsic_control/intrinsic/models/robot_definitions/abb/"
    "irb1300_10_115/visual/irb1300_visual_1.glb",
)
LFS_PULL_INCLUDE = (
    "third_party/**,intrinsic_control/intrinsic/models/robot_definitions/**"
)
BUILD_CLASS = "build"
TEST_CLASS = "test"
_FAILURE_ABORT_REASONS = frozenset(
    {
        "ANALYSIS_FAILURE",
        "LOADING_FAILURE",
    }
)
_LOG_FAILURE_PATTERNS = (
    re.compile(r"^Target (//\S+) failed to build$", re.MULTILINE),
    re.compile(r"Analysis of target '(//[^']+)' failed"),
    re.compile(r"\(from target (//[^)\s]+)\)"),
    re.compile(r"no such target '(//[^']+)'"),
    re.compile(r"Skipping '(//[^']+)': (?:no such package|no such target)"),
)


def repo_root() -> Path:
  return Path(__file__).resolve().parents[1]


def sampled_lfs_pointer_errors(root: Path) -> list[str]:
  """Return problems for sampled meshes that are still LFS pointers."""
  errors: list[str] = []
  for relative in SAMPLED_LFS_MESHES:
    path = root / relative
    if not path.is_file():
      errors.append(f"{relative} is missing")
      continue
    data = path.read_bytes()
    if (
        data.startswith(_LFS_POINTER_PREFIX)
        or len(data) < _MIN_SAMPLED_MESH_BYTES
    ):
      errors.append(
          f"{relative} is a Git LFS pointer or too small ({len(data)} bytes)"
      )
  return errors


def require_sampled_meshes(root: Path) -> None:
  """Fail with a `git lfs pull` hint when sampled meshes are pointers."""
  errors = sampled_lfs_pointer_errors(root)
  if not errors:
    print(
        "Checked"
        f" {len(SAMPLED_LFS_MESHES)} manipulator meshes are not Git LFS"
        " pointers"
    )
    return
  print("Manipulator meshes are still Git LFS pointers.", file=sys.stderr)
  print("From the repository root, run:", file=sys.stderr)
  print(
      f'  git lfs pull --include="{LFS_PULL_INCLUDE}"',
      file=sys.stderr,
  )
  print("Unset GIT_LFS_SKIP_SMUDGE if it is set.", file=sys.stderr)
  for item in errors:
    print(f"  {item}", file=sys.stderr)
  raise SystemExit(1)


def load_build_labels(path: Path) -> list[str]:
  """Return `build` labels in manifest order."""
  text = path.read_text(encoding="utf-8")
  if "\r" in text:
    raise SystemExit(f"{path} must use LF line endings")
  if not text.startswith(HEADER):
    raise SystemExit(f"{path} is missing the expected header")
  if not text.endswith("\n"):
    raise SystemExit(f"{path} must end with a newline")
  labels: list[str] = []
  seen: set[str] = set()
  for line_number, line in enumerate(text.splitlines()[1:], start=2):
    fields = line.split("\t")
    if len(fields) != 4 or any(field == "" for field in fields):
      raise SystemExit(f"{path}:{line_number}: expected 4 columns")
    label, _subsystem, build_vs_test, _owner = fields
    if build_vs_test not in (BUILD_CLASS, TEST_CLASS):
      raise SystemExit(
          f"{path}:{line_number}: build_vs_test must be "
          f"'{BUILD_CLASS}' or '{TEST_CLASS}'"
      )
    if not label.startswith("//") or ":" not in label or " " in label:
      raise SystemExit(f"{path}:{line_number}: invalid label {label!r}")
    if label in seen:
      raise SystemExit(f"{path}:{line_number}: duplicate label {label}")
    seen.add(label)
    if build_vs_test == BUILD_CLASS:
      labels.append(label)
  if not labels:
    raise SystemExit(f"{path} has no {BUILD_CLASS} rows")
  return labels


def _event_labels(event: dict) -> list[str]:
  ident = event.get("id")
  if not isinstance(ident, dict):
    return []
  labels: list[str] = []
  for key in ("targetCompleted", "actionCompleted"):
    body = ident.get(key)
    if isinstance(body, dict) and isinstance(body.get("label"), str):
      labels.append(body["label"])
  pattern = ident.get("pattern")
  if isinstance(pattern, dict) and isinstance(pattern.get("pattern"), list):
    for item in pattern["pattern"]:
      if (
          isinstance(item, str)
          and item.startswith("//")
          and ":" in item
          and " " not in item
      ):
        labels.append(item)
  return labels


def _event_failed(event: dict) -> bool:
  completed = event.get("completed")
  if isinstance(completed, dict) and completed.get("success") is False:
    return True
  action = event.get("action")
  if isinstance(action, dict) and action.get("success") is False:
    return True
  aborted = event.get("aborted")
  if (
      isinstance(aborted, dict)
      and aborted.get("reason") in _FAILURE_ABORT_REASONS
  ):
    return True
  return False


def failed_labels_from_bep(text: str) -> list[str]:
  """Return labels Bazel marked failed in build-event JSON order."""
  ordered: list[str] = []
  seen: set[str] = set()
  for line in text.splitlines():
    stripped = line.strip()
    if not stripped:
      continue
    try:
      event = json.loads(stripped)
    except json.JSONDecodeError:
      continue
    if not isinstance(event, dict) or not _event_failed(event):
      continue
    for label in _event_labels(event):
      if label not in seen:
        seen.add(label)
        ordered.append(label)
  return ordered


def failed_labels_from_log(text: str) -> list[str]:
  """Return labels named by Bazel's plain-text failure lines."""
  ordered: list[str] = []
  seen: set[str] = set()
  for pattern in _LOG_FAILURE_PATTERNS:
    for match in pattern.finditer(text):
      label = match.group(1)
      if label not in seen:
        seen.add(label)
        ordered.append(label)
  return ordered


def first_failing_target(
    manifest_labels: list[str], failed: list[str]
) -> str | None:
  """Pick the earliest manifest label that failed, else the first failure."""
  if not failed:
    return None
  failed_set = set(failed)
  for label in manifest_labels:
    if label in failed_set:
      return label
  return failed[0]


def bazel_build_command(
    bazel: str, pattern_file: Path, bep_file: Path
) -> list[str]:
  """Build command. Does not pass `--config` or UUV flags."""
  return [
      bazel,
      "build",
      f"--target_pattern_file={pattern_file}",
      f"--build_event_json_file={bep_file}",
  ]


def _stream_process(cmd: list[str], cwd: Path) -> tuple[int, str]:
  process = subprocess.Popen(
      cmd,
      cwd=cwd,
      stdout=subprocess.PIPE,
      stderr=subprocess.STDOUT,
      text=True,
  )
  chunks: list[str] = []
  assert process.stdout is not None
  for line in process.stdout:
    sys.stdout.write(line)
    sys.stdout.flush()
    chunks.append(line)
  returncode = process.wait()
  if returncode < 0:
    returncode = 1
  return returncode, "".join(chunks)


def build_labels(root: Path, bazel: str, labels: list[str]) -> int:
  with tempfile.TemporaryDirectory(
      prefix="protected-manipulator-build-"
  ) as tmp:
    tmp_dir = Path(tmp)
    pattern_file = tmp_dir / "targets.txt"
    bep_file = tmp_dir / "bep.json"
    pattern_file.write_text(
        "".join(f"{label}\n" for label in labels), encoding="utf-8"
    )
    cmd = bazel_build_command(bazel, pattern_file, bep_file)
    try:
      returncode, log_text = _stream_process(cmd, root)
    except FileNotFoundError:
      print(f"Bazel binary not found: {bazel}", file=sys.stderr)
      return 127
    if returncode == 0:
      print(f"Built {len(labels)} protected manipulator targets")
      return 0
    bep_text = (
        bep_file.read_text(encoding="utf-8") if bep_file.is_file() else ""
    )
    failed = failed_labels_from_bep(bep_text)
    for label in failed_labels_from_log(log_text):
      if label not in failed:
        failed.append(label)
    first = first_failing_target(labels, failed)
    if first is None:
      print(
          "Bazel build failed before a failing target could be identified.",
          file=sys.stderr,
      )
    else:
      print(f"First failing target: {first}", file=sys.stderr)
    return returncode


def main() -> None:
  parser = argparse.ArgumentParser(description=__doc__)
  parser.add_argument(
      "--bazel",
      default=os.environ.get("BAZEL", "bazel"),
      help="Bazel binary. Defaults to $BAZEL or 'bazel'.",
  )
  parser.add_argument(
      "--manifest",
      type=Path,
      default=DEFAULT_MANIFEST,
      help="Checked-in target manifest. Defaults to the baseline TSV.",
  )
  parser.add_argument(
      "--dry-run",
      action="store_true",
      help="Print the build count without running Bazel.",
  )
  parser.add_argument(
      "--check-lfs-meshes",
      action="store_true",
      help="Fail if sampled manipulator meshes are still Git LFS pointers.",
  )
  args = parser.parse_args()
  root = repo_root()
  if args.check_lfs_meshes:
    require_sampled_meshes(root)
    return
  manifest = args.manifest
  if not manifest.is_absolute():
    manifest = root / manifest
  labels = load_build_labels(manifest)
  display = manifest
  try:
    display = manifest.relative_to(root)
  except ValueError:
    display = manifest
  if args.dry_run:
    print(
        f"Would build {len(labels)} protected manipulator targets"
        f" from {display}"
    )
    return
  require_sampled_meshes(root)
  print(f"Building {len(labels)} protected manipulator targets from {display}")
  raise SystemExit(build_labels(root, args.bazel, labels))


if __name__ == "__main__":
  main()
