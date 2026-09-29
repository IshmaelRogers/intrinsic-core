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

"""Dependency, visibility, and timing checks for the vehicle scaffold.

The allowed graph is the scaffold graph plus the dynamics-to-parameters
edge used by the rigid-body mass matrix, the added-mass matrix, the
rigid-body Coriolis matrix, the added-mass Coriolis matrix, the
linear/quadratic damping wrench, the gravity/buoyancy restoring wrench,
and the water-current relative velocity. A later edge needs an update
here and in README.md. Dynamics interface sources include only the C++
standard library and headers in that package. The mass-matrix, Coriolis,
damping, restoring, and relative-velocity terms may include marine model
parameter headers.
"""

import os
import re
import unittest

_PACKAGES = (
    "allocation",
    "control",
    "dynamics",
    "guidance",
    "parameters",
    "state",
    "testing",
    "vehicle",
)

# Sibling packages each BUILD file may name. `vehicle` is the parent.
_ALLOWED = {name: frozenset() for name in _PACKAGES}
_ALLOWED["dynamics"] = frozenset(("parameters",))
_ALLOWED["testing"] = frozenset(
    (
        "allocation",
        "control",
        "dynamics",
        "guidance",
        "parameters",
        "state",
    )
)

_ALLOWED_LABELS = frozenset(
    (
        "//visibility:public",
        "@com_google_googletest//:gtest_main",
    )
)
_VEHICLE_PREFIX = "//intrinsic_vehicle/intrinsic/vehicle"

_HEADER_MARKERS = {
    "allocation.h": "Real-time package scaffold",
    "control.h": "Soft-real-time package scaffold",
    "dynamics.h": "Soft-real-time package scaffold",
    "guidance.h": "Soft-real-time package scaffold",
    "parameters.h": "Soft-real-time parameter package",
    "scaffold.h": "Test-only umbrella",
    "state.h": "Real-time package scaffold",
}

_DYNAMICS_SOURCES = (
    "BUILD",
    "added_mass_coriolis_matrix.cc",
    "added_mass_coriolis_matrix.h",
    "added_mass_matrix.cc",
    "added_mass_matrix.h",
    "dynamics.h",
    "gravity_buoyancy_restoring_wrench.cc",
    "gravity_buoyancy_restoring_wrench.h",
    "linear_quadratic_damping_wrench.cc",
    "linear_quadratic_damping_wrench.h",
    "rigid_body_coriolis_matrix.cc",
    "rigid_body_coriolis_matrix.h",
    "rigid_body_mass_matrix.cc",
    "rigid_body_mass_matrix.h",
    "vehicle_dynamics.cc",
    "vehicle_dynamics.h",
    "water_current_relative_velocity.cc",
    "water_current_relative_velocity.h",
    "zero_force_dynamics.cc",
    "zero_force_dynamics.h",
)

# Mass-matrix, Coriolis, damping, restoring, and relative-velocity sources
# read marine model parameters. Interface sources do not.
_MASS_MATRIX_SOURCES = frozenset(
    (
        "added_mass_coriolis_matrix.cc",
        "added_mass_coriolis_matrix.h",
        "added_mass_matrix.cc",
        "added_mass_matrix.h",
        "gravity_buoyancy_restoring_wrench.cc",
        "gravity_buoyancy_restoring_wrench.h",
        "linear_quadratic_damping_wrench.cc",
        "linear_quadratic_damping_wrench.h",
        "rigid_body_coriolis_matrix.cc",
        "rigid_body_coriolis_matrix.h",
        "rigid_body_mass_matrix.cc",
        "rigid_body_mass_matrix.h",
        "water_current_relative_velocity.cc",
        "water_current_relative_velocity.h",
    )
)

_FORBIDDEN_INCLUDE_TOKENS = (
    "absl",
    "gazebo",
    "grpc",
    "icon",
    "ignition",
    "proto",
    "sdformat",
    "vendor",
)

_LABEL_RE = re.compile(r'"((?:@|//)[^"]+)"')
_PACKAGE_RE = re.compile(
    r"^//intrinsic_vehicle/intrinsic/vehicle(?:/([a-z_]+))?(?::|$)"
)


def _strip_comments(text):
  lines = []
  for line in text.splitlines():
    if "#" in line:
      line = line[: line.index("#")]
    lines.append(line)
  return "\n".join(lines)


def _strip_loads(text):
  lines = []
  skipping = False
  for line in text.splitlines():
    stripped = line.strip()
    if not skipping and stripped.startswith("load("):
      if not stripped.endswith(")"):
        skipping = True
      continue
    if skipping:
      if stripped.endswith(")"):
        skipping = False
      continue
    lines.append(line)
  return "\n".join(lines)


def _include_target(line):
  stripped = line.strip()
  if not stripped.startswith("#include"):
    return None
  rest = stripped[len("#include") :].strip()
  if len(rest) >= 2 and rest[0] in "<\"" and rest[-1] in ">\"":
    return rest[1:-1]
  return rest


def _package_of(label):
  match = _PACKAGE_RE.match(label)
  if not match:
    return None
  return match.group(1) or "vehicle"


class PackageGraphTest(unittest.TestCase):

  def _roots(self):
    srcdir = os.environ.get("TEST_SRCDIR", "")
    if srcdir:
      return [srcdir]
    here = os.path.dirname(os.path.abspath(__file__))
    return [os.path.abspath(os.path.join(here, "..", "..", ".."))]

  def _collect(self):
    builds = {}
    headers = {}
    readme = None
    for root in self._roots():
      for dirpath, _, filenames in os.walk(root):
        for filename in filenames:
          path = os.path.join(dirpath, filename)
          norm = path.replace("\\", "/")
          if "intrinsic_vehicle/intrinsic/vehicle" not in norm:
            continue
          if filename in ("BUILD", "BUILD.bazel"):
            key = os.path.basename(dirpath)
            builds[key] = path
          elif filename in _HEADER_MARKERS:
            headers[filename] = path
          elif filename == "README.md" and norm.endswith(
              "intrinsic_vehicle/intrinsic/vehicle/README.md"
          ):
            readme = path
    return builds, headers, readme

  def test_visibility_and_dependency_graph(self):
    builds, _, _ = self._collect()
    self.assertCountEqual(builds, _PACKAGES)
    graph = {}
    for name, path in sorted(builds.items()):
      with open(path, encoding="utf-8") as handle:
        text = handle.read()
      self.assertIn(
          'default_visibility = ["//visibility:public"]',
          text,
          name,
      )
      if name == "testing":
        self.assertIn("testonly = 1", text)
      else:
        self.assertNotIn("testonly", text, name)
      code = _strip_loads(_strip_comments(text))
      labels = _LABEL_RE.findall(code)
      unexpected = []
      deps = set()
      for label in labels:
        if label in _ALLOWED_LABELS or label.startswith(_VEHICLE_PREFIX):
          package = _package_of(label)
          # The parent package is documentation and filegroups. It is not a
          # code edge. Self-labels are the package's own filegroup.
          if package is not None and package != name and package != "vehicle":
            deps.add(package)
          continue
        unexpected.append(label)
      self.assertEqual(unexpected, [], name)
      self.assertEqual(deps, _ALLOWED[name], name)
      graph[name] = deps
    self.assertEqual(graph, _ALLOWED)
    self._assert_acyclic(graph)

  def test_timing_boundaries_are_documented(self):
    _, headers, readme_path = self._collect()
    self.assertIsNotNone(readme_path)
    with open(readme_path, encoding="utf-8") as handle:
      readme = handle.read()
    self.assertIn("### Real-time", readme)
    self.assertIn("### Soft-real-time", readme)
    self.assertIn("### Test-only", readme)
    self.assertEqual(readme.count("| Real-time |"), 2)
    self.assertEqual(readme.count("| Soft-real-time |"), 4)
    self.assertEqual(readme.count("| Test-only |"), 1)
    self.assertIn("no Bazel dependency in either direction", readme)
    self.assertIn("manipulator_targets.tsv", readme)
    self.assertCountEqual(headers, _HEADER_MARKERS)
    for filename, marker in sorted(_HEADER_MARKERS.items()):
      with open(headers[filename], encoding="utf-8") as handle:
        self.assertIn(marker, handle.read(), filename)

  def test_dynamics_headers_stay_in_package(self):
    found = {}
    for root in self._roots():
      for dirpath, _, filenames in os.walk(root):
        norm = dirpath.replace("\\", "/")
        if not norm.endswith("intrinsic/vehicle/dynamics"):
          continue
        for filename in filenames:
          if filename in _DYNAMICS_SOURCES:
            found[filename] = os.path.join(dirpath, filename)
    self.assertCountEqual(found, _DYNAMICS_SOURCES)
    for filename, path in sorted(found.items()):
      if not filename.endswith((".h", ".cc")):
        continue
      with open(path, encoding="utf-8") as handle:
        for line in handle:
          target = _include_target(line)
          if target is None:
            continue
          lowered = target.lower()
          for token in _FORBIDDEN_INCLUDE_TOKENS:
            self.assertNotIn(
                token,
                lowered,
                "%s includes %s" % (filename, target),
            )
          self.assertTrue(
              "/" not in target
              or target.startswith("intrinsic/vehicle/dynamics/")
              or (
                  filename in _MASS_MATRIX_SOURCES
                  and target.startswith("intrinsic/vehicle/parameters/")
              ),
              "%s includes %s" % (filename, target),
          )

  def _assert_acyclic(self, graph):
    visiting = set()
    visited = set()

    def walk(node, stack):
      if node in visiting:
        self.fail("cycle: %s -> %s" % (" -> ".join(stack), node))
      if node in visited:
        return
      visiting.add(node)
      stack.append(node)
      for dep in sorted(graph[node]):
        walk(dep, stack)
      stack.pop()
      visiting.remove(node)
      visited.add(node)

    for node in sorted(graph):
      walk(node, [])


if __name__ == "__main__":
  unittest.main()
