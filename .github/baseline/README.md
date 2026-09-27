# Protected manipulator Bazel baseline

This directory holds the reviewed list of existing manipulator build and test
targets for the compatibility baseline ([issue #45](https://github.com/IshmaelRogers/intrinsic-core/issues/45),
parent work package #13). Later jobs may execute this list. This directory does
not add a CI job.

## Manifest

[manipulator_targets.tsv](manipulator_targets.tsv) is a UTF-8, LF, tab-separated
file sorted by label. The header is:

| Column | Meaning |
| --- | --- |
| `label` | Bazel target label |
| `subsystem` | Package-path classification used only to group this inventory |
| `build_vs_test` | `build` or `test` |
| `owner` | Matching CODEOWNERS entry, otherwise the Bazel package |

There is no `CODEOWNERS` file in this repository, so `owner` is the package
path. MoveIt is not built in this repo (see the MoveIt tutorial and the
separate `intrinsic-moveit` repository). No in-repo MoveIt target is listed.
No vehicle or UUV package is listed.

The generator is [`tools/inventory_manipulator_targets.py`](../../tools/inventory_manipulator_targets.py).
It only runs `bazel query` and writes the manifest. It does not modify `BUILD`
files. Selection rules live in that script. In short, it keeps libraries,
binaries, proto and FlatBuffer schema libraries, public aliases, arm
scene-object/xacro rules, skill rules, and tests from:

- `intrinsic_kinematics`
- `intrinsic_motion_planning`
- `intrinsic/manipulation` skills
- ICON arm, gripper, and force-torque parts, joint/Cartesian feature
  interfaces, joint command types, control algorithms, collision helpers,
  reflexxes, and arm hardware modules including the Gazebo hardware-module stub
- Gazebo hardware-module and gripper plugins
- robot-definition xacro and scene-object targets
- `third_party/abb_hardware_module` and `third_party/abb_egm`

Mixed packages drop targets that are not manipulator joint/Cartesian/gripper
behavior (for example IMU, rangefinder, laser tracker, ADIO, and camera
plugins). OCI image layers, data filegroups, license rules, and macro-internal
targets are omitted. Public libraries that wrap those internals stay in the
list.

## Regenerate and check

From the repository root:

```bash
python3 tools/inventory_manipulator_targets.py \
  --output .github/baseline/manipulator_targets.tsv
python3 tools/inventory_manipulator_targets.py \
  --verify .github/baseline/manipulator_targets.tsv
```

Two consecutive generations must be byte-identical. `--verify` fails if any
manifest label does not resolve through `bazel query`.
