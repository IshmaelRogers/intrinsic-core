# Protected manipulator Bazel baseline

This directory holds the reviewed list of existing manipulator build and test
targets for the compatibility baseline ([issue #45](https://github.com/IshmaelRogers/intrinsic-core/issues/45),
parent work package #13). The protected build job reads this list. It does not
enable UUV configuration.

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

## CI build

[`.github/workflows/protected-manipulator-build.yml`](../workflows/protected-manipulator-build.yml)
builds every manifest row whose `build_vs_test` column is `build`. Test rows
are left for a later job. The workflow calls the helper below and does not
copy labels into the YAML. The helper's Bazel invocation is plain `bazel
build` plus a target-pattern file and a build-event log. It does not pass
`--config`, so no UUV configuration is enabled.
`.github/workflows/postsubmit.yml` and `.github/workflows/release.yml` are
unchanged.

The job uses the same runner and Bazel setup as the post-submit build
(`ubuntu-24.04-8core`, `bazel-contrib/setup-bazel@0.14.0`, shared
`intrinsic-core-build` cache). That runner is a GitHub larger runner. A
repository whose Actions runners do not provide `ubuntu-24.04-8core` will
queue this job instead of building.

From the repository root, the local equivalent of the CI step is:

```bash
python3 tools/build_protected_manipulator_targets.py \
  --manifest .github/baseline/manipulator_targets.tsv
```

On failure the helper prints the earliest manifest label that failed, after
Bazel's own log:

```text
First failing target: //pkg:name
```

If the only recorded failure is outside the manifest, that label is printed
instead. `--dry-run` checks the manifest and prints how many `build` labels
would be built, without invoking Bazel.
