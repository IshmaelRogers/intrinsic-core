# Gazebo hardware-module path golden

Checked-in snapshot of the simulated manipulator hardware-module path before
any other embodiment is linked (issue
[#52](https://github.com/IshmaelRogers/intrinsic-core/issues/52), parent work
package [#20](https://github.com/IshmaelRogers/intrinsic-core/issues/20)).

The fixture is
[`testdata/gazebo_hwm_manipulator.world.sdf`](testdata/gazebo_hwm_manipulator.world.sdf).
The compared record is
[`testdata/gazebo_hwm_path.golden.json`](testdata/gazebo_hwm_path.golden.json).
[`gazebo_hwm_path_golden_test.py`](gazebo_hwm_path_golden_test.py) loads that
fixture headlessly: XML only, with rendering left off the way
`DefaultServerConfig` sets it. The world template and the UR5e model are
existing assets. The test does not start a Gazebo server, and it does not edit
production sources, `BUILD` files, or
[`.github/baseline/manipulator_targets.tsv`](../../../../.github/baseline/manipulator_targets.tsv).

## What the golden records

- Headless launch: `SetHeadlessRendering(true)`, the default Bullet
  Featherstone physics plugin, plugins from the existing world template, and
  the `HardwareModuleLauncher` plugin named in the fixture.
- The UR5e asset resolved through `model.config`, and the revolute joints
  discovered from that model in document order.
- Interfaces the current inference path registers for a jointed model, plus
  the `safety_status` interface the module always adds. Each row has the
  advertise call, command or status direction, flatbuffer, and whether the
  interface tracks the joint count.
- Command and status behavior on the null realtime-clock path: an inactive
  tick is a no-op, `RequestIconTick` arms one `ReadStatus`, `ApplyCommand`
  does not block, and `WaitForIconTicksToFinish` returns once that read has
  happened.

The fixture does not name a vehicle, UUV, marine, thruster, or bathymetry
plugin or component. No such plugin is loaded.

## CI runtime budget

The golden compare must finish in **5 seconds**. It parses in-repo XML and
C++ and does not sleep, step physics, or start `gz`. The checked-in record
stores the same budget.

## Commands

From the repository root, compare the current path to the fixture:

```bash
python3 -m unittest discover \
  -s intrinsic/simulation/gazebo/plugins \
  -p 'gazebo_hwm_path_golden_test.py' -v
```

Two renders in one process must be byte-identical. The unittest path only
reads the golden. It does not rewrite it.

Rewriting the fixture requires this flag. A senior owner must review the
golden diff before it lands:

```bash
python3 intrinsic/simulation/gazebo/plugins/gazebo_hwm_path_golden_test.py \
  --update-golden
```

Without `--update-golden`, that command exits and leaves the fixture unchanged.
The updater also refuses to write a fixture that references a vehicle feature.
