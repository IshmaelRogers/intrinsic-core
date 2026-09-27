# Manipulator planning-request golden

Checked-in snapshot of how one manipulator `MotionPlanningRequest` is
serialized and validated before any vehicle target is linked (issue
[#51](https://github.com/IshmaelRogers/intrinsic-core/issues/51), parent work
package [#20](https://github.com/IshmaelRogers/intrinsic-core/issues/20)).

The fixture is
[`testdata/manipulator_planning_request.golden.json`](testdata/manipulator_planning_request.golden.json).
[`planning_request_golden_test.py`](planning_request_golden_test.py) reads
`CreateMotionPlanningRequestFromPointPath` and the protos that factory fills.
It does not plan a trajectory, and it does not edit production sources, `BUILD`
files, or
[`.github/baseline/manipulator_targets.tsv`](../../../../.github/baseline/manipulator_targets.tsv).

## What the golden records

One representative 6-DOF joint-space request for robot `agilus_04`, plus the
two invalid inputs the factory rejects:

- Valid: one `JOINT` segment. Status `OK`. The textproto sets
  `robot_specification` and `motion_specification` only.
- Invalid: `point_path` and `motion_types` have different lengths. Status
  `INVALID_ARGUMENT` and the current size-mismatch message. No request is
  returned.
- Invalid: both lists are empty, so the size check passes and the empty-path
  check fires. Status `INVALID_ARGUMENT` and the current empty-path message.
  No request is returned.

Stable output fields on the valid request are the robot object name, the
`JOINT` enum number, the segment count, and the target joints. Wire tags for
`object_name`, `motion_type`, and `joints` are recorded from the current
protos. `world_id` and the other request fields this factory leaves unset stay
in `fields_left_unset`.

The fixture does not name a vehicle, UUV, marine, thruster, or bathymetry
feature.

## Commands

From the repository root, compare the current factory to the fixture:

```bash
python3 -m unittest discover \
  -s intrinsic_motion_planning/intrinsic/motion_planning/service \
  -p 'planning_request_golden_test.py' -v
```

Two renders in one process must be byte-identical. The unittest path only
reads the golden. It does not rewrite it.

Rewriting the fixture requires this flag. A senior owner must review the
golden diff before it lands:

```bash
python3 intrinsic_motion_planning/intrinsic/motion_planning/service/planning_request_golden_test.py \
  --update-golden
```

Without `--update-golden`, that command exits and leaves the fixture unchanged.
The updater also refuses to write a fixture that references a vehicle feature.
