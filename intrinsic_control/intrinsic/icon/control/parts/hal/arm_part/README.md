# HalArmPart feature-registration golden

Checked-in snapshot of the arm part and feature-interface registration behavior
that exists before any vehicle feature is linked (issue
[#50](https://github.com/IshmaelRogers/intrinsic-core/issues/50), parent work
package [#20](https://github.com/IshmaelRogers/intrinsic-core/issues/20)).

The fixture is
[`testdata/arm_feature_registration.golden.json`](testdata/arm_feature_registration.golden.json).
[`feature_registration_golden_test.py`](feature_registration_golden_test.py)
reads the current `HalArmPart` sources, the feature-interface registry, and
`FeatureInterfaceTypes`. It does not edit production arm code, `BUILD` files,
or [`.github/baseline/manipulator_targets.tsv`](../../../../../../../.github/baseline/manipulator_targets.tsv).

## What the golden records

- Part type `HalArmPart`, config message `intrinsic_proto.icon.HalArmPartConfig`,
  and factory `HalArmPart::FromProto`.
- Part-factory results: inserting a new part type returns true, registering the
  same part type again returns false, and a missing part type yields an empty
  factory.
- Each feature class `HalArmPart` registers, the `FeatureInterfaceTypes` value
  and number it exports, and how many construction sites register it.
- Registry status values for those interfaces: `OK` on success,
  `INVALID_ARGUMENT` when the pointer is null, and `ALREADY_EXISTS` when that
  interface is already registered, including the current message text.
- Stable `absl::Status` codes and message literals returned while
  `HalArmPart::FromProto` registers interfaces.

The fixture does not name a vehicle, UUV, marine, thruster, or bathymetry
feature. Gripper, IMU, rangefinder, ADIO, and force-torque parts are omitted
because this arm part does not register them.

## Commands

From the repository root, compare the current arm sources to the fixture:

```bash
python3 -m unittest discover \
  -s intrinsic_control/intrinsic/icon/control/parts/hal/arm_part \
  -p 'feature_registration_golden_test.py' -v
```

Two renders in one process must be byte-identical. The unittest path only
reads the golden. It does not rewrite it.

Rewriting the fixture requires this flag. A senior owner must review the
golden diff before it lands:

```bash
python3 intrinsic_control/intrinsic/icon/control/parts/hal/arm_part/feature_registration_golden_test.py \
  --update-golden
```

Without `--update-golden`, that command exits and leaves the fixture unchanged.
The updater also refuses to write a fixture that references a vehicle feature.
