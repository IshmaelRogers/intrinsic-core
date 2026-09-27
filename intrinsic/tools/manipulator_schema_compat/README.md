# Schema compatibility comparison

Compares a generated schema inventory with the protected manipulator baseline
and reports wire-incompatible removals or renumbering (issue
[#48](https://github.com/IshmaelRogers/intrinsic-core/issues/48), parent work
package [#13](https://github.com/IshmaelRogers/intrinsic-core/issues/13)).

The protected scope stays `manipulator-public-wire-v1`. This package does not
edit [`manipulator_public_schema_inventory.json`](../manipulator_schema_baseline/manipulator_public_schema_inventory.json),
does not expand `out_of_scope_imports`, and does not parse or change `.proto`
or `.fbs` sources. The #47 extractor's local `--compare` remains an in-script
stand-in. This package is the fixture-backed gate.

## Policy

Compatible (exit 0):

- identical inventories
- a new field, enum value, message, RPC, or schema whose proto tag or
  FlatBuffer id was not already used and is not inside a baseline reserved
  range

Incompatible (exit 1), one line per finding:

- removed schema, message, table, struct, field, enum, enum value, service,
  RPC, extend, or union member
- a proto tag reused by a different field, including a tag that still belongs
  to a removed field
- a FlatBuffer vtable id or struct positional id that changed
- an enum value whose number changed, or a new name that takes a baseline
  number
- a changed field type, label, oneof, default, packed flag, required bit, or
  id kind

`--strict` also fails when the only findings are additive.

Deprecating a field and changing `json_name` are not wire breaks. Summary
counts, the selection-rule prose, and the `out_of_scope_imports` list are not
compared. Those imports are not opened and their symbols are not required.

Exit 2 means an inventory could not be read or is missing the #47 record
shape. That is not a wire-compatibility result.

## Commands

From the repository root:

```bash
python3 intrinsic/tools/manipulator_schema_compat/compare_schema_inventory.py \
  intrinsic/tools/manipulator_schema_baseline/manipulator_public_schema_inventory.json \
  /tmp/inventory.json

python3 -m unittest discover \
  -s intrinsic/tools/manipulator_schema_compat \
  -p '*_test.py' -v
```

One path compares that candidate to the checked-in baseline:

```bash
python3 intrinsic/tools/manipulator_schema_compat/compare_schema_inventory.py \
  /tmp/inventory.json
```

Fixtures under [`testdata/`](testdata/) are synthetic inventories in the #47
shape. They are not production schemas. Each candidate has a
`.expected.txt` file with the exact report.

## Assumptions

- The #47 JSON shape is the comparison input. Field numbers are proto tags,
  FlatBuffer vtable ids, or struct positional indexes (`id_kind`).
- Additive evolution keeps old tags and ids and allocates a fresh number.
- Reserving a number and then using it is a tag reuse, even though the #47
  in-script compare classifies that new field as additive.
- #49 owns an intentional break of real schema sources and the CI wiring.
  This package does not add that job.
