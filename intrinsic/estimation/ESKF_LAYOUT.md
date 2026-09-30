# ESKF layout (#81)

Layout, types, defaults, and index-order tests only. No propagation, no
measurement update, no gating, no injection or reset of error into nominal
(`#82+`), no F/G/H/Q/R or discrete Phi, no ICON, no safety authority. Water
current, scale factors, and lever arms are deferred and would need a new locked
contract. Contract: parent WP #30, issue #81. Manipulator contracts and
`.github/baseline/manipulator_targets.tsv` are unchanged.

Files: `eskf_state.{h,cc}` (C++), `eskf_state.py` (Python mirror),
`eskf_state_test.{cc,py}` (paired tests).

## Nominal state `EskfNominal` (16 scalars)

SI, body REP-103, world ENU at the platform boundary.

| Index | Symbol | Qty | Unit | Notes |
| --- | --- | --- | --- | --- |
| 0..2 | `p_enu` | position | m | world ENU |
| 3..6 | `q_wxyz` | attitude | 1 | unit quaternion, **wxyz** order, body→world |
| 7..9 | `v_body` | linear velocity | m/s | body frame |
| 10..12 | `b_a` | accel bias | m/s² | body |
| 13..15 | `b_g` | gyro bias | rad/s | body |

## Error state `EskfError` (15 scalars)

Attitude error is 3, not 4.

| Index | Symbol | Qty | Unit |
| --- | --- | --- | --- |
| 0..2 | `δp` | position error | m |
| 3..5 | `δθ` | attitude error (small-angle) | rad |
| 6..8 | `δv` | velocity error | m/s |
| 9..11 | `δb_a` | accel-bias error | m/s² |
| 12..14 | `δb_g` | gyro-bias error | rad/s |

In code the error symbols are ASCII: `dp`, `dtheta`, `dv`, `dba`, `dbg`.
Unit strings in `kNominalFields` / `kErrorFields` (and the Python
`NOMINAL_FIELDS` / `ERROR_FIELDS`) are `m`, `1`, `m/s`, `m/s^2`, `rad`,
`rad/s`.

## Covariance `P`

Fixed **15×15**, row-major storage of the error-state ordering above. Unknown
covariance is **unset/absent**, never the all-zero matrix (match #17, where an
unset parent field means unknown). `EskfCovariance` default-constructs with no
storage (`has_value()` false). `EskfCovariance::Identity()` /
`EskfCovariance.identity()` is a separate test helper giving a known diagonal;
it is not the default.

## Constants

`kNominalDim = 16`, `kErrorDim = 15`, `kCovDim = 15`. C++ `inline constexpr`
in namespace `intrinsic::estimation`, with `static_assert`s. Python
module-level in `eskf_state.py` (also as `NOMINAL_DIM`, `ERROR_DIM`,
`COV_DIM`). `FromVector` / `FromRowMajor` (C++, returns `std::nullopt`) and
`from_sequence` / `EskfCovariance(...)` (Python, raises `ValueError`) reject a
wrong size.

## Defaults (deterministic)

| Type | Default |
| --- | --- |
| `EskfNominal` | `p = 0`, `q = (1, 0, 0, 0)`, `v = 0`, `b_a = 0`, `b_g = 0` |
| `EskfError` | all zeros |
| `EskfCovariance` | unspecified (empty), not zeros |

## Mapping to #17 `VehicleState` (documentation only)

For a later leaf. This package does not change #17 protos and implements no
filter.

| ESKF nominal | `VehicleState` |
| --- | --- |
| `p_enu` + `q_wxyz` | `pose_world_from_body` (`header.frame_id` = `world_enu`). Position in m. Orientation is stored x, y, z, w in the proto, so `q_wxyz` (w, x, y, z) must be reordered. |
| `v_body` | `body_twist` linear part (body frame, m/s). |
| `b_a`, `b_g` | Internal. Not published. |

Covariance `P` (error-state order) is not published as is. #17 carries
`pose_covariance` and `twist_covariance` as 6×6 in their own orders, and unset
means unknown. The projection from `P` is left to the later leaf.
