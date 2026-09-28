# intrinsic_vehicle

Opt-in free-body vehicle packages. The first consumer is a UUV. Manipulator
joint and Cartesian contracts stay in `intrinsic_kinematics` and ICON. This
tree and `intrinsic_kinematics` have no Bazel dependency in either direction.

[ADR 0001](../../../docs/adr/0001-multi-embodiment-capability-architecture.md)
records the decision. PDR §4.1 and §15 place vehicle state math, dynamics,
allocation, and guidance/control interfaces in this tree. Gazebo APIs, vendor
SDKs, and mission behavior stay outside this tree. This scaffold does not
implement dynamics equations, allocation, or guidance. Marine model
parameter schemas and validation are in `parameters`.

Protobuf `VehicleState` and `DesiredMotion` stay in
[`intrinsic_apis/intrinsic/vehicle/proto`](../../../intrinsic_apis/intrinsic/vehicle/proto/README.md).
ICON `BodyState` and `BodyWrenchCommand` stay in `intrinsic_control`. This
tree does not parse protobuf, register feature interfaces, or write actuator
commands.

## Packages

| Bazel package | Timing boundary | Scaffold |
| --- | --- | --- |
| `//intrinsic_vehicle/intrinsic/vehicle/state` | Real-time | Empty marker. Fixed-size state math lands here later. |
| `//intrinsic_vehicle/intrinsic/vehicle/allocation` | Real-time | Empty marker. Bounded allocation lands here later. |
| `//intrinsic_vehicle/intrinsic/vehicle/dynamics` | Soft-real-time | Empty marker. Dynamics land here later, outside ICON. |
| `//intrinsic_vehicle/intrinsic/vehicle/parameters` | Soft-real-time | Marine model schemas and validation. Not loaded by ICON. |
| `//intrinsic_vehicle/intrinsic/vehicle/guidance` | Soft-real-time | Empty marker. Guidance lands here later, outside ICON. |
| `//intrinsic_vehicle/intrinsic/vehicle/control` | Soft-real-time | Empty marker. Reference control lands here later, outside ICON. |
| `//intrinsic_vehicle/intrinsic/vehicle/testing` | Test-only | Test-only umbrella and the package-graph check. |

Include paths strip the `intrinsic_vehicle` root, the same way the other
`intrinsic_*` trees do. A header in `state` is included as
`intrinsic/vehicle/state/state.h`.

## Real-time and soft-real-time

PDR §5, §7, and §11. FlatBuffers and fixed-size C++ stay inside the ICON
cycle. Protobuf and services stay outside that cycle. Conversion stays at
one audited boundary. This scaffold does not convert.

### Real-time

ICON control and allocation run at 100 to 1000 Hz and never wait (PDR §11).

| Package | Role on this boundary |
| --- | --- |
| `state` | Future fixed-size vehicle state values. Safe to read from a latched ICON cycle once those types exist. No protobuf in this package. |
| `allocation` | Future bounded allocator invoked from `ApplyCommand`. No heap, no blocking, and no protobuf on that path. |

`ReadStatus` and `ApplyCommand` stay in ICON. This scaffold does not link
ICON and does not run on a real-time thread.

### Soft-real-time

Guidance runs at 10 to 50 Hz on the latest valid snapshot and fails closed
on expiry (PDR §11). Reference control sits on that side of the boundary.
Dynamics are shared by later planning tests and a simulator adapter. ICON
does not load hydrodynamic parameters (PDR §4.1), so dynamics stay outside
the ICON cycle. The `parameters` package is that configuration-time check.

| Package | Role on this boundary |
| --- | --- |
| `guidance` | Future DesiredMotion-to-reference step. Intent is not an actuator command. |
| `control` | Future reference-to-body-wrench step. ICON remains the only writer of actuator commands. |
| `parameters` | Marine mass, inertia, buoyancy, added mass, damping, centers, environment, and thruster geometry. Validated at configuration time. |
| `dynamics` | Future `VehicleDynamics` evaluations outside ICON. No Gazebo API in this tree. |

### Test-only

`testing` is `testonly`. Production packages do not depend on it. It is not
a runtime loop.

## Dependency rules

The only present edge inside this tree is from `testing` to `state`,
`dynamics`, `allocation`, `guidance`, `control`, and `parameters`. That
graph is acyclic.

Allowed later, and not wired in this scaffold:

- `dynamics` may depend on `parameters` and `state`.
- `allocation` may depend on `parameters` and `state`.
- `guidance` may depend on `state`.
- `control` may depend on `guidance` and `state`.

Forbidden:

- `parameters` depending on `state`, `dynamics`, `allocation`, `guidance`, `control`, or `testing`.
- `state` depending on `dynamics`, `allocation`, `guidance`, `control`, `parameters`, or `testing`.
- Any production package depending on `testing`.
- Either direction between this tree and `intrinsic_kinematics`.
- A dependency on Gazebo, a vendor SDK, mission behavior, or ICON.

`//intrinsic_vehicle/...` is public so a later opt-in consumer can depend on
a package. Public visibility does not add a dependency to the manipulator
baseline. [`.github/baseline/manipulator_targets.tsv`](../../../.github/baseline/manipulator_targets.tsv)
does not list these targets. `tools/inventory_manipulator_targets.py` does
not query this tree.

With nothing in the baseline depending on these packages, manipulator
behavior is unchanged when the vehicle capability is absent.

## Checks

From the repository root:

```bash
bazel build //intrinsic_vehicle/...
bazel test //intrinsic_vehicle/...
bazel query 'somepath(//intrinsic_kinematics/..., //intrinsic_vehicle/...)'
bazel query 'somepath(//intrinsic_vehicle/..., //intrinsic_kinematics/...)'
bazel query 'deps(//intrinsic_vehicle/...) intersect //intrinsic_kinematics/...'
```

`//intrinsic_vehicle/intrinsic/vehicle/testing:package_graph_test` checks
the in-tree edges, rejects labels outside this tree except `gtest_main` and
`//visibility:public`, and checks that each package sets that visibility.
An empty `somepath` means that direction has no dependency. Bazel analysis
rejects a dependency cycle before a target builds.

## Marine model parameters

`//intrinsic_vehicle/intrinsic/vehicle/parameters:parameters` defines one
free-body marine model in SI units and validates it.
`ValidateMarineModel` is explicit. An empty model is not valid, and this
tree does not install a model when the vehicle capability is absent. ICON
does not load these parameters. The check reads only its argument.

Body axes are REP-103: x forward, y left, z up. Degree-of-freedom order is
surge, sway, heave, roll, pitch, yaw. Matrices are row-major. Symmetry
uses an absolute tolerance of `1e-9`. Positive definiteness is unpivoted
Cholesky with no pivot slack. Every defect is returned, tagged with a
field path and `ModelErrorCode`.

| Schema | Fields | Check |
| --- | --- | --- |
| Mass / inertia | `mass_kg`; 3x3 inertia about the center of gravity, kg·m² | Mass > 0. Inertia symmetric and positive definite. |
| Centers | Center of gravity and center of buoyancy, m from the body origin | Finite. |
| Buoyancy | Displaced volume, m³ | > 0. Restoring force is a later term. |
| Added mass | 6x6 coefficients | Symmetric and positive definite. |
| Damping | 6x6 linear coefficients; 6 quadratic coefficients | Linear symmetric and positive definite. Each quadratic coefficient >= 0. |
| Environment | Gravity magnitude m/s², fluid density kg/m³, current m/s and frame id | Gravity and density > 0. Current finite. Frame id is `world_enu`, `world_ned`, or `body`. No conversion. |
| Thruster geometry | Name, position m, unit direction, forward and reverse thrust bounds N | Field checks only. No allocation. |

`//intrinsic_vehicle/intrinsic/vehicle/parameters:six_thruster_uuv_example`
is a calm-water six-thruster UUV. Slew, efficiency, health, and allocation
are later issues.

## Out of scope

Dynamics equations, thruster allocation, guidance laws, control laws,
Gazebo plugins, and ICON feature wiring are later issues.
