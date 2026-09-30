# intrinsic_vehicle

Opt-in free-body vehicle packages. The first consumer is a UUV. Manipulator
joint and Cartesian contracts stay in `intrinsic_kinematics` and ICON. This
tree and `intrinsic_kinematics` have no Bazel dependency in either direction.

[ADR 0001](../../../docs/adr/0001-multi-embodiment-capability-architecture.md)
records the decision. PDR §4.1 and §15 place vehicle state math, dynamics,
allocation, and guidance/control interfaces in this tree. Gazebo APIs, vendor
SDKs, and mission behavior stay outside this tree. `dynamics` provides the
`VehicleDynamics` interface, a zero-force test double, the rigid-body mass
matrix, the added-mass matrix, the rigid-body Coriolis matrix, the
added-mass Coriolis matrix, the linear/quadratic damping wrench, the
gravity/buoyancy restoring wrench, the water-current relative
velocity, and marine-force composition. `allocation` builds the
thruster effectiveness matrix, solves unconstrained least-squares
thrust allocation, clamps those commands to per-actuator thrust
bounds, and applies thruster health to those columns and bounds.
`guidance` is the DesiredMotion-to-reference step. `control` is the
reference-to-body-wrench step. Guidance is a soft-real-time interface
with deterministic fakes and no guidance law. `control` holds the
reference depth controller, the reference heading controller, the
neutral-wrench fake, scalar heading wrap, a bounded integrator, and
back-calculation. The depth controller is heave only. The heading
controller is yaw torque only. The scalar helpers are not a control
law. Forward speed is not controlled here.
Marine model parameter schemas and validation are in `parameters`.

Protobuf `VehicleState` and `DesiredMotion` stay in
[`intrinsic_apis/intrinsic/vehicle/proto`](../../../intrinsic_apis/intrinsic/vehicle/proto/README.md).
ICON `BodyState` and `BodyWrenchCommand` stay in `intrinsic_control`. This
tree does not parse protobuf, register feature interfaces, or write actuator
commands.

## Packages

| Bazel package | Timing boundary | Scaffold |
| --- | --- | --- |
| `//intrinsic_vehicle/intrinsic/vehicle/state` | Real-time | Empty marker. Fixed-size state math lands here later. |
| `//intrinsic_vehicle/intrinsic/vehicle/allocation` | Real-time | Thruster effectiveness matrix from body-frame geometry and an explicit mask. Unconstrained least-squares allocation from that matrix, a hard clamp onto per-actuator thrust bounds, and a health adapter that scales those columns and bounds before the clamp. |
| `//intrinsic_vehicle/intrinsic/vehicle/dynamics` | Soft-real-time | `VehicleDynamics` interface, zero-force test double, rigid-body mass matrix, added-mass matrix, rigid-body Coriolis matrix, added-mass Coriolis matrix, linear/quadratic damping wrench, gravity/buoyancy restoring wrench, water-current relative velocity, and marine-force composition. Outside ICON. |
| `//intrinsic_vehicle/intrinsic/vehicle/parameters` | Soft-real-time | Marine model schemas and validation. Not loaded by ICON. |
| `//intrinsic_vehicle/intrinsic/vehicle/guidance` | Soft-real-time | `GuidanceStep` maps `DesiredMotionRt` to `MotionReferenceRt` with typed status. `EchoGuidance` echoes a pose or twist. `NullGuidance` returns a missing-objective status and an empty reference. Outside ICON. 10 to 50 Hz. Fails closed on a stale snapshot or an expired horizon. |
| `//intrinsic_vehicle/intrinsic/vehicle/control` | Soft-real-time | `ReferenceController` maps `MotionReferenceRt` and vehicle state to `BodyWrenchRt` with typed status. `ZeroWrenchController` validates inputs and returns a neutral zero body wrench. `ReferenceDepthController` maps ENU depth error to a saturated heave force and leaves surge and torque at zero. `ReferenceHeadingController` maps ENU heading error to a saturated yaw torque and leaves force, roll, and pitch at zero. Scalar helpers wrap a heading error into (-pi, pi], clamp one integrator state, and add back-calculation. Outside ICON. ICON remains the only actuator writer. |
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
| `allocation` | `BuildThrusterEffectivenessMatrix` writes body-frame `B` (6×N) into caller storage. An explicit mask zeros disabled columns. `AllocateUnconstrainedLeastSquares` maps a requested body wrench through the minimum-norm right inverse of `B`. `AllocateBoundedLeastSquares` clamps that command onto inclusive per-actuator thrust bounds and writes the achieved wrench, residual wrench, saturation flags, and residual L2 norm. No heap, no blocking, and no protobuf. ICON `ApplyCommand` wiring is later. |

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
| `guidance` | `GuidanceStep::Evaluate` at 10 to 50 Hz. DesiredMotion is not an actuator command. Fails closed on expiry and on a stale snapshot. |
| `control` | `ReferenceController::Evaluate` maps a reference to a body wrench. A zero wrench is neutral only when the status is `kOk`. Scalar heading wrap, bounded integration, and back-calculation do not write an actuator command. ICON remains the only writer of actuator commands. |
| `parameters` | Marine mass, inertia, buoyancy, added mass, damping, centers, environment, and thruster parameters (pose, axis, bounds, slew, efficiency, and health defaults). Validated at configuration time. |
| `dynamics` | `VehicleDynamics::Evaluate` outside ICON. Fixed-size inputs and results. No Gazebo API and no thruster allocation. |

### Test-only

`testing` is `testonly`. Production packages do not depend on it. It is not
a runtime loop.

## Dependency rules

Present edges inside this tree are from `testing` to `state`, `dynamics`,
`allocation`, `guidance`, `control`, and `parameters`, from `dynamics`
to `parameters` for the rigid-body mass matrix, the added-mass matrix, the
rigid-body Coriolis matrix, the added-mass Coriolis matrix, the
linear/quadratic damping wrench, the gravity/buoyancy restoring wrench,
the water-current relative velocity, and marine-force composition,
from `allocation` to `parameters` for the thruster effectiveness matrix,
unconstrained least-squares allocation, bounded thrust allocation, and
the thruster-health input adapter, and from `control` to `guidance`
for the motion-reference types the controller reads. `guidance` does
not depend on `dynamics`, `state`, or `allocation`. `control` does not
depend on `dynamics` or `allocation`.
That graph is acyclic.

Allowed later, and not wired in this scaffold:

- `dynamics` may depend on `state`.
- `allocation` may depend on `state`.
- `guidance` may depend on `state`.
- `control` may depend on `state`.

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
| Thruster parameters | Name; frame id `body`; position m from the body origin; unit axis; forward and reverse thrust bounds N; forward and reverse slew N/s; dimensionless efficiency in (0, 1]; health default and matching derate | Field checks only. Zero axis, invalid bounds, non-finite values, and invalid efficiency are rejected. The effectiveness matrix is built in `allocation`. |

`//intrinsic_vehicle/intrinsic/vehicle/parameters:six_thruster_uuv_example`
is a calm-water six-thruster UUV. Each thruster is in the body frame, with
slew limits, ideal efficiency, and nominal health. The effectiveness
matrix for this geometry is built in `allocation`. Unconstrained
least-squares allocation of that matrix, the hard clamp of that
command onto thrust bounds, and the health-to-bounds adapter are in
`allocation`.

## Vehicle dynamics interface

`//intrinsic_vehicle/intrinsic/vehicle/dynamics:dynamics` defines
`VehicleDynamics::Evaluate`. The call accepts a fixed-size state, a body
wrench, an environment snapshot, and a time step. It returns a state
derivative and diagnostics, or `DynamicsErrorCode::kInvalidArgument`.

The derivative is the instantaneous rate of pose and body velocity. The
time step is validated and reported. It is not an integration horizon.
`Evaluate` does not advance the pose.

The interface is opt-in. Nothing in this tree constructs a dynamics model
unless the caller does. ICON does not load this package. Manipulator
targets do not depend on it.

`ZeroForceDynamics` is the deterministic test double. For a valid input it
reports the rigid-body kinematic derivative of the pose from the body twist
and a zero body acceleration. Gravity, fluid density, current, and the
input wrench do not change that acceleration. The same inputs produce the
same outputs. Invalid inputs return a status and a zero, finite derivative.

`ComputeRigidBodyMassMatrix` is the rigid-body mass matrix term. It reads
mass, inertia about the center of gravity, and the body-frame center of
gravity from `parameters`. The result is the body-frame 6x6 matrix in
surge, sway, heave, roll, pitch, yaw order. Coriolis, damping, buoyancy,
added mass, and integration are not this function. `Evaluate` does not call
it. The same inputs produce the same matrix. Invalid mass, inertia, or
center of gravity returns `kInvalidArgument` and a zero, finite matrix.

`ComputeAddedMassMatrix` is the hydrodynamic added-mass matrix term. It
reads the row-major 6x6 `AddedMass::coefficients` stored for
`ValidateMarineModel`. The result is that matrix in the body frame, in
surge, sway, heave, roll, pitch, yaw order. The function copies the stored
coefficients. It does not derive them from geometry and does not convert
ENU and NED. Coriolis, damping, buoyancy, the rigid-body mass matrix, and
integration are not this function. `Evaluate` does not call it. The same
inputs produce the same matrix. Non-finite entries, asymmetry beyond
`1e-9`, or a matrix that is not positive definite return
`kInvalidArgument` and a zero, finite matrix.

`ComputeRigidBodyCoriolisMatrix` is the rigid-body Coriolis and centripetal
matrix. It reads mass, inertia about the center of gravity, the body-frame
center of gravity, and a body twist in surge, sway, heave, roll, pitch, yaw
order. The result is the body-frame 6x6 matrix `C_RB`. Added-mass Coriolis,
damping, buoyancy, and integration are not this function.
`ZeroForceDynamics::Evaluate` does not call it.
`MarineForceDynamics::Evaluate` calls it on the body twist. The same
inputs produce the same matrix. Invalid mass,
inertia, or center of gravity, or a non-finite twist, returns
`kInvalidArgument` and a zero, finite matrix.

`ComputeAddedMassCoriolisMatrix` is the added-mass Coriolis and centripetal
matrix. It reads the added-mass coefficients validated by
`ComputeAddedMassMatrix` and a body twist in surge, sway, heave, roll,
pitch, yaw order. The result is the body-frame 6x6 matrix `C_A`. Momentum
comes from that added-mass matrix only. The function does not add the
rigid-body Coriolis matrix, does not derive added mass from geometry, and
does not convert ENU and NED. Damping, buoyancy, and integration are not
this function. `ZeroForceDynamics::Evaluate` does not call it.
`MarineForceDynamics::Evaluate` calls it on the relative twist. The same
inputs produce the same matrix. Invalid added mass, or a non-finite twist, returns
`kInvalidArgument` and a zero, finite matrix.

`ComputeLinearQuadraticDampingWrench` is the linear and quadratic damping
wrench. It reads `Damping::linear_coefficients`, the row-major 6x6 matrix
`D_L`, and `Damping::quadratic_coefficients`, the length-6 diagonal
coefficients `d_q`. The body twist is the supplied relative velocity in
surge, sway, heave, roll, pitch, yaw order. The result is the body-frame
wrench `τ_d = -(D_L + D_Q(|ν|)) ν`, where `D_Q(|ν|) = diag(d_q ∘ |ν|)`.
The function uses that twist as given. It does not convert ENU and NED and
does not subtract an environment current. Coriolis, restoring, buoyancy,
and integration are not this function. `ZeroForceDynamics::Evaluate` does
not call it. `MarineForceDynamics::Evaluate` calls it on the relative
twist. The same inputs produce the same wrench. Non-finite coefficients,
linear asymmetry beyond `1e-9`, a linear matrix that is not positive definite, a
negative quadratic coefficient, or a non-finite twist return
`kInvalidArgument` and a zero, finite wrench.

`ComputeGravityBuoyancyRestoringWrench` is the gravity and buoyancy
restoring wrench. It reads `MassInertia::mass_kg`,
`Buoyancy::displaced_volume_m3`, the two body-frame centers,
`Environment::gravity_m_s2`, and `Environment::fluid_density_kg_m3`, plus
the navigation frame and the navigation-from-body Hamilton quaternion.
Weight is `W = m g` and buoyancy is `B = ρ g V`. World-down is
`[0, 0, +1]` in NED and `[0, 0, -1]` in ENU. `R` is the body-to-navigation
rotation of that quaternion, the same active map as the zero-force pose
derivative. The body wrench is

`τ_g = [f_W^b + f_B^b; r_g × f_W^b + r_b × f_B^b]`

with `f_W^b = R^T (W e_down)` and `f_B^b = R^T (-B e_down)`, in surge,
sway, heave, roll, pitch, yaw order. Inertia, damping, added mass,
current, and thrusters are not read. Coriolis, drag, and integration are
not this function. `ZeroForceDynamics::Evaluate` does not call it.
`MarineForceDynamics::Evaluate` calls it. The same inputs produce
the same wrench. A non-finite or non-positive parameter, a pose frame that
is not `world_enu` or `world_ned`, a non-finite orientation, or a
quaternion outside the `1e-9` unit tolerance returns `kInvalidArgument`
and a zero, finite wrench. The first defect wins.

`ComputeWaterCurrentRelativeVelocity` is the water-current relative-velocity
helper. It reads `Environment::current_velocity_m_s` and
`Environment::current_frame_id`, the navigation frame, the
navigation-from-body Hamilton quaternion, and a body twist in surge, sway,
heave, roll, pitch, yaw order. Linear current in the body frame is the
supplied vector when the frame id is `body`, and `R^T` times that vector
when the frame id is `world_enu` or `world_ned`. `R` is the
body-to-navigation rotation of that quaternion, the same active map as the
restoring wrench. The frame id selects the basis the current vector already
uses. This function does not swap ENU and NED axes. Angular current is
zero, so the relative twist is

`ν_r = [ν1 − v_c^b; ν2]`

in surge, sway, heave, roll, pitch, yaw order. A zero current leaves
`ν_r = ν`. Gravity, density, mass, damping, added mass, and thrusters are
not read. The result is a twist in meters/second and radians/second. It is
not a wrench. Coriolis, damping, restoring, and integration are not this
function. `ZeroForceDynamics::Evaluate` does not call it.
`MarineForceDynamics::Evaluate` calls it. The same inputs produce the same
twist. A non-finite current, an empty or unknown current frame, a pose
frame that is not `world_enu` or `world_ned`, a non-finite orientation, a
quaternion outside the `1e-9` unit tolerance, a non-finite twist, or a
non-finite relative twist returns `kInvalidArgument` and a zero, finite
relative twist. The first defect wins.

`MarineForceDynamics` composes those helpers into `DynamicsResult`. It does
not re-derive their equations. Rigid-body Coriolis uses the body twist `ν`.
Added-mass Coriolis and damping use `ν_r`. Restoring uses the pose and the
evaluation snapshot for gravity and density. Coefficient matrices, mass,
centers, and displaced volume come from the stored model. The stored
environment and the thruster list are not read.

The damping and restoring helpers already return the force on the body.
The hydrodynamic wrench, excluding the input wrench, is

`τ_hydro = -C_RB(ν) ν - C_A(ν_r) ν_r + τ_damp(ν_r) + τ_g`

which is `-C_RB(ν) ν - C_A(ν_r) ν_r - D(ν_r) ν_r - g(η)` after that
substitution. `model_force_n` and `model_torque_n_m` copy `τ_hydro`. The
input wrench is passed through as `total_wrench = τ_input + τ_hydro`.
That total is the derivative input. `DynamicsResult` has no mass-matrix
field, so this evaluation does not form `M = M_RB + M_A` and does not
solve `M ν̇ = τ`. `body_acceleration` stays zero. The pose rate is the
same kinematic map as `ZeroForceDynamics`. `dt` is reported and is not an
integration step. `allocation_invoked` stays false.

Diagnostics store `ν_r` and each signed force term. The four force terms
sum to `hydrodynamic_wrench` and to the model wrench. A failed helper
returns that status. Every failure writes finite zeros. The first defect
wins. The same inputs produce the same outputs.

### Allocation

Thruster allocation stays in `allocation`. `Evaluate` does not map a wrench
to actuator commands, does not read actuator health, and does not call
`BuildThrusterEffectivenessMatrix` or `AllocateUnconstrainedLeastSquares`.
`DynamicsDiagnostics::allocation_invoked` stays false for the
implementations in this package.

Returned values are fixed-size. `ValidateEvaluationInputs` and
`ZeroForceDynamics` do not allocate heap memory. Status text is a static
string view. `Evaluate` does not retain its arguments after it returns.

### Thread safety

`Evaluate` is const. Concurrent `Evaluate` calls on one instance are safe
for an implementation with no unsynchronized mutable members.
`ZeroForceDynamics` has no data members. `ValidateEvaluationInputs` reads
only its arguments. This package takes no locks and is not called from the
ICON cycle.

### Input contract

Body axes are REP-103. Twist order is surge, sway, heave, roll, pitch, yaw.
Orientation is a Hamilton quaternion stored x, y, z, w. Pose frame is
`world_enu` or `world_ned`. Wrench frame is `body`. Current frame is
`world_enu`, `world_ned`, or `body`. This package does not convert frames.

`ValidateEvaluationInputs` returns the first defect. A zero time step is
valid. A negative or non-finite time step is not. Gravity and density may
be zero and may not be negative. Non-finite values and a non-unit
quaternion are invalid. Quaternion tolerance is an absolute `1e-9` on
`|norm - 1|`. The check order is pose frame, state finiteness, unit
quaternion, wrench frame, wrench finiteness, environment finiteness,
gravity sign, density sign, current finiteness, current frame, then time
step.

### Dependencies

The dynamics library links the C++ standard library and `parameters`. It
does not depend on Gazebo, a vendor SDK, ICON, protobuf, or `allocation`.

`//intrinsic_vehicle/intrinsic/vehicle/dynamics:vehicle_dynamics_test`
checks this contract and the zero-force double.
`//intrinsic_vehicle/intrinsic/vehicle/dynamics:rigid_body_mass_matrix_test`
checks the rigid-body mass matrix fixtures.
`//intrinsic_vehicle/intrinsic/vehicle/dynamics:added_mass_matrix_test`
checks the added-mass matrix fixtures.
`//intrinsic_vehicle/intrinsic/vehicle/dynamics:rigid_body_coriolis_matrix_test`
checks the rigid-body Coriolis fixtures.
`//intrinsic_vehicle/intrinsic/vehicle/dynamics:added_mass_coriolis_matrix_test`
checks the added-mass Coriolis fixtures.
`//intrinsic_vehicle/intrinsic/vehicle/dynamics:linear_quadratic_damping_wrench_test`
checks the linear and quadratic damping fixtures.
`//intrinsic_vehicle/intrinsic/vehicle/dynamics:gravity_buoyancy_restoring_wrench_test`
checks the gravity and buoyancy restoring fixtures.
`//intrinsic_vehicle/intrinsic/vehicle/dynamics:water_current_relative_velocity_test`
checks the water-current relative-velocity fixtures.
`//intrinsic_vehicle/intrinsic/vehicle/dynamics:marine_force_dynamics_test`
checks marine-force composition.
`//intrinsic_vehicle/intrinsic/vehicle/dynamics:marine_force_analytic_regression_test`
checks hand-calculated analytic fixtures for that composition.

## Guidance and reference control

`//intrinsic_vehicle/intrinsic/vehicle/guidance:guidance` defines
`GuidanceStep::Evaluate`. The call accepts a fixed-size `DesiredMotionRt`,
an optional vehicle-state snapshot, and an update period. It returns a
`MotionReferenceRt` or a `GuidanceStatus`. A failure returns an empty
reference. `has_pose` and `has_twist` are false, and every numeric field
is a finite zero. That empty reference is not a pose and not a command.

`DesiredMotionRt` mirrors the approved `DesiredMotion` fields. This
package does not parse protobuf. `snapshot_id` is the monotonic stamped
identity. Callers copy `StampedHeader.sequence` or the world snapshot id.
The objective is a pose, a body twist, or an opaque trajectory id of at
most 64 bytes. The id is not a trajectory sample. `horizon_s` and
`confidence` are optional. A supplied confidence lies in `[0, 1]`. A
supplied zero is present. An omitted value is not read.

`MotionReferenceRt` is the reference the controller consumes. A pose
objective copies that pose and leaves the twist unset. A twist objective
copies that body twist and leaves the pose unset. `snapshot_id` and
`update_period` are echoed from the intent and the step. On failure the
update period is zero, including when the supplied period was rejected.

`VehicleStateRt` matches `dynamics::VehicleStateRt` for pose frame,
position, orientation, and body twist, and appends `snapshot_id`.
`FrameId` enumerators match `dynamics::FrameId`. Guidance does not
depend on `dynamics`. Body axes are REP-103. This package does not
convert ENU and NED.

`EchoGuidance` is the identity-map test double. It applies no guidance
law. The state pose and twist are validated and then left unused. A
trajectory id is not sampled: `Evaluate` returns `kInvalid` and an empty
reference. `ValidateGuidanceInputs` still returns `kOk` for that id.
`NullGuidance` uses the same input checks. A usable pose or twist then
returns `kMissingObjective` and an empty reference. An unset objective
returns `kMissingObjective` with the message `objective is unset`.

`//intrinsic_vehicle/intrinsic/vehicle/control:control` defines
`ReferenceController::Evaluate`. The call accepts a `MotionReferenceRt`,
a `VehicleStateRt`, and an update period. It returns a `BodyWrenchRt` or
a `ControlStatus`.

`BodyWrenchRt` matches `dynamics::BodyWrenchRt`: body frame, force in
newtons, and torque in newton-meters. Control does not depend on
`dynamics` or `allocation`, so the type is defined here. `MotionReferenceRt`,
`VehicleStateRt`, `Duration`, and `FrameId` are the guidance types.
`ZeroWrenchController` validates inputs and returns that zero wrench with
`kOk`. That is the neutral wrench. It is not an actuator command. The
same zero bits with any other status are not a command. ICON remains the
only actuator writer.

### Status

Both boundaries use the same code names. `kOk` is the only success code.

| Code | Meaning |
| --- | --- |
| `kOk` | The reference or the neutral wrench is present. |
| `kInvalidArgument` | Non-finite value, confidence outside `[0, 1]`, negative horizon, negative update period, or a trajectory id longer than 64 bytes. |
| `kInvalid` | Bad frame, non-unit quaternion, empty trajectory id, or a trajectory id that `EchoGuidance` does not sample. |
| `kMissingObjective` | The objective or the reference was not supplied. `NullGuidance` also uses this code when it refuses to echo. |
| `kStale` | Snapshot ids differ, or a supplied horizon is shorter than the update period. |

The first defect wins. Failure writes finite zeros.

### Update period

Zero is an instantaneous sample. Negative or non-finite is
`kInvalidArgument`. Soft-real-time guidance runs at 10 to 50 Hz
(PDR §11). The interface does not reject another non-negative period.
A horizon equal to the update period still covers that step. A zero
horizon with a zero update period is not expired. These packages are
not on the ICON cycle.

### Check order

Guidance: update period; state, when the pointer is non-null (pose
frame, finiteness, unit quaternion); supplied confidence; supplied
horizon; objective; snapshot identity, when state is non-null; horizon
expiry. A null state pointer skips the state and snapshot checks.
Fields of an objective that was not selected are not read.

Control: update period; state pose frame, finiteness, and unit
quaternion; reference update period; present pose; present twist;
missing pose and twist; snapshot identity. A reference field that is
not present is not read. Stored numbers with `has_pose` and `has_twist`
both false are not a pose. The depth controller and the heading
controller then require a pose and a `world_enu` frame on the state
and on that pose. A twist-only reference is `kMissingObjective`. A
`world_ned` pose is `kInvalid`.

### Thread safety and ownership

`Evaluate` is const. `EchoGuidance`, `NullGuidance`, and
`ZeroWrenchController` have no data members. Concurrent `Evaluate` calls
on one of those instances do not share mutable state.
`ReferenceDepthController` owns its integral force in a mutable member.
`ReferenceHeadingController` owns its integral torque the same way.
Concurrent `Evaluate` or `Reset` calls on either instance are not safe.
The functions do not allocate, do not retain arguments, and do not take
locks. Status text is a static string view. `TrajectoryId::view()` is
valid only while that object is alive. The caller owns the intent, the
state, the reference, and the result.

### Dependencies

The guidance library links the C++ standard library only. The control
library links guidance. Neither links protobuf, Gazebo, a vendor SDK,
ICON, allocation, or kinematics.

`//intrinsic_vehicle/intrinsic/vehicle/guidance:motion_guidance_test`
checks this contract and the two guidance fakes.
`//intrinsic_vehicle/intrinsic/vehicle/control:reference_control_test`
checks this contract and the neutral-wrench fake.

### Scalar control math

`//intrinsic_vehicle/intrinsic/vehicle/control:control` also defines
scalar helpers in `control_math.h`. They take doubles. They do not read
`MotionReferenceRt`, `VehicleStateRt`, or `BodyWrenchRt`.

`ShortestSignedAngle` returns `atan2(sin(delta), cos(delta))` in
(-pi, pi]. A result of exactly -pi is returned as +pi.
`HeadingError(reference, measured)` is that wrap of
`reference - measured`.

`BoundedIntegrator::Integrate` computes `x + u * dt` and then clamps
`x` to the configured inclusive limits. The write happens only after
the checks succeed. A zero time step with a finite input leaves `x`
unchanged and returns success. A negative or non-finite time step, a
non-finite input, or a non-finite product or sum returns
`kInvalidArgument` and leaves `x` unchanged.

`BackCalculation` is `k_aw * (u_sat - u_unsat)`.
`IntegrateBackCalculation` adds that term to the integrator input and
then calls `Integrate`. The caller supplies the unsaturated command,
the saturated command, and the gain. The helper does not clamp the
command and does not choose `k_aw`. A negative finite gain is accepted.

These functions do not allocate. Status text is a static string view.
`Integrate` and `IntegrateBackCalculation` mutate one integrator and
are not safe for concurrent calls on that object. The pure functions
do not share mutable state. This is not a controller, a gain policy,
or a closed-loop plant.

`//intrinsic_vehicle/intrinsic/vehicle/control:control_math_test`
checks the pi boundaries, saturation entry and exit, a zero time step,
non-finite rejection, and integrator recovery after back-calculation.

### Reference depth controller

`ReferenceDepthController` maps one ENU depth objective to body heave.
Depth is positive down: `depth_m = -z_world`. Heave is wrench index 2,
newtons, +z up. The measured depth rate is the ENU up component of the
body linear twist, negated. The state orientation is the
body-to-navigation map. This package does not convert NED and does not
depend on `dynamics`.

Contract revision 1 sets `k_p = 20` N/m, `k_i = 0.2` N/(m·s), and
`k_d = 45` N/(m/s). Heave is clamped to [-50, 50] N. The controller
owns the integral force `S` in newtons, clamped to [-40, 40].
`Reset()` sets `S` to 0.

```
e = depth_cmd_m - depth_meas_m
v_depth = -v_z_world
u_unsat = -k_p * e + S + k_d * v_depth
```

Away from the clamp and from saturation, `S = -k_i * ∫ e dt`, so the
command is `-k_p * e - k_i * I + k_d * v_depth`. The derivative
opposes positive-down rate. A positive depth error produces negative
heave. Surge and every torque stay 0. `S` is advanced with
`IntegrateBackCalculation`. The integrator input is `-k_i * e` and
`k_aw` is 0.2.

A rejected call leaves `S` unchanged and returns finite zeros. That
zero wrench is not a command. `ValidateControlInputs` runs first. A
missing pose is `kMissingObjective`. A non-ENU world frame is
`kInvalid`. Non-finite depth, rate, or heave is `kInvalidArgument`.
A snapshot mismatch is `kStale`.

`//intrinsic_vehicle/intrinsic/vehicle/control:reference_depth_controller_test`
checks the heave sign, the derivative, anti-windup against pure
integration, the integrator clamp, rejected inputs, and the 2 m
step on a point-mass heave plant (`m = 60` kg, `b = 3` N·s/m,
`dt = 0.1` s, semi-implicit Euler). The fixture bounds are overshoot
at most 15% of the step and settling within 30 s into ±0.05 m.

### Reference heading controller

`ReferenceHeadingController` maps one ENU heading objective to body
yaw torque. Heading is the yaw of body +x about world +z. Yaw torque
is `torque_n_m[2]`, newton-meters, which is spatial index 5. The
measured yaw comes from the state quaternion. The yaw rate is the body
twist yaw component. This package does not convert NED and does not
depend on `dynamics`.

Contract revision 1 sets `k_p = 10` N·m/rad, `k_i = 0.3` N·m/(rad·s),
and `k_d = 8` N·m/(rad/s). Yaw torque is clamped to [-15, 15] N·m. The
controller owns the integral torque `S` in newton-meters, clamped to
[-12, 12]. `Reset()` sets `S` to 0.

```
e = HeadingError(yaw_cmd, yaw_meas)
r = body yaw rate
u_unsat = k_p * e + S - k_d * r
```

Away from the clamp and from saturation, `S = k_i * ∫ e dt`, so the
command is `k_p * e + k_i * I - k_d * r`. The derivative opposes
positive body yaw rate. A positive heading error produces positive yaw
torque. Force, roll, and pitch stay 0. `S` is advanced with
`IntegrateBackCalculation`. The integrator input is `k_i * e` and
`k_aw` is 0.2.

A rejected call leaves `S` unchanged and returns finite zeros. That
zero wrench is not a command. `ValidateControlInputs` runs first. A
missing pose is `kMissingObjective`. A non-ENU world frame is
`kInvalid`. Non-finite yaw, yaw rate, or yaw torque is
`kInvalidArgument`. A snapshot mismatch is `kStale`. An attitude that
fails the unit-quaternion check is `kInvalid`.

`//intrinsic_vehicle/intrinsic/vehicle/control:reference_heading_controller_test`
checks the yaw sign, the derivative, anti-windup against pure
integration, the integrator clamp, the shortest direction across ±π,
rejected inputs, and the 0.5 rad step on a point-mass yaw plant
(`J = 5` kg m^2, `b = 1` N·m·s/rad, `dt = 0.1` s, semi-implicit Euler).
The fixture bounds are overshoot at most 15% of the step and settling
within 20 s into ±0.02 rad.

## Thruster effectiveness matrix

`//intrinsic_vehicle/intrinsic/vehicle/allocation:allocation` defines
`BuildThrusterEffectivenessMatrix`. The call reads body-frame thruster
geometry and an explicit enable mask. It writes the 6×N matrix `B` into
caller-provided columns. Column `i` is the body wrench, in newtons and
newton-meters, produced by one newton of thrust along `direction_body`:

`force = direction_body`

`moment = position_m × direction_body`

Row order is surge, sway, heave, roll, pitch, yaw. Body axes are REP-103.
`B[row, col] = columns[col][row]`. `enabled[i] == false` writes a zero
column and leaves column `i` aligned with thruster `i`. The mask is the
only exclusion input. Health, derate, efficiency, slew, and thrust bounds
are not read and do not scale `B`. Health scaling is the adapter below.

`frame_id` must be `body`. `world_enu`, `world_ned`, an empty id, and any
other id are rejected. This function does not convert a world-frame pose
into the body frame.

An empty thruster list, a mask or column span whose length is not the
thruster count, a non-finite position or direction, a zero axis, and a
non-unit direction return `AllocationErrorCode::kInvalidArgument`. The
output columns are finite zeros. The first defect wins. The function does
not allocate. It does not solve for thrust, project bounds, or report a
residual. ICON does not call it.

`//intrinsic_vehicle/intrinsic/vehicle/allocation:thruster_effectiveness_matrix_test`
checks the hand-calculated columns, the six-thruster example, the mask,
and rejected inputs.

## Unconstrained least-squares allocation

`AllocateUnconstrainedLeastSquares` reads columns of `B` and a requested
body wrench `τ`. It does not read thrust bounds, slew, efficiency, health,
or health derate. For full row rank it returns the minimum-norm solution

`u = Bᵀ (B Bᵀ)⁻¹ τ`

`B Bᵀ` is factored with unpivoted Cholesky. A pivot must exceed
`kCholeskyPivotRelativeTolerance` (1e-12) times the largest diagonal
entry of `B Bᵀ`. A pivot that does not exceed that floor, or a
non-finite factor entry, returns `AllocationErrorCode::kRankDeficient`.
No Tikhonov term is added. The
thrust commands and the residual `τ - B u` are written into caller
storage. The function does not allocate. A bad size or a non-finite input
returns `kInvalidArgument`, finite-zero commands, and a finite-zero
residual. Rank deficiency writes finite-zero commands and the residual of
that zero command, which is `τ`. The first defect wins. The same inputs
produce the same commands. Full row rank on the six-thruster example
reconstructs `τ` within `1e-9`. ICON does not call this function.
Per-actuator bounds are applied by `AllocateBoundedLeastSquares`.

`//intrinsic_vehicle/intrinsic/vehicle/allocation:least_squares_allocator_test`
checks that reconstruction, a rank-deficient matrix, rejected inputs, and
bit-stable repeated calls.

## Bounded thrust allocation

`AllocateBoundedLeastSquares` reads columns of `B`, a requested body
wrench `τ`, and inclusive per-actuator command bounds. It computes the
unconstrained command with `AllocateUnconstrainedLeastSquares`, then
clamps each command into `[min_thrust_n, max_thrust_n]`. The clamp does
not move surplus wrench onto unsaturated actuators. Slew, efficiency,
health, and health derate are not inputs. Apply health with
`ApplyThrusterHealthToAllocationInputs` before this call.

`ThrustCommandBoundsFromGeometry` reads only
`max_reverse_thrust_n` and `max_forward_thrust_n`. The command interval
is `[-max_reverse_thrust_n, max_forward_thrust_n]`.

The call writes the bounded command, the achieved wrench `B u`, the
residual `τ - B u`, one saturation flag per actuator, and the residual
L2 norm. A flag is true when the stored command equals either bound,
including a command that was already on the bound. Saturation is not an
error. A successful clamp returns `kOk`.

Storage is caller-owned. The function does not allocate. A bad size or
a non-finite input returns `kInvalidArgument` and finite-zero outputs:
commands, achieved wrench, residual wrench, clear saturation flags, and
residual norm `+0`. The first defect wins. `kRankDeficient` is
propagated from the unconstrained solver. On that status the commands
stay the solver's finite zeros and are not clamped, so an interval that
excludes 0 can contain none of them. The achieved wrench is zero, the
residual is `τ`, the saturation flags are false, and the residual norm
is `||τ||_2` when that norm is finite. A non-finite residual norm
returns `kInvalidArgument` and finite zeros instead.

When the unconstrained command already lies inside every bound, the
bounded command matches it and achieved plus residual reconstructs `τ`
within `1e-9`. After a clamp, no successful command leaves its interval,
and the stored residual is `τ - B u` within `1e-9`. The same inputs
produce the same outputs. ICON does not call this function.

`//intrinsic_vehicle/intrinsic/vehicle/allocation:bounded_allocator_test`
checks the interior match, single- and multi-actuator saturation, an
exactly-at-bound command, residual identity, bit-stable repeats, bound
compliance, rank-deficient fail-closed behavior, and rejected inputs.

## Thruster health adapter

`ApplyThrusterHealthToAllocationInputs` reads thruster health and
`health_derate`. It writes an enabled mask, thrust-command bounds, and,
when a column span is provided, health-scaled effectiveness columns.
The caller passes those inputs to `AllocateBoundedLeastSquares`. This
function does not solve, clamp, or factor `B Bᵀ`. It does not read
efficiency or slew. It does not discover faults or command hardware.

`kNominal` (`health_derate` 1) leaves the column bit-identical to the
unscaled matrix and writes the geometry command interval. Allocation
then matches `AllocateBoundedLeastSquares` on those unscaled inputs.
`kDerated` (`health_derate` in (0, 1)) multiplies both the geometry
command interval and the effectiveness column by that same derate.
`kDisabled`, `kStuckOff`, and `kFailed` (`health_derate` 0) clear the
mask, write the neutral interval `[0, 0]`, and clear the column to
`+0`. That is the column `BuildThrusterEffectivenessMatrix` writes for
`enabled == false`. The neutral command is 0.

An empty column span skips column writes. Storage is caller-owned. The
function does not allocate. A bad size, an unknown health value, a
`health_derate` that does not match health, or a non-finite derate,
thrust limit, or column returns `kInvalidArgument` and finite-zero
outputs. The first defect wins. The same inputs produce the same
outputs. ICON does not call this function.

`//intrinsic_vehicle/intrinsic/vehicle/allocation:thruster_health_adapter_test`
checks the nominal bit match, neutral commands, consistent derating,
a single-failure residual, determinism, and rejected inputs.

## Out of scope

Slew limiting, efficiency scaling, redistributing a saturated wrench
onto unsaturated thrusters, mass-matrix acceleration, body-state time
integration, guidance laws, surge control, trajectory sampling,
Gazebo plugins, and ICON feature wiring are later issues.
The guidance interface above is not a guidance law. The scalar
integrator in `control` clamps the integral force owned by the depth
controller and the integral torque owned by the heading controller.
It does not integrate a body state.
