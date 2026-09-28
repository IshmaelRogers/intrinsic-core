# ADR 0001: Additive multi-embodiment capability architecture

## Status

Accepted. This ADR records the executive decision in the Preliminary Design
Review cited below. It does not implement that design, and it does not add
vehicle packages, protos, FlatBuffers, Gazebo plugins, or production build
changes.

## Source

Recorded from the owner-provided Preliminary Design Review:

- **Title:** Intrinsic Core Multi Embodiment Physical AI Platform —
  Preliminary Design Review for regression safe UUV enablement
- **Prepared:** 26 September 2026
- **Repository:** IshmaelRogers/intrinsic-core
- **PDR evidence pin:** `64abb3cf1c645426b507f6cbe1f0bd297095ee8c`
- **Checked-in source text:**
  - [Intrinsic-Core-Multi-Embodiment-PDR.txt](assets/Intrinsic-Core-Multi-Embodiment-PDR.txt)
  - [Intrinsic-Core-Multi-Embodiment-PDR.docx](assets/Intrinsic-Core-Multi-Embodiment-PDR.docx)

The evidence pin is the commit the PDR inspected. The tip of `main` may
differ. PDR Appendix C says the PDR should be revised if upstream changes
materially alter ICON cycle semantics, feature registration, World
components, simulator service boundaries, or inference protocol handling.

Section numbers below refer to that PDR.

## Context

Intrinsic Core is a manipulator platform. The PDR inspected the repository at
the evidence pin, including root architecture guidance, the Bazel package
layout, the ICON real-time part and feature-interface registry, FlatBuffer
HAL schemas, `intrinsic_kinematics` model interfaces, motion-planning
packages, World component protos, the Gazebo simulator service and plugins,
camera-oriented perception packages, and the Open Inference Protocol /
Triton-backed inference service. The design focuses on stable extension seams
(PDR §2).

The observed baseline and the design consequence stated in PDR §2 are:

| Observed baseline | Design consequence |
| --- | --- |
| ICON `RealtimePartInterface` brackets each cycle with `ReadStatus` and `ApplyCommand` and exposes action-facing features through `FeatureInterfaceRegistry`. | Add body-state and body-wrench features through the registry; do not generalize every joint class. |
| HAL messages are FlatBuffers; public services and skills are protobuf/gRPC. | Maintain two representations with checked boundary conversions. |
| `intrinsic_kinematics` models articulated trees and single-DoF joints. | Keep it unchanged; put free-body dynamics in `intrinsic_vehicle`. |
| World already uses additive entity components. | Add marine components and a synchronized snapshot service rather than a parallel world database. |
| Simulation has a polymorphic Simulator interface and Gazebo plugin adapters. | Add opt-in marine plugins and share equations with `intrinsic_vehicle`. |
| Inference already serves OIP/Triton models and manages model assets. | Add deadlines, provenance, validity, and async orchestration around OIP; do not replace it. |
| Skills are instantiated per interaction and must be stateless. | State estimators, world fusion, policy execution, and recording are persistent service assets; skills remain thin clients. |

Goals in PDR §3:

- Preserve source, wire, configuration, build, and behavioral compatibility
  for supported manipulator paths.
- Support a free-swimming six-DoF UUV in simulation with canonical state,
  guidance, safety, planning, perception contracts, and shadow-mode Physical
  AI.
- Make future UAV, UGV, USV, legged, and mobile-manipulator support possible
  without editing a central embodiment enumeration.
- Use the same public sensor and actuator contracts in simulation and
  hardware adapters.
- Make every safety-relevant decision replayable and attributable to state,
  world snapshot, configuration, and model versions.

## Decision

### Executive decision

Approve an additive capability architecture. Intrinsic Core should not be
converted from a manipulator system into a vehicle system. It should become
an embodiment-neutral platform whose current manipulator contracts remain the
first compatibility profile and whose UUV implementation is a second profile
composed from new capabilities. ICON remains deterministic execution
authority; learned systems produce timestamped intent and never actuator
commands (PDR §1).

| Decision | PDR position |
| --- | --- |
| Compatibility | No renaming, widening, or reinterpretation of existing joint, Cartesian, kinematics, motion-planning, World, or Gazebo contracts. |
| Polymorphism | Select capabilities through registries and interfaces. Do not add a central switch on robot type. |
| Real time | FlatBuffers and fixed-size C++ types inside ICON. Protobuf and services outside the real-time loop. |
| Safety | AI intent passes through deterministic validation, planning, guidance, ICON limits, allocation, and hardware watchdogs. |
| Simulation | Gazebo plugins are opt-in adapters that reuse the same vehicle dynamics library and HAL contracts. |
| Delivery | Gate every phase on manipulator goldens, UUV conformance tests, replay determinism, and a headless scenario. |

### Definitions

These terms use the PDR's language. They do not rename or widen existing
joint, Cartesian, kinematics, motion-planning, World, or Gazebo contracts.

**Embodiment.** A robot class hosted as a compatibility profile on an
embodiment-neutral platform. The current manipulator contracts are the first
compatibility profile. The UUV implementation is a second profile composed
from new capabilities. An embodiment descriptor states what a resource
provides. Adding a robot class registers implementations; it does not extend
a platform-wide enum or change existing consumers (PDR §1, §3, §4).

**Capability.** A behavior selected through registries and interfaces, not
through a central switch on robot type. Business logic requests interfaces
such as `BodyState`, `BodyWrenchCommand`, `StateSpace`, `DynamicsModel`,
`RangeObservation`, or `SafetyRule`. Capability discovery is additive; old
clients may ignore unknown capability IDs. If the vehicle capability is
missing, action creation fails with `FailedPrecondition` and the manipulator
remains unaffected (PDR §1, §4, §7, §12).

**State.** The latched condition deterministic execution reads. Inside ICON,
`ReadStatus` latches body state and actuator health at cycle start. Outside
the real-time loop, the persistent estimation service publishes the
authoritative `VehicleState` (PDR §5 and §8): stamped header, world-from-body
pose, body twist, body acceleration, pose and twist covariance, navigation
mode, source health, and estimator epoch. World remains the spatial
authority. Planning, safety, and AI consume the same immutable snapshot ID.
Invalid and absent are distinct. Non-finite values are invalid. This term
does not reinterpret joint state or `KinematicsComponent` /
`PhysicsComponent` (PDR §5, §7, §8, §12).

**Command.** A bounded actuator value written only by ICON. `ApplyCommand`
performs final validation, invokes the allocator, writes bounded actuator
commands, and neutralizes on timeout or fault. Learned systems produce
timestamped intent (`DesiredMotion`: a pose, twist, or trajectory objective
with horizon, confidence, and model provenance) and never actuator commands.
Vehicle actions access `BodyWrenchCommand` through
`FeatureInterfaceRegistry`. Existing `HalArmPart` and joint features are
untouched. Existing joint and Cartesian command contracts are not renamed,
widened, or reinterpreted (PDR §1, §5, §7, §12).

**Adapter.** An opt-in boundary that keeps existing contracts stable.
Gazebo plugins are opt-in adapters that reuse the same vehicle dynamics
library and HAL contracts. Simulation and hardware adapters use the same
public sensor and actuator contracts. World is ENU at platform boundaries;
marine NED is an explicit adapter; body axes follow REP-103. A frame is never
inferred from message type. C++ extension is new namespaces, packages, and
adapters only: no removal or signature change to current public manipulator
classes. With components absent or the plugin disabled, existing terrestrial
worlds execute unchanged (PDR §1, §5, §10, §12).

**Execution authority.** ICON. ICON remains the deterministic execution
authority. Learned systems produce timestamped intent and never actuator
commands. AI intent passes through deterministic validation, planning,
guidance, ICON limits, allocation, and hardware watchdogs. The safety
authority state is separate from ICON and gates that path: shadow produces
logs only; recommend requires operator/executive acceptance; constrained
allows validated intent; revoked/emergency fail closed. ICON applies
system/application limits and command age every cycle. Hardware keeps
driver-level saturation, neutral-on-disable, health feedback, and an
independent watchdog. The inference package has no dependency path to
actuator command APIs (PDR §1, §9).

### Capability composition

The architecture uses capability composition. An embodiment descriptor states
what a resource provides, but business logic requests interfaces. Adding a
robot class registers implementations; it does not extend a platform-wide
enum or change existing consumers (PDR §4).

There is no central robot-type switch or embodiment enumeration.

### Representation and contract rules

Non-real-time contracts are protobuf. Real-time contracts are fixed-size C++
and FlatBuffers. Conversion occurs at one audited boundary and validates
units, frames, age, finiteness, quaternion normalization, covariance shape,
limits, and sequence monotonicity (PDR §5).

PDR §5 names `StampedHeader`, `VehicleState`, and `DesiredMotion` as the
canonical non-real-time sketches. The message text stays in the checked-in
PDR. This ADR does not add those `.proto` files.

Invariants from PDR §5:

| Invariant | Rule |
| --- | --- |
| Frames | World is ENU at platform boundaries; marine NED is an explicit adapter. Body axes follow REP-103. Never infer a frame from message type. |
| Units | SI only. Field names include a unit suffix when ambiguity remains, for example `depth_m` and `pressure_pa`. |
| Time | Source time is measurement/model time; receive time is ingestion time; ICON uses monotonic age. Wall clock never drives watchdogs. |
| Validity | Invalid and absent are distinct. Non-finite values are invalid. Every derived value carries age/source. |
| Covariance | Fixed order is documented and validated. Unknown covariance is explicit, not an all-zero matrix. |
| Evolution | Append fields and enum values; reserve removed tags/names; preserve unknown values; provide golden serialization fixtures. |

PDR §6 names narrow interfaces at backend and algorithm boundaries:
`VehicleDynamics`, `ControlAllocator`, `StateSpace`, `SafetyRule`, and
`InferenceClient`. Plain value types remain concrete. Real-time calls use
fixed-size values, explicit ownership, bounded complexity, no exceptions, and
status returns. Persistent services may use gRPC and protobuf but expose
deadline and cancellation semantics. Those interfaces are not added by this
ADR.

### ICON and hardware execution

A `VehicleRealtimePart` implements the existing `RealtimePartInterface`.
`ReadStatus` latches body state and actuator health at cycle start. Actions
access `BodyState`, `BodyWrenchCommand`, and `VehicleLimits` through
`FeatureInterfaceRegistry`. `ApplyCommand` performs final validation, invokes
the allocator, writes bounded actuator commands, and neutralizes on timeout
or fault. Existing `HalArmPart` and joint features are untouched (PDR §7).

Cycle order from PDR §7:

`ReadStatus` → latch state/health → run ICON actions → validate wrench →
allocate bounded actuator commands → `ApplyCommand`.

Failure precedence from PDR §7:

fatal hardware fault > safety emergency > watchdog expiry > allocator
failure > action failure > normal command.

| Failure | Required response |
| --- | --- |
| Missing vehicle capability | Action creation fails with `FailedPrecondition`; manipulator remains unaffected. |
| Stale/invalid state | Reject new wrench; use configured neutral/hold behavior. |
| Command timeout | Latch watchdog status, command neutral, require explicit reset. |
| Allocator infeasible | Report residual and saturation; safety/controller chooses degrade or abort. |
| Actuator fault | Recompute with health/derating if safe; otherwise neutral and escalate. |

ICON control and allocation runs at 100 to 1000 Hz, platform dependent, and
never waits; a stale command neutralizes. No allocation, blocking inference,
protobuf parsing, or unbounded work runs in the ICON cycle (PDR §3, §11).

### Extension seams

New behavior attaches at the seams below. It does not replace the manipulator
profile.

| Seam | How the PDR extends it |
| --- | --- |
| ICON feature interfaces | `FeatureInterfaceRegistry` already exposes action-facing features. Add body-state, body-wrench, and vehicle-limit features through the registry. Do not generalize every joint class. Existing `HalArmPart` and joint features stay untouched (PDR §2, §7). |
| World components | World already uses additive entity components and remains the spatial authority. Add marine components (bathymetry, current-field, occupancy, semantic-contact, uncertainty, validity) and a `WorldSnapshotBuilder` that returns an immutable snapshot. Do not add a parallel world database. Do not reinterpret `KinematicsComponent` or `PhysicsComponent` (PDR §2, §8, §12). |
| Assets | State estimators, world fusion, policy execution, and recording are persistent service assets. No default UUV assets or plugins are added to the industrial profile. A marine profile composes existing services. Rollback removes marine profile, plugin, and asset registration (PDR §2, §12). |
| Skills | Skills are instantiated per interaction and must be stateless. They remain thin clients of those persistent services (PDR §2). |
| Simulator | The simulator stays replaceable through the existing Simulator interface. Gazebo receives opt-in marine components and plugins. The hydrodynamics plugin calls `intrinsic_vehicle` `VehicleDynamics` so simulation and planning/control tests share equations. Plugin absent means identical existing world behavior (PDR §2, §10, §12). |

### Package ownership

Decided ownership from PDR §4.1. These rows are ownership boundaries. This
ADR does not create, move, or rename packages.

| Package | Owns | Must not own |
| --- | --- | --- |
| `intrinsic_control` ICON | Real-time parts, features, actions, limits, watchdogs | Planning, inference, protobuf parsing, hydrodynamic parameter loading |
| `intrinsic_vehicle` | Vehicle state math, dynamics, allocation, guidance/control interfaces | Gazebo APIs, vendor SDKs, mission behavior |
| `intrinsic_hardware` | Vendor-neutral sensor/actuator contracts and driver adapters | Navigation fusion and mission logic |
| `intrinsic_estimation` | Persistent navigation filter and source-health logic | Perception semantics or actuator commands |
| `intrinsic_world` | Entities, marine components, immutable snapshot construction | Learned-policy execution |
| `intrinsic_motion_planning` | Vehicle state space, constraints, trajectory and planner registry | Low-level control allocation |
| `intrinsic_safety` | Deterministic rules and authority state machine | Neural inference backends |
| `intrinsic_inference` | Existing OIP model serving plus async envelopes | ICON cycle or direct hardware dependencies |
| `intrinsic_simulation` | Simulator adapters/plugins and deterministic scenario configuration | Duplicate vehicle equations |
| `intrinsic_data` | Episode schema, recorder, replay, digests | Control authority |

### Compatibility and migration

Preserve source, wire, configuration, build, and behavioral compatibility for
supported manipulator paths (PDR §3).

| Compatibility surface | Policy |
| --- | --- |
| C++ | No removal or signature change to current public manipulator classes. New namespaces/packages and adapters only. |
| Proto | Append fields; reserve removed identifiers; golden bytes and unknown-field tests. |
| FlatBuffers | Append fields/types with defaults; stable feature IDs; verifier fixtures. |
| Bazel | No new mandatory dependency in baseline targets; UUV targets are separate/opt-in. |
| Runtime manifests | No default UUV assets or plugins in the industrial profile. Add a marine profile that composes existing services. |
| World | New component types; no reinterpretation of `KinematicsComponent` or `PhysicsComponent`. |
| Motion planning | New request/state-space types and registry; current manipulator services remain. |
| Simulation | Plugin absent means identical existing world behavior. |
| SDK | Capability discovery is additive; old clients may ignore unknown capability IDs. |

A feature may graduate only after its compatibility adapter and rollback path
exist. Rollback removes the marine profile, plugin, and asset registration.
It does not require a data migration or restoration of old manipulator
schemas (PDR §12).

Joint and Cartesian behavior stays source- and wire-compatible: no renaming,
widening, or reinterpretation of those contracts, and no new mandatory
dependency on the protected manipulator baseline.

### Safety path

Recorded from PDR §9:

AI result → envelope validation → authority state → `SafetyRule` set →
`ACCEPT` | `PROJECT` | `REJECT` | `ABORT` | `SURFACE` → planner feasibility →
guidance/controller → ICON limits → allocation → actuator watchdog.

Rules are deterministic, independent, side-effect free, and versioned.
Projection precedence is safety emergency, collision/clearance, operating
envelope, energy, then mission preferences. Conflicting hard projections
reject rather than choose opportunistically.

### Simulation

The simulator remains replaceable through the existing Simulator interface.
Gazebo receives opt-in marine components and plugins. The hydrodynamics
plugin calls `intrinsic_vehicle` `VehicleDynamics`. It applies buoyancy,
restoring forces, added mass, damping, current-relative velocity, and
actuator dynamics to configured free bodies. With components absent or the
plugin disabled, existing terrestrial worlds execute unchanged (PDR §10).

Fidelity is staged. Phase one validates architecture with coefficient-based
six-DoF dynamics; later models may implement the same `VehicleDynamics`
interface. CFD coupling is an external backend and cannot leak into control
or planning contracts (PDR §10).

Tests never command real hardware. The reference solution is
simulation-only. Physical trials require a separate readiness review,
explicit operator consent, hardware emergency procedures, HIL evidence, and a
validated operating envelope (PDR §13).

### Non-goals for the first release

Non-goals for the first release are certified functional safety, unrestricted
end-to-end learned control, production acoustic swarm coordination,
vendor-specific drivers, high-fidelity CFD, or real-water deployment. Those
require later qualification after the architecture and evidence pipeline are
stable (PDR §3).

The communications/swarm epic is intentionally deferred. Its next
decomposition should begin only after the PDR's single-vehicle contracts,
replay, authority, and extension mechanics are proved (PDR §17).

### Regression guard

The protected manipulator baseline in #13 remains the gate.

- PDR §17: each implementation task carries the regression guard that #13
  remains green and the feature is opt-in.
- PDR §18: maintainers accept #13 and #20 as permanent merge gates.
- PDR §3 exit target: protected build/tests and golden status/output digests
  stay unchanged, including manipulator regression with UUV linked and with
  UUV excluded (PDR §13).
- PDR §7: when the vehicle capability is absent, action creation fails with
  `FailedPrecondition` and the manipulator remains unaffected.
- PDR §10 and §12: with the marine component or plugin absent, existing
  terrestrial worlds execute unchanged, and UUV Bazel targets stay
  separate and opt-in. No new mandatory dependency is added to baseline
  targets. The industrial profile does not gain default UUV assets or
  plugins.

New behavior is opt-in when the capability or configuration is absent.

### Recorded sequence

PDR §14 and §17 record the authorized implementation sequence. This ADR does
not start that work and does not apply the §15 change map.

| Phase | Scope (issue ids as stated in the PDR) | Exit gate |
| --- | --- | --- |
| 0 Baseline | #13 through #16 | Protected manipulator surface and architecture rules are executable in CI. |
| 1 Contracts and ICON | #17 through #20 | Vehicle feature cycle works with fakes; manipulator goldens unchanged. |
| 2 Vehicle and HAL | #21 through #28 | Dynamics, guidance, allocation, sensors, and thruster fakes pass unit/conformance tests. |
| 3 Estimation and safety | #29 through #32 | Dive/recovery replay and fail-closed authority/rule tests pass. |
| 4 Planning, world, perception, inference | #33 through #37 | Coherent snapshot drives planner and async policy envelope; sonar-only is valid. |
| 5 Simulation and data | #38 through #42 | Fixed-seed UUV scenario and deterministic replay pass in CI. |
| 6 Reference and shadow mode | #43 and #44 | Policy cannot command actuators; classical behavior remains authoritative. |
| 7 Future embodiments | #41 plus new profiles | New embodiment passes the conformance kit without core switches or baseline changes. |

PDR §18 records the approval checklist that accompanies this decision:
architecture owners accept additive capability composition and the
no-central-switch rule; ICON owners accept the new feature-interface path and
fixed-size real-time representations; World and simulation owners accept
additive components and opt-in plugins; safety reviewers accept layered
authority and the prohibition on learned actuator commands; maintainers accept
#13 and #20 as permanent merge gates; the team agrees that issues #13–#44 are
the authorized implementation sequence and that broader scope requires a PDR
or ADR update.

## Consequences

- Manipulator joint and Cartesian contracts remain the first compatibility
  profile. UUV support, when later implemented, is a second profile.
- Callers select capabilities by interface. No platform-wide robot-type enum
  is introduced.
- ICON stays the only writer of actuator commands. Learned output stops at
  timestamped `DesiredMotion`.
- Real-time and non-real-time paths keep separate wire formats, with one
  audited conversion boundary.
- `intrinsic_kinematics` stays an articulated-tree model. Free-body dynamics
  are owned by `intrinsic_vehicle` when that package is implemented.
- World grows by new component types and immutable snapshots. Existing
  kinematics and physics components keep their meaning.
- Gazebo marine behavior is an opt-in plugin. Absent plugin or absent
  capability leaves the current manipulator and terrestrial simulation path
  unchanged.
- Rollback of a future marine profile removes registration. It does not
  migrate data or restore old manipulator schemas.
- The first release excludes certified functional safety, unrestricted
  end-to-end learned control, production acoustic swarm coordination,
  vendor-specific drivers, high-fidelity CFD, and real-water deployment.
- Verification of later phases is simulation and test only until a separate
  readiness review. This record does not command hardware.

This change records the decision and links it from the architecture docs.
Proposed paths in PDR §15 (`intrinsic_vehicle`, marine HAL, estimation,
safety, vehicle protos, FlatBuffers, Gazebo hydro plugins, marine profiles,
and the rest of that change map) are not created here.
