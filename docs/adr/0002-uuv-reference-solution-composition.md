# ADR 0002: UUV reference-solution composition and staged packaging

## Status

Accepted. bit cloud (@IshmaelRogers) approved decisions D43-1, D43-2, D43-3
and D43-7 on 30 October 2026, under lock `adr-0002-uuv-reference-v1` on
issue #256. This ADR is leaf S43-0 of work package #43. It records decisions
and amends the wording of the #43 Definition of Done. It does not add
packages, code, BUILD targets, protos, assets, runtime manifests, or CI
changes.

## Source

- **Work package:** #43, "[Reference] Add simulation-only UUV Physical AI
  solution". Dependencies #34, #36, #37, #40 and #42. Phase 6 of the PDR.
- **Decision issue and lock:** #256, lock `adr-0002-uuv-reference-v1`.
- **Base:** `main` @ `40e8d231fa49fdc2e0e8b0aedcfea6f83b5a3d7c` (#238, PR
  #251).
- **Architecture record:**
  [ADR 0001](0001-multi-embodiment-capability-architecture.md).
- **PDR text:**
  [Intrinsic-Core-Multi-Embodiment-PDR.txt](assets/Intrinsic-Core-Multi-Embodiment-PDR.txt)
  (evidence pin `64abb3cf1c645426b507f6cbe1f0bd297095ee8c`).
- **Precedent:** #134, lock `uuv-dive-resurface-scenario-v1`, in
  [`intrinsic/simulation/gazebo/scenarios/dive_resurface`](../../intrinsic/simulation/gazebo/scenarios/dive_resurface/README.md).

Section numbers below refer to the PDR.

## Context

#43 is the phase 6 reference solution. Its exit gate is "Policy cannot
command actuators; classical behavior remains authoritative" (PDR §14). The
PDR says "The reference solution is simulation-only" (PDR §13) and sets the
UUV vertical-slice target "Headless dive, transit, avoid, and resurface
scenario completes inside CI budget" (PDR §3). Scenarios are
"Repository-owned textproto/SDF assets; headless fixed-step CI execution"
(PDR §10).

The #43 Definition of Done as filed is:

1. Package assets, persistent services, stateless skills, behavior tree, and
   configs.
2. Demonstrate dive, transit, perception update, safe replan, and surface
   fallback.
3. Launch through the existing runtime in sim mode.
4. Do not enable real hardware by default.

Items 1 and 3 need architecture that is not locked:

- There is no marine runtime profile. PDR §15 lists
  `intrinsic_runtime/profiles/marine` as a proposed path, and ADR 0001 says
  the §15 paths "are not created here". `intrinsic_runtime` has no
  `profiles` tree on the base commit.
- There are no UUV service assets, skills, or behavior tree. ADR 0001 records
  that estimators, world fusion, policy execution, and recording are
  persistent service assets and that skills are stateless thin clients
  (PDR §2), but no manifest or skill contract for them exists.
- There is no real `EstimatorService`. The error-state filter pieces
  (#81–#90) are functions, not a service, and their propagation needs IMU.
- There is no thrust-input path into simulation. No wrench or thruster
  applicator plugin exists. The #122 Physics-consumption follow-ups are not
  filed. #238 merged as `40e8d231`, but it does not provide a thrust-input
  contract or a closed-loop dynamics proof.

What does exist on the base commit:

- #134 runs an in-process `gz::sim::Server` inside a `cc_test` with a
  kinematic mover, four sensor plugins and `FakeEstimatorService`, and checks
  that a second run is bit-identical. Its README states that it "does not
  prove dynamics or navigation accuracy".
- The service interfaces and fakes the reference solution composes:
  `EstimatorService` and `FakeEstimatorService` (#29,
  `intrinsic/estimation`), `WorldSnapshotBuilder` (#113,
  `intrinsic/world/world_snapshot`), and `InferenceClient` (#37,
  `intrinsic_inference/queue`).
- The vehicle state space (#33,
  `intrinsic_motion_planning/intrinsic/motion_planning/vehicle/vehicle_state_space.h`)
  with a plain-value `Interpolate`, and the planner registry, deadline
  harness and replan fixture (#103–#109).
- Safety rules, the rule aggregator, and the authority and decision-kind
  tables (#91–#100, `intrinsic/safety`).

PDR §18 and the epic rule require a PDR or ADR update for any scope change.
This ADR is that update for #43.

## Decision

### Summary

| Id | Decision | Pick |
| --- | --- | --- |
| D43-1 | Meaning of #43 DoD items 1 and 3 for v1 | **A.** In-process, headless, fixed-step harness. Runtime packaging is a staged follow-on. |
| D43-2 | How the vehicle moves | **A.** Kinematic trajectory playback. No physics-driven motion. |
| D43-3 | Estimator | **A.** `FakeEstimatorService`. |
| D43-7 | Where the reference solution and mission logic live | **A.** Private sim-only scenario package. Filter and plan adapter in their owning packages. |

D43-4, D43-5 and D43-6 are open. See [Open decisions](#open-decisions).

### D43-1: composition and staged packaging

The v1 reference solution is an in-process, headless, fixed-step harness:

- A `gz::sim::Server` runs inside a Bazel `cc_test`, as #134 does. Sim time
  is the only clock. The harness advances one fixed step at a time and reads
  no wall clock.
- "Persistent services" in v1 are the existing service interfaces with their
  fakes or plain implementations, composed in process. No service is
  deployed, and no new service interface is added by this decision.
- "Behavior tree" in v1 is a deterministic mission sequencer with
  behavior-tree-shaped nodes, such as sequence and fallback. It is
  single-threaded and reads only sim time. Its phases, tick rate and
  fallback branch are locked by the sequencer leaf (L43-7), not here.
- "Configs" are repository-owned SDF and text assets in the scenario package
  (PDR §10), parsed by tests.

Flowstate service assets, skills, a behavior-tree asset, service manifests,
and a marine runtime profile under `intrinsic_runtime` are **not** part of
v1. They are a staged follow-on work package. That work package needs its
own ADR before any leaf is filed, because it would create the marine profile
architecture that PDR §15 only proposes.

### Amended #43 Definition of Done

Items 1 and 3 are amended. Items 2 and 4 and the regression guard are
unchanged.

| # | As filed | Amended for v1 |
| --- | --- | --- |
| 1 | Package assets, persistent services, stateless skills, behavior tree, and configs. | Package the scenario assets and configs in the reference scenario package. Compose the existing service interfaces with their fakes in process. Implement the mission as a deterministic sequencer with behavior-tree-shaped nodes. Flowstate assets, skills, a behavior-tree asset and a marine profile are a staged follow-on (ADR 0002). |
| 2 | Demonstrate dive, transit, perception update, safe replan, and surface fallback. | Unchanged. |
| 3 | Launch through the existing runtime in sim mode. | Run headless as an in-process, fixed-step `gz::sim::Server` inside a Bazel `cc_test`. Launch through `intrinsic_runtime` in sim mode is the staged follow-on (ADR 0002). |
| 4 | Do not enable real hardware by default. | Unchanged. |

### D43-2: kinematic trajectory playback

The vehicle pose follows the accepted `VehicleTrajectory`, sampled with the
vehicle state-space `Interpolate`
(`intrinsic::motion_planning::vehicle::Interpolate` in
`vehicle_state_space.h`). There is no Physics system, hydrodynamics or
thrust input in the loop that moves the vehicle. Any controller or allocator
output that a later leaf computes in the scenario does not drive motion.

Reasons:

- Force-driven motion needs a thrust-input contract, a wrench or thruster
  applicator, and verified Physics consumption. None is locked or proven on
  the base commit.
- A test-local twist integrator would be body-state time integration, which
  the vehicle README lists as out of scope, and would need its own numerics
  lock.
- Playback reuses a locked, plain-value API. #134 already uses the same
  kinematic approach.

The step and timing rules of the playback system are locked by its leaf
(L43-3), following the #134 sim-time-derived step rule.

### D43-3: estimator

The reference solution uses `FakeEstimatorService` (#29), as #134 does. Its
estimator mode is the fake's documented mode rule, not a navigation filter
output. An ESKF-backed `EstimatorService` is a new service API plus a
numerics lock, and its propagation needs IMU. It is deferred to a post-#238
work package.

### D43-7: location and ownership

| Piece | Location | Why |
| --- | --- | --- |
| Scenario assets, config, trajectory-playback system, mission sequencer, end-to-end test | New private package `intrinsic/simulation/gazebo/scenarios/uuv_reference/`. Sim and test only. | `intrinsic_simulation` owns deterministic scenario configuration (PDR §4.1). `intrinsic_vehicle` must not own mission behavior (PDR §4.1). A new top-level mission tree would be unlocked architecture. |
| Reference safety filter (L43-1) | `intrinsic/safety` | `intrinsic_safety` owns deterministic rules and the authority state machine (PDR §4.1). #44 reuses the filter. |
| `DesiredMotion` to plan-request adapter (L43-2) | `intrinsic_motion_planning/intrinsic/motion_planning/vehicle` | Motion planning owns the vehicle state space, constraints, trajectory and planner registry (PDR §4.1). #44 reuses the adapter. |

Edits outside the scenario package are limited to what each leaf's lock
allows, such as additive visibility lines, as in #134. The sequencer stays
in the scenario package and is not a reusable mission API.

### Open decisions

These are not decided by this ADR. S43-1 owns them.

| Id | Question |
| --- | --- |
| D43-4 | How a perception update changes the plan. |
| D43-5 | Which conditions trigger surface fallback, and how authority changes. |
| D43-6 | What the reference safety filter may project, and what it rejects. |

#44 decisions (D44-1 to D44-7) are separate and are not part of this ADR.

### Safety and authority

This ADR does not change the safety path, the authority transition table,
or the decision-kinds-under-authority table recorded in ADR 0001 and #92.
The reference safety filter (L43-1) and the surface-fallback wiring (L43-6)
are locked by their own leaves, after S43-1 settles D43-5 and D43-6. The
phase 6 gate still holds: "Policy cannot command actuators; classical
behavior remains authoritative" (PDR §14). Policy execution is #44's scope.

### Regression guard

- The protected manipulator baseline in #13 and the manipulator goldens in
  #20 remain the gates.
- Every reference-solution target is opt-in, test-only or private. None is
  added to `.github/baseline/manipulator_targets.tsv`.
- No target in the reference solution depends on a HAL driver, socket or
  hardware path. Real hardware is not enabled by default (#43 DoD item 4).
- Rollback removes the reference scenario package and its additive
  visibility lines. It needs no data migration.

## Consequences

- #43 can be completed in CI without a marine runtime profile, a cluster, or
  new service manifests.
- #43 DoD items 1 and 3 are narrower than filed. Full runtime packaging is
  deferred, not dropped. It needs its own ADR and work package.
- The reference solution proves orchestration, gating, determinism and the
  safety and planning chain with fakes. It does not prove dynamics or
  navigation accuracy. The scenario README and integration docs must say so,
  as #134's does.
- Closed-loop, force-driven motion and an ESKF-backed estimator service wait
  for a post-#238 work package with a thrust-input contract and the #122
  Physics-consumption follow-ups.
- The safety filter and plan adapter live in their owning packages, so #44
  can reuse them without depending on the scenario package.
- The mission sequencer lives in a sim-only scenario package. Any later
  production mission or behavior-tree layer will be designed in the staged
  packaging work package. It does not graduate from this sequencer.
- Nothing in this ADR adds code, targets, protos, assets, manifests or CI.
  The manipulator baseline is unchanged.
