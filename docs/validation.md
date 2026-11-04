# Executed validation report

Validation ran on **2026-09-23** in this workspace. These are actual Isaac Sim/PhysX and ROS 2 executions, not estimated results.

## Environment and source integrity

- Ubuntu 24.04.4, system Python 3.12.3, ROS 2 Jazzy.
- Isaac Sim `6.0.0-rc.59+release.41464.5f2772bc.gl`, bundled Python 3.12.13.
- RTX 5070 Ti, 16 GB VRAM, NVIDIA driver 595.84.
- CPU PhysX TGS at 240 Hz; RTX RGB rendering; native system `rclpy` in Isaac.
- All **15 original model files** match their recorded SHA-256 checksums.
- Native USD audit: metres, Z up, 3 revolute joints, 12 spherical joints, no articulation roots, 11 converted CAD visual meshes. Collision geometry is separate from CAD triangle visuals.

Evidence: [environment manifest](../artifacts/inspection/environment.json), [Isaac/ROS probe](../artifacts/inspection/isaac_probe.json), [CAD inventory](../artifacts/inspection/cad_inventory.json), [scene audit](../artifacts/inspection/scene_audit.json).

GUI application startup and rendering were also smoke-tested for 2 simulation seconds, producing 20 RGB frames without issuing any grasp requests: [GUI runtime summary](../artifacts/gui_smoke/simulation.json). The autonomous campaigns below ran headless.

## Incremental execution

| Run | Sorting attempts | Physical successful picks | Correct bin placements | Failures |
| --- | ---: | ---: | ---: | ---: |
| Initial single red-cap cycle | 1 | 1 | 1 | 0 |
| Repeated nominal sorting run | **20** | **20** | **20** | **0** |
| Deliberately failed vacuum, including one seal retry | 1 | 0 | 0 | 1, injected |

The nominal run sorted five red, five blue, five green and five yellow caps. Detected picking centers ranged from **-90.93 to +17.24 mm in X** and **-28.90 to +25.09 mm in Y**. Cap yaw, initial small tilt and lateral lane positions varied, using seed 17. The belt physically fed successive caps and stopped for every pick.

That run executed 95,234 main-loop physics steps, published 3,969 RGB frames, and made 20 vacuum attachment requests. It ended at **396.817 simulation seconds**, taking **597.851 wall seconds excluding application startup**. Some negative tests ran concurrently on a different ROS domain, so this is not a dedicated performance benchmark.

Results are evaluated using the physically held cap and its final position, not the coordinator's intended bin. The evaluator requires the cap to rise above the belt, then settle inside the correct bin with center clearance for its radius. Its ground-truth channel is isolated from all perception and task inputs.

Evidence: [single-cycle results](../artifacts/single_pick/results.json), [20-attempt results with individual placements](../artifacts/twenty_attempts/results.json), [task events](../artifacts/twenty_attempts/events.jsonl), [simulation counters](../artifacts/twenty_attempts/simulation.json). Sample RGB images are in each run directory.

Across the three sorting runs in the table, **22 target-selection/pick-and-place attempts were executed: 21 successful picks and 21 correct placements, plus one deliberately failed attempt**. The failure run issued two seal requests within that one attempt. Empty-cup service tests are separate actuator checks and are not counted as sorting attempts.

## Negative and recovery checks

| Check | Executed outcome | Evidence |
| --- | --- | --- |
| Target at `[1,0,-0.35]` m | Motion action rejected before commanding movement | [interlock results](../artifacts/edge_tests/interlocks/checks.json) |
| NaN target | Motion action rejected | same interlock results |
| Empty cup over empty workcell | Grasp service reported failure | same interlock results |
| Latched stop | Conveyor start and new motion rejected | same interlock results |
| Reset after stop | Stop cleared; ROS-driven motion completed with measured TCP error 0.000001822 m | same interlock results |
| Empty belt, autonomous state machine | 0 attempts, 0 grasp requests, conveyor stopped after timeout | [empty run](../artifacts/edge_tests/empty/results.json) |
| Gray, uncertain-color caps in rendered camera images | 0 attempts, 0 grasp requests, clean timeout | [uncertain run](../artifacts/edge_tests/uncertain/results.json) |
| Injected failed vacuum on a real visible cap | One retry, failed attempt reported, vertical recovery lift, return home, feed resumed, then safe completion | [failure run](../artifacts/edge_tests/failed_grasp/results.json), [events](../artifacts/edge_tests/failed_grasp/events.jsonl) |

The micrometre-scale reset tracking result is numerical simulation tracking under this model; it is **not** a claim about real robot or real camera accuracy. An early version of the interlock test harness exited the simulator on its intentional stop event before the reset move finished. The corrected live test keeps the simulator running through reset; the linked results are from that corrected test.

Reproduce the live checks after building:

```bash
./scripts/validate_edges.sh
```

This uses domain 43 by default and writes `artifacts/edge_tests/`. Set `DELTA_TEST_DOMAIN` to another unused ROS domain if needed. No other running ROS workcell is commanded by this test.

## Kinematics, perception and asset tests

**23 automated tests passed.** They cover source-reference six-rod closure, random FK/IK round trips and rod-length residuals, invalid/unreachable coordinates, all four bin routes, joint rate/acceleration limits, belt/bin tool-clearance rejection, four colors at multiple image positions, empty/gray/purple images, same-color overlap, different-color overlap, split-color ambiguity, cropped borders, and calibrated ray/plane projection.

An initial different-color overlap test exposed a round foreground cap being accepted while the adjacent crescent was rejected. The detector now checks nearby pixels from all other color masks, including rejected fragments; the regression test passes. These synthetic tests supplement the actual rendered-image sorting run; they are not a substitute for it.

The workspace sampler accepted 14,408 and rejected 15,710 points on a 30,118-point grid under the commissioned bounds and joint limits. Those accepted points are **IK-feasible points**, not a blanket collision-free workspace. Complete motion paths still undergo the fixed-cell clearance checks. An earlier wider bin layout exceeded the assumed shoulder limits and was rejected before physical commissioning.

```bash
python3 -m pytest -q tests
python3 scripts/sample_workspace.py
.venv/bin/python scripts/audit_scene.py
```

Evidence: [test output](../artifacts/inspection/unit_tests.txt), [workspace summary](../artifacts/inspection/workspace.json), [sampled workspace CSV](../assets/generated/workspace.csv).

## Known limits and remaining work

The requested baseline sorting cycle is implemented and executed. Remaining work concerns fidelity, generalization and deployment rather than a missing nominal cycle:

- This is tested with the **installed Isaac 6 release candidate and Jazzy**, not other releases. The installation uses APIs that differ from older Isaac tutorials.
- The supplied CAD has no measured limits or physical material properties. Shoulder limits, masses, body-centered inertias, drive gains and torque limits are assumptions. Gear backlash, flexible links, belt transmission and motor dynamics are omitted. See [model details](robot_model.md).
- The physical closed loops are present, but robot self-collision pairs are filtered and planning checks conservative tool envelopes. Arbitrary cell layouts require a full collision planner and measured joint travel before use.
- Vacuum is an idealized proximity-triggered, breakable fixed joint. It does not model pressure, leaks, porous materials or seal deformation. Caps use solid cylindrical collision proxies instead of hollow shells.
- RGB projection assumes upright caps of known height on a flat belt. Visible ambiguous/overlapping caps are rejected, but fully hidden stacked objects cannot be reliably inferred from this view. Broad lighting/domain randomization and real-camera calibration are unverified.
- The nominal evidence is one 20-attempt seeded run plus the initial single cycle. It does not establish a production success rate, endurance reliability, or performance under arbitrary cap populations. The default scene contains 24 caps; the reported nominal campaign attempts 20.
- Continuous picking from a moving conveyor, pressure-sensor feedback, detailed gear transmission, hardware deployment and safety-rated controls are not implemented. This is a simulation safe-stop mechanism, not a certified industrial safety system.

No implementation or execution blocker remains for the validated stop-and-pick demonstration.

## Follow-up: floating fastener correction

Four CAD fasteners had incorrect visual-body ownership: wrist parts 255/256 were attached to `upper_1`, and elbow parts 259/260 to the platform. Their reference-pose positions looked correct, but they separated from their mounts during movement. The assignments and generated meshes/USD scene are corrected. The asset audit now checks all 24 moving fasteners against the nearest measured mounting joint. Kinematics and collision shapes are unchanged, and all original CAD hashes still match.

An additional full ROS-driven cycle after this correction completed **1 pick and 1 correct placement**, with no failures. Moving-pose camera and overview renders were checked. See the [verification record](../artifacts/floating_component_fix/verification.json) and [corrected moving-pose render](../artifacts/floating_component_fix/cycle/overview_00050.png). Use `--capture-overview` with the launcher to save overview snapshots during future runs.
