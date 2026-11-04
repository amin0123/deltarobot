# Source model, mechanical interpretation and kinematics

## Source audit

`Delta+Robot` contains one Autodesk-exported STEP AP214 assembly (`Delta_assembly.step`, dated 2021-10-29) and 14 binary STL component meshes: corner bracket, center bracket, gear, gear bracket, back/front/bottom motor-mount parts, end-effector plate, rod end, servo mount, gripper slide, gripper, gripper arm, and rotation bracket. There is no URDF, SDF, USD, actuator specification, controller, joint-limit table, or exported kinematic joint definition.

The STEP inventory contains 149 product definitions and 368 assembly occurrences; OpenCascade resolves 309 positioned leaf instances. The full tessellation has 3,006,548 triangles. The scene's derived visuals have 661,912 triangles. Conversion preserves the assembly transforms and millimetre scale before converting coordinates to metres. The standalone STLs are in part-local coordinates and are not a substitute for the positioned assembly.

The complete names, mesh bounds, analytic cylinder/sphere surfaces, triangle counts, source hashes and assembly hierarchy are in `artifacts/inspection/cad_inventory.json`. `cad_parts.txt` is the readable instance listing. `robot_geometry.json` records which original leaf instances form each physical link. Original files are read only by all project scripts.

The STEP supplies 30 named RGB appearance entries, including matte black/white plastic, glossy blue plastic, satin steel/aluminum, chrome, red/yellow enamel paint, soft rubber, and polished stainless steel. Their names and exact RGB values are in `artifacts/inspection/source_materials.json`. These are appearance labels, with no validated masses, friction, density, joint limits or actuator ratings. The runtime uses neutral robot materials, colored caps/bins, and explicit assumed physics properties. It is a functional sorting simulation, not an actuator-sizing or fatigue model.

## Measured geometry

The ball centers are extracted from the STEP's spherical faces, and shoulder axes from the 6.25 mm radius gear-bearing cylinders. Each shoulder center is the point on its bearing axis aligned with the midpoint of its two elbow balls.

| Quantity | Measured value |
| --- | --- |
| Upper-arm pivot-to-elbow length, all three legs | 0.1779954352256 m |
| Lower-rod ball-center distance, all six rods | 0.2450400000003 m |
| Shoulder-circle radius | approximately 0.0817943 m |
| Reference upper-arm angle | 0.87263676 rad, approximately 50 degrees below horizontal |
| Reference moving-platform position | `[0, 0, -0.306337716]` m in `base_link` |
| Added vacuum TCP offset from platform origin | `[0, 0, -0.038]` m |
| Conservative assumed shoulder limits | 5 to 85 degrees |

The STEP assembly center is `[-111.0370019341, 15.4855858985, 71.0229567329]` mm. The conversion is:

```text
base_x =  (cad_x + 111.0370019341) / 1000
base_y =  (cad_z -  71.0229567329) / 1000
base_z = -(cad_y -  15.4855858985) / 1000
```

This is a proper rotation, not a reflection. The source hangs along CAD +Y; the converted robot hangs along world -Z. Shoulder pivots in metres are approximately:

```text
shoulder_0 = [-0.07086836, -0.04084099, 0]
shoulder_1 = [ 0.07080352, -0.04095330, 0]
shoulder_2 = [ 0.00006484,  0.08179430, 0]
```

Small assembly offsets are retained. Joint axes are the in-plane tangents `a_i = Z × r_i`, where `r_i` is the outward horizontal direction of the corresponding upper arm. Positive joint rotation lowers that arm. ROS joint angles are absolute angles below horizontal; the USD joint's zero is the CAD reference pose, so drive targets subtract the reference angle.

## Physical mechanism and approximations

The scene uses three driven ordinary `UsdPhysics.RevoluteJoint`s, six dynamic lower rods, a dynamic moving plate, and **12 passive `UsdPhysics.SphericalJoint`s**. All rod joints close through the same moving plate. This is a maximal-coordinate rigid-body mechanism, with no reduced-coordinate articulation tree and no fake Cartesian robot. The solver enforces the closed loops, including the paired-rod parallelograms that constrain platform orientation. Only the shoulder drives receive motion commands. The platform and attached caps are never repositioned during motion.

This representation avoids the tree restriction on reduced-coordinate articulations. NVIDIA documents the [closed-loop restriction and regular-joint closure approach](https://docs.omniverse.nvidia.com/kit/docs/omni_physics/110.0/dev_guide/rigid_bodies_articulations/articulations.html). A 240 Hz CPU PhysX TGS solver and increased position iterations keep the small linkage stable.

Simulation approximations are explicit:

- Gear belts and bearing internals are rigidly merged; backlash, gearing compliance, motor electrical dynamics and rotor motion are omitted. Joint commands act at the shoulder output shaft.
- Each upper assembly uses 0.22 kg, each lower rod assembly 0.022 kg, and the platform/cup 0.16 kg. Stabilizing diagonal inertias and body-centered mass frames are assumed, not derived from material density. Upper-body mass is centered at its pivot in this baseline. This limits the accuracy of gravity torque and inertial load prediction.
- Original CAD visuals are retained for the frame/drive hardware, geared upper assemblies, six rod assemblies and moving plate. The servo/finger gripper, cable routing and unidentified final solid are omitted; their IDs remain in the conversion manifest.
- The plate uses a cylinder collision proxy, upper arms use capsules, and lower rods use thin capsules. Robot self-collision pairs are filtered to keep adjacent ball joints and imperfect CAD clearances from fighting the constraints. The fixed CAD base is visual; the added rigid support frame has box collisions. This is not a full self-collision or gear-tooth interference study.
- A small vacuum cup replaces the original finger mechanism. A proximity/orientation check creates a breakable physical fixed joint without snapping either body. The joint's local frames are computed from the measured poses at attachment. Disabling vacuum removes the joint; gravity and contact determine the subsequent fall.
- Caps use solid convex cylinder collision proxies, 28 mm diameter, 12 mm height, 2.2 g mass, friction 0.45 and restitution 0.03. They represent upright closed-side-up caps. Hollow internal surfaces and pressure dynamics are omitted.
- The conveyor is a fixed kinematic collider with PhysX surface velocity. Friction moves and stops the free caps. No per-cap belt-following transforms are used.

## Kinematic equations

For leg `i`, let `B_i` be the shoulder, `r_i` its outward horizontal unit vector, `u_i` the upper-arm length, `l_i` the lower-rod length, and `o_i` the midpoint of its two platform attachment offsets. With `d = [0,0,-1]`:

```text
E_i(q_i) = B_i + u_i (r_i cos(q_i) + d sin(q_i))
P = TCP - tool_offset
||P + o_i - E_i(q_i)||² = l_i²
```

Writing `D_i = P + o_i - B_i` gives `A cos(q) + B sin(q) = C`, where `A=D·r`, `B=D·d`, and `C=(D·D+u²-l²)/(2u)`. The implemented inverse solution chooses the outward-elbow assembly branch and rejects non-real solutions, limits, and near-singular transmission. Forward kinematics intersects the three spheres centered at `E_i-o_i` and selects the lower solution.

The motion action samples the complete Cartesian segment, solves IK at every sample, applies quintic time scaling, and lengthens duration to meet the configured joint velocity and acceleration limits. Completion requires both physical tool-position feedback and measured joint tracking. The planner checks a conservative fixed-cell tool envelope around the belt and bins; it is not a general mesh collision planner.

The commissioned envelope is radius 185 mm and TCP Z between -385 and -280 mm, **further restricted by IK and all three joint limits**. This radius/height box alone is not a reachable-workspace guarantee. Current bin centers at `(±60, ±110)` mm are validated. A proposed `(±80, ±140)` mm layout was rejected by IK and is not used.
