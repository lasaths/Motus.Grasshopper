# Example Grasshopper Definitions



Twelve focused `.ghx` files cover every Motus component and planner input. Regenerate them after component changes:



```bash

node scripts/generate-examples.mjs

node scripts/validate-ghx.mjs

```



Open any file in Rhino 8 / Grasshopper, click **Plan** on **Motus Plan** or **Motus Program Plan**, then **Play** on **Motus Preview**.



## Example index



| File | What it demonstrates |

|------|----------------------|

| `01_joint_planning.ghx` | Preset robot, joint goal, joint-linear plan, Preview, Export, Trajectory Data |

| `02_cartesian_planning.ghx` | Joint State → TCP Pose (FK) → Cartesian LIN plan |

| `03_collision_rrt.ghx` | ColSphere → ColScene → Plan.Collision (RRT-Connect) |

| `04_collision_shapes.ghx` | ColSphere + ColBox merged in ColScene → RRT |

| `05_srdf_group_attach.ghx` | SRDF path, Planning Group, Attach Body on Plan |

| `06_urdf_load.ghx` | Motus Load URDF → plan + preview (set Path panel) |

| `07_frames_and_start.ghx` | Base override + Motus Tool on Robot, Plan Start, Preview ShowStart |

| `08_motion_program.ghx` | PTP + LIN + CIRC segments → Program Plan → Preview / Export |
| `09_tool_tcp.ghx` | Motus Tool (TCP + gripper box) → Robot.Tool → Plan → Preview / Export |
| `10_robotiq_tool.ghx` | Robotiq 2F-85 STL → Load Mesh → Motus Tool → UR10e Plan + Preview |
| `11_gripper_motion_program.ghx` | PTP + SET gripper close → Program Plan → Preview / Export (toolState on trajectory) |
| `12_sequential_goals.ghx` | Multiple goals (Joint State + Planes) on Plan.Goal list → chained trajectory → Preview / Export |



## Component coverage



| Component / option | 01 | 02 | 03 | 04 | 05 | 06 | 07 | 08 | 09 | 10 | 11 | 12 |

|--------------------|:--:|:--:|:--:|:--:|:--:|:--:|:--:|:--:|:--:|:--:|:--:|:--:|

| Motus Robot (preset dropdown) | ✓ | ✓ | ✓ | ✓ | ✓ | | ✓ | ✓ | | | ✓ | ✓ |

| Motus Load URDF | | | | | | ✓ | | | ✓ | ✓ | | |

| Motus Joint State | ✓ | | ✓ | ✓ | ✓ | ✓ | ✓ | ✓ | ✓ | ✓ | ✓ | ✓ |
| Motus TCP Pose | | ✓ | | | | | | | | | | |

| Joint validation (Robot on Joints) | ✓ | | ✓ | ✓ | ✓ | ✓ | ✓ | ✓ | | | | |

| Plane goal (Cartesian LIN) | | ✓ | | | | | | ✓ | | | | ✓ |

| Motus Motion Segment | | | | | | | | ✓ | | | ✓ | |

| Motus Program Plan | | | | | | | | ✓ | | | ✓ | |

| Motus Plan — Goal list | | | | | | | | | | | | ✓ |

| Motus Plan — Start | | | | | | | ✓ | ✓ | | | ✓ | ✓ |

| Motus Plan — Collision | | | ✓ | ✓ | ✓ | | | | | | | |

| Motus Plan — Group | | | | | ✓ | | | | | | | |

| Motus Plan — Attach | | | | | ✓ | | | | | | | |

| Motus Collision Sphere | | | ✓ | ✓ | ✓ | | | | | | | |

| Motus Collision Box | | | | ✓ | | | | | | | | |

| Motus Collision Mesh | | | | *(note)* | | | | | | | | |

| Motus Collision Scene | | | ✓ | ✓ | ✓ | | | | | | | |

| ColScene SRDF | | | | | ✓ | | | | | | | |

| Motus Planning Group | | | | | ✓ | | | | | | | |

| Motus Attach Body | | | | | ✓ | | | | | | | |

| Motus Preview | ✓ | ✓ | ✓ | ✓ | ✓ | ✓ | ✓ | ✓ | ✓ | ✓ | ✓ | ✓ |

| Preview ShowStart | | | | | | | ✓ | | | | | |

| Motus Export | ✓ | | | | | | | ✓ | ✓ | ✓ | ✓ | ✓ |

| Motus Trajectory Data | ✓ | | | | | | | | | | | |

| Robot Base / Tool override | | | | | | | ✓ | | ✓ | ✓ | | |

| JsonPath on Robot | | | | | | | *(unwired)* | | | | | |



**Col Mesh:** wire any Rhino mesh or Brep into **Motus Collision Mesh** the same way **04** wires sphere + box into **ColScene** `Objects` (list input accepts multiple sources).



**JsonPath:** optional on **Motus Robot** — point at a custom preset JSON to override the dropdown model.



**Degrees** on **Motus Joint State:** right-click the **J** input and toggle **Degrees**; examples use radians by default.



## Typical flows



### Cartesian LIN (02)



```

Value List → Robot ─┬→ TCP Pose ← Joint State (goal joints)

                    └→ Plan [Plan] ← TCP Pose.Plane

Plan → Preview [Play]

```



### Joint-linear (01)



```

Value List → Robot ─┐

Joint State (Rb) ───┼→ Plan [Plan] → Preview [Play]

                    ├→ Export (Json / Csv)

                    └→ Trajectory Data

```



### Sequential goals — Plan list (12)



```

Robot + Start (optional) ─┐

Joint State / Planes ───┼→ Plan.Goal (list) [Plan] → Preview [Play] / Export

                          └→ wire multiple sources into Goal, or use Merge

```



For PTP/LIN/CIRC arcs, gripper SET, and WAIT dwells, use **Motion Segment** nodes and **Program Plan** instead (`08_motion_program.ghx`, `11_gripper_motion_program.ghx`).



### Motion program (08)



```

Robot + Joint States / Planes → Motion Segment (PTP/LIN/CIRC) ─┐

                                                                ├→ Program Plan [Plan] → Preview / Export

Start (optional) ───────────────────────────────────────────────┘

```



### Collision RRT (03–05)



```

ColSphere / ColBox / ColMesh → ColScene → Plan.Collision

Joint State → Plan.Goal

(optional) SRDF panel → ColScene → Group → Plan.Group

(optional) ColObject → Attach → Plan.Attach

```



### URDF (06)



URDF assets in `examples/ur10e/` — see that folder’s README. Run `node scripts/fetch-ur10e-assets.mjs` for arm + Robotiq meshes. Use `ur10e_minimal.urdf` for CI/smoke, `ur10e_robotiq.urdf` for arm+gripper with local mesh paths, or `ur10e.urdf` for arm only.


## SRDF



`examples/srdf/table_base.srdf` disables checks between `link:0` and obstacle `table`, and defines a `manipulator` planning group. In **05**, edit the **Srdf** panel if the relative path does not resolve (ColScene walks up from the Grasshopper working directory, same as Load URDF).



## URDF preview notes



- `Motus Load URDF` feeds visual geometry into **Motus Preview** when supported (`box`, `cylinder`, `sphere`, `.stl`).

- `.dae` visuals are skipped; use `*_minimal.urdf` for reliable in-app preview without external meshes.



## Editing



Re-save from Grasshopper after tweaks so the archive matches your installed component version. Prefer editing `scripts/generate-examples.mjs` for structural changes, then re-run the generator.



`03_collision_rrt.ghx` is **hand-tuned** (Populate 3D / Center Box obstacle layout). `node scripts/generate-examples.mjs` skips overwriting it. Pass `--force-hand-tuned` only if you intentionally want the generator template back.



External plugin workflows (UR RTDE, VirtualRobot, etc.): [docs/external-plugin-workflows.md](../docs/external-plugin-workflows.md).

