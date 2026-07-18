# Example Grasshopper Definitions

Twelve focused `.ghx` files cover every Motus component and planner input. Regenerate them after component changes:

```bash
node scripts/generate-examples.mjs
node scripts/validate-ghx.mjs
```

Open any file in Rhino 8 / Grasshopper, click **Plan** on **Motus Plan** or **Motus Program**, then **Play** on **Motus Preview**.

## Example index

| File | What it demonstrates |
|------|----------------------|
| `01_joint_planning.ghx` | UR10e preset, joint goal, joint-linear plan, Preview, Export, Trajectory Data |
| `02_cartesian_planning.ghx` | Joint State → TCP Pose (FK) → Cartesian LIN plan |
| `03_collision_rrt.ghx` | ColSphere → ColScene → Plan.Collision (RRT-Connect) |
| `04_collision_shapes.ghx` | ColSphere + ColBox merged in ColScene → RRT |
| `05_srdf_group_attach.ghx` | SRDF path, Planning Group, Attach Body on Plan |
| `06_urdf_load.ghx` | Motus Robot (URDF Path) → plan + preview |
| `07_frames_and_start.ghx` | Base override + Motus Tool on Robot, Plan Start, Preview ShowStart |
| `08_motion_program.ghx` | PTP + LIN + CIRC Moves → Motus Program → Preview / Export |
| `09_tool_tcp.ghx` | Motus Tool (TCP + gripper box) → Robot.Tool → Plan → Preview / Export |
| `10_robotiq_tool.ghx` | Robotiq 2F-85 STL → Load Mesh → Motus Tool → UR10e Plan + Preview |
| `11_gripper_motion_program.ghx` | PTP + SET gripper close → Motus Program → Preview / Export (toolState on trajectory) |
| `12_sequential_goals.ghx` | Multiple goals on Plan.Goal list → concatenated Preview / Export |

## Component coverage

| Component / option | 01 | 02 | 03 | 04 | 05 | 06 | 07 | 08 | 09 | 10 | 11 | 12 |
|--------------------|:--:|:--:|:--:|:--:|:--:|:--:|:--:|:--:|:--:|:--:|:--:|:--:|
| Motus UR10e Robotiq | ✓ | ✓ | ✓ | ✓ | ✓ | | ✓ | ✓ | | | ✓ | ✓ |
| Motus Robot (URDF Path) | | | | | | ✓ | ✓ | | ✓ | ✓ | | |
| Motus Joint State | ✓ | | ✓ | ✓ | ✓ | ✓ | ✓ | ✓ | ✓ | ✓ | ✓ | ✓ |
| Motus TCP Pose | | ✓ | | | | | | | | | | |
| Plane goal (Cartesian LIN) | | ✓ | | | | | | ✓ | | | | ✓ |
| Motus Move | | | | | | | | ✓ | | | ✓ | |
| Motus Program | | | | | | | | ✓ | | | ✓ | |
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

**Col Mesh:** wire any Rhino mesh or Brep into **Motus Collision Mesh** the same way **04** wires sphere + box into **ColScene** `Objects`.

**Degrees** on **Motus Joint State:** right-click the **J** input and toggle **Degrees**; examples use radians by default.

**Plan advanced inputs:** Collision / Group / Attach / RrtSettings are hidden by default. Right-click Motus Plan → Show Collision (etc.), or open examples 03–05 which already include those pins.

**Tool / ToolState:** Motus Tool has an explicit **Cap** dropdown (`None` / `Robotiq2F85`). Motus Tool State accepts **Tool or Robot** on `Tl` (UR10e bundled tool works when you wire the robot). Gripper SET/WAIT uses Motus Program + Motus Move, not Motus Plan.

**Preview:** Trajectory list from Motus Plan concatenates sequential goals. Debug outputs (Index / Invalid / ToolState / Width) are behind right-click → Show debug outputs.

## Typical flows

### Cartesian LIN (02)

```
UR10e ─┬→ TCP Pose ← Joint State (goal joints)
       └→ Plan [Plan] ← TCP Pose.Plane
Plan → Preview [Play]
```

### Joint-linear (01)

```
UR10e ─┐
Joints ┼→ Plan [Plan] → Preview [Play]
       ├→ Export (Json / Csv)
       └→ Trajectory Data
```

### Sequential goals — Plan list (12)

```
Robot + Start (optional) ─┐
Joint State / Planes ───┼→ Plan.Goal (list) [Plan] → Preview [Play] / Export
                          └→ wire multiple sources into Goal, or use Merge
```

Preview / Export accept the Trajectory **list** and concatenate sequential goals (with a remark/warning when N>1).

### Motion program (08)

```
Robot + Joint States / Planes → Motus Move (PTP/LIN/CIRC ▾) ─┐
                                                              ├→ Motus Program [Plan] → Preview / Export
Start (optional) ─────────────────────────────────────────────┘
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

`examples/srdf/table_base.srdf` disables checks between `link:0` and obstacle `table`, and defines a `manipulator` planning group. In **05**, edit the **Srdf** panel if the relative path does not resolve (ColScene walks up from the Grasshopper working directory, same as Motus Robot Path).

## URDF preview notes

- `Motus Robot` feeds visual geometry into **Motus Preview** when supported (`box`, `cylinder`, `sphere`, `.stl`).
- `.dae` visuals are skipped; use `*_minimal.urdf` for reliable in-app preview without external meshes.

## Editing

Re-save from Grasshopper after tweaks so the archive matches your installed component version. Prefer editing `scripts/generate-examples.mjs` for structural changes, then re-run the generator.

`03_collision_rrt.ghx` is **hand-tuned** (Populate 3D / Center Box obstacle layout). `node scripts/generate-examples.mjs` skips overwriting it. Pass `--force-hand-tuned` only if you intentionally want the generator template back.

External plugin workflows (UR RTDE, VirtualRobot, etc.): [docs/external-plugin-workflows.md](../docs/external-plugin-workflows.md).
