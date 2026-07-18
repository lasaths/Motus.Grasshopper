# Example Grasshopper Definitions

Four lean `.ghx` files cover every Motus component and planner input. Regenerate them after component changes:

```bash
node scripts/generate-examples.mjs
node scripts/validate-ghx.mjs
```

Open any file in Rhino 8 / Grasshopper. **Motus Plan** and **Motus Program** examples ship with **Auto Plan** on. Drag **Motus Scrub** or **Play** on **Motus Preview** after the trajectory appears.

## Example index

| File | What it demonstrates |
|------|----------------------|
| `01_quick_plan.ghx` | Sequential goals (joint + TCP Pose LIN + plane), Preview, Export, Trajectory Data |
| `02_collision_srdf.ghx` | ColSphere + ColBox → ColScene (SRDF) + Group + Attach + RRT Settings → Plan |
| `03_urdf_tool_frames.ghx` | Motus Robot URDF + Base + Robotiq Tool (Load Mesh) + Start + Preview ShowStart |
| `04_motion_program.ghx` | PTP + LIN + CIRC + SET gripper → Motus Program → Preview / Export |

## Component coverage

| Component / option | 01 | 02 | 03 | 04 |
|--------------------|:--:|:--:|:--:|:--:|
| Motus UR10e Robotiq | ✓ | ✓ | | ✓ |
| Motus Robot (URDF Path) | | | ✓ | |
| Motus Joint State | ✓ | ✓ | ✓ | ✓ |
| Motus TCP Pose | ✓ | | | |
| Plane goal (Cartesian LIN) | ✓ | | | ✓ |
| Motus Move | | | | ✓ |
| Motus Program | | | | ✓ |
| Motus Plan — Goal list | ✓ | | | |
| Motus Plan — Start | ✓ | | ✓ | ✓ |
| Motus Plan — Collision | | ✓ | | |
| Motus Plan — Group | | ✓ | | |
| Motus Plan — Attach | | ✓ | | |
| Motus RRT Settings | | ✓ | | |
| Motus Collision Sphere | | ✓ | | |
| Motus Collision Box | | ✓ | | |
| Motus Collision Mesh | | *(note)* | | |
| Motus Collision Scene | | ✓ | | |
| ColScene SRDF | | ✓ | | |
| Motus Planning Group | | ✓ | | |
| Motus Attach Body | | ✓ | | |
| Motus Tool | | | ✓ | |
| Motus Load Mesh | | | ✓ | |
| Motus Tool State | | | | ✓ |
| Motus Preview | ✓ | ✓ | ✓ | ✓ |
| Preview ShowStart | | | ✓ | |
| Motus Export | ✓ | | ✓ | ✓ |
| Motus Trajectory Data | ✓ | | | |
| Robot Base / Tool override | | | ✓ | |

**Col Mesh:** wire any Rhino mesh or Brep into **Motus Collision Mesh** the same way **02** wires sphere + box into **ColScene** `Objects`.

**Degrees** on **Motus Joint State:** right-click the **J** input and toggle **Degrees**; examples use radians by default.

**Plan advanced inputs:** Collision / Group / Attach / RrtSettings are hidden by default. Right-click Motus Plan → Show Collision (etc.), or open **02** which already includes those pins.

**Tool / ToolState:** Motus Tool has an explicit **Cap** dropdown (`None` / `Robotiq2F85`). Motus Tool State accepts **Tool or Robot** on `Tl` (UR10e bundled tool works when you wire the robot). Gripper SET/WAIT uses Motus Program + Motus Move, not Motus Plan.

**Preview:** Trajectory list from Motus Plan concatenates sequential goals. Debug outputs (Index / Invalid / ToolState / Width) are behind right-click → Show debug outputs.

## Typical flows

### Quick plan (01)

```
UR10e + Start ─┐
Joint State ───┼→ Plan.Goal (list) [Auto Plan] → Preview / Export / TrajData
TCP Pose ──────┤
Plane ─────────┘
```

### Collision + SRDF (02)

```
ColSphere / ColBox → ColScene (+ SRDF) → Plan.Collision
Joint State → Plan.Goal
Group / Attach / RrtSettings → Plan advanced pins
```

### URDF + tool frames (03)

```
URDF Path + Base + Tool (Load Mesh / Cap) → Motus Robot
Start + Goal → Plan [Auto Plan] → Preview (ShowStart) / Export
```

### Motion program (04)

```
Robot + Joint States / Planes / ToolState → Motus Move (PTP/LIN/CIRC/SET ▾) ─┐
                                                                              ├→ Motus Program [Plan] → Preview / Export
Start (optional) ─────────────────────────────────────────────────────────────┘
```

## SRDF

`examples/srdf/table_base.srdf` disables checks between `link:0` and obstacle `table`, and defines a `manipulator` planning group. In **02**, edit the **Srdf** panel if the relative path does not resolve (ColScene walks up from the Grasshopper working directory, same as Motus Robot Path).

## URDF preview notes

- `Motus Robot` feeds visual geometry into **Motus Preview** when supported (`box`, `cylinder`, `sphere`, `.stl`).
- `.dae` visuals are skipped; use `*_minimal.urdf` for reliable in-app preview without external meshes.
- URDF assets in `examples/ur10e/` — see that folder’s README. Run `node scripts/fetch-ur10e-assets.mjs` for arm + Robotiq meshes.

## Editing

Re-save from Grasshopper after tweaks so the archive matches your installed component version. Prefer editing `scripts/generate-examples.mjs` for structural changes, then re-run the generator.

External plugin workflows (UR RTDE, VirtualRobot, etc.): [docs/external-plugin-workflows.md](../docs/external-plugin-workflows.md).
