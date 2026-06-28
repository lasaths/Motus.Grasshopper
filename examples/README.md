# Example Grasshopper Definitions

Two definitions cover the whole 0.3 palette. They ship pre-wired with sensible
default values; open one in Rhino 8 / Grasshopper, press **Plan**, and (after a
quick look) **Save** to re-bake them against your installed component version.

| File | Workflow |
|------|----------|
| `01_basic_planning.ghx` | Robot + joint goal -> Plan -> Preview + Export |
| `02_collision_planning.ghx` | Obstacle -> Scene -> Plan (RRT) -> Preview |

## Minimal workflow (01)

```
Motus Robot (UR5e) ----\
Motus Joint State -------> Motus Plan [Plan] --> Trajectory --> Motus Preview [Play]
                                                            \--> Motus Export (Json / Csv)
```

1. **Motus Robot** — pick `UR5e` from the dropdown. (Or drop a Rhino **Plane** into `Goal` for a Cartesian target instead of a joint state.)
2. **Motus Plan** — leave `Start` unwired (defaults to home), click **Plan**.
3. **Motus Preview** — wire `Trajectory`, click **Play** to animate.
4. **Motus Export** / **Motus Trajectory Data** — hang off the same `Trajectory`.

## Collision-aware (02)

Add **Motus Collision Sphere** / **Box** → **Motus Collision Scene**, wire the scene
into **Motus Plan** `Collision`. With a joint goal this plans with RRT-Connect.

## Editing / re-saving

The `.ghx` are authored directly against the Grasshopper archive format, so they
open with every node placed, wired, and seeded. If you tweak a graph, just
**File → Save** from Grasshopper — re-saving guarantees the archive matches the
installed component version.

### External plugins (optional)

| File | Needs |
|------|-------|
| `05_external_ur_rtde.gh` | UR.RTDE.Grasshopper |
| `06_external_virtualrobot.gh` | VirtualRobot |
