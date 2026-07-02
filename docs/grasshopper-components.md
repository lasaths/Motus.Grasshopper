# Grasshopper Components

All components live under the **Motus** tab. The palette is intentionally small: pick a robot, give a target, plan, preview, export.

## Model

| Component | Inputs | Outputs |
|-----------|--------|---------|
| Motus Robot | Model name (dropdown) or JsonPath | Robot model |
| Motus Joint State | Joint list, UseDegrees | Joint state |

`Motus Robot` loads a bundled preset by name (dropdown attached automatically) or, when `JsonPath` is wired, from a JSON file. It outputs a ready-to-use robot model directly (no separate wrapping step).

## Plan

| Component | Notes |
|-----------|-------|
| Motus Plan | One planner for all goals. Click the **Plan** button to compute. |

`Motus Plan` inputs: `Robot`, `Goal`, optional `Start`, optional `Collision`.

- `Goal` is either a **Plane** (Cartesian target, solved via IK) or a **Joint State**.
- `Start` is optional; unwired it defaults to the robot home (all-zeros joint state).
- The planner is inferred from the inputs:
  - `Goal` is a plane -> Cartesian linear plan (IK).
  - `Goal` is joints + `Collision` wired -> RRT-Connect.
  - `Goal` is joints, no collision -> joint-linear plan.
- `Status` reports success, errors, or validation warnings. Toggling inputs does not replan; press **Plan** again.

## Collision

| Component | Notes |
|-----------|-------|
| Motus Collision Sphere | Center point + radius (m) |
| Motus Collision Box | Plane + half extents (m) |
| Motus Collision Mesh | Rhino mesh obstacle (meters) |
| Motus Collision Scene | Merge collision objects; optional **Srdf** path for allowed pairs (`link:N` or obstacle names) |

## Preview

| Component | Notes |
|-----------|-------|
| Motus Preview | Animated FK preview with a built-in **Play / Stop** button |

`Motus Preview` takes a `Trajectory` and outputs link `Meshes` and `Links` at the current playback frame, the full `TCP Path` polyline, the `State` / `Time` / `Index` at the playhead, and `Invalid` TCP segments (joint/velocity/acceleration limits). Base and tool frames come from the preset.

## Export

| Component | Output |
|-----------|--------|
| Motus Trajectory Data | TCP `Planes`, waypoint `Times`, per-axis `Joints` tree |
| Motus Export | `Json` and `Csv` strings |

Point count is the length of `Times`; duration is the last `Times` value (native Grasshopper list ops).

## Units

Joint inputs: **radians** by default. Enable **UseDegrees** on Motus Joint State for degree input. Planes and preview geometry use **meters**.
