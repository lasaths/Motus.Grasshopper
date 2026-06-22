# Grasshopper Components

All components live under the **Motus** tab.

## Model

| Component | Inputs | Outputs |
|-----------|--------|---------|
| Motus UR Preset | Model name | Preset |
| Motus KUKA Preset | Model name | Preset |
| Motus Custom Robot | JSON path | Preset |
| Motus Robot Model | Preset | Robot model |
| Motus Joint State | Joint list, UseDegrees | Joint state |
| Motus Tool Frame | Plane (m) | Tool frame |
| Motus Base Frame | Plane (m) | Base frame |

## Plan

| Component | Notes |
|-----------|-------|
| Motus Plan Joint Path | **Run** must be true to plan; caches last result when idle |
| Motus Validate Trajectory | Errors and warnings lists |
| Motus Trajectory Info | Point count, duration, robot name |

## Export

| Component | Output |
|-----------|--------|
| Motus Trajectory to Joint Lists | Times + per-axis joint trees (rad) |
| Motus Trajectory to Planes | Simplified TCP planes (placeholder FK) |
| Motus Trajectory to Poses | Motus frames per point |
| Motus Trajectory to JSON | JSON string |
| Motus Trajectory to CSV | CSV string |

## Preview

| Component | Output |
|-----------|--------|
| Motus Preview Robot | Stick-figure link lines for one joint state |
| Motus Preview TCP Path | Polyline from trajectory |
| Motus Preview Trajectory | Start/goal points + goal stick figure |

## Data types

Custom Goo wrappers: `RobotModelGoo`, `JointStateGoo`, `TrajectoryGoo`, `FrameGoo`, `ToolFrameGoo`, `BaseFrameGoo`. Components also expose native GH types (numbers, text, planes, curves) where useful.

## Units

Joint inputs: **radians** by default. Enable **UseDegrees** on Motus Joint State for degree input. Planes and preview geometry use **meters** (Rhino document units should match).
