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
| Motus Cartesian Pose | Plane (m) | Cartesian TCP goal |

## Collision

| Component | Notes |
|-----------|-------|
| Motus Collision Sphere | Center point + radius (m) |
| Motus Collision Box | Plane + half extents (m) |
| Motus Collision Scene | Merge collision objects |

## Plan

| Component | Notes |
|-----------|-------|
| Motus Plan Joint Path | **Run** or **AutoReplan**; optional collision scene |
| Motus Plan Cartesian Path | IK goal + joint-linear path |
| Motus Plan RRT Connect | RRT-Connect with collision; supports solution cancel |
| Motus Validate Trajectory | Joint limits, velocity, acceleration, optional collision |
| Motus Trajectory Info | Point count, duration, robot name |

## Export

| Component | Output |
|-----------|--------|
| Motus Trajectory to Joint Lists | Times + per-axis joint trees (rad) |
| Motus Trajectory to Planes | FK TCP planes |
| Motus Trajectory to Poses | FK TCP frames |
| Motus Trajectory to JSON | JSON string |
| Motus Trajectory to CSV | CSV string |

## Preview

| Component | Output |
|-----------|--------|
| Motus Preview Robot | Link lines, link meshes, TCP plane |
| Motus Preview TCP Path | FK TCP polyline |
| Motus Preview Trajectory | Start/goal TCP, valid/invalid segments, goal links |

## Data types

Custom Goo wrappers: `RobotModelGoo`, `JointStateGoo`, `TrajectoryGoo`, `FrameGoo`, `ToolFrameGoo`, `BaseFrameGoo`, `CollisionSceneGoo`, `CartesianPoseGoo`.

## Units

Joint inputs: **radians** by default. Enable **UseDegrees** on Motus Joint State for degree input. Planes and preview geometry use **meters**.
