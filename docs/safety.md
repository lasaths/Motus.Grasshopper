# Safety (Grasshopper)

## Planning only

The Motus Grasshopper plugin does **not** connect to or command physical robots. It plans, validates, previews, and exports trajectories.

## Run gate

**Motus Plan** only computes when you click its **Plan** button. Editing inputs does not replan; the last result stays cached and is re-emitted until you press **Plan** again. This avoids accidental re-planning on every Grasshopper solution.

## External control

If you connect Motus exports to UR.RTDE.Grasshopper, VirtualRobot, Robots, or custom scripts:

- You are responsible for safe speeds, workspace limits, and e-stop readiness
- Always verify trajectories in simulation or reduced speed first
- Preset joint limits may not match your controller configuration

## No silent failures

Components emit Grasshopper runtime messages for errors. Warnings (e.g. no collision checking) appear on planning outputs.

See also [Motus.NET safety](../../Motus.NET/docs/safety.md).
