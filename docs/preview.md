# Preview

Motus includes a **basic** Rhino viewport preview — not a replacement for VirtualRobot or Robots.

## What is shown

| Component | Geometry |
|-----------|----------|
| Motus Preview Robot | Stick-figure links from base following joint angles (simplified 2.5D heuristic) |
| Motus Preview TCP Path | Polyline sampling first joint contribution along trajectory |
| Motus Preview Trajectory | Start point, goal point, goal-configuration stick figure |

## Limitations (milestone 1)

- No accurate forward kinematics mesh model yet
- Link lengths are fixed heuristics (~0.12–0.15 m per segment)
- Tool frame and base frame components affect data model but preview uses simplified stick geometry
- Invalid trajectory segments are not yet highlighted in preview

## Tips

- Connect preview components to a **Geometry** pipeline or use Grasshopper preview bubbles
- Keep Rhino document units in **meters** to match Motus internal units
- For production visualization, export trajectory data to VirtualRobot or Robots

## Future

- FK-based link transforms from preset DH parameters
- TCP frame display and invalid segment coloring
- Optional mesh preview from preset link lengths
