# P8 Regression Matrix

Short checklist for close-open-dev GH wiring against Motus.NET 0.12.0.

- [ ] Build GH with local Motus.NET: `dotnet build src/Motus.GH/Motus.GH.csproj -c Release -p:UseMotusNetProjectReference=true --nologo`.
- [ ] Serial UR10e quick plan: plane LIN, joint-linear, and joint goal + collision RRT still succeed.
- [ ] Stewart: TCP plane LIN passes collision options; collided LIN reports collision or falls back to leg-length RRT; Waypoints/Export warn that `Q` is meters, not MoveJ radians.
- [ ] RRT Settings: `Step` tooltip/docs state radians for serial/legged and meters for `Family=stewart`; invalid step error is not radians-only.
- [ ] Joint Table: `BaseSE2` still previews as base override and joint goals with SE2 route through `PlanningOptions.Mobility=HolonomicSE2`.
- [ ] WalkHex: Path/Planes gait emits `Tr` for Preview/Export/Waypoints; `LeggedGait.ValidateForPlan` hard failures surface as errors and soft provenance/SSM messages remain remarks.
- [ ] Export: family warnings mirror Waypoints; `Retime` remains bool and optional `Retimer` defaults to `TotgLite`.
- [ ] Docs/examples: component reference, README, AGENTS, and generated `.ghx` metadata reflect any pin or pin-description changes.
