# P8 Regression Matrix

Short checklist for close-open-dev GH wiring against Motus.NET 0.15.0.

- [ ] Build GH with local Motus.NET: `dotnet build src/Motus.GH/Motus.GH.csproj -c Release -p:UseMotusNetProjectReference=true --nologo`.
- [ ] Serial UR10e quick plan: plane LIN, joint-linear, and joint goal + collision RRT still succeed.
- [ ] Stewart: TCP plane LIN passes collision options; collided LIN reports collision or falls back to leg-length RRT; Waypoints/Export warn that `Q` is meters, not MoveJ radians.
- [ ] RRT Settings: `Step` tooltip/docs state radians for serial/legged and meters for `Family=stewart`; invalid step error is not radians-only.
- [ ] Joint Table: `BaseSE2` still previews as base override; AllDrivers promotes tip+side Plan DOF; joint goals with SE2 route through `PlanningOptions.Mobility=HolonomicSE2`.
- [ ] Motus Tool: Cap=`Custom` + Rd + Bd; Internalise Tool keeps Mechanism; Cap=`None` rejects Bd.
- [ ] Custom serial / Joint Table far plane Status names `IK NoConvergence` (or Singular/InvalidInput).
- [ ] Motus Walk: Path/Planes gait emits `Tr` for Preview/Export/Waypoints; `LeggedGait.ValidateForPlan` hard failures surface as errors and soft provenance/SSM messages remain remarks.
- [ ] Motus Plan legged: Walk `Rb` (Mechanism) + ≥2 planes → full-driver gait `Tr` (`PlanBodyPath`, hard SSM, not TCP LIN); tip joint / 1-plane LIN unchanged; mixed plane+joint fails named.
- [ ] Export: family warnings mirror Waypoints; `Retime` remains bool and optional `Retimer` defaults to `TotgLite`.
- [ ] Docs/examples: component reference, README, AGENTS, and generated `.ghx` metadata reflect any pin or pin-description changes.
