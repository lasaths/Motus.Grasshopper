# P8 / Milestone 1.8 Regression Matrix

Host-readiness checklist for Motus.Grasshopper against Motus.NET **0.17.0** ([nuget.org](https://www.nuget.org/packages/Motus.Core/0.17.0)).

**Default path:** NuGet pin (`build/MotusNetPackages.props`). **CI / tip path:** `-UseLocal` sibling Motus.NET (still required in GitHub Actions). **Yak:** pack-only — `motus` is **not** on Rhino Package Manager (first public Yak target is **2.0.0**).

## Non-Rhino (automatable)

CI smoke already covers GHX regenerate/validate, planner-only fence, `verify-motus-net-pin.mjs`, and QaSmoke **compile**. Also run:

- [ ] `node scripts/verify-motus-net-pin.mjs` — props pin = **0.17.0**, nuget.org lists Motus.Core **0.17.0**, docs do not claim NuGet unreleased / UseLocal-only.
- [ ] Default NuGet restore/build (no UseLocal): `dotnet build src/Motus.GH/Motus.GH.csproj -c Release` (local / release gate; CI `build-*-nuget` jobs).
- [ ] UseLocal still builds: `dotnet build src/Motus.GH/Motus.GH.csproj -c Release -p:UseMotusNetProjectReference=true` (CI does this).
- [ ] Motus.NET tip: `RegressionMatrixLogicTests` (serial / Stewart / SE2 / PlanBodyPath / Example 10 / export TotgLite) + Cap `ToolCapContract.TryValidateBinding` via qa-smoke Cap block.
## Rhino (manual — required before Rhino-touching release)

- [ ] Serial UR10e quick plan: plane LIN, joint-linear, and joint goal + collision RRT still succeed.
- [ ] Stewart: TCP plane LIN passes collision options; collided LIN reports collision or falls back to leg-length RRT; Waypoints/Export warn that `Q` is meters, not MoveJ radians.
- [ ] RRT Settings: `Step` tooltip/docs state radians for serial/legged and meters for `Family=stewart`; invalid step error is not radians-only.
- [ ] Joint Table: `BaseSE2` still previews as base override; AllDrivers promotes tip+side Plan DOF; joint goals with SE2 route through `PlanningOptions.Mobility=HolonomicSE2`.
- [ ] Motus Tool: Cap=`Custom` + Rd + Bd; Internalise Tool keeps Mechanism; Cap=`None` rejects Bd.
- [ ] Custom serial / Joint Table far plane Status names `IK NoConvergence` (or Singular/InvalidInput).
- [ ] Motus Walk: Path/Planes gait emits `Tr` for Preview/Export/Waypoints; `LeggedGait.ValidateForPlan` hard failures surface as errors and soft provenance/SSM messages remain remarks.
- [ ] Motus Plan legged: Walk `Rb` (Mechanism) + ≥2 planes → full-driver gait `Tr` (`PlanBodyPath`, hard SSM, not TCP LIN); tip joint / 1-plane LIN unchanged; mixed plane+joint fails named.
- [ ] Export: family warnings mirror Waypoints; `Retime` remains bool and optional `Retimer` defaults to `TotgLite`.
- [ ] Example 10: Pick Place → one Program Auto Plan; SET 0.085/0.04; Preview holds Detach poses between cycles; plan ColScene = table only.
- [ ] Docs/examples: component reference, README, AGENTS, and generated `.ghx` metadata reflect pin **0.17.0**; no Package Manager / Yak published claim.

## Experimental mobility (Motus.NET tip — not GA)

These land on Motus.NET **master** after the **0.17.0** NuGet cut (`HolonomicSE3`, Unitree H2 fixture, catalog fixtures). They are **not** Motus.Grasshopper GA and must not be marketed as Package Manager / 2.0 features.

- [ ] Motus.NET tests green on tip: catalog robot smoke, `UnitreeH2FixtureTests`, HolonomicSE3 / aerial fixture (sibling UseLocal).
- [ ] Optional Motus Robot load of meshless H2 / free-flyer URDF from Motus.NET fixtures via UseLocal — Status / remarks must say **experimental** (LoadTree + FK scrub only; no Walk, no biped, no flight controller).
- [ ] Waypoints + Export: `Family=aerial` runtime warning — body SE(3) / bodyPose, not UR MoveJ (string Family gate; works on NuGet 0.17.0 and UseLocal tip).
- [ ] **No** Motus GH “Aerial” / “H2” / SE3 mobility components claiming GA until a Motus.NET pin ships those types and a later milestone wires honest experimental Status.

See Motus.NET [`docs/aerial.md`](https://github.com/lasaths/Motus.NET/blob/master/docs/aerial.md), ADR holonomic-se3-aerial, and H2 fixture README.
