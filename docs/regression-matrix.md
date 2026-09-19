# P8 / Milestone 2.0 Regression Matrix

Host-readiness checklist for Motus.Grasshopper against Motus.NET **2.0.0** (UseLocal until NuGet publish).

**Default path (after publish):** NuGet pin (`build/MotusNetPackages.props`). **CI / tip / pre-publish path:** `-UseLocal` sibling Motus.NET (required in GitHub Actions until **2.0.0** is on nuget.org; NuGet CI jobs override to live **1.8.0** while **1.9.0** may lag). **Yak:** identity **2.0.0** — pack OK; production push is a separate authenticated step ([packaging/yak/README.md](../packaging/yak/README.md)).

## Non-Rhino (automatable)

CI smoke already covers GHX regenerate/validate, planner-only fence, `verify-motus-net-pin.mjs`, and QaSmoke **compile**. Also run:

- [ ] `node scripts/verify-motus-net-pin.mjs` — props pin = **2.0.0**; pre-publish allows UseLocal docs; post-publish requires Motus.Core **2.0.0** on nuget.org.
- [ ] `node scripts/verify-yak-packaging.mjs` — csproj / `MotusGhPlugin` / manifest / Motus.NET pin agree at **2.0.0**; icon + dual-TFM pack path present; do **not** claim Package Manager published until `yak push` succeeds.
- [ ] Default NuGet restore/build (no UseLocal): `dotnet build src/Motus.GH/Motus.GH.csproj -c Release` (local / release gate after Motus.NET **2.0.0** NuGet; CI `build-*-nuget` uses live **1.8.0** override until then).
- [ ] UseLocal still builds: `dotnet build src/Motus.GH/Motus.GH.csproj -c Release -p:UseMotusNetProjectReference=true` (CI does this).
- [ ] Motus.NET tip: `RegressionMatrixLogicTests` (serial / Stewart / SE2 / PlanBodyPath / Example 10 / export TotgLite) + Cap `ToolCapContract.TryValidateBinding` + Pick Place `PickPlaceTouchContract.TryRequireTouch` + `FamilyHandoffWarnings` / `ExperimentalUrdfLoad` via qa-smoke.
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
- [ ] Docs/examples: component reference, README, AGENTS, and generated `.ghx` metadata reflect pin **2.0.0**; no Package Manager / Yak published claim until push.

## Experimental mobility (Motus.NET tip — not GA)

Experimental mobility (`HolonomicSE3`, Unitree H2/Go2 fixtures, catalog fixtures) ships in Motus.NET **2.0.0** package identity but remains **not** Motus.Grasshopper Supported Yak GA — see [supported-surface.md](supported-surface.md).

- [ ] Motus.NET tests green on tip: catalog robot smoke, `UnitreeH2FixtureTests`, HolonomicSE3 / aerial fixture (sibling UseLocal).
- [ ] Optional Motus Robot load of meshless H2 / free-flyer URDF from Motus.NET fixtures via UseLocal — Status / remarks must say **experimental** (LoadTree + FK scrub only; no Walk, no biped, no flight controller).
- [ ] Waypoints + Export: `Family=aerial` runtime warning — body SE(3) / bodyPose, not UR MoveJ (string Family gate; works on NuGet 0.17.0 and UseLocal tip).
- [ ] **No** Motus GH “Aerial” / “H2” / SE3 mobility components claiming GA until a Motus.NET pin ships those types and a later milestone wires honest experimental Status.

See Motus.NET [`docs/aerial.md`](https://github.com/lasaths/Motus.NET/blob/master/docs/aerial.md), ADR holonomic-se3-aerial, and H2 fixture README.
