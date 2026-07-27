# Motus.NET 0.12.0 — Close Open Developments patches

These patches implement Motus.NET phases P0–P7 (xacro/TOTG/constraints/PRM*/CHOMP/Stewart RRT/tree/SE2/legged ValidateForPlan) plus METHODS.md / REFERENCES.bib.

**Apply on Motus.NET `master` (commit `9590f78` or later with terrain gait):**

```bash
cd ../Motus.NET
git checkout -b cursor/close-open-developments-2212
git am ../Motus.Grasshopper/patches/motus-net-0.12.0/000*.patch
# or: git apply patches/motus-net-0.12.0/combined.diff
dotnet test tests/Motus.OMPL.Tests -c Release
# publish 0.12.0 then pin Motus.Grasshopper MotusNetPackages.props
```

Cloud agent could not push to `lasaths/Motus.NET` (token write scope is Grasshopper-only). Maintainer must apply + tag/release NuGet 0.12.0.

See Motus.NET `docs/METHODS.md`, `docs/MULTI_AGENT.md`, `docs/REFERENCES.bib` after apply.
