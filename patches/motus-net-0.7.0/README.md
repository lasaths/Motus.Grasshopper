# Motus.NET 0.7.0 patches

Apply onto Motus.NET `master` **after** commit `eeba1aa` (plane landing with conflict markers):

```bash
cd ../Motus.NET
git am ../Motus.Grasshopper/patches/motus-net-0.7.0/*.patch
```

Or run from Grasshopper root (does NET + NuGet wait + GH pin/tag):

```powershell
./scripts/ship-0.7.0.ps1
```
