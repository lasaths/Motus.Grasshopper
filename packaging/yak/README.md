# Yak packaging (`motus`)

Rhino Package Manager package for Motus.Grasshopper.

| Rule | Detail |
|------|--------|
| Package name | `motus` (do not rename) |
| First **public** push | **2.0.0** only — no 0.x / 1.8 / 1.9 on yak.rhino3d.com |
| Pack anytime | Allowed for local / test-server dry-runs |
| Motus.NET | `-Yak` always builds against **NuGet** (never UseLocal) |

## Anatomy

`build.ps1 -Yak` stages:

```
manifest.yml          # version overwritten from Motus.GH.csproj <Version>
icon.png
LICENSE
net8.0-windows/       # Rhino 8 Windows (.gha + Motus.*.dll + resources)
net8.0/               # Rhino 8 macOS
```

In-repo `manifest.yml` version is a **template**; the pack step rewrites it from the csproj so SemVer bumps need only touch assembly identity surfaces (csproj + `MotusGhPlugin.Version` + Motus.NET pin).

## Pack

```powershell
# Windows (pwsh) — dual-TFM Release + .yak
./build.ps1 -Configuration Release -Yak

# macOS with pwsh
pwsh ./build.ps1 -Configuration Release -Yak

# macOS without pwsh — see scripts/pack-yak.sh after dual-TFM Release build
./scripts/pack-yak.sh
```

Output: `dist/motus-<version>-rh8_*-any.yak`

## Push (2.0.0 GA only)

Prerequisites (human / authenticated host):

1. Motus.NET **2.0.0** on nuget.org; GH pin + csproj + `MotusGhPlugin.Version` + manifest template all **2.0.0**
2. Clean tree (or stash WIP) — do not bake dirty WIP into the package
3. Fresh dual-TFM Release build identity matches **2.0.0**
4. Yak auth: `~/.mcneel/yak.yml` or `YAK_TOKEN` (`yak login` / `yak login --ci`)
5. `node scripts/verify/verify-yak-packaging.mjs` green

```bash
# Prefer production only when Version == 2.0.0
yak push dist/motus-2.0.0-rh8_*-any.yak
yak search --all motus

# Pre-GA dry-run → test server only
yak push --source https://test.yak.rhino3d.com dist/motus-*.yak
```

Do **not** push 0.17 / 1.8 / 1.9 to production Yak — policy in the Motus 2.0 release plan.
