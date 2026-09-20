# Yak packaging (`motus`)

Rhino Package Manager package for Motus.Grasshopper.

| Rule | Detail |
|------|--------|
| Package name | `motus` (do not rename) |
| First **public** push | **2.0.0** — live on yak.rhino3d.com (`yak search --all motus`) |
| Pack anytime | Allowed for local rebuilds / test-server |
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

## Push (post-GA)

`motus` **2.0.0** is live on production Yak. Further SemVers need a version bump (csproj + `MotusGhPlugin.Version` + manifest template + Motus.NET pin when required), fresh dual-TFM pack, then:

```bash
yak push dist/motus-<version>-rh8_*-any.yak
yak search --all motus
```

Do **not** re-push older pre-GA identities (0.17 / 1.8 / 1.9) to production.
