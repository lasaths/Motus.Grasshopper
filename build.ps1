#!/usr/bin/env pwsh
# Build Grasshopper plugin (Motus.NET from NuGet).
param(
    [ValidateSet("Debug", "Release")]
    [string]$Configuration = "Release",
    [ValidateSet("net8.0-windows", "net8.0")]
    [string]$Tfm = "net8.0-windows",
    [switch]$Zip,
    [switch]$Yak,
    [switch]$Install,
    # Force Motus.NET NuGet packages (default; kept for scripts that already pass -UseNuGet).
    [switch]$UseNuGet,
    # Use sibling ../Motus.NET project refs instead of NuGet (CLI only; VS needs Motus.NET in the solution).
    [switch]$UseLocal
)

$ErrorActionPreference = "Stop"
$root = $PSScriptRoot
$out = Join-Path $root "src\Motus.GH\bin\$Configuration\$Tfm"

function Stage-MotusPlugin([string]$StageDir, [string]$OutputDir = $out) {
    if (Test-Path $StageDir) { Remove-Item $StageDir -Recurse -Force }
    New-Item -ItemType Directory -Force -Path $StageDir | Out-Null
    Copy-Item "$OutputDir\Motus.GH.gha" $StageDir
    Copy-Item "$OutputDir\Motus.*.dll" $StageDir
    $stageResources = Join-Path $StageDir "resources"
    New-Item -ItemType Directory -Force -Path $stageResources | Out-Null
    Copy-Item "$OutputDir\resources\*" $stageResources -Recurse
}

$msbuildProps = @()
if ($UseLocal -and -not $Yak) {
    $msbuildProps += "-p:UseMotusNetProjectReference=true"
    Write-Host "Using sibling Motus.NET project refs."
} else {
    $msbuildProps += "-p:UseMotusNetProjectReference=false"
    Write-Host "Using Motus.NET from NuGet."
}

Write-Host "Building Motus.Grasshopper ($Configuration)..."
dotnet restore (Join-Path $root "Motus.Grasshopper.slnx") --force-evaluate @msbuildProps
if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }
dotnet build (Join-Path $root "Motus.Grasshopper.slnx") -c $Configuration --no-restore @msbuildProps
if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }

if (-not (Test-Path "$out\Motus.GH.gha")) {
    throw "Missing $out\Motus.GH.gha — build the $Tfm target first."
}

Write-Host "`nBuilt: $out\Motus.GH.gha"
Write-Host "Copy to Grasshopper Libraries\Motus:"
Write-Host "  Motus.GH.gha"
Write-Host "  Motus.*.dll (from NuGet)"
Write-Host "  resources\robots\ (from Motus.Presets package)"

if ($Install) {
    $ghLib = Join-Path $env:APPDATA "Grasshopper\Libraries\Motus"
    New-Item -ItemType Directory -Force -Path $ghLib | Out-Null
    # Drop stale Motus.*.dll left from older layouts (e.g. Motus.Rhino) so GH cannot pick them up.
    Remove-Item "$ghLib\Motus.*.dll", "$ghLib\Motus.GH.gha" -Force -ErrorAction SilentlyContinue
    Copy-Item "$out\Motus.GH.gha" $ghLib -Force
    Copy-Item "$out\Motus.*.dll" $ghLib -Force
    $destResources = Join-Path $ghLib "resources"
    New-Item -ItemType Directory -Force -Path $destResources | Out-Null
    Copy-Item "$out\resources\*" $destResources -Recurse -Force
    $nested = Join-Path $destResources "resources"
    if (Test-Path $nested) { Remove-Item $nested -Recurse -Force }
    Write-Host "`nInstalled to: $ghLib"
    Write-Host "Restart Grasshopper (or Rhino) to load Motus."
}

if ($Zip -or $Yak) {
    $dist = Join-Path $root "dist"
    New-Item -ItemType Directory -Force -Path $dist | Out-Null
}

if ($Zip) {
    $zipPath = Join-Path $dist "Motus.Grasshopper-$Configuration.zip"
    if (Test-Path $zipPath) { Remove-Item $zipPath }
    $stage = Join-Path $env:TEMP "motus-gh-stage"
    Stage-MotusPlugin $stage
    Compress-Archive -Path "$stage\*" -DestinationPath $zipPath
    Remove-Item $stage -Recurse -Force
    Write-Host "`nRelease zip: $zipPath"
}

if ($Yak) {
    $cli = Get-Command yak -ErrorAction SilentlyContinue
    if ($cli) {
        $yakExe = $cli.Source
    } else {
        $yakExe = Join-Path ${env:ProgramFiles} "Rhino 8\System\Yak.exe"
        if (-not (Test-Path $yakExe)) { throw "yak not found — install Rhino 8 or add Yak.exe to PATH." }
    }

    $csproj = Get-Content (Join-Path $root "src\Motus.GH\Motus.GH.csproj") -Raw
    if ($csproj -notmatch "<Version>([^<]+)</Version>") { throw "Could not read <Version> from Motus.GH.csproj" }
    $version = $Matches[1]

    # Rhino 8 multi-target: Windows loads net8.0-windows, Mac loads net8.0.
    # manifest.yml must sit above the TFM folders (McNeel yak anatomy).
    $stage = Join-Path $env:TEMP "motus-yak-stage"
    if (Test-Path $stage) { Remove-Item $stage -Recurse -Force }
    New-Item -ItemType Directory -Force -Path $stage | Out-Null

    $binRoot = Join-Path $root "src\Motus.GH\bin\$Configuration"
    foreach ($tfmDir in @("net8.0-windows", "net8.0")) {
        $src = Join-Path $binRoot $tfmDir
        if (-not (Test-Path "$src\Motus.GH.gha")) {
            throw "Missing $src\Motus.GH.gha — both TFMs required for cross-platform Yak."
        }
        Stage-MotusPlugin (Join-Path $stage $tfmDir) $src
    }

    Copy-Item (Join-Path $root "LICENSE") (Join-Path $stage "LICENSE") -Force
    $manifest = Get-Content (Join-Path $root "packaging\yak\manifest.yml") -Raw
    $manifest = $manifest -replace "(?m)^version:\s*.*$", "version: $version"
    Set-Content -Path (Join-Path $stage "manifest.yml") -Value $manifest -NoNewline

    Push-Location $stage
    try {
        & $yakExe build
        if ($LASTEXITCODE -ne 0) { throw "yak build failed ($LASTEXITCODE)" }
        Get-ChildItem *.yak | ForEach-Object {
            $dest = Join-Path $dist $_.Name
            Move-Item $_.FullName $dest -Force
            Write-Host "`nYak package (Win+Mac): $dest"
            Write-Host "  net8.0-windows/ → Rhino 8 Windows"
            Write-Host "  net8.0/         → Rhino 8 macOS"
            Write-Host "Push when ready: yak push `"$dest`""
            Write-Host "Test server:     yak push --source https://test.yak.rhino3d.com `"$dest`""
        }
    }
    finally {
        Pop-Location
        Remove-Item $stage -Recurse -Force
    }
}
