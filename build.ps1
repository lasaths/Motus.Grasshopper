#!/usr/bin/env pwsh
# Build Grasshopper plugin (Motus.NET from NuGet).
param(
    [ValidateSet("Debug", "Release")]
    [string]$Configuration = "Release",
    [switch]$Zip,
    [switch]$Install
)

$ErrorActionPreference = "Stop"
$root = $PSScriptRoot
$out = Join-Path $root "src\Motus.GH\bin\$Configuration\net8.0-windows"

Write-Host "Building Motus.Grasshopper ($Configuration)..."
dotnet build (Join-Path $root "Motus.Grasshopper.slnx") -c $Configuration
if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }

Write-Host "`nBuilt: $out\Motus.GH.gha"
Write-Host "Copy to Grasshopper Libraries\Motus:"
Write-Host "  Motus.GH.gha"
Write-Host "  Motus.*.dll (from NuGet + Motus.Rhino.dll)"
Write-Host "  resources\robots\ (from Motus.Presets package)"

if ($Install) {
    $ghLib = Join-Path $env:APPDATA "Grasshopper\Libraries\Motus"
    New-Item -ItemType Directory -Force -Path $ghLib | Out-Null
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

if ($Zip) {
    $dist = Join-Path $root "dist"
    New-Item -ItemType Directory -Force -Path $dist | Out-Null
    $zipPath = Join-Path $dist "Motus.Grasshopper-$Configuration.zip"
    if (Test-Path $zipPath) { Remove-Item $zipPath }
    $stage = Join-Path $env:TEMP "motus-gh-stage"
    if (Test-Path $stage) { Remove-Item $stage -Recurse -Force }
    New-Item -ItemType Directory -Force -Path $stage | Out-Null
    Copy-Item "$out\Motus.GH.gha" $stage
    Copy-Item "$out\Motus.*.dll" $stage
    $stageResources = Join-Path $stage "resources"
    New-Item -ItemType Directory -Force -Path $stageResources | Out-Null
    Copy-Item "$out\resources\*" $stageResources -Recurse
    Compress-Archive -Path "$stage\*" -DestinationPath $zipPath
    Remove-Item $stage -Recurse -Force
    Write-Host "`nRelease zip: $zipPath"
}
