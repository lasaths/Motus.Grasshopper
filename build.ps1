#!/usr/bin/env pwsh
# ponytail: build Motus.NET DLLs, then Grasshopper plugin + optional release zip
param(
    [ValidateSet("Debug", "Release")]
    [string]$Configuration = "Release",
    [switch]$Zip,
    [switch]$Install
)

$ErrorActionPreference = "Stop"
$root = $PSScriptRoot
$motusNet = Join-Path $root "..\Motus.NET"
$out = Join-Path $root "src\Motus.GH\bin\$Configuration\net8.0-windows"

if (-not (Test-Path $motusNet)) {
    throw "Motus.NET not found at $motusNet — clone as sibling of Motus.Grasshopper."
}

Write-Host "Building Motus.NET ($Configuration)..."
dotnet build (Join-Path $motusNet "Motus.NET.slnx") -c $Configuration
if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }

Write-Host "Building Motus.Grasshopper ($Configuration)..."
dotnet build (Join-Path $root "Motus.Grasshopper.slnx") -c $Configuration /p:MotusNetConfiguration=$Configuration
if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }

Write-Host "`nBuilt: $out\Motus.GH.gha"
Write-Host "Copy to Grasshopper Libraries\Motus:"
Write-Host "  Motus.GH.gha"
Write-Host "  Motus.Core.dll, Motus.Geometry.dll, Motus.Presets.dll, Motus.OMPL.NET.dll, Motus.Rhino.dll"
Write-Host "  resources\robots\"

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
