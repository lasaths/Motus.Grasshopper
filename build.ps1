#!/usr/bin/env pwsh
# ponytail: build Motus.NET DLLs, then Grasshopper plugin + optional release zip
param(
    [ValidateSet("Debug", "Release")]
    [string]$Configuration = "Release",
    [switch]$Zip
)

$ErrorActionPreference = "Stop"
$root = $PSScriptRoot
$motusNet = Join-Path $root "..\Motus.NET"
$out = Join-Path $root "src\Motus.GH\bin\$Configuration\net8.0"

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
Write-Host "Copy to Grasshopper Libraries:"
Write-Host "  Motus.GH.gha"
Write-Host "  Motus.Core.dll, Motus.Geometry.dll, Motus.Presets.dll, Motus.OMPL.NET.dll, Motus.Rhino.dll"
Write-Host "  resources\robots\"

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
    Copy-Item "$out\resources" (Join-Path $stage "resources") -Recurse
    Compress-Archive -Path "$stage\*" -DestinationPath $zipPath
    Remove-Item $stage -Recurse -Force
    Write-Host "`nRelease zip: $zipPath"
}
