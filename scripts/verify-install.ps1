#!/usr/bin/env pwsh
# Verify Release build output has all files needed for Grasshopper install.
param(
    [string]$Configuration = "Release"
)

$ErrorActionPreference = "Stop"
$out = Join-Path $PSScriptRoot "..\src\Motus.GH\bin\$Configuration\net8.0-windows"
$required = @(
    "Motus.GH.gha",
    "Motus.Core.dll",
    "Motus.Geometry.dll",
    "Motus.Presets.dll",
    "Motus.OMPL.NET.dll",
    "Motus.Rhino.dll"
)

$missing = @()
foreach ($f in $required) {
    if (-not (Test-Path (Join-Path $out $f))) { $missing += $f }
}
$robots = Join-Path $out "resources\robots\UR\UR5e.json"
if (-not (Test-Path $robots)) { $missing += "resources\robots\UR\UR5e.json" }

if ($missing.Count -gt 0) {
    Write-Error "Missing: $($missing -join ', '). Run ./build.ps1 -Configuration $Configuration first."
}
Write-Host "OK: $out ready for Grasshopper Libraries\Motus copy."
