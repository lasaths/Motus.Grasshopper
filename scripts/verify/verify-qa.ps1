#!/usr/bin/env pwsh
# Automated QA: Motus.NET tests + Rhino adapter smoke + install artifact check.
param(
    [ValidateSet("Debug", "Release")]
    [string]$Configuration = "Release",
    [switch]$Install
)

$ErrorActionPreference = "Stop"
$root = Split-Path $PSScriptRoot -Parent
$motusNet = Join-Path $root "..\Motus.NET"

Write-Host "=== Motus verify-qa ($Configuration) ==="

Write-Host "`n[1/4] Build..."
& (Join-Path $root "build.ps1") -Configuration $Configuration
if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }

Write-Host "`n[2/4] Install artifacts check..."
& (Join-Path $root "scripts\verify-install.ps1") -Configuration $Configuration
if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }

Write-Host "`n[3/4] Motus.NET unit tests..."
dotnet test (Join-Path $motusNet "Motus.NET.slnx") -c $Configuration --no-build
if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }

Write-Host "`n[4/4] Rhino adapter QA smoke..."
dotnet run --project (Join-Path $root "scripts\qa-smoke\QaSmoke.csproj") -c $Configuration
if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }

if ($Install) {
    Write-Host "`n[Install] Copying to Grasshopper Libraries\Motus..."
    & (Join-Path $root "build.ps1") -Configuration $Configuration -Install
    if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }
}

Write-Host "`n=== verify-qa PASSED ==="
Write-Host "Rhino UI checks (plugin load, Motus tab, Esc cancel in GH) still require opening examples in Rhino 8."
