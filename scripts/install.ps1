#!/usr/bin/env pwsh
param(
    [ValidateSet("Debug", "Release")]
    [string]$Configuration = "Release"
)

$ErrorActionPreference = "Stop"
$src = Join-Path $PSScriptRoot "..\src\Motus.GH\bin\$Configuration\net8.0"
$dest = Join-Path $env:APPDATA "Grasshopper\Libraries\Motus"

& (Join-Path $PSScriptRoot "..\build.ps1") -Configuration $Configuration
& (Join-Path $PSScriptRoot "verify-install.ps1") -Configuration $Configuration

New-Item -ItemType Directory -Force -Path $dest | Out-Null
Copy-Item "$src\Motus.GH.gha" $dest -Force
Copy-Item "$src\Motus.*.dll" $dest -Force
Copy-Item "$src\resources" (Join-Path $dest "resources") -Recurse -Force

Write-Host "`nInstalled to: $dest"
Write-Host "Restart Grasshopper (or Rhino) to load Motus."
