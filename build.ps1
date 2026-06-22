#!/usr/bin/env pwsh
# ponytail: build + print copy path
$ErrorActionPreference = "Stop"
dotnet build Motus.Grasshopper.slnx
$out = Join-Path $PSScriptRoot "src\Motus.GH\bin\Debug\net8.0"
Write-Host "`nBuilt: $out\Motus.GH.gha"
Write-Host "Copy Motus.GH.gha, Motus.*.dll, and resources\ to Grasshopper Libraries."
