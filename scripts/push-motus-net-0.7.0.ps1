#!/usr/bin/env pwsh
# Minimal Motus.NET 0.7.0 publish — Grasshopper 0.7.0 already released.
# Run from anywhere; uses sibling ../Motus.NET next to Motus.Grasshopper.
$ErrorActionPreference = "Stop"
$ghRoot = Split-Path $PSScriptRoot -Parent
$net = [IO.Path]::GetFullPath((Join-Path $ghRoot "..\Motus.NET"))
if (-not (Test-Path (Join-Path $net "Motus.NET.slnx"))) {
    throw "Motus.NET not found at $net — clone it next to Motus.Grasshopper."
}
$patchDir = Join-Path $ghRoot "patches\motus-net-0.7.0"
$branch = "cursor/cell-aware-0.7-96a0"

Set-Location $net
git fetch origin master
git checkout master
git pull origin master

if (git branch --list $branch) { git checkout $branch } else { git checkout -b $branch }

$log = git log --oneline -30 | Out-String
if ($log -notmatch "Resolve plane merge conflicts and cut 0\.7\.0") {
    Get-ChildItem $patchDir -Filter "*.patch" | Sort-Object Name | ForEach-Object {
        git am $_.FullName
        if ($LASTEXITCODE -ne 0) { throw "git am failed: $($_.Name)" }
    }
}

dotnet test tests/Motus.Core.Tests/Motus.Core.Tests.csproj -c Release --filter "FullyQualifiedName~CollisionPlane|FullyQualifiedName~LinCollisionRrtFallback"
if ($LASTEXITCODE -ne 0) { throw "tests failed" }

git push -u origin $branch
Write-Host "Merge $branch into master on GitHub, then press Enter."
[void](Read-Host)

git checkout master
git pull origin master
$ver = (Get-Content motus-net.version -Raw).Trim()
if ($ver -ne "0.7.0") { throw "master motus-net.version is '$ver' — merge the PR first" }

if (-not (git tag -l "v0.7.0")) {
    git tag v0.7.0
    git push origin v0.7.0
} else {
    Write-Host "Tag v0.7.0 already exists"
}

Write-Host "Waiting for NuGet Motus.Core 0.7.0 ..."
$deadline = (Get-Date).AddMinutes(30)
while ((Get-Date) -lt $deadline) {
    try {
        $j = Invoke-RestMethod "https://api.nuget.org/v3-flatcontainer/motus.core/index.json"
        if ($j.versions -contains "0.7.0") {
            Write-Host "Motus.Core 0.7.0 is on nuget.org"
            exit 0
        }
    } catch {}
    Start-Sleep 20
}
throw "Timed out waiting for Motus.Core 0.7.0"
