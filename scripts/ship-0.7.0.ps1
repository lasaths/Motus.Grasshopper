#!/usr/bin/env pwsh
# One-shot Motus 0.7.0 ship: Motus.NET patch → test → tag → wait NuGet → pin GH → tag GH.
# Run from Motus.Grasshopper root with push access to BOTH repos and gh authenticated as you.
param(
    [string]$MotusNetRoot = (Join-Path (Split-Path $PSScriptRoot -Parent) "..\Motus.NET"),
    [switch]$SkipRhinoSmoke,
    [switch]$DryRun
)

$ErrorActionPreference = "Stop"
$ghRoot = Split-Path $PSScriptRoot -Parent
$MotusNetRoot = [System.IO.Path]::GetFullPath($MotusNetRoot)
$patchDir = Join-Path $ghRoot "patches\motus-net-0.7.0"
$branch = "cursor/cell-aware-0.7-96a0"
$version = "0.7.0"

function Step($msg) { Write-Host "`n=== $msg ===" -ForegroundColor Cyan }
function Run($cmd) {
    Write-Host "> $cmd" -ForegroundColor DarkGray
    if ($DryRun) { return }
    Invoke-Expression $cmd
    if ($LASTEXITCODE -ne 0 -and $null -ne $LASTEXITCODE) { throw "Command failed ($LASTEXITCODE): $cmd" }
}

if (-not (Test-Path (Join-Path $MotusNetRoot "Motus.NET.slnx"))) {
    throw "Motus.NET not found at $MotusNetRoot — clone sibling next to Motus.Grasshopper."
}
if (-not (Test-Path $patchDir)) { throw "Missing patches at $patchDir" }

Step "1/7 Motus.NET: branch + apply 0.7 patches"
Push-Location $MotusNetRoot
try {
    Run "git fetch origin master"
    Run "git checkout master"
    Run "git pull origin master"
    $onBranch = (git rev-parse --abbrev-ref HEAD)
    $hasBranch = git branch --list $branch
    if ($hasBranch) {
        Run "git checkout $branch"
    } else {
        Run "git checkout -b $branch"
    }

    # Skip am if commits already present (idempotent)
    $log = git log --oneline -20
    if ($log -notmatch "Resolve plane merge conflicts and cut 0\.7\.0") {
        Get-ChildItem $patchDir -Filter "*.patch" | Sort-Object Name | ForEach-Object {
            Run "git am `"$($_.FullName)`""
        }
    } else {
        Write-Host "Patches already applied — skipping git am"
    }

    Step "2/7 Motus.NET: tests"
    Run "dotnet test tests/Motus.Core.Tests/Motus.Core.Tests.csproj -c Release --filter `"FullyQualifiedName~CollisionPlane|FullyQualifiedName~LinCollisionRrtFallback`""

    Step "3/7 Motus.NET: push branch + tag v$version"
    Run "git push -u origin $branch"
    Write-Host "Open/merge PR for Motus.NET $branch → master, then press Enter to tag (or Ctrl+C to stop)."
    if (-not $DryRun) { [void](Read-Host) }
    Run "git checkout master"
    Run "git pull origin master"
    $verFile = (Get-Content motus-net.version -Raw).Trim()
    if ($verFile -ne $version) { throw "master motus-net.version is '$verFile', expected $version — merge the branch first." }
    $tagExists = git tag -l "v$version"
    if (-not $tagExists) {
        Run "git tag v$version"
        Run "git push origin v$version"
    } else {
        Write-Host "Tag v$version already exists"
    }
}
finally { Pop-Location }

Step "4/7 Wait for NuGet Motus.Core $version"
$deadline = (Get-Date).AddMinutes(30)
while ((Get-Date) -lt $deadline) {
    try {
        $json = Invoke-RestMethod "https://api.nuget.org/v3-flatcontainer/motus.core/index.json"
        if ($json.versions -contains $version) {
            Write-Host "Motus.Core $version is on nuget.org"
            break
        }
    } catch { }
    Write-Host "… not on nuget.org yet, sleep 20s"
    if ($DryRun) { break }
    Start-Sleep -Seconds 20
}
if (-not $DryRun) {
    $json = Invoke-RestMethod "https://api.nuget.org/v3-flatcontainer/motus.core/index.json"
    if ($json.versions -notcontains $version) { throw "Timed out waiting for Motus.Core $version on nuget.org" }
}

Step "5/7 Motus.Grasshopper: bump pin + version to $version"
Push-Location $ghRoot
try {
    Run "git checkout $branch"
    $files = @(
        "build\MotusNetPackages.props",
        "build\MotusNetLocal.props",
        "src\Motus.GH\Motus.GH.csproj",
        "README.md",
        "AGENTS.md"
    )
    foreach ($rel in $files) {
        $path = Join-Path $ghRoot $rel
        if (-not (Test-Path $path)) { continue }
        $text = Get-Content $path -Raw
        $text = $text -replace "0\.6\.9-local", "$version-local"
        $text = $text -replace "0\.6\.9", $version
        if (-not $DryRun) { Set-Content -Path $path -Value $text -NoNewline }
        Write-Host "Updated $rel"
    }
    # AssemblyVersion needs 4-part
    $csproj = Join-Path $ghRoot "src\Motus.GH\Motus.GH.csproj"
    if (-not $DryRun) {
        (Get-Content $csproj -Raw) `
            -replace "<AssemblyVersion>${version}</AssemblyVersion>", "<AssemblyVersion>${version}.0</AssemblyVersion>" `
            -replace "<FileVersion>${version}</FileVersion>", "<FileVersion>${version}.0</FileVersion>" |
            Set-Content $csproj -NoNewline
    }

    Step "6/7 Motus.Grasshopper: build + qa-smoke"
    Run "dotnet build src/Motus.GH/Motus.GH.csproj -c Release"
    if (-not $SkipRhinoSmoke) {
        Run "dotnet run --project scripts/qa-smoke/QaSmoke.csproj -c Release"
    } else {
        Write-Host "SkipRhinoSmoke set — remember to run qa-smoke locally"
    }

    Run "git add build/MotusNetPackages.props build/MotusNetLocal.props src/Motus.GH/Motus.GH.csproj README.md AGENTS.md patches"
    Run "git commit -m `"Release ${version}: pin Motus.NET ${version} for cell-aware planning.`""
    Run "git push -u origin $branch"
    Write-Host "Merge Grasshopper PR, then press Enter to tag v$version."
    if (-not $DryRun) { [void](Read-Host) }
    Run "git checkout master"
    Run "git pull origin master"
    $tagExists = git tag -l "v$version"
    if (-not $tagExists) {
        Run "git tag v$version"
        Run "git push origin v$version"
    }

    Step "7/7 Zip release asset"
    Run "./build.ps1 -Configuration Release -Zip"
    $zip = Get-ChildItem (Join-Path $ghRoot "dist") -Filter "*$version*" -ErrorAction SilentlyContinue |
        Sort-Object LastWriteTime -Descending | Select-Object -First 1
    if (-not $zip) {
        $zip = Get-ChildItem (Join-Path $ghRoot "dist") -Filter "*.zip" -ErrorAction SilentlyContinue |
            Sort-Object LastWriteTime -Descending | Select-Object -First 1
    }
    if ($zip) {
        Write-Host "Zip: $($zip.FullName)"
        Write-Host "Attach to GitHub release v$version if the workflow did not."
    }
}
finally { Pop-Location }

Write-Host "`nDone. Rhino UI checklist:" -ForegroundColor Green
Write-Host "  - ColPlane floor + plane goal through ColSphere → Success + RRT warning"
Write-Host "  - Free-space plane → LIN"
Write-Host "  - Motus Program LIN → validate-only"
