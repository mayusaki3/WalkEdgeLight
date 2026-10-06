param(
    [string]$OutputDirectory = (Join-Path $PSScriptRoot "build"),
    [string]$UnityProjectDirectory = ""
)

$ErrorActionPreference = "Stop"

$p0Root = [System.IO.Path]::GetFullPath((Join-Path $PSScriptRoot ".."))
$sourceRoot = Join-Path $p0Root "src"
$simulationRoot = Join-Path $p0Root "unity\Assets\WalkEdgeLight.Validation\Simulation"

$assetsRoot = Join-Path $OutputDirectory "Assets"
$probeRoot = Join-Path $assetsRoot "WalkEdgeLight\Probe"
$runtimeRoot = Join-Path $probeRoot "Runtime"
$editorRoot = Join-Path $probeRoot "Editor"

if (Test-Path $OutputDirectory) {
    Remove-Item $OutputDirectory -Recurse -Force
}

New-Item -ItemType Directory -Force -Path $runtimeRoot | Out-Null
New-Item -ItemType Directory -Force -Path $editorRoot | Out-Null

$coreFiles = @(Get-ChildItem -Path $sourceRoot -Filter "*.cs" -File -Recurse)
$simulationFiles = @(Get-ChildItem -Path $simulationRoot -Filter "*.cs" -File)
$setupSource = Join-Path $PSScriptRoot "Editor\ProbeSetup.cs"

if ($coreFiles.Count -eq 0) {
    throw "No validation core C# files found: $sourceRoot"
}
if ($simulationFiles.Count -eq 0) {
    throw "No Unity simulation C# files found: $simulationRoot"
}
if (-not (Test-Path $setupSource)) {
    throw "ProbeSetup.cs not found: $setupSource"
}

Copy-Item $coreFiles.FullName -Destination $runtimeRoot
Copy-Item $simulationFiles.FullName -Destination $runtimeRoot
Copy-Item $setupSource -Destination $editorRoot

Write-Host "WALKEDGE_PROBE|BUILD.CORE_CS_COUNT|$($coreFiles.Count)"
Write-Host "WALKEDGE_PROBE|BUILD.SIMULATION_CS_COUNT|$($simulationFiles.Count)"
Write-Host "WALKEDGE_PROBE|BUILD.OUTPUT|$assetsRoot"

if (-not [string]::IsNullOrWhiteSpace($UnityProjectDirectory)) {
    $unityAssetsRoot = Join-Path $UnityProjectDirectory "Assets"
    if (-not (Test-Path $unityAssetsRoot)) {
        throw "Unity Project Assets directory not found: $unityAssetsRoot"
    }

    $unityProductRoot = Join-Path $unityAssetsRoot "WalkEdgeLight"
    $unityProbeRoot = Join-Path $unityProductRoot "Probe"

    if (Test-Path $unityProbeRoot) {
        Remove-Item $unityProbeRoot -Recurse -Force
    }

    New-Item -ItemType Directory -Force -Path $unityProductRoot | Out-Null
    Copy-Item $probeRoot -Destination $unityProductRoot -Recurse

    Write-Host "WALKEDGE_PROBE|DEPLOY.UNITY_PROJECT|$UnityProjectDirectory"
    Write-Host "WALKEDGE_PROBE|DEPLOY.TARGET|$unityProbeRoot"
    Write-Host "WALKEDGE_PROBE|DEPLOY.RESULT|PASS"
}

Write-Host "WALKEDGE_PROBE|BUILD.RESULT|PASS"
