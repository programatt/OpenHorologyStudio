# SPDX-License-Identifier: Apache-2.0
# One-command build and test on Windows: .\build.ps1 [-Debug] [-Jobs N] [-NoTest]
#
# Needs Visual Studio 2022 17.10+ (the C++ workload), CMake 3.28+, Git and Python 3.10+. The
# dependencies come from vcpkg, bootstrapped into .vcpkg\ the first time (Open CASCADE builds from
# source and takes a while; later runs use vcpkg's binary cache). This script mirrors build.sh and
# has not been run in the environment that produced the 1.0 sources.
param(
    [switch]$Debug,
    [int]$Jobs = 0,
    [switch]$NoTest
)
$ErrorActionPreference = "Stop"
Set-Location $PSScriptRoot

foreach ($tool in "cmake", "git") {
    if (-not (Get-Command $tool -ErrorAction SilentlyContinue)) {
        Write-Error "missing: $tool (install CMake 3.28+ and Git, and open a Developer PowerShell for VS 2022)"
    }
}

$vcpkg = if ($env:VCPKG_ROOT) { $env:VCPKG_ROOT } else { Join-Path $PSScriptRoot ".vcpkg" }
if (-not (Test-Path (Join-Path $vcpkg "vcpkg.exe"))) {
    if (-not (Test-Path $vcpkg)) { git clone https://github.com/microsoft/vcpkg.git $vcpkg }
    & (Join-Path $vcpkg "bootstrap-vcpkg.bat") -disableMetrics
}
$env:VCPKG_ROOT = $vcpkg

$config = if ($Debug) { "debug" } else { "release" }
$preset = "windows-$config"
cmake --preset $preset
if ($LASTEXITCODE -ne 0) { throw "configure failed" }
$parallel = if ($Jobs -gt 0) { @("--parallel", $Jobs) } else { @("--parallel") }
cmake --build --preset $preset @parallel
if ($LASTEXITCODE -ne 0) { throw "build failed" }
if (-not $NoTest) {
    ctest --preset $preset
    if ($LASTEXITCODE -ne 0) { throw "tests failed" }
}
Write-Host "ohs binary: build\$preset\src\ohs.exe"
