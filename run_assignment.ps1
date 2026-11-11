<#
.SYNOPSIS
Run Assignment 1 with the project's Windows Python environment.
.EXAMPLE
powershell -NoProfile -ExecutionPolicy Bypass -File .\run_assignment.ps1 verify
.EXAMPLE
powershell -NoProfile -ExecutionPolicy Bypass -File .\run_assignment.ps1 ik -Headless
#>
[CmdletBinding()]
param(
    [Parameter(Position = 0)]
    [ValidateSet('test', 'fk', 'ik', 'view', 'report', 'verify')]
    [string]$Action = 'test',
    [switch]$Headless,
    [Parameter(ValueFromRemainingArguments = $true)]
    [string[]]$ScriptArgs = @()
)

Set-StrictMode -Version Latest
$ErrorActionPreference = 'Stop'
$PythonPath = Join-Path $PSScriptRoot '.venv\Scripts\python.exe'
if (-not (Test-Path -LiteralPath $PythonPath)) {
    throw 'Run setup_windows.ps1 first to create the Python 3.11 environment.'
}
if ($Headless -and $Action -ne 'ik') {
    throw '-Headless is only valid for the ik action; verify always runs headlessly.'
}

$AssignmentPath = Join-Path $PSScriptRoot 'assignments\assignment_1_kinematics'
$PythonArgs = switch ($Action) {
    'test' { @('-m', 'unittest', 'discover', '-s', $AssignmentPath, '-p', 'test_*.py', '-v') }
    'fk' { @(Join-Path $AssignmentPath 'forward_kinematics.py') }
    'ik' { @(Join-Path $AssignmentPath 'inverse_kinematics.py') }
    'view' { @(Join-Path $AssignmentPath 'view_model.py') }
    'report' { @(Join-Path $AssignmentPath 'build_report.py') }
    'verify' { @(Join-Path $AssignmentPath 'verify_assignment.py') }
}
$PythonArgs = @($PythonArgs)
if ($Headless) { $PythonArgs += '--headless' }
$PythonArgs += $ScriptArgs
Push-Location $PSScriptRoot
try {
    & $PythonPath @PythonArgs
    if ($LASTEXITCODE -ne 0) {
        throw "Assignment action '$Action' failed with exit code $LASTEXITCODE."
    }
} finally {
    Pop-Location
}
