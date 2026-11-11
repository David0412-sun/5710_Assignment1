<#
.SYNOPSIS
Create the assignment's Python 3.11 environment and install pinned dependencies.
.EXAMPLE
powershell -NoProfile -ExecutionPolicy Bypass -File .\setup_windows.ps1 -Report
#>
[CmdletBinding()]
param([switch]$Report)

Set-StrictMode -Version Latest
$ErrorActionPreference = 'Stop'
$ProjectRoot = $PSScriptRoot
$VenvPath = Join-Path $ProjectRoot '.venv'
$PythonPath = Join-Path $VenvPath 'Scripts\python.exe'

function Invoke-Checked {
    param([string]$Executable, [string[]]$Arguments)
    & $Executable @Arguments
    if ($LASTEXITCODE -ne 0) {
        throw "Command failed with exit code ${LASTEXITCODE}: $Executable $($Arguments -join ' ')"
    }
}

if (Test-Path -LiteralPath $VenvPath) {
    if (-not (Test-Path -LiteralPath $PythonPath)) {
        throw "The existing .venv is not a Windows environment. Rename it, then rerun setup."
    }
    & $PythonPath -c 'import sys; sys.exit(sys.version_info[:2] != (3, 11))'
    if ($LASTEXITCODE -ne 0) {
        throw 'The existing .venv does not use Python 3.11. Rename it, then rerun setup.'
    }
} else {
    $Python311 = $null
    $Launcher = Get-Command py -ErrorAction SilentlyContinue
    if ($null -ne $Launcher) {
        # Missing launcher versions write to stderr; this is only a probe.
        $SavedErrorPreference = $ErrorActionPreference
        try {
            $ErrorActionPreference = 'Continue'
            $Candidate = & $Launcher.Source -3.11 -c 'import sys; print(sys.executable)' 2>$null
            if ($LASTEXITCODE -eq 0) { $Python311 = [string]$Candidate }
        } finally {
            $ErrorActionPreference = $SavedErrorPreference
        }
    }
    if ($Python311) {
        Invoke-Checked $Python311 @('-m', 'venv', $VenvPath)
    } else {
        $Uv = Get-Command uv -ErrorAction SilentlyContinue
        if ($null -eq $Uv) {
            throw 'Install Python 3.11 (with the py launcher) or uv, then rerun setup.'
        }
        $PreviousCache = $env:UV_CACHE_DIR
        $PreviousPythonInstall = $env:UV_PYTHON_INSTALL_DIR
        try {
            $env:UV_CACHE_DIR = Join-Path $ProjectRoot '.tmp\uv-cache'
            $env:UV_PYTHON_INSTALL_DIR = Join-Path $ProjectRoot '.tmp\python'
            Invoke-Checked $Uv.Source @('venv', '--python', '3.11', $VenvPath)
        } finally {
            $env:UV_CACHE_DIR = $PreviousCache
            $env:UV_PYTHON_INSTALL_DIR = $PreviousPythonInstall
        }
    }
}

& $PythonPath -c "import importlib.util, sys; sys.exit(importlib.util.find_spec('pip') is None)"
if ($LASTEXITCODE -ne 0) {
    Invoke-Checked $PythonPath @('-m', 'ensurepip', '--upgrade')
}
$RequirementFile = if ($Report) { 'requirements-report.txt' } else { 'requirements.txt' }
$Requirements = Join-Path $ProjectRoot "assignments\assignment_1_kinematics\$RequirementFile"
Invoke-Checked $PythonPath @('-m', 'pip', 'install', '-r', $Requirements)
Invoke-Checked $PythonPath @('-m', 'pip', 'check')
Write-Host "Ready: $PythonPath"
Write-Host 'Run: powershell -NoProfile -ExecutionPolicy Bypass -File .\run_assignment.ps1 test'
