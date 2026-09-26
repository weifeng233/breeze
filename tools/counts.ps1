# README's counts, measured and compared.
#
# The status line restates four measurements: the kernel's test count, the example
# application's, the algorithm layer's, and the number of cross-compiled targets.
# That kind of sentence goes stale silently - it had already drifted three times
# before anyone noticed - so CI checks it. This script is that check, extracted so
# it can also be run by hand:
#
#   pwsh tools/counts.ps1                       # zig from PATH
#   pwsh tools/counts.ps1 -Zig C:\path\to\zig  # or an explicit one
#
# It exists as a script rather than inline YAML because of how it failed first: a
# round added four algorithm tests and forgot the README line, and the only thing
# that noticed was CI - one push later. Every other gate in this repo
# (`sizes.ps1`, and the C layer's before it) is a script for exactly that reason.
#
# The three suites are separate on purpose: the kernel's own tests; the example
# application's, which run on the host because its modules take their I/O surface
# as a comptime parameter; and the algorithm layer's, which import neither kernel
# nor platform.
param(
    [string]$Zig = $(if ($env:ZIG) { $env:ZIG } else { 'zig' })
)

$ErrorActionPreference = 'Stop'

if (Get-Variable -Name PSNativeCommandUseErrorActionPreference -ErrorAction SilentlyContinue) {
    $PSNativeCommandUseErrorActionPreference = $false
}

$repoRoot = [System.IO.Path]::GetFullPath((Join-Path $PSScriptRoot '..'))
$readme = Join-Path $repoRoot 'README.md'

Push-Location $repoRoot
try {
    function Invoke-Zig {
        param([string[]]$Arguments)
        $output = & $Zig @Arguments 2>&1 | Out-String
        if ($LASTEXITCODE -ne 0) {
            throw "zig $($Arguments -join ' ') failed:`n$output"
        }
        return $output
    }

    function Get-TestCount {
        param([string]$Text, [string]$What)
        $match = [regex]::Match($Text, '(?m)^All (\d+) tests passed\.\s*$')
        if (-not $match.Success) { throw "could not parse the $What test count" }
        return [int]$match.Groups[1].Value
    }

    $kernel = Get-TestCount (Invoke-Zig @('test', 'src/breeze.zig')) 'kernel'
    $app = Get-TestCount (Invoke-Zig @('test', '--dep', 'breeze', '-Mroot=examples/smartcar/app_test.zig', '-Mbreeze=src/breeze.zig')) 'application'
    $algorithms = Get-TestCount (Invoke-Zig @('test', 'src/algorithms.zig')) 'algorithm'

    # Assigned first on purpose: `Invoke-Zig @(...) -split "`n"` parses `-split` as
    # a parameter name for the function rather than the operator, which silently
    # produced a count of zero the first time this script ran.
    $targetSummary = Invoke-Zig @('build', 'check-targets', '--summary', 'all')
    $targets = @($targetSummary -split "`n" |
        Where-Object { $_ -match '^\+- compile obj breeze_fw_' }).Count

    $text = Get-Content -Raw $readme
    function Get-Claim {
        param([string]$Pattern, [string]$What)
        $match = [regex]::Match($text, $Pattern)
        if (-not $match.Success) { throw "could not parse README's $What claim" }
        return [int]$match.Groups[1].Value
    }

    $rows = @(
        @{ Name = 'kernel tests';     Claim = Get-Claim '\*\*(\d+) 个内核单元测试\*\*' 'kernel test'; Measured = $kernel }
        @{ Name = 'application tests'; Claim = Get-Claim '\*\*(\d+) 个应用测试\*\*' 'application test'; Measured = $app }
        @{ Name = 'algorithm tests';  Claim = Get-Claim '\*\*(\d+) 个算法测试\*\*' 'algorithm test'; Measured = $algorithms }
        @{ Name = 'targets';          Claim = Get-Claim '\*\*(\d+) 个目标交叉编译\*\*' 'target'; Measured = $targets }
    )

    Write-Output ("{0,-20} {1,10} {2,10}   {3}" -f 'count', 'README', 'measured', 'result')
    $failures = 0
    foreach ($row in $rows) {
        $ok = $row.Claim -eq $row.Measured
        if (-not $ok) { $failures++ }
        Write-Output ("{0,-20} {1,10} {2,10}   {3}" -f $row.Name, $row.Claim, $row.Measured, $(if ($ok) { 'ok' } else { 'DRIFT' }))
    }

    if ($failures -gt 0) {
        Write-Output ''
        Write-Output "README's status line no longer matches what the build produces."
        Write-Output 'Update the number in README.md (twice, if it is also in the command list).'
        exit 1
    }

    Write-Output ''
    Write-Output "README's counts match reality."
} finally {
    Pop-Location
}
