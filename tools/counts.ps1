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
# The fifth row is the frozen corpus's case count, which was *found* stale while
# writing REVIEW §55: the README and both ARCHITECTURE mentions still said 556 after
# three cases had been deleted. It is checked in all three places for the same
# reason the others are - a number restated in prose rots, and fixing it by hand
# only schedules the next drift.
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
$architecture = Join-Path $repoRoot 'docs/ARCHITECTURE.md'
$corpus = Join-Path $repoRoot 'src/math/testdata/math_corpus.txt'

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

    $readmeText = Get-Content -Raw $readme
    $architectureText = Get-Content -Raw $architecture

    # The corpus holds one case per line; its header is the only thing commented out.
    $corpusCases = @(Get-Content $corpus | Where-Object { $_ -notmatch '^\s*#' -and $_.Trim() -ne '' }).Count

    function Get-Claim {
        param([string]$Text, [string]$Pattern, [string]$What)
        $match = [regex]::Match($Text, $Pattern)
        if (-not $match.Success) { throw "could not parse the $What claim" }
        return [int]$match.Groups[1].Value
    }

    $rows = @(
        @{ Name = 'kernel tests';     Claim = Get-Claim $readmeText '\*\*(\d+) 个内核单元测试\*\*' 'README kernel test'; Measured = $kernel }
        @{ Name = 'application tests'; Claim = Get-Claim $readmeText '\*\*(\d+) 个应用测试\*\*' 'README application test'; Measured = $app }
        @{ Name = 'algorithm tests';  Claim = Get-Claim $readmeText '\*\*(\d+) 个算法测试\*\*' 'README algorithm test'; Measured = $algorithms }
        @{ Name = 'targets';          Claim = Get-Claim $readmeText '\*\*(\d+) 个目标交叉编译\*\*' 'README target'; Measured = $targets }
        @{ Name = 'corpus (README)';  Claim = Get-Claim $readmeText '它的 (\d+) 条答案' 'README corpus'; Measured = $corpusCases }
        @{ Name = 'corpus (ARCH §8)'; Claim = Get-Claim $architectureText '提交进仓库（(\d+) 条用例）' 'ARCHITECTURE corpus'; Measured = $corpusCases }
        @{ Name = 'corpus (ARCH §9)'; Claim = Get-Claim $architectureText '算法层的 (\d+) 条对照值' 'ARCHITECTURE limitation'; Measured = $corpusCases }
    )

    Write-Output ("{0,-20} {1,10} {2,10}   {3}" -f 'count', 'claimed', 'measured', 'result')
    $failures = 0
    foreach ($row in $rows) {
        $ok = $row.Claim -eq $row.Measured
        if (-not $ok) { $failures++ }
        Write-Output ("{0,-20} {1,10} {2,10}   {3}" -f $row.Name, $row.Claim, $row.Measured, $(if ($ok) { 'ok' } else { 'DRIFT' }))
    }

    if ($failures -gt 0) {
        Write-Output ''
        Write-Output 'A documented count no longer matches what the repository produces.'
        Write-Output 'Update it in README.md (twice, if it is also in the command list) or in docs/ARCHITECTURE.md.'
        exit 1
    }

    Write-Output ''
    Write-Output "Every documented count matches reality."
} finally {
    Pop-Location
}
