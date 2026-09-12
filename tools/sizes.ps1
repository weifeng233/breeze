# sizes.ps1 - measure the firmware images and compare them with README.md.
#
# The cost table in README.md is the single source for these numbers: it is what
# a reader quotes and the one place that has to stay true. Nothing used to check
# it, and it drifted three times in a single day - twice because the kernel
# changed, once because tools/elfsize.ps1 was under-reporting RAM by the size of
# `.data`. A table that restates a measurement needs the measurement re-run, so
# this script runs it and fails when the table stops being true.
#
# It deliberately shells out to `zig build-obj` with the same flags the
# reproduction command in docs/ARCHITECTURE.md uses, rather than reusing objects
# produced by `zig build check-targets`: those are built without `-fstrip
# -fno-compiler-rt` and are therefore not the same bytes the table describes.
#
# Usage:
#   pwsh tools/sizes.ps1           # measure, compare with README, fail on drift
#   pwsh tools/sizes.ps1 -Show     # print what was measured, no comparison

param(
    [switch]$Show
)

$ErrorActionPreference = 'Stop'

$repoRoot = [System.IO.Path]::GetFullPath((Join-Path $PSScriptRoot '..'))
$readme = Join-Path $repoRoot 'README.md'
$elfsize = Join-Path $PSScriptRoot 'elfsize.ps1'
$zig = if ($env:ZIG) { $env:ZIG } else { 'zig' }

# The images README quotes. `Label` must appear in that table's row; the row is
# found by label so that re-ordering the table does not break this.
$targets = @(
    @{ Label = 'Cortex-M0 固件骨架'; Bin = 'cm0'
       Root = 'examples/firmware_cortex_m.zig'
       Triple = 'thumb-freestanding-eabi'; Cpu = 'cortex_m0' }
    @{ Label = 'RISC-V32 固件骨架'; Bin = 'rv32'
       Root = 'examples/firmware_riscv.zig'
       Triple = 'riscv32-freestanding-eabi'; Cpu = 'baseline_rv32' }
    @{ Label = '智能车融合固件'; Bin = 'fusion_cyt2bl3'
       Root = 'examples/firmware_smartcar.zig'
       Triple = 'thumb-freestanding-eabihf'; Cpu = 'cortex_m4+vfp4d16sp' }
)

$outDir = Join-Path ([System.IO.Path]::GetTempPath()) ('breeze-sizes-' + [guid]::NewGuid().ToString('N').Substring(0, 8))
New-Item -ItemType Directory -Force -Path $outDir | Out-Null

function Get-Measured {
    param($root, $triple, $cpu, $obj)

    Push-Location $repoRoot
    try {
        $output = & $zig build-obj -target $triple -mcpu $cpu `
            -OReleaseSmall -fstrip -fno-unwind-tables -fno-compiler-rt `
            --dep breeze "-Mroot=$root" "-Mbreeze=src/breeze.zig" `
            "-femit-bin=$obj" 2>&1
        if ($LASTEXITCODE -ne 0) {
            throw "zig build-obj failed for $root ($triple/$cpu):`n$($output -join "`n")"
        }
    } finally {
        Pop-Location
    }

    $report = & pwsh -NoProfile -File $elfsize $obj 2>&1
    $flash = [int](($report | Select-String 'FLASH \(allocated\)\s+(\d+)').Matches.Groups[1].Value | Select-Object -First 1)
    $ram = [int](($report | Select-String 'RAM \(allocated\)\s+(\d+)').Matches.Groups[1].Value | Select-Object -First 1)
    return @{ Flash = $flash; Ram = $ram }
}

# Expected values come from README, parsed rather than duplicated: a second copy
# in this file would be one more thing to drift.
function Get-Claimed {
    param($label)

    $line = Select-String -Path $readme -Pattern ([regex]::Escape($label)) |
        Where-Object { $_.Line -match '\d+ B flash / \d+ B RAM' } |
        Select-Object -First 1
    if (-not $line) {
        throw "README has no cost row for '$label'; the table and this script disagree about what is measured"
    }
    $m = [regex]::Match($line.Line, '(\d+) B flash / (\d+) B RAM')
    return @{ Flash = [int]$m.Groups[1].Value; Ram = [int]$m.Groups[2].Value }
}

$bad = 0
Write-Output ('{0,-22} {1,18} {2,18}' -f 'image', 'flash (claim/now)', 'ram (claim/now)')

foreach ($t in $targets) {
    $obj = Join-Path $outDir ($t.Bin + '.o')
    $now = Get-Measured -root $t.Root -triple $t.Triple -cpu $t.Cpu -obj $obj
    $claim = Get-Claimed -label $t.Label

    $ok = ($now.Flash -eq $claim.Flash) -and ($now.Ram -eq $claim.Ram)
    if (-not $ok) { $bad++ }

    Write-Output ('{0,-22} {1,18} {2,18}  {3}' -f `
        $t.Label, `
        ("{0} / {1}" -f $claim.Flash, $now.Flash), `
        ("{0} / {1}" -f $claim.Ram, $now.Ram), `
        $(if ($ok) { 'ok' } else { 'DRIFT' }))
}

Remove-Item -Recurse -Force $outDir -ErrorAction SilentlyContinue

if ($Show) { exit 0 }

if ($bad -gt 0) {
    Write-Output ''
    Write-Output "$bad image(s) no longer match README.md. Re-measure and update the"
    Write-Output "'实测开销' table - and docs/ARCHITECTURE.md and docs/FUSION.md, which quote the"
    Write-Output 'same measurements.'
    exit 1
}

Write-Output ''
Write-Output 'README cost table matches the images.'
