# vendor.ps1 - copy the Breeze kernel into another repository.
#
# Vendoring exists so a consumer can build without a package manager or a
# relative path back to this checkout. Its cost is that the copy can silently
# drift from the source, so this script writes a VENDORED.md recording the
# commit it copied from and a hash of every file. Re-running it on an unchanged
# Breeze tree produces a byte-identical result; `-Check` reports drift instead
# of writing.
#
# Usage:
#   pwsh tools/vendor.ps1 -Dest <dir>            # copy into <dir>
#   pwsh tools/vendor.ps1 -Dest <dir> -Check     # verify an existing copy
#
# The destination receives:
#   <dir>/breeze.zig        kernel-only root (from src/breeze_kernel.zig)
#   <dir>/app.zig
#   <dir>/kernel/*.zig
#   <dir>/VENDORED.md

[CmdletBinding()]
param(
    [Parameter(Mandatory = $true)]
    [string] $Dest,
    [switch] $Check
)

Set-StrictMode -Version Latest
$ErrorActionPreference = 'Stop'

$repoRoot = [System.IO.Path]::GetFullPath((Join-Path $PSScriptRoot '..'))

# The kernel sources, in the order they are copied.
#
# `hal/host.zig` is included even though a consumer supplies its own HAL, and
# even though nothing in a firmware build calls it. The reason is a Zig
# behaviour that is easy to trip over: **`@import` inside a `test` block is
# resolved even when building an object rather than running tests.** Verified by
# building a file whose test block imported a missing path - the build failed
# with `unable to load`. Several kernel files have test fixtures that import
# `../hal/host.zig`, so omitting it makes the vendored tree unusable.
#
# Shipping it costs nothing: `build-obj` emits no code for it, and it is the
# backend the kernel's own tests run against, so a consumer who wants to run
# those tests has everything they need.
$kernelFiles = @(
    'tick.zig',
    'events.zig',
    'program.zig',
    'scheduler.zig',
    'hal.zig',
    'shared.zig',
    'chan.zig',
    'topic.zig'
)

# Platform backends copied alongside the kernel, as `hal/<name>`.
$halFiles = @(
    'host.zig'
)

$rootFile = 'src/breeze_kernel.zig'
$appFile = 'src/app.zig'

function Get-FileHashHex {
    param([string] $Path)
    (Get-FileHash -Path $Path -Algorithm SHA256).Hash.ToLowerInvariant()
}

# --- describe the source tree ----------------------------------------------

$entries = [System.Collections.Generic.List[object]]::new()

foreach ($name in $kernelFiles) {
    $src = Join-Path $repoRoot "src/kernel/$name"
    if (-not (Test-Path $src)) { throw "missing kernel source: $src" }
    $entries.Add([pscustomobject]@{
        Source = "src/kernel/$name"
        Dest   = "kernel/$name"
        Full   = $src
    })
}
foreach ($name in $halFiles) {
    $src = Join-Path $repoRoot "src/hal/$name"
    if (-not (Test-Path $src)) { throw "missing HAL source: $src" }
    $entries.Add([pscustomobject]@{
        Source = "src/hal/$name"
        Dest   = "hal/$name"
        Full   = $src
    })
}
$entries.Add([pscustomobject]@{
    Source = $rootFile
    Dest   = 'breeze.zig'
    Full   = (Join-Path $repoRoot $rootFile)
})
$entries.Add([pscustomobject]@{
    Source = $appFile
    Dest   = 'app.zig'
    Full   = (Join-Path $repoRoot $appFile)
})

foreach ($e in $entries) {
    if (-not (Test-Path $e.Full)) { throw "missing source: $($e.Full)" }
}

# --- check mode -------------------------------------------------------------

if ($Check) {
    $problems = 0
    foreach ($e in $entries) {
        $target = Join-Path $Dest $e.Dest
        if (-not (Test-Path $target)) {
            Write-Output "MISSING  $($e.Dest)"
            $problems++
            continue
        }
        $want = Get-FileHashHex $e.Full
        $got = Get-FileHashHex $target
        if ($want -ne $got) {
            Write-Output "DIFFERS  $($e.Dest)"
            $problems++
        }
    }
    $manifest = Join-Path $Dest 'VENDORED.md'
    if (-not (Test-Path $manifest)) {
        Write-Output 'MISSING  VENDORED.md'
        $problems++
    }
    if ($problems -eq 0) {
        Write-Output "in sync with $repoRoot ($($entries.Count) files)"
        exit 0
    }
    Write-Output "$problems problem(s); re-run without -Check to refresh"
    exit 1
}

# --- copy -------------------------------------------------------------------

$kernelDir = Join-Path $Dest 'kernel'
New-Item -ItemType Directory -Force -Path $Dest, $kernelDir | Out-Null
foreach ($e in $entries) {
    $target = Join-Path $Dest $e.Dest
    New-Item -ItemType Directory -Force -Path (Split-Path $target) | Out-Null
    Copy-Item -Force $e.Full $target
}

# --- provenance -------------------------------------------------------------

$commit = 'unknown'
$describe = 'unknown'
Push-Location $repoRoot
try {
    $commit = (git rev-parse HEAD 2>$null)
    $describe = (git describe --always --dirty 2>$null)
} catch {
    # Not a git checkout: vendoring still works, provenance is just weaker.
} finally {
    Pop-Location
}

$lines = [System.Collections.Generic.List[string]]::new()
$lines.Add('# Vendored Breeze kernel')
$lines.Add('')
$lines.Add('**Do not edit these files by hand.** They are copies produced by')
$lines.Add('`tools/vendor.ps1` in the Breeze repository, so that a consumer can build')
$lines.Add('without a package manager. Edit Breeze and re-run the script; the')
$lines.Add('`-Check` mode reports drift.')
$lines.Add('')
$lines.Add("| | |")
$lines.Add('|---|---|')
$lines.Add("| vendor script | ``tools/vendor.ps1`` |")
$lines.Add("| copied at     | $([DateTime]::UtcNow.ToString('yyyy-MM-dd HH:mm:ss')) UTC |")
$lines.Add("| commit        | ``$commit`` |")
$lines.Add("| describe      | ``$describe`` |")
$lines.Add('')
$lines.Add('## Files')
$lines.Add('')
$lines.Add('| file | sha256 (first 16) |')
$lines.Add('|---|---|')
foreach ($e in $entries) {
    $hash = (Get-FileHashHex $e.Full).Substring(0, 16)
    $lines.Add("| ``$($e.Dest)`` | ``$hash`` |")
}
$lines.Add('')
$lines.Add('## Not vendored, on purpose')
$lines.Add('')
$lines.Add('- `hal/cortex_m.zig` and `hal/riscv.zig`. A consumer supplies its own HAL.')
$lines.Add('  That is what the three-function contract is for.')
$lines.Add('- The C algorithm library under `include/` and `src/`. It is not part of')
$lines.Add('  the kernel and is not currently compilable.')
$lines.Add('')
$lines.Add('## Why `hal/host.zig` is present')
$lines.Add('')
$lines.Add('Nothing in a firmware build calls it, but Zig resolves `@import` inside')
$lines.Add('`test` blocks even when building an object rather than running tests. The')
$lines.Add('kernel''s test fixtures import it, so leaving it out makes the vendored tree')
$lines.Add('fail to compile. Remove it only after removing those fixtures.')
$lines.Add('')
$lines.Add('## Licence')
$lines.Add('')
$lines.Add('MIT. See the Breeze repository for the full text.')

Set-Content -Path (Join-Path $Dest 'VENDORED.md') -Value $lines -Encoding utf8

Write-Output "vendored $($entries.Count) files into $Dest"
Write-Output "  commit $describe"
foreach ($e in $entries) {
    Write-Output "    $($e.Dest)"
}
