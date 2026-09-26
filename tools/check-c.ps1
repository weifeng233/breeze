# check-c.ps1 - compile, link and run the whole C algorithm layer.
#
# `include/` and `src/` hold the C algorithm library that predates the Zig
# kernel. It stays in the repository as the numerical reference the Zig
# migration is checked against (docs/ARCHITECTURE.md §8), and that only works if
# it actually builds - which it did not:
#
#   * two headers made every translation unit that includes `breeze/breeze.h`
#     fail outright (`BreezeCommBuffer` defined twice as two distinct anonymous
#     structs, and `BreezeSplineInterpolation_Free` used above its own static
#     definition);
#   * seven headers called `malloc`/`free`/`fabsf`/`sqrtf`/`expf` without
#     including `<stdlib.h>`/`<math.h>`, so C fell back to an implicit
#     declaration. That is not a style issue: an implicit `malloc` returns
#     `int`, which truncates the pointer on a 64-bit host, and an implicit
#     `fabsf` is read out of the integer register. The code compiled and then
#     computed the wrong answer.
#
# `zig build ci` never looks at any of this - the C layer is not part of the
# Zig build - so the rot went unnoticed until someone tried to include the
# header. This script is the gate that was missing. It *discovers* files rather
# than listing them, because a hand-maintained list is how the layer rots again:
#
#   headers       every include/breeze/**/*.h compiles standalone under
#                 -Wall -Wextra -Werror. A header that only works when another
#                 header is included first is a defect, not a convention.
#   units         every .c compiles under the same flags.
#   examples      the examples link against -lm and run to completion.
#   applications  the applications link and run. They are interactive - they
#                 read stdin in a loop and quit on `q` - so the `q` is fed in
#                 and the run is bounded by a timeout instead of hanging CI.
#   tests         every tests/**/*.c builds, runs and reports zero failures.
#   corpus        the answers committed in src/math/testdata/ still match a fresh
#                 run of the C code, so the Zig ports are not checked against a
#                 stale oracle (see docs/ARCHITECTURE.md §8).
#
# The counts are then compared with the claim in README.md, for the same reason
# tools/sizes.ps1 checks the cost table: a document that restates a measurement
# goes stale silently.
#
# Usage:
#   pwsh tools/check-c.ps1          # check everything, fail on the first problem
#   pwsh tools/check-c.ps1 -Show    # keep the binaries and print their output

param(
    [switch]$Show,
    [string]$CC = $(if ($env:CC) { $env:CC } else { 'gcc' })
)

$ErrorActionPreference = 'Stop'

# PowerShell 7.4+ can promote a native command's non-zero exit into a
# terminating error. This script reads exit codes itself, so turn that off where
# the preference exists.
if (Get-Variable -Name PSNativeCommandUseErrorActionPreference -ErrorAction SilentlyContinue) {
    $PSNativeCommandUseErrorActionPreference = $false
}

$repoRoot   = [System.IO.Path]::GetFullPath((Join-Path $PSScriptRoot '..'))
$includeDir = Join-Path $repoRoot 'include'
$readme     = Join-Path $repoRoot 'README.md'
$cflags     = @('-std=gnu17', '-Wall', '-Wextra', '-Werror')
$onWindows  = ($env:OS -eq 'Windows_NT')

$outDir = Join-Path ([System.IO.Path]::GetTempPath()) ('breeze-ccheck-' + [guid]::NewGuid().ToString('N').Substring(0, 8))
New-Item -ItemType Directory -Force -Path $outDir | Out-Null

$failures = [System.Collections.Generic.List[string]]::new()

# Run a tool (compiler) and return its exit code plus captured output.
function Invoke-Tool {
    param([string]$Exe, [string[]]$Arguments)
    Push-Location $repoRoot
    try {
        $text = & $Exe @Arguments 2>&1
        $code = $LASTEXITCODE
    } finally {
        Pop-Location
    }
    return @{ Code = $code; Text = ($text | Out-String) }
}

# Start a program, feed it stdin, and bound how long it may take. A program that
# hangs is a failure to report, not a reason to hang CI.
function Invoke-Program {
    param([string]$Exe, [string]$StdIn, [int]$TimeoutMs = 30000)

    $psi = [System.Diagnostics.ProcessStartInfo]::new()
    $psi.FileName = $Exe
    $psi.UseShellExecute = $false
    $psi.RedirectStandardInput = $true
    $psi.RedirectStandardOutput = $true
    $psi.RedirectStandardError = $true
    # Sources are UTF-8 (the examples print Chinese); decode as such on every
    # host rather than inheriting the console code page.
    $utf8 = [System.Text.UTF8Encoding]::new($false)
    $psi.StandardOutputEncoding = $utf8
    $psi.StandardErrorEncoding = $utf8

    $p = [System.Diagnostics.Process]::Start($psi)

    # Start draining both pipes *before* waiting, and wait with a deadline.
    # Reading to end first looks natural and is wrong: a program that hangs
    # without printing anything blocks that read forever, so the timeout below
    # would never be reached - the check would hang CI instead of reporting the
    # hang it exists to catch.
    $outTask = $p.StandardOutput.ReadToEndAsync()
    $errTask = $p.StandardError.ReadToEndAsync()

    if ($StdIn) { $p.StandardInput.Write($StdIn) }
    $p.StandardInput.Close()

    $timedOut = -not $p.WaitForExit($TimeoutMs)
    if ($timedOut) {
        try { $p.Kill($true) } catch { }
        try { $p.WaitForExit(5000) } catch { }
    }
    $code = if ($timedOut) { -1 } else { $p.ExitCode }
    # The pipes are closed by now (killed or exited), so these complete; the
    # bound is belt and braces.
    $out = if ($outTask.Wait(5000)) { $outTask.Result } else { '' }
    $err = if ($errTask.Wait(5000)) { $errTask.Result } else { '' }
    $p.Dispose()

    return @{ Code = $code; Out = $out; Err = $err; TimedOut = $timedOut }
}

function Get-ExePath {
    param([string]$Name)
    if ($onWindows) { Join-Path $outDir ($Name + '.exe') } else { Join-Path $outDir $Name }
}

function Write-Diagnostics {
    param([string]$Text, [int]$Lines = 5)
    $errs = @($Text -split "`n" | Where-Object { $_ -match 'error:' } | Select-Object -First $Lines)
    if ($errs.Count -eq 0) { $errs = @($Text -split "`n" | Where-Object { $_.Trim() } | Select-Object -First $Lines) }
    foreach ($e in $errs) { Write-Output ('    ' + $e.Trim()) }
}

$banner = (& $CC --version 2>&1 | Select-Object -First 1)
Write-Output "cc:    $banner"
Write-Output "flags: $($cflags -join ' ')"
Write-Output ''

# ---------------------------------------------------------------- headers ---
# One translation unit per header, each in its own file: reusing a single file
# races with the compiler and produced a survey where every header reported
# exactly one error.
$headers = @(Get-ChildItem -Path (Join-Path $includeDir 'breeze') -Recurse -Filter *.h -File | Sort-Object FullName)
$headerBad = 0
foreach ($h in $headers) {
    $rel = $h.FullName.Substring($includeDir.Length + 1).Replace('\', '/')
    $tu  = Join-Path $outDir ('h_' + ($rel -replace '[/.]', '_') + '.c')
    Set-Content -Path $tu -Value "#include `"$rel`"" -Encoding ascii

    $r = Invoke-Tool -Exe $CC -Arguments ($cflags + @('-fsyntax-only', '-I', $includeDir, $tu))
    if ($r.Code -ne 0) {
        $headerBad++
        $failures.Add("header $rel does not compile standalone")
        Write-Output "--- $rel (does not compile standalone)"
        Write-Diagnostics -Text $r.Text
    }
}
Write-Output ("headers      {0,3} found, {1} failing" -f $headers.Count, $headerBad)

# ------------------------------------------------------------------ units ---
$sourceDirs = @('src', 'examples', 'applications', 'tests') | ForEach-Object { Join-Path $repoRoot $_ }
$units = @(Get-ChildItem -Path ($sourceDirs + $includeDir) -Recurse -Filter *.c -File | Sort-Object FullName)
$unitBad = 0
foreach ($u in $units) {
    $rel = $u.FullName.Substring($repoRoot.Length + 1)
    $obj = Join-Path $outDir (($rel -replace '[/\\]', '_') + '.o')
    $r = Invoke-Tool -Exe $CC -Arguments ($cflags + @('-I', $includeDir, '-I', (Join-Path $repoRoot 'src'), '-c', $u.FullName, '-o', $obj))
    if ($r.Code -ne 0) {
        $unitBad++
        $failures.Add("translation unit $rel does not compile")
        Write-Output "--- $rel (does not compile)"
        Write-Diagnostics -Text $r.Text
    }
}
Write-Output ("units        {0,3} found, {1} failing" -f $units.Count, $unitBad)

# ----------------------------------------------------------------- corpus ---
# The C library is the oracle the ported Zig modules are checked against
# (ARCHITECTURE.md §8), and an oracle that drifts is worse than none: the Zig
# tests would keep passing against yesterday's answers. So the generator is
# re-run and its output compared with the committed copy. Regenerating is a
# deliberate act, and this is what makes it deliberate.
$corpusSrc = Join-Path $repoRoot 'tools/corpus/gen_math_corpus.c'
$corpusFile = Join-Path $repoRoot 'src/math/testdata/math_corpus.txt'
if ((Test-Path $corpusSrc) -and -not (Test-Path $corpusFile)) {
    # A generator with nowhere to write is not "nothing to check": the Zig tests
    # would fail to compile instead, with a less obvious message.
    $failures.Add("$corpusFile is missing but the generator that produces it exists")
    Write-Output '--- corpus: generator present, committed answers missing'
}
if ((Test-Path $corpusSrc) -and (Test-Path $corpusFile)) {
    $gen = Get-ExePath -Name 'gen_math_corpus'
    $built = Invoke-Tool -Exe $CC -Arguments ($cflags + @($corpusSrc, '-o', $gen, '-lm'))
    if ($built.Code -ne 0) {
        $failures.Add('the corpus generator does not compile')
        Write-Output '--- tools/corpus/gen_math_corpus.c (does not compile)'
        Write-Diagnostics -Text $built.Text
    } else {
        $run = Invoke-Program -Exe $gen -StdIn ''
        if ($run.Code -ne 0) {
            $failures.Add("the corpus generator exited with $($run.Code)")
            Write-Output "--- gen_math_corpus (exit $($run.Code))"
        } else {
            # Line endings are normalised away: the file is committed with LF and
            # a checkout elsewhere may not be.
            $normalise = {
                param($s)
                (($s -split "`r?`n" | ForEach-Object { $_.TrimEnd() }) -join "`n").Trim()
            }
            $fresh = & $normalise $run.Out
            $committed = & $normalise (Get-Content -Raw $corpusFile)
            if ($fresh -ne $committed) {
                $failures.Add('src/math/testdata/math_corpus.txt no longer matches a fresh run of the C generator')
                Write-Output '--- corpus DRIFT: the committed answers differ from the C library'
                $freshLines = $fresh -split "`n"
                $committedLines = $committed -split "`n"
                for ($i = 0; $i -lt [Math]::Max($freshLines.Count, $committedLines.Count); $i++) {
                    $a = if ($i -lt $committedLines.Count) { $committedLines[$i] } else { '<missing>' }
                    $b = if ($i -lt $freshLines.Count) { $freshLines[$i] } else { '<missing>' }
                    if ($a -ne $b) {
                        Write-Output ("    line {0}: committed '{1}' / fresh '{2}'" -f ($i + 1), $a, $b)
                        break
                    }
                }
                Write-Output '    regenerate with tools/corpus/gen_math_corpus.c and commit the result,'
                Write-Output '    then check that the Zig ports still agree with it.'
            } else {
                Write-Output 'corpus       the committed C answers still match a fresh run'
            }
        }
    }
}

# --------------------------------------------------------------- programs ---
# Programs are discovered by convention: examples at the top level of
# examples/, applications one directory down, tests anywhere under tests/.
$programs = @()
foreach ($f in @(Get-ChildItem -Path (Join-Path $repoRoot 'examples') -Filter *.c -File | Sort-Object Name)) {
    $programs += @{ Kind = 'example'; Name = $f.BaseName; Src = $f.FullName; Extra = @() }
}
foreach ($f in @(Get-ChildItem -Path (Join-Path $repoRoot 'applications') -Recurse -Filter *.c -File | Sort-Object Name)) {
    $programs += @{ Kind = 'application'; Name = $f.BaseName; Src = $f.FullName; Extra = @() }
}
foreach ($f in @(Get-ChildItem -Path (Join-Path $repoRoot 'tests') -Recurse -Filter *.c -File | Sort-Object Name)) {
    # The test framework's globals live in a .c file of their own.
    $programs += @{ Kind = 'test'; Name = $f.BaseName; Src = $f.FullName
                    Extra = @((Join-Path $includeDir 'breeze/core/globals.c')) }
}

$programBad = 0
$suiteCounts = @{}
foreach ($p in $programs) {
    $exe = Get-ExePath -Name ($p.Kind + '_' + $p.Name)
    $r = Invoke-Tool -Exe $CC -Arguments ($cflags + @('-I', $includeDir, $p.Src) + $p.Extra + @('-o', $exe, '-lm'))
    if ($r.Code -ne 0) {
        $programBad++
        $failures.Add("$($p.Kind) $($p.Name) does not link")
        Write-Output "--- $($p.Kind) $($p.Name) (does not link)"
        Write-Diagnostics -Text $r.Text
        continue
    }

    # Interactive programs read stdin; `q` is the documented quit key in both
    # applications. Detected from the source so a new interactive program is
    # covered without editing this script.
    $src = Get-Content -Raw $p.Src
    $stdin = if ($src -match '\bscanf\s*\(' -or $src -match '\bgetchar\s*\(') { "q`n" } else { '' }

    $run = Invoke-Program -Exe $exe -StdIn $stdin
    if ($run.TimedOut) {
        $programBad++
        $failures.Add("$($p.Kind) $($p.Name) did not finish within 30s")
        Write-Output "--- $($p.Kind) $($p.Name) (timed out; stdin '$($stdin.Trim())')"
        continue
    }
    if ($run.Code -ne 0) {
        $programBad++
        $failures.Add("$($p.Kind) $($p.Name) exited with $($run.Code)")
        Write-Output "--- $($p.Kind) $($p.Name) (exit $($run.Code))"
        Write-Diagnostics -Text $run.Err -Lines 3
        continue
    }
    if ($run.Out.Trim().Length -eq 0) {
        $programBad++
        $failures.Add("$($p.Kind) $($p.Name) produced no output")
        Write-Output "--- $($p.Kind) $($p.Name) (exit 0 but no output)"
        continue
    }

    if ($Show) {
        Write-Output "--- $($p.Kind) $($p.Name)"
        ($run.Out -split "`n" | Select-Object -First 6) | ForEach-Object { Write-Output ('    ' + $_.TrimEnd()) }
    }

    if ($p.Kind -eq 'test') {
        $m = [regex]::Match($run.Out, 'Tests:\s+(\d+) run,\s+(\d+) passed,\s+(\d+) failed')
        $a = [regex]::Match($run.Out, 'Assertions:\s+(\d+) run,\s+(\d+) passed,\s+(\d+) failed')
        if (-not $m.Success -or -not $a.Success) {
            $programBad++
            $failures.Add("$($p.Name): could not parse the test summary")
            Write-Output "--- $($p.Name): could not parse the test summary"
            continue
        }
        if ([int]$m.Groups[3].Value -ne 0 -or [int]$a.Groups[3].Value -ne 0) {
            $programBad++
            $failures.Add("$($p.Name): $($m.Groups[3].Value) test(s) and $($a.Groups[3].Value) assertion(s) failed")
            Write-Output "--- $($p.Name): FAILURES"
            ($run.Out -split "`n" | Where-Object { $_ -match 'FAILED|failed' }) | ForEach-Object { Write-Output ('    ' + $_.Trim()) }
        }
        $suiteCounts[$p.Name] = @{
            Tests      = [int]$m.Groups[1].Value
            Assertions = [int]$a.Groups[1].Value
        }
        Write-Output ("{0,-12} {1,3} tests, {2} assertions, {3} failed" -f $p.Name, $m.Groups[1].Value, $a.Groups[1].Value, $m.Groups[3].Value)
    } else {
        Write-Output ("{0,-12} {1,-22} ran, {2} lines of output" -f $p.Kind, $p.Name, @($run.Out -split "`n").Count)
    }
}

# ------------------------------------------------------- README's claims ---
# README is the single source for these counts. Bold markers are stripped first
# so that re-bolding a number does not break the check.
$claimText = (Get-Content -Raw $readme) -replace '\*\*', ''
$claim = [regex]::Match($claimText, '(\d+) 个头文件、(\d+) 个示例、(\d+) 个应用、(\d+) 个通信测试（(\d+) 条断言）')
if (-not $claim.Success) {
    $failures.Add('README.md no longer states the C layer counts this script checks')
    Write-Output ''
    Write-Output 'Could not find the C layer claim in README.md (expected a line shaped like'
    Write-Output '"35 个头文件、5 个示例、2 个应用、11 个通信测试（42 条断言）").'
} else {
    $examples     = @($programs | Where-Object { $_.Kind -eq 'example' }).Count
    $applications = @($programs | Where-Object { $_.Kind -eq 'application' }).Count
    $tests        = @($programs | Where-Object { $_.Kind -eq 'test' }).Count

    $measured = @{
        Headers      = $headers.Count
        Examples     = $examples
        Applications = $applications
    }
    if ($tests -eq 1) {
        $measured.CTests      = $suiteCounts.Values[0].Tests
        $measured.CAssertions = $suiteCounts.Values[0].Assertions
    }

    $pairs = @(
        @{ Name = 'headers';      Claim = [int]$claim.Groups[1].Value; Now = $measured.Headers }
        @{ Name = 'examples';     Claim = [int]$claim.Groups[2].Value; Now = $measured.Examples }
        @{ Name = 'applications'; Claim = [int]$claim.Groups[3].Value; Now = $measured.Applications }
        @{ Name = 'comm tests';   Claim = [int]$claim.Groups[4].Value; Now = $measured.CTests }
        @{ Name = 'assertions';   Claim = [int]$claim.Groups[5].Value; Now = $measured.CAssertions }
    )

    Write-Output ''
    Write-Output ('{0,-14} {1,16} {2,10}' -f 'count', 'README / measured', 'result')
    foreach ($p in $pairs) {
        $ok = ($p.Now -ne $null) -and ($p.Claim -eq $p.Now)
        if (-not $ok) { $failures.Add("README claims $($p.Claim) $($p.Name), measured $($p.Now)") }
        Write-Output ('{0,-14} {1,16} {2,10}' -f $p.Name, ("{0} / {1}" -f $p.Claim, $p.Now), $(if ($ok) { 'ok' } else { 'DRIFT' }))
    }
}

if (-not $Show) { Remove-Item -Recurse -Force $outDir -ErrorAction SilentlyContinue }

if ($failures.Count -gt 0) {
    Write-Output ''
    Write-Output "$($failures.Count) problem(s):"
    foreach ($f in $failures) { Write-Output "  - $f" }
    exit 1
}

Write-Output ''
Write-Output 'C layer compiles, links, runs, and matches README.'
