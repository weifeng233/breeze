# elfsize.ps1 - report flash / RAM cost of an ELF object or executable.
#
# `zig objdump` is a stub in Zig 0.16 ("TODO dump elf file"), so this walks the
# section header table directly. Works for ELF32 and ELF64, which covers
# Cortex-M (ELF32), RISC-V 32/64 and the host.
#
# Usage:
#   pwsh tools/elfsize.ps1 <file.o|file.elf> [more files...]

[CmdletBinding()]
param(
    [Parameter(Mandatory = $true, ValueFromRemainingArguments = $true)]
    [string[]] $Path
)

Set-StrictMode -Version Latest
$ErrorActionPreference = 'Stop'

function Get-ElfSections {
    param([string] $File)

    $bytes = [System.IO.File]::ReadAllBytes($File)
    if ($bytes.Length -lt 0x40 -or $bytes[0] -ne 0x7F -or $bytes[1] -ne 0x45 -or
        $bytes[2] -ne 0x4C -or $bytes[3] -ne 0x46) {
        throw "$File is not an ELF file"
    }

    $is64 = $bytes[4] -eq 2
    if ($is64) {
        $shoff   = [BitConverter]::ToUInt64($bytes, 0x28)
        $entSize = [BitConverter]::ToUInt16($bytes, 0x3A)
        $shnum   = [BitConverter]::ToUInt16($bytes, 0x3C)
        $strIdx  = [BitConverter]::ToUInt16($bytes, 0x3E)
    } else {
        $shoff   = [BitConverter]::ToUInt32($bytes, 0x20)
        $entSize = [BitConverter]::ToUInt16($bytes, 0x2E)
        $shnum   = [BitConverter]::ToUInt16($bytes, 0x30)
        $strIdx  = [BitConverter]::ToUInt16($bytes, 0x32)
    }

    if ($shoff -eq 0 -or $shnum -eq 0) { throw "$File has no section headers" }

    $strBase = $shoff + $strIdx * $entSize
    $strOff = if ($is64) { [BitConverter]::ToUInt64($bytes, $strBase + 0x18) }
              else       { [BitConverter]::ToUInt32($bytes, $strBase + 0x10) }

    $rows  = [System.Collections.Generic.List[object]]::new()
    $flash = [uint64]0
    $ram   = [uint64]0

    for ($i = 0; $i -lt $shnum; $i++) {
        $b = $shoff + $i * $entSize
        $nameOff = [BitConverter]::ToUInt32($bytes, $b)
        $type    = [BitConverter]::ToUInt32($bytes, $b + 4)
        $flags   = if ($is64) { [BitConverter]::ToUInt64($bytes, $b + 8) }
                   else       { [BitConverter]::ToUInt32($bytes, $b + 8) }
        $size    = if ($is64) { [BitConverter]::ToUInt64($bytes, $b + 0x20) }
                   else       { [BitConverter]::ToUInt32($bytes, $b + 0x14) }

        # Only allocated sections cost flash or RAM.
        if ($size -eq 0 -or ($flags -band 0x2) -eq 0) { continue }

        $p = $strOff + $nameOff
        $name = ''
        while ($p -lt $bytes.Length -and $bytes[$p] -ne 0) {
            $name += [char]$bytes[$p]
            $p++
        }

        # SHT_NOBITS (.bss) occupies RAM but no file space.
        $isRam = $type -eq 8
        if ($isRam) { $ram += $size } else { $flash += $size }

        $rows.Add([pscustomobject]@{
            Section = $name
            Size    = $size
            Kind    = if ($isRam) { 'ram' } else { 'flash' }
        })
    }

    [pscustomobject]@{ Rows = $rows; Flash = $flash; Ram = $ram }
}

$any = $false
foreach ($f in $Path) {
    if (-not (Test-Path $f)) { Write-Warning "skip (not found): $f"; continue }
    $r = Get-ElfSections -File $f
    $any = $true

    Write-Output ''
    Write-Output "$f"
    Write-Output ('  {0,-22} {1,10}  {2}' -f 'section', 'size', 'kind')
    foreach ($row in $r.Rows) {
        Write-Output ('  {0,-22} {1,10}  {2}' -f $row.Section, $row.Size, $row.Kind)
    }
    Write-Output ('  {0,-22} {1,10}' -f 'FLASH (allocated)', $r.Flash)
    Write-Output ('  {0,-22} {1,10}' -f 'RAM (allocated)', $r.Ram)
}

if (-not $any) { Write-Warning 'nothing measured' }
