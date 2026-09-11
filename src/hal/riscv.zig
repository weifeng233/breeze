//! Breeze Framework - RISC-V HAL reference implementation.
//!
//! Targets: RV32I/RV32IMAC and RV64 (`riscv32-freestanding`,
//! `riscv64-freestanding`), machine mode.
//!
//! Because RISC-V has no standard memory map, the platform specifics are
//! comptime parameters rather than hardcoded addresses:
//!
//! ```zig
//! const hal = @import("breeze").hal.riscv;
//! hal.configure(.{
//!     .mtime_addr = 0x0200_BFF8,   // SiFive CLINT
//!     .timer_hz = 32_768,          // mtime frequency
//! });
//! ```
//!
//! # Wiring
//!
//! The machine timer interrupt is delivered through `mtvec`; the handler's only
//! kernel duty is to advance the timebase:
//!
//! ```zig
//! export fn machineTimerHandler() callconv(.c) void {
//!     hal.tickIsr();
//!     hal.rearmTimer();
//! }
//! ```

const tick = @import("../kernel/tick.zig");
const shared = @import("../kernel/shared.zig");
const Tick = tick.Tick;
const builtin = @import("builtin");

pub const Config = struct {
    /// Address of the 64-bit `mtime` register.
    mtime_addr: usize = 0x0200_BFF8,
    /// Frequency of `mtime`, in Hz. Used to convert to milliseconds.
    timer_hz: u32 = 32_768,
    /// Address of the 64-bit `mtimecmp` register.
    mtimecmp_addr: usize = 0x0200_4000,
};

var cfg: Config = .{};

/// Apply platform parameters. Call once before `init`.
pub fn configure(c: Config) void {
    cfg = c;
}

/// Milliseconds accumulated since boot.
var ticks: Tick = 0;

/// Configure the machine timer to interrupt every millisecond.
pub fn init() void {
    rearmTimer();
}

/// Monotonic milliseconds since boot. Safe to call from an ISR.
///
/// Uses a volatile access rather than an atomic one; see `kernel/shared.zig`.
/// `baseline_rv32` happens to inline monotonic atomics, but `generic_rv32`
/// does not, so relying on it would make the kernel's linkability depend on the
/// chosen `-mcpu`.
pub inline fn now() Tick {
    return shared.load(Tick, &ticks);
}

/// Advance the timebase. Call from the machine timer handler and nowhere else.
pub inline fn tickIsr() void {
    shared.increment(Tick, &ticks);
}

/// Schedule the next machine-timer interrupt one millisecond from now.
pub fn rearmTimer() void {
    const now_raw = rawTimer();
    const period: u64 = cfg.timer_hz / 1000;
    writeMtimecmp(now_raw +% period);
}

/// Read the raw 64-bit platform timer.
///
/// On RV32 a 64-bit read is not atomic, so the high word is read twice and the
/// sequence retried if it changed. Without this, a read that straddles a carry
/// from the low word yields a timestamp almost 2^32 ticks in the future.
pub fn rawTimer() u64 {
    const hi_ptr: *const volatile u32 = @ptrFromInt(cfg.mtime_addr + 4);
    const lo_ptr: *const volatile u32 = @ptrFromInt(cfg.mtime_addr);

    if (builtin.cpu.arch == .riscv64) {
        const full: *const volatile u64 = @ptrFromInt(cfg.mtime_addr);
        return full.*;
    }

    var hi: u32 = undefined;
    var lo: u32 = undefined;
    var hi_again: u32 = undefined;
    while (true) {
        hi = hi_ptr.*;
        lo = lo_ptr.*;
        hi_again = hi_ptr.*;
        if (hi == hi_again) break;
    }
    return (@as(u64, hi) << 32) | @as(u64, lo);
}

fn writeMtimecmp(value: u64) void {
    const hi_ptr: *volatile u32 = @ptrFromInt(cfg.mtimecmp_addr + 4);
    const lo_ptr: *volatile u32 = @ptrFromInt(cfg.mtimecmp_addr);

    if (builtin.cpu.arch == .riscv64) {
        const full: *volatile u64 = @ptrFromInt(cfg.mtimecmp_addr);
        full.* = value;
        return;
    }

    // Write the high word to all-ones first so the timer cannot fire on a
    // half-updated comparison while the low word is being written.
    hi_ptr.* = 0xFFFF_FFFF;
    lo_ptr.* = @truncate(value);
    hi_ptr.* = @truncate(value >> 32);
}

// --- interrupt control -----------------------------------------------------

/// Mask machine interrupts by clearing MIE in mstatus.
///
/// The kernel's contract is strictly nested enter/exit; the pair is
/// intentionally unconditional so that a stray extra `criticalExit` cannot
/// leave interrupts in an unexpected state.
pub inline fn criticalEnter() void {
    asm volatile ("csrrci zero, mstatus, 8" ::: .{ .memory = true }); // MIE = bit 3
}

/// Unmask machine interrupts by setting MIE in mstatus.
pub inline fn criticalExit() void {
    asm volatile ("csrrsi zero, mstatus, 8" ::: .{ .memory = true });
}

/// Wait for an interrupt. `wfi` stops the hart until one arrives.
pub inline fn idle() void {
    asm volatile ("wfi");
}

/// Service the watchdog; override per platform.
pub inline fn watchdogKick() void {}

/// Enable machine timer interrupts (MIE.MTIE).
///
/// The mask goes in a register rather than using the immediate form of `csrrs`:
/// `csrrsi` encodes its operand in 5 bits, so bit 7 (MTIE) and bit 11 (MEIE)
/// do not fit and the assembler rejects them.
pub fn enableTimerInterrupt() void {
    const mask: usize = 1 << 7; // MTIE
    asm volatile ("csrrs zero, mie, %[mask]"
        :
        : [mask] "r" (mask),
        : .{ .memory = true });
}

/// Enable machine external interrupts (MIE.MEIE).
pub fn enableExternalInterrupt() void {
    const mask: usize = 1 << 11; // MEIE
    asm volatile ("csrrs zero, mie, %[mask]"
        :
        : [mask] "r" (mask),
        : .{ .memory = true });
}

/// Bytes queued count for a single-producer ring.
///
/// The ring itself is architecture independent, so it is defined once and
/// re-exported here for symmetry with the Cortex-M HAL.
pub const RxRing = @import("cortex_m.zig").RxRing;
