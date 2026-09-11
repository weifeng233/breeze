//! Breeze Framework - ARM Cortex-M HAL reference implementation.
//!
//! Targets: Cortex-M0/M0+/M3/M4/M7/M33 (`thumb-freestanding`).
//!
//! This file is intentionally not part of the host-testable module: it is
//! compiled only when building for a Cortex-M target, and verified in CI with
//!
//!     zig build check-targets
//!
//! # Wiring
//!
//! ```zig
//! const hal = @import("breeze").hal.cortex_m;
//! const S = breeze.Scheduler(hal, tasks);
//! var sched = S.init();
//!
//! export fn SysTick_Handler() callconv(.c) void {
//!     hal.tickIsr();          // the ISR's ONLY job
//! }
//!
//! export fn UART0_Handler() callconv(.c) void {
//!     const byte = uart0.data.read();
//!     _ = hal.pushRxByte(byte);        // fill the ring
//!     sched.events.setFromIsr(EVT_UART_RX);  // raise the flag
//! }
//! ```
//!
//! Observe what `SysTick_Handler` does *not* do: it does not run the scheduler,
//! does not allocate, and does not print. The scheduler runs in thread mode.

const tick = @import("../kernel/tick.zig");
const shared = @import("../kernel/shared.zig");
const Tick = tick.Tick;

// --- Cortex-M system control space ----------------------------------------

const systick_base: usize = 0xE000_E010;
const systick_ctrl: *volatile u32 = @ptrFromInt(systick_base + 0x00);
const systick_load: *volatile u32 = @ptrFromInt(systick_base + 0x04);
const systick_val: *volatile u32 = @ptrFromInt(systick_base + 0x08);
const systick_calib: *const volatile u32 = @ptrFromInt(systick_base + 0x0C);

const ctrl_enable: u32 = 1 << 0;
const ctrl_tickint: u32 = 1 << 1;
const ctrl_clksource: u32 = 1 << 2;
const ctrl_countflag: u32 = 1 << 16;

/// Milliseconds accumulated since boot.
var ticks: Tick = 0;

/// Sub-millisecond reload value captured at init.
var reload: u32 = 0;

/// Configure SysTick to interrupt every millisecond.
///
/// `cpu_hz` is the core clock feeding SysTick. The reload value is clamped to
/// the 24-bit SysTick range; a core slower than 1 MHz cannot produce a 1 ms
/// tick this way and is rejected at comptime.
pub fn init(comptime cpu_hz: u32) void {
    comptime {
        if (cpu_hz < 1000) {
            @compileError("cpu_hz must be at least 1 MHz for a 1 ms SysTick reload");
        }
    }
    reload = cpu_hz / 1000 - 1;
    if (reload > 0x00FF_FFFF) reload = 0x00FF_FFFF;

    systick_load.* = reload;
    systick_val.* = 0;
    systick_ctrl.* = ctrl_clksource | ctrl_tickint | ctrl_enable;
}

/// Monotonic milliseconds since boot. Safe to call from an ISR.
///
/// Uses a volatile access rather than an atomic one; see `kernel/shared.zig`
/// for why atomics would make this file unlinkable on Cortex-M0/M0+.
pub inline fn now() Tick {
    return shared.load(Tick, &ticks);
}

/// Advance the timebase. Call from `SysTick_Handler` and nowhere else.
pub inline fn tickIsr() void {
    shared.increment(Tick, &ticks);
}

/// Read the raw SysTick counter, for sub-millisecond timestamps.
pub inline fn fineCounter() u32 {
    return systick_val.*;
}

/// True if the SysTick counter wrapped since the last call.
pub inline fn tickPending() bool {
    return (systick_ctrl.* & ctrl_countflag) != 0;
}

/// CPU frequency the SysTick was configured with, in Hz.
pub inline fn reloadValue() u32 {
    return reload;
}

/// True if SysTick was found to be present and usable.
pub inline fn isAvailable() bool {
    return (systick_calib.* & (1 << 31)) != 0;
}

// --- interrupt control -----------------------------------------------------

/// Mask configurable interrupts by setting PRIMASK.
///
/// Returns nothing because the kernel's contract is strictly nested
/// enter/exit; if a caller needs the previous state it should model that
/// itself. Keeping the pair symmetric means a stray extra `criticalExit`
/// cannot silently unmask interrupts.
pub inline fn criticalEnter() void {
    asm volatile ("cpsid i" ::: .{ .memory = true });
}

/// Unmask configurable interrupts by clearing PRIMASK.
pub inline fn criticalExit() void {
    asm volatile ("cpsie i" ::: .{ .memory = true });
}

/// Wait for an interrupt: the kernel calls this when nothing is runnable.
///
/// `wfi` lets the core stop until the next interrupt, which is what makes a
/// tick-driven superloop power-efficient without any sleep framework.
pub inline fn idle() void {
    asm volatile ("wfi");
}

/// Service the independent watchdog. Override by declaring your own `idle`
/// hook if the project uses a different watchdog.
pub inline fn watchdogKick() void {}

// --- a lock-free single-producer RX ring -----------------------------------

/// Capacity must be a power of two so the index wrap is a mask.
pub const RxRing = struct {
    buf: [capacity]u8 = undefined,
    head: u32 = 0, // written by the ISR only
    tail: u32 = 0, // written by the task only

    pub const capacity: u32 = 64;
    const mask: u32 = capacity - 1;

    comptime {
        if (capacity & (capacity - 1) != 0) {
            @compileError("RxRing.capacity must be a power of two");
        }
    }

    /// Push one byte from interrupt context. Drops the byte if full.
    ///
    /// The data store is published before the index advance, and the ISR cannot
    /// be reordered against itself, so the consumer never observes an index
    /// that points at a byte which has not been written yet.
    pub fn pushFromIsr(self: *RxRing, byte: u8) bool {
        const next = (self.head +% 1) & mask;
        if (next == (self.tail & mask)) return false; // full
        self.buf[self.head & mask] = byte;
        shared.store(u32, &self.head, next);
        return true;
    }

    /// Pop one byte from task context.
    pub fn pop(self: *RxRing) ?u8 {
        const tail = self.tail & mask;
        if (tail == shared.load(u32, &self.head)) return null;
        const byte = self.buf[tail];
        self.tail = (self.tail +% 1) & mask;
        return byte;
    }

    /// Bytes currently queued.
    pub fn count(self: *const RxRing) u32 {
        return (shared.load(u32, &self.head) -% self.tail) & mask;
    }
};
