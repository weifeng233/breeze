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
const Channel = @import("../kernel/chan.zig").Channel;
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
/// # This pair does not nest
///
/// `cpsid i` / `cpsie i` set and clear PRIMASK unconditionally, so a nested
/// `criticalExit` unmasks interrupts while an outer section still expects them
/// masked. That is acceptable only because **the kernel never nests**: the sole
/// user is `kernel/events.zig`, which has two flat enter/exit pairs and calls
/// neither from within the other.
///
/// If a caller introduces nesting - a task signalling an event from inside a
/// section it already opened - it must save and restore PRIMASK itself:
///
/// ```zig
/// const saved = hal.criticalEnterSaved();
/// defer hal.criticalExitRestore(saved);
/// ```
///
/// The host backend (`hal/host.zig`) tracks a depth counter instead, so the two
/// backends differ here; nesting is a documented limitation rather than a
/// supported behaviour. See ARCHITECTURE.md §9.
pub inline fn criticalEnter() void {
    asm volatile ("cpsid i" ::: .{ .memory = true });
}

/// Unmask configurable interrupts by clearing PRIMASK.
///
/// See `criticalEnter` for why this must not be nested.
pub inline fn criticalExit() void {
    asm volatile ("cpsie i" ::: .{ .memory = true });
}

/// Mask interrupts, returning the previous PRIMASK so it can be restored.
///
/// Use this in place of `criticalEnter`/`criticalExit` when nesting is
/// unavoidable; the kernel itself does not need it.
pub inline fn criticalEnterSaved() u32 {
    const primask: u32 = asm volatile ("mrs %[out], primask"
        : [out] "=r" (-> u32),
    );
    asm volatile ("cpsid i" ::: .{ .memory = true });
    return primask;
}

/// Restore the PRIMASK value returned by `criticalEnterSaved`.
pub inline fn criticalExitRestore(saved: u32) void {
    asm volatile ("msr primask, %[value]"
        :
        : [value] "r" (saved),
        : .{ .memory = true });
}

/// C-callable wrapper for `criticalEnterSaved`.
///
/// Present for two reasons: C code in a mixed project can nest critical
/// sections, and - more importantly for this file - an exported function is
/// emitted unconditionally, so the save/restore assembly above is always
/// assembled and therefore always checked.
///
/// A bare `comptime { _ = &criticalEnterSaved; }` reference does **not**
/// achieve that: it was tried, and a deliberately corrupted mnemonic still
/// compiled, proving the inline assembly had never been analysed.
export fn breeze_critical_enter_saved() callconv(.c) u32 {
    return criticalEnterSaved();
}

/// C-callable wrapper for `criticalExitRestore`.
export fn breeze_critical_exit_restore(saved: u32) callconv(.c) void {
    criticalExitRestore(saved);
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

/// A byte channel with the storage built in.
///
/// This is `kernel/chan.zig`'s `Channel` plus an owned buffer, for the common
/// case where a HAL just wants a receive ring without declaring backing storage
/// at the call site. It exists so there is exactly *one* SPSC implementation in
/// the framework: the index arithmetic, the full/empty rule and the volatile
/// access discipline all live in `Channel`, and this only supplies the array.
///
/// Capacity is one less than the buffer, because a ring that can hold `n` bytes
/// needs `n + 1` slots to distinguish full from empty.
pub const RxRing = struct {
    storage: [slots]u8 = undefined,
    chan: Channel(u8, capacity) = undefined,

    /// Usable bytes. One slot is sacrificed, and `capacity + 1` must be a power
    /// of two so the index wrap stays a mask - the same rule `Channel` enforces.
    pub const capacity: usize = 63;
    const slots = capacity + 1;

    /// Bind the channel to this struct's own buffer.
    ///
    /// Must be called once before use. `RxRing` cannot do it in a field
    /// initialiser because the pointer would not survive the value being moved.
    pub fn init(self: *RxRing) void {
        self.chan = Channel(u8, capacity).init(&self.storage);
    }

    /// Push one byte from interrupt context. Returns false if full.
    pub fn pushFromIsr(self: *RxRing, byte: u8) bool {
        return self.chan.pushFromIsr(byte);
    }

    /// Pop one byte from task context.
    pub fn pop(self: *RxRing) ?u8 {
        return self.chan.pop();
    }

    /// Bytes currently queued.
    pub fn count(self: *const RxRing) u32 {
        return self.chan.count();
    }
};
