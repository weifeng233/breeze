//! Breeze Framework - hardware abstraction contract.
//!
//! The whole target-specific surface of the kernel is three functions. That is
//! a deliberate reduction from the C framework's HAL, which exposed a single
//! hardware timer (`timer_start`/`timer_expired`) that cannot express N
//! concurrent deadlines, plus a blocking delay that a cooperative scheduler
//! must never call.
//!
//! A HAL is a plain `struct` namespace, not a vtable: the scheduler is
//! parameterised by the HAL *type* at comptime, so every call inlines to a
//! direct function call (or to a register read). There is no indirection to
//! pay for on Cortex-M0.
//!
//! # Required declarations
//!
//! ```zig
//! pub fn now() Tick;                 // monotonic milliseconds since boot
//! pub fn criticalEnter() void;       // mask interrupts that touch kernel state
//! pub fn criticalExit() void;        // restore the previous mask state
//! ```
//!
//! # Optional declarations
//!
//! ```zig
//! pub fn idle() void;                // called when no task is runnable
//! pub fn watchdogKick() void;
//! ```
//!
//! # Rules for interrupt handlers
//!
//! An ISR running on this kernel may only:
//!   1. advance the timebase that `now()` reads,
//!   2. push bytes into a lock-free single-producer ring,
//!   3. set event flags with `EventFlags.setFromIsr`.
//!
//! An ISR must never call the scheduler, allocate, format strings, or invoke
//! any kernel entry point. `setFromIsr` is safe because the ISR is the only
//! writer; readers clear flags inside a critical section.

const std = @import("std");
const Tick = @import("tick.zig").Tick;

/// Compile-time conformance check. Called by the scheduler so that a
/// mis-specified HAL fails with a readable message instead of a type error
/// deep inside the kernel.
///
/// Only the *presence* and function-ness of the declarations is checked here;
/// the exact signatures are enforced by the call sites themselves, which keeps
/// this check robust across Zig releases.
pub fn validate(comptime Hal: type) void {
    inline for (.{ "now", "criticalEnter", "criticalExit" }) |name| {
        if (!@hasDecl(Hal, name)) {
            @compileError("HAL '" ++ @typeName(Hal) ++ "' is missing required declaration `" ++ name ++ "`");
        }
        if (@typeInfo(@TypeOf(@field(Hal, name))) != .@"fn") {
            @compileError("HAL '" ++ @typeName(Hal) ++ "'.`" ++ name ++ "` must be a function");
        }
    }
}

/// True if the HAL offers the optional idle hook.
pub fn hasIdle(comptime Hal: type) bool {
    if (!@hasDecl(Hal, "idle")) return false;
    return @typeInfo(@TypeOf(Hal.idle)) == .@"fn";
}

/// True if the HAL offers the optional watchdog hook.
pub fn hasWatchdog(comptime Hal: type) bool {
    if (!@hasDecl(Hal, "watchdogKick")) return false;
    return @typeInfo(@TypeOf(Hal.watchdogKick)) == .@"fn";
}

/// Entry into a critical section, tolerating a HAL without interrupt masking
/// (the host backend, where the test runner is single-threaded).
pub inline fn criticalEnter(comptime Hal: type) void {
    Hal.criticalEnter();
}

pub inline fn criticalExit(comptime Hal: type) void {
    Hal.criticalExit();
}

test "validate accepts a conforming HAL" {
    const Good = struct {
        pub fn now() Tick {
            return 0;
        }
        pub fn criticalEnter() void {}
        pub fn criticalExit() void {}
    };
    validate(Good);
    try std.testing.expect(!hasIdle(Good));
    try std.testing.expect(!hasWatchdog(Good));
}

test "validate recognises optional hooks" {
    const WithHooks = struct {
        pub fn now() Tick {
            return 0;
        }
        pub fn criticalEnter() void {}
        pub fn criticalExit() void {}
        pub fn idle() void {}
        pub fn watchdogKick() void {}
    };
    validate(WithHooks);
    try std.testing.expect(hasIdle(WithHooks));
    try std.testing.expect(hasWatchdog(WithHooks));
}
