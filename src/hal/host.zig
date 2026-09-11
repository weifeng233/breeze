//! Breeze Framework - host HAL: a virtual clock for tests, examples and
//! simulation.
//!
//! This backend makes the kernel testable on a workstation with no sleeping and
//! no threads: tests move the clock explicitly and then run the scheduler, so a
//! ten-minute firmware scenario executes in microseconds and is perfectly
//! reproducible. The same scheduler binary logic is what runs on the target;
//! only this file differs.
//!
//! It also counts critical-section nesting, which lets tests assert that the
//! kernel actually masks interrupts where it claims to.

const std = @import("std");
const tick = @import("../kernel/tick.zig");
const Tick = tick.Tick;

/// Virtual-clock HAL. Not thread safe by design - the whole point is that the
/// test harness is the only thing driving time.
pub const HostHal = struct {
    var now_ms: Tick = 0;
    var crit_depth: u32 = 0;
    var crit_entries: u32 = 0;
    var idle_calls: u32 = 0;

    /// Current virtual time in milliseconds.
    pub fn now() Tick {
        return now_ms;
    }

    /// Enter a critical section.
    ///
    /// On a single-threaded host there is nothing to mask, but the depth is
    /// tracked so tests can verify the kernel's locking discipline.
    pub fn criticalEnter() void {
        crit_depth += 1;
        crit_entries += 1;
    }

    /// Leave a critical section.
    pub fn criticalExit() void {
        std.debug.assert(crit_depth > 0);
        crit_depth -= 1;
    }

    /// Optional kernel hook: called when no task is runnable.
    pub fn idle() void {
        idle_calls += 1;
    }

    /// Move the virtual clock forward.
    pub fn advance(ms: u32) void {
        now_ms +%= ms;
    }

    /// Jump the virtual clock to an absolute time.
    pub fn setNow(t: Tick) void {
        now_ms = t;
    }

    /// Reset clock and counters. Call at the top of every test.
    pub fn reset() void {
        now_ms = 0;
        crit_depth = 0;
        crit_entries = 0;
        idle_calls = 0;
    }

    // --- test introspection ------------------------------------------------

    /// Current critical-section nesting depth; must be 0 between kernel calls.
    pub fn criticalDepth() u32 {
        return crit_depth;
    }

    /// Total number of critical sections entered.
    pub fn criticalEntries() u32 {
        return crit_entries;
    }

    /// Times the kernel reported itself idle.
    pub fn idleCalls() u32 {
        return idle_calls;
    }

    /// Drive a scheduler forward by `ms` virtual milliseconds.
    ///
    /// Each iteration runs the superloop body and *then* advances the clock by
    /// 1 ms, which is what a real 1 ms systick does: the loop runs at t=0
    /// before the first tick arrives, and the clock reads `ms` when the call
    /// returns. So a 10 ms task fires exactly `ms/10` times, with its first
    /// activation at t=0 and no initial lateness.
    ///
    /// Advancing *before* running instead would make the first activation land
    /// at t=1 and show up as 1 ms of spurious lateness, which is an artefact of
    /// the harness rather than of the kernel.
    pub fn runFor(sched: anytype, ms: u32) void {
        var i: u32 = 0;
        while (i < ms) : (i += 1) {
            sched.run();
            now_ms +%= 1;
        }
    }
};

test "virtual clock advances and wraps" {
    HostHal.reset();
    try std.testing.expectEqual(@as(Tick, 0), HostHal.now());
    HostHal.advance(250);
    try std.testing.expectEqual(@as(Tick, 250), HostHal.now());
    HostHal.setNow(0xFFFF_FFF0);
    HostHal.advance(32);
    try std.testing.expectEqual(@as(Tick, 16), HostHal.now());
}

test "critical sections are balanced and counted" {
    HostHal.reset();
    HostHal.criticalEnter();
    HostHal.criticalEnter();
    try std.testing.expectEqual(@as(u32, 2), HostHal.criticalDepth());
    HostHal.criticalExit();
    HostHal.criticalExit();
    try std.testing.expectEqual(@as(u32, 0), HostHal.criticalDepth());
    try std.testing.expectEqual(@as(u32, 2), HostHal.criticalEntries());
}
