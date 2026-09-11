//! Breeze Framework - event flags: the ISR-to-task channel.
//!
//! A 32-bit mask, one word, no allocation. The concurrency contract is
//! asymmetric on purpose:
//!
//!   * `setFromIsr` is called only from interrupt context and is the sole
//!     writer on that path, so a bare `|=` is safe.
//!   * `raise` and `clear` run in task context and therefore mask interrupts,
//!     because an ISR firing mid read-modify-write would otherwise be lost.
//!
//! `test`/`isSet` are plain reads of an aligned word and need no masking.

const std = @import("std");
const hal = @import("hal.zig");
const Tick = @import("tick.zig").Tick;

pub const EventFlags = struct {
    bits: u32 = 0,

    /// Raise flags from interrupt context.
    ///
    /// Safe without a critical section because the ISR is the only writer on
    /// this path; task-context clears are protected by `clear`.
    pub inline fn setFromIsr(self: *EventFlags, mask: u32) void {
        self.bits |= mask;
    }

    /// Raise flags from task context.
    ///
    /// `Hal` is a comptime parameter placed after `self` so that ordinary
    /// method syntax `flags.raise(Hal, mask)` resolves.
    pub inline fn raise(self: *EventFlags, comptime Hal: type, mask: u32) void {
        hal.criticalEnter(Hal);
        self.bits |= mask;
        hal.criticalExit(Hal);
    }

    /// Lower flags. Must be called from task context.
    pub inline fn clear(self: *EventFlags, comptime Hal: type, mask: u32) void {
        hal.criticalEnter(Hal);
        self.bits &= ~mask;
        hal.criticalExit(Hal);
    }

    /// Non-destructive test: true if *any* bit in `mask` is set.
    ///
    /// Named `isSet` rather than `test` because `test` is a Zig keyword.
    pub inline fn isSet(self: *const EventFlags, mask: u32) bool {
        return (self.bits & mask) != 0;
    }

    /// Non-destructive test: true if *all* bits in `mask` are set.
    pub inline fn isAllSet(self: *const EventFlags, mask: u32) bool {
        return (self.bits & mask) == mask;
    }

    /// Snapshot of the current mask.
    ///
    /// The scheduler passes this snapshot to tasks so that every task in one
    /// pass observes a consistent view, and so a task cannot observe a flag it
    /// is itself about to clear in a different order than its peers.
    pub inline fn snapshot(self: *const EventFlags) u32 {
        return self.bits;
    }

    pub inline fn reset(self: *EventFlags) void {
        self.bits = 0;
    }
};

test "set, test, clear" {
    const HostHal = @import("../hal/host.zig").HostHal;
    var flags = EventFlags{};

    flags.setFromIsr(0b1010);
    try std.testing.expect(flags.isSet(0b0010));
    try std.testing.expect(flags.isSet(0b1000));
    try std.testing.expect(!flags.isSet(0b0100));
    try std.testing.expect(flags.isAllSet(0b1010));
    try std.testing.expect(!flags.isAllSet(0b1110));

    flags.clear(HostHal, 0b0010);
    try std.testing.expect(!flags.isSet(0b0010));
    try std.testing.expect(flags.isSet(0b1000));

    flags.raise(HostHal, 0b0100);
    try std.testing.expect(flags.isSet(0b0100));

    try std.testing.expectEqual(@as(u32, 0b1100), flags.snapshot());
    flags.reset();
    try std.testing.expectEqual(@as(u32, 0), flags.snapshot());
}

test "snapshot is independent of later writes" {
    var flags = EventFlags{};
    flags.setFromIsr(0x1);
    const snap = flags.snapshot();
    flags.setFromIsr(0x2);
    try std.testing.expectEqual(@as(u32, 0x1), snap);
    try std.testing.expectEqual(@as(u32, 0x3), flags.bits);
}
