//! Breeze Framework - event flags: the ISR-to-task channel.
//!
//! A 32-bit mask, one word, no allocation. The concurrency contract is
//! asymmetric on purpose:
//!
//!   * `setFromIsr` is called only from interrupt context, where masking is not
//!     available portably, so it is a bare read-modify-write. That is safe
//!     under one condition, stated below.
//!   * `raise` and `clear` run in task context and therefore mask interrupts,
//!     because an ISR firing mid read-modify-write would otherwise be lost.
//!
//! # The condition on `setFromIsr`: one interrupt priority
//!
//! **Every interrupt that raises flags must run at the same priority.** A bare
//! read-modify-write cannot be atomic on ARMv6-M (no `LDREX`/`STREX`), so if a
//! higher-priority ISR preempts a lower-priority one between the load and the
//! store, the lower one writes back the value it loaded and the higher one's
//! flag is gone - permanently, for a source that raises it once. Every ISR
//! writes the *same* word, so this is the normal case, not an exotic one.
//!
//! The alternative was masking interrupts inside the handler, which works on
//! Cortex-M (`cpsid i` nests safely inside an ISR) but not on RISC-V, where
//! `criticalExit` sets MIE unconditionally and would therefore enable
//! interrupts inside a trap handler. A shared priority is the one rule that
//! holds on both backends, and it is what most firmware does by default.
//! `hal.zig` states it as part of the interrupt-handler contract.
//!
//! # Triggers are level-based, and nothing clears itself
//!
//! A flag stays set until someone calls `clear`. It is not consumed by being
//! observed, and `snapshot` does not reset anything. Two consequences follow,
//! and both are deliberate:
//!
//! * A task waiting on a flag that nobody clears will keep firing. Since
//!   `Program` restarts itself when it runs off the end, a `wait_event` on a
//!   stale flag re-runs the whole sequence on every pass.
//! * An event raised twice before it is cleared is indistinguishable from one
//!   raised once. There is no pending count.
//!
//! That is the right trade for the things this kernel is for - "the link came
//! up", "the frame is ready", "the button is down" are states, not counted
//! occurrences - and it keeps the whole channel to one word with no allocation.
//! A counted or edge-triggered channel is `Channel`, which has storage and
//! counts drops.
//!
//! The rule for callers is therefore: **clear what you have handled**, at the
//! end of the pass that handled it. `scheduler.zig`'s tests pin both the
//! level-triggered repeat and the fact that clearing one flag does not disturb
//! another raised in the same pass.
//!
//! # Why not auto-clear, or edge semantics
//!
//! This was considered and rejected, so the reasoning is recorded rather than
//! left to be re-litigated:
//!
//! * Auto-clearing would make two tasks that care about the same flag race -
//!   whichever runs first consumes it and the other never sees it. Under the
//!   level rule both observe it, which is the property a cooperative scheduler
//!   with declaration-order execution actually wants.
//! * Edge semantics needs a count or a sequence number to avoid losing events
//!   that arrive between passes. That stops being one word with no allocation,
//!   and `Channel` already covers it - with storage, and with drop counting.
//!   The split between the two is deliberate.
//!
//! The cost is that callers must remember to clear. That is made explicit here
//! and guaranteed by tests rather than left as folklore.
//!
//! # Every access goes through `shared` (volatile)
//!
//! `bits` is written by an interrupt handler and read by the scheduler, so it
//! is shared mutable state and `kernel/shared.zig` governs how such state is
//! touched. This file previously read and wrote `self.bits` directly, which
//! happened to work but broke the rule the rest of the kernel follows.
//!
//! The failure mode is not hypothetical. Without a volatile access the compiler
//! is entitled to keep `bits` in a register across a loop, because from its
//! point of view nothing in the loop writes it:
//!
//! ```zig
//! while (!flags.isSet(EVT_LINK_UP)) {}   // may never re-read `bits`
//! ```
//!
//! Such a loop can be compiled into an infinite one, and the ISR that would
//! have ended it never gets observed. Reads are therefore volatile too, not
//! just writes. `shared.zig` has the same argument in full.
//!
//! What volatile does *not* provide is atomicity of the read-modify-write in
//! `raise`/`clear`; that is what their critical section is for.

const std = @import("std");
const hal = @import("hal.zig");
const shared = @import("shared.zig");
const Tick = @import("tick.zig").Tick;

pub const EventFlags = struct {
    bits: u32 = 0,

    /// Raise flags from interrupt context.
    ///
    /// **Every interrupt that calls this must run at the same priority** - see
    /// the module comment. The read-modify-write below cannot be made atomic on
    /// ARMv6-M, and an ISR that preempts another one between the load and the
    /// store loses the preempting interrupt's flag for good.
    ///
    /// Task-context raises are not affected: `raise` masks interrupts around its
    /// own read-modify-write.
    ///
    /// The read-modify-write is still split across two volatile accesses so that
    /// neither half can be optimised away.
    pub inline fn setFromIsr(self: *EventFlags, mask: u32) void {
        shared.store(u32, &self.bits, shared.load(u32, &self.bits) | mask);
    }

    /// Raise flags from task context.
    ///
    /// `Hal` is a comptime parameter placed after `self` so that ordinary
    /// method syntax `flags.raise(Hal, mask)` resolves.
    pub inline fn raise(self: *EventFlags, comptime Hal: type, mask: u32) void {
        hal.criticalEnter(Hal);
        shared.store(u32, &self.bits, shared.load(u32, &self.bits) | mask);
        hal.criticalExit(Hal);
    }

    /// Lower flags. Must be called from task context.
    pub inline fn clear(self: *EventFlags, comptime Hal: type, mask: u32) void {
        hal.criticalEnter(Hal);
        shared.store(u32, &self.bits, shared.load(u32, &self.bits) & ~mask);
        hal.criticalExit(Hal);
    }

    /// Non-destructive test: true if *any* bit in `mask` is set.
    ///
    /// Named `isSet` rather than `test` because `test` is a Zig keyword.
    pub inline fn isSet(self: *const EventFlags, mask: u32) bool {
        return (shared.load(u32, &self.bits) & mask) != 0;
    }

    /// Non-destructive test: true if *all* bits in `mask` are set.
    pub inline fn isAllSet(self: *const EventFlags, mask: u32) bool {
        return (shared.load(u32, &self.bits) & mask) == mask;
    }

    /// Snapshot of the current mask.
    ///
    /// The scheduler passes this snapshot to tasks so that every task in one
    /// pass observes a consistent view, and so a task cannot observe a flag it
    /// is itself about to clear in a different order than its peers.
    pub inline fn snapshot(self: *const EventFlags) u32 {
        return shared.load(u32, &self.bits);
    }

    pub inline fn reset(self: *EventFlags) void {
        shared.store(u32, &self.bits, 0);
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

test "a polling loop sees a flag raised from interrupt context" {
    // This is the pattern the volatile discipline exists for. The loop below
    // must re-read `bits` on every iteration; if `isSet` were a plain load, the
    // compiler would be free to hoist the read out of the loop (nothing
    // visible to it writes `bits`) and the loop would never terminate.
    //
    // The write is placed inside the loop body so that the two are genuinely
    // interleaved, exactly as an ISR firing mid-loop would be.
    const EVT_LINK_UP: u32 = 1 << 3;

    var flags = EventFlags{};
    var iterations: u32 = 0;
    var raised_at: u32 = 0;

    while (!flags.isSet(EVT_LINK_UP)) {
        iterations += 1;
        if (iterations == 5) {
            flags.setFromIsr(EVT_LINK_UP);
            raised_at = iterations;
        }
        // A real bound, so a compiler that *did* hoist the load would hang the
        // test suite rather than quietly passing. The test asserts on the
        // iteration count below.
        if (iterations > 1000) break;
    }

    try std.testing.expectEqual(@as(u32, 5), raised_at);
    try std.testing.expectEqual(@as(u32, 5), iterations);
    try std.testing.expect(flags.isSet(EVT_LINK_UP));
}

test "a flag set between snapshot and clear is not silently dropped" {
    // The scheduler snapshots once per pass and tasks clear bits. A bit raised
    // by an ISR after the snapshot must survive the clear of an unrelated bit,
    // otherwise an event that arrived during the pass is lost until the next
    // one - and if it is level-triggered by nature, lost for good.
    const EVT_A: u32 = 1 << 0;
    const EVT_B: u32 = 1 << 1;
    const HostHal = @import("../hal/host.zig").HostHal;

    var flags = EventFlags{};
    flags.setFromIsr(EVT_A);

    const snap = flags.snapshot();
    try std.testing.expect(flags.isSet(EVT_A));

    // ISR raises B while the task is working on the A it saw.
    flags.setFromIsr(EVT_B);
    flags.clear(HostHal, EVT_A);

    // A is consumed, B survived.
    try std.testing.expect(!flags.isSet(EVT_A));
    try std.testing.expect(flags.isSet(EVT_B));

    // And the snapshot the task is still holding is unchanged, which is what
    // keeps every task in one pass looking at the same world.
    try std.testing.expectEqual(EVT_A, snap);
}

test "setFromIsr preserves bits it was not asked to set" {
    // Guards the read-modify-write: a store that forgot to read first would
    // clobber concurrently-set flags.
    var flags = EventFlags{};
    flags.setFromIsr(0b0001);
    flags.setFromIsr(0b0100);
    flags.setFromIsr(0b0010);
    try std.testing.expectEqual(@as(u32, 0b0111), flags.snapshot());
}
