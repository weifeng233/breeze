//! Coordination primitives for multi-step and multi-branch work.
//!
//! # Why these live outside the scheduler
//!
//! None of these knows anything about tasks, ticks or the scheduler. They are
//! counters with a rule attached. That is deliberate: it means they can be used
//! from an ISR, from a `Program` step, from a task, or from host tests, without
//! the scheduler having to grow a feature.
//!
//! They were adopted from a sibling project (`zig-orch`, a cooperative
//! orchestration kernel) where the same two ideas appear as `Join` and
//! `Limiter`, themselves distilled from `step`'s `this.parallel()` and
//! `async.eachLimit`. The semantics here match; the framing is Breeze's.
//!
//! # How they combine with `Program`
//!
//! `Program` is how Breeze expresses a sequence. These primitives are what a
//! sequence waits on, which is why they compose without either side knowing
//! about the other:
//!
//! ```zig
//! var join: breeze.Join = .{};
//!
//! fn startBoth(ctx: *Ctx) void {
//!     join.enter();
//!     startTransferA();
//!     join.enter();
//!     startTransferB();
//! }
//!
//! fn bothDone(ctx: *Ctx) bool { return join.allDone(); }
//!
//! const prog = breeze.Program(Ctx, &.{
//!     .{ .call = startBoth },
//!     .{ .wait_event = EVT_DMA_A },     // each ISR calls join.leave(true)
//!     .{ .wait_event = EVT_DMA_B },
//!     .{ .call_until = bothDone },      // re-polled until it returns true
//! });
//! ```
//!
//! # Why not in the scheduler
//!
//! Breeze's scheduler is a comptime task table with no dynamic task creation.
//! A `Join` over "branches" therefore cannot mean "spawned tasks"; it means
//! "operations I started and will be told about". That is the useful reading
//! here, and it needs no kernel support at all.
//!
//! # Which one
//!
//! | question                                    | type              | cost                   |
//! |---------------------------------------------|-------------------|------------------------|
//! | are all N branches in?                      | `Join`            | 4 bytes, any N         |
//! | ...and what did each of them produce?       | `Join.Of(T, cap)` | `cap` bytes + `cap`×`T`|
//! | how much of a large batch may be in flight? | `Limiter`         | 12 bytes, any N        |
//!
//! `Join` counts branches and never learns anything about them. `Join.Of` gives
//! each branch a slot it can deposit a value into, which costs one state byte
//! per slot and caps the fan-out at 32 - the point past which an array of
//! results is what you want anyway, at which point use `Limiter` to bound the
//! concurrency and your own array to hold the output.
//!
//! # Concurrency
//!
//! All three are built for the split Breeze is organised around: one scheduling
//! context starts work, interrupt handlers report it finished. Two rules follow.
//!
//! **Register before starting.** `enter` must run before the operation that will
//! report completion. Otherwise a completion that arrives immediately is counted
//! against a branch the join has not been told about.
//!
//! **`Join` needs a critical section; `Join.Of` does not.** `Join.pending` is a
//! read-modify-write. If two completion paths can preempt each other - two IRQs
//! at different priorities, or an IRQ arriving in the middle of `enter` - one
//! decrement is lost and the join never completes. With no `LDREX`/`STREX` on
//! ARMv6-M there is no atomic counter to fall back on, so those calls must be
//! wrapped in the HAL's `criticalEnter`/`criticalExit` unless every completion
//! path shares one interrupt priority.
//!
//! `Join.Of` has no such requirement. Each slot is its own word and every phase
//! writes it with a plain whole-value store, so two completions never touch the
//! same memory and there is no read-modify-write to lose. This is the rule
//! `shared.zig` states - one writer per word - applied per slot, and it is the
//! reason to prefer `Join.Of` when the interrupt priorities are not yours to
//! choose.

const std = @import("std");
const shared = @import("shared.zig");

/// Counts outstanding branches and remembers whether any of them failed.
///
/// The pattern is: `enter` once per unit of work before starting it, `leave`
/// once per completion. `allDone` is then the join condition.
///
/// # It does not carry results
///
/// There is nowhere to put a value, so a branch that produces one must write it
/// somewhere the waiter can see - a module's `State`, a `Channel`, or a field on
/// the caller's context. When that is the shape you want, `Join.Of(T, cap)` is
/// the same pattern with a named slot per branch.
///
/// # Failure is sticky
///
/// Once any branch reports `ok = false`, `failed` stays true for the lifetime
/// of the `Join`. A caller that wants to retry must build a new one, which
/// makes "did something already go wrong?" impossible to lose track of. This
/// mirrors `Promise.all` semantics, where the first rejection decides.
///
/// # Concurrent completions
///
/// `enter` and `leave` are read-modify-writes on `pending`, so two of them that
/// can preempt each other will lose one update - see the module comment. Guard
/// them with the HAL's critical section when the completion paths are interrupt
/// handlers at different priorities. `Join.Of` exists partly to avoid needing
/// that.
pub const Join = struct {
    pending: u16 = 0,
    failed: bool = false,

    /// Register one unit of work. Call *before* starting it, or a completion
    /// that arrives immediately can be counted before it is expected.
    pub fn enter(self: *Join) void {
        self.pending += 1;
    }

    /// Report one unit of work finished.
    ///
    /// Returns true when this was the last outstanding branch, so the common
    /// case needs no second call:
    ///
    /// ```zig
    /// if (join.leave(true)) { /* everyone is in */ }
    /// ```
    ///
    /// `ok = false` marks the whole join failed. Note that the counter still
    /// decrements, so a caller waiting on `allDone` is not stranded by a
    /// failure - it is told about it via `failed` instead.
    pub fn leave(self: *Join, ok: bool) bool {
        if (!ok) self.failed = true;
        if (self.pending > 0) self.pending -= 1;
        return self.pending == 0;
    }

    /// True when every registered branch has reported in.
    ///
    /// A `Join` that was never entered is trivially done, which makes
    /// "no branches were needed" behave the same as "all branches finished".
    pub fn allDone(self: *const Join) bool {
        return self.pending == 0;
    }

    /// What one slot of a `Join.Of` is currently doing.
    ///
    /// Declared here rather than inside `Of` so that every fan-out size shares
    /// one type: `Join.Of(u16, 4).stateOf(0)` and `Join.Of(u32, 8).stateOf(3)`
    /// hand back the same enum, and a helper can take it as a parameter.
    pub const SlotState = enum(u8) {
        /// Not part of the current round.
        idle,
        /// Registered by `enter`, not yet reported.
        running,
        /// Reported by `finish`; its value is readable.
        done,
        /// Reported by `fail`; it has no value this round.
        failed,
    };

    /// A `Join` whose branches deposit a value each.
    ///
    /// `cap` slots are declared at compile time and the caller names the one it
    /// means. The naming is the design, not a convenience: it is what lets two
    /// interrupt handlers finish out of order without either of them carrying a
    /// ticket, and what lets the waiter read a result by name.
    ///
    /// ```zig
    /// const GYRO = 0;
    /// const ACCEL = 1;
    ///
    /// var sensors: breeze.Join.Of(i16, 2) = .{};
    ///
    /// fn startSampling() void {
    ///     _ = sensors.enter(GYRO);
    ///     _ = sensors.enter(ACCEL);
    ///     startGyro();
    ///     startAccel();
    /// }
    ///
    /// fn gyroReady(rate: i16) void { sensors.finish(GYRO, rate); }  // from an ISR
    /// fn accelFault() void { sensors.fail(ACCEL); }
    ///
    /// fn useThem() void {
    ///     if (!sensors.allDone() or sensors.anyFailed()) return;
    ///     const rate = sensors.result(GYRO).?;
    ///     const accel = sensors.result(ACCEL).?;
    ///     _ = .{ rate, accel };
    /// }
    /// ```
    ///
    /// # Failure is per round
    ///
    /// This is the one place the semantics differ from `Join`, and it falls out
    /// of the storage rather than being a choice: with a slot per branch there is
    /// somewhere to reset. Entering a slot clears what the previous round left
    /// in it, so a periodic gather reads this round's answer instead of a latch
    /// from three rounds ago. A caller that wants "did anything ever go wrong"
    /// keeps its own latch.
    ///
    /// # Slots are named, not looped over
    ///
    /// `enterAll` registers every slot at once, which is what a loop over
    /// `0..cap` would have done. Naming slots individually is for the case where
    /// branches start at different times.
    pub fn Of(comptime T: type, comptime cap: comptime_int) type {
        comptime {
            if (cap < 1) @compileError("Join.Of needs at least one slot");
            if (cap > 32) @compileError(
                "Join.Of supports at most 32 slots; for a bigger batch use " ++
                    "Limiter plus an array of your own",
            );
        }

        return struct {
            const Self = @This();

            /// One word per slot. No two branches share memory and no phase
            /// needs a read-modify-write, which is what makes this safe against
            /// nested interrupts where `Join` is not. See the module comment.
            state: [cap]SlotState = @splat(.idle),

            /// What `finish` deposited. Valid while the slot reads `.done`;
            /// volatile access in `result` keeps a polling waiter from caching
            /// a value it loaded before the branch reported.
            values: [cap]T = undefined,

            /// Set when a slot index was out of range, or when a completion
            /// arrived for a slot that was not running.
            ///
            /// Reported through `anyFailed` rather than by panicking: this runs
            /// in interrupt handlers, and a join that cannot be trusted should
            /// say so, not halt the board. Without it a caller who ignored the
            /// rejected call would read a result that was never written.
            misused: bool = false,

            /// Register `slot` as running, clearing whatever the previous round
            /// left in it.
            ///
            /// Returns false if the index is out of range or the slot is
            /// already running, and sets `misused` either way. Ignoring the
            /// result is survivable: the branch is then simply not part of this
            /// round, and `anyFailed` says so.
            pub fn enter(self: *Self, slot: u16) bool {
                if (slot >= cap) {
                    self.misused = true;
                    return false;
                }
                if (shared.load(SlotState, &self.state[slot]) == .running) {
                    self.misused = true;
                    return false;
                }
                shared.store(SlotState, &self.state[slot], .running);
                return true;
            }

            /// Register every slot at once: the whole fan-out in one call.
            ///
            /// Equivalent to `enter(i)` for each `i`, except that a slot still
            /// running from the previous round is replaced rather than refused.
            /// Starting a fresh round over every slot is the point of the call,
            /// so it reports the overlap through `misused` and carries on.
            pub fn enterAll(self: *Self) void {
                for (&self.state) |*s| {
                    if (shared.load(SlotState, s) == .running) self.misused = true;
                    shared.store(SlotState, s, .running);
                }
            }

            /// Report `slot` finished, with the value it produced.
            ///
            /// The value is stored before the slot reads `.done`, and both go
            /// through `shared`, so a waiter that sees `.done` cannot be looking
            /// at the previous round's value.
            ///
            /// Returns nothing on purpose. This is usually called from an ISR,
            /// and answering "was that the last one?" costs a scan of every
            /// slot; a waiter that wants to know polls `allDone`.
            pub fn finish(self: *Self, slot: u16, value: T) void {
                if (!self.canComplete(slot)) return;
                if (T != void) shared.store(T, &self.values[slot], value);
                shared.store(SlotState, &self.state[slot], .done);
            }

            /// Report `slot` finished without a usable value.
            pub fn fail(self: *Self, slot: u16) void {
                if (!self.canComplete(slot)) return;
                shared.store(SlotState, &self.state[slot], .failed);
            }

            /// True when no slot is still running.
            ///
            /// Slots that were never entered do not hold it back, so a join that
            /// registered nothing is trivially done - the same rule as `Join`.
            pub fn allDone(self: *const Self) bool {
                for (&self.state) |*s| {
                    if (shared.load(SlotState, s) == .running) return false;
                }
                return true;
            }

            /// True when a branch failed, or when the join was used wrongly.
            ///
            /// Covers `misused` so that "is every slot valid?" is one question
            /// rather than two.
            pub fn anyFailed(self: *const Self) bool {
                if (self.misused) return true;
                for (&self.state) |*s| {
                    if (shared.load(SlotState, s) == .failed) return true;
                }
                return false;
            }

            /// The lowest-numbered slot that reported a failure, or null.
            ///
            /// Null is also what comes back when `anyFailed` is true purely
            /// because of misuse: there is no branch to name in that case.
            pub fn failedSlot(self: *const Self) ?u16 {
                for (&self.state, 0..) |*s, i| {
                    if (shared.load(SlotState, s) == .failed) return @intCast(i);
                }
                return null;
            }

            /// How many branches have not reported yet.
            pub fn outstanding(self: *const Self) u16 {
                var n: u16 = 0;
                for (&self.state) |*s| {
                    if (shared.load(SlotState, s) == .running) n += 1;
                }
                return n;
            }

            /// What `slot` produced, or null if it has no value to hand back.
            ///
            /// Null covers all three of "still running", "failed" and "out of
            /// range" - exactly the cases in which there is nothing to return.
            pub fn result(self: *const Self, slot: u16) ?T {
                if (slot >= cap) return null;
                if (shared.load(SlotState, &self.state[slot]) != .done) return null;
                if (T == void) return {};
                return shared.load(T, &self.values[slot]);
            }

            /// What `slot` is doing, or null if the index is out of range.
            pub fn stateOf(self: *const Self, slot: u16) ?SlotState {
                if (slot >= cap) return null;
                return shared.load(SlotState, &self.state[slot]);
            }

            /// Return every slot to `.idle`, ready for another round.
            ///
            /// `join = .{}` does the same thing. This exists because that
            /// assignment does not say out loud that reusing a join is meant to
            /// be possible.
            pub fn reset(self: *Self) void {
                for (&self.state) |*s| shared.store(SlotState, s, .idle);
                self.misused = false;
            }

            /// Shared validity check for the two completion paths. Separated so
            /// that neither of them can store anything before the check passes -
            /// in particular so `finish` cannot mark a slot done before the
            /// value is in it.
            fn canComplete(self: *Self, slot: u16) bool {
                if (slot >= cap) {
                    self.misused = true;
                    return false;
                }
                if (shared.load(SlotState, &self.state[slot]) != .running) {
                    self.misused = true;
                    return false;
                }
                return true;
            }
        };
    }
};

/// Bounds how many units of a large batch are in flight at once.
///
/// A cursor hands out indices; the caller starts work for each and reports
/// completion. The window is what stops a 200-item batch from trying to be 200
/// things at once on a part with 3 KB of RAM.
///
/// ```zig
/// var lim = breeze.Limiter.init(2, pages.len);   // at most 2 at a time
///
/// while (lim.acquire()) |i| { startErase(pages[i]); }
/// // ... and in the completion path:
/// if (lim.release()) { /* the whole batch is finished */ }
/// ```
pub const Limiter = struct {
    cap: u16,
    in_flight: u16 = 0,
    cursor: u32 = 0,
    total: u32 = 0,

    pub fn init(cap: u16, total: u32) Limiter {
        // A zero window can never admit anything, so `acquire` would always
        // return null and the batch would silently never run. Rejecting here
        // turns a silent no-op into a failed assertion.
        std.debug.assert(cap > 0);
        return .{ .cap = cap, .total = total };
    }

    /// Take the next index, or null if the window is full or the batch is done.
    ///
    /// The two null cases are deliberately not distinguished: a caller loops
    /// on `acquire` until it returns null, and the difference only matters to
    /// `release`, which is what decides completion.
    pub fn acquire(self: *Limiter) ?u32 {
        if (self.in_flight >= self.cap) return null;
        if (self.cursor >= self.total) return null;
        const idx = self.cursor;
        self.cursor += 1;
        self.in_flight += 1;
        return idx;
    }

    /// Report one unit finished. Returns true when the whole batch is done.
    ///
    /// "Done" means every index was handed out *and* every one came back. A
    /// caller that only checked `cursor >= total` would declare victory while
    /// the last few operations were still running.
    pub fn release(self: *Limiter) bool {
        if (self.in_flight > 0) self.in_flight -= 1;
        return self.cursor >= self.total and self.in_flight == 0;
    }

    /// Indices not yet handed out.
    pub fn remaining(self: *const Limiter) u32 {
        return if (self.cursor >= self.total) 0 else self.total - self.cursor;
    }
};

// --- tests -----------------------------------------------------------------

test "a Join over no branches is already done" {
    const join = Join{};
    try std.testing.expect(join.allDone());
    try std.testing.expect(!join.failed);
}

test "Join completes on the last branch only" {
    var join = Join{};

    join.enter();
    join.enter();
    join.enter();
    try std.testing.expect(!join.allDone());

    try std.testing.expect(!join.leave(true));
    try std.testing.expect(!join.leave(true));
    try std.testing.expect(join.leave(true));
    try std.testing.expect(join.allDone());
}

test "Join failure is sticky and does not strand the waiter" {
    var join = Join{};

    join.enter();
    join.enter();

    // One branch fails; the count still comes down.
    try std.testing.expect(!join.leave(false));
    try std.testing.expect(join.failed);
    try std.testing.expectEqual(@as(u16, 1), join.pending);

    // A later success must not clear the failure.
    try std.testing.expect(join.leave(true));
    try std.testing.expect(join.allDone());
    try std.testing.expect(join.failed);
}

test "Join tolerates a completion it never registered" {
    // The counter is guarded rather than allowed to wrap: a stray `leave`
    // would otherwise push `pending` to 65535 and the join would never finish.
    var join = Join{};
    try std.testing.expect(join.leave(true));
    try std.testing.expectEqual(@as(u16, 0), join.pending);
    try std.testing.expect(join.allDone());
}

test "Join: enter after the last leave is not lost" {
    // Guards against using `allDone` as the latch for a branch that has not
    // been registered yet - a restartable batch must re-enter before it can be
    // considered complete again.
    var join = Join{};
    join.enter();
    try std.testing.expect(join.leave(true));
    try std.testing.expect(join.allDone());

    join.enter();
    try std.testing.expect(!join.allDone());
}

// --- Join.Of ---------------------------------------------------------------

test "Join.Of: a result lands in the slot it was sent to" {
    var join: Join.Of(u16, 3) = .{};
    join.enterAll();
    try std.testing.expect(!join.allDone());
    try std.testing.expectEqual(@as(u16, 3), join.outstanding());

    // Out of order, and one of them is zero - which must not read as "no
    // value", the way a bare sentinel would.
    join.finish(2, 0x0202);
    join.finish(0, 0x0000);
    try std.testing.expect(!join.allDone());
    try std.testing.expectEqual(@as(u16, 1), join.outstanding());

    join.finish(1, 0x0101);
    try std.testing.expect(join.allDone());
    try std.testing.expectEqual(@as(u16, 0), join.outstanding());
    try std.testing.expect(!join.anyFailed());

    try std.testing.expectEqual(@as(?u16, 0x0000), join.result(0));
    try std.testing.expectEqual(@as(?u16, 0x0101), join.result(1));
    try std.testing.expectEqual(@as(?u16, 0x0202), join.result(2));
}

test "Join.Of: slots that were never entered do not hold the join open" {
    var join: Join.Of(u8, 4) = .{};
    _ = join.enter(1);
    _ = join.enter(3);
    try std.testing.expectEqual(@as(u16, 2), join.outstanding());

    join.finish(3, 9);
    try std.testing.expect(!join.allDone()); // slot 1 is still out
    join.finish(1, 7);
    try std.testing.expect(join.allDone());

    try std.testing.expectEqual(Join.SlotState.idle, join.stateOf(0).?);
    try std.testing.expectEqual(@as(?u8, null), join.result(0));
    try std.testing.expectEqual(@as(?u8, null), join.result(2));
    try std.testing.expectEqual(@as(?u8, 7), join.result(1));
}

test "Join.Of: a failed branch has no value without hiding the others" {
    var join: Join.Of(i16, 3) = .{};
    join.enterAll();

    join.fail(1);
    join.finish(0, -42);
    join.finish(2, 42);

    try std.testing.expect(join.allDone());
    try std.testing.expect(join.anyFailed());
    try std.testing.expectEqual(@as(?u16, 1), join.failedSlot());
    try std.testing.expectEqual(@as(?i16, null), join.result(1));
    try std.testing.expectEqual(@as(?i16, -42), join.result(0));
    try std.testing.expectEqual(@as(?i16, 42), join.result(2));
}

test "Join.Of: failure is per round where Join's is for the lifetime" {
    // The documented divergence, both halves of it.
    var counted = Join{};
    counted.enter();
    _ = counted.leave(false);
    counted.enter();
    _ = counted.leave(true);
    try std.testing.expect(counted.failed); // sticky, by design

    var slotted: Join.Of(u8, 1) = .{};
    _ = slotted.enter(0);
    slotted.fail(0);
    try std.testing.expect(slotted.anyFailed());

    _ = slotted.enter(0);
    slotted.finish(0, 5);
    try std.testing.expect(!slotted.anyFailed()); // this round is clean
    try std.testing.expectEqual(@as(?u16, null), slotted.failedSlot());
    try std.testing.expectEqual(@as(?u8, 5), slotted.result(0));
}

test "Join.Of: reopening a slot discards the previous round's value" {
    var join: Join.Of(u8, 1) = .{};
    _ = join.enter(0);
    join.finish(0, 11);
    try std.testing.expectEqual(@as(?u8, 11), join.result(0));

    _ = join.enter(0);
    // The old value must not be readable while the new branch is still out.
    try std.testing.expectEqual(Join.SlotState.running, join.stateOf(0).?);
    try std.testing.expectEqual(@as(?u8, null), join.result(0));
}

test "Join.Of: an out-of-range slot is reported, not obeyed" {
    var join: Join.Of(u8, 2) = .{};
    try std.testing.expect(!join.enter(2));
    try std.testing.expect(join.anyFailed());
    try std.testing.expectEqual(@as(?u16, null), join.failedSlot()); // no branch to blame
    try std.testing.expect(join.allDone()); // and nothing was registered
    try std.testing.expectEqual(@as(?u8, null), join.result(2));
    try std.testing.expectEqual(@as(?Join.SlotState, null), join.stateOf(2));

    join.finish(2, 1);
    join.fail(9);
    try std.testing.expectEqual(@as(u16, 0), join.outstanding());
}

test "Join.Of: completing a slot that was never entered is reported" {
    var join: Join.Of(u8, 2) = .{};
    join.finish(1, 3);

    try std.testing.expect(join.anyFailed());
    try std.testing.expectEqual(@as(?u8, null), join.result(1));
    try std.testing.expectEqual(Join.SlotState.idle, join.stateOf(1).?);
    try std.testing.expect(join.allDone());
}

test "Join.Of: entering a running slot is reported and changes nothing" {
    var join: Join.Of(u8, 2) = .{};
    try std.testing.expect(join.enter(0));
    try std.testing.expect(!join.misused);

    try std.testing.expect(!join.enter(0)); // already out
    try std.testing.expect(join.misused);
    try std.testing.expectEqual(@as(u16, 1), join.outstanding());

    join.finish(0, 1);
    try std.testing.expect(join.allDone());
}

test "Join.Of: enterAll replaces a slot left running by the previous round" {
    var join: Join.Of(u8, 2) = .{};
    _ = join.enter(0);
    join.enterAll(); // slot 0 was still out
    try std.testing.expect(join.misused);
    try std.testing.expectEqual(@as(u16, 2), join.outstanding());

    join.finish(0, 1);
    join.finish(1, 2);
    try std.testing.expect(join.allDone());

    join.reset();
    try std.testing.expect(!join.misused);
    try std.testing.expectEqual(@as(?u8, null), join.result(0));
    try std.testing.expect(join.allDone());
}

test "Join.Of: a join with no payload still names its branches" {
    // `T = void` is the counting join that can say *which* branch failed.
    var join: Join.Of(void, 3) = .{};
    join.enterAll();
    join.finish(0, {});
    join.fail(1);
    join.finish(2, {});

    try std.testing.expect(join.allDone());
    try std.testing.expect(join.anyFailed());
    try std.testing.expectEqual(@as(?u16, 1), join.failedSlot());
    try std.testing.expect(join.result(0) != null);
    try std.testing.expect(join.result(1) == null);
}

test "Join.Of: a waiter can poll while the branches report" {
    // The shape a `Program` uses: a step that answers "not yet" until every
    // branch is in, with completions landing between polls as an ISR's would.
    var join: Join.Of(u8, 2) = .{};
    join.enterAll();

    var polls: u32 = 0;
    while (!join.allDone()) {
        polls += 1;
        switch (polls) {
            1 => join.finish(1, 0xB1),
            2 => join.finish(0, 0xA0),
            else => {},
        }
        try std.testing.expect(polls < 8);
    }

    try std.testing.expectEqual(@as(u32, 2), polls);
    try std.testing.expectEqual(@as(?u8, 0xA0), join.result(0));
    try std.testing.expectEqual(@as(?u8, 0xB1), join.result(1));
}

test "Join.Of: a fan-out that reports values" {
    var adc: Join.Of(u16, 4) = .{};
    adc.enterAll();
    adc.finish(3, 4003);
    adc.finish(0, 4000);
    adc.finish(2, 4002);
    adc.finish(1, 4001);

    try std.testing.expect(adc.allDone());
    try std.testing.expect(!adc.anyFailed());

    var sum: u32 = 0;
    inline for (0..4) |i| sum += (adc.result(i) orelse 0);
    try std.testing.expectEqual(@as(u32, 16006), sum);
}

test "Join.Of: a struct payload survives the round trip" {
    const Sample = struct { raw: u16, at: u32 };

    var join: Join.Of(Sample, 2) = .{};
    join.enterAll();
    join.finish(1, .{ .raw = 0x1234, .at = 99 });
    join.finish(0, .{ .raw = 0xABCD, .at = 1 });

    try std.testing.expectEqual(@as(u16, 0x1234), join.result(1).?.raw);
    try std.testing.expectEqual(@as(u32, 99), join.result(1).?.at);
    try std.testing.expectEqual(@as(u16, 0xABCD), join.result(0).?.raw);
}

test "Join.Of: the marginal cost is one state byte and one value per slot" {
    try std.testing.expectEqual(@as(usize, 1), @sizeOf(Join.SlotState));
    try std.testing.expectEqual(
        4 * (@sizeOf(Join.SlotState) + @sizeOf(u16)),
        @sizeOf(Join.Of(u16, 8)) - @sizeOf(Join.Of(u16, 4)),
    );
}

test "the counter hazard that Join.Of is shaped to avoid" {
    // This is not a bug report - it is the executable form of what the module
    // comment says, and the reason `Join.Of` spends a byte per branch instead
    // of a counter. Two handlers, each loading the counter and storing its own
    // decremented copy, lose one of the two decrements:
    var join = Join{};
    join.enter();
    join.enter();

    const from_isr_a = join.pending; // A loads 2
    const from_isr_b = join.pending; // B preempts and also loads 2
    join.pending = from_isr_a - 1; // A stores 1
    join.pending = from_isr_b - 1; // B stores 1; A's decrement is gone

    try std.testing.expectEqual(@as(u16, 1), join.pending);
    try std.testing.expect(!join.allDone()); // and it stays that way

    // The slot version has no shared word for the two to race over: each
    // completion writes only its own slot, so both orders agree exactly.
    var forwards: Join.Of(u8, 2) = .{};
    forwards.enterAll();
    forwards.finish(0, 1);
    forwards.finish(1, 2);

    var backwards: Join.Of(u8, 2) = .{};
    backwards.enterAll();
    backwards.finish(1, 2);
    backwards.finish(0, 1);

    try std.testing.expect(forwards.allDone() and backwards.allDone());
    try std.testing.expectEqual(forwards.result(0), backwards.result(0));
    try std.testing.expectEqual(forwards.result(1), backwards.result(1));
}

test "Limiter never exceeds its window" {
    var lim = Limiter.init(2, 10);

    var issued: u32 = 0;
    var completed: u32 = 0;

    while (completed < 10) {
        // Fill the window, checking the bound on every admission.
        while (lim.acquire()) |_| {
            issued += 1;
            try std.testing.expect(lim.in_flight <= 2);
        }
        // Drain it, so the cursor can advance.
        while (lim.in_flight > 0) {
            completed += 1;
            _ = lim.release();
        }
    }

    try std.testing.expectEqual(@as(u32, 10), issued);
    try std.testing.expectEqual(@as(u32, 10), completed);
    try std.testing.expectEqual(@as(u32, 0), lim.remaining());
}

test "Limiter reports completion only when the last unit returns" {
    var lim = Limiter.init(3, 4);

    // The window admits three; the fourth index cannot be handed out until one
    // of them comes back.
    var got: u32 = 0;
    while (lim.acquire()) |_| got += 1;
    try std.testing.expectEqual(@as(u32, 3), got);
    try std.testing.expectEqual(@as(u16, 3), lim.in_flight);
    try std.testing.expectEqual(@as(u32, 1), lim.remaining());

    // One return makes room for the last index, and is not completion.
    try std.testing.expect(!lim.release());
    try std.testing.expectEqual(@as(?u32, 3), lim.acquire());
    try std.testing.expectEqual(@as(u32, 0), lim.remaining());

    // The cursor is now past the end, but two are still in flight. This is the
    // case a cursor-only completion check gets wrong: it would declare the
    // batch finished here, while work is still outstanding.
    try std.testing.expect(!lim.release());
    try std.testing.expect(!lim.release());
    try std.testing.expect(lim.release());
}

test "Limiter with a window of one serialises the batch" {
    var lim = Limiter.init(1, 3);

    try std.testing.expectEqual(@as(?u32, 0), lim.acquire());
    // Full: the second acquire must be refused while the first is outstanding.
    try std.testing.expectEqual(@as(?u32, null), lim.acquire());

    try std.testing.expect(!lim.release());
    try std.testing.expectEqual(@as(?u32, 1), lim.acquire());
    try std.testing.expect(!lim.release());
    try std.testing.expectEqual(@as(?u32, 2), lim.acquire());
    try std.testing.expect(lim.release());

    // Every index was used exactly once.
    try std.testing.expectEqual(@as(?u32, null), lim.acquire());
}

test "Limiter over an empty batch is complete immediately" {
    var lim = Limiter.init(4, 0);
    try std.testing.expectEqual(@as(?u32, null), lim.acquire());
    try std.testing.expectEqual(@as(u32, 0), lim.remaining());
}

test "Join and Limiter together describe a bounded fan-out" {
    // The combination the two are for: run a large batch, at most N at a time,
    // and know when the whole thing is done.
    var join = Join{};
    var lim = Limiter.init(2, 7);

    var finished: u32 = 0;
    var issued: u32 = 0;

    while (true) {
        // Fill the window, registering each unit before it starts.
        while (lim.acquire()) |_| {
            join.enter();
            issued += 1;
        }
        if (issued == 0) break;
        if (lim.in_flight == 0) break;

        // Finish everything currently in flight.
        const batch = lim.in_flight;
        var i: u16 = 0;
        while (i < batch) : (i += 1) {
            finished += 1;
            _ = join.leave(true);
            _ = lim.release();
        }
        if (finished == 7) break;
    }

    try std.testing.expectEqual(@as(u32, 7), issued);
    try std.testing.expectEqual(@as(u32, 7), finished);
    try std.testing.expect(join.allDone());
    try std.testing.expect(!join.failed);
}
