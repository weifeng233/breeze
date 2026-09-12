//! Coordination primitives for multi-step and multi-branch work.
//!
//! # Why these live outside the scheduler
//!
//! Neither `Join` nor `Limiter` knows anything about tasks, ticks or the
//! scheduler. They are counters with a rule attached. That is deliberate: it
//! means they can be used from an ISR, from a `Program` step, from a task, or
//! from host tests, without the scheduler having to grow a feature.
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

const std = @import("std");

/// Counts outstanding branches and remembers whether any of them failed.
///
/// The pattern is: `enter` once per unit of work before starting it, `leave`
/// once per completion. `allDone` is then the join condition.
///
/// # It does not carry results
///
/// There is no place to put a value, so a branch that produces one must write
/// it somewhere the waiter can see - a module's `State`, a `Channel`, or a
/// field on the caller's context. That is a real limitation and it is the same
/// one `zig-orch` records as its highest-priority gap. It is left as-is here
/// rather than guessed at, because the right shape depends on whether the
/// results are homogeneous, fixed in number, and known at comptime - and a
/// wrong guess would be worse than the omission.
///
/// # Failure is sticky
///
/// Once any branch reports `ok = false`, `failed` stays true for the lifetime
/// of the `Join`. A caller that wants to retry must build a new one, which
/// makes "did something already go wrong?" impossible to lose track of. This
/// mirrors `Promise.all` semantics, where the first rejection decides.
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
