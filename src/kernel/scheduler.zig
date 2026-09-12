//! Breeze Framework - deterministic cooperative scheduler.
//!
//! # Model
//!
//! * One scheduling context: the `main` superloop, or a single RTOS task if the
//!   project already has one. There is no preemption and no context switch.
//! * The task table is a comptime tuple. `inline for` unrolls it into direct
//!   calls, so there is no function-pointer indirection and no task table in
//!   RAM - only per-task scheduling state.
//! * Time comes from `Hal.now()`. The ISR's only job is to advance the timebase;
//!   the scheduler never runs in interrupt context.
//!
//! # Determinism
//!
//! Tasks run in declaration order. A task that is not due is skipped without
//! touching it. A periodic task is scheduled against a fixed grid
//! (`next_run += period`), so the average rate is exact and jitter does not
//! accumulate; if the grid is missed by more than one period the task resyncs
//! and the skip is counted rather than silently bursting to catch up.
//!
//! # Starvation
//!
//! A task that returns `.pending` is re-polled on every pass, which is what
//! makes a multi-step sequence complete promptly. The flip side is that a task
//! which never suspends will monopolise the loop. `max_late` on each task is
//! the instrument for detecting that, and `Program`'s per-poll budget bounds
//! the damage a pathological jump-only program can do.

const std = @import("std");
const tick = @import("tick.zig");
const Tick = tick.Tick;
const hal = @import("hal.zig");
const EventFlags = @import("events.zig").EventFlags;
const Step = @import("program.zig").Step;

pub const TaskState = struct {
    /// Tick at which this task is next due. Meaningless for `period_ms == 0`.
    next_run: Tick = 0,
    /// Number of times the task body has been entered.
    runs: u32 = 0,
    /// Whether the task suspended mid-sequence and must be re-polled promptly.
    pending: bool = false,
    /// Lateness of the most recent periodic activation, in ms.
    last_late: u32 = 0,
    /// Worst lateness observed. Non-zero means the loop is overloaded.
    max_late: u32 = 0,
    /// Times the fixed grid was missed by more than a full period.
    resyncs: u32 = 0,
    /// Activations later than the task's `.max_late_ms` threshold.
    ///
    /// Stays zero for a task that does not set one, so the field costs nothing
    /// to carry for projects that never look at it.
    overruns: u32 = 0,
};

/// What an overrun hook is told.
pub const Overrun = struct {
    /// Index into the task table, i.e. the declaration order.
    index: u32,
    /// Comptime name of the task, from its definition.
    name: []const u8,
    /// How late this activation was, in milliseconds.
    late_ms: u32,
};

/// Called when a task's activation exceeds its `.max_late_ms` threshold.
///
/// # What this is for
///
/// `worstLateness()` is a *pull* metric: something has to ask. This is the
/// *push* side, for a project that wants to log, count or trip a fault the
/// moment a deadline is missed rather than at the next poll.
///
/// # Restrictions
///
/// The hook runs inside `run()`, between the moment lateness is measured and
/// the moment the task body is entered. It must therefore not call back into
/// the same scheduler - `raise`, `clearEvents` and a nested `run` are all
/// off-limits. Setting a flag, incrementing a counter, or writing to a
/// `Channel` is fine. Treat it like an ISR with a slightly larger budget.
///
/// Keep it short: it executes while the task it is complaining about is already
/// late.
pub const OverrunHook = *const fn (info: Overrun, user: ?*anyopaque) void;

/// Build a scheduler over a compile-time task table.
///
/// Each definition is an anonymous struct with:
///
/// ```zig
/// .{
///     .name = "control",              // comptime string, for diagnostics
///     .period_ms = 20,                // 0 = poll on every pass
///     .ctx = &control_ctx,            // pointer to the task's context struct
///     .poll = Control.poll,           // fn (*Ctx, Tick, u32) Step
///     .max_late_ms = 5,               // optional: lateness that counts as an overrun
/// }
/// ```
///
/// `poll` receives the current tick and a snapshot of the event mask. Keeping
/// the context in a caller-owned struct is not a stylistic choice: with no
/// stack to save, anything that must survive a suspension has to live outside
/// the task body. Making that explicit is the point.
pub fn Scheduler(comptime Hal: type, comptime defs: anytype) type {
    hal.validate(Hal);
    comptime validateDefs(defs);

    return struct {
        const Self = @This();

        pub const task_count: usize = defs.len;

        /// Shared event flags. ISRs raise bits on this via `setFromIsr`.
        events: EventFlags = .{},

        /// Per-task scheduling state.
        states: [task_count]TaskState = [_]TaskState{.{}} ** task_count,

        /// Number of `run()` passes.
        passes: u32 = 0,

        /// Passes on which no task was runnable.
        idle_passes: u32 = 0,

        /// Optional notification when a task's activation exceeds its
        /// `.max_late_ms` threshold. See `OverrunHook`.
        ///
        /// Null costs the counter below nothing and the call site one
        /// predictable-not-taken branch, so a project that does not want this
        /// pays only the eight bytes of the two pointers. Install it with
        /// `setOverrunHook`.
        on_overrun: ?OverrunHook = null,

        /// Passed through to `on_overrun` untouched; the scheduler never looks
        /// at it.
        overrun_user: ?*anyopaque = null,

        pub fn init() Self {
            return .{};
        }

        /// Install (or clear) the overrun notification.
        ///
        /// Separate from `init` so that `init` stays a pure value with no
        /// arguments, and so that a project can turn the hook on later without
        /// the scheduler type changing.
        pub fn setOverrunHook(self: *Self, hook: ?OverrunHook, user: ?*anyopaque) void {
            self.on_overrun = hook;
            self.overrun_user = user;
        }

        /// Comptime name of task `i`.
        pub fn taskName(comptime i: usize) []const u8 {
            return defs[i].name;
        }

        /// Read-only view of one task's scheduling state.
        pub fn state(self: *const Self, i: usize) *const TaskState {
            return &self.states[i];
        }

        /// Raise flags from task context.
        pub fn raise(self: *Self, mask: u32) void {
            self.events.raise(Hal, mask);
        }

        /// Lower flags from task context.
        pub fn clearEvents(self: *Self, mask: u32) void {
            self.events.clear(Hal, mask);
        }

        /// Run every task that is currently due. This is the superloop body.
        pub fn run(self: *Self) void {
            const now = Hal.now();
            const events = self.events.snapshot();
            var ran_any = false;

            // Kick the watchdog once per pass, not once per task: what it is
            // watching is that the loop keeps turning. A pass in which no task
            // was due is still a healthy loop.
            if (comptime hal.hasWatchdog(Hal)) Hal.watchdogKick();

            inline for (defs, 0..) |def, i| {
                const st = &self.states[i];
                const period: u32 = def.period_ms;

                // A suspended sequence is re-polled immediately; a periodic
                // task waits for its grid slot; a period-0 task always runs.
                //
                // This is an `if (due)` block rather than an early `continue`:
                // inside an `inline for`, `continue` is comptime control flow
                // and may not sit inside a runtime conditional.
                const due = st.pending or period == 0 or tick.reached(now, st.next_run);

                if (due) {
                    // Lateness is charged to the *activation*, not to every pass
                    // the activation stays suspended. `next_run` deliberately
                    // does not move while a task is `.pending`, so measuring it
                    // again on each re-poll would report the time the task has
                    // been suspended as lateness and fire one overrun per pass
                    // for a single miss - a task parked on an event for a second
                    // would report a hundred overruns and a second of lateness.
                    const fresh_activation = !st.pending;

                    if (period != 0 and fresh_activation and tick.reached(now, st.next_run)) {
                        const late = tick.elapsed(st.next_run, now);
                        st.last_late = late;
                        if (late > st.max_late) st.max_late = late;

                        // Overrun detection is opt-in per task: a definition
                        // that does not carry `.max_late_ms` never evaluates
                        // this, so the common case is one comptime-known
                        // comparison that folds away.
                        if (comptime maxLateOf(def)) |threshold| {
                            if (late > threshold) {
                                st.overruns +%= 1;
                                if (self.on_overrun) |hook| {
                                    hook(.{
                                        .index = @intCast(i),
                                        .name = def.name,
                                        .late_ms = late,
                                    }, self.overrun_user);
                                }
                            }
                        }
                    }

                    st.runs += 1;
                    ran_any = true;

                    const result = def.poll(def.ctx, now, events);
                    st.pending = result == .pending;

                    if (period != 0 and !st.pending) {
                        const next = st.next_run +% period;
                        if (tick.reached(now, next)) {
                            // Fell behind by more than one period: resync
                            // instead of bursting through the backlog.
                            st.resyncs += 1;
                            st.next_run = now +% period;
                        } else {
                            st.next_run = next;
                        }
                    }
                }
            }

            self.passes += 1;
            if (!ran_any) {
                self.idle_passes += 1;
                if (comptime hal.hasIdle(Hal)) Hal.idle();
            }
        }

        /// Worst lateness across all tasks - the headline health metric.
        pub fn worstLateness(self: *const Self) u32 {
            var worst: u32 = 0;
            inline for (0..task_count) |i| {
                if (self.states[i].max_late > worst) worst = self.states[i].max_late;
            }
            return worst;
        }

        /// Total missed-grid events across all tasks.
        pub fn totalResyncs(self: *const Self) u32 {
            var total: u32 = 0;
            inline for (0..task_count) |i| {
                total += self.states[i].resyncs;
            }
            return total;
        }

        /// Total activations that exceeded a task's `.max_late_ms` threshold.
        ///
        /// Tasks that set no threshold contribute nothing, so a zero result
        /// means either "nothing overran" or "nothing is being watched" - check
        /// that at least one definition carries `.max_late_ms` before reading
        /// this as an all-clear.
        pub fn totalOverruns(self: *const Self) u32 {
            var total: u32 = 0;
            inline for (0..task_count) |i| {
                total += self.states[i].overruns;
            }
            return total;
        }

        /// True if any task definition declares `.max_late_ms`.
        ///
        /// Lets a monitoring task distinguish "no overruns happened" from "no
        /// overruns could have been seen", which is the difference between a
        /// green light and a light that was never wired up.
        pub fn observesOverruns() bool {
            inline for (defs) |def| {
                if (comptime maxLateOf(def) != null) return true;
            }
            return false;
        }
    };
}

fn validateDefs(comptime defs: anytype) void {
    if (defs.len == 0) {
        @compileError("Scheduler requires at least one task");
    }
    inline for (defs, 0..) |def, i| {
        if (!@hasField(@TypeOf(def), "name")) {
            @compileError(std.fmt.comptimePrint("task {d} is missing field `name`", .{i}));
        }
        inline for (.{ "period_ms", "ctx", "poll" }) |field| {
            if (!@hasField(@TypeOf(def), field)) {
                @compileError(std.fmt.comptimePrint(
                    "task '{s}' is missing field `{s}`",
                    .{ def.name, field },
                ));
            }
        }
        // `max_late_ms` is optional, and the type is checked when it is
        // present - an integer literal has type `comptime_int`, not `u32`, so
        // both are accepted and range and sign are left to the coercion in
        // `maxLateOf`.
        //
        // What this cannot do is catch a *misspelling*. Task definitions are
        // anonymous struct literals, so `.max_late = 5` is a perfectly legal
        // field that this loop never looks at, and the task silently gets no
        // overrun detection. A review demonstrated exactly that. The only real
        // fix is a named task-definition type, which would cost the table its
        // current shape; until then, the way to check is `observesOverruns()`,
        // which answers "is anyone watching?" rather than "was anyone late?".
        if (@hasField(@TypeOf(def), "max_late_ms")) {
            switch (@typeInfo(@TypeOf(def.max_late_ms))) {
                .int, .comptime_int => {},
                else => @compileError(std.fmt.comptimePrint(
                    "task '{s}'.max_late_ms must be an integer number of milliseconds, got {s}",
                    .{ def.name, @typeName(@TypeOf(def.max_late_ms)) },
                )),
            }
        }
    }
}

/// The `.max_late_ms` threshold of a task definition, or null if it has none.
///
/// Returned as `?u32` so the caller can use `if (comptime ...)` and have the
/// whole overrun check disappear for definitions that do not opt in. A value
/// that cannot coerce to `u32` is a compile error here, which is where a
/// negative or oversized threshold gets caught.
fn maxLateOf(comptime def: anytype) ?u32 {
    if (!@hasField(@TypeOf(def), "max_late_ms")) return null;
    return @as(u32, def.max_late_ms);
}

// --- test fixtures ---------------------------------------------------------
//
// Task contexts live at container scope, not inside the test bodies. That is
// not a testing workaround: a context has to outlive the scheduler pass, so it
// must have static lifetime. The kernel enforces this by requiring `ctx` to be
// comptime-known, which a pointer to a local `var` is not.

const host = @import("../hal/host.zig");
const program = @import("program.zig");

const Fixture = struct {
    // simple counters
    var counter_a: u32 = 0;
    var counter_b: u32 = 0;
    var seq_steps: u32 = 0;
    var seen: u32 = 0;

    // periodic capture
    var fire_ticks: [64]Tick = undefined;
    var fire_count: u32 = 0;

    fn reset() void {
        counter_a = 0;
        counter_b = 0;
        seq_steps = 0;
        seen = 0;
        fire_count = 0;
        host.HostHal.reset();
    }

    const CountA = struct {
        fn poll(c: *u32, now: Tick, events: u32) Step {
            _ = .{ now, events };
            c.* += 1;
            return .finished;
        }
    };

    const CountB = struct {
        fn poll(c: *u32, now: Tick, events: u32) Step {
            _ = .{ now, events };
            c.* += 1;
            return .finished;
        }
    };

    const Capture = struct {
        fn poll(c: *u32, now: Tick, events: u32) Step {
            _ = events;
            if (c.* < fire_ticks.len) fire_ticks[c.*] = now;
            c.* += 1;
            return .finished;
        }
    };

    /// Suspends twice before finishing, to exercise `.pending` re-polling.
    const Sequencer = struct {
        fn poll(c: *u32, now: Tick, events: u32) Step {
            _ = .{ now, events };
            c.* += 1;
            return if (c.* < 3) .pending else .finished;
        }
    };

    /// A task driven by an event, expressed as a compiled program.
    const Ack: u32 = 0x0001;

    const EvtCtx = struct {
        prog: Prog = .{},
        fn mark(c: *@This()) void {
            seen += 1;
            _ = c;
        }
        const Prog = program.Program(@This(), &.{
            .{ .wait_event = Ack },
            .{ .call = mark },
        });
        fn poll(c: *@This(), now: Tick, events: u32) Step {
            return c.prog.poll(c, now, events);
        }
    };
    var evt_ctx: EvtCtx = .{};
};

test "period-0 task runs on every pass" {
    Fixture.reset();

    const S = Scheduler(host.HostHal, .{
        .{ .name = "every", .period_ms = 0, .ctx = &Fixture.counter_a, .poll = Fixture.CountA.poll },
    });
    var sched = S.init();

    sched.run();
    sched.run();
    sched.run();

    try std.testing.expectEqual(@as(u32, 3), Fixture.counter_a);
    try std.testing.expectEqual(@as(u32, 3), sched.states[0].runs);
}

test "periodic task fires at its rate with exact average and bounded jitter" {
    Fixture.reset();

    const S = Scheduler(host.HostHal, .{
        .{ .name = "p20", .period_ms = 20, .ctx = &Fixture.fire_count, .poll = Fixture.Capture.poll },
    });
    var sched = S.init();

    // 1 ms systick, 1 second of virtual time.
    var t: Tick = 0;
    while (t < 1000) : (t += 1) {
        host.HostHal.setNow(t);
        sched.run();
    }

    // First activation at t=0, then every 20 ms: 0,20,...,980 -> 50 runs.
    try std.testing.expectEqual(@as(u32, 50), Fixture.fire_count);
    try std.testing.expectEqual(@as(Tick, 0), Fixture.fire_ticks[0]);
    try std.testing.expectEqual(@as(Tick, 20), Fixture.fire_ticks[1]);
    try std.testing.expectEqual(@as(Tick, 980), Fixture.fire_ticks[49]);
    try std.testing.expectEqual(@as(u32, 0), sched.worstLateness());
    try std.testing.expectEqual(@as(u32, 0), sched.totalResyncs());
}

test "a stalled loop is detected rather than silently bursting" {
    Fixture.reset();

    const S = Scheduler(host.HostHal, .{
        .{ .name = "p10", .period_ms = 10, .ctx = &Fixture.counter_a, .poll = Fixture.CountA.poll },
    });
    var sched = S.init();
    host.HostHal.setNow(0);
    sched.run();
    try std.testing.expectEqual(@as(u32, 1), Fixture.counter_a);

    // The loop stalls for 100 ms - ten whole periods.
    host.HostHal.setNow(100);
    sched.run();

    // Exactly one catch-up run, not ten: the grid resynced.
    try std.testing.expectEqual(@as(u32, 2), Fixture.counter_a);
    try std.testing.expectEqual(@as(u32, 1), sched.states[0].resyncs);
    try std.testing.expectEqual(@as(u32, 90), sched.states[0].last_late);
    try std.testing.expectEqual(@as(u32, 90), sched.worstLateness());
}

test "two periodic tasks keep independent grids" {
    Fixture.reset();

    const S = Scheduler(host.HostHal, .{
        .{ .name = "fast", .period_ms = 10, .ctx = &Fixture.counter_a, .poll = Fixture.CountA.poll },
        .{ .name = "slow", .period_ms = 50, .ctx = &Fixture.counter_b, .poll = Fixture.CountB.poll },
    });
    var sched = S.init();

    var t: Tick = 0;
    while (t < 1000) : (t += 1) {
        host.HostHal.setNow(t);
        sched.run();
    }

    try std.testing.expectEqual(@as(u32, 100), Fixture.counter_a);
    try std.testing.expectEqual(@as(u32, 20), Fixture.counter_b);
    try std.testing.expectEqual(@as(u32, 0), sched.worstLateness());
}

test "a periodic task keeps its rate across the u32 tick wrap" {
    // Every comparison in the kernel is a signed difference, so the wrap at
    // 2^32 ms should be invisible: the grid must keep its spacing and report no
    // lateness, rather than stalling for 24 days or bursting.
    //
    // The clock is started 200 ms *before* the wrap, which also pins a piece of
    // startup semantics worth being explicit about. A task's grid begins at
    // `next_run = 0`, and `reached(now, 0)` is a signed comparison, so while
    // the clock reads in the upper half of the u32 range it is interpreted as
    // "0 is still 200 ms in the future" - correctly, because that is what a
    // wrapped clock means. The first activation therefore lands at 0, not at
    // `start`, and nothing fires during the 200 ms before it.
    //
    // On real hardware the tick counter begins at 0 and this cannot arise: a
    // scheduler started at a small positive `now` is due immediately. The case
    // only appears when a test jumps the clock into the far future, which is
    // exactly what this test does.
    Fixture.reset();

    const S = Scheduler(host.HostHal, .{
        .{ .name = "p20", .period_ms = 20, .ctx = &Fixture.fire_count, .poll = Fixture.Capture.poll },
    });
    var sched = S.init();

    const start: Tick = 0xFFFF_FF38; // 200 ms before the wrap
    var i: u32 = 0;
    while (i < 400) : (i += 1) {
        host.HostHal.setNow(start +% i);
        sched.run();

        // Nothing may run while the clock still reads as "before 0".
        if (i < 200) {
            try std.testing.expectEqual(@as(u32, 0), Fixture.fire_count);
        }
    }

    // 200 ms of post-wrap ticks at 20 ms is 10 activations.
    try std.testing.expectEqual(@as(u32, 10), Fixture.fire_count);

    // The first lands exactly on the wrap point, and the rest keep the grid.
    try std.testing.expectEqual(@as(Tick, 0), Fixture.fire_ticks[0]);
    try std.testing.expectEqual(@as(Tick, 180), Fixture.fire_ticks[9]);

    var k: u32 = 1;
    while (k < Fixture.fire_count) : (k += 1) {
        const gap = Fixture.fire_ticks[k] -% Fixture.fire_ticks[k - 1];
        try std.testing.expectEqual(@as(Tick, 20), gap);
    }

    // No lateness and no resynchronisation: the wrap is not an error.
    try std.testing.expectEqual(@as(u32, 0), sched.worstLateness());
    try std.testing.expectEqual(@as(u32, 0), sched.totalResyncs());
}

test "a deadline set before the wrap is honoured after it" {
    // The other half of wrap safety: a deadline computed before the wrap must
    // still be reached afterwards, and must not read as reached too early.
    Fixture.reset();

    const S = Scheduler(host.HostHal, .{
        .{ .name = "p100", .period_ms = 100, .ctx = &Fixture.fire_count, .poll = Fixture.Capture.poll },
    });
    var sched = S.init();

    // Establish the grid at t = 0xFFFFFFE0, 32 ms before the wrap.
    host.HostHal.setNow(0xFFFF_FFE0);
    sched.states[0].next_run = 0xFFFF_FFE0;
    sched.run();
    try std.testing.expectEqual(@as(u32, 1), Fixture.fire_count);
    try std.testing.expectEqual(@as(Tick, 0xFFFF_FFE0), Fixture.fire_ticks[0]);
    // Next slot is 100 ms later, which is on the far side of the wrap.
    try std.testing.expectEqual(@as(Tick, 0x0000_0044), sched.states[0].next_run);

    // It must not fire early, on any tick before that deadline.
    var t: Tick = 0xFFFF_FFE1;
    while (t != 0x0000_0044) : (t +%= 1) {
        host.HostHal.setNow(t);
        sched.run();
        try std.testing.expectEqual(@as(u32, 1), Fixture.fire_count);
    }

    // And it must fire exactly on the deadline.
    host.HostHal.setNow(0x0000_0044);
    sched.run();
    try std.testing.expectEqual(@as(u32, 2), Fixture.fire_count);
    try std.testing.expectEqual(@as(Tick, 0x0000_0044), Fixture.fire_ticks[1]);
    try std.testing.expectEqual(@as(u32, 0), sched.worstLateness());
}

test "an event raised by an ISR mid-pass is delivered on the next pass" {
    // The scheduler snapshots the event mask once per pass and hands that
    // snapshot to every task, so an ISR arriving during the pass must not
    // change what the current pass's tasks observe - but it must be visible on
    // the following one, which is what makes event-driven tasks make progress
    // at all.
    //
    // This test also pins the *trigger semantics*, which are easy to get wrong:
    // a flag is level-triggered and is never cleared automatically. Combined
    // with `Program` restarting itself once it runs off the end, a task that
    // waits on a flag nobody consumes will re-run on every single pass. That is
    // by design - clearing is the caller's job, via `clearEvents` - but it is
    // the kind of design that has to be visible, so it is asserted here rather
    // than described only in a comment.
    Fixture.reset();

    const S = Scheduler(host.HostHal, .{
        .{ .name = "evt", .period_ms = 0, .ctx = &Fixture.evt_ctx, .poll = Fixture.EvtCtx.poll },
    });
    var sched = S.init();

    // Pass 1: nothing raised yet, so the program parks on `wait_event`.
    host.HostHal.setNow(0);
    sched.run();
    try std.testing.expectEqual(@as(u32, 0), Fixture.seen);

    // An ISR raises the flag between passes - the interleaving a real UART or
    // timer interrupt produces.
    sched.events.setFromIsr(Fixture.Ack);

    // Pass 2: the new snapshot carries the flag, the program runs to the end.
    host.HostHal.setNow(1);
    sched.run();
    try std.testing.expectEqual(@as(u32, 1), Fixture.seen);

    // Pass 3: the program finished, so it restarted at instruction 0; the flag
    // is still set, so `wait_event` is satisfied immediately and it marks
    // again. Two marks, one event.
    host.HostHal.setNow(2);
    sched.run();
    try std.testing.expectEqual(@as(u32, 2), Fixture.seen);

    // Consuming the flag is what stops it repeating.
    sched.clearEvents(Fixture.Ack);
    host.HostHal.setNow(3);
    sched.run();
    try std.testing.expectEqual(@as(u32, 2), Fixture.seen);
}

test "a level-triggered flag re-fires every pass until it is cleared" {
    // The same property without a Program in the way, so the semantics are
    // stated on their own: `isSet` stays true, and the only thing that ends the
    // condition is an explicit `clearEvents`.
    Fixture.reset();

    const S = Scheduler(host.HostHal, .{
        .{ .name = "p", .period_ms = 0, .ctx = &Fixture.counter_a, .poll = Fixture.CountA.poll },
    });
    var sched = S.init();

    sched.events.setFromIsr(Fixture.Ack);

    var pass: u32 = 0;
    while (pass < 5) : (pass += 1) {
        host.HostHal.setNow(pass);
        sched.run();
        // Still set after every pass, because nothing consumed it.
        try std.testing.expect(sched.events.isSet(Fixture.Ack));
    }
    // And the task ran on every one of those passes.
    try std.testing.expectEqual(@as(u32, 5), Fixture.counter_a);

    sched.clearEvents(Fixture.Ack);
    try std.testing.expect(!sched.events.isSet(Fixture.Ack));
}

test "clearing one event does not disturb another raised during the pass" {
    // A task that consumes the flag it saw must not consume a flag that
    // arrived while it was working; otherwise an event is lost until the next
    // pass, and for an edge-like source, lost for good.
    Fixture.reset();

    const OTHER: u32 = 0x0002;

    const S = Scheduler(host.HostHal, .{
        .{ .name = "p", .period_ms = 0, .ctx = &Fixture.counter_a, .poll = Fixture.CountA.poll },
    });
    var sched = S.init();

    host.HostHal.setNow(0);
    sched.events.setFromIsr(Fixture.Ack);
    const snapshot = sched.events.snapshot();
    try std.testing.expect((snapshot & Fixture.Ack) != 0);
    try std.testing.expect((snapshot & OTHER) == 0);

    // The second event arrives after the snapshot was taken.
    sched.events.setFromIsr(OTHER);

    // Consuming the first must leave the second alone.
    sched.clearEvents(Fixture.Ack);
    try std.testing.expect(!sched.events.isSet(Fixture.Ack));
    try std.testing.expect(sched.events.isSet(OTHER));

    // The snapshot the tasks were handed is unchanged, so every task in the
    // pass agreed on what it saw.
    try std.testing.expectEqual(Fixture.Ack, snapshot);
}

test "a suspended sequence is re-polled promptly instead of waiting a period" {
    Fixture.reset();

    const S = Scheduler(host.HostHal, .{
        .{ .name = "seq", .period_ms = 1000, .ctx = &Fixture.seq_steps, .poll = Fixture.Sequencer.poll },
    });
    var sched = S.init();

    host.HostHal.setNow(0);
    sched.run(); // step 1, pending
    sched.run(); // step 2, pending
    sched.run(); // step 3, finished
    try std.testing.expectEqual(@as(u32, 3), Fixture.seq_steps);

    // The period now applies: nothing runs again until the next grid slot.
    sched.run();
    try std.testing.expectEqual(@as(u32, 3), Fixture.seq_steps);
}

test "lateness is charged once per activation, not once per re-poll" {
    Fixture.reset();

    const S = Scheduler(host.HostHal, .{
        .{
            .name = "seq",
            .period_ms = 100,
            .ctx = &Fixture.seq_steps,
            .poll = Fixture.Sequencer.poll,
            .max_late_ms = 1,
        },
    });
    var sched = S.init();

    // Starts on time at t=0 and then works across three passes while the clock
    // moves. `next_run` deliberately stays put while a task is suspended, so a
    // scheduler that re-measures lateness on every re-poll reports the time the
    // task has been *suspended* as lateness - and fires one overrun per pass
    // for one activation that was never late.
    host.HostHal.setNow(0);
    sched.run();
    host.HostHal.setNow(10);
    sched.run();
    host.HostHal.setNow(20);
    sched.run(); // finishes here, 80 ms before its next slot

    try std.testing.expectEqual(@as(u32, 3), Fixture.seq_steps);
    try std.testing.expectEqual(@as(u32, 0), sched.worstLateness());
    try std.testing.expectEqual(@as(u32, 0), sched.totalOverruns());
    try std.testing.expectEqual(@as(u32, 0), sched.totalResyncs());

    // The next activation is genuinely late and is still counted.
    host.HostHal.setNow(250);
    sched.run();
    try std.testing.expectEqual(@as(u32, 150), sched.worstLateness());
    try std.testing.expectEqual(@as(u32, 1), sched.totalOverruns());
}

test "the watchdog hook is called once per pass, and only if declared" {
    Fixture.reset();

    const S = Scheduler(host.HostHal, .{
        .{ .name = "slow", .period_ms = 100, .ctx = &Fixture.counter_a, .poll = Fixture.CountA.poll },
    });
    var sched = S.init();

    host.HostHal.setNow(0);
    sched.run(); // runs
    host.HostHal.setNow(50);
    sched.run(); // nothing due

    // Both passes kicked, including the one where no task was due: what the
    // watchdog watches is that the loop keeps turning.
    try std.testing.expectEqual(@as(u32, 2), host.HostHal.watchdogKicks());

    // A HAL without the hook compiles to nothing - `hasWatchdog` is comptime,
    // so this is not a runtime branch.
    const Bare = struct {
        pub fn now() Tick {
            return 0;
        }
        pub fn criticalEnter() void {}
        pub fn criticalExit() void {}
    };
    const BareS = Scheduler(Bare, .{
        .{ .name = "t", .period_ms = 10, .ctx = &Fixture.counter_a, .poll = Fixture.CountA.poll },
    });
    var bare = BareS.init();
    bare.run();
    try std.testing.expect(!hal.hasWatchdog(Bare));
}

test "idle hook fires only when nothing is runnable" {
    Fixture.reset();

    const S = Scheduler(host.HostHal, .{
        .{ .name = "slow", .period_ms = 100, .ctx = &Fixture.counter_a, .poll = Fixture.CountA.poll },
    });
    var sched = S.init();

    host.HostHal.setNow(0);
    sched.run(); // runs
    host.HostHal.setNow(50);
    sched.run(); // idle

    try std.testing.expectEqual(@as(u32, 1), host.HostHal.idleCalls());
    try std.testing.expectEqual(@as(u32, 1), sched.idle_passes);
}

test "event raised by an ISR wakes a task that waits on it" {
    Fixture.reset();
    Fixture.evt_ctx = .{};

    const S = Scheduler(host.HostHal, .{
        .{ .name = "isr-driven", .period_ms = 0, .ctx = &Fixture.evt_ctx, .poll = Fixture.EvtCtx.poll },
    });
    var sched = S.init();

    host.HostHal.setNow(0);
    sched.run();
    try std.testing.expectEqual(@as(u32, 0), Fixture.seen);

    // Stands in for a UART ISR: the ISR raises a flag and does nothing else.
    sched.events.setFromIsr(Fixture.Ack);

    host.HostHal.setNow(1);
    sched.run();
    try std.testing.expectEqual(@as(u32, 1), Fixture.seen);
}

test "task names are readable at comptime" {
    const S = Scheduler(host.HostHal, .{
        .{ .name = "alpha", .period_ms = 1, .ctx = &Fixture.counter_a, .poll = Fixture.CountA.poll },
        .{ .name = "beta", .period_ms = 2, .ctx = &Fixture.counter_b, .poll = Fixture.CountB.poll },
    });

    try std.testing.expectEqualStrings("alpha", S.taskName(0));
    try std.testing.expectEqualStrings("beta", S.taskName(1));
    try std.testing.expectEqual(@as(usize, 2), S.task_count);
}

// --- overrun detection -----------------------------------------------------

/// Records what the overrun hook was told. File scope for the same reason task
/// contexts are: the scheduler holds a pointer to it for the whole run.
const OverrunLog = struct {
    calls: u32 = 0,
    last_index: u32 = 0,
    last_late: u32 = 0,
    last_name: []const u8 = "",

    var log: OverrunLog = .{};

    fn reset() void {
        log = .{};
    }

    fn hook(info: Overrun, user: ?*anyopaque) void {
        _ = user;
        log.calls += 1;
        log.last_index = info.index;
        log.last_late = info.late_ms;
        log.last_name = info.name;
    }
};

test "a task with no max_late_ms is never counted as overrunning" {
    Fixture.reset();
    OverrunLog.reset();

    const S = Scheduler(host.HostHal, .{
        .{ .name = "plain", .period_ms = 10, .ctx = &Fixture.counter_a, .poll = Fixture.CountA.poll },
    });
    var sched = S.init();
    sched.setOverrunHook(OverrunLog.hook, null);

    // The scheduler is deliberately not observable for this task, which is
    // what `observesOverruns` exists to let a monitor notice.
    try std.testing.expect(!S.observesOverruns());

    host.HostHal.setNow(0);
    sched.run();
    // Stall past ten whole periods: lateness is large, but nothing is watching.
    host.HostHal.setNow(100);
    sched.run();

    try std.testing.expect(sched.worstLateness() > 0);
    try std.testing.expectEqual(@as(u32, 0), sched.totalOverruns());
    try std.testing.expectEqual(@as(u32, 0), OverrunLog.log.calls);
}

test "the overrun hook fires once per late activation, with the task's name" {
    Fixture.reset();
    OverrunLog.reset();

    const S = Scheduler(host.HostHal, .{
        .{
            .name = "watched",
            .period_ms = 10,
            .ctx = &Fixture.counter_a,
            .poll = Fixture.CountA.poll,
            .max_late_ms = 5,
        },
    });
    var sched = S.init();
    sched.setOverrunHook(OverrunLog.hook, null);

    try std.testing.expect(S.observesOverruns());

    host.HostHal.setNow(0);
    sched.run();
    // On time: no overrun.
    try std.testing.expectEqual(@as(u32, 0), sched.totalOverruns());
    try std.testing.expectEqual(@as(u32, 0), OverrunLog.log.calls);

    // 100 ms late, which is over the 5 ms threshold.
    host.HostHal.setNow(100);
    sched.run();
    try std.testing.expectEqual(@as(u32, 1), sched.totalOverruns());
    try std.testing.expectEqual(@as(u32, 1), OverrunLog.log.calls);
    try std.testing.expectEqualStrings("watched", OverrunLog.log.last_name);
    try std.testing.expectEqual(@as(u32, 0), OverrunLog.log.last_index);
    try std.testing.expectEqual(@as(u32, 90), OverrunLog.log.last_late);

    // The grid resynced, so the next activation is on time again: the counter
    // tracks overruns, not "has ever overrun".
    host.HostHal.setNow(110);
    sched.run();
    try std.testing.expectEqual(@as(u32, 1), sched.totalOverruns());
    try std.testing.expectEqual(@as(u32, 1), OverrunLog.log.calls);
}

test "the overrun hook reports which task, when several are watched" {
    Fixture.reset();
    OverrunLog.reset();

    const S = Scheduler(host.HostHal, .{
        .{ .name = "first", .period_ms = 10, .ctx = &Fixture.counter_a, .poll = Fixture.CountA.poll, .max_late_ms = 1 },
        .{ .name = "second", .period_ms = 10, .ctx = &Fixture.counter_b, .poll = Fixture.CountB.poll, .max_late_ms = 1 },
    });
    var sched = S.init();
    sched.setOverrunHook(OverrunLog.hook, null);

    host.HostHal.setNow(0);
    sched.run();
    try std.testing.expectEqual(@as(u32, 0), OverrunLog.log.calls);

    // Both are late by the same amount, so both must report; the log keeps the
    // last one, and the index must match the second declaration.
    host.HostHal.setNow(50);
    sched.run();
    try std.testing.expectEqual(@as(u32, 2), OverrunLog.log.calls);
    try std.testing.expectEqual(@as(u32, 1), OverrunLog.log.last_index);
    try std.testing.expectEqualStrings("second", OverrunLog.log.last_name);
    try std.testing.expectEqual(@as(u32, 2), sched.totalOverruns());
}

test "clearing the overrun hook stops the notifications but not the counting" {
    Fixture.reset();
    OverrunLog.reset();

    const S = Scheduler(host.HostHal, .{
        .{ .name = "w", .period_ms = 10, .ctx = &Fixture.counter_a, .poll = Fixture.CountA.poll, .max_late_ms = 1 },
    });
    var sched = S.init();

    host.HostHal.setNow(50);
    sched.run();
    try std.testing.expectEqual(@as(u32, 1), sched.totalOverruns());
    try std.testing.expectEqual(@as(u32, 0), OverrunLog.log.calls);

    sched.setOverrunHook(OverrunLog.hook, null);
    host.HostHal.setNow(100);
    sched.run();
    try std.testing.expectEqual(@as(u32, 2), sched.totalOverruns());
    try std.testing.expectEqual(@as(u32, 1), OverrunLog.log.calls);

    sched.setOverrunHook(null, null);
    host.HostHal.setNow(200);
    sched.run();
    // Counting continues; only the push notification stopped.
    try std.testing.expectEqual(@as(u32, 3), sched.totalOverruns());
    try std.testing.expectEqual(@as(u32, 1), OverrunLog.log.calls);
}
