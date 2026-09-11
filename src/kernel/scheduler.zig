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
};

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

        pub fn init() Self {
            return .{};
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
                    if (period != 0 and tick.reached(now, st.next_run)) {
                        const late = tick.elapsed(st.next_run, now);
                        st.last_late = late;
                        if (late > st.max_late) st.max_late = late;
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
    }
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
