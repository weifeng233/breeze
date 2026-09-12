//! Bring-up sequence: two peripherals up at once, retry, then release the
//! chassis.
//!
//! This is the module that shows the two halves of Breeze's execution model
//! working together:
//!
//! * the *sequence* - attempt, wait, decide, retry, give up - is a comptime
//!   instruction list expanded into a state machine by `breeze.Program`, with no
//!   hand-written `switch` and no blocking delay;
//! * the *fan-out* - the IMU and the ESC coming up together, each reporting
//!   from its own interrupt - is a `breeze.Join.Of`, so the timeout can fail
//!   whichever branch is still missing instead of waiting forever.
//!
//! Generic over `Io` like the rest - see `imu.zig`.

const breeze = @import("breeze");

const Tick = breeze.Tick;
const Step = breeze.Step;

/// Build the module over an I/O surface.
///
/// `Io` must provide `startImu()` and `startEsc()`.
pub fn Boot(comptime Io: type) type {
    return struct {
        pub const manifest = breeze.Manifest{
            .name = "Boot",
            .description = "Brings the IMU and the ESC up together, then enables the chassis",
            .depends = &.{"imu"},
            .hardware = &.{"esc0"},
            .provides = &.{"booted"},
            .period_ms = 0, // 0 = polled on every scheduler pass
            .stack_hint = 48,
        };

        pub const Config = struct {
            /// How long both peripherals have to answer before they are written off.
            timeout_ms: u32 = 250,
            /// How many times the whole gather is retried before giving up.
            attempts_max: u32 = 3,
        };

        /// The two branches, named once. Each hardware handler completes the
        /// slot it owns, so a retry never has to work out which one answered.
        pub const IMU = 0;
        pub const ESC = 1;

        /// The join type, so that the slot count lives in exactly one place:
        /// this declaration and the two names above.
        const Join = breeze.Join.Of(u16, 2);

        pub const State = struct {
            prog: Prog = .{},
            join: Join = .{},
            cfg: Config = .{},
            now: Tick = 0,
            started_ms: Tick = 0,
            attempts: u32 = 0,

            /// Null until the sequence settles, then whether it succeeded.
            ///
            /// The latch is not optional: a `Program` resets to instruction 0
            /// when it finishes, so without it the boot sequence would start
            /// over on every later pass.
            outcome: ?bool = null,

            fn startAttempt(ctx: *@This()) void {
                ctx.attempts += 1;
                ctx.started_ms = ctx.now;
                // Clearing first is what makes a retry a *new* round: a slot
                // left failed by the previous attempt would otherwise keep the
                // join failed even once every branch had answered.
                ctx.join.reset();
                // Register both slots before starting either transfer, or a
                // completion that arrives immediately has nowhere to land.
                ctx.join.enterAll();
                Io.startImu();
                Io.startEsc();
            }

            /// The gather's condition, evaluated on every pass until it says stop.
            ///
            /// The timeout lives here rather than in a `wait_event_timeout`,
            /// because what is being waited on is not one flag. A branch that
            /// has not reported when the budget is gone is not going to - and
            /// naming slots is what makes saying so a one-liner. There is no way
            /// to fail *the missing branch* of a plain counter.
            fn bothIn(ctx: *@This()) bool {
                if (!ctx.join.allDone()) {
                    if (breeze.time.elapsed(ctx.started_ms, ctx.now) < ctx.cfg.timeout_ms) {
                        return false;
                    }
                    inline for (0..Join.capacity) |slot| {
                        if (ctx.join.stateOf(slot) == .running) ctx.join.fail(slot);
                    }
                }
                return true;
            }

            fn settled(ctx: *@This()) bool {
                return ctx.outcome != null;
            }
            fn degraded(ctx: *@This()) bool {
                return ctx.join.anyFailed();
            }
            fn mayRetry(ctx: *@This()) bool {
                return ctx.attempts < ctx.cfg.attempts_max;
            }
            fn onOk(ctx: *@This()) void {
                ctx.outcome = true;
            }
            fn onFail(ctx: *@This()) void {
                ctx.outcome = false;
            }

            const instrs = [_]breeze.Instr(@This()){
                // 0: the sequence runs once, so the first thing it does is check
                //    whether it already has.
                .{ .branch_if = .{ .pred = settled, .target = 8 } },
                // 1
                .{ .call = startAttempt },
                // 2: re-polled every pass until both branches are in, or the
                //    attempt times out and its stragglers are failed
                .{ .call_until = bothIn },
                // 3: anything missing?
                .{ .branch_if = .{ .pred = degraded, .target = 6 } },
                // 4
                .{ .call = onOk },
                // 5: success, so skip the retry arms
                .{ .jump = 8 },
                // 6: retry the whole gather if attempts remain
                .{ .branch_if = .{ .pred = mayRetry, .target = 1 } },
                // 7
                .{ .call = onFail },
                // 8
                .finish,
            };
            const Prog = breeze.Program(@This(), &instrs);
        };

        pub fn init(self: *State, cfg: Config) void {
            self.cfg = cfg;
            self.prog = .{};
            self.join = .{};
            self.attempts = 0;
            self.outcome = null;
        }

        pub fn poll(self: *State, now: Tick, events: u32) Step {
            self.now = now;
            return self.prog.poll(self, now, events);
        }
    };
}
