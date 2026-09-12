//! Fusion proof: an application built from modules on a real Smartcar target.
//!
//! This is a firmware skeleton for CYT2BL3 / CYT4BB7 / RT1064 that exercises
//! every piece of the XRobot-informed design at once:
//!
//!   * `App` composes modules with compile-time dependency checking;
//!   * `Program` gives a module sequential logic without a stack;
//!   * `Channel` moves bytes from an ISR to a task with caller-owned storage;
//!   * `Topic` frames telemetry in LibXR's wire format;
//!   * the whole thing is one `Scheduler` over a comptime task table.
//!
//! Verified by `zig build check-targets`, which compiles it for
//! `cortex_m0plus`, `cortex_m4+vfp4d16sp` and `cortex_m7+fp_armv8d16sp`.
//!
//! The point of this file is the *absence* of things: no heap, no YAML, no
//! generated main, no runtime registry, no lock, and no per-module stack.

const breeze = @import("breeze");
const hal = breeze.hal.cortex_m;
const Tick = breeze.Tick;
const Step = breeze.Step;

// --- events -----------------------------------------------------------------

const EVT_UART_RX: u32 = 1 << 0;
const EVT_TX_DONE: u32 = 1 << 1;

// --- topics -----------------------------------------------------------------

/// Attitude, published at the IMU rate.
const Attitude = breeze.Topic("attitude", extern struct {
    roll: f32,
    pitch: f32,
    yaw: f32,
});

/// Encoder counts, published by the chassis module.
const Encoders = breeze.Topic("encoders", extern struct {
    left: i32,
    right: i32,
    tick_ms: u32,
});

/// Kernel health, published slowly so a bench session can see stalls.
const Health = breeze.Topic("health", extern struct {
    worst_late_ms: u32,
    resyncs: u32,
    rx_dropped: u32,
});

// --- shared ports -----------------------------------------------------------
//
// Storage is declared here, at file scope, so the RAM cost of every driver is
// visible in one place and is known at compile time. This is the main
// deliberate difference from LibXR, whose SPSCQueue allocates in its
// constructor.

var uart_rx_storage: [64]u8 = undefined;
var uart_rx = breeze.CountedChannel(u8, 63).init(&uart_rx_storage);

var tx_storage: [64]u8 = undefined;
var tx_queue = breeze.Channel(u8, 63).init(&tx_storage);

/// Where an outgoing topic frame is assembled. One buffer, reused, because
/// frames are packed and handed to the UART in the same pass.
var frame_buf: [64]u8 = undefined;

// --- module: IMU ------------------------------------------------------------

const Imu = struct {
    pub const manifest = breeze.Manifest{
        .name = "Imu",
        .description = "Samples the IMU and publishes attitude",
        .hardware = &.{"i2c0"},
        .provides = &.{"imu"},
        .publishes = &.{"attitude"},
        .period_ms = 5, // 200 Hz
        .stack_hint = 64,
    };

    pub const Config = struct {
        /// Low-pass coefficient, 0 = no filtering.
        alpha: f32 = 0.2,
        /// Mounting offset subtracted from yaw, in radians.
        yaw_offset: f32 = 0.0,
    };

    pub const State = struct {
        cfg: Config = .{},
        attitude: Attitude.Value = .{ .roll = 0, .pitch = 0, .yaw = 0 },
        samples: u32 = 0,
    };

    pub fn init(self: *State, cfg: Config) void {
        self.cfg = cfg;
        self.attitude = .{ .roll = 0, .pitch = 0, .yaw = 0 };
        self.samples = 0;
        // i2c0.configure(.{ .rate_hz = 400_000 });
    }

    pub fn poll(self: *State, now: Tick, events: u32) Step {
        _ = events;
        if (!imuDataReady()) return .finished;

        // Stands in for a real read; the filter below is the interesting part.
        const raw_roll: f32 = 0.01;
        const raw_pitch: f32 = -0.02;
        const raw_yaw: f32 = 0.5;

        self.attitude.roll += self.cfg.alpha * (raw_roll - self.attitude.roll);
        self.attitude.pitch += self.cfg.alpha * (raw_pitch - self.attitude.pitch);
        self.attitude.yaw = raw_yaw - self.cfg.yaw_offset;
        self.samples += 1;

        _ = publish(Attitude, &self.attitude, now);
        return .finished;
    }

    fn imuDataReady() bool {
        // return (i2c0.status() & DATA_READY) != 0;
        return true;
    }
};

// --- module: chassis --------------------------------------------------------

const Chassis = struct {
    pub const manifest = breeze.Manifest{
        .name = "Chassis",
        .description = "Reads encoders and drives the motors",
        .hardware = &.{ "encoder_l", "encoder_r", "motor_l", "motor_r" },
        .provides = &.{"chassis"},
        .publishes = &.{"encoders"},
        .period_ms = 10, // 100 Hz
        .stack_hint = 96,
    };

    pub const Config = struct {
        counts_per_meter: f32 = 2000.0,
        max_duty: u16 = 9000,
    };

    pub const State = struct {
        cfg: Config = .{},
        left: i32 = 0,
        right: i32 = 0,
        target_left: i16 = 0,
        target_right: i16 = 0,
    };

    pub fn init(self: *State, cfg: Config) void {
        self.cfg = cfg;
        self.left = 0;
        self.right = 0;
        self.target_left = 0;
        self.target_right = 0;
    }

    pub fn poll(self: *State, now: Tick, events: u32) Step {
        _ = events;
        // Four-sided quadrature decoding happens in hardware.
        self.left = readEncoder(0);
        self.right = readEncoder(1);

        // A proportional loop is enough to show the shape; the real one is a
        // `Program` that also ramps, measures, and reports a fault.
        setMotor(0, self.target_left);
        setMotor(1, self.target_right);

        const sample = Encoders.Value{
            .left = self.left,
            .right = self.right,
            .tick_ms = now,
        };
        _ = publish(Encoders, &sample, now);
        return .finished;
    }

    fn readEncoder(id: u8) i32 {
        _ = id;
        return 0;
    }

    fn setMotor(id: u8, duty: i16) void {
        _ = .{ id, duty };
    }
};

// --- module: telemetry uplink ----------------------------------------------
//
// This is the module a student actually watches. It owns the byte channel the
// UART ISR fills, and it reconstructs frames from that stream. Because the ISR
// only ever pushes bytes and raises a flag, and this task is the only consumer,
// no lock is needed anywhere on the path.

const Uplink = struct {
    pub const manifest = breeze.Manifest{
        .name = "Uplink",
        .description = "Frames telemetry and drains the UART RX ring",
        .hardware = &.{"uart0"},
        .provides = &.{"telemetry"},
        .publishes = &.{"health"},
        .period_ms = 0, // 0 = polled on every pass; the ring is drained often
        .stack_hint = 128,
    };

    pub const State = struct {
        rx_bytes: u32 = 0,
        bad_frames: u32 = 0,
        last_health_ms: Tick = 0,
    };

    pub fn poll(self: *State, now: Tick, events: u32) Step {
        // Drain whatever the ISR queued. The ISR never blocks and never loops;
        // this is where the bytes are actually looked at.
        var scratch: [32]u8 = undefined;
        const n = readAvailable(&scratch);
        self.rx_bytes += @intCast(n);
        if (n > 0 and !looksLikeFrame(scratch[0..n])) {
            self.bad_frames += 1;
        }

        if ((events & EVT_UART_RX) != 0) {
            sched.clearEvents(EVT_UART_RX);
        }

        // A slow health beacon, so a benched car reports its own scheduling
        // problems instead of just looking jerky.
        if (breeze.time.elapsed(self.last_health_ms, now) >= 1000) {
            self.last_health_ms = now;
            const h = Health.Value{
                .worst_late_ms = sched.worstLateness(),
                .resyncs = sched.totalResyncs(),
                .rx_dropped = uart_rx.droppedCount(),
            };
            _ = publish(Health, &h, now);
        }
        return .finished;
    }

    fn readAvailable(out: []u8) usize {
        var i: usize = 0;
        while (i < out.len) : (i += 1) {
            out[i] = uart_rx.pop() orelse break;
        }
        return i;
    }

    fn looksLikeFrame(bytes: []const u8) bool {
        return bytes.len == 0 or bytes[0] == breeze.telemetry.packet_prefix;
    }
};

// --- module: bootstrap sequence --------------------------------------------
//
// Sequential startup logic - bring two pieces of hardware up at the same time,
// retry, then release the motors - written as a comptime program rather than a
// state machine or a blocking delay loop.

const Boot = struct {
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

    pub const State = struct {
        prog: Prog = .{},
        join: Join = .{},
        cfg: Config = .{},
        now: Tick = 0,
        started_ms: Tick = 0,
        attempts: u32 = 0,

        /// Null until the sequence settles, then whether it succeeded.
        ///
        /// The latch is not optional: a `Program` resets to instruction 0 when
        /// it finishes, so without it the boot sequence would start over on
        /// every later pass.
        outcome: ?bool = null,

        /// The two branches, named once. Each hardware handler completes the
        /// slot it owns, so a retry never has to work out which one answered.
        pub const IMU = 0;
        pub const ESC = 1;

        /// The join type, so that the slot count lives in exactly one place:
        /// this declaration and the two names above.
        const Join = breeze.Join.Of(u16, 2);

        fn startAttempt(ctx: *@This()) void {
            ctx.attempts += 1;
            ctx.started_ms = ctx.now;
            // Clearing first is what makes a retry a *new* round: a slot left
            // failed by the previous attempt would otherwise keep the join
            // failed even once every branch had answered.
            ctx.join.reset();
            // Register both slots before starting either transfer, or a
            // completion that arrives immediately has nowhere to land.
            ctx.join.enterAll();
            // imu0.configure(); esc0.selfTest();
        }

        /// The gather's condition, evaluated on every pass until it says stop.
        ///
        /// The timeout lives here rather than in a `wait_event_timeout`, because
        /// what is being waited on is not one flag. A branch that has not
        /// reported when the budget is gone is not going to - and naming slots
        /// is what makes saying so a one-liner. There is no way to fail *the
        /// missing branch* of a plain counter.
        fn bothIn(ctx: *@This()) bool {
            if (!ctx.join.allDone()) {
                if (breeze.time.elapsed(ctx.started_ms, ctx.now) < ctx.cfg.timeout_ms) return false;
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

// --- application ------------------------------------------------------------

var imu_state: Imu.State = .{};
var chassis_state: Chassis.State = .{};
var uplink_state: Uplink.State = .{};
var boot_state: Boot.State = .{};

const App = breeze.AppWithHardware(.{
    .{ .module = Boot, .state = &boot_state, .config = .{ .timeout_ms = 250 } },
    .{ .module = Imu, .state = &imu_state, .config = .{ .alpha = 0.3 } },
    .{ .module = Chassis, .state = &chassis_state, .config = .{ .counts_per_meter = 1850.0 } },
    .{ .module = Uplink, .state = &uplink_state },
}, &.{
    "i2c0",
    "uart0",
    "esc0",
    "encoder_l",
    "encoder_r",
    "motor_l",
    "motor_r",
});

const Sched = App.SchedulerFor(hal);
var sched = Sched.init();

// --- helpers ----------------------------------------------------------------

/// Frame `value` and hand it to the UART transmit queue.
///
/// Returns false when the frame did not fit or the queue was full. A dropped
/// telemetry frame is a diagnosable event, not something to block on.
fn publish(comptime T: type, value: *const T.Value, now: Tick) bool {
    const frame = T.pack(&frame_buf, value, micros(now)) catch return false;
    for (frame) |b| {
        if (!tx_queue.pushFromIsr(b)) return false;
    }
    startTransmit();
    return true;
}

fn startTransmit() void {
    // uart0.startTx(tx_queue) - the real driver drains the queue from its TX
    // interrupt and raises EVT_TX_DONE when it empties.
}

fn micros(now: Tick) u64 {
    return @as(u64, now) * 1000;
}

// --- interrupts -------------------------------------------------------------
//
// The complete ISR vocabulary of this application: advance time, push a byte,
// raise a flag, complete a gather slot. No module code, no allocation, no
// formatting.
//
// UART0_RX is the only handler here that raises an event flag, so the
// "flag-raising interrupts share one priority" rule in `breeze.kernel.hal` is
// satisfied by there being only one of them. The moment a second handler calls
// `setFromIsr`, both must be assigned the same NVIC priority - they cannot
// preempt each other, or the preempted one writes back a stale word and the
// other's flag is lost. The two handlers below that complete gather slots are
// not affected: `Join.Of` gives each branch its own word.

export fn SysTick_Handler() callconv(.c) void {
    hal.tickIsr();
}

export fn UART0_RX_Handler() callconv(.c) void {
    _ = uart_rx.pushFromIsr(0x00); // uart0.rx.read()
    sched.events.setFromIsr(EVT_UART_RX);
}

/// A gather slot, seen from the interrupt side: name the branch, hand over what
/// it measured.
///
/// If this fires before `Boot` has started an attempt, the slot is not running
/// and the completion is refused - the join will time out and retry, which is
/// the right answer for a peripheral that answered before it was asked.
export fn IMU_INT_Handler() callconv(.c) void {
    boot_state.join.finish(Boot.State.IMU, 0); // imu0.whoAmI()
}

export fn ESC_READY_Handler() callconv(.c) void {
    boot_state.join.finish(Boot.State.ESC, 0); // esc0.selfTestResult()
}

// --- entry point ------------------------------------------------------------

export fn breeze_main() callconv(.c) noreturn {
    hal.init(160_000_000); // CYT2BL3 runs at 160 MHz

    App.initAll();

    while (true) {
        sched.run(); // `wfi` happens inside when nothing is runnable
    }
}
