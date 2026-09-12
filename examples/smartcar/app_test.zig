//! Host tests for the application.
//!
//! # Why this runs on a workstation
//!
//! `modules/` takes its I/O surface as a comptime type parameter, so nothing in
//! it names a register, a pin or a peripheral. The same is true of `app.zig` for
//! the *wiring*: `App(Io, Hal)` is a function, so the module graph the firmware
//! runs is the graph instantiated below, not a copy of it.
//!
//! That distinction is the whole point. The Smartcar template mirrors its wiring
//! in its own test (`app_test.zig` there) and says so, because its board pulls in
//! the SeekFree C library; here the board is Zig, so there is nothing to mirror
//! and no way for the two to disagree.
//!
//! What this buys, concretely: the bring-up sequence below is exercised through
//! its success, retry and timeout paths, the one-shot latch that keeps it from
//! restarting is pinned, and the frames the modules produce are unpacked with the
//! real `Topic` code rather than inspected as bytes. None of that needs hardware.
//!
//! # Running
//!
//! ```text
//! zig build test-example      # or: zig test examples/smartcar/app_test.zig
//! ```

const std = @import("std");
const breeze = @import("breeze");

/// Breeze's own virtual-clock backend. The template keeps a local copy of this
/// contract because the vendored kernel is a separate module there; in the
/// kernel's own repository it is simply imported.
const host = breeze.hal.host;

const app = @import("app.zig");
const topics = @import("topics.zig");

const imu = @import("modules/imu.zig");
const uplink = @import("modules/uplink.zig");

const Tick = breeze.Tick;

// --- the fake board ---------------------------------------------------------

/// Records what a module asks the board to do, instead of doing it.
///
/// Fixed buffers and no allocation, mirroring the constraint the firmware is
/// under. The captured frame is a real frame - `publish` runs the same
/// `Topic.pack` the board does - so a test can unpack it.
const FakeIo = struct {
    /// What `readImu` returns; null models a part with nothing new.
    var imu_sample: ?imu.Sample = .{ .roll = 1.0, .pitch = 2.0, .yaw = 0.5 };
    var encoder_counts: [2]i32 = .{ 10, -20 };
    var motor_duty: [2]i16 = .{ 0, 0 };
    var motor_writes: u32 = 0;
    var started_imu: u32 = 0;
    var started_esc: u32 = 0;

    var rx: [64]u8 = undefined;
    var rx_len: usize = 0;
    var rx_read: usize = 0;
    var cleared_events: u32 = 0;

    var health_value: uplink.HealthReading = .{};

    var frame_scratch: [64]u8 = undefined;
    var attitude_frames: u32 = 0;
    var encoder_frames: u32 = 0;
    var health_frames: u32 = 0;
    var last_attitude: [64]u8 = undefined;
    var last_attitude_len: usize = 0;
    var last_health: [64]u8 = undefined;
    var last_health_len: usize = 0;

    fn reset() void {
        imu_sample = .{ .roll = 1.0, .pitch = 2.0, .yaw = 0.5 };
        encoder_counts = .{ 10, -20 };
        motor_duty = .{ 0, 0 };
        motor_writes = 0;
        started_imu = 0;
        started_esc = 0;
        rx_len = 0;
        rx_read = 0;
        cleared_events = 0;
        health_value = .{};
        attitude_frames = 0;
        encoder_frames = 0;
        health_frames = 0;
        last_attitude_len = 0;
        last_health_len = 0;
    }

    pub fn readImu() ?imu.Sample {
        return imu_sample;
    }

    pub fn readEncoder(id: u8) i32 {
        return encoder_counts[id];
    }

    pub fn setMotor(id: u8, duty: i16) void {
        motor_duty[id] = duty;
        motor_writes += 1;
    }

    pub fn startImu() void {
        started_imu += 1;
    }

    pub fn startEsc() void {
        started_esc += 1;
    }

    pub fn drainRx(out: []u8) usize {
        const available = rx_len - rx_read;
        const n = @min(out.len, available);
        @memcpy(out[0..n], rx[rx_read..][0..n]);
        rx_read += n;
        return n;
    }

    pub fn clearEvents(mask: u32) void {
        cleared_events |= mask;
    }

    pub fn health() uplink.HealthReading {
        return health_value;
    }

    pub fn publish(comptime T: type, value: *const T.Value, now: Tick) bool {
        const frame = T.pack(&frame_scratch, value, @as(u64, now) * 1000) catch return false;
        if (T == topics.Attitude) {
            @memcpy(last_attitude[0..frame.len], frame);
            last_attitude_len = frame.len;
            attitude_frames += 1;
        } else if (T == topics.Encoders) {
            encoder_frames += 1;
        } else if (T == topics.Health) {
            @memcpy(last_health[0..frame.len], frame);
            last_health_len = frame.len;
            health_frames += 1;
        }
        return true;
    }
};

// --- the application under test ---------------------------------------------

/// The real wiring, over the fake board and the virtual clock.
const App = app.App(FakeIo, host.HostHal);

// --- helpers ---------------------------------------------------------------

fn resetAll() void {
    FakeIo.reset();
    host.HostHal.reset();
    App.boot_state = .{};
    App.imu_state = .{};
    App.chassis_state = .{};
    App.uplink_state = .{};
    App.initAll();
}

/// Play the role of the two interrupts that complete the boot gather.
///
/// The handlers in `firmware.zig` are exactly these two lines; nothing else
/// about them is testable, which is the reason they are kept that small.
fn imuReports() void {
    App.boot_state.join.finish(App.Boot.IMU, 0);
}
fn escReports() void {
    App.boot_state.join.finish(App.Boot.ESC, 0);
}
fn imuFails() void {
    App.boot_state.join.fail(App.Boot.IMU);
}

// --- the graph --------------------------------------------------------------

test "the application declares four modules in order" {
    resetAll();

    try std.testing.expectEqual(@as(usize, 4), App.module_count);
    try std.testing.expectEqualStrings("boot0", App.instanceName(0));
    try std.testing.expectEqualStrings("imu0", App.instanceName(1));
    try std.testing.expectEqualStrings("chassis0", App.instanceName(2));
    try std.testing.expectEqualStrings("uplink0", App.instanceName(3));
}

test "periods come from the manifests, not from a second list" {
    resetAll();

    try std.testing.expectEqual(@as(u32, 0), App.tasks[0].period_ms); // Boot: every pass
    try std.testing.expectEqual(@as(u32, 5), App.tasks[1].period_ms); // Imu: 200 Hz
    try std.testing.expectEqual(@as(u32, 10), App.tasks[2].period_ms); // Chassis: 100 Hz
    try std.testing.expectEqual(@as(u32, 0), App.tasks[3].period_ms); // Uplink: every pass
}

test "initAll runs the module initialisers in declaration order" {
    resetAll();

    // Imu copies its configured alpha.
    try std.testing.expectEqual(@as(f32, 0.3), App.imu_state.cfg.alpha);
    // Chassis copies its counts-per-metre.
    try std.testing.expectEqual(@as(f32, 1850.0), App.chassis_state.cfg.counts_per_meter);
    // Uplink was told which event bit to consume.
    try std.testing.expectEqual(app.EVT_UART_RX, App.uplink_state.cfg.rx_event);
}

// --- the modules ------------------------------------------------------------

test "the IMU samples at its period and publishes unpackable frames" {
    resetAll();

    const S = App.Scheduler;
    var sched = S.init();
    host.HostHal.runFor(&sched, 100); // t = 0..99

    // 5 ms period over 100 ms of virtual time, first activation at t=0.
    try std.testing.expectEqual(@as(u32, 20), App.imu_state.samples);
    try std.testing.expectEqual(@as(u32, 20), FakeIo.attitude_frames);
    try std.testing.expectEqual(@as(u32, 0), App.imu_state.drops);

    // The captured frame is a real frame: unpack it with the real decoder.
    const back = try topics.Attitude.unpack(FakeIo.last_attitude[0..FakeIo.last_attitude_len]);
    try std.testing.expectEqual(@as(f32, 0.5), back.yaw); // raw yaw, no offset configured
    try std.testing.expect(back.roll > 0.0 and back.roll < 1.0); // filtered towards 1.0
    try std.testing.expectEqual(@as(u32, 0), sched.worstLateness());
}

test "an IMU with nothing new publishes nothing" {
    resetAll();
    FakeIo.imu_sample = null;

    var sched = App.Scheduler.init();
    host.HostHal.runFor(&sched, 100);

    try std.testing.expectEqual(@as(u32, 0), App.imu_state.samples);
    try std.testing.expectEqual(@as(u32, 0), FakeIo.attitude_frames);
}

test "the chassis clamps motor duty to the configured maximum" {
    resetAll();
    App.chassis_state.target_left = 32000;
    App.chassis_state.target_right = -32000;

    var sched = App.Scheduler.init();
    host.HostHal.runFor(&sched, 10);

    // max_duty is 9000 by default; both motors see the clamp, not the request.
    try std.testing.expectEqual(@as(i16, 9000), FakeIo.motor_duty[App.Chassis.LEFT]);
    try std.testing.expectEqual(@as(i16, -9000), FakeIo.motor_duty[App.Chassis.RIGHT]);
    try std.testing.expectEqual(@as(i32, 10), App.chassis_state.left);
    try std.testing.expectEqual(@as(u32, 1), FakeIo.encoder_frames);
}

test "the uplink drains the ring, judges the header, and clears the flag" {
    resetAll();
    var sched = App.Scheduler.init();
    sched.events.setFromIsr(app.EVT_UART_RX);

    // A drain that does not start with the frame prefix cannot be a frame.
    FakeIo.rx[0] = 0x00;
    FakeIo.rx[1] = breeze.telemetry.packet_prefix;
    FakeIo.rx_len = 2;
    host.HostHal.runFor(&sched, 1);

    try std.testing.expectEqual(@as(u32, 2), App.uplink_state.rx_bytes);
    try std.testing.expectEqual(@as(u32, 1), App.uplink_state.bad_frames);
    try std.testing.expectEqual(app.EVT_UART_RX, FakeIo.cleared_events);

    // Level-triggered: having cleared it, the task is not re-run for the same
    // bytes, and a second drain finds the ring empty.
    const before = App.uplink_state.rx_bytes;
    host.HostHal.runFor(&sched, 5);
    try std.testing.expectEqual(before, App.uplink_state.rx_bytes);

    // A full header claiming the wrong version is a bad frame...
    FakeIo.rx_read = 0;
    FakeIo.rx_len = breeze.telemetry.header_len;
    FakeIo.rx[0] = breeze.telemetry.packet_prefix;
    FakeIo.rx[14] = 0xFF;
    host.HostHal.runFor(&sched, 1);
    try std.testing.expectEqual(@as(u32, 2), App.uplink_state.bad_frames);

    // ...but a partial one is not judged at all: the rest is still in the ring.
    FakeIo.rx_read = 0;
    FakeIo.rx_len = 3;
    host.HostHal.runFor(&sched, 1);
    try std.testing.expectEqual(@as(u32, 2), App.uplink_state.bad_frames);
}

test "the health beacon reports what the board was told" {
    resetAll();
    FakeIo.health_value = .{ .worst_late_ms = 7, .resyncs = 2, .rx_dropped = 9 };

    var sched = App.Scheduler.init();
    // 1001 passes: `runFor` covers t = 0..1000 inclusive, and the first beacon
    // is due at exactly t = 1000. The module starts with an origin, so the
    // period is measured from boot rather than firing immediately.
    host.HostHal.runFor(&sched, 1001);

    try std.testing.expectEqual(@as(u32, 1), FakeIo.health_frames);

    const back = try topics.Health.unpack(FakeIo.last_health[0..FakeIo.last_health_len]);
    try std.testing.expectEqual(@as(u32, 7), back.worst_late_ms);
    try std.testing.expectEqual(@as(u32, 2), back.resyncs);
    try std.testing.expectEqual(@as(u32, 9), back.rx_dropped);
}

// --- the bring-up sequence --------------------------------------------------

test "boot brings both peripherals up at once and settles once" {
    resetAll();

    var sched = App.Scheduler.init();

    // First pass: both slots are registered and both transfers are started,
    // before either completion can arrive.
    sched.run();
    try std.testing.expectEqual(@as(u32, 1), App.boot_state.attempts);
    try std.testing.expectEqual(@as(u32, 1), FakeIo.started_imu);
    try std.testing.expectEqual(@as(u32, 1), FakeIo.started_esc);
    try std.testing.expectEqual(@as(u16, 2), App.boot_state.join.outstanding());
    try std.testing.expect(App.boot_state.outcome == null);

    // The two interrupts report, in the opposite order to the one they were
    // registered in, and the next pass settles the sequence.
    escReports();
    imuReports();
    sched.run();

    try std.testing.expectEqual(@as(?bool, true), App.boot_state.outcome);
    try std.testing.expectEqual(@as(u32, 1), App.boot_state.attempts);
}

test "boot retries the whole gather when a branch fails" {
    resetAll();

    var sched = App.Scheduler.init();
    sched.run(); // attempt 1

    // One branch fails, the other answers. The join is failed, so the sequence
    // retries rather than half-succeeding.
    imuFails();
    escReports();
    sched.run();

    try std.testing.expectEqual(@as(u32, 2), App.boot_state.attempts);
    try std.testing.expect(App.boot_state.outcome == null);
    try std.testing.expectEqual(@as(u32, 2), FakeIo.started_esc);

    // Second attempt succeeds.
    imuReports();
    escReports();
    sched.run();

    try std.testing.expectEqual(@as(?bool, true), App.boot_state.outcome);
    try std.testing.expectEqual(@as(u32, 2), App.boot_state.attempts);
}

test "boot times out a branch that never answers, then gives up" {
    resetAll();

    var sched = App.Scheduler.init();

    // Nothing ever reports. Each attempt lasts timeout_ms (250), and after
    // attempts_max (3) the sequence settles as a failure.
    host.HostHal.runFor(&sched, 800);

    try std.testing.expectEqual(@as(u32, 3), App.boot_state.attempts);
    try std.testing.expectEqual(@as(?bool, false), App.boot_state.outcome);
}

test "a settled boot sequence does not restart on later passes" {
    resetAll();

    var sched = App.Scheduler.init();
    sched.run();
    imuReports();
    escReports();
    sched.run();
    try std.testing.expectEqual(@as(?bool, true), App.boot_state.outcome);

    // `Program` resets to instruction 0 when it finishes, and this task is
    // polled on every pass: without the outcome latch it would start the whole
    // sequence again, forever. This is the regression test for exactly that.
    const attempts = App.boot_state.attempts;
    const starts = FakeIo.started_esc;
    host.HostHal.runFor(&sched, 1000);

    try std.testing.expectEqual(attempts, App.boot_state.attempts);
    try std.testing.expectEqual(starts, FakeIo.started_esc);
    try std.testing.expectEqual(@as(?bool, true), App.boot_state.outcome);
}

// --- what the whole thing costs ---------------------------------------------

test "the application runs its modules at their declared rates, with no lateness" {
    resetAll();

    var sched = App.Scheduler.init();
    host.HostHal.runFor(&sched, 1000);

    try std.testing.expectEqual(@as(u32, 200), App.imu_state.samples); // 5 ms
    try std.testing.expectEqual(@as(u32, 100), FakeIo.encoder_frames); // 10 ms
    try std.testing.expectEqual(@as(u32, 0), sched.worstLateness());
    try std.testing.expectEqual(@as(u32, 0), sched.totalResyncs());
}

test "describe renders the graph the firmware would print" {
    resetAll();

    var buf: [1024]u8 = undefined;
    var writer = BufWriter{ .buf = &buf };
    try App.describe(writer.writer());
    const text = writer.written();

    try std.testing.expect(std.mem.indexOf(u8, text, "application: 4 module(s)") != null);
    try std.testing.expect(std.mem.indexOf(u8, text, "[1] imu0 (Imu)  period=5ms") != null);
    try std.testing.expect(std.mem.indexOf(u8, text, "depends: imu") != null);
    try std.testing.expect(std.mem.indexOf(u8, text, "hardware: i2c0") != null);
    try std.testing.expect(std.mem.indexOf(u8, text, "publishes: attitude") != null);
    // The uplink *provides* the telemetry service and *publishes* health on it.
    try std.testing.expect(std.mem.indexOf(u8, text, "provides: telemetry") != null);
    try std.testing.expect(std.mem.indexOf(u8, text, "publishes: health") != null);
}

const BufWriter = struct {
    buf: []u8,
    len: usize = 0,

    const Error = error{NoSpaceLeft};

    fn write(self: *BufWriter, bytes: []const u8) Error!usize {
        if (self.len + bytes.len > self.buf.len) return error.NoSpaceLeft;
        @memcpy(self.buf[self.len..][0..bytes.len], bytes);
        self.len += bytes.len;
        return bytes.len;
    }

    fn written(self: *const BufWriter) []const u8 {
        return self.buf[0..self.len];
    }

    const Writer = struct {
        inner: *BufWriter,

        pub fn print(self: *Writer, comptime fmt: []const u8, args: anytype) !void {
            var scratch: [192]u8 = undefined;
            const text = try std.fmt.bufPrint(&scratch, fmt, args);
            _ = try self.inner.write(text);
        }

        pub fn writeAll(self: *Writer, bytes: []const u8) !void {
            _ = try self.inner.write(bytes);
        }
    };

    fn writer(self: *BufWriter) Writer {
        return .{ .inner = self };
    }
};
