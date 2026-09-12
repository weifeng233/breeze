//! IMU sampling, as a Breeze module.
//!
//! # Why this is generic over `Io`
//!
//! A module that named `i2c0` and a register address directly would be a
//! board file wearing a manifest: it could only ever run on one board, and it
//! could only ever be exercised on that board. Taking the I/O surface as a
//! comptime type parameter is the same device the kernel uses for its HAL
//! (`Scheduler(Hal, ...)`) - no vtable, no indirect call, `Io.readImu()`
//! resolves to a direct call - and it is what lets `app_test.zig` run this
//! exact file on a workstation.
//!
//! The manifest says `hardware = &.{"i2c0"}`: it needs *an* I2C bus and the
//! application must declare one. Which pins that bus is on is the board's
//! business, not this file's.

const breeze = @import("breeze");
const topics = @import("../topics.zig");

const Tick = breeze.Tick;
const Step = breeze.Step;

/// One raw reading. The board converts whatever the part reports into these
/// units; the module does not know the part.
///
/// Declared at file scope rather than inside the generic struct so that a board
/// can name it: `BoardIo.readImu()` returns one of these, and the board cannot
/// instantiate the module over itself to find the type.
pub const Sample = struct {
    roll: f32,
    pitch: f32,
    yaw: f32,
};

/// Build the module over an I/O surface.
///
/// `Io` must provide `readImu() ?Sample` and
/// `publish(comptime T: type, value: *const T.Value, now: Tick) bool`.
pub fn Imu(comptime Io: type) type {
    return struct {
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
            attitude: topics.Attitude.Value = .{ .roll = 0, .pitch = 0, .yaw = 0 },
            samples: u32 = 0,
            /// Frames the board refused to queue. A dropped telemetry frame is
            /// a diagnosable event, not something to block on.
            drops: u32 = 0,
        };

        pub fn init(self: *State, cfg: Config) void {
            self.cfg = cfg;
            self.attitude = .{ .roll = 0, .pitch = 0, .yaw = 0 };
            self.samples = 0;
            self.drops = 0;
        }

        pub fn poll(self: *State, now: Tick, events: u32) Step {
            _ = events;

            const raw = Io.readImu() orelse return .finished;

            // The filter is the interesting part; everything else here is the
            // module harness around it.
            self.attitude.roll += self.cfg.alpha * (raw.roll - self.attitude.roll);
            self.attitude.pitch += self.cfg.alpha * (raw.pitch - self.attitude.pitch);
            self.attitude.yaw = raw.yaw - self.cfg.yaw_offset;
            self.samples += 1;

            if (!Io.publish(topics.Attitude, &self.attitude, now)) self.drops += 1;
            return .finished;
        }
    };
}
