//! Chassis: encoders in, motor duty out. A Breeze module.
//!
//! Generic over `Io` for the same reason as every other module here - see
//! `imu.zig` for the argument. This one is the clearest case for it: the
//! closed-loop behaviour is the part worth testing, and it can be tested
//! exactly by handing it an I/O surface that returns a fixed encoder count.

const std = @import("std");
const breeze = @import("breeze");
const topics = @import("../topics.zig");

const Tick = breeze.Tick;
const Step = breeze.Step;

/// Build the module over an I/O surface.
///
/// `Io` must provide `readEncoder(id: u8) i32`, `setMotor(id: u8, duty: i16)`
/// and `publish(comptime T: type, value: *const T.Value, now: Tick) bool`.
pub fn Chassis(comptime Io: type) type {
    return struct {
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
            drops: u32 = 0,
        };

        /// Left and right, named once. The board is told which index means
        /// which side; nothing else in the application repeats it.
        pub const LEFT = 0;
        pub const RIGHT = 1;

        pub fn init(self: *State, cfg: Config) void {
            self.cfg = cfg;
            self.left = 0;
            self.right = 0;
            self.target_left = 0;
            self.target_right = 0;
            self.drops = 0;
        }

        pub fn poll(self: *State, now: Tick, events: u32) Step {
            _ = events;

            // Four-sided quadrature decoding happens in hardware, so reading an
            // encoder is one register read.
            self.left = Io.readEncoder(LEFT);
            self.right = Io.readEncoder(RIGHT);

            setMotor(self, LEFT, self.target_left);
            setMotor(self, RIGHT, self.target_right);

            const sample = topics.Encoders.Value{
                .left = self.left,
                .right = self.right,
                .tick_ms = now,
            };
            if (!Io.publish(topics.Encoders, &sample, now)) self.drops += 1;
            return .finished;
        }

        fn setMotor(self: *State, id: u8, duty: i16) void {
            const limit: i16 = @intCast(self.cfg.max_duty);
            Io.setMotor(id, std.math.clamp(duty, -limit, limit));
        }
    };
}
