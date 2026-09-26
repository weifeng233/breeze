//! Ackermann steering platform, ported from
//! `include/breeze/control/platform/ackermann_steering.h`.
//!
//! It shares the shape of the other platform controllers (a comptime platform
//! type, PID objects by value, the kinematics as a pure function), but three
//! things in this file are unusual enough to name up front. All three are
//! recorded rather than corrected, and all three are pinned by the corpus:
//!
//! * **`wheelAngles` loses the turn direction.** Its comment says it keeps the
//!   sign of the input; above the small-angle branch it does not. For a right
//!   turn the radius comes out negative, `atan` therefore returns negative
//!   angles, and the closing `if (steering_angle < 0) negate` flips them back to
//!   positive - so `wheelAngles(-a)` equals `wheelAngles(a)` for `|a| >= 0.01`,
//!   while for `|a| < 0.01` the early return hands the input straight through and
//!   the sign survives. The function is discontinuous in how it treats sign, and
//!   the corpus records both sides of that boundary.
//! * **The steering PID is never computed.** `update` writes its setpoint and
//!   then uses a plain feedforward division for the steering output, so its gains
//!   and integral have no effect on anything - `setSteeringPidParams` sets a knob
//!   that turns nothing. The corpus records the integral still being zero after
//!   an update.
//! * **`steering_ratio` is not carried.** The C config has the field and nothing
//!   reads it, so it has no place here: a parameter that cannot affect anything
//!   is a parameter that invites a caller to believe it did.

const std = @import("std");

const corpus_mod = @import("../math/corpus.zig");
const pid_mod = @import("pid.zig");
const platform_mod = @import("platform.zig");

pub const Pid = pid_mod.Pid;
const requirePlatform = platform_mod.requirePlatform;

/// Inner and outer front-wheel steering angles, in radians.
pub const WheelAngles = struct {
    inner: f32,
    outer: f32,
};

pub fn AckermannSteering(comptime Platform: type) type {
    requirePlatform(Platform);

    return struct {
        /// Front-to-rear axle distance, in metres.
        wheelbase: f32,
        /// Left-to-right wheel spacing, in metres.
        track_width: f32,
        wheel_radius: f32,
        max_speed: f32,
        max_steering_angle: f32,

        drive_motor_id: i32,
        steering_motor_id: i32,
        encoder_id: i32,
        encoder_resolution: f32,

        speed_pid: Pid,
        /// Kept because the C struct has it and `update` writes its setpoint;
        /// nothing ever computes from it (see the module comment).
        steering_pid: Pid,

        target_speed: f32 = 0,
        target_steering_angle: f32 = 0,
        dt: f32,

        const Self = @This();

        pub fn init(
            wheelbase: f32,
            track_width: f32,
            wheel_radius: f32,
            max_speed: f32,
            max_steering_angle: f32,
            drive_motor_id: i32,
            steering_motor_id: i32,
            encoder_id: i32,
            encoder_resolution: f32,
            dt: f32,
        ) Self {
            return .{
                .wheelbase = wheelbase,
                .track_width = track_width,
                .wheel_radius = wheel_radius,
                .max_speed = max_speed,
                .max_steering_angle = max_steering_angle,
                .drive_motor_id = drive_motor_id,
                .steering_motor_id = steering_motor_id,
                .encoder_id = encoder_id,
                .encoder_resolution = encoder_resolution,
                .speed_pid = Pid.init(.position, 1.0, 0.1, 0.05, dt, -1.0, 1.0),
                .steering_pid = Pid.init(.position, 1.0, 0.1, 0.05, dt, -1.0, 1.0),
                .dt = dt,
            };
        }

        pub fn setSpeedPidParams(self: *Self, kp: f32, ki: f32, kd: f32) void {
            self.speed_pid.kp = kp;
            self.speed_pid.ki = ki;
            self.speed_pid.kd = kd;
        }

        /// Sets gains on a PID that `update` never computes; see the module
        /// comment. Kept because the C API has it.
        pub fn setSteeringPidParams(self: *Self, kp: f32, ki: f32, kd: f32) void {
            self.steering_pid.kp = kp;
            self.steering_pid.ki = ki;
            self.steering_pid.kd = kd;
        }

        /// Speed and steering are clamped separately, each to its own maximum -
        /// there is no radial clamp here, unlike the omni and mecanum platforms.
        pub fn setTargets(self: *Self, speed_in: f32, steering_angle_in: f32) void {
            var speed = speed_in;
            var steering_angle = steering_angle_in;

            if (speed > self.max_speed) {
                speed = self.max_speed;
            } else if (speed < -self.max_speed) {
                speed = -self.max_speed;
            }

            if (steering_angle > self.max_steering_angle) {
                steering_angle = self.max_steering_angle;
            } else if (steering_angle < -self.max_steering_angle) {
                steering_angle = -self.max_steering_angle;
            }

            self.target_speed = speed;
            self.target_steering_angle = steering_angle;
        }

        pub fn encoderToSpeed(self: Self, encoder_counts: f32) f32 {
            const wheel_circumference = 2.0 * 3.14159 * self.wheel_radius;
            const wheel_revolutions = encoder_counts / self.encoder_resolution;
            return wheel_revolutions * wheel_circumference / self.dt;
        }

        /// The Ackermann geometry: the inner wheel steers further than the outer
        /// one so both describe the same turning circle.
        ///
        /// Note the sign behaviour described in the module comment - the small
        /// angle branch preserves the input's sign and everything above it does
        /// not.
        pub fn wheelAngles(self: Self, steering_angle: f32) WheelAngles {
            if (@abs(steering_angle) < 0.01) {
                return .{ .inner = steering_angle, .outer = steering_angle };
            }

            const turning_radius = self.wheelbase / @tan(steering_angle);

            var inner_radius: f32 = undefined;
            var outer_radius: f32 = undefined;
            if (steering_angle > 0) {
                inner_radius = turning_radius - self.track_width / 2.0;
                outer_radius = turning_radius + self.track_width / 2.0;
            } else {
                inner_radius = turning_radius + self.track_width / 2.0;
                outer_radius = turning_radius - self.track_width / 2.0;
            }

            var inner = std.math.atan(self.wheelbase / inner_radius);
            var outer = std.math.atan(self.wheelbase / outer_radius);

            if (steering_angle < 0) {
                inner = -inner;
                outer = -outer;
            }

            return .{ .inner = inner, .outer = outer };
        }

        /// One control pass. The speed loop is closed; the steering output is
        /// `target / max` with no feedback at all, which is what the C version
        /// does (and says it does).
        ///
        /// The C version also computes the wheel angles here and discards them
        /// ("for reference only, not used in this simplified control"). The port
        /// does not, because a computation with no effect is not worth keeping
        /// alive - `wheelAngles` is a function any caller can use.
        pub fn update(self: *Self) void {
            const counts = Platform.readEncoder(self.encoder_id, true);
            const current_speed = self.encoderToSpeed(counts);

            self.speed_pid.setSetpoint(self.target_speed);
            self.steering_pid.setSetpoint(self.target_steering_angle);

            const speed_output = self.speed_pid.compute(current_speed);
            const steering_output = self.target_steering_angle / self.max_steering_angle;

            Platform.setMotor(self.drive_motor_id, speed_output);
            Platform.setMotor(self.steering_motor_id, steering_output);
        }
    };
}

// --- tests ------------------------------------------------------------------

const test_platform = @import("test_platform.zig");
const RecordingPlatform = test_platform.Recording;

fn testAckermann() AckermannSteering(RecordingPlatform) {
    return AckermannSteering(RecordingPlatform).init(
        0.3,
        0.2,
        0.03,
        1.5,
        0.5,
        1,
        2,
        1,
        1000.0,
        0.01,
    );
}

test "ackermann: construction and clamps match the C answers" {
    const corpus = try corpus_mod.Corpus.load();

    var ak = testAckermann();
    try corpus.expectValue("ackermann_speed_pid_kp", ak.speed_pid.kp);
    try corpus.expectValue("ackermann_speed_pid_output_max", ak.speed_pid.output_max);
    try corpus.expectValue("ackermann_steering_pid_kp", ak.steering_pid.kp);
    try corpus.expectValue("ackermann_target_speed_init", ak.target_speed);
    try corpus.expectValue("ackermann_encoder_to_speed", ak.encoderToSpeed(1000.0));

    ak.setTargets(0.5, 0.2);
    try corpus.expectValue("ackermann_target_speed", ak.target_speed);
    try corpus.expectValue("ackermann_target_steering", ak.target_steering_angle);

    // Each clamped against its own maximum, independently.
    ak.setTargets(9.0, -9.0);
    try corpus.expectValue("ackermann_clamped_speed", ak.target_speed);
    try corpus.expectValue("ackermann_clamped_steering", ak.target_steering_angle);
    try std.testing.expectEqual(@as(f32, 1.5), ak.target_speed);
    try std.testing.expectEqual(@as(f32, -0.5), ak.target_steering_angle);
}

test "ackermann: the wheel angles lose the turn direction above the small-angle branch" {
    const corpus = try corpus_mod.Corpus.load();

    const ak = testAckermann();

    const left = ak.wheelAngles(0.2);
    const right = ak.wheelAngles(-0.2);
    try corpus.expectValue("ackermann_angles_left_inner", left.inner);
    try corpus.expectValue("ackermann_angles_left_outer", left.outer);
    try corpus.expectValue("ackermann_angles_right_inner", right.inner);
    try corpus.expectValue("ackermann_angles_right_outer", right.outer);

    // The defect, asserted: turning left and turning right give the *same*
    // wheel angles, so the geometry cannot tell a driver which way to steer.
    try std.testing.expectEqual(left.inner, right.inner);
    try std.testing.expectEqual(left.outer, right.outer);

    // The inner wheel steers further than the outer one, which is the part of
    // the geometry that does work.
    try std.testing.expect(left.inner > left.outer);

    // Below the threshold the sign survives, because the early return passes the
    // input through: the two branches disagree about what a sign means.
    const small_left = ak.wheelAngles(0.005);
    const small_right = ak.wheelAngles(-0.005);
    try corpus.expectValue("ackermann_angles_small_pos_inner", small_left.inner);
    try corpus.expectValue("ackermann_angles_small_pos_outer", small_left.outer);
    try corpus.expectValue("ackermann_angles_small_neg_inner", small_right.inner);
    try corpus.expectValue("ackermann_angles_small_neg_outer", small_right.outer);

    try std.testing.expectEqual(small_left.inner, -small_right.inner);
    try std.testing.expectEqual(@as(f32, 0.0), ak.wheelAngles(0.0).inner);
    try corpus.expectValue("ackermann_angles_zero_inner", 0.0);
}

test "ackermann: steering is feedforward and the steering PID is never computed" {
    const corpus = try corpus_mod.Corpus.load();

    var ak = testAckermann();
    ak.setTargets(0.4, 0.3);
    RecordingPlatform.clearLog();
    ak.update();

    try corpus.expectValue("ackermann_speed_setpoint", ak.speed_pid.setpoint);
    try corpus.expectValue("ackermann_steering_setpoint", ak.steering_pid.setpoint);

    // The setpoint is written and then never consumed: the PID's integral is
    // still zero, and the gains could be anything.
    try corpus.expectValue("ackermann_steering_integral_after", ak.steering_pid.integral);
    try std.testing.expectEqual(@as(f32, 0.0), ak.steering_pid.integral);

    try corpus.expectValues("ackermann_update_motors", &.{
        RecordingPlatform.log[0].speed, RecordingPlatform.log[1].speed,
    });
    try corpus.expectInt("ackermann_drive_motor_id", RecordingPlatform.log[0].motor_id);
    try corpus.expectInt("ackermann_steering_motor_id", RecordingPlatform.log[1].motor_id);

    // Whatever the steering gains are, the output does not change - which is
    // what makes `setSteeringPidParams` a knob that turns nothing.
    ak.setSteeringPidParams(100.0, 100.0, 100.0);
    RecordingPlatform.startPass(0);
    ak.update();
    try std.testing.expectEqual(
        ak.target_steering_angle / ak.max_steering_angle,
        RecordingPlatform.log[1].speed,
    );
}

test "ackermann: a run matches the C answers, and the steering output is constant" {
    const corpus = try corpus_mod.Corpus.load();

    var ak = testAckermann();
    ak.setTargets(0.4, 0.3);

    var drive: [3]f32 = undefined;
    var steer: [3]f32 = undefined;
    for (0..3) |i| {
        RecordingPlatform.startPass(i);
        ak.update();
        drive[i] = RecordingPlatform.log[0].speed;
        steer[i] = RecordingPlatform.log[1].speed;
        try std.testing.expect(RecordingPlatform.allResets());
    }
    try corpus.expectValues("ackermann_run_drive", &drive);
    try corpus.expectValues("ackermann_run_steering", &steer);

    // The steering channel is a constant: with no feedback there is nothing to
    // vary, however the speed loop moves.
    for (steer) |s| try std.testing.expectEqual(steer[0], s);

    // Independent of the corpus: the feedforward ratio is the target over the
    // maximum, and it saturates at ±1.
    var saturated = testAckermann();
    saturated.setTargets(0.0, 0.5);
    try std.testing.expectEqual(@as(f32, 1.0), saturated.target_steering_angle / saturated.max_steering_angle);
}
