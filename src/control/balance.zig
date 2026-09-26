//! Balancing platform, ported from
//! `include/breeze/control/platform/balance_controller.h`.
//!
//! A cascaded controller: the speed loop's output becomes the angle loop's
//! setpoint, the angle loop's output is the base motor power, and a turn loop
//! adds a differential. The tilt estimate comes from the complementary filter in
//! the filter stage - the first place where one migrated stage uses another.
//!
//! **Two failures that the C version reports as the same number.** `Update`
//! returns `0` both when the IMU read fails and when the tilt estimate exceeds
//! the safety limit - different situations with different responses (one commands
//! nothing, the other actively zeroes the motors). The port answers with
//! `error.ImuReadFailed` and `error.Tilted`, which a caller cannot conflate. The
//! corpus records both C outcomes, so the difference is documented rather than
//! invented.
//!
//! The platform surface grows by one function here: `readImu`, returning
//! `?ImuData` rather than an `int` success flag, so the failure has to be handled
//! before the data can be read.

const std = @import("std");

const corpus_mod = @import("../math/corpus.zig");
const complementary = @import("../filter/complementary.zig");
const pid_mod = @import("pid.zig");
const platform_mod = @import("platform.zig");

pub const Pid = pid_mod.Pid;
pub const ImuData = platform_mod.ImuData;
const requirePlatform = platform_mod.requirePlatform;

/// Adds the IMU to the usual platform surface.
fn requireBalancePlatform(comptime Platform: type) void {
    requirePlatform(Platform);
    if (!@hasDecl(Platform, "readImu")) {
        @compileError("platform '" ++ @typeName(Platform) ++ "' must declare readImu() ?ImuData for the balance controller");
    }
}

/// Why an update did not produce a motor command.
pub const Error = error{
    /// The platform could not supply an IMU sample.
    ImuReadFailed,
    /// The tilt estimate passed `max_tilt_angle`; the motors were zeroed.
    Tilted,
};

pub fn BalanceController(comptime Platform: type) type {
    requireBalancePlatform(Platform);

    return struct {
        wheel_radius: f32,
        wheel_distance: f32,
        max_tilt_angle: f32,
        max_speed: f32,
        max_angular_speed: f32,
        /// Tilt the angle loop aims for, normally zero.
        target_tilt_angle: f32,

        left_motor_id: i32,
        right_motor_id: i32,
        left_encoder_id: i32,
        right_encoder_id: i32,
        encoder_resolution: f32,

        angle_pid: Pid,
        speed_pid: Pid,
        turn_pid: Pid,
        imu_filter: complementary.Complementary,

        target_speed: f32 = 0,
        target_turn_rate: f32 = 0,
        current_speed: f32 = 0,
        current_angle: f32 = 0,
        dt: f32,

        const Self = @This();

        pub fn init(
            wheel_radius: f32,
            wheel_distance: f32,
            max_tilt_angle: f32,
            max_speed: f32,
            max_angular_speed: f32,
            target_tilt_angle: f32,
            left_motor_id: i32,
            right_motor_id: i32,
            left_encoder_id: i32,
            right_encoder_id: i32,
            encoder_resolution: f32,
            dt: f32,
        ) Self {
            return .{
                .wheel_radius = wheel_radius,
                .wheel_distance = wheel_distance,
                .max_tilt_angle = max_tilt_angle,
                .max_speed = max_speed,
                .max_angular_speed = max_angular_speed,
                .target_tilt_angle = target_tilt_angle,
                .left_motor_id = left_motor_id,
                .right_motor_id = right_motor_id,
                .left_encoder_id = left_encoder_id,
                .right_encoder_id = right_encoder_id,
                .encoder_resolution = encoder_resolution,
                // The three loops get very different gains in the C `Init`: the
                // angle loop is fast and stiff, the speed loop slow, the turn
                // loop proportional only.
                .angle_pid = Pid.init(.position, 10.0, 0.0, 0.1, dt, -1.0, 1.0),
                .speed_pid = Pid.init(.position, 0.5, 0.05, 0.0, dt, -0.5, 0.5),
                .turn_pid = Pid.init(.position, 1.0, 0.0, 0.0, dt, -0.5, 0.5),
                .imu_filter = complementary.Complementary.init(0.98, dt),
                .dt = dt,
            };
        }

        pub fn setAnglePidParams(self: *Self, kp: f32, ki: f32, kd: f32) void {
            self.angle_pid.kp = kp;
            self.angle_pid.ki = ki;
            self.angle_pid.kd = kd;
        }

        pub fn setSpeedPidParams(self: *Self, kp: f32, ki: f32, kd: f32) void {
            self.speed_pid.kp = kp;
            self.speed_pid.ki = ki;
            self.speed_pid.kd = kd;
        }

        pub fn setTurnPidParams(self: *Self, kp: f32, ki: f32, kd: f32) void {
            self.turn_pid.kp = kp;
            self.turn_pid.ki = ki;
            self.turn_pid.kd = kd;
        }

        pub fn setTargets(self: *Self, speed_in: f32, turn_rate_in: f32) void {
            var speed = speed_in;
            var turn_rate = turn_rate_in;

            if (speed > self.max_speed) {
                speed = self.max_speed;
            } else if (speed < -self.max_speed) {
                speed = -self.max_speed;
            }

            if (turn_rate > self.max_angular_speed) {
                turn_rate = self.max_angular_speed;
            } else if (turn_rate < -self.max_angular_speed) {
                turn_rate = -self.max_angular_speed;
            }

            self.target_speed = speed;
            self.target_turn_rate = turn_rate;
        }

        pub fn encoderToSpeed(self: Self, encoder_counts: f32) f32 {
            const wheel_circumference = 2.0 * 3.14159 * self.wheel_radius;
            const wheel_revolutions = encoder_counts / self.encoder_resolution;
            return wheel_revolutions * wheel_circumference / self.dt;
        }

        /// One control pass.
        ///
        /// On `error.Tilted` both motors have been commanded to zero, exactly as
        /// the C version does; on `error.ImuReadFailed` no motor command has been
        /// issued at all, which is also what the C version does - and is why the
        /// two cases are worth distinguishing.
        pub fn update(self: *Self) Error!void {
            const imu = Platform.readImu() orelse return Error.ImuReadFailed;

            self.imu_filter.update(
                imu.gyro_x,
                imu.gyro_y,
                imu.accel_x,
                imu.accel_y,
                imu.accel_z,
            );

            // The balance axis is the estimated pitch.
            self.current_angle = self.imu_filter.getPitch();

            if (@abs(self.current_angle) > self.max_tilt_angle) {
                Platform.setMotor(self.left_motor_id, 0.0);
                Platform.setMotor(self.right_motor_id, 0.0);
                return Error.Tilted;
            }

            const left_counts = Platform.readEncoder(self.left_encoder_id, true);
            const right_counts = Platform.readEncoder(self.right_encoder_id, true);

            const left_speed = self.encoderToSpeed(left_counts);
            const right_speed = self.encoderToSpeed(right_counts);

            self.current_speed = (left_speed + right_speed) / 2.0;

            // Cascaded: speed asks for a tilt, tilt asks for motor power.
            self.speed_pid.setSetpoint(self.target_speed);
            const speed_output = self.speed_pid.compute(self.current_speed);

            self.angle_pid.setSetpoint(self.target_tilt_angle + speed_output);
            const angle_output = self.angle_pid.compute(self.current_angle);

            self.turn_pid.setSetpoint(self.target_turn_rate);
            const turn_output = self.turn_pid.compute((right_speed - left_speed) / self.wheel_distance);

            var left_output = angle_output - turn_output;
            var right_output = angle_output + turn_output;

            if (left_output > 1.0) left_output = 1.0;
            if (left_output < -1.0) left_output = -1.0;
            if (right_output > 1.0) right_output = 1.0;
            if (right_output < -1.0) right_output = -1.0;

            Platform.setMotor(self.left_motor_id, left_output);
            Platform.setMotor(self.right_motor_id, right_output);
        }
    };
}

// --- tests ------------------------------------------------------------------

const test_platform = @import("test_platform.zig");
const RecordingPlatform = test_platform.Recording;

fn testBalance() BalanceController(RecordingPlatform) {
    return BalanceController(RecordingPlatform).init(
        0.03,
        0.15,
        0.5,
        1.0,
        3.0,
        0.0,
        1,
        2,
        1,
        2,
        1000.0,
        0.01,
    );
}

test "balance: construction and clamps match the C answers" {
    const corpus = try corpus_mod.Corpus.load();

    var b = testBalance();
    try corpus.expectValue("balance_angle_kp", b.angle_pid.kp);
    try corpus.expectValue("balance_angle_kd", b.angle_pid.kd);
    try corpus.expectValue("balance_angle_output_max", b.angle_pid.output_max);
    try corpus.expectValue("balance_speed_kp", b.speed_pid.kp);
    try corpus.expectValue("balance_speed_output_max", b.speed_pid.output_max);
    try corpus.expectValue("balance_turn_kp", b.turn_pid.kp);
    try corpus.expectValue("balance_filter_alpha", b.imu_filter.alpha);
    try corpus.expectValue("balance_filter_dt", b.imu_filter.dt);
    try corpus.expectValue("balance_current_angle_init", b.current_angle);
    try corpus.expectValue("balance_encoder_to_speed", b.encoderToSpeed(1000.0));

    b.setTargets(0.5, 0.2);
    try corpus.expectValue("balance_target_speed", b.target_speed);
    try corpus.expectValue("balance_target_turn", b.target_turn_rate);

    b.setTargets(9.0, -9.0);
    try corpus.expectValue("balance_clamped_speed", b.target_speed);
    try corpus.expectValue("balance_clamped_turn", b.target_turn_rate);
}

test "balance: a run on the mild script matches the C answers" {
    const corpus = try corpus_mod.Corpus.load();

    var b = testBalance();
    b.setTargets(0.3, 0.1);

    var ok: [3]f32 = undefined;
    var left: [3]f32 = undefined;
    var right: [3]f32 = undefined;
    for (0..3) |i| {
        RecordingPlatform.startPass(i);
        b.update() catch |err| {
            // The mild script stays well inside the limits, so an error here
            // would itself be the finding.
            std.debug.print("unexpected {s} on pass {d}\n", .{ @errorName(err), i });
            return err;
        };
        ok[i] = 1;
        left[i] = RecordingPlatform.log[0].speed;
        right[i] = RecordingPlatform.log[1].speed;
    }

    try corpus.expectValues("balance_run_ok", &ok);
    try corpus.expectValues("balance_run_left", &left);
    try corpus.expectValues("balance_run_right", &right);
    try corpus.expectValue("balance_current_angle_after", b.current_angle);
    try corpus.expectValue("balance_current_speed_after", b.current_speed);

    // The estimate stays small, which is what makes this the "balanced" case.
    try std.testing.expect(@abs(b.current_angle) < b.max_tilt_angle);
}

test "balance: an IMU that fails produces no motor command at all" {
    const corpus = try corpus_mod.Corpus.load();

    var b = testBalance();
    RecordingPlatform.imu_mode = .failing;
    RecordingPlatform.startPass(0);

    try corpus.expectInt("balance_imu_failure_ok", 0);
    try std.testing.expectError(Error.ImuReadFailed, b.update());

    // The distinction the C `int` return cannot make: nothing was commanded.
    try corpus.expectInt("balance_imu_failure_motor_calls", RecordingPlatform.log_len);
    try std.testing.expectEqual(@as(usize, 0), RecordingPlatform.log_len);

    RecordingPlatform.imu_mode = .mild;
}

test "balance: past the tilt limit the motors are zeroed and the error says why" {
    const corpus = try corpus_mod.Corpus.load();

    var b = testBalance();
    RecordingPlatform.imu_mode = .tilting;

    var ok: [5]f32 = undefined;
    var left: [5]f32 = undefined;
    var right: [5]f32 = undefined;
    for (0..5) |i| {
        RecordingPlatform.startPass(i);
        if (b.update()) |_| {
            ok[i] = 1;
        } else |err| {
            try std.testing.expectEqual(Error.Tilted, err);
            ok[i] = 0;
        }
        left[i] = RecordingPlatform.log[0].speed;
        right[i] = RecordingPlatform.log[1].speed;
    }

    try corpus.expectValues("balance_tilt_ok", &ok);
    try corpus.expectValues("balance_tilt_left", &left);
    try corpus.expectValues("balance_tilt_right", &right);
    try corpus.expectValue("balance_tilt_angle_after", b.current_angle);

    // The first four passes are still balanced and the fifth is not, which is
    // what makes this case about the limit rather than about the script.
    try std.testing.expectEqual(@as(f32, 1.0), ok[0]);
    try std.testing.expectEqual(@as(f32, 0.0), ok[4]);

    // Tilted means both motors were commanded to zero - not "nothing was
    // commanded", which is the IMU failure above.
    try std.testing.expectEqual(@as(f32, 0.0), left[4]);
    try std.testing.expectEqual(@as(f32, 0.0), right[4]);

    RecordingPlatform.imu_mode = .mild;
}
