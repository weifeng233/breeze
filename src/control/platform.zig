//! Mobile platform controllers, ported from `include/breeze/control/platform/`.
//!
//! In progress: the differential drive is done; ackermann, mecanum, omni and
//! balance are not yet.
//!
//! **The I/O surface is a type parameter, not a function pointer.** The C headers
//! store `BreezeMotorControlFunc` and `BreezeEncoderFunc` in the controller
//! struct, so a platform is chosen at run time and every call goes through a
//! pointer that can be null (which is why every C entry point null-checks it).
//! Here the platform is a comptime type whose functions are called directly -
//! the same shape the kernel uses for its HAL and the example application for its
//! modules. Two consequences, and the trade is deliberate:
//!
//! * a missing function is a compile error that names what is missing (see
//!   `requirePlatform`), rather than a call through a null pointer;
//! * a controller cannot be handed a different platform at run time, which is
//!   exactly the flexibility it never used - the C examples pass one platform in
//!   `Init` and never change it.
//!
//! Hardware numbers are copied as written, including `2 * 3.14159 * radius`:
//! the C code does not use `M_PI`, and `std.math.pi` would change the result in
//! the fifth decimal. The corpus would catch it, which is the point of having it.

const std = @import("std");

const corpus_mod = @import("../math/corpus.zig");
const pid_mod = @import("pid.zig");

pub const Pid = pid_mod.Pid;

/// What a platform must provide for the controllers here.
///
/// Checked by `requirePlatform` so that a missing function is reported by name.
pub const platform_doc =
    \\  pub fn setMotor(motor_id: i32, speed: f32) void
    \\  pub fn readEncoder(encoder_id: i32, reset: bool) f32
;

fn requirePlatform(comptime Platform: type) void {
    if (!@hasDecl(Platform, "setMotor")) {
        @compileError("platform '" ++ @typeName(Platform) ++ "' must declare setMotor(motor_id: i32, speed: f32) void");
    }
    if (!@hasDecl(Platform, "readEncoder")) {
        @compileError("platform '" ++ @typeName(Platform) ++ "' must declare readEncoder(encoder_id: i32, reset: bool) f32");
    }
}

/// IMU sample, as the C HAL defines it.
pub const ImuData = struct {
    gyro_x: f32,
    gyro_y: f32,
    gyro_z: f32,
    accel_x: f32,
    accel_y: f32,
    accel_z: f32,
};

pub const DifferentialConfig = struct {
    /// Distance between the left and right wheels, in metres.
    wheel_distance: f32,
    wheel_radius: f32,
    max_linear_speed: f32,
    max_angular_speed: f32,
    left_motor_id: i32,
    right_motor_id: i32,
    left_encoder_id: i32,
    right_encoder_id: i32,
    /// Encoder counts per wheel revolution.
    encoder_resolution: f32,
};

/// A two-wheeled differential (tank) drive.
pub fn DifferentialDrive(comptime Platform: type) type {
    requirePlatform(Platform);

    return struct {
        config: DifferentialConfig,
        left_pid: Pid,
        right_pid: Pid,
        target_linear_speed: f32 = 0,
        target_angular_speed: f32 = 0,
        dt: f32,

        const Self = @This();

        /// Both wheel PIDs start as position controllers with fixed gains and a
        /// `±1` output range, as in the C `Init`.
        pub fn init(config: DifferentialConfig, dt: f32) Self {
            return .{
                .config = config,
                .left_pid = Pid.init(.position, 1.0, 0.1, 0.05, dt, -1.0, 1.0),
                .right_pid = Pid.init(.position, 1.0, 0.1, 0.05, dt, -1.0, 1.0),
                .dt = dt,
            };
        }

        /// Applies `kp`, `ki` and `kd` to both wheels. The C version leaves the
        /// other PID settings alone and so does this.
        pub fn setPidParams(self: *Self, kp: f32, ki: f32, kd: f32) void {
            self.left_pid.kp = kp;
            self.left_pid.ki = ki;
            self.left_pid.kd = kd;
            self.right_pid.kp = kp;
            self.right_pid.ki = ki;
            self.right_pid.kd = kd;
        }

        /// Sets the target, clamping each component against its own maximum.
        pub fn setSpeed(self: *Self, linear_speed: f32, angular_speed: f32) void {
            var linear = linear_speed;
            var angular = angular_speed;

            if (linear > self.config.max_linear_speed) {
                linear = self.config.max_linear_speed;
            } else if (linear < -self.config.max_linear_speed) {
                linear = -self.config.max_linear_speed;
            }

            if (angular > self.config.max_angular_speed) {
                angular = self.config.max_angular_speed;
            } else if (angular < -self.config.max_angular_speed) {
                angular = -self.config.max_angular_speed;
            }

            self.target_linear_speed = linear;
            self.target_angular_speed = angular;
        }

        /// Encoder counts since the last read, to metres per second.
        pub fn encoderToSpeed(self: Self, encoder_counts: f32) f32 {
            const wheel_circumference = 2.0 * 3.14159 * self.config.wheel_radius;
            const wheel_revolutions = encoder_counts / self.config.encoder_resolution;
            return wheel_revolutions * wheel_circumference / self.dt;
        }

        /// One control pass: read the encoders, run both wheel PIDs, command the
        /// motors. The C version applies no deadline or rate limit here; the
        /// caller decides how often to call it (`dt` must match).
        pub fn update(self: *Self) void {
            const left_target = self.target_linear_speed -
                (self.target_angular_speed * self.config.wheel_distance / 2.0);
            const right_target = self.target_linear_speed +
                (self.target_angular_speed * self.config.wheel_distance / 2.0);

            const left_counts = Platform.readEncoder(self.config.left_encoder_id, true);
            const right_counts = Platform.readEncoder(self.config.right_encoder_id, true);

            const left_current = self.encoderToSpeed(left_counts);
            const right_current = self.encoderToSpeed(right_counts);

            self.left_pid.setSetpoint(left_target);
            self.right_pid.setSetpoint(right_target);

            const left_output = self.left_pid.compute(left_current);
            const right_output = self.right_pid.compute(right_current);

            Platform.setMotor(self.config.left_motor_id, left_output);
            Platform.setMotor(self.config.right_motor_id, right_output);
        }
    };
}

// --- tests ------------------------------------------------------------------

/// A platform that records what it was told, so the commands a run produces can
/// be compared with the C version's. The encoder counts are the same scripted
/// sequence `tools/corpus/gen_math_corpus.c` feeds its fake.
const RecordingPlatform = struct {
    pub const Call = struct { motor_id: i32, speed: f32 };

    var log: [16]Call = undefined;
    var log_len: usize = 0;
    var encoder_step: usize = 0;
    /// Every `reset` flag the controller passed, in call order. Recording only
    /// the last one hides a change to the other read - which is exactly what
    /// happened when the first version of this fake kept only the last value.
    var encoder_resets: [16]bool = undefined;
    var encoder_reset_len: usize = 0;

    const left_counts = [_]f32{ 25.0, 28.0, 24.0 };
    const right_counts = [_]f32{ 27.0, 26.0, 30.0 };

    pub fn setMotor(motor_id: i32, speed: f32) void {
        if (log_len < log.len) {
            log[log_len] = .{ .motor_id = motor_id, .speed = speed };
            log_len += 1;
        }
    }

    pub fn readEncoder(encoder_id: i32, reset: bool) f32 {
        if (encoder_reset_len < encoder_resets.len) {
            encoder_resets[encoder_reset_len] = reset;
            encoder_reset_len += 1;
        }
        const counts = if (encoder_id == 1)
            left_counts[encoder_step % 3]
        else
            right_counts[encoder_step % 3];
        // The controller reads left then right; advance after the second.
        if (encoder_id != 1) encoder_step += 1;
        return counts;
    }

    fn clearLog() void {
        log_len = 0;
        encoder_step = 0;
        encoder_reset_len = 0;
    }

    /// True only if every read so far asked for a reset.
    fn allReadsReset() bool {
        for (encoder_resets[0..encoder_reset_len]) |asked| {
            if (!asked) return false;
        }
        return encoder_reset_len > 0;
    }
};

fn testConfig() DifferentialConfig {
    return .{
        .wheel_distance = 0.15,
        .wheel_radius = 0.03,
        .max_linear_speed = 1.0,
        .max_angular_speed = 3.0,
        .left_motor_id = 1,
        .right_motor_id = 2,
        .left_encoder_id = 1,
        .right_encoder_id = 2,
        .encoder_resolution = 1000.0,
    };
}

test "differential drive: init and configuration match the C answers" {
    const corpus = try corpus_mod.Corpus.load();

    const dd = DifferentialDrive(RecordingPlatform).init(testConfig(), 0.01);
    try corpus.expectValue("diffdrive_left_kp", dd.left_pid.kp);
    try corpus.expectValue("diffdrive_left_ki", dd.left_pid.ki);
    try corpus.expectValue("diffdrive_left_kd", dd.left_pid.kd);
    try corpus.expectValue("diffdrive_left_output_min", dd.left_pid.output_min);
    try corpus.expectValue("diffdrive_left_output_max", dd.left_pid.output_max);
    try corpus.expectValue("diffdrive_right_kp", dd.right_pid.kp);
    try corpus.expectValue("diffdrive_dt", dd.dt);
    try corpus.expectValue("diffdrive_target_init", dd.target_linear_speed);
}

test "differential drive: setSpeed clamps like the C version" {
    const corpus = try corpus_mod.Corpus.load();

    var dd = DifferentialDrive(RecordingPlatform).init(testConfig(), 0.01);
    dd.setSpeed(0.5, 0.2);
    try corpus.expectValue("diffdrive_target_linear", dd.target_linear_speed);
    try corpus.expectValue("diffdrive_target_angular", dd.target_angular_speed);

    dd.setSpeed(5.0, -9.0);
    try corpus.expectValue("diffdrive_clamped_linear", dd.target_linear_speed);
    try corpus.expectValue("diffdrive_clamped_angular", dd.target_angular_speed);

    // Independent of the corpus: the clamps are per component, not shared.
    dd.setSpeed(100.0, 0.0);
    try std.testing.expectEqual(@as(f32, 1.0), dd.target_linear_speed);
    try std.testing.expectEqual(@as(f32, 0.0), dd.target_angular_speed);
}

test "differential drive: encoder counts become wheel speed like the C version" {
    const corpus = try corpus_mod.Corpus.load();

    const dd = DifferentialDrive(RecordingPlatform).init(testConfig(), 0.01);
    try corpus.expectValue("diffdrive_encoder_to_speed", dd.encoderToSpeed(1000.0));

    // One revolution is the circumference, so the mapping is linear in counts.
    try std.testing.expectApproxEqAbs(dd.encoderToSpeed(500.0) * 2.0, dd.encoderToSpeed(1000.0), 1.0e-4);
}

test "differential drive: a run commands the motors the C version commands" {
    const corpus = try corpus_mod.Corpus.load();

    var dd = DifferentialDrive(RecordingPlatform).init(testConfig(), 0.01);
    dd.setSpeed(0.5, 0.2);

    RecordingPlatform.clearLog();
    var left: [3]f32 = undefined;
    var right: [3]f32 = undefined;
    for (0..3) |i| {
        dd.update();
        // Two commands per pass, left first.
        try std.testing.expectEqual(@as(i32, 1), RecordingPlatform.log[2 * i].motor_id);
        try std.testing.expectEqual(@as(i32, 2), RecordingPlatform.log[2 * i + 1].motor_id);
        left[i] = RecordingPlatform.log[2 * i].speed;
        right[i] = RecordingPlatform.log[2 * i + 1].speed;
    }
    try corpus.expectValues("diffdrive_update_left", &left);
    try corpus.expectValues("diffdrive_update_right", &right);

    // The motor ids themselves are part of the contract.
    try corpus.expectInt("diffdrive_left_motor_id", RecordingPlatform.log[0].motor_id);
    try corpus.expectInt("diffdrive_right_motor_id", RecordingPlatform.log[1].motor_id);

    // And so is asking the encoder to reset after the read: the controller wants
    // the counts *since the last pass*, which only works if it says so.
    try corpus.expectInt("diffdrive_encoder_reset_flag", @intFromBool(RecordingPlatform.allReadsReset()));
    try std.testing.expect(RecordingPlatform.allReadsReset());

    // Independent of the corpus: a turn in place drives the wheels towards
    // opposite targets, which is the kinematic relation the controller rests on.
    // The *command* is deliberately not compared here - on the first pass the
    // derivative term's kick from a zero initial measurement dominates both PIDs
    // and saturates them, so the targets are the thing worth asserting.
    var turning = DifferentialDrive(RecordingPlatform).init(testConfig(), 0.01);
    turning.setSpeed(0.0, 2.0);
    RecordingPlatform.clearLog();
    turning.update();
    try std.testing.expectEqual(@as(f32, -0.15), turning.left_pid.setpoint);
    try std.testing.expectEqual(@as(f32, 0.15), turning.right_pid.setpoint);

    // Driving straight gives both wheels the same target.
    var straight = DifferentialDrive(RecordingPlatform).init(testConfig(), 0.01);
    straight.setSpeed(0.5, 0.0);
    straight.update();
    try std.testing.expectEqual(straight.left_pid.setpoint, straight.right_pid.setpoint);
}

test "differential drive: the tuned gains match the C answers" {
    const corpus = try corpus_mod.Corpus.load();

    var dd = DifferentialDrive(RecordingPlatform).init(testConfig(), 0.01);
    dd.setPidParams(2.0, 0.5, 0.25);
    try corpus.expectValue("diffdrive_setparams_left_kp", dd.left_pid.kp);
    try corpus.expectValue("diffdrive_setparams_right_kd", dd.right_pid.kd);

    dd.setSpeed(0.5, 0.2);
    RecordingPlatform.clearLog();
    var left: [3]f32 = undefined;
    var right: [3]f32 = undefined;
    for (0..3) |i| {
        dd.update();
        left[i] = RecordingPlatform.log[2 * i].speed;
        right[i] = RecordingPlatform.log[2 * i + 1].speed;
    }
    try corpus.expectValues("diffdrive_update_tuned_left", &left);
    try corpus.expectValues("diffdrive_update_tuned_right", &right);

    // Independent of the corpus: the wheel PIDs are clamped to +-1, so no
    // command can leave that range however large the error gets.
    for (left, right) |l, r| {
        try std.testing.expect(l >= -1.0 and l <= 1.0);
        try std.testing.expect(r >= -1.0 and r <= 1.0);
    }
}
