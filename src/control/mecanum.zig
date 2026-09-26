//! Mecanum platform, ported from
//! `include/breeze/control/platform/mecanum_drive.h`.
//!
//! Four wheels, so the wheel count is not a parameter and the arrays have no
//! unused tail. The shape of the port is the same as `omni.zig`: the inverse
//! kinematics is a pure function (`wheelTargets`) rather than a block inside
//! `update`, because that is the part worth reading and testing.
//!
//! **Its kinematics is the contrast to the omni platform's.** The rotation term
//! here changes sign between the left and right sides:
//!
//!     front-left  = vx - vy - (l_x + l_y)·ω
//!     front-right = vx + vy + (l_x + l_y)·ω
//!     rear-left   = vx + vy - (l_x + l_y)·ω
//!     rear-right  = vx - vy + (l_x + l_y)·ω
//!
//! which is what a rotation does. The omni code adds one constant to every wheel
//! instead (see `omni.zig` and REVIEW §36), and the two files sit next to each
//! other in the same directory - which is why the corpus records this one's pure
//! rotation as two values with opposite signs, right beside the omni's three
//! equal ones.

const std = @import("std");

const corpus_mod = @import("../math/corpus.zig");
const pid_mod = @import("pid.zig");
const platform_mod = @import("platform.zig");

pub const Pid = pid_mod.Pid;
const requirePlatform = platform_mod.requirePlatform;

/// Wheel positions, in the order the C enum uses. The constants exist so call
/// sites can say what they mean.
pub const Wheel = enum(usize) {
    front_left = 0,
    front_right = 1,
    rear_left = 2,
    rear_right = 3,
};

pub fn MecanumDrive(comptime Platform: type) type {
    requirePlatform(Platform);

    return struct {
        wheel_radius: f32,
        /// Front-to-rear wheel spacing, in metres.
        wheel_distance_x: f32,
        /// Left-to-right wheel spacing, in metres.
        wheel_distance_y: f32,
        max_linear_speed: f32,
        max_angular_speed: f32,
        encoder_resolution: f32,

        motor_ids: [4]i32,
        encoder_ids: [4]i32,

        wheel_pid: [4]Pid,

        target_vx: f32 = 0,
        target_vy: f32 = 0,
        target_omega: f32 = 0,
        dt: f32,

        const Self = @This();

        pub fn init(
            wheel_radius: f32,
            wheel_distance_x: f32,
            wheel_distance_y: f32,
            max_linear_speed: f32,
            max_angular_speed: f32,
            motor_ids: [4]i32,
            encoder_ids: [4]i32,
            encoder_resolution: f32,
            dt: f32,
        ) Self {
            var self = Self{
                .wheel_radius = wheel_radius,
                .wheel_distance_x = wheel_distance_x,
                .wheel_distance_y = wheel_distance_y,
                .max_linear_speed = max_linear_speed,
                .max_angular_speed = max_angular_speed,
                .encoder_resolution = encoder_resolution,
                .motor_ids = motor_ids,
                .encoder_ids = encoder_ids,
                .wheel_pid = undefined,
                .dt = dt,
            };
            for (&self.wheel_pid) |*pid| {
                pid.* = Pid.init(.position, 1.0, 0.1, 0.05, dt, -1.0, 1.0);
            }
            return self;
        }

        pub fn setPidParams(self: *Self, kp: f32, ki: f32, kd: f32) void {
            for (&self.wheel_pid) |*pid| {
                pid.kp = kp;
                pid.ki = ki;
                pid.kd = kd;
            }
        }

        /// Radial clamp on the linear part, per-component on the angular part -
        /// the same asymmetry as the omni platform.
        pub fn setVelocity(self: *Self, vx_in: f32, vy_in: f32, omega_in: f32) void {
            var vx = vx_in;
            var vy = vy_in;
            var omega = omega_in;

            const linear_speed = @sqrt(vx * vx + vy * vy);
            if (linear_speed > self.max_linear_speed and linear_speed > 0) {
                const scale = self.max_linear_speed / linear_speed;
                vx *= scale;
                vy *= scale;
            }

            if (omega > self.max_angular_speed) {
                omega = self.max_angular_speed;
            } else if (omega < -self.max_angular_speed) {
                omega = -self.max_angular_speed;
            }

            self.target_vx = vx;
            self.target_vy = vy;
            self.target_omega = omega;
        }

        pub fn encoderToSpeed(self: Self, encoder_counts: f32) f32 {
            const wheel_circumference = 2.0 * 3.14159 * self.wheel_radius;
            const wheel_revolutions = encoder_counts / self.encoder_resolution;
            return wheel_revolutions * wheel_circumference / self.dt;
        }

        /// The wheel speeds a commanded platform velocity asks for, including
        /// the "no wheel may exceed the maximum linear speed" rescale.
        pub fn wheelTargets(self: Self, vx: f32, vy: f32, omega: f32) [4]f32 {
            const l_x = self.wheel_distance_x / 2.0;
            const l_y = self.wheel_distance_y / 2.0;
            const spin = (l_x + l_y) * omega;

            var targets: [4]f32 = undefined;
            targets[@intFromEnum(Wheel.front_left)] = vx - vy - spin;
            targets[@intFromEnum(Wheel.front_right)] = vx + vy + spin;
            targets[@intFromEnum(Wheel.rear_left)] = vx + vy - spin;
            targets[@intFromEnum(Wheel.rear_right)] = vx - vy + spin;

            var max_speed: f32 = 0;
            for (targets) |t| {
                const abs = @abs(t);
                if (abs > max_speed) max_speed = abs;
            }

            if (max_speed > self.max_linear_speed and max_speed > 0) {
                const scale = self.max_linear_speed / max_speed;
                for (&targets) |*t| t.* *= scale;
            }

            return targets;
        }

        pub fn currentWheelTargets(self: Self) [4]f32 {
            return self.wheelTargets(self.target_vx, self.target_vy, self.target_omega);
        }

        pub fn update(self: *Self) void {
            const targets = self.currentWheelTargets();

            for (0..4) |i| {
                const counts = Platform.readEncoder(self.encoder_ids[i], true);
                const current = self.encoderToSpeed(counts);

                self.wheel_pid[i].setSetpoint(targets[i]);
                var output = self.wheel_pid[i].compute(current);

                if (output > 1.0) output = 1.0;
                if (output < -1.0) output = -1.0;

                Platform.setMotor(self.motor_ids[i], output);
            }
        }
    };
}

// --- tests ------------------------------------------------------------------

const test_platform = @import("test_platform.zig");
const RecordingPlatform = test_platform.Recording;

fn testMecanum() MecanumDrive(RecordingPlatform) {
    return MecanumDrive(RecordingPlatform).init(
        0.03,
        0.2,
        0.16,
        1.0,
        3.0,
        .{ 11, 12, 13, 14 },
        .{ 1, 2, 3, 4 },
        1000.0,
        0.01,
    );
}

test "mecanum: construction and clamps match the C answers" {
    const corpus = try corpus_mod.Corpus.load();

    var md = testMecanum();
    try corpus.expectValue("mecanum_pid_kp", md.wheel_pid[0].kp);
    try corpus.expectValue("mecanum_pid_output_max", md.wheel_pid[0].output_max);
    try corpus.expectValue("mecanum_target_vx_init", md.target_vx);
    try corpus.expectValue("mecanum_encoder_to_speed", md.encoderToSpeed(1000.0));

    md.setVelocity(0.3, -0.2, 0.1);
    try corpus.expectValue("mecanum_target_vx", md.target_vx);
    try corpus.expectValue("mecanum_target_vy", md.target_vy);
    try corpus.expectValue("mecanum_target_omega", md.target_omega);

    md.setVelocity(3.0, 4.0, 9.0);
    try corpus.expectValue("mecanum_clamped_vx", md.target_vx);
    try corpus.expectValue("mecanum_clamped_vy", md.target_vy);
    try corpus.expectValue("mecanum_clamped_omega", md.target_omega);
}

test "mecanum: a run commands the motors the C version commands" {
    const corpus = try corpus_mod.Corpus.load();

    var md = testMecanum();
    md.setPidParams(1.0, 0.2, 0.1);
    try corpus.expectValue("mecanum_setparams_ki", md.wheel_pid[3].ki);

    md.setVelocity(0.3, -0.2, 0.1);
    var w: [4][3]f32 = undefined;
    for (0..3) |i| {
        RecordingPlatform.startPass(i);
        md.update();
        for (0..4) |wheel| {
            try std.testing.expectEqual(@as(i32, @intCast(11 + wheel)), RecordingPlatform.log[wheel].motor_id);
            w[wheel][i] = RecordingPlatform.log[wheel].speed;
        }
    }

    try corpus.expectValues("mecanum_run_wheel0", &w[0]);
    try corpus.expectValues("mecanum_run_wheel1", &w[1]);
    try corpus.expectValues("mecanum_run_wheel2", &w[2]);
    try corpus.expectValues("mecanum_run_wheel3", &w[3]);
    try corpus.expectInt("mecanum_motor_id_0", RecordingPlatform.log[0].motor_id);
    try corpus.expectInt("mecanum_motor_id_3", RecordingPlatform.log[3].motor_id);
    try std.testing.expect(RecordingPlatform.allResets());
}

test "mecanum: pure rotation turns the sides against each other" {
    const corpus = try corpus_mod.Corpus.load();

    var md = testMecanum();
    md.setVelocity(0.0, 0.0, 1.0);

    // The finding this file exists to contrast: the rotation term differs by
    // side, so a pure rotation gives two values with opposite signs - unlike the
    // omni platform, where all wheels get the same one (REVIEW §36).
    const targets = md.currentWheelTargets();
    const fl = targets[@intFromEnum(Wheel.front_left)];
    const fr = targets[@intFromEnum(Wheel.front_right)];
    const rl = targets[@intFromEnum(Wheel.rear_left)];
    const rr = targets[@intFromEnum(Wheel.rear_right)];

    try std.testing.expectEqual(fl, rl);
    try std.testing.expectEqual(fr, rr);
    try std.testing.expectApproxEqAbs(-fr, fl, 1.0e-6);

    // And the same targets as the C version produced, read out of its PID
    // setpoints by the generator.
    try corpus.expectValues("mecanum_pure_rotation_targets", &.{ fl, fr, rl, rr });

    // Independent of the corpus: with omega = 0 the targets are the classic
    // mecanum combinations of vx and vy.
    var straight = testMecanum();
    straight.setVelocity(0.3, -0.2, 0.0);
    const t = straight.currentWheelTargets();
    try std.testing.expectApproxEqAbs(@as(f32, 0.5), t[@intFromEnum(Wheel.front_left)], 1.0e-6);
    try std.testing.expectApproxEqAbs(@as(f32, 0.1), t[@intFromEnum(Wheel.front_right)], 1.0e-6);
    try std.testing.expectApproxEqAbs(@as(f32, 0.1), t[@intFromEnum(Wheel.rear_left)], 1.0e-6);
    try std.testing.expectApproxEqAbs(@as(f32, 0.5), t[@intFromEnum(Wheel.rear_right)], 1.0e-6);
}

test "mecanum: the per-wheel rescale fires at full speed and full rotation" {
    const corpus = try corpus_mod.Corpus.load();

    var md = testMecanum();
    md.setVelocity(1.0, 0.0, 3.0);
    const targets = md.currentWheelTargets();
    try corpus.expectValues("mecanum_rescaled_targets", &targets);

    var largest: f32 = 0;
    for (targets) |t| {
        if (@abs(t) > largest) largest = @abs(t);
    }
    try std.testing.expectApproxEqAbs(md.max_linear_speed, largest, 1.0e-6);
}
