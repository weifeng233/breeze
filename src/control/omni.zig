//! Omni-directional platforms, ported from
//! `include/breeze/control/platform/omni_drive.h`.
//!
//! Two wheel layouts, three at 120 degrees and four at 90. The wheel count is a
//! comptime property of the kind, so the arrays have no unused tail and the
//! "3 or 4" is not a runtime number anyone can get wrong.
//!
//! **The inverse kinematics is exposed as a function.** In C it is written inline
//! in `Update`, which makes it invisible to a test and hard to read; here
//! `wheelTargets` is a pure function of the three commanded velocities, and
//! `update` is the loop around it. Same arithmetic, in the same order.
//!
//! That is how the defect below is pinned. The C rotation term is
//!
//!     wheel_targets[i] = vx·cos(θᵢ) + vy·sin(θᵢ) + wheel_distance·ω
//!
//! - the same `+ d·ω` for every wheel. A rotation turns wheels at *different*
//!   angles in different directions (the tangential direction depends on where
//!   the wheel sits); adding one constant to all of them is a translation along
//!   the average wheel direction, not a rotation. The port reproduces it and a
//!   test asserts the equality, so the behaviour is pinned rather than described
//!   in a comment. Fixing it is a change to the platform's control law, and that
//!   needs its own decision and its own tests - see REVIEW §36.
//!
//! Angles use the literal `3.14159` and f32 arithmetic, as the C code does;
//! `std.math.pi` would move them (and the corpus would say so).

const std = @import("std");

const corpus_mod = @import("../math/corpus.zig");
const pid_mod = @import("pid.zig");
const platform_mod = @import("platform.zig");

pub const Pid = pid_mod.Pid;
const requirePlatform = platform_mod.requirePlatform;

pub const OmniKind = enum { three_wheel_120deg, four_wheel_90deg };

fn wheelCount(comptime kind: OmniKind) usize {
    return switch (kind) {
        .three_wheel_120deg => 3,
        .four_wheel_90deg => 4,
    };
}

/// Wheel angles measured from the x axis, computed the way the C `Init` does.
fn wheelAngles(comptime kind: OmniKind) [wheelCount(kind)]f32 {
    const pi: f32 = 3.14159;
    return switch (kind) {
        .three_wheel_120deg => .{ 0.0, 2.0 * pi / 3.0, 4.0 * pi / 3.0 },
        .four_wheel_90deg => .{ pi / 4.0, 3.0 * pi / 4.0, 5.0 * pi / 4.0, 7.0 * pi / 4.0 },
    };
}

pub fn OmniDrive(comptime Platform: type, comptime kind: OmniKind) type {
    requirePlatform(Platform);

    const wheels = comptime wheelCount(kind);
    const angles = comptime wheelAngles(kind);

    return struct {
        wheel_radius: f32,
        /// Distance from the platform centre to each wheel, in metres.
        wheel_distance: f32,
        max_linear_speed: f32,
        max_angular_speed: f32,
        encoder_resolution: f32,

        motor_ids: [wheels]i32,
        encoder_ids: [wheels]i32,

        wheel_pid: [wheels]Pid,

        target_vx: f32 = 0,
        target_vy: f32 = 0,
        target_omega: f32 = 0,
        dt: f32,

        const Self = @This();

        pub const wheel_count = wheels;
        pub const wheel_angles = angles;

        /// Every wheel PID starts as a position controller with the same fixed
        /// gains and `±1` output range the C `Init` uses.
        pub fn init(
            wheel_radius: f32,
            wheel_distance: f32,
            max_linear_speed: f32,
            max_angular_speed: f32,
            motor_ids: [wheels]i32,
            encoder_ids: [wheels]i32,
            encoder_resolution: f32,
            dt: f32,
        ) Self {
            var self = Self{
                .wheel_radius = wheel_radius,
                .wheel_distance = wheel_distance,
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

        /// The linear part is clamped **radially** - both components scale
        /// together, so the commanded direction survives - while the angular part
        /// is clamped on its own. That asymmetry is the C behaviour.
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
        ///
        /// See the module comment: the rotation term is a constant added to every
        /// wheel, which is what the C code does.
        pub fn wheelTargets(self: Self, vx: f32, vy: f32, omega: f32) [wheels]f32 {
            var targets: [wheels]f32 = undefined;
            for (angles, 0..) |angle, i| {
                targets[i] = vx * @cos(angle) + vy * @sin(angle);
                targets[i] += self.wheel_distance * omega;
            }

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

        /// `wheelTargets` for the velocity currently commanded.
        pub fn currentWheelTargets(self: Self) [wheels]f32 {
            return self.wheelTargets(self.target_vx, self.target_vy, self.target_omega);
        }

        pub fn update(self: *Self) void {
            const targets = self.currentWheelTargets();

            for (0..wheels) |i| {
                const counts = Platform.readEncoder(self.encoder_ids[i], true);
                const current = self.encoderToSpeed(counts);

                self.wheel_pid[i].setSetpoint(targets[i]);
                var output = self.wheel_pid[i].compute(current);

                // The C version clamps here as well as inside the PID, whose
                // limits are already ±1; kept so the two agree whatever the
                // caller did to the PID.
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

fn testOmni3() OmniDrive(RecordingPlatform, .three_wheel_120deg) {
    return OmniDrive(RecordingPlatform, .three_wheel_120deg).init(
        0.03,
        0.1,
        1.0,
        3.0,
        .{ 11, 12, 13 },
        .{ 1, 2, 3 },
        1000.0,
        0.01,
    );
}

test "omni: construction and the wheel layout match the C answers" {
    const corpus = try corpus_mod.Corpus.load();

    const od = testOmni3();
    try corpus.expectInt("omni3_num_wheels", @TypeOf(od).wheel_count);
    try corpus.expectValues("omni3_angles", &@TypeOf(od).wheel_angles);
    try corpus.expectValue("omni3_wheel_radius", od.wheel_radius);
    try corpus.expectValue("omni3_pid_kp", od.wheel_pid[0].kp);
    try corpus.expectValue("omni3_pid_output_max", od.wheel_pid[0].output_max);
    try corpus.expectValue("omni3_target_vx_init", od.target_vx);
    try corpus.expectValue("omni3_encoder_to_speed", od.encoderToSpeed(1000.0));

    const od4 = OmniDrive(RecordingPlatform, .four_wheel_90deg).init(
        0.03,
        0.12,
        1.0,
        3.0,
        .{ 11, 12, 13, 14 },
        .{ 1, 2, 3, 4 },
        1000.0,
        0.01,
    );
    try corpus.expectInt("omni4_num_wheels", @TypeOf(od4).wheel_count);
    try corpus.expectValues("omni4_angles", &@TypeOf(od4).wheel_angles);
}

test "omni: setVelocity clamps like the C version" {
    const corpus = try corpus_mod.Corpus.load();

    var od = testOmni3();
    od.setVelocity(0.5, 0.0, 0.0);
    try corpus.expectValue("omni3_target_vx", od.target_vx);

    // (3, 4) is 5 long, so both components scale by 0.2 - radially, keeping the
    // direction. A per-component clamp would give (1, 1).
    od.setVelocity(3.0, 4.0, 0.0);
    try corpus.expectValue("omni3_clamped_vx", od.target_vx);
    try corpus.expectValue("omni3_clamped_vy", od.target_vy);
    try std.testing.expectApproxEqAbs(@as(f32, 0.6 / 0.8), od.target_vx / od.target_vy, 1.0e-6);

    od.setVelocity(0.0, 0.0, 9.0);
    try corpus.expectValue("omni3_angular_clamped", od.target_omega);

    od.setVelocity(0.0, 0.0, -9.0);
    try std.testing.expectEqual(@as(f32, -3.0), od.target_omega);

    // The per-wheel rescale, which the ordinary run never reaches: at full
    // linear speed *and* full rotation the raw targets exceed the maximum, so
    // every wheel is scaled together. Without this case, dropping the rescale
    // passed every test - the probe found the gap.
    var fast = testOmni3();
    fast.setVelocity(1.0, 0.0, 3.0);
    try corpus.expectValues("omni3_unclamped_velocity", &.{ fast.target_vx, fast.target_omega });

    const rescaled = fast.currentWheelTargets();
    try corpus.expectValues("omni3_rescaled_targets", &rescaled);

    var largest: f32 = 0;
    for (rescaled) |t| {
        if (@abs(t) > largest) largest = @abs(t);
    }
    try std.testing.expectApproxEqAbs(fast.max_linear_speed, largest, 1.0e-6);
}

test "omni: a run commands the motors the C version commands" {
    const corpus = try corpus_mod.Corpus.load();

    var od = testOmni3();
    od.setPidParams(1.0, 0.2, 0.1);
    try corpus.expectValue("omni3_setparams_ki", od.wheel_pid[2].ki);

    od.setVelocity(0.3, 0.1, 0.2);
    var w0: [3]f32 = undefined;
    var w1: [3]f32 = undefined;
    var w2: [3]f32 = undefined;
    for (0..3) |i| {
        RecordingPlatform.startPass(i);
        od.update();
        try std.testing.expectEqual(@as(i32, 11), RecordingPlatform.log[0].motor_id);
        try std.testing.expectEqual(@as(i32, 12), RecordingPlatform.log[1].motor_id);
        try std.testing.expectEqual(@as(i32, 13), RecordingPlatform.log[2].motor_id);
        w0[i] = RecordingPlatform.log[0].speed;
        w1[i] = RecordingPlatform.log[1].speed;
        w2[i] = RecordingPlatform.log[2].speed;
    }
    try corpus.expectValues("omni3_run_wheel0", &w0);
    try corpus.expectValues("omni3_run_wheel1", &w1);
    try corpus.expectValues("omni3_run_wheel2", &w2);
    try corpus.expectInt("omni3_motor_id_0", RecordingPlatform.log[0].motor_id);
    try corpus.expectInt("omni3_motor_id_2", RecordingPlatform.log[2].motor_id);

    // Every wheel read asks for a reset, as the differential drive does.
    try std.testing.expect(RecordingPlatform.allResets());

    // Four wheels: the fourth motor is commanded too.
    var od4 = OmniDrive(RecordingPlatform, .four_wheel_90deg).init(
        0.03,
        0.12,
        1.0,
        3.0,
        .{ 11, 12, 13, 14 },
        .{ 1, 2, 3, 4 },
        1000.0,
        0.01,
    );
    od4.setVelocity(0.3, 0.1, 0.2);
    var w3: [3]f32 = undefined;
    for (0..3) |i| {
        RecordingPlatform.startPass(i);
        od4.update();
        w3[i] = RecordingPlatform.log[3].speed;
    }
    try corpus.expectValues("omni4_run_wheel3", &w3);
    try corpus.expectInt("omni4_motor_id_3", RecordingPlatform.log[3].motor_id);
}

test "omni: pure rotation gives every wheel the same target" {
    const corpus = try corpus_mod.Corpus.load();

    var od = testOmni3();
    od.setVelocity(0.0, 0.0, 1.0);

    // The finding, asserted: with no translation the C inverse kinematics adds
    // the same `d * omega` to each wheel, so the three targets are identical -
    // and a rotation cannot be, because the wheels sit at different angles. The
    // end-to-end commands still differ (the wheels measured different speeds),
    // which is why this is asserted on the targets rather than on the commands.
    const targets = od.currentWheelTargets();
    try std.testing.expectEqual(targets[0], targets[1]);
    try std.testing.expectEqual(targets[1], targets[2]);
    try std.testing.expectApproxEqAbs(@as(f32, 0.1), targets[0], 1.0e-6);

    // The run the corpus recorded, for the same scenario.
    var rot0: [3]f32 = undefined;
    var rot1: [3]f32 = undefined;
    var rot2: [3]f32 = undefined;
    for (0..3) |i| {
        RecordingPlatform.startPass(i);
        od.update();
        rot0[i] = RecordingPlatform.log[0].speed;
        rot1[i] = RecordingPlatform.log[1].speed;
        rot2[i] = RecordingPlatform.log[2].speed;
    }
    try corpus.expectValues("omni3_pure_rotation_wheel0", &rot0);
    try corpus.expectValues("omni3_pure_rotation_wheel1", &rot1);
    try corpus.expectValues("omni3_pure_rotation_wheel2", &rot2);

    // Independent of the corpus: with the rotation term at zero the targets are
    // the projection of the commanded velocity onto each wheel's direction.
    var straight = testOmni3();
    straight.setVelocity(0.4, 0.0, 0.0);
    const projected = straight.currentWheelTargets();
    for (@TypeOf(straight).wheel_angles, 0..) |angle, i| {
        try std.testing.expectApproxEqAbs(0.4 * @cos(angle), projected[i], 1.0e-6);
    }
}
