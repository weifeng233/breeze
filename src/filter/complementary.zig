//! Complementary filter for roll and pitch, ported from
//! `include/breeze/filter/complementary_filter.h`.
//!
//! The C signature takes `gyro_z` and never reads it - the state has no yaw
//! field, so there is nothing to integrate it into. That was documented in the C
//! header rather than acted on; here the parameter is simply **not in the
//! signature**, because a parameter the port cannot use is a parameter that
//! invites a caller to think it did something. A caller that has three gyro axes
//! ignores the third.
//!
//! The filter is stateful, so `update` takes `self: *Self`.

const std = @import("std");

const corpus_mod = @import("../math/corpus.zig");

pub const Complementary = struct {
    roll: f32 = 0,
    pitch: f32 = 0,
    alpha: f32 = 0,
    dt: f32 = 0,

    pub fn init(alpha: f32, dt: f32) Complementary {
        return .{ .alpha = alpha, .dt = dt };
    }

    /// One update from a gyro and accelerometer sample.
    ///
    /// `gyro_x` and `gyro_y` are integrated; the accelerometer gives the absolute
    /// reference through its gravity vector. `alpha` weights the gyro estimate
    /// against the accelerometer estimate.
    pub fn update(
        self: *Complementary,
        gyro_x: f32,
        gyro_y: f32,
        accel_x: f32,
        accel_y: f32,
        accel_z: f32,
    ) void {
        const accel_roll = std.math.atan2(accel_y, accel_z);
        const accel_pitch = std.math.atan2(
            -accel_x,
            @sqrt(accel_y * accel_y + accel_z * accel_z),
        );

        const gyro_roll = self.roll + gyro_x * self.dt;
        const gyro_pitch = self.pitch + gyro_y * self.dt;

        self.roll = self.alpha * gyro_roll + (1.0 - self.alpha) * accel_roll;
        self.pitch = self.alpha * gyro_pitch + (1.0 - self.alpha) * accel_pitch;
    }

    pub fn getRoll(self: Complementary) f32 {
        return self.roll;
    }

    pub fn getPitch(self: Complementary) f32 {
        return self.pitch;
    }
};

// --- tests ------------------------------------------------------------------

test "complementary: a run matches the C answers" {
    const corpus = try corpus_mod.Corpus.load();

    const gx = [_]f32{ 0.01, 0.02, -0.01, 0.03, 0.0 };
    const gy = [_]f32{ 0.0, 0.01, 0.02, -0.02, 0.01 };
    const ax = [_]f32{ 0.0, 0.1, 0.2, -0.1, 0.05 };
    const ay = [_]f32{ 0.0, 0.05, 0.1, 0.0, -0.05 };
    const az = [_]f32{ 1.0, 0.99, 0.98, 1.0, 0.97 };

    var f = Complementary.init(0.9, 0.01);
    try corpus.expectValue("complementary_roll_init", f.roll);
    try corpus.expectValue("complementary_pitch_init", f.pitch);
    try corpus.expectValue("complementary_alpha_init", f.alpha);
    try corpus.expectValue("complementary_dt_init", f.dt);

    var roll: [5]f32 = undefined;
    var pitch: [5]f32 = undefined;
    for (0..5) |i| {
        f.update(gx[i], gy[i], ax[i], ay[i], az[i]);
        roll[i] = f.getRoll();
        pitch[i] = f.getPitch();
    }
    try corpus.expectValues("complementary_run_roll", &roll);
    try corpus.expectValues("complementary_run_pitch", &pitch);
}

test "complementary: alpha chooses between the two estimates" {
    // alpha = 0 means "trust the accelerometer entirely", so a single update
    // lands exactly on the angle the gravity vector implies.
    var accel_only = Complementary.init(0.0, 0.01);
    accel_only.update(999.0, 999.0, 0.0, 1.0, 0.0); // gyro is ignored
    try std.testing.expectApproxEqAbs(std.math.pi / 2.0, accel_only.roll, 1.0e-6);
    try std.testing.expectApproxEqAbs(@as(f32, 0.0), accel_only.pitch, 1.0e-6);

    // alpha = 1 means "trust the gyro entirely": with a level accelerometer the
    // accelerometer estimate is zero, so the output is the integrated rate.
    var gyro_only = Complementary.init(1.0, 0.5);
    gyro_only.update(0.2, -0.4, 0.0, 0.0, 1.0);
    try std.testing.expectApproxEqAbs(@as(f32, 0.1), gyro_only.roll, 1.0e-6);
    try std.testing.expectApproxEqAbs(@as(f32, -0.2), gyro_only.pitch, 1.0e-6);

    // Level and still: nothing moves.
    var level = Complementary.init(0.5, 0.01);
    level.update(0.0, 0.0, 0.0, 0.0, 1.0);
    try std.testing.expectApproxEqAbs(@as(f32, 0.0), level.roll, 1.0e-6);
    try std.testing.expectApproxEqAbs(@as(f32, 0.0), level.pitch, 1.0e-6);
}
