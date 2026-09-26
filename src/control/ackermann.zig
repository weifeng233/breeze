//! Ackermann steering platform, ported from
//! `include/breeze/control/platform/ackermann_steering.h`.
//!
//! It shares the shape of the other platform controllers (a comptime platform
//! type, PID objects by value, the kinematics as a pure function). Three things
//! were unusual here; two of them are now fixed, and the third was already
//! resolved by dropping the field:
//!
//! * **`wheelAngles` used to lose the turn direction** (§38). The C computed a
//!   negative turning radius for a right turn - which is right, `atan` then
//!   returns negative angles - and *then* negated them, so a right turn came out
//!   looking exactly like a left one. The negation is gone; a right turn now gives
//!   two negative angles with the inner wheel further round than the outer, which
//!   is the mirror of the left turn. REVIEW §52.
//! * **The steering PID is gone** (§38). The C has a `steering_pid` and a setter
//!   for its gains, writes its setpoint every pass, and never computes it: the
//!   steering command is `target / max`, feedforward. Closing that loop would need
//!   a steering-angle measurement, and the platform interface has none - one
//!   encoder, on the drive motor - so computing it would mean *inventing* a
//!   feedback path. The knob is removed instead, and the test asserts its absence,
//!   because a parameter that cannot affect anything invites a caller to believe it
//!   did.
//! * **`steering_ratio` is not carried.** The C config has the field and nothing
//!   reads it, so it has no place here - same reasoning as the PID above.
//!
//! One limitation is left, and it is the C's: the geometry divides by
//! `turning_radius ∓ track_width / 2`, which passes through zero when the turn is
//! tight enough that the radius is under half the track. `setTargets` cannot reach
//! that (its clamp is a steering angle, not a radius), but `wheelAngles` is public
//! and takes any angle. It is recorded rather than guarded, because a guard would
//! be a new decision about what a physically impossible turn should answer.

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
                .dt = dt,
            };
        }

        pub fn setSpeedPidParams(self: *Self, kp: f32, ki: f32, kd: f32) void {
            self.speed_pid.kp = kp;
            self.speed_pid.ki = ki;
            self.speed_pid.kd = kd;
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
        /// The sign of `steering_angle` survives: a right turn (negative input)
        /// gives two negative angles. It did not before - see the module comment.
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

            // The signs come out of the division: a right turn gives a negative
            // radius, so both quotients are negative and both angles are too. The C
            // negated them here, which turned every right turn into a left one.
            const inner = std.math.atan(self.wheelbase / inner_radius);
            const outer = std.math.atan(self.wheelbase / outer_radius);

            return .{ .inner = inner, .outer = outer };
        }

        /// One control pass. The speed loop is closed. The steering channel is
        /// `target / max` with no feedback at all - feedforward *by design*, since
        /// the platform has no steering-angle measurement; see the module comment
        /// for why the C's unused steering PID is not reproduced.
        ///
        /// The C version also computes the wheel angles here and discards them
        /// ("for reference only, not used in this simplified control"). The port
        /// does not, because a computation with no effect is not worth keeping
        /// alive - `wheelAngles` is a function any caller can use.
        pub fn update(self: *Self) void {
            const counts = Platform.readEncoder(self.encoder_id, true);
            const current_speed = self.encoderToSpeed(counts);

            self.speed_pid.setSetpoint(self.target_speed);

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
    try corpus.expectValue("ackermann_target_speed_init", ak.target_speed);
    try corpus.expectValue("ackermann_encoder_to_speed", ak.encoderToSpeed(1000.0));

    // The three cases the C recorded for its steering PID are gone from the
    // corpus, along with the field: there is no loop to compute. This asserts the
    // absence, so re-adding a knob that nothing reads is a failing test rather
    // than a quiet regression.
    try std.testing.expect(!@hasField(@TypeOf(ak), "steering_pid"));
    try std.testing.expect(!@hasDecl(@TypeOf(ak), "setSteeringPidParams"));

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

test "ackermann: a right turn is the mirror of a left turn" {
    const corpus = try corpus_mod.Corpus.load();

    const ak = testAckermann();

    const left = ak.wheelAngles(0.2);
    const right = ak.wheelAngles(-0.2);
    try corpus.expectValue("ackermann_angles_left_inner", left.inner);
    try corpus.expectValue("ackermann_angles_left_outer", left.outer);
    try corpus.expectValue("ackermann_angles_right_inner", right.inner);
    try corpus.expectValue("ackermann_angles_right_outer", right.outer);

    // The fix, asserted: a right turn steers the other way. The C negated both
    // angles at the end, which made this pair equal.
    try std.testing.expect(right.inner < 0);
    try std.testing.expect(right.outer < 0);
    try std.testing.expect(!std.math.approxEqAbs(f32, left.inner, right.inner, 1.0e-6));

    // Mirror symmetry, which is the property the geometry has to have and which a
    // copied number would not establish.
    try std.testing.expectApproxEqAbs(left.inner, -right.inner, 1.0e-6);
    try std.testing.expectApproxEqAbs(left.outer, -right.outer, 1.0e-6);

    // The inner wheel steers further than the outer one, on both sides: it is the
    // one nearer the turning centre.
    try std.testing.expect(left.inner > left.outer);
    try std.testing.expect(@abs(right.inner) > @abs(right.outer));

    // The small-angle branch passes the input through, so it agrees with the
    // geometry about sign now instead of contradicting it.
    const small_left = ak.wheelAngles(0.005);
    const small_right = ak.wheelAngles(-0.005);
    try corpus.expectValue("ackermann_angles_small_pos_inner", small_left.inner);
    try corpus.expectValue("ackermann_angles_small_pos_outer", small_left.outer);
    try corpus.expectValue("ackermann_angles_small_neg_inner", small_right.inner);
    try corpus.expectValue("ackermann_angles_small_neg_outer", small_right.outer);

    try std.testing.expectEqual(small_left.inner, -small_right.inner);
    try std.testing.expectEqual(@as(f32, 0.0), ak.wheelAngles(0.0).inner);
    try corpus.expectValue("ackermann_angles_zero_inner", 0.0);

    // And the two branches agree at the boundary: just above it the geometry must
    // not answer a different sign than just below.
    try std.testing.expect(ak.wheelAngles(0.0101).inner > 0);
    try std.testing.expect(ak.wheelAngles(-0.0101).inner < 0);
}

test "ackermann: the steering channel is feedforward, and that is all there is" {
    const corpus = try corpus_mod.Corpus.load();

    var ak = testAckermann();
    ak.setTargets(0.4, 0.3);
    RecordingPlatform.clearLog();
    ak.update();

    try corpus.expectValue("ackermann_speed_setpoint", ak.speed_pid.setpoint);

    try corpus.expectValues("ackermann_update_motors", &.{
        RecordingPlatform.log[0].speed, RecordingPlatform.log[1].speed,
    });
    try corpus.expectInt("ackermann_drive_motor_id", RecordingPlatform.log[0].motor_id);
    try corpus.expectInt("ackermann_steering_motor_id", RecordingPlatform.log[1].motor_id);

    // The steering command is the target over the maximum, with nothing between
    // them: no state, no gains, no history to vary.
    try std.testing.expectEqual(
        ak.target_steering_angle / ak.max_steering_angle,
        RecordingPlatform.log[1].speed,
    );

    // The steering channel is stateless, so a second pass commands exactly the
    // same value - which a loop would have made false.
    const second = RecordingPlatform.log[1].speed;
    RecordingPlatform.startPass(0);
    ak.update();
    try std.testing.expectEqual(second, RecordingPlatform.log[1].speed);
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
