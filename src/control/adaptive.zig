//! Adaptive controllers, ported from
//! `include/breeze/control/adaptive_controller.h`.
//!
//! Two controllers in one header: a model-reference adaptive controller (MRAC)
//! and a self-tuning PID. Both are stateful and both adjust their own parameters
//! as they run, so the corpus cases are runs rather than single calls.
//!
//! Three things are copied rather than tidied:
//!
//! * **MRAC's `prev_x` and `prev_u` are written and never read.** They are state
//!   a caller can inspect, so they are kept and the corpus records them - unlike
//!   a *configuration* input nothing reads, which the port drops (REVIEW §33,
//!   §38). The distinction is between state that is merely unused and a knob that
//!   cannot do anything.
//! * **`error_threshold` does two jobs.** It gates the whole adaptation block and
//!   it is also the threshold for the derivative term's `|error_change|`, so the
//!   same number means "a large error" in one place and "a fast change" in the
//!   other.
//! * **The P, I and D adjustments are heuristics, not a control law.** The C code
//!   lists three rules of thumb keyed on the sign of `error * prev_error` and on
//!   `error_change`; they are reproduced exactly, and the corpus pins the
//!   resulting gains. Whether they are *good* rules is a separate question that a
//!   migration has no business answering.
//!
//! The dead `output_change` local that the C version computed (and that nothing
//! read) was already removed during the C repair - REVIEW §26 - so it does not
//! appear here either.

const std = @import("std");

const corpus_mod = @import("../math/corpus.zig");
const pid_mod = @import("pid.zig");

pub const Pid = pid_mod.Pid;
pub const PidKind = pid_mod.Kind;

pub const Mrac = struct {
    /// Reference model pole coefficient.
    a_m: f32,
    /// Reference model input gain.
    b_m: f32,

    theta: [2]f32 = .{ 0, 0 },
    gamma: [2]f32,

    x_m: f32 = 0,
    x: f32 = 0,

    /// Written on every update and read by nothing. Kept because the C struct
    /// has them and a caller can look; see the module comment.
    prev_x: f32 = 0,
    prev_u: f32 = 0,

    u_min: f32,
    u_max: f32,
    dt: f32,

    pub fn init(
        a_m: f32,
        b_m: f32,
        gamma1: f32,
        gamma2: f32,
        u_min: f32,
        u_max: f32,
        dt: f32,
    ) Mrac {
        return .{
            .a_m = a_m,
            .b_m = b_m,
            .gamma = .{ gamma1, gamma2 },
            .u_min = u_min,
            .u_max = u_max,
            .dt = dt,
        };
    }

    /// One adaptation step: advance the reference model, update the two
    /// parameters from the tracking error, then produce the control input.
    pub fn update(self: *Mrac, r: f32, y: f32) f32 {
        self.prev_x = self.x;
        self.x = y;

        self.x_m = (1.0 + self.a_m * self.dt) * self.x_m + self.b_m * self.dt * r;

        const err = self.x_m - self.x;

        self.theta[0] += self.gamma[0] * err * r * self.dt;
        self.theta[1] += self.gamma[1] * err * self.x * self.dt;

        var u = self.theta[0] * r + self.theta[1] * self.x;
        if (u > self.u_max) {
            u = self.u_max;
        } else if (u < self.u_min) {
            u = self.u_min;
        }

        self.prev_u = u;
        return u;
    }
};

pub const AdaptivePid = struct {
    /// The controller whose gains are tuned.
    pid: Pid,

    kp_min: f32,
    kp_max: f32,
    ki_min: f32,
    ki_max: f32,
    kd_min: f32,
    kd_max: f32,

    adaptation_rate: f32,
    /// Gates the adaptation block, and is also the threshold for `error_change`.
    error_threshold: f32,

    prev_error: f32 = 0,
    prev_output: f32 = 0,

    adaptation_counter: i32 = 0,
    adaptation_period: i32,

    /// The initial gains define the allowed range: a tenth of each up to five
    /// times each, as the C `Init` sets it.
    pub fn init(
        kind: PidKind,
        kp: f32,
        ki: f32,
        kd: f32,
        dt: f32,
        output_min: f32,
        output_max: f32,
        adaptation_rate: f32,
        error_threshold: f32,
        adaptation_period: i32,
    ) AdaptivePid {
        return .{
            .pid = Pid.init(kind, kp, ki, kd, dt, output_min, output_max),
            .kp_min = kp * 0.1,
            .kp_max = kp * 5.0,
            .ki_min = ki * 0.1,
            .ki_max = ki * 5.0,
            .kd_min = kd * 0.1,
            .kd_max = kd * 5.0,
            .adaptation_rate = adaptation_rate,
            .error_threshold = error_threshold,
            .adaptation_period = adaptation_period,
        };
    }

    /// One control step with adaptation. `setpoint` is applied to the inner PID.
    pub fn compute(self: *AdaptivePid, setpoint: f32, measurement: f32) f32 {
        self.pid.setpoint = setpoint;

        const err = setpoint - measurement;
        const output = self.pid.compute(measurement);
        const error_change = err - self.prev_error;

        self.adaptation_counter += 1;
        if (self.adaptation_counter >= self.adaptation_period) {
            self.adaptation_counter = 0;

            // Only a large error triggers adaptation at all.
            if (@abs(err) > self.error_threshold) {
                var kp_delta: f32 = 0;
                var ki_delta: f32 = 0;
                var kd_delta: f32 = 0;

                // Growing error in the same direction: more P.
                if (err * self.prev_error > 0.0 and @abs(err) > @abs(self.prev_error)) {
                    kp_delta = self.adaptation_rate * 0.1;
                } else {
                    kp_delta = -self.adaptation_rate * 0.05;
                }

                // Sustained error: more I, otherwise less.
                if (err * self.prev_error > 0.0) {
                    ki_delta = self.adaptation_rate * 0.05;
                } else {
                    ki_delta = -self.adaptation_rate * 0.1;
                }

                // Fast change: more D. Note that this reuses `error_threshold`,
                // which above gated the error itself.
                if (@abs(error_change) > @abs(self.error_threshold)) {
                    kd_delta = self.adaptation_rate * 0.1;
                } else {
                    kd_delta = -self.adaptation_rate * 0.05;
                }

                self.pid.kp = clampGain(self.pid.kp + kp_delta, self.kp_min, self.kp_max);
                self.pid.ki = clampGain(self.pid.ki + ki_delta, self.ki_min, self.ki_max);
                self.pid.kd = clampGain(self.pid.kd + kd_delta, self.kd_min, self.kd_max);
            }
        }

        self.prev_error = err;
        // Written and never read - the same situation as MRAC's `prev_x`.
        self.prev_output = output;

        return output;
    }
};

/// The C version's two-comparison clamp, written the same way: `std.math.clamp`
/// is `@min(@max(...))`, which treats a NaN differently from the if-chain.
fn clampGain(value: f32, low: f32, high: f32) f32 {
    var v = value;
    if (v < low) v = low;
    if (v > high) v = high;
    return v;
}

// --- tests ------------------------------------------------------------------

test "mrac: construction and a run match the C answers" {
    const corpus = try corpus_mod.Corpus.load();

    var m = Mrac.init(-1.0, 2.0, 0.5, 0.75, -10.0, 10.0, 0.01);
    try corpus.expectValue("mrac_theta0_init", m.theta[0]);
    try corpus.expectValue("mrac_theta1_init", m.theta[1]);
    try corpus.expectValue("mrac_gamma0", m.gamma[0]);
    try corpus.expectValue("mrac_gamma1", m.gamma[1]);
    try corpus.expectValue("mrac_x_m_init", m.x_m);
    try corpus.expectValue("mrac_prev_x_init", m.prev_x);

    const ys = [_]f32{ 0.0, 0.1, 0.25, 0.5, 0.8 };
    var out: [5]f32 = undefined;
    for (ys, 0..) |y, i| out[i] = m.update(1.0, y);
    try corpus.expectValues("mrac_run", &out);

    try corpus.expectValue("mrac_theta0_after", m.theta[0]);
    try corpus.expectValue("mrac_theta1_after", m.theta[1]);
    try corpus.expectValue("mrac_x_m_after", m.x_m);
    try corpus.expectValue("mrac_prev_x_after", m.prev_x);
    try corpus.expectValue("mrac_prev_u_after", m.prev_u);

    // Independent of the corpus: the reference model with a negative pole decays
    // towards b_m*r/(-a_m), and the tracking error drives the parameters.
    var m2 = Mrac.init(-1.0, 2.0, 0.5, 0.75, -10.0, 10.0, 0.01);
    _ = m2.update(1.0, 0.0);
    try std.testing.expect(m2.x_m > 0.0);
    try std.testing.expect(m2.theta[0] > 0.0);
}

test "mrac: the control input is clamped and adaptation continues underneath" {
    const corpus = try corpus_mod.Corpus.load();

    var m = Mrac.init(-1.0, 2.0, 100.0, 100.0, -0.5, 0.5, 0.01);
    var out: [5]f32 = undefined;
    for (0..5) |i| out[i] = m.update(10.0, 0.0);
    try corpus.expectValues("mrac_clamped_run", &out);
    try corpus.expectValue("mrac_clamped_theta0_after", m.theta[0]);

    for (out) |u| try std.testing.expect(u >= -0.5 and u <= 0.5);
    // The clamp does not stop the adaptation: theta keeps growing even though the
    // output cannot.
    try std.testing.expect(m.theta[0] > 1.0);
}

test "adaptive PID: construction and a run that adapts match the C answers" {
    const corpus = try corpus_mod.Corpus.load();

    var ap = AdaptivePid.init(.position, 2.0, 0.5, 0.1, 0.01, -5.0, 5.0, 0.1, 0.5, 3);
    try corpus.expectValue("adaptive_kp_min", ap.kp_min);
    try corpus.expectValue("adaptive_kp_max", ap.kp_max);
    try corpus.expectValue("adaptive_ki_min", ap.ki_min);
    try corpus.expectValue("adaptive_kd_max", ap.kd_max);
    try corpus.expectInt("adaptive_period", ap.adaptation_period);
    try corpus.expectValue("adaptive_pid_kp_init", ap.pid.kp);
    try corpus.expectValue("adaptive_prev_error_init", ap.prev_error);
    try corpus.expectValue("adaptive_prev_output_init", ap.prev_output);

    const measurements = [_]f32{ 0.0, 0.5, 1.2, 0.9, 1.1, 1.0 };
    var out: [6]f32 = undefined;
    for (measurements, 0..) |y, i| out[i] = ap.compute(3.0, y);
    try corpus.expectValues("adaptive_run", &out);

    try corpus.expectValue("adaptive_kp_after", ap.pid.kp);
    try corpus.expectValue("adaptive_ki_after", ap.pid.ki);
    try corpus.expectValue("adaptive_kd_after", ap.pid.kd);
    try corpus.expectValue("adaptive_prev_error_after", ap.prev_error);
    try corpus.expectValue("adaptive_prev_output_after", ap.prev_output);
    try corpus.expectInt("adaptive_counter_after", ap.adaptation_counter);

    // The first version of this case used a setpoint of 1.0, where no error ever
    // exceeded the threshold and the gains came out equal to the initial ones -
    // the adaptation block was never reached. Reading the data caught it.
    try std.testing.expect(ap.pid.kp != 2.0);

    // Independent of the corpus: the gains stay inside the range `init` set.
    try std.testing.expect(ap.pid.kp >= ap.kp_min and ap.pid.kp <= ap.kp_max);
    try std.testing.expect(ap.pid.ki >= ap.ki_min and ap.pid.ki <= ap.ki_max);
    try std.testing.expect(ap.pid.kd >= ap.kd_min and ap.pid.kd <= ap.kd_max);
}

test "adaptive PID: a small error never adapts" {
    const corpus = try corpus_mod.Corpus.load();

    var ap = AdaptivePid.init(.position, 2.0, 0.5, 0.1, 0.01, -5.0, 5.0, 0.1, 0.5, 3);
    var out: [6]f32 = undefined;
    for (0..6) |i| out[i] = ap.compute(1.0, 0.99);
    try corpus.expectValues("adaptive_quiet_run", &out);

    try corpus.expectValue("adaptive_quiet_kp_after", ap.pid.kp);
    try corpus.expectInt("adaptive_quiet_counter_after", ap.adaptation_counter);

    // The error is 0.01 against a threshold of 0.5, so the gains never move -
    // even though the counter still cycles.
    try std.testing.expectEqual(@as(f32, 2.0), ap.pid.kp);
    try std.testing.expectEqual(@as(i32, 0), ap.adaptation_counter);
}

test "adaptive PID: the gains are clamped to the range init set" {
    const corpus = try corpus_mod.Corpus.load();

    // Large rate, step error, adaptation every step: each gain lands on a
    // different limit. The ordinary run moves them by thousandths and never gets
    // near one, so without this case deleting the clamp passed every test - a
    // probe found that, and this case is the answer.
    var ap = AdaptivePid.init(.position, 2.0, 0.5, 0.1, 0.01, -50.0, 50.0, 50.0, 0.1, 1);
    var out: [4]f32 = undefined;
    for (0..4) |i| out[i] = ap.compute(10.0, 0.0);
    try corpus.expectValues("adaptive_fast_run", &out);

    try corpus.expectValue("adaptive_fast_kp_after", ap.pid.kp);
    try corpus.expectValue("adaptive_fast_ki_after", ap.pid.ki);
    try corpus.expectValue("adaptive_fast_kd_after", ap.pid.kd);

    try std.testing.expectEqual(ap.kp_min, ap.pid.kp);
    try std.testing.expectEqual(ap.ki_max, ap.pid.ki);
    try std.testing.expectEqual(ap.kd_min, ap.pid.kd);
}
