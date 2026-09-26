//! PID controllers, ported from `include/breeze/control/pid_controller.h`.
//!
//! Two forms share a state struct: the position (absolute) form and the
//! incremental (velocity) form. The C version keeps both in one type selected by
//! a `type` field, and that is kept here as `Kind` - a caller can still switch,
//! and `compute` still dispatches on it.
//!
//! **The derivative filter does not filter.** In the position form the C code
//! blends two terms that are analytically the same quantity:
//!
//!     derivative          = (measurement - prev_measurement) / dt
//!     (prev_error - error) = (measurement - prev_measurement) / dt      [fixed setpoint]
//!
//! so `alpha * derivative + (1 - alpha) * derivative` is a convex combination of
//! a value with itself. The corpus shows exactly how far that goes and no
//! further: `pid_position_alpha0_run`, `..._alpha07_...` and `..._alpha1_...`
//! differ from each other in the *first* sample - where `prev_error` is still its
//! initial 0 while the first error is not - and afterwards agree to the last bit
//! except for the rounding of the blend. `setDerivativeFilter` therefore changes
//! nothing but the first output and the last bit, and the port reproduces that
//! rather than simplifying it away: a migration that "cleaned this up" would stop
//! matching the implementation it is supposed to replace.
//!
//! In the incremental form `alpha` is not read at all.

const std = @import("std");

const corpus_mod = @import("../math/corpus.zig");

pub const Kind = enum { position, incremental };

pub const Pid = struct {
    kind: Kind,

    kp: f32,
    ki: f32,
    kd: f32,

    setpoint: f32 = 0,
    integral: f32 = 0,
    prev_error: f32 = 0,
    prev_prev_error: f32 = 0,
    prev_measurement: f32 = 0,
    prev_output: f32 = 0,

    output_min: f32,
    output_max: f32,

    integral_min: f32,
    integral_max: f32,

    /// Derivative filter coefficient. See the module comment: it barely matters
    /// in the position form and not at all in the incremental one.
    alpha: f32 = 0.1,

    dt: f32,

    /// `Init` sets the integral limits equal to the output limits.
    pub fn init(
        kind: Kind,
        kp: f32,
        ki: f32,
        kd: f32,
        dt: f32,
        output_min: f32,
        output_max: f32,
    ) Pid {
        return .{
            .kind = kind,
            .kp = kp,
            .ki = ki,
            .kd = kd,
            .output_min = output_min,
            .output_max = output_max,
            .integral_min = output_min,
            .integral_max = output_max,
            .dt = dt,
        };
    }

    pub fn setDerivativeFilter(self: *Pid, alpha: f32) void {
        self.alpha = if (alpha < 0.0) 0.0 else if (alpha > 1.0) 1.0 else alpha;
    }

    pub fn setIntegralLimits(self: *Pid, integral_min: f32, integral_max: f32) void {
        self.integral_min = integral_min;
        self.integral_max = integral_max;
    }

    pub fn setSetpoint(self: *Pid, setpoint: f32) void {
        self.setpoint = setpoint;
    }

    /// Clears the history. The gains, the setpoint and the limits stay.
    pub fn reset(self: *Pid) void {
        self.integral = 0.0;
        self.prev_error = 0.0;
        self.prev_prev_error = 0.0;
        self.prev_measurement = 0.0;
        self.prev_output = 0.0;
    }

    pub fn computePosition(self: *Pid, measurement: f32) f32 {
        const err = self.setpoint - measurement;

        const p_term = self.kp * err;

        self.integral += err * self.dt;
        if (self.integral > self.integral_max) {
            self.integral = self.integral_max;
        } else if (self.integral < self.integral_min) {
            self.integral = self.integral_min;
        }
        const i_term = self.ki * self.integral;

        // Both terms are kept, including their rounding, because the corpus
        // records what the C code produced bit for bit (see the module comment).
        const derivative = (measurement - self.prev_measurement) / self.dt;
        const filtered_derivative = self.alpha * derivative +
            (1.0 - self.alpha) * (self.prev_error - err) / self.dt;
        const d_term = -self.kd * filtered_derivative;

        self.prev_error = err;
        self.prev_measurement = measurement;

        var output = p_term + i_term + d_term;
        if (output > self.output_max) {
            output = self.output_max;
        } else if (output < self.output_min) {
            output = self.output_min;
        }
        return output;
    }

    pub fn computeIncremental(self: *Pid, measurement: f32) f32 {
        const err = self.setpoint - measurement;

        const delta_p = self.kp * (err - self.prev_error);
        const delta_i = self.ki * err * self.dt;
        // Note the dt squared: for a small step this term dominates, which is
        // why the incremental run in the corpus saturates on every sample.
        const delta_d = self.kd * (err - 2.0 * self.prev_error + self.prev_prev_error) / (self.dt * self.dt);

        const delta_u = delta_p + delta_i + delta_d;

        self.prev_prev_error = self.prev_error;
        self.prev_error = err;

        var output = self.prev_output + delta_u;
        if (output > self.output_max) {
            output = self.output_max;
        } else if (output < self.output_min) {
            output = self.output_min;
        }
        self.prev_output = output;
        return output;
    }

    pub fn compute(self: *Pid, measurement: f32) f32 {
        return switch (self.kind) {
            .position => self.computePosition(measurement),
            .incremental => self.computeIncremental(measurement),
        };
    }
};

// --- tests ------------------------------------------------------------------

const measurements = [_]f32{ 0.0, 0.5, 1.2, 0.9, 1.1, 1.0 };

fn runPosition(pid: *Pid) [measurements.len]f32 {
    var out: [measurements.len]f32 = undefined;
    for (measurements, 0..) |m, i| out[i] = pid.compute(m);
    return out;
}

test "pid: the position form matches the C answers" {
    const corpus = try corpus_mod.Corpus.load();

    var pid = Pid.init(.position, 2.0, 0.5, 0.1, 0.01, -10.0, 10.0);
    pid.setSetpoint(1.0);
    const out = runPosition(&pid);
    try corpus.expectValues("pid_position_run", &out);

    try corpus.expectValue("pid_position_integral_after", pid.integral);
    try corpus.expectValue("pid_position_prev_error_after", pid.prev_error);
    try corpus.expectValue("pid_position_prev_measurement_after", pid.prev_measurement);
}

test "pid: the incremental form matches the C answers" {
    const corpus = try corpus_mod.Corpus.load();

    var pid = Pid.init(.incremental, 2.0, 0.5, 0.1, 0.01, -10.0, 10.0);
    pid.setSetpoint(1.0);
    var out: [measurements.len]f32 = undefined;
    for (measurements, 0..) |m, i| out[i] = pid.compute(m);
    try corpus.expectValues("pid_incremental_run", &out);

    try corpus.expectValue("pid_incremental_prev_output_after", pid.prev_output);
    try corpus.expectValue("pid_incremental_prev_error_after", pid.prev_error);
    try corpus.expectValue("pid_incremental_prev_prev_error_after", pid.prev_prev_error);
}

test "pid: alpha changes only the first sample, which is the point" {
    const corpus = try corpus_mod.Corpus.load();

    var runs: [3][measurements.len]f32 = undefined;
    for ([_]f32{ 0.0, 0.7, 1.0 }, 0..) |alpha, i| {
        var pid = Pid.init(.position, 2.0, 0.5, 0.1, 0.01, -10.0, 10.0);
        pid.setDerivativeFilter(alpha);
        pid.setSetpoint(1.0);
        runs[i] = runPosition(&pid);
    }

    try corpus.expectValues("pid_position_alpha0_run", &runs[0]);
    try corpus.expectValues("pid_position_alpha07_run", &runs[1]);
    try corpus.expectValues("pid_position_alpha1_run", &runs[2]);

    // The finding, asserted rather than described: the first output depends on
    // alpha, and from the second sample on the runs agree.
    try std.testing.expect(runs[0][0] != runs[1][0]);
    try std.testing.expect(runs[1][0] != runs[2][0]);
    for (1..measurements.len) |i| {
        try std.testing.expectEqual(runs[0][i], runs[1][i]);
        try std.testing.expectEqual(runs[1][i], runs[2][i]);
    }

    // And the coefficient itself is clamped, as in C.
    var pid = Pid.init(.position, 1, 1, 1, 0.01, -1, 1);
    pid.setDerivativeFilter(-1.0);
    try corpus.expectValue("pid_alpha_clamped_low", pid.alpha);
    pid.setDerivativeFilter(2.0);
    try corpus.expectValue("pid_alpha_clamped_high", pid.alpha);
}

test "pid: integral and output limits match the C answers" {
    const corpus = try corpus_mod.Corpus.load();

    // Anti-windup: with the integral limits tighter than the output limits the
    // integral stops growing, and the output follows it.
    var pid = Pid.init(.position, 0.0, 1.0, 0.0, 0.01, -10.0, 10.0);
    pid.setIntegralLimits(-0.5, 0.5);
    pid.setSetpoint(10.0);
    var out: [measurements.len]f32 = undefined;
    for (0..measurements.len) |i| out[i] = pid.compute(0.0);
    try corpus.expectValues("pid_position_integral_clamped_run", &out);
    try corpus.expectValue("pid_integral_after_clamp", pid.integral);

    var clamped = Pid.init(.position, 100.0, 0.0, 0.0, 0.01, -1.0, 1.0);
    clamped.setSetpoint(1.0);
    for (measurements, 0..) |m, i| out[i] = clamped.compute(m);
    try corpus.expectValues("pid_position_output_clamped_run", &out);

    // Independent of the corpus: the output never leaves its limits.
    for (out) |v| try std.testing.expect(v >= -1.0 and v <= 1.0);
}

test "pid: reset matches the C answers" {
    const corpus = try corpus_mod.Corpus.load();

    var pid = Pid.init(.position, 2.0, 0.5, 0.1, 0.01, -10.0, 10.0);
    pid.setSetpoint(1.0);
    for (measurements[0..3]) |m| _ = pid.compute(m);
    pid.reset();

    try corpus.expectValue("pid_integral_after_reset", pid.integral);
    try corpus.expectValue("pid_prev_error_after_reset", pid.prev_error);
    try corpus.expectValue("pid_prev_output_after_reset", pid.prev_output);

    const out = runPosition(&pid);
    try corpus.expectValues("pid_run_after_reset", &out);
}
