//! PID controllers, ported from `include/breeze/control/pid_controller.h`.
//!
//! Two forms share a state struct: the position (absolute) form and the
//! incremental (velocity) form. The C version keeps both in one type selected by
//! a `type` field, and that is kept here as `Kind` - a caller can still switch,
//! and `compute` still dispatches on it.
//!
//! ## The derivative filter did not filter (§34), and the C's own comments say so
//!
//! The C wrote
//!
//!     derivative          = (measurement - prev_measurement) / dt
//!     filtered_derivative = alpha·derivative + (1 - alpha)·(prev_error - error) / dt
//!
//! and for a fixed setpoint `(prev_error - error) / dt` **is** `(measurement -
//! prev_measurement) / dt` - the same quantity - so the expression is a convex
//! combination of a value with itself. `alpha` therefore changed only the first
//! sample, where `prev_error` is still its initial zero, and otherwise the last
//! bit of the rounding.
//!
//! That is not what the header says it is. The same file calls `alpha` the
//! *"微分滤波系数"* and documents `@param alpha` as *"值越低滤波效果越强"* - lower
//! means stronger filtering - and the term above it is *"基于测量值以避免微分突跳"*,
//! derivative on the measurement to avoid a setpoint kick. Those three statements
//! describe a one-pole low-pass on the measurement derivative:
//!
//!     filtered = alpha · derivative + (1 - alpha) · filtered_previous
//!
//! which is the same shape as the line the C wrote, with the **previous filtered
//! value** where the error-derivative sits. The struct has no field to keep that
//! value, and that missing state is exactly why the expression degenerates: it is a
//! low-pass with its feedback term replaced by something that happens to be equal
//! to its input.
//!
//! So the port keeps the documented convention (`alpha = 1` is no filtering,
//! `alpha = 0` is the strongest) and gives the filter the state it needs. The
//! error-derivative term is gone; it was the raw signal the filter was supposed to
//! be smoothing. REVIEW §53 has the before/after and the corpus churn.
//!
//! In the incremental form `alpha` is still not read at all - the C does not read
//! it there either, and that is a separate shape, not a filter that does nothing.

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

    /// The derivative filter's state: the last value it produced. The C has no
    /// field for this, which is why its filter could not filter.
    filtered_derivative: f32 = 0,

    output_min: f32,
    output_max: f32,

    integral_min: f32,
    integral_max: f32,

    /// Derivative filter coefficient, 0..1. **Lower means stronger filtering**,
    /// as the C header documents: 1 is the raw measurement derivative, 0 freezes
    /// the derivative at its last value.
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
        self.filtered_derivative = 0.0;
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

        // Derivative on the measurement, so a setpoint step does not kick the
        // output, then one pole of low-pass with `alpha` as the weight on the raw
        // value (see the module comment for why that is the direction the C header
        // documents). The read of `filtered_derivative` is the *previous* value:
        // Zig evaluates the right-hand side before the assignment.
        const derivative = (measurement - self.prev_measurement) / self.dt;
        const previous_filtered = self.filtered_derivative;
        self.filtered_derivative = self.alpha * derivative + (1.0 - self.alpha) * previous_filtered;
        const d_term = -self.kd * self.filtered_derivative;

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

test "pid: alpha filters the derivative, in the direction the C header documents" {
    const corpus = try corpus_mod.Corpus.load();

    var runs: [3][measurements.len]f32 = undefined;
    for ([_]f32{ 0.0, 0.7, 1.0 }, 0..) |alpha, i| {
        var pid = Pid.init(.position, 2.0, 0.5, 0.1, 0.01, -10.0, 10.0);
        pid.setDerivativeFilter(alpha);
        pid.setSetpoint(1.0);
        runs[i] = runPosition(&pid);
    }

    // These three moved when the filter learned to filter: before the fix they
    // differed only in the first sample. Re-locked, not re-derived - see REVIEW §53.
    try corpus.expectValues("pid_position_alpha0_run", &runs[0]);
    try corpus.expectValues("pid_position_alpha07_run", &runs[1]);
    try corpus.expectValues("pid_position_alpha1_run", &runs[2]);

    // The knob now does something wherever the derivative exists. From the second
    // sample on the measurement has moved, so the raw derivative is non-zero and
    // the two extremes disagree; on the first sample it has not moved yet, so no
    // filter can say anything - which is why this starts at 1.
    for (1..measurements.len) |i| {
        try std.testing.expect(runs[0][i] != runs[2][i]);
    }
    try std.testing.expectEqual(runs[0][0], runs[2][0]);

    // alpha = 1 is no filtering at all: the state is the raw measurement
    // derivative. Between 0.0 and 0.5 with dt = 0.01 that is 50.
    var unfiltered = Pid.init(.position, 0.0, 0.0, 0.1, 0.01, -10.0, 10.0);
    unfiltered.setDerivativeFilter(1.0);
    unfiltered.setSetpoint(1.0);
    _ = unfiltered.compute(0.0);
    _ = unfiltered.compute(0.5);
    try std.testing.expectApproxEqAbs(@as(f32, 50.0), unfiltered.filtered_derivative, 1.0e-3);

    // alpha = 0 is the strongest filtering, which in this form means the state
    // never leaves zero - so the D term is exactly zero for every sample, and a
    // controller with ki = 0 behaves as if kd were zero.
    var frozen = Pid.init(.position, 2.0, 0.0, 0.1, 0.01, -10.0, 10.0);
    frozen.setDerivativeFilter(0.0);
    frozen.setSetpoint(1.0);
    var p_only = Pid.init(.position, 2.0, 0.0, 0.0, 0.01, -10.0, 10.0);
    p_only.setSetpoint(1.0);
    for (measurements) |m| {
        try std.testing.expectEqual(p_only.compute(m), frozen.compute(m));
    }
    try std.testing.expectEqual(@as(f32, 0.0), frozen.filtered_derivative);

    // The low-pass itself, on a step and back: each sample keeps (1 - alpha) of
    // the previous filtered value.
    var lp = Pid.init(.position, 0.0, 0.0, 0.0, 0.01, -10.0, 10.0);
    lp.setDerivativeFilter(0.25);
    lp.setSetpoint(0.0);
    _ = lp.compute(0.0); // flat: raw 0
    _ = lp.compute(0.4); // raw jumps to 40
    try std.testing.expectApproxEqAbs(@as(f32, 10.0), lp.filtered_derivative, 1.0e-4);
    _ = lp.compute(0.4); // raw back to 0, the state keeps 0.75 of 10
    try std.testing.expectApproxEqAbs(@as(f32, 7.5), lp.filtered_derivative, 1.0e-4);

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
