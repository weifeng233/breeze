//! One-dimensional Kalman filter, ported from
//! `include/breeze/filter/kalman_filter.h`.
//!
//! The filter is stateful, so `update` takes `self: *Self`. Nothing is clamped
//! or validated in the C version and nothing is here either: a Kalman filter
//! with a nonsensical `r` produces nonsense, and inventing a policy for that
//! would be a design change rather than a port.
//!
//! **What is deliberately not here**: the header also declares
//! `BreezeKalmanFilterND`, a 4-dimensional type with `dim`/`meas_dim` and six
//! 4x4 matrices - and not one function. Its comment says the implementation is
//! waiting for the math module, which now exists, but writing that filter is a
//! feature, not a migration: there is no behaviour to match, so there is nothing
//! to check a port against. The corpus records the type's layout
//! (`kalman_nd_layout 408 4`) so the gap is visible rather than forgotten.

const std = @import("std");

const corpus_mod = @import("../math/corpus.zig");

pub const Kalman1D = struct {
    x: f32,
    p: f32,
    q: f32,
    r: f32,
    k: f32,
    /// State transition coefficient; 1 means "the state does not change".
    a: f32,
    /// Measurement coefficient; 1 means "the measurement is the state".
    h: f32,

    pub fn init(q: f32, r: f32, p_init: f32, x_init: f32) Kalman1D {
        return .{
            .x = x_init,
            .p = p_init,
            .q = q,
            .r = r,
            .k = 0.0,
            .a = 1.0,
            .h = 1.0,
        };
    }

    pub fn setStateTransition(self: *Kalman1D, a: f32) void {
        self.a = a;
    }

    pub fn setMeasurementCoefficient(self: *Kalman1D, h: f32) void {
        self.h = h;
    }

    /// Predict, then correct with `measurement`; returns the new state estimate.
    pub fn update(self: *Kalman1D, measurement: f32) f32 {
        // Predict.
        self.x = self.a * self.x;
        self.p = self.a * self.a * self.p + self.q;

        // Correct.
        self.k = self.p * self.h / (self.h * self.p * self.h + self.r);
        self.x = self.x + self.k * (measurement - self.h * self.x);
        self.p = (1.0 - self.k * self.h) * self.p;

        return self.x;
    }

    pub fn state(self: Kalman1D) f32 {
        return self.x;
    }

    pub fn covariance(self: Kalman1D) f32 {
        return self.p;
    }

    pub fn gain(self: Kalman1D) f32 {
        return self.k;
    }
};

// --- tests ------------------------------------------------------------------

const measurements = [_]f32{ 1.0, 2.0, 3.0, 2.5, 4.0 };

test "kalman: construction matches the C answers" {
    const corpus = try corpus_mod.Corpus.load();

    const f = Kalman1D.init(0.01, 0.1, 1.0, 0.0);
    try corpus.expectValue("kalman_x_init", f.x);
    try corpus.expectValue("kalman_p_init", f.p);
    try corpus.expectValue("kalman_q_init", f.q);
    try corpus.expectValue("kalman_r_init", f.r);
    try corpus.expectValue("kalman_k_init", f.k);
    try corpus.expectValue("kalman_a_init", f.a);
    try corpus.expectValue("kalman_h_init", f.h);

    // The 4x4 type the header declares and never implements. Nothing here reads
    // its layout; the corpus keeps it so the omission is on the record.
    try corpus.expectValues("kalman_nd_layout", &.{ 408, 4 });
}

test "kalman: a run matches the C answers" {
    const corpus = try corpus_mod.Corpus.load();

    var f = Kalman1D.init(0.01, 0.1, 1.0, 0.0);
    var out: [measurements.len]f32 = undefined;
    for (measurements, 0..) |m, i| out[i] = f.update(m);
    try corpus.expectValues("kalman_run", &out);

    try corpus.expectValue("kalman_x_after", f.state());
    try corpus.expectValue("kalman_p_after", f.covariance());
    try corpus.expectValue("kalman_k_after", f.gain());

    // Independent of the corpus: with a lot of process noise and little
    // measurement noise the filter tracks the measurement closely; with the
    // opposite ratio it barely moves.
    var trusting = Kalman1D.init(100.0, 0.001, 1.0, 0.0);
    var tracked: f32 = 0;
    for (0..20) |_| tracked = trusting.update(5.0);
    try std.testing.expectApproxEqAbs(@as(f32, 5.0), tracked, 1.0e-2);

    var sceptical = Kalman1D.init(0.0001, 1000.0, 1.0, 0.0);
    var ignored: f32 = 0;
    for (0..20) |_| ignored = sceptical.update(5.0);
    try std.testing.expect(ignored < 0.5);

    // The gain is bounded by 1, and the covariance stays positive.
    var g = Kalman1D.init(0.01, 0.1, 1.0, 0.0);
    for (0..50) |_| {
        _ = g.update(1.0);
        try std.testing.expect(g.gain() > 0.0 and g.gain() <= 1.0);
        try std.testing.expect(g.covariance() > 0.0);
    }
}

test "kalman: the settable coefficients match the C answers" {
    const corpus = try corpus_mod.Corpus.load();

    var f = Kalman1D.init(0.01, 0.1, 1.0, 0.0);
    f.setStateTransition(0.9);
    var out: [measurements.len]f32 = undefined;
    for (measurements, 0..) |m, i| out[i] = f.update(m);
    try corpus.expectValues("kalman_transition_run", &out);
    try corpus.expectValue("kalman_a_after_set", f.a);

    var g = Kalman1D.init(0.01, 0.1, 1.0, 0.0);
    g.setMeasurementCoefficient(2.0);
    for (measurements, 0..) |m, i| out[i] = g.update(m);
    try corpus.expectValues("kalman_measurement_run", &out);
    try corpus.expectValue("kalman_h_after_set", g.h);

    // A measurement coefficient of 2 halves the apparent measurement, so the
    // converged estimate sits near half of it.
    var converged: f32 = 0;
    for (0..200) |_| converged = g.update(4.0);
    try std.testing.expectApproxEqAbs(@as(f32, 2.0), converged, 0.05);
}
