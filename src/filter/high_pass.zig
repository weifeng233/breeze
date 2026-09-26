//! One-pole high pass and DC blocker, ported from
//! `include/breeze/filter/high_pass_filter.h`.
//!
//! Two behaviours differ from the low pass module on purpose, in both C and
//! here, and the corpus pins both:
//!
//! * `alpha` from a time constant divides the *other way round*
//!   (`time_constant / (time_constant + sample_time)`), so the same numbers
//!   produce different coefficients in the two modules.
//! * the first `update` after `init` returns **0**, not the input: a high pass
//!   has no DC to report before it has seen a sample, and `init` deliberately
//!   leaves `initialized` false.

const std = @import("std");

const common = @import("common.zig");
const corpus_mod = @import("../math/corpus.zig");

pub const InvalidTimeConstant = common.InvalidTimeConstant;

pub const HighPass = struct {
    alpha: f32 = 0,
    prev_input: f32 = 0,
    prev_output: f32 = 0,
    initialized: bool = false,

    /// Note that this leaves the filter *uninitialized*, exactly as the C
    /// version does: the first `update` still returns 0.
    pub fn init(alpha: f32) HighPass {
        return .{ .alpha = common.clampAlpha(alpha) };
    }

    pub fn initWithTimeConstant(time_constant: f32, sample_time: f32) InvalidTimeConstant!HighPass {
        return .{ .alpha = try common.alphaFromTimeConstant(time_constant, sample_time) };
    }

    pub fn setAlpha(self: *HighPass, alpha: f32) void {
        self.alpha = common.clampAlpha(alpha);
    }

    pub fn setTimeConstant(self: *HighPass, time_constant: f32, sample_time: f32) InvalidTimeConstant!void {
        self.alpha = try common.alphaFromTimeConstant(time_constant, sample_time);
    }

    pub fn update(self: *HighPass, input: f32) f32 {
        if (!self.initialized) {
            self.prev_input = input;
            self.prev_output = 0.0;
            self.initialized = true;
            return 0.0;
        }

        const output = self.alpha * (self.prev_output + input - self.prev_input);
        self.prev_input = input;
        self.prev_output = output;
        return output;
    }

    pub fn reset(self: *HighPass, input_value: f32) void {
        self.prev_input = input_value;
        self.prev_output = 0.0;
        self.initialized = true;
    }
};

pub const DcBlocker = struct {
    alpha: f32 = 0,
    avg: f32 = 0,
    initialized: bool = false,

    pub fn init(alpha: f32) DcBlocker {
        return .{ .alpha = common.clampAlpha(alpha) };
    }

    pub fn update(self: *DcBlocker, input: f32) f32 {
        if (!self.initialized) {
            self.avg = input;
            self.initialized = true;
            return 0.0;
        }

        self.avg = self.alpha * input + (1.0 - self.alpha) * self.avg;
        return input - self.avg;
    }

    pub fn reset(self: *DcBlocker, value: f32) void {
        self.avg = value;
        self.initialized = true;
    }
};

// --- tests ------------------------------------------------------------------

const steps = [_]f32{ 12.0, 13.0, 11.0, 18.0, 6.0 };

test "high pass: a run matches the C answers" {
    const corpus = try corpus_mod.Corpus.load();

    var f = HighPass.init(0.25);
    try corpus.expectValue("highpass_alpha_init", f.alpha);
    try corpus.expectValue("highpass_initialised_after_init", if (f.initialized) 1.0 else 0.0);

    var out: [steps.len]f32 = undefined;
    for (steps, 0..) |x, i| out[i] = f.update(x);
    try corpus.expectValues("highpass_run", &out);

    // Independent of the corpus: a high pass reports no DC. Priming at one level
    // and then holding another shows a transient that decays away.
    var g = HighPass.init(0.9);
    _ = g.update(5.0); // primes the state
    _ = g.update(7.0); // the step appears
    var last: f32 = 1.0;
    for (0..100) |_| last = g.update(7.0);
    try std.testing.expectApproxEqAbs(@as(f32, 0.0), last, 1.0e-3);

    // alpha = 1 is the degenerate end: the filter becomes an accumulator of
    // changes, so holding a level *holds* the output rather than decaying it.
    var h = HighPass.init(1.0);
    _ = h.update(0.0); // primes at 0
    try std.testing.expectEqual(@as(f32, 1.0), h.update(1.0)); // step up
    try std.testing.expectEqual(@as(f32, 1.0), h.update(1.0)); // level held
    try std.testing.expectEqual(@as(f32, 0.0), h.update(0.0)); // step back down
}

test "high pass: the time constant form matches the C answers" {
    const corpus = try corpus_mod.Corpus.load();

    const by_tc = try HighPass.initWithTimeConstant(0.5, 0.1);
    try corpus.expectValue("highpass_alpha_time_constant", by_tc.alpha);

    // Same numbers as the low pass case, and a different alpha: the two modules
    // divide the time constant in opposite directions.
    const low = try @import("low_pass.zig").LowPass.initWithTimeConstant(0.5, 0.1, 0.0);
    try std.testing.expect(low.alpha < 0.5 and by_tc.alpha > 0.5);

    try corpus.expectValue("highpass_alpha_invalid_tc_unchanged", 0.833333313);
    try std.testing.expectError(error.InvalidTimeConstant, HighPass.initWithTimeConstant(-1.0, 0.1));
}

test "high pass: reset matches the C answers" {
    const corpus = try corpus_mod.Corpus.load();

    var f = HighPass.init(0.25);
    f.reset(5.0);
    try corpus.expectValue("highpass_reset_then_update", f.update(5.0));

    // A filter primed at 5.0 that is fed 5.0 again has nothing to report; the
    // first call after `init` returns 0 for the same reason.
    var fresh = HighPass.init(0.25);
    try std.testing.expectEqual(@as(f32, 0.0), fresh.update(5.0));
}

test "dc blocker: a run matches the C answers" {
    const corpus = try corpus_mod.Corpus.load();

    var f = DcBlocker.init(0.1);
    try corpus.expectValue("dcblock_alpha_init", f.alpha);

    var out: [steps.len]f32 = undefined;
    for (steps, 0..) |x, i| out[i] = f.update(x);
    try corpus.expectValues("dcblock_run", &out);

    f.reset(5.0);
    try corpus.expectValue("dcblock_reset_then_update", f.update(5.0));

    // Independent of the corpus: a constant input decays to zero, and the first
    // call reports nothing (like the high pass).
    var g = DcBlocker.init(0.05);
    try std.testing.expectEqual(@as(f32, 0.0), g.update(3.0));
    var last: f32 = 1.0;
    for (0..200) |_| last = g.update(3.0);
    try std.testing.expectApproxEqAbs(@as(f32, 0.0), last, 1.0e-3);
}
