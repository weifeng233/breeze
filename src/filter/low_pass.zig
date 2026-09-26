//! One-pole low pass and EWMA filters, ported from
//! `include/breeze/filter/low_pass_filter.h`.
//!
//! Both are stateful, so unlike the math ports these take `self: *Self` and
//! mutate. What changed is only what C's null checks were standing in for: there
//! is no pointer to be null (`if (!filter) return input;` has no counterpart),
//! and an invalid time constant is an error rather than a silent no-op (see
//! `common.zig`).
//!
//! The `initialized` flag is kept, including its odd consequence: a
//! default-constructed filter (`LowPass{}`) has never been initialized, so its
//! first `update` passes the input straight through. That is reachable in C via
//! `BreezeLowPassFilter f = {0}` and is pinned by the corpus.

const std = @import("std");

const common = @import("common.zig");
const corpus_mod = @import("../math/corpus.zig");

pub const InvalidTimeConstant = common.InvalidTimeConstant;

pub const LowPass = struct {
    alpha: f32 = 0,
    prev_output: f32 = 0,
    initialized: bool = false,

    /// `alpha` is clamped into `[0, 1]`, as every C entry point does.
    pub fn init(alpha: f32, initial_value: f32) LowPass {
        return .{
            .alpha = common.clampAlpha(alpha),
            .prev_output = initial_value,
            .initialized = true,
        };
    }

    pub fn initWithTimeConstant(
        time_constant: f32,
        sample_time: f32,
        initial_value: f32,
    ) InvalidTimeConstant!LowPass {
        return .{
            .alpha = try common.alphaFromSampleTime(time_constant, sample_time),
            .prev_output = initial_value,
            .initialized = true,
        };
    }

    pub fn setAlpha(self: *LowPass, alpha: f32) void {
        self.alpha = common.clampAlpha(alpha);
    }

    pub fn setTimeConstant(self: *LowPass, time_constant: f32, sample_time: f32) InvalidTimeConstant!void {
        self.alpha = try common.alphaFromSampleTime(time_constant, sample_time);
    }

    pub fn update(self: *LowPass, input: f32) f32 {
        if (!self.initialized) {
            self.prev_output = input;
            self.initialized = true;
            return input;
        }

        const output = self.alpha * input + (1.0 - self.alpha) * self.prev_output;
        self.prev_output = output;
        return output;
    }

    pub fn reset(self: *LowPass, value: f32) void {
        self.prev_output = value;
        self.initialized = true;
    }
};

pub const Ewma = struct {
    alpha: f32 = 0,
    avg: f32 = 0,
    initialized: bool = false,

    pub fn init(alpha: f32, initial_value: f32) Ewma {
        return .{
            .alpha = common.clampAlpha(alpha),
            .avg = initial_value,
            .initialized = true,
        };
    }

    pub fn update(self: *Ewma, input: f32) f32 {
        if (!self.initialized) {
            self.avg = input;
            self.initialized = true;
            return input;
        }

        self.avg = self.alpha * input + (1.0 - self.alpha) * self.avg;
        return self.avg;
    }

    pub fn reset(self: *Ewma, value: f32) void {
        self.avg = value;
        self.initialized = true;
    }
};

// --- tests ------------------------------------------------------------------

/// The input sequence every run case in the corpus uses.
const steps = [_]f32{ 12.0, 13.0, 11.0, 18.0, 6.0 };

test "low pass: a run matches the C answers" {
    const corpus = try corpus_mod.Corpus.load();

    var f = LowPass.init(0.25, 10.0);
    try corpus.expectValue("lowpass_alpha_init", f.alpha);
    try corpus.expectValue("lowpass_prev_output_init", f.prev_output);

    var out: [steps.len]f32 = undefined;
    for (steps, 0..) |x, i| out[i] = f.update(x);
    try corpus.expectValues("lowpass_run", &out);

    // Independent of the corpus: a one-pole low pass never overshoots and moves
    // monotonically towards the input.
    var g = LowPass.init(0.5, 0.0);
    var previous: f32 = 0;
    for ([_]f32{ 1, 1, 1, 1, 1 }) |x| {
        const y = g.update(x);
        try std.testing.expect(y >= previous and y <= x);
        previous = y;
    }
    // alpha = 1 means "trust the input entirely".
    var h = LowPass.init(1.0, 0.0);
    try std.testing.expectEqual(@as(f32, 5.0), h.update(5.0));
    // alpha = 0 means "ignore the input entirely".
    var k = LowPass.init(0.0, 3.0);
    try std.testing.expectEqual(@as(f32, 3.0), k.update(5.0));
}

test "low pass: alpha clamping and the time constant form match the C answers" {
    const corpus = try corpus_mod.Corpus.load();

    try corpus.expectValue("lowpass_alpha_clamped_low", LowPass.init(-1.0, 0.0).alpha);
    try corpus.expectValue("lowpass_alpha_clamped_high", LowPass.init(2.0, 0.0).alpha);

    const by_tc = try LowPass.initWithTimeConstant(0.5, 0.1, 0.0);
    try corpus.expectValue("lowpass_alpha_time_constant", by_tc.alpha);

    var f = LowPass.init(0.25, 0.0);
    f.setAlpha(0.5);
    try corpus.expectValue("lowpass_alpha_set", f.alpha);
    try f.setTimeConstant(0.5, 0.5);
    try corpus.expectValue("lowpass_alpha_set_time_constant", f.alpha);

    // The C version does nothing at all here; the corpus records that, and the
    // port's answer is an error instead.
    try corpus.expectValue("lowpass_alpha_invalid_tc_unchanged", 0.25);
    try corpus.expectValue("lowpass_prev_output_invalid_tc_unchanged", 10.0);
    try corpus.expectValue("lowpass_alpha_invalid_set_tc_unchanged", 0.25);
    try std.testing.expectError(error.InvalidTimeConstant, LowPass.initWithTimeConstant(-1.0, 0.1, 99.0));
    var g = LowPass.init(0.25, 10.0);
    try std.testing.expectError(error.InvalidTimeConstant, g.setTimeConstant(0.0, 0.1));
    // A failed setTimeConstant leaves alpha alone, which is the C behaviour too.
    try std.testing.expectEqual(@as(f32, 0.25), g.alpha);
}

test "low pass: the uninitialised and reset paths match the C answers" {
    const corpus = try corpus_mod.Corpus.load();

    // `LowPass{}` is the C `= {0}` struct: never initialized.
    var fresh = LowPass{};
    try corpus.expectValue("lowpass_first_update_uninitialised", fresh.update(7.5));
    try std.testing.expect(fresh.initialized);

    var f = LowPass.init(0.5, 0.0);
    f.reset(4.0);
    try corpus.expectValue("lowpass_reset_then_update", f.update(8.0));

    // 0.5 * 8 + 0.5 * 4 = 6
    var g = LowPass.init(0.5, 4.0);
    try std.testing.expectEqual(@as(f32, 6.0), g.update(8.0));
}

test "ewma: a run matches the C answers" {
    const corpus = try corpus_mod.Corpus.load();

    var f = Ewma.init(0.25, 2.0);
    try corpus.expectValue("ewma_alpha_init", f.alpha);
    try corpus.expectValue("ewma_avg_init", f.avg);

    var out: [steps.len]f32 = undefined;
    for (steps, 0..) |x, i| out[i] = f.update(x);
    try corpus.expectValues("ewma_run", &out);

    f.reset(3.0);
    try corpus.expectValue("ewma_reset_then_update", f.update(9.0));

    var fresh = Ewma{};
    try corpus.expectValue("ewma_first_update_uninitialised", fresh.update(7.5));

    // EWMA and the one-pole low pass are the same recurrence under different
    // names; the corpus run values agree with each other only because the
    // initial values differ, which is worth knowing rather than assuming.
    var lp = LowPass.init(0.25, 2.0);
    var ew = Ewma.init(0.25, 2.0);
    for (steps) |x| {
        try std.testing.expectEqual(lp.update(x), ew.update(x));
    }
}
