//! Fuzzy controller, ported from `include/breeze/control/fuzzy_controller.h`.
//!
//! A Mamdani controller: membership functions per variable, a rule table, min for
//! AND, truncated max for OR, and centroid defuzzification over a discretised
//! output grid.
//!
//! **The allocations are gone, and there were two.** The C version `malloc`s the
//! discretisation grid on the first `compute` call and caches it, then `malloc`s
//! and `free`s a second array of per-grid-point memberships **on every call** -
//! a heap allocation in a control loop. Here the level is a comptime parameter,
//! so both arrays are fields: `grid` is built once in `init` and `scratch` is
//! reused. The port allocates nothing at all.
//!
//! That also fixes two things the C version cannot express:
//!
//! * `discretization_level` of 1 divides by zero when the grid step is computed
//!   (`(max - min) / (level - 1)`). Here the level is a comptime parameter with a
//!   `@compileError` below 2, so the case cannot arise.
//! * the cached grid goes stale if `output_min`/`output_max` change after the
//!   first call, because it is only built once and never rebuilt. The grid is
//!   built in `init` here, so the ranges and the grid cannot disagree.
//!
//! Two fields are dropped because nothing reads them, the same rule as
//! `steering_ratio` (REVIEW §38) and the LQR model (§33): `BreezeFuzzyMembership`
//! carries a `name` that no code touches, and the three membership *counts* are
//! stored but never used - the Zig slices carry their own length, so a count and
//! a pointer cannot drift apart.

const std = @import("std");

const corpus_mod = @import("../math/corpus.zig");

pub const MembershipKind = enum { triangular, trapezoidal, gaussian };

pub const Membership = struct {
    kind: MembershipKind,
    /// `[a, b, c]` for triangular, `[a, b, c, d]` for trapezoidal, `[centre,
    /// sigma]` for gaussian. The unused tail is zero.
    params: [4]f32 = @splat(0),
};

pub const Rule = struct {
    input1: usize,
    input2: usize,
    output: usize,
};

pub fn triangular(x: f32, a: f32, b: f32, c: f32) f32 {
    if (x <= a or x >= c) return 0.0;
    if (x <= b) return (x - a) / (b - a);
    return (c - x) / (c - b);
}

pub fn trapezoidal(x: f32, a: f32, b: f32, c: f32, d: f32) f32 {
    if (x <= a or x >= d) return 0.0;
    if (x >= b and x <= c) return 1.0;
    if (x < b) return (x - a) / (b - a);
    return (d - x) / (d - c);
}

pub fn gaussian(x: f32, centre: f32, sigma: f32) f32 {
    const temp = (x - centre) / sigma;
    return @exp(-0.5 * temp * temp);
}

pub fn membershipValue(m: Membership, x: f32) f32 {
    return switch (m.kind) {
        .triangular => triangular(x, m.params[0], m.params[1], m.params[2]),
        .trapezoidal => trapezoidal(x, m.params[0], m.params[1], m.params[2], m.params[3]),
        .gaussian => gaussian(x, m.params[0], m.params[1]),
    };
}

/// A fuzzy controller with `level` points in its discretised output space.
///
/// `level` must be at least 2; the C version has no such requirement and divides
/// by `level - 1`.
pub fn Fuzzy(comptime level: usize) type {
    if (level < 2) @compileError("a fuzzy controller needs at least two discretisation points");

    return struct {
        input1_min: f32,
        input1_max: f32,
        input2_min: f32,
        input2_max: f32,
        output_min: f32,
        output_max: f32,

        input1: []const Membership,
        input2: []const Membership,
        output: []const Membership,
        rules: []const Rule,

        /// The discretised output space, built once. The C version builds it
        /// lazily on the first `compute` and caches it.
        grid: [level]f32,
        /// The per-grid-point memberships the C version `malloc`s on every call.
        scratch: [level]f32 = @splat(0),

        const Self = @This();

        pub fn init(
            input1_min: f32,
            input1_max: f32,
            input2_min: f32,
            input2_max: f32,
            output_min: f32,
            output_max: f32,
            input1: []const Membership,
            input2: []const Membership,
            output: []const Membership,
            rules: []const Rule,
        ) Self {
            var self = Self{
                .input1_min = input1_min,
                .input1_max = input1_max,
                .input2_min = input2_min,
                .input2_max = input2_max,
                .output_min = output_min,
                .output_max = output_max,
                .input1 = input1,
                .input2 = input2,
                .output = output,
                .rules = rules,
                .grid = undefined,
            };

            const step = (output_max - output_min) / @as(f32, @floatFromInt(level - 1));
            for (&self.grid, 0..) |*g, i| {
                g.* = output_min + @as(f32, @floatFromInt(i)) * step;
            }

            return self;
        }

        /// Defuzzified output for one pair of inputs.
        ///
        /// Not re-entrant: `scratch` is part of the value. In this kernel a
        /// controller is called from one task, and the C version's per-call
        /// allocation is what that buys - a heap in the control loop.
        pub fn compute(self: *Self, input1_in: f32, input2_in: f32) f32 {
            if (self.rules.len == 0) return 0.0;

            var input1 = input1_in;
            var input2 = input2_in;
            if (input1 < self.input1_min) input1 = self.input1_min;
            if (input1 > self.input1_max) input1 = self.input1_max;
            if (input2 < self.input2_min) input2 = self.input2_min;
            if (input2 > self.input2_max) input2 = self.input2_max;

            @memset(&self.scratch, 0.0);

            for (self.rules) |rule| {
                const m1 = membershipValue(self.input1[rule.input1], input1);
                const m2 = membershipValue(self.input2[rule.input2], input2);
                // Min for AND.
                const activation = if (m1 < m2) m1 else m2;
                if (activation == 0.0) continue;

                for (self.grid, 0..) |g, j| {
                    const out_membership = membershipValue(self.output[rule.output], g);
                    // Truncate at the activation, then max for OR.
                    const truncated = if (activation < out_membership) activation else out_membership;
                    if (truncated > self.scratch[j]) self.scratch[j] = truncated;
                }
            }

            // Centroid.
            var numerator: f32 = 0;
            var denominator: f32 = 0;
            for (self.grid, self.scratch) |g, m| {
                numerator += g * m;
                denominator += m;
            }

            // Nothing fired: the C version answers with the midpoint.
            var output_value: f32 = if (denominator < 0.000001)
                (self.output_min + self.output_max) / 2.0
            else
                numerator / denominator;

            if (output_value < self.output_min) output_value = self.output_min;
            if (output_value > self.output_max) output_value = self.output_max;

            return output_value;
        }
    };
}

// --- tests ------------------------------------------------------------------

/// The three sets and the 3x3 table the corpus generator uses.
const test_sets = [3]Membership{
    .{ .kind = .triangular, .params = .{ -1.0, -1.0, 0.0, 0.0 } },
    .{ .kind = .triangular, .params = .{ -1.0, 0.0, 1.0, 0.0 } },
    .{ .kind = .triangular, .params = .{ 0.0, 1.0, 1.0, 0.0 } },
};

const test_rules = [9]Rule{
    .{ .input1 = 0, .input2 = 0, .output = 0 },
    .{ .input1 = 0, .input2 = 1, .output = 0 },
    .{ .input1 = 0, .input2 = 2, .output = 1 },
    .{ .input1 = 1, .input2 = 0, .output = 0 },
    .{ .input1 = 1, .input2 = 1, .output = 1 },
    .{ .input1 = 1, .input2 = 2, .output = 2 },
    .{ .input1 = 2, .input2 = 0, .output = 1 },
    .{ .input1 = 2, .input2 = 1, .output = 2 },
    .{ .input1 = 2, .input2 = 2, .output = 2 },
};

fn testFuzzy() Fuzzy(5) {
    return Fuzzy(5).init(-1.0, 1.0, -1.0, 1.0, -1.0, 1.0, &test_sets, &test_sets, &test_sets, &test_rules);
}

test "fuzzy: the membership functions match the C answers" {
    const corpus = try corpus_mod.Corpus.load();

    try corpus.expectValue("fuzzy_tri_left", triangular(-0.5, -1.0, 0.0, 1.0));
    try corpus.expectValue("fuzzy_tri_centre", triangular(0.0, -1.0, 0.0, 1.0));
    try corpus.expectValue("fuzzy_tri_right", triangular(0.5, -1.0, 0.0, 1.0));
    try corpus.expectValue("fuzzy_tri_at_a", triangular(-1.0, -1.0, 0.0, 1.0));
    try corpus.expectValue("fuzzy_tri_at_c", triangular(1.0, -1.0, 0.0, 1.0));

    // A triangle whose `b` equals its `a`: the left branch would divide by zero,
    // and the `x <= a` guard is what stops it. The C answer is 0.
    try corpus.expectValue("fuzzy_tri_degenerate", triangular(-1.0, -1.0, -1.0, 1.0));

    try corpus.expectValue("fuzzy_trap_left", trapezoidal(0.1, 0.0, 0.2, 0.5, 1.0));
    try corpus.expectValue("fuzzy_trap_shoulder", trapezoidal(0.3, 0.0, 0.2, 0.5, 1.0));
    try corpus.expectValue("fuzzy_trap_right", trapezoidal(0.75, 0.0, 0.2, 0.5, 1.0));

    try corpus.expectValue("fuzzy_gauss_centre", gaussian(0.0, 0.0, 0.5));
    try corpus.expectValue("fuzzy_gauss_one_sigma", gaussian(0.5, 0.0, 0.5));
    try corpus.expectValue("fuzzy_gauss_far", gaussian(2.0, 0.0, 0.5));

    // Independent of the corpus: a gaussian at one sigma is exp(-0.5), and a
    // triangle reaches 1 exactly at its centre.
    try std.testing.expectApproxEqAbs(@exp(@as(f32, -0.5)), gaussian(0.5, 0.0, 0.5), 1.0e-6);
    try std.testing.expectEqual(@as(f32, 1.0), triangular(0.0, -1.0, 0.0, 1.0));
    try std.testing.expectEqual(@as(f32, 1.0), trapezoidal(0.3, 0.0, 0.2, 0.5, 1.0));
}

test "fuzzy: the discretisation grid matches the C answers" {
    const corpus = try corpus_mod.Corpus.load();

    const f = testFuzzy();
    try corpus.expectValues("fuzzy_grid", &f.grid);
    try corpus.expectValue("fuzzy_input1_min", f.input1_min);
}

test "fuzzy: the centroid matches the C answers" {
    const corpus = try corpus_mod.Corpus.load();

    var f = testFuzzy();
    try corpus.expectValue("fuzzy_compute_zero", f.compute(0.0, 0.0));
    try corpus.expectValue("fuzzy_compute_pos_pos", f.compute(0.8, 0.8));
    try corpus.expectValue("fuzzy_compute_neg_neg", f.compute(-0.8, -0.8));
    try corpus.expectValue("fuzzy_compute_mixed", f.compute(0.5, -0.5));

    // Inputs outside the range are clamped to it, so the extreme values repeat
    // the answers at the corners.
    try corpus.expectValue("fuzzy_compute_clamped_high", f.compute(9.0, 9.0));
    try corpus.expectValue("fuzzy_compute_clamped_low", f.compute(-9.0, -9.0));

    // Independent of the corpus: the table is antisymmetric, so mirroring both
    // inputs mirrors the output.
    const positive = f.compute(0.8, 0.8);
    const negative = f.compute(-0.8, -0.8);
    try std.testing.expectApproxEqAbs(-positive, negative, 1.0e-6);

    // And the output always stays inside the configured range.
    for ([_]f32{ -1.0, -0.5, 0.0, 0.5, 1.0 }) |x| {
        const y = f.compute(x, x);
        try std.testing.expect(y >= f.output_min and y <= f.output_max);
    }
}

test "fuzzy: no rules and an empty consequent match the C answers" {
    const corpus = try corpus_mod.Corpus.load();

    // No rules at all: the C returns 0 before allocating anything.
    var bare = Fuzzy(5).init(-1.0, 1.0, -1.0, 1.0, -1.0, 1.0, &test_sets, &test_sets, &test_sets, &.{});
    try corpus.expectValue("fuzzy_compute_no_rules", bare.compute(0.5, 0.5));

    // A rule whose output set lies outside the output range: every grid point has
    // zero membership, so the denominator stays zero and the C answers with the
    // midpoint of the range.
    const far = [1]Membership{
        .{ .kind = .triangular, .params = .{ 5.0, 6.0, 7.0, 0.0 } },
    };
    const one_rule = [1]Rule{.{ .input1 = 1, .input2 = 1, .output = 0 }};
    var empty_consequent = Fuzzy(5).init(-1.0, 1.0, -1.0, 1.0, -1.0, 1.0, &test_sets, &test_sets, &far, &one_rule);
    try corpus.expectValue("fuzzy_compute_empty_consequent", empty_consequent.compute(0.5, 0.5));

    try std.testing.expectEqual(@as(f32, 0.0), empty_consequent.compute(0.5, 0.5));
}
