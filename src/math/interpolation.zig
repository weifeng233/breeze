//! Interpolation, ported from `include/breeze/math/interpolation.h`.
//!
//! This is the module ARCHITECTURE.md §8.2 singles out: the C spline allocates
//! thirty-four times through `malloc`/`free` - six coefficient arrays that
//! outlive `Init`, plus five scratch arrays that do not - and an embedded target
//! with no heap cannot use any of it. The port answers that in the two ways the
//! section asks for:
//!
//! * `Spline(n)` is the caller-provides-the-buffer form. The knot count is a
//!   comptime parameter, every array is a field of the value, and the scratch the
//!   solver needs lives on the stack for the duration of `init`. **No allocator
//!   appears anywhere**, which is stronger than passing one in.
//! * `Dyn` takes an allocator for the case where the knot count is only known at
//!   run time, and releases the scratch before `init` returns, so a caller
//!   cannot leak it or confuse it with the coefficients.
//!
//! The coefficient arrays are sized to the range the C solver actually writes.
//! The C version allocates `a`, `b` and `d` with `n` entries and never writes
//! `a[n-1]`, `b[n-1]` or `d[n-1]` - `Evaluate` cannot reach them, since the last
//! interval ends at index `n-2` - so those three slots are uninitialised memory
//! there. Here `a`, `b` and `d` have `n-1` entries (one per interval) and `c` has
//! `n`, because the back-substitution reads `c[i+1]`. The uninitialised question
//! does not arise, and the corpus prints only the range the C code filled.
//!
//! `linear` and `cubicHermite` copy the C arithmetic exactly, including
//! `linear`'s refusal to divide when the two x values are within 1e-6 of each
//! other (it answers with the midpoint) and the strictly-increasing requirement
//! on the knots, which arrives here as `error.NotIncreasing`.

const std = @import("std");

const corpus_mod = @import("corpus.zig");

pub const Error = error{
    TooFewPoints,
    NotIncreasing,
    LengthMismatch,
};

/// The x-span below which `linear` refuses to divide, copied from the C code.
pub const min_span: f32 = 1.0e-6;

/// The knot spacing below which the spline solver refuses, same constant.
pub const min_spacing: f32 = 1.0e-6;

/// Linear interpolation between two points.
///
/// When `x0` and `x1` are within `min_span` the answer is the midpoint of `y0`
/// and `y1`, which is what the C code does instead of dividing by (almost) zero.
pub fn linear(x0: f32, y0: f32, x1: f32, y1: f32, x: f32) f32 {
    if (@abs(x1 - x0) < min_span) return (y0 + y1) * 0.5;
    return y0 + (y1 - y0) * (x - x0) / (x1 - x0);
}

/// Cosine interpolation, eased by `(1 - cos(mu * pi)) / 2`.
pub fn cosine(y0: f32, y1: f32, mu: f32) f32 {
    const mu2 = (1.0 - @cos(mu * std.math.pi)) * 0.5;
    return y0 * (1.0 - mu2) + y1 * mu2;
}

/// Cubic Hermite interpolation between `y1` and `y2`, with `y0` and `y3` giving
/// the neighbouring points and `tension` / `bias` shaping the tangents.
pub fn cubicHermite(y0: f32, y1: f32, y2: f32, y3: f32, mu: f32, tension: f32, bias: f32) f32 {
    const mu2 = mu * mu;
    const mu3 = mu2 * mu;

    var m0 = (y2 - y0) * (1.0 + bias) * (1.0 - tension) * 0.5;
    m0 += (y3 - y1) * (1.0 - bias) * (1.0 - tension) * 0.5;

    var m1 = (y3 - y1) * (1.0 + bias) * (1.0 - tension) * 0.5;
    m1 += (y2 - y0) * (1.0 - bias) * (1.0 - tension) * 0.5;

    const a0 = 2.0 * mu3 - 3.0 * mu2 + 1.0;
    const a1 = mu3 - 2.0 * mu2 + mu;
    const a2 = mu3 - mu2;
    const a3 = -2.0 * mu3 + 3.0 * mu2;

    return a0 * y1 + a1 * m0 + a2 * m1 + a3 * y2;
}

/// A point on a Bezier curve.
pub const BezierPoint = struct {
    x: f32,
    y: f32,
};

/// Quadratic Bezier at parameter `t`.
pub fn bezierQuadratic(p0: BezierPoint, p1: BezierPoint, p2: BezierPoint, t: f32) BezierPoint {
    const t1 = 1.0 - t;
    const t1_squared = t1 * t1;
    const t_squared = t * t;
    const t1_t_2 = 2.0 * t1 * t;
    return .{
        .x = t1_squared * p0.x + t1_t_2 * p1.x + t_squared * p2.x,
        .y = t1_squared * p0.y + t1_t_2 * p1.y + t_squared * p2.y,
    };
}

/// Cubic Bezier at parameter `t`.
pub fn bezierCubic(p0: BezierPoint, p1: BezierPoint, p2: BezierPoint, p3: BezierPoint, t: f32) BezierPoint {
    const t1 = 1.0 - t;
    const t1_squared = t1 * t1;
    const t1_cubed = t1_squared * t1;
    const t_squared = t * t;
    const t_cubed = t_squared * t;
    const t1_squared_t = 3.0 * t1_squared * t;
    const t1_t_squared = 3.0 * t1 * t_squared;
    return .{
        .x = t1_cubed * p0.x + t1_squared_t * p1.x + t1_t_squared * p2.x + t_cubed * p3.x,
        .y = t1_cubed * p0.y + t1_squared_t * p1.y + t1_t_squared * p2.y + t_cubed * p3.y,
    };
}

/// Scratch the solver needs and nobody keeps. Passed explicitly so that the
/// fixed-size form can put it on the stack and the dynamic form can allocate it -
/// without the solver knowing which.
const Scratch = struct {
    h: []f32,
    alpha: []f32,
    l: []f32,
    mu: []f32,
    z: []f32,

    fn slices(self: Scratch) [5][]f32 {
        return .{ self.h, self.alpha, self.l, self.mu, self.z };
    }
};

/// The tridiagonal solve from the C `Init`, over slices.
///
/// `a`, `b`, `d` have one entry per interval; `c` has one per knot because the
/// back-substitution reads `c[i + 1]`.
fn solve(
    x: []const f32,
    y: []const f32,
    a: []f32,
    b: []f32,
    c: []f32,
    d: []f32,
    scratch: Scratch,
) Error!void {
    const n = x.len;
    const h = scratch.h;
    const alpha = scratch.alpha;
    const l = scratch.l;
    const mu = scratch.mu;
    const z = scratch.z;

    for (0..n - 1) |i| {
        h[i] = x[i + 1] - x[i];
        if (h[i] < min_spacing) return Error.NotIncreasing;
    }

    for (1..n - 1) |i| {
        alpha[i] = 3.0 * ((y[i + 1] - y[i]) / h[i] - (y[i] - y[i - 1]) / h[i - 1]);
    }

    l[0] = 1.0;
    mu[0] = 0.0;
    z[0] = 0.0;

    for (1..n - 1) |i| {
        l[i] = 2.0 * (x[i + 1] - x[i - 1]) - h[i - 1] * mu[i - 1];
        mu[i] = h[i] / l[i];
        z[i] = (alpha[i] - h[i - 1] * z[i - 1]) / l[i];
    }

    l[n - 1] = 1.0;
    z[n - 1] = 0.0;
    c[n - 1] = 0.0;

    var i = n - 1;
    while (i > 0) {
        i -= 1;
        c[i] = z[i] - mu[i] * c[i + 1];
        b[i] = (y[i + 1] - y[i]) / h[i] - h[i] * (c[i + 1] + 2.0 * c[i]) / 3.0;
        d[i] = (c[i + 1] - c[i]) / (3.0 * h[i]);
    }

    for (0..n - 1) |k| a[k] = y[k];
}

/// Evaluation shared by both forms. Clamps outside the knot range, and lands
/// exactly on `y[i]` when `at` is a knot.
fn evaluateSlices(
    x: []const f32,
    y: []const f32,
    a: []const f32,
    b: []const f32,
    c: []const f32,
    d: []const f32,
    at: f32,
) f32 {
    const n = x.len;
    if (at <= x[0]) return y[0];
    if (at >= x[n - 1]) return y[n - 1];

    var i: usize = 0;
    while (i < n - 1) : (i += 1) {
        if (at < x[i + 1]) break;
    }

    const dx = at - x[i];
    return a[i] + b[i] * dx + c[i] * dx * dx + d[i] * dx * dx * dx;
}

/// A cubic spline through `n` knots, with all of its storage in the value.
///
/// `n` must be at least 2; that is a compile error rather than the C version's
/// `return 0`, since a spline with one knot is not a thing anyone can use.
pub fn Spline(comptime n: usize) type {
    if (n < 2) @compileError("a cubic spline needs at least two knots");

    return struct {
        x: [n]f32,
        y: [n]f32,

        /// One coefficient per interval.
        a: [n - 1]f32,
        b: [n - 1]f32,
        d: [n - 1]f32,
        /// One per knot: the solver reads `c[i + 1]` while back-substituting.
        c: [n]f32,

        const Self = @This();

        pub const knots = n;

        pub fn init(x: [n]f32, y: [n]f32) Error!Self {
            var out: Self = .{
                .x = x,
                .y = y,
                .a = undefined,
                .b = undefined,
                .c = undefined,
                .d = undefined,
            };

            // Scratch, on the stack, gone when init returns.
            var h: [n - 1]f32 = undefined;
            var alpha: [n - 1]f32 = undefined;
            var l: [n]f32 = undefined;
            var mu: [n]f32 = undefined;
            var z: [n]f32 = undefined;

            try solve(&out.x, &out.y, &out.a, &out.b, &out.c, &out.d, .{
                .h = &h,
                .alpha = &alpha,
                .l = &l,
                .mu = &mu,
                .z = &z,
            });
            return out;
        }

        pub fn evaluate(s: Self, at: f32) f32 {
            return evaluateSlices(&s.x, &s.y, &s.a, &s.b, &s.c, &s.d, at);
        }
    };
}

/// A spline whose knot count is only known at run time.
///
/// The allocator owns the knots and the coefficients; the scratch the solver
/// needs is taken and released inside `init`.
pub const Dyn = struct {
    allocator: std.mem.Allocator,
    x: []f32,
    y: []f32,
    a: []f32,
    b: []f32,
    c: []f32,
    d: []f32,

    pub fn init(allocator: std.mem.Allocator, x: []const f32, y: []const f32) !Dyn {
        if (x.len != y.len) return Error.LengthMismatch;
        if (x.len < 2) return Error.TooFewPoints;

        const n = x.len;
        var out = Dyn{
            .allocator = allocator,
            .x = try allocator.dupe(f32, x),
            .y = undefined,
            .a = undefined,
            .b = undefined,
            .c = undefined,
            .d = undefined,
        };
        errdefer allocator.free(out.x);

        out.y = try allocator.dupe(f32, y);
        errdefer allocator.free(out.y);

        out.a = try allocator.alloc(f32, n - 1);
        errdefer allocator.free(out.a);
        out.b = try allocator.alloc(f32, n - 1);
        errdefer allocator.free(out.b);
        out.d = try allocator.alloc(f32, n - 1);
        errdefer allocator.free(out.d);
        out.c = try allocator.alloc(f32, n);

        const h = try allocator.alloc(f32, n - 1);
        defer allocator.free(h);
        const alpha = try allocator.alloc(f32, n - 1);
        defer allocator.free(alpha);
        const l = try allocator.alloc(f32, n);
        defer allocator.free(l);
        const mu = try allocator.alloc(f32, n);
        defer allocator.free(mu);
        const z = try allocator.alloc(f32, n);
        defer allocator.free(z);

        solve(out.x, out.y, out.a, out.b, out.c, out.d, .{
            .h = h,
            .alpha = alpha,
            .l = l,
            .mu = mu,
            .z = z,
        }) catch |err| {
            allocator.free(out.c);
            return err;
        };

        return out;
    }

    pub fn deinit(self: Dyn) void {
        self.allocator.free(self.x);
        self.allocator.free(self.y);
        self.allocator.free(self.a);
        self.allocator.free(self.b);
        self.allocator.free(self.c);
        self.allocator.free(self.d);
    }

    pub fn evaluate(self: Dyn, at: f32) f32 {
        return evaluateSlices(self.x, self.y, self.a, self.b, self.c, self.d, at);
    }
};

// --- tests ------------------------------------------------------------------

test "interpolation: the scalar curves match the C answers" {
    const corpus = try corpus_mod.Corpus.load();

    try corpus.expectValue("interp_linear", linear(0.0, 0.0, 2.0, 4.0, 0.5));
    // x0 and x1 within 1e-6: the midpoint, not a division by nearly zero.
    try corpus.expectValue("interp_linear_degenerate", linear(1.0, 3.0, 1.0 + 1.0e-7, 7.0, 5.0));

    try corpus.expectValue("interp_cosine_mu0", cosine(0.0, 10.0, 0.0));
    try corpus.expectValue("interp_cosine_mu025", cosine(0.0, 10.0, 0.25));
    try corpus.expectValue("interp_cosine_mu05", cosine(0.0, 10.0, 0.5));
    try corpus.expectValue("interp_cosine_mu1", cosine(0.0, 10.0, 1.0));

    try corpus.expectValue("interp_hermite", cubicHermite(0.0, 1.0, 2.0, 3.0, 0.5, 0.0, 0.0));
    try corpus.expectValue("interp_hermite_tension_bias", cubicHermite(0.0, 1.0, 2.0, 3.0, 0.5, 0.3, 0.2));

    // Independent of the corpus: linear hits its endpoints, and cosine is a
    // smoothstep between them.
    try std.testing.expectEqual(@as(f32, 3.0), linear(0.0, 3.0, 2.0, 7.0, 0.0));
    try std.testing.expectEqual(@as(f32, 7.0), linear(0.0, 3.0, 2.0, 7.0, 2.0));
    try std.testing.expectEqual(@as(f32, 0.0), cosine(0.0, 10.0, 0.0));
    try std.testing.expectEqual(@as(f32, 10.0), cosine(0.0, 10.0, 1.0));
    try std.testing.expectEqual(@as(f32, 3.5), cosine(0.0, 7.0, 0.5));
}

test "spline: the coefficients match the C answers" {
    const corpus = try corpus_mod.Corpus.load();

    const s = try Spline(5).init(.{ 0, 1, 2, 3, 4 }, .{ 0, 1, 0, 1, 0 });

    try corpus.expectValues("spline_a", &.{ s.a[0], s.a[1], s.a[2], s.a[3] });
    try corpus.expectValues("spline_b", &.{ s.b[0], s.b[1], s.b[2], s.b[3] });
    try corpus.expectValues("spline_c", &.{ s.c[0], s.c[1], s.c[2], s.c[3], s.c[4] });
    try corpus.expectValues("spline_d", &.{ s.d[0], s.d[1], s.d[2], s.d[3] });
}

test "spline: evaluation matches the C answers" {
    const corpus = try corpus_mod.Corpus.load();

    const s = try Spline(5).init(.{ 0, 1, 2, 3, 4 }, .{ 0, 1, 0, 1, 0 });

    // Outside the knot range the C version clamps rather than extrapolating.
    try corpus.expectValue("spline_eval_below_range", s.evaluate(-1.0));
    try corpus.expectValue("spline_eval_at_knot0", s.evaluate(0.0));
    try corpus.expectValue("spline_eval_at_knot2", s.evaluate(2.0));
    try corpus.expectValue("spline_eval_between", s.evaluate(1.5));
    try corpus.expectValue("spline_eval_above_range", s.evaluate(9.0));

    // Independent of the corpus: a spline passes through its own knots, and
    // clamps to the end values outside them.
    const knots = [_]f32{ 0, 1, 2, 3, 4 };
    const values = [_]f32{ 0, 1, 0, 1, 0 };
    for (knots, values) |x, want| {
        try std.testing.expectApproxEqAbs(want, s.evaluate(x), 1.0e-6);
    }
    try std.testing.expectEqual(values[0], s.evaluate(-100.0));
    try std.testing.expectEqual(values[4], s.evaluate(100.0));
}

test "spline: collinear knots reproduce the line exactly" {
    const corpus = try corpus_mod.Corpus.load();

    // A natural cubic spline through collinear points *is* that line, with all
    // the curvature coefficients zero - so this is arithmetic, not just oracle
    // agreement.
    const s = try Spline(4).init(.{ 0, 1, 2, 3 }, .{ 1, 3, 5, 7 });

    try corpus.expectValue("spline_linear_at_0p5", s.evaluate(0.5));
    try corpus.expectValue("spline_linear_at_2p5", s.evaluate(2.5));

    try std.testing.expectEqual(@as(f32, 0.0), s.c[0]);
    try std.testing.expectEqual(@as(f32, 0.0), s.c[3]);
    for ([_]f32{ 0.25, 0.5, 1.25, 2.5, 2.75 }) |x| {
        try std.testing.expectApproxEqAbs(2.0 * x + 1.0, s.evaluate(x), 1.0e-6);
    }
}

test "spline: the C refusals are errors here" {
    const corpus = try corpus_mod.Corpus.load();

    // Two knots at the same x: the C solver would divide by (almost) zero and
    // answers with a status code instead. Both statuses are read from the corpus.
    try corpus.expectInt("spline_init_not_increasing_ok", 0);
    try std.testing.expectError(Error.NotIncreasing, Spline(4).init(
        .{ 0, 1, 1, 3 },
        .{ 0, 1, 2, 3 },
    ));

    // Knots closer together than the spacing threshold are refused too.
    try std.testing.expectError(Error.NotIncreasing, Spline(3).init(
        .{ 0, 1, 1.0 + 1.0e-7 },
        .{ 0, 1, 2 },
    ));

    // Fewer than two knots cannot even be expressed: `Spline(1)` is a compile
    // error, so the C `n < 2` status has no counterpart to check. It is still
    // recorded in the corpus.
    try corpus.expectInt("spline_init_too_few_ok", 0);
}

test "spline: the run-time-sized form agrees with the fixed-size one" {
    const corpus = try corpus_mod.Corpus.load();

    const x = [_]f32{ 0, 1, 2, 3, 4 };
    const y = [_]f32{ 0, 1, 0, 1, 0 };

    var dyn = try Dyn.init(std.testing.allocator, &x, &y);
    defer dyn.deinit();

    const fixed = try Spline(5).init(x, y);

    try corpus.expectValues("spline_b", &.{ dyn.b[0], dyn.b[1], dyn.b[2], dyn.b[3] });
    for ([_]f32{ -1, 0, 0.5, 1.5, 2, 3.75, 9 }) |at| {
        try std.testing.expectEqual(fixed.evaluate(at), dyn.evaluate(at));
    }

    // The corpus values are reached through this form too, so the shared solver
    // is doing the same work on both paths.
    try corpus.expectValue("spline_eval_between", dyn.evaluate(1.5));
}

test "dyn spline: bad input is refused, and nothing leaks" {
    const allocator = std.testing.allocator;

    try std.testing.expectError(Error.LengthMismatch, Dyn.init(allocator, &.{ 0, 1, 2 }, &.{ 0, 1 }));
    try std.testing.expectError(Error.TooFewPoints, Dyn.init(allocator, &.{0}, &.{0}));
    try std.testing.expectError(Error.NotIncreasing, Dyn.init(
        allocator,
        &.{ 0, 1, 1, 3 },
        &.{ 0, 1, 2, 3 },
    ));
    // std.testing.allocator fails the test if any of the above leaked.
}

test "bezier: matches the C answers" {
    const corpus = try corpus_mod.Corpus.load();

    const p0 = BezierPoint{ .x = 0, .y = 0 };
    const p1 = BezierPoint{ .x = 1, .y = 3 };
    const p2 = BezierPoint{ .x = 4, .y = 3 };
    const p3 = BezierPoint{ .x = 5, .y = 0 };

    const q05 = bezierQuadratic(p0, p1, p2, 0.5);
    try corpus.expectValues("bezier_quadratic_t05", &.{ q05.x, q05.y });

    const q0 = bezierQuadratic(p0, p1, p2, 0.0);
    try corpus.expectValues("bezier_quadratic_t0", &.{ q0.x, q0.y });

    const c025 = bezierCubic(p0, p1, p2, p3, 0.25);
    try corpus.expectValues("bezier_cubic_t025", &.{ c025.x, c025.y });

    const c05 = bezierCubic(p0, p1, p2, p3, 0.5);
    try corpus.expectValues("bezier_cubic_t05", &.{ c05.x, c05.y });

    const c1 = bezierCubic(p0, p1, p2, p3, 1.0);
    try corpus.expectValues("bezier_cubic_t1", &.{ c1.x, c1.y });

    try corpus.expectValues("bezier_layout", &.{
        @floatFromInt(@sizeOf(BezierPoint)), @floatFromInt(@alignOf(BezierPoint)),
    });

    // Independent of the corpus: a Bezier curve starts at p0 and ends at the
    // last control point.
    try std.testing.expectEqual(p0.x, bezierCubic(p0, p1, p2, p3, 0.0).x);
    try std.testing.expectEqual(p0.y, bezierCubic(p0, p1, p2, p3, 0.0).y);
    try std.testing.expectEqual(p3.x, c1.x);
    try std.testing.expectEqual(p3.y, c1.y);
}
