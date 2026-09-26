//! Matrices up to 4x4, ported from `include/breeze/math/matrix.h`.
//!
//! The C type is `float data[4][4]` plus a runtime `rows`/`cols` pair, so every
//! function starts by checking the shape it was handed and every failure is a
//! `0` return the caller may ignore. Here the shape is a comptime parameter of
//! the type, which turns four classes of runtime failure into compile errors:
//!
//! * adding a 2x3 to a 3x2,
//! * multiplying with mismatched inner dimensions,
//! * taking the determinant of a non-square matrix,
//! * and asking for a 4x4 determinant or inverse, which the C version answers
//!   with `0` ("4x4 and larger are not implemented") - the port refuses to
//!   compile instead of returning a plausible number. Filling that gap is a
//!   deliberate future change, not something a migration should smuggle in.
//!
//! Two more differences worth naming:
//!
//! * **Storage beyond the shape is gone.** The C `Init` zeroes only the used
//!   region and leaves the rest of the 4x4 array untouched; `Mat(2, 3)` has no
//!   rest. The corpus keeps a case recording what the C storage looked like,
//!   because a reader is entitled to know the port dropped it on purpose.
//! * **In-place operations are safe.** The C `Multiply` and `Transpose` allocate
//!   a temporary specifically to allow `result` to alias an input - and the C
//!   `Inverse` does *not*, so `Inverse2x2(&m, &m)` corrupts `m` by writing
//!   `[0][0]` before reading it for `[1][1]`. Values make both spellings safe
//!   here; the inverse is covered by a test that multiplies a matrix by its own
//!   inverse and expects the identity.
//!
//! The arithmetic is checked against the C implementation through
//! `testdata/math_corpus.txt` (see `corpus.zig`).

const std = @import("std");

const corpus_mod = @import("corpus.zig");

/// Returned by `inverse` when the determinant is too small to divide by.
pub const Singular = error{Singular};

/// The determinant below which the C code refuses to invert. Copied, not chosen.
pub const min_determinant: f32 = 1.0e-6;

/// A `rows` x `cols` matrix of `f32`.
pub fn Mat(comptime rows: usize, comptime cols: usize) type {
    return struct {
        data: [rows][cols]f32,

        const Self = @This();

        pub const n_rows = rows;
        pub const n_cols = cols;

        pub const zero: Self = .{ .data = [_][cols]f32{[_]f32{0} ** cols} ** rows };

        pub fn init(data: [rows][cols]f32) Self {
            return .{ .data = data };
        }

        /// Ones on the diagonal, out to `min(rows, cols)` - the C version's
        /// `min_dim` rule, kept because a non-square identity is what it
        /// produces.
        pub fn identity() Self {
            var out = zero;
            inline for (0..@min(rows, cols)) |i| out.data[i][i] = 1;
            return out;
        }

        /// Bounds are checked by the compiler's own safety checks on indexing;
        /// the C version's silent ignore / return 0 has no counterpart here.
        pub fn at(m: Self, row: usize, col: usize) f32 {
            return m.data[row][col];
        }

        pub fn add(a: Self, b: Self) Self {
            var out: Self = undefined;
            inline for (0..rows) |i| {
                inline for (0..cols) |j| out.data[i][j] = a.data[i][j] + b.data[i][j];
            }
            return out;
        }

        pub fn sub(a: Self, b: Self) Self {
            var out: Self = undefined;
            inline for (0..rows) |i| {
                inline for (0..cols) |j| out.data[i][j] = a.data[i][j] - b.data[i][j];
            }
            return out;
        }

        /// Matrix product. The inner dimension is checked at comptime, so
        /// mismatched shapes do not compile - the C version returns 0.
        ///
        /// Each element accumulates in ascending `k`, matching the C loop order,
        /// so the two agree to the last bit and the corpus comparison is exact.
        pub fn mul(a: Self, b: anytype) Mat(rows, @TypeOf(b).n_cols) {
            const B = @TypeOf(b);
            if (B.n_rows != cols) {
                @compileError("matrix multiply: a.n_cols must equal b.n_rows");
            }

            var out = Mat(rows, B.n_cols).zero;
            for (0..rows) |i| {
                for (0..B.n_cols) |j| {
                    var sum: f32 = 0;
                    for (0..cols) |k| sum += a.data[i][k] * b.data[k][j];
                    out.data[i][j] = sum;
                }
            }
            return out;
        }

        pub fn scale(a: Self, factor: f32) Self {
            var out: Self = undefined;
            inline for (0..rows) |i| {
                inline for (0..cols) |j| out.data[i][j] = a.data[i][j] * factor;
            }
            return out;
        }

        pub fn transpose(a: Self) Mat(cols, rows) {
            var out = Mat(cols, rows).zero;
            inline for (0..rows) |i| {
                inline for (0..cols) |j| out.data[j][i] = a.data[i][j];
            }
            return out;
        }

        /// 2x2 and 3x3 only; anything else is a compile error rather than the
        /// C version's `0.0f`.
        pub fn determinant(a: Self) f32 {
            if (rows != cols) @compileError("matrix determinant: must be square");
            return switch (rows) {
                2 => a.data[0][0] * a.data[1][1] - a.data[0][1] * a.data[1][0],
                3 => a.data[0][0] * (a.data[1][1] * a.data[2][2] - a.data[1][2] * a.data[2][1]) -
                    a.data[0][1] * (a.data[1][0] * a.data[2][2] - a.data[1][2] * a.data[2][0]) +
                    a.data[0][2] * (a.data[1][0] * a.data[2][1] - a.data[1][1] * a.data[2][0]),
                else => @compileError("matrix determinant is implemented for 2x2 and 3x3 only"),
            };
        }

        /// 2x2 and 3x3 only, by the adjugate over the determinant - the same
        /// formula, in the same order, as the C version.
        ///
        /// `error.Singular` replaces the C `0` return; unlike the C function,
        /// failing produces no half-written matrix to be confused by.
        pub fn inverse(a: Self) Singular!Self {
            if (rows != cols) @compileError("matrix inverse: must be square");

            const det = a.determinant();
            if (@abs(det) < min_determinant) return error.Singular;

            return switch (rows) {
                2 => Mat(2, 2).init(.{
                    .{ a.data[1][1] / det, -a.data[0][1] / det },
                    .{ -a.data[1][0] / det, a.data[0][0] / det },
                }),
                3 => Mat(3, 3).init(.{
                    .{
                        (a.data[1][1] * a.data[2][2] - a.data[1][2] * a.data[2][1]) / det,
                        (a.data[0][2] * a.data[2][1] - a.data[0][1] * a.data[2][2]) / det,
                        (a.data[0][1] * a.data[1][2] - a.data[0][2] * a.data[1][1]) / det,
                    },
                    .{
                        (a.data[1][2] * a.data[2][0] - a.data[1][0] * a.data[2][2]) / det,
                        (a.data[0][0] * a.data[2][2] - a.data[0][2] * a.data[2][0]) / det,
                        (a.data[0][2] * a.data[1][0] - a.data[0][0] * a.data[1][2]) / det,
                    },
                    .{
                        (a.data[1][0] * a.data[2][1] - a.data[1][1] * a.data[2][0]) / det,
                        (a.data[0][1] * a.data[2][0] - a.data[0][0] * a.data[2][1]) / det,
                        (a.data[0][0] * a.data[1][1] - a.data[0][1] * a.data[1][0]) / det,
                    },
                }),
                else => @compileError("matrix inverse is implemented for 2x2 and 3x3 only"),
            };
        }
    };
}

pub const Mat2 = Mat(2, 2);
pub const Mat3 = Mat(3, 3);
pub const Mat4 = Mat(4, 4);

// --- tests ------------------------------------------------------------------

/// Independent of the corpus on purpose: it says the inverse is *right* in the
/// sense the caller cares about, not merely equal to what C computed.
fn expectIdentity(m: anytype) !void {
    const M = @TypeOf(m);
    for (0..M.n_rows) |i| {
        for (0..M.n_cols) |j| {
            const want: f32 = if (i == j) 1.0 else 0.0;
            try std.testing.expectApproxEqAbs(want, m.data[i][j], 1.0e-5);
        }
    }
}

test "matrix: float storage matches the C struct's data array" {
    const corpus = try corpus_mod.Corpus.load();

    // The C struct also carries `rows` and `cols` (72 bytes in total); the Zig
    // type does not, because the shape is in the type. What must agree is the
    // float storage itself.
    try corpus.expectInt("matrix_data_bytes", @sizeOf(Mat4));
    try std.testing.expectEqual(@as(usize, 64), @sizeOf(Mat4));
}

test "matrix: construction matches the C versions" {
    const corpus = try corpus_mod.Corpus.load();

    const zeros = Mat(2, 3).zero;
    try corpus.expectValues("matrix_init_2x3", &.{
        zeros.data[0][0], zeros.data[0][1], zeros.data[0][2],
        zeros.data[1][0], zeros.data[1][1], zeros.data[1][2],
    });

    const id3 = Mat3.identity();
    try corpus.expectValues("matrix_identity_3x3", &.{
        id3.data[0][0], id3.data[0][1], id3.data[0][2],
        id3.data[1][0], id3.data[1][1], id3.data[1][2],
        id3.data[2][0], id3.data[2][1], id3.data[2][2],
    });

    // Non-square: ones only out to min(rows, cols).
    const id23 = Mat(2, 3).identity();
    try corpus.expectValues("matrix_identity_2x3", &.{
        id23.data[0][0], id23.data[0][1], id23.data[0][2],
        id23.data[1][0], id23.data[1][1], id23.data[1][2],
    });
}

test "matrix: element access matches the C versions" {
    const corpus = try corpus_mod.Corpus.load();

    var m = Mat2.zero;
    m.data[0][1] = 7.5;
    try corpus.expectValue("matrix_get_element", m.at(0, 1));
    try corpus.expectValues("matrix_set_get_2x2", &.{
        m.data[0][0], m.data[0][1], m.data[1][0], m.data[1][1],
    });

    // The C version's out-of-range `SetElement` is silently ignored and its
    // `GetElement` returns 0. Neither exists here: indexing out of range is a
    // safety panic, and the shape cannot be wrong. The corpus keeps both C
    // cases so the difference is documented rather than assumed.
    try corpus.expectValue("matrix_get_out_of_range", 0.0);
    try corpus.expectValues("matrix_set_out_of_range_2x2", &.{
        m.data[0][0], m.data[0][1], m.data[1][0], m.data[1][1],
    });
}

test "matrix: arithmetic matches the C answers" {
    const corpus = try corpus_mod.Corpus.load();

    const a = Mat2.init(.{ .{ 4, 7 }, .{ 2, 6 } });
    const b = Mat2.init(.{ .{ 1, 2 }, .{ 3, 4 } });

    const sum = a.add(b);
    try corpus.expectValues("matrix_add_2x2", &.{
        sum.data[0][0], sum.data[0][1], sum.data[1][0], sum.data[1][1],
    });

    const diff = a.sub(b);
    try corpus.expectValues("matrix_subtract_2x2", &.{
        diff.data[0][0], diff.data[0][1], diff.data[1][0], diff.data[1][1],
    });

    const prod = a.mul(b);
    try corpus.expectValues("matrix_multiply_2x2", &.{
        prod.data[0][0], prod.data[0][1], prod.data[1][0], prod.data[1][1],
    });

    // The C caller's in-place spelling, which the C version supports by
    // multiplying into a temporary.
    var in_place = a;
    in_place = in_place.mul(b);
    try corpus.expectValues("matrix_multiply_in_place_2x2", &.{
        in_place.data[0][0], in_place.data[0][1], in_place.data[1][0], in_place.data[1][1],
    });

    // Non-square product: 2x3 times 3x2.
    const p = Mat(2, 3).init(.{ .{ 1, 2, 3 }, .{ 4, 5, 6 } });
    const q = Mat(3, 2).init(.{ .{ 7, 8 }, .{ 9, 10 }, .{ 11, 12 } });
    const r = p.mul(q);
    try corpus.expectValues("matrix_multiply_2x3_3x2", &.{
        r.data[0][0], r.data[0][1], r.data[1][0], r.data[1][1],
    });

    const s = Mat(3, 2).init(.{ .{ 2, -1 }, .{ 0.5, 3 }, .{ 4, -2 } });
    const scaled = s.scale(-1.5);
    try corpus.expectValues("matrix_scalar_multiply_3x2", &.{
        scaled.data[0][0], scaled.data[0][1],
        scaled.data[1][0], scaled.data[1][1],
        scaled.data[2][0], scaled.data[2][1],
    });

    const t = p.transpose();
    try corpus.expectValues("matrix_transpose_2x3", &.{
        t.data[0][0], t.data[0][1],
        t.data[1][0], t.data[1][1],
        t.data[2][0], t.data[2][1],
    });

    var square = Mat3.init(.{ .{ 1, 2, 3 }, .{ 4, 5, 6 }, .{ 7, 8, 9 } });
    square = square.transpose();
    try corpus.expectValues("matrix_transpose_in_place_3x3", &.{
        square.data[0][0], square.data[0][1], square.data[0][2],
        square.data[1][0], square.data[1][1], square.data[1][2],
        square.data[2][0], square.data[2][1], square.data[2][2],
    });
}

test "matrix: determinant matches the C answers" {
    const corpus = try corpus_mod.Corpus.load();

    const a = Mat2.init(.{ .{ 4, 7 }, .{ 2, 6 } });
    try corpus.expectValue("matrix_det_2x2", a.determinant());
    try corpus.expectValue("matrix_det_dispatch_2x2", a.determinant());

    // Determinant 1, so the inverse below is exact in float and any difference
    // is the port's fault rather than rounding.
    const b = Mat3.init(.{ .{ 1, 2, 3 }, .{ 0, 1, 4 }, .{ 5, 6, 0 } });
    try corpus.expectValue("matrix_det_3x3", b.determinant());
    try corpus.expectValue("matrix_det_dispatch_3x3", b.determinant());
}

test "matrix: inverse matches the C answers" {
    const corpus = try corpus_mod.Corpus.load();

    const a = Mat2.init(.{ .{ 4, 7 }, .{ 2, 6 } });
    const ia = try a.inverse();
    try corpus.expectValues("matrix_inverse_2x2", &.{
        ia.data[0][0], ia.data[0][1], ia.data[1][0], ia.data[1][1],
    });
    try corpus.expectValues("matrix_inverse_dispatch_2x2", &.{
        ia.data[0][0], ia.data[0][1], ia.data[1][0], ia.data[1][1],
    });

    // Determinant 1, and the inverse is the textbook integer matrix.
    const b = Mat3.init(.{ .{ 1, 2, 3 }, .{ 0, 1, 4 }, .{ 5, 6, 0 } });
    const ib = try b.inverse();
    try corpus.expectValues("matrix_inverse_3x3", &.{
        ib.data[0][0], ib.data[0][1], ib.data[0][2],
        ib.data[1][0], ib.data[1][1], ib.data[1][2],
        ib.data[2][0], ib.data[2][1], ib.data[2][2],
    });
}

test "matrix: a singular matrix is refused, and says so in the type" {
    const corpus = try corpus_mod.Corpus.load();

    // The corpus records the C status code (0 = refused) and the fact that the
    // C version leaves the caller's output untouched. The port answers with an
    // error instead, so there is nothing half-written to check - and the C
    // status is still read here, so a change to the C refusal shows up.
    try corpus.expectInt("matrix_inverse_singular_ok", 0);
    try corpus.expectValues("matrix_inverse_singular_untouched", &.{ 1, 2, 3, 4 });

    const singular = Mat2.init(.{ .{ 1, 2 }, .{ 2, 4 } });
    try std.testing.expectError(error.Singular, singular.inverse());
    try std.testing.expectEqual(@as(f32, 0), singular.determinant());
}

test "matrix: an inverse really is an inverse" {
    // Checks the port against arithmetic rather than against the oracle: if both
    // implementations were wrong in the same way, the corpus comparison above
    // would still pass and this would not.
    const m = Mat3.init(.{ .{ 4, 7, 2 }, .{ 3, 6, 1 }, .{ 2, 5, 3 } });
    const product = m.mul(try m.inverse());
    try expectIdentity(product);

    const n = Mat2.init(.{ .{ 3, 1 }, .{ 2, 4 } });
    try expectIdentity(n.mul(try n.inverse()));

    // The C version's `Inverse2x2(&m, &m)` corrupts `m`; here the input is a
    // copy, so the in-place spelling is safe. Doing it twice is the cheapest
    // witness: the second inverse must equal the original matrix again.
    var in_place = m;
    in_place = try in_place.inverse();
    in_place = try in_place.inverse();
    for (0..3) |i| {
        for (0..3) |j| {
            try std.testing.expectApproxEqAbs(m.data[i][j], in_place.data[i][j], 1.0e-4);
        }
    }
}
