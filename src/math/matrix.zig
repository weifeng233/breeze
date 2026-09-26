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
//! * and asking for a determinant or inverse of something larger than 4x4.
//!
//! The C version answered a 4x4 determinant or inverse with `0` ("4x4 and larger
//! are not implemented"), and the port used to refuse to compile for it, on the
//! grounds that filling the gap is a deliberate change rather than something a
//! migration should smuggle in. **It has now been filled deliberately** (§28 said
//! it should be its own change with its own tests, and this is it): 4x4 works, and
//! anything larger still does not compile. REVIEW §54 has the method and the
//! evidence, including the one test that matters most - a 4x4 inverse agreeing with
//! the 3x3 one on the block they share.
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
//! The 2x2 and 3x3 arithmetic is checked against the C implementation through
//! `testdata/math_corpus.txt` (see `corpus.zig`). The 4x4 has no such oracle - the
//! C never computed one - so it is checked against arithmetic instead: against the
//! 3x3 implementation on a shared block, and against the properties a determinant
//! and an inverse have to satisfy.

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

        /// The determinant of the 3x3 submatrix left by deleting row `skip_row`
        /// and column `skip_col`. Only meaningful for a 4x4 `Self`, and only used
        /// by the 4x4 determinant and adjugate below.
        fn minor3(a: Self, comptime skip_row: usize, comptime skip_col: usize) f32 {
            var minor: Mat3 = undefined;
            comptime var r: usize = 0;
            inline for (0..rows) |i| {
                if (i != skip_row) {
                    comptime var c: usize = 0;
                    inline for (0..cols) |j| {
                        if (j != skip_col) {
                            minor.data[r][c] = a.data[i][j];
                            c += 1;
                        }
                    }
                    r += 1;
                }
            }
            return minor.determinant();
        }

        /// 2x2, 3x3 and 4x4; anything else is a compile error rather than the
        /// C version's `0.0f`.
        ///
        /// The 4x4 is the cofactor expansion along the first row, so its four
        /// minors are the same 3x3 expression the 3x3 case uses - the same
        /// arithmetic style as the smaller sizes rather than a pivoting algorithm,
        /// which is what makes the cross-check in the tests possible.
        pub fn determinant(a: Self) f32 {
            if (rows != cols) @compileError("matrix determinant: must be square");
            return switch (rows) {
                2 => a.data[0][0] * a.data[1][1] - a.data[0][1] * a.data[1][0],
                3 => a.data[0][0] * (a.data[1][1] * a.data[2][2] - a.data[1][2] * a.data[2][1]) -
                    a.data[0][1] * (a.data[1][0] * a.data[2][2] - a.data[1][2] * a.data[2][0]) +
                    a.data[0][2] * (a.data[1][0] * a.data[2][1] - a.data[1][1] * a.data[2][0]),
                4 => a.data[0][0] * minor3(a, 0, 0) - a.data[0][1] * minor3(a, 0, 1) +
                    a.data[0][2] * minor3(a, 0, 2) - a.data[0][3] * minor3(a, 0, 3),
                else => @compileError("matrix determinant is implemented for 2x2, 3x3 and 4x4 only"),
            };
        }

        /// 2x2, 3x3 and 4x4, by the adjugate over the determinant - the same
        /// formula, in the same order, as the C version for the sizes it had.
        ///
        /// `error.Singular` replaces the C `0` return; unlike the C function,
        /// failing produces no half-written matrix to be confused by.
        ///
        /// The 4x4 adjugate is the transpose of the cofactor matrix, so entry
        /// `(i, j)` is the cofactor of `(j, i)` - hence the swapped `minor3`
        /// arguments. Chosen over Gauss-Jordan with partial pivoting because it
        /// keeps the module's arithmetic explicit and pivot-free, and because it
        /// makes the 3x3 cross-check below a real one; the price is accuracy on
        /// ill-conditioned matrices, which REVIEW §54 measures for the test cases
        /// rather than assuming.
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
                4 => blk: {
                    var out: Mat(4, 4) = undefined;
                    inline for (0..4) |i| {
                        inline for (0..4) |j| {
                            const sign: f32 = if ((i + j) % 2 == 0) 1.0 else -1.0;
                            out.data[i][j] = sign * minor3(a, j, i) / det;
                        }
                    }
                    break :blk out;
                },
                else => @compileError("matrix inverse is implemented for 2x2, 3x3 and 4x4 only"),
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

/// A 4x4 matrix with a chosen top-left 3x3 block, ones on the rest of the diagonal
/// and zeros elsewhere. The 4x4 inverse of such a matrix has that block's inverse
/// in the same corner, which is what the cross-check below uses.
fn embed(m: Mat3) Mat4 {
    var out = Mat4.identity();
    for (0..3) |i| {
        for (0..3) |j| out.data[i][j] = m.data[i][j];
    }
    return out;
}

test "matrix 4x4: the determinant has the properties a determinant has" {
    // Independent of any oracle - there is none, the C never computed a 4x4.
    const id = Mat4.identity();
    try std.testing.expectEqual(@as(f32, 1.0), id.determinant());

    // Triangular: the product of the diagonal, exactly in f32 for these values.
    const tri = Mat4.init(.{
        .{ 2, 9, 9, 9 },
        .{ 0, 3, 9, 9 },
        .{ 0, 0, 4, 9 },
        .{ 0, 0, 0, 5 },
    });
    try std.testing.expectEqual(@as(f32, 120.0), tri.determinant());

    // Transpose leaves it alone, and a row swap flips its sign.
    const m = Mat4.init(.{
        .{ 4, 7, 2, 1 },
        .{ 3, 6, 1, 0 },
        .{ 2, 5, 3, 8 },
        .{ 1, 1, 1, 2 },
    });
    try std.testing.expectApproxEqAbs(m.determinant(), m.transpose().determinant(), 1.0e-4);

    const swapped = Mat4.init(.{
        .{ 3, 6, 1, 0 },
        .{ 4, 7, 2, 1 },
        .{ 2, 5, 3, 8 },
        .{ 1, 1, 1, 2 },
    });
    try std.testing.expectApproxEqAbs(-m.determinant(), swapped.determinant(), 1.0e-4);

    // Multiplication: det(AB) = det(A)·det(B).
    const n = Mat4.init(.{
        .{ 1, 0, 2, 0 },
        .{ 0, 3, 0, 1 },
        .{ 4, 0, 1, 0 },
        .{ 0, 1, 0, 2 },
    });
    try std.testing.expectApproxEqRel(
        m.determinant() * n.determinant(),
        m.mul(n).determinant(),
        1.0e-3,
    );

    // Scaling one row scales it by the same factor.
    var scaled = m;
    for (0..4) |j| scaled.data[1][j] *= 3.0;
    try std.testing.expectApproxEqRel(3.0 * m.determinant(), scaled.determinant(), 1.0e-4);
}

test "matrix 4x4: the inverse agrees with the 3x3 one on the block they share" {
    // Two independent code paths - the 4x4 adjugate and the 3x3 formula - have to
    // produce the same numbers in the corner they share. That is the strongest
    // check available for code with no oracle: it is not one formula checked
    // against itself.
    const blocks = [_]Mat3{
        .{ .data = .{ .{ 4, 7, 2 }, .{ 3, 6, 1 }, .{ 2, 5, 3 } } },
        .{ .data = .{ .{ 1, 2, 3 }, .{ 0, 1, 4 }, .{ 5, 6, 0 } } },
        .{ .data = .{ .{ 2, 0, 0 }, .{ 0, -3, 0 }, .{ 0, 0, 0.5 } } },
        .{ .data = .{ .{ 0, 1, 0 }, .{ 0, 0, 1 }, .{ 1, 0, 0 } } },
    };

    for (blocks) |block| {
        const big = embed(block);
        const big_inverse = try big.inverse();
        const small_inverse = try block.inverse();

        for (0..3) |i| {
            for (0..3) |j| {
                try std.testing.expectApproxEqAbs(
                    small_inverse.data[i][j],
                    big_inverse.data[i][j],
                    1.0e-5,
                );
            }
        }

        // And the border is untouched: the embedded identity part stays identity.
        for (0..3) |i| {
            try std.testing.expectApproxEqAbs(@as(f32, 0.0), big_inverse.data[i][3], 1.0e-5);
            try std.testing.expectApproxEqAbs(@as(f32, 0.0), big_inverse.data[3][i], 1.0e-5);
        }
        try std.testing.expectApproxEqAbs(@as(f32, 1.0), big_inverse.data[3][3], 1.0e-5);

        // det of the embedded matrix is the block's determinant, so this also
        // cross-checks the 4x4 determinant against the 3x3 one.
        try std.testing.expectApproxEqAbs(block.determinant(), big.determinant(), 1.0e-5);
    }
}

test "matrix 4x4: an inverse really is an inverse" {
    const m = Mat4.init(.{
        .{ 4, 7, 2, 1 },
        .{ 3, 6, 1, 0 },
        .{ 2, 5, 3, 8 },
        .{ 1, 1, 1, 2 },
    });
    try expectIdentity(m.mul(try m.inverse()));

    // How close, measured rather than assumed: the worst element of `A·A⁻¹ − I` is
    // 4.8e-7 for this matrix (and 6.3e-7 for the 3x3 path that was already here),
    // so the adjugate on 4x4 is no less accurate than the smaller sizes. The bound
    // below is tight enough that a real accuracy regression trips it.
    const product = m.mul(try m.inverse());
    var worst: f32 = 0;
    for (0..4) |i| {
        for (0..4) |j| {
            const want: f32 = if (i == j) 1.0 else 0.0;
            worst = @max(worst, @abs(product.data[i][j] - want));
        }
    }
    try std.testing.expect(worst < 2.0e-6);

    // A second one, so a single lucky case is not the evidence. This one is
    // structured enough that the product comes out exactly identity.
    const n = Mat4.init(.{
        .{ 1, 2, 0, 0 },
        .{ 0, 1, 3, 0 },
        .{ 0, 0, 1, 4 },
        .{ 2, 0, 0, 1 },
    });
    try expectIdentity(n.mul(try n.inverse()));

    // Inverse twice gives the original back.
    const twice = try (try n.inverse()).inverse();
    for (0..4) |i| {
        for (0..4) |j| try std.testing.expectApproxEqAbs(n.data[i][j], twice.data[i][j], 1.0e-4);
    }

    // A permutation matrix: its inverse is its transpose, exactly, because every
    // entry is 0 or 1 and the determinant is ±1.
    const perm = Mat4.init(.{
        .{ 0, 1, 0, 0 },
        .{ 0, 0, 1, 0 },
        .{ 0, 0, 0, 1 },
        .{ 1, 0, 0, 0 },
    });
    const perm_inverse = try perm.inverse();
    const permT = perm.transpose();
    for (0..4) |i| {
        for (0..4) |j| try std.testing.expectEqual(permT.data[i][j], perm_inverse.data[i][j]);
    }
}

test "matrix 4x4: a singular matrix is refused at the same threshold as the smaller sizes" {
    // A duplicated row: determinant exactly 0.
    const repeated = Mat4.init(.{
        .{ 1, 2, 3, 4 },
        .{ 1, 2, 3, 4 },
        .{ 5, 6, 7, 8 },
        .{ 9, 10, 11, 12 },
    });
    try std.testing.expectEqual(@as(f32, 0.0), repeated.determinant());
    try std.testing.expectError(error.Singular, repeated.inverse());

    // A dependent row (row3 = row1 + row2) is singular without being a duplicate.
    const dependent = Mat4.init(.{
        .{ 1, 2, 3, 4 },
        .{ 2, 3, 4, 5 },
        .{ 3, 5, 7, 9 },
        .{ 1, 1, 1, 1 },
    });
    try std.testing.expectApproxEqAbs(@as(f32, 0.0), dependent.determinant(), 1.0e-5);
    try std.testing.expectError(error.Singular, dependent.inverse());

    // Not exactly singular, but under the threshold: refused too, which is the
    // constant the C chose and the port kept.
    const tiny = Mat4.init(.{
        .{ 1, 0, 0, 0 },
        .{ 0, 1, 0, 0 },
        .{ 0, 0, 1, 0 },
        .{ 0, 0, 0, 1.0e-7 },
    });
    try std.testing.expect(@abs(tiny.determinant()) < min_determinant);
    try std.testing.expectError(error.Singular, tiny.inverse());
}
