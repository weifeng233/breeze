//! The Sobel operator, ported from `include/breeze/image/sobel_operator.h`.
//!
//! Three functions: the gradient magnitude, the magnitude plus a quantised
//! gradient direction, and the magnitude put through a threshold. The two C
//! details worth naming before the code:
//!
//!   * The magnitude is `(|gx| + |gy|) / 2` in **integer** arithmetic - the
//!     header's "not the square-root approximation" - so it is an L1 gradient
//!     halved, not a Euclidean one. Linear ramps therefore measure differently
//!     from what a Sobel magnitude usually means, and the corpus records that.
//!   * The direction is `(atan2(gy, gx) + PI) * 128 / PI` cast to a byte. For a
//!     leftward gradient the angle is exactly +PI and that expression is exactly
//!     256, which does not fit in an `unsigned char`: the C's cast is undefined
//!     behaviour there. x86 truncates it to 0 - the same byte as angle -PI - and
//!     glibc and mingw agree on that byte, so the port reproduces the measured
//!     result and says why. See `directionOf` below and docs/REVIEW.md §43.

const std = @import("std");

const common = @import("common.zig");
const corpus_mod = @import("../math/corpus.zig");

pub const Error = common.Error;

/// The C spells its π as `3.14159265f` in both places it appears; the port keeps
/// that literal rather than a more accurate one, because the direction byte is a
/// truncation and a different π could move it by one.
const pi: f32 = 3.14159265;

const Gradient = struct { gx: i32, gy: i32 };

inline fn px(img: []const u8, index: usize) i32 {
    return @intCast(img[index]);
}

/// The two Sobel gradients at `(x, y)`, which must be an interior pixel.
inline fn gradientAt(img: []const u8, stride: usize, x: usize, y: usize) Gradient {
    const p00 = (y - 1) * stride + (x - 1);
    const p01 = (y - 1) * stride + x;
    const p02 = (y - 1) * stride + (x + 1);
    const p10 = y * stride + (x - 1);
    const p12 = y * stride + (x + 1);
    const p20 = (y + 1) * stride + (x - 1);
    const p21 = (y + 1) * stride + x;
    const p22 = (y + 1) * stride + (x + 1);

    //           -1 0 1            -1 -2 -1
    //  gx over  -2 0 2   ,  gy over  0  0  0
    //           -1 0 1               1  2  1
    return .{
        .gx = -px(img, p00) - 2 * px(img, p10) - px(img, p20) +
            px(img, p02) + 2 * px(img, p12) + px(img, p22),
        .gy = -px(img, p00) - 2 * px(img, p01) - px(img, p02) +
            px(img, p20) + 2 * px(img, p21) + px(img, p22),
    };
}

/// `(|gx| + |gy|) / 2`, clamped to a byte.
///
/// The C also has `if (magnitude < 0) magnitude = 0;` after this. It cannot fire:
/// the value is a sum of absolute values halved, so it is never negative. Dropped
/// as provably dead rather than copied (docs/REVIEW.md §43).
inline fn magnitudeOf(g: Gradient) u8 {
    const magnitude = @divTrunc(@abs(g.gx) + @abs(g.gy), 2);
    return if (magnitude > 255) 255 else @intCast(magnitude);
}

/// The direction byte, or 0 when there is no gradient at all.
///
/// The C skips `atan2f` when `gx == 0 && gy == 0` (the angle would be 0/0), which
/// is why a flat image has direction 0 rather than the encoding's midpoint 128.
fn directionOf(g: Gradient) u8 {
    if (g.gx == 0 and g.gy == 0) return 0;

    const angle = std.math.atan2(
        @as(f32, @floatFromInt(g.gy)),
        @as(f32, @floatFromInt(g.gx)),
    );
    const scaled = (angle + pi) * 128.0 / pi;

    // `scaled` is in [0, 256]. The C casts the 256 case straight to
    // `unsigned char`, which is undefined; on x86 the truncation yields 0, and
    // that is the byte both test libms produce for a leftward gradient. Spelled
    // out here instead of relying on the same undefined conversion.
    if (scaled >= 256.0) return 0;
    return @intFromFloat(scaled);
}

/// Every pixel of the region becomes its gradient magnitude; the border stays 0.
pub fn sobel(
    src: []const u8,
    dst: []u8,
    width: usize,
    height: usize,
    stride_bytes: usize,
) Error!void {
    const stride = common.strideOf(width, stride_bytes);
    try common.checkRegion(src.len, width, height, stride);
    try common.checkRegion(dst.len, width, height, stride);

    // The C clears the whole image first, and that is what leaves the 1-pixel
    // border zero: the 3x3 window never visits it, so whatever was in `dst` would
    // otherwise survive there.
    for (0..height) |y| {
        for (0..width) |x| dst[y * stride + x] = 0;
    }

    // Smaller than the 3x3 window means no interior at all: the C's loops are
    // empty because `1 < width - 1` is false, and this says the same thing
    // without subtracting to a negative length.
    if (width < 3 or height < 3) return;

    for (1..height - 1) |y| {
        for (1..width - 1) |x| {
            dst[y * stride + x] = magnitudeOf(gradientAt(src, stride, x, y));
        }
    }
}

/// The magnitude and the quantised direction, side by side.
pub fn sobelWithDirection(
    src: []const u8,
    magnitude: []u8,
    direction: []u8,
    width: usize,
    height: usize,
    stride_bytes: usize,
) Error!void {
    const stride = common.strideOf(width, stride_bytes);
    try common.checkRegion(src.len, width, height, stride);
    try common.checkRegion(magnitude.len, width, height, stride);
    try common.checkRegion(direction.len, width, height, stride);

    for (0..height) |y| {
        for (0..width) |x| {
            magnitude[y * stride + x] = 0;
            direction[y * stride + x] = 0;
        }
    }

    if (width < 3 or height < 3) return;

    for (1..height - 1) |y| {
        for (1..width - 1) |x| {
            const g = gradientAt(src, stride, x, y);
            magnitude[y * stride + x] = magnitudeOf(g);
            direction[y * stride + x] = directionOf(g);
        }
    }
}

/// The magnitude, then `dst = magnitude > threshold ? 255 : 0` over every pixel.
///
/// `scratch` is the C's `malloc(stride * height)`, and it is still needed even
/// though the threshold could be applied in one pass: the C buffers the
/// magnitudes so that `dst` may alias `src`, and a single-pass version would read
/// neighbours it had already overwritten. The port keeps that property and takes
/// the buffer from the caller.
pub fn sobelThreshold(
    src: []const u8,
    dst: []u8,
    scratch: []u8,
    width: usize,
    height: usize,
    threshold: u8,
    stride_bytes: usize,
) Error!void {
    const stride = common.strideOf(width, stride_bytes);
    try common.checkRegion(src.len, width, height, stride);
    try common.checkRegion(dst.len, width, height, stride);
    try common.checkRegion(scratch.len, width, height, stride);

    try sobel(src, scratch, width, height, stride_bytes);

    for (0..height) |y| {
        for (0..width) |x| {
            const idx = y * stride + x;
            dst[idx] = if (scratch[idx] > threshold) 255 else 0;
        }
    }
}

// --- tests ------------------------------------------------------------------

const varied = [16]u8{
    0, 0,   0,   0,
    0, 10,  60,  0,
    0, 120, 200, 0,
    0, 0,   0,   0,
};

const flat = [_]u8{100} ** 16;

const right_step = [16]u8{
    0, 0, 200, 200,
    0, 0, 200, 200,
    0, 0, 200, 200,
    0, 0, 200, 200,
};

const left_step = [16]u8{
    200, 200, 0, 0,
    200, 200, 0, 0,
    200, 200, 0, 0,
    200, 200, 0, 0,
};

fn expectBytes(corpus: *const corpus_mod.Corpus, name: []const u8, bytes: []const u8) !void {
    var as_floats: [64]f32 = undefined;
    for (bytes, 0..) |b, i| as_floats[i] = @floatFromInt(b);
    try corpus.expectValues(name, as_floats[0..bytes.len]);
}

test "sobel: the magnitude image matches the C on every recorded image" {
    const corpus = try corpus_mod.Corpus.load();

    var mag = [_]u8{0xEE} ** 16;
    try sobel(&varied, &mag, 4, 4, 0);
    try expectBytes(&corpus, "sobel_magnitude", &mag);

    // The border is cleared, not left as it was: 0xEE is what the buffer held.
    for (0..4) |i| {
        try std.testing.expectEqual(@as(u8, 0), mag[i]);
        try std.testing.expectEqual(@as(u8, 0), mag[12 + i]);
        try std.testing.expectEqual(@as(u8, 0), mag[i * 4]);
        try std.testing.expectEqual(@as(u8, 0), mag[i * 4 + 3]);
    }

    var flat_mag = [_]u8{0xEE} ** 16;
    try sobel(&flat, &flat_mag, 4, 4, 0);
    try expectBytes(&corpus, "sobel_uniform_magnitude", &flat_mag);

    // Smaller than the window: cleared, and nothing else.
    var small = [_]u8{0xEE} ** 4;
    try sobel(&[_]u8{ 10, 20, 30, 40 }, &small, 2, 2, 0);
    try expectBytes(&corpus, "sobel_small_magnitude", &small);

    // The whole width is 1: the x range is empty rather than underflowing.
    var column = [_]u8{0xEE} ** 4;
    try sobel(&[_]u8{ 10, 20, 30, 40 }, &column, 1, 4, 0);
    try std.testing.expectEqualSlices(u8, &[_]u8{ 0, 0, 0, 0 }, &column);
}

test "sobel: the direction image matches the C, seam included" {
    const corpus = try corpus_mod.Corpus.load();

    var mag = [_]u8{0xEE} ** 16;
    var dir = [_]u8{0xEE} ** 16;
    try sobelWithDirection(&varied, &mag, &dir, 4, 4, 0);
    try expectBytes(&corpus, "sobel_direction_magnitude", &mag);
    try expectBytes(&corpus, "sobel_direction", &dir);

    // The two functions must agree on the magnitude they compute.
    var mag_only = [_]u8{0xEE} ** 16;
    try sobel(&varied, &mag_only, 4, 4, 0);
    try std.testing.expectEqualSlices(u8, &mag_only, &mag);

    // A flat image never reaches `atan2f`, so it keeps the initial byte rather
    // than the encoding's midpoint.
    try sobelWithDirection(&flat, &mag, &dir, 4, 4, 0);
    try expectBytes(&corpus, "sobel_uniform_direction", &dir);

    // Leftward against rightward: the same magnitude, different direction - and
    // the leftward one is the seam, where the C's cast is undefined and x86
    // yields the same byte as angle -PI.
    var left_mag = [_]u8{0xEE} ** 16;
    var left_dir = [_]u8{0xEE} ** 16;
    try sobelWithDirection(&left_step, &left_mag, &left_dir, 4, 4, 0);
    try expectBytes(&corpus, "sobel_left_step_magnitude", &left_mag);
    try expectBytes(&corpus, "sobel_left_step_direction", &left_dir);

    var right_mag = [_]u8{0xEE} ** 16;
    var right_dir = [_]u8{0xEE} ** 16;
    try sobelWithDirection(&right_step, &right_mag, &right_dir, 4, 4, 0);
    try expectBytes(&corpus, "sobel_right_step_magnitude", &right_mag);
    try expectBytes(&corpus, "sobel_right_step_direction", &right_dir);

    try std.testing.expectEqualSlices(u8, &left_mag, &right_mag);
    try std.testing.expectEqual(@as(u8, 0), left_dir[5]);
    try std.testing.expectEqual(@as(u8, 128), right_dir[5]);

    var small_mag = [_]u8{0xEE} ** 4;
    var small_dir = [_]u8{0xEE} ** 4;
    try sobelWithDirection(&[_]u8{ 10, 20, 30, 40 }, &small_mag, &small_dir, 2, 2, 0);
    try expectBytes(&corpus, "sobel_small_direction_magnitude", &small_mag);
    try expectBytes(&corpus, "sobel_small_direction", &small_dir);
}

test "sobel: the threshold keeps strictly-above pixels" {
    const corpus = try corpus_mod.Corpus.load();

    var mag = [_]u8{0} ** 16;
    try sobel(&varied, &mag, 4, 4, 0);

    // Both thresholds are read back out of the image itself, so each case sits
    // exactly on the boundary: one on a clamped magnitude, one on the only
    // interior magnitude that is not clamped.
    var dst = [_]u8{0xEE} ** 16;
    var scratch = [_]u8{0} ** 16;
    try sobelThreshold(&varied, &dst, &scratch, 4, 4, mag[10], 0);
    try corpus.expectInt("sobel_threshold_at_magnitude", mag[10]);
    try expectBytes(&corpus, "sobel_threshold_at_magnitude_result", &dst);

    try sobelThreshold(&varied, &dst, &scratch, 4, 4, mag[9], 0);
    try corpus.expectInt("sobel_threshold_at_clamped", mag[9]);
    try expectBytes(&corpus, "sobel_threshold_at_clamped_result", &dst);

    // Threshold 0: every non-zero magnitude survives, and only those.
    try sobelThreshold(&varied, &dst, &scratch, 4, 4, 0, 0);
    try expectBytes(&corpus, "sobel_threshold_zero", &dst);
    for (mag, dst) |m, d| {
        try std.testing.expectEqual(@as(u8, if (m > 0) 255 else 0), d);
    }
}

test "sobel: a stride wider than the image neither reads nor writes the padding" {
    const corpus = try corpus_mod.Corpus.load();

    var widesrc = [_]u8{0xAA} ** 24;
    for (0..4) |y| @memcpy(widesrc[y * 6 ..][0..4], varied[y * 4 ..][0..4]);

    var mag = [_]u8{0xEE} ** 24;
    try sobel(&widesrc, &mag, 4, 4, 6);
    try expectBytes(&corpus, "sobel_stride_magnitude", &mag);

    // 0xEE is 238: a stray write would be visible, and a read of the padding
    // would change the gradients beside it.
    for (0..4) |y| {
        try std.testing.expectEqual(@as(u8, 0xEE), mag[y * 6 + 4]);
        try std.testing.expectEqual(@as(u8, 0xEE), mag[y * 6 + 5]);
    }
}

test "sobel: an undersized buffer is an error, not a read or write past the end" {
    var dst: [15]u8 = undefined;
    try std.testing.expectError(Error.BufferTooSmall, sobel(&varied, &dst, 4, 4, 0));
    try std.testing.expectError(Error.BufferTooSmall, sobel(&dst, &dst, 4, 4, 0));
    try std.testing.expectError(Error.InvalidSize, sobel(&varied, &dst, 0, 4, 0));

    // The scratch is checked too - the C's malloc failure check, made precise.
    var exact: [16]u8 = undefined;
    var small_scratch: [15]u8 = undefined;
    try std.testing.expectError(
        Error.BufferTooSmall,
        sobelThreshold(&varied, &exact, &small_scratch, 4, 4, 0, 0),
    );
    try std.testing.expectError(
        Error.BufferTooSmall,
        sobelThreshold(&varied, &small_scratch, &exact, 4, 4, 0, 0),
    );
    try sobelThreshold(&varied, &exact, &exact, 4, 4, 0, 0);
}
