//! Otsu's method, ported from `include/breeze/image/otsu_threshold.h`.
//!
//! The C routine is the textbook between-class variance maximisation, and two
//! of its details are load-bearing enough to have their own corpus cases:
//!
//!   * `wB == 0` **continues** - a bin with no background yet is skipped, so the
//!     threshold never lands below the first populated bin;
//!   * `wF == 0` **breaks** - once every pixel is background there is no
//!     foreground left to separate, and the loop stops instead of dividing by
//!     zero.
//!
//! A uniform image therefore returns the *initial* 0 rather than its own value:
//! the scan breaks on the first populated bin, having compared nothing. That
//! looks wrong and is what the C does, so `otsu_uniform` pins it.

const std = @import("std");

const common = @import("common.zig");
const binary = @import("binary.zig");
const corpus_mod = @import("../math/corpus.zig");

pub const Error = common.Error;

/// The threshold that maximises the between-class variance of `src`.
pub fn otsu(src: []const u8, width: usize, height: usize, stride_bytes: usize) Error!u8 {
    const stride = common.strideOf(width, stride_bytes);
    try common.checkRegion(src.len, width, height, stride);

    var histogram: [256]i32 = @splat(0);
    var sum: f32 = 0;
    for (0..height) |y| {
        for (0..width) |x| {
            const value = src[y * stride + x];
            histogram[value] += 1;
            sum += @floatFromInt(value);
        }
    }

    const total_pixels: f32 = @floatFromInt(width * height);
    var sum_b: f32 = 0;
    var w_b: f32 = 0;
    var max_variance: f32 = 0;
    var threshold_value: u8 = 0;

    for (0..256) |i| {
        w_b += @floatFromInt(histogram[i]);
        // Both guards below are mutations these tests cannot see, and that is a
        // measured result rather than an assumption: when `w_b == 0` the running
        // sum is 0 as well (both accumulate at the same bins in the same order),
        // so the background mean two lines down is 0/0 = NaN and
        // `NaN > max_variance` is false; and once `w_b` reaches the total it stays
        // there, so every later iteration would be skipped by that same
        // comparison. Deleting either guard leaves the whole suite green - see
        // docs/REVIEW.md §42. They stay because the C has them, and because they
        // are what keeps a division by zero out of the arithmetic at all.
        if (w_b == 0) continue;
        const w_f = total_pixels - w_b;
        if (w_f == 0) break;

        // The C multiplies two ints and adds the product to a float. Doing the
        // multiply in float would round for bins holding more than ~65k pixels;
        // widening to i64 keeps the C's exact integer product without the
        // overflow its `int` would have.
        const weighted: i64 = @as(i64, @intCast(i)) * @as(i64, histogram[i]);
        sum_b += @floatFromInt(weighted);

        const mean_b = sum_b / w_b;
        const mean_f = (sum - sum_b) / w_f;
        const variance = w_b * w_f * (mean_b - mean_f) * (mean_b - mean_f);
        if (variance > max_variance) {
            max_variance = variance;
            threshold_value = @intCast(i);
        }
    }
    return threshold_value;
}

/// Otsu's threshold, then the forward threshold with it.
pub fn applyOtsu(
    src: []const u8,
    dst: []u8,
    width: usize,
    height: usize,
    max_value: u8,
    stride_bytes: usize,
) Error!u8 {
    const value = try otsu(src, width, height, stride_bytes);
    try binary.threshold(src, dst, width, height, value, max_value, stride_bytes);
    return value;
}

// --- tests ------------------------------------------------------------------

// The exact images `tools/corpus/gen_math_corpus.c` passes, byte for byte.
//
// They are not interchangeable with rearrangements of themselves. The first
// version of these tests sorted `img` into six dark pixels followed by six
// bright ones and compared against the corpus: same multiset, so the same
// histogram and the same threshold of 14, so the threshold case passed - and the
// applied image failed at its first pixel, because `img` interleaves the two
// clusters in a different order. A histogram is all Otsu looks at; the image it
// produces is not a function of the histogram alone.
const img = [12]u8{ 10, 12, 200, 210, 11, 13, 205, 190, 9, 14, 195, 215 };
const flat = [6]u8{ 77, 77, 77, 77, 77, 77 };
const two = [8]u8{ 5, 5, 5, 5, 200, 200, 200, 200 };
const zero = [_]u8{0} ** 12;
const maxv = [_]u8{255} ** 12;

fn expectBytes(corpus: *const corpus_mod.Corpus, name: []const u8, bytes: []const u8) !void {
    var as_floats: [64]f32 = undefined;
    for (bytes, 0..) |b, i| as_floats[i] = @floatFromInt(b);
    try corpus.expectValues(name, as_floats[0..bytes.len]);
}

test "otsu matches the C threshold for every image the corpus records" {
    const corpus = try corpus_mod.Corpus.load();

    try corpus.expectInt("otsu_bimodal", try otsu(&img, 4, 3, 0));
    try corpus.expectInt("otsu_uniform", try otsu(&flat, 3, 2, 0));
    try corpus.expectInt("otsu_two_levels", try otsu(&two, 4, 2, 0));

    // Both ends of the range, where the two guards fire in opposite ways: at 0
    // the background is still empty at the first populated bin (`continue`), at
    // 255 the foreground empties at the last one (`break`). Neither can raise the
    // variance above the initial 0, so both answer 0 - and neither answer says
    // anything about the pixels, which is exactly why the C has to be asked.
    try corpus.expectInt("otsu_all_zero", try otsu(&zero, 4, 3, 0));
    try corpus.expectInt("otsu_all_max", try otsu(&maxv, 4, 3, 0));

    // The uniform case is the one worth naming out loud: nothing was ever
    // compared, so the answer is the initial value rather than the pixel value.
    try std.testing.expectEqual(@as(u8, 77), flat[0]);
    try std.testing.expectEqual(@as(u8, 0), try otsu(&flat, 3, 2, 0));
}

test "applyOtsu returns the threshold it used, and matches the C image" {
    const corpus = try corpus_mod.Corpus.load();

    var dst = [_]u8{0} ** 12;
    const value = try applyOtsu(&img, &dst, 4, 3, 255, 0);
    try corpus.expectInt("apply_otsu_threshold", value);
    try expectBytes(&corpus, "apply_otsu_result", &dst);

    // The returned value is the one that was applied, pixel by pixel.
    for (img, dst) |s, d| {
        try std.testing.expectEqual(@as(u8, if (s > value) 255 else 0), d);
    }
}

test "otsu: the padding is outside the histogram, and survives the write" {
    const corpus = try corpus_mod.Corpus.load();

    // The image at stride 5 as the generator lays it out, padding 0xAA (170) in
    // the trailing column: reading it as a pixel would pull the histogram
    // somewhere the packed image never goes.
    var widesrc = [_]u8{0xAA} ** 15;
    for (0..3) |y| @memcpy(widesrc[y * 5 ..][0..4], img[y * 4 ..][0..4]);

    try corpus.expectInt("otsu_stride", try otsu(&widesrc, 4, 3, 5));
    try std.testing.expectEqual(try otsu(&img, 4, 3, 0), try otsu(&widesrc, 4, 3, 5));

    // 0xEE is 238, far above the threshold of 14: an over-eager write would turn
    // the padding into 255, so "still 0xEE" is a real assertion.
    var out = [_]u8{0xEE} ** 15;
    const value = try applyOtsu(&widesrc, &out, 4, 3, 255, 5);
    try corpus.expectInt("apply_otsu_stride_threshold", value);
    try expectBytes(&corpus, "apply_otsu_stride_result", &out);
    for (0..3) |y| try std.testing.expectEqual(@as(u8, 0xEE), out[y * 5 + 4]);
}

test "otsu: the pixel count is the image's, not the stride's" {
    const corpus = try corpus_mod.Corpus.load();

    // A histogram that is not two far-apart clusters. Here the argmax really does
    // depend on how many pixels the image has, so this is the case that tells the
    // C's `width * height` apart from `stride * height`: 60 when the padding is
    // excluded, 250 when it is counted. It exists because a probe that made that
    // change stayed green against every other case - see probe_otsu_total.c.
    var mixed = [_]u8{0xAA} ** 12;
    mixed[0] = 60;
    mixed[1] = 60;
    mixed[2] = 10;
    mixed[3] = 200;
    mixed[6] = 10;
    mixed[7] = 10;
    mixed[8] = 250;
    mixed[9] = 10;

    try corpus.expectInt("otsu_stride_total", try otsu(&mixed, 4, 2, 6));

    var out = [_]u8{0xEE} ** 12;
    const value = try applyOtsu(&mixed, &out, 4, 2, 255, 6);
    try corpus.expectInt("apply_otsu_stride_total_threshold", value);
    try expectBytes(&corpus, "apply_otsu_stride_total_result", &out);

    for (0..2) |y| {
        try std.testing.expectEqual(@as(u8, 0xEE), out[y * 6 + 4]);
        try std.testing.expectEqual(@as(u8, 0xEE), out[y * 6 + 5]);
    }
}

test "otsu: applyOtsu hands its stride to the threshold it computes" {
    const corpus = try corpus_mod.Corpus.load();

    // Read strided this image answers 10; read as if the rows were contiguous it
    // answers 120. So this case is what pins the inner `otsu` call receiving the
    // caller's stride - a probe that hard-coded 0 there stayed green until this
    // case existed, because the two earlier strided images agreed either way.
    var rows = [_]u8{0xAA} ** 12;
    rows[0] = 10;
    rows[1] = 250;
    rows[2] = 120;
    rows[3] = 120;
    rows[6] = 200;
    rows[7] = 120;
    rows[8] = 10;
    rows[9] = 200;

    try corpus.expectInt("otsu_stride_read", try otsu(&rows, 4, 2, 6));
    try std.testing.expectEqual(@as(u8, 120), try otsu(&rows, 4, 2, 0));

    var out = [_]u8{0xEE} ** 12;
    const value = try applyOtsu(&rows, &out, 4, 2, 255, 6);
    try corpus.expectInt("apply_otsu_stride_read_threshold", value);
    try expectBytes(&corpus, "apply_otsu_stride_read_result", &out);
}

test "otsu: an undersized buffer is an error, not a read past the end" {
    // The C returns 0 for a null pointer and for a width of 0 alike, and 0 is
    // also a legitimate threshold. The error union is what tells them apart.
    const small: [8]u8 = undefined;
    try std.testing.expectError(Error.BufferTooSmall, otsu(&small, 4, 3, 0));
    try std.testing.expectError(Error.InvalidSize, otsu(&small, 0, 3, 0));

    const exact: [12]u8 = undefined;
    _ = try otsu(&exact, 4, 3, 0);

    var out: [8]u8 = undefined;
    try std.testing.expectError(Error.BufferTooSmall, applyOtsu(&img, &out, 4, 3, 255, 0));
}
