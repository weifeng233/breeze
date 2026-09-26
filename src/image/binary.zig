//! Binary thresholding, ported from `include/breeze/image/binary_threshold.h`.
//!
//! Two functions and one boundary rule: the forward version keeps
//! `src > threshold`, the inverse keeps `src <= threshold`. At exactly the
//! threshold they are complements, and the corpus records both - a pair that
//! would be easy to get subtly wrong in one direction and never notice.

const std = @import("std");

const common = @import("common.zig");
const corpus_mod = @import("../math/corpus.zig");

pub const Error = common.Error;

/// `dst = src > threshold ? max_value : 0`.
pub fn threshold(
    src: []const u8,
    dst: []u8,
    width: usize,
    height: usize,
    threshold_value: u8,
    max_value: u8,
    stride_bytes: usize,
) Error!void {
    const stride = common.strideOf(width, stride_bytes);
    try common.checkRegion(src.len, width, height, stride);
    try common.checkRegion(dst.len, width, height, stride);

    for (0..height) |y| {
        for (0..width) |x| {
            const idx = y * stride + x;
            dst[idx] = if (src[idx] > threshold_value) max_value else 0;
        }
    }
}

/// `dst = src <= threshold ? max_value : 0`.
pub fn inverseThreshold(
    src: []const u8,
    dst: []u8,
    width: usize,
    height: usize,
    threshold_value: u8,
    max_value: u8,
    stride_bytes: usize,
) Error!void {
    const stride = common.strideOf(width, stride_bytes);
    try common.checkRegion(src.len, width, height, stride);
    try common.checkRegion(dst.len, width, height, stride);

    for (0..height) |y| {
        for (0..width) |x| {
            const idx = y * stride + x;
            dst[idx] = if (src[idx] <= threshold_value) max_value else 0;
        }
    }
}

// --- tests ------------------------------------------------------------------

const img = [12]u8{ 10, 12, 200, 210, 11, 13, 205, 190, 9, 14, 195, 215 };

fn expectBytes(corpus: *const corpus_mod.Corpus, name: []const u8, bytes: []const u8) !void {
    var as_floats: [64]f32 = undefined;
    for (bytes, 0..) |b, i| as_floats[i] = @floatFromInt(b);
    try corpus.expectValues(name, as_floats[0..bytes.len]);
}

test "binary threshold: forward and inverse match the C answers" {
    const corpus = try corpus_mod.Corpus.load();

    var dst = [_]u8{0} ** 12;
    try threshold(&img, &dst, 4, 3, 100, 255, 0);
    try expectBytes(&corpus, "threshold_forward", &dst);

    var inv = [_]u8{0} ** 12;
    try inverseThreshold(&img, &inv, 4, 3, 100, 255, 0);
    try expectBytes(&corpus, "threshold_inverse", &inv);

    // Independent of the corpus: the two are complements, byte for byte.
    for (dst, inv) |a, b| {
        try std.testing.expectEqual(@as(u8, 255), a + b);
    }

    // And the boundary rule is what makes them complements: at exactly the
    // threshold the forward version drops the pixel and the inverse keeps it.
    var one = [_]u8{100};
    var out = [_]u8{0};
    try threshold(&one, &out, 1, 1, 100, 255, 0);
    try corpus.expectInt("threshold_at_value_forward", out[0]);
    try std.testing.expectEqual(@as(u8, 0), out[0]);
    try inverseThreshold(&one, &out, 1, 1, 100, 255, 0);
    try corpus.expectInt("threshold_at_value_inverse", out[0]);
    try std.testing.expectEqual(@as(u8, 255), out[0]);
}

test "binary threshold: a strided image only writes its own region" {
    const corpus = try corpus_mod.Corpus.load();

    // The image laid out at stride 5 exactly as the C generator lays it out. The
    // *source* needs its padding too: the first version of that case handed the C
    // a packed 12-byte image at stride 5, so its last pixel read landed one byte
    // past the array and the recorded value came from the stack. The buffer
    // check below is what refused to reproduce it.
    var widesrc = [_]u8{0xAA} ** 15;
    for (0..3) |y| @memcpy(widesrc[y * 5 ..][0..4], img[y * 4 ..][0..4]);

    // 0xEE is 238, above the threshold: a stray write into the padding would
    // turn it into 255, so "still 0xEE" is a real assertion.
    var wide = [_]u8{0xEE} ** 15;
    try threshold(&widesrc, &wide, 3, 3, 100, 255, 5);
    try expectBytes(&corpus, "threshold_stride", &wide);

    for (0..3) |y| {
        try std.testing.expectEqual(@as(u8, 0xEE), wide[y * 5 + 3]);
        try std.testing.expectEqual(@as(u8, 0xEE), wide[y * 5 + 4]);
    }
}

test "binary threshold: an undersized buffer is an error, not a write past the end" {
    // The C version checks neither buffer; this is the check it does not have.
    var small: [8]u8 = undefined;
    try std.testing.expectError(Error.BufferTooSmall, threshold(&img, &small, 4, 3, 100, 255, 0));
    try std.testing.expectError(Error.BufferTooSmall, threshold(&small, &small, 4, 3, 100, 255, 0));
    try std.testing.expectError(Error.InvalidSize, threshold(&img, &small, 0, 3, 100, 255, 0));

    var exact: [12]u8 = undefined;
    try threshold(&img, &exact, 4, 3, 100, 255, 0);
}
