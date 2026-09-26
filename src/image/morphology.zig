//! Binary morphology, ported from `include/breeze/image/morphology.h`.
//!
//! Six functions: the structure element, dilation, erosion, the two composites
//! (open and close) and the gradient. Three shape changes:
//!
//!   * **The kernel size is comptime.** The C takes `kernel_size` at run time and
//!     refuses an even one by returning `void` early; here the size is part of the
//!     kernel array's type, so an even size does not compile and the "did it
//!     build a kernel at all" question disappears. Every function that walks the
//!     element takes it as `*const [size * size]u8`, and the C's separate
//!     `kernel_size` argument - which is only ever the square root of the array's
//!     length - is gone.
//!   * **`shape` is an enum.** The C's `int` falls through to the rectangle for
//!     any value that is not 0, 1 or 2, including negatives; the corpus records
//!     the fallback (`morphology_kernel_unknown_shape_3_is_rect`) because an enum
//!     cannot express it.
//!   * **The two scratch buffers are the caller's.** `Open` and `Close` each
//!     allocate one `stride * height` buffer, the gradient allocates two.
//!
//! One thing here is unlike the rest of the stage, and it is why `common` has a
//! second buffer check: dilation and erosion clear their destination with
//! `memset(dst, 0, stride * height)`, so they write the padding. Everywhere else
//! in this stage the padding is left alone.

const std = @import("std");

const common = @import("common.zig");
const corpus_mod = @import("../math/corpus.zig");

pub const Error = common.Error;

/// The C's `shape` argument: 0 rectangle, 1 cross, 2 circle, anything else
/// rectangle. The fourth case is not representable here.
pub const Shape = enum {
    rect,
    cross,
    circle,
};

/// Fill a `size` x `size` structure element. `size` must be odd.
pub fn createKernel(comptime size: usize, kernel: *[size * size]u8, shape: Shape) void {
    comptime if (size == 0 or size % 2 == 0)
        @compileError("a structure element must have an odd, positive size");

    const half = size / 2;
    @memset(kernel, 0);

    switch (shape) {
        .rect => @memset(kernel, 1),
        .cross => {
            for (0..size) |i| {
                kernel[i * size + half] = 1;
                kernel[half * size + i] = 1;
            }
        },
        .circle => {
            for (0..size) |row| {
                for (0..size) |col| {
                    const dx: f32 = @floatFromInt(@as(isize, @intCast(col)) - @as(isize, @intCast(half)));
                    const dy: f32 = @floatFromInt(@as(isize, @intCast(row)) - @as(isize, @intCast(half)));
                    if (@sqrt(dx * dx + dy * dy) <= @as(f32, @floatFromInt(half))) {
                        kernel[row * size + col] = 1;
                    }
                }
            }
        },
    }
}

/// The byte a set pixel gets. The C uses 255 in three functions and never reads
/// the source's value, so a source of `1` and a source of `255` behave alike.
const on: u8 = 255;

/// Dilate: a set source pixel sets every destination pixel its element reaches.
///
/// The C skips source pixels that are zero, and that skip is load-bearing rather
/// than an optimisation: without it a zero pixel would still set its neighbours,
/// which is not dilation. A probe that removes it turns the corpus red.
pub fn dilate(
    comptime size: usize,
    src: []const u8,
    dst: []u8,
    kernel: *const [size * size]u8,
    width: usize,
    height: usize,
    stride_bytes: usize,
) Error!void {
    const stride = common.strideOf(width, stride_bytes);
    try common.checkRegion(src.len, width, height, stride);
    try common.checkFullBuffer(dst.len, width, height, stride);

    // The whole buffer, padding included - see the module comment.
    @memset(dst[0 .. stride * height], 0);

    const half = size / 2;
    for (0..height) |y| {
        for (0..width) |x| {
            if (src[y * stride + x] == 0) continue;

            for (0..size) |kj| {
                for (0..size) |ki| {
                    if (kernel[kj * size + ki] == 0) continue;

                    const dy = @as(isize, @intCast(kj)) - @as(isize, @intCast(half));
                    const dx = @as(isize, @intCast(ki)) - @as(isize, @intCast(half));
                    const dst_y = @as(isize, @intCast(y)) + dy;
                    const dst_x = @as(isize, @intCast(x)) + dx;

                    // The C drops anything outside the image rather than clamping;
                    // there is no edge extension here.
                    if (dst_x < 0 or dst_y < 0) continue;
                    if (dst_x >= width or dst_y >= height) continue;

                    dst[@as(usize, @intCast(dst_y)) * stride + @as(usize, @intCast(dst_x))] = on;
                }
            }
        }
    }
}

/// Erode: a destination pixel is set only if the whole element fits inside the
/// source's set pixels - with the outside of the image counting as unset.
pub fn erode(
    comptime size: usize,
    src: []const u8,
    dst: []u8,
    kernel: *const [size * size]u8,
    width: usize,
    height: usize,
    stride_bytes: usize,
) Error!void {
    const stride = common.strideOf(width, stride_bytes);
    try common.checkRegion(src.len, width, height, stride);
    try common.checkFullBuffer(dst.len, width, height, stride);

    @memset(dst[0 .. stride * height], 0);

    const half = size / 2;
    for (0..height) |y| {
        for (0..width) |x| {
            var matched = true;

            // The C's loops carry `&& match`, so they stop at the first tap that
            // fails. The walk order is the same here, which matters only for which
            // tap is examined; the answer is the same either way.
            outer: for (0..size) |kj| {
                for (0..size) |ki| {
                    if (kernel[kj * size + ki] == 0) continue;

                    const dy = @as(isize, @intCast(kj)) - @as(isize, @intCast(half));
                    const dx = @as(isize, @intCast(ki)) - @as(isize, @intCast(half));
                    const src_y = @as(isize, @intCast(y)) + dy;
                    const src_x = @as(isize, @intCast(x)) + dx;

                    if (src_x < 0 or src_y < 0 or src_x >= width or src_y >= height) {
                        matched = false;
                        break :outer;
                    }
                    if (src[@as(usize, @intCast(src_y)) * stride + @as(usize, @intCast(src_x))] == 0) {
                        matched = false;
                        break :outer;
                    }
                }
            }

            if (matched) dst[y * stride + x] = on;
        }
    }
}

/// Erode, then dilate.
pub fn open(
    comptime size: usize,
    src: []const u8,
    dst: []u8,
    temp: []u8,
    kernel: *const [size * size]u8,
    width: usize,
    height: usize,
    stride_bytes: usize,
) Error!void {
    try erode(size, src, temp, kernel, width, height, stride_bytes);
    try dilate(size, temp, dst, kernel, width, height, stride_bytes);
}

/// Dilate, then erode.
pub fn close(
    comptime size: usize,
    src: []const u8,
    dst: []u8,
    temp: []u8,
    kernel: *const [size * size]u8,
    width: usize,
    height: usize,
    stride_bytes: usize,
) Error!void {
    try dilate(size, src, temp, kernel, width, height, stride_bytes);
    try erode(size, temp, dst, kernel, width, height, stride_bytes);
}

/// Dilated minus eroded, per pixel.
///
/// The C clamps the difference to 0..255. The upper clamp cannot fire: both
/// operands are 0 or 255, so the difference is -255, 0 or 255. It is dropped as
/// provably dead (docs/REVIEW.md §45); the lower one is real, and it is what turns
/// "inside the set but not on its edge" into 0.
pub fn gradient(
    comptime size: usize,
    src: []const u8,
    dst: []u8,
    dilated: []u8,
    eroded: []u8,
    kernel: *const [size * size]u8,
    width: usize,
    height: usize,
    stride_bytes: usize,
) Error!void {
    const stride = common.strideOf(width, stride_bytes);
    try common.checkRegion(src.len, width, height, stride);
    try common.checkFullBuffer(dst.len, width, height, stride);

    try dilate(size, src, dilated, kernel, width, height, stride_bytes);
    try erode(size, src, eroded, kernel, width, height, stride_bytes);

    for (0..height) |y| {
        for (0..width) |x| {
            const idx = y * stride + x;
            dst[idx] = dilated[idx] -| eroded[idx];
        }
    }
}

// --- tests ------------------------------------------------------------------

/// A solid 3x3 block in the middle of a 5x5 frame: the smallest arrangement where
/// erosion keeps anything at all, and where dilation grows it to the frame.
const block = [25]u8{
    0, 0,   0,   0,   0,
    0, 255, 255, 255, 0,
    0, 255, 255, 255, 0,
    0, 255, 255, 255, 0,
    0, 0,   0,   0,   0,
};

fn expectBytes(corpus: *const corpus_mod.Corpus, name: []const u8, bytes: []const u8) !void {
    var as_floats: [64]f32 = undefined;
    for (bytes, 0..) |b, i| as_floats[i] = @floatFromInt(b);
    try corpus.expectValues(name, as_floats[0..bytes.len]);
}

test "morphology kernel: the three shapes match the C" {
    const corpus = try corpus_mod.Corpus.load();

    var k3: [9]u8 = undefined;
    var k5: [25]u8 = undefined;

    createKernel(3, &k3, .rect);
    try expectBytes(&corpus, "morphology_kernel_rect_3", &k3);
    createKernel(5, &k5, .rect);
    try expectBytes(&corpus, "morphology_kernel_rect_5", &k5);

    createKernel(3, &k3, .cross);
    try expectBytes(&corpus, "morphology_kernel_cross_3", &k3);
    createKernel(5, &k5, .cross);
    try expectBytes(&corpus, "morphology_kernel_cross_5", &k5);

    createKernel(3, &k3, .circle);
    try expectBytes(&corpus, "morphology_kernel_circle_3", &k3);
    createKernel(5, &k5, .circle);
    try expectBytes(&corpus, "morphology_kernel_circle_5", &k5);

    // The C's default branch is the rectangle, which the corpus records and an
    // enum cannot ask for.
    var fallback: [9]u8 = undefined;
    createKernel(3, &fallback, .rect);
    try expectBytes(&corpus, "morphology_kernel_unknown_shape_3_is_rect", &fallback);

    // The refusals: the C returns before writing, so the caller's buffer keeps its
    // 0xEE. Both are compile errors here, so there is nothing to compare against
    // except the fixture's record of that.
    const untouched = [_]f32{238} ** 16;
    try corpus.expectValues("morphology_kernel_even_size_untouched", &untouched);
    try corpus.expectValues("morphology_kernel_zero_size_untouched", &untouched);

    // Size 3 is where the cross and the circle coincide: every cell within
    // distance 1 of the centre is exactly the plus.
    var cross3: [9]u8 = undefined;
    var circle3: [9]u8 = undefined;
    createKernel(3, &cross3, .cross);
    createKernel(3, &circle3, .circle);
    try std.testing.expectEqualSlices(u8, &cross3, &circle3);
}

test "morphology: dilation and erosion match the C" {
    const corpus = try corpus_mod.Corpus.load();

    var k3: [9]u8 = undefined;
    createKernel(3, &k3, .rect);

    var out = [_]u8{0xEE} ** 25;
    try dilate(3, &block, &out, &k3, 5, 5, 0);
    try expectBytes(&corpus, "morphology_dilate_rect_3", &out);

    out = [_]u8{0xEE} ** 25;
    try erode(3, &block, &out, &k3, 5, 5, 0);
    try expectBytes(&corpus, "morphology_erode_rect_3", &out);

    // Erosion needs the whole element inside the set, so on a 3x3 block only its
    // centre survives - and the border of the image is never inside anything,
    // because the outside counts as unset rather than being extended.
    for (out, 0..) |byte, i| {
        try std.testing.expectEqual(@as(u8, if (i == 12) 255 else 0), byte);
    }

    createKernel(3, &k3, .cross);
    out = [_]u8{0xEE} ** 25;
    try dilate(3, &block, &out, &k3, 5, 5, 0);
    try expectBytes(&corpus, "morphology_dilate_cross_3", &out);

    out = [_]u8{0xEE} ** 25;
    try erode(3, &block, &out, &k3, 5, 5, 0);
    try expectBytes(&corpus, "morphology_erode_cross_3", &out);

    var k5: [25]u8 = undefined;
    createKernel(5, &k5, .circle);
    out = [_]u8{0xEE} ** 25;
    try dilate(5, &block, &out, &k5, 5, 5, 0);
    try expectBytes(&corpus, "morphology_dilate_circle_5", &out);
}

test "morphology: open, close and the gradient match the C" {
    const corpus = try corpus_mod.Corpus.load();

    var k3: [9]u8 = undefined;
    createKernel(3, &k3, .rect);

    var temp = [_]u8{0xEE} ** 25;
    var out = [_]u8{0xEE} ** 25;
    try open(3, &block, &out, &temp, &k3, 5, 5, 0);
    try expectBytes(&corpus, "morphology_open_rect_3", &out);

    out = [_]u8{0xEE} ** 25;
    try close(3, &block, &out, &temp, &k3, 5, 5, 0);
    try expectBytes(&corpus, "morphology_close_rect_3", &out);

    // The solid block answers the same both ways, which is why it cannot tell the
    // two composites apart. A 2x2 block can: erosion has no full 3x3 to sit in, so
    // opening erases it while closing grows it and erodes back to a 2x2.
    const thin = [25]u8{
        0, 0,   0,   0, 0,
        0, 255, 255, 0, 0,
        0, 255, 255, 0, 0,
        0, 0,   0,   0, 0,
        0, 0,   0,   0, 0,
    };
    var opened = [_]u8{0xEE} ** 25;
    var closed = [_]u8{0xEE} ** 25;
    try open(3, &thin, &opened, &temp, &k3, 5, 5, 0);
    try expectBytes(&corpus, "morphology_open_thin_rect_3", &opened);
    try close(3, &thin, &closed, &temp, &k3, 5, 5, 0);
    try expectBytes(&corpus, "morphology_close_thin_rect_3", &closed);
    try std.testing.expect(!std.mem.eql(u8, &opened, &closed));

    var dilated = [_]u8{0} ** 25;
    var eroded = [_]u8{0} ** 25;
    out = [_]u8{0xEE} ** 25;
    try gradient(3, &block, &out, &dilated, &eroded, &k3, 5, 5, 0);
    try expectBytes(&corpus, "morphology_gradient_rect_3", &out);

    // Independently of the fixture: the gradient is 255 exactly where dilation
    // and erosion disagree.
    for (out, 0..) |byte, i| {
        const expected: u8 = if (dilated[i] != eroded[i]) 255 else 0;
        try std.testing.expectEqual(expected, byte);
    }
}

test "morphology: a stride wider than the image does write its padding" {
    const corpus = try corpus_mod.Corpus.load();

    // 5 wide at stride 6, so each row has one trailing byte. The C clears
    // `stride * height`, so that byte comes back 0 rather than the 0xEE the buffer
    // held - the opposite of every other function in this stage.
    var widesrc = [_]u8{0xAA} ** 30;
    for (0..5) |y| @memcpy(widesrc[y * 6 ..][0..5], block[y * 5 ..][0..5]);

    var k3: [9]u8 = undefined;
    createKernel(3, &k3, .rect);

    var wideout = [_]u8{0xEE} ** 30;
    try dilate(3, &widesrc, &wideout, &k3, 5, 5, 6);
    try expectBytes(&corpus, "morphology_dilate_stride", &wideout);
    for (0..5) |y| try std.testing.expectEqual(@as(u8, 0), wideout[y * 6 + 5]);

    wideout = [_]u8{0xEE} ** 30;
    try erode(3, &widesrc, &wideout, &k3, 5, 5, 6);
    try expectBytes(&corpus, "morphology_erode_stride", &wideout);
    for (0..5) |y| try std.testing.expectEqual(@as(u8, 0), wideout[y * 6 + 5]);
}

test "morphology: the buffers are checked, and the padding byte is required" {
    var k3: [9]u8 = undefined;
    createKernel(3, &k3, .rect);

    // 4 wide at stride 5, 3 rows: the region is 14 bytes but the clear writes 15,
    // so a 14-byte destination must be refused. This is the difference between
    // checkRegion and checkFullBuffer, and no other module in the stage has it.
    var fourteen: [14]u8 = undefined;
    try std.testing.expectError(Error.BufferTooSmall, dilate(3, &block, &fourteen, &k3, 4, 3, 5));
    try std.testing.expectError(Error.BufferTooSmall, erode(3, &block, &fourteen, &k3, 4, 3, 5));

    var fifteen: [15]u8 = undefined;
    try dilate(3, &[_]u8{0} ** 15, &fifteen, &k3, 4, 3, 5);

    // The source only needs its region: 13 bytes is one short of the 14 the region
    // spans, while the destination beside it passes the stricter rule.
    var thirteen: [13]u8 = undefined;
    try std.testing.expectError(Error.BufferTooSmall, dilate(3, &thirteen, &fifteen, &k3, 4, 3, 5));
    try std.testing.expectError(Error.InvalidSize, dilate(3, &block, &fifteen, &k3, 0, 3, 5));

    // Open, close and the gradient check their scratch too.
    var out = [_]u8{0} ** 25;
    var small_scratch: [24]u8 = undefined;
    try std.testing.expectError(
        Error.BufferTooSmall,
        open(3, &block, &out, &small_scratch, &k3, 5, 5, 0),
    );
    try std.testing.expectError(
        Error.BufferTooSmall,
        gradient(3, &block, &out, &small_scratch, &out, &k3, 5, 5, 0),
    );
}
