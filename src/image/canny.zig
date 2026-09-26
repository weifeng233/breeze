//! Canny edge detection, ported from `include/breeze/image/canny_edge.h`.
//!
//! Four stages, each its own function, plus the pipeline that chains them. The C
//! allocates six buffers across the four (`blurred`, the blur's own temporary,
//! the Gaussian kernel, `magnitude`, `direction`, `nms`); all six are the
//! caller's here.
//!
//! Three things about the C are worth knowing before reading the code. Two were
//! defects and are fixed here (docs/REVIEW.md §46 records the C's behaviour, §56
//! the fixes and their evidence); the third is a behaviour the port keeps:
//!
//!   * **Only the source and the blurred image were strided.** `BreezeCannyGradient`
//!     reads with `stride` but writes `magnitude` and `direction` at `y * width + x`;
//!     suppression takes no stride at all; and the pipeline wrote its `dst` packed
//!     even though it accepts a `stride_bytes`, so a caller passing a strided `dst`
//!     got it written as if it were packed. `magnitude`, `direction` and `nms` are
//!     still `width * height` by design - they are the C's `malloc`'d intermediates -
//!     but `dst` is the caller's image and now honours the stride.
//!   * **The hysteresis was a single pass and said so.** Its comment noted that a
//!     weak pixel promoted to strong is not re-examined, so propagation depended on
//!     the scan order and a chain running against it stopped at its first weak
//!     pixel. It now propagates to a fixed point, so the two directions agree.
//!   * **The gradient leaves its border untouched.** The 3x3 window only visits
//!     the interior, and unlike the threshold functions there is no clearing pass
//!     first, so `magnitude` and `direction` keep whatever the caller left at
//!     rows 0 and `height - 1` and at columns 0 and `width - 1`. Suppression is
//!     what zeroes its own border. That one is kept: it is definable, and the
//!     `canny_gradient_border_*` cases pin it.

const std = @import("std");

const common = @import("common.zig");
const gaussian = @import("gaussian.zig");
const corpus_mod = @import("../math/corpus.zig");

pub const Error = gaussian.Error;

/// The C's `3.14159265f`, kept as written: the angle is compared against
/// quantities like 22.5, so a more accurate π is not obviously an improvement.
const pi: f32 = 3.14159265;

const Gradient = struct { gx: i32, gy: i32 };

inline fn px(img: []const u8, index: usize) i32 {
    return @intCast(img[index]);
}

inline fn gradientAt(img: []const u8, stride: usize, x: usize, y: usize) Gradient {
    const p00 = (y - 1) * stride + (x - 1);
    const p01 = (y - 1) * stride + x;
    const p02 = (y - 1) * stride + x + 1;
    const p10 = y * stride + (x - 1);
    const p12 = y * stride + x + 1;
    const p20 = (y + 1) * stride + (x - 1);
    const p21 = (y + 1) * stride + x;
    const p22 = (y + 1) * stride + x + 1;

    return .{
        .gx = -px(img, p00) - 2 * px(img, p10) - px(img, p20) +
            px(img, p02) + 2 * px(img, p12) + px(img, p22),
        .gy = -px(img, p00) - 2 * px(img, p01) - px(img, p02) +
            px(img, p20) + 2 * px(img, p21) + px(img, p22),
    };
}

/// The C's four-way quantisation of the gradient angle.
///
/// After the `if (angle < 0) angle += 180` adjustment the angle is in [0, 180],
/// which is what makes the C's lower bounds (`angle >= 0`, `angle >= 22.5`, ...)
/// redundant and its `angle <= 180` on the last branch redundant with it.
inline fn quantiseDirection(gx: f32, gy: f32) u8 {
    var angle = std.math.atan2(gy, gx) * 180.0 / pi;
    if (angle < 0) angle += 180.0;

    if (angle < 22.5 or angle >= 157.5) return 0;
    if (angle < 67.5) return 1;
    if (angle < 112.5) return 2;
    return 3;
}

/// Gradient magnitude and quantised direction.
///
/// `magnitude` and `direction` are `width * height` and **their border is left
/// alone** - see the module comment.
pub fn gradient(
    src: []const u8,
    magnitude: []f32,
    direction: []u8,
    width: usize,
    height: usize,
    stride_bytes: usize,
) Error!void {
    const stride = common.strideOf(width, stride_bytes);
    try common.checkRegion(src.len, width, height, stride);
    try common.checkFullBuffer(magnitude.len, width, height, width);
    try common.checkFullBuffer(direction.len, width, height, width);

    if (width < 3 or height < 3) return;

    for (1..height - 1) |y| {
        for (1..width - 1) |x| {
            const g = gradientAt(src, stride, x, y);
            const gx: f32 = @floatFromInt(g.gx);
            const gy: f32 = @floatFromInt(g.gy);

            const idx = y * width + x;
            magnitude[idx] = @sqrt(gx * gx + gy * gy);
            direction[idx] = quantiseDirection(gx, gy);
        }
    }
}

/// Non-maximum suppression: keep a pixel only if it is at least as large as its
/// two neighbours along the gradient direction.
///
/// The comparison is `>=`, so a plateau keeps both of its inner pixels; a `>`
/// would drop both. The corpus case `canny_nms_plateau` is built for exactly
/// that, with two equal neighbours.
pub fn nonMaxSuppression(
    magnitude: []const f32,
    direction: []const u8,
    result: []f32,
    width: usize,
    height: usize,
) Error!void {
    try common.checkFullBuffer(magnitude.len, width, height, width);
    try common.checkFullBuffer(direction.len, width, height, width);
    try common.checkFullBuffer(result.len, width, height, width);

    // The C clears the result first, which is what leaves its border at 0.
    for (result[0 .. width * height]) |*value| value.* = 0;

    if (width < 3 or height < 3) return;

    for (1..height - 1) |y| {
        for (1..width - 1) |x| {
            const idx = y * width + x;
            const mag = magnitude[idx];

            // The C's switch has no default, so a direction byte outside 0..3 -
            // which it never writes itself, but a caller's own buffer can hold -
            // leaves both neighbours at 0 and keeps the pixel.
            var mag1: f32 = 0;
            var mag2: f32 = 0;
            switch (direction[idx]) {
                0 => {
                    mag1 = magnitude[idx - 1];
                    mag2 = magnitude[idx + 1];
                },
                1 => {
                    mag1 = magnitude[(y - 1) * width + (x + 1)];
                    mag2 = magnitude[(y + 1) * width + (x - 1)];
                },
                2 => {
                    mag1 = magnitude[(y - 1) * width + x];
                    mag2 = magnitude[(y + 1) * width + x];
                },
                3 => {
                    mag1 = magnitude[(y - 1) * width + (x - 1)];
                    mag2 = magnitude[(y + 1) * width + (x + 1)];
                },
                else => {},
            }

            if (mag >= mag1 and mag >= mag2) result[idx] = mag;
        }
    }
}

/// Double thresholding and hysteresis.
///
/// `nms` and `scratch` are packed, `width * height` each: they are the C's
/// `malloc`'d intermediates. `edges` is the caller's image and is written at
/// `stride_bytes`. That last part is the fix for the C's
/// `BreezeCannyHysteresis`, which took no stride and wrote `dst` packed - a
/// strided destination got its padding filled with edge values, and a caller
/// laying the image out in a wider buffer got it scrambled
/// (docs/REVIEW.md §46, §56).
///
/// `scratch` is the C's `calloc(width * height)`, and it is zeroed here so that
/// the caller's buffer contents do not matter - the C gets zeros from `calloc`
/// whatever was in the heap before.
pub fn hysteresis(
    nms: []const f32,
    edges: []u8,
    scratch: []u8,
    width: usize,
    height: usize,
    low_threshold: f32,
    high_threshold: f32,
    stride_bytes: usize,
) Error!void {
    const stride = common.strideOf(width, stride_bytes);
    try common.checkFullBuffer(nms.len, width, height, width);
    try common.checkRegion(edges.len, width, height, stride);
    try common.checkFullBuffer(scratch.len, width, height, width);

    const strong_edges = scratch[0 .. width * height];
    @memset(strong_edges, 0);

    for (0..height) |y| {
        for (0..width) |x| {
            const cell = y * width + x;
            const mag = nms[cell];
            const idx = y * stride + x;

            if (mag >= high_threshold) {
                edges[idx] = 255;
                strong_edges[cell] = 1;
            } else if (mag >= low_threshold) {
                edges[idx] = 128;
            } else {
                edges[idx] = 0;
            }
        }
    }

    // Propagate to a fixed point instead of in one pass in scan order.
    //
    // The C's comment admits its single pass "depends on the next iteration": a
    // strong pixel could only promote weak neighbours it had not already walked
    // past, so a chain pointing one way came out whole while the same chain
    // pointing the other way stopped at its first weak pixel - the two cases
    // below used to disagree, and the test asserted the disagreement
    // (docs/REVIEW.md §46).
    //
    // This pass is monotone - it only ever turns a 128 into a 255 - so repeating
    // it until a pass changes nothing reaches the *least* fixed point, which is
    // independent of scan order: the answer is the closure of the strong set
    // under the weak-neighbour relation, which is what hysteresis means.
    //
    // The cost is passes, not a queue, and it is the chain length plus one: the
    // last pass is the one that changes nothing and ends the loop. Measured on the
    // fixtures below - 1 pass when nothing propagates, 2 for the two-pixel chain,
    // 5 for the four-pixel chain that runs against the scan order - where the C's
    // single pass cost one and got the last one wrong. That is the trade for
    // order-independence, and it needs no buffer the caller has to supply.
    //
    // Still the interior only, as in the C: a border pixel can be promoted but
    // does not itself promote, which is why this cannot become a single sweep.
    // `width < 3` is the C's empty loop; without the check `1..width - 1` is a
    // panic in Zig rather than a no-op (the C's `1 < width - 1` is simply false).
    if (width >= 3 and height >= 3) {
        var changed = true;
        while (changed) {
            changed = false;
            for (1..height - 1) |y| {
                for (1..width - 1) |x| {
                    const cell = y * width + x;
                    if (strong_edges[cell] == 0) continue;

                    for (0..3) |kj| {
                        for (0..3) |ki| {
                            if (kj == 1 and ki == 1) continue;

                            const neighbour_y = y + kj - 1;
                            const neighbour_x = x + ki - 1;
                            if (neighbour_x >= width or neighbour_y >= height) continue;

                            const neighbour_cell = neighbour_y * width + neighbour_x;
                            const neighbour = neighbour_y * stride + neighbour_x;
                            if (edges[neighbour] == 128) {
                                edges[neighbour] = 255;
                                strong_edges[neighbour_cell] = 1;
                                changed = true;
                            }
                        }
                    }
                }
            }
        }
    }

    for (0..height) |y| {
        for (0..width) |x| {
            const idx = y * stride + x;
            if (edges[idx] == 128) edges[idx] = 0;
        }
    }
}

/// The whole pipeline: blur, gradient, suppression, hysteresis.
///
/// Five caller-provided buffers for the C's six. The blurred image and the blur's
/// own temporary are `stride * height` bytes, the kernel is
/// `autoKernelSize(sigma)` floats, and `magnitude`, `direction` and `nms_result`
/// are `width * height`. The sixth is the C's `strong_edges` inside the
/// hysteresis: that one is not a parameter here because `blurred` is reused for
/// it - the gradient has already consumed it, and the hysteresis zeroes it before
/// reading anything.
///
/// `dst` is written at `stride_bytes` too - it is the caller's image, and the C's
/// `BreezeCannyHysteresis` writing it packed was a bug (see the module comment).
pub fn edgeDetection(
    src: []const u8,
    dst: []u8,
    blurred: []u8,
    blur_temp: []u8,
    kernel_scratch: []f32,
    magnitude: []f32,
    direction: []u8,
    nms_result: []f32,
    width: usize,
    height: usize,
    sigma: f32,
    low_threshold: f32,
    high_threshold: f32,
    stride_bytes: usize,
) Error!void {
    const stride = common.strideOf(width, stride_bytes);
    try common.checkRegion(src.len, width, height, stride);
    try common.checkFullBuffer(blurred.len, width, height, stride);
    try common.checkFullBuffer(magnitude.len, width, height, width);
    try common.checkFullBuffer(direction.len, width, height, width);
    try common.checkFullBuffer(nms_result.len, width, height, width);
    try common.checkRegion(dst.len, width, height, stride);

    // The C mallocs `magnitude` and `direction`, the gradient below writes only
    // their interior, and suppression reads border pixels from them whenever a
    // diagonal direction at an interior pixel points at one - so the C's pipeline
    // is not a function of its input. probe_canny_uninitialized.c (archived) gets 20 or 24
    // set pixels from one image depending on what was in the heap, at exactly the
    // four pixels whose suppression compares against a border value.
    //
    // Defining the border here is the one place the port deliberately does not
    // reproduce the C. `gradient` keeps the C's behaviour of leaving it alone,
    // which canny_gradient_border_* pins.
    for (magnitude[0 .. width * height]) |*value| value.* = 0;
    @memset(direction[0 .. width * height], 0);

    // The C's first step is the blur with `kernel_size` 0, i.e. the size rule.
    try gaussian.blurSized(
        src,
        blurred,
        blur_temp,
        kernel_scratch,
        width,
        height,
        sigma,
        0,
        stride_bytes,
    );

    try gradient(blurred, magnitude, direction, width, height, stride_bytes);
    try nonMaxSuppression(magnitude, direction, nms_result, width, height);
    try hysteresis(nms_result, dst, blurred, width, height, low_threshold, high_threshold, stride_bytes);
}

// --- tests ------------------------------------------------------------------

/// A bright 2x2 square in a 5x5 frame, the image the pipeline cases use.
const square = [25]u8{
    0, 0, 0,   0,   0,
    0, 0, 0,   0,   0,
    0, 0, 255, 255, 0,
    0, 0, 255, 255, 0,
    0, 0, 0,   0,   0,
};

fn expectBytes(corpus: *const corpus_mod.Corpus, name: []const u8, bytes: []const u8) !void {
    var as_floats: [64]f32 = undefined;
    for (bytes, 0..) |b, i| as_floats[i] = @floatFromInt(b);
    try corpus.expectValues(name, as_floats[0..bytes.len]);
}

test "canny gradient: the direction quantiser matches the C on both sides of 22.5" {
    const corpus = try corpus_mod.Corpus.load();

    // Each 3x3 has one interior pixel, so the answer is a single gradient. The
    // pairs differ in one pixel and land either side of a quantiser threshold -
    // all four thresholds, because a probe that moved 67.5 to 70 stayed green
    // until the pair for it existed.
    const cases = [_]struct { name: []const u8, px: [9]u8 }{
        .{ .name = "canny_dir_east", .px = .{ 0, 0, 100, 0, 0, 100, 0, 0, 100 } },
        .{ .name = "canny_dir_45", .px = .{ 0, 0, 0, 0, 0, 0, 0, 0, 100 } },
        .{ .name = "canny_dir_south", .px = .{ 0, 0, 0, 0, 0, 0, 100, 100, 100 } },
        .{ .name = "canny_dir_135", .px = .{ 0, 0, 0, 0, 0, 0, 100, 0, 0 } },
        .{ .name = "canny_dir_22_below", .px = .{ 0, 0, 29, 0, 0, 0, 0, 0, 69 } },
        .{ .name = "canny_dir_22_above", .px = .{ 0, 0, 28, 0, 0, 0, 0, 0, 70 } },
        .{ .name = "canny_dir_67_below", .px = .{ 0, 0, 0, 0, 0, 0, 41, 0, 100 } },
        .{ .name = "canny_dir_67_above", .px = .{ 0, 0, 0, 0, 0, 0, 42, 0, 100 } },
        .{ .name = "canny_dir_112_below", .px = .{ 0, 0, 0, 0, 0, 0, 100, 71, 0 } },
        .{ .name = "canny_dir_112_above", .px = .{ 0, 0, 0, 0, 0, 0, 100, 70, 0 } },
        .{ .name = "canny_dir_157_below", .px = .{ 0, 0, 0, 70, 0, 0, 100, 0, 0 } },
        .{ .name = "canny_dir_157_above", .px = .{ 0, 0, 0, 71, 0, 0, 100, 0, 0 } },
    };

    for (cases) |case| {
        var magnitude = [_]f32{0} ** 9;
        var direction = [_]u8{0} ** 9;
        try gradient(&case.px, &magnitude, &direction, 3, 3, 0);
        try expectBytes(&corpus, case.name, &direction);
    }

    // The magnitude of two of them, which the direction alone cannot pin.
    var magnitude = [_]f32{0} ** 9;
    var direction = [_]u8{0} ** 9;
    try gradient(&cases[0].px, &magnitude, &direction, 3, 3, 0);
    try corpus.expectValues("canny_dir_east_magnitude", &magnitude);

    var south_magnitude = [_]f32{0} ** 9;
    var south_direction = [_]u8{0} ** 9;
    try gradient(&cases[2].px, &south_magnitude, &south_direction, 3, 3, 0);
    try corpus.expectValues("canny_dir_south_magnitude", &south_magnitude);

    // Independent of the fixture: a pure x gradient is direction 0, a pure y
    // gradient is direction 2, and the two have the same magnitude.
    try std.testing.expectEqual(@as(u8, 0), direction[4]);
    try std.testing.expectEqual(@as(f32, 400.0), magnitude[4]);
    try std.testing.expectEqual(@as(u8, 2), south_direction[4]);
    try std.testing.expectEqualSlices(f32, &magnitude, &south_magnitude);
}

test "canny gradient: the border is left untouched and the outputs are packed" {
    const corpus = try corpus_mod.Corpus.load();

    // 7.0 and 9 stand in for whatever a caller's buffers held.
    var magnitude = [_]f32{7.0} ** 25;
    var direction = [_]u8{9} ** 25;
    try gradient(&square, &magnitude, &direction, 5, 5, 0);
    try corpus.expectValues("canny_gradient_border_magnitude", &magnitude);
    try expectBytes(&corpus, "canny_gradient_border_direction", &direction);

    for (0..25) |i| {
        const on_border = i < 5 or i >= 20 or i % 5 == 0 or i % 5 == 4;
        if (on_border) {
            try std.testing.expectEqual(@as(f32, 7.0), magnitude[i]);
            try std.testing.expectEqual(@as(u8, 9), direction[i]);
        } else {
            try std.testing.expect(magnitude[i] > 0);
        }
    }

    // A strided source, packed outputs: the same magnitudes as the packed call.
    var widesrc = [_]u8{0xAA} ** 30;
    for (0..5) |y| @memcpy(widesrc[y * 6 ..][0..5], square[y * 5 ..][0..5]);

    var wide_magnitude = [_]f32{0} ** 25;
    var wide_direction = [_]u8{0} ** 25;
    try gradient(&widesrc, &wide_magnitude, &wide_direction, 5, 5, 6);
    try corpus.expectValues("canny_gradient_stride_magnitude", &wide_magnitude);

    var packed_magnitude = [_]f32{0} ** 25;
    var packed_direction = [_]u8{0} ** 25;
    try gradient(&square, &packed_magnitude, &packed_direction, 5, 5, 0);
    try std.testing.expectEqualSlices(f32, &packed_magnitude, &wide_magnitude);
}

test "canny suppression: plateaus keep both pixels" {
    const corpus = try corpus_mod.Corpus.load();

    var magnitude = [_]f32{0} ** 25;
    var direction = [_]u8{0} ** 25;
    var nms = [_]f32{0xEE} ** 25;
    try gradient(&square, &magnitude, &direction, 5, 5, 0);
    try nonMaxSuppression(&magnitude, &direction, &nms, 5, 5);
    try corpus.expectValues("canny_nms", &nms);

    // A flat vertical edge, so neighbouring magnitudes along the gradient are
    // equal: both survive, which is what `>=` does and `>` would not.
    const flat = [15]u8{
        0,   0,   0,
        0,   0,   0,
        100, 100, 100,
        100, 100, 100,
        100, 100, 100,
    };
    var fmag = [_]f32{0} ** 15;
    var fdir = [_]u8{0} ** 15;
    try gradient(&flat, &fmag, &fdir, 3, 5, 0);
    try expectBytes(&corpus, "canny_plateau_direction", &fdir);

    var fnms = [_]f32{0xEE} ** 15;
    try nonMaxSuppression(&fmag, &fdir, &fnms, 3, 5);
    try corpus.expectValues("canny_nms_plateau", &fnms);
    try std.testing.expect(fnms[4] > 0);
    try std.testing.expectEqual(fnms[4], fnms[7]);
}

test "canny hysteresis: thresholds, and a chain that propagates both ways" {
    const corpus = try corpus_mod.Corpus.load();

    var nms = [_]f32{10.0} ** 25;
    var edges = [_]u8{0xEE} ** 25;
    var scratch: [25]u8 = undefined;

    nms[12] = 100.0;
    try hysteresis(&nms, &edges, &scratch, 5, 5, 50.0, 100.0, 0);
    try expectBytes(&corpus, "canny_hysteresis_thresholds", &edges);
    try std.testing.expectEqual(@as(u8, 255), edges[12]);
    try std.testing.expectEqual(@as(u8, 0), edges[11]);

    // A chain of weak pixels hanging off a strong one is the closure of the strong
    // set, whichever way the chain points. The C's single pass propagated with the
    // scan order, so the same three pixels reversed stopped at the first weak one -
    // §46 recorded both directions and asserted that they disagreed. They are the
    // same pattern with the strong pixel at the other end, so they must agree now.
    var forward = [_]f32{0} ** 25;
    forward[11] = 100.0;
    forward[12] = 50.0;
    forward[13] = 50.0;
    var forward_edges = [_]u8{0xEE} ** 25;
    try hysteresis(&forward, &forward_edges, &scratch, 5, 5, 50.0, 100.0, 0);
    try expectBytes(&corpus, "canny_hysteresis_chain_forward", &forward_edges);

    var backward = [_]f32{0} ** 25;
    backward[13] = 100.0;
    backward[12] = 50.0;
    backward[11] = 50.0;
    var backward_edges = [_]u8{0xEE} ** 25;
    try hysteresis(&backward, &backward_edges, &scratch, 5, 5, 50.0, 100.0, 0);
    try expectBytes(&corpus, "canny_hysteresis_chain_backward", &backward_edges);

    try std.testing.expectEqual(@as(u8, 255), forward_edges[11]);
    try std.testing.expectEqual(@as(u8, 255), backward_edges[11]);
    try std.testing.expectEqualSlices(u8, &forward_edges, &backward_edges);

    // The closure is not "keep everything above the low threshold": a weak pixel
    // with no strong pixel next to it is still cleared.
    var island = [_]f32{0} ** 25;
    island[6] = 100.0;
    island[18] = 50.0;
    var island_edges = [_]u8{0xEE} ** 25;
    try hysteresis(&island, &island_edges, &scratch, 5, 5, 50.0, 100.0, 0);
    try std.testing.expectEqual(@as(u8, 255), island_edges[6]);
    try std.testing.expectEqual(@as(u8, 0), island_edges[18]);

    // A chain running *against* the scan order pins the fixed point rather than
    // "one extra pass". One pass promotes nothing here at all: the loop walks
    // (1,1) first and reaches the strong pixel at (5,5) last, so the only pixel it
    // can promote is (4,4), which it has already passed. Two passes reach (3,3),
    // and the closure needs as many passes as the chain is long.
    var long_chain = [_]f32{0} ** 49;
    for (1..6) |k| long_chain[k * 7 + k] = 50.0;
    long_chain[5 * 7 + 5] = 100.0;
    var long_edges = [_]u8{0xEE} ** 49;
    var long_scratch = [_]u8{0} ** 49;
    try hysteresis(&long_chain, &long_edges, &long_scratch, 7, 7, 50.0, 100.0, 0);
    for (1..6) |k| try std.testing.expectEqual(@as(u8, 255), long_edges[k * 7 + k]);

    // Smaller than 3x3 is no interior at all, which the C gets as an empty loop.
    // Without the guard `1..width - 1` is an underflow panic in Zig, so this is a
    // crash becoming a no-op rather than a behaviour change.
    var one_nms = [_]f32{100.0} ** 1;
    var one_edges = [_]u8{0xEE} ** 1;
    var one_scratch = [_]u8{0} ** 1;
    try hysteresis(&one_nms, &one_edges, &one_scratch, 1, 1, 50.0, 100.0, 0);
    try std.testing.expectEqual(@as(u8, 255), one_edges[0]);

    var column_nms = [_]f32{0} ** 5;
    column_nms[2] = 100.0;
    var column_edges = [_]u8{0xEE} ** 5;
    var column_scratch = [_]u8{0} ** 5;
    try hysteresis(&column_nms, &column_edges, &column_scratch, 1, 5, 50.0, 100.0, 0);
    try std.testing.expectEqual(@as(u8, 255), column_edges[2]);
    try std.testing.expectEqual(@as(u8, 0), column_edges[0]);
}

test "canny: the whole pipeline matches the C when the border is defined" {
    const corpus = try corpus_mod.Corpus.load();

    // The corpus holds a *staged* pipeline rather than the C's own
    // BreezeCannyEdgeDetection, because that one reads the uninitialized border of
    // its malloc'd intermediates - see the case's comment and
    // the archived probe. The port defines the border, so it must agree
    // with the staged run.
    var dst = [_]u8{0xEE} ** 25;
    var blurred = [_]u8{0} ** 25;
    var blur_temp = [_]u8{0} ** 25;
    var kernel: [31]f32 = undefined;
    var magnitude = [_]f32{0} ** 25;
    var direction = [_]u8{0} ** 25;
    var nms = [_]f32{0} ** 25;

    try edgeDetection(
        &square,
        &dst,
        &blurred,
        &blur_temp,
        &kernel,
        &magnitude,
        &direction,
        &nms,
        5,
        5,
        1.0,
        20.0,
        60.0,
        0,
    );
    try expectBytes(&corpus, "canny_staged_edges", &dst);

    dst = [_]u8{0xEE} ** 25;
    try edgeDetection(
        &square,
        &dst,
        &blurred,
        &blur_temp,
        &kernel,
        &magnitude,
        &direction,
        &nms,
        5,
        5,
        0.5,
        20.0,
        60.0,
        0,
    );
    try expectBytes(&corpus, "canny_staged_edges_sigma_0p5", &dst);

    // A caller whose intermediates hold garbage gets the same answer, because the
    // pipeline defines the border itself.
    var dirty_magnitude = [_]f32{1.0e16} ** 25;
    var dirty_direction = [_]u8{9} ** 25;
    var clean = [_]u8{0xEE} ** 25;
    try edgeDetection(
        &square,
        &clean,
        &blurred,
        &blur_temp,
        &kernel,
        &dirty_magnitude,
        &dirty_direction,
        &nms,
        5,
        5,
        1.0,
        20.0,
        60.0,
        0,
    );
    var reference = [_]u8{0xEE} ** 25;
    var clean_magnitude = [_]f32{0} ** 25;
    var clean_direction = [_]u8{0} ** 25;
    try edgeDetection(
        &square,
        &reference,
        &blurred,
        &blur_temp,
        &kernel,
        &clean_magnitude,
        &clean_direction,
        &nms,
        5,
        5,
        1.0,
        20.0,
        60.0,
        0,
    );
    try std.testing.expectEqualSlices(u8, &reference, &clean);
    try expectBytes(&corpus, "canny_staged_edges", &clean);

    // A strided source: the blur, the gradient and the output all use the stride,
    // so the answer is the same edges laid out in a wider image. Reading the
    // strided output back row by row and comparing the result with the corpus's
    // packed case is what pins the stride reaching `dst`; the padding check below
    // is the other half, because the C wrote `dst` packed and so not only scrambled
    // this layout but wrote edge values into the padding.
    var widesrc = [_]u8{0xAA} ** 30;
    for (0..5) |y| @memcpy(widesrc[y * 6 ..][0..5], square[y * 5 ..][0..5]);
    var wide_dst = [_]u8{0xAA} ** 30;
    var wide_blurred = [_]u8{0} ** 30;
    var wide_blur_temp = [_]u8{0} ** 30;
    try edgeDetection(
        &widesrc,
        &wide_dst,
        &wide_blurred,
        &wide_blur_temp,
        &kernel,
        &magnitude,
        &direction,
        &nms,
        5,
        5,
        1.0,
        20.0,
        60.0,
        6,
    );
    var wide_packed = [_]u8{0} ** 25;
    for (0..5) |y| @memcpy(wide_packed[y * 5 ..][0..5], wide_dst[y * 6 ..][0..5]);
    try expectBytes(&corpus, "canny_staged_edges", &wide_packed);
    for (0..5) |y| try std.testing.expectEqual(@as(u8, 0xAA), wide_dst[y * 6 + 5]);

    // With thresholds above every magnitude the answer is a zeroed image rather
    // than the 0xEE the buffer held: `dst` is written everywhere, unlike the
    // gradient's outputs above.
    dst = [_]u8{0xEE} ** 25;
    try edgeDetection(
        &square,
        &dst,
        &blurred,
        &blur_temp,
        &kernel,
        &magnitude,
        &direction,
        &nms,
        5,
        5,
        1.0,
        1.0e9,
        1.0e9,
        0,
    );
    try expectBytes(&corpus, "canny_edges_5x5_all_below", &dst);
    for (dst) |byte| try std.testing.expectEqual(@as(u8, 0), byte);
}

test "canny: every buffer is checked" {
    var magnitude = [_]f32{0} ** 25;
    var direction = [_]u8{0} ** 25;
    var nms = [_]f32{0} ** 25;
    var edges = [_]u8{0} ** 25;

    var small_bytes: [24]u8 = undefined;
    var small_floats: [24]f32 = undefined;

    // The gradient needs packed outputs of width * height, and a strided source.
    try std.testing.expectError(Error.BufferTooSmall, gradient(&square, &small_floats, &direction, 5, 5, 0));
    try std.testing.expectError(Error.BufferTooSmall, gradient(&square, &magnitude, &small_bytes, 5, 5, 0));
    try std.testing.expectError(Error.BufferTooSmall, gradient(&small_bytes, &magnitude, &direction, 5, 5, 0));
    try std.testing.expectError(Error.InvalidSize, gradient(&square, &magnitude, &direction, 0, 5, 0));

    try std.testing.expectError(Error.BufferTooSmall, nonMaxSuppression(&small_floats, &direction, &nms, 5, 5));
    try std.testing.expectError(Error.BufferTooSmall, hysteresis(&nms, &edges, &small_bytes, 5, 5, 1.0, 2.0, 0));

    // `edges` is written at the stride now, so a packed-sized destination is not
    // enough for a strided one: 5x5 at stride 6 touches 29 bytes, and refusing 25
    // is what keeps the last row from being dropped or overrun.
    var scratch = [_]u8{0} ** 25;
    try std.testing.expectError(Error.BufferTooSmall, hysteresis(&nms, &edges, &scratch, 5, 5, 1.0, 2.0, 6));

    // The pipeline carries one stride for every image it touches, `dst` included,
    // so the same 25-byte destination is refused there too - and it is refused at
    // `dst`, since the blurred buffers below are sized for the stride.
    var wide_src = [_]u8{0} ** 30;
    var wide_blurred = [_]u8{0} ** 30;
    var wide_blur_temp = [_]u8{0} ** 30;
    var kernel: [31]f32 = undefined;
    try std.testing.expectError(
        Error.BufferTooSmall,
        edgeDetection(&wide_src, &edges, &wide_blurred, &wide_blur_temp, &kernel, &magnitude, &direction, &nms, 5, 5, 1.0, 1.0, 2.0, 6),
    );

    // The pipeline's kernel scratch is sized by the sigma rule, so a too-small one
    // is refused rather than overrun.
    var tiny_kernel: [2]f32 = undefined;
    var blurred = [_]u8{0} ** 25;
    var blur_temp = [_]u8{0} ** 25;
    try std.testing.expectError(
        Error.BufferTooSmall,
        edgeDetection(&square, &edges, &blurred, &blur_temp, &tiny_kernel, &magnitude, &direction, &nms, 5, 5, 1.0, 1.0, 2.0, 0),
    );
    try std.testing.expectError(
        Error.NonPositiveSigma,
        edgeDetection(&square, &edges, &blurred, &blur_temp, &tiny_kernel, &magnitude, &direction, &nms, 5, 5, 0.0, 1.0, 2.0, 0),
    );
}
