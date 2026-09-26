//! Hough transform, ported from `include/breeze/image/hough_transform.h`.
//!
//! Straight lines and circles, both by voting into an accumulator that the C
//! allocates and this port takes from the caller. The line accumulator is a flat
//! `(2 * diagonal + 1) * 180` ints; the circle one is `width * height *
//! radius_count`, which the C builds as a three-level `int***` table and which is
//! one flat slice here - the index arithmetic replaces the pointer table, and the
//! C's partially-failed-allocation cleanup paths disappear with it, since there
//! is nothing left to fail.
//!
//! Three of the C's behaviours were defects and are fixed here; docs/REVIEW.md
//! §55 records each one with the C's old values, because the corpus used to pin
//! them (`hough_lines_cross`, `hough_lines_vertical_only`,
//! `hough_lines_horizontal_only`, `hough_lines_long_vertical`):
//!
//!   * **theta = 0 and theta = 179 degrees were unreachable.** The C's peak scan
//!     ran `j` from 1 to `theta_count - 2`, so those two columns were never
//!     candidates. A vertical line's votes pile up in the first of them, and what
//!     came back instead were the near-boundary bins collecting the same pixels -
//!     the line, rotated by the width of that band. Every column is scanned now.
//!     With a small output cap the answer can still be that twin, but only because
//!     rho is the outer loop and the twin sits at a smaller rho index - not
//!     because the peak is invisible, which is what the wide-cap assertions in the
//!     tests separate.
//!   * **A reported rho was truncated, not rounded.** The bin index was
//!     `(int)(rho + diagonal)`, so the horizontal-line case reported rho 1 where
//!     the line is at 2. The nearest bin is used now.
//!   * **Both scans stop early on a cap**, and the order they stop in is rho
//!     before theta for lines and x before y before radius for circles - so a cap
//!     keeps whichever maximum comes first in that order, not the strongest. That
//!     one is the caller's contract, not a defect, and is unchanged.
//!
//! The C's two `calloc`s are the reason nothing here reads uninitialized memory,
//! unlike the Canny pipeline (docs/REVIEW.md §46). The port keeps that by zeroing
//! the caller's accumulator.

const std = @import("std");

const common = @import("common.zig");
const corpus_mod = @import("../math/corpus.zig");

pub const Error = common.Error || error{
    /// `max_radius <= min_radius`. The C refuses this by returning 0, which is
    /// indistinguishable from "no circles found".
    InvalidRadiusRange,
};

pub const theta_count = 180;
pub const theta_step: f32 = 3.14159265 / 180.0;
pub const rho_step: f32 = 1.0;

pub const Line = struct {
    rho: f32,
    theta: f32,
    votes: i32,
};

pub const Circle = struct {
    x: usize,
    y: usize,
    radius: usize,
    votes: i32,
};

/// `(int)ceil(sqrt(width^2 + height^2))`, the largest rho the image can produce.
pub fn diagonal(width: usize, height: usize) usize {
    const squared: f32 = @floatFromInt(width * width + height * height);
    return @intFromFloat(@ceil(@sqrt(squared)));
}

/// How many ints `houghLines` needs: `(2 * diagonal + 1) * 180`.
pub fn linesAccumulatorLen(width: usize, height: usize) usize {
    return (diagonal(width, height) * 2 + 1) * theta_count;
}

/// How many ints `houghCircles` needs.
pub fn circlesAccumulatorLen(width: usize, height: usize, min_radius: usize, max_radius: usize) usize {
    if (max_radius < min_radius) return 0;
    return width * height * (max_radius - min_radius + 1);
}

/// The C's `(int)` cast on a float, made total.
///
/// Every conversion in the C from a float to `int` is undefined once the value
/// leaves the range of `int`, and `drawLine` divides by a cosine that can be tiny.
/// Clamping keeps the port defined; the corpus never reaches the clamp, because
/// its lines come from the detector.
inline fn toI64(value: f32) i64 {
    if (std.math.isNan(value)) return 0;
    const limit: f32 = 4.0e18;
    if (value > limit) return 4611686018427387904;
    if (value < -limit) return -4611686018427387904;
    return @intFromFloat(value);
}

/// Detect straight lines. The return value is how many were written.
pub fn houghLines(
    src: []const u8,
    lines: []Line,
    accumulator: []i32,
    width: usize,
    height: usize,
    threshold: i32,
    stride_bytes: usize,
) Error!usize {
    const stride = common.strideOf(width, stride_bytes);
    try common.checkRegion(src.len, width, height, stride);
    if (width == 0 or height == 0) return Error.InvalidSize;

    // The C returns 0 for "no room for lines" as well as for "found none"; with a
    // slice, a caller who wants none simply passes an empty one.
    if (lines.len == 0) return 0;

    const rho_count = diagonal(width, height) * 2 + 1;
    if (linesAccumulatorLen(width, height) > accumulator.len) return Error.BufferTooSmall;

    const table = accumulator[0 .. rho_count * theta_count];
    @memset(table, 0);

    for (0..height) |y| {
        for (0..width) |x| {
            if (src[y * stride + x] == 0) continue;

            for (0..theta_count) |i| {
                const theta = @as(f32, @floatFromInt(i)) * theta_step;
                const rho = @as(f32, @floatFromInt(x)) * @cos(theta) +
                    @as(f32, @floatFromInt(y)) * @sin(theta);

                // Round to the nearest rho bin; the C floored. `(int)(rho + diagonal)`
                // on a non-negative sum *is* a floor, and it put a line whose pixels
                // measured rho 1.93 into the bin that reports 1, so a horizontal line
                // at y = 2 came back as rho 1. The diagonal offset stays, because it
                // is what makes a negative rho indexable at all - and it has to be
                // applied to the rounded value, since rounding after the offset would
                // round the offset too.
                const rho_idx = toI64(@round(rho)) + @as(i64, @intCast(rho_count / 2));

                if (rho_idx >= 0 and rho_idx < rho_count) {
                    table[@as(usize, @intCast(rho_idx)) * theta_count + i] += 1;
                }
            }
        }
    }

    var line_count: usize = 0;
    // The whole accumulator is scanned now, and the neighbourhood is taken with the
    // two axes treated as what they are: `rho` is bounded, so its index is clamped;
    // `theta` is an orientation with period 180 degrees, so its index wraps. The C
    // ran `j` from 1 to `theta_count - 2`, which made theta = 0 and theta = 179
    // unreachable - and a vertical line's votes pile up in the first of those.
    for (0..rho_count) |i| {
        if (line_count == lines.len) break;
        for (0..theta_count) |j| {
            if (line_count == lines.len) break;

            const value = table[i * theta_count + j];
            if (value <= threshold) continue;

            var is_max = true;
            for (0..3) |ni| {
                for (0..3) |nj| {
                    if (ni == 1 and nj == 1) continue;

                    const neighbour_rho = std.math.clamp(
                        @as(isize, @intCast(i)) + @as(isize, @intCast(ni)) - 1,
                        0,
                        @as(isize, @intCast(rho_count - 1)),
                    );
                    const neighbour_theta = (j + nj + theta_count - 1) % theta_count;
                    if (table[@as(usize, @intCast(neighbour_rho)) * theta_count + neighbour_theta] > value) {
                        is_max = false;
                    }
                }
            }

            if (is_max) {
                lines[line_count] = .{
                    .rho = @as(f32, @floatFromInt(@as(i64, @intCast(i)) - @as(i64, @intCast(rho_count / 2)))) * rho_step,
                    .theta = @as(f32, @floatFromInt(j)) * theta_step,
                    .votes = value,
                };
                line_count += 1;
            }
        }
    }
    return line_count;
}

/// Draw a line with Bresenham, clipping it to the image.
pub fn drawLine(
    dst: []u8,
    width: usize,
    height: usize,
    line: Line,
    color: u8,
    stride_bytes: usize,
) Error!void {
    const stride = common.strideOf(width, stride_bytes);
    try common.checkRegion(dst.len, width, height, stride);
    if (width == 0 or height == 0) return Error.InvalidSize;

    const cos_theta = @cos(line.theta);
    const sin_theta = @sin(line.theta);
    const last_x: i64 = @intCast(width - 1);
    const last_y: i64 = @intCast(height - 1);

    var x0: i64 = undefined;
    var y0: i64 = undefined;
    var x1: i64 = undefined;
    var y1: i64 = undefined;

    if (@abs(sin_theta) < 0.001) {
        // Vertical: the C computes x0 = x1 = rho / cos(theta).
        x0 = toI64(line.rho / cos_theta);
        x1 = x0;
        y0 = 0;
        y1 = last_y;
    } else if (@abs(cos_theta) < 0.001) {
        y0 = toI64(line.rho / sin_theta);
        y1 = y0;
        x0 = 0;
        x1 = last_x;
    } else {
        x0 = 0;
        y0 = toI64(line.rho / sin_theta);
        x1 = last_x;
        y1 = toI64((line.rho - @as(f32, @floatFromInt(x1)) * cos_theta) / sin_theta);

        if (y0 < 0 or y0 >= @as(i64, @intCast(height))) {
            if (y0 < 0) {
                y0 = 0;
                x0 = toI64((line.rho - @as(f32, @floatFromInt(y0)) * sin_theta) / cos_theta);
            } else {
                y0 = last_y;
                x0 = toI64((line.rho - @as(f32, @floatFromInt(y0)) * sin_theta) / cos_theta);
            }
        }

        if (y1 < 0 or y1 >= @as(i64, @intCast(height))) {
            if (y1 < 0) {
                y1 = 0;
                x1 = toI64((line.rho - @as(f32, @floatFromInt(y1)) * sin_theta) / cos_theta);
            } else {
                y1 = last_y;
                x1 = toI64((line.rho - @as(f32, @floatFromInt(y1)) * sin_theta) / cos_theta);
            }
        }
    }

    const dx = if (x1 > x0) x1 - x0 else x0 - x1;
    const dy = if (y1 > y0) y1 - y0 else y0 - y1;
    const sx: i64 = if (x0 < x1) 1 else -1;
    const sy: i64 = if (y0 < y1) 1 else -1;
    var err = dx - dy;

    var x = x0;
    var y = y0;
    while (true) {
        if (x >= 0 and y >= 0 and x < @as(i64, @intCast(width)) and y < @as(i64, @intCast(height))) {
            dst[@as(usize, @intCast(y)) * stride + @as(usize, @intCast(x))] = color;
        }
        if (x == x1 and y == y1) break;

        const e2 = 2 * err;
        if (e2 > -dy) {
            err -= dy;
            x += sx;
        }
        if (e2 < dx) {
            err += dx;
            y += sy;
        }
    }
}

/// Detect circles. The return value is how many were written.
pub fn houghCircles(
    src: []const u8,
    circles: []Circle,
    accumulator: []i32,
    width: usize,
    height: usize,
    min_radius: usize,
    max_radius: usize,
    threshold: i32,
    stride_bytes: usize,
) Error!usize {
    const stride = common.strideOf(width, stride_bytes);
    try common.checkRegion(src.len, width, height, stride);
    if (width == 0 or height == 0) return Error.InvalidSize;
    if (max_radius <= min_radius) return Error.InvalidRadiusRange;
    if (circles.len == 0) return 0;

    const radius_count = max_radius - min_radius + 1;
    if (circlesAccumulatorLen(width, height, min_radius, max_radius) > accumulator.len) {
        return Error.BufferTooSmall;
    }

    const table = accumulator[0 .. width * height * radius_count];
    @memset(table, 0);

    for (0..height) |y| {
        for (0..width) |x| {
            if (src[y * stride + x] == 0) continue;

            for (min_radius..max_radius + 1) |r| {
                // Five degrees per vote, 72 samples around the circle.
                var angle: usize = 0;
                while (angle < 360) : (angle += 5) {
                    // The C writes this one out as `angle * 3.14159265f / 180.0f`,
                    // where the line loop uses its precomputed `theta_step`. The two
                    // agree to the last bit only sometimes, and this one decides
                    // votes: a probe of the difference shows the ring below losing a
                    // vote if the precomputed step is used here instead.
                    const rad = @as(f32, @floatFromInt(angle)) * 3.14159265 / 180.0;
                    const a = toI64(@as(f32, @floatFromInt(x)) - @as(f32, @floatFromInt(r)) * @cos(rad));
                    const b = toI64(@as(f32, @floatFromInt(y)) - @as(f32, @floatFromInt(r)) * @sin(rad));

                    if (a >= 0 and b >= 0 and a < @as(i64, @intCast(width)) and b < @as(i64, @intCast(height))) {
                        const centre = @as(usize, @intCast(a)) * height + @as(usize, @intCast(b));
                        table[centre * radius_count + (r - min_radius)] += 1;
                    }
                }
            }
        }
    }

    var circle_count: usize = 0;
    // A 1-pixel-wide or 1-pixel-tall image has no interior to scan, and the C's
    // loops are simply empty there (`1 < width - 1` is false). In Zig
    // `1..width - 1` underflows instead, so the empty case is spelled out.
    if (width < 3 or height < 3) return 0;

    // x outside, then y, then radius - the order a cap cuts off in.
    for (1..width - 1) |x| {
        if (circle_count == circles.len) break;
        for (1..height - 1) |y| {
            if (circle_count == circles.len) break;
            for (0..radius_count) |r| {
                if (circle_count == circles.len) break;

                const value = table[(x * height + y) * radius_count + r];
                if (value <= threshold) continue;

                var is_max = true;
                for (0..3) |dx| {
                    for (0..3) |dy| {
                        for (0..3) |dr| {
                            if (dx == 1 and dy == 1 and dr == 1) continue;

                            const nx = @as(i64, @intCast(x)) + @as(i64, @intCast(dx)) - 1;
                            const ny = @as(i64, @intCast(y)) + @as(i64, @intCast(dy)) - 1;
                            const nr = @as(i64, @intCast(r)) + @as(i64, @intCast(dr)) - 1;

                            if (nx < 0 or ny < 0 or nr < 0) continue;
                            if (nx >= @as(i64, @intCast(width)) or ny >= @as(i64, @intCast(height))) continue;
                            if (nr >= @as(i64, @intCast(radius_count))) continue;

                            const neighbour = (@as(usize, @intCast(nx)) * height + @as(usize, @intCast(ny))) *
                                radius_count + @as(usize, @intCast(nr));
                            if (table[neighbour] > value) is_max = false;
                        }
                    }
                }

                if (is_max) {
                    circles[circle_count] = .{
                        .x = x,
                        .y = y,
                        .radius = r + min_radius,
                        .votes = value,
                    };
                    circle_count += 1;
                }
            }
        }
    }
    return circle_count;
}

// --- tests ------------------------------------------------------------------

const cross = [25]u8{
    0,   0,   255, 0,   0,
    0,   0,   255, 0,   0,
    255, 255, 255, 255, 255,
    0,   0,   255, 0,   0,
    0,   0,   255, 0,   0,
};

const ring = [49]u8{
    0, 0,   0,   0,   0,   0,   0,
    0, 0,   255, 255, 255, 0,   0,
    0, 255, 0,   0,   0,   255, 0,
    0, 255, 0,   0,   0,   255, 0,
    0, 255, 0,   0,   0,   255, 0,
    0, 0,   255, 255, 255, 0,   0,
    0, 0,   0,   0,   0,   0,   0,
};

fn expectBytes(corpus: *const corpus_mod.Corpus, name: []const u8, bytes: []const u8) !void {
    var as_floats: [64]f32 = undefined;
    for (bytes, 0..) |b, i| as_floats[i] = @floatFromInt(b);
    try corpus.expectValues(name, as_floats[0..bytes.len]);
}

/// The line cases are `name count rho theta votes ...`, so the count is the first
/// value and each line is three of them. The count is compared here rather than
/// with `expectInt`, which is for cases that hold that one number.
fn expectLines(corpus: *const corpus_mod.Corpus, name: []const u8, lines: []const Line, count: usize) !void {
    const values = try corpus.get(name);
    if (values.len == 0) return error.ExpectedOneValue;

    if (values[0] != @as(f32, @floatFromInt(count))) {
        std.debug.print("corpus case '{s}': C found {d} line(s), port found {d}\n", .{ name, values[0], count });
        return error.LineCountMismatch;
    }
    try std.testing.expectEqual(@as(usize, 1 + count * 3), values.len);

    for (lines[0..count], 0..) |line, i| {
        try compareLine(corpus, name, values, line, i);
    }
}

fn compareLine(corpus: *const corpus_mod.Corpus, name: []const u8, values: []const f32, line: Line, i: usize) !void {
    try corpus.expectClose(name, 1 + i * 3, values[1 + i * 3], line.rho);
    try corpus.expectClose(name, 2 + i * 3, values[2 + i * 3], line.theta);
    try std.testing.expectEqual(@as(i32, @intFromFloat(values[3 + i * 3])), line.votes);
}

fn expectCircles(corpus: *const corpus_mod.Corpus, name: []const u8, circles: []const Circle, count: usize) !void {
    const values = try corpus.get(name);
    if (values.len == 0) return error.ExpectedOneValue;

    if (values[0] != @as(f32, @floatFromInt(count))) {
        std.debug.print("corpus case '{s}': C found {d} circle(s), port found {d}\n", .{ name, values[0], count });
        return error.CircleCountMismatch;
    }
    try std.testing.expectEqual(@as(usize, 1 + count * 4), values.len);

    for (circles[0..count], 0..) |circle, i| {
        try std.testing.expectEqual(@as(usize, @intFromFloat(values[1 + i * 4])), circle.x);
        try std.testing.expectEqual(@as(usize, @intFromFloat(values[2 + i * 4])), circle.y);
        try std.testing.expectEqual(@as(usize, @intFromFloat(values[3 + i * 4])), circle.radius);
        try std.testing.expectEqual(@as(i32, @intFromFloat(values[4 + i * 4])), circle.votes);
    }
}

test "hough lines: the cross, the cap, and the blank image" {
    const corpus = try corpus_mod.Corpus.load();

    var accumulator: [64 * theta_count]i32 = undefined;
    var lines: [8]Line = undefined;

    var count = try houghLines(&cross, &lines, &accumulator, 5, 5, 3, 0);
    try expectLines(&corpus, "hough_lines_cross", &lines, count);

    // A threshold at the winning vote count finds nothing: the comparison is
    // strictly `>`.
    count = try houghLines(&cross, &lines, &accumulator, 5, 5, 5, 0);
    try expectLines(&corpus, "hough_lines_threshold_at_peak", &lines, count);

    // A blank image writes no line at all, which the -1 sentinels make visible.
    for (&lines) |*line| line.* = .{ .rho = -1, .theta = -1, .votes = -1 };
    const blank = [_]u8{0} ** 25;
    count = try houghLines(&blank, &lines, &accumulator, 5, 5, 0, 0);
    try expectLines(&corpus, "hough_lines_blank", &lines, count);
    try std.testing.expectEqual(@as(f32, -1), lines[0].rho);

    // The cap cuts the scan off in rho-then-theta order, so one line means the
    // first maximum in that order, not the strongest.
    var one: [1]Line = undefined;
    count = try houghLines(&cross, &one, &accumulator, 5, 5, 3, 0);
    try std.testing.expectEqual(@as(usize, 1), count);
    try expectLines(&corpus, "hough_lines_capped_at_1", &one, 1);

    // And an empty output slice is "no room", which is not an error.
    try std.testing.expectEqual(@as(usize, 0), try houghLines(&cross, &[_]Line{}, &accumulator, 5, 5, 3, 0));
}

test "hough lines: the vertical, the horizontal, and the columns theta 0 and 179" {
    const corpus = try corpus_mod.Corpus.load();

    var vertical = [_]u8{0} ** 25;
    var horizontal = [_]u8{0} ** 25;
    for (0..5) |i| {
        vertical[i * 5 + 2] = 255;
        horizontal[2 * 5 + i] = 255;
    }

    var accumulator: [64 * theta_count]i32 = undefined;
    var lines: [4]Line = undefined;

    var count = try houghLines(&vertical, &lines, &accumulator, 5, 5, 3, 0);
    try expectLines(&corpus, "hough_lines_vertical_only", &lines, count);

    // Four lines and every one of them at the far end of the range: with a cap
    // this small the answer is the near-180 twin of the peak, which holds the same
    // five pixels. That is scan order, not invisibility - rho is the outer loop and
    // the twin is at rho -2, so the cap is full before the scan reaches the peak at
    // rho +2. The wide-cap call below separates the two.
    for (lines[0..count]) |line| {
        try std.testing.expect(line.theta > 2.8);
        try std.testing.expect(line.theta < 3.14159265);
    }

    // Raise the cap and the peak is there, at one of the two columns the C never
    // examined: theta = 0, rho = 2, all five pixels.
    var wide: [64]Line = undefined;
    const wide_count = try houghLines(&vertical, &wide, &accumulator, 5, 5, 3, 0);
    var found_theta_zero = false;
    for (wide[0..wide_count]) |line| {
        if (line.theta == 0.0 and line.rho == 2.0 and line.votes == 5) found_theta_zero = true;
    }
    try std.testing.expect(found_theta_zero);

    // At the peak's own vote count the answer is empty either way: the comparison
    // is strictly `>`.
    count = try houghLines(&vertical, &lines, &accumulator, 5, 5, 5, 0);
    try expectLines(&corpus, "hough_lines_vertical_at_peak", &lines, count);
    try std.testing.expectEqual(@as(usize, 0), count);

    // The same line rotated is found at rho 2 - the line's own distance, because
    // the bin is now the nearest one. The C reported 1: at theta = 80 degrees the
    // five pixels measure rho 1.97 to 2.66, and its floor put all of them in the
    // bin below. Four of them are in the nearest bin and the fifth rounds up to 3,
    // which is why the first line here has four votes rather than five - the same
    // spread the floor collapsed into one bin, one bin too low.
    count = try houghLines(&horizontal, &lines, &accumulator, 5, 5, 3, 0);
    try expectLines(&corpus, "hough_lines_horizontal_only", &lines, count);
    try std.testing.expectEqual(@as(f32, 2.0), lines[0].rho);
    try std.testing.expectEqual(@as(i32, 4), lines[0].votes);

    // A long vertical line, at a threshold only its peak clears: at theta = 0 all
    // eleven pixels land in one bin, while one degree away it is ten and one, and
    // ten is not greater than ten. The answer now *contains* that line - the C's
    // four lines were all the rho -5 twin, and the peak was nowhere in the answer.
    var tall = [_]u8{0} ** 121;
    for (0..11) |i| tall[i * 11 + 5] = 255;
    var tall_lines: [5]Line = undefined;
    const tall_count = try houghLines(&tall, &tall_lines, &accumulator, 11, 11, 10, 0);
    try expectLines(&corpus, "hough_lines_long_vertical", &tall_lines, tall_count);

    var found_tall_peak = false;
    for (tall_lines[0..tall_count]) |line| {
        if (line.theta == 0.0 and line.rho == 5.0 and line.votes == 11) found_tall_peak = true;
    }
    try std.testing.expect(found_tall_peak);
    // The other four are the twin again, and the cap is now exactly full: the
    // rounding change is why there are only two of them rather than the C's four.
    try std.testing.expectEqual(@as(usize, 5), tall_count);
    var twins: usize = 0;
    for (tall_lines[0..tall_count]) |line| {
        if (line.rho == -5.0) {
            twins += 1;
            try std.testing.expect(line.theta > 3.0);
        }
    }
    try std.testing.expectEqual(@as(usize, 2), twins);
}

test "hough lines: theta is an orientation, so 0 and 179 are neighbours" {
    // Fixing the scan range raises a question the C never had to answer: is the bin
    // at theta = 0 a maximum? Its neighbour across the seam is at theta = 179, and
    // whether that one is compared decides the answer. This fixture makes the two
    // differ by a single pixel.
    //
    // Five pixels in the x = 0 column land in the rho = 0 bin at *both* ends of the
    // range, because x = 0 leaves rho = y * sin(theta) and y = 0..4 stays inside
    // half a bin of zero at either end. One more pixel, at x = 1, y = 29, measures
    // rho = -0.494 at 179 degrees - it rounds into that same bin - while at 0
    // degrees it measures 1. So the bin at 179 holds six votes and the one at 0
    // holds five, and at 0 degrees the seam neighbour is strictly larger.
    var src = [_]u8{0} ** (5 * 30);
    for (0..5) |y| src[y * 5] = 255;
    src[29 * 5 + 1] = 255;

    var accumulator: [64 * theta_count]i32 = undefined;
    var lines: [96]Line = undefined;
    const count = try houghLines(&src, &lines, &accumulator, 5, 30, 4, 0);

    var at_zero = false;
    var at_178 = false;
    var at_179 = false;
    for (lines[0..count]) |line| {
        if (line.rho != 0.0) continue;
        if (line.theta == 0.0) at_zero = true;
        if (line.theta == @as(f32, 178.0) * theta_step and line.votes == 6) at_178 = true;
        if (line.theta == @as(f32, 179.0) * theta_step and line.votes == 6) at_179 = true;
    }
    // Compared against its seam neighbour, the five-vote bin is not a maximum...
    try std.testing.expect(!at_zero);
    // ...while the six-vote bin is, and so is its equal neighbour one degree in.
    try std.testing.expect(at_178);
    try std.testing.expect(at_179);
}

test "hough draw: vertical, horizontal and oblique clipping match the C" {
    const corpus = try corpus_mod.Corpus.load();

    var dst = [_]u8{0xEE} ** 25;
    try drawLine(&dst, 5, 5, .{ .rho = 2, .theta = 0, .votes = 5 }, 255, 0);
    try expectBytes(&corpus, "hough_draw_vertical", &dst);

    dst = [_]u8{0xEE} ** 25;
    try drawLine(&dst, 5, 5, .{ .rho = 2, .theta = 1.57079633, .votes = 5 }, 255, 0);
    try expectBytes(&corpus, "hough_draw_horizontal", &dst);

    dst = [_]u8{0xEE} ** 25;
    try drawLine(&dst, 5, 5, .{ .rho = 3, .theta = 0.78539816, .votes = 5 }, 255, 0);
    try expectBytes(&corpus, "hough_draw_oblique", &dst);

    // A 4:2 slope, which is what it takes to land Bresenham's error term exactly
    // on its decision boundary. The three above have dx or dy zero, or dx equal to
    // dy, and never reach it - so a probe that changed the comparison from `>` to
    // `>=` stayed green until this case existed.
    dst = [_]u8{0xEE} ** 25;
    try drawLine(&dst, 5, 5, .{ .rho = 2, .theta = 1.10714872, .votes = 5 }, 255, 0);
    try expectBytes(&corpus, "hough_draw_shallow", &dst);

    // Steeper still, and this is the one that reaches Bresenham's boundary: `e2`
    // only equals `-dy` once the error term has gone negative, which needs dy
    // greater than dx. The 4:2 slope above does not get there, and a probe that
    // changed `>` to `>=` stayed green against it.
    dst = [_]u8{0xEE} ** 25;
    try drawLine(&dst, 5, 5, .{ .rho = 3.969, .theta = 0.12435499, .votes = 5 }, 255, 0);
    try expectBytes(&corpus, "hough_draw_steep", &dst);

    // Independent of the fixture: the vertical line writes exactly one column and
    // the horizontal exactly one row, each five pixels long.
    var column = [_]u8{0} ** 25;
    try drawLine(&column, 5, 5, .{ .rho = 2, .theta = 0, .votes = 5 }, 255, 0);
    var column_set: usize = 0;
    for (column, 0..) |byte, i| {
        if (byte != 0) {
            column_set += 1;
            try std.testing.expectEqual(@as(usize, 2), i % 5);
        }
    }
    try std.testing.expectEqual(@as(usize, 5), column_set);
}

test "hough circles: the ring, and a threshold above every vote" {
    const corpus = try corpus_mod.Corpus.load();

    var accumulator: [7 * 7 * 3]i32 = undefined;
    var circles: [4]Circle = undefined;

    var count = try houghCircles(&ring, &circles, &accumulator, 7, 7, 1, 3, 3, 0);
    try expectCircles(&corpus, "hough_circles_ring", &circles, count);

    // The true centre is among them - at radius 2, where the ring is.
    var found_true_centre = false;
    for (circles[0..count]) |circle| {
        if (circle.x == 3 and circle.y == 3 and circle.radius == 2) found_true_centre = true;
    }
    try std.testing.expect(found_true_centre);

    for (&circles) |*circle| circle.* = .{ .x = 0, .y = 0, .radius = 0, .votes = -1 };
    count = try houghCircles(&ring, &circles, &accumulator, 7, 7, 1, 3, 1000, 0);
    try expectCircles(&corpus, "hough_circles_threshold_above_all", &circles, count);
    try std.testing.expectEqual(@as(usize, 0), count);
    try std.testing.expectEqual(@as(i32, -1), circles[0].votes);

    // A one-pixel image has no interior for the peak scan, which is an empty loop
    // in the C and an underflow panic in Zig without the guard: this is a crash
    // becoming a no-op, not a behaviour change.
    var single = [_]u8{255};
    var single_acc = [_]i32{0} ** 2;
    var single_circles: [1]Circle = undefined;
    try std.testing.expectEqual(
        @as(usize, 0),
        try houghCircles(&single, &single_circles, &single_acc, 1, 1, 1, 2, 0, 0),
    );
}

test "hough: the buffers are checked, and the accumulator sizes are exact" {
    var accumulator: [64 * theta_count]i32 = undefined;
    var lines: [4]Line = undefined;
    var circles: [4]Circle = undefined;

    // The accumulator helper agrees with what the function needs.
    try std.testing.expectEqual(@as(usize, (8 * 2 + 1) * theta_count), linesAccumulatorLen(5, 5));
    try std.testing.expectEqual(@as(usize, 7 * 7 * 3), circlesAccumulatorLen(7, 7, 1, 3));

    var small = [_]i32{0} ** 16;
    const small_src = [_]u8{0} ** 24;
    try std.testing.expectError(Error.BufferTooSmall, houghLines(&cross, &lines, &small, 5, 5, 3, 0));
    try std.testing.expectError(Error.BufferTooSmall, houghLines(&small_src, &lines, &accumulator, 5, 5, 3, 0));
    try std.testing.expectError(Error.InvalidSize, houghLines(&cross, &lines, &accumulator, 0, 5, 3, 0));

    try std.testing.expectError(
        Error.BufferTooSmall,
        houghCircles(&ring, &circles, &small, 7, 7, 1, 3, 3, 0),
    );
    // The C refuses a single-radius search by returning 0; here it is an error.
    try std.testing.expectError(
        Error.InvalidRadiusRange,
        houghCircles(&ring, &circles, &accumulator, 7, 7, 3, 3, 3, 0),
    );

    var dst: [24]u8 = undefined;
    try std.testing.expectError(Error.BufferTooSmall, drawLine(&dst, 5, 5, .{ .rho = 2, .theta = 0, .votes = 5 }, 255, 0));
    try std.testing.expectError(Error.InvalidSize, drawLine(&dst, 0, 5, .{ .rho = 2, .theta = 0, .votes = 5 }, 255, 0));
}
