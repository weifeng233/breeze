//! Histograms and equalisation, ported from `include/breeze/image/histogram.h`.
//!
//! Four functions. `compute` and `equalize` take an image; `cumulative` takes a
//! 256-bin histogram, and `clahe` builds one per tile and interpolates between
//! them. The C allocates 2 * tile_count tables of 256 entries inside CLAHE (26
//! allocation calls in the file); those are two caller-provided slices here, and
//! the three-level pointer tables become flat slices with index arithmetic, so
//! the C's four-stage partial-allocation cleanup disappears with them.
//!
//! Two things the C does are worth stating before the code:
//!
//!   * **The tiles do not necessarily cover the image.** `tile_width` is
//!     `width / tile_count_x`, an integer division, so a 5x5 image with a tile
//!     size of 2 gives three tiles of width 1 and columns 3 and 4 are never
//!     histogrammed - though the final interpolation still reads a lookup table
//!     for them. The corpus records it (`histogram_clahe_5x5_tile2_uneven`).
//!   * **One tile along an axis is a crash in the C.** The interpolation pulls
//!     `ty_i` back with `ty_i = tile_count_y - 2` when `ty_i >= tile_count_y - 1`,
//!     which is -1 when there is only one row of tiles, and the next line reads
//!     `luts[-1]`. The archived `probe_clahe_single_tile.c` reproduces it: a 6x4
//!     image with a tile size of 6 dies with an access violation. A caller reaches
//!     this by passing a tile size at or above the image size - something the
//!     function's own clamping makes ordinary. There is no behaviour to reproduce
//!     there, so the port uses the single tile directly, which is also what
//!     interpolating between one tile means (docs/REVIEW.md §48).

const std = @import("std");

const common = @import("common.zig");
const corpus_mod = @import("../math/corpus.zig");

pub const Error = common.Error;

pub const bins = 256;

/// The histogram of a grayscale image.
pub fn compute(
    src: []const u8,
    histogram: *[bins]i32,
    width: usize,
    height: usize,
    stride_bytes: usize,
) Error!void {
    const stride = common.strideOf(width, stride_bytes);
    try common.checkRegion(src.len, width, height, stride);

    @memset(histogram, 0);
    for (0..height) |y| {
        for (0..width) |x| {
            histogram[src[y * stride + x]] += 1;
        }
    }
}

/// The prefix sums of a histogram.
pub fn cumulative(histogram: *const [bins]i32, cumulative_histogram: *[bins]i32) void {
    cumulative_histogram[0] = histogram[0];
    for (1..bins) |i| {
        cumulative_histogram[i] = cumulative_histogram[i - 1] + histogram[i];
    }
}

/// The lookup table equalisation builds: the scaled cumulative, rounded.
inline fn equalisationLut(cumulative_histogram: *const [bins]i32, total_pixels: usize) [bins]u8 {
    var lut: [bins]u8 = undefined;
    const total: f32 = @floatFromInt(total_pixels);
    for (0..bins) |i| {
        // `255 * cum / total` is at most 255, so the cast is always in range.
        const scaled = 255.0 * @as(f32, @floatFromInt(cumulative_histogram[i])) / total + 0.5;
        lut[i] = @intFromFloat(scaled);
    }
    return lut;
}

/// Global histogram equalisation.
pub fn equalize(
    src: []const u8,
    dst: []u8,
    width: usize,
    height: usize,
    stride_bytes: usize,
) Error!void {
    const stride = common.strideOf(width, stride_bytes);
    try common.checkRegion(src.len, width, height, stride);
    try common.checkRegion(dst.len, width, height, stride);

    var histogram: [bins]i32 = undefined;
    var cumulative_histogram: [bins]i32 = undefined;
    try compute(src, &histogram, width, height, stride_bytes);
    cumulative(&histogram, &cumulative_histogram);

    const lut = equalisationLut(&cumulative_histogram, width * height);
    for (0..height) |y| {
        for (0..width) |x| {
            const idx = y * stride + x;
            dst[idx] = lut[src[idx]];
        }
    }
}

const TileGrid = struct {
    count_x: usize,
    count_y: usize,
    width: usize,
    height: usize,
};

fn tileGrid(width: usize, height: usize, tile_size: usize) Error!TileGrid {
    if (width == 0 or height == 0 or tile_size == 0) return Error.InvalidSize;

    var tile = tile_size;
    if (tile > width) tile = width;
    if (tile > height) tile = height;

    const count_x = (width + tile - 1) / tile;
    const count_y = (height + tile - 1) / tile;
    return .{
        .count_x = count_x,
        .count_y = count_y,
        .width = width / count_x,
        .height = height / count_y,
    };
}

/// How many ints `clahe` needs for its histograms - the same number of bytes for
/// its lookup tables.
pub fn claheTablesLen(width: usize, height: usize, tile_size: usize) usize {
    const grid = tileGrid(width, height, tile_size) catch return 0;
    return grid.count_x * grid.count_y * bins;
}

/// Contrast-limited adaptive equalisation, with bilinear interpolation between
/// the tiles.
pub fn clahe(
    src: []const u8,
    dst: []u8,
    histograms: []i32,
    luts: []u8,
    width: usize,
    height: usize,
    tile_size: usize,
    clip_limit: f32,
    stride_bytes: usize,
) Error!void {
    const stride = common.strideOf(width, stride_bytes);
    try common.checkRegion(src.len, width, height, stride);
    try common.checkRegion(dst.len, width, height, stride);

    const grid = try tileGrid(width, height, tile_size);
    const tables = grid.count_x * grid.count_y * bins;
    if (tables > histograms.len) return Error.BufferTooSmall;
    if (tables > luts.len) return Error.BufferTooSmall;

    // Per-tile histograms, then the clip, then each tile's lookup table.
    for (0..grid.count_y) |ty| {
        for (0..grid.count_x) |tx| {
            const hist = histograms[(ty * grid.count_x + tx) * bins ..][0..bins];
            @memset(hist, 0);

            const start_y = ty * grid.height;
            const end_y = @min((ty + 1) * grid.height, height);
            const start_x = tx * grid.width;
            const end_x = @min((tx + 1) * grid.width, width);

            for (start_y..end_y) |y| {
                for (start_x..end_x) |x| {
                    hist[src[y * stride + x]] += 1;
                }
            }

            if (clip_limit > 0) {
                const tile_pixels = (end_y - start_y) * (end_x - start_x);
                const clip_value: i32 = @intFromFloat(
                    clip_limit * @as(f32, @floatFromInt(tile_pixels)) / 256.0 + 0.5,
                );

                var redistribution: i32 = 0;
                for (hist) |*count| {
                    // `>` against `>=` makes no difference here, and that is worth
                    // knowing rather than assuming: when a count equals the limit,
                    // one branch leaves it at the limit and the other assigns the
                    // limit, and both add `count - clip_value`, which is zero. A
                    // probe that changed the comparison stayed green.
                    if (count.* > clip_value) {
                        redistribution += count.* - clip_value;
                        count.* = clip_value;
                    }
                }

                // The remainder of the division is dropped - the C does the same,
                // and on a small tile it is the whole redistribution.
                const per_bin = @divTrunc(redistribution, bins);
                for (hist) |*count| count.* += per_bin;
            }

            var cumulative_histogram: [bins]i32 = undefined;
            cumulative(hist, &cumulative_histogram);

            const lut = luts[(ty * grid.count_x + tx) * bins ..][0..bins];
            const total = @min((ty + 1) * grid.height, height) - ty * grid.height;
            const total_x = @min((tx + 1) * grid.width, width) - tx * grid.width;
            const tile_pixels = total * total_x;
            const tile_total: f32 = @floatFromInt(tile_pixels);
            for (0..bins) |i| {
                lut[i] = @intFromFloat(255.0 * @as(f32, @floatFromInt(cumulative_histogram[i])) / tile_total + 0.5);
            }
        }
    }

    // Bilinear interpolation between the four nearest tiles.
    const tile_height_f: f32 = @floatFromInt(grid.height);
    const tile_width_f: f32 = @floatFromInt(grid.width);

    for (0..height) |y| {
        for (0..width) |x| {
            const ty_f = @as(f32, @floatFromInt(y)) / tile_height_f;
            const tx_f = @as(f32, @floatFromInt(x)) / tile_width_f;
            var ty_i: usize = @intFromFloat(ty_f);
            var tx_i: usize = @intFromFloat(tx_f);
            var ty_alpha = ty_f - @as(f32, @floatFromInt(ty_i));
            var tx_alpha = tx_f - @as(f32, @floatFromInt(tx_i));

            // With a single tile along an axis there is nothing to interpolate
            // along it, and the C's `tile_count - 2` would be -1 - see the module
            // comment. Multi-tile axes keep the C's clamp, which is what keeps
            // `ty_i + 1` in range.
            if (grid.count_y == 1) {
                ty_i = 0;
                ty_alpha = 0.0;
            } else if (ty_i >= grid.count_y - 1) {
                ty_i = grid.count_y - 2;
                ty_alpha = 1.0;
            }
            if (grid.count_x == 1) {
                tx_i = 0;
                tx_alpha = 0.0;
            } else if (tx_i >= grid.count_x - 1) {
                tx_i = grid.count_x - 2;
                tx_alpha = 1.0;
            }

            const value = src[y * stride + x];
            // With one tile along an axis the upper neighbour does not exist, and
            // reading it would be exactly the out-of-range access the C makes - the
            // alpha being zero is not enough, because the value is read first. The
            // first version of this fix did read it, and the bounds check above
            // turned the C's memory corruption into a panic here.
            const ty_upper = if (grid.count_y == 1) 0 else ty_i + 1;
            const tx_upper = if (grid.count_x == 1) 0 else tx_i + 1;

            const row0 = ty_i * grid.count_x;
            const row1 = ty_upper * grid.count_x;
            const v00: f32 = @floatFromInt(luts[(row0 + tx_i) * bins + value]);
            const v01: f32 = @floatFromInt(luts[(row0 + tx_upper) * bins + value]);
            const v10: f32 = @floatFromInt(luts[(row1 + tx_i) * bins + value]);
            const v11: f32 = @floatFromInt(luts[(row1 + tx_upper) * bins + value]);

            const v0 = v00 * (1.0 - tx_alpha) + v01 * tx_alpha;
            const v1 = v10 * (1.0 - tx_alpha) + v11 * tx_alpha;
            const v = v0 * (1.0 - ty_alpha) + v1 * ty_alpha;

            // Four bytes at most, weighted by fractions of one: always in range.
            dst[y * stride + x] = @intFromFloat(v + 0.5);
        }
    }
}

// --- tests ------------------------------------------------------------------

const img = [16]u8{
    0,   0,   0,   77,
    77,  77,  200, 200,
    200, 255, 255, 255,
    0,   77,  200, 255,
};

fn expectBytes(corpus: *const corpus_mod.Corpus, name: []const u8, bytes: []const u8) !void {
    var as_floats: [64]f32 = undefined;
    for (bytes, 0..) |b, i| as_floats[i] = @floatFromInt(b);
    try corpus.expectValues(name, as_floats[0..bytes.len]);
}

/// The histogram cases are `name pairs index count ...`, i.e. only the non-zero
/// bins. Rebuilding the full 256 and comparing exactly is stronger than checking
/// the listed bins: a stray count anywhere else shows up.
fn expectHistogram(corpus: *const corpus_mod.Corpus, name: []const u8, histogram: *const [bins]i32) !void {
    const values = try corpus.get(name);
    const pairs: usize = @intFromFloat(values[0]);
    try std.testing.expectEqual(@as(usize, 1 + pairs * 2), values.len);

    var expected = [_]i32{0} ** bins;
    for (0..pairs) |i| {
        const index: usize = @intFromFloat(values[1 + i * 2]);
        const count: i32 = @intFromFloat(values[2 + i * 2]);
        expected[index] = count;
    }
    try std.testing.expectEqualSlices(i32, &expected, histogram);
}

/// The cumulative cases list the same bins plus bin 255, so this checks those
/// exactly and the prefix-sum property everywhere else.
fn expectCumulative(corpus: *const corpus_mod.Corpus, name: []const u8, cumulative_histogram: *const [bins]i32) !void {
    const values = try corpus.get(name);
    const pairs: usize = @intFromFloat(values[0]);
    try std.testing.expectEqual(@as(usize, 1 + pairs * 2), values.len);

    for (0..pairs) |i| {
        const index: usize = @intFromFloat(values[1 + i * 2]);
        const want: i32 = @intFromFloat(values[2 + i * 2]);
        if (cumulative_histogram[index] != want) {
            std.debug.print("corpus case '{s}': bin {d}: C says {d}, port says {d}\n", .{
                name, index, want, cumulative_histogram[index],
            });
            return error.CumulativeMismatch;
        }
    }

    for (1..bins) |i| {
        try std.testing.expect(cumulative_histogram[i] >= cumulative_histogram[i - 1]);
    }
}

test "histogram: compute and cumulative match the C" {
    const corpus = try corpus_mod.Corpus.load();

    var histogram: [bins]i32 = undefined;
    try compute(&img, &histogram, 4, 4, 0);
    try expectHistogram(&corpus, "histogram_compute_small", &histogram);

    // Independently of the fixture: sixteen pixels in, sixteen counted.
    var total: i32 = 0;
    for (histogram) |count| total += count;
    try std.testing.expectEqual(@as(i32, 16), total);

    var cumulative_histogram: [bins]i32 = undefined;
    cumulative(&histogram, &cumulative_histogram);
    try expectCumulative(&corpus, "histogram_cumulative_small", &cumulative_histogram);
    try std.testing.expectEqual(@as(i32, 16), cumulative_histogram[bins - 1]);

    // A stride wider than the image: the padding is not counted.
    var widesrc = [_]u8{0xAA} ** 24;
    for (0..4) |y| @memcpy(widesrc[y * 6 ..][0..4], img[y * 4 ..][0..4]);
    var wide_histogram: [bins]i32 = undefined;
    try compute(&widesrc, &wide_histogram, 4, 4, 6);
    try expectHistogram(&corpus, "histogram_compute_stride", &wide_histogram);
    try std.testing.expectEqualSlices(i32, &histogram, &wide_histogram);

    // Gaps: the cumulative is constant between populated bins, which the pair list
    // alone cannot show.
    var sparse = [_]i32{0} ** bins;
    sparse[0] = 3;
    sparse[5] = 4;
    sparse[255] = 1;
    cumulative(&sparse, &cumulative_histogram);
    try expectCumulative(&corpus, "histogram_cumulative_sparse_bins", &cumulative_histogram);

    for (1..bins) |i| {
        const expected: i32 = if (i >= 255) 8 else if (i >= 5) 7 else 3;
        try std.testing.expectEqual(expected, cumulative_histogram[i]);
    }
    // The C ran the same check on itself and recorded the answer.
    try corpus.expectInt("histogram_cumulative_sparse_flat", 1);
}

test "histogram: equalisation matches the C, degenerate case included" {
    const corpus = try corpus_mod.Corpus.load();

    var dst = [_]u8{0xEE} ** 16;
    try equalize(&img, &dst, 4, 4, 0);
    try expectBytes(&corpus, "histogram_equalize_small", &dst);

    // A single-valued image: the lookup table is 0 below the value and 255 from it
    // up, so everything saturates.
    const flat = [_]u8{100} ** 16;
    dst = [_]u8{0xEE} ** 16;
    try equalize(&flat, &dst, 4, 4, 0);
    try expectBytes(&corpus, "histogram_equalize_uniform", &dst);
    for (dst) |byte| try std.testing.expectEqual(@as(u8, 255), byte);

    // The mapping is monotone: a brighter pixel never comes out darker.
    dst = [_]u8{0} ** 16;
    try equalize(&img, &dst, 4, 4, 0);
    for (img, dst) |before, after| _ = .{ before, after };
    for (0..16) |i| {
        for (0..16) |j| {
            if (img[i] < img[j]) try std.testing.expect(dst[i] <= dst[j]);
        }
    }
}

test "histogram: CLAHE matches the C on the tile grids the C survives" {
    const corpus = try corpus_mod.Corpus.load();

    var histograms: [4 * bins]i32 = undefined;
    var luts: [4 * bins]u8 = undefined;
    try std.testing.expectEqual(@as(usize, 4 * bins), claheTablesLen(4, 4, 2));

    var dst = [_]u8{0xEE} ** 16;
    try clahe(&img, &dst, &histograms, &luts, 4, 4, 2, 0.0, 0);
    try expectBytes(&corpus, "histogram_clahe_4x4_tile2", &dst);

    // A clip limit on a four-pixel tile clips every bin to zero and the remainder
    // of the redistribution division is dropped, so the image comes out black.
    dst = [_]u8{0xEE} ** 16;
    try clahe(&img, &dst, &histograms, &luts, 4, 4, 2, 1.5, 0);
    try expectBytes(&corpus, "histogram_clahe_4x4_tile2_clip", &dst);
    for (dst) |byte| try std.testing.expectEqual(@as(u8, 0), byte);

    // Uneven tiles: 5 wide with a tile size of 2 is three tiles of width 1, so
    // columns 3 and 4 are never histogrammed but are still interpolated.
    var big: [25]u8 = undefined;
    for (0..25) |i| big[i] = @intCast((i * 37) % 256);

    var uneven_histograms: [9 * bins]i32 = undefined;
    var uneven_luts: [9 * bins]u8 = undefined;
    try std.testing.expectEqual(@as(usize, 9 * bins), claheTablesLen(5, 5, 2));

    var uneven_dst = [_]u8{0xEE} ** 25;
    try clahe(&big, &uneven_dst, &uneven_histograms, &uneven_luts, 5, 5, 2, 0.0, 0);
    try expectBytes(&corpus, "histogram_clahe_5x5_tile2_uneven", &uneven_dst);

    // The redistribution after clipping. The four-pixel tile above cannot reach it
    // (`redistribution / 256` is zero there, so the step does nothing), and neither
    // can a large tile with a spread-out histogram: this one is 400 pixels
    // concentrated in four bins, which leaves 392 votes to spread, one per bin.
    var wide: [1600]u8 = undefined;
    const levels = [4]u8{ 0, 64, 128, 192 };
    for (0..1600) |i| wide[i] = levels[i % 4];

    var wide_histograms: [4 * bins]i32 = undefined;
    var wide_luts: [4 * bins]u8 = undefined;
    var wide_dst = [_]u8{0xEE} ** 1600;
    try clahe(&wide, &wide_dst, &wide_histograms, &wide_luts, 40, 40, 20, 1.0, 0);
    try expectBytes(&corpus, "histogram_clahe_clip_redistribution", wide_dst[0..8]);
}

test "histogram: one tile is handled, where the C reads before its table" {
    // the archived probe_clahe_single_tile.c: a 6x4 image with a tile size of 6 dies with an
    // access violation, because the interpolation computes `tile_count - 2` for
    // the y axis, which is -1. There is no behaviour to reproduce, so the port
    // uses the tile it has.
    var histograms: [bins]i32 = undefined;
    var luts: [bins]u8 = undefined;
    var dst = [_]u8{0xEE} ** 16;

    try clahe(&img, &dst, &histograms, &luts, 4, 4, 4, 0.0, 0);

    // One tile covering everything, with no clipping, *is* the global
    // equalisation - which is an equality the corpus cannot state, since it has no
    // C answer for this input.
    var reference = [_]u8{0} ** 16;
    try equalize(&img, &reference, 4, 4, 0);
    try std.testing.expectEqualSlices(u8, &reference, &dst);

    // A tile size above the image is clamped down to it, so this is the same call.
    var dst_again = [_]u8{0} ** 16;
    try clahe(&img, &dst_again, &histograms, &luts, 4, 4, 9, 0.0, 0);
    try std.testing.expectEqualSlices(u8, &dst, &dst_again);
}

test "histogram: the buffers and arguments are checked" {
    var histogram: [bins]i32 = undefined;
    var dst = [_]u8{0} ** 16;
    var histograms: [bins]i32 = undefined;
    var luts: [bins]u8 = undefined;

    const small_src = [_]u8{0} ** 15;
    try std.testing.expectError(Error.BufferTooSmall, compute(&small_src, &histogram, 4, 4, 0));
    try std.testing.expectError(Error.InvalidSize, compute(&img, &histogram, 0, 4, 0));

    var small_dst: [15]u8 = undefined;
    try std.testing.expectError(Error.BufferTooSmall, equalize(&img, &small_dst, 4, 4, 0));

    // Two tables of 256 for a 2x2 grid of tiles, so one is too few.
    try std.testing.expectError(
        Error.BufferTooSmall,
        clahe(&img, &dst, &histograms, &luts, 4, 4, 2, 0.0, 0),
    );
    try std.testing.expectError(
        Error.InvalidSize,
        clahe(&img, &dst, &histograms, &luts, 4, 4, 0, 0.0, 0),
    );
}
