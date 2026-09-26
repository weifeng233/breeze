//! Median filters, ported from `include/breeze/filter/median_filter.h`.
//!
//! Two forms, and the difference between them is storage:
//!
//! * `Median(window)` is the 1D running median. The C version takes two caller
//!   buffers (`buffer` and `sorted`) because that is how C spells "the caller
//!   decides where this lives"; here both arrays are fields of the value, so the
//!   window is a comptime parameter and there is no second pointer to keep in
//!   step with the first. The C's `Update` with a NULL buffer passes the input
//!   through; that state is unrepresentable here.
//! * `filterImage` takes its scratch as a slice. The only `malloc` in the whole
//!   filter stage is the C image version's window, and the port replaces it with
//!   a caller-provided buffer - the sharpest form of "do not force a heap". A
//!   caller who wants the old behaviour allocates `k * k` bytes and passes them.
//!
//! The image function's boundary rule is replicated edges (the C clamps `nx`/`ny`
//! into range rather than skipping or zeroing), an even kernel size is bumped up
//! to the next odd one, and only `dst[y * stride + x]` is written - the padding
//! columns of a strided destination are left exactly as they were, which the
//! corpus pins by pre-filling them with 0xEE.

const std = @import("std");

const corpus_mod = @import("../math/corpus.zig");

pub const Error = error{
    /// The scratch buffer is smaller than `kernel_size * kernel_size`.
    ScratchTooSmall,
    /// Width, height or kernel size is not positive.
    InvalidSize,
};

/// The sort the C header exposes and uses internally. Kept as a public function
/// because it is public there, and because the corpus checks it directly.
pub fn insertionSort(values: []f32) void {
    var i: usize = 1;
    while (i < values.len) : (i += 1) {
        const key = values[i];
        var j = i;
        while (j > 0 and values[j - 1] > key) : (j -= 1) {
            values[j] = values[j - 1];
        }
        values[j] = key;
    }
}

/// A running median over the last `window` samples.
///
/// `window` is a comptime parameter; the two arrays live in the value, so the
/// default-constructed `Median(5){}` is the C's freshly-initialised filter.
pub fn Median(comptime window: usize) type {
    if (window == 0) @compileError("a median filter needs a non-empty window");

    return struct {
        buffer: [window]f32 = @splat(0),
        sorted: [window]f32 = @splat(0),
        index: usize = 0,
        count: usize = 0,

        const Self = @This();

        pub fn init() Self {
            return .{};
        }

        /// Feed one sample, get the median of what has been seen so far.
        ///
        /// Before the window is full the median is over fewer samples, and an
        /// even count averages the two middle values - both as in the C version.
        pub fn update(self: *Self, input: f32) f32 {
            self.buffer[self.index] = input;
            self.index = (self.index + 1) % window;
            if (self.count < window) self.count += 1;

            @memcpy(self.sorted[0..self.count], self.buffer[0..self.count]);
            insertionSort(self.sorted[0..self.count]);

            if (self.count % 2 == 0) {
                return (self.sorted[self.count / 2 - 1] + self.sorted[self.count / 2]) / 2.0;
            }
            return self.sorted[self.count / 2];
        }

        pub fn reset(self: *Self) void {
            self.buffer = @splat(0);
            self.sorted = @splat(0);
            self.index = 0;
            self.count = 0;
        }
    };
}

/// Median filter an 8-bit image, using `scratch` as the window.
///
/// `scratch` must be at least `kernel_size * kernel_size` bytes (after an even
/// kernel size is rounded up); it is neither kept nor freed.
pub fn filterImage(
    src: []const u8,
    dst: []u8,
    width: usize,
    height: usize,
    kernel_size_in: usize,
    stride_bytes: usize,
    scratch: []u8,
) Error!void {
    if (width == 0 or height == 0 or kernel_size_in == 0) return Error.InvalidSize;

    // The C version bumps an even kernel up to the next odd one.
    const kernel = if (kernel_size_in % 2 == 0) kernel_size_in + 1 else kernel_size_in;
    if (scratch.len < kernel * kernel) return Error.ScratchTooSmall;

    const stride = if (stride_bytes > 0) stride_bytes else width;
    const half = kernel / 2;

    const w: isize = @intCast(width);
    const h: isize = @intCast(height);

    for (0..height) |y| {
        for (0..width) |x| {
            var n: usize = 0;
            var j: isize = -@as(isize, @intCast(half));
            while (j <= @as(isize, @intCast(half))) : (j += 1) {
                var i: isize = -@as(isize, @intCast(half));
                while (i <= @as(isize, @intCast(half))) : (i += 1) {
                    var nx: isize = @as(isize, @intCast(x)) + i;
                    var ny: isize = @as(isize, @intCast(y)) + j;

                    // Replicated edges, as in the C version.
                    if (nx < 0) nx = 0;
                    if (nx >= w) nx = w - 1;
                    if (ny < 0) ny = 0;
                    if (ny >= h) ny = h - 1;

                    scratch[n] = src[@as(usize, @intCast(ny)) * stride + @as(usize, @intCast(nx))];
                    n += 1;
                }
            }

            // Bubble sort, matching the C version's ordering exactly. The window
            // is at most a few dozen bytes, and this is the code being migrated.
            var a: usize = 0;
            while (a + 1 < n) : (a += 1) {
                var b: usize = 0;
                while (b + 1 < n - a) : (b += 1) {
                    if (scratch[b] > scratch[b + 1]) {
                        const t = scratch[b];
                        scratch[b] = scratch[b + 1];
                        scratch[b + 1] = t;
                    }
                }
            }

            dst[y * stride + x] = scratch[n / 2];
        }
    }
}

// --- tests ------------------------------------------------------------------

const impulses = [_]f32{ 1.0, 100.0, 2.0, 3.0, 2.0, 1.5, 2.2 };

test "median: a run matches the C answers" {
    const corpus = try corpus_mod.Corpus.load();

    var f = Median(5).init();
    var out: [impulses.len]f32 = undefined;
    for (impulses, 0..) |x, i| out[i] = f.update(x);
    try corpus.expectValues("median_run", &out);

    f.reset();
    try corpus.expectValue("median_reset_then_update", f.update(9.0));

    // Independent of the corpus: this is what a median filter is for. An impulse
    // in the middle of a steady signal never reaches the output.
    var g = Median(5).init();
    _ = g.update(10.0);
    _ = g.update(10.0);
    _ = g.update(10.0);
    const with_impulse = g.update(1000.0);
    try std.testing.expectEqual(@as(f32, 10.0), with_impulse);
}

test "median: an even window averages the two middle values" {
    const corpus = try corpus_mod.Corpus.load();

    var f = Median(4).init();
    var out: [6]f32 = undefined;
    for (impulses[0..6], 0..) |x, i| out[i] = f.update(x);
    try corpus.expectValues("median_even_window_run", &out);

    // Two samples: the answer is their mean, not one of them.
    var g = Median(4).init();
    _ = g.update(2.0);
    try std.testing.expectEqual(@as(f32, 6.0), g.update(10.0));
}

test "median: the exposed insertion sort matches the C answers" {
    const corpus = try corpus_mod.Corpus.load();

    var values = [_]f32{ 3.0, 1.0, 4.0, 1.5, 5.0, 2.0 };
    insertionSort(&values);
    try corpus.expectValues("median_insertion_sort", &values);

    // Degenerate inputs must not panic.
    var empty: [0]f32 = .{};
    insertionSort(&empty);
    var single = [_]f32{7.0};
    insertionSort(&single);
    try std.testing.expectEqual(@as(f32, 7.0), single[0]);
}

test "median image: matches the C answers" {
    const corpus = try corpus_mod.Corpus.load();

    const src = [_]u8{ 10, 20, 30, 40, 50, 200, 60, 70, 80, 90, 100, 110 };
    var scratch: [25]u8 = undefined;

    var dst = [_]u8{0} ** 12;
    try filterImage(&src, &dst, 4, 3, 3, 0, &scratch);
    const want_3x3 = try corpus.get("median_image_3x3");
    for (dst, 0..) |v, i| try std.testing.expectEqual(@as(f32, @floatFromInt(v)), want_3x3[i]);
    // What a median filter is for: the impulse of 200 never reaches the output.
    for (dst) |v| try std.testing.expect(v != 200);

    // An even kernel is rounded up to the next odd size, and produces different
    // output than the 3x3 case - so the rounding is observable, not cosmetic.
    var dst_even = [_]u8{0} ** 12;
    try filterImage(&src, &dst_even, 4, 3, 4, 0, &scratch);
    const want_even = try corpus.get("median_image_even_kernel");
    for (dst_even, 0..) |v, i| try std.testing.expectEqual(@as(f32, @floatFromInt(v)), want_even[i]);

    // A stride wider than the width: the padding columns are never written, so
    // the 0xEE they were pre-filled with is still there.
    const src_str = [_]u8{ 1, 2, 3, 99, 99, 4, 5, 6, 99, 99, 7, 8, 9, 99, 99 };
    var dst_str = [_]u8{0xEE} ** 15;
    try filterImage(&src_str, &dst_str, 3, 3, 3, 5, &scratch);
    const want_stride = try corpus.get("median_image_stride");
    for (dst_str, 0..) |v, i| try std.testing.expectEqual(@as(f32, @floatFromInt(v)), want_stride[i]);
    try std.testing.expectEqual(@as(u8, 0xEE), dst_str[3]);
    try std.testing.expectEqual(@as(u8, 0xEE), dst_str[4]);
}

test "median image: the scratch requirement is enforced" {
    const src = [_]u8{ 1, 2, 3, 4 };
    var dst = [_]u8{0} ** 4;
    var small: [8]u8 = undefined;
    var enough: [9]u8 = undefined;

    try std.testing.expectError(Error.ScratchTooSmall, filterImage(&src, &dst, 2, 2, 3, 0, &small));
    try filterImage(&src, &dst, 2, 2, 3, 0, &enough);

    // A 4x4 kernel is promoted to 5x5, so 16 bytes of scratch is not enough
    // even though the caller asked for a 4-wide kernel.
    try std.testing.expectError(Error.ScratchTooSmall, filterImage(&src, &dst, 2, 2, 4, 0, &small));

    try std.testing.expectError(Error.InvalidSize, filterImage(&src, &dst, 0, 2, 3, 0, &enough));
    try std.testing.expectError(Error.InvalidSize, filterImage(&src, &dst, 2, 2, 0, 0, &enough));
}
