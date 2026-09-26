//! Gaussian blur, ported from `include/breeze/image/gaussian_blur.h`.
//!
//! Four functions and one shape change. The C allocates two buffers inside
//! `BreezeGaussianBlur` - the kernel and a `stride * height` temporary - and both
//! become caller-provided slices, so nothing here allocates either.
//!
//! Two things the C hides in arithmetic are made explicit:
//!
//!   * The kernel size is a **comptime** parameter in `blur`, and an even one is
//!     a compile error. The C signals the same thing with a return value of 0,
//!     which is only reachable through the public `Kernel1D` function because
//!     `BreezeGaussianBlur` bumps even sizes itself. That runtime path is still
//!     here as `blurSized`, which keeps the C's rule exactly.
//!   * An even kernel slice passed to the 1-D filters is `error.EvenKernelSize`.
//!     The C reads one float past the end of the kernel for such a size - the
//!     loop runs `-size/2 .. +size/2`, which is `size + 1` taps - and
//!     the archived tools/corpus/probe_gaussian.c shows that read by changing the
//!     float that lands there. There is nothing to reproduce faithfully, so the
//!     port refuses instead (docs/REVIEW.md §44).

const std = @import("std");

const common = @import("common.zig");
const corpus_mod = @import("../math/corpus.zig");

pub const Error = common.Error || error{
    /// `sigma <= 0`. The C returns without touching the destination.
    NonPositiveSigma,
    /// A kernel slice whose length is even.
    EvenKernelSize,
    /// A sigma so large that the C's own `(int)` cast would be undefined.
    KernelTooLarge,
};

/// `kernel[i] = exp(-x^2 / (2 sigma^2))`, normalized so the taps sum to 1.
///
/// The slice's length is the kernel size, and it must be odd: the C's
/// `!(size % 2)` refusal is a compile-time property here.
pub fn gaussianKernel(kernel: []f32, sigma: f32) Error!void {
    if (kernel.len == 0) return Error.InvalidSize;
    if (kernel.len % 2 == 0) return Error.EvenKernelSize;
    if (!(sigma > 0)) return Error.NonPositiveSigma;

    const half = kernel.len / 2;
    var sum: f32 = 0;
    for (0..kernel.len) |i| {
        const x: f32 = @floatFromInt(@as(i32, @intCast(i)) - @as(i32, @intCast(half)));
        kernel[i] = @exp(-(x * x) / (2.0 * sigma * sigma));
        sum += kernel[i];
    }

    // The C guards the division with `if (sum != 0)`. It cannot be zero: the
    // centre tap is `exp(0) = 1` for every size the loop reaches, so `sum >= 1`.
    // The guard is dropped rather than copied (docs/REVIEW.md §44).
    for (kernel) |*tap| tap.* /= sum;
}

/// The C's rule for `kernel_size <= 0`: `6 * sigma` rounded, odd, at least 3.
pub fn autoKernelSize(sigma: f32) Error!usize {
    if (!(sigma > 0)) return Error.NonPositiveSigma;

    // The C does `(int)(sigma * 6.0f + 0.5f)`, which is undefined once the result
    // leaves `int`. The port refuses that range instead of truncating something
    // the C never defined.
    const scaled = sigma * 6.0 + 0.5;
    if (!(scaled < 2147483648.0)) return Error.KernelTooLarge;

    var size: usize = @intFromFloat(scaled);
    if (size % 2 == 0) size += 1;
    if (size < 3) size = 3;
    return size;
}

/// `(int)(sum + 0.5)` clamped to a byte.
///
/// The C's cast is `(int)` followed by two clamps, so an out-of-range sum is
/// defined there; here the clamps happen first, in float, and the conversion only
/// ever sees a value it can represent. A non-finite sum is undefined in C and a
/// safety-check panic here.
inline fn toByte(sum: f32) u8 {
    const rounded = sum + 0.5;
    if (rounded >= 255.0) return 255;
    if (rounded <= 0.0) return 0;
    return @intFromFloat(rounded);
}

/// The weighted sum of one pixel's neighbourhood, with the C's edge clamping.
///
/// `along_y` picks which axis the kernel runs along; the two C functions are
/// otherwise identical, and their accumulation order is i ascending, which is
/// what the float sums depend on.
inline fn weightedSum(
    comptime along_y: bool,
    src: []const u8,
    stride: usize,
    x: usize,
    y: usize,
    width: usize,
    height: usize,
    kernel: []const f32,
) f32 {
    const half = kernel.len / 2;
    const limit: isize = @intCast(if (along_y) height else width);
    const fixed: isize = @intCast(if (along_y) x else y);
    const start: isize = @intCast(if (along_y) y else x);

    var sum: f32 = 0;
    for (kernel, 0..) |tap, i| {
        // The C walks i from -half to +half and indexes kernel[i + half], which is
        // this loop's i. Walk the same way so the additions happen in the same
        // order.
        var sample = start + @as(isize, @intCast(i)) - @as(isize, @intCast(half));
        if (sample < 0) sample = 0;
        if (sample >= limit) sample = limit - 1;

        const index: usize = if (along_y)
            @as(usize, @intCast(sample)) * stride + @as(usize, @intCast(fixed))
        else
            @as(usize, @intCast(fixed)) * stride + @as(usize, @intCast(sample));
        sum += @as(f32, @floatFromInt(src[index])) * tap;
    }
    return sum;
}

fn checkKernel(kernel: []const f32) Error!void {
    if (kernel.len == 0) return Error.InvalidSize;
    if (kernel.len % 2 == 0) return Error.EvenKernelSize;
}

/// One horizontal pass, `src` to `temp`.
pub fn horizontal(
    src: []const u8,
    temp: []u8,
    width: usize,
    height: usize,
    kernel: []const f32,
    stride_bytes: usize,
) Error!void {
    const stride = common.strideOf(width, stride_bytes);
    try checkKernel(kernel);
    try common.checkRegion(src.len, width, height, stride);
    try common.checkRegion(temp.len, width, height, stride);

    for (0..height) |y| {
        for (0..width) |x| {
            temp[y * stride + x] = toByte(weightedSum(false, src, stride, x, y, width, height, kernel));
        }
    }
}

/// One vertical pass, `temp` to `dst`.
pub fn vertical(
    temp: []const u8,
    dst: []u8,
    width: usize,
    height: usize,
    kernel: []const f32,
    stride_bytes: usize,
) Error!void {
    const stride = common.strideOf(width, stride_bytes);
    try checkKernel(kernel);
    try common.checkRegion(temp.len, width, height, stride);
    try common.checkRegion(dst.len, width, height, stride);

    for (0..height) |y| {
        for (0..width) |x| {
            dst[y * stride + x] = toByte(weightedSum(true, temp, stride, x, y, width, height, kernel));
        }
    }
}

/// The C's `BreezeGaussianBlur`, with its two allocations supplied by the caller.
///
/// `kernel_size_arg` follows the C: zero or negative means "derive it from
/// sigma", and an even value is bumped to the next odd one. `kernel_scratch` must
/// have room for the size that comes out of that.
pub fn blurSized(
    src: []const u8,
    dst: []u8,
    temp: []u8,
    kernel_scratch: []f32,
    width: usize,
    height: usize,
    sigma: f32,
    kernel_size_arg: i32,
    stride_bytes: usize,
) Error!void {
    const stride = common.strideOf(width, stride_bytes);

    // The C checks all of this, and returns with `dst` untouched, before it
    // allocates anything.
    try common.checkRegion(src.len, width, height, stride);
    try common.checkRegion(dst.len, width, height, stride);
    try common.checkRegion(temp.len, width, height, stride);
    if (!(sigma > 0)) return Error.NonPositiveSigma;

    var size: usize = undefined;
    if (kernel_size_arg <= 0) {
        size = try autoKernelSize(sigma);
    } else {
        size = @intCast(kernel_size_arg);
        if (size % 2 == 0) size += 1;
    }
    if (kernel_scratch.len < size) return Error.BufferTooSmall;

    const kernel = kernel_scratch[0..size];
    try gaussianKernel(kernel, sigma);
    try horizontal(src, temp, width, height, kernel, stride_bytes);
    try vertical(temp, dst, width, height, kernel, stride_bytes);
}

/// The same blur with the kernel size known at compile time and checked there.
///
/// The C's `malloc(kernel_size * sizeof(float))` becomes a local array, so a
/// firmware using a fixed kernel allocates nothing at all - and an even size is
/// rejected by the compiler instead of bumped at run time.
pub fn blur(
    comptime kernel_size: usize,
    src: []const u8,
    dst: []u8,
    temp: []u8,
    width: usize,
    height: usize,
    sigma: f32,
    stride_bytes: usize,
) Error!void {
    comptime if (kernel_size == 0 or kernel_size % 2 == 0)
        @compileError("the Gaussian kernel size must be odd and positive; use blurSized for the C's bump-even-sizes rule");

    var kernel: [kernel_size]f32 = undefined;
    return blurSized(
        src,
        dst,
        temp,
        &kernel,
        width,
        height,
        sigma,
        @intCast(kernel_size),
        stride_bytes,
    );
}

// --- tests ------------------------------------------------------------------

/// A single bright pixel, which is what makes a blur's weights legible: every
/// output pixel is a kernel weight times 200.
const dot = [16]u8{
    0, 0, 0,   0,
    0, 0, 0,   0,
    0, 0, 200, 0,
    0, 0, 0,   0,
};

fn expectBytes(corpus: *const corpus_mod.Corpus, name: []const u8, bytes: []const u8) !void {
    var as_floats: [64]f32 = undefined;
    for (bytes, 0..) |b, i| as_floats[i] = @floatFromInt(b);
    try corpus.expectValues(name, as_floats[0..bytes.len]);
}

test "gaussian kernel: the taps match the C, and the refusals become types" {
    const corpus = try corpus_mod.Corpus.load();

    var k3: [3]f32 = undefined;
    try gaussianKernel(&k3, 1.0);
    try corpus.expectValues("gaussian_kernel_3_sigma_1", &k3);

    var k5: [5]f32 = undefined;
    try gaussianKernel(&k5, 1.0);
    try corpus.expectValues("gaussian_kernel_5_sigma_1", &k5);

    try gaussianKernel(&k5, 2.0);
    try corpus.expectValues("gaussian_kernel_5_sigma_2", &k5);

    // Every tap is positive and the taps sum to 1 - which the corpus alone cannot
    // say, because a kernel scaled by a constant would still be "close" nowhere,
    // but a *shifted* one could match the recorded digits of a symmetric kernel.
    for (k5) |tap| try std.testing.expect(tap > 0);
    var total: f32 = 0;
    for (k5) |tap| total += tap;
    try std.testing.expectApproxEqAbs(@as(f32, 1.0), total, 1.0e-6);

    // The corpus records the C returning 1 for the success case and 0 for each
    // refusal; those inputs are exactly the ones that are an error here.
    try corpus.expectInt("gaussian_kernel_ok", 1);
    try corpus.expectInt("gaussian_kernel_even_refused", 0);
    try corpus.expectInt("gaussian_kernel_zero_size_refused", 0);
    try corpus.expectInt("gaussian_kernel_negative_sigma_refused", 0);
    try corpus.expectInt("gaussian_kernel_sigma_zero_refused", 0);

    var even: [4]f32 = undefined;
    try std.testing.expectError(Error.EvenKernelSize, gaussianKernel(&even, 1.0));
    try std.testing.expectError(Error.NonPositiveSigma, gaussianKernel(&k3, 0.0));
    try std.testing.expectError(Error.NonPositiveSigma, gaussianKernel(&k3, -1.0));
    try std.testing.expectError(Error.InvalidSize, gaussianKernel(&[_]f32{}, 1.0));
}

test "gaussian: the two passes match the C" {
    const corpus = try corpus_mod.Corpus.load();

    var k3: [3]f32 = undefined;
    try gaussianKernel(&k3, 1.0);

    var temp = [_]u8{0xEE} ** 16;
    try horizontal(&dot, &temp, 4, 4, &k3, 0);
    try expectBytes(&corpus, "gaussian_horizontal_3", &temp);

    var out = [_]u8{0xEE} ** 16;
    try vertical(&temp, &out, 4, 4, &k3, 0);
    try expectBytes(&corpus, "gaussian_vertical_3", &out);

    // A 2-D Gaussian is separable, so the outer product of the taps is what the
    // two passes must produce on a delta image: out = 200 * k[y] * k[x].
    for (0..3) |y| {
        for (0..3) |x| {
            const got = out[(y + 1) * 4 + (x + 1)];
            const want: f32 = 200.0 * k3[y] * k3[x];
            try std.testing.expectApproxEqAbs(want, @as(f32, @floatFromInt(got)), 1.0);
        }
    }
}

test "gaussian blur: the auto size rule matches the C's outputs" {
    const corpus = try corpus_mod.Corpus.load();

    var temp = [_]u8{0} ** 16;
    var out = [_]u8{0xEE} ** 16;
    var scratch: [31]f32 = undefined;

    // kernel_size 0 runs the C's rule; 7 is what it should produce for sigma 1
    // ((int)(6.5) = 6, bumped to odd). The two must agree byte for byte.
    try blurSized(&dot, &out, &temp, &scratch, 4, 4, 1.0, 0, 0);
    try expectBytes(&corpus, "gaussian_blur_auto_sigma_1", &out);
    try std.testing.expectEqual(@as(usize, 7), try autoKernelSize(1.0));

    var explicit = [_]u8{0xEE} ** 16;
    try blurSized(&dot, &explicit, &temp, &scratch, 4, 4, 1.0, 7, 0);
    try expectBytes(&corpus, "gaussian_blur_explicit_7", &explicit);
    try std.testing.expectEqualSlices(u8, &explicit, &out);

    // An even size is bumped, and 6 would be one tap short of the same kernel.
    try blurSized(&dot, &out, &temp, &scratch, 4, 4, 1.0, 6, 0);
    try expectBytes(&corpus, "gaussian_blur_even_6_becomes_7", &out);
    try std.testing.expectEqualSlices(u8, &explicit, &out);

    // A one-tap kernel is the identity.
    try blurSized(&dot, &out, &temp, &scratch, 4, 4, 1.0, 1, 0);
    try expectBytes(&corpus, "gaussian_blur_kernel_1_identity", &out);
    try std.testing.expectEqualSlices(u8, &dot, &out);

    // A narrow sigma is invisible in 8 bits: the corpus records this as the
    // identity, and the archived probe shows that no sigma reaching the `< 3` floor
    // can look any different. So these two cases do not pin the floor.
    try blurSized(&dot, &out, &temp, &scratch, 4, 4, 0.2, 0, 0);
    try expectBytes(&corpus, "gaussian_blur_auto_narrow_sigma_0p2", &out);
    try blurSized(&dot, &out, &temp, &scratch, 4, 4, 0.2, 3, 0);
    try expectBytes(&corpus, "gaussian_blur_explicit_3_sigma_0p2", &out);
    try std.testing.expectEqual(@as(usize, 3), try autoKernelSize(0.2));
}

test "gaussian blur: a non-positive sigma is refused without touching anything" {
    const corpus = try corpus_mod.Corpus.load();

    var temp = [_]u8{0} ** 16;
    var out = [_]u8{0xEE} ** 16;
    var scratch: [31]f32 = undefined;

    try std.testing.expectError(
        Error.NonPositiveSigma,
        blurSized(&dot, &out, &temp, &scratch, 4, 4, 0.0, 3, 0),
    );

    // The C returns with `dst` untouched, and the corpus records the 0xEE that
    // was there. The port must leave it alone too.
    try std.testing.expectEqualSlices(u8, &[_]u8{0xEE} ** 16, &out);
    const untouched = [_]f32{238} ** 16;
    try corpus.expectValues("gaussian_blur_sigma_zero_untouched", &untouched);
}

test "gaussian blur: the rounding rule is round-half-up, not truncation" {
    const corpus = try corpus_mod.Corpus.load();

    // `[0, 0.5, 0.5]` puts the horizontal sum on exactly 0.5 for every pixel of a
    // checkerboard row, where `(int)(sum + 0.5)` gives 1 and truncation gives 0.
    const checker = [16]u8{
        0, 1, 0, 1,
        0, 1, 0, 1,
        0, 1, 0, 1,
        0, 1, 0, 1,
    };
    const half_kernel = [3]f32{ 0.0, 0.5, 0.5 };

    var out = [_]u8{0xEE} ** 16;
    try horizontal(&checker, &out, 4, 4, &half_kernel, 0);
    try expectBytes(&corpus, "gaussian_rounding_half", &out);
    for (out) |byte| try std.testing.expectEqual(@as(u8, 1), byte);
}

test "gaussian blur: a stride wider than the image keeps to its own bytes" {
    const corpus = try corpus_mod.Corpus.load();

    var widesrc = [_]u8{0xAA} ** 24;
    for (0..4) |y| @memcpy(widesrc[y * 6 ..][0..4], dot[y * 4 ..][0..4]);

    var wideout = [_]u8{0xEE} ** 24;
    var widetemp = [_]u8{0x00} ** 24;
    var scratch: [31]f32 = undefined;
    try blurSized(&widesrc, &wideout, &widetemp, &scratch, 4, 4, 1.0, 3, 6);
    try expectBytes(&corpus, "gaussian_blur_stride", &wideout);

    for (0..4) |y| {
        try std.testing.expectEqual(@as(u8, 0xEE), wideout[y * 6 + 4]);
        try std.testing.expectEqual(@as(u8, 0xEE), wideout[y * 6 + 5]);
    }
}

test "gaussian blur: the byte clamp is reached, and saturates at 255" {
    const corpus = try corpus_mod.Corpus.load();

    // Nothing else in the corpus gets near 255 - the delta image's brightest pixel
    // is 200 - so without these two, a clamp that saturates at 253 (or at whatever
    // value no image ever reaches) passes every other test. A mutation probe that
    // moved it to 253 stayed green until these cases existed.
    const bright = [_]u8{255} ** 16;

    var temp = [_]u8{0} ** 16;
    var out = [_]u8{0xEE} ** 16;
    var scratch: [31]f32 = undefined;

    // A saturated image through a real kernel: every sum is 255.
    try blurSized(&bright, &out, &temp, &scratch, 4, 4, 1.0, 7, 0);
    try expectBytes(&corpus, "gaussian_blur_saturated", &out);

    // `[1, 1, 1]` is not normalized, so the sum is 765 and only the clamp can
    // bring it back into a byte.
    var unit_kernel = [_]f32{ 1.0, 1.0, 1.0 };
    try horizontal(&bright, &out, 4, 4, &unit_kernel, 0);
    try expectBytes(&corpus, "gaussian_rounding_over_255", &out);
    for (out) |byte| try std.testing.expectEqual(@as(u8, 255), byte);
}

test "gaussian: an even kernel is refused where the C reads past the end" {
    // The C's 1-D filters loop `-size/2 .. +size/2`, which is one tap more than an
    // even-sized kernel holds; the archived probe changes the float that lands in
    // that slot and the output changes with it.
    var even_kernel = [_]f32{ 0.0, 0.5, 0.5, 0.0 };
    var out: [16]u8 = undefined;
    try std.testing.expectError(
        Error.EvenKernelSize,
        horizontal(&dot, &out, 4, 4, &even_kernel, 0),
    );
    try std.testing.expectError(
        Error.EvenKernelSize,
        vertical(&dot, &out, 4, 4, &even_kernel, 0),
    );

    // And the buffers are checked before anything is read or written.
    var small: [15]u8 = undefined;
    var k3: [3]f32 = undefined;
    try gaussianKernel(&k3, 1.0);
    try std.testing.expectError(Error.BufferTooSmall, horizontal(&dot, &small, 4, 4, &k3, 0));
    try std.testing.expectError(Error.InvalidSize, horizontal(&dot, &out, 0, 4, &k3, 0));
    try std.testing.expectError(Error.InvalidSize, horizontal(&dot, &out, 4, 4, &[_]f32{}, 0));

    var temp: [16]u8 = undefined;
    var tiny_scratch: [2]f32 = undefined;
    try std.testing.expectError(
        Error.BufferTooSmall,
        blurSized(&dot, &out, &temp, &tiny_scratch, 4, 4, 1.0, 0, 0),
    );

    // The compile-time form is a compile-time refusal, which is the point of
    // having it: this line would not build.
    //   _ = blur(4, &dot, &out, &temp, 4, 4, 1.0, 0);
    _ = blur;
}
