//! Shared pieces of the image stage.
//!
//! The C functions check their pointers and their dimensions and nothing else:
//! whether `dst` is large enough for `height` rows of `stride` bytes is the
//! caller's problem. Here the region is checked against the slices, so an
//! undersized buffer is `error.BufferTooSmall` rather than a write past the end.
//!
//! That check is the one systematic change this stage makes to every function,
//! and it matters more here than anywhere else in the library: these are the
//! functions that walk memory the caller sized.

const std = @import("std");

pub const Error = error{
    /// Width or height is zero.
    InvalidSize,
    /// The slice is smaller than `width` x `height` at this stride.
    BufferTooSmall,
};

/// The C rule: a `stride_bytes` of 0 means "rows are contiguous".
pub fn strideOf(width: usize, stride_bytes: usize) usize {
    return if (stride_bytes > 0) stride_bytes else width;
}

/// How many bytes a `width` x `height` region touches at this stride.
pub fn regionLen(width: usize, height: usize, stride: usize) usize {
    if (width == 0 or height == 0) return 0;
    return (height - 1) * stride + width;
}

pub fn checkRegion(len: usize, width: usize, height: usize, stride: usize) Error!void {
    if (width == 0 or height == 0) return Error.InvalidSize;
    if (regionLen(width, height, stride) > len) return Error.BufferTooSmall;
}

/// The stricter check the morphology functions need.
///
/// They clear their destination with `memset(dst, 0, stride * height)` - the whole
/// buffer, padding included - which is the one place in this stage where the
/// padding is deliberately written. A buffer that fits the image region but not
/// the final row's padding is therefore still too small, and the corpus records
/// the zeros that appear in those bytes.
pub fn checkFullBuffer(len: usize, width: usize, height: usize, stride: usize) Error!void {
    if (width == 0 or height == 0) return Error.InvalidSize;
    if (stride * height > len) return Error.BufferTooSmall;
}

test "region arithmetic" {
    try std.testing.expectEqual(@as(usize, 4), strideOf(4, 0));
    try std.testing.expectEqual(@as(usize, 5), strideOf(4, 5));
    try std.testing.expectEqual(@as(usize, 12), regionLen(4, 3, 4));
    try std.testing.expectEqual(@as(usize, 14), regionLen(4, 3, 5));

    // A one-row image needs only its width.
    try std.testing.expectEqual(@as(usize, 4), regionLen(4, 1, 5));

    const buf: [11]u8 = undefined;
    try std.testing.expectError(Error.BufferTooSmall, checkRegion(buf.len, 4, 3, 4));
    try std.testing.expectError(Error.InvalidSize, checkRegion(buf.len, 0, 3, 4));

    const exact: [12]u8 = undefined;
    try checkRegion(exact.len, 4, 3, 4);
}

test "the full-buffer check is stricter than the region check" {
    // 4 wide at stride 5, 3 rows: the region ends at 2 * 5 + 4 = 14, but the
    // buffer a morphology function writes is 5 * 3 = 15 bytes. A 14-byte buffer
    // passes the region check and must fail this one.
    const fourteen: [14]u8 = undefined;
    try checkRegion(fourteen.len, 4, 3, 5);
    try std.testing.expectError(Error.BufferTooSmall, checkFullBuffer(fourteen.len, 4, 3, 5));

    const fifteen: [15]u8 = undefined;
    try checkFullBuffer(fifteen.len, 4, 3, 5);

    // With no padding the two agree.
    try checkFullBuffer(fifteen.len - 3, 4, 3, 4);
    try std.testing.expectError(Error.InvalidSize, checkFullBuffer(fifteen.len, 0, 3, 5));
}
