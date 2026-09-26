//! The image stage: 8-bit grayscale processing ported from
//! `include/breeze/image/`.
//!
//! In progress - the two allocation-free threshold headers are done; the
//! filters, morphology, Canny, Hough and histogram are not yet.
//!
//! This is the first stage whose functions do not own their memory. The C
//! versions take a source and a destination and use `width`, `height` and
//! `stride_bytes` to walk them, checking the pointers but never the sizes;
//! `common.zig` adds the size check back, and everything here returns
//! `common.Error` instead of silently returning. Where C returns 0 for both
//! "the answer is 0" and "your arguments were nonsense", the error union is the
//! only thing that tells those apart.
//!
//! The remaining headers allocate scratch buffers through the caller's allocator;
//! those will take their scratch as a caller-provided slice, the way the filter
//! stage does, so nothing in this stage allocates.

const std = @import("std");

pub const common = @import("common.zig");
pub const binary = @import("binary.zig");
pub const otsu = @import("otsu.zig");

pub const Error = common.Error;
pub const threshold = binary.threshold;
pub const inverseThreshold = binary.inverseThreshold;
pub const otsuThreshold = otsu.otsu;
pub const applyOtsu = otsu.applyOtsu;

test {
    std.testing.refAllDecls(@This());
}
