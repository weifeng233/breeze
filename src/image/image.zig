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
pub const sobel = @import("sobel.zig");
pub const gaussian = @import("gaussian.zig");
pub const morphology = @import("morphology.zig");
pub const canny = @import("canny.zig");
pub const hough = @import("hough.zig");
pub const histogram = @import("histogram.zig");

pub const Error = common.Error;
pub const threshold = binary.threshold;
pub const inverseThreshold = binary.inverseThreshold;
pub const otsuThreshold = otsu.otsu;
pub const applyOtsu = otsu.applyOtsu;
pub const sobelOperator = sobel.sobel;
pub const sobelOperatorWithDirection = sobel.sobelWithDirection;
pub const sobelOperatorThreshold = sobel.sobelThreshold;
pub const gaussianKernel = gaussian.gaussianKernel;
pub const gaussianBlur = gaussian.blur;
pub const gaussianBlurSized = gaussian.blurSized;
pub const autoGaussianKernelSize = gaussian.autoKernelSize;
pub const Shape = morphology.Shape;
pub const createStructureElement = morphology.createKernel;
pub const dilate = morphology.dilate;
pub const erode = morphology.erode;
pub const open = morphology.open;
pub const close = morphology.close;
pub const gradient = morphology.gradient;
pub const cannyGradient = canny.gradient;
pub const cannyNonMaxSuppression = canny.nonMaxSuppression;
pub const cannyHysteresis = canny.hysteresis;
pub const cannyEdgeDetection = canny.edgeDetection;
pub const HoughLine = hough.Line;
pub const HoughCircle = hough.Circle;
pub const houghLines = hough.houghLines;
pub const drawHoughLine = hough.drawLine;
pub const houghCircles = hough.houghCircles;
pub const histogramCompute = histogram.compute;
pub const histogramCumulative = histogram.cumulative;
pub const histogramEqualize = histogram.equalize;
pub const histogramEqualizeClahe = histogram.clahe;

test {
    std.testing.refAllDecls(@This());
}
