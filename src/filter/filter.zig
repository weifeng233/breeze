//! The filter stage: stateful signal filters, ported from `include/breeze/filter/`.
//!
//! Unlike the math modules these hold state and are updated in place, so the
//! ports take `self: *Self`. The C null checks have no counterpart (there is no
//! pointer to be null), and the places where C silently did nothing - an invalid
//! time constant - report an error instead, with the C outcome still recorded in
//! the corpus.
//!
//! `common.zig` holds what the headers repeat: the alpha clamp, and the two
//! *opposite* ways they derive alpha from a time constant.

const std = @import("std");

pub const common = @import("common.zig");
pub const low_pass = @import("low_pass.zig");
pub const high_pass = @import("high_pass.zig");
pub const complementary = @import("complementary.zig");

/// One-pole low pass. Renamed from `low_pass.LowPass` for callers who import the
/// stage rather than the file.
pub const LowPass = low_pass.LowPass;
pub const Ewma = low_pass.Ewma;
pub const HighPass = high_pass.HighPass;
pub const DcBlocker = high_pass.DcBlocker;
pub const Complementary = complementary.Complementary;

test {
    std.testing.refAllDecls(@This());
}
