//! The migrated algorithm layer: everything ported from the C library in
//! `include/breeze/`, stage by stage (docs/ARCHITECTURE.md §8).
//!
//! This is the root of the algorithm test suite. It is separate from the kernel
//! suite because nothing here imports the kernel - and because the kernel
//! suite's size is a claim README makes about *the kernel*, which folding
//! algorithm tests into it would quietly change.
//!
//! The corpus every stage is checked against lives in `math/corpus.zig` (the
//! fixture predates the second stage; it is the algorithm layer's corpus, not
//! math's). Its chain is: `tools/corpus/gen_math_corpus.c` runs the C code, its
//! output is committed as `math/testdata/math_corpus.txt`, and
//! `tools/check-c.ps1` re-runs the generator so the answers cannot go stale.

const std = @import("std");

pub const math = @import("math/math.zig");
pub const filter = @import("filter/filter.zig");
pub const control = @import("control/control.zig");
pub const image = @import("image/image.zig");

test {
    std.testing.refAllDecls(@This());
}
