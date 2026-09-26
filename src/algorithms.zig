//! The algorithm layer: four stages - math, filter, control, image - ported from
//! the C library that used to live in `include/breeze/` (docs/ARCHITECTURE.md §8).
//!
//! **That C library is gone from this tree.** The migration finished, and the
//! reference implementation, its generator, its examples and its gate were
//! removed; all of it is preserved on the `archive/c-algorithm-layer` branch, and
//! the record of what was found while porting it is docs/REVIEW.md §27-§50. The
//! "ported from `include/breeze/...`" lines in the module headers name the file
//! each module came from, which is where the archive is useful.
//!
//! This is the root of the algorithm test suite. It is separate from the kernel
//! suite because nothing here imports the kernel - and because the kernel
//! suite's size is a claim README makes about *the kernel*, which folding
//! algorithm tests into it would quietly change.
//!
//! The 556 cases every stage is checked against live in `math/corpus.zig` (the
//! fixture predates the second stage; it is the algorithm layer's corpus, not
//! math's). It is a **frozen** record of the C library's answers, so nothing can
//! silently re-derive it - see that file's header for what that means when a
//! ported function is deliberately changed.

const std = @import("std");

pub const math = @import("math/math.zig");
pub const filter = @import("filter/filter.zig");
pub const control = @import("control/control.zig");
pub const image = @import("image/image.zig");

test {
    std.testing.refAllDecls(@This());
}
