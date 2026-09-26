//! The algorithm layer, migrated module by module from the C library in
//! `include/breeze/` (see docs/ARCHITECTURE.md §8).
//!
//! Nothing here depends on the kernel, on purpose: these are pure functions over
//! values, and a task is merely one possible caller. That keeps the layer usable
//! from a host tool, from an ISR, or from a firmware that does not use the
//! scheduler at all - and it keeps the kernel's own surface unchanged.
//!
//! Each module is checked against the C implementation it replaces, through the
//! corpus in `testdata/` (see `corpus.zig` for how that chain is enforced).

const std = @import("std");

pub const vector = @import("vector.zig");

test {
    std.testing.refAllDecls(@This());
}
