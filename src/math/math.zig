//! The math stage: pure functions, ported module by module from the C library in
//! `include/breeze/` (see docs/ARCHITECTURE.md §8). That C library is no longer in
//! this tree - it is preserved on the `archive/c-algorithm-layer` branch.
//!
//! Nothing here depends on the kernel, on purpose: these are pure functions over
//! values, and a task is merely one possible caller. That keeps the layer usable
//! from a host tool, from an ISR, or from a firmware that does not use the
//! scheduler at all - and it keeps the kernel's own surface unchanged.
//!
//! Each module is checked against the C implementation it replaces, through the
//! corpus in `testdata/`. That corpus is a frozen record now, not a live oracle:
//! `corpus.zig` says what that means for a deliberate change.

const std = @import("std");

pub const vector = @import("vector.zig");
pub const matrix = @import("matrix.zig");
pub const quaternion = @import("quaternion.zig");
pub const interpolation = @import("interpolation.zig");

test {
    std.testing.refAllDecls(@This());
}
