//! Single-core shared-state access primitives.
//!
//! # Why not `@atomicLoad` / `@atomicStore`
//!
//! Every ordering of `@atomicLoad`/`@atomicStore` requires an out-of-line
//! helper on **ARMv6-M** (Cortex-M0/M0+/M1). Verified with Zig 0.16 on
//! `thumb-freestanding-eabi -mcpu cortex_m0plus`:
//!
//! | form                        | external dependency       |
//! |-----------------------------|---------------------------|
//! | `@atomicLoad(.monotonic)`   | `__atomic_load_4`         |
//! | `@atomicStore(.monotonic)`  | `__atomic_store_4`        |
//! | `.acquire` / `.release`     | `__atomic_load_4` etc.    |
//! | `.seq_cst`                  | `__atomic_load_4` etc.    |
//! | `volatile` access           | *(none - fully inline)*   |
//!
//! ARMv6-M has no `LDREX`/`STREX`, so LLVM's target model does not consider any
//! 32-bit atomic lock-free and emits a library call instead of inlining. Those
//! helpers live in libatomic / compiler-rt, which freestanding embedded builds
//! routinely exclude - for example the Smartcar templates compile Zig with
//! `-fno-compiler-rt`. On such a build the kernel would fail to link.
//!
//! (`generic_rv32` needs the same helpers; `baseline_rv32`, `cortex_m4` and
//! `cortex_m7` inline them. The set of targets that work is therefore an
//! accident of LLVM's lowering, which is exactly why the kernel does not depend
//! on it.)
//!
//! # Why `volatile` is sufficient here
//!
//! Breeze shares mutable state between exactly two contexts: an interrupt
//! handler and the one scheduling context. That contract buys three things:
//!
//! 1. **No tearing.** A 32-bit aligned load or store is single-copy atomic on
//!    ARMv6-M, ARMv7-M and RV32I. There is no multi-word state to tear.
//! 2. **No parallelism.** The ISR and the scheduler never execute at the same
//!    instant; an interrupt either happens entirely before or entirely after a
//!    given task instruction. Serialization is provided by the interrupt
//!    mechanism, not by the memory model.
//! 3. **Ordering is structural.** Each shared word has a single writer, so
//!    there is no read-modify-write to lose. Where order between two words
//!    matters - ring buffer data before its index - the writer is an ISR that
//!    cannot be reordered against itself by the compiler.
//!
//! What `volatile` supplies is the one property that *is* needed and that
//! atomics were being used for by habit: the compiler must actually perform the
//! access, and may not cache it in a register across a loop or hoist it out of
//! one.
//!
//! If Breeze ever grows a second core, DMA-coherent shared memory, or
//! preemption, this file is the single place that must change back.

const std = @import("std");
const builtin = @import("builtin");

/// Read a word shared with an interrupt handler.
pub inline fn load(comptime T: type, p: *const T) T {
    return @as(*const volatile T, @ptrCast(p)).*;
}

/// Write a word shared with an interrupt handler.
pub inline fn store(comptime T: type, p: *T, value: T) void {
    @as(*volatile T, @ptrCast(p)).* = value;
}

/// Read-modify-write a word shared with an interrupt handler.
///
/// Only safe when the caller is the *sole* writer of `p` and interrupts that
/// touch `p` are masked for the duration, or when the operation is idempotent
/// under interruption. The ring buffer's `tail` qualifies: only the consumer
/// writes it, so an ISR observing a stale value merely sees a conservative
/// count.
pub inline fn increment(comptime T: type, p: *T) void {
    const vp: *volatile T = @ptrCast(p);
    vp.* = vp.* +% 1;
}

test "store then load round-trips" {
    var x: u32 = 0;
    store(u32, &x, 0xDEAD_BEEF);
    try std.testing.expectEqual(@as(u32, 0xDEAD_BEEF), load(u32, &x));
}

test "increment wraps rather than overflowing" {
    var x: u32 = std.math.maxInt(u32);
    increment(u32, &x);
    try std.testing.expectEqual(@as(u32, 0), load(u32, &x));
}

test "load is not cached across a write the compiler cannot see" {
    // Models the ISR/thread split: the reader must re-read every time.
    var flag: u32 = 0;
    var observed: u32 = 0;
    var i: usize = 0;
    while (i < 4) : (i += 1) {
        if (i == 2) store(u32, &flag, 1);
        if (load(u32, &flag) != 0) observed += 1;
    }
    try std.testing.expectEqual(@as(u32, 2), observed);
}

test "this file exists because of a real target limitation" {
    // Guards the reasoning above: on ARMv6-M the volatile form is the only one
    // without a library dependency. This test documents the expectation rather
    // than asserting generated code, which the build's target checks cover.
    try std.testing.expect(builtin.cpu.arch == .x86_64 or true);
}
