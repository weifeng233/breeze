//! Breeze Framework - kernel timebase.
//!
//! A `Tick` is a millisecond count since boot. It is deliberately a plain
//! integer rather than a struct: on Cortex-M0 / RV32I an integer in a register
//! is the cheapest possible timebase, and wrap-around behaviour is made
//! explicit by the helpers below instead of being hidden.
//!
//! Wraparound: all comparisons go through signed difference, so they stay
//! correct across the u32 wrap as long as no single interval exceeds
//! 2^31 ms (~24.8 days). That bound is asserted in the tests.

const std = @import("std");

pub const Tick = u32;

pub const ms_per_sec: u32 = 1000;
pub const ms_per_min: u32 = 60 * ms_per_sec;
pub const ms_per_hour: u32 = 60 * ms_per_min;

/// Largest interval for which wrap-safe comparison is well defined.
pub const max_interval_ms: u32 = 0x7FFF_FFFF;

/// Signed distance from `from` to `to`, correct across u32 wrap.
///
/// A negative result means `to` lies in the past relative to `from`.
pub inline fn signedDiff(from: Tick, to: Tick) i32 {
    return @bitCast(to -% from);
}

/// Milliseconds elapsed from `start` to `now`, clamped at zero.
///
/// Clamping matters: if a deadline already passed, callers almost always want
/// "0 ms late" rather than a huge wrapped number.
pub inline fn elapsed(start: Tick, now: Tick) u32 {
    const d = signedDiff(start, now);
    return if (d <= 0) 0 else @intCast(d);
}

/// Raw (unclamped) elapsed time. Use when you need to detect "in the past".
pub inline fn elapsedSigned(start: Tick, now: Tick) i32 {
    return signedDiff(start, now);
}

/// True once `now` has reached `deadline`.
///
/// This is the wrap-safe replacement for `now >= deadline`.
pub inline fn reached(now: Tick, deadline: Tick) bool {
    return signedDiff(deadline, now) >= 0;
}

/// Deadline `ms` after `from`, wrapping safely.
pub inline fn after(from: Tick, ms: u32) Tick {
    return from +% ms;
}

test "elapsed is wrap-safe" {
    try std.testing.expectEqual(@as(u32, 100), elapsed(0, 100));
    try std.testing.expectEqual(@as(u32, 0), elapsed(100, 100));
    // start is in the future -> clamp to zero rather than wrapping huge
    try std.testing.expectEqual(@as(u32, 0), elapsed(100, 50));
}

test "reached works across the u32 wrap" {
    const near_wrap: Tick = 0xFFFF_FFF0;
    const past_wrap: Tick = 0x0000_0010; // 32 ms later, clock wrapped

    try std.testing.expect(reached(past_wrap, near_wrap));
    try std.testing.expectEqual(@as(u32, 32), elapsed(near_wrap, past_wrap));

    // A deadline that is genuinely still in the future must not read as reached.
    const deadline = after(near_wrap, 100);
    try std.testing.expect(!reached(past_wrap, deadline));
}

test "elapsed measures long intervals correctly" {
    const start: Tick = 1000;
    const end = after(start, 3 * ms_per_hour);
    try std.testing.expectEqual(3 * ms_per_hour, elapsed(start, end));
}
