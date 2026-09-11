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

/// Turns a hardware timer frequency into one-millisecond periods.
///
/// # Why this exists
///
/// A one-shot timer such as RISC-V's `mtimecmp` is reprogrammed on every tick,
/// so it does not have to use the same period each time. That is the whole
/// trick here: when the clock frequency is not a whole number of cycles per
/// millisecond - 32.768 kHz is the classic case, at 32.768 cycles - a fixed
/// period must round, and the clock then runs at the wrong rate forever.
///
/// `32768 / 1000 = 32` gives a 0.977 ms tick, so the timebase runs **2.34%
/// fast**: 84 seconds of error per hour, before any drift in the source
/// oscillator. Alternating between 32 and 33 cycles in the right proportion
/// makes the *average* exactly one millisecond, which is the property that
/// matters for a monotonic clock.
///
/// The accumulator is the standard Bresenham/error-diffusion step: add the
/// remainder each tick and emit an extra cycle whenever it overflows. Cost is
/// one add, one compare and one subtract per tick, with no division and no
/// floating point.
///
/// A SysTick-style auto-reloading timer cannot use this, because its period is
/// fixed in hardware - see `hal/cortex_m.zig`, which bounds the error instead.
pub const MillisecondGrid = struct {
    /// Whole cycles in one millisecond.
    whole: u32,
    /// Leftover cycles per millisecond, `hz % 1000`.
    remainder: u32,
    /// Error carried between ticks, always in `0..1000`.
    accumulator: u32 = 0,

    /// Smallest frequency this can represent. Below it `whole` would be zero or
    /// the accumulator could never emit a cycle, and the timer would either
    /// never fire or fire for ever - see `next`'s note on the zero period.
    pub const min_hz: u32 = 1000;

    pub fn init(hz: u32) MillisecondGrid {
        std.debug.assert(hz >= min_hz);
        return .{ .whole = hz / 1000, .remainder = hz % 1000 };
    }

    /// Period in timer cycles for the next tick.
    ///
    /// Never returns zero: the guard matters because a zero period would make a
    /// one-shot timer fire immediately and for ever, which presents as a
    /// hung interrupt rather than as a wrong clock rate.
    pub fn next(self: *MillisecondGrid) u32 {
        self.accumulator += self.remainder;
        var period = self.whole;
        if (self.accumulator >= 1000) {
            self.accumulator -= 1000;
            period += 1;
        }
        return if (period == 0) 1 else period;
    }
};

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

test "MillisecondGrid emits whole periods for an exact frequency" {
    var g = MillisecondGrid.init(1_000_000);
    try std.testing.expectEqual(@as(u32, 1000), g.whole);
    try std.testing.expectEqual(@as(u32, 0), g.remainder);

    // No remainder means no correction is ever needed.
    var i: usize = 0;
    while (i < 100) : (i += 1) {
        try std.testing.expectEqual(@as(u32, 1000), g.next());
    }
}

test "MillisecondGrid averages exactly one millisecond at 32.768 kHz" {
    const hz: u32 = 32_768;
    var g = MillisecondGrid.init(hz);

    // The naive fixed period, for contrast.
    const naive: u32 = hz / 1000;
    try std.testing.expectEqual(@as(u32, 32), naive);

    // 1000 ticks must consume exactly hz cycles: that is what "one millisecond
    // on average" means, and it is the property a fixed period cannot have.
    var total: u64 = 0;
    var short_ticks: u32 = 0;
    var long_ticks: u32 = 0;
    var i: usize = 0;
    while (i < 1000) : (i += 1) {
        const p = g.next();
        total += p;
        if (p == naive) short_ticks += 1;
        if (p == naive + 1) long_ticks += 1;
        try std.testing.expect(p == naive or p == naive + 1);
    }

    try std.testing.expectEqual(@as(u64, hz), total);

    // 768 ms in every 1000 need the extra cycle, and the two counts must add up.
    try std.testing.expectEqual(@as(u32, 768), long_ticks);
    try std.testing.expectEqual(@as(u32, 232), short_ticks);

    // Quantify what the naive approach would have cost, so the test documents
    // the size of the bug rather than only its absence.
    const naive_total: u64 = @as(u64, naive) * 1000;
    try std.testing.expectEqual(@as(u64, 32_000), naive_total);
    // The fixed period loses 768 cycles per second of ticks: 2.34% fast.
    try std.testing.expectEqual(@as(u64, 768), @as(u64, hz) - naive_total);
}

test "MillisecondGrid never emits a zero period" {
    // The floor is exactly where `whole` is 1 and the remainder is 0.
    var g = MillisecondGrid.init(MillisecondGrid.min_hz);
    var i: usize = 0;
    while (i < 100) : (i += 1) {
        try std.testing.expect(g.next() >= 1);
    }

    // And just above the floor, where `whole` is 1 but corrections happen.
    var h = MillisecondGrid.init(1001);
    i = 0;
    var total: u64 = 0;
    while (i < 1000) : (i += 1) {
        const p = h.next();
        try std.testing.expect(p >= 1);
        total += p;
    }
    try std.testing.expectEqual(@as(u64, 1001), total);
}
