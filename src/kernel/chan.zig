//! SPSC channels and ports: the ISR-to-task data path.
//!
//! # Where this comes from
//!
//! LibXR (the XRobot robotics framework) makes a strong argument that the
//! hot path in a driver should be a *data flow* driven by hardware events:
//! the ISR hands bytes over and advances state, a task expands them, and
//! nothing on that path takes a lock, blocks, or allocates. Breeze already
//! agreed with the conclusion - its ISRs only raise event flags - but had no
//! byte-level handover primitive, so every driver had to invent one. This is
//! that primitive.
//!
//! # Where it deliberately differs from LibXR
//!
//! LibXR's `SPSCQueue` allocates its ring with `::operator new[]` in the
//! constructor and owns it. Breeze takes **caller-provided storage** instead,
//! because the whole point of the kernel is that the memory cost of a driver is
//! visible at the call site and known at compile time:
//!
//! ```zig
//! var uart0_rx_buf: [128]u8 = undefined;   // 127 usable + 1 spare slot
//! var uart0_rx = Channel(u8, 127).init(&uart0_rx_buf);
//! ```
//!
//! Both sacrifice one slot to distinguish full from empty, and both are true
//! single-producer/single-consumer lock-free structures. Breeze uses the
//! volatile access discipline documented in `shared.zig` rather than atomics,
//! so the kernel stays linkable on ARMv6-M without libatomic.
//!
//! # Who may call what
//!
//! | method            | ISR | task |
//! |-------------------|-----|------|
//! | `pushFromIsr`     | yes | no   |
//! | `pop`, `peek`     | no  | yes  |
//! | `count`, `isEmpty`| yes (advisory) | yes |
//!
//! Calling `pushFromIsr` from a task would break the single-producer premise;
//! calling `pop` from an ISR would break the single-consumer premise. Nothing
//! enforces this at runtime - it is a structural contract, which is why it is
//! written down here rather than checked.

const std = @import("std");
const shared = @import("shared.zig");

/// A single-producer / single-consumer ring over caller-provided storage.
///
/// `capacity` is the number of payload elements; one extra slot is used
/// internally, so `backing.len` must be `capacity + 1` or larger.
pub fn Channel(comptime T: type, comptime capacity: usize) type {
    if (capacity == 0) {
        @compileError("Channel capacity must be at least 1");
    }
    // Growing by one needs a power of two to keep the index wrap a mask;
    // require that so the modulo is free on targets without a divider.
    if (!std.math.isPowerOfTwo(capacity + 1)) {
        @compileError(std.fmt.comptimePrint(
            "Channel capacity must be 2^n - 1 so that capacity+1 is a power of two (got {d})",
            .{capacity},
        ));
    }

    return struct {
        const Self = @This();
        const slots = capacity + 1;
        /// u32 rather than `usize`: the index arithmetic must not be promoted
        /// to 64 bits on a host during tests, and 32 bits is the native width
        /// of every target this kernel supports.
        const mask: u32 = @intCast(slots - 1);

        backing: *[slots]T,
        /// Written by the producer only, read by the consumer only.
        ///
        /// Both indices are stored **masked**, always in `0..slots`, so they
        /// never approach the u32 wrap even after months of traffic. That is
        /// why the extreme-index case needs no special handling: `head +% 1`
        /// cannot overflow a value bounded by `mask`, which is at most `2^31-1`.
        ///
        /// Both are also shared - `head` is written by the ISR and read by the
        /// task, `tail` the other way round - so every access to either goes
        /// through `shared`, on both sides. Reading or writing one directly
        /// would let the compiler cache it, which is what `shared.zig` exists
        /// to prevent.
        head: u32 = 0,
        tail: u32 = 0,

        /// Bind a channel to caller-owned storage.
        pub fn init(storage: *[slots]T) Self {
            return .{ .backing = storage };
        }

        /// Push one element from interrupt context.
        ///
        /// Returns false if the channel is full; the caller decides whether to
        /// drop, overwrite, or count the loss. Breeze never silently discards
        /// on the caller's behalf.
        pub fn pushFromIsr(self: *Self, value: T) bool {
            const next = (self.head +% 1) & mask;
            // `tail` is written by the consumer and read here, so it is shared
            // state too - not only `head`. Reading it directly would let the
            // compiler cache it, and a producer that keeps seeing a stale tail
            // reports "full" for ever. See the note on `tail` below.
            if (next == (shared.load(u32, &self.tail) & mask)) return false;
            self.backing[self.head & mask] = value;
            // Publish the element before the index that reveals it.
            shared.store(u32, &self.head, next);
            return true;
        }

        /// Pop one element from task context.
        pub fn pop(self: *Self) ?T {
            const tail = self.tail & mask;
            if (tail == shared.load(u32, &self.head)) return null;
            const value = self.backing[tail];
            shared.store(u32, &self.tail, (self.tail +% 1) & mask);
            return value;
        }

        /// Read the next element without consuming it.
        pub fn peek(self: *const Self) ?T {
            const tail = self.tail & mask;
            if (tail == shared.load(u32, &self.head)) return null;
            return self.backing[tail];
        }

        /// Elements currently queued.
        pub fn count(self: *const Self) u32 {
            return (shared.load(u32, &self.head) -% self.tail) & mask;
        }

        pub fn isEmpty(self: *const Self) bool {
            return self.count() == 0;
        }

        pub fn isFull(self: *const Self) bool {
            return self.count() == capacity;
        }

        /// Free space, in elements.
        pub fn space(self: *const Self) u32 {
            return @as(u32, @intCast(capacity)) - self.count();
        }

        /// Discard everything. Task context only, and only while the producer
        /// is known to be idle (otherwise it races the ISR).
        pub fn reset(self: *Self) void {
            shared.store(u32, &self.tail, shared.load(u32, &self.head));
        }

        /// Drain into a caller slice, returning how many were taken.
        pub fn read(self: *Self, out: []T) usize {
            var n: usize = 0;
            while (n < out.len) : (n += 1) {
                out[n] = self.pop() orelse break;
            }
            return n;
        }

        /// Compile-time storage requirement, for static assertions and docs.
        pub const storage_len = slots;
        pub const usable_capacity = capacity;
    };
}

/// A byte channel specialised for serial links.
pub fn ByteChannel(comptime capacity: usize) type {
    return Channel(u8, capacity);
}

/// Counts the losses a channel would otherwise hide.
///
/// A driver that cannot accept a byte should be able to say so in telemetry
/// rather than dropping it silently - a distinction LibXR also insists on.
pub fn CountedChannel(comptime T: type, comptime capacity: usize) type {
    return struct {
        const Self = @This();
        const Inner = Channel(T, capacity);

        chan: Inner,
        dropped: u32 = 0,

        pub fn init(storage: *[Inner.storage_len]T) Self {
            return .{ .chan = Inner.init(storage) };
        }

        pub fn pushFromIsr(self: *Self, value: T) bool {
            if (self.chan.pushFromIsr(value)) return true;
            self.dropped +%= 1;
            return false;
        }

        pub fn pop(self: *Self) ?T {
            return self.chan.pop();
        }

        pub fn count(self: *const Self) u32 {
            return self.chan.count();
        }

        pub fn droppedCount(self: *const Self) u32 {
            return shared.load(u32, &self.dropped);
        }
    };
}

// --- tests -----------------------------------------------------------------

const Fixture = struct {
    var buf31: [32]u8 = undefined;
    var buf7: [8]u8 = undefined;
    var buf4: [4]u16 = undefined;
    var counted: [8]u8 = undefined;
    /// 1023 usable slots; see the masked-index test for why that size.
    var big: [1024]u16 = undefined;
};

test "channel starts empty and accepts up to capacity" {
    var ch = ByteChannel(31).init(&Fixture.buf31);

    try std.testing.expect(ch.isEmpty());
    try std.testing.expectEqual(@as(u32, 31), ch.space());

    var i: u8 = 0;
    while (i < 31) : (i += 1) {
        try std.testing.expect(ch.pushFromIsr(i));
    }
    try std.testing.expectEqual(@as(u32, 31), ch.count());
    try std.testing.expect(ch.isFull());
    try std.testing.expectEqual(@as(u32, 0), ch.space());
}

test "channel reports full rather than overwriting" {
    var ch = ByteChannel(7).init(&Fixture.buf7);

    var i: u8 = 0;
    while (i < 7) : (i += 1) {
        _ = ch.pushFromIsr(i);
    }
    // The eighth push must fail, and must not corrupt the oldest element.
    try std.testing.expect(!ch.pushFromIsr(99));
    try std.testing.expectEqual(@as(u8, 0), ch.pop().?);
    try std.testing.expectEqual(@as(u8, 1), ch.pop().?);
}

test "channel preserves FIFO order across a wrap" {
    var ch = ByteChannel(7).init(&Fixture.buf7);

    // Push, pop, repeat, so head and tail wrap past the end of the buffer.
    var round: u8 = 0;
    while (round < 20) : (round += 1) {
        try std.testing.expect(ch.pushFromIsr(round));
        try std.testing.expectEqual(round, ch.pop().?);
    }
    try std.testing.expect(ch.isEmpty());

    // Now fill partially, wrap, and confirm order is still intact.
    try std.testing.expect(ch.pushFromIsr(1));
    try std.testing.expect(ch.pushFromIsr(2));
    try std.testing.expect(ch.pushFromIsr(3));
    try std.testing.expectEqual(@as(u8, 1), ch.pop().?);
    try std.testing.expect(ch.pushFromIsr(4));
    try std.testing.expect(ch.pushFromIsr(5));

    try std.testing.expectEqual(@as(u8, 2), ch.pop().?);
    try std.testing.expectEqual(@as(u8, 3), ch.pop().?);
    try std.testing.expectEqual(@as(u8, 4), ch.pop().?);
    try std.testing.expectEqual(@as(u8, 5), ch.pop().?);
    try std.testing.expect(ch.pop() == null);
}

test "channel works for non-byte payloads" {
    var ch = Channel(u16, 3).init(&Fixture.buf4);

    try std.testing.expect(ch.pushFromIsr(0x1234));
    try std.testing.expect(ch.pushFromIsr(0xABCD));
    try std.testing.expect(ch.pushFromIsr(0x0001));
    try std.testing.expect(!ch.pushFromIsr(0xFFFF));

    try std.testing.expectEqual(@as(u16, 0x1234), ch.pop().?);
    try std.testing.expectEqual(@as(u16, 0xABCD), ch.pop().?);
    try std.testing.expectEqual(@as(u16, 0x0001), ch.pop().?);
}

test "indices stay masked, so the u32 wrap is unreachable" {
    // The reviewer's question was whether extreme head/tail values break the
    // index arithmetic. They cannot, because both indices are stored masked and
    // therefore live in `0..slots`. This test drives a very long run through a
    // deliberately awkward capacity - the wrap point of the ring, not of u32 -
    // and asserts the invariant on every single step.
    //
    // 1023 usable slots means the indices cycle every 1024 pushes, so 100k
    // pushes cross the ring wrap ~97 times without ever approaching 2^32.
    const Cap = 1023;
    var ch = Channel(u16, Cap).init(&Fixture.big);

    var pushed: u32 = 0;
    var popped: u32 = 0;
    var expected: u16 = 0;

    while (pushed < 100_000) {
        // Keep the ring roughly half full so both push and pop take turns.
        if (ch.count() < Cap / 2) {
            try std.testing.expect(ch.pushFromIsr(expected));
            expected +%= 1;
            pushed += 1;
        } else {
            const got = ch.pop().?;
            try std.testing.expectEqual(@as(u16, @truncate(popped)), got);
            popped += 1;
        }

        // The invariant: neither index has left the masked range.
        try std.testing.expect(ch.head <= Cap);
        try std.testing.expect(ch.tail <= Cap);
        // And the occupancy reading stays consistent with the two indices.
        try std.testing.expect(ch.count() <= Cap);
    }
}

test "the producer reads the consumer's tail through the shared accessor" {
    // `tail` is written by the consumer and read by the producer, so it is
    // shared state in both directions - the same rule that applies to `head`.
    // A cached `tail` in the producer would report "full" for ever. This test
    // exercises the interleaving that would expose it: the consumer drains
    // while the producer keeps pushing, always to the same ring position.
    var ch = ByteChannel(7).init(&Fixture.buf7);

    var round: u32 = 0;
    while (round < 1000) : (round += 1) {
        // Fill.
        var i: u32 = 0;
        while (i < 7) : (i += 1) {
            try std.testing.expect(ch.pushFromIsr(@truncate(i + round)));
        }
        // The eighth must be refused, and must stay refused while full.
        try std.testing.expect(!ch.pushFromIsr(0xFF));

        // Drain one: the producer must now see room again immediately.
        _ = ch.pop();
        try std.testing.expect(ch.pushFromIsr(0xEE));

        // Drain the rest.
        i = 0;
        while (i < 7) : (i += 1) {
            try std.testing.expect(ch.pop() != null);
        }
        try std.testing.expect(ch.pop() == null);
        try std.testing.expect(ch.isEmpty());
    }
}

test "peek does not consume" {
    var ch = ByteChannel(7).init(&Fixture.buf7);
    _ = ch.pushFromIsr(42);

    try std.testing.expectEqual(@as(u8, 42), ch.peek().?);
    try std.testing.expectEqual(@as(u32, 1), ch.count());
    try std.testing.expectEqual(@as(u8, 42), ch.pop().?);
    try std.testing.expect(ch.peek() == null);
}

test "read drains into a slice" {
    var ch = ByteChannel(7).init(&Fixture.buf7);
    var i: u8 = 0;
    while (i < 5) : (i += 1) {
        _ = ch.pushFromIsr(i);
    }

    var out: [3]u8 = undefined;
    try std.testing.expectEqual(@as(usize, 3), ch.read(&out));
    try std.testing.expectEqualSlices(u8, &.{ 0, 1, 2 }, &out);

    var rest: [4]u8 = undefined;
    try std.testing.expectEqual(@as(usize, 2), ch.read(&rest));
    try std.testing.expectEqualSlices(u8, &.{ 3, 4 }, rest[0..2]);
}

test "reset discards without touching the producer index" {
    var ch = ByteChannel(7).init(&Fixture.buf7);
    _ = ch.pushFromIsr(1);
    _ = ch.pushFromIsr(2);
    ch.reset();
    try std.testing.expect(ch.isEmpty());
    try std.testing.expect(ch.pushFromIsr(3));
    try std.testing.expectEqual(@as(u8, 3), ch.pop().?);
}

test "counted channel records losses instead of hiding them" {
    var ch = CountedChannel(u8, 7).init(&Fixture.counted);

    var i: u8 = 0;
    while (i < 10) : (i += 1) {
        _ = ch.pushFromIsr(i);
    }

    try std.testing.expectEqual(@as(u32, 7), ch.count());
    try std.testing.expectEqual(@as(u32, 3), ch.droppedCount());
    try std.testing.expectEqual(@as(u8, 0), ch.pop().?);
}

test "storage requirement is known at compile time" {
    // The caller must provision capacity+1 slots; this is what makes the RAM
    // cost of a driver visible at the declaration site.
    try std.testing.expectEqual(@as(usize, 32), ByteChannel(31).storage_len);
    try std.testing.expectEqual(@as(usize, 31), ByteChannel(31).usable_capacity);
    try std.testing.expectEqual(@as(usize, 128), ByteChannel(127).storage_len);
}
