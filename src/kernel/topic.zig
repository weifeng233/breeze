//! Compile-time topics and the telemetry packet format.
//!
//! # What this is for
//!
//! A smartcar has to explain itself: the student needs to see the gyro, the
//! encoder counts and the PID terms on a PC while the car is on the bench. That
//! is a publish/subscribe problem, and XRobot/LibXR has already specified a
//! wire format for it that is worth being compatible with, because a compatible
//! format means an existing host tool can parse Breeze telemetry.
//!
//! # Where it deliberately differs from LibXR
//!
//! LibXR topics are resolved **at runtime by name**: `Topic::FindOrCreate(name,
//! domain)` looks the name up (via CRC32) in a static red-black tree, and
//! subscribers are attached to a lock-free list. That is what lets its code
//! generator wire modules together by string.
//!
//! Breeze does not need that flexibility, and does not want its cost. A topic
//! here is a **type**, created at comptime:
//!
//! ```zig
//! pub const Attitude = breeze.Topic("attitude", extern struct {
//!     roll: f32, pitch: f32, yaw: f32,
//! });
//! ```
//!
//! `Attitude.id` is the CRC32 of `"attitude"` folded at compile time, so there
//! is no registry, no tree, no lookup, no lock - and the compiler rejects a
//! subscribe to a topic that does not exist. The name is kept only so that the
//! generated frames remain parseable by name-based tools.
//!
//! # Wire format (LibXR-compatible)
//!
//! ```text
//! offset  size  field
//!      0     1  prefix = 0x5A
//!      1     3  payload length, little-endian (24-bit)
//!      4     4  CRC32 of the topic name, little-endian
//!      8     6  timestamp, microseconds, little-endian (48-bit)
//!     14     1  version = 0x01
//!     15     1  CRC8 of bytes 0..14
//!     16     N  payload (raw struct bytes)
//!  16+N     1  CRC8 of bytes 0..15+N
//! ```
//!
//! # Two caveats on compatibility
//!
//! The field layout above is taken from the LibXR documentation. The **CRC
//! polynomials are not specified in it**, so this file uses CRC-32/ISO-HDLC
//! (reflected, polynomial `0xEDB88320`) and CRC-8/ATM (polynomial `0x07`). If
//! byte-for-byte interop with a real LibXR host tool is required, verify both
//! against the LibXR source before trusting them; the tests below pin the
//! current behaviour so a change is at least deliberate.
//!
//! Payloads are written as raw struct bytes, which is only correct while the
//! target is little-endian. Cortex-M and RISC-V both are.

const std = @import("std");
const builtin = @import("builtin");

// --- checksums -------------------------------------------------------------

/// CRC-32/ISO-HDLC: reflected, polynomial 0xEDB88320, init/final 0xFFFFFFFF.
pub fn crc32(bytes: []const u8) u32 {
    var crc: u32 = 0xFFFF_FFFF;
    for (bytes) |b| {
        crc ^= b;
        var i: u4 = 0;
        while (i < 8) : (i += 1) {
            // Branch-free: mask is all-ones when the low bit is set.
            const mask: u32 = @as(u32, 0) -% (crc & 1);
            crc = (crc >> 1) ^ (0xEDB8_8320 & mask);
        }
    }
    return ~crc;
}

/// CRC-8/ATM: polynomial 0x07, init 0x00, no reflection, no final xor.
///
/// The left shift is done in 16 bits and truncated rather than on `u8`
/// directly: CRC-8 is defined by a polynomial division that *discards* the bit
/// shifted out of the top, which in Zig is not what `crc << 1` does. Doing the
/// shift on `u8` there is a silent off-by-one-polynomial bug, which is exactly
/// what the check-value test below exists to catch.
pub fn crc8(bytes: []const u8) u8 {
    var crc: u8 = 0x00;
    for (bytes) |b| {
        crc ^= b;
        var i: u4 = 0;
        while (i < 8) : (i += 1) {
            const wide = @as(u16, crc) << 1;
            crc = if ((crc & 0x80) != 0)
                @truncate(wide ^ 0x07)
            else
                @truncate(wide);
        }
    }
    return crc;
}

// --- packet constants ------------------------------------------------------

pub const packet_prefix: u8 = 0x5A;
pub const packet_version: u8 = 0x01;

/// Bytes before the payload.
pub const header_len: usize = 16;

/// Header plus the trailing checksum, i.e. the framing overhead of any packet.
pub const frame_overhead: usize = header_len + 1;

/// Largest payload the 24-bit length field can describe.
pub const max_payload_len: u32 = 0x00FF_FFFF;

// --- topic -----------------------------------------------------------------

/// A compile-time publish/subscribe channel.
///
/// `name` identifies the topic on the wire; `Payload` is the value type. The
/// payload must have a stable byte layout, so it has to be an `extern struct`.
pub fn Topic(comptime name: []const u8, comptime Payload: type) type {
    if (name.len == 0) {
        @compileError("Topic name must not be empty");
    }
    if (@typeInfo(Payload) != .@"struct") {
        @compileError("Topic '" ++ name ++ "' payload must be a struct");
    }
    comptime {
        const info = @typeInfo(Payload).@"struct";
        if (info.layout != .@"extern") {
            @compileError(
                "Topic '" ++ name ++ "' payload must be an `extern struct` so that its byte " ++
                    "layout is defined; a plain struct has no guaranteed layout",
            );
        }

        // The payload is copied out as raw bytes, so every property that makes
        // those bytes meaningful has to hold. These checks turn a class of
        // bug that would otherwise appear as "the host decodes garbage" into a
        // compile error naming the field.
        if (builtin.cpu.arch.endian() != .little) {
            @compileError(
                "Topic '" ++ name ++ "' packs its payload as raw struct bytes, which is " ++
                    "only the wire format on little-endian targets. This target is big-endian.",
            );
        }

        var packed_size: usize = 0;
        for (info.fields) |f| {
            // A field whose width depends on the pointer size would make the
            // frame layout differ between RV32 and RV64 for the *same* source.
            if (f.type == usize or f.type == isize) {
                @compileError(
                    "Topic '" ++ name ++ "' field '" ++ f.name ++ "' is " ++ @typeName(f.type) ++
                        ", whose width follows the target's pointer size; use a fixed-width " ++
                        "integer so the frame is identical on every target",
                );
            }
            switch (@typeInfo(f.type)) {
                .pointer => @compileError(
                    "Topic '" ++ name ++ "' field '" ++ f.name ++ "' is a pointer. Pointers " ++
                        "are meaningless to a host parser and their width varies by target.",
                ),
                else => {},
            }
            packed_size += @sizeOf(f.type);
        }

        // Padding bytes are copied into the frame but are not part of any
        // field, and Zig does not promise to initialise them. Two packs of the
        // same value could therefore produce different bytes, and `unpack`
        // would write indeterminate bytes back into the payload. The frame's
        // own CRC covers whatever was sent, so a receiver cannot detect it.
        if (packed_size != @sizeOf(Payload)) {
            @compileError(std.fmt.comptimePrint(
                "Topic '{s}' payload has {d} byte(s) of padding ({d} bytes of fields, " ++
                    "{d} bytes of struct). Padding is copied into the frame but is never " ++
                    "initialised, so the same value can serialise to different bytes. " ++
                    "Reorder the fields, or add explicitly named padding fields so the " ++
                    "bytes are deterministic.",
                .{ name, @sizeOf(Payload) - packed_size, packed_size, @sizeOf(Payload) },
            ));
        }
    }

    return struct {
        const Self = @This();

        /// The topic's name, as it appears on the wire.
        pub const topic_name = name;

        /// Compile-time topic identifier: CRC32 of the name.
        pub const id: u32 = crc32(name);

        /// The value type carried by this topic.
        pub const Value = Payload;

        /// Payload size in bytes.
        pub const payload_len = @sizeOf(Payload);

        /// Total bytes a frame for this topic occupies.
        pub const frame_len = header_len + payload_len + 1;

        /// Number of header bytes this topic's frames start with.
        pub const header_size = header_len;

        /// True if `buffer` is large enough to pack a value of this topic.
        pub fn fits(buffer_len: usize) bool {
            return buffer_len >= frame_len;
        }

        /// Serialise `value` into `buffer`, returning the frame slice.
        ///
        /// The timestamp is passed in microseconds and truncated to the 48 bits
        /// the format provides, which wraps every ~8.9 years - far longer than
        /// any firmware session, and the same tradeoff LibXR makes.
        pub fn pack(buffer: []u8, value: *const Payload, timestamp_us: u64) ![]u8 {
            if (buffer.len < frame_len) return error.BufferTooSmall;

            buffer[0] = packet_prefix;

            const len: u32 = payload_len;
            buffer[1] = @truncate(len);
            buffer[2] = @truncate(len >> 8);
            buffer[3] = @truncate(len >> 16);

            std.mem.writeInt(u32, buffer[4..8], id, .little);

            const ts: u48 = @truncate(timestamp_us);
            buffer[8] = @truncate(ts);
            buffer[9] = @truncate(ts >> 8);
            buffer[10] = @truncate(ts >> 16);
            buffer[11] = @truncate(ts >> 24);
            buffer[12] = @truncate(ts >> 32);
            buffer[13] = @truncate(ts >> 40);

            buffer[14] = packet_version;
            buffer[15] = crc8(buffer[0..15]);

            // Raw struct bytes. Valid because the target is little-endian and
            // the payload is an extern struct with no padding surprises on
            // 32-bit targets that matter here.
            const src: [*]const u8 = @ptrCast(value);
            @memcpy(buffer[header_len..][0..payload_len], src[0..payload_len]);

            buffer[header_len + payload_len] =
                crc8(buffer[0 .. header_len + payload_len]);

            return buffer[0..frame_len];
        }

        /// Parse a frame previously produced by `pack`.
        ///
        /// Every field is checked, including the topic id: without that check a
        /// frame of any other topic whose payload happens to be the same length
        /// decodes cleanly, with valid checksums, into this topic's payload.
        /// Two 12-byte topics are all it takes to make that a real mix-up.
        pub fn unpack(frame: []const u8) !Payload {
            if (frame.len < frame_len) return error.FrameTooShort;
            if (frame[0] != packet_prefix) return error.BadPrefix;
            if (frame[14] != packet_version) return error.BadVersion;

            const len = @as(u32, frame[1]) |
                (@as(u32, frame[2]) << 8) |
                (@as(u32, frame[3]) << 16);
            if (len != payload_len) return error.LengthMismatch;
            // Integrity first, then identity: a corrupted frame should report
            // that it is corrupt, not that it belongs to another topic.
            if (crc8(frame[0..15]) != frame[15]) return error.BadHeaderChecksum;
            if (std.mem.readInt(u32, frame[4..8], .little) != id) {
                return error.TopicMismatch;
            }

            const body = frame[0 .. header_len + payload_len];
            if (crc8(body) != frame[header_len + payload_len]) {
                return error.BadPayloadChecksum;
            }

            var out: Payload = undefined;
            const dst: [*]u8 = @ptrCast(&out);
            @memcpy(dst[0..payload_len], frame[header_len..][0..payload_len]);
            return out;
        }

        /// Read the timestamp field out of a frame, in microseconds.
        pub fn timestampOf(frame: []const u8) u48 {
            var ts: u48 = 0;
            var i: u6 = 0;
            while (i < 6) : (i += 1) {
                ts |= @as(u48, frame[8 + i]) << @intCast(i * 8);
            }
            return ts;
        }
    };
}

// --- a concrete example used by the tests ----------------------------------

pub const TestAttitude = Topic("attitude", extern struct {
    roll: f32,
    pitch: f32,
    yaw: f32,
});

pub const TestEncoder = Topic("encoder", extern struct {
    left: i32,
    right: i32,
    tick_ms: u32,
});

// --- tests -----------------------------------------------------------------

test "crc32 matches the standard check value" {
    // The canonical CRC-32/ISO-HDLC check value for "123456789".
    try std.testing.expectEqual(@as(u32, 0xCBF4_3926), crc32("123456789"));
}

test "crc8 matches the standard check value" {
    // CRC-8/ATM check value for "123456789".
    try std.testing.expectEqual(@as(u8, 0xF4), crc8("123456789"));
}

test "topic id is the crc32 of its name, folded at comptime" {
    try std.testing.expectEqual(crc32("attitude"), TestAttitude.id);
    try std.testing.expectEqual(crc32("encoder"), TestEncoder.id);
    // Distinct names must not collide in practice.
    try std.testing.expect(TestAttitude.id != TestEncoder.id);
}

test "sizes are known at compile time" {
    // attitude: three f32 = 12 bytes.
    try std.testing.expectEqual(@as(usize, 12), TestAttitude.payload_len);
    try std.testing.expectEqual(@as(usize, 16 + 12 + 1), TestAttitude.frame_len);
    // encoder: i32 + i32 + u32 = 12 bytes (not 8 - three 4-byte fields).
    try std.testing.expectEqual(@as(usize, 12), TestEncoder.payload_len);
    try std.testing.expectEqual(@as(usize, 16 + 12 + 1), TestEncoder.frame_len);
    try std.testing.expect(TestAttitude.fits(TestAttitude.frame_len));
    try std.testing.expect(!TestAttitude.fits(TestAttitude.frame_len - 1));
}

test "pack then unpack round-trips" {
    var buf: [64]u8 = undefined;
    const value = TestAttitude.Value{ .roll = 1.5, .pitch = -2.25, .yaw = 3.125 };

    const frame = try TestAttitude.pack(&buf, &value, 123_456);
    try std.testing.expectEqual(TestAttitude.frame_len, frame.len);

    const back = try TestAttitude.unpack(frame);
    try std.testing.expectEqual(value.roll, back.roll);
    try std.testing.expectEqual(value.pitch, back.pitch);
    try std.testing.expectEqual(value.yaw, back.yaw);
}

test "unpack refuses a frame that belongs to a different topic" {
    // The two topics have the same 12-byte payload length, which is all it
    // takes: without the id check the encoders frame below decodes into an
    // attitude with valid checksums and entirely plausible floats.
    try std.testing.expectEqual(TestAttitude.payload_len, TestEncoder.payload_len);

    var buf: [64]u8 = undefined;
    const value = TestEncoder.Value{ .left = 1, .right = -2, .tick_ms = 3 };
    const frame = try TestEncoder.pack(&buf, &value, 0);

    try std.testing.expectError(error.TopicMismatch, TestAttitude.unpack(frame));

    // ...and it is still readable as the topic it actually is.
    const back = try TestEncoder.unpack(frame);
    try std.testing.expectEqual(value.left, back.left);
    try std.testing.expectEqual(value.tick_ms, back.tick_ms);
}

test "frame header carries the documented fields" {
    var buf: [64]u8 = undefined;
    const value = TestAttitude.Value{ .roll = 0, .pitch = 0, .yaw = 0 };
    const frame = try TestAttitude.pack(&buf, &value, 0x0000_0000_0001);

    try std.testing.expectEqual(@as(u8, 0x5A), frame[0]);

    // 24-bit little-endian payload length.
    const len = @as(u32, frame[1]) | (@as(u32, frame[2]) << 8) | (@as(u32, frame[3]) << 16);
    try std.testing.expectEqual(@as(u32, 12), len);

    // CRC32 of the topic name, little-endian.
    try std.testing.expectEqual(TestAttitude.id, std.mem.readInt(u32, frame[4..8], .little));

    try std.testing.expectEqual(@as(u8, 0x01), frame[14]);
    try std.testing.expectEqual(crc8(frame[0..15]), frame[15]);
}

test "timestamp is a 48-bit microsecond value" {
    var buf: [64]u8 = undefined;
    const value = TestAttitude.Value{ .roll = 0, .pitch = 0, .yaw = 0 };

    const frame = try TestAttitude.pack(&buf, &value, 0x0000_1234_5678_9ABC);
    // Truncated to the low 48 bits, little-endian.
    try std.testing.expectEqual(@as(u48, 0x1234_5678_9ABC), TestAttitude.timestampOf(frame));
}

test "pack refuses a buffer it would overflow" {
    var small: [20]u8 = undefined;
    const value = TestAttitude.Value{ .roll = 0, .pitch = 0, .yaw = 0 };
    try std.testing.expectError(
        error.BufferTooSmall,
        TestAttitude.pack(&small, &value, 0),
    );
}

test "unpack rejects corruption at each layer" {
    var buf: [64]u8 = undefined;
    const value = TestAttitude.Value{ .roll = 1, .pitch = 2, .yaw = 3 };
    const frame = try TestAttitude.pack(&buf, &value, 7);

    // Wrong prefix.
    var bad_prefix: [64]u8 = undefined;
    @memcpy(bad_prefix[0..frame.len], frame);
    bad_prefix[0] = 0x00;
    try std.testing.expectError(error.BadPrefix, TestAttitude.unpack(bad_prefix[0..frame.len]));

    // Wrong version.
    var bad_version: [64]u8 = undefined;
    @memcpy(bad_version[0..frame.len], frame);
    bad_version[14] = 0x02;
    try std.testing.expectError(error.BadVersion, TestAttitude.unpack(bad_version[0..frame.len]));

    // Corrupted header: the header checksum must catch it.
    var bad_header: [64]u8 = undefined;
    @memcpy(bad_header[0..frame.len], frame);
    bad_header[4] ^= 0xFF;
    try std.testing.expectError(error.BadHeaderChecksum, TestAttitude.unpack(bad_header[0..frame.len]));

    // Corrupted payload: the trailing checksum must catch it.
    var bad_payload: [64]u8 = undefined;
    @memcpy(bad_payload[0..frame.len], frame);
    bad_payload[16] ^= 0xFF;
    try std.testing.expectError(error.BadPayloadChecksum, TestAttitude.unpack(bad_payload[0..frame.len]));

    // Truncated.
    try std.testing.expectError(error.FrameTooShort, TestAttitude.unpack(frame[0 .. frame.len - 1]));
}

test "integer payloads round-trip exactly" {
    var buf: [32]u8 = undefined;
    const value = TestEncoder.Value{
        .left = -12345,
        .right = 67890,
        .tick_ms = 0xDEAD_BEEF,
    };
    const frame = try TestEncoder.pack(&buf, &value, 99);
    const back = try TestEncoder.unpack(frame);

    try std.testing.expectEqual(@as(i32, -12345), back.left);
    try std.testing.expectEqual(@as(i32, 67890), back.right);
    try std.testing.expectEqual(@as(u32, 0xDEAD_BEEF), back.tick_ms);
    try std.testing.expectEqual(@as(u48, 99), TestEncoder.timestampOf(frame));
}
