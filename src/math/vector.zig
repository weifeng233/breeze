//! 2D, 3D and 4D vectors, ported from `include/breeze/math/vector.h`.
//!
//! These are plain values, so the port drops two things the C version carries
//! and keeps one it could have dropped:
//!
//! * **The out parameter is gone.** The C signature takes `result` so the caller
//!   picks the storage; here the caller still picks it (`var r = a.add(b);`) and
//!   there is no pointer to be null, so the `if (!result || !a || !b) return;`
//!   guard in all 22 functions disappears with its failure mode.
//! * **`Normalize` returns an error, not `0`.** The C caller has to remember to
//!   test the return value; here the value cannot be had without handling the
//!   case. `min_length` is the same `1e-6f` threshold.
//! * **The threshold and the refusal are kept.** They are a contract the existing
//!   callers rely on, so the port copies them rather than quietly "improving"
//!   them - which is also why `testdata/math_corpus.txt` includes cases just
//!   below and just above the boundary (see `corpus.zig`).
//!
//! Layout is deliberately identical to the C structs, and that is a checked
//! claim rather than a comment: the `*_layout` and offset cases come from
//! `_Alignof`/`offsetof` on the C types.

const std = @import("std");

const corpus_mod = @import("corpus.zig");

/// Returned by `normalized` when a vector is too short to have a direction.
pub const Degenerate = error{Degenerate};

/// The length below which the C code refuses to normalize. Copied, not chosen.
pub const min_length: f32 = 1.0e-6;

pub const Vec2 = struct {
    x: f32,
    y: f32,

    pub const zero: Vec2 = .{ .x = 0, .y = 0 };

    pub fn init(x: f32, y: f32) Vec2 {
        return .{ .x = x, .y = y };
    }

    pub fn add(a: Vec2, b: Vec2) Vec2 {
        return .{ .x = a.x + b.x, .y = a.y + b.y };
    }

    pub fn sub(a: Vec2, b: Vec2) Vec2 {
        return .{ .x = a.x - b.x, .y = a.y - b.y };
    }

    pub fn scale(a: Vec2, s: f32) Vec2 {
        return .{ .x = a.x * s, .y = a.y * s };
    }

    pub fn dot(a: Vec2, b: Vec2) f32 {
        return a.x * b.x + a.y * b.y;
    }

    pub fn lengthSquared(a: Vec2) f32 {
        return a.dot(a);
    }

    pub fn length(a: Vec2) f32 {
        return @sqrt(a.lengthSquared());
    }

    /// Unit vector in the same direction, or `Degenerate` if `a` is shorter than
    /// `min_length` - the C version returns 0 and leaves the output untouched.
    pub fn normalized(a: Vec2) Degenerate!Vec2 {
        const len = a.length();
        if (len < min_length) return error.Degenerate;
        return a.scale(1.0 / len);
    }
};

pub const Vec3 = struct {
    x: f32,
    y: f32,
    z: f32,

    pub const zero: Vec3 = .{ .x = 0, .y = 0, .z = 0 };

    pub fn init(x: f32, y: f32, z: f32) Vec3 {
        return .{ .x = x, .y = y, .z = z };
    }

    pub fn add(a: Vec3, b: Vec3) Vec3 {
        return .{ .x = a.x + b.x, .y = a.y + b.y, .z = a.z + b.z };
    }

    pub fn sub(a: Vec3, b: Vec3) Vec3 {
        return .{ .x = a.x - b.x, .y = a.y - b.y, .z = a.z - b.z };
    }

    pub fn scale(a: Vec3, s: f32) Vec3 {
        return .{ .x = a.x * s, .y = a.y * s, .z = a.z * s };
    }

    pub fn dot(a: Vec3, b: Vec3) f32 {
        return a.x * b.x + a.y * b.y + a.z * b.z;
    }

    /// Right-handed cross product.
    ///
    /// The C version computes into a temporary "to support in-place operation";
    /// here the arguments are copies, so `a.cross(b)` is safe for any aliasing
    /// by construction. `testdata/math_corpus.txt` keeps both in-place spellings
    /// the C caller could write, and they are checked.
    pub fn cross(a: Vec3, b: Vec3) Vec3 {
        return .{
            .x = a.y * b.z - a.z * b.y,
            .y = a.z * b.x - a.x * b.z,
            .z = a.x * b.y - a.y * b.x,
        };
    }

    pub fn lengthSquared(a: Vec3) f32 {
        return a.dot(a);
    }

    pub fn length(a: Vec3) f32 {
        return @sqrt(a.lengthSquared());
    }

    pub fn normalized(a: Vec3) Degenerate!Vec3 {
        const len = a.length();
        if (len < min_length) return error.Degenerate;
        return a.scale(1.0 / len);
    }
};

pub const Vec4 = struct {
    x: f32,
    y: f32,
    z: f32,
    w: f32,

    pub const zero: Vec4 = .{ .x = 0, .y = 0, .z = 0, .w = 0 };

    pub fn init(x: f32, y: f32, z: f32, w: f32) Vec4 {
        return .{ .x = x, .y = y, .z = z, .w = w };
    }

    pub fn add(a: Vec4, b: Vec4) Vec4 {
        return .{ .x = a.x + b.x, .y = a.y + b.y, .z = a.z + b.z, .w = a.w + b.w };
    }

    pub fn sub(a: Vec4, b: Vec4) Vec4 {
        return .{ .x = a.x - b.x, .y = a.y - b.y, .z = a.z - b.z, .w = a.w - b.w };
    }

    pub fn scale(a: Vec4, s: f32) Vec4 {
        return .{ .x = a.x * s, .y = a.y * s, .z = a.z * s, .w = a.w * s };
    }

    pub fn dot(a: Vec4, b: Vec4) f32 {
        return a.x * b.x + a.y * b.y + a.z * b.z + a.w * b.w;
    }

    pub fn lengthSquared(a: Vec4) f32 {
        return a.dot(a);
    }

    pub fn length(a: Vec4) f32 {
        return @sqrt(a.lengthSquared());
    }

    pub fn normalized(a: Vec4) Degenerate!Vec4 {
        const len = a.length();
        if (len < min_length) return error.Degenerate;
        return a.scale(1.0 / len);
    }
};

// --- tests ------------------------------------------------------------------

/// Whether normalization should succeed is read from the corpus, not written
/// here: if the C threshold ever moves, the port has to move with it, and this
/// makes that a test failure instead of a silent divergence.
fn expectRefusal(comptime V: type, corpus: *const corpus_mod.Corpus, case: []const u8, result: Degenerate!V) !void {
    if (try corpus.one(case) == 0) {
        try std.testing.expectError(error.Degenerate, result);
    } else {
        _ = try result;
    }
}

test "vector: the three types have the layout the C structs have" {
    const corpus = try corpus_mod.Corpus.load();

    try corpus.expectValues("vec2_layout", &.{
        @floatFromInt(@sizeOf(Vec2)), @floatFromInt(@alignOf(Vec2)),
    });
    try corpus.expectValues("vec3_layout", &.{
        @floatFromInt(@sizeOf(Vec3)), @floatFromInt(@alignOf(Vec3)),
    });
    try corpus.expectValues("vec4_layout", &.{
        @floatFromInt(@sizeOf(Vec4)), @floatFromInt(@alignOf(Vec4)),
    });

    try corpus.expectInt("vec2_offset_x", @offsetOf(Vec2, "x"));
    try corpus.expectInt("vec2_offset_y", @offsetOf(Vec2, "y"));
    try corpus.expectInt("vec3_offset_z", @offsetOf(Vec3, "z"));
    try corpus.expectInt("vec4_offset_w", @offsetOf(Vec4, "w"));
}

test "vec2: every operation matches the C answers" {
    const corpus = try corpus_mod.Corpus.load();

    const a = Vec2.init(1.0, 2.0);
    const b = Vec2.init(3.0, -4.0);

    const r = Vec2.init(1.5, -2.5);
    try corpus.expectValues("vec2_init", &.{ r.x, r.y });

    const sum = a.add(b);
    try corpus.expectValues("vec2_add", &.{ sum.x, sum.y });

    const diff = a.sub(b);
    try corpus.expectValues("vec2_subtract", &.{ diff.x, diff.y });

    const scaled = a.scale(-3.0);
    try corpus.expectValues("vec2_scalar_multiply", &.{ scaled.x, scaled.y });

    try corpus.expectValue("vec2_dot", Vec2.dot(a, b));
    try corpus.expectValue("vec2_length", b.length());

    const unit = try b.normalized();
    try corpus.expectValues("vec2_normalize", &.{ unit.x, unit.y });

    const twice = try unit.normalized();
    try corpus.expectValues("vec2_normalize_twice", &.{ twice.x, twice.y });

    // The degenerate case, and the fact that the C version leaves its output
    // alone: `99 99` is what the fixture recorded for the untouched struct.
    try expectRefusal(Vec2, &corpus, "vec2_normalize_zero_ok", Vec2.zero.normalized());
    try corpus.expectValues("vec2_normalize_zero_untouched", &.{ 99.0, 99.0 });
}

test "vec3: every operation matches the C answers" {
    const corpus = try corpus_mod.Corpus.load();

    const a = Vec3.init(1.0, 2.0, 3.0);
    const b = Vec3.init(-4.0, 5.0, 6.0);

    const r = Vec3.init(1.5, -2.5, 3.25);
    try corpus.expectValues("vec3_init", &.{ r.x, r.y, r.z });

    const sum = a.add(b);
    try corpus.expectValues("vec3_add", &.{ sum.x, sum.y, sum.z });

    const diff = a.sub(b);
    try corpus.expectValues("vec3_subtract", &.{ diff.x, diff.y, diff.z });

    const scaled = a.scale(0.5);
    try corpus.expectValues("vec3_scalar_multiply", &.{ scaled.x, scaled.y, scaled.z });

    try corpus.expectValue("vec3_dot", Vec3.dot(a, b));
    try corpus.expectValue("vec3_length", b.length());

    const n = a.cross(b);
    try corpus.expectValues("vec3_cross", &.{ n.x, n.y, n.z });

    // Both in-place spellings the C caller could write. They are the reason the
    // C version uses a temporary, so they are worth pinning - here they are the
    // natural Zig spelling too, since assigning back to a copy is how in-place
    // use looks when values are returned.
    var in_place_a = a;
    in_place_a = in_place_a.cross(b);
    try corpus.expectValues("vec3_cross_in_place_a", &.{ in_place_a.x, in_place_a.y, in_place_a.z });

    const in_place_b = a.cross(b);
    try corpus.expectValues("vec3_cross_in_place_b", &.{ in_place_b.x, in_place_b.y, in_place_b.z });

    const self_cross = a.cross(a);
    try corpus.expectValues("vec3_cross_self", &.{ self_cross.x, self_cross.y, self_cross.z });

    const unit = try b.normalized();
    try corpus.expectValues("vec3_normalize", &.{ unit.x, unit.y, unit.z });

    try expectRefusal(Vec3, &corpus, "vec3_normalize_zero_ok", Vec3.zero.normalized());
    try corpus.expectValues("vec3_normalize_zero_untouched", &.{ 99.0, 99.0, 99.0 });

    // Either side of the 1e-6 threshold, which the corpus fixed for us.
    try expectRefusal(Vec3, &corpus, "vec3_normalize_tiny_ok", Vec3.init(1.0e-7, 0, 0).normalized());
    try expectRefusal(Vec3, &corpus, "vec3_normalize_just_above_ok", Vec3.init(1.0e-3, 0, 0).normalized());
}

test "vec4: every operation matches the C answers" {
    const corpus = try corpus_mod.Corpus.load();

    const a = Vec4.init(1.0, 2.0, 3.0, 4.0);
    const b = Vec4.init(5.0, -6.0, 7.0, 8.0);

    const r = Vec4.init(1.0, 2.0, 3.0, 4.0);
    try corpus.expectValues("vec4_init", &.{ r.x, r.y, r.z, r.w });

    const sum = a.add(b);
    try corpus.expectValues("vec4_add", &.{ sum.x, sum.y, sum.z, sum.w });

    const diff = a.sub(b);
    try corpus.expectValues("vec4_subtract", &.{ diff.x, diff.y, diff.z, diff.w });

    const scaled = a.scale(2.5);
    try corpus.expectValues("vec4_scalar_multiply", &.{ scaled.x, scaled.y, scaled.z, scaled.w });

    try corpus.expectValue("vec4_dot", Vec4.dot(a, b));
    try corpus.expectValue("vec4_length", b.length());

    const unit = try b.normalized();
    try corpus.expectValues("vec4_normalize", &.{ unit.x, unit.y, unit.z, unit.w });
}
