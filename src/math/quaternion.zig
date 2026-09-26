//! Quaternions and 3D rotations, ported from `include/breeze/math/quaternion.h`.
//!
//! Shape of the port, same as `vector`: values in, values out, so the null
//! checks and out-parameters are gone and `result` can no longer alias an input.
//! Three returns that were `int` status codes become errors or plain values:
//! `Normalize` and `Inverse` refuse a degenerate quaternion with
//! `error.Degenerate`, and `ToEulerZYX`'s three out-parameters become one
//! `EulerZYX` value.
//!
//! Two behaviours are copied rather than improved, because the port has to
//! match the implementation it replaces:
//!
//! * **`fromAxisAngle` returns the identity for a degenerate axis.** "Rotate by
//!   any angle about no axis" has an answer, and the C code gives it; a caller
//!   who needs to distinguish can normalize the axis first and check.
//! * **The two 1e-6 thresholds measure different things.**
//!   `normalized` compares the *magnitude* against 1e-6; `inverse` compares the
//!   *squared* magnitude against the same number. A quaternion of magnitude 1e-4
//!   is therefore normalized happily and refused inversion. The corpus has
//!   cases on both sides of that boundary (`quat_normalize_boundary_ok` vs
//!   `quat_inverse_boundary_ok`), so the asymmetry is pinned by data rather than
//!   described in a comment.
//!
//! `slerp` clamps `t` to `[0, 1]`, flips `b` when the dot product is negative
//! (shortest path), uses linear ratios when the inputs are within 0.9999 of each
//! other, and normalizes the result - ignoring a refusal, which the port also
//! does (`catch out`) because a result that cannot be normalized is still the
//! answer the C code returns.

const std = @import("std");

const corpus_mod = @import("corpus.zig");
const vector = @import("vector.zig");

const Vec3 = vector.Vec3;

/// Returning this means a quaternion was too close to zero to divide by.
pub const Degenerate = vector.Degenerate;

/// Magnitude below which `normalized` refuses. Copied, not chosen.
pub const min_magnitude: f32 = 1.0e-6;

/// *Squared* magnitude below which `inverse` refuses. The same number as above,
/// applied to a different quantity - the C code's choice, kept deliberately.
pub const min_magnitude_squared: f32 = 1.0e-6;

/// Roll, pitch and yaw in radians, ZYX order.
pub const EulerZYX = struct {
    roll: f32,
    pitch: f32,
    yaw: f32,
};

pub const Quat = struct {
    /// Real part.
    w: f32,
    x: f32,
    y: f32,
    z: f32,

    pub const identity: Quat = .{ .w = 1, .x = 0, .y = 0, .z = 0 };

    pub fn init(w: f32, x: f32, y: f32, z: f32) Quat {
        return .{ .w = w, .x = x, .y = y, .z = z };
    }

    /// ZYX order: yaw about Z, then pitch about Y, then roll about X.
    pub fn fromEulerZYX(roll: f32, pitch: f32, yaw: f32) Quat {
        const cr = @cos(roll * 0.5);
        const cp = @cos(pitch * 0.5);
        const cy = @cos(yaw * 0.5);

        const sr = @sin(roll * 0.5);
        const sp = @sin(pitch * 0.5);
        const sy = @sin(yaw * 0.5);

        const cpcy = cp * cy;
        const spsy = sp * sy;
        const cpsy = cp * sy;
        const spcy = sp * cy;

        return .{
            .w = cr * cpcy + sr * spsy,
            .x = sr * cpcy - cr * spsy,
            .y = cr * spcy + sr * cpsy,
            .z = cr * cpsy - sr * spcy,
        };
    }

    /// Rotation of `angle` radians about `axis`, which is normalized first.
    ///
    /// An axis too short to normalize yields the identity, as in the C version.
    pub fn fromAxisAngle(axis: Vec3, angle: f32) Quat {
        const unit = axis.normalized() catch return identity;
        const half = angle * 0.5;
        const sin_half = @sin(half);
        return .{
            .w = @cos(half),
            .x = unit.x * sin_half,
            .y = unit.y * sin_half,
            .z = unit.z * sin_half,
        };
    }

    /// The C version's gimbal-lock handling, kept: past `|sin(pitch)| >= 1` the
    /// pitch is pinned to `±pi/2` by the sign of that value, and once
    /// `|sin(pitch)| >= 0.99999` the roll is zero by convention and the yaw comes
    /// from the alternative formula.
    pub fn toEulerZYX(q: Quat) EulerZYX {
        const sinp = 2.0 * (q.w * q.y - q.z * q.x);

        var e: EulerZYX = undefined;
        if (@abs(sinp) >= 1.0) {
            // Typed explicitly: `copysign` takes its result type from the first
            // argument, and a comptime_float there would demand a comptime
            // second argument too.
            const half_pi: f32 = std.math.pi / 2.0;
            e.pitch = std.math.copysign(half_pi, sinp);
        } else {
            e.pitch = std.math.asin(sinp);
        }

        if (@abs(sinp) < 0.99999) {
            const sinr_cosp = 2.0 * (q.w * q.x + q.y * q.z);
            const cosr_cosp = 1.0 - 2.0 * (q.x * q.x + q.y * q.y);
            e.roll = std.math.atan2(sinr_cosp, cosr_cosp);

            const siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
            const cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
            e.yaw = std.math.atan2(siny_cosp, cosy_cosp);
        } else {
            e.roll = 0.0;
            const siny_cosp = 2.0 * (q.x * q.z - q.w * q.y);
            const cosy_cosp = 1.0 - 2.0 * (q.x * q.x + q.z * q.z);
            e.yaw = std.math.atan2(siny_cosp, cosy_cosp);
        }
        return e;
    }

    pub fn magnitude(q: Quat) f32 {
        return @sqrt(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z);
    }

    pub fn normalized(q: Quat) Degenerate!Quat {
        const m = q.magnitude();
        if (m < min_magnitude) return error.Degenerate;
        return .{ .w = q.w / m, .x = q.x / m, .y = q.y / m, .z = q.z / m };
    }

    pub fn conjugate(q: Quat) Quat {
        return .{ .w = q.w, .x = -q.x, .y = -q.y, .z = -q.z };
    }

    /// Conjugate over the squared magnitude.
    ///
    /// Note the threshold: it is `min_magnitude_squared`, not `min_magnitude`.
    pub fn inverse(q: Quat) Degenerate!Quat {
        const m2 = q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z;
        if (m2 < min_magnitude_squared) return error.Degenerate;
        return .{ .w = q.w / m2, .x = -q.x / m2, .y = -q.y / m2, .z = -q.z / m2 };
    }

    /// Hamilton product. The C version multiplies into a temporary so that
    /// `result` may alias an input; values make that safe by construction.
    pub fn mul(a: Quat, b: Quat) Quat {
        return .{
            .w = a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z,
            .x = a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
            .y = a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
            .z = a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w,
        };
    }

    /// `q * v * q*`, which rotates `v` when `q` is a unit quaternion.
    pub fn rotate(q: Quat, v: Vec3) Vec3 {
        const as_quat = Quat{ .w = 0, .x = v.x, .y = v.y, .z = v.z };
        const rotated = q.mul(as_quat).mul(q.conjugate());
        return .{ .x = rotated.x, .y = rotated.y, .z = rotated.z };
    }

    /// Spherical linear interpolation, clamped to `[0, 1]`.
    pub fn slerp(a: Quat, b: Quat, t_in: f32) Quat {
        var t = t_in;
        if (t < 0.0) t = 0.0;
        if (t > 1.0) t = 1.0;

        var cos_half_theta = a.w * b.w + a.x * b.x + a.y * b.y + a.z * b.z;

        // A negative dot means the two quaternions point the long way round;
        // negating b takes the short way without changing the rotation.
        var qb = b;
        if (cos_half_theta < 0.0) {
            qb = .{ .w = -b.w, .x = -b.x, .y = -b.y, .z = -b.z };
            cos_half_theta = -cos_half_theta;
        }

        var ratio_a: f32 = undefined;
        var ratio_b: f32 = undefined;
        if (cos_half_theta > 0.9999) {
            ratio_a = 1.0 - t;
            ratio_b = t;
        } else {
            const half_theta = std.math.acos(cos_half_theta);
            const sin_half_theta = @sin(half_theta);
            ratio_a = @sin((1.0 - t) * half_theta) / sin_half_theta;
            ratio_b = @sin(t * half_theta) / sin_half_theta;
        }

        const out = Quat{
            .w = ratio_a * a.w + ratio_b * qb.w,
            .x = ratio_a * a.x + ratio_b * qb.x,
            .y = ratio_a * a.y + ratio_b * qb.y,
            .z = ratio_a * a.z + ratio_b * qb.z,
        };

        // The C version normalizes here and ignores a refusal, leaving the
        // unnormalized result in place. Same answer, spelled as a fallback.
        return out.normalized() catch out;
    }
};

// --- tests ------------------------------------------------------------------

const axis_x = Vec3.init(1, 0, 0);
const axis_y = Vec3.init(0, 1, 0);
const axis_z = Vec3.init(0, 0, 1);

test "quaternion: layout matches the C struct" {
    const corpus = try corpus_mod.Corpus.load();
    try corpus.expectValues("quat_layout", &.{
        @floatFromInt(@sizeOf(Quat)), @floatFromInt(@alignOf(Quat)),
    });
}

test "quaternion: construction matches the C answers" {
    const corpus = try corpus_mod.Corpus.load();

    const id = Quat.identity;
    try corpus.expectValues("quat_identity", &.{ id.w, id.x, id.y, id.z });

    const q = Quat.init(1.5, -2.5, 3.25, -4.75);
    try corpus.expectValues("quat_init", &.{ q.w, q.x, q.y, q.z });

    const e = Quat.fromEulerZYX(0.3, -0.4, 1.1);
    try corpus.expectValues("quat_from_euler_zyx", &.{ e.w, e.x, e.y, e.z });

    const zero = Quat.fromEulerZYX(0, 0, 0);
    try corpus.expectValues("quat_from_euler_zyx_zero", &.{ zero.w, zero.x, zero.y, zero.z });

    const aa = Quat.fromAxisAngle(Vec3.init(1, 2, 2), 0.7);
    try corpus.expectValues("quat_from_axis_angle", &.{ aa.w, aa.x, aa.y, aa.z });

    // Degenerate axis: identity, because "about no axis" has an answer.
    const degenerate = Quat.fromAxisAngle(Vec3.zero, 0.7);
    try corpus.expectValues("quat_from_axis_angle_degenerate", &.{
        degenerate.w, degenerate.x, degenerate.y, degenerate.z,
    });

    const x60 = Quat.fromAxisAngle(axis_x, std.math.pi / 3.0);
    try corpus.expectValues("quat_from_axis_angle_x", &.{ x60.w, x60.x, x60.y, x60.z });
}

test "quaternion: euler conversion matches the C answers, gimbal lock included" {
    const corpus = try corpus_mod.Corpus.load();

    const round_trip = Quat.fromEulerZYX(0.3, -0.4, 1.1).toEulerZYX();
    try corpus.expectValues("quat_euler_roundtrip", &.{
        round_trip.roll, round_trip.pitch, round_trip.yaw,
    });

    // pitch = pi/2: the branch where the C code pins pitch, sets roll to zero by
    // convention and takes yaw from a different formula.
    const gimbal = Quat.fromEulerZYX(0.3, std.math.pi / 2.0, 1.1);
    try corpus.expectValues("quat_from_euler_zyx_gimbal", &.{
        gimbal.w, gimbal.x, gimbal.y, gimbal.z,
    });

    const gimbal_euler = gimbal.toEulerZYX();
    try corpus.expectValues("quat_euler_gimbal", &.{
        gimbal_euler.roll, gimbal_euler.pitch, gimbal_euler.yaw,
    });
}

test "quaternion: magnitude and normalization match the C answers" {
    const corpus = try corpus_mod.Corpus.load();

    const q = Quat.init(1.5, -2.5, 3.25, -4.75);
    try corpus.expectValue("quat_magnitude", q.magnitude());

    const unit = try q.normalized();
    try corpus.expectValues("quat_normalize", &.{ unit.w, unit.x, unit.y, unit.z });

    // The C code leaves its output untouched when it refuses; the port returns
    // an error, and the corpus keeps the C case so the refusal itself is checked.
    try corpus.expectInt("quat_normalize_degenerate_ok", 0);
    try corpus.expectValues("quat_normalize_degenerate_untouched", &.{ 9, 9, 9, 9 });
    try std.testing.expectError(error.Degenerate, Quat.init(1.0e-7, 0, 0, 0).normalized());
}

test "quaternion: the two 1e-6 thresholds are not the same threshold" {
    const corpus = try corpus_mod.Corpus.load();

    // Magnitude 1e-4: squared magnitude is 1e-8, below the inverse threshold and
    // above the normalize one. The C code accepts one and refuses the other, and
    // the corpus records exactly that pair.
    try corpus.expectInt("quat_normalize_boundary_ok", 1);
    try corpus.expectInt("quat_inverse_boundary_ok", 0);

    const tiny = Quat.init(1.0e-4, 0, 0, 0);
    _ = try tiny.normalized();
    try std.testing.expectError(error.Degenerate, tiny.inverse());

    // Both refuse a quaternion of magnitude 1e-7, for different reasons.
    try corpus.expectInt("quat_inverse_degenerate_ok", 0);
    const negligible = Quat.init(1.0e-7, 0, 0, 0);
    try std.testing.expectError(error.Degenerate, negligible.normalized());
    try std.testing.expectError(error.Degenerate, negligible.inverse());
}

test "quaternion: conjugate and inverse match the C answers" {
    const corpus = try corpus_mod.Corpus.load();

    const q = Quat.init(1.5, -2.5, 3.25, -4.75);
    const c = q.conjugate();
    try corpus.expectValues("quat_conjugate", &.{ c.w, c.x, c.y, c.z });

    const inv = try q.inverse();
    try corpus.expectValues("quat_inverse", &.{ inv.w, inv.x, inv.y, inv.z });

    // Independent of the corpus: a unit quaternion's inverse is its conjugate,
    // and the product is the identity.
    const unit = try q.normalized();
    const product = unit.mul(try unit.inverse());
    try std.testing.expectApproxEqAbs(@as(f32, 1.0), product.w, 1.0e-6);
    try std.testing.expectApproxEqAbs(@as(f32, 0.0), product.x, 1.0e-6);
    try std.testing.expectApproxEqAbs(@as(f32, 0.0), product.y, 1.0e-6);
    try std.testing.expectApproxEqAbs(@as(f32, 0.0), product.z, 1.0e-6);
}

test "quaternion: multiplication and rotation match the C answers" {
    const corpus = try corpus_mod.Corpus.load();

    const z90 = Quat.fromAxisAngle(axis_z, std.math.pi / 2.0);
    const x90 = Quat.fromAxisAngle(axis_x, std.math.pi / 2.0);
    try corpus.expectValues("quat_z90", &.{ z90.w, z90.x, z90.y, z90.z });
    try corpus.expectValues("quat_x90", &.{ x90.w, x90.x, x90.y, x90.z });

    const product = z90.mul(x90);
    try corpus.expectValues("quat_multiply", &.{ product.w, product.x, product.y, product.z });

    var in_place = z90;
    in_place = in_place.mul(x90);
    try corpus.expectValues("quat_multiply_in_place", &.{
        in_place.w, in_place.x, in_place.y, in_place.z,
    });

    const rotated = z90.rotate(axis_x);
    try corpus.expectValues("quat_rotate", &.{ rotated.x, rotated.y, rotated.z });

    // The C caller's in-place spelling; the C version writes its result only at
    // the end, so it is safe there as well.
    const rotated_in_place = z90.rotate(axis_x);
    try corpus.expectValues("quat_rotate_in_place", &.{
        rotated_in_place.x, rotated_in_place.y, rotated_in_place.z,
    });

    // Independent of the corpus: a 90 degrees rotation about Z maps +X to +Y.
    try std.testing.expectApproxEqAbs(@as(f32, 0.0), rotated.x, 1.0e-6);
    try std.testing.expectApproxEqAbs(@as(f32, 1.0), rotated.y, 1.0e-6);
    try std.testing.expectApproxEqAbs(@as(f32, 0.0), rotated.z, 1.0e-6);

    // And rotation preserves length.
    const v = Vec3.init(0.3, -1.2, 2.5);
    const r = z90.rotate(v);
    try std.testing.expectApproxEqAbs(v.length(), r.length(), 1.0e-6);
}

test "quaternion: slerp matches the C answers on every branch" {
    const corpus = try corpus_mod.Corpus.load();

    const z90 = Quat.fromAxisAngle(axis_z, std.math.pi / 2.0);

    const t0 = Quat.slerp(Quat.identity, z90, 0.0);
    try corpus.expectValues("quat_slerp_t0", &.{ t0.w, t0.x, t0.y, t0.z });

    const t025 = Quat.slerp(Quat.identity, z90, 0.25);
    try corpus.expectValues("quat_slerp_t025", &.{ t025.w, t025.x, t025.y, t025.z });

    const t05 = Quat.slerp(Quat.identity, z90, 0.5);
    try corpus.expectValues("quat_slerp_t05", &.{ t05.w, t05.x, t05.y, t05.z });

    const t1 = Quat.slerp(Quat.identity, z90, 1.0);
    try corpus.expectValues("quat_slerp_t1", &.{ t1.w, t1.x, t1.y, t1.z });

    // t outside [0, 1] is clamped, not extrapolated.
    const clamped = Quat.slerp(Quat.identity, z90, 1.5);
    try corpus.expectValues("quat_slerp_clamped", &.{ clamped.w, clamped.x, clamped.y, clamped.z });

    // Identical inputs take the linear branch (dot > 0.9999).
    const linear = Quat.slerp(Quat.identity, Quat.identity, 0.5);
    try corpus.expectValues("quat_slerp_linear_branch", &.{
        linear.w, linear.x, linear.y, linear.z,
    });

    // Negative dot: b is negated to take the short way round, which is the same
    // rotation and lands on the same midpoint as the positive-dot case.
    const negative = Quat.slerp(z90, Quat.init(-1, 0, 0, 0), 0.5);
    try corpus.expectValues("quat_slerp_negative_dot", &.{
        negative.w, negative.x, negative.y, negative.z,
    });

    // Independent of the corpus: the halfway rotation is half the angle.
    const halfway = Quat.fromAxisAngle(axis_z, std.math.pi / 4.0);
    try std.testing.expectApproxEqAbs(halfway.w, t05.w, 1.0e-6);
    try std.testing.expectApproxEqAbs(halfway.z, t05.z, 1.0e-6);

    // A slerp between a rotation and itself is that rotation, anywhere in t.
    const same = Quat.slerp(z90, z90, 0.37);
    try std.testing.expectApproxEqAbs(z90.w, same.w, 1.0e-6);
    try std.testing.expectApproxEqAbs(z90.z, same.z, 1.0e-6);
}
