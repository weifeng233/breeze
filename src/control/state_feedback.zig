//! State feedback and LQR, ported from
//! `include/breeze/control/state_feedback_controller.h`.
//!
//! The C header has two structs. `BreezeStateFeedbackController` holds a gain
//! vector and computes `-K·state + N·reference`; `BreezeLQRController` holds the
//! same gains and forward gain *plus* an `A`/`B`/`Q`/`R` model and Riccati
//! iteration parameters - and its comment says the gain solver "will be
//! implemented in the math module", which now exists. It never was.
//!
//! Two consequences, and they are the whole shape of this port:
//!
//! * **The model fields are gone.** `A`, `B`, `Q`, `R`, `max_iterations` and
//!   `convergence_tol` are stored by the C `Init` and read by nothing. Carrying
//!   them here would let a caller pass a system model and believe a gain was
//!   computed from it, which is worse than not offering the parameters at all:
//!   with them absent, that misunderstanding cannot compile.
//! * **There is one type, not two.** The C `Compute` functions are the same code
//!   over the same fields, so `testdata/math_corpus.txt` records both and the
//!   single implementation here is compared against both. A caller with an
//!   offline-computed LQR gain passes it exactly as a state-feedback gain.
//!
//! Writing the Riccati solver is a feature with its own tests, not a migration:
//! there is no C behaviour to match. The corpus keeps the stored model
//! (`lqr_a00_stored`, `lqr_max_iterations_stored`, ...) so the gap stays visible.
//!
//! The C state dimension is a runtime value capped at 4 by the fixed arrays; here
//! it is a comptime parameter with no ceiling, because the cap was a storage
//! artefact and not a rule about the control law.

const std = @import("std");

const corpus_mod = @import("../math/corpus.zig");

/// Linear state feedback over `states` state variables.
pub fn StateFeedback(comptime states: usize) type {
    if (states == 0) @compileError("state feedback needs at least one state");

    return struct {
        /// Feedback gains, applied with a minus sign.
        k: [states]f32,
        reference: f32 = 0,
        /// Feedforward gain for reference tracking - the C version's `N`.
        feedforward: f32,
        u_min: f32,
        u_max: f32,

        const Self = @This();

        pub fn init(k: [states]f32, feedforward: f32, u_min: f32, u_max: f32) Self {
            return .{ .k = k, .feedforward = feedforward, .u_min = u_min, .u_max = u_max };
        }

        pub fn setReference(self: *Self, reference: f32) void {
            self.reference = reference;
        }

        /// `u = -K·state + N·reference`, clamped to `[u_min, u_max]`.
        pub fn compute(self: Self, state: [states]f32) f32 {
            var u: f32 = 0;
            for (self.k, state) |gain, s| u -= gain * s;
            u += self.feedforward * self.reference;

            if (u > self.u_max) {
                u = self.u_max;
            } else if (u < self.u_min) {
                u = self.u_min;
            }
            return u;
        }
    };
}

// --- tests ------------------------------------------------------------------

test "state feedback: layout and construction match the C answers" {
    const corpus = try corpus_mod.Corpus.load();

    // The C struct also carries a runtime state_dim and the same six fields the
    // Zig type has; the corpus records its size for the record, not as a target.
    try corpus.expectValues("statefb_layout", &.{ 36, 4 });

    var f = StateFeedback(2).init(.{ 1.5, 0.5 }, 0.5, -5.0, 5.0);
    try corpus.expectValue("statefb_k0", f.k[0]);
    try corpus.expectValue("statefb_k1", f.k[1]);
    try corpus.expectValue("statefb_reference_init", f.reference);
    try corpus.expectValue("statefb_n_init", f.feedforward);

    try corpus.expectValue("statefb_compute", f.compute(.{ 0.2, -0.3 }));

    f.setReference(2.0);
    try corpus.expectValue("statefb_compute_with_reference", f.compute(.{ 0.2, -0.3 }));

    f.setReference(100.0);
    try corpus.expectValue("statefb_compute_clamped_high", f.compute(.{ 0.2, -0.3 }));
    f.setReference(-100.0);
    try corpus.expectValue("statefb_compute_clamped_low", f.compute(.{ 0.2, -0.3 }));

    const wide = StateFeedback(4).init(.{ 1.0, 2.0, 3.0, 4.0 }, 1.0, -100.0, 100.0);
    try corpus.expectValue("statefb4_compute", wide.compute(.{ 0.1, 0.2, 0.3, 0.4 }));
}

test "lqr: the C model fields are stored and never read; the gains are the API" {
    const corpus = try corpus_mod.Corpus.load();

    // What the C `BreezeLQRController_Init` leaves behind. None of it feeds a
    // computation - there is no Riccati solver - which is why the Zig type has
    // nowhere to put it.
    try corpus.expectValues("lqr_layout", &.{ 144, 4 });
    try corpus.expectValue("lqr_k0_after_init", 0.0);
    try corpus.expectValue("lqr_k1_after_init", 0.0);
    try corpus.expectValue("lqr_a00_stored", 1.0);
    try corpus.expectValue("lqr_a01_stored", 1.0);
    try corpus.expectValue("lqr_b1_stored", 1.0);
    try corpus.expectValue("lqr_q0_stored", 1.0);
    try corpus.expectValue("lqr_r_stored", 0.1);
    try corpus.expectValue("lqr_max_iterations_stored", 100.0);
    try corpus.expectValue("lqr_convergence_tol_stored", 1.0e-4);

    // With gains supplied, the LQR computes what state feedback computes - the
    // corpus records both, and one implementation here answers both.
    var lqr = StateFeedback(2).init(.{ 1.5, 0.5 }, 0.5, -5.0, 5.0);
    try corpus.expectValue("lqr_compute", lqr.compute(.{ 0.2, -0.3 }));
    lqr.setReference(2.0);
    try corpus.expectValue("lqr_compute_with_reference", lqr.compute(.{ 0.2, -0.3 }));
}

test "state feedback: the control law behaves like a control law" {
    // Independent of the corpus. Zero state and zero reference gives no effort;
    // a state in the direction of the gains is pushed back towards zero.
    var f = StateFeedback(2).init(.{ 2.0, 1.0 }, 0.0, -100.0, 100.0);
    try std.testing.expectEqual(@as(f32, 0.0), f.compute(.{ 0.0, 0.0 }));
    try std.testing.expect(f.compute(.{ 1.0, 1.0 }) < 0.0);
    try std.testing.expect(f.compute(.{ -1.0, -1.0 }) > 0.0);

    // The feedforward term is what lets a nonzero reference produce effort.
    f.setReference(3.0);
    var g = StateFeedback(2).init(.{ 2.0, 1.0 }, 0.5, -100.0, 100.0);
    g.setReference(3.0);
    try std.testing.expectApproxEqAbs(@as(f32, 1.5), g.compute(.{ 0.0, 0.0 }), 1.0e-6);

    // A sign flip in a gain flips that term's contribution.
    const positive = StateFeedback(1).init(.{1.0}, 0.0, -10.0, 10.0);
    const negative = StateFeedback(1).init(.{-1.0}, 0.0, -10.0, 10.0);
    try std.testing.expectEqual(-positive.compute(.{2.0}), negative.compute(.{2.0}));
}
