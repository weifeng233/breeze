//! A recording platform for the controller tests.
//!
//! Every platform module needs the same thing: something that records the motor
//! commands it is given and replays a scripted encoder sequence, so a run can be
//! compared against what the C version commanded. This is that one thing - the
//! first three modules each grew their own copy before it was factored out.
//!
//! The encoder script is keyed by encoder id (1..4) and by *pass index*, which
//! the test sets with `startPass` before each `update`. A version that advanced a
//! counter inside `readEncoder` instead would depend on how many wheels a
//! platform reads, and the platforms read three, four or one.

const std = @import("std");

pub const Recording = struct {
    pub const Call = struct { motor_id: i32, speed: f32 };

    /// Counts chosen so the wheel speeds land near the target: one count is
    /// `2*pi*r/resolution/dt` = 0.01885 m/s, so ~26 counts is ~0.5 m/s. Values
    /// that saturate the PID would only exercise the clamp.
    const seq = [4][3]f32{
        .{ 25.0, 28.0, 24.0 },
        .{ 27.0, 26.0, 30.0 },
        .{ 26.0, 25.0, 29.0 },
        .{ 28.0, 27.0, 25.0 },
    };

    pub var log: [16]Call = undefined;
    pub var log_len: usize = 0;
    pub var pass: usize = 0;
    /// Every `reset` flag passed, in call order. Keeping only the last one hides
    /// a change to the other reads - which is exactly what an earlier version did
    /// (REVIEW §35).
    var resets: [16]bool = undefined;
    var resets_len: usize = 0;

    pub fn setMotor(motor_id: i32, speed: f32) void {
        if (log_len < log.len) {
            log[log_len] = .{ .motor_id = motor_id, .speed = speed };
            log_len += 1;
        }
    }

    pub fn readEncoder(encoder_id: i32, reset: bool) f32 {
        if (resets_len < resets.len) {
            resets[resets_len] = reset;
            resets_len += 1;
        }
        const idx: usize = if (encoder_id >= 1 and encoder_id <= 4) @intCast(encoder_id - 1) else 0;
        return seq[idx][pass % 3];
    }

    /// Start a control pass: clear the log and choose which encoder samples the
    /// platform will report.
    pub fn startPass(pass_index: usize) void {
        log_len = 0;
        resets_len = 0;
        pass = pass_index;
    }

    /// True only if every read so far asked for a reset.
    pub fn allResets() bool {
        for (resets[0..resets_len]) |r| {
            if (!r) return false;
        }
        return resets_len > 0;
    }

    /// Convenience for a single-pass test, and for a run that only cares about
    /// the first sample.
    pub fn clearLog() void {
        startPass(0);
    }
};
