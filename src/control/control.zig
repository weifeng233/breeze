//! The control stage: controllers ported from `include/breeze/control/`.
//!
//! In progress - `pid` and `state_feedback` are done; `fuzzy`, `adaptive` and
//! the six `platform/` controllers are not yet.
//!
//! The controllers are stateful like the filters, but they carry more policy:
//! output limits, anti-windup, and in one case a derivative filter that cannot
//! filter (see `pid.zig`). The C null checks become nothing, and what C did
//! silently - a gain the model never receives, an alpha that does not matter -
//! is either made unrepresentable or recorded in the corpus.

const std = @import("std");

pub const pid = @import("pid.zig");
pub const state_feedback = @import("state_feedback.zig");
pub const platform = @import("platform.zig");
pub const omni = @import("omni.zig");
pub const mecanum = @import("mecanum.zig");
pub const adaptive = @import("adaptive.zig");
pub const ackermann = @import("ackermann.zig");

pub const Pid = pid.Pid;
pub const PidKind = pid.Kind;
pub const StateFeedback = state_feedback.StateFeedback;
pub const DifferentialDrive = platform.DifferentialDrive;
pub const DifferentialConfig = platform.DifferentialConfig;
pub const ImuData = platform.ImuData;
pub const OmniDrive = omni.OmniDrive;
pub const OmniKind = omni.OmniKind;
pub const MecanumDrive = mecanum.MecanumDrive;
pub const MecanumWheel = mecanum.Wheel;
pub const AckermannSteering = ackermann.AckermannSteering;
pub const WheelAngles = ackermann.WheelAngles;
pub const Mrac = adaptive.Mrac;
pub const AdaptivePid = adaptive.AdaptivePid;

test {
    std.testing.refAllDecls(@This());
}
