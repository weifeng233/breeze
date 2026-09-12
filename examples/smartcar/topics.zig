//! The wire vocabulary this application speaks.
//!
//! Topics are declared once here rather than next to whichever module happens
//! to produce them, because they are a contract *between* modules: the uplink
//! does not care who publishes `health`, and the module that publishes it does
//! not care who reads it. A topic defined inside its producer would make the
//! consumer import the producer to name a payload type.
//!
//! `breeze.Topic` folds the name to a CRC32 at compile time, and the frame
//! layout is LibXR-compatible - see `docs/FUSION.md` §4. Nothing here is
//! instantiated at run time, and nothing here costs RAM.

const breeze = @import("breeze");

/// Attitude, published at the IMU rate.
pub const Attitude = breeze.Topic("attitude", extern struct {
    roll: f32,
    pitch: f32,
    yaw: f32,
});

/// Encoder counts, published by the chassis module.
pub const Encoders = breeze.Topic("encoders", extern struct {
    left: i32,
    right: i32,
    tick_ms: u32,
});

/// Kernel health, published slowly so a bench session can see stalls.
pub const Health = breeze.Topic("health", extern struct {
    worst_late_ms: u32,
    resyncs: u32,
    rx_dropped: u32,
});
