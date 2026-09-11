//! Kernel-only entry point.
//!
//! Same as `breeze.zig` except that it does **not** reference any HAL backend.
//! Use it when a project supplies its own HAL and does not want the host,
//! Cortex-M and RISC-V backends pulled into its import graph.
//!
//! This exists mainly so that vendoring the kernel into another repository
//! (see `tools/vendor.ps1`) can copy one tree and get a root file that does not
//! dangle references to `hal/*.zig`. It is also the right entry point for a new
//! MCU port that provides `now` / `criticalEnter` / `criticalExit` itself.
//!
//! ```zig
//! const breeze = @import("breeze/breeze_kernel.zig");
//!
//! pub const BoardHal = struct {
//!     pub fn now() breeze.Tick { ... }
//!     pub fn criticalEnter() void { ... }
//!     pub fn criticalExit() void { ... }
//! };
//!
//! const Sched = breeze.Scheduler(BoardHal, .{ ... });
//! ```

const std = @import("std");

// --- kernel ----------------------------------------------------------------

pub const kernel = struct {
    pub const tick = @import("kernel/tick.zig");
    pub const events = @import("kernel/events.zig");
    pub const program = @import("kernel/program.zig");
    pub const scheduler = @import("kernel/scheduler.zig");
    pub const hal_contract = @import("kernel/hal.zig");
    pub const shared = @import("kernel/shared.zig");
    pub const chan = @import("kernel/chan.zig");
    pub const topic = @import("kernel/topic.zig");
};

/// Milliseconds since boot.
pub const Tick = kernel.tick.Tick;

/// Wrap-safe time arithmetic.
pub const time = struct {
    pub const elapsed = kernel.tick.elapsed;
    pub const elapsedSigned = kernel.tick.elapsedSigned;
    pub const reached = kernel.tick.reached;
    pub const after = kernel.tick.after;
    pub const signedDiff = kernel.tick.signedDiff;
};

/// The ISR-to-task event channel.
pub const EventFlags = kernel.events.EventFlags;

/// Result of polling a task.
pub const Step = kernel.program.Step;

/// One instruction of a sequenced task.
pub const Instr = kernel.program.Instr;

/// Compile a task body from a comptime instruction list.
pub const Program = kernel.program.Program;

/// Build a scheduler over a compile-time task table.
pub const Scheduler = kernel.scheduler.Scheduler;

/// Per-task scheduling state, for health monitoring.
pub const TaskState = kernel.scheduler.TaskState;

/// A single-producer / single-consumer ring over caller-provided storage.
pub const Channel = kernel.chan.Channel;

/// A byte channel with loss counting, for serial links.
pub const CountedChannel = kernel.chan.CountedChannel;

/// A compile-time publish/subscribe topic with LibXR-compatible framing.
pub const Topic = kernel.topic.Topic;

/// Telemetry framing constants and checksums.
pub const telemetry = struct {
    pub const packet_prefix = kernel.topic.packet_prefix;
    pub const packet_version = kernel.topic.packet_version;
    pub const header_len = kernel.topic.header_len;
    pub const frame_overhead = kernel.topic.frame_overhead;
    pub const max_payload_len = kernel.topic.max_payload_len;
    pub const crc32 = kernel.topic.crc32;
    pub const crc8 = kernel.topic.crc8;
};

// --- application layer -----------------------------------------------------

/// Assemble an application from a list of module instances.
pub const App = @import("app.zig").App;

/// Assemble an application that also declares its platform peripherals.
pub const AppWithHardware = @import("app.zig").AppWithHardware;

/// What a module declares about itself.
pub const Manifest = @import("app.zig").Manifest;

/// Version of the vendored kernel, mirrored into VENDORED.md by the vendor tool.
pub const version = "0.1.0";

test {
    std.testing.refAllDecls(kernel);
}
