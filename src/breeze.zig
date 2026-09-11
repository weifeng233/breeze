//! # Breeze Framework
//!
//! A deterministic, cooperative, allocation-free task kernel for ARM Cortex-M
//! and RISC-V bare-metal targets, plus a virtual-clock host backend that makes
//! the same kernel testable on a workstation.
//!
//! ## What this is
//!
//! The kernel answers one question: *how do several periodic and event-driven
//! jobs share one CPU without an RTOS and without a stack per job?*
//!
//! * `Scheduler` runs a compile-time task table in declaration order.
//! * `Program` turns a comptime instruction list into a resumable state
//!   machine, which is how a task expresses "do A, wait 5 ms, do B, time out
//!   after 50 ms" without blocking and without hand-writing a `switch`.
//! * `EventFlags` is the only channel from interrupt handlers into tasks.
//! * A HAL is three functions: `now`, `criticalEnter`, `criticalExit`.
//!
//! ## What this is not
//!
//! There is no preemption, no priority inversion handling, no dynamic task
//! creation and no heap. If a project needs preemptive scheduling, run this
//! kernel inside one RTOS task and let the RTOS handle the hard real-time work.
//!
//! ## Cost
//!
//! A `Program` costs one `u32` index plus a `Tick` origin and two flags
//! (12 bytes on a 32-bit target) regardless of how many instructions it
//! contains, and it saves no stack. A periodic task with no sequencing needs no
//! state in the kernel at all beyond its `TaskState` (20 bytes).
//!
//! ## Example
//!
//! ```zig
//! const breeze = @import("breeze");
//! const hal = breeze.hal.host;
//!
//! const Ctx = struct { blinks: u32 = 0 };
//!
//! fn toggle(ctx: *Ctx, now: breeze.Tick, events: u32) breeze.Step {
//!     _ = .{ now, events };
//!     ctx.blinks += 1;
//!     return .finished;
//! }
//!
//! var ctx = Ctx{};
//! const S = breeze.Scheduler(hal.HostHal, .{
//!     .{ .name = "blink", .period_ms = 500, .ctx = &ctx, .poll = toggle },
//! });
//!
//! var sched = S.init();
//! hal.HostHal.runFor(&sched, 5000);   // virtual time, no sleeping
//! // ctx.blinks == 10
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

/// Composable modules and applications.
pub const app = @import("app.zig");

/// Milliseconds since boot.
pub const Tick = kernel.tick.Tick;

/// Free functions for wrap-safe time arithmetic.
pub const time = struct {
    pub const elapsed = kernel.tick.elapsed;
    pub const elapsedSigned = kernel.tick.elapsedSigned;
    pub const reached = kernel.tick.reached;
    pub const after = kernel.tick.after;
    pub const signedDiff = kernel.tick.signedDiff;
};

/// The ISR-to-task event channel.
pub const EventFlags = kernel.events.EventFlags;

/// Result of polling a task: still running, or done for this activation.
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

/// Telemetry framing constants.
pub const telemetry = struct {
    pub const packet_prefix = kernel.topic.packet_prefix;
    pub const packet_version = kernel.topic.packet_version;
    pub const header_len = kernel.topic.header_len;
    pub const frame_overhead = kernel.topic.frame_overhead;
    pub const max_payload_len = kernel.topic.max_payload_len;
    pub const crc32 = kernel.topic.crc32;
    pub const crc8 = kernel.topic.crc8;
};

/// Assemble an application from a list of module instances.
pub const App = app.App;

/// Assemble an application that also declares the platform peripherals its
/// modules are allowed to require.
pub const AppWithHardware = app.AppWithHardware;

/// What a module declares about itself.
pub const Manifest = app.Manifest;

// --- hardware abstraction backends -----------------------------------------

pub const hal = struct {
    /// Virtual-clock backend for tests, examples and simulation.
    pub const host = @import("hal/host.zig");

    /// ARM Cortex-M backend. Only analysable when building for `thumb`;
    /// reference it from target code, never from a host test.
    pub const cortex_m = @import("hal/cortex_m.zig");

    /// RISC-V machine-mode backend. Only analysable when building for
    /// `riscv32`/`riscv64`.
    pub const riscv = @import("hal/riscv.zig");
};

// --- version ---------------------------------------------------------------

pub const version = "0.1.0";

// Force analysis of the kernel sources so that `zig build test` collects their
// unit tests. The two target-only HAL backends are deliberately excluded: their
// inline assembly and memory-mapped registers only make sense when building for
// their own architecture, and `zig build check-targets` compiles them there.
test {
    std.testing.refAllDecls(kernel.tick);
    std.testing.refAllDecls(kernel.events);
    std.testing.refAllDecls(kernel.program);
    std.testing.refAllDecls(kernel.scheduler);
    std.testing.refAllDecls(kernel.hal_contract);
    std.testing.refAllDecls(kernel.shared);
    std.testing.refAllDecls(kernel.chan);
    std.testing.refAllDecls(kernel.topic);
    std.testing.refAllDecls(app);
    std.testing.refAllDecls(hal.host);
}
