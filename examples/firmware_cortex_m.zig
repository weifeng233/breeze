//! Minimal Cortex-M firmware skeleton - also the CI check that the kernel and
//! the Cortex-M HAL compile for `thumb-freestanding`.
//!
//!     zig build check-targets
//!
//! This file is documentation as much as it is a test: it shows the complete
//! wiring a real project needs, and nothing more.
//!
//! Note that the same task bodies in `examples/scheduler_demo.zig` run here
//! unchanged. Only this shell - clock, entry point, vector names - is
//! target-specific.

const breeze = @import("breeze");
const hal = breeze.hal.cortex_m;
const Tick = breeze.Tick;
const Step = breeze.Step;

// --- events raised by interrupts -------------------------------------------

const EVT_UART_RX: u32 = 1 << 0;

// --- tasks -----------------------------------------------------------------

/// 1 kHz control loop. A plain function: periodic work with no sequencing needs
/// no state machine.
const Control = struct {
    ticks: u32 = 0,
};

fn controlPoll(ctx: *Control, now: Tick, events: u32) Step {
    _ = .{ now, events };
    ctx.ticks +%= 1;
    return .finished;
}

/// A handshake sequence expressed as a comptime program: send, wait up to 20 ms
/// for an ACK, retry.
const Handshake = struct {
    // Fields must precede declarations in a Zig container, so the program
    // counter lives here at the top.
    prog: Prog = .{},
    attempts: u32 = 0,
    acks: u32 = 0,
    give_ups: u32 = 0,

    fn send(ctx: *Handshake) void {
        ctx.attempts +%= 1;
        // uart0.write(REQUEST);
    }
    fn onAck(ctx: *Handshake) void {
        ctx.acks +%= 1;
    }
    fn onGiveUp(ctx: *Handshake) void {
        ctx.give_ups +%= 1;
        // Enter a degraded mode, raise an alarm, or reset the peripheral.
    }
    fn mayRetry(ctx: *Handshake) bool {
        return ctx.attempts < 5;
    }

    const instrs = [_]breeze.Instr(@This()){
        // 0
        .{ .call = send },
        // 1: wait up to 20 ms for an ACK
        .{ .wait_event_timeout = .{ .mask = EVT_UART_RX, .timeout_ms = 20 } },
        // 2: ACK -> success arm at 6
        .{ .branch_event = 6 },
        // 3: no ACK and attempts remain -> retry from 0
        .{ .branch_if = .{ .pred = mayRetry, .target = 0 } },
        // 4: out of attempts
        .{ .call = onGiveUp },
        // 5: leave before the success arm
        .{ .jump = 7 },
        // 6
        .{ .call = onAck },
        // 7
        .finish,
    };
    const Prog = breeze.Program(@This(), &instrs);

    fn poll(ctx: *@This(), now: Tick, events: u32) Step {
        return ctx.prog.poll(ctx, now, events);
    }
};

// Contexts have static lifetime: a context must outlive every scheduler pass,
// and the kernel enforces that by requiring the pointer to be comptime-known.
var control = Control{};
var handshake = Handshake{};

const Sched = breeze.Scheduler(hal, .{
    .{ .name = "control", .period_ms = 1, .ctx = &control, .poll = controlPoll },
    .{ .name = "handshake", .period_ms = 0, .ctx = &handshake, .poll = Handshake.poll },
});

var sched = Sched.init();

// --- interrupt handlers ----------------------------------------------------
//
// The complete ISR contract: advance time, fill a ring, raise a flag. No
// scheduler call, no allocation, no formatting.

export fn SysTick_Handler() callconv(.c) void {
    hal.tickIsr();
}

var uart_rx_ring = hal.RxRing{};

export fn UART0_Handler() callconv(.c) void {
    // _ = uart_rx_ring.pushFromIsr(uart0.data.read());
    sched.events.setFromIsr(EVT_UART_RX);
}

export fn HardFault_Handler() callconv(.c) void {
    // A real project would latch diagnostics here rather than spin silently.
    while (true) {}
}

// --- the superloop ---------------------------------------------------------

/// Called from the reset handler after `.data`/`.bss` initialisation.
export fn breeze_main() callconv(.c) noreturn {
    hal.init(64_000_000); // 64 MHz core -> 1 ms SysTick

    while (true) {
        sched.run(); // runs in thread mode; `wfi` when nothing is due
    }
}
