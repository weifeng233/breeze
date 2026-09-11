//! Minimal RISC-V (machine mode) firmware skeleton - also the CI check that the
//! kernel and the RISC-V HAL compile for `riscv32-freestanding`.
//!
//!     zig build check-targets
//!
//! Platform parameters are explicit here because RISC-V has no standard memory
//! map. The values below are the SiFive CLINT layout used by the FE310 and by
//! QEMU's `sifive_e`/`virt` machines.

const breeze = @import("breeze");
const hal = breeze.hal.riscv;
const Tick = breeze.Tick;
const Step = breeze.Step;

const EVT_UART_RX: u32 = 1 << 0;

// --- tasks -----------------------------------------------------------------

const Telemetry = struct {
    frames: u32 = 0,
};

fn telemetryPoll(ctx: *Telemetry, now: Tick, events: u32) Step {
    _ = .{ now, events };
    ctx.frames +%= 1;
    return .finished;
}

const Handshake = struct {
    // Fields must precede declarations in a Zig container.
    prog: Prog = .{},
    attempts: u32 = 0,
    acks: u32 = 0,
    give_ups: u32 = 0,

    fn send(ctx: *Handshake) void {
        ctx.attempts +%= 1;
    }
    fn onAck(ctx: *Handshake) void {
        ctx.acks +%= 1;
    }
    fn onGiveUp(ctx: *Handshake) void {
        ctx.give_ups +%= 1;
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

var telemetry = Telemetry{};
var handshake = Handshake{};
var uart_rx_ring = hal.RxRing{};

const Sched = breeze.Scheduler(hal, .{
    .{ .name = "telemetry", .period_ms = 100, .ctx = &telemetry, .poll = telemetryPoll },
    .{ .name = "handshake", .period_ms = 0, .ctx = &handshake, .poll = Handshake.poll },
});

var sched = Sched.init();

// --- traps and interrupts --------------------------------------------------

/// Machine timer interrupt: the platform timer fired, so advance the timebase
/// and rearm. This is the entire ISR contract.
export fn machineTimerTrap() callconv(.c) void {
    hal.tickIsr();
    hal.rearmTimer();
}

export fn machineExternalTrap() callconv(.c) void {
    // _ = uart_rx_ring.pushFromIsr(uart0.rx.read());
    sched.events.setFromIsr(EVT_UART_RX);
}

/// A trap vector table. `mtvec` requires 4-byte alignment, and in vectored mode
/// one entry per exception cause. Zig's grammar puts `align` before
/// `linksection`.
export const trap_vector_table align(4) linksection(".trapvec") = [_]*const fn () callconv(.c) void{
    machineTimerTrap,
    machineExternalTrap,
    machineTimerTrap,
    machineExternalTrap,
};

// --- the superloop ---------------------------------------------------------

/// Called from the reset handler after `.data`/`.bss` initialisation.
export fn breeze_main() callconv(.c) noreturn {
    hal.configure(.{
        .mtime_addr = 0x0200_BFF8,
        .mtimecmp_addr = 0x0200_4000,
        // 32.768 cycles per millisecond. Not a whole number, so the period
        // alternates between 32 and 33 to average exactly 1 ms; a fixed 32
        // would run 2.34% fast. See `MillisecondGrid`.
        .timer_hz = 32_768,
    });
    hal.init();
    uart_rx_ring.init(); // bind the ring's channel to its own buffer

    // Point mtvec at the table (direct mode) and unmask the sources used.
    const table_addr: usize = @intFromPtr(&trap_vector_table);
    asm volatile ("csrw mtvec, %[addr]"
        :
        : [addr] "r" (table_addr),
    );
    hal.enableTimerInterrupt();
    hal.enableExternalInterrupt();

    while (true) {
        sched.run();
    }
}
