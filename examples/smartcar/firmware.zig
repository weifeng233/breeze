//! CYT2BL3 firmware root: interrupts, entry point, and nothing else.
//!
//! This is the only target-specific file in the example, and it is what
//! `zig build check-targets` compiles for the four Smartcar targets. Everything
//! above it - modules, wiring, board surfaces - is ordinary Zig that also runs on
//! a workstation, which is what `app_test.zig` does.
//!
//! The divide is worth stating plainly, because it is the whole reason the
//! example is split this way: an interrupt handler and a `noreturn` entry point
//! cannot be tested anywhere but on the part, so they are kept as small as
//! possible and everything interesting is moved out from under them.

const breeze = @import("breeze");
const hal = breeze.hal.cortex_m;

const app = @import("app.zig");
const board = @import("board.zig");

/// The application, over this board's I/O surface and this board's HAL.
pub const Smartcar = app.App(board.BoardIo, board.BoardHal);

var sched = Smartcar.Scheduler.init();

// --- interrupts -------------------------------------------------------------
//
// The complete ISR vocabulary of this application: advance time, push a byte,
// raise a flag, complete a gather slot. No module code, no allocation, no
// formatting.
//
// Every interrupt that raises an event flag must run at the **same priority** -
// here that is trivially true, because there is only one of them. A second one
// would have to be given the same NVIC priority: `setFromIsr` is a
// read-modify-write and two flag-raising interrupts that can preempt each other
// lose one of the two flags. See `breeze.kernel.hal`. The two handlers that
// complete gather slots are not affected: `Join.Of` gives each branch its own
// word.

export fn SysTick_Handler() callconv(.c) void {
    hal.tickIsr();
}

export fn UART0_RX_Handler() callconv(.c) void {
    _ = board.uart_rx.pushFromIsr(0x00); // uart0.rx.read()
    sched.events.setFromIsr(app.EVT_UART_RX);
}

/// The IMU is up.
///
/// If this fires before `Boot` has started an attempt, the slot is not running
/// and the completion is refused - the join then times out and retries, which is
/// the right answer for a peripheral that answered before it was asked.
export fn IMU_INT_Handler() callconv(.c) void {
    Smartcar.boot_state.join.finish(Smartcar.Boot.IMU, 0); // imu0.whoAmI()
}

/// The ESC's self-test finished.
export fn ESC_READY_Handler() callconv(.c) void {
    Smartcar.boot_state.join.finish(Smartcar.Boot.ESC, 0); // esc0.selfTestResult()
}

// --- entry point ------------------------------------------------------------

/// Called from the reset handler after `.data`/`.bss` initialisation.
export fn breeze_main() callconv(.c) noreturn {
    hal.init(board.SYSTEM_CLOCK_HZ);

    // The one piece of kernel state the board is allowed to see, handed over
    // once. The board carries it because a module must not reach for the
    // scheduler; the firmware connects it because the firmware is what owns the
    // scheduler.
    board.events = &sched.events;

    Smartcar.initAll();

    while (true) {
        sched.run(); // `wfi` happens inside when nothing is due

        // Refresh what the health beacon will report on its next period. This
        // loop is the only place that can read the scheduler's own counters
        // without the board or a module depending on the kernel's globals.
        board.health_readings = .{
            .worst_late_ms = sched.worstLateness(),
            .resyncs = sched.totalResyncs(),
            .rx_dropped = board.uart_rx.droppedCount(),
        };
    }
}
