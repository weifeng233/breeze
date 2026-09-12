//! Board support: the two surfaces this application needs, and the only place
//! that names a pin, a peripheral or a register.
//!
//! There are two surfaces, and keeping them apart is deliberate:
//!
//! * `BoardHal` is what the **kernel** needs - a millisecond clock and a
//!   critical section, plus an optional `idle`. The scheduler takes it as a
//!   comptime type parameter, so these inline to direct calls.
//! * `BoardIo` is what a **module** needs - read the IMU, drive a motor, put
//!   bytes on the debug link. Modules take *it* as a comptime type parameter
//!   rather than importing this file, which is what lets the same module source
//!   run against the host fixture in `app_test.zig` and against this board in
//!   the firmware.
//!
//! The kernel must work on a board with no UART, and a module must be testable
//! without an interrupt controller. One surface cannot serve both, which is why
//! there are two.
//!
//! # The kernel globals the board is allowed to see
//!
//! `events` and `health` are the two pieces of *kernel* state any of this needs,
//! and the board is the layer that carries them, because a module that reached
//! for the scheduler directly would stop being testable. The firmware connects
//! them: it hands over the scheduler's flag word once at start-up and refreshes
//! the health readings once per pass. Nothing else writes either.

const breeze = @import("breeze");
const hal = breeze.hal.cortex_m;

const imu = @import("modules/imu.zig");
const uplink = @import("modules/uplink.zig");

const Tick = breeze.Tick;

// --- what this board is -----------------------------------------------------

/// CYT2BL3 runs at 160 MHz.
pub const SYSTEM_CLOCK_HZ: u32 = 160_000_000;

// --- shared storage ---------------------------------------------------------
//
// Storage is declared at file scope so that the RAM cost of every buffer this
// application owns is visible in one place and known at compile time. This is
// the main deliberate difference from LibXR, whose SPSCQueue allocates in its
// constructor.

/// Bytes the UART ISR has received and the uplink has not read yet.
var uart_rx_storage: [64]u8 = undefined;
pub var uart_rx = breeze.CountedChannel(u8, 63).init(&uart_rx_storage);

/// Outgoing frames. The task produces, the transmit interrupt consumes - the
/// opposite direction from `uart_rx`, and equally legal: a ring needs one
/// producer and one consumer, not one *ISR*.
var tx_storage: [64]u8 = undefined;
var tx_queue = breeze.Channel(u8, 63).init(&tx_storage);

/// Where an outgoing frame is assembled. One buffer, reused, because frames are
/// packed and handed to the UART in the same pass.
var frame_buf: [64]u8 = undefined;

// --- kernel globals, connected by the firmware ------------------------------

/// The scheduler's flag word. `breeze_main` sets this before the first pass.
pub var events: ?*breeze.EventFlags = null;

/// Kernel health, refreshed by the firmware once per pass.
///
/// Named `health_readings` rather than `health` so that `BoardIo.health()` can
/// refer to it without the two names colliding.
pub var health_readings: uplink.HealthReading = .{};

// --- surface 1: what the kernel needs ---------------------------------------

/// The HAL, as the kernel sees it.
///
/// A plain namespace rather than a vtable: `Scheduler` takes it as a comptime
/// type parameter, so every call inlines with no indirection to pay for on
/// Cortex-M0.
pub const BoardHal = struct {
    /// Milliseconds since boot, from the SysTick the timebase interrupt advances.
    pub fn now() Tick {
        return hal.now();
    }

    pub fn criticalEnter() void {
        hal.criticalEnter();
    }

    pub fn criticalExit() void {
        hal.criticalExit();
    }

    /// Called by the scheduler when no task was due this pass.
    ///
    /// A task suspended mid-sequence is due on *every* pass, so this is not
    /// reached while one is waiting - see the `idle` contract in
    /// `breeze/kernel/hal.zig`. Deliberate: sleeping there would slow every
    /// multi-pass sequence to the tick rate.
    pub fn idle() void {
        hal.idle();
    }
};

// --- surface 2: what a module needs -----------------------------------------

/// The board's I/O surface, as modules see it.
///
/// Every method here is a *stub with a comment*: this repository ships the
/// kernel, not a motor driver, so the bodies say what a real board would do and
/// return something harmless. That is the honest shape for an example - the
/// structure is real, the registers are not.
pub const BoardIo = struct {
    // --- sensors and actuators ---

    /// One IMU reading, or null when the part has nothing new.
    pub fn readImu() ?imu.Sample {
        // i2c0.readRegs(IMU_ADDR, BURST, &raw); convert to radians.
        return .{ .roll = 0.01, .pitch = -0.02, .yaw = 0.5 };
    }

    /// Encoder counts. Four-sided quadrature decoding happens in hardware, so
    /// this is one register read.
    pub fn readEncoder(id: u8) i32 {
        _ = id;
        return 0;
    }

    /// Motor duty, already clamped by the module to `Config.max_duty`.
    pub fn setMotor(id: u8, duty: i16) void {
        // pwm.setDuty(MOTOR_CHANNEL[id], duty)
        _ = .{ id, duty };
    }

    /// Ask the IMU to start converting. The interrupt it raises completes the
    /// boot gather's `IMU` slot.
    pub fn startImu() void {
        // i2c0.writeReg(IMU_ADDR, CTRL, START);
    }

    /// Ask the ESC to run its self-test. Its ready line completes the `ESC`
    /// slot.
    pub fn startEsc() void {
        // esc0.selfTest();
    }

    // --- the byte channel ---

    /// Move whatever has arrived into `out`, and return how much.
    ///
    /// The board owns this rather than the module because the ring is the
    /// board's: the module gets bytes, not a queue to manage.
    pub fn drainRx(out: []u8) usize {
        var i: usize = 0;
        while (i < out.len) : (i += 1) {
            out[i] = uart_rx.pop() orelse break;
        }
        return i;
    }

    /// Lower event flags the module has finished handling.
    ///
    /// A flag is level-triggered and never clears itself, so a task that does
    /// not clear what it handled is re-run on every pass for ever - see
    /// `breeze.kernel.events`.
    pub fn clearEvents(mask: u32) void {
        if (events) |e| e.clear(BoardHal, mask);
    }

    /// What the kernel has been doing, for the health beacon.
    pub fn health() uplink.HealthReading {
        return health_readings;
    }

    // --- telemetry ---

    /// Frame `value` and hand it to the transmit queue.
    ///
    /// Returns false when the frame did not fit or the queue was full. A dropped
    /// telemetry frame is a diagnosable event, not something to block on.
    ///
    /// The board owns the frame buffer because there is one UART: three modules
    /// packing into three buffers would be 192 bytes of RAM to save a copy.
    pub fn publish(comptime T: type, value: *const T.Value, now: Tick) bool {
        const frame = T.pack(&frame_buf, value, micros(now)) catch return false;
        for (frame) |b| {
            if (!tx_queue.push(b)) return false;
        }
        startTransmit();
        return true;
    }

    fn startTransmit() void {
        // uart0.startTx(tx_queue) - the real driver drains the queue from its
        // transmit interrupt and raises EVT_TX_DONE when it empties.
    }
};

/// Microseconds since boot, as the frame header wants them.
pub fn micros(now: Tick) u64 {
    return @as(u64, now) * 1000;
}
