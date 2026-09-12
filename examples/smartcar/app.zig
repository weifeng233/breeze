//! The application: which modules exist, over which I/O surface, in what order.
//!
//! This file is the whole wiring. There is no YAML block, no generator step and
//! no runtime registry to keep in sync - `AppWithHardware` is a comptime
//! function, so the module graph *is* the source, and a mistake in it is a
//! compile error rather than something discovered on the bench.
//!
//! What the compiler checks here, in order:
//!
//! * every name in a module's `depends` is `provides`d by a *different* module
//!   in this list - `Boot` depends on `imu`, which `Imu` provides;
//! * no module lists a name under both `depends` and `provides`, which would be
//!   a self-satisfying typo;
//! * every name in a module's `hardware` appears in the hardware list below -
//!   "i2c0", "uart0", "esc0" and the encoder/motor pins;
//! * no two modules claim to provide the same name, and no two instances share
//!   an `id`;
//! * every instance's `state` is a pointer to that module's own `State` type;
//! * every field named in a `config` literal exists on the module's `Config`.
//!
//! # Why `App` is a function of `Io` and `Hal`
//!
//! Both are comptime parameters, so the *same* wiring below is what
//! `app_test.zig` instantiates over a fake board and a virtual clock, and what
//! `firmware.zig` instantiates over `BoardIo` and the real HAL. The test
//! therefore exercises the real module graph rather than a copy of it - which is
//! the failure mode this arrangement exists to avoid. (The Smartcar template has
//! to mirror its wiring in its own test, because its board pulls in the SeekFree
//! C library; this board is Zig, so there is nothing to mirror.)
//!
//! # Adding a module
//!
//! Write it under `modules/`, add `pub const X = x.X(Io);` and a file-scope
//! `var` for its state, then one line in the list below. If it needs a
//! peripheral, name that too.

const breeze = @import("breeze");

const boot = @import("modules/boot.zig");
const imu = @import("modules/imu.zig");
const chassis = @import("modules/chassis.zig");
const uplink = @import("modules/uplink.zig");

/// Build the application over an I/O surface and a HAL.
pub fn App(comptime Io: type, comptime Hal: type) type {
    return struct {
        const Self = @This();

        /// The modules, instantiated for this I/O surface.
        ///
        /// A module is a function from a surface to a type, so "which board" is
        /// answered once, here, and every module below inherits the answer.
        pub const Boot = boot.Boot(Io);
        pub const Imu = imu.Imu(Io);
        pub const Chassis = chassis.Chassis(Io);
        pub const Uplink = uplink.Uplink(Io);

        // Breeze saves no per-task stack, so module state lives at file scope
        // and the App is handed its address at comptime. That is also why these
        // are `var` and not `const`: the scheduler writes through the pointer.
        pub var boot_state: Boot.State = .{};
        pub var imu_state: Imu.State = .{};
        pub var chassis_state: Chassis.State = .{};
        pub var uplink_state: Uplink.State = .{};

        /// The application.
        ///
        /// Declaration order is the order `initAll` runs in and the order the
        /// scheduler walks. It is kept as authored so that the schedule stays
        /// reviewable.
        pub const graph = breeze.AppWithHardware(.{
            .{
                .id = "boot0",
                .module = Boot,
                .state = &boot_state,
                .config = .{ .timeout_ms = 250, .attempts_max = 3 },
            },
            .{
                .id = "imu0",
                .module = Imu,
                .state = &imu_state,
                .config = .{ .alpha = 0.3 },
            },
            .{
                .id = "chassis0",
                .module = Chassis,
                .state = &chassis_state,
                .config = .{ .counts_per_meter = 1850.0 },
            },
            .{
                .id = "uplink0",
                .module = Uplink,
                .state = &uplink_state,
                // The event bit is wired in here rather than hard-coded in the
                // module: which bit an interrupt raises is a fact about the
                // board, and the module should not know it.
                .config = .{ .rx_event = EVT_UART_RX },
            },
        }, &.{
            "i2c0",
            "uart0",
            "esc0",
            "encoder_l",
            "encoder_r",
            "motor_l",
            "motor_r",
        });

        /// The scheduler for this application, over this HAL.
        ///
        /// Deriving it from the graph rather than hand-writing a task table is
        /// what keeps the two from drifting: there is only one list.
        pub const Scheduler = graph.SchedulerFor(Hal);

        /// Run every module's `init`, in declaration order.
        pub fn initAll() void {
            graph.initAll();
        }

        /// Render the module graph. The firmware prints it at boot and the host
        /// test asserts on it, so the two cannot describe different
        /// applications.
        pub fn describe(writer: anytype) !void {
            try graph.describe(writer);
        }

        /// Re-exported so callers do not have to spell `graph`.
        pub const module_count = graph.module_count;
        pub const tasks = graph.tasks;
        /// Comptime, like the kernel's, so that an unknown instance id is a
        /// compile error rather than a runtime index into a table that is not
        /// there.
        pub fn instanceName(comptime i: usize) []const u8 {
            return graph.instanceName(i);
        }
    };
}

/// The event bit the UART interrupt raises.
///
/// Declared by the *application* rather than by `board.zig`, because it is part
/// of the wiring: the uplink is told which bit to consume, and the ISR in
/// `firmware.zig` raises the same constant. One declaration, two users, and no
/// board needs to agree with another board about it.
pub const EVT_UART_RX: u32 = 1 << 0;
