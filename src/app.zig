//! Composable modules and applications.
//!
//! # What this is for
//!
//! Breeze already had a comptime task table, but no concept of a *module* - a
//! reusable unit that declares what it needs, what it produces, and how often
//! it wants to run. Every project therefore hand-wrote one flat table, which
//! does not compose and cannot be checked.
//!
//! XRobot solves this with a manifest: a YAML block embedded in a C++ comment,
//! parsed by a Python generator that then emits `XRobotMain`. Its modules are
//! constructed as `Module(hw, appmgr, args...)` and wired by string name at
//! runtime.
//!
//! Breeze takes the *idea* and keeps it in the type system instead:
//!
//! ```zig
//! pub const Imu = struct {
//!     pub const manifest = breeze.Manifest{
//!         .name = "Imu",
//!         .description = "Samples the IMU and publishes attitude",
//!         .hardware = &.{"i2c0"},
//!         .provides = &.{"attitude"},
//!         .consumes = &.{},
//!     };
//!
//!     port: *I2c,
//!     rate_hz: u32,
//!     ...
//!     pub fn init(self: *@This(), cfg: Config) void { ... }
//!     pub fn poll(self: *@This(), now: Tick, events: u32) Step { ... }
//! };
//! ```
//!
//! Then an application declares which modules it uses:
//!
//! ```zig
//! var imu0: Imu.State = .{};
//! var motor0: Motor.State = .{};
//!
//! const App = breeze.App(.{
//!     .{ .module = Imu, .state = &imu0, .config = .{ .rate_hz = 200 } },
//!     .{ .module = Motor, .state = &motor0, .config = .{} },
//! });
//! ```
//!
//! The wiring, the dependency check and the scheduler task table are all
//! resolved by the compiler. There is no YAML to keep in sync, no generator to
//! run, and no way to ship a build whose manifest disagrees with its code.
//!
//! # What the compiler checks
//!
//! * every name in `depends` is provided by another module in the app;
//! * every name in `hardware` is declared by the application;
//! * no two modules provide the same name;
//! * every module has the `poll` and (if declared) `init` entry points;
//! * each module's `config` literal names fields that actually exist.
//!
//! The `depends` / `hardware` split is deliberate. A peripheral is not a
//! module: forcing `i2c0` to be wrapped in a fake module just to satisfy a
//! dependency check would add a layer that does nothing. XRobot draws the same
//! line with its `depends` and `required_hardware` lists.
//!
//! # What is deliberately *not* here
//!
//! There is no runtime registry. LibXR needs one because its code generator
//! emits wiring from YAML; Breeze has the wiring in the source, so a registry
//! would be pure overhead. `describe()` is provided for tooling that wants to
//! print the module graph, but it reads comptime data, not a runtime table.

const std = @import("std");
const Tick = @import("kernel/tick.zig").Tick;
const Step = @import("kernel/program.zig").Step;
const Scheduler = @import("kernel/scheduler.zig").Scheduler;

/// What a module declares about itself.
pub const Manifest = struct {
    /// Short module name, for diagnostics and for `zig build manifest`.
    name: []const u8,
    /// One-line description.
    description: []const u8 = "",
    /// Names of *other modules* this one needs. Checked against the `provides`
    /// of every other module in the application.
    depends: []const []const u8 = &.{},
    /// Names of *platform peripherals* this one needs, e.g. "i2c0" or "uart0".
    /// Checked against the hardware list the application declares.
    ///
    /// The split between `depends` and `hardware` matters: a peripheral is not
    /// a module, and conflating them means every driver has to be wrapped in a
    /// fake module just to satisfy the dependency check. XRobot draws the same
    /// line with its `depends` / `required_hardware` pair.
    hardware: []const []const u8 = &.{},
    /// Names this module offers to other modules.
    provides: []const []const u8 = &.{},
    /// Topics this module publishes.
    publishes: []const []const u8 = &.{},
    /// How often the module wants to run, in milliseconds. 0 means "every
    /// scheduler pass", which suits event-driven modules.
    period_ms: u32 = 0,
    /// Extra stack the module needs beyond the caller's, in bytes. Purely
    /// documentary: Breeze saves no stack, so this is the deepest call the
    /// module makes, used by `zig build manifest` to size the main stack.
    stack_hint: u32 = 0,
};

/// One entry in an application's module list.
///
/// Declared as documentation of the expected shape; `App` validates the literal
/// structurally so that callers can also supply an `id`.
///
/// ```zig
/// .{ .id = "imu0", .module = Imu, .state = &imu0_state, .config = .{...} }
/// ```
pub const instance_doc =
    "An instance literal is `.{ .module = M, .state = &static_state, .config = ... }` " ++
    "with an optional `.id = \"name\"`.";

/// An application with no declared platform hardware.
///
/// Use this when no module lists `hardware` requirements, which is the common
/// case for host-testable logic.
pub fn App(comptime decls: anytype) type {
    return AppWithHardware(decls, &.{});
}

/// An application: an ordered list of module instances, the platform
/// peripherals they are allowed to require, and a scheduler.
///
/// ```zig
/// const App = breeze.AppWithHardware(.{
///     .{ .module = Imu, .state = &imu_state, .config = .{ .alpha = 0.3 } },
/// }, &.{ "i2c0", "uart0" });
/// ```
pub fn AppWithHardware(comptime decls: anytype, comptime hardware: []const []const u8) type {
    comptime validateTop(decls, hardware);
    comptime validateDepends(decls);
    comptime validateProvides(decls);
    comptime validateHardware(decls, hardware);

    return struct {
        const Self = @This();

        /// Number of module instances.
        pub const module_count = decls.len;

        /// Peripherals this application declares as available.
        pub const hardware_names = hardware;

        /// The tasks this application runs, ready for `Scheduler`.
        ///
        /// Ordering follows the declaration order, which is what makes the
        /// schedule deterministic and reviewable.
        pub const tasks = buildTaskTable(decls);

        /// A `Scheduler` type over the given HAL.
        pub fn SchedulerFor(comptime Hal: type) type {
            return Scheduler(Hal, tasks);
        }

        /// Comptime name of instance `i`.
        pub fn instanceName(comptime i: usize) []const u8 {
            return if (@hasField(@TypeOf(decls[i]), "id"))
                decls[i].id
            else
                decls[i].module.manifest.name;
        }

        /// Comptime manifest of instance `i`.
        pub fn manifestOf(comptime i: usize) Manifest {
            return decls[i].module.manifest;
        }

        /// Run every module's `init`, in declaration order.
        ///
        /// Initialisation is not a task: it runs once, before the scheduler
        /// starts, in a plain function with a normal stack. That keeps the
        /// awkward "first pass is different" case out of the task bodies.
        pub fn initAll() void {
            inline for (decls) |decl| {
                if (@hasDecl(decl.module, "init")) {
                    if (!@hasDecl(decl.module, "Config")) {
                        @compileError(decl.module.manifest.name ++
                            " declares `init` but no `pub const Config`");
                    }
                    decl.module.init(
                        decl.state,
                        coerceConfig(decl.module.Config, decl.config, decl.module.manifest.name),
                    );
                }
            }
        }

        /// Total `stack_hint` across modules, for sizing the main stack.
        pub fn stackHint() u32 {
            var total: u32 = 0;
            inline for (decls) |decl| {
                total += decl.module.manifest.stack_hint;
            }
            return total;
        }

        /// Render the module graph as text. For build tooling, not runtime.
        ///
        /// The writer is copied into a local so that writers whose `print`
        /// takes `*Self` work: a function parameter is immutable in Zig, and
        /// `&writer` on a parameter would be a const pointer.
        pub fn describe(writer: anytype) !void {
            var w = writer;
            try w.print("application: {d} module(s)\n", .{module_count});
            inline for (decls, 0..) |decl, i| {
                const m = decl.module.manifest;
                try w.print(
                    "  [{d}] {s} ({s})  period={d}ms\n",
                    .{ i, instanceName(i), m.name, m.period_ms },
                );
                if (m.description.len != 0) {
                    try w.print("        {s}\n", .{m.description});
                }
                if (m.depends.len != 0) {
                    try w.print("        depends:", .{});
                    for (m.depends) |r| try w.print(" {s}", .{r});
                    try w.print("\n", .{});
                }
                if (m.hardware.len != 0) {
                    try w.print("        hardware:", .{});
                    for (m.hardware) |h| try w.print(" {s}", .{h});
                    try w.print("\n", .{});
                }
                if (m.provides.len != 0) {
                    try w.print("        provides:", .{});
                    for (m.provides) |p| try w.print(" {s}", .{p});
                    try w.print("\n", .{});
                }
                if (m.publishes.len != 0) {
                    try w.print("        publishes:", .{});
                    for (m.publishes) |p| try w.print(" {s}", .{p});
                    try w.print("\n", .{});
                }
            }
        }
    };
}

/// What every module type must expose.
fn validateModule(comptime Module: type, comptime decl: anytype, comptime i: usize) void {
    // The `hasDecl` checks must precede any read of `Module.manifest`, so the
    // location string is built without it.
    if (!@hasDecl(Module, "manifest")) {
        @compileError(std.fmt.comptimePrint(
            "module {d} ('{s}') is missing `pub const manifest: breeze.Manifest`",
            .{ i, @typeName(Module) },
        ));
    }

    const where = std.fmt.comptimePrint("module {d} ('{s}')", .{ i, Module.manifest.name });

    if (@TypeOf(Module.manifest) != Manifest) {
        @compileError(where ++ ".manifest must be a `breeze.Manifest`");
    }
    if (!@hasDecl(Module, "State")) {
        @compileError(where ++ " is missing `pub const State = struct { ... }`");
    }
    if (!@hasDecl(Module, "poll")) {
        @compileError(where ++ " is missing `pub fn poll(self: *State, now: Tick, events: u32) Step`");
    }
    if (!@hasField(@TypeOf(decl), "state")) {
        @compileError(where ++ " instance is missing a `state` pointer");
    }
    const StatePtr = @TypeOf(decl.state);
    if (StatePtr != *Module.State) {
        @compileError(where ++ " instance `state` is " ++ @typeName(StatePtr) ++
            ", expected *" ++ @typeName(Module.State));
    }
}

fn validateTop(comptime decls: anytype, comptime hardware: []const []const u8) void {
    _ = hardware;
    if (decls.len == 0) {
        @compileError("App requires at least one module instance");
    }
    inline for (decls, 0..) |decl, i| {
        if (!@hasField(@TypeOf(decl), "module")) {
            @compileError(std.fmt.comptimePrint("instance {d} is missing `module`", .{i}));
        }
        validateModule(decl.module, decl, i);
        // Duplicate instance names would make telemetry and diagnostics
        // ambiguous, so they are rejected rather than renamed.
        inline for (decls, 0..) |other, j| {
            if (i != j) {
                const a = if (@hasField(@TypeOf(decl), "id")) decl.id else decl.module.manifest.name;
                const b = if (@hasField(@TypeOf(other), "id")) other.id else other.module.manifest.name;
                if (std.mem.eql(u8, a, b)) {
                    @compileError("duplicate module instance name: '" ++ a ++ "'");
                }
            }
        }
    }
}

/// Every `depends` name must be provided by some module in this app.
fn validateDepends(comptime decls: anytype) void {
    inline for (decls, 0..) |decl, i| {
        for (decl.module.manifest.depends) |need| {
            var found = false;
            inline for (decls) |candidate| {
                for (candidate.module.manifest.provides) |offer| {
                    if (std.mem.eql(u8, need, offer)) found = true;
                }
            }
            if (!found) {
                @compileError(std.fmt.comptimePrint(
                    "instance {d} ('{s}') depends on module '{s}', but no module in this " ++
                        "application provides it (did you mean to list it under `hardware`?)",
                    .{ i, instanceNameOf(decl), need },
                ));
            }
        }
    }
}

/// Every `hardware` name must be declared by the application.
fn validateHardware(comptime decls: anytype, comptime hardware: []const []const u8) void {
    inline for (decls, 0..) |decl, i| {
        for (decl.module.manifest.hardware) |need| {
            var found = false;
            for (hardware) |offer| {
                if (std.mem.eql(u8, need, offer)) found = true;
            }
            if (!found) {
                @compileError(std.fmt.comptimePrint(
                    "instance {d} ('{s}') needs hardware '{s}', which this application does " ++
                        "not declare. Add it to the hardware list passed to AppWithHardware.",
                    .{ i, instanceNameOf(decl), need },
                ));
            }
        }
    }
}

/// A module must not claim to provide something another module also provides.
fn validateProvides(comptime decls: anytype) void {
    inline for (decls, 0..) |decl, i| {
        for (decl.module.manifest.provides) |offer| {
            inline for (decls, 0..) |other, j| {
                if (i != j) {
                    for (other.module.manifest.provides) |other_offer| {
                        if (std.mem.eql(u8, offer, other_offer)) {
                            @compileError(std.fmt.comptimePrint(
                                "'{s}' is provided by both instance {d} ('{s}') and instance {d} ('{s}')",
                                .{ offer, i, instanceNameOf(decl), j, instanceNameOf(other) },
                            ));
                        }
                    }
                }
            }
        }
    }
}

fn instanceNameOf(comptime decl: anytype) []const u8 {
    return if (@hasField(@TypeOf(decl), "id")) decl.id else decl.module.manifest.name;
}

/// Build a module's `Config` from an instance's anonymous config literal.
///
/// Zig will not coerce an already-typed anonymous struct value into a named
/// struct type, so the call site would otherwise have to spell out
/// `FakeSensor.Config{ .bias = 5 }` every time. Field-by-field construction
/// keeps the ergonomic form working, applies the module's defaults for anything
/// omitted, and - because the loop is `inline` over the declared fields - lets
/// a typo in the call site be a compile error rather than a silently ignored
/// setting.
fn coerceConfig(comptime Config: type, comptime supplied: anytype, comptime who: []const u8) Config {
    const Supplied = @TypeOf(supplied);

    // A config that is already exactly the right type needs no work.
    if (Supplied == Config) return supplied;

    comptime {
        if (@typeInfo(Supplied) != .@"struct") {
            @compileError(who ++ ": `config` must be a struct literal");
        }
        const decl_fields = @typeInfo(Config).@"struct".fields;
        const given_fields = @typeInfo(Supplied).@"struct".fields;

        // Reject unknown keys: a misspelled `.bais = 5` would otherwise be
        // accepted and silently do nothing.
        for (given_fields) |g| {
            var known = false;
            for (decl_fields) |d| {
                if (std.mem.eql(u8, d.name, g.name)) known = true;
            }
            if (!known) {
                @compileError(std.fmt.comptimePrint(
                    "{s}.Config has no field '{s}'",
                    .{ who, g.name },
                ));
            }
        }
    }

    var cfg: Config = .{};
    inline for (@typeInfo(Config).@"struct".fields) |f| {
        if (@hasField(Supplied, f.name)) {
            @field(cfg, f.name) = @field(supplied, f.name);
        }
    }
    return cfg;
}

/// Turn the module list into the task table `Scheduler` expects.
fn buildTaskTable(comptime decls: anytype) [decls.len]TaskDef {
    var table: [decls.len]TaskDef = undefined;
    for (decls, 0..) |decl, i| {
        table[i] = .{
            .name = instanceNameOf(decl),
            .period_ms = decl.module.manifest.period_ms,
            .ctx = decl.state,
            .poll = makePoll(decl.module),
        };
    }
    return table;
}

/// Adapt a module's `poll(self, now, events)` into the scheduler's
/// `poll(ctx, now, events)`.
///
/// Both have the same shape today, so this is an identity in practice; it
/// exists so the two can diverge without every module having to change.
fn makePoll(comptime Module: type) *const fn (*anyopaque, Tick, u32) Step {
    return struct {
        fn call(ctx: *anyopaque, now: Tick, events: u32) Step {
            const self: *Module.State = @ptrCast(@alignCast(ctx));
            return Module.poll(self, now, events);
        }
    }.call;
}

/// The shape `Scheduler` consumes. Declared structurally rather than as a
/// nominal type so that `Scheduler` and `App` stay independent.
pub const TaskDef = struct {
    name: []const u8,
    period_ms: u32,
    ctx: *anyopaque,
    poll: *const fn (*anyopaque, Tick, u32) Step,
};

// --- tests -----------------------------------------------------------------

/// A provider module: owns a hardware port.
const FakeSensor = struct {
    pub const manifest = Manifest{
        .name = "FakeSensor",
        .description = "Test provider",
        .provides = &.{"sensor"},
        .period_ms = 10,
    };

    pub const State = struct { samples: u32 = 0 };

    pub const Config = struct { bias: i32 = 0 };

    pub fn init(self: *State, cfg: Config) void {
        self.samples = @intCast(@max(cfg.bias, 0));
    }

    pub fn poll(self: *State, now: Tick, events: u32) Step {
        _ = .{ now, events };
        self.samples += 1;
        return .finished;
    }
};

/// A consumer module: depends on the sensor module.
const FakeController = struct {
    pub const manifest = Manifest{
        .name = "FakeController",
        .description = "Test consumer",
        .depends = &.{"sensor"},
        .publishes = &.{"command"},
        .period_ms = 20,
    };

    pub const State = struct { updates: u32 = 0 };

    pub fn poll(self: *State, now: Tick, events: u32) Step {
        _ = .{ now, events };
        self.updates += 1;
        return .finished;
    }
};

/// An event-driven module with no period.
const FakeTelemetry = struct {
    pub const manifest = Manifest{
        .name = "FakeTelemetry",
        .depends = &.{"sensor"},
        .publishes = &.{"attitude"},
        .period_ms = 0,
    };

    pub const State = struct { frames: u32 = 0 };

    pub fn poll(self: *State, now: Tick, events: u32) Step {
        _ = .{ now, events };
        self.frames += 1;
        return .finished;
    }
};

const host = @import("hal/host.zig");

var sensor_state: FakeSensor.State = .{};
var controller_state: FakeController.State = .{};
var telemetry_state: FakeTelemetry.State = .{};

const TestApp = App(.{
    .{ .module = FakeSensor, .state = &sensor_state, .config = .{ .bias = 5 } },
    .{ .module = FakeController, .state = &controller_state },
    .{ .module = FakeTelemetry, .state = &telemetry_state },
});

test "app builds a task table in declaration order" {
    try std.testing.expectEqual(@as(usize, 3), TestApp.module_count);
    try std.testing.expectEqualStrings("FakeSensor", TestApp.instanceName(0));
    try std.testing.expectEqualStrings("FakeController", TestApp.instanceName(1));
    try std.testing.expectEqualStrings("FakeTelemetry", TestApp.instanceName(2));

    try std.testing.expectEqual(@as(u32, 10), TestApp.tasks[0].period_ms);
    try std.testing.expectEqual(@as(u32, 20), TestApp.tasks[1].period_ms);
    try std.testing.expectEqual(@as(u32, 0), TestApp.tasks[2].period_ms);
}

test "an explicit id overrides the manifest name" {
    // FakeSensor has no `depends`, so it stands alone as an application.
    const Named = App(.{
        .{ .id = "left_motor", .module = FakeSensor, .state = &sensor_state },
    });
    try std.testing.expectEqualStrings("left_motor", Named.instanceName(0));
}

test "initAll runs module initialisers with their config" {
    sensor_state.samples = 0;
    TestApp.initAll();
    // FakeSensor.init seeded the counter from cfg.bias.
    try std.testing.expectEqual(@as(u32, 5), sensor_state.samples);
}

test "the app drives a real scheduler" {
    host.HostHal.reset();
    sensor_state.samples = 0;
    controller_state.updates = 0;
    telemetry_state.frames = 0;

    const S = TestApp.SchedulerFor(host.HostHal);
    var sched = S.init();

    host.HostHal.runFor(&sched, 1000);

    // 10 ms module: t=0,10,...,990 -> 100 runs. 20 ms module -> 50.
    try std.testing.expectEqual(@as(u32, 100), sensor_state.samples);
    try std.testing.expectEqual(@as(u32, 50), controller_state.updates);
    // Period 0 runs on every pass; 1000 ticks means 1000 passes.
    try std.testing.expectEqual(@as(u32, 1000), telemetry_state.frames);
    try std.testing.expectEqual(@as(u32, 0), sched.worstLateness());
}

test "manifest data is readable at comptime for tooling" {
    const m = TestApp.manifestOf(1);
    try std.testing.expectEqualStrings("FakeController", m.name);
    try std.testing.expectEqual(@as(usize, 1), m.depends.len);
    try std.testing.expectEqualStrings("sensor", m.depends[0]);
    try std.testing.expectEqualStrings("command", m.publishes[0]);
}

/// Collects text into a fixed buffer, for `describe` tests and for firmware
/// that wants the module graph over a serial link without pulling in a
/// formatting stack.
const BufWriter = struct {
    buf: []u8,
    len: usize = 0,

    const Error = error{NoSpaceLeft};

    fn write(self: *BufWriter, bytes: []const u8) Error!usize {
        if (self.len + bytes.len > self.buf.len) return error.NoSpaceLeft;
        @memcpy(self.buf[self.len..][0..bytes.len], bytes);
        self.len += bytes.len;
        return bytes.len;
    }

    fn written(self: *const BufWriter) []const u8 {
        return self.buf[0..self.len];
    }

    const Writer = struct {
        inner: *BufWriter,

        pub fn print(self: *Writer, comptime fmt: []const u8, args: anytype) !void {
            var scratch: [256]u8 = undefined;
            const text = try std.fmt.bufPrint(&scratch, fmt, args);
            _ = try self.inner.write(text);
        }

        pub fn writeAll(self: *Writer, bytes: []const u8) !void {
            _ = try self.inner.write(bytes);
        }
    };

    fn writer(self: *BufWriter) Writer {
        return .{ .inner = self };
    }
};

test "describe renders the module graph" {
    var buf: [512]u8 = undefined;
    var w = BufWriter{ .buf = &buf };
    try TestApp.describe(w.writer());

    const text = w.written();
    try std.testing.expect(std.mem.indexOf(u8, text, "FakeSensor") != null);
    try std.testing.expect(std.mem.indexOf(u8, text, "depends: sensor") != null);
    try std.testing.expect(std.mem.indexOf(u8, text, "publishes: command") != null);
    try std.testing.expect(std.mem.indexOf(u8, text, "period=20ms") != null);
}

test "stack hint sums across modules" {
    try std.testing.expectEqual(@as(u32, 0), TestApp.stackHint());
}
