//! Telemetry uplink: drain the UART ring, publish kernel health.
//!
//! This is the module a student actually watches. It consumes the byte channel
//! the UART ISR fills, counts what arrives, and reports the kernel's own health
//! on a slow beacon. Because the ISR only ever pushes bytes and raises a flag,
//! and this task is the only consumer, no lock is needed anywhere on the path.
//!
//! Generic over `Io` like the rest - see `imu.zig`.
//!
//! # Where the health numbers come from
//!
//! `worstLateness` and friends live on the *scheduler*, which is a global the
//! firmware owns. Rather than reach for it (which would make this module
//! untestable and couple it to the kernel's global state), the board is asked:
//! `Io.health()` returns the readings, and the firmware refreshes them once per
//! pass. The host test supplies its own numbers instead of a scheduler.

const breeze = @import("breeze");
const topics = @import("../topics.zig");

const Tick = breeze.Tick;
const Step = breeze.Step;

/// What the board knows about the kernel's own behaviour.
pub const HealthReading = struct {
    worst_late_ms: u32 = 0,
    resyncs: u32 = 0,
    /// Bytes the RX ring dropped because the consumer fell behind.
    rx_dropped: u32 = 0,
};

/// Build the module over an I/O surface.
///
/// `Io` must provide `drainRx(out: []u8) usize`, `clearEvents(mask: u32)`,
/// `health() HealthReading` and
/// `publish(comptime T: type, value: *const T.Value, now: Tick) bool`.
pub fn Uplink(comptime Io: type) type {
    return struct {
        pub const manifest = breeze.Manifest{
            .name = "Uplink",
            .description = "Frames telemetry and drains the UART RX ring",
            .hardware = &.{"uart0"},
            .provides = &.{"telemetry"},
            .publishes = &.{"health"},
            // 0 = polled on every pass. The ring is drained often, and the
            // health beacon below is rate-limited by its own timestamp.
            .period_ms = 0,
            .stack_hint = 128,
        };

        pub const Config = struct {
            /// The event bit the UART ISR raises. The board names it, the
            /// application wires it here, and this module never hard-codes it -
            /// which is what keeps the module free of board knowledge.
            rx_event: u32 = 0,
            /// How often to publish `health`. 0 disables the beacon.
            health_period_ms: u32 = 1000,
        };

        pub const State = struct {
            cfg: Config = .{},
            rx_bytes: u32 = 0,
            bad_frames: u32 = 0,
            last_health_ms: Tick = 0,
            drops: u32 = 0,
        };

        /// Scratch for one drain. Sized by the caller's ring, not by the
        /// module: whoever builds the module knows how much is worth taking in
        /// one pass, and a bigger buffer here would be RAM nobody asked for.
        pub const drain_len = 32;

        pub fn init(self: *State, cfg: Config) void {
            self.cfg = cfg;
            self.rx_bytes = 0;
            self.bad_frames = 0;
            self.last_health_ms = 0;
            self.drops = 0;
        }

        pub fn poll(self: *State, now: Tick, events: u32) Step {
            var scratch: [drain_len]u8 = undefined;
            const n = Io.drainRx(&scratch);
            self.rx_bytes += @intCast(n);
            if (n > 0 and !looksLikeFrame(scratch[0..n])) self.bad_frames += 1;

            // The flag is level-triggered and nothing clears it for us: an
            // un-cleared event re-runs this task on every pass forever. See
            // `breeze.kernel.events`.
            if (self.cfg.rx_event != 0 and (events & self.cfg.rx_event) != 0) {
                Io.clearEvents(self.cfg.rx_event);
            }

            if (self.cfg.health_period_ms != 0 and
                breeze.time.reached(now, self.last_health_ms +% self.cfg.health_period_ms))
            {
                self.last_health_ms = now;
                const h = Io.health();
                const value = topics.Health.Value{
                    .worst_late_ms = h.worst_late_ms,
                    .resyncs = h.resyncs,
                    .rx_dropped = h.rx_dropped,
                };
                if (!Io.publish(topics.Health, &value, now)) self.drops += 1;
            }
            return .finished;
        }

        /// Whether this drain could be the start of a frame.
        ///
        /// Deliberately a *header* check, not a decode: the module counts
        /// drains it cannot make sense of, and a full decode needs to know which
        /// topic it is looking at, which is the reader's business. A partial
        /// header is not accused - the rest of it is still in the ring.
        fn looksLikeFrame(bytes: []const u8) bool {
            if (bytes.len == 0) return true;
            if (bytes[0] != breeze.telemetry.packet_prefix) return false;
            if (bytes.len < breeze.telemetry.header_len) return true;
            return bytes[14] == breeze.telemetry.packet_version;
        }
    };
}
