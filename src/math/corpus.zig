//! Reads `testdata/math_corpus.txt`: the answers the C library gave.
//!
//! ARCHITECTURE.md §8 says each migrated module is checked against the C version
//! it replaces. This file is what makes that a fact rather than an intention.
//! The chain is:
//!
//!   `tools/corpus/gen_math_corpus.c` runs the C code and prints what it
//!   returned; its output is committed as `testdata/math_corpus.txt`;
//!   `tools/check-c.ps1` re-runs the generator and fails if the committed copy
//!   no longer matches it; and the ported modules compare against the file case
//!   by case.
//!
//! So a drifting oracle is a red build, and a test comparing against the wrong
//! numbers cannot pass quietly.
//!
//! There is no allocator here on purpose. A corpus line holds at most four
//! floats, so a fixed table holds the whole file and a lookup is a short linear
//! scan; tests are not where an allocator earns its keep. The file itself is
//! embedded only from `test` blocks, so it costs a firmware nothing.

const std = @import("std");

/// The committed corpus, verbatim.
pub const text = @embedFile("testdata/math_corpus.txt");

/// The widest case in the file: a 4x4 matrix. Raising this is expected as
/// modules are added - the parser says which case overflowed if it is not enough.
pub const max_values = 16;

pub const max_cases = 512;

pub const Entry = struct {
    name: []const u8,
    values: [max_values]f32 = @splat(0),
    len: usize = 0,

    pub fn slice(self: *const Entry) []const f32 {
        return self.values[0..self.len];
    }
};

pub const Corpus = struct {
    entries: [max_cases]Entry = undefined,
    count: usize = 0,

    pub const Error = error{
        TooManyCases,
        TooManyValues,
        NotANumber,
        NoSuchCase,
        ExpectedOneValue,
        ValueCountMismatch,
        ValueMismatch,
        IntMismatch,
    };

    pub fn load() Error!Corpus {
        var self = Corpus{};
        var lines = std.mem.tokenizeScalar(u8, text, '\n');
        while (lines.next()) |raw| {
            const line = std.mem.trim(u8, raw, " \r");
            if (line.len == 0 or line[0] == '#') continue;
            if (self.count == max_cases) return Error.TooManyCases;

            var fields = std.mem.tokenizeScalar(u8, line, ' ');
            var entry = Entry{ .name = fields.next() orelse continue };
            while (fields.next()) |field| {
                if (entry.len == max_values) {
                    std.debug.print(
                        "corpus case '{s}' has more than {d} values; raise Corpus.max_values\n",
                        .{ entry.name, max_values },
                    );
                    return Error.TooManyValues;
                }
                entry.values[entry.len] = std.fmt.parseFloat(f32, field) catch return Error.NotANumber;
                entry.len += 1;
            }
            self.entries[self.count] = entry;
            self.count += 1;
        }
        return self;
    }

    fn find(self: *const Corpus, name: []const u8) ?*const Entry {
        for (self.entries[0..self.count]) |*entry| {
            if (std.mem.eql(u8, entry.name, name)) return entry;
        }
        return null;
    }

    /// Every value the C code produced for `name`.
    pub fn get(self: *const Corpus, name: []const u8) Error![]const f32 {
        const entry = self.find(name) orelse return Error.NoSuchCase;
        return entry.slice();
    }

    /// The single value the C code produced for `name`.
    pub fn one(self: *const Corpus, name: []const u8) Error!f32 {
        const values = try self.get(name);
        if (values.len != 1) return Error.ExpectedOneValue;
        return values[0];
    }

    /// Assert that the port returned exactly what the C code returned.
    pub fn expectValues(self: *const Corpus, name: []const u8, actual: []const f32) Error!void {
        const want = try self.get(name);
        if (want.len != actual.len) {
            std.debug.print(
                "corpus case '{s}': C produced {d} value(s), port produced {d}\n",
                .{ name, want.len, actual.len },
            );
            return Error.ValueCountMismatch;
        }
        for (want, actual, 0..) |w, a, i| try self.expectClose(name, i, w, a);
    }

    pub fn expectValue(self: *const Corpus, name: []const u8, actual: f32) Error!void {
        return self.expectValues(name, &.{actual});
    }

    /// Integer-valued cases: layout sizes, offsets, and the C `int` status codes
    /// (`1` for success, `0` for refusal). These are counts, not measurements, so
    /// they must match exactly.
    pub fn expectInt(self: *const Corpus, name: []const u8, actual: anytype) Error!void {
        const want = try self.one(name);
        const got: f32 = @floatFromInt(actual);
        if (want != got) {
            std.debug.print("corpus case '{s}': C says {d}, port says {d}\n", .{ name, want, got });
            return Error.IntMismatch;
        }
    }

    /// Floats are compared with a tolerance rather than bit-exactly. The port
    /// keeps the C operation order, so the arithmetic agrees to the last bit
    /// today; the tolerance exists because a backend difference in a
    /// transcendental would otherwise read as a porting error. It is tight
    /// enough that a real mistake - a swapped axis, a missing term - cannot hide
    /// inside it.
    pub fn expectClose(_: *const Corpus, name: []const u8, index: usize, want: f32, actual: f32) Error!void {
        // NaN has to be rejected before the comparison, not by it: every
        // comparison with NaN is false, so `|want - actual| > tolerance` is
        // false and a NaN would pass as a match. A probe found this - removing a
        // guard that prevented a 0/0 left every test green.
        if (std.math.isNan(want) or std.math.isNan(actual)) {
            std.debug.print(
                "corpus case '{s}'[{d}]: NaN on one side (C says {d}, port says {d})\n",
                .{ name, index, want, actual },
            );
            return Error.ValueMismatch;
        }

        const scale = @max(@abs(want), @abs(actual));
        const tolerance: f32 = @max(1.0e-6, scale * 1.0e-6);
        if (@abs(want - actual) > tolerance) {
            std.debug.print(
                "corpus case '{s}'[{d}]: C says {d}, port says {d} (tolerance {d})\n",
                .{ name, index, want, actual, tolerance },
            );
            return Error.ValueMismatch;
        }
    }
};

test "the corpus parses, and carries the cases the ports name" {
    const corpus = try Corpus.load();

    try std.testing.expect(corpus.count > 40);
    try std.testing.expectEqual(@as(usize, 3), (try corpus.get("vec3_cross")).len);
    try std.testing.expectEqual(@as(usize, 2), (try corpus.get("vec2_layout")).len);
}

test "a case that does not exist is an error, not a zero" {
    const corpus = try Corpus.load();

    // The failure mode this guards: a renamed case comparing against 0 forever.
    try std.testing.expectError(Corpus.Error.NoSuchCase, corpus.get("vec3_this_case_was_renamed"));
    try std.testing.expectError(Corpus.Error.ExpectedOneValue, corpus.one("vec2_add"));
}
