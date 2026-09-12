const std = @import("std");

pub fn build(b: *std.Build) void {
    const target = b.standardTargetOptions(.{});
    const optimize = b.standardOptimizeOption(.{});

    // The library module. Consumers use it as `@import("breeze")`.
    const breeze = b.addModule("breeze", .{
        .root_source_file = b.path("src/breeze.zig"),
        .target = target,
        .optimize = optimize,
    });

    // --- unit tests --------------------------------------------------------
    const unit_tests = b.addTest(.{ .root_module = breeze });
    const run_unit_tests = b.addRunArtifact(unit_tests);

    const test_step = b.step("test", "Run kernel unit tests and the application tests");
    test_step.dependOn(&run_unit_tests.step);

    // The example application's tests run on the host like any other, because
    // its modules take their I/O surface as a comptime parameter rather than
    // reaching for a peripheral. Without this the example would only ever be
    // *compiled*: a boot sequence that restarted itself on every pass, or a
    // gather that never completed, would be a bench discovery. Both of those
    // happened before this step existed.
    const app_tests = b.addTest(.{
        .root_module = b.createModule(.{
            .root_source_file = b.path("examples/smartcar/app_test.zig"),
            .target = target,
            .optimize = optimize,
            .imports = &.{.{ .name = "breeze", .module = breeze }},
        }),
    });
    const run_app_tests = b.addRunArtifact(app_tests);
    test_step.dependOn(&run_app_tests.step);

    // --- host demo ---------------------------------------------------------
    const demo = b.addExecutable(.{
        .name = "breeze-demo",
        .root_module = b.createModule(.{
            .root_source_file = b.path("examples/scheduler_demo.zig"),
            .target = target,
            .optimize = optimize,
            .imports = &.{.{ .name = "breeze", .module = breeze }},
        }),
    });
    b.installArtifact(demo);

    const run_demo = b.addRunArtifact(demo);
    const demo_step = b.step("demo", "Run the scheduler demo on the host");
    demo_step.dependOn(&run_demo.step);

    // --- freestanding target checks ----------------------------------------
    // The firmware skeletons in examples/ wire the kernel to the Cortex-M and
    // RISC-V HALs and contain inline assembly and memory-mapped registers, so
    // they are compiled for their own architectures rather than for the host.
    // This step is what keeps the target backends honest in CI.
    //
    // The CPU/ABI triples below are not arbitrary: they are exactly the ones
    // the Smartcar-Template Makefiles use, so a change that breaks one of
    // those boards fails here. Compare them with `ZIG_TARGET` / `ZIG_CPU` in
    // `templates/base/<chip>/Makefile` of that repository.
    //
    // `smartcar_rt1064` duplicates `smartcar_cyt4bb7_cm7`'s triple, because
    // RT1064 and the CYT4BB7 CM7 core are both `cortex_m7+fp_armv8d16sp`.
    // Listing it separately costs one more object compile and buys something
    // worth more: the claim "RT1064 is compile-verified" becomes checkable by
    // reading the step's output, instead of resting on the reader noticing
    // that two boards happen to share a triple. Coverage by coincidence is
    // coverage nobody can audit.
    const check_step = b.step("check-targets", "Compile the firmware skeletons for Cortex-M and RISC-V");

    const freestanding_targets = [_]struct {
        name: []const u8,
        example: []const u8,
        query: std.Target.Query,
    }{
        .{
            .name = "cortex_m0",
            .example = "examples/firmware_cortex_m.zig",
            .query = .{
                .cpu_arch = .thumb,
                .os_tag = .freestanding,
                .abi = .eabi,
                .cpu_model = .{ .explicit = &std.Target.arm.cpu.cortex_m0 },
            },
        },
        .{
            .name = "cortex_m4",
            .example = "examples/firmware_cortex_m.zig",
            .query = .{
                .cpu_arch = .thumb,
                .os_tag = .freestanding,
                .abi = .eabi,
                .cpu_model = .{ .explicit = &std.Target.arm.cpu.cortex_m4 },
            },
        },
        .{
            .name = "riscv32",
            .example = "examples/firmware_riscv.zig",
            .query = .{
                .cpu_arch = .riscv32,
                .os_tag = .freestanding,
                .abi = .eabi,
                .cpu_model = .{ .explicit = &std.Target.riscv.cpu.baseline_rv32 },
            },
        },
        // --- the three Smartcar targets, plus the fusion example -----------
        .{
            .name = "smartcar_cyt2bl3",
            .example = "examples/smartcar/firmware.zig",
            .query = .{
                .cpu_arch = .thumb,
                .os_tag = .freestanding,
                .abi = .eabihf,
                .cpu_model = .{ .explicit = &std.Target.arm.cpu.cortex_m4 },
                .cpu_features_add = std.Target.arm.featureSet(&.{.vfp4d16sp}),
            },
        },
        .{
            .name = "smartcar_cyt4bb7_cm0p",
            .example = "examples/smartcar/firmware.zig",
            .query = .{
                .cpu_arch = .thumb,
                .os_tag = .freestanding,
                .abi = .eabi,
                .cpu_model = .{ .explicit = &std.Target.arm.cpu.cortex_m0plus },
            },
        },
        .{
            .name = "smartcar_cyt4bb7_cm7",
            .example = "examples/smartcar/firmware.zig",
            .query = .{
                .cpu_arch = .thumb,
                .os_tag = .freestanding,
                .abi = .eabihf,
                .cpu_model = .{ .explicit = &std.Target.arm.cpu.cortex_m7 },
                .cpu_features_add = std.Target.arm.featureSet(&.{.fp_armv8d16sp}),
            },
        },
        .{
            // Same triple as the entry above; see the comment on check_step.
            .name = "smartcar_rt1064",
            .example = "examples/smartcar/firmware.zig",
            .query = .{
                .cpu_arch = .thumb,
                .os_tag = .freestanding,
                .abi = .eabihf,
                .cpu_model = .{ .explicit = &std.Target.arm.cpu.cortex_m7 },
                .cpu_features_add = std.Target.arm.featureSet(&.{.fp_armv8d16sp}),
            },
        },
    };

    for (freestanding_targets) |t| {
        const resolved = b.resolveTargetQuery(t.query);
        const fw = b.addObject(.{
            .name = b.fmt("breeze_fw_{s}", .{t.name}),
            .root_module = b.createModule(.{
                .root_source_file = b.path(t.example),
                .target = resolved,
                .optimize = .ReleaseSmall,
                .imports = &.{.{ .name = "breeze", .module = breeze }},
            }),
        });
        check_step.dependOn(&fw.step);

        // An artifact that nothing consumes is compiled with `-fno-emit-bin`,
        // which stops the compiler after semantic analysis. That quietly
        // weakens this whole step: it accepts a target whose code cannot
        // actually be *emitted*. It was not theoretical - the RISC-V skeleton
        // passed this check for as long as it existed while being unbuildable,
        // because the Cortex-M assembly that leaked into it only fails at
        // codegen. Installing the object forces the binary to be emitted, so
        // "compiles for this target" now means all the way to an object file.
        const emit = b.addInstallFileWithDir(
            fw.getEmittedBin(),
            .{ .custom = "fwcheck" },
            b.fmt("{s}.o", .{t.name}),
        );
        check_step.dependOn(&emit.step);
    }

    // --- formatting / lint shortcuts ---------------------------------------
    const fmt_step = b.step("fmt", "Reformat all sources");
    const fmt = b.addFmt(.{ .paths = &.{ "src", "examples", "build.zig" } });
    fmt_step.dependOn(&fmt.step);

    const fmt_check = b.step("fmt-check", "Check formatting without writing");
    const fmt_c = b.addFmt(.{ .paths = &.{ "src", "examples", "build.zig" }, .check = true });
    fmt_check.dependOn(&fmt_c.step);

    // A single gate for CI.
    const ci = b.step("ci", "Format check, unit tests and target builds");
    ci.dependOn(&fmt_c.step);
    ci.dependOn(&run_unit_tests.step);
    ci.dependOn(&run_app_tests.step);
    ci.dependOn(check_step);
}
