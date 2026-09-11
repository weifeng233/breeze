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

    const test_step = b.step("test", "Run kernel unit tests");
    test_step.dependOn(&run_unit_tests.step);

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
    // The CPU/ABI triples below are not arbitrary: the `smartcar_*` entries are
    // exactly the ones the Smartcar-Template Makefiles use for CYT2BL3,
    // CYT4BB7 and RT1064, so a change that breaks those boards fails here.
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
            .example = "examples/firmware_smartcar.zig",
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
            .example = "examples/firmware_smartcar.zig",
            .query = .{
                .cpu_arch = .thumb,
                .os_tag = .freestanding,
                .abi = .eabi,
                .cpu_model = .{ .explicit = &std.Target.arm.cpu.cortex_m0plus },
            },
        },
        .{
            .name = "smartcar_cyt4bb7_cm7",
            .example = "examples/firmware_smartcar.zig",
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
    ci.dependOn(check_step);
}
