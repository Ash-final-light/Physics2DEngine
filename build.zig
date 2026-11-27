const std = @import("std");
const Build = std.Build;
const Compile = Build.Step.Compile;
const ResolvedTarget = Build.ResolvedTarget;
const Module = Build.Module;
const OptimizeMode = std.builtin.OptimizeMode;

const Translator = @import("translate_c").Translator;

pub const Options = struct {
    shared: bool,
    c_unit_tests: bool,
    zig_unit_tests: bool,
    disable_simd: bool,

    const defaults = Options{
        .shared = false,
        .c_unit_tests = false,
        .zig_unit_tests = false,
        .disable_simd = false,
    };

    pub fn getOptions(b: *Build) Options {
        return .{
            .shared = b.option(bool, "shared", "Compile as shared library") orelse defaults.shared,
            .c_unit_tests = b.option(bool, "c_unit_tests", "Compile the c unit tests") orelse defaults.c_unit_tests,
            .zig_unit_tests = b.option(bool, "zig_unit_tests", "Compile the zig unit tests") orelse defaults.zig_unit_tests,
            .disable_simd = b.option(bool, "disable_simd", "Disable SIMD math (slower)") orelse defaults.disable_simd,
        };
    }
};

pub fn build(b: *Build) !void {
    const target = b.standardTargetOptions(.{});
    const optimize = b.standardOptimizeOption(.{});
    const options = Options.getOptions(b);
    const lib = try compileBox2d(b, target, optimize, options);

    b.installArtifact(lib);

    if (options.c_unit_tests) {
        buildCTests(b, target, optimize, lib);
    }

    // Translate the box2d headers and export them as a module
    const translate_c = b.dependency("translate_c", .{});
    const translator: Translator = .init(translate_c, .{
        .name = "translate_box2d",
        .c_source_file = b.path("include/box2d/box2d.h"),
        .target = target,
        .optimize = optimize,
    });
    translator.addIncludePath(b.path("include"));

    const box2d_mod = b.addModule("box2d", .{
        .root_source_file = translator.output_file,
    });
    box2d_mod.linkLibrary(lib);

    if (options.zig_unit_tests) {
        buildZigTest(b, target, optimize, box2d_mod);
    }
}

fn compileBox2d(b: *Build, target: ResolvedTarget, optimize: OptimizeMode, options: Options) !*Compile {
    const module = b.addModule("box2dc", .{
        .target = target,
        .optimize = optimize,
        .link_libc = true,
        .link_libcpp = true,
    });

    var box2d_flags_arr = std.ArrayList([]const u8).empty;
    defer box2d_flags_arr.deinit(b.allocator);

    try box2d_flags_arr.appendSlice(b.allocator, &[_][]const u8{
        "-std=c++20",
        "-D_GNU_SOURCE",
        // Contraction into FMA is a value changing transform, keep it off for determinism
        "-ffp-contract=off",
    });

    if (options.shared) {
        try box2d_flags_arr.appendSlice(b.allocator, &[_][]const u8{
            "-fPIC",
        });

        // base.h picks dllexport/visibility from this define
        module.addCMacro("box2d_EXPORTS", "");
    }

    if (options.disable_simd) {
        module.addCMacro("BOX2D_DISABLE_SIMD", "");
    } else if (target.result.os.tag == .emscripten) {
        try box2d_flags_arr.appendSlice(b.allocator, &[_][]const u8{ "-msimd128", "-msse2" });
    }

    const linkage: std.builtin.LinkMode = if (options.shared) .dynamic else .static;
    const box2d = b.addLibrary(.{
        .root_module = module,
        .name = "box2d",
        .linkage = linkage,
    });

    const c_source_files = &[_][]const u8{
        "src/aabb.cpp",
        "src/arena_allocator.cpp",
        "src/bitset.cpp",
        "src/body.cpp",
        "src/broad_phase.cpp",
        "src/chain_builder.cpp",
        "src/constraint_graph.cpp",
        "src/contact.cpp",
        "src/contact_solver.cpp",
        "src/contact_solver_w4.cpp",
        "src/contact_solver_w8.cpp",
        "src/core.cpp",
        "src/distance.cpp",
        "src/distance_joint.cpp",
        "src/dynamic_tree.cpp",
        "src/geometry.cpp",
        "src/hull.cpp",
        "src/id_pool.cpp",
        "src/island.cpp",
        "src/joint.cpp",
        "src/manifold.cpp",
        "src/math_functions.cpp",
        "src/motor_joint.cpp",
        "src/mover.cpp",
        "src/mover_joint.cpp",
        "src/parallel_for.cpp",
        "src/physics_world.cpp",
        "src/pogo_joint.cpp",
        "src/prismatic_joint.cpp",
        "src/recording.cpp",
        "src/replay.cpp",
        "src/revolute_joint.cpp",
        "src/scheduler.cpp",
        "src/sensor.cpp",
        "src/shape.cpp",
        "src/simd.cpp",
        "src/snapshot.cpp",
        "src/solver.cpp",
        "src/solver_set.cpp",
        "src/state_hash.cpp",
        "src/table.cpp",
        "src/timer.cpp",
        "src/types.cpp",
        "src/weld_joint.cpp",
        "src/wheel_joint.cpp",
    };

    box2d.root_module.addIncludePath(b.path("include"));
    box2d.installHeadersDirectory(b.path("include/box2d"), "box2d", .{});

    box2d.root_module.addCSourceFiles(.{
        .files = c_source_files,
        .flags = box2d_flags_arr.items,
        .language = .cpp,
    });

    return box2d;
}

fn buildCTests(b: *Build, target: ResolvedTarget, optimize: OptimizeMode, box2d_lib: *Compile) void {
    const test_module = b.createModule(.{
        .target = target,
        .optimize = optimize,
    });

    if (Options.getOptions(b).disable_simd) test_module.addCMacro("BOX2D_DISABLE_SIMD", "");

    if (Options.getOptions(b).shared) {
        test_module.addCSourceFiles(.{ .files = &.{"test/api_compat.c"}, .flags = &.{"-std=c17"}, .language = .c });
        if (target.result.os.tag == .windows) test_module.addCMacro("BOX2D_DLL", "");
    } else {
        test_module.addCSourceFiles(.{
            .files = &[_][]const u8{
                "test/main.cpp",
                "test/test_toolchain.cpp",
                "test/test_log.cpp",
                "test/test_length.cpp",
                "test/test_memory.cpp",
                "test/test_scalar.cpp",
                "test/test_vector.cpp",
                "test/test_rotation.cpp",
                "test/test_transform.cpp",
                "test/test_matrix.cpp",
                "test/test_trig.cpp",
                "test/test_array.cpp",
                "test/test_bitset.cpp",
                "test/test_id_pool.cpp",
                "test/test_table.cpp",
                "test/test_stack.cpp",
                "test/test_timer.cpp",
                "test/test_thread.cpp",
                "test/test_aabb.cpp",
                "test/test_geometry.cpp",
                "test/test_distance.cpp",
                "test/test_collision.cpp",
                "test/test_chain.cpp",
                "test/test_dynamic_tree.cpp",
                "test/test_world.cpp",
                "test/test_step.cpp",
                "test/test_joint.cpp",
                "test/test_mover.cpp",
                "test/test_query.cpp",
                "test/test_sensor.cpp",
                "test/test_contact.cpp",
                "test/test_solver_set.cpp",
                "test/test_island.cpp",
                "test/test_constraint_graph.cpp",
                "test/test_contact_solver.cpp",
                "test/test_shape.cpp",
                "test/test_broad_phase.cpp",
                "test/test_state_hash.cpp",
                "test/test_snapshot.cpp",
                "test/test_recording.cpp",
                "test/test_precision.cpp",
                "test/test_helpers.cpp",
                "shared/utils.cpp",
                "shared/determinism.cpp",
                "shared/human.cpp",
                "test/test_replay.cpp",
                "test/contact_solver_scalar4.cpp",
                "test/test_hull.cpp",
            },
            .flags = &.{"-std=c++20", "-ffp-contract=off"},
            .language = .cpp,
        });
        test_module.addCSourceFiles(.{
            .files = &[_][]const u8{
                "test/test_version.c",
                "test/test_assert.c",
                "test/toolchain_c.c",
            },
            .flags = &.{"-std=c17"},
            .language = .c,
        });
        test_module.addIncludePath(b.path("src"));
    }
    test_module.addIncludePath(b.path("include"));
    test_module.addIncludePath(b.path("test"));
    test_module.addIncludePath(b.path("shared"));

    const test_exe = b.addExecutable(.{
        .name = "c_test",
        .root_module = test_module,
    });

    test_module.linkLibrary(box2d_lib);

    b.installArtifact(test_exe);
}

// Build a simple zig test to validate that translate_c works as expected
fn buildZigTest(b: *Build, target: ResolvedTarget, optimize: OptimizeMode, box2d_mod: *Module) void {
    const mod = b.createModule(.{
        .root_source_file = b.path("test/zig/test_world.zig"),
        .target = target,
        .optimize = optimize,
    });

    mod.addImport("box2d", box2d_mod);

    const zig_test = b.addTest(.{
        .name = "zig_test",
        .root_module = mod,
    });

    b.installArtifact(zig_test);
}
