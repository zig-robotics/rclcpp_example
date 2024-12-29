const std = @import("std");

const ZigRos = @import("zigros").ZigRos;

pub fn build(b: *std.Build) void {
    const target = b.standardTargetOptions(.{});
    const optimize = b.standardOptimizeOption(.{});
    const linkage = .static;

    var pub_sub_node = b.addExecutable(.{
        .name = "node",
        .target = target,
        .optimize = optimize,
        .strip = if (optimize == .Debug) false else true,
    });

    pub_sub_node.want_lto = true;
    pub_sub_node.link_function_sections = true;
    pub_sub_node.link_data_sections = true;
    pub_sub_node.link_gc_sections = true;

    const zigros = ZigRos.init(b.dependency("zigros", .{
        .target = target,
        .optimize = optimize,
        .linkage = linkage,
        .@"system-python" = false,
    })) orelse return; // return early if lazy deps are needed

    pub_sub_node.linkLibCpp();
    zigros.linkRclcpp(&pub_sub_node.root_module);
    zigros.linkRmwCycloneDds(&pub_sub_node.root_module);
    zigros.linkLoggerSpd(&pub_sub_node.root_module);

    pub_sub_node.addCSourceFiles(.{
        .root = b.path("src"),
        .files = &.{"main.cpp"},
        .flags = &.{
            "--std=c++17",
            "-Wno-deprecated-declarations",
        },
    });

    var interface = zigros.createInterface(
        b,
        "zigros_example_interface",
        .{ .target = target, .optimize = optimize, .linkage = linkage },
    );
    interface.addInterfaces(b.path(""), &.{"msg/Example.msg"});

    // the example message uses the standard header message, so we must add builtin_interfaces
    // as a dependency
    interface.addDependency("builtin_interfaces", zigros.ros_libraries.builtin_interfaces);
    interface.artifacts.linkCpp(&pub_sub_node.root_module);

    b.installArtifact(pub_sub_node);
}
