const std = @import("std");

pub fn build(b: *std.Build) void {
    const target = b.standardTargetOptions(.{});

    const optimize = b.standardOptimizeOption(.{});

    const linkage = .static;

    const rclcpp = b.dependency("rclcpp", .{ .target = target, .optimize = optimize, .linkage = linkage });
    const rmw_cyclonedds_dep = b.dependency("rmw_cyclonedds", .{ .target = target, .optimize = optimize, .linkage = linkage });
    const cyclonedds_dep = rmw_cyclonedds_dep.builder.dependency("cyclonedds", .{ .target = target, .optimize = optimize, .linkage = linkage });
    const rcl_logging_dep = b.dependency("rcl_logging", .{ .target = target, .optimize = optimize, .linkage = linkage });

    const rcl = rclcpp.builder.dependency("rcl", .{
        .target = target,
        .optimize = optimize,
        .linkage = linkage,
    });

    const libyaml = rcl.builder.dependency("libyaml", .{
        .target = target,
        .optimize = optimize,
        .linkage = linkage,
    });

    const rmw = rcl.builder.dependency("rmw", .{
        .target = target,
        .optimize = optimize,
        .linkage = linkage,
    });

    const ros2_tracing = rcl.builder.dependency("ros2_tracing", .{
        .target = target,
        .optimize = optimize,
        .linkage = linkage,
    });

    const rcl_interfaces = rcl.builder.dependency("rcl_interfaces", .{
        .target = target,
        .optimize = optimize,
        .linkage = linkage,
    });

    const rosidl = rcl.builder.dependency("rosidl", .{
        .target = target,
        .optimize = optimize,
        .linkage = linkage,
    });

    const rcutils = rcl.builder.dependency("rcutils", .{
        .target = target,
        .optimize = optimize,
        .linkage = linkage,
    });

    const rmw_dds_common = rmw_cyclonedds_dep.builder.dependency("rmw_dds_common", .{
        .target = target,
        .optimize = optimize,
        .linkage = linkage,
    });

    const rcpputils = rmw_dds_common.builder.dependency("rcpputils", .{
        .target = target,
        .optimize = optimize,
        .linkage = linkage,
    });

    const libstatistics_collector = rclcpp.builder.dependency("libstatistics_collector", .{
        .target = target,
        .optimize = optimize,
        .linkage = linkage,
    });
    // const rmw_dds_common = rclcpp.builder.dependency("rmw_cyclonedds", .{
    //     .target = target,
    //     .optimize = optimize,
    //     .linkage = linkage,
    // }).builder.dependency("rmw_dds_common", .{
    //     .target = target,
    //     .optimize = optimize,
    //     .linkage = linkage,
    // });

    var pub_sub_node = b.addExecutable(.{
        .name = "node",
        .target = target,
        .optimize = optimize,
        .strip = if (optimize == .Debug) false else true,
        // .strip = false,
    });

    // if (linkage == .static) {
    pub_sub_node.addLibraryPath(std.Build.LazyPath{ .cwd_relative = "/home/matt/clang-18/lib/clang/18/lib/x86_64-unknown-linux-gnu" });
    //     pub_sub_node.linkSystemLibrary2("ubsan", .{ .preferred_link_mode = .static });
    //     pub_sub_node.linkSystemLibrary2("ubsan_cxx", .{ .preferred_link_mode = .static });
    // } else {
    //     pub_sub_node.linkSystemLibrary("ubsan");
    //     pub_sub_node.linkSystemLibrary2("ubsan", .{ .preferred_link_mode = .dynamic });
    // pub_sub_node.linkSystemLibrary2("ubsan_cxx", .{ .preferred_link_mode = .dynamic });
    // }
    // pub_sub_node.addLibraryPath(std.Build.LazyPath{ .cwd_relative = "/home/matt/clang18" });
    // pub_sub_node.linkSystemLibrary2("clang_rt.ubsan_standalone", .{ .preferred_link_mode = .static });
    // pub_sub_node.linkSystemLibrary2("clang_rt.ubsan_standalone_cxx", .{ .preferred_link_mode = .static });
    // pub_sub_node.linkSystemLibrary2("clang_rt.asan", .{ .preferred_link_mode = .static });
    // pub_sub_node.linkSystemLibrary2("clang_rt.asan_cxx", .{ .preferred_link_mode = .static });
    // pub_sub_node.linkSystemLibrary("ubsan");
    // pub_sub_node.linkSystemLibrary2("ubsan", .{ .preferred_link_mode = .dynamic });

    // pub_sub_node.want_lto = true;
    // pub_sub_node.link_function_sections = true;
    // pub_sub_node.link_data_sections = true;
    // pub_sub_node.link_gc_sections = true;

    pub_sub_node.linkLibCpp();
    pub_sub_node.linkLibrary(rclcpp.artifact("rclcpp"));
    pub_sub_node.linkLibrary(rmw_cyclonedds_dep.artifact("rmw_cyclonedds_cpp"));
    pub_sub_node.linkLibrary(cyclonedds_dep.artifact("cyclonedds"));
    pub_sub_node.linkLibrary(rcl_logging_dep.artifact("rcl_logging_spdlog"));
    pub_sub_node.addIncludePath(rcl_interfaces.namedWriteFiles("builtin_interfaces__rosidl_generator_cpp").getDirectory());
    pub_sub_node.addIncludePath(rcl_interfaces.namedWriteFiles("statistics_msgs__rosidl_generator_cpp").getDirectory());
    pub_sub_node.addIncludePath(rcl_interfaces.namedWriteFiles("rosgraph_msgs__rosidl_generator_cpp").getDirectory());
    pub_sub_node.addIncludePath(rcl_interfaces.namedWriteFiles("service_msgs__rosidl_generator_cpp").getDirectory());
    pub_sub_node.linkLibrary(rcl_interfaces.artifact("builtin_interfaces__rosidl_typesupport_cpp"));
    pub_sub_node.linkLibrary(rcl_interfaces.artifact("statistics_msgs__rosidl_typesupport_cpp"));
    pub_sub_node.linkLibrary(rcl_interfaces.artifact("rosgraph_msgs__rosidl_typesupport_cpp"));
    pub_sub_node.linkLibrary(rcl_interfaces.artifact("service_msgs__rosidl_typesupport_cpp"));

    pub_sub_node.addIncludePath(rcl_interfaces.namedWriteFiles("rcl_interfaces__rosidl_generator_cpp").getDirectory());
    pub_sub_node.linkLibrary(rcl_interfaces.artifact("rcl_interfaces__rosidl_typesupport_cpp"));
    // pub_sub_node.linkLibrary(rcl_interfaces.artifact("rcl_interfaces__rosidl_typesupport_introspection_cpp"));
    // pub_sub_node.linkLibrary(rcl_interfaces.artifact("rcl_interfaces__rosidl_typesupport_introspection_c"));
    // pub_sub_node.linkLibrary(rcl_interfaces.artifact("rcl_interfaces__rosidl_typesupport_c"));
    // pub_sub_node.linkLibrary(rcl_interfaces.artifact("rcl_interfaces__rosidl_generator_c"));

    pub_sub_node.addIncludePath(rosidl.namedWriteFiles("rosidl_typesupport_interface").getDirectory());
    pub_sub_node.linkLibrary(rosidl.artifact("rosidl_runtime_c"));
    pub_sub_node.linkLibrary(rosidl.artifact("rosidl_dynamic_typesupport"));
    pub_sub_node.linkLibrary(rosidl.artifact("rosidl_typesupport_introspection_cpp"));
    pub_sub_node.addIncludePath(rosidl.namedWriteFiles("rosidl_runtime_cpp").getDirectory());
    pub_sub_node.linkLibrary(rcutils.artifact("rcutils"));

    pub_sub_node.linkLibrary(rcl.artifact("rcl"));
    // needed for rcl
    pub_sub_node.linkLibrary(rcl_interfaces.artifact("builtin_interfaces__rosidl_generator_c"));
    pub_sub_node.linkLibrary(rcl_interfaces.artifact("service_msgs__rosidl_generator_c"));
    pub_sub_node.linkLibrary(rcl_interfaces.artifact("type_description_interfaces__rosidl_generator_c"));

    pub_sub_node.linkLibrary(rcl.artifact("rcl_yaml_param_parser"));
    pub_sub_node.linkLibrary(libyaml.artifact("yaml"));
    pub_sub_node.linkLibrary(rmw.artifact("rmw"));
    pub_sub_node.addIncludePath(ros2_tracing.namedWriteFiles("tracetools").getDirectory());
    pub_sub_node.linkLibrary(rcpputils.artifact("rcpputils"));
    pub_sub_node.linkLibrary(libstatistics_collector.artifact("libstatistics_collector"));

    // DDS stuff? (this one is probably always needed tho?)
    pub_sub_node.linkLibrary(rmw_dds_common.artifact("rmw_dds_common"));

    // note I haven't prooved that all these are needed, just spray and pray at this point
    pub_sub_node.addIncludePath(rmw_dds_common.namedWriteFiles("rmw_dds_common__rosidl_generator_cpp").getDirectory());
    pub_sub_node.linkLibrary(rmw_dds_common.artifact("rmw_dds_common__rosidl_typesupport_cpp"));
    pub_sub_node.linkLibrary(rmw_dds_common.artifact("rmw_dds_common__rosidl_typesupport_introspection_cpp"));
    // pub_sub_node.linkLibrary(rmw_dds_common.artifact("rmw_dds_common__rosidl_typesupport_introspection_c"));
    // pub_sub_node.linkLibrary(rmw_dds_common.artifact("rmw_dds_common__rosidl_typesupport_c"));
    // pub_sub_node.linkLibrary(rmw_dds_common.artifact("rmw_dds_common__rosidl_generator_c"));

    // pub_sub_node.addIncludePath(rclcpp.builder.dependency("rcl", .{}).builder.dependency("rcl_interfaces", .{}).namedWriteFiles("builtin_interfaces__rosidl_generator_cpp").getDirectory());

    pub_sub_node.addCSourceFiles(.{
        .root = b.path("src"),
        .files = &.{"main.cpp"},
        .flags = &.{
            "--std=c++17",
            "-Wno-deprecated-declarations",
            // "-fsanitize=address",
        },
    });
    b.installArtifact(pub_sub_node);
}
