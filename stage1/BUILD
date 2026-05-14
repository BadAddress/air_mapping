load("//tools:apollo_package.bzl", "apollo_cc_library", "apollo_package")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

apollo_cc_library(
    name = "stage1",
    srcs = [
        "stage1_artifact_writer.cc",
        "stage1_config.cc",
        "stage1_runner.cc",
    ],
    hdrs = [
        "stage1_artifact_writer.h",
        "stage1_config.h",
        "stage1_runner.h",
    ],
    copts = [
        "-DMODULE_NAME=\\\"air_mapping\\\"",
        "-O2",
        "-march=native",
        "-fopenmp",
        "-DNDEBUG",
        "-DPCL_NO_PRECOMPILE",
    ],
    linkopts = [
        "-fopenmp",
    ],
    deps = [
        "//cyber",
        "//modules/air_mapping/system:lightning_common",
        "//modules/air_mapping/system:lightning_system",
        "//modules/common_msgs/localization_msgs:gps_cc_proto",
        "//modules/common_msgs/localization_msgs:imu_cc_proto",
        "//modules/common_msgs/sensor_msgs:gnss_best_pose_cc_proto",
        "//modules/common_msgs/sensor_msgs:heading_cc_proto",
        "//modules/common_msgs/sensor_msgs:imu_cc_proto",
        "//modules/common_msgs/sensor_msgs:ins_cc_proto",
        "//modules/common_msgs/sensor_msgs:pointcloud_cc_proto",
        "@com_github_jbeder_yaml_cpp//:yaml-cpp",
        "@local_config_pcl//:pcl",
    ],
)

apollo_package()
cpplint()
