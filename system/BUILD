load("//tools:cpplint.bzl", "cpplint")
load("//tools:apollo_package.bzl", "apollo_cc_library", "apollo_package")

package(default_visibility = ["//visibility:public"])

# 1. Common Library
apollo_cc_library(
    name = "lightning_common",
    srcs = glob([
        "common/*.cc",
    ]),
    hdrs = glob([
        "common/*.h",
        "common/*.hpp",
    ]),
    copts = [
        "-O2",
        "-march=native",
        "-fopenmp",
        "-DNDEBUG",
        "-DPCL_NO_PRECOMPILE",
    ],
    linkopts = ["-ltbb", "-fopenmp"],
    includes = ["."],
    deps = [
        "//cyber",  # For Apollo logging (AINFO, AWARN, AERROR)
        "@eigen",
        "@com_github_jbeder_yaml_cpp//:yaml-cpp",
        "//modules/air_mapping/system/thirdparty/Sophus:sophus",
        "@local_config_pcl//:pcl",
    ],
)

# 2. Utils Library
apollo_cc_library(
    name = "lightning_utils",
    srcs = glob([
        "utils/*.cc",
    ]),
    hdrs = glob([
        "utils/*.h",
    ]),
    copts = [
        "-O2",
        "-march=native",
        "-fopenmp",
        "-DNDEBUG",
        "-DPCL_NO_PRECOMPILE",
    ],
    linkopts = ["-fopenmp"],
    includes = ["."],
    deps = [
        ":lightning_common",
        "//cyber",  # For Apollo logging
        "@eigen",
        "//modules/common_msgs/sensor_msgs:pointcloud_cc_proto",
        "@local_config_pcl//:pcl",
    ],
)

# 3. IO Library
apollo_cc_library(
    name = "lightning_io",
    srcs = glob([
        "io/*.cc",
    ]),
    hdrs = glob([
        "io/*.h",
    ]),
    copts = [
        "-O2",
        "-march=native",
        "-fopenmp",
        "-DNDEBUG",
        "-DPCL_NO_PRECOMPILE",
    ],
    linkopts = ["-fopenmp"],
    includes = ["."],
    deps = [
        ":lightning_common",
        "//cyber",  # For Apollo logging
        "@com_github_jbeder_yaml_cpp//:yaml-cpp",
    ],
)

# 4. G2P5 Library (2D Grid Map - No OpenCV)
apollo_cc_library(
    name = "lightning_g2p5",
    srcs = glob([
        "core/g2p5/*.cc",
    ]),
    hdrs = glob([
        "core/g2p5/*.h",
    ]),
    copts = [
        "-O2",
        "-march=native",
        "-fopenmp",
        "-DNDEBUG",
        "-DPCL_NO_PRECOMPILE",
    ],
    linkopts = ["-fopenmp"],
    includes = ["."],
    deps = [
        ":lightning_common",
        ":lightning_utils",
        "//cyber",
        "@eigen",
        "@local_config_pcl//:pcl",
        "@com_github_jbeder_yaml_cpp//:yaml-cpp",
    ],
)

# 5. Core Algorithm Library
apollo_cc_library(
    name = "lightning_core",
    srcs = glob([
        "core/lio/*.cc",
        "core/maps/*.cc",
        "core/gps_fusion/*.cc",
    ]),
    hdrs = glob([
        "core/**/*.h",
        "core/**/*.hpp",
    ], exclude = ["core/g2p5/**"]),
    copts = [
        "-O2",
        "-march=native",
        "-fopenmp",
        "-DNDEBUG",
        "-DPCL_NO_PRECOMPILE",
    ],
    linkopts = ["-ltbb", "-fopenmp"],
    includes = [
        ".",
        "core",
    ], 
    deps = [
        ":lightning_common",
        ":lightning_utils",
        ":lightning_io",
        "//cyber",  # For Apollo logging
        "//modules/air_mapping/system/core/miao:miao",
        "@eigen",
        "@local_config_pcl//:pcl",
        "@boost",
        "//modules/common_msgs/sensor_msgs:pointcloud_cc_proto",
        "//modules/common_msgs/sensor_msgs:imu_cc_proto",
    ],
)

# 6. System Logic (Corrected glob path)
apollo_cc_library(
    name = "lightning_system",
    srcs = glob([
        "core/system/*.cc",
    ]),
    hdrs = glob([
        "core/system/*.h",
    ]),
    copts = [
        "-O2",
        "-march=native",
        "-fopenmp",
        "-DNDEBUG",
        "-DPCL_NO_PRECOMPILE",
    ],
    linkopts = ["-fopenmp"],
    includes = ["."],
    deps = [
        ":lightning_core",
        ":lightning_g2p5",  # 2D 栅格地图
        "//cyber",
        "//modules/common_msgs/sensor_msgs:pointcloud_cc_proto",
        "//modules/common_msgs/sensor_msgs:imu_cc_proto",
        "//modules/common_msgs/sensor_msgs:gnss_best_pose_cc_proto",
        "//modules/common_msgs/sensor_msgs:gnss_cc_proto",
    ],
)

apollo_package()
cpplint()
