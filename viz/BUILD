load("//tools:apollo_package.bzl", "apollo_cc_binary", "apollo_cc_library", "apollo_package")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

apollo_cc_library(
    name = "air_mapping_viz_server_lib",
    srcs = ["air_mapping_viz_server.cc"],
    hdrs = ["air_mapping_viz_server.h"],
    deps = [
        "//cyber",
        "@com_github_jbeder_yaml_cpp//:yaml-cpp",
        "@com_github_nlohmann_json//:json",
        "@eigen",
        "@local_config_pcl//:pcl",
    ],
)

apollo_cc_binary(
    name = "air_mapping_viz",
    srcs = ["main.cc"],
    deps = [
        ":air_mapping_viz_server_lib",
        "//cyber",
        "@com_github_gflags_gflags//:gflags",
    ],
)

filegroup(
    name = "frontend_files",
    srcs = glob(["frontend/**/*"]),
)

filegroup(
    name = "hdmap_files",
    srcs = glob(["hdmap/**/*"]),
)

apollo_package()
cpplint()
