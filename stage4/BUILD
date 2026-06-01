load("//tools:apollo_package.bzl", "apollo_cc_library", "apollo_package")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

apollo_cc_library(
    name = "stage4",
    srcs = [
        "stage4_config.cc",
        "stage4_runner.cc",
    ],
    hdrs = [
        "stage4_config.h",
        "stage4_runner.h",
    ],
    copts = [
        "-DMODULE_NAME=\\\"air_mapping_stage4\\\"",
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
        "@com_github_jbeder_yaml_cpp//:yaml-cpp",
        "@eigen",
        "@local_config_pcl//:pcl",
    ],
)

apollo_package()
cpplint()
