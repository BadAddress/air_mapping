load("//tools:apollo_package.bzl", "apollo_cc_library", "apollo_package")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

apollo_cc_library(
    name = "stage3",
    srcs = [
        "stage2_artifact_reader.cc",
        "stage3_artifact_writer.cc",
        "stage3_config.cc",
        "stage3_runner.cc",
    ],
    hdrs = [
        "stage2_artifact_reader.h",
        "stage3_artifact_writer.h",
        "stage3_config.h",
        "stage3_runner.h",
        "stage3_types.h",
    ],
    copts = [
        "-DMODULE_NAME=\\\"air_mapping_stage3\\\"",
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
        "//modules/air_mapping/system/core/miao:miao",
        "//modules/air_mapping/system:lightning_common",
        "@com_github_jbeder_yaml_cpp//:yaml-cpp",
        "@eigen",
        "@local_config_pcl//:pcl",
    ],
)

apollo_package()
cpplint()
