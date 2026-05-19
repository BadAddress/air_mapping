load("//tools:apollo_package.bzl", "apollo_cc_library", "apollo_package")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

apollo_cc_library(
    name = "stage2",
    srcs = [
        "stage1_artifact_reader.cc",
        "stage2_artifact_writer.cc",
        "stage2_config.cc",
        "stage2_runner.cc",
    ],
    hdrs = [
        "stage1_artifact_reader.h",
        "stage2_artifact_writer.h",
        "stage2_config.h",
        "stage2_runner.h",
        "stage2_types.h",
    ],
    copts = [
        "-DMODULE_NAME=\\\"air_mapping_stage2\\\"",
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
