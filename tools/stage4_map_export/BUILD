load("//tools:apollo_package.bzl", "apollo_cc_binary", "apollo_package")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

apollo_cc_binary(
    name = "stage4_map_export",
    srcs = ["stage4_map_export_main.cc"],
    copts = [
        "-DMODULE_NAME=\\\"air_mapping_stage4_map_export\\\"",
    ],
    deps = [
        "//cyber",
        "//modules/air_mapping/stage4",
        "@com_github_gflags_gflags//:gflags",
    ],
)

apollo_package()
cpplint()
