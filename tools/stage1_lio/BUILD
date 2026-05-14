load("//tools:apollo_package.bzl", "apollo_cc_binary", "apollo_package")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

apollo_cc_binary(
    name = "stage1_lio",
    srcs = ["stage1_lio_main.cc"],
    copts = [
        "-DMODULE_NAME=\\\"air_mapping_stage1_lio\\\"",
    ],
    deps = [
        "//cyber",
        "//modules/air_mapping/stage1",
        "@com_github_gflags_gflags//:gflags",
    ],
)

apollo_package()
cpplint()

