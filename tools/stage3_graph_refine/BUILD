load("//tools:apollo_package.bzl", "apollo_cc_binary", "apollo_package")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

apollo_cc_binary(
    name = "stage3_graph_refine",
    srcs = ["stage3_graph_refine_main.cc"],
    copts = [
        "-DMODULE_NAME=\\\"air_mapping_stage3_graph_refine\\\"",
    ],
    deps = [
        "//cyber",
        "//modules/air_mapping/stage3",
        "@com_github_gflags_gflags//:gflags",
    ],
)

apollo_package()
cpplint()
