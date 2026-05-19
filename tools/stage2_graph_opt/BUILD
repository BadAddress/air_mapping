load("//tools:apollo_package.bzl", "apollo_cc_binary", "apollo_package")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

apollo_cc_binary(
    name = "stage2_graph_opt",
    srcs = ["stage2_graph_opt_main.cc"],
    copts = [
        "-DMODULE_NAME=\\\"air_mapping_stage2_graph_opt\\\"",
    ],
    deps = [
        "//cyber",
        "//modules/air_mapping/stage2",
        "@com_github_gflags_gflags//:gflags",
    ],
)

apollo_package()
cpplint()

