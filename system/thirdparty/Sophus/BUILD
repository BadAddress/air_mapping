load("//tools:apollo_package.bzl", "apollo_cc_library", "apollo_package")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

apollo_cc_library(
    name = "sophus",
    hdrs = glob(["*.hpp"]),
    includes = ["."], 
    deps = [
        "@eigen",
    ],
)

apollo_package()
cpplint()

