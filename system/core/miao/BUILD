load("//tools:cpplint.bzl", "cpplint")
load("//tools:apollo_package.bzl", "apollo_cc_library", "apollo_package")

package(default_visibility = ["//visibility:public"])

apollo_cc_library(
    name = "miao",
    srcs = glob([
        "core/**/*.cc",
        "utils/*.cc",
    ]),
    hdrs = glob([
        "core/**/*.h",
        "core/**/*.hpp",
        "utils/*.h",
    ]),
    copts = [
        "-O2",
        "-march=native",
        "-fopenmp",
        "-DNDEBUG",
    ],
    linkopts = ["-ltbb", "-fopenmp"],
    includes = [
        ".",
        "core", 
    ],
    deps = [
        "//cyber",  # For Apollo logging
        "@eigen",
        "//modules/air_mapping/system:lightning_common",
    ],
)

apollo_package()
cpplint()
