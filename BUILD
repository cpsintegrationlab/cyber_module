load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "depth_clustering_lib",
    srcs = [
        "lib/depth_clustering/install/lib/libdepth_clustering.so",
    ],
    hdrs = glob(["lib/depth_clustering/src/src/**/*.h"]),
    copts = [
        '-DMODULE_NAME=\\"safety_layer\\"',
    ],
)

cpplint()
