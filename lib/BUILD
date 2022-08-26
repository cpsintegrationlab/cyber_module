load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "depth_clustering_lib",
    srcs = glob([
        "depth_clustering/install/amd64/depth_clustering/release/lib/libdc_*.so",
    ]),
    includes = [
		"depth_clustering/src/depth_clustering",
	],
    copts = [
        '-DMODULE_NAME=\\"safety_layer\\"',
    ],
    deps = [
		"//modules/safety_layer/lib:depth_clustering_eigen_lib",
	],
)

cc_library(
    name = "depth_clustering_eigen_lib",
    includes = [
		"depth_clustering/install/amd64/eigen/include/eigen3",
	],
    copts = [
        '-DMODULE_NAME=\\"safety_layer\\"',
    ],
)

cpplint()
