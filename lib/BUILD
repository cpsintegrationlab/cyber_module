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
)

cc_library(
    name = "verifiable_obstacle_detection_lib",
    srcs = glob([
        "verifiable_obstacle_detection/install/amd64/verifiable_obstacle_detection/release/lib/libvod_*.so",
    ]),
    includes = [
		"verifiable_obstacle_detection/src/verifiable_obstacle_detection",
	],
    copts = [
        '-DMODULE_NAME=\\"safety_layer\\"',
    ],
)


cpplint()
