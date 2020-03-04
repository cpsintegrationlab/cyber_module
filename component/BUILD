load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_binary(
    name = "libsafety_layer_component.so",
    linkshared = True,
    linkstatic = False,
    deps = [":safety_layer_component_lib"],
)

cc_library(
    name = "safety_layer_component_lib",
    srcs = [
        "depth_camera_component.cc",
        "lidar_component.cc",
    ],
    hdrs = [
        "depth_camera_component.h",
        "lidar_component.h",
    ],
    copts = [
        '-DMODULE_NAME=\\"safety_layer\\"',
    ],
    deps = [
        "//cyber",
        "//modules/drivers/proto:sensor_proto",
        "//modules/perception/camera/common:camera_frame",
    ],
)

cpplint()
