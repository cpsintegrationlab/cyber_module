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
        "safety_layer_component.cc",
    ],
    hdrs = [
        "safety_layer_component.h",
    ],
    deps = [
        "//cyber",
        "//modules/drivers/proto:sensor_proto",
        "//modules/perception/camera/common:camera_frame",
    ],
)

cpplint()
