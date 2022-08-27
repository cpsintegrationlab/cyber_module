load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_binary(
	name = "libsafety_layer_component.so",
	deps = [":safety_layer_component_lib"],
	linkshared = True,
	linkstatic = False,
)

cc_library(
	name = "safety_layer_component_lib",
	srcs = [
		"lidar_component.cc",
		"decision_component.cc",
	],
	hdrs = [
		"lidar_component.h",
		"decision_component.h",
	],
	copts = [
		'-DMODULE_NAME=\\"safety_layer\\"',
	],
	linkopts = [
		"-lboost_date_time",
	],
	deps = [
		"//cyber",
		"//modules/drivers/proto:sensor_proto",
		"//modules/canbus/proto:canbus_proto",
		"//modules/control/proto:control_proto",
		"//modules/perception/proto:perception_proto",
		"//modules/safety_layer/proto:safety_layer_proto",
		"//modules/perception/camera/common:camera_frame",
		"//modules/safety_layer/lib:depth_clustering_lib",
		"@eigen",
	],
)

cpplint()
