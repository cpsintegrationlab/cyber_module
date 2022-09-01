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
		"decision_component.cc",
		"lidar_component.cc",
		"logging_component.cc",
	],
	hdrs = [
		"decision_component.h",
		"lidar_component.h",
		"logging_component.h",
	],
	copts = [
		'-DMODULE_NAME=\\"safety_layer\\"',
	],
	linkopts = [
		"-lboost_filesystem",
	],
	deps = [
		"//cyber",
		"//modules/canbus/proto:canbus_proto",
		"//modules/control/proto:control_proto",
		"//modules/perception/proto:perception_proto",
		"//modules/drivers/proto:sensor_proto",
		"//modules/safety_layer/lib:depth_clustering_lib",
		"//modules/safety_layer/lib:verifiable_obstacle_detection_lib",
		"//modules/planning/proto:planning_proto",
		"//modules/localization/proto:pose_proto",
		"//modules/localization/proto:gps_proto",
		"@eigen",
	],
)

cpplint()
