#include <memory>
#include <Eigen/Dense>

#include "cyber/class_loader/class_loader.h"
#include "cyber/component/component.h"
#include "cyber/component/timer_component.h"
#include "lgsvl_pkgs/lgsvl_msgs/proto/detection3darray.pb.h"
#include "modules/canbus/proto/chassis.pb.h"
#include "modules/control/proto/control_cmd.pb.h"

namespace apollo
{
namespace safety_layer
{
class DecisionComponent final : public cyber::TimerComponent
{
public:

	DecisionComponent();

	bool
	Init() override;

	bool
	Proc() override;

private:

	void
	ProcessChassis(const std::shared_ptr<canbus::Chassis> chassis_message);

	void
	ProcessDepthClusteringDetection(
			const std::shared_ptr<common::Detection3DArray> depth_clustering_detection_message);

	void
	ProcessControlCommand(const std::shared_ptr<control::ControlCommand> control_command_message);

	double
	CalculateBoundingBoxDistance(const Eigen::Vector3d& center, const Eigen::Vector3d& extent);

	std::shared_ptr<cyber::Reader<canbus::Chassis>> chassis_reader_;
	std::shared_ptr<cyber::Reader<common::Detection3DArray>> depth_clustering_detection_reader_;
	std::shared_ptr<cyber::Reader<control::ControlCommand>> control_command_reader_;
	std::shared_ptr<cyber::Writer<control::ControlCommand>> control_command_writer_;

	bool cruise_;
	double target_speed_mps_;
	double braking_acceleration_;
	double braking_distance_;
	double braking_slack_;

	bool override_;
	double override_braking_percentage_;
};

CYBER_REGISTER_COMPONENT (DecisionComponent)
} // safety_layer
} // apollo
