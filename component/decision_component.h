#include <Eigen/Dense>
#include <memory>

#include "modules/canbus/proto/chassis.pb.h"
#include "cyber/class_loader/class_loader.h"
#include "cyber/component/component.h"
#include "modules/control/proto/control_cmd.pb.h"
#include "modules/perception/proto/perception_obstacle.pb.h"
#include "cyber/component/timer_component.h"

namespace apollo
{
namespace safety_layer
{
class DecisionComponent final : public cyber::TimerComponent
{
public:

	DecisionComponent();

	~DecisionComponent();

	bool
	Init() override;

	bool
	Proc() override;

private:

	void
	ProcessChassis(const std::shared_ptr<canbus::Chassis> chassis);

	void
	ProcessControlCommand(const std::shared_ptr<control::ControlCommand> control_command);

	void
	ProcessDepthClusteringDetections(
		const std::shared_ptr<perception::PerceptionObstacles> depth_clustering_detections);

	void
	ProcessDepthClusteringDetectionsCube(
		const std::shared_ptr<perception::PerceptionObstacles> depth_clustering_detections);

	void
	ProcessDepthClusteringDetectionsPolygon(
		const std::shared_ptr<perception::PerceptionObstacles> depth_clustering_detections);

	void
	ProcessDepthClusteringDetectionsFlat(
		const std::shared_ptr<perception::PerceptionObstacles> depth_clustering_detections);

	std::shared_ptr<cyber::Reader<canbus::Chassis>> reader_chassis_;
	std::shared_ptr<cyber::Reader<control::ControlCommand>> reader_control_command_;
	std::shared_ptr<cyber::Reader<perception::PerceptionObstacles>> reader_depth_clustering_detections_;
	std::shared_ptr<cyber::Writer<control::ControlCommand>> writer_control_command_;
	const std::string channel_name_reader_chassis_;
	const std::string channel_name_reader_control_command_;
	const std::string channel_name_reader_depth_clustering_detections_;
	const std::string channel_name_writer_control_command_;

	bool override_;
	float chassis_speed_mps_;
	float control_command_brake_;
	const std::string depth_clustering_config_file_name_;
};

CYBER_REGISTER_COMPONENT (DecisionComponent)
}	/* safety_layer */
}	/* apollo */
