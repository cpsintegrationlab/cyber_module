#include <Eigen/Dense>
#include <memory>

#include "modules/safety_layer/lib/depth_clustering/src/depth_clustering/api/api.h"
#include "modules/safety_layer/lib/verifiable_obstacle_detection/src/verifiable_obstacle_detection/api/api.h"
#include "modules/canbus/proto/chassis.pb.h"
#include "cyber/class_loader/class_loader.h"
#include "cyber/component/component.h"
#include "modules/control/proto/control_cmd.pb.h"
#include "modules/localization/proto/gps.pb.h"
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
	ProcessDetections(
		const std::shared_ptr<perception::PerceptionObstacles> detections_mission,
		const std::shared_ptr<perception::PerceptionObstacles> detections_safety,
		const std::shared_ptr<localization::Gps> gps);

	std::vector<verifiable_obstacle_detection::Polygon>
	convertMissionDetectionsToPolygons(const std::shared_ptr<perception::PerceptionObstacles> detections,
		const std::shared_ptr<localization::Gps> gps);

	std::vector<verifiable_obstacle_detection::Polygon>
	convertSafetyDetectionsToPolygons(const std::shared_ptr<perception::PerceptionObstacles> detections,
		depth_clustering::BoundingBox::Type bounding_box_type);

	std::shared_ptr<cyber::Reader<canbus::Chassis>> reader_chassis_;
	std::shared_ptr<cyber::Reader<control::ControlCommand>> reader_control_command_;
	std::shared_ptr<cyber::Reader<perception::PerceptionObstacles>> reader_detections_mission_;
	std::shared_ptr<cyber::Reader<perception::PerceptionObstacles>> reader_detections_safety_;
	std::shared_ptr<cyber::Reader<localization::Gps>> reader_gps_;
	std::shared_ptr<cyber::Writer<control::ControlCommand>> writer_control_command_;
	const std::string channel_name_reader_chassis_;
	const std::string channel_name_reader_control_command_;
	const std::string channel_name_reader_detections_mission_;
	const std::string channel_name_reader_detections_safety_;
	const std::string channel_name_reader_gps_;
	const std::string channel_name_writer_control_command_;

	std::shared_ptr<verifiable_obstacle_detection::VerifiableObstacleDetection> verifiable_obstacle_detection_;

	bool override_;
	Eigen::Vector2d translation_vector_lidar_;
	float chassis_speed_mps_;
	float control_command_brake_;
	const std::string depth_clustering_config_file_name_;
	const std::string log_directory_name_verifiable_obstacle_detection_;
};

CYBER_REGISTER_COMPONENT (DecisionComponent)
}	/* safety_layer */
}	/* apollo */
