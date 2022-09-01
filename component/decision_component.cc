#include "modules/safety_layer/component/decision_component.h"

namespace apollo
{
namespace safety_layer
{
DecisionComponent::DecisionComponent() :
	reader_chassis_(nullptr), reader_control_command_(
	nullptr), reader_detections_mission_(nullptr), reader_detections_safety_(
	nullptr), writer_control_command_(nullptr), channel_name_reader_chassis_(
	"/apollo/canbus/chassis"), channel_name_reader_control_command_(
	"/apollo/control"), channel_name_reader_detections_mission_(
	"/apollo/perception/obstacles"), channel_name_reader_detections_safety_(
	"/apollo/safety_layer/lidar/depth_clustering/detections"), channel_name_writer_control_command_(
	"/apollo/safety_layer/decision/control"),
	override_(false), chassis_speed_mps_(0), control_command_brake_(
	100), depth_clustering_config_file_name_(
	"/apollo/modules/safety_layer/conf/depth_clustering.json"), log_directory_name_verifiable_obstacle_detection_(
    "/apollo/data/log/safety_layer.verifiable_obstacle_detection.log.d")
{
}

DecisionComponent::~DecisionComponent()
{
}

bool
DecisionComponent::Init()
{
	reader_chassis_ = node_->CreateReader<canbus::Chassis>(
		channel_name_reader_chassis_);

    if (!reader_chassis_)
	{
		AWARN << "Failed to create chassis reader.";
	}

	reader_control_command_ = node_->CreateReader<control::ControlCommand>(
		channel_name_reader_control_command_);

	if (!reader_control_command_)
	{
		AWARN << "Failed to create control command reader.";
	}

	reader_detections_mission_ = node_->CreateReader<perception::PerceptionObstacles>(
	channel_name_reader_detections_mission_);

	if (!reader_detections_mission_)
	{
		AWARN << "Failed to create mission detections reader.";
	}

	reader_detections_safety_ = node_->CreateReader<perception::PerceptionObstacles>(
		channel_name_reader_detections_safety_);

	if (!reader_detections_safety_)
	{
		AWARN << "Failed to create safety detections reader.";
	}

	writer_control_command_ = node_->CreateWriter<control::ControlCommand>(
		channel_name_writer_control_command_);

	if (!writer_control_command_)
	{
		AWARN << "Failed to create control command writer.";
	}

	verifiable_obstacle_detection_ = std::make_shared<verifiable_obstacle_detection::VerifiableObstacleDetection>();

	if (verifiable_obstacle_detection_)
	{
		if (!verifiable_obstacle_detection_->initializeForApollo(log_directory_name_verifiable_obstacle_detection_))
		{
			AERROR << "Failed to initialize Verifiable Obstacle Detection.";
			return false;
		}

		if (FLAGS_minloglevel > 0)
		{
			verifiable_obstacle_detection_->disableConsoleLogging();
		}
	}
	else
	{
		AWARN << "Failed to create Verifiable Obstacle Detection.";
	}

	return true;
}

bool
DecisionComponent::Proc()
{
	if (reader_chassis_)
    {
		reader_chassis_->Observe();
		ProcessChassis(reader_chassis_->GetLatestObserved());
    }
    else
	{
		AWARN << "Chassis reader missing.";
	}

	if (reader_detections_mission_)
    {
		reader_detections_mission_->Observe();

		if (reader_detections_safety_)
		{
			reader_detections_safety_->Observe();
			ProcessDetections(reader_detections_mission_->GetLatestObserved(),
				reader_detections_safety_->GetLatestObserved());
		}
		else
		{
			AWARN << "Safety detections reader missing.";
		}
    }
    else
	{
		AWARN << "Mission detections reader missing.";
	}

	if (reader_control_command_)
    {
		reader_control_command_->Observe();
		ProcessControlCommand(reader_control_command_->GetLatestObserved());
    }
    else
	{
		AWARN << "Control command reader missing.";
	}

	return true;
}

void
DecisionComponent::ProcessChassis(const std::shared_ptr<canbus::Chassis> chassis)
{
	if (!chassis)
	{
		AERROR << "Chassis missing.";
        return;
	}

	AINFO << "Processing chassis.";

	chassis_speed_mps_ = chassis->speed_mps();
}

void
DecisionComponent::ProcessControlCommand(const
	std::shared_ptr<control::ControlCommand> control_command)
{
	if (!control_command)
	{
		AERROR << "Control command missing.";
        return;
	}

	if (!writer_control_command_)
	{
		AERROR << "Control command writer missing.";
        return;
	}

	AINFO << "Processing control command.";

	auto control_command_override = std::make_shared<control::ControlCommand>(*control_command);

	if (override_)
	{
		control_command_override->set_throttle(0);
		control_command_override->set_brake(control_command_brake_);
		AWARN << "Activated safety override.";
	}

	writer_control_command_->Write(control_command_override);
}

void
DecisionComponent::ProcessDetections(
	const std::shared_ptr<perception::PerceptionObstacles> detections_mission,
	const std::shared_ptr<perception::PerceptionObstacles> detections_safety)
{
	if (!detections_mission)
	{
		AERROR << "Mission detections missing.";
        return;
	}

	if (!detections_safety)
	{
		AERROR << "Safety detections missing.";
        return;
	}

	AINFO << "Processing detections.";

	const std::string sequence_num = std::to_string(detections_mission->header().sequence_num());
	const std::string frame_name = sequence_num + ".bin";
	auto depth_clustering_parameter_factory = depth_clustering::ParameterFactory(
		depth_clustering_config_file_name_);
	const auto& depth_clustering_parameter = depth_clustering_parameter_factory.getDepthClusteringParameter();
	const auto& bounding_box_type = depth_clustering_parameter.bounding_box_type;

	verifiable_obstacle_detection_->processOneFrameForApollo(frame_name,
		convertDetectionsToPolygons(detections_mission, depth_clustering::BoundingBox::Type::Polygon),
		convertDetectionsToPolygons(detections_safety, bounding_box_type));
}

std::vector<verifiable_obstacle_detection::Polygon>
DecisionComponent::convertDetectionsToPolygons(const std::shared_ptr<perception::PerceptionObstacles> detections,
	depth_clustering::BoundingBox::Type bounding_box_type)
{
	std::vector<verifiable_obstacle_detection::Polygon> polygons;

	switch (bounding_box_type)
	{
	default:
	{
		AWARN << "Unknown bounding box type " << static_cast<int>(bounding_box_type) << ".";
	}
	case depth_clustering::BoundingBox::Type::Cube:
	{
		AINFO << "Converting cube detections to polygons.";
		AWARN << "Polygon conversion for cube detections not implemented.";
		break;
	}
	case depth_clustering::BoundingBox::Type::Polygon:
	{
		AINFO << "Converting polygon detections to polygons.";

		for (const auto& perception_obstacle : detections->perception_obstacle())
		{
			verifiable_obstacle_detection::Polygon polygon;
			std::vector<verifiable_obstacle_detection::Point2D> polygon_points;

			for (const auto& polygon_point : perception_obstacle.polygon_point())
			{
				polygon_points.push_back(verifiable_obstacle_detection::Point2D(polygon_point.x(), polygon_point.y()));
			}

			boost::geometry::assign_points(polygon, polygon_points);
			boost::geometry::correct(polygon);

			polygons.push_back(polygon);
		}

		break;
	}
	case depth_clustering::BoundingBox::Type::Flat:
	{
		AINFO << "Converting flat detections to polygons.";
		AWARN << "Polygon conversion for flat detections not implemented.";
		break;
	}
	}

	return polygons;
}
}	/* safety_layer */
}	/* apollo */
