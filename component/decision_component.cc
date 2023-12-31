#include "modules/safety_layer/lib/depth_clustering/src/depth_clustering/api/api.h"
#include "modules/safety_layer/component/decision_component.h"

namespace apollo
{
namespace safety_layer
{
DecisionComponent::DecisionComponent() :
	reader_chassis_(nullptr), reader_control_command_(
	nullptr), reader_depth_clustering_detections_(nullptr), writer_control_command_(
	nullptr), channel_name_reader_chassis_(
	"/apollo/canbus/chassis"), channel_name_reader_control_command_(
	"/apollo/control"), channel_name_reader_depth_clustering_detections_(
	"/apollo/safety_layer/lidar/depth_clustering/detections"), channel_name_writer_control_command_(
	"/apollo/safety_layer/decision/control"),
	override_(false), chassis_speed_mps_(0), control_command_brake_(
	100), depth_clustering_config_file_name_(
	"/apollo/modules/safety_layer/conf/depth_clustering.json")
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

	reader_depth_clustering_detections_ = node_->CreateReader<perception::PerceptionObstacles>(
		channel_name_reader_depth_clustering_detections_);

	if (!reader_depth_clustering_detections_)
	{
		AWARN << "Failed to create Depth Clustering detections reader.";
	}

	writer_control_command_ = node_->CreateWriter<control::ControlCommand>(
		channel_name_writer_control_command_);

	if (!writer_control_command_)
	{
		AWARN << "Failed to create control command writer.";
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

	if (reader_depth_clustering_detections_)
    {
		reader_depth_clustering_detections_->Observe();
		ProcessDepthClusteringDetections(reader_depth_clustering_detections_->GetLatestObserved());
    }
    else
	{
		AWARN << "Depth Clustering detections reader missing.";
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
DecisionComponent::ProcessDepthClusteringDetections(const
	std::shared_ptr<perception::PerceptionObstacles> depth_clustering_detections)
{
	if (!depth_clustering_detections)
	{
		AERROR << "Depth Clustering detections missing.";
        return;
	}

	AINFO << "Processing Depth Clustering detections.";

	auto depth_clustering_parameter_factory = depth_clustering::ParameterFactory(
		depth_clustering_config_file_name_);
	const auto& depth_clustering_parameter = depth_clustering_parameter_factory.getDepthClusteringParameter();
	const auto& bounding_box_type = depth_clustering_parameter.bounding_box_type;

	switch (bounding_box_type)
	{
	case depth_clustering::BoundingBox::Type::Cube:
	{
		AINFO << "Processing Depth Clustering cube detections.";
		ProcessDepthClusteringDetectionsCube(depth_clustering_detections);
		break;
	}
	case depth_clustering::BoundingBox::Type::Polygon:
	{
		AINFO << "Processing Depth Clustering polygon detections.";
		ProcessDepthClusteringDetectionsPolygon(depth_clustering_detections);
		break;
	}
	case depth_clustering::BoundingBox::Type::Flat:
	{
		AINFO << "Processing Depth Clustering flat detections.";
		ProcessDepthClusteringDetectionsFlat(depth_clustering_detections);
		break;
	}
	default:
	{
		AWARN << "Unknown bounding box type " << static_cast<int>(bounding_box_type) << ".";
		AINFO << "Processing Depth Clustering cube detections.";
		ProcessDepthClusteringDetectionsCube(depth_clustering_detections);
		break;
	}
	}
}

void
DecisionComponent::ProcessDepthClusteringDetectionsCube(
	const std::shared_ptr<perception::PerceptionObstacles> depth_clustering_detections)
{
	if (!depth_clustering_detections)
	{
		AERROR << "Depth Clustering detections missing.";
        return;
	}

	AWARN << "Not implemeneted for Depth Clustering cube detections.";
	override_ = false;
}

void
DecisionComponent::ProcessDepthClusteringDetectionsPolygon(
	const std::shared_ptr<perception::PerceptionObstacles> depth_clustering_detections)
{
	if (!depth_clustering_detections)
	{
		AERROR << "Depth Clustering detections missing.";
        return;
	}

	AWARN << "Not implemeneted for Depth Clustering polygon detections.";
	override_ = false;
}

void
DecisionComponent::ProcessDepthClusteringDetectionsFlat(
	const std::shared_ptr<perception::PerceptionObstacles> depth_clustering_detections)
{
	if (!depth_clustering_detections)
	{
		AERROR << "Depth Clustering detections missing.";
        return;
	}

	AWARN << "Not implemeneted for Depth Clustering flat detections.";
	override_ = false;
}
}	/* safety_layer */
}	/* apollo */
