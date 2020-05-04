#include <limits>
#include <Eigen/Dense>

#include "modules/safety_layer/component/decision_component.h"

namespace apollo
{
namespace safety_layer
{
DecisionComponent::DecisionComponent() :
		chassis_reader_(nullptr), depth_clustering_detection_reader_(nullptr), control_command_reader_(
				nullptr), control_command_writer_(nullptr), braking_acceleration_(0.8 * 9.81), braking_distance_(
				std::numeric_limits<double>::infinity()), braking_slack_(10.0), override_(false), override_braking_percentage_(
				100.0)
{
}

bool
DecisionComponent::Init()
{
	chassis_reader_ = node_->CreateReader<canbus::Chassis>(
		"/apollo/canbus/chassis");
	depth_clustering_detection_reader_ = node_->CreateReader<common::Detection3DArray>(
		"/apollo/safety_layer/depth_clustering_detections");
	control_command_reader_ = node_->CreateReader<control::ControlCommand>(
		"/apollo/control");
	control_command_writer_ = node_->CreateWriter<control::ControlCommand>(
		"/apollo/safety_layer/control");

	return true;
}

bool
DecisionComponent::Proc()
{
	if (chassis_reader_ == nullptr)
	{
		AERROR << "Chassis reader missing.";
		return false;
	}

	if (depth_clustering_detection_reader_ == nullptr)
	{
		AERROR << "Depth clustering detection reader missing.";
		return false;
	}

	if (control_command_reader_ == nullptr)
	{
		AERROR << "Control command reader missing.";
		return false;
	}

	chassis_reader_->Observe();
	depth_clustering_detection_reader_->Observe();
	control_command_reader_->Observe();

	const auto& chassis = chassis_reader_->GetLatestObserved();
	const auto& depth_clustering_detection = depth_clustering_detection_reader_->GetLatestObserved();
	const auto& control_command = control_command_reader_->GetLatestObserved();

	if (chassis != nullptr)
	{
		ProcessChassis(chassis);
	}
	else
	{
		AERROR << "Chassis message missing.";
	}

	if (depth_clustering_detection != nullptr)
	{
		ProcessDepthClusteringDetection(depth_clustering_detection);
	}
	else
	{
		AERROR << "Depth clustering detection message missing.";
	}

	if (control_command != nullptr)
	{
		ProcessControlCommand(control_command);
	}
	else
	{
		AERROR << "Control command message missing.";
	}

	return true;
}

void
DecisionComponent::ProcessChassis(const std::shared_ptr<canbus::Chassis> chassis_message)
{
	AERROR << "Processing chassis.";

	float vehicle_speed_mps = chassis_message->speed_mps();

	braking_distance_ = (vehicle_speed_mps * vehicle_speed_mps) / (2 * braking_acceleration_);
}

void
DecisionComponent::ProcessDepthClusteringDetection(const
	std::shared_ptr<common::Detection3DArray> depth_clustering_detection_message)
{
	AERROR << "Processing depth clustering detections.";

	for (const auto& detection : depth_clustering_detection_message->detections())
	{
		const auto& bounding_box = detection.bbox();
		Eigen::Vector3d bounding_box_center;
		Eigen::Vector3d bounding_box_extent;

		bounding_box_center.x() = bounding_box.position().position().x();
		bounding_box_center.y() = bounding_box.position().position().y();
		bounding_box_center.z() = bounding_box.position().position().z();
		bounding_box_extent.x() = bounding_box.size().x();
		bounding_box_extent.y() = bounding_box.size().y();
		bounding_box_extent.z() = bounding_box.size().z();

		double bounding_box_volume = bounding_box_extent.x() * bounding_box_extent.y()
						* bounding_box_extent.z();

		if (bounding_box_volume <= 0.2)
		{
			continue;
		}

		if (bounding_box_center.norm() < braking_distance_ + braking_slack_)
		{
			override_ = true;
			break;
		}
	}
}

void
DecisionComponent::ProcessControlCommand(const
	std::shared_ptr<control::ControlCommand> control_command_message)
{
	AERROR << "Processing control command.";

	auto control_command = std::make_shared<control::ControlCommand>(*control_command_message);

	if (override_)
	{
		control_command->set_throttle(0);
		control_command->set_brake(override_braking_percentage_);
		AERROR << "Overridden.";
	}
	else
	{
		control_command->set_throttle(20);
		control_command->set_brake(0);
	}

	control_command_writer_->Write(control_command);
}

} // safety_layer
} // apollo
