#include "modules/safety_layer/component/decision_component.h"

namespace apollo
{
namespace safety_layer
{
DecisionComponent::DecisionComponent() :
		depth_clustering_detection_reader_(nullptr), control_command_writer_(nullptr)
{
}

bool
DecisionComponent::Init()
{
	depth_clustering_detection_reader_ = node_->CreateReader<common::Detection3DArray>(
		"/apollo/safety_layer/depth_clustering_detections");
	control_command_writer_ = node_->CreateWriter<control::ControlCommand>(
		"/apollo/control");

	return true;
}

bool
DecisionComponent::Proc()
{
	if (depth_clustering_detection_reader_ == nullptr)
	{
		AERROR << "Depth clustering detection reader missing.";
		return false;
	}

	depth_clustering_detection_reader_->Observe();

	const auto& depth_clustering_detection = depth_clustering_detection_reader_->GetLatestObserved();

	if (depth_clustering_detection == nullptr)
	{
		return false;
	}

	ProcessDepthClusteringDetection(depth_clustering_detection);

	return true;
}

void
DecisionComponent::ProcessDepthClusteringDetection(const
	std::shared_ptr<common::Detection3DArray> depth_clustering_detection_message)
{
	AERROR << "Processing depth clustering detections.";

	auto control_command = std::make_shared<control::ControlCommand>();

	control_command->set_throttle(50);
	control_command->set_brake(0);

	control_command_writer_->Write(control_command);
}

} // safety_layer
} // apollo
