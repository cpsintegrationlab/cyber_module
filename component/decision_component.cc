#include <ctime>
#include <limits>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/date_time/posix_time/posix_time_io.hpp>

#include "modules/safety_layer/component/decision_component.h"

namespace apollo
{
namespace safety_layer
{
DecisionComponent::DecisionComponent() :
		frame_reader_(nullptr), chassis_reader_(nullptr), depth_clustering_detection_reader_(
				nullptr), control_command_reader_(nullptr), control_command_writer_(nullptr), cruise_(
				false), target_speed_mps_(15.0), braking_acceleration_(0.8 * 9.81), braking_distance_(
				0.0), braking_slack_(10.0), restart_slack_(15.0), override_(false), override_braking_percentage_(
				100.0), frame_counter_(0), log_(false)
{
}

DecisionComponent::~DecisionComponent()
{
	chassis_log_file_.close();
}

bool
DecisionComponent::Init()
{
	frame_reader_ = node_->CreateReader<Frame>(
		"/apollo/safety_layer/frame");
	chassis_reader_ = node_->CreateReader<canbus::Chassis>(
		"/apollo/canbus/chassis");
	depth_clustering_detection_reader_ = node_->CreateReader<common::Detection3DArray>(
		"/apollo/safety_layer/depth_clustering_detections");
	control_command_reader_ = node_->CreateReader<control::ControlCommand>(
		"/apollo/control");
	control_command_writer_ = node_->CreateWriter<control::ControlCommand>(
		"/apollo/safety_layer/control");

	if (log_)
	{
		chassis_log_file_.open(chassis_log_file_name_, std::fstream::out | std::fstream::trunc);

		if (chassis_log_file_.is_open() && chassis_log_file_.good())
		{
			chassis_log_file_ << "Target Vehicle Speed (mps)\tBraking Slack (m)" << std::endl;
			chassis_log_file_ << target_speed_mps_ << "\t" << braking_slack_ << std::endl;
			chassis_log_file_ << std::endl;
			chassis_log_file_ << "Timestamp\tFrame Counter\tVehicle Speed (mps)\tBraking Distance (m)" << std::endl;
		}
		else
		{
			AERROR << "Failed to open chassis log file.";
		}
	}

	return true;
}

bool
DecisionComponent::Proc()
{
	if (frame_reader_ == nullptr)
	{
		AERROR << "Frame reader missing.";
		return false;
	}

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

	frame_reader_->Observe();
	chassis_reader_->Observe();
	depth_clustering_detection_reader_->Observe();
	control_command_reader_->Observe();

	const auto& frame = frame_reader_->GetLatestObserved();
	const auto& chassis = chassis_reader_->GetLatestObserved();
	const auto& depth_clustering_detection = depth_clustering_detection_reader_->GetLatestObserved();
	const auto& control_command = control_command_reader_->GetLatestObserved();

	if (frame != nullptr)
	{
		ProcessFrame(frame);
	}
	else
	{
		AERROR << "Frame message missing.";
	}

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
DecisionComponent::ProcessFrame(const std::shared_ptr<Frame> frame_message)
{
	AERROR << "Processing frame.";

	frame_counter_ = frame_message->counter();
}

void
DecisionComponent::ProcessChassis(const std::shared_ptr<canbus::Chassis> chassis_message)
{
	AERROR << "Processing chassis.";

	float vehicle_speed_mps = chassis_message->speed_mps();

	if (!cruise_ && vehicle_speed_mps >= target_speed_mps_)
	{
		cruise_ = true;
	}

	braking_distance_ = (vehicle_speed_mps * vehicle_speed_mps) / (2 * braking_acceleration_);

	if (log_)
	{
		if (chassis_log_file_.is_open() && chassis_log_file_.good())
		{
			boost::posix_time::ptime timestamp = boost::posix_time::microsec_clock::universal_time();

			chassis_log_file_ << to_simple_string(timestamp) << "\t" << frame_counter_ << "\t" << vehicle_speed_mps
					<< "\t" << braking_distance_ << std::endl;
		}
		else
		{
			AERROR << "Chassis log file object missing.";
		}
	}
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

		if (bounding_box_center.z() <= -2.2)
		{
			continue;
		}

		double bounding_box_distance = CalculateBoundingBoxDistance(bounding_box_center, bounding_box_extent);

		if (bounding_box_distance > braking_distance_ + restart_slack_ && override_ == true)
		{
			override_ = false;
			cruise_ = false;
			break;
		}

		if (bounding_box_distance < braking_distance_ + braking_slack_ && override_ == false)
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
		if (cruise_)
		{
			control_command->set_throttle(5);
			control_command->set_brake(0);
		}
		else
		{
			control_command->set_throttle(100);
			control_command->set_brake(0);
		}
	}

	control_command_writer_->Write(control_command);
}

double
DecisionComponent::CalculateBoundingBoxDistance(const Eigen::Vector3d& center, const Eigen::Vector3d& extent)
{
	Eigen::Vector3d bounding_box_vertex;
	std::vector<double> bounding_box_vertex_distances;
	double bounding_box_vertex_distance_min = std::numeric_limits<double>::max();

	bounding_box_vertex.x() = center.x() - (extent.x() / 2.0);
	bounding_box_vertex.y() = center.y() - (extent.y() / 2.0);
	bounding_box_vertex.z() = center.z() - (extent.z() / 2.0);
	bounding_box_vertex_distances.push_back(bounding_box_vertex.norm());

	bounding_box_vertex.x() = center.x() - (extent.x() / 2.0);
	bounding_box_vertex.y() = center.y() - (extent.y() / 2.0);
	bounding_box_vertex.z() = center.z() + (extent.z() / 2.0);
	bounding_box_vertex_distances.push_back(bounding_box_vertex.norm());

	bounding_box_vertex.x() = center.x() - (extent.x() / 2.0);
	bounding_box_vertex.y() = center.y() + (extent.y() / 2.0);
	bounding_box_vertex.z() = center.z() - (extent.z() / 2.0);
	bounding_box_vertex_distances.push_back(bounding_box_vertex.norm());

	bounding_box_vertex.x() = center.x() - (extent.x() / 2.0);
	bounding_box_vertex.y() = center.y() + (extent.y() / 2.0);
	bounding_box_vertex.z() = center.z() + (extent.z() / 2.0);
	bounding_box_vertex_distances.push_back(bounding_box_vertex.norm());

	bounding_box_vertex.x() = center.x() + (extent.x() / 2.0);
	bounding_box_vertex.y() = center.y() - (extent.y() / 2.0);
	bounding_box_vertex.z() = center.z() - (extent.z() / 2.0);
	bounding_box_vertex_distances.push_back(bounding_box_vertex.norm());

	bounding_box_vertex.x() = center.x() + (extent.x() / 2.0);
	bounding_box_vertex.y() = center.y() - (extent.y() / 2.0);
	bounding_box_vertex.z() = center.z() + (extent.z() / 2.0);
	bounding_box_vertex_distances.push_back(bounding_box_vertex.norm());

	bounding_box_vertex.x() = center.x() + (extent.x() / 2.0);
	bounding_box_vertex.y() = center.y() + (extent.y() / 2.0);
	bounding_box_vertex.z() = center.z() - (extent.z() / 2.0);
	bounding_box_vertex_distances.push_back(bounding_box_vertex.norm());

	bounding_box_vertex.x() = center.x() + (extent.x() / 2.0);
	bounding_box_vertex.y() = center.y() + (extent.y() / 2.0);
	bounding_box_vertex.z() = center.z() + (extent.z() / 2.0);
	bounding_box_vertex_distances.push_back(bounding_box_vertex.norm());

	for (const auto& bounding_box_vertex_distance : bounding_box_vertex_distances)
	{
		if (bounding_box_vertex_distance < bounding_box_vertex_distance_min)
		{
			bounding_box_vertex_distance_min = bounding_box_vertex_distance;
		}
	}

	return bounding_box_vertex_distance_min;
}
} // safety_layer
} // apollo
