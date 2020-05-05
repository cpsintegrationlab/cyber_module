#include <fstream>
#include <string>
#include <boost/property_tree/json_parser.hpp>

#include "modules/perception/base/image_8u.h"
#include "modules/safety_layer/component/lidar_component.h"

namespace apollo
{
namespace safety_layer
{
LidarComponent::LidarComponent() : point_cloud_reader_(nullptr), frame_counter_(0), log_(false)
{
}

LidarComponent::~LidarComponent()
{
	if (log_)
	{
		ground_truth_3d_log_file_.open(ground_truth_3d_log_file_name_, std::fstream::out | std::fstream::trunc);

		if (!ground_truth_3d_log_file_.is_open() || !ground_truth_3d_log_file_.good())
		{
			return;
		}

		boost::property_tree::write_json(ground_truth_3d_log_file_, ground_truth_3d_log_file_tree_);

		ground_truth_3d_log_file_.close();
	}

	depth_clustering_->finish();
}

bool
LidarComponent::Init()
{
	ground_truth_3d_reader_ = node_->CreateReader<common::Detection3DArray>(
		"/apollo/perception/ground_truth/3d_detections");
	point_cloud_reader_ = node_->CreateReader<drivers::PointCloud>(
		"/apollo/sensor/lidar128/compensator/PointCloud2");
	depth_clustering_detection_writer_ = node_->CreateWriter<common::Detection3DArray>(
		"/apollo/safety_layer/depth_clustering_detections");
	depth_clustering_ = std::make_shared<DepthClustering>(10, 10000, 7, 10, 5, log_);

	depth_clustering_->init_apollo_box();

	return true;
}

bool
LidarComponent::Proc()
{
	if (ground_truth_3d_reader_ == nullptr)
	{
		AERROR << "3D ground truth reader missing.";
		return false;
	}

	if (point_cloud_reader_ == nullptr)
	{
		AERROR << "Point cloud reader missing.";
		return false;
	}

	ground_truth_3d_reader_->Observe();
	point_cloud_reader_->Observe();

	const auto& ground_truth_3d = ground_truth_3d_reader_->GetLatestObserved();
	const auto& point_cloud = point_cloud_reader_->GetLatestObserved();

	if (point_cloud != nullptr)
	{
		ProcessPointCloud(point_cloud);
	}
	else
	{
		AERROR << "Point cloud message missing.";
	}

	if (log_)
	{
		if (point_cloud != nullptr && ground_truth_3d != nullptr)
		{
			LogGroundTruth3D(ground_truth_3d);
			LogPointCloud(point_cloud);

			frame_counter_ ++;
		}
		else
		{
			AERROR << "Logging message missing.";
		}
	}

	return true;
}

void
LidarComponent::ProcessGroundTruth3D(const
	std::shared_ptr<common::Detection3DArray> ground_truth_3d_message)
{
	AERROR << "Processing 3D ground truth.";
}

void
LidarComponent::ProcessPointCloud(const
	std::shared_ptr<drivers::PointCloud> point_cloud_message)
{
	AERROR << "Processing point cloud.";

	std::vector<Eigen::Vector3f> point_cloud;
	std::vector<std::pair<Eigen::Vector3f, Eigen::Vector3f>> output_box_frame;

	for (int i = 0; i < point_cloud_message->point_size(); i ++)
	{
		const apollo::drivers::PointXYZIT& point = point_cloud_message->point(i);

		if (std::isnan(point.x()) || std::isnan(point.y()) || std::isnan(point.z()))
		{
			continue;
		}

		Eigen::Vector3f point_eigen;

		point_eigen.x() = point.x();
		point_eigen.y() = point.y();
		point_eigen.z() = point.z();

		point_cloud.push_back(point_eigen);
	}

	std::string cloud_file_name = "frame_" + std::to_string(frame_counter_) + ".bin";
	output_box_frame = depth_clustering_->process_apollo_box(cloud_file_name, point_cloud);

	auto detection_3d_array = std::make_shared<common::Detection3DArray>();

	for (const auto& output_box : output_box_frame)
	{
		common::Detection3D detection_3d;

		detection_3d.mutable_bbox()->mutable_position()->mutable_position()->set_x(output_box.first.x());
		detection_3d.mutable_bbox()->mutable_position()->mutable_position()->set_y(output_box.first.y());
		detection_3d.mutable_bbox()->mutable_position()->mutable_position()->set_z(output_box.first.z());
		detection_3d.mutable_bbox()->mutable_size()->set_x(output_box.second.x());
		detection_3d.mutable_bbox()->mutable_size()->set_y(output_box.second.y());
		detection_3d.mutable_bbox()->mutable_size()->set_z(output_box.second.z());

		detection_3d_array->add_detections()->CopyFrom(detection_3d);
	}

	depth_clustering_detection_writer_->Write(detection_3d_array);
}

void
LidarComponent::LogGroundTruth3D(const
	std::shared_ptr<common::Detection3DArray> ground_truth_3d_message)
{
	AERROR << "Logging 3D ground truth.";

	std::string cloud_file_name = "frame_" + std::to_string(frame_counter_) + ".bin";
	boost::property_tree::ptree cloud_file_array;

	for (const auto& detection : ground_truth_3d_message->detections())
	{
		boost::property_tree::ptree cloud_object_array_value;
		boost::property_tree::ptree cloud_object_array;

		const auto& bounding_box = detection.bbox();

		cloud_object_array_value.put_value(bounding_box.position().position().x());
		cloud_object_array.push_back(std::make_pair("", cloud_object_array_value));

		cloud_object_array_value.put_value(bounding_box.position().position().y());
		cloud_object_array.push_back(std::make_pair("", cloud_object_array_value));

		cloud_object_array_value.put_value(bounding_box.position().position().z());
		cloud_object_array.push_back(std::make_pair("", cloud_object_array_value));

		cloud_object_array_value.put_value(bounding_box.size().x());
		cloud_object_array.push_back(std::make_pair("", cloud_object_array_value));

		cloud_object_array_value.put_value(bounding_box.size().y());
		cloud_object_array.push_back(std::make_pair("", cloud_object_array_value));

		cloud_object_array_value.put_value(bounding_box.size().z());
		cloud_object_array.push_back(std::make_pair("", cloud_object_array_value));

		cloud_object_array_value.put_value(bounding_box.position().orientation().qw());
		cloud_object_array.push_back(std::make_pair("", cloud_object_array_value));

		cloud_object_array_value.put_value(bounding_box.position().orientation().qx());
		cloud_object_array.push_back(std::make_pair("", cloud_object_array_value));

		cloud_object_array_value.put_value(bounding_box.position().orientation().qy());
		cloud_object_array.push_back(std::make_pair("", cloud_object_array_value));

		cloud_object_array_value.put_value(bounding_box.position().orientation().qz());
		cloud_object_array.push_back(std::make_pair("", cloud_object_array_value));

		cloud_object_array_value.put_value(detection.label());
		cloud_object_array.push_back(std::make_pair("", cloud_object_array_value));

		cloud_file_array.push_back(std::make_pair("", cloud_object_array));
	}

	ground_truth_3d_log_file_tree_.add_child(boost::property_tree::ptree::path_type(
			cloud_file_name, '/'), cloud_file_array);
}

void
LidarComponent::LogPointCloud(const
	std::shared_ptr<drivers::PointCloud> point_cloud_message)
{
	AERROR << "Logging point cloud.";

	std::string point_cloud_log_file_name = "/apollo/data/lidar/frame_" + std::to_string(frame_counter_) + ".bin";
	std::fstream point_cloud_log_file(point_cloud_log_file_name, std::ios::out | std::ios::binary);

	if (point_cloud_log_file.good())
	{
		point_cloud_log_file.seekg(0, std::ios::beg);

		for (int i = 0; i < point_cloud_message->point_size(); i ++)
		{
			const apollo::drivers::PointXYZIT& point = point_cloud_message->point(i);

			if (std::isnan(point.x()) || std::isnan(point.y()) || std::isnan(point.z()))
			{
				continue;
			}

			float point_x = point.x();
			float point_y = point.y();
			float point_z = point.z();
			unsigned point_intensity = point.intensity();

			point_cloud_log_file.write(reinterpret_cast<char*>(&point_x), sizeof(float));
			point_cloud_log_file.write(reinterpret_cast<char*>(&point_y), sizeof(float));
			point_cloud_log_file.write(reinterpret_cast<char*>(&point_z), sizeof(float));
			point_cloud_log_file.write(reinterpret_cast<char*>(&point_intensity), sizeof(float));
		}
	
		point_cloud_log_file.close();
	}
}
} // safety_layer
} // apollo
