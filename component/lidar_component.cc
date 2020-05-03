#include <fstream>
#include <string>
#include <boost/property_tree/json_parser.hpp>

#include "modules/perception/base/image_8u.h"
#include "modules/safety_layer/component/lidar_component.h"

namespace apollo
{
namespace safety_layer
{
LidarComponent::LidarComponent() : point_cloud_reader_(nullptr), frame_counter_(0), log_(true)
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
	depth_clustering_ = std::make_shared<DepthClustering>(10, 10000, 5, 10, 9, true);

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

	if (ground_truth_3d == nullptr)
	{
		return false;
	}

	if (point_cloud == nullptr)
	{
		return false;
	}

	ProcessPointCloud(point_cloud);

	if (log_)
	{
		LogGroundTruth3D(ground_truth_3d);
		LogPointCloud(point_cloud);

		frame_counter_ ++;
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
