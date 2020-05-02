#include <fstream>
#include <string>

#include "modules/perception/base/image_8u.h"
#include "modules/safety_layer/component/lidar_component.h"

namespace apollo
{
namespace safety_layer
{
LidarComponent::LidarComponent() : point_cloud_reader_(nullptr), frame_counter_(0), log_(false)
{
}

bool
LidarComponent::Init()
{
	point_cloud_reader_ = node_->CreateReader<drivers::PointCloud>(
		"/apollo/sensor/lidar128/compensator/PointCloud2");
	depth_clustering_ = std::make_shared<DepthClustering>();

	depth_clustering_->init_apollo_box();

	return true;
}

bool
LidarComponent::Proc()
{
	if (point_cloud_reader_ == nullptr)
	{
		AERROR << "Point cloud reader missing.";
		return false;
	}

	point_cloud_reader_->Observe();
	const auto& point_cloud = point_cloud_reader_->GetLatestObserved();

	if (point_cloud == nullptr)
	{
		return false;
	}

	ProcessPointCloud(point_cloud);

	if (log_)
	{
		LogPointCloud(point_cloud);
	}

	return true;
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

	output_box_frame = depth_clustering_->process_apollo_box(std::to_string(frame_counter_), point_cloud);
}

void
LidarComponent::LogPointCloud(const
	std::shared_ptr<drivers::PointCloud> point_cloud_message)
{
	AERROR << "Logging point cloud.";

	std::string point_cloud_file_name = "/apollo/data/lidar/frame_" + std::to_string(frame_counter_) + ".bin";
	std::fstream point_cloud_file(point_cloud_file_name, std::ios::out | std::ios::binary);

	if (point_cloud_file.good())
	{
		point_cloud_file.seekg(0, std::ios::beg);

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

			point_cloud_file.write(reinterpret_cast<char*>(&point_x), sizeof(float));
			point_cloud_file.write(reinterpret_cast<char*>(&point_y), sizeof(float));
			point_cloud_file.write(reinterpret_cast<char*>(&point_z), sizeof(float));
			point_cloud_file.write(reinterpret_cast<char*>(&point_intensity), sizeof(float));
		}
	
		point_cloud_file.close();
		frame_counter_ ++;
	}
}
} // safety_layer
} // apollo
