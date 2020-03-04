#include <fstream>
#include <string>

#include "modules/perception/base/image_8u.h"
#include "modules/safety_layer/component/lidar_component.h"

namespace apollo
{
namespace safety_layer
{
LidarComponent::LidarComponent() : point_cloud_reader_(nullptr), frame_counter_(0)
{
}

bool
LidarComponent::Init()
{
	point_cloud_reader_ = node_->CreateReader<drivers::PointCloud>(
		"/apollo/sensor/lidar128/compensator/PointCloud2");

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

	OnPointCloudMessage(point_cloud);

	return true;
}

void
LidarComponent::OnPointCloudMessage(const
	std::shared_ptr<drivers::PointCloud> point_cloud_message)
{
	AERROR << "Received point cloud message.";

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