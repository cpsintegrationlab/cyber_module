#include <boost/filesystem.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <fstream>

#include "modules/safety_layer/component/logging_component.h"

namespace apollo
{
namespace safety_layer
{
LoggingComponent::LoggingComponent() : reader_ground_truth_3d_(
    nullptr), reader_point_cloud_(nullptr), channel_name_reader_ground_truth_3d_(
    "/apollo/perception/ground_truth/3d_detections"), channel_name_reader_point_cloud_(
    "/apollo/sensor/lidar128/compensator/PointCloud2"), log_(false), log_directory_name_point_cloud_(
    "/apollo/data/log/safety_layer.lidar.log.d"), log_file_name_ground_truth_3d_(
    "/apollo/data/log/safety_layer.ground_truth_3d.log.json")
{
}

LoggingComponent::~LoggingComponent()
{
    writeLogGroundTruth3D();
}

bool
LoggingComponent::Init()
{
	reader_ground_truth_3d_ = node_->CreateReader<perception::PerceptionObstacles>(
		channel_name_reader_ground_truth_3d_);

    if (!reader_ground_truth_3d_)
	{
		AWARN << "Failed to create 3D ground truth reader.";
	}

	reader_point_cloud_ = node_->CreateReader<drivers::PointCloud>(
		channel_name_reader_point_cloud_);

    if (!reader_point_cloud_)
	{
		AWARN << "Failed to create point cloud reader.";
	}
    else
    {
        createLogDirectoryPointCloud();
    }

	return true;
}

bool
LoggingComponent::Proc()
{
    if (!log_)
    {
        return true;
    }

	if (reader_ground_truth_3d_)
    {
        reader_ground_truth_3d_->Observe();
        LogGroundTruth3D(reader_ground_truth_3d_->GetLatestObserved());
    }
    else
	{
		AWARN << "3D ground truth reader missing.";
	}

	if (reader_point_cloud_)
    {
        reader_point_cloud_->Observe();
        LogPointCloud(reader_point_cloud_->GetLatestObserved());
    }
    else
	{
		AWARN << "Point cloud reader missing.";
	}

	return true;
}

void
LoggingComponent::createLogDirectoryPointCloud()
{
    boost::filesystem::path log_directory_point_cloud(log_directory_name_point_cloud_);

    if(boost::filesystem::exists(log_directory_point_cloud))
    {
        if (!boost::filesystem::remove_all(log_directory_point_cloud))
        {
            AWARN << "Failed to remove existing point cloud log directory.";
        }
    }

    if (!boost::filesystem::create_directory(log_directory_point_cloud))
    {
        AWARN << "Failed to create point cloud log directory.";
    }
}

void
LoggingComponent::LogGroundTruth3D(const
	std::shared_ptr<perception::PerceptionObstacles> message)
{
    if (!message)
	{
		AERROR << "3D ground truth message missing.";
        return;
	}

	AINFO << "Logging 3D ground truth.";

    const std::string sequence_num = std::to_string(message->header().sequence_num());
	boost::property_tree::ptree log_file_tree_perception_obstacles;

	for (const auto& perception_obstacle : message->perception_obstacle())
	{
		boost::property_tree::ptree log_file_tree_perception_obstacle;
		boost::property_tree::ptree log_file_tree_perception_obstacle_entry;

		log_file_tree_perception_obstacle_entry.put_value(perception_obstacle.position().x());
		log_file_tree_perception_obstacle.push_back(std::make_pair("", log_file_tree_perception_obstacle_entry));

		log_file_tree_perception_obstacle_entry.put_value(perception_obstacle.position().y());
		log_file_tree_perception_obstacle.push_back(std::make_pair("", log_file_tree_perception_obstacle_entry));

		log_file_tree_perception_obstacle_entry.put_value(perception_obstacle.position().z());
		log_file_tree_perception_obstacle.push_back(std::make_pair("", log_file_tree_perception_obstacle_entry));

		log_file_tree_perception_obstacle_entry.put_value(perception_obstacle.length());
		log_file_tree_perception_obstacle.push_back(std::make_pair("", log_file_tree_perception_obstacle_entry));

		log_file_tree_perception_obstacle_entry.put_value(perception_obstacle.width());
		log_file_tree_perception_obstacle.push_back(std::make_pair("", log_file_tree_perception_obstacle_entry));

		log_file_tree_perception_obstacle_entry.put_value(perception_obstacle.height());
		log_file_tree_perception_obstacle.push_back(std::make_pair("", log_file_tree_perception_obstacle_entry));

		log_file_tree_perception_obstacle_entry.put_value(perception_obstacle.theta());
		log_file_tree_perception_obstacle.push_back(std::make_pair("", log_file_tree_perception_obstacle_entry));

		log_file_tree_perception_obstacle_entry.put_value(perception_obstacle.type());
		log_file_tree_perception_obstacle.push_back(std::make_pair("", log_file_tree_perception_obstacle_entry));

		log_file_tree_perception_obstacles.push_back(std::make_pair("", log_file_tree_perception_obstacle));
	}

	log_file_tree_ground_truth_3d_.add_child(boost::property_tree::ptree::path_type(
        sequence_num, '/'), log_file_tree_perception_obstacles);
}

void
LoggingComponent::LogPointCloud(const
	std::shared_ptr<drivers::PointCloud> message)
{
    if (!message)
	{
		AERROR << "Point cloud message missing.";
        return;
	}

	AINFO << "Logging point cloud.";

    if(!boost::filesystem::exists(boost::filesystem::path(log_directory_name_point_cloud_)))
    {
        AERROR << "Point cloud log directory missing.";
        return;
    }

    const std::string sequence_num = std::to_string(message->header().sequence_num());
	const std::string log_file_name = log_directory_name_point_cloud_ + "/" + sequence_num + ".bin";
	std::fstream log_file(log_file_name, std::ios::out | std::fstream::trunc | std::ios::binary);

    if (!log_file.is_open() || !log_file.good())
    {
        AERROR << "Failed to open point cloud log file.";
        return;
    }

    log_file.seekg(0, std::ios::beg);

    for (int i = 0; i < message->point_size(); i ++)
    {
        const auto& point = message->point(i);

        if (std::isnan(point.x()) || std::isnan(point.y()) || std::isnan(point.z()))
        {
            continue;
        }

        float point_x = point.x();
        float point_y = point.y();
        float point_z = point.z();
        unsigned point_intensity = point.intensity();

        log_file.write(reinterpret_cast<char*>(&point_x), sizeof(float));
        log_file.write(reinterpret_cast<char*>(&point_y), sizeof(float));
        log_file.write(reinterpret_cast<char*>(&point_z), sizeof(float));
        log_file.write(reinterpret_cast<char*>(&point_intensity), sizeof(float));
    }

    log_file.close();
}

void
LoggingComponent::writeLogGroundTruth3D()
{
    if (!log_)
    {
        return;
    }

    std::ofstream log_file(log_file_name_ground_truth_3d_, std::fstream::out | std::fstream::trunc);

    if (!log_file.is_open() || !log_file.good())
    {
        AERROR << "Failed to open 3D ground truth log file.";
        return;
    }

    boost::property_tree::write_json(log_file, log_file_tree_ground_truth_3d_);

    log_file.close();
}
}   /* safety_layer */
}   /* apollo */
