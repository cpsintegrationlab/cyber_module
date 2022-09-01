#include <boost/filesystem.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <fstream>

#include "modules/safety_layer/component/logging_component.h"

namespace apollo
{
namespace safety_layer
{
LoggingComponent::LoggingComponent() : reader_chassis_(nullptr), reader_ground_truth_3d_(
    nullptr), reader_point_cloud_(nullptr), channel_name_reader_chassis_(
    "/apollo/canbus/chassis"), channel_name_reader_ground_truth_3d_(
    "/apollo/perception/ground_truth/3d_detections"), channel_name_reader_point_cloud_(
    "/apollo/sensor/lidar128/compensator/PointCloud2"), log_(true), log_directory_name_point_cloud_(
    "/apollo/data/log/safety_layer.point_cloud.log.d"), log_directory_name_verifiable_obstacle_detection_(
    "/apollo/data/log/safety_layer.verifiable_obstacle_detection.log.d"), log_file_name_chassis_(
    "/apollo/data/log/safety_layer.chassis.log.txt"), log_file_name_ground_truth_3d_(
    "/apollo/data/log/safety_layer.ground_truth_3d.log.json")
{
    if (FLAGS_minloglevel > 0)
    {
        log_ = false;
    }
}

LoggingComponent::~LoggingComponent()
{
    if (!log_)
    {
        return;
    }

    log_file_chassis_.close();

    writeLogGroundTruth3D();
}

bool
LoggingComponent::Init()
{
    if (!log_)
    {
        return true;
    }

    reader_chassis_ = node_->CreateReader<canbus::Chassis>(
        channel_name_reader_chassis_);

    if (!reader_chassis_)
	{
		AWARN << "Failed to create chassis reader.";
	}
    else
    {
        createLogFileChassis();
    }

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

	createLogDirectoryVerifiableObstacleDetection();

	return true;
}

bool
LoggingComponent::Proc()
{
    if (!log_)
    {
        return true;
    }

    if (reader_chassis_)
    {
        reader_chassis_->Observe();
        LogChassis(reader_chassis_->GetLatestObserved());
    }
    else
	{
		AWARN << "Chassis reader missing.";
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
    if (!log_)
    {
        return;
    }

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
LoggingComponent::createLogDirectoryVerifiableObstacleDetection()
{
    if (!log_)
    {
        return;
    }

    boost::filesystem::path log_directory_verifiable_obstacle_detection(log_directory_name_verifiable_obstacle_detection_);

    if(boost::filesystem::exists(log_directory_verifiable_obstacle_detection))
    {
        if (!boost::filesystem::remove_all(log_directory_verifiable_obstacle_detection))
        {
            AWARN << "Failed to remove existing Verifiable Obstacle Detection log directory.";
        }
    }

    if (!boost::filesystem::create_directory(log_directory_verifiable_obstacle_detection))
    {
        AWARN << "Failed to create Verifiable Obstacle Detection log directory.";
    }
}

void
LoggingComponent::createLogFileChassis()
{
    if (!log_)
    {
        return;
    }

    log_file_chassis_.open(log_file_name_chassis_, std::fstream::out | std::fstream::trunc);

    if (log_file_chassis_.is_open() && log_file_chassis_.good())
    {
        log_file_chassis_ << "Sequence Number\tSpeed (m/s)" << std::endl;
    }
    else
    {
        AERROR << "Failed to open chassis log file.";
    }
}

void
LoggingComponent::LogChassis(const std::shared_ptr<canbus::Chassis> chassis)
{
    if (!log_)
    {
        return;
    }

    if (!chassis)
	{
		AERROR << "Chassis missing.";
        return;
	}

	AINFO << "Logging chassis.";

    const unsigned& sequence_num = chassis->header().sequence_num();
    const float& speed_mps = chassis->speed_mps();

    if (log_file_chassis_.is_open() && log_file_chassis_.good())
    {
        log_file_chassis_ << sequence_num << "\t" << speed_mps << std::endl;
    }
    else
    {
        AERROR << "Invalid chassis log file stream.";
    }
}

void
LoggingComponent::LogGroundTruth3D(const
	std::shared_ptr<perception::PerceptionObstacles> ground_truth_3d)
{
    if (!log_)
    {
        return;
    }

    if (!ground_truth_3d)
	{
		AERROR << "3D ground truth missing.";
        return;
	}

	AINFO << "Logging 3D ground truth.";

    const std::string sequence_num = std::to_string(ground_truth_3d->header().sequence_num());
	boost::property_tree::ptree log_file_tree_perception_obstacles;

	for (const auto& perception_obstacle : ground_truth_3d->perception_obstacle())
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
	std::shared_ptr<drivers::PointCloud> point_cloud)
{
    if (!log_)
    {
        return;
    }

    if (!point_cloud)
	{
		AERROR << "Point cloud missing.";
        return;
	}

	AINFO << "Logging point cloud.";

    if(!boost::filesystem::exists(boost::filesystem::path(log_directory_name_point_cloud_)))
    {
        AERROR << "Point cloud log directory missing.";
        return;
    }

    const std::string sequence_num = std::to_string(point_cloud->header().sequence_num());
	const std::string log_file_name = log_directory_name_point_cloud_ + "/" + sequence_num + ".bin";
	std::fstream log_file(log_file_name, std::ios::out | std::fstream::trunc | std::ios::binary);

    if (!log_file.is_open() || !log_file.good())
    {
        AERROR << "Failed to open point cloud log file.";
        return;
    }

    log_file.seekg(0, std::ios::beg);

    for (int i = 0; i < point_cloud->point_size(); i ++)
    {
        const auto& point = point_cloud->point(i);

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
