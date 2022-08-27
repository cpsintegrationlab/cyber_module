#include <string>
#include <boost/property_tree/json_parser.hpp>

#include "modules/perception/base/image_8u.h"
#include "modules/safety_layer/component/lidar_component.h"

namespace apollo
{
namespace safety_layer
{
LidarComponent::LidarComponent() : ground_truth_3d_reader_(nullptr), point_cloud_reader_(nullptr), frame_writer_(
		nullptr), depth_clustering_detection_writer_(nullptr), depth_clustering_(nullptr), frame_counter_(0), log_(false)
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
}

bool
LidarComponent::Init()
{
	ground_truth_3d_reader_ = node_->CreateReader<perception::PerceptionObstacles>(
		"/apollo/perception/ground_truth/3d_detections");
	point_cloud_reader_ = node_->CreateReader<drivers::PointCloud>(
		"/apollo/sensor/lidar128/compensator/PointCloud2");
	frame_writer_ = node_->CreateWriter<Frame>(
		"/apollo/safety_layer/frame");
	depth_clustering_detection_writer_ = node_->CreateWriter<perception::PerceptionObstacles>(
		"/apollo/safety_layer/depth_clustering_detections");
	depth_clustering_ = std::make_shared<depth_clustering::DepthClustering>();

	if (!depth_clustering_->initializeForApollo("/apollo/modules/safety_layer/conf/depth_clustering_config.json",
		"/apollo/data/log"))
	{
		AERROR << "Failed to initialize Depth Clustering.";
		return false;
	}

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

	if (point_cloud != nullptr && ground_truth_3d != nullptr)
	{
		if (log_)
		{
			LogGroundTruth3D(ground_truth_3d);
			LogPointCloud(point_cloud);
		}

		auto frame = std::make_shared<Frame>();

		frame->set_counter(frame_counter_);
		frame_writer_->Write(frame);

		frame_counter_ ++;
	}
	else
	{
		AERROR << "Some messages missing.";
	}

	return true;
}

void
LidarComponent::ProcessGroundTruth3D(const
	std::shared_ptr<perception::PerceptionObstacles> ground_truth_3d_message)
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
	depth_clustering_->processOneFrameForApollo(cloud_file_name, point_cloud);

	auto bounding_box = depth_clustering_->getBoundingBox();

	switch (depth_clustering_->getParameter().bounding_box_type)
	{
	case depth_clustering::BoundingBox::Type::Cube:
	{
		auto perception_obstacles = std::make_shared<perception::PerceptionObstacles>();
		auto bounding_box_frame_cube = bounding_box->getFrameCube();

		for (const auto& bounding_box_cube : *bounding_box_frame_cube)
		{
			perception::PerceptionObstacle perception_obstacle;

			auto bounding_box_cube_center = std::get<0>(bounding_box_cube);
			auto bounding_box_cube_extent = std::get<1>(bounding_box_cube);
			auto bounding_box_cube_rotation = std::get<2>(bounding_box_cube);

			perception_obstacle.mutable_position()->set_x(bounding_box_cube_center.x());
			perception_obstacle.mutable_position()->set_y(bounding_box_cube_center.y());
			perception_obstacle.mutable_position()->set_z(bounding_box_cube_center.z());
			perception_obstacle.set_length(bounding_box_cube_extent.x());
			perception_obstacle.set_width(bounding_box_cube_extent.y());
			perception_obstacle.set_height(bounding_box_cube_extent.z());
			perception_obstacle.set_theta(bounding_box_cube_rotation);

			perception_obstacles->add_perception_obstacle()->CopyFrom(perception_obstacle);
		}

		depth_clustering_detection_writer_->Write(perception_obstacles);

		break;
	}
	case depth_clustering::BoundingBox::Type::Polygon:
	{
		auto perception_obstacles = std::make_shared<perception::PerceptionObstacles>();
		auto bounding_box_frame_polygon = bounding_box->getFramePolygon();

		for (const auto& bounding_box_polygon : *bounding_box_frame_polygon)
		{
			perception::PerceptionObstacle perception_obstacle;

			auto bounding_box_polygon_hull = std::get<0>(bounding_box_polygon);
			auto bounding_box_polygon_height = std::get<1>(bounding_box_polygon);

			for (const auto& bounding_box_polygon_hull_point : bounding_box_polygon_hull)
			{
				common::Point3D polygon_point;

				polygon_point.set_x(bounding_box_polygon_hull_point.x());
				polygon_point.set_y(bounding_box_polygon_hull_point.y());
				polygon_point.set_z(bounding_box_polygon_hull_point.z());

				perception_obstacle.add_polygon_point()->CopyFrom(polygon_point);
			}

			perception_obstacle.set_height(bounding_box_polygon_height);

			perception_obstacles->add_perception_obstacle()->CopyFrom(perception_obstacle);
		}

		depth_clustering_detection_writer_->Write(perception_obstacles);

		break;
	}
	case depth_clustering::BoundingBox::Type::Flat:
	{
		break;
	}
	default:
	{
		break;
	}
	}
}

void
LidarComponent::LogGroundTruth3D(const
	std::shared_ptr<perception::PerceptionObstacles> ground_truth_3d_message)
{
	AERROR << "Logging 3D ground truth.";

	std::string cloud_file_name = "frame_" + std::to_string(frame_counter_) + ".bin";
	boost::property_tree::ptree cloud_file_array;

	for (const auto& perception_obstacle : ground_truth_3d_message->perception_obstacle())
	{
		boost::property_tree::ptree cloud_object_array_value;
		boost::property_tree::ptree cloud_object_array;

		cloud_object_array_value.put_value(perception_obstacle.position().x());
		cloud_object_array.push_back(std::make_pair("", cloud_object_array_value));

		cloud_object_array_value.put_value(perception_obstacle.position().y());
		cloud_object_array.push_back(std::make_pair("", cloud_object_array_value));

		cloud_object_array_value.put_value(perception_obstacle.position().z());
		cloud_object_array.push_back(std::make_pair("", cloud_object_array_value));

		cloud_object_array_value.put_value(perception_obstacle.length());
		cloud_object_array.push_back(std::make_pair("", cloud_object_array_value));

		cloud_object_array_value.put_value(perception_obstacle.width());
		cloud_object_array.push_back(std::make_pair("", cloud_object_array_value));

		cloud_object_array_value.put_value(perception_obstacle.height());
		cloud_object_array.push_back(std::make_pair("", cloud_object_array_value));

		cloud_object_array_value.put_value(perception_obstacle.theta());
		cloud_object_array.push_back(std::make_pair("", cloud_object_array_value));

		cloud_object_array_value.put_value(perception_obstacle.type());
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
