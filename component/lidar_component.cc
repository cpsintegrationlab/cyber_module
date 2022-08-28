#include "modules/safety_layer/component/lidar_component.h"

namespace apollo
{
namespace safety_layer
{
LidarComponent::LidarComponent() : reader_point_cloud_(
	nullptr), writer_depth_clustering_detections_(nullptr), reader_point_cloud_channel_name_(
	"/apollo/sensor/lidar128/compensator/PointCloud2"), writer_depth_clustering_detections_channel_name_(
	"/apollo/safety_layer/lidar/depth_clustering/detections"), depth_clustering_(nullptr), depth_clustering_config_file_name_(
	"/apollo/modules/safety_layer/conf/depth_clustering.json"), depth_clustering_log_directory_(
	"/apollo/data/log")
{
}

LidarComponent::~LidarComponent()
{
}

bool
LidarComponent::Init()
{
	reader_point_cloud_ = node_->CreateReader<drivers::PointCloud>(
		reader_point_cloud_channel_name_);

    if (!reader_point_cloud_)
	{
		AWARN << "Failed to create point cloud reader.";
	}

	writer_depth_clustering_detections_ = node_->CreateWriter<perception::PerceptionObstacles>(
		writer_depth_clustering_detections_channel_name_);

	if (!writer_depth_clustering_detections_)
	{
		AWARN << "Failed to create Depth Clustering detections writer.";
	}

	depth_clustering_ = std::make_shared<depth_clustering::DepthClustering>();

	if (depth_clustering_)
	{
		if (!depth_clustering_->initializeForApollo(
			depth_clustering_config_file_name_, depth_clustering_log_directory_))
		{
			AERROR << "Failed to initialize Depth Clustering.";
			return false;
		}

		if (FLAGS_minloglevel > 0)
		{
			depth_clustering_->disableConsoleLogging();
		}
	}
	else
	{
		AWARN << "Failed to create Depth Clustering.";
	}

	return true;
}

bool
LidarComponent::Proc()
{
	if (reader_point_cloud_)
    {
        reader_point_cloud_->Observe();
		ProcessPointCloud(reader_point_cloud_->GetLatestObserved());
    }
    else
	{
		AWARN << "Point cloud reader missing.";
	}

	return true;
}

void
LidarComponent::ProcessPointCloud(const
	std::shared_ptr<drivers::PointCloud> point_cloud)
{
	if (!point_cloud)
	{
		AERROR << "Point cloud missing.";
        return;
	}

	if (!depth_clustering_)
	{
		AERROR << "Depth clustering missing.";
        return;
	}

	AINFO << "Processing point cloud.";

	const std::string sequence_num = std::to_string(point_cloud->header().sequence_num());
	const std::string frame_name = sequence_num + ".bin";
	std::vector<Eigen::Vector3f> point_cloud_eigen;

	for (int i = 0; i < point_cloud->point_size(); i ++)
	{
		const auto& point = point_cloud->point(i);

		if (std::isnan(point.x()) || std::isnan(point.y()) || std::isnan(point.z()))
		{
			continue;
		}

		Eigen::Vector3f point_eigen;

		point_eigen.x() = point.x();
		point_eigen.y() = point.y();
		point_eigen.z() = point.z();

		point_cloud_eigen.push_back(point_eigen);
	}

	depth_clustering_->processOneFrameForApollo(frame_name, point_cloud_eigen);

	const auto bounding_box = depth_clustering_->getBoundingBox();
	const auto& bounding_box_type = depth_clustering_->getParameter().bounding_box_type;

	switch (bounding_box_type)
	{
	case depth_clustering::BoundingBox::Type::Cube:
	{
		AINFO << "Writing Depth Clustering cube detections.";
		writeCyberDepthClusteringDetectionsCube(bounding_box);
		break;
	}
	case depth_clustering::BoundingBox::Type::Polygon:
	{
		AINFO << "Writing Depth Clustering polygon detections.";
		writeCyberDepthClusteringDetectionsPolygon(bounding_box);
		break;
	}
	case depth_clustering::BoundingBox::Type::Flat:
	{
		AINFO << "Writing Depth Clustering flat detections.";
		writeCyberDepthClusteringDetectionsFlat(bounding_box);
		break;
	}
	default:
	{
		AWARN << "Unknown bounding box type " << static_cast<int>(bounding_box_type) << ".";
		AINFO << "Writing Depth Clustering cube detections.";
		writeCyberDepthClusteringDetectionsCube(bounding_box);
		break;
	}
	}
}

void
LidarComponent::writeCyberDepthClusteringDetectionsCube(
	const std::shared_ptr<depth_clustering::BoundingBox> bounding_box)
{
	if (!writer_depth_clustering_detections_)
	{
		AERROR << "Depth Clustering detections writer missing.";
        return;
	}

	auto perception_obstacles = std::make_shared<perception::PerceptionObstacles>();
	auto bounding_box_frame_cube = bounding_box->getFrameCube();

	for (const auto& bounding_box_cube : *bounding_box_frame_cube)
	{
		perception::PerceptionObstacle perception_obstacle;

		const auto& bounding_box_cube_center = std::get<0>(bounding_box_cube);
		const auto& bounding_box_cube_extent = std::get<1>(bounding_box_cube);
		const auto& bounding_box_cube_rotation = std::get<2>(bounding_box_cube);

		perception_obstacle.mutable_position()->set_x(bounding_box_cube_center.x());
		perception_obstacle.mutable_position()->set_y(bounding_box_cube_center.y());
		perception_obstacle.mutable_position()->set_z(bounding_box_cube_center.z());
		perception_obstacle.set_length(bounding_box_cube_extent.x());
		perception_obstacle.set_width(bounding_box_cube_extent.y());
		perception_obstacle.set_height(bounding_box_cube_extent.z());
		perception_obstacle.set_theta(bounding_box_cube_rotation);

		perception_obstacles->add_perception_obstacle()->CopyFrom(perception_obstacle);
	}

	writer_depth_clustering_detections_->Write(perception_obstacles);
}

void
LidarComponent::writeCyberDepthClusteringDetectionsPolygon(
	const std::shared_ptr<depth_clustering::BoundingBox> bounding_box)
{
	if (!writer_depth_clustering_detections_)
	{
		AERROR << "Depth Clustering detections writer missing.";
        return;
	}

	auto perception_obstacles = std::make_shared<perception::PerceptionObstacles>();
	auto bounding_box_frame_polygon = bounding_box->getFramePolygon();

	for (const auto& bounding_box_polygon : *bounding_box_frame_polygon)
	{
		perception::PerceptionObstacle perception_obstacle;

		const auto& bounding_box_polygon_hull = std::get<0>(bounding_box_polygon);
		const auto& bounding_box_polygon_height = std::get<1>(bounding_box_polygon);

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

	writer_depth_clustering_detections_->Write(perception_obstacles);
}

void
LidarComponent::writeCyberDepthClusteringDetectionsFlat(
	const std::shared_ptr<depth_clustering::BoundingBox> bounding_box)
{
	if (!writer_depth_clustering_detections_)
	{
		AERROR << "Depth Clustering detections writer missing.";
        return;
	}

	auto perception_obstacles = std::make_shared<perception::PerceptionObstacles>();
	auto bounding_box_frame_flat = bounding_box->getFrameFlat();

	for (const auto& bounding_box_flat : *bounding_box_frame_flat)
	{
		perception::PerceptionObstacle perception_obstacle;

		const auto& bounding_box_flat_point_upper_left = std::get<0>(bounding_box_flat);
		const auto& bounding_box_flat_point_lower_right = std::get<1>(bounding_box_flat);

		perception_obstacle.mutable_bbox2d()->set_xmin(bounding_box_flat_point_upper_left.x());
		perception_obstacle.mutable_bbox2d()->set_ymin(bounding_box_flat_point_upper_left.y());
		perception_obstacle.mutable_bbox2d()->set_xmax(bounding_box_flat_point_lower_right.x());
		perception_obstacle.mutable_bbox2d()->set_ymax(bounding_box_flat_point_lower_right.y());

		perception_obstacles->add_perception_obstacle()->CopyFrom(perception_obstacle);
	}

	writer_depth_clustering_detections_->Write(perception_obstacles);
}
}	/* safety_layer */
}	/* apollo */
