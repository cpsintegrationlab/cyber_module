#include <fstream>
#include <memory>

#include "modules/safety_layer/lib/depth_clustering/src/depth_clustering/api/api.h"
#include "cyber/class_loader/class_loader.h"
#include "cyber/component/component.h"
#include "modules/perception/proto/perception_obstacle.pb.h"
#include "modules/drivers/proto/pointcloud.pb.h"
#include "cyber/component/timer_component.h"

namespace apollo
{
namespace safety_layer
{
class LidarComponent final : public cyber::TimerComponent
{
public:

	LidarComponent();

	~LidarComponent();

	bool
	Init() override;

	bool
	Proc() override;

private:

	void
	createLogFileTiming();

	void
	logTiming(const int& timing);

	void
	ProcessPointCloud(const std::shared_ptr<drivers::PointCloud> point_cloud);

	void
	writeCyberDepthClusteringDetectionsCube(
		const std::shared_ptr<depth_clustering::BoundingBox> bounding_box);

	void
	writeCyberDepthClusteringDetectionsPolygon(
		const std::shared_ptr<depth_clustering::BoundingBox> bounding_box);

	void
	writeCyberDepthClusteringDetectionsFlat(
		const std::shared_ptr<depth_clustering::BoundingBox> bounding_box);

	std::shared_ptr<cyber::Reader<drivers::PointCloud>> reader_point_cloud_;
	std::shared_ptr<cyber::Writer<perception::PerceptionObstacles>> writer_depth_clustering_detections_;
	const std::string reader_point_cloud_channel_name_;
	const std::string writer_depth_clustering_detections_channel_name_;

	std::shared_ptr<depth_clustering::DepthClustering> depth_clustering_;
	const std::string depth_clustering_config_file_name_;
	const std::string depth_clustering_log_directory_;

	bool log_;
	std::ofstream log_file_timing_;
	const std::string log_file_name_timing_;
};

CYBER_REGISTER_COMPONENT(LidarComponent)
}	/* safety_layer */
}	/* apollo */
