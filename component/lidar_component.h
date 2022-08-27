#include <fstream>
#include <memory>
#include <boost/property_tree/ptree.hpp>

#include "cyber/class_loader/class_loader.h"
#include "cyber/component/component.h"
#include "cyber/component/timer_component.h"
#include "modules/drivers/proto/pointcloud.pb.h"
#include "modules/perception/proto/perception_obstacle.pb.h"
#include "modules/safety_layer/proto/frame.pb.h"
#include "modules/safety_layer/lib/depth_clustering/src/depth_clustering/api/api.h"

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
	ProcessGroundTruth3D(const std::shared_ptr<perception::PerceptionObstacles> ground_truth_3d_message);

	void
	ProcessPointCloud(const std::shared_ptr<drivers::PointCloud> point_cloud_message);

	void
	LogGroundTruth3D(const std::shared_ptr<perception::PerceptionObstacles> ground_truth_3d_message);

	void
	LogPointCloud(const std::shared_ptr<drivers::PointCloud> point_cloud_message);

	std::shared_ptr<cyber::Reader<perception::PerceptionObstacles>> ground_truth_3d_reader_;
	std::shared_ptr<cyber::Reader<drivers::PointCloud>> point_cloud_reader_;
	std::shared_ptr<cyber::Writer<Frame>> frame_writer_;
	std::shared_ptr<cyber::Writer<perception::PerceptionObstacles>> depth_clustering_detection_writer_;
	std::shared_ptr<depth_clustering::DepthClustering> depth_clustering_;

	const std::string ground_truth_3d_log_file_name_ = "/apollo/data/log/safety_layer.ground_truth_3d.json";
	boost::property_tree::ptree ground_truth_3d_log_file_tree_;
	std::ofstream ground_truth_3d_log_file_;
	unsigned long frame_counter_;
	bool log_;
};

CYBER_REGISTER_COMPONENT(LidarComponent)
} // safety_layer
} // apollo
